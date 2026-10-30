#ifndef AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP
#define AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace auto_aim
{

// Robust event clock for a four-armor small gyro.
//
// The caller supplies the EKF body yaw relative to the vehicle-center bearing,
// folded modulo pi/2 into [-pi/4, pi/4].  Folding makes all four armor ids share
// one continuous face-center event and prevents an armor association change
// from moving the clock by a whole face.  Observed crossings update a bounded
// phase-locked loop (PLL): robust windows estimate period, phase and their live
// uncertainty while bounded alpha-beta corrections prevent outlier jumps.
class GyroPhaseScheduler
{
public:
  struct Config
  {
    double crossing_capture_rad = 20.0 * M_PI / 180.0;
    double crossing_rearm_rad = 12.0 * M_PI / 180.0;
    double min_crossing_separation_ratio = 0.55;
    double min_face_period_s = 0.12;
    double max_face_period_s = 1.20;
    double period_ema_alpha = 0.20;
    double max_period_relative_error = 0.22;
    double max_period_step_ratio = 0.025;
    double omega_period_weight = 0.15;
    double omega_alias_guard_relative_error = 0.35;
    double period_residual_alpha = 0.08;
    double phase_correction_alpha = 0.15;
    double max_phase_correction_s = 0.003;
    double max_phase_residual_s = 0.090;
    double phase_uncertainty_floor_s = 0.002;
    double max_stale_periods = 2.5;
    int period_window_size = 7;
    int phase_residual_window_size = 5;
    int min_lock_updates = 4;
    int relock_after_rejections = 3;
  };

  struct Observation
  {
    bool crossing = false;
    bool period_updated = false;
    bool rejected = false;
    bool reacquired = false;
    bool locked = false;
    bool omega_alias_guarded = false;
    int periods_elapsed = 0;
    int lock_updates = 0;
    int consecutive_rejections = 0;
    double crossing_time_s = 0.0;
    double period_sample_s = 0.0;
    double face_period_s = 0.0;
    double phase_residual_s = 0.0;
    double applied_phase_correction_s = 0.0;
    double phase_uncertainty_s = std::numeric_limits<double>::infinity();
  };

  struct Solution
  {
    bool valid = false;
    bool interval_safe = false;
    std::int64_t hit_cycle = -1;
    double command_due_time_s = 0.0;
    double time_until_command_s = std::numeric_limits<double>::infinity();
    double time_until_hit_center_s = std::numeric_limits<double>::infinity();
    double face_period_s = 0.0;
    double omega_rad_s = 0.0;
    double half_hit_window_rad = 0.0;
    double worst_interval_phase_rad = std::numeric_limits<double>::infinity();
    double phase_uncertainty_rad = std::numeric_limits<double>::infinity();
  };

  GyroPhaseScheduler() = default;
  explicit GyroPhaseScheduler(const Config & config) : config_(config) {}

  void reset()
  {
    have_sample_ = false;
    have_crossing_ = false;
    crossing_armed_ = true;
    period_estimate_valid_ = false;
    locked_ = false;
    previous_time_s_ = 0.0;
    previous_phase_error_rad_ = 0.0;
    last_observed_crossing_time_s_ = 0.0;
    phase_anchor_time_s_ = 0.0;
    face_period_s_ = 0.0;
    crossing_cycle_ = 0;
    lock_updates_ = 0;
    consecutive_rejections_ = 0;
    period_samples_.clear();
    phase_residuals_.clear();
    phase_uncertainty_s_ = std::numeric_limits<double>::infinity();
  }

  Observation observe(
    double sample_time_s, double folded_phase_error_rad, double ekf_omega_rad_s)
  {
    Observation out;
    if (!std::isfinite(sample_time_s) || !std::isfinite(folded_phase_error_rad)) {
      have_sample_ = false;
      return out;
    }

    folded_phase_error_rad = std::remainder(folded_phase_error_rad, M_PI / 2.0);
    if (!have_sample_) {
      have_sample_ = true;
      previous_time_s_ = sample_time_s;
      previous_phase_error_rad_ = folded_phase_error_rad;
      return out;
    }

    const double sample_dt_s = sample_time_s - previous_time_s_;
    const double previous_error = previous_phase_error_rad_;
    previous_time_s_ = sample_time_s;
    previous_phase_error_rad_ = folded_phase_error_rad;

    // A noisy folded phase can bounce around zero for several frames. Accept
    // only one crossing until the phase has moved well outside a Schmitt band.
    // This rejects the 17--38 ms duplicate "periods" seen in the field log
    // without treating them as a real PLL phase loss.
    if (!crossing_armed_) {
      if (std::abs(folded_phase_error_rad) >=
          std::max(0.0, config_.crossing_rearm_rad)) {
        crossing_armed_ = true;
      }
      // Even a sparse replay sample is allowed to re-arm the Schmitt trigger,
      // but a discontinuous dt is never used to interpolate a crossing.
      if (!(sample_dt_s > 0.0 && sample_dt_s <= 0.10)) return out;
      if (!crossing_armed_) return out;
    }
    if (!(sample_dt_s > 0.0 && sample_dt_s <= 0.10)) return out;

    const bool rising_crossing =
      previous_error <= 0.0 && folded_phase_error_rad >= 0.0;
    const bool falling_crossing =
      previous_error >= 0.0 && folded_phase_error_rad <= 0.0;
    const bool sign_crossing = std::isfinite(ekf_omega_rad_s) &&
        std::abs(ekf_omega_rad_s) > 0.05
      ? (ekf_omega_rad_s > 0.0 ? rising_crossing : falling_crossing)
      : (rising_crossing || falling_crossing);
    if (!sign_crossing ||
        std::max(std::abs(previous_error), std::abs(folded_phase_error_rad)) >
          config_.crossing_capture_rad) {
      return out;
    }

    const double denominator = std::abs(previous_error) + std::abs(folded_phase_error_rad);
    const double fraction = denominator > 1e-9
      ? std::clamp(std::abs(previous_error) / denominator, 0.0, 1.0)
      : 0.5;
    const double crossing_time_s = sample_time_s - sample_dt_s + fraction * sample_dt_s;
    crossing_armed_ = false;
    out.crossing = true;
    out.crossing_time_s = crossing_time_s;

    if (!have_crossing_) {
      have_crossing_ = true;
      last_observed_crossing_time_s_ = crossing_time_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ = 0;
      return out;
    }

    const double elapsed_s = crossing_time_s - last_observed_crossing_time_s_;
    double expected_period_s = face_period_s_;
    const double omega_period_s = std::isfinite(ekf_omega_rad_s) &&
        std::abs(ekf_omega_rad_s) > 0.05
      ? (M_PI / 2.0) / std::abs(ekf_omega_rad_s)
      : 0.0;
    if (!(expected_period_s > 0.0)) expected_period_s = omega_period_s;
    if (!(expected_period_s > 0.0)) expected_period_s = elapsed_s;

    const double min_crossing_separation_s = std::clamp(
      config_.min_crossing_separation_ratio, 0.0, 0.95) * expected_period_s;
    if (elapsed_s < min_crossing_separation_s) {
      // Keep the last physical crossing and lock state untouched. Marking a
      // duplicate as a phase residual rejection used to start an unnecessary
      // four-crossing relock sequence and caused multi-second fire gaps.
      out.crossing = false;
      return out;
    }

    // A stale PLL period can turn one slow physical interval into three fast
    // intervals (the 2026-09-11 field failure).  When the independently
    // estimated EKF rate strongly disagrees with the PLL, use it only to
    // disambiguate the integer number of elapsed face periods.  The robust
    // crossing median remains the frequency estimator after that choice.
    double period_count_reference_s = expected_period_s;
    if (omega_period_s >= config_.min_face_period_s &&
        omega_period_s <= config_.max_face_period_s &&
        face_period_s_ > 0.0 &&
        std::abs(omega_period_s - face_period_s_) /
            std::max(omega_period_s, 1e-6) >
          std::max(0.0, config_.omega_alias_guard_relative_error)) {
      period_count_reference_s = omega_period_s;
      out.omega_alias_guarded = true;
    }

    int periods_elapsed = static_cast<int>(
      std::llround(elapsed_s / period_count_reference_s));
    periods_elapsed = std::clamp(periods_elapsed, 1, 4);
    const double period_sample_s = elapsed_s / static_cast<double>(periods_elapsed);
    const double relative_error =
      std::abs(period_sample_s - period_count_reference_s) /
      std::max(period_count_reference_s, 1e-6);

    out.periods_elapsed = periods_elapsed;
    out.period_sample_s = period_sample_s;
    if (out.omega_alias_guarded) {
      // The old frequency hypothesis is no longer credible. Re-seed from the
      // independent rate immediately and require a fresh lock sequence; mixing
      // old fast samples with new slow samples is what created the 3x alias.
      face_period_s_ = omega_period_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ += periods_elapsed;
      last_observed_crossing_time_s_ = crossing_time_s;
      period_estimate_valid_ = true;
      locked_ = false;
      lock_updates_ = 1;
      consecutive_rejections_ = 0;
      period_samples_.clear();
      period_samples_.push_back(omega_period_s);
      phase_residuals_.clear();
      phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
      out.reacquired = true;
      fill_observation(out, 0.0, 0.0);
      return out;
    }
    if (period_sample_s < config_.min_face_period_s ||
        period_sample_s > config_.max_face_period_s ||
        relative_error > config_.max_period_relative_error) {
      // This is not a near-zero duplicate (those were suppressed above), so
      // temporarily veto firing until a consistent physical crossing arrives.
      // Preserve the mature anchor and lock count so one bad PnP phase sample
      // costs at most one event instead of forcing a full reacquisition.
      ++consecutive_rejections_;
      phase_uncertainty_s_ = std::abs(
        elapsed_s - static_cast<double>(periods_elapsed) * expected_period_s);
      out.rejected = true;
      out.consecutive_rejections = consecutive_rejections_;
      return out;
    }

    if (!period_estimate_valid_) {
      // Do not seed the PLL from a single noisy image interval.  Field data
      // contains paired long/short crossings; the already-converged EKF yaw
      // rate provides a substantially better initial period, after which the
      // robust image-event median remains authoritative.
      face_period_s_ = omega_period_s >= config_.min_face_period_s &&
          omega_period_s <= config_.max_face_period_s
        ? 0.98 * omega_period_s + 0.02 * period_sample_s
        : period_sample_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ += periods_elapsed;
      last_observed_crossing_time_s_ = crossing_time_s;
      period_estimate_valid_ = true;
      lock_updates_ = 1;
      consecutive_rejections_ = 0;
      push_period_sample(period_sample_s);
      phase_residuals_.clear();
      phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
      fill_observation(out, 0.0, 0.0);
      return out;
    }

    const double predicted_crossing_time_s = phase_anchor_time_s_ +
      static_cast<double>(periods_elapsed) * face_period_s_;
    const double phase_residual_s = crossing_time_s - predicted_crossing_time_s;
    if (std::abs(phase_residual_s) > config_.max_phase_residual_s) {
      // One bad monocular phase observation must not destroy a mature clock.
      // Keep the last accepted anchor so the next physical crossing can span
      // two periods and restore normal operation immediately. Several
      // consecutive, mutually consistent residuals still trigger a safe
      // reacquisition for a genuine phase discontinuity.
      ++consecutive_rejections_;
      phase_uncertainty_s_ = std::abs(phase_residual_s);

      out.rejected = true;
      out.phase_residual_s = phase_residual_s;
      out.consecutive_rejections = consecutive_rejections_;

      if (consecutive_rejections_ >= std::max(1, config_.relock_after_rejections)) {
        // Re-anchor only after several mutually consistent physical crossings.
        // The image-event median is the long-term frequency reference; EKF
        // omega is retained as a light prior instead of dominating it.
        double reacquired_period_s = median_period_sample();
        if (omega_period_s >= config_.min_face_period_s &&
            omega_period_s <= config_.max_face_period_s &&
            std::abs(omega_period_s - reacquired_period_s) /
                std::max(reacquired_period_s, 1e-6) <=
              config_.max_period_relative_error) {
          const double omega_weight = std::clamp(config_.omega_period_weight, 0.0, 1.0);
          reacquired_period_s =
            (1.0 - omega_weight) * reacquired_period_s +
            omega_weight * omega_period_s;
        }
        face_period_s_ = std::clamp(
          reacquired_period_s, config_.min_face_period_s, config_.max_face_period_s);
        phase_anchor_time_s_ = crossing_time_s;
        crossing_cycle_ += periods_elapsed;
        last_observed_crossing_time_s_ = crossing_time_s;
        locked_ = false;
        lock_updates_ = 1;
        consecutive_rejections_ = 0;
        phase_residuals_.clear();
        phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
        out.reacquired = true;
        fill_observation(out, phase_residual_s, crossing_time_s - predicted_crossing_time_s);
      }
      return out;
    }

    push_period_sample(period_sample_s);
    push_phase_residual(phase_residual_s);
    const double robust_phase_residual_s = median_value(phase_residuals_);
    double target_period_s = median_period_sample();
    if (omega_period_s >= config_.min_face_period_s &&
        omega_period_s <= config_.max_face_period_s &&
        std::abs(omega_period_s - target_period_s) /
            std::max(target_period_s, 1e-6) <= config_.max_period_relative_error) {
      // Body-yaw rate is continuous across armor handovers, but field replay
      // showed a persistent EKF-rate bias (about 320 ms versus 327 ms per
      // face).  Keep omega as a light prior and let the robust event median
      // remove that bias before it integrates into a phase-lock loss.
      const double omega_weight = std::clamp(config_.omega_period_weight, 0.0, 1.0);
      target_period_s =
        (1.0 - omega_weight) * target_period_s + omega_weight * omega_period_s;
    }
    target_period_s += config_.period_residual_alpha *
      robust_phase_residual_s / static_cast<double>(std::max(1, periods_elapsed));
    const double max_period_step_s =
      std::max(1e-6, config_.max_period_step_ratio * face_period_s_);
    target_period_s = std::clamp(
      target_period_s, face_period_s_ - max_period_step_s,
      face_period_s_ + max_period_step_s);
    face_period_s_ = (1.0 - config_.period_ema_alpha) * face_period_s_ +
      config_.period_ema_alpha * target_period_s;

    const double requested_phase_correction_s =
      config_.phase_correction_alpha * robust_phase_residual_s;
    const double applied_phase_correction_s = std::clamp(
      requested_phase_correction_s, -config_.max_phase_correction_s,
      config_.max_phase_correction_s);
    phase_anchor_time_s_ = predicted_crossing_time_s + applied_phase_correction_s;
    const double robust_spread_s = median_absolute_deviation(
      phase_residuals_, robust_phase_residual_s);
    const double robust_standard_error_s = phase_residuals_.empty()
      ? 0.0
      : 1.4826 * robust_spread_s /
        std::sqrt(static_cast<double>(phase_residuals_.size()));
    // The persistent residual mean is part of the field-calibrated fire-delay
    // zero and must not be counted a second time as random uncertainty.  Use
    // the robust residual spread for the physical hit-width gate; abrupt phase
    // changes are still handled by the rejection/relock path above.
    phase_uncertainty_s_ = std::max(
      std::max(0.0, config_.phase_uncertainty_floor_s),
      robust_standard_error_s);
    crossing_cycle_ += periods_elapsed;
    last_observed_crossing_time_s_ = crossing_time_s;
    ++lock_updates_;
    consecutive_rejections_ = 0;
    locked_ = lock_updates_ >= std::max(1, config_.min_lock_updates);
    fill_observation(out, phase_residual_s, applied_phase_correction_s);
    return out;
  }

  Solution solve(
    double now_s,
    double nominal_fire_delay_s,
    double min_fire_delay_s,
    double max_fire_delay_s,
    double flight_time_s,
    double flight_time_uncertainty_s,
    double armor_radius_m,
    double armor_width_m,
    double armor_edge_margin_m,
    double model_phase_uncertainty_rad) const
  {
    Solution out;
    if (!locked_ || consecutive_rejections_ > 0 || !std::isfinite(now_s) ||
        !std::isfinite(nominal_fire_delay_s) || !std::isfinite(flight_time_s) ||
        !(face_period_s_ > 0.0) || !std::isfinite(armor_radius_m) ||
        armor_radius_m <= 0.0 || !std::isfinite(armor_width_m) ||
        now_s - last_observed_crossing_time_s_ > config_.max_stale_periods * face_period_s_) {
      return out;
    }

    const double nominal_total_s = nominal_fire_delay_s + flight_time_s;
    std::int64_t steps = static_cast<std::int64_t>(std::ceil(
      (now_s + nominal_total_s - phase_anchor_time_s_) / face_period_s_ - 1e-9));
    steps = std::max<std::int64_t>(1, steps);
    const double hit_center_time_s = phase_anchor_time_s_ +
      static_cast<double>(steps) * face_period_s_;

    out.valid = true;
    out.face_period_s = face_period_s_;
    out.omega_rad_s = (M_PI / 2.0) / face_period_s_;
    out.hit_cycle = crossing_cycle_ + steps;
    out.command_due_time_s = hit_center_time_s - nominal_total_s;
    out.time_until_command_s = out.command_due_time_s - now_s;
    out.time_until_hit_center_s = hit_center_time_s - now_s;

    const double usable_half_width_m = std::max(
      0.0, armor_width_m * 0.5 - std::max(0.0, armor_edge_margin_m));
    out.half_hit_window_rad = std::atan2(
      usable_half_width_m, std::max(armor_radius_m, 1e-6));

    const double requested_min_fire_delay_s = min_fire_delay_s;
    min_fire_delay_s = std::min(requested_min_fire_delay_s, max_fire_delay_s);
    max_fire_delay_s = std::max(requested_min_fire_delay_s, max_fire_delay_s);
    const double earliest_total_s = min_fire_delay_s +
      std::max(0.0, flight_time_s - std::max(0.0, flight_time_uncertainty_s));
    const double latest_total_s = max_fire_delay_s + flight_time_s +
      std::max(0.0, flight_time_uncertainty_s);
    const double early_phase = out.omega_rad_s * (earliest_total_s - nominal_total_s);
    const double late_phase = out.omega_rad_s * (latest_total_s - nominal_total_s);
    out.worst_interval_phase_rad =
      std::max(std::abs(early_phase), std::abs(late_phase)) +
      std::max(0.0, model_phase_uncertainty_rad) +
      out.omega_rad_s * std::max(0.0, phase_uncertainty_s_);
    out.phase_uncertainty_rad =
      out.omega_rad_s * std::max(0.0, phase_uncertainty_s_);
    out.interval_safe = out.worst_interval_phase_rad <= out.half_hit_window_rad;
    return out;
  }

  bool period_valid() const { return period_estimate_valid_; }
  bool locked() const { return locked_; }
  int lock_updates() const { return lock_updates_; }
  double face_period_s() const { return face_period_s_; }
  double phase_anchor_time_s() const { return phase_anchor_time_s_; }
  double phase_uncertainty_s() const { return phase_uncertainty_s_; }
  std::int64_t crossing_cycle() const { return crossing_cycle_; }

private:
  void push_period_sample(double sample_s)
  {
    period_samples_.push_back(sample_s);
    const auto max_size = static_cast<std::size_t>(
      std::max(3, config_.period_window_size));
    while (period_samples_.size() > max_size) period_samples_.pop_front();
  }

  double median_period_sample() const
  {
    return median_value(period_samples_);
  }

  template<typename Container>
  static double median_value(const Container & values)
  {
    if (values.empty()) return 0.0;
    std::vector<double> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t middle = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
      return 0.5 * (sorted[middle - 1] + sorted[middle]);
    }
    return sorted[middle];
  }

  void push_phase_residual(double residual_s)
  {
    phase_residuals_.push_back(residual_s);
    const auto max_size = static_cast<std::size_t>(
      std::max(3, config_.phase_residual_window_size));
    while (phase_residuals_.size() > max_size) phase_residuals_.pop_front();
  }

  template<typename Container>
  static double median_absolute_deviation(const Container & values, double median)
  {
    if (values.empty()) return 0.0;
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (const double value : values) deviations.push_back(std::abs(value - median));
    return median_value(deviations);
  }

  void fill_observation(
    Observation & out, double phase_residual_s, double applied_phase_correction_s) const
  {
    out.period_updated = true;
    out.face_period_s = face_period_s_;
    out.phase_residual_s = phase_residual_s;
    out.applied_phase_correction_s = applied_phase_correction_s;
    out.phase_uncertainty_s = phase_uncertainty_s_;
    out.lock_updates = lock_updates_;
    out.consecutive_rejections = consecutive_rejections_;
    out.locked = locked_;
  }

  Config config_{};
  bool have_sample_ = false;
  bool have_crossing_ = false;
  bool crossing_armed_ = true;
  bool period_estimate_valid_ = false;
  bool locked_ = false;
  double previous_time_s_ = 0.0;
  double previous_phase_error_rad_ = 0.0;
  double last_observed_crossing_time_s_ = 0.0;
  double phase_anchor_time_s_ = 0.0;
  double face_period_s_ = 0.0;
  std::int64_t crossing_cycle_ = 0;
  int lock_updates_ = 0;
  int consecutive_rejections_ = 0;
  std::deque<double> period_samples_;
  std::deque<double> phase_residuals_;
  double phase_uncertainty_s_ = std::numeric_limits<double>::infinity();
};

// Fixed-cadence request clock for a steadily spinning four-armor target.
//
// Why this exists: asking "is the plate aligned right now" every frame cannot
// work at speed. One face passes the bore in (pi/2)/|omega| seconds and the
// usable window is a small fraction of that; at 60 fps the target turns several
// degrees per frame, so a per-frame alignment test rejects nearly every frame and
// the few it accepts are decided by luck. This class instead anchors a clock on
// one physical face-centre observation, then requests a shot once per N faces at
// a configured phase offset. Observed face crossings refine period and phase
// slowly, so the clock tracks the real target instead of a nominal speed.
//
// The caller must still apply every safety gate (AUTO, attitude, fresh
// observation, gimbal readiness, lower-level permit). This answers "is it time to
// request", not "is it safe to fire".
class FixedCadenceTrigger
{
public:
  struct Config
  {
    double anchor_tolerance_rad = 20.0 * M_PI / 180.0;
    double crossing_capture_rad = 20.0 * M_PI / 180.0;
    double crossing_rearm_rad = 12.0 * M_PI / 180.0;
    double min_crossing_separation_ratio = 0.55;
    double max_crossing_gap_faces = 2.5;
    std::size_t max_crossings = 24;
    // Calibration needs a few crossings, not a long history. The original value of
    // eight assumed a clock that survives a whole session; in practice the anchor
    // is dropped by target loss and re-acquisition, so a high threshold meant the
    // period was never learned at all (field log 2026-10-31_04-15-44: calib stayed
    // 0 for 87 s).
    std::size_t min_crossings_for_calibration = 4;
    std::int64_t min_slope_span_faces = 2;
    double period_min_ratio = 0.90;
    double period_max_ratio = 1.10;
    double max_period_step_ratio = 0.005;
    double max_phase_step_s = 0.003;
    double phase_gain = 0.25;
    // Fractional rate deviation tolerated before the reference rate is re-seeded.
    // Not a reset: the learned period is kept.
    double rate_reseed_ratio = 0.25;
    // The test chassis spins at a mechanically fixed rate, so the period is a
    // constant and there is nothing for calibration to learn. With this off the
    // clock keeps the anchored period exactly; crossings are still recorded, but
    // only as diagnostics. Leaving calibration on against a constant-rate target
    // just adds noise to a value that is already correct, and every bounded step it
    // takes moves the request phase away from the tuned offset.
    bool calibrate_period = true;
  };

  struct Solution
  {
    bool valid = false;
    std::int64_t cycle = -1;
    double command_due_time_s = 0.0;
    double time_until_command_s = std::numeric_limits<double>::infinity();
    double face_period_s = 0.0;
    double nominal_face_period_s = 0.0;
    double phase_error_rad = NAN;
    // Measured face-crossing time minus the time the clock predicted, in seconds.
    // Positive means the face arrived late relative to the clock, so the request
    // phase sits early; negative means the opposite.
    double crossing_residual_s = NAN;
    int crossing_count = 0;
    double last_phase_adjustment_s = 0.0;
    std::uint64_t calibration_updates = 0;
  };

  FixedCadenceTrigger() = default;
  explicit FixedCadenceTrigger(const Config & config) : config_(config) {}

  void set_config(const Config & config) { config_ = config; }
  const Config & config() const { return config_; }

  void reset()
  {
    anchored_ = false;
    last_sample_s_ = NAN;
    period_s_ = 0.0;
    calibration_updates_ = 0;
    large_residual_updates_ = 0;
    last_phase_adjustment_s_ = 0.0;
    last_crossing_residual_s_ = NAN;
    crossing_count_ = 0;
    have_phase_sample_ = false;
    crossing_armed_ = true;
    crossings_.clear();
    ++cycle_;
  }

  // now_s         : decision time. sample_s stays separate so a replay can feed
  //                 capture-time observations without changing the clock.
  // sample_s      : capture time of the observation that produced phase_rad
  // phase_rad     : body yaw relative to the target-centre bearing, caller-folded
  // signed_omega  : signed rotation rate, rad/s
  // offset_s      : request this long after a face centre passes
  // every_n_faces : request once per N faces
  Solution observe(
    double now_s, double sample_s, double phase_rad, double signed_omega,
    double offset_s, int every_n_faces, double max_age_s,
    double max_observation_gap_s = 0.25, double late_tolerance_s = 0.03)
  {
    Solution out;
    if (!std::isfinite(now_s) || !std::isfinite(sample_s) ||
        !std::isfinite(phase_rad) || !std::isfinite(signed_omega) ||
        !std::isfinite(offset_s) || !std::isfinite(max_age_s) || max_age_s <= 0.0 ||
        !std::isfinite(max_observation_gap_s) || max_observation_gap_s <= 0.0 ||
        !std::isfinite(late_tolerance_s) || late_tolerance_s < 0.0 ||
        sample_s > now_s || now_s - sample_s > max_age_s ||
        std::abs(signed_omega) < 0.05 || every_n_faces < 1) {
      reset();
      return out;
    }

    // Direction reversal, a stride/offset change or a lost observation invalidate
    // the clock. Ordinary rate jitter must NOT: the rate here comes from a live
    // estimator that moves every frame, so keying the clock on it would reset it
    // continuously and calibration would never accumulate. The period is learned
    // by calibrate() instead, seeded by the rate seen at anchor time.
    if (anchored_ && (signed_omega * nominal_omega_ < 0.0 ||
        every_n_faces != every_n_faces_ ||
        std::abs(offset_s - configured_offset_s_) > 1e-9 ||
        (std::isfinite(last_sample_s_) &&
          (sample_s < last_sample_s_ || sample_s - last_sample_s_ > max_observation_gap_s)))) {
      reset();
    }
    // Re-seed the reference rate only when the estimate drifts far outside what
    // calibration can absorb. The bound is loose on purpose: each calibration step
    // is clamped to a small fraction of a period, so a slightly wrong seed
    // converges instead of forcing a reset.
    if (anchored_ && std::abs(std::abs(signed_omega) - std::abs(nominal_omega_)) >
        config_.rate_reseed_ratio * std::abs(nominal_omega_)) {
      nominal_omega_ = signed_omega;
    }
    last_sample_s_ = sample_s;

    const double nominal_face_period_s = (M_PI / 2.0) / std::abs(signed_omega);
    const double folded = std::remainder(phase_rad, M_PI / 2.0);

    if (!anchored_) {
      // Only a physically near-centre observation starts the clock, so the anchor
      // is a real face centre rather than an arbitrary instant.
      if (std::abs(folded) > config_.anchor_tolerance_rad) return out;
      nominal_omega_ = omega_ = signed_omega;
      every_n_faces_ = every_n_faces;
      configured_offset_s_ = offset_s;
      period_s_ = every_n_faces * nominal_face_period_s;
      offset_in_face_s_ = offset_modulo(offset_s, nominal_face_period_s);
      anchor_s_ = sample_s - folded / omega_;
      next_due_s_ = anchor_s_ + offset_in_face_s_;
      anchored_ = true;
      ++cycle_;
    }

    // Record crossings always (they drive the phase diagnostic), but only let them
    // move the period when calibration is enabled. On a mechanically fixed-rate
    // target the anchored period is already exact.
    if (config_.calibrate_period) {
      observe_crossing(sample_s, folded);
    } else {
      record_crossing_only(sample_s, folded);
    }

    // Skip missed slots rather than firing a burst to catch up.
    if (next_due_s_ < now_s - late_tolerance_s) {
      const auto skip = static_cast<std::int64_t>(std::ceil(
        (now_s - late_tolerance_s - next_due_s_) / period_s_));
      next_due_s_ += skip * period_s_;
      cycle_ += skip;
    }

    // Phase error diagnostic.
    //
    // The previous expression compared the observed phase against the clock's own
    // extrapolation, which is identically zero by construction (the anchor was
    // derived from the same observation), so it reported +-0.00 forever and told
    // the operator nothing. What actually matters is how far the measured face
    // crossings sit from where the clock said they would be: that is the residual
    // the phase offset has to absorb.
    out.phase_error_rad = std::remainder(
      last_crossing_residual_s_ * std::abs(omega_), M_PI / 2.0);
    out.crossing_residual_s = last_crossing_residual_s_;
    out.crossing_count = crossing_count_;
    out.cycle = cycle_;
    out.command_due_time_s = next_due_s_;
    out.time_until_command_s = std::max(0.0, next_due_s_ - now_s);
    out.face_period_s = period_s_ / every_n_faces_;
    out.nominal_face_period_s = nominal_face_period_s;
    out.last_phase_adjustment_s = last_phase_adjustment_s_;
    out.calibration_updates = calibration_updates_;
    out.valid = now_s - next_due_s_ <= late_tolerance_s;
    return out;
  }

  void mark_fired()
  {
    if (anchored_) {
      next_due_s_ += period_s_;
      ++cycle_;
    }
  }

  bool anchored() const { return anchored_; }
  double command_interval_s() const { return anchored_ ? period_s_ : 0.0; }
  double crossing_residual_s() const { return anchored_ ? last_crossing_residual_s_ : NAN; }
  int crossing_count() const { return crossing_count_; }
  std::uint64_t calibration_updates() const { return calibration_updates_; }
  std::uint64_t large_residual_updates() const { return large_residual_updates_; }

private:
  struct CrossingSample
  {
    std::int64_t index;
    double time_s;
  };

  static double median(std::vector<double> values)
  {
    if (values.empty()) return NAN;
    std::sort(values.begin(), values.end());
    const auto middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
  }

  static double offset_modulo(double offset, double face_period)
  {
    if (!(face_period > 0.0)) return 0.0;
    double value = std::fmod(offset, face_period);
    return value < 0.0 ? value + face_period : value;
  }

  // Interpolate the instant a face centre crossed zero between two samples.
  // Returns NAN when this sample pair is not a usable crossing: not armed yet, too
  // far from centre to trust the interpolation, or closer than half a face to the
  // previous one (a noisy duplicate rather than the next plate).
  double detect_crossing(double sample_s, double folded)
  {
    const double previous_s = previous_phase_time_s_;
    const double previous_phase = previous_phase_rad_;
    previous_phase_time_s_ = sample_s;
    previous_phase_rad_ = folded;
    if (!have_phase_sample_) {
      have_phase_sample_ = true;
      return NAN;
    }
    const double dt = sample_s - previous_s;
    if (!crossing_armed_ && std::abs(folded) >= config_.crossing_rearm_rad) crossing_armed_ = true;
    if (!crossing_armed_ || dt <= 0.0 || dt > 0.10) return NAN;

    const bool crossing = nominal_omega_ > 0.0
      ? previous_phase <= 0.0 && folded >= 0.0
      : previous_phase >= 0.0 && folded <= 0.0;
    if (!crossing ||
      std::max(std::abs(previous_phase), std::abs(folded)) > config_.crossing_capture_rad) {
      return NAN;
    }

    const double denominator = std::abs(previous_phase) + std::abs(folded);
    if (denominator <= 1e-12) return NAN;
    crossing_armed_ = false;
    const double cross_s = previous_s + dt * std::abs(previous_phase) / denominator;

    const double face_s = period_s_ / every_n_faces_;
    if (!(face_s > 0.0)) return NAN;
    if (!crossings_.empty()) {
      const double gap_s = cross_s - crossings_.back().time_s;
      if (gap_s < config_.min_crossing_separation_ratio * face_s) return NAN;
      if (gap_s > config_.max_crossing_gap_faces * face_s) crossings_.clear();
    }
    return cross_s;
  }

  // Detect a face centre passing zero and keep a bounded history. Pairwise slopes
  // spanning several passages are medianed instead of adjacent differences: field
  // data shows alternating long/short intervals that would bias the frequency.
  void observe_crossing(double sample_s, double folded)
  {
    const double cross_s = detect_crossing(sample_s, folded);
    if (!std::isfinite(cross_s)) return;
    ++crossing_count_;

    const double face_s = period_s_ / every_n_faces_;
    const auto index = crossings_.empty() ? std::int64_t{0}
      : crossings_.back().index + std::max<std::int64_t>(1,
          std::llround((cross_s - crossings_.back().time_s) / face_s));
    if (std::isfinite(anchor_s_) && face_s > 0.0) {
      const double predicted_s = anchor_s_ + std::round((cross_s - anchor_s_) / face_s) * face_s;
      last_crossing_residual_s_ = cross_s - predicted_s;
    }
    crossings_.push_back({index, cross_s});
    while (crossings_.size() > config_.max_crossings) crossings_.pop_front();
    if (crossings_.size() < config_.min_crossings_for_calibration) return;

    std::vector<double> slopes;
    for (std::size_t i = 0; i < crossings_.size(); ++i) {
      for (std::size_t j = i + 1; j < crossings_.size(); ++j) {
        const auto count = crossings_[j].index - crossings_[i].index;
        if (count >= config_.min_slope_span_faces) {
          slopes.push_back(
            (crossings_[j].time_s - crossings_[i].time_s) / static_cast<double>(count));
        }
      }
    }
    const double fitted_period_s = median(slopes);
    if (!std::isfinite(fitted_period_s)) return;

    std::vector<double> anchors;
    anchors.reserve(crossings_.size());
    for (const auto & point : crossings_) {
      anchors.push_back(point.time_s + (index - point.index) * fitted_period_s);
    }
    calibrate(fitted_period_s, median(anchors));
  }

  // Diagnostic-only variant: same crossing detection and residual bookkeeping, but
  // never moves the period or the anchor. Used when the target rate is mechanically
  // fixed, so the anchored period is already exact and calibration would only push
  // a correct value around.
  void record_crossing_only(double sample_s, double folded)
  {
    const double cross_s = detect_crossing(sample_s, folded);
    if (!std::isfinite(cross_s)) return;
    ++crossing_count_;
    // What the clock expected for this crossing, measured against the anchor and
    // the whole number of faces elapsed.
    const double face_s = period_s_ / every_n_faces_;
    if (face_s > 0.0 && std::isfinite(anchor_s_)) {
      const double faces_elapsed = std::round((cross_s - anchor_s_) / face_s);
      const double predicted_s = anchor_s_ + faces_elapsed * face_s;
      last_crossing_residual_s_ = cross_s - predicted_s;
    }
  }

  // Slew period and phase slowly. Snapping to each observation would let one noisy
  // crossing move the clock a large fraction of a face; small bounded steps keep
  // the request phase continuous across a whole session.
  void calibrate(double observed_face_period_s, double observed_anchor_s)
  {
    const double old_face_period_s = period_s_ / every_n_faces_;
    const double nominal_face_period_s = (M_PI / 2.0) / std::abs(nominal_omega_);
    if (!std::isfinite(observed_face_period_s) || !std::isfinite(observed_anchor_s) ||
        !(old_face_period_s > 0.0) || !(nominal_face_period_s > 0.0) ||
        observed_face_period_s < config_.period_min_ratio * nominal_face_period_s ||
        observed_face_period_s > config_.period_max_ratio * nominal_face_period_s) {
      return;
    }

    // Pivot at the recent crossing instead of rescaling elapsed time from t0,
    // otherwise a small period update still makes a large late-session jump.
    const double crossing_index = std::round((observed_anchor_s - anchor_s_) / old_face_period_s);
    const double predicted_anchor_s = anchor_s_ + crossing_index * old_face_period_s;
    const double residual_s = observed_anchor_s - predicted_anchor_s;
    if (std::abs(residual_s) > 0.25 * old_face_period_s) ++large_residual_updates_;

    const double faces_to_slot = std::round(
      (next_due_s_ - offset_in_face_s_ - predicted_anchor_s) / old_face_period_s);

    const double step = config_.max_period_step_ratio;
    const double new_face_period_s = std::clamp(
      observed_face_period_s, old_face_period_s * (1.0 - step),
      old_face_period_s * (1.0 + step));

    last_phase_adjustment_s_ = std::clamp(
      config_.phase_gain * residual_s, -config_.max_phase_step_s, config_.max_phase_step_s);
    anchor_s_ = predicted_anchor_s + last_phase_adjustment_s_;
    period_s_ = every_n_faces_ * new_face_period_s;
    omega_ = std::copysign((M_PI / 2.0) / new_face_period_s, nominal_omega_);
    offset_in_face_s_ = offset_modulo(configured_offset_s_, new_face_period_s);
    next_due_s_ = anchor_s_ + faces_to_slot * new_face_period_s + offset_in_face_s_;
    ++calibration_updates_;
  }

  Config config_{};
  bool anchored_ = false;
  double nominal_omega_ = 0.0;
  double omega_ = 0.0;
  double period_s_ = 0.0;
  double next_due_s_ = 0.0;
  int every_n_faces_ = 1;
  double configured_offset_s_ = 0.0;
  double offset_in_face_s_ = 0.0;
  double anchor_s_ = 0.0;
  double last_sample_s_ = NAN;
  // Measured crossing time minus the clock's prediction, seconds. This is the
  // quantity the phase offset has to absorb, and the only phase diagnostic that
  // carries information.
  double last_crossing_residual_s_ = NAN;
  int crossing_count_ = 0;
  double last_phase_adjustment_s_ = 0.0;
  std::uint64_t calibration_updates_ = 0;
  std::uint64_t large_residual_updates_ = 0;
  bool have_phase_sample_ = false;
  bool crossing_armed_ = true;
  double previous_phase_time_s_ = 0.0;
  double previous_phase_rad_ = 0.0;
  std::deque<CrossingSample> crossings_;
  std::int64_t cycle_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP
