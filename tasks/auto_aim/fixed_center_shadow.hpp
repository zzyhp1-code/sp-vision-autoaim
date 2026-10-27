#ifndef AUTO_AIM__FIXED_CENTER_SHADOW_HPP
#define AUTO_AIM__FIXED_CENTER_SHADOW_HPP

#include "face_prediction_residual.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace auto_aim
{

// Diagnostic candidate for a stationary, rotating four-face chassis. Fit one
// centre, two radii and a shared phase/rate. No tracker state is modified and
// there is deliberately no control-eligibility or automatic takeover API.
class FixedCenterShadow
{
public:
  using Context = FacePredictionResidual::Context;
  using Observation = FacePredictionResidual::Observation;
  using Xyz = FacePredictionResidual::Xyz;
  enum class Reason {
    ready, warmup, coverage, invalid_input, context_changed, clock_gap,
    stale_observation, stale_model, fit_failed, invalid_time, invalid_face,
    holdout_error, moving_center, motion_coverage
  };
  static const char * reason_name(Reason reason)
  {
    switch (reason) {
      case Reason::ready: return "ready";
      case Reason::warmup: return "warmup";
      case Reason::coverage: return "coverage";
      case Reason::invalid_input: return "invalid_input";
      case Reason::context_changed: return "context_changed";
      case Reason::clock_gap: return "clock_gap";
      case Reason::stale_observation: return "stale_observation";
      case Reason::stale_model: return "stale_model";
      case Reason::fit_failed: return "fit_failed";
      case Reason::invalid_time: return "invalid_time";
      case Reason::invalid_face: return "invalid_face";
      case Reason::holdout_error: return "holdout_error";
      case Reason::moving_center: return "moving_center";
      case Reason::motion_coverage: return "motion_coverage";
    }
    return "invalid_input";
  }
  struct Config
  {
    std::int64_t window_ns = 4000000000LL;
    std::int64_t holdout_ns = 500000000;
    std::int64_t sample_interval_ns = 25000000;
    std::int64_t fit_interval_ns = 100000000;
    std::int64_t max_frame_gap_ns = 250000000;
    std::int64_t max_prediction_age_ns = 250000000;
    std::size_t max_samples = 800;
    std::size_t min_train_samples = 32, min_holdout_samples = 8;
    std::size_t min_train_per_face = 3;
    double min_train_span_s = 1.5;
    double min_rotation_rad = 6.2831853071795864769;
    double min_omega_rad_s = 0.5, max_omega_rad_s = 15.0;
    double min_radius_m = 0.03, max_radius_m = 0.6;
    // These are fit weights, not physical covariance or validated uncertainty.
    double radial_scale_m = 0.12, transverse_scale_m = 0.02;
    double huber_norm = 3.0;
    // Audit labels only. These are not validated firing/takeover thresholds.
    double audit_max_yaw_p90_rad = 0.75 * 3.14159265358979323846 / 180.0;
    double audit_max_position_p90_m = 0.35;
    double audit_max_center_speed_mps = 0.25;
    double audit_min_motion_span_s = 0.15;
  };
  struct Seed
  {
    double center_x = 0.0, center_y = 0.0;
    double phase_rad = 0.0, omega_rad_s = 0.0;
    std::array<double, 2> radius_m{};
    std::array<double, 2> height_m{};
  };
  struct Model
  {
    std::int64_t reference_ns = 0;
    Seed state;
  };
  struct Output
  {
    bool fitted = false;     // A new chronological training/holdout fit this call.
    bool evaluable = false;  // A recent, converged candidate exists; not eligibility.
    std::uint64_t epoch = 0, revision = 0;
    std::size_t samples = 0, train_samples = 0, heldout_samples = 0;
    unsigned heldout_face_mask = 0;
    double heldout_yaw_p90_rad = std::numeric_limits<double>::quiet_NaN();
    double heldout_position_p90_m = std::numeric_limits<double>::quiet_NaN();
    double training_cost = std::numeric_limits<double>::quiet_NaN();
    Model model;
    Reason reason = Reason::warmup;
    bool fit_attempted = false, motion_evaluable = false;
    double heldout_center_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double heldout_motion_span_s = 0.0;
  };
  struct Forecast
  {
    bool available = false, stationary_supported = false;
    Reason reason = Reason::warmup;
    Xyz xyz{};
    Output fit;
    std::int64_t issued_ns = 0, prediction_ns = 0;
    int face_id = -1;
  };
  struct Counters { std::uint64_t fit_attempts = 0, fit_failures = 0, resets = 0; };

  FixedCenterShadow() : FixedCenterShadow(Config{}) {}
  explicit FixedCenterShadow(const Config & config) : config_(config)
  {
    if (config_.window_ns <= config_.holdout_ns || config_.holdout_ns <= 0 ||
        config_.sample_interval_ns <= 0 || config_.fit_interval_ns <= 0 ||
        config_.max_frame_gap_ns <= 0 || config_.max_prediction_age_ns <= 0 ||
        config_.min_train_samples < 6 || config_.min_holdout_samples == 0 ||
        config_.min_train_per_face == 0 ||
        config_.max_samples < config_.min_train_samples + config_.min_holdout_samples ||
        !positive(config_.min_train_span_s) || !positive(config_.min_rotation_rad) ||
        !positive(config_.min_omega_rad_s) || !positive(config_.max_omega_rad_s) ||
        config_.max_omega_rad_s <= config_.min_omega_rad_s ||
        !positive(config_.min_radius_m) || !positive(config_.max_radius_m) ||
        config_.max_radius_m <= config_.min_radius_m || !positive(config_.radial_scale_m) ||
        !positive(config_.transverse_scale_m) || !positive(config_.huber_norm) ||
        !positive(config_.audit_max_yaw_p90_rad) || !positive(config_.audit_max_position_p90_m) ||
        !positive(config_.audit_max_center_speed_mps) || !positive(config_.audit_min_motion_span_s))
      throw std::invalid_argument("Invalid fixed centre shadow limits");
  }

  void reset()
  {
    ++counters_.resets;
    samples_.clear();
    context_.reset();
    last_frame_ns_ = 0;
    last_stored_ns_ = 0;
    last_fit_attempt_ns_ = 0;
    last_fit_ns_ = 0;
    last_observation_ns_ = 0;
    output_ = {};
    output_.epoch = ++epoch_;
  }

  Output update(std::int64_t frame_ns, const Context & context, const Seed & seed,
    const std::vector<Observation> & observations)
  {
    if (frame_ns <= 0 || !valid_context(context) || !valid_seed(seed, context.rotation_sign)) {
      reset();
      output_.reason = Reason::invalid_input;
      return output_;
    }
    if (context_ && !same_context(*context_, context)) {
      reset();
      output_.reason = Reason::context_changed;
    } else if (last_frame_ns_ && (frame_ns <= last_frame_ns_ ||
          frame_ns - last_frame_ns_ > config_.max_frame_gap_ns)) {
      reset();
      output_.reason = Reason::clock_gap;
    } else if (last_observation_ns_ &&
        frame_ns - last_observation_ns_ > config_.max_frame_gap_ns) {
      reset();
      output_.reason = Reason::stale_observation;
    }
    if (!context_) context_ = context;
    last_frame_ns_ = frame_ns;
    output_.fitted = false;
    output_.fit_attempted = false;
    std::array<int, 4> counts{};
    for (const auto & o : observations) if (valid_face(o.face_id)) ++counts[o.face_id];
    bool fresh = false, stored = false;
    for (const auto & o : observations) {
      if (!valid_face(o.face_id) || counts[o.face_id] != 1 || !valid_xyz(o.xyz)) continue;
      fresh = true;
      if (!last_stored_ns_ || frame_ns - last_stored_ns_ >= config_.sample_interval_ns) {
        samples_.push_back({frame_ns, o.face_id, o.xyz, seed});
        stored = true;
      }
    }
    if (fresh) last_observation_ns_ = frame_ns;
    if (stored) last_stored_ns_ = frame_ns;
    while (!samples_.empty() && (samples_.size() > config_.max_samples ||
      frame_ns - samples_.front().time_ns > config_.window_ns)) samples_.pop_front();
    output_.samples = samples_.size();
    output_.evaluable = last_fit_ns_ &&
      frame_ns - last_fit_ns_ <= config_.max_prediction_age_ns;
    if (last_fit_ns_ && !output_.evaluable) output_.reason = Reason::stale_model;
    if (!fresh || (last_fit_attempt_ns_ &&
        frame_ns - last_fit_attempt_ns_ < config_.fit_interval_ns)) return output_;
    last_fit_attempt_ns_ = frame_ns;

    std::vector<Sample> train, heldout;
    for (const auto & sample : samples_)
      (sample.time_ns <= frame_ns - config_.holdout_ns ? train : heldout).push_back(sample);
    if (train.size() < config_.min_train_samples || heldout.size() < config_.min_holdout_samples) {
      output_.evaluable = false;
      if (last_fit_ns_ || output_.reason == Reason::ready) output_.reason = Reason::coverage;
      last_fit_ns_ = 0;
      return output_;
    }
    const double span_s = (train.back().time_ns - train.front().time_ns) / 1e9;
    std::array<std::size_t, 4> face_counts{};
    for (const auto & sample : train) ++face_counts[sample.face];
    if (span_s < config_.min_train_span_s ||
        std::abs(seed.omega_rad_s) * span_s < config_.min_rotation_rad ||
        *std::min_element(face_counts.begin(), face_counts.end()) < config_.min_train_per_face) {
      output_.evaluable = false;
      output_.reason = Reason::coverage;
      last_fit_ns_ = 0;
      return output_;
    }

    // Even the nonlinear initializer comes from the training side of the time
    // split. A current EKF seed may already contain the held-out observations.
    const auto & train_seed = train.back().seed;
    Parameters fitted{train_seed.center_x, train_seed.center_y,
      train_seed.phase_rad + train_seed.omega_rad_s * ((frame_ns - train.back().time_ns) / 1e9),
      train_seed.omega_rad_s, train_seed.radius_m[0], train_seed.radius_m[1]};
    output_.fit_attempted = true;
    ++counters_.fit_attempts;
    if (!fit(train, frame_ns, context.rotation_sign, fitted)) {
      ++counters_.fit_failures;
      // A failed newer fit invalidates the previous candidate immediately.
      output_.evaluable = false;
      last_fit_ns_ = 0;
      output_.reason = Reason::fit_failed;
      return output_;
    }
    std::array<double, 2> heights{};
    for (int parity = 0; parity < 2; ++parity) {
      std::vector<double> values;
      for (const auto & sample : train)
        if (sample.face % 2 == parity) values.push_back(sample.xyz[2]);
      heights[parity] = median(values);
    }
    std::vector<double> yaw_errors, position_errors;
    unsigned mask = 0;
    for (const auto & sample : heldout) {
      const auto predicted = position(fitted, heights, frame_ns, sample.time_ns, sample.face);
      yaw_errors.push_back(std::abs(std::remainder(std::atan2(predicted[1], predicted[0]) -
        std::atan2(sample.xyz[1], sample.xyz[0]), 2.0 * pi)));
      position_errors.push_back(std::hypot(std::hypot(predicted[0] - sample.xyz[0],
        predicted[1] - sample.xyz[1]), predicted[2] - sample.xyz[2]));
      mask |= 1u << sample.face;
    }
    output_.model = {frame_ns,
      {fitted[0], fitted[1], std::remainder(fitted[2], 2.0 * pi), fitted[3],
        {fitted[4], fitted[5]}, heights}};
    output_.training_cost = cost(train, frame_ns, fitted);
    output_.train_samples = train.size();
    output_.heldout_samples = heldout.size();
    output_.heldout_face_mask = mask;
    output_.heldout_yaw_p90_rad = p90(yaw_errors);
    output_.heldout_position_p90_m = p90(position_errors);
    // Fit a diagnostic centre-drift slope on chronological heldout data only.
    // Reconstructed centre motion may also reflect PnP/association errors;
    // it is evidence against the fixed-centre assumption, not a motion truth.
    double mean_t = 0.0, mean_x = 0.0, mean_y = 0.0;
    for (const auto & sample : heldout) {
      const auto predicted = position(fitted, heights, frame_ns, sample.time_ns, sample.face);
      mean_t += (sample.time_ns - frame_ns) / 1e9;
      mean_x += sample.xyz[0] - predicted[0];
      mean_y += sample.xyz[1] - predicted[1];
    }
    mean_t /= heldout.size(); mean_x /= heldout.size(); mean_y /= heldout.size();
    double variance_t = 0.0, covariance_x = 0.0, covariance_y = 0.0;
    for (const auto & sample : heldout) {
      const auto predicted = position(fitted, heights, frame_ns, sample.time_ns, sample.face);
      const double dt = (sample.time_ns - frame_ns) / 1e9 - mean_t;
      variance_t += dt * dt;
      covariance_x += dt * (sample.xyz[0] - predicted[0] - mean_x);
      covariance_y += dt * (sample.xyz[1] - predicted[1] - mean_y);
    }
    output_.heldout_motion_span_s = (heldout.back().time_ns - heldout.front().time_ns) / 1e9;
    output_.motion_evaluable = variance_t > 1e-9 &&
      output_.heldout_motion_span_s >= config_.audit_min_motion_span_s;
    output_.heldout_center_speed_mps = output_.motion_evaluable
      ? std::hypot(covariance_x, covariance_y) / variance_t
      : std::numeric_limits<double>::quiet_NaN();
    output_.fitted = true;
    output_.evaluable = true;
    output_.reason = Reason::ready;
    output_.revision++;
    last_fit_ns_ = frame_ns;
    // The prediction model is the training-only fit. Never use the held-out
    // points to refit and then claim that its earlier held-out score validates it.
    return output_;
  }

  const Output & output() const { return output_; }
  const Config & config() const { return config_; }
  const Counters & counters() const { return counters_; }
  bool matches_context(const Context & context) const
  { return context_ && same_context(*context_, context); }

  Forecast forecast(std::int64_t issued_ns, std::int64_t prediction_ns, int face_id) const
  {
    Forecast result;
    result.fit = output_;
    result.reason = output_.reason;
    result.issued_ns = issued_ns; result.prediction_ns = prediction_ns; result.face_id = face_id;
    const auto reject = [&](Reason reason) { result.reason = reason; return result; };
    if (!valid_face(face_id)) return reject(Reason::invalid_face);
    if (issued_ns <= 0 || issued_ns < last_frame_ns_ || prediction_ns <= issued_ns ||
        prediction_ns - issued_ns > 2000000000LL) return reject(Reason::invalid_time);
    if (!output_.evaluable || !last_fit_ns_) return result;
    if (!last_observation_ns_ || issued_ns - last_observation_ns_ > config_.max_frame_gap_ns)
      return reject(Reason::stale_observation);
    if (issued_ns - last_fit_ns_ > config_.max_prediction_age_ns)
      return reject(Reason::stale_model);
    const auto & s = output_.model.state;
    const Parameters p{s.center_x, s.center_y, s.phase_rad, s.omega_rad_s,
      s.radius_m[0], s.radius_m[1]};
    result.xyz = position(p, s.height_m, output_.model.reference_ns, prediction_ns, face_id);
    if (!valid_xyz(result.xyz)) return reject(Reason::invalid_input);
    result.available = true;
    if (!output_.motion_evaluable || !std::isfinite(output_.heldout_center_speed_mps))
      return reject(Reason::motion_coverage);
    if (output_.heldout_center_speed_mps > config_.audit_max_center_speed_mps)
      return reject(Reason::moving_center);
    if (!std::isfinite(output_.heldout_yaw_p90_rad) ||
        !std::isfinite(output_.heldout_position_p90_m) ||
        output_.heldout_yaw_p90_rad > config_.audit_max_yaw_p90_rad ||
        output_.heldout_position_p90_m > config_.audit_max_position_p90_m)
      return reject(Reason::holdout_error);
    result.stationary_supported = true;
    result.reason = Reason::ready;
    return result;
  }

  std::optional<Xyz> predict(std::int64_t prediction_ns, int face_id) const
  {
    if (!output_.evaluable || !valid_face(face_id) || prediction_ns <= last_frame_ns_ ||
        prediction_ns - last_frame_ns_ > 2000000000LL || !last_fit_ns_ ||
        last_frame_ns_ - last_fit_ns_ > config_.max_prediction_age_ns) return {};
    const auto & s = output_.model.state;
    const Parameters p{s.center_x, s.center_y, s.phase_rad, s.omega_rad_s,
      s.radius_m[0], s.radius_m[1]};
    return position(p, s.height_m, output_.model.reference_ns, prediction_ns, face_id);
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  using Parameters = std::array<double, 6>;
  using Matrix = std::array<Parameters, 6>;
  struct Sample { std::int64_t time_ns; int face; Xyz xyz; Seed seed; };
  Config config_;
  std::deque<Sample> samples_;
  std::optional<Context> context_;
  Output output_;
  Counters counters_;
  std::int64_t last_frame_ns_ = 0, last_stored_ns_ = 0, last_fit_attempt_ns_ = 0;
  std::int64_t last_fit_ns_ = 0, last_observation_ns_ = 0;
  std::uint64_t epoch_ = 0;

  static bool positive(double v) { return std::isfinite(v) && v > 0.0; }
  static bool valid_face(int face) { return face >= 0 && face < 4; }
  static bool valid_xyz(const Xyz & xyz)
  {
    return std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]) &&
      std::isfinite(std::hypot(xyz[0], xyz[1])) && std::hypot(xyz[0], xyz[1]) > 0.1;
  }
  bool valid_context(const Context & c) const
  {
    return c.model_generation && c.target_id >= 0 && c.source >= 0 && c.geometry >= 0 &&
      c.speed_mode >= 0 && (c.rotation_sign == -1 || c.rotation_sign == 1) &&
      positive(c.bullet_speed_mps) && std::isfinite(c.base_delay_s) && c.base_delay_s >= 0.0 &&
      positive(c.center_range_m) && std::isfinite(c.omega_rad_s) &&
      c.omega_rad_s * c.rotation_sign >= config_.min_omega_rad_s &&
      c.omega_rad_s * c.rotation_sign <= config_.max_omega_rad_s;
  }
  static bool same_context(const Context & a, const Context & b)
  {
    return a.model_generation == b.model_generation && a.target_id == b.target_id &&
      a.geometry == b.geometry && a.source == b.source && a.speed_mode == b.speed_mode &&
      a.rotation_sign == b.rotation_sign &&
      std::abs(a.bullet_speed_mps - b.bullet_speed_mps) <= 0.5 &&
      std::abs(a.base_delay_s - b.base_delay_s) <= 0.001 &&
      std::abs(a.omega_rad_s - b.omega_rad_s) <= 1.0 &&
      std::abs(a.center_range_m - b.center_range_m) <= 0.5;
  }
  bool valid_seed(const Seed & s, int sign) const
  {
    return std::isfinite(s.center_x) && std::isfinite(s.center_y) &&
      std::hypot(s.center_x, s.center_y) > 0.1 && std::isfinite(s.phase_rad) &&
      std::isfinite(s.omega_rad_s) && s.omega_rad_s * sign >= config_.min_omega_rad_s &&
      s.omega_rad_s * sign <= config_.max_omega_rad_s &&
      positive(s.radius_m[0]) && positive(s.radius_m[1]) &&
      s.radius_m[0] >= config_.min_radius_m && s.radius_m[1] >= config_.min_radius_m &&
      s.radius_m[0] <= config_.max_radius_m && s.radius_m[1] <= config_.max_radius_m &&
      std::isfinite(s.height_m[0]) && std::isfinite(s.height_m[1]);
  }
  bool valid_parameters(const Parameters & p, int sign) const
  {
    return valid_seed({p[0], p[1], p[2], p[3], {p[4], p[5]}, {0.0, 0.0}}, sign);
  }
  static double median(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : values[n / 2 - 1] / 2.0 + values[n / 2] / 2.0;
  }
  static double p90(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(std::ceil(0.9 * values.size())) - 1];
  }
  static Xyz position(const Parameters & p, const std::array<double, 2> & heights,
    std::int64_t reference_ns, std::int64_t time_ns, int face)
  {
    const double angle = p[2] + p[3] * ((time_ns - reference_ns) / 1e9) + face * pi / 2.0;
    return {p[0] - p[4 + face % 2] * std::cos(angle),
      p[1] - p[4 + face % 2] * std::sin(angle), heights[face % 2]};
  }
  std::array<double, 2> residual(const Sample & s, std::int64_t reference_ns,
    const Parameters & p) const
  {
    const auto point = position(p, {0.0, 0.0}, reference_ns, s.time_ns, s.face);
    const double range = std::hypot(s.xyz[0], s.xyz[1]);
    const double rx = s.xyz[0] / range, ry = s.xyz[1] / range;
    const double dx = s.xyz[0] - point[0], dy = s.xyz[1] - point[1];
    return {(rx * dx + ry * dy) / config_.radial_scale_m,
      (-ry * dx + rx * dy) / config_.transverse_scale_m};
  }
  double cost(const std::vector<Sample> & samples, std::int64_t reference_ns,
    const Parameters & p) const
  {
    double result = 0.0;
    for (const auto & s : samples) {
      const auto r = residual(s, reference_ns, p);
      const double norm = std::hypot(r[0], r[1]);
      result += norm <= config_.huber_norm ? norm * norm / 2.0 :
        config_.huber_norm * (norm - config_.huber_norm / 2.0);
    }
    return result / samples.size();
  }
  // Scale the normal matrix before pivoting. A rank-deficient model is refused;
  // no diagonal prior is added to disguise missing geometric observability.
  static bool solve(Matrix normal, Parameters rhs, Parameters & solution)
  {
    Parameters scale{};
    for (std::size_t i = 0; i < 6; ++i) {
      if (!positive(normal[i][i])) return false;
      scale[i] = std::sqrt(normal[i][i]);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      rhs[i] /= scale[i];
      for (std::size_t j = 0; j < 6; ++j) normal[i][j] /= scale[i] * scale[j];
    }
    for (std::size_t column = 0; column < 6; ++column) {
      auto pivot = column;
      for (std::size_t row = column + 1; row < 6; ++row)
        if (std::abs(normal[row][column]) > std::abs(normal[pivot][column])) pivot = row;
      if (!std::isfinite(normal[pivot][column]) || std::abs(normal[pivot][column]) < 1e-8)
        return false;
      std::swap(normal[pivot], normal[column]);
      std::swap(rhs[pivot], rhs[column]);
      for (std::size_t row = column + 1; row < 6; ++row) {
        const double ratio = normal[row][column] / normal[column][column];
        for (std::size_t j = column; j < 6; ++j) normal[row][j] -= ratio * normal[column][j];
        rhs[row] -= ratio * rhs[column];
      }
    }
    Parameters scaled_solution{};
    for (int row = 5; row >= 0; --row) {
      double value = rhs[static_cast<std::size_t>(row)];
      for (std::size_t j = static_cast<std::size_t>(row) + 1; j < 6; ++j)
        value -= normal[static_cast<std::size_t>(row)][j] * scaled_solution[j];
      scaled_solution[static_cast<std::size_t>(row)] = value /
        normal[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
      solution[static_cast<std::size_t>(row)] = scaled_solution[static_cast<std::size_t>(row)] /
        scale[static_cast<std::size_t>(row)];
      if (!std::isfinite(solution[static_cast<std::size_t>(row)])) return false;
    }
    return true;
  }
  bool fit(const std::vector<Sample> & samples, std::int64_t reference_ns, int sign,
    Parameters & p) const
  {
    for (int iteration = 0; iteration < 24; ++iteration) {
      Matrix normal{};
      Parameters rhs{};
      for (const auto & s : samples) {
        const double dt = (s.time_ns - reference_ns) / 1e9;
        const double angle = p[2] + p[3] * dt + s.face * pi / 2.0;
        const double sa = std::sin(angle), ca = std::cos(angle), radius = p[4 + s.face % 2];
        Parameters jx{1.0, 0.0, radius * sa, radius * sa * dt, 0.0, 0.0};
        Parameters jy{0.0, 1.0, -radius * ca, -radius * ca * dt, 0.0, 0.0};
        jx[4 + s.face % 2] = -ca;
        jy[4 + s.face % 2] = -sa;
        const double range = std::hypot(s.xyz[0], s.xyz[1]);
        const double rx = s.xyz[0] / range, ry = s.xyz[1] / range;
        std::array<Parameters, 2> jacobian{};
        for (std::size_t j = 0; j < 6; ++j) {
          jacobian[0][j] = (rx * jx[j] + ry * jy[j]) / config_.radial_scale_m;
          jacobian[1][j] = (-ry * jx[j] + rx * jy[j]) / config_.transverse_scale_m;
        }
        const auto r = residual(s, reference_ns, p);
        const double norm = std::hypot(r[0], r[1]);
        const double weight = norm > config_.huber_norm ? config_.huber_norm / norm : 1.0;
        for (std::size_t i = 0; i < 6; ++i) {
          for (std::size_t j = 0; j < 6; ++j)
            normal[i][j] += weight *
              (jacobian[0][i] * jacobian[0][j] + jacobian[1][i] * jacobian[1][j]);
          rhs[i] += weight * (jacobian[0][i] * r[0] + jacobian[1][i] * r[1]);
        }
      }
      Parameters step{};
      if (!solve(normal, rhs, step)) return false;
      double maximum_step = 0.0;
      for (double value : step) maximum_step = std::max(maximum_step, std::abs(value));
      if (maximum_step < 1e-7) return valid_parameters(p, sign);
      const double old_cost = cost(samples, reference_ns, p);
      bool improved = false;
      for (double factor = 1.0; factor >= 1.0 / 128.0; factor /= 2.0) {
        Parameters candidate = p;
        for (std::size_t j = 0; j < 6; ++j) candidate[j] += factor * step[j];
        if (!valid_parameters(candidate, sign)) continue;
        const double new_cost = cost(samples, reference_ns, candidate);
        if (std::isfinite(new_cost) && new_cost < old_cost) {
          p = candidate;
          improved = true;
          if (old_cost - new_cost <= 1e-9 * (1.0 + old_cost)) return true;
          break;
        }
      }
      if (!improved) return false;
    }
    return false;  // A finite but unconverged fit is still not an evaluable model.
  }
};

}  // namespace auto_aim

#endif  // AUTO_AIM__FIXED_CENTER_SHADOW_HPP
