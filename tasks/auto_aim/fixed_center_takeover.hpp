#ifndef AUTO_AIM__FIXED_CENTER_TAKEOVER_HPP
#define AUTO_AIM__FIXED_CENTER_TAKEOVER_HPP

#include "fixed_center_shadow.hpp"

namespace auto_aim
{

// Immutable invocation snapshot. Never installed in Target or Tracker.
struct FixedCenterAimModel
{
  FixedCenterShadow::Model model;
  FacePredictionResidual::Context context;
  std::uint64_t epoch = 0, revision = 0;
  std::int64_t valid_until_ns = 0;

  bool valid_at(std::int64_t now_ns) const
  {
    const auto & s = model.state;
    return context.model_generation && context.target_id >= 0 && revision > 0 &&
      (context.rotation_sign == -1 || context.rotation_sign == 1) &&
      model.reference_ns > 0 && now_ns >= model.reference_ns && now_ns <= valid_until_ns &&
      valid_until_ns >= model.reference_ns && valid_until_ns - model.reference_ns <= 250000000 &&
      std::isfinite(s.center_x) && std::isfinite(s.center_y) &&
      std::isfinite(std::hypot(s.center_x, s.center_y)) && std::hypot(s.center_x, s.center_y) > 0.1 &&
      std::isfinite(s.phase_rad) && std::isfinite(s.omega_rad_s) &&
      s.omega_rad_s * context.rotation_sign >= 0.5 && std::abs(s.omega_rad_s) <= 15.0 &&
      std::abs(s.omega_rad_s - context.omega_rad_s) <= 1.0 &&
      std::isfinite(s.radius_m[0]) && std::isfinite(s.radius_m[1]) &&
      s.radius_m[0] > 0.05 && s.radius_m[0] < 0.5 &&
      s.radius_m[1] > 0.05 && s.radius_m[1] < 0.5 &&
      std::isfinite(s.height_m[0]) && std::isfinite(s.height_m[1]);
  }

  std::array<double, 4> face_at(std::int64_t time_ns, int face) const
  {
    const auto & s = model.state;
    const double phase = std::remainder(s.phase_rad + s.omega_rad_s *
      ((static_cast<double>(time_ns) - model.reference_ns) / 1e9) +
      face * 1.57079632679489661923, 6.2831853071795864769);
    return {s.center_x - s.radius_m[face % 2] * std::cos(phase),
      s.center_y - s.radius_m[face % 2] * std::sin(phase), s.height_m[face % 2], phase};
  }
};

// Selection quality only. Rejecting the candidate selects the original model;
// this policy has no firing permission, cadence or actuator API.
class FixedCenterTakeover
{
public:
  struct Decision
  {
    bool use_fixed = false;
    const char * reason = "disabled";
    FixedCenterShadow::Forecast forecast;
    FixedCenterAimModel snapshot;
  };

  void reset() { epoch_ = generation_ = revision_ = 0; stable_since_ns_ = 0; fits_ = 0; }

  // Ownership ends only when the model stopped being this model: another target,
  // model generation, speed mode or rotation direction, a fit that no longer
  // describes a stationary chassis, an expired snapshot or an explicit rebuild.
  // A single untrusted, uncovered or briefly unusable frame closes the request
  // for that frame but keeps the quality history: rebuilding it costs three new
  // fits and 200 ms of uninterrupted support. Field log 2026-10-28_01-29-51
  // spent 139 records in `settling` and reset ownership on `moving_center`
  // (130) / `ineligible` (62) / `coverage` (14) / `holdout_error` (5).
  Decision update(bool enabled, bool eligible, std::int64_t frame_ns, std::int64_t now_ns,
    const FacePredictionResidual::Context & context, const FixedCenterShadow & shadow,
    bool fresh_observation = true)
  {
    Decision out;
    const auto invalidate = [&](const char * reason) { reset(); out.reason = reason; return out; };
    const auto hold = [&](const char * reason) { out.reason = reason; return out; };
    if (!enabled) return invalidate("disabled");
    if (!shadow.matches_context(context)) return invalidate("context_changed");
    out.forecast = shadow.forecast(now_ns, now_ns + 1, 0);
    if (!out.forecast.available) {
      // Transient: no evaluable fit yet (warmup/coverage/motion_coverage) or a
      // quality rejection of the newest fit. Invalid: the fit is gone or the
      // model no longer describes this target. Only the second kind may erase
      // ownership; the first kind is retried on the next frame.
      const auto reason = out.forecast.reason;
      const bool invalidated = reason == FixedCenterShadow::Reason::stale_model ||
        reason == FixedCenterShadow::Reason::stale_observation ||
        reason == FixedCenterShadow::Reason::clock_gap ||
        reason == FixedCenterShadow::Reason::fit_failed ||
        reason == FixedCenterShadow::Reason::invalid_input ||
        reason == FixedCenterShadow::Reason::invalid_face ||
        reason == FixedCenterShadow::Reason::context_changed;
      return invalidated ? invalidate(FixedCenterShadow::reason_name(reason))
                         : hold(FixedCenterShadow::reason_name(reason));
    }
    // Movement or held-out error rejects this frame's authority, never the fit
    // itself: the next successful revision is judged on its own evidence.
    if (!out.forecast.stationary_supported)
      return hold(FixedCenterShadow::reason_name(out.forecast.reason));
    const auto & fit = out.forecast.fit;
    if (fit.epoch != epoch_ || context.model_generation != generation_) {
      reset(); epoch_ = fit.epoch; generation_ = context.model_generation;
    }
    if (!fits_) { stable_since_ns_ = now_ns; revision_ = fit.revision; fits_ = 1; }
    else if (fit.revision > revision_) { revision_ = fit.revision; fits_ = std::min(3, fits_ + 1); }
    out.snapshot = {fit.model, context, fit.epoch, fit.revision,
      std::min(fit.model.reference_ns + shadow.config().max_prediction_age_ns,
        frame_ns + shadow.config().max_frame_gap_ns)};
    if (!out.snapshot.valid_at(now_ns)) return invalidate("invalid_snapshot");
    // Losing tracker trust, convergence or the high-speed mode for a frame is a
    // request-opportunity gap; the fit keeps maturing underneath it.
    if (!eligible) return hold("ineligible");
    // Repeated calls on one cached fit cannot mature a candidate. Require
    // three successful revisions and 200 ms of uninterrupted quality support.
    if (fits_ < 3 || now_ns - stable_since_ns_ < 200000000) {
      out.reason = "settling"; return out;
    }
    // A brief visual gap cannot use this predictor for control or requests.
    // Keep a still-fresh model's quality history so recovery does not demand
    // another full settling interval after every normal face hand-over.
    if (!fresh_observation) { out.reason = "no_observation"; return out; }
    out.use_fixed = true; out.reason = "ready";
    return out;
  }

private:
  std::uint64_t epoch_ = 0, generation_ = 0, revision_ = 0;
  std::int64_t stable_since_ns_ = 0;
  int fits_ = 0;
};

}  // namespace auto_aim
#endif
