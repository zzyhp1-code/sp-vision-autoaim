#ifndef AUTO_AIM__YAW_ESKF_HPP
#define AUTO_AIM__YAW_ESKF_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim
{

// Error-state filter on SO(2) for rigid-body yaw.  The nominal angle remains
// continuous while every measurement innovation is evaluated on the circle.
// State: [body_yaw, yaw_rate, yaw_acceleration].
class YawESKF
{
public:
  struct Config
  {
    double angular_jerk_variance = 20.0;
    double measurement_variance_floor = std::pow(1.0 * M_PI / 180.0, 2);
    double nis_reject_threshold = 16.0;
    // Deprecated: predictions are no longer clamped to this value.  F and Q are
    // exact discretisations for any dt, and clamping desynchronised the state
    // from the timestamp the caller asked for.  Kept only so existing
    // aggregate initialisers and the YAML loader still compile.
    double max_dt_s = 0.10;
    int min_trusted_updates = 6;
    double trusted_rate_variance = 4.0;
    double max_trusted_abs_rate_rad_s = std::numeric_limits<double>::infinity();
    double max_trusted_abs_acceleration_rad_s2 =
      std::numeric_limits<double>::infinity();
  };

  struct UpdateResult
  {
    bool accepted = false;
    double innovation_rad = std::numeric_limits<double>::quiet_NaN();
    double nis = std::numeric_limits<double>::quiet_NaN();
  };

  YawESKF() = default;
  explicit YawESKF(const Config & config) : config_(config) {}

  void reset(double measured_yaw_rad, double initial_rate_rad_s = 0.0)
  {
    x_ << measured_yaw_rad, initial_rate_rad_s, 0.0;
    P_.setZero();
    P_(0, 0) = std::pow(30.0 * M_PI / 180.0, 2);
    P_(1, 1) = 100.0;
    P_(2, 2) = 400.0;
    initialized_ = std::isfinite(measured_yaw_rad) &&
      std::isfinite(initial_rate_rad_s);
    accepted_updates_ = 0;
    rejected_updates_ = 0;
    last_nis_ = std::numeric_limits<double>::quiet_NaN();
  }

  // Propagate the filter over the FULL dt_s.
  //
  // The ballistic solver asks for predictions as long as the projectile flight
  // time (~0.15-0.30 s).  A single call used to be clamped to max_dt_s
  // (0.10 s), so the body angle advanced only a fraction of the required
  // rotation while Target::t_ still moved to the requested time.  Because
  // Target::ekf_x() overwrites the body angle/rate with this filter's estimate
  // whenever it is trusted, that truncation made the aim point lag the real
  // rotation and could select the neighbouring face (90 deg away).
  //
  // F and Q below are exact discretisations of the white-noise-jerk driven
  // constant-acceleration model for any dt > 0, so the whole interval is
  // propagated in one step.  No clamping happens here: silently shortening dt
  // would desynchronise the state from t_.  A long extrapolation instead grows
  // P, which makes trusted() false and lets Target::ekf_x() fall back to the
  // main EKF -- that is the intended guard.
  //
  // Negative dt is still ignored, as before.  Planner::get_trajectory rewinds
  // the target to seed its finite-difference velocity, but a backward step is
  // not the time reverse of a forward one: the process noise must be carried
  // through D*Q*D with D = diag(1, -1, 1).  Extrapolating backwards with plain
  // Q would be inconsistent, and recovering the historical posterior would
  // need smoothing rather than a negative prediction.  The rewind only feeds
  // the planner's own velocity estimate, so leaving it out keeps the previous
  // behaviour for that path while fixing the forward case that matters here.
  void predict(double dt_s)
  {
    if (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0.0) return;

    // Guard against absurd inputs (stalled clock, corrupt timestamp) that would
    // overflow the polynomial terms.  Such a gap is not a usable prediction.
    if (dt_s > kMaxPropagationDtS) {
      initialized_ = false;
      return;
    }

    const double dt = dt_s;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    Eigen::Matrix3d F;
    F << 1.0, dt, 0.5 * dt2,
         0.0, 1.0, dt,
         0.0, 0.0, 1.0;
    x_ = F * x_;

    const double q = std::max(0.0, config_.angular_jerk_variance);
    Eigen::Matrix3d Q;
    Q << dt5 / 20.0, dt4 / 8.0, dt3 / 6.0,
         dt4 / 8.0, dt3 / 3.0, dt2 / 2.0,
         dt3 / 6.0, dt2 / 2.0, dt;
    P_ = F * P_ * F.transpose() + q * Q;

    stabilize_covariance();
  }

  UpdateResult update(double measured_body_yaw_rad, double measurement_variance)
  {
    UpdateResult out;
    if (!std::isfinite(measured_body_yaw_rad)) return out;
    if (!initialized_) {
      reset(measured_body_yaw_rad);
      out.accepted = true;
      out.innovation_rad = 0.0;
      out.nis = 0.0;
      return out;
    }

    const double R = std::max(
      std::max(1e-12, config_.measurement_variance_floor),
      std::isfinite(measurement_variance) ? measurement_variance : 0.0);
    const double innovation = std::remainder(
      measured_body_yaw_rad - x_[0], 2.0 * M_PI);
    const double S = P_(0, 0) + R;
    if (!std::isfinite(S) || S <= 1e-12) return out;

    out.innovation_rad = innovation;
    out.nis = innovation * innovation / S;
    last_nis_ = out.nis;
    if (!std::isfinite(out.nis) ||
        out.nis > std::max(0.0, config_.nis_reject_threshold)) {
      ++rejected_updates_;
      return out;
    }

    const Eigen::Vector3d K = P_.col(0) / S;
    const Eigen::Vector3d error_state = K * innovation;
    x_ += error_state;

    // Joseph form preserves positive semidefiniteness after the error-state
    // injection.  The SO(2) reset Jacobian is identity for an additive tangent.
    Eigen::RowVector3d H;
    H << 1.0, 0.0, 0.0;
    const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    stabilize_covariance();
    ++accepted_updates_;
    out.accepted = true;
    return out;
  }

  // Identical armor faces make body-yaw observations equivalent modulo one
  // face pitch. Map the observation to the branch nearest the continuous
  // nominal state before updating. A transient face-id hand-over can then no
  // longer inject a +/-90 degree step into the rate estimate of a four-armor
  // vehicle.
  UpdateResult update_equivalent(
    double measured_body_yaw_rad, double measurement_variance,
    double equivalence_period_rad)
  {
    // With identical faces an early association error can select the wrong
    // periodic branch and drive the constant-acceleration state to hundreds of
    // rad/s while still producing a small wrapped innovation.  Recover at the
    // measurement boundary before that state can escape into Aimer/model
    // selection.  Resetting the rate to zero costs only the normal six-update
    // trust dwell and is much safer than perpetually extrapolating a bad alias.
    if (initialized_ && !dynamics_within_configured_bounds()) {
      reset(measured_body_yaw_rad);
    }
    if (initialized_ && std::isfinite(measured_body_yaw_rad) &&
        std::isfinite(equivalence_period_rad) && equivalence_period_rad > 1e-6 &&
        equivalence_period_rad < 2.0 * M_PI) {
      measured_body_yaw_rad += equivalence_period_rad * std::round(
        (x_[0] - measured_body_yaw_rad) / equivalence_period_rad);
    }
    auto out = update(measured_body_yaw_rad, measurement_variance);
    if (initialized_ && !dynamics_within_configured_bounds()) {
      reset(measured_body_yaw_rad);
      out.accepted = true;
      out.innovation_rad = 0.0;
      out.nis = 0.0;
    }
    return out;
  }

  bool initialized() const { return initialized_; }
  bool trusted() const
  {
    return initialized_ &&
      accepted_updates_ >= std::max(1, config_.min_trusted_updates) &&
      std::isfinite(P_(1, 1)) &&
      P_(1, 1) <= std::max(1e-6, config_.trusted_rate_variance) &&
      std::isfinite(x_[1]) && std::isfinite(x_[2]) &&
      std::abs(x_[1]) <= std::max(0.0, config_.max_trusted_abs_rate_rad_s) &&
      std::abs(x_[2]) <=
        std::max(0.0, config_.max_trusted_abs_acceleration_rad_s2);
  }

  double angle() const { return std::remainder(x_[0], 2.0 * M_PI); }
  double continuous_angle() const { return x_[0]; }
  double rate() const { return x_[1]; }
  double acceleration() const { return x_[2]; }
  double last_nis() const { return last_nis_; }
  int accepted_updates() const { return accepted_updates_; }
  int rejected_updates() const { return rejected_updates_; }
  const Eigen::Matrix3d & covariance() const { return P_; }

private:
  // Hard ceiling for a single propagation request.  A real ballistic prediction
  // is at most a few hundred milliseconds; anything near this bound means the
  // clock stalled or the timestamp is corrupt, so the state is declared
  // invalid instead of extrapolating garbage.  Note this is NOT a prediction
  // horizon: the horizon is governed by the covariance growth that trusted()
  // observes.
  static constexpr double kMaxPropagationDtS = 2.0;

  bool dynamics_within_configured_bounds() const
  {
    return std::isfinite(x_[1]) && std::isfinite(x_[2]) &&
      std::abs(x_[1]) <= std::max(0.0, config_.max_trusted_abs_rate_rad_s) &&
      std::abs(x_[2]) <=
        std::max(0.0, config_.max_trusted_abs_acceleration_rad_s2);
  }

  void stabilize_covariance()
  {
    // .eval() forces a temporary: writing P_ while reading both P_ and
    // P_.transpose() in the same expression is an Eigen aliasing hazard.
    P_ = (0.5 * (P_ + P_.transpose())).eval();
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(P_(i, i)) || P_(i, i) < 1e-12) P_(i, i) = 1e-12;
    }
  }

  Config config_{};
  Eigen::Vector3d x_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
  bool initialized_ = false;
  int accepted_updates_ = 0;
  int rejected_updates_ = 0;
  double last_nis_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YAW_ESKF_HPP
