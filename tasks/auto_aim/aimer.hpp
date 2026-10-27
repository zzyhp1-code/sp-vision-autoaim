#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <limits>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"
#include "fixed_center_takeover.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

// 选板/预测用哪套几何：
//   existing  —— 沿用 ekf_x()（x[6]/x[7] 被 YawESKF 覆盖），静止/低速/前哨站走这条；
//   whole_ekf —— 直接用主 EKF 的 11 维状态（小陀螺时用）。YawESKF 的角速度在换面
//                节律上有 0.5~0.9 rad/s 摆动，用它外推弹着时刻会把板选到 45° 以外；
//                主 EKF 的整车 yaw 更稳，代价是不做单板精修。
enum class AimGeometry { existing, whole_ekf };

inline const char * aim_geometry_name(AimGeometry geometry)
{
  return geometry == AimGeometry::whole_ekf ? "whole_ekf" : "existing";
}

// One frame's model decision, shared by prediction and the caller's mode/trust
// gates. The state and angular rate always come from the selected geometry.
struct AimPredictionModel
{
  AimGeometry geometry = AimGeometry::existing;
  Eigen::VectorXd state;
  bool finite = false, trusted = false;
  double angular_velocity_rad_s = std::numeric_limits<double>::quiet_NaN();
};

enum class AimPredictionFailure {
  none, no_target, initial_selection, iterative_selection, final_selection,
  trajectory, non_finite, not_converged, stale_model
};

inline const char * prediction_failure_name(AimPredictionFailure failure)
{
  switch (failure) {
    case AimPredictionFailure::none: return "none";
    case AimPredictionFailure::no_target: return "no_target";
    case AimPredictionFailure::initial_selection: return "initial_selection";
    case AimPredictionFailure::iterative_selection: return "iterative_selection";
    case AimPredictionFailure::final_selection: return "final_selection";
    case AimPredictionFailure::trajectory: return "trajectory";
    case AimPredictionFailure::non_finite: return "non_finite";
    case AimPredictionFailure::not_converged: return "not_converged";
    case AimPredictionFailure::stale_model: return "stale_model";
  }
  return "unknown";
}

// Trace of the predictor actually selected for this invocation.
struct AimPredictionTrace
{
  AimPredictionFailure failure = AimPredictionFailure::none;
  bool valid = false, converged = false;
  bool whole_ekf_geometry = false;
  bool fixed_face = false;
  bool fixed_center = false;
  std::uint64_t fixed_center_epoch = 0, fixed_center_revision = 0;
  std::int64_t model_reference_ns = 0, model_valid_until_ns = 0;
  AimGeometry geometry = AimGeometry::existing;
  int face_id = -1, iterations = 0;
  int64_t base_prediction_ns = 0, prediction_ns = 0;
  double delay_s = std::numeric_limits<double>::quiet_NaN();
  double speed_mps = std::numeric_limits<double>::quiet_NaN();
  double flight_time_used_s = std::numeric_limits<double>::quiet_NaN();
  double flight_time_s = std::numeric_limits<double>::quiet_NaN();
  double angular_velocity_rad_s = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector4d aim_xyza = Eigen::Vector4d::Constant(
    std::numeric_limits<double>::quiet_NaN());
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing,
    const FixedCenterAimModel * fixed_center = nullptr);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing,
    const FixedCenterAimModel * fixed_center = nullptr);

  // Applies the same bounded runtime correction to both speed branches.  Their
  // calibrated base values remain unchanged and are restored with zero.
  void set_delay_adjustment(double adjustment_s);

  AimPredictionModel prediction_model(
    const Target & target, AimGeometry requested = AimGeometry::whole_ekf) const;

  void reset_target()
  {
    lock_id_ = -1;
    debug_aim_point = {false, Eigen::Vector4d::Zero()};
  }

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double delay_adjustment_time_ = 0.0;
  double decision_speed_;

  AimPoint choose_aim_point(
    const Target & target, bool whole_ekf_geometry, int * face_id = nullptr);

  bool rotation_face_selectable(
    const Target & target, const Eigen::VectorXd & state,
    const Eigen::Vector4d & face) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
