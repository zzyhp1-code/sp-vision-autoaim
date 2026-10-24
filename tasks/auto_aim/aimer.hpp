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

enum class AimPredictionFailure {
  none, no_target, initial_selection, iterative_selection, trajectory, non_finite
};

inline const char * prediction_failure_name(AimPredictionFailure failure)
{
  switch (failure) {
    case AimPredictionFailure::none: return "none";
    case AimPredictionFailure::no_target: return "no_target";
    case AimPredictionFailure::initial_selection: return "initial_selection";
    case AimPredictionFailure::iterative_selection: return "iterative_selection";
    case AimPredictionFailure::trajectory: return "trajectory";
    case AimPredictionFailure::non_finite: return "non_finite";
  }
  return "unknown";
}

// 只做观测，不参与控制。
struct AimPredictionTrace
{
  AimPredictionFailure failure = AimPredictionFailure::none;
  bool valid = false, converged = false;
  bool whole_ekf_geometry = false;
  int face_id = -1, iterations = 0;
  int64_t base_prediction_ns = 0, prediction_ns = 0;
  int64_t evaluation_ns = 0;
  double delay_s = std::numeric_limits<double>::quiet_NaN();
  double speed_mps = std::numeric_limits<double>::quiet_NaN();
  double flight_time_used_s = std::numeric_limits<double>::quiet_NaN();
  double flight_time_s = std::numeric_limits<double>::quiet_NaN();
  bool rotation_override_used = false;
  double rotation_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
};

struct AimPredictionContext
{
  std::optional<std::chrono::steady_clock::time_point> evaluation_time;
  std::optional<double> rotation_rate_rad_s;
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
    const AimPredictionContext & context = {});

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing,
    const AimPredictionContext & context = {});

  // Applies the same bounded runtime correction to both speed branches.  Their
  // calibrated base values remain unchanged and are restored with zero.
  void set_delay_adjustment(double adjustment_s);

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
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
