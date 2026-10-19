#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "armor_motion_filter.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "yaw_eskf.hpp"

namespace auto_aim
{

class Target
{
public:
  struct IteratedUpdateConfig
  {
    bool enabled = true;
    int max_iterations = 4;
    double stop_threshold = 1e-4;
    double huber_threshold = 2.5;
  };

  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig,
    const YawESKF::Config & yaw_eskf_config = YawESKF::Config{});
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, const YawESKF::Config & yaw_eskf_config,
    const IteratedUpdateConfig & iterated_update_config);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  void update(const Armor & armor, int armor_id);
  void update_batch(
    const std::vector<std::pair<Armor, int>> & observations, int primary_armor_id);

  // Seed the two geometric offsets that only the off-axis faces (1 and 3) can
  // observe.  They carry no process noise, so their starting value is what the
  // predicted positions of those faces are based on until one of them is
  // associated.  Leaving them at zero puts faces 1/3 on the same radius as
  // faces 0/2, which biases their predicted image position and keeps their
  // association cost above the gate.  Called once, right after construction.
  void seed_geometry(double radius_delta, double height_delta);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  bool yaw_eskf_trusted() const;
  double yaw_eskf_angle() const;
  double yaw_eskf_rate() const;
  double yaw_eskf_acceleration() const;
  double yaw_eskf_nis() const;
  int yaw_eskf_accepted_updates() const;
  int yaw_eskf_rejected_updates() const;
  bool armor_motion_trusted(int id) const;
  Eigen::Vector3d armor_motion_velocity(int id) const;
  double armor_motion_nis(int id) const;
  std::vector<Eigen::Vector4d> armor_xyza_list(bool prefer_direct_motion = false) const;
  // 直接用主 EKF 的 11 维状态生成四块板（x[6] 是整车 yaw，不被 YawESKF 覆盖）。
  // 小陀螺时 Aimer 走这条路径：YawESKF 的角速度在换面节律上有 0.5~0.9 rad/s 的摆动，
  // 用它外推弹着时刻会把选中的板选到 45° 以外（2026-10-19 借鉴 Fujian_AutoAim）。
  std::vector<Eigen::Vector4d> whole_ekf_armor_xyza_list() const;
  int armor_num() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  YawESKF yaw_eskf_;
  std::array<ArmorMotionFilter, 4> armor_motion_filters_;
  IteratedUpdateConfig iterated_update_config_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_yaw_eskf(const Armor & armor, int id);

  // Bound the two geometric offsets that only the off-axis faces constrain.
  //
  // `l = r2 - r1` and `h` receive no process noise, so a single mis-associated
  // observation of face 1 or face 3 can move them far from zero in one update.
  // Field logs showed `l` going 0.000 -> 0.435 m in a single frame, which pushed
  // `r + l` outside diverged()'s 0.05..0.5 band and reset an otherwise healthy
  // track.  A real 4-armor chassis has |r2 - r1| of a few centimetres, so the
  // estimate is clamped to that physical band after every update.
  void clamp_geometry();

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
