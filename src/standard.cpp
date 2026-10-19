#include <fmt/core.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>  // for setenv
#include <deque>
#include <iterator>
#include <thread>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

// ROS2 headers (仅在 ROS2 可用时编译，用于可视化)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include "tools/ros2_visualizer.hpp"
#endif

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/center_follow_lead.hpp"
#include "tasks/auto_aim/gyro_face_selector.hpp"
#include "tasks/auto_aim/gyro_phase_scheduler.hpp"
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include "tasks/auto_aim/shot_delay_adapter.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

using namespace std::chrono;

// 开火判据（2026-10-19 借鉴 Fujian_AutoAim 26.10.18 的 RotationFireWindow）：
// 不用"误差小于某个窗"，而是用最近两帧的实测弹着偏航误差外推下一帧，
// 在哪一帧误差到达谷底（下一步会变差）就那一帧开火。实现见下面 fire_candidate 之后的
// 【误差拐点开火】一段；这里不再保留状态机结构体，选板连续性由 stable-window 负责。

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 🚀 关键性能优化：全局线程控制
  // vtune分析发现88个线程抢16个核心，导致严重性能下降
  // - cv::cvtColor的Bayer转RGB触发TBB并行：61.660s (10.9%)
  // - 线程调度开销（__sched_yield）：94.800s (16.7%)
  // - Spin Time浪费：99.970s

  // 1. 强制OpenCV单线程（避免cvtColor创建大量线程）
  cv::setNumThreads(1);

  // 2. 限制TBB全局线程池（OpenVINO和OpenCV共用）
  setenv("TBB_NUM_THREADS", "4", 1);  // NUC有16核，4线程足够

  // 3. 限制OpenMP线程数（某些OpenCV编译可能使用）
  setenv("OMP_NUM_THREADS", "4", 1);

  tools::logger()->info("[Performance] Thread limits: OpenCV=1, TBB=4, OMP=4");

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto integration_yaml = YAML::LoadFile(config_path);
  const bool competition_headless =
    integration_yaml["competition_headless"] &&
    integration_yaml["competition_headless"].as<bool>();
  const bool direct_yaw_guard_enabled =
    integration_yaml["xuc_direct_yaw_guard"] &&
    integration_yaml["xuc_direct_yaw_guard"].as<bool>();
  const bool prediction_lead_limit_configured =
    integration_yaml["xuc_max_prediction_lead_rad"].IsDefined();
  const double max_prediction_lead_rad =
    prediction_lead_limit_configured
      ? std::max(0.0, integration_yaml["xuc_max_prediction_lead_rad"].as<double>())
      : 0.0;
  const double direct_yaw_filter_tau_s =
    integration_yaml["xuc_direct_yaw_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_hold_s =
    integration_yaml["xuc_direct_yaw_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_max_rate_rad_s =
    integration_yaml["xuc_direct_yaw_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double final_yaw_command_max_rate_rad_s =
    integration_yaml["xuc_final_yaw_command_max_rate_dps"]
      ? std::max(
          0.0, integration_yaml["xuc_final_yaw_command_max_rate_dps"].as<double>()) *
          M_PI / 180.0
      : 0.0;
  // Vehicle-centre follow deliberately tracks the rotation centre instead of the
  // visible plate, which removes plate-to-plate stepping.  A spinning vehicle
  // turns about its own centre, so that costs no ballistic lead at all; what can
  // lag is the centre estimate itself when the centre is genuinely translating.
  // When enabled, advance the centre by its own velocity over the flight time.
  const bool center_follow_lead_enabled =
    integration_yaml["xuc_center_follow_lead_enabled"] &&
    integration_yaml["xuc_center_follow_lead_enabled"].as<bool>();
  // Below this centre speed the vehicle is treated as spinning in place, where
  // the geometry owes no lead and a correction would only add noise.
  const double xuc_center_follow_lead_min_center_speed_mps =
    integration_yaml["xuc_center_follow_lead_min_center_speed_mps"]
      ? std::max(0.0,
          integration_yaml["xuc_center_follow_lead_min_center_speed_mps"].as<double>())
      : 0.30;
  // The armour sits only atan(r / d) off the centre bearing (~2.3 deg at
  // r = 0.26 m, d = 4.5 m), so the correction must stay well inside that.
  // Anything larger commands the gimbal off the plate instead of onto it; in
  // particular omega * flight_time (~21 deg) is roughly an order of magnitude
  // too large and is not a valid lead for a body rotating about its centre.
  const double center_follow_lead_max_rad =
    integration_yaml["xuc_center_follow_lead_max_deg"]
      ? std::clamp(
          integration_yaml["xuc_center_follow_lead_max_deg"].as<double>(),
          0.0, 5.0) * M_PI / 180.0
      : 3.0 * M_PI / 180.0;
  // Keep the fixed camera/gimbal boresight zero separate from the empirical
  // impact correction.  The former centers the visual line of sight; the latter
  // deliberately leaves the armor off-centre so the bore, not the camera, points
  // at the desired impact point.
  const double direct_yaw_boresight_correction_rad =
    integration_yaml["xuc_direct_yaw_boresight_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_boresight_correction_deg"].as<double>(),
          -5.0,
          5.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_impact_correction_rad =
    integration_yaml["xuc_direct_yaw_impact_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_impact_correction_deg"].as<double>(),
          -10.0,
          10.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_command_correction_rad = std::clamp(
    direct_yaw_boresight_correction_rad + direct_yaw_impact_correction_rad,
    -10.0 * M_PI / 180.0,
    10.0 * M_PI / 180.0);
  // The lower yaw loop can retain a static error even though the absolute target
  // is received correctly (field logs showed about 1.2 deg).  Close that gap in
  // the one coordinate both packet directions actually share: raw lower-board
  // yaw.  This layer sits after target prediction, so it removes actuator lag and
  // bias without replacing the ESKF/phase lead used for moving targets.
  const bool yaw_tracking_pi_enabled =
    integration_yaml["xuc_yaw_tracking_pi_enabled"] &&
    integration_yaml["xuc_yaw_tracking_pi_enabled"].as<bool>();
  const double yaw_tracking_kp = integration_yaml["xuc_yaw_tracking_kp"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_kp"].as<double>(), 0.0, 2.0)
    : 0.0;
  const double yaw_tracking_ki_per_s = integration_yaml["xuc_yaw_tracking_ki_per_s"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_ki_per_s"].as<double>(), 0.0, 5.0)
    : 0.0;
  const double yaw_tracking_integral_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integral_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integral_limit_deg"].as<double>(),
          0.0,
          5.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_output_limit_rad =
    (integration_yaml["xuc_yaw_tracking_output_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_output_limit_deg"].as<double>(),
          0.0,
          8.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_integrate_error_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"].as<double>(),
          0.1,
          10.0)
      : 3.0) * M_PI / 180.0;
  const double yaw_tracking_arm_dwell_s =
    (integration_yaml["xuc_yaw_tracking_arm_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_arm_dwell_ms"].as<double>(),
          0.0,
          2000.0)
      : 500.0) / 1000.0;
  if (yaw_tracking_pi_enabled) {
    tools::logger()->info(
      "[XUC][YAW-PI] enabled kp={:.2f} ki={:.2f}/s "
      "integral_limit={:.2f}deg output_limit={:.2f}deg "
      "integrate_error_limit={:.2f}deg arm_dwell={:.0f}ms",
      yaw_tracking_kp, yaw_tracking_ki_per_s,
      yaw_tracking_integral_limit_rad * 180.0 / M_PI,
      yaw_tracking_output_limit_rad * 180.0 / M_PI,
      yaw_tracking_integrate_error_limit_rad * 180.0 / M_PI,
      yaw_tracking_arm_dwell_s * 1000.0);
  }
  const bool direct_pitch_guard_enabled =
    integration_yaml["xuc_direct_pitch_guard"] &&
    integration_yaml["xuc_direct_pitch_guard"].as<bool>();
  const double direct_pitch_filter_tau_s =
    integration_yaml["xuc_direct_pitch_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_hold_s =
    integration_yaml["xuc_direct_pitch_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_max_rate_rad_s =
    integration_yaml["xuc_direct_pitch_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double direct_pitch_error_gain =
    integration_yaml["xuc_direct_pitch_error_gain"]
      ? std::clamp(integration_yaml["xuc_direct_pitch_error_gain"].as<double>(), 0.0, 2.0)
      : 1.0;
  // The lower-board feedback field is temporarily used for feeder diagnostics,
  // so it must never be interpreted as projectile speed.  Keep trajectory timing
  // on an explicit, bounded configuration value instead.
  const double xuc_projectile_speed_mps = integration_yaml["xuc_projectile_speed_mps"]
    ? std::clamp(integration_yaml["xuc_projectile_speed_mps"].as<double>(), 1.0, 50.0)
    : 23.0;
  // The real-shot reference was recorded at a specific friction-wheel speed.
  // Keep that calibration speed independent from the current launch speed;
  // otherwise changing wheel RPM would silently re-fit the fixed boresight bias
  // and cancel most of the intended trajectory change.
  const double xuc_ballistic_reference_projectile_speed_mps =
    integration_yaml["xuc_ballistic_reference_projectile_speed_mps"]
      ? std::clamp(
          integration_yaml["xuc_ballistic_reference_projectile_speed_mps"].as<double>(),
          1.0,
          50.0)
      : xuc_projectile_speed_mps;
  const bool xuc_ballistic_pitch_enabled = integration_yaml["xuc_ballistic_pitch_enabled"] &&
    integration_yaml["xuc_ballistic_pitch_enabled"].as<bool>();
  const double xuc_camera_forward_of_muzzle_m =
    integration_yaml["xuc_camera_forward_of_muzzle_m"]
      ? std::max(0.0, integration_yaml["xuc_camera_forward_of_muzzle_m"].as<double>())
      : 0.0;
  const double xuc_camera_above_muzzle_m = integration_yaml["xuc_camera_above_muzzle_m"]
    ? std::max(0.0, integration_yaml["xuc_camera_above_muzzle_m"].as<double>())
    : 0.0;
  const double xuc_ballistic_reference_camera_range_m =
    integration_yaml["xuc_ballistic_reference_camera_range_m"]
      ? std::max(0.10, integration_yaml["xuc_ballistic_reference_camera_range_m"].as<double>())
      : 1.0;
  const double xuc_ballistic_reference_total_drop_m =
    integration_yaml["xuc_ballistic_reference_total_drop_m"]
      ? std::max(0.0, integration_yaml["xuc_ballistic_reference_total_drop_m"].as<double>())
      : 0.0;
  const double xuc_ballistic_max_compensation_rad =
    (integration_yaml["xuc_ballistic_max_compensation_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_max_compensation_deg"].as<double>(), 0.0, 25.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_ballistic_pitch_trim_rad =
    (integration_yaml["xuc_ballistic_pitch_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_pitch_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  auto_aim::RobustRangeFilter::Config robust_range_config;
  robust_range_config.time_constant_s =
    integration_yaml["xuc_range_filter_tau_ms"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_tau_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : 0.08;
  robust_range_config.max_rate_mps =
    integration_yaml["xuc_range_filter_max_rate_mps"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_max_rate_mps"].as<double>(), 0.1, 20.0)
      : 6.0;
  robust_range_config.raw_blend =
    integration_yaml["xuc_range_filter_raw_blend"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_raw_blend"].as<double>(), 0.0, 0.5)
      : 0.05;
  robust_range_config.max_raw_model_residual_m =
    integration_yaml["xuc_range_filter_max_raw_residual_m"]
      ? std::clamp(
          integration_yaml["xuc_range_filter_max_raw_residual_m"].as<double>(),
          0.05,
          2.0)
      : 0.40;
  const double direct_pitch_focal_y_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 5
      ? integration_yaml["camera_matrix"][4].as<double>()
      : 0.0;
  const double direct_yaw_focal_x_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 1
      ? integration_yaml["camera_matrix"][0].as<double>()
      : 0.0;
  const double xuc_pitch_sign_for_log = integration_yaml["xuc_pitch_sign"]
    ? integration_yaml["xuc_pitch_sign"].as<double>()
    : -1.0;
  const bool xuc_fire_guard_enabled = integration_yaml["xuc_fire_guard_enabled"] &&
    integration_yaml["xuc_fire_guard_enabled"].as<bool>();
  // 仅用于监控窗口显示"开火总开关是否已解除"；真正生效位置在驱动 io/xuc.cpp。
  const bool xuc_allow_shoot_configured = integration_yaml["xuc_allow_shoot"] &&
    integration_yaml["xuc_allow_shoot"].as<bool>();
  // 2026-10-14 新增：常驻 control 开关。开启后，AUTO 档内目标丢失超短时窗口时，
  // 保持 control=1（摩擦轮继续转）并锁定当前姿态（云台不动）。
  const bool xuc_control_persist_enabled = integration_yaml["xuc_control_persist_enabled"] &&
    integration_yaml["xuc_control_persist_enabled"].as<bool>();
  // 开火节奏回退间隔（秒）。>0 时不依赖拨弹遥测，改为固定间隔重复开火。
  // 0 = 保持原遥测模式。见 fire cadence gate 处注释。
  const double xuc_fire_cadence_fallback_s =
    integration_yaml["xuc_fire_cadence_fallback_ms"]
      ? std::clamp(
          integration_yaml["xuc_fire_cadence_fallback_ms"].as<double>(), 0.0, 5000.0) / 1000.0
      : 0.0;
  // 2026-10-14 新增：开火脉冲保持时长（秒）。0 = 关闭（单帧脉冲，原行为）。
  // >0 时，开火上升沿后把 shoot 维持该时长，使下位机拨弹轮（AUTO 档速度模式）
  // 获得确定性转动时间。实测单帧约 10ms 不足，一颗弹常需 3~4 次脉冲。
  const double xuc_shoot_hold_s =
    integration_yaml["xuc_shoot_hold_ms"]
      ? std::clamp(integration_yaml["xuc_shoot_hold_ms"].as<double>(), 0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_fire_stable_s = integration_yaml["xuc_fire_stable_ms"]
    ? std::max(0.0, integration_yaml["xuc_fire_stable_ms"].as<double>()) / 1000.0
    : 0.5;
  const double xuc_fire_yaw_tolerance_rad = integration_yaml["xuc_fire_yaw_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_yaw_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const double xuc_fire_pitch_tolerance_rad = integration_yaml["xuc_fire_pitch_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_pitch_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const bool xuc_gyro_center_follow_enabled =
    integration_yaml["xuc_gyro_center_follow_enabled"] &&
    integration_yaml["xuc_gyro_center_follow_enabled"].as<bool>();
  const double xuc_gyro_omega_threshold_rad_s =
    integration_yaml["xuc_gyro_omega_threshold_rad_s"]
      ? std::max(0.0, integration_yaml["xuc_gyro_omega_threshold_rad_s"].as<double>())
      : 1.0;
  const double xuc_vehicle_model_enter_rad_s =
    integration_yaml["xuc_vehicle_model_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_enter_rad_s"].as<double>(), 0.0, 10.0)
      : xuc_gyro_omega_threshold_rad_s;
  const double xuc_vehicle_model_exit_rad_s =
    integration_yaml["xuc_vehicle_model_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_exit_rad_s"].as<double>(), 0.0,
          xuc_vehicle_model_enter_rad_s)
      : std::min(0.5, xuc_vehicle_model_enter_rad_s);
  const double xuc_high_speed_enter_rad_s =
    integration_yaml["xuc_high_speed_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_enter_rad_s"].as<double>(), 0.1, 20.0)
      : 3.2;
  const double xuc_high_speed_exit_rad_s =
    integration_yaml["xuc_high_speed_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_exit_rad_s"].as<double>(), 0.0,
          xuc_high_speed_enter_rad_s)
      : std::min(2.5, xuc_high_speed_enter_rad_s);
  const double xuc_rotation_mode_dwell_s =
    integration_yaml["xuc_rotation_mode_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_mode_dwell_ms"].as<double>(), 0.0, 1000.0) /
          1000.0
      : 0.15;
  const double xuc_rotation_tracker_dropout_hold_s =
    integration_yaml["xuc_rotation_tracker_dropout_hold_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_tracker_dropout_hold_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_filter_tau_s =
    integration_yaml["xuc_rotation_rate_filter_tau_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_filter_tau_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_disagreement_rad_s =
    integration_yaml["xuc_rotation_rate_disagreement_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const double xuc_rotation_speed_step_reset_rad_s =
    integration_yaml["xuc_rotation_speed_step_reset_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_speed_step_reset_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const int xuc_rotation_rate_agreement_updates =
    integration_yaml["xuc_rotation_rate_agreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_agreement_updates"].as<int>(), 1, 60)
      : 2;
  const int xuc_rotation_rate_disagreement_updates =
    integration_yaml["xuc_rotation_rate_disagreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_updates"].as<int>(), 2, 120)
      : 3;
  const double xuc_gyro_center_hold_s =
    integration_yaml["xuc_gyro_center_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_center_hold_ms"].as<double>()) / 1000.0
      : 0.6;
  const bool xuc_fire_require_tracker_converged =
    integration_yaml["xuc_fire_require_tracker_converged"] &&
    integration_yaml["xuc_fire_require_tracker_converged"].as<bool>();
  const bool xuc_one_face_once_fire =
    integration_yaml["xuc_one_face_once_fire"] &&
    integration_yaml["xuc_one_face_once_fire"].as<bool>();
  const double xuc_one_face_limit_rad =
    (integration_yaml["xuc_one_face_limit_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_limit_deg"].as<double>(), 1.0, 45.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_one_face_rearm_rad =
    (integration_yaml["xuc_one_face_rearm_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_rearm_deg"].as<double>(), 5.0, 90.0)
       : 34.0) * M_PI / 180.0;
  const double xuc_gyro_fire_stable_s =
    integration_yaml["xuc_gyro_fire_stable_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_fire_stable_ms"].as<double>()) / 1000.0
      : 0.03;
  // 【弹着相位门】center_follow 下炮口指向旋转中心，弹丸沿中心线飞出，命中只
  // 取决于"弹到瞬间哪块装甲板压在视线上"。这里不碰 YawESKF（它的角速度在换面
  // 节律上有 0.5~0.9 rad/s 摆动，折到 0.37 s 预测时域 = ±13~19°），改用实测
  // 体相位 x[6] + 最近 2.5 s 的角度差分外推；台架是匀速，窗口平均可压到
  // 0.05~0.1 rad/s（≈2~3°）。
  const double xuc_fire_phase_window_rad =
    (integration_yaml["xuc_fire_phase_window_deg"]
       ? std::clamp(integration_yaml["xuc_fire_phase_window_deg"].as<double>(), 0.0, 40.0)
       : 10.0) * M_PI / 180.0;
  const double xuc_fire_phase_rate_window_s =
    (integration_yaml["xuc_fire_phase_rate_window_ms"]
       ? std::clamp(
           integration_yaml["xuc_fire_phase_rate_window_ms"].as<double>(), 500.0, 6000.0)
       : 2500.0) / 1000.0;
  // 与 Aimer 一致的弹着时刻 = 帧龄 + 发弹延时 + 飞行时间
  const double xuc_phase_high_speed_delay_s =
    integration_yaml["high_speed_delay_time"]
      ? std::max(0.0, integration_yaml["high_speed_delay_time"].as<double>())
      : 0.1;
  const double xuc_phase_low_speed_delay_s =
    integration_yaml["low_speed_delay_time"]
      ? std::max(0.0, integration_yaml["low_speed_delay_time"].as<double>())
      : 0.1;
  const double xuc_phase_decision_speed_rad_s =
    integration_yaml["decision_speed"]
      ? std::max(0.0, integration_yaml["decision_speed"].as<double>())
      : 0.8;
  // A stationary target is still qualified by the proven image-space gate.
  // In gyro mode, however, the visible plate is moving while the gimbal follows
  // the vehicle centre. Qualify the armor selected by Aimer at projectile impact
  // time instead of reusing the current-frame pixel error as the precise trigger.
  const bool xuc_gyro_predicted_fire_enabled =
    integration_yaml["xuc_gyro_predicted_fire_enabled"] &&
    integration_yaml["xuc_gyro_predicted_fire_enabled"].as<bool>();
  // A native yaw-error fallback was useful while bringing up the event clock,
  // but it has no phase/cycle information.  Let field configs fail closed while
  // the PLL is unlocked, without changing legacy configs that omit this key.
  const bool xuc_gyro_require_event_lock_for_fire =
    integration_yaml["xuc_gyro_require_event_lock_for_fire"] &&
    integration_yaml["xuc_gyro_require_event_lock_for_fire"].as<bool>();
  const double xuc_gyro_predicted_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"].as<double>(),
           0.1,
           10.0)
       : 1.0) * M_PI / 180.0;
  const double xuc_gyro_predicted_yaw_trim_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  const double xuc_gyro_visibility_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"].as<double>(),
           0.5,
           20.0)
       : 6.0) * M_PI / 180.0;
  const int xuc_gyro_fire_min_consecutive_frames =
    integration_yaml["xuc_gyro_fire_min_consecutive_frames"]
      ? std::clamp(
          integration_yaml["xuc_gyro_fire_min_consecutive_frames"].as<int>(), 1, 10)
      : 2;
  // Predictive fire uses one-frame pulses. Pending-shot and live feeder feedback
  // prevent overlapping requests; an optional post-feed hold remains available
  // for lower boards that require it, while zero enables feedback-driven repeat.
  const double xuc_gyro_post_feed_recovery_s =
    integration_yaml["xuc_gyro_post_feed_recovery_ms"]
      ? std::clamp(
          integration_yaml["xuc_gyro_post_feed_recovery_ms"].as<double>(),
          0.0,
          2000.0) /
          1000.0
      : 0.6;
  // Event-clock fire timing. The calibrated nominal remains the fallback;
  // receive-thread feeder timing may add a small, bounded runtime correction.
  const double xuc_event_fire_delay_s =
    integration_yaml["xuc_event_fire_delay_s"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_s"].as<double>(), 0.0, 0.5)
      : 0.10;
  const double xuc_event_fire_delay_min_s =
    integration_yaml["xuc_event_fire_delay_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_min_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : std::max(0.0, xuc_event_fire_delay_s - 0.020);
  const double xuc_event_fire_delay_max_s =
    integration_yaml["xuc_event_fire_delay_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_max_ms"].as<double>(), 0.0, 500.0) /
          1000.0
      : std::min(0.5, xuc_event_fire_delay_s + 0.020);
  auto_aim::ShotDelayAdapter::Config shot_delay_config;
  shot_delay_config.enabled =
    integration_yaml["xuc_shot_delay_adaptation_enabled"] &&
    integration_yaml["xuc_shot_delay_adaptation_enabled"].as<bool>();
  shot_delay_config.nominal_delay_s = xuc_event_fire_delay_s;
  shot_delay_config.min_delay_s = std::min(
    xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
  shot_delay_config.max_delay_s = std::max(
    xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
  shot_delay_config.reference_feedback_s =
    integration_yaml["xuc_shot_delay_reference_ms"]
      ? std::max(0.0, integration_yaml["xuc_shot_delay_reference_ms"].as<double>()) /
          1000.0
      : 0.0;
  shot_delay_config.sample_min_s =
    (integration_yaml["xuc_shot_delay_sample_min_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_sample_min_ms"].as<double>(), 0.0, 200.0)
       : 5.0) / 1000.0;
  shot_delay_config.sample_max_s =
    (integration_yaml["xuc_shot_delay_sample_max_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_sample_max_ms"].as<double>(), 5.0, 500.0)
       : 100.0) / 1000.0;
  shot_delay_config.window_size = static_cast<std::size_t>(
    integration_yaml["xuc_shot_delay_window"]
      ? std::clamp(integration_yaml["xuc_shot_delay_window"].as<int>(), 3, 31)
      : 9);
  shot_delay_config.min_samples = static_cast<std::size_t>(
    integration_yaml["xuc_shot_delay_min_samples"]
      ? std::clamp(integration_yaml["xuc_shot_delay_min_samples"].as<int>(), 3, 31)
      : 5);
  shot_delay_config.ewma_alpha =
    integration_yaml["xuc_shot_delay_ewma_alpha"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_ewma_alpha"].as<double>(), 0.01, 1.0)
      : 0.35;
  shot_delay_config.adaptation_gain =
    integration_yaml["xuc_shot_delay_adaptation_gain"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_adaptation_gain"].as<double>(), 0.0, 1.0)
      : 1.0;
  shot_delay_config.max_step_s =
    (integration_yaml["xuc_shot_delay_max_step_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_max_step_ms"].as<double>(), 0.0, 10.0)
       : 1.5) / 1000.0;
  shot_delay_config.outlier_sigma =
    integration_yaml["xuc_shot_delay_outlier_sigma"]
      ? std::clamp(
          integration_yaml["xuc_shot_delay_outlier_sigma"].as<double>(), 1.0, 10.0)
      : 3.5;
  shot_delay_config.outlier_floor_s =
    (integration_yaml["xuc_shot_delay_outlier_floor_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_outlier_floor_ms"].as<double>(), 0.0, 50.0)
       : 4.0) / 1000.0;
  shot_delay_config.min_prediction_half_width_s =
    (integration_yaml["xuc_shot_delay_95pi_min_half_ms"]
       ? std::clamp(
           integration_yaml["xuc_shot_delay_95pi_min_half_ms"].as<double>(), 1.0, 30.0)
       : 8.0) / 1000.0;
  const double xuc_event_flight_time_uncertainty_s =
    integration_yaml["xuc_event_flight_time_uncertainty_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_flight_time_uncertainty_ms"].as<double>(),
          0.0,
          100.0) /
          1000.0
      : 0.005;
  // If the precise due time lies shortly after this frame, wait until that
  // steady-clock instant and send the one-frame pulse there.  This removes the
  // 16 ms quantization of a ~60 FPS vision loop without delaying ordinary
  // tracking frames.
  const double xuc_event_trigger_window_s =
    integration_yaml["xuc_event_trigger_window_ms"]
      ? std::max(0.0, integration_yaml["xuc_event_trigger_window_ms"].as<double>()) /
          1000.0
      : 0.025;
  const double xuc_event_send_late_tolerance_s =
    integration_yaml["xuc_event_send_late_tolerance_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_send_late_tolerance_ms"].as<double>(),
          0.0,
          10.0) /
          1000.0
      : 0.002;
  const double xuc_event_crossing_capture_rad =
    (integration_yaml["xuc_event_crossing_capture_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_capture_deg"].as<double>(), 5.0, 45.0)
       : 20.0) * M_PI / 180.0;
  const double xuc_event_crossing_rearm_rad =
    (integration_yaml["xuc_event_crossing_rearm_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_rearm_deg"].as<double>(), 2.0, 35.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_event_min_crossing_separation_ratio =
    integration_yaml["xuc_event_min_crossing_separation_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_min_crossing_separation_ratio"].as<double>(),
          0.20,
          0.90)
      : 0.55;
  const double xuc_event_period_min_s =
    integration_yaml["xuc_event_period_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_min_ms"].as<double>(), 30.0, 2000.0) /
          1000.0
      : 0.12;
  const double xuc_event_period_max_s =
    integration_yaml["xuc_event_period_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_max_ms"].as<double>(), 50.0, 5000.0) /
          1000.0
      : 1.20;
  const double xuc_event_period_ema_alpha =
    integration_yaml["xuc_event_period_ema_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_ema_alpha"].as<double>(), 0.01, 1.0)
      : 0.25;
  const double xuc_event_max_period_relative_error =
    integration_yaml["xuc_event_max_period_relative_error"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_relative_error"].as<double>(),
          0.05,
          0.50)
      : 0.22;
  const double xuc_event_max_period_step_ratio =
    integration_yaml["xuc_event_max_period_step_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_step_ratio"].as<double>(),
          0.001,
          0.20)
      : 0.025;
  const double xuc_event_omega_period_weight =
    integration_yaml["xuc_event_omega_period_weight"]
      ? std::clamp(
          integration_yaml["xuc_event_omega_period_weight"].as<double>(),
          0.0,
          1.0)
      : 0.15;
  const double xuc_event_period_residual_alpha =
    integration_yaml["xuc_event_period_residual_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_residual_alpha"].as<double>(),
          0.0,
          0.5)
      : 0.08;
  const double xuc_event_phase_correction_alpha =
    integration_yaml["xuc_event_phase_correction_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_correction_alpha"].as<double>(),
          0.0,
          1.0)
      : 0.45;
  const double xuc_event_max_phase_correction_s =
    integration_yaml["xuc_event_max_phase_correction_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_correction_ms"].as<double>(),
          0.0,
          20.0) /
          1000.0
      : 0.015;
  const double xuc_event_max_phase_residual_s =
    integration_yaml["xuc_event_max_phase_residual_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_residual_ms"].as<double>(),
          5.0,
          150.0) /
          1000.0
      : 0.090;
  const int xuc_event_period_window_size =
    integration_yaml["xuc_event_period_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_period_window_size"].as<int>(), 3, 15)
      : 7;
  const double xuc_event_phase_uncertainty_floor_s =
    integration_yaml["xuc_event_phase_uncertainty_floor_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_uncertainty_floor_ms"].as<double>(),
          0.0,
          50.0) /
          1000.0
      : 0.002;
  const int xuc_event_phase_residual_window_size =
    integration_yaml["xuc_event_phase_residual_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_residual_window_size"].as<int>(), 3, 15)
      : 5;
  const int xuc_event_min_lock_updates =
    integration_yaml["xuc_event_min_lock_updates"]
      ? std::clamp(
          integration_yaml["xuc_event_min_lock_updates"].as<int>(), 2, 12)
      : 4;
  const int xuc_event_relock_rejections =
    integration_yaml["xuc_event_relock_rejections"]
      ? std::clamp(
          integration_yaml["xuc_event_relock_rejections"].as<int>(), 2, 12)
      : 3;
  const double xuc_event_armor_width_m =
    integration_yaml["xuc_event_armor_width_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_width_m"].as<double>(), 0.05, 0.40)
      : 0.135;
  const double xuc_event_armor_edge_margin_m =
    integration_yaml["xuc_event_armor_edge_margin_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_edge_margin_m"].as<double>(), 0.0, 0.10)
      : 0.005;
  const double xuc_event_model_phase_uncertainty_rad =
    (integration_yaml["xuc_event_model_phase_uncertainty_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_model_phase_uncertainty_deg"].as<double>(),
           0.0,
           20.0)
       : 1.0) * M_PI / 180.0;
  // A small, explicit grace angle relaxes the conservative worst-case budget
  // without bypassing phase lock or gimbal tracking.
  const double xuc_event_phase_grace_rad =
    (integration_yaml["xuc_event_phase_grace_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_phase_grace_deg"].as<double>(), 0.0, 4.0)
       : 0.0) * M_PI / 180.0;
  // The crossing PLL observes the physical face-center event directly.  The
  // EKF future-face prediction is useful telemetry, but its phase ripples at
  // armor hand-over can be made a soft check for higher fire availability.
  const bool xuc_event_require_face_consistency =
    !integration_yaml["xuc_event_require_face_consistency"] ||
    integration_yaml["xuc_event_require_face_consistency"].as<bool>();
  const double xuc_event_face_max_alignment_rad =
    (integration_yaml["xuc_event_face_max_alignment_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_face_max_alignment_deg"].as<double>(),
           1.0,
           45.0)
       : 15.0) * M_PI / 180.0;
  const double xuc_event_gimbal_tracking_tolerance_rad =
    (integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"].as<double>(),
           0.2,
           10.0)
       : 2.5) * M_PI / 180.0;
  const double xuc_shot_feedback_timeout_s =
    integration_yaml["xuc_shot_feedback_timeout_ms"]
      ? std::clamp(
          integration_yaml["xuc_shot_feedback_timeout_ms"].as<double>(), 100.0, 1000.0) /
          1000.0
      : 0.35;
  // A face switch is approximately 90 degrees on a four-armor target. The
  // same face moves only about five degrees per frame at 60 FPS and 5 rad/s.
  // Reject a two-frame qualification that silently crosses a face switch.
  const double xuc_gyro_candidate_max_phase_step_rad =
    (integration_yaml["xuc_gyro_candidate_max_phase_step_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_candidate_max_phase_step_deg"].as<double>(),
           5.0,
           45.0)
       : 20.0) * M_PI / 180.0;

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("standard_node", "standard_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::XucSender xuc(config_path);  // 新版XUC：接收IMU，发送目标角

  auto_aim::YOLO detector(config_path, false);  // 启用调试，显示检测窗口
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::ShotDelayAdapter shot_delay_adapter(shot_delay_config);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
  auto_aim::GyroPhaseScheduler::Config gyro_phase_config;
  gyro_phase_config.crossing_capture_rad = xuc_event_crossing_capture_rad;
  gyro_phase_config.crossing_rearm_rad = xuc_event_crossing_rearm_rad;
  gyro_phase_config.min_crossing_separation_ratio =
    xuc_event_min_crossing_separation_ratio;
  gyro_phase_config.min_face_period_s = std::min(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.max_face_period_s = std::max(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.period_ema_alpha = xuc_event_period_ema_alpha;
  gyro_phase_config.max_period_relative_error =
    xuc_event_max_period_relative_error;
  gyro_phase_config.max_period_step_ratio = xuc_event_max_period_step_ratio;
  gyro_phase_config.omega_period_weight = xuc_event_omega_period_weight;
  gyro_phase_config.omega_alias_guard_relative_error = 0.35;
  gyro_phase_config.period_residual_alpha = xuc_event_period_residual_alpha;
  gyro_phase_config.phase_correction_alpha = xuc_event_phase_correction_alpha;
  gyro_phase_config.max_phase_correction_s = xuc_event_max_phase_correction_s;
  gyro_phase_config.max_phase_residual_s = xuc_event_max_phase_residual_s;
  gyro_phase_config.phase_uncertainty_floor_s = xuc_event_phase_uncertainty_floor_s;
  gyro_phase_config.period_window_size = xuc_event_period_window_size;
  gyro_phase_config.phase_residual_window_size = xuc_event_phase_residual_window_size;
  gyro_phase_config.min_lock_updates = xuc_event_min_lock_updates;
  gyro_phase_config.relock_after_rejections = xuc_event_relock_rejections;
  auto_aim::GyroPhaseScheduler gyro_phase_scheduler(gyro_phase_config);
  auto_aim::GyroFaceSelector gyro_face_selector(xuc_event_face_max_alignment_rad);
  auto_aim::RobustRangeFilter robust_range_filter(robust_range_config);
  auto_aim::RotationModelManager::Config rotation_model_config;
  rotation_model_config.vehicle_enter_rad_s = xuc_vehicle_model_enter_rad_s;
  rotation_model_config.vehicle_exit_rad_s = xuc_vehicle_model_exit_rad_s;
  rotation_model_config.high_speed_enter_rad_s = xuc_high_speed_enter_rad_s;
  rotation_model_config.high_speed_exit_rad_s = xuc_high_speed_exit_rad_s;
  rotation_model_config.mode_dwell_s = xuc_rotation_mode_dwell_s;
  rotation_model_config.tracker_dropout_hold_s =
    xuc_rotation_tracker_dropout_hold_s;
  rotation_model_config.rate_filter_tau_s = xuc_rotation_rate_filter_tau_s;
  rotation_model_config.max_rate_disagreement_rad_s =
    xuc_rotation_rate_disagreement_rad_s;
  rotation_model_config.speed_step_reset_rad_s =
    xuc_rotation_speed_step_reset_rad_s;
  rotation_model_config.rate_agreement_updates =
    xuc_rotation_rate_agreement_updates;
  rotation_model_config.rate_disagreement_updates =
    xuc_rotation_rate_disagreement_updates;
  auto_aim::RotationModelManager rotation_model_manager(rotation_model_config);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");

#ifdef AMENT_CMAKE_FOUND
  // 发布静态TF: gimbal -> camera（使用标定参数）
  visualizer->publish_static_tf("gimbal", "camera",
    solver.R_camera2gimbal(),
    solver.t_camera2gimbal());
  tools::logger()->info("[ROS2] Published static TF: gimbal -> camera");
#endif

  tools::logger()->info("=== Entering main loop ===");

  cv::Mat img;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 性能分析计时器
  std::chrono::steady_clock::time_point t_start, t_end;

  // 🆕 同步匹配相关变量（需要在循环外声明，以便后续日志使用）
  uint64_t frame_id = 0;
  uint64_t frame_id_last =0;
  int64_t trigger_imu_count = 0;

  // 📊 相机帧率测量变量
  std::chrono::steady_clock::time_point camera_last_frame_time;
  std::chrono::steady_clock::time_point camera_current_frame_time;
  double camera_fps_instant = 0.0;     // 瞬时帧率
  double camera_fps_avg = 0.0;         // 平均帧率
  int camera_frame_count = 0;          // 帧计数

  // Bounded yaw conditioner.  Detector measurements remain authoritative while
  // a small, validated native prediction lead helps fast lateral tracking.
  // Long visual loss drops control without destroying reacquisition continuity.
  bool direct_yaw_initialized = false;
  bool direct_yaw_control_confirmed = false;
  double direct_yaw_filtered = 0.0;
  bool direct_yaw_long_loss = false;
  std::chrono::steady_clock::time_point direct_yaw_last_update;
  std::chrono::steady_clock::time_point direct_yaw_last_measurement;
  // Limit the complete yaw reference after model selection and prediction.
  // The measurement conditioner alone cannot bound an abruptly changing
  // native prediction lead or a vehicle-center/armor-model hand-over.
  bool final_yaw_command_initialized = false;
  bool final_yaw_lower_armed = false;
  double final_yaw_command_filtered = 0.0;
  std::chrono::steady_clock::time_point final_yaw_command_last_update;
  double yaw_tracking_integral_raw_rad = 0.0;
  std::chrono::steady_clock::time_point yaw_tracking_last_update;
  std::chrono::steady_clock::time_point yaw_tracking_armed_since;
  bool yaw_tracking_initialized = false;
  bool yaw_tracking_lower_armed = false;
  bool direct_pitch_initialized = false;
  double direct_pitch_filtered = 0.0;
  bool direct_pitch_long_loss = false;
  std::chrono::steady_clock::time_point direct_pitch_last_update;
  std::chrono::steady_clock::time_point direct_pitch_last_measurement;
  bool xuc_fire_candidate_active = false;
  bool xuc_fire_ready_logged = false;
  int xuc_fire_candidate_frames = 0;
  double xuc_fire_candidate_face_yaw_rad = INFINITY;
  std::chrono::steady_clock::time_point xuc_fire_candidate_since;
  bool xuc_one_face_armed = true;
  bool xuc_gyro_center_follow_logged = false;
  bool xuc_gyro_center_memory_valid = false;
  double xuc_gyro_center_memory_yaw = 0.0;
  std::chrono::steady_clock::time_point xuc_gyro_center_last_fresh;
  bool xuc_feeder_activity_seen = false;
  bool xuc_feeder_active = false;
  uint64_t xuc_feeder_transition_sequence = 0;
  bool xuc_gyro_cadence_hold_logged = false;
  // 误差拐点判据用的上一帧弹着偏航误差（rad）；仅观测，不参与控制以外的东西。
  double prev_pred_yaw_error_rad = NAN;
  // 体相位历史（时间[s], 体相位[rad]）与窗口平均角速度，供弹着相位门使用。
  // 只依赖实测体相位，与 YawESKF 的瞬时 ω 解耦。
  std::deque<std::pair<double, double>> body_phase_history;
  double body_phase_rate_window = 0.0;
  long xuc_fire_inhibit_count = 0;   // 被拨弹门控拦下的累计次数（监控窗口用）
  // 【诊断】持续 shoot 模式的计数与结束标记；仅用于 SPV_SHOOT_HOLD_MS 诊断，验收后删除
  long xuc_shoot_hold_frames = 0;
  bool xuc_shoot_hold_ended_logged = false;
  // 最近一次实际开出 shoot=1 的时刻；供“定时回退”节奏门控使用（见 cadence gate）
  std::chrono::steady_clock::time_point xuc_last_shot_issue_time{};
  // 开火脉冲保持：xuc_shoot_pulse_active 为真时，把 shoot 维持到 xuc_shoot_pulse_until。
  // 仅在开火上升沿武装；保持期间的续帧不视为新开火。
  std::chrono::steady_clock::time_point xuc_shoot_pulse_until{};
  bool xuc_shoot_pulse_active = false;
  long xuc_shoot_hold_pulse_count = 0;   // 已武装的脉冲数（监控窗口用）
  std::int64_t event_last_fired_cycle = -1;
  std::chrono::steady_clock::time_point xuc_feeder_last_active;
  uint64_t xuc_next_shot_id = 1;
  uint64_t xuc_pending_shot_id = 0;
  uint64_t xuc_active_shot_id = 0;
  bool xuc_pending_shot = false;
  std::chrono::steady_clock::time_point xuc_pending_shot_time;
  std::chrono::steady_clock::time_point xuc_active_shot_start_time;
  std::chrono::steady_clock::time_point fps_measure_start; // 平均帧率测量开始时间
  bool fps_measure_started = false;

  // 初始化默认命令（第一帧使用）
  io::Command default_command;
  default_command.yaw = 0.0;
  default_command.pitch = 0.0;
  default_command.control = false;
  default_command.shoot = false;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();  // 🔍 性能监控：循环开始

    // 🎯 关键修改：先发送命令，触发相机曝光，然后再读取图像
    // 这样确保电控板收到命令后，才会发出硬触发信号让相机曝光
    static bool first_loop = true;
    if (first_loop) {
      // 第一帧发送默认命令
      if (xuc.enabled()) {
        xuc.send(default_command);
      } else {
        cboard.send(default_command);
      }
      tools::logger()->info("[SYNC] 发送默认命令，等待相机触发");
      first_loop = false;
    }

    camera.read(img, t);

    // 📊 相机帧率测量
    camera_current_frame_time = std::chrono::steady_clock::now();
    if (!fps_measure_started) {
      // 初始化帧率测量
      camera_last_frame_time = camera_current_frame_time;
      fps_measure_start = camera_current_frame_time;
      fps_measure_started = true;
      camera_frame_count = 0;
    } else {
      // 计算瞬时帧率（基于相邻两帧的时间间隔）
      auto frame_interval = std::chrono::duration<double>(camera_current_frame_time - camera_last_frame_time).count();
      if (frame_interval > 0) {
        camera_fps_instant = 1.0 / frame_interval;
      }

      // 计算平均帧率（基于总时间和总帧数）
      camera_frame_count++;
      auto total_time = std::chrono::duration<double>(camera_current_frame_time - fps_measure_start).count();
      if (total_time > 0) {
        camera_fps_avg = camera_frame_count / total_time;
      }

      // 更新上一帧时间
      camera_last_frame_time = camera_current_frame_time;

      // 每50帧打印一次帧率统计
      static int fps_log_counter = 0;
      if (++fps_log_counter % 50 == 0) {
        tools::logger()->info(
          "📊 [CAMERA FPS] 瞬时帧率={:.1f}fps, 平均帧率={:.1f}fps, 总帧数={}, 运行时间={:.1f}s",
          camera_fps_instant, camera_fps_avg, camera_frame_count, total_time);
      }
    }

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发,每来一帧图像对应一个IMU计数（0-15循环）
      // 映射关系：frame_id % 16 = imu_count
      // MCU发送的imu_count是4位二进制，范围0-15
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数（微调偏移）

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();

      // 新版硬件中IMU连接下位机，姿态由XUC上行包提供。
      if (xuc.enabled()) {
        if (xuc.rx_valid()) {
          const double imu_yaw = xuc.imu_yaw();
          const double imu_pitch = xuc.imu_pitch();
          q = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY());
          q.normalize();
          solver.set_R_gimbal2world(q);
          cboard.mode = static_cast<io::Mode>(xuc.mode());
          const double lower_diagnostic_value = xuc.bullet_speed();
          const auto feeder_feedback = xuc.feeder_feedback();
          const auto feedback_now = std::chrono::steady_clock::now();
          if (feeder_feedback.transition_sequence != xuc_feeder_transition_sequence) {
            xuc_feeder_transition_sequence = feeder_feedback.transition_sequence;
            const auto feeder_edge_time = std::chrono::steady_clock::time_point(
              std::chrono::nanoseconds(feeder_feedback.transition_time_ns));
            xuc_feeder_active = feeder_feedback.active;
            if (feeder_feedback.active) {
              xuc_feeder_activity_seen = true;
              xuc_feeder_last_active = feeder_edge_time;
              if (xuc_pending_shot) {
                const double command_to_feeder_ms =
                  std::chrono::duration<double, std::milli>(
                    feeder_edge_time - xuc_pending_shot_time).count();
                if (command_to_feeder_ms >= 0.0 &&
                    command_to_feeder_ms <= xuc_shot_feedback_timeout_s * 1000.0) {
                  xuc_active_shot_id = xuc_pending_shot_id;
                  xuc_active_shot_start_time = feeder_edge_time;
                  const auto adaptation =
                    shot_delay_adapter.update(command_to_feeder_ms / 1000.0);
                  if (adaptation.accepted) {
                    aimer.set_delay_adjustment(shot_delay_adapter.delay_adjustment_s());
                    if (adaptation.regime_change) {
                      tools::logger()->warn(
                        "[SHOTADAPT] persistent feeder-delay shift detected; "
                        "rebuilding robust window");
                    }
                    tools::logger()->info(
                      "[SHOTADAPT] id={} feeder_ms={:.2f} samples={} ready={} "
                      "delay_ms={:.2f} pi95_ms=[{:.2f},{:.2f}]",
                      xuc_active_shot_id, command_to_feeder_ms,
                      adaptation.sample_count, adaptation.ready ? 1 : 0,
                      adaptation.delay_s * 1000.0,
                      adaptation.delay_min_s * 1000.0,
                      adaptation.delay_max_s * 1000.0);
                  } else if (adaptation.rejected_outlier) {
                    tools::logger()->warn(
                      "[SHOTADAPT] id={} rejected feeder outlier {:.2f} ms; "
                      "delay held at {:.2f} ms",
                      xuc_active_shot_id, command_to_feeder_ms,
                      shot_delay_adapter.delay_s() * 1000.0);
                  }
                  tools::logger()->info(
                    "[SHOT] id={} feeder_start command_to_feeder_ms={:.2f} rpm={:.0f}",
                    xuc_active_shot_id, command_to_feeder_ms, feeder_feedback.rpm);
                } else {
                  tools::logger()->warn(
                    "[SHOT] id={} feeder_start_unmatched age_ms={:.2f} rpm={:.0f}",
                    xuc_pending_shot_id, command_to_feeder_ms, feeder_feedback.rpm);
                }
                xuc_pending_shot = false;
                xuc_pending_shot_id = 0;
              } else {
                tools::logger()->info(
                  "[SHOT] feeder_start_unpaired rpm={:.0f}", feeder_feedback.rpm);
              }
            } else {
              xuc_feeder_last_active = feeder_edge_time;
              if (xuc_active_shot_id != 0) {
                const double feeder_motion_ms =
                  std::chrono::duration<double, std::milli>(
                    feeder_edge_time - xuc_active_shot_start_time).count();
                tools::logger()->info(
                  "[SHOT] id={} feeder_stop feeder_motion_ms={:.1f}",
                  xuc_active_shot_id, feeder_motion_ms);
                xuc_active_shot_id = 0;
              }
              tools::logger()->info(
                "[XUC][FIRE] feeder stopped; post-feed hold={:.0f} ms",
                xuc_gyro_post_feed_recovery_s * 1000.0);
            }
          }
          if (xuc_pending_shot &&
              std::chrono::duration<double>(feedback_now - xuc_pending_shot_time).count() >
                xuc_shot_feedback_timeout_s) {
            tools::logger()->warn(
              "[SHOT] id={} no_feeder_feedback timeout_ms={:.0f}",
              xuc_pending_shot_id, xuc_shot_feedback_timeout_s * 1000.0);
            xuc_pending_shot = false;
            xuc_pending_shot_id = 0;
          }
          const uint8_t motion_gate_bits = xuc.robot_id();
          const uint16_t packed_wheel_rpm = xuc.bullet_count();
          const unsigned wheel0_rpm =
            static_cast<unsigned>(packed_wheel_rpm & 0xffU) * 40U;
          const unsigned wheel1_rpm =
            static_cast<unsigned>((packed_wheel_rpm >> 8U) & 0xffU) * 40U;
          cboard.bullet_speed = xuc_projectile_speed_mps;
          mode = cboard.mode;

          static int xuc_imu_log_counter = 0;
          const int xuc_log_period_frames = (xuc.mode() == 1) ? 5 : 50;
          if (++xuc_imu_log_counter % xuc_log_period_frames == 0) {
            tools::logger()->info(
              "[SYNC][XUC] mode={} yaw={:.2f} deg pitch={:.2f} deg "
              "feeder_rpm={:.2f} wheel0_rpm={} wheel1_rpm={} wheel_delta={} "
              "projectile_speed={:.2f} gate=0x{:02X} "
              "switch={} rc={} sticks={} imu={} serial={} control={} yaw_armed={} pitch_active={}",
              static_cast<int>(xuc.mode()),
              imu_yaw * 180.0 / M_PI, imu_pitch * 180.0 / M_PI,
              lower_diagnostic_value, wheel0_rpm, wheel1_rpm,
              static_cast<int>(wheel0_rpm) - static_cast<int>(wheel1_rpm),
              cboard.bullet_speed, static_cast<unsigned>(motion_gate_bits),
              (motion_gate_bits >> 0U) & 1U, (motion_gate_bits >> 1U) & 1U,
              (motion_gate_bits >> 2U) & 1U, (motion_gate_bits >> 3U) & 1U,
              (motion_gate_bits >> 4U) & 1U, (motion_gate_bits >> 5U) & 1U,
              (motion_gate_bits >> 6U) & 1U, (motion_gate_bits >> 7U) & 1U);
          }
        } else {
          cboard.mode = io::Mode::idle;
          mode = io::Mode::idle;
        }
      // 海康等不提供 frame_id 的相机使用 steady_clock 时间戳插值。
      } else if (frame_id == 0) {
        q = cboard.imu_at(t);
        solver.set_R_gimbal2world(q);

        static int timestamp_sync_log_counter = 0;
        if (++timestamp_sync_log_counter % 50 == 0) {
          auto sync_ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
          tools::logger()->info(
            "[SYNC][Timestamp] yaw={:.2f} deg pitch={:.2f} deg roll={:.2f} deg",
            sync_ypr[0] * 180.0 / M_PI,
            sync_ypr[1] * 180.0 / M_PI,
            sync_ypr[2] * 180.0 / M_PI);
        }

        mode = cboard.mode;
      } else if (frame_id - frame_id_last != 0) {
      // ✅ 修复：根据frame_id计算对应的IMU计数（0-15循环）
      trigger_imu_count = (frame_id + frame_id_to_imu_offset) % 16;
      if (trigger_imu_count < 0) trigger_imu_count += 16;

      // 🔍 检查frame_id是否跳帧
      if (frame_id - frame_id_last > 1) {
        tools::logger()->warn("[SYNC] ⚠️ 相机跳帧！frame_id: {} -> {} (跳过{}帧)",
          frame_id_last, frame_id, frame_id - frame_id_last - 1);
      }

      //使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(trigger_imu_count);

      if (imu_result.valid) {
        // 环形数组查询成功
        q = imu_result.q;  // 四元数
        t = imu_result.timestamp;

        // 🔍 计算帧间隔和跳帧情况
        static auto last_t = t;
        static uint64_t last_valid_frame_id = 0;
        auto frame_interval_ms = std::chrono::duration<double, std::milli>(t - last_t).count();
        uint64_t frame_gap = frame_id - last_valid_frame_id;

        // 相机理论帧率 = 1000 / (实际间隔ms / 实际帧数差)
        double camera_theoretical_fps = frame_gap * 1000.0 / frame_interval_ms;

        last_t = t;
        last_valid_frame_id = frame_id;

        // 🔍 同步调试日志（每50帧打印一次）
        static int sync_log_counter = 0;
        if (++sync_log_counter % 50 == 0) {
          tools::logger()->info("[SYNC] ✅ frame_id={}, trigger_imu_count={}, 间隔={:.1f}ms, 跳过{}帧, 相机理论帧率≈{:.0f}fps",
            frame_id, trigger_imu_count, frame_interval_ms, frame_gap - 1, camera_theoretical_fps);
        }

        // ✅ 在获取到有效IMU数据后立即设置solver，确保姿态同步
        solver.set_R_gimbal2world(q);

#ifdef AMENT_CMAKE_FOUND
        // 发布动态TF: world -> gimbal（使用MCU四元数）
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        rclcpp::Time ros_time(ns);
        Eigen::Vector3d zero_trans(0, 0, 0);  // world和gimbal原点重合
        visualizer->publish_dynamic_tf("world", "gimbal", q, zero_trans, ros_time);
#endif
      } else {
        // 🔍 同步调试：IMU数据无效（可能是时序问题或环形缓冲区被覆盖）
        tools::logger()->warn("[SYNC] ❌ frame_id={}, trigger_imu_count={}, IMU数据无效（可能还未到达或已被覆盖）",
          frame_id, trigger_imu_count);
      }
    mode = cboard.mode;
    frame_id_last=frame_id;
     }
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }
    // recorder.record(img, q, t);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto t1 = std::chrono::steady_clock::now();  // 🔍 性能监控
    auto armors = detector.detect(img);
    auto t2 = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);
    auto t3 = std::chrono::steady_clock::now();

    bool tracker_converged = false;
    bool yaw_motion_model_trusted = false;
    bool vehicle_model_fresh = false;
    bool gyro_model_fresh = false;
    bool gyro_phase_model_trusted = false;
    bool gyro_center_follow_active = false;
    double gyro_center_yaw = 0.0;
    double gyro_body_phase_rad = NAN;
    double gyro_omega_rad_s = 0.0;
    double gyro_radius_m = 0.0;
    double gyro_center_velocity_mps = 0.0;
    double center_x_m = 0.0;
    double center_y_m = 0.0;
    double center_vx_mps = 0.0;
    double center_vy_mps = 0.0;
    if (!targets.empty()) {
      auto & tracked_target = targets.front();
      tracker_converged = tracked_target.convergened();
      yaw_motion_model_trusted = tracked_target.yaw_eskf_trusted();
      const auto tracked_x = tracked_target.ekf_x();
      if (tracked_x.size() >= 9) {
        gyro_center_yaw = std::atan2(tracked_x[2], tracked_x[0]);
        gyro_body_phase_rad = tracked_x[6];
        gyro_omega_rad_s = tracked_x[7];
        gyro_radius_m = tracked_x[8];
        // Centre position/velocity in the world frame.  Used to distinguish
        // "the vehicle is merely spinning about a fixed centre" (no lead is
        // owed) from "the centre is actually translating" (a lead is owed).
        center_x_m = tracked_x[0];
        center_y_m = tracked_x[2];
        center_vx_mps = tracked_x[1];
        center_vy_mps = tracked_x[3];
        gyro_center_velocity_mps = std::hypot(center_vx_mps, center_vy_mps);
      }
      static int yaw_eskf_log_counter = 0;
      if (++yaw_eskf_log_counter % 50 == 0) {
        const Eigen::Vector3d armor_velocity =
          tracked_target.armor_motion_velocity(tracked_target.last_id);
        tools::logger()->info(
          "[YAWESKF] trusted={} angle={:+.3f} omega={:+.3f} alpha={:+.3f} "
          "raw_ekf_omega={:+.3f} nis={:.2f} accepted={} rejected={} "
          "whole_iekf_nis={:.2f} armor_cv_trusted={} armor_v=({:+.2f},{:+.2f},{:+.2f}) "
          "armor_cv_nis={:.2f}",
          tracked_target.yaw_eskf_trusted() ? 1 : 0,
          tracked_target.yaw_eskf_angle(), tracked_target.yaw_eskf_rate(),
          tracked_target.yaw_eskf_acceleration(),
          tracked_target.ekf().x.size() >= 8 ? tracked_target.ekf().x[7] : NAN,
          tracked_target.yaw_eskf_nis(),
          tracked_target.yaw_eskf_accepted_updates(),
          tracked_target.yaw_eskf_rejected_updates(),
          tracked_target.ekf().last_nis,
          tracked_target.armor_motion_trusted(tracked_target.last_id) ? 1 : 0,
          armor_velocity.x(), armor_velocity.y(), armor_velocity.z(),
          tracked_target.armor_motion_nis(tracked_target.last_id));
      }
    }
    // ---- 弹着相位门的窗口平均角速度（与 YawESKF 的瞬时 ω 解耦）----
    if (!targets.empty() && std::isfinite(gyro_body_phase_rad)) {
      const double now_phase_s =
        std::chrono::duration<double>(t.time_since_epoch()).count();
      body_phase_history.emplace_back(now_phase_s, gyro_body_phase_rad);
      while (!body_phase_history.empty() &&
             now_phase_s - body_phase_history.front().first >
               xuc_fire_phase_rate_window_s) {
        body_phase_history.pop_front();
      }
      const double phase_span_s = now_phase_s - body_phase_history.front().first;
      if (body_phase_history.size() >= 3 && phase_span_s > 0.4) {
        double unwrapped = 0.0;
        for (std::size_t i = 1; i < body_phase_history.size(); ++i) {
          unwrapped += std::remainder(
            body_phase_history[i].second - body_phase_history[i - 1].second,
            M_PI / 2.0);
        }
        body_phase_rate_window = unwrapped / phase_span_s;
      } else {
        body_phase_rate_window = gyro_omega_rad_s;   // 刚捕获目标时回退
      }
    } else {
      body_phase_history.clear();
      body_phase_rate_window = 0.0;
    }
    const double model_now_s = std::chrono::duration<double>(
      t.time_since_epoch()).count();
    const bool prior_phase_rate_valid = gyro_phase_scheduler.period_valid();
    const double prior_phase_omega_rad_s = prior_phase_rate_valid
      ? (M_PI / 2.0) / gyro_phase_scheduler.face_period_s()
      : NAN;
    const auto rotation_model = rotation_model_manager.update(
      model_now_s, tracker_converged && yaw_motion_model_trusted,
      gyro_omega_rad_s,
      prior_phase_rate_valid, prior_phase_omega_rad_s);
    if (rotation_model.phase_reset) {
      gyro_phase_scheduler.reset();
      gyro_face_selector.reset();
      event_last_fired_cycle = -1;
      xuc_one_face_armed = true;
      body_phase_history.clear();
      body_phase_rate_window = 0.0;
      tools::logger()->warn(
        "[MODEL] phase reset mode={} ekf_omega={:+.3f} pll_omega={:+.3f} "
        "rate_delta={:.3f}",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s, prior_phase_omega_rad_s,
        rotation_model.rate_disagreement_rad_s);
    }
    if (rotation_model.mode_changed) {
      tools::logger()->info(
        "[MODEL] switched to {} ekf_omega={:+.3f}rad_s",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s);
    }
    vehicle_model_fresh = xuc_gyro_center_follow_enabled &&
      rotation_model.tracker_trusted &&
      rotation_model.mode != auto_aim::RotationModelManager::Mode::armor;
    gyro_model_fresh = vehicle_model_fresh &&
      rotation_model.mode ==
        auto_aim::RotationModelManager::Mode::high_speed_phase;
    gyro_phase_model_trusted = gyro_model_fresh &&
      rotation_model.phase_fire_trusted;
    if (vehicle_model_fresh) {
      // Both low/medium rotation and high-speed gyro share the same stable
      // vehicle-centre gimbal reference.  Only their firing clocks differ:
      // continuous future-face prediction below the high-speed threshold and
      // the physical phase-event scheduler above it.
      gyro_center_follow_active = true;
      xuc_gyro_center_memory_valid = true;
      xuc_gyro_center_memory_yaw = gyro_center_yaw;
      xuc_gyro_center_last_fresh = t;
    } else if (xuc_gyro_center_memory_valid) {
      const double center_age_s =
        std::chrono::duration<double>(t - xuc_gyro_center_last_fresh).count();
      if (center_age_s <= xuc_gyro_center_hold_s) {
        // A tracker reset must not make the gimbal chase whichever plate is
        // visible for one frame. Hold the last validated vehicle center, but
        // keep gyro_model_fresh=false so the fire gate remains closed.
        gyro_center_follow_active = true;
        gyro_center_yaw = xuc_gyro_center_memory_yaw;
      } else {
        xuc_gyro_center_memory_valid = false;
      }
    }
    if (gyro_center_follow_active != xuc_gyro_center_follow_logged) {
      tools::logger()->info(
        "[XUC][GYRO] center_follow={} tracker={} omega={:+.3f}rad_s radius={:.3f}m",
        gyro_center_follow_active, tracker.state(), gyro_omega_rad_s, gyro_radius_m);
      xuc_gyro_center_follow_logged = gyro_center_follow_active;
    }

    // 调试：打印 armors 和 targets 数量
    if (!competition_headless) {
      fmt::print("[DEBUG] armors={}, targets={}\n", armors.size(), targets.size());
    }

#ifdef AMENT_CMAKE_FOUND
    // 发布装甲板Marker（可视化检测结果）
    if (!armors.empty()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      rclcpp::Time ros_time(ns);

      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & armor : armors) {
        auto marker = visualizer->create_sphere_marker(
          "world", "armors", marker_id++,
          armor.xyz_in_world.x(),
          armor.xyz_in_world.y(),
          armor.xyz_in_world.z(),
          1.0, 0.0, 0.0, 0.8,  // 红色，80%不透明
          0.1,  // 10cm直径
          ros_time
        );
        marker_array.markers.push_back(marker);
      }
      visualizer->publish_marker_array(marker_array);
    }
#endif

    // Keep raw-measurement control associated with the currently followed
    // bearing.  On a fast blue target YOLO can return two armor candidates in
    // one frame; blindly taking list::front() makes the command jump between
    // candidates and breaks both slew and fire qualification.
    const auto_aim::Armor * direct_measurement_armor = nullptr;
    if (!armors.empty()) {
      const int associated_detection_index = tracker.primary_detection_index();
      if (vehicle_model_fresh && associated_detection_index >= 0 &&
          associated_detection_index < static_cast<int>(armors.size())) {
        auto associated = armors.begin();
        std::advance(associated, associated_detection_index);
        direct_measurement_armor = &*associated;
      } else {
        direct_measurement_armor = &armors.front();
      }
      if ((!vehicle_model_fresh || associated_detection_index < 0) &&
          direct_yaw_initialized && armors.size() > 1) {
        double best_yaw_distance = INFINITY;
        for (const auto & armor : armors) {
          const double armor_yaw =
            std::atan2(armor.xyz_in_world.y(), armor.xyz_in_world.x());
          const double yaw_distance = std::abs(
            std::remainder(armor_yaw - direct_yaw_filtered, 2.0 * M_PI));
          if (std::isfinite(yaw_distance) && yaw_distance < best_yaw_distance) {
            best_yaw_distance = yaw_distance;
            direct_measurement_armor = &armor;
          }
        }
      }
    }

    // Use the rigid-body geometry as the range source for trajectory timing.
    // At a face-centre gyro event the near armor is approximately one radius
    // closer than the vehicle centre.  Raw oblique-PnP depth is only a small,
    // bounded correction; the latest field log ranged from 3.24 to 5.92 m for
    // a nominal 5 m target and otherwise injected more than 30 degrees of
    // time-of-flight phase error.
    double raw_pnp_range_m = NAN;
    if (direct_measurement_armor != nullptr) {
      raw_pnp_range_m = std::hypot(
        direct_measurement_armor->xyz_in_world[0],
        direct_measurement_armor->xyz_in_world[1]);
    }
    double modeled_armor_range_m = NAN;
    if (!targets.empty()) {
      const auto tracked_x = targets.front().ekf_x();
      if (gyro_model_fresh && tracked_x.size() >= 9) {
        const double center_range_m = std::hypot(tracked_x[0], tracked_x[2]);
        if (std::isfinite(center_range_m) && std::isfinite(tracked_x[8])) {
          modeled_armor_range_m = center_range_m - std::abs(tracked_x[8]);
        }
      } else {
        const int primary_face_id = tracker.primary_face_id();
        const auto modeled_faces = targets.front().armor_xyza_list();
        if (primary_face_id >= 0 &&
            primary_face_id < static_cast<int>(modeled_faces.size())) {
          modeled_armor_range_m = std::hypot(
            modeled_faces[primary_face_id][0], modeled_faces[primary_face_id][1]);
        }
      }
    }
    const auto robust_range = robust_range_filter.update(
      model_now_s, modeled_armor_range_m, raw_pnp_range_m);
    const double robust_target_range_m = robust_range.valid
      ? robust_range.range_m
      : raw_pnp_range_m;
    static int range_log_counter = 0;
    if (++range_log_counter % 50 == 0) {
      tools::logger()->info(
        "[RANGE] raw={:.3f}m model={:.3f}m filtered={:.3f}m "
        "model_used={} raw_limited={}",
        raw_pnp_range_m, modeled_armor_range_m, robust_target_range_m,
        robust_range.used_model ? 1 : 0, robust_range.raw_limited ? 1 : 0);
    }

    // 小陀螺时用主 EKF 的整车几何选板/预测（AimGeometry::whole_ekf），
    // 静止/低速仍走原来的 ekf_x() 路径；trace 只用于诊断。
    auto_aim::AimPredictionTrace native_fire_prediction;
    auto command = aimer.aim(
      targets, t, cboard.bullet_speed, true, &native_fire_prediction,
      auto_aim::AimGeometry::whole_ekf);  // to_now=true，生成当前时刻命令
    const bool native_prediction_valid =
      command.control && !targets.empty() && std::isfinite(command.yaw);
    const bool native_prediction_trusted =
      native_prediction_valid && tracker_converged && yaw_motion_model_trusted;
    const double native_predicted_yaw = command.yaw;
    double gyro_predicted_impact_yaw_error_rad = INFINITY;
    bool gyro_predicted_impact_yaw_valid = false;
    bool gyro_event_schedule_valid = false;
    bool gyro_event_interval_safe = false;
    std::int64_t gyro_event_hit_cycle = -1;
    double gyro_event_lead_error_s = INFINITY;
    double gyro_event_face_period_s = NAN;
    double gyro_event_half_window_rad = NAN;
    double gyro_event_worst_phase_rad = NAN;
    double gyro_event_combined_phase_rad = NAN;
    double gyro_event_phase_uncertainty_rad = NAN;
    bool gyro_event_face_valid = false;
    int gyro_event_face_id = -1;
    double gyro_event_face_alignment_rad = INFINITY;
    double gyro_event_gimbal_tracking_error_rad = INFINITY;
    std::chrono::steady_clock::time_point gyro_event_command_due_time{};

    // Phase 2D.5 integration guard.  The detector pose is much more stable
    // than the current high-dynamic EKF tuning at close range.  Keep the
    // complete Tracker/Aimer prediction path, but prevent a divergent state
    // from commanding a yaw far away from the armor actually in the image.
    // If the tracker briefly rejects convergence while a valid enemy armor is
    // still detected, degrade to measurement-only yaw tracking and never fire.
    bool measurement_fallback = false;
    bool direct_yaw_measurement_fresh = false;
    bool direct_pitch_measurement_fresh = false;
    double direct_yaw_pixel_error_rad = INFINITY;
    double direct_pitch_pixel_error_rad = INFINITY;
    double direct_pitch_raw_pixel_error_rad = INFINITY;
    double ballistic_pitch_compensation_rad = 0.0;
    double ballistic_camera_range_m = NAN;
    double ballistic_total_holdover_m = 0.0;
    // Impact-time yaw lead actually applied on top of vehicle-centre follow.
    // Kept as a frame-local diagnostic so the field log can prove whether the
    // lead was commanded and whether it sat at its envelope.
    double center_command_lead_rad = 0.0;
    // Per-shot diagnostics only; these values never participate in control.
    double shot_diag_signed_phase_rad = NAN;
    double shot_diag_phase_horizon_s = NAN;
    // A newly constructed tracker returns provisional targets before its
    // normal convergence window.  Keep those detections visible, but never let
    // a one-frame pose regain actuator authority after a genuine track loss.
    if (targets.empty()) direct_yaw_control_confirmed = false;
    if (direct_yaw_guard_enabled) {
      bool valid_measurement = false;
      double measured_yaw = 0.0;
      if (direct_measurement_armor != nullptr) {
        const auto & measured_armor = *direct_measurement_armor;
        measured_yaw = gyro_center_follow_active
          ? gyro_center_yaw
          : std::atan2(measured_armor.xyz_in_world.y(), measured_armor.xyz_in_world.x());
        valid_measurement = std::isfinite(measured_yaw);
        if (valid_measurement && direct_yaw_focal_x_px > 0.0 && !img.empty()) {
          double center_x_px = measured_armor.center.x;
          if (!measured_armor.points.empty()) {
            center_x_px = 0.0;
            for (const auto & point : measured_armor.points) center_x_px += point.x;
            center_x_px /= static_cast<double>(measured_armor.points.size());
          }
          const double raw_yaw_pixel_error_rad = std::atan2(
            center_x_px - 0.5 * static_cast<double>(img.cols), direct_yaw_focal_x_px);
          // On the installed yaw convention, a negative impact correction moves
          // the armor to image-left by the same angle.  Fire on the residual from
          // that requested offset, not on distance from the image centre.
          direct_yaw_pixel_error_rad =
            raw_yaw_pixel_error_rad - direct_yaw_impact_correction_rad;
          direct_yaw_measurement_fresh = std::isfinite(direct_yaw_pixel_error_rad);
        }
      }

      if (valid_measurement) {
        if (!direct_yaw_initialized) {
          direct_yaw_filtered = measured_yaw;
          direct_yaw_initialized = true;
        } else {
          // Cap dt so a camera stall or a long loss cannot turn the rate limit
          // into one large step on the first recovered frame.
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_yaw_last_update).count(),
            0.0, 0.05);
          const double yaw_error =
            std::remainder(measured_yaw - direct_yaw_filtered, 2.0 * M_PI);
          const double alpha = direct_yaw_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_yaw_filter_tau_s)
            : 1.0;
          double yaw_step = alpha * yaw_error;
          if (direct_yaw_max_rate_rad_s > 0.0) {
            const double max_step = direct_yaw_max_rate_rad_s * dt;
            yaw_step = std::clamp(yaw_step, -max_step, max_step);
          }
          direct_yaw_filtered = std::remainder(
            direct_yaw_filtered + yaw_step, 2.0 * M_PI);
        }
        direct_yaw_last_update = t;
        direct_yaw_last_measurement = t;
        if (direct_yaw_long_loss) {
          tools::logger()->info("[XUC][YAW] measurement reacquired with continuous slew");
        }
        direct_yaw_long_loss = false;

        measurement_fallback = true;
        command.shoot = false;
        if (!direct_yaw_control_confirmed && tracker_converged) {
          direct_yaw_control_confirmed = true;
          tools::logger()->info(
            "[XUC][YAW] tracker convergence confirmed; actuator reacquired");
        }
        command.control = direct_yaw_control_confirmed;
        if (!direct_yaw_control_confirmed) {
          command.yaw = direct_yaw_filtered;
        } else if (gyro_center_follow_active) {
          // In every rotating-target mode the visible plate bearing oscillates
          // around the vehicle. Following the EKF rotation center avoids
          // switching the gimbal at each of the four plate hand-overs.
          double center_command = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
          if (center_follow_lead_enabled && gyro_radius_m > 0.05 &&
              std::isfinite(robust_target_range_m) && robust_target_range_m > 0.10) {
            // A spinning vehicle turns about its own centre, so the centre does
            // not translate and there is NO omega * flight_time lead to add:
            // the armour sits at only atan(r/d) off the centre bearing (~2 deg).
            // What does lag is the centre estimate itself, which is fitted from
            // a short recent window.  Advance the centre by its own lateral
            // velocity over the flight time and correct toward that bearing.
            const double center_flight_time_s =
              robust_target_range_m / xuc_projectile_speed_mps;
            const bool center_translating =
              gyro_center_velocity_mps > xuc_center_follow_lead_min_center_speed_mps;
            if (center_translating) {
              const double predicted_cx =
                center_x_m + center_vx_mps * center_flight_time_s;
              const double predicted_cy =
                center_y_m + center_vy_mps * center_flight_time_s;
              center_command_lead_rad = auto_aim::center_follow_lead_rad(
                auto_aim::center_azimuth_rad(predicted_cx, predicted_cy),
                auto_aim::center_azimuth_rad(center_x_m, center_y_m),
                center_follow_lead_max_rad);
              center_command = std::remainder(
                center_command + center_command_lead_rad, 2.0 * M_PI);
            }
          }
          command.yaw = center_command;
        } else if (prediction_lead_limit_configured && native_prediction_trusted) {
          const double prediction_lead =
            std::remainder(native_predicted_yaw - measured_yaw, 2.0 * M_PI);
          command.yaw = std::remainder(
            direct_yaw_filtered + std::clamp(
              prediction_lead, -max_prediction_lead_rad, max_prediction_lead_rad) +
              direct_yaw_command_correction_rad,
            2.0 * M_PI);
        } else {
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
        }
      } else if (direct_yaw_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_yaw_last_measurement).count();
        if (visual_gap <= direct_yaw_hold_s) {
          // During a short detector gap, allow only the already-bounded native
          // prediction.  This keeps fast motion continuous without trusting an
          // unconstrained EKF state.
          command.control = true;
          command.shoot = false;
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
          if (prediction_lead_limit_configured && native_prediction_trusted) {
            const double prediction_delta =
              std::remainder(native_predicted_yaw - direct_yaw_filtered, 2.0 * M_PI);
            command.yaw = std::remainder(
              direct_yaw_filtered + std::clamp(
                prediction_delta, -max_prediction_lead_rad, max_prediction_lead_rad) +
                direct_yaw_command_correction_rad,
              2.0 * M_PI);
          }
          measurement_fallback = true;
        } else {
          // A genuine loss must stop control, but retaining the filtered yaw and
          // refreshing last_update prevents a full-angle jump on reacquisition.
          if (!direct_yaw_long_loss) {
            tools::logger()->warn("[XUC][YAW] visual loss: control stopped, yaw state retained");
          }
          direct_yaw_long_loss = true;
          direct_yaw_control_confirmed = false;
          command.control = false;
          command.shoot = false;
          measurement_fallback = true;
        }
        direct_yaw_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
        measurement_fallback = true;
      }
    }
    if (direct_pitch_guard_enabled) {
      bool valid_pitch_measurement = false;
      double measured_pitch = 0.0;
      if (direct_measurement_armor != nullptr && direct_pitch_focal_y_px > 0.0 && !img.empty()) {
        const auto & measured_armor = *direct_measurement_armor;
        double center_y_px = measured_armor.center.y;
        if (!measured_armor.points.empty()) {
          center_y_px = 0.0;
          for (const auto & point : measured_armor.points) center_y_px += point.y;
          center_y_px /= static_cast<double>(measured_armor.points.size());
        }
        const double pixel_pitch_error = std::atan2(
          center_y_px - 0.5 * static_cast<double>(img.rows),
          direct_pitch_focal_y_px);
        direct_pitch_raw_pixel_error_rad = pixel_pitch_error;

        if (xuc_ballistic_pitch_enabled) {
          const double raw_camera_range_m = std::hypot(
            measured_armor.xyz_in_gimbal.x(), measured_armor.xyz_in_gimbal.y());
          ballistic_camera_range_m = std::isfinite(robust_target_range_m)
            ? robust_target_range_m
            : raw_camera_range_m;
          if (std::isfinite(ballistic_camera_range_m) && ballistic_camera_range_m >= 0.10) {
            // Reuse the same closed-form low-arc solver that is exercised by
            // the Gestalt Tongji/SHtech Aimer.  The camera is forward of and
            // above the muzzle, while camera/gimbal Z points down.
            const double range_scale = raw_camera_range_m > 0.10
              ? ballistic_camera_range_m / raw_camera_range_m
              : 1.0;
            const double muzzle_x_m = measured_armor.xyz_in_gimbal.x() * range_scale +
              xuc_camera_forward_of_muzzle_m;
            const double muzzle_y_m = measured_armor.xyz_in_gimbal.y() * range_scale;
            const double muzzle_range_m = std::hypot(muzzle_x_m, muzzle_y_m);
            const double target_height_from_muzzle_m =
              -measured_armor.xyz_in_gimbal.z() + xuc_camera_above_muzzle_m;
            const double camera_los_up_rad = std::atan2(
              -measured_armor.xyz_in_gimbal.z(), ballistic_camera_range_m);
            const double reference_muzzle_range_m =
              xuc_ballistic_reference_camera_range_m + xuc_camera_forward_of_muzzle_m;
            const double reference_measured_compensation_rad = std::atan2(
              xuc_ballistic_reference_total_drop_m, reference_muzzle_range_m);
            const tools::Trajectory reference_trajectory(
              xuc_ballistic_reference_projectile_speed_mps,
              reference_muzzle_range_m,
              xuc_camera_above_muzzle_m);
            const tools::Trajectory current_trajectory(
              xuc_projectile_speed_mps, muzzle_range_m, target_height_from_muzzle_m);
            if (!reference_trajectory.unsolvable && !current_trajectory.unsolvable) {
              // The one real-shot datum calibrates only the fixed mechanical /
              // boresight bias.  Distance, height, gravity and flight time are
              // then solved physically instead of extrapolating a measured miss
              // with distance squared.
              const double fixed_boresight_bias_rad =
                reference_measured_compensation_rad - reference_trajectory.pitch;
              ballistic_pitch_compensation_rad = std::clamp(
                fixed_boresight_bias_rad + current_trajectory.pitch - camera_los_up_rad +
                  xuc_ballistic_pitch_trim_rad,
                0.0,
                xuc_ballistic_max_compensation_rad);
              ballistic_total_holdover_m =
                std::tan(ballistic_pitch_compensation_rad) * muzzle_range_m;
            } else {
              ballistic_pitch_compensation_rad = NAN;
              ballistic_total_holdover_m = NAN;
            }
          } else {
            ballistic_pitch_compensation_rad = NAN;
          }
        }
        const double compensated_pitch_error =
          pixel_pitch_error - ballistic_pitch_compensation_rad;
        // Physical acceptance established that a target below image centre
        // requires a larger lower-board IMU pitch target. Closing the loop in
        // image space avoids the currently inconsistent pitch hand-eye offset.
        // Positive ballistic compensation deliberately settles the armor below
        // image centre so the muzzle is elevated above the visual line of sight.
        measured_pitch = xuc.imu_pitch() + direct_pitch_error_gain * compensated_pitch_error;
        valid_pitch_measurement = std::isfinite(measured_pitch);
        direct_pitch_pixel_error_rad = compensated_pitch_error;
        direct_pitch_measurement_fresh = valid_pitch_measurement &&
          std::isfinite(direct_pitch_pixel_error_rad);
      }

      if (valid_pitch_measurement) {
        if (!direct_pitch_initialized) {
          direct_pitch_filtered = measured_pitch;
          direct_pitch_initialized = true;
        } else {
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_pitch_last_update).count(),
            0.0, 0.05);
          const double pitch_error = measured_pitch - direct_pitch_filtered;
          const double alpha = direct_pitch_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_pitch_filter_tau_s)
            : 1.0;
          double pitch_step = alpha * pitch_error;
          if (direct_pitch_max_rate_rad_s > 0.0) {
            const double max_step = direct_pitch_max_rate_rad_s * dt;
            pitch_step = std::clamp(pitch_step, -max_step, max_step);
          }
          direct_pitch_filtered += pitch_step;
        }
        direct_pitch_last_update = t;
        direct_pitch_last_measurement = t;
        if (direct_pitch_long_loss) {
          tools::logger()->info("[XUC][PITCH] measurement reacquired with continuous slew");
        }
        direct_pitch_long_loss = false;
        command.pitch = direct_pitch_filtered;
      } else if (direct_pitch_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_pitch_last_measurement).count();
        if (visual_gap <= direct_pitch_hold_s) {
          command.pitch = direct_pitch_filtered;
        } else {
          if (!direct_pitch_long_loss) {
            tools::logger()->warn("[XUC][PITCH] visual loss: control stopped, pitch state retained");
          }
          direct_pitch_long_loss = true;
          command.control = false;
          command.shoot = false;
        }
        direct_pitch_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
      }
    }

    // The final rate limiter includes the bounded ESKF lead and every model
    // hand-over.  Synchronize it to the actual lower-board yaw outside AUTO or
    // on a genuine visual loss, so re-entering AUTO cannot dump a stale target
    // into the inner position loop in one frame.
    // The current protocol carries a robot ID, not the old yaw-arm bitfield.
    const bool lower_yaw_control_ready = xuc.enabled() && xuc.control_ready() &&
      xuc.rx_valid() && xuc.mode() == static_cast<uint8_t>(io::Mode::auto_aim);

    // 2026-10-14 新增：常驻 control 保持逻辑。
    // 触发条件：开关开启 + 下位机 AUTO + 视觉丢失（超短时窗口）。
    // 行为：保持 control=1（摩擦轮继续转），锁定当前姿态（云台不动）。
    // 排除：瞄准点无效、弹道无解等其他 control=false 原因。
    // 安全：退出 AUTO 或链路失效时自动失效（lower_yaw_control_ready 为 false）。
    const bool can_hold_attitude = lower_yaw_control_ready &&
      std::isfinite(xuc.raw_imu_yaw()) && std::isfinite(xuc.raw_imu_pitch());
    const bool visual_lost = targets.empty() && direct_yaw_initialized &&
      std::chrono::duration<double>(t - direct_yaw_last_measurement).count() >
        direct_yaw_hold_s;
    const bool persist_hold_active = xuc_control_persist_enabled &&
      can_hold_attitude && visual_lost && !command.control;
    if (persist_hold_active) {
      command.control = true;
      command.yaw = xuc.raw_yaw_correction_to_command(xuc.raw_imu_yaw());
      command.pitch = xuc.raw_pitch_correction_to_command(xuc.raw_imu_pitch());
      // shoot 由后续 fire_candidate 计算，因 fresh=false 而恒为 false
      static bool persist_logged = false;
      if (!persist_logged) {
        tools::logger()->info(
          "[XUC][PERSIST] visual lost: holding control, locking attitude");
        persist_logged = true;
      }
    }

    if (final_yaw_command_max_rate_rad_s > 0.0 && xuc.enabled()) {
      const auto limiter_now = std::chrono::steady_clock::now();
      const bool actual_yaw_valid = xuc.rx_valid() &&
        std::isfinite(xuc.raw_imu_yaw());
      const double actual_command_yaw = actual_yaw_valid
        ? xuc.raw_yaw_correction_to_command(xuc.raw_imu_yaw())
        : 0.0;

      if (!lower_yaw_control_ready || !command.control || !std::isfinite(command.yaw)) {
        if (actual_yaw_valid) {
          final_yaw_command_filtered = actual_command_yaw;
          final_yaw_command_initialized = true;
        }
        final_yaw_command_last_update = limiter_now;
        final_yaw_lower_armed = false;
      } else {
        if (!final_yaw_command_initialized) {
          final_yaw_command_filtered = actual_yaw_valid
            ? actual_command_yaw
            : command.yaw;
          final_yaw_command_initialized = true;
          final_yaw_command_last_update = limiter_now;
        }
        if (!final_yaw_lower_armed) {
          final_yaw_lower_armed = true;
          final_yaw_command_last_update = limiter_now;
          tools::logger()->info("[XUC][YAW] final slew limiter active: max_rate={:.1f} deg/s",
            final_yaw_command_max_rate_rad_s * 180.0 / M_PI);
        }
        const double dt = std::clamp(
          std::chrono::duration<double>(
            limiter_now - final_yaw_command_last_update).count(),
          0.0, 0.05);
        final_yaw_command_last_update = limiter_now;
        const double desired_step = std::remainder(
          command.yaw - final_yaw_command_filtered, 2.0 * M_PI);
        const double max_step = final_yaw_command_max_rate_rad_s * dt;
        final_yaw_command_filtered = std::remainder(
          final_yaw_command_filtered +
            std::clamp(desired_step, -max_step, max_step),
          2.0 * M_PI);
        command.yaw = final_yaw_command_filtered;
      }
    }
    const bool native_shoot = shooter.shoot(command, aimer, targets, ypr);
    if (xuc_fire_guard_enabled) {
      bool face_valid = false;
      double face_error_rad = INFINITY;
      if (!targets.empty() && aimer.debug_aim_point.valid) {
        const auto tracked_x = targets.front().ekf_x();
        if (tracked_x.size() >= 3) {
          const double center_bearing = std::atan2(tracked_x[2], tracked_x[0]);
          face_error_rad = std::abs(std::remainder(
            aimer.debug_aim_point.xyza[3] - center_bearing, 2.0 * M_PI));
          face_valid = std::isfinite(face_error_rad);
        }
      }
      // 弹着相位：实测体相位外推到弹丸到达时刻，取最近装甲板相对中心方位的
      // 夹角（fold 到 90° 一个面，0..45°）。center_follow 下这就是命中判据。
      double impact_phase_err_rad = INFINITY;
      if (!targets.empty() && std::isfinite(gyro_body_phase_rad) &&
          std::isfinite(gyro_center_yaw) && std::isfinite(robust_target_range_m) &&
          robust_target_range_m > 0.1) {
        const double processing_age_s = std::max(0.0, std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t).count());
        const double shot_delay_s =
          std::abs(gyro_omega_rad_s) > xuc_phase_decision_speed_rad_s
            ? xuc_phase_high_speed_delay_s : xuc_phase_low_speed_delay_s;
        const double flight_s = robust_target_range_m / xuc_projectile_speed_mps;
        // 相位源必须与 AimGeometry 同源：whole_ekf 时选板/预测用的是主 EKF 的
        // x[6]，而 gyro_body_phase_rad 来自被 YawESKF 覆盖的 ekf_x()[6]，两者实测
        // 差约 30°，拿它算弹着相位会让弹着相位诊断和瞄准链路对不上。
        const auto & whole_phase_state = targets.front().ekf().x;
        const double body_phase_used =
          (native_fire_prediction.whole_ekf_geometry && whole_phase_state.size() == 11)
            ? whole_phase_state[6]
            : gyro_body_phase_rad;
        const double phase_at_impact =
          body_phase_used + body_phase_rate_window *
            (processing_age_s + shot_delay_s + flight_s);
        impact_phase_err_rad = std::abs(std::remainder(
          phase_at_impact - gyro_center_yaw, M_PI / 2.0));
        shot_diag_signed_phase_rad = std::remainder(
          phase_at_impact - gyro_center_yaw, M_PI / 2.0);
        shot_diag_phase_horizon_s = processing_age_s + shot_delay_s + flight_s;
      }
      const bool gyro_predicted_fire_active =
        vehicle_model_fresh && gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      const bool vehicle_predicted_fire_active =
        vehicle_model_fresh && !gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      // One-shot-per-face is meaningful only for the high-speed phase clock.
      // At low/medium speed, a valid aim window may last much longer than one
      // face passage; let feeder feedback re-arm repeated one-frame pulses.
      const bool one_face_policy_active =
        xuc_one_face_once_fire && gyro_predicted_fire_active;
      const bool predictive_fire_active =
        gyro_predicted_fire_active || vehicle_predicted_fire_active;
      // 【节奏门控】两种模式：
      //  (a) 遥测模式（默认，xuc_fire_cadence_fallback_ms == 0）：依赖下位机上报的
      //      拨弹转速来判定“上一发已结束”。前提是固件已接入 feeder 遥测。
      //  (b) 定时回退模式（xuc_fire_cadence_fallback_ms > 0）：不依赖拨弹遥测，
      //      改为“距上次开火至少间隔 N 毫秒”。
      //      启用原因（2026-10-13 实测）：电控固件把 bullet_speed/bullet_count 写死 0，
      //      遥测模式下永远等不到拨弹反馈，导致开火被反复 inhibit，实测 30 秒仅 7 发。
      //      同一实测证明：单帧 shoot=1（约 10 ms）即可触发拨弹动作，故固定间隔可行。
      const bool cadence_uses_fallback = xuc_fire_cadence_fallback_s > 0.0;
      const double since_last_shot_s =
        xuc_last_shot_issue_time.time_since_epoch().count() > 0
          ? std::chrono::duration<double>(t - xuc_last_shot_issue_time).count()
          : std::numeric_limits<double>::infinity();
      const bool cadence_fallback_gate_open =
        cadence_uses_fallback &&
        (!xuc_pending_shot && xuc_active_shot_id == 0) &&
        since_last_shot_s >= xuc_fire_cadence_fallback_s;
      const bool feedback_cadence_gate_open =
        cadence_uses_fallback
          ? cadence_fallback_gate_open
          : (!xuc.enabled() ||
             (!xuc_pending_shot && xuc_active_shot_id == 0 &&
              (!xuc_feeder_activity_seen ||
               (!xuc_feeder_active &&
                std::chrono::duration<double>(t - xuc_feeder_last_active).count() >=
                  xuc_gyro_post_feed_recovery_s))));
      if (xuc.enabled() && !feedback_cadence_gate_open) {
        ++xuc_fire_inhibit_count;   // 供监控窗口统计“被拨弹门控拦下”的次数
        if (!xuc_gyro_cadence_hold_logged) {
          tools::logger()->info(
            "[XUC][FIRE] request inhibited until {}",
            cadence_uses_fallback ? "cadence fallback interval elapses"
                                  : "feeder is available");
          xuc_gyro_cadence_hold_logged = true;
        }
      } else if (xuc_gyro_cadence_hold_logged) {
        tools::logger()->info(
          "[XUC][FIRE] {} available; fresh qualification required",
          cadence_uses_fallback ? "cadence fallback" : "feeder");
        xuc_gyro_cadence_hold_logged = false;
      }
      if (predictive_fire_active && native_prediction_valid) {
        // native_predicted_yaw is produced from the same future AimPoint used by
        // face_error_rad and already includes configured actuation delay plus the
        // iterated projectile flight time. Compare it with actual gimbal attitude,
        // not the centre-follow command, so servo lag is included in qualification.
        gyro_predicted_impact_yaw_error_rad = std::remainder(
          native_predicted_yaw - ypr[0] - xuc_gyro_predicted_yaw_trim_rad,
          2.0 * M_PI);
        gyro_predicted_impact_yaw_valid =
          std::isfinite(gyro_predicted_impact_yaw_error_rad);
      }
      const bool convergence_gate_open =
        !xuc_fire_require_tracker_converged || tracker_converged;
      const bool stationary_yaw_gate_open = direct_yaw_measurement_fresh &&
        std::abs(direct_yaw_pixel_error_rad) <= xuc_fire_yaw_tolerance_rad;
      // A held gyro centre is useful for smooth reacquisition, but it is neither
      // a fresh moving model nor a stationary-target solution. Do not fall through
      // to the stationary fire policy until centre-memory mode has ended.
      const bool motion_transition_gate_open =
        !gyro_center_follow_active || vehicle_model_fresh;
      // ---- Event-clock phase scheduler ----
      // Fold the EKF body yaw modulo one face (pi/2).  Every armor id therefore
      // shares the same physical front-center zero and a detector association
      // switch cannot move the event clock by an entire face.  Raw PnP armor
      // yaw is intentionally not used as the clock because field logs showed
      // paired long/short zero crossings and clustered hit/miss intervals.
      const bool ev_gyro_active = gyro_predicted_fire_active && gyro_model_fresh;
      if (ev_gyro_active && direct_measurement_armor != nullptr &&
          direct_yaw_measurement_fresh && std::isfinite(gyro_center_yaw) &&
          std::isfinite(gyro_body_phase_rad)) {
        const double observed_phase_error = std::remainder(
          gyro_body_phase_rad - gyro_center_yaw, M_PI / 2.0);
        const double observation_time_s = std::chrono::duration<double>(
          t.time_since_epoch()).count();
        const auto event_observation = gyro_phase_scheduler.observe(
          observation_time_s, observed_phase_error, gyro_omega_rad_s);
        if (event_observation.period_updated) {
          tools::logger()->info(
            "[GYROPLL] phase_zero cycle={} sample={:.1f} ms periods={} alias_guard={} "
            "filtered={:.1f} ms residual={:+.1f} ms correction={:+.1f} ms "
            "uncertainty={:.1f} ms "
            "locks={} locked={} reacquired={} rejects={} omega={:.3f} ekf_omega={:+.3f}",
            gyro_phase_scheduler.crossing_cycle(),
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.face_period_s * 1000.0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.applied_phase_correction_s * 1000.0,
            event_observation.phase_uncertainty_s * 1000.0,
            event_observation.lock_updates,
            event_observation.locked ? 1 : 0,
            event_observation.reacquired ? 1 : 0,
            event_observation.consecutive_rejections,
            (M_PI / 2.0) / event_observation.face_period_s,
            gyro_omega_rad_s);
        } else if (event_observation.crossing && event_observation.rejected) {
          tools::logger()->warn(
            "[GYROPLL] rejected phase crossing sample={:.1f} ms periods={} alias_guard={} "
            "residual={:+.1f} ms rejects={} ekf_omega={:+.3f}",
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.consecutive_rejections,
            gyro_omega_rad_s);
        }
      } else if (!gyro_center_follow_active) {
        gyro_phase_scheduler.reset();
        gyro_face_selector.reset();
        event_last_fired_cycle = -1;
      }
      if (!ev_gyro_active) gyro_face_selector.reset();

      // The flight-time estimate uses the current measured armor range.  The
      // interval gate below separately accounts for flight-time uncertainty.
      double event_flight_time_s = NAN;
      if (ev_gyro_active && direct_measurement_armor != nullptr) {
        const auto & measured_xyz = direct_measurement_armor->xyz_in_world;
        const double d_face_m = std::isfinite(robust_target_range_m)
          ? robust_target_range_m
          : std::hypot(measured_xyz[0], measured_xyz[1]);
        event_flight_time_s = d_face_m / xuc_projectile_speed_mps;
        const tools::Trajectory trj(
          xuc_projectile_speed_mps, d_face_m, measured_xyz[2]);
        if (!trj.unsolvable && trj.fly_time > 0.0) {
          event_flight_time_s = trj.fly_time;
        }
      }

      auto event_solution = auto_aim::GyroPhaseScheduler::Solution{};
      if (ev_gyro_active && std::isfinite(event_flight_time_s)) {
        const double decision_now_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        event_solution = gyro_phase_scheduler.solve(
          decision_now_s,
          shot_delay_adapter.delay_s(),
          shot_delay_adapter.delay_min_s(),
          shot_delay_adapter.delay_max_s(),
          event_flight_time_s,
          xuc_event_flight_time_uncertainty_s,
          gyro_radius_m,
          xuc_event_armor_width_m,
          xuc_event_armor_edge_margin_m,
          xuc_event_model_phase_uncertainty_rad);
        if (event_solution.valid) {
          gyro_event_schedule_valid = true;
          // The PLL is tied to directly observed physical face crossings.  Use
          // its temporal uncertainty as the primary event gate.  Deployments
          // that trust the EKF face phase may additionally require the harder
          // combined face-consistency check below.
          const double relaxed_half_window_rad =
            event_solution.half_hit_window_rad + xuc_event_phase_grace_rad;
          gyro_event_interval_safe =
            !xuc_event_require_face_consistency &&
            event_solution.worst_interval_phase_rad <= relaxed_half_window_rad;
          gyro_event_hit_cycle = event_solution.hit_cycle;
          gyro_event_lead_error_s = event_solution.time_until_command_s;
          gyro_event_face_period_s = event_solution.face_period_s;
          gyro_event_half_window_rad = event_solution.half_hit_window_rad;
          gyro_event_worst_phase_rad = event_solution.worst_interval_phase_rad;
          gyro_event_phase_uncertainty_rad = event_solution.phase_uncertainty_rad;
          gyro_event_command_due_time = std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(event_solution.command_due_time_s)));

          // Predict the complete rigid body to this exact impact event, then
          // commit one physical face id.  The commitment is held for repeated
          // frames of the same event cycle and hands over only at the next
          // cycle (or if the prior face becomes geometrically impossible).
          if (!targets.empty()) {
            auto future_target = targets.front();
            const double processing_age_s = std::max(
              0.0, std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t).count());
            future_target.predict(
              processing_age_s + event_solution.time_until_hit_center_s);
            const auto future_x = future_target.ekf_x();
            if (future_x.size() >= 3) {
              const double future_center_yaw = std::atan2(future_x[2], future_x[0]);
              std::vector<auto_aim::GyroFaceSelector::Candidate> face_candidates;
              const auto future_faces = future_target.armor_xyza_list();
              face_candidates.reserve(future_faces.size());
              for (std::size_t face_id = 0; face_id < future_faces.size(); ++face_id) {
                face_candidates.push_back({
                  static_cast<int>(face_id), future_faces[face_id][3]});
              }
              const auto face_selection = gyro_face_selector.select(
                face_candidates, future_center_yaw, gyro_event_hit_cycle);
              gyro_event_face_valid = face_selection.valid;
              gyro_event_face_id = face_selection.face_id;
              gyro_event_face_alignment_rad = face_selection.alignment_rad;
              if (face_selection.valid) {
                // The temporal PLL interval and future-face alignment are
                // partially independent error sources.  Root-sum-square keeps
                // the face check meaningful without the excessive fire-rate
                // loss caused by linearly adding two conservative bounds.
                gyro_event_combined_phase_rad =
                  std::hypot(
                    event_solution.worst_interval_phase_rad,
                    face_selection.alignment_rad);
                if (xuc_event_require_face_consistency) {
                  gyro_event_interval_safe =
                    event_solution.worst_interval_phase_rad <= relaxed_half_window_rad &&
                    gyro_event_combined_phase_rad <= relaxed_half_window_rad;
                }
              }
              if (face_selection.valid && face_selection.changed) {
                tools::logger()->info(
                  "[FACELOCK] handoff cycle={} face={} alignment={:.2f}deg",
                  gyro_event_hit_cycle, gyro_event_face_id,
                  gyro_event_face_alignment_rad * 180.0 / M_PI);
              }
            }
          }

          gyro_event_gimbal_tracking_error_rad = std::abs(std::remainder(
            command.yaw - ypr[0], 2.0 * M_PI));

          static int event_schedule_log = 0;
          if (++event_schedule_log % 10 == 0) {
            tools::logger()->info(
              "[GYROSCHED] cycle={} due_in={:+.1f} ms hit_in={:.1f} ms "
              "period={:.1f} ms window={:.2f}deg worst={:.2f}deg "
              "combined={:.2f}deg "
              "phase_unc={:.2f}deg face={} face_align={:.2f}deg "
              "gimbal_err={:.2f}deg safe={}",
              gyro_event_hit_cycle, gyro_event_lead_error_s * 1000.0,
              event_solution.time_until_hit_center_s * 1000.0,
              gyro_event_face_period_s * 1000.0,
              gyro_event_half_window_rad * 180.0 / M_PI,
              gyro_event_worst_phase_rad * 180.0 / M_PI,
              gyro_event_combined_phase_rad * 180.0 / M_PI,
              gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
              gyro_event_face_id,
              gyro_event_face_alignment_rad * 180.0 / M_PI,
              gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
              gyro_event_interval_safe);
          }
        }
      }

      // A new physical face-center cycle re-arms a shot.  Outside AUTO the
      // estimator may continue learning the period, but fire state and cooldown
      // are never consumed.
      const bool auto_mode_active = mode == io::Mode::auto_aim;
      if (!auto_mode_active) {
        xuc_one_face_armed = true;
        event_last_fired_cycle = -1;
      } else if (one_face_policy_active) {
        if (gyro_event_schedule_valid) {
          // A PLL correction may move a scheduled event back to an already
          // consumed cycle.  Explicitly disarm in that case; merely rearming
          // on a different future cycle allowed duplicate requests in field
          // log cycles 41 and 146.
          xuc_one_face_armed = gyro_event_hit_cycle > event_last_fired_cycle;
        } else if (!gyro_event_schedule_valid &&
                   (!face_valid || face_error_rad >= xuc_one_face_rearm_rad)) {
          xuc_one_face_armed = true;
        }
      }
      if (!gyro_center_follow_active) xuc_one_face_armed = true;
      // 【相位门】只在"弹到瞬间最近装甲板压在视线上"才允许开火。仅 center_follow
      // （炮口指中心）且在旋转时生效；静止靶走直接测角分支，不适用。
      // 2026-10-18 靶车自转复测：稳定相位速率约 1.5 rad/s 时，瞬时 gyro_omega_rad_s
      // 会短暂跌破 0.30 rad/s，造成相位门旁路；门控阈值与弹着相位预测统一使用窗口速率。
      const bool impact_phase_gate_active =
        xuc_fire_phase_window_rad > 0.0 && gyro_center_follow_active &&
        std::abs(body_phase_rate_window) >= 0.30;
      const bool impact_phase_gate_open =
        !impact_phase_gate_active ||
        (std::isfinite(impact_phase_err_rad) &&
         impact_phase_err_rad <= xuc_fire_phase_window_rad);
      const bool one_face_gate_open = !one_face_policy_active ||
        (gyro_predicted_fire_active
           ? (gyro_event_schedule_valid
                ? xuc_one_face_armed
                : (!xuc_gyro_require_event_lock_for_fire && face_valid &&
                   face_error_rad <= xuc_one_face_limit_rad &&
                   xuc_one_face_armed))
           : (face_valid && face_error_rad <= xuc_one_face_limit_rad &&
              xuc_one_face_armed));

      // In event mode, issue only before the due instant and wait up to one
      // frame for a precise steady-clock send.  Never accept a late sample.
      const bool gyro_yaw_gate_native_open = gyro_predicted_impact_yaw_valid &&
        direct_yaw_measurement_fresh &&
        std::abs(gyro_predicted_impact_yaw_error_rad) <=
          xuc_gyro_predicted_yaw_tolerance_rad &&
        std::abs(direct_yaw_pixel_error_rad) <=
          xuc_gyro_visibility_yaw_tolerance_rad;
      const bool gyro_event_gate_open = gyro_event_schedule_valid &&
        gyro_phase_model_trusted && gyro_event_interval_safe &&
        (!xuc_event_require_face_consistency || gyro_event_face_valid) &&
        gyro_event_gimbal_tracking_error_rad <=
          xuc_event_gimbal_tracking_tolerance_rad &&
        gyro_event_lead_error_s >= 0.0 &&
        gyro_event_lead_error_s <= xuc_event_trigger_window_s;
      const bool yaw_gate_open = gyro_predicted_fire_active
        ? (gyro_event_schedule_valid
             ? gyro_event_gate_open
             : (!xuc_gyro_require_event_lock_for_fire &&
                gyro_yaw_gate_native_open))
        : (vehicle_predicted_fire_active
             ? gyro_yaw_gate_native_open
             : stationary_yaw_gate_open);
      const bool fire_candidate = command.control &&
        auto_mode_active &&
        convergence_gate_open && one_face_gate_open && motion_transition_gate_open &&
        feedback_cadence_gate_open &&
        impact_phase_gate_open &&
        direct_yaw_measurement_fresh && direct_pitch_measurement_fresh &&
        yaw_gate_open &&
        std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad;
      // 【误差拐点开火】(2026-10-19 借鉴 Fujian_AutoAim 26.10.18)
      // 用最近两帧的实测弹着偏航误差外推下一帧：next = e + (e - e_prev)。
      // 若当前误差不大于外推值（误差已到谷底、下一步会变差），或下一帧会超出容差，
      // 本帧就是这一窗口里最好的开火时刻。不建模型、不需要准确的 ω。
      const double pred_err_rad = gyro_predicted_impact_yaw_error_rad;
      double next_pred_err_rad = NAN;
      bool current_frame_is_better = false;
      if (predictive_fire_active && std::isfinite(pred_err_rad) &&
          std::isfinite(prev_pred_yaw_error_rad)) {
        next_pred_err_rad = std::remainder(
          pred_err_rad + std::remainder(pred_err_rad - prev_pred_yaw_error_rad, 2.0 * M_PI),
          2.0 * M_PI);
        current_frame_is_better =
          std::abs(pred_err_rad) <= std::abs(next_pred_err_rad) ||
          std::abs(next_pred_err_rad) > xuc_gyro_predicted_yaw_tolerance_rad;
      }
      prev_pred_yaw_error_rad =
        (predictive_fire_active && std::isfinite(pred_err_rad)) ? pred_err_rad : NAN;
      // Fire-gate diagnostic for both medium-speed prediction and the high-speed
      // event clock; low-speed dead zones were previously invisible here.
      static int b2_diag_log = 0;
      if (predictive_fire_active && ++b2_diag_log % 50 == 0) {
        tools::logger()->info(
          "[B2DIAG] auto={} control={} conv={} oneface_gate={} armed={} motion={} "
          "cadence={} dyaw_fresh={} dpitch_fresh={} yawgate={} event={} safe={} "
          "face={} face_ok={} phasegate={} phase_err_deg={:.2f} "
          "phase_rate={:+.2f} face_align_deg={:.2f} phase_unc_deg={:.2f} "
          "gimbal_err_deg={:.2f} due_ms={:+.1f} window_deg={:.2f} "
          "worst_deg={:.2f} combined_deg={:.2f} "
          "yawerr_deg={:.2f} pitcherr_deg={:.2f}",
          auto_mode_active ? 1 : 0, command.control ? 1 : 0,
          convergence_gate_open ? 1 : 0,
          one_face_gate_open ? 1 : 0, xuc_one_face_armed ? 1 : 0,
          motion_transition_gate_open ? 1 : 0,
          feedback_cadence_gate_open ? 1 : 0,
          direct_yaw_measurement_fresh ? 1 : 0,
          direct_pitch_measurement_fresh ? 1 : 0,
          yaw_gate_open ? 1 : 0, gyro_event_schedule_valid ? 1 : 0,
          gyro_event_interval_safe ? 1 : 0,
          gyro_event_face_id, gyro_event_face_valid ? 1 : 0,
          impact_phase_gate_open ? 1 : 0, impact_phase_err_rad * 180.0 / M_PI,
          body_phase_rate_window,
          gyro_event_face_alignment_rad * 180.0 / M_PI,
          gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
          gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
          gyro_event_lead_error_s * 1000.0,
          gyro_event_half_window_rad * 180.0 / M_PI,
          gyro_event_worst_phase_rad * 180.0 / M_PI,
          gyro_event_combined_phase_rad * 180.0 / M_PI,
          direct_yaw_pixel_error_rad * 180.0 / M_PI,
          direct_pitch_pixel_error_rad * 180.0 / M_PI);
      }
      if (fire_candidate) {
        const double candidate_face_yaw_rad = gyro_event_face_valid
          ? static_cast<double>(gyro_event_face_id) * M_PI / 2.0
          : aimer.debug_aim_point.xyza[3];
        if (predictive_fire_active && xuc_fire_candidate_active &&
            std::isfinite(xuc_fire_candidate_face_yaw_rad) &&
            std::isfinite(candidate_face_yaw_rad)) {
          const double phase_step_rad = std::abs(std::remainder(
            candidate_face_yaw_rad - xuc_fire_candidate_face_yaw_rad,
            2.0 * M_PI));
          if (phase_step_rad > xuc_gyro_candidate_max_phase_step_rad) {
            tools::logger()->info(
              "[XUC][FIRE] candidate face switched by {:.1f} deg; restarting window",
              phase_step_rad * 180.0 / M_PI);
            xuc_fire_candidate_active = false;
            xuc_fire_ready_logged = false;
            xuc_fire_candidate_frames = 0;
          }
        }
        if (!xuc_fire_candidate_active) {
          xuc_fire_candidate_active = true;
          xuc_fire_ready_logged = false;
          xuc_fire_candidate_frames = 1;
          xuc_fire_candidate_since = t;
          xuc_fire_candidate_face_yaw_rad = candidate_face_yaw_rad;
          tools::logger()->info("[XUC][FIRE] stable-window candidate started");
        } else {
          ++xuc_fire_candidate_frames;
          xuc_fire_candidate_face_yaw_rad = candidate_face_yaw_rad;
        }
        const double stable_time =
          std::chrono::duration<double>(t - xuc_fire_candidate_since).count();
        if (predictive_fire_active) {
          // Predictive fire always uses a one-frame pulse. High-speed phase mode
          // allows one pulse per physical face event; low/medium speed is
          // re-armed by completed feeder feedback, enabling controlled repeated
          // fire without turning a satisfied gate into a long shoot level.
          // Event mode: the dynamic-lead gate is a short time window (~1 frame
          // at high omega), so a single qualifying frame is sufficient to fire.
          // Native fallback (no event omega/tracker) keeps the 2-frame rule.
          const int ev_need_frames = gyro_event_schedule_valid
            ? 1
            : xuc_gyro_fire_min_consecutive_frames;
          command.shoot =
            (xuc_fire_candidate_frames >= ev_need_frames) && !xuc_fire_ready_logged &&
            current_frame_is_better;
        } else {
          const double required_stable_s = one_face_policy_active
            ? xuc_gyro_fire_stable_s
            : xuc_fire_stable_s;
          // XUC also receives a one-frame pulse for stationary targets. Once
          // feeder feedback closes this shot, a still-valid gate starts the
          // next pulse immediately; no frame-rate-level command flood and no
          // fixed one-second software divider are involved.
          command.shoot = stable_time >= required_stable_s &&
            (!xuc.enabled() || !xuc_fire_ready_logged);
        }
        if (command.shoot && !xuc_fire_ready_logged) {
          tools::logger()->info(
            "[XUC][FIRE] qualified after {:.0f} ms/{} frames, "
            "current_pixel_error_deg=({:.2f},{:.2f}) "
            "predicted_impact_yaw_error_deg={:.2f} gyro_predict={} "
            "face_error={:.2f}deg committed_face={} face_align={:.2f}deg "
            "phase_unc={:.2f}deg gimbal_err={:.2f}deg omega={:+.3f}rad_s armed={} "
            "phase_win={:.0f}deg phasegate_on={} phase_err={:.2f}deg "
            "phase_rate={:+.3f} next_err={:+.2f}deg better={} "
            "whole_ekf={} trace_face={}",
            stable_time * 1000.0,
            xuc_fire_candidate_frames,
            direct_yaw_pixel_error_rad * 180.0 / M_PI,
            direct_pitch_pixel_error_rad * 180.0 / M_PI,
            gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI,
            predictive_fire_active, face_error_rad * 180.0 / M_PI,
            gyro_event_face_id, gyro_event_face_alignment_rad * 180.0 / M_PI,
            gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
            gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
            gyro_omega_rad_s, xuc_one_face_armed,
            xuc_fire_phase_window_rad * 180.0 / M_PI, impact_phase_gate_active,
            impact_phase_err_rad * 180.0 / M_PI, body_phase_rate_window,
            next_pred_err_rad * 180.0 / M_PI, current_frame_is_better,
            native_fire_prediction.whole_ekf_geometry, native_fire_prediction.face_id);
          xuc_fire_ready_logged = true;
          if (one_face_policy_active) {
            xuc_one_face_armed = false;
            if (gyro_event_schedule_valid) {
              event_last_fired_cycle = gyro_event_hit_cycle;
            }
          }
        }
      } else {
        if (xuc_fire_candidate_active) {
          tools::logger()->info("[XUC][FIRE] stable window reset");
        }
        xuc_fire_candidate_active = false;
        xuc_fire_ready_logged = false;
        xuc_fire_candidate_frames = 0;
        xuc_fire_candidate_face_yaw_rad = INFINITY;
        command.shoot = false;
      }
    } else {
      command.shoot = native_shoot;
      if (measurement_fallback) command.shoot = false;
    }

    // 开火请求计数与显示：把"上位机决定开火"的次数与当前帧状态打到监控窗口。
    // 用途：试射时直接看到开火请求有没有发出去、发了多少次，以及被哪一级门控拦下。
    // 注意：这只是上位机的决策计数，不代表下位机真的拨弹（下位机 AUTOAIM 分支
    // 目前不驱动拨弹机构，且射手遥测未接入）。
    {
      static long shoot_request_count = 0;
      static long shoot_frames_total = 0;
      static auto last_shoot_print = std::chrono::steady_clock::now();
      ++shoot_frames_total;
      if (command.shoot) ++shoot_request_count;
      const auto now_print = std::chrono::steady_clock::now();
      if (!competition_headless &&
          std::chrono::duration<double>(now_print - last_shoot_print).count() >= 0.25) {
        last_shoot_print = now_print;
        fmt::print(
          "[SHOOT] 本帧 shoot={}  control={}  累计开火请求={}  被节奏门控拦下={}  帧={}  "
          "允许开火(配置)={} 稳定门(ms)={:.0f}  脉冲保持(ms)={:.0f}/{}  节奏={}\n",
          command.shoot ? 1 : 0, command.control ? 1 : 0, shoot_request_count,
          xuc_fire_inhibit_count, shoot_frames_total, xuc_allow_shoot_configured ? 1 : 0,
          xuc_fire_stable_s * 1000.0, xuc_shoot_hold_s * 1000.0,
          xuc_shoot_hold_pulse_count,
          xuc_fire_cadence_fallback_s > 0.0 ? fmt::format("定时 {:.0f}ms", xuc_fire_cadence_fallback_s * 1000.0)
                                : std::string("遥测"));
      }
    }

    // Final actuator tracking layer.  Form the error before adding PI output so
    // the integrator converges to the extra command needed to make actual raw IMU
    // yaw follow the ESKF/direct-guard reference instead of chasing its own
    // boosted output.  Reset on loss/exit to prevent stale wind-up at reacquire.
    double yaw_tracking_error_raw_rad = NAN;
    double yaw_tracking_output_raw_rad = 0.0;
    if (!lower_yaw_control_ready) {
      // The vision pipeline continues producing targets while the RC switch is
      // outside AUTO.  Never integrate that physically untrackable error and
      // then dump it into the gimbal on the AUTO edge.
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      yaw_tracking_lower_armed = false;
    } else if (!yaw_tracking_lower_armed) {
      yaw_tracking_lower_armed = true;
      yaw_tracking_armed_since = std::chrono::steady_clock::now();
      yaw_tracking_last_update = yaw_tracking_armed_since;
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = true;
    }

    if (yaw_tracking_pi_enabled && lower_yaw_control_ready &&
        yaw_tracking_lower_armed && xuc.rx_valid() && command.control &&
        std::isfinite(command.yaw)) {
      const auto pi_now = std::chrono::steady_clock::now();
      const double armed_age_s = std::chrono::duration<double>(
        pi_now - yaw_tracking_armed_since).count();
      const double dt = yaw_tracking_initialized &&
          armed_age_s >= yaw_tracking_arm_dwell_s
        ? std::clamp(
            std::chrono::duration<double>(pi_now - yaw_tracking_last_update).count(),
            0.0,
            0.05)
        : 0.0;
      yaw_tracking_last_update = pi_now;
      yaw_tracking_initialized = true;

      const double reference_raw_rad = xuc.command_yaw_to_raw(command.yaw);
      const double feedback_raw_rad = xuc.raw_imu_yaw();
      yaw_tracking_error_raw_rad = std::remainder(
        reference_raw_rad - feedback_raw_rad, 2.0 * M_PI);
      const bool low_dynamic_measurement = direct_yaw_measurement_fresh &&
        tracker_converged &&
        (!gyro_model_fresh || std::abs(gyro_omega_rad_s) <
          xuc_gyro_omega_threshold_rad_s);
      if (armed_age_s >= yaw_tracking_arm_dwell_s &&
          low_dynamic_measurement &&
          std::isfinite(yaw_tracking_error_raw_rad) &&
          std::abs(yaw_tracking_error_raw_rad) <=
            yaw_tracking_integrate_error_limit_rad) {
        yaw_tracking_integral_raw_rad = std::clamp(
          yaw_tracking_integral_raw_rad +
            yaw_tracking_ki_per_s * yaw_tracking_error_raw_rad * dt,
          -yaw_tracking_integral_limit_rad,
          yaw_tracking_integral_limit_rad);
      }
      if (armed_age_s >= yaw_tracking_arm_dwell_s) {
        // Proportional action is permitted only on a fresh, low-dynamic
        // measurement.  During gyro motion retain at most the slowly learned
        // static trim; ESKF/phase prediction remains the sole dynamic lead.
        const double proportional_raw_rad = low_dynamic_measurement
          ? yaw_tracking_kp * yaw_tracking_error_raw_rad
          : 0.0;
        yaw_tracking_output_raw_rad = std::clamp(
          proportional_raw_rad + yaw_tracking_integral_raw_rad,
          -yaw_tracking_output_limit_rad,
          yaw_tracking_output_limit_rad);
      }
      command.yaw = std::remainder(
        command.yaw +
          xuc.raw_yaw_correction_to_command(yaw_tracking_output_raw_rad),
        2.0 * M_PI);
    } else {
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      if (!lower_yaw_control_ready) yaw_tracking_lower_armed = false;
    }

    // Dry-run diagnostics: world-coordinate stability and final command.
    static int dryrun_log_counter = 0;
    if (++dryrun_log_counter % 20 == 0) {
      constexpr double RAD2DEG = 180.0 / M_PI;
      if (!targets.empty()) {
        auto tracked_xyza_list = targets.front().armor_xyza_list();
        if (!tracked_xyza_list.empty() && direct_measurement_armor != nullptr) {
          const auto & measured_gimbal = direct_measurement_armor->xyz_in_gimbal;
          const auto & measured_world = direct_measurement_armor->xyz_in_world;
          const auto & tracked = tracked_xyza_list.front();
          double armor_center_y_px = direct_measurement_armor->center.y;
          if (!direct_measurement_armor->points.empty()) {
            armor_center_y_px = 0.0;
            for (const auto & point : direct_measurement_armor->points) armor_center_y_px += point.y;
            armor_center_y_px /= static_cast<double>(direct_measurement_armor->points.size());
          }
          const double pixel_pitch_error_deg = direct_pitch_focal_y_px > 0.0
            ? std::atan2(
                armor_center_y_px - 0.5 * static_cast<double>(img.rows),
                direct_pitch_focal_y_px) * RAD2DEG
            : 0.0;

          const double target_gimbal_yaw_deg =
            std::atan2(measured_gimbal.y(), measured_gimbal.x()) * RAD2DEG;
          const double target_gimbal_pitch_deg =
            std::atan2(
              measured_gimbal.z(),
              std::hypot(measured_gimbal.x(), measured_gimbal.y())) * RAD2DEG;
          const double target_world_yaw_deg =
            std::atan2(measured_world.y(), measured_world.x()) * RAD2DEG;
          const double target_world_pitch_deg =
            std::atan2(
              measured_world.z(),
              std::hypot(measured_world.x(), measured_world.y())) * RAD2DEG;

          tools::logger()->info(
            "[DRYRUN] measured_gimbal=({:.3f},{:.3f},{:.3f}) "
            "measured_world=({:.3f},{:.3f},{:.3f}) "
            "tracked_world=({:.3f},{:.3f},{:.3f}) "
            "target_gimbal_deg=({:.2f},{:.2f}) "
            "target_world_deg=({:.2f},{:.2f}) "
            "gimbal_deg=({:.2f},{:.2f}) "
            "armor_y_px={:.1f} pixel_error_deg=({:.2f},{:.2f}) "
            "ballistic=(range={:.3f}m holdover={:.3f}m comp={:.2f}deg residual={:.2f}deg) "
            "center_lead={:.2f}deg center_follow={} "
            "q_wxyz=({:.6f},{:.6f},{:.6f},{:.6f}) "
            "cmd_deg=({:.2f},{:.2f}) packed_pitch_deg={:.2f} "
            "yaw_pi=(error={:.2f}deg integral={:.2f}deg output={:.2f}deg) "
            "control={} shoot={}",
            measured_gimbal.x(), measured_gimbal.y(), measured_gimbal.z(),
            measured_world.x(), measured_world.y(), measured_world.z(),
            tracked.x(), tracked.y(), tracked.z(),
            target_gimbal_yaw_deg, target_gimbal_pitch_deg,
            target_world_yaw_deg, target_world_pitch_deg,
            ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
            armor_center_y_px, direct_yaw_pixel_error_rad * RAD2DEG, pixel_pitch_error_deg,
            ballistic_camera_range_m, ballistic_total_holdover_m,
            ballistic_pitch_compensation_rad * RAD2DEG,
            direct_pitch_pixel_error_rad * RAD2DEG,
            center_command_lead_rad * RAD2DEG, gyro_center_follow_active,
            q.w(), q.x(), q.y(), q.z(),
            command.yaw * RAD2DEG, command.pitch * RAD2DEG,
            xuc_pitch_sign_for_log * command.pitch * RAD2DEG,
            yaw_tracking_error_raw_rad * RAD2DEG,
            yaw_tracking_integral_raw_rad * RAD2DEG,
            yaw_tracking_output_raw_rad * RAD2DEG,
            command.control, command.shoot);
        }
      } else {
        tools::logger()->info(
          "[DRYRUN] target=none gimbal_deg=({:.2f},{:.2f}) control={} shoot={}",
          ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
          command.control, command.shoot);
      }
    }

    // For an event-qualified gyro shot, wait for the exact steady-clock due
    // instant instead of quantizing the request to the next camera frame.  A
    // sample that was already late before this deliberate wait is discarded;
    // the small tolerance only covers OS wake-up jitter after sleep_until().
    double gyro_event_send_lateness_ms = NAN;
    if (xuc.enabled() && command.shoot) {
      if (xuc.mode() != static_cast<uint8_t>(io::Mode::auto_aim)) {
        tools::logger()->warn(
          "[SHOT] request suppressed because lower-board mode={} is not AUTO",
          static_cast<int>(xuc.mode()));
        command.shoot = false;
      } else if (gyro_event_schedule_valid) {
        const auto wait_started = std::chrono::steady_clock::now();
        if (wait_started > gyro_event_command_due_time) {
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            wait_started - gyro_event_command_due_time).count();
          tools::logger()->warn(
            "[GYROSCHED] cycle={} missed before send by {:.2f} ms; shot discarded",
            gyro_event_hit_cycle, gyro_event_send_lateness_ms);
          command.shoot = false;
        } else {
          std::this_thread::sleep_until(gyro_event_command_due_time);
          const auto woke_at = std::chrono::steady_clock::now();
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            woke_at - gyro_event_command_due_time).count();
          if (gyro_event_send_lateness_ms >
              xuc_event_send_late_tolerance_s * 1000.0) {
            tools::logger()->warn(
              "[GYROSCHED] cycle={} wake late by {:.2f} ms; shot discarded",
              gyro_event_hit_cycle, gyro_event_send_lateness_ms);
            command.shoot = false;
          }
        }
      }
    }

    // Attach a software shot id to every pulse actually sent.  The next feeder
    // rising edge closes the command-to-feeder timing measurement; it is not
    // treated as projectile muzzle time.
    bool shot_rising_edge_this_frame = false;
    std::chrono::steady_clock::time_point shot_diag_issue_time{};
    if (xuc.enabled() && command.shoot) {
      const auto issue_time = std::chrono::steady_clock::now();
      shot_diag_issue_time = issue_time;
      const uint64_t shot_id = xuc_next_shot_id++;
      shot_rising_edge_this_frame = true;
      if (xuc_pending_shot) {
        tools::logger()->warn(
          "[SHOT] id={} superseded_without_feedback by={}",
          xuc_pending_shot_id, shot_id);
      }
      xuc_pending_shot = true;
      xuc_pending_shot_id = shot_id;
      xuc_pending_shot_time = issue_time;
      // 武装开火脉冲保持：本发 shoot 将维持到该时刻
      if (xuc_shoot_hold_s > 0.0) {
        xuc_shoot_pulse_until = issue_time + std::chrono::milliseconds(
          static_cast<int>(xuc_shoot_hold_s * 1000.0 + 0.5));
        xuc_shoot_pulse_active = true;
        ++xuc_shoot_hold_pulse_count;
      }
      const double processing_age_ms =
        std::chrono::duration<double, std::milli>(issue_time - t).count();
      tools::logger()->info(
        "[SHOT] id={} issue mode={} gyro={} event={} cycle={} "
        "processing_age_ms={:.1f} due_error_ms={:+.2f} "
        "period_ms={:.1f} window_deg={:.2f} worst_deg={:.2f} combined_deg={:.2f} "
        "face={} face_align_deg={:.2f} phase_unc_deg={:.2f} gimbal_err_deg={:.2f} "
        "phase_signed_deg={:+.2f} phase_rate_rad_s={:+.3f} phase_horizon_ms={:.1f} "
        "body_phase_deg={:+.2f} center_yaw_deg={:+.2f} "
        "cmd_yaw_deg={:+.2f} gimbal_yaw_deg={:+.2f} "
        "cmd_pitch_deg={:+.2f} gimbal_pitch_deg={:+.2f} "
        "range_m={:.3f} projectile_speed_mps={:.2f}",
        shot_id, static_cast<int>(xuc.mode()), gyro_model_fresh ? 1 : 0,
        gyro_event_schedule_valid ? 1 : 0, gyro_event_hit_cycle,
        processing_age_ms, gyro_event_send_lateness_ms,
        gyro_event_face_period_s * 1000.0,
        gyro_event_half_window_rad * 180.0 / M_PI,
        gyro_event_worst_phase_rad * 180.0 / M_PI,
        gyro_event_combined_phase_rad * 180.0 / M_PI,
        gyro_event_face_id,
        gyro_event_face_alignment_rad * 180.0 / M_PI,
        gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
        gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
        shot_diag_signed_phase_rad * 180.0 / M_PI, body_phase_rate_window,
        shot_diag_phase_horizon_s * 1000.0,
        gyro_body_phase_rad * 180.0 / M_PI, gyro_center_yaw * 180.0 / M_PI,
        command.yaw * 180.0 / M_PI, ypr[0] * 180.0 / M_PI,
        command.pitch * 180.0 / M_PI, ypr[1] * 180.0 / M_PI,
        robust_target_range_m, xuc_projectile_speed_mps);
    }

    // 发送命令到 PlotJuggler（显示实际发送的值）
    nlohmann::json plot_data;
    plot_data["t"] = std::chrono::duration<double>(t - std::chrono::steady_clock::time_point()).count();
    plot_data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
    plot_data["cmd_pitch"] =
      xuc_pitch_sign_for_log * command.pitch * 180.0 / M_PI;
    plot_data["yaw_tracking_error_raw_deg"] = std::isfinite(yaw_tracking_error_raw_rad)
      ? nlohmann::json(yaw_tracking_error_raw_rad * 180.0 / M_PI)
      : nlohmann::json(nullptr);
    plot_data["yaw_tracking_integral_raw_deg"] =
      yaw_tracking_integral_raw_rad * 180.0 / M_PI;
    plot_data["yaw_tracking_output_raw_deg"] =
      yaw_tracking_output_raw_rad * 180.0 / M_PI;
    plot_data["control"] = command.control;
    plot_data["shoot"] = command.shoot;
    if (gyro_predicted_impact_yaw_valid) {
      plot_data["gyro_predicted_impact_yaw_error_deg"] =
        gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_predicted_impact_yaw_error_deg"] = nullptr;
    }
    if (gyro_event_schedule_valid) {
      plot_data["gyro_event_due_ms"] = gyro_event_lead_error_s * 1000.0;
      plot_data["gyro_event_period_ms"] = gyro_event_face_period_s * 1000.0;
      plot_data["gyro_event_interval_safe"] = gyro_event_interval_safe;
      plot_data["gyro_event_half_window_deg"] =
        gyro_event_half_window_rad * 180.0 / M_PI;
      plot_data["gyro_event_worst_phase_deg"] =
        gyro_event_worst_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_combined_phase_deg"] =
        gyro_event_combined_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_phase_uncertainty_deg"] =
        gyro_event_phase_uncertainty_rad * 180.0 / M_PI;
      plot_data["gyro_event_face_id"] = gyro_event_face_id;
      plot_data["gyro_event_face_alignment_deg"] =
        gyro_event_face_alignment_rad * 180.0 / M_PI;
      plot_data["gyro_event_gimbal_tracking_error_deg"] =
        gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_event_due_ms"] = nullptr;
      plot_data["gyro_event_period_ms"] = nullptr;
      plot_data["gyro_event_interval_safe"] = false;
      plot_data["gyro_event_half_window_deg"] = nullptr;
      plot_data["gyro_event_worst_phase_deg"] = nullptr;
      plot_data["gyro_event_combined_phase_deg"] = nullptr;
      plot_data["gyro_event_phase_uncertainty_deg"] = nullptr;
      plot_data["gyro_event_face_id"] = nullptr;
      plot_data["gyro_event_face_alignment_deg"] = nullptr;
      plot_data["gyro_event_gimbal_tracking_error_deg"] = nullptr;
    }
    // 电控的欧拉角（从MCU获取的姿态）
    plot_data["mcu_yaw"] = ypr[0] * 180.0 / M_PI;
    plot_data["mcu_pitch"] = ypr[1] * 180.0 / M_PI;
    plot_data["mcu_roll"] = ypr[2] * 180.0 / M_PI;
    // 电控的四元数（从MCU获取）
    plot_data["mcu_q_w"] = q.w();
    plot_data["mcu_q_x"] = q.x();
    plot_data["mcu_q_y"] = q.y();
    plot_data["mcu_q_z"] = q.z();
    plotter.plot(plot_data);

    // 🎯 关键：将 send 移到循环开头
    // 这样下一次循环时，会先发送这个命令，然后才读取相机
    // 形成闭环：发送命令 → 电控触发相机 → 读取图像 → 处理 → 发送下一条命令
    if (xuc.enabled()) {
      // 【诊断模式】持续 shoot：SPV_SHOOT_HOLD_MS 指定时长内无视算法决策，强制
      // shoot=1 连续下发。用途：电控反馈“shoot 只有一帧、持续极短，拨弹轮没动”，
      // 需要区分“指令太短”还是“下位机不执行”。
      // 用法：SPV_SHOOT_HOLD_MS=3000 ./build/standard -c=configs/standard.yaml
      // 安全：仍要求 xuc.allow_shoot（驱动内部开关）与 control=1，且只在持续时间
      // 内生效；结束时自动恢复算法决策。仅诊断用，验收后应删除。
      static const int shoot_hold_ms = [] {
        const char * env = std::getenv("SPV_SHOOT_HOLD_MS");
        return env ? std::clamp(std::atoi(env), 0, 30000) : 0;
      }();
      static const auto shoot_hold_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(shoot_hold_ms);
      const bool shoot_hold_active =
        shoot_hold_ms > 0 &&
        std::chrono::steady_clock::now() < shoot_hold_deadline &&
        command.control && xuc_allow_shoot_configured;
      if (shoot_hold_active) {
        static bool hold_logged = false;
        if (!hold_logged) {
          tools::logger()->warn(
            "[SHOOT-HOLD] 持续 shoot 诊断模式启动：{} ms 内强制 shoot=1（要求 control=1）",
            shoot_hold_ms);
          hold_logged = true;
        }
        command.shoot = true;
        ++xuc_shoot_hold_frames;
        static auto last_hold_print = std::chrono::steady_clock::now();
        const auto hold_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(hold_now - last_hold_print).count() >= 0.5) {
          last_hold_print = hold_now;
          fmt::print(
            "[SHOOT-HOLD] 持续下发中：帧={} 剩余={:.1f}s\n", xuc_shoot_hold_frames,
            std::chrono::duration<double>(shoot_hold_deadline - hold_now).count());
        }
      } else if (shoot_hold_ms > 0 && xuc_shoot_hold_frames > 0 &&
                 !xuc_shoot_hold_ended_logged) {
        tools::logger()->warn(
          "[SHOOT-HOLD] 持续 shoot 结束：共下发 {} 帧（{} ms）", xuc_shoot_hold_frames,
          shoot_hold_ms);
        xuc_shoot_hold_ended_logged = true;
      }
      // 2026-10-14 新增：开火脉冲保持。把上升沿的单帧 shoot 延长到配置时长，
      // 使下位机拨弹轮（AUTO 档速度模式）有足够且确定的时间转过一个弹位。
      // 安全：仍要求 control=1 与配置允许开火；退出 AUTO 或链路失效自动结束。
      if (xuc_shoot_hold_s > 0.0 && xuc_shoot_pulse_active) {
        if (std::chrono::steady_clock::now() < xuc_shoot_pulse_until &&
            command.control && xuc_allow_shoot_configured) {
          command.shoot = true;
        } else {
          xuc_shoot_pulse_active = false;
        }
      }
      xuc.send(command);
      // 节奏时刻只在**上升沿**更新：保持期间的续帧不得刷新它，
      // 否则 cadence 门控会被无限推迟（实测会退化成死锁）。
      if (shot_rising_edge_this_frame) {
        xuc_last_shot_issue_time = std::chrono::steady_clock::now();
        // This is a host serial-write-call timestamp, not physical muzzle time.
        const int64_t tx_call_ns = xuc.last_shoot_tx_ns();
        const int64_t issue_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          shot_diag_issue_time.time_since_epoch()).count();
        const int64_t image_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        const bool tx_call_matches = tx_call_ns >= issue_ns;
        tools::logger()->info(
          "[SHOTTX] id={} tx_call_matches={} issue_to_tx_call_ms={:.3f} "
          "image_to_tx_call_ms={:.3f}",
          xuc_pending_shot_id, tx_call_matches ? 1 : 0,
          tx_call_matches ? (tx_call_ns - issue_ns) / 1e6 : NAN,
          tx_call_matches ? (tx_call_ns - image_ns) / 1e6 : NAN);
      }
      // pending is already registered before send above. Feedback cadence
      // additionally replaces its issue-time origin with the write-call time.
      if (shot_rising_edge_this_frame && !shoot_hold_active &&
          xuc_fire_cadence_fallback_s <= 0.0) {
        const int64_t tx_ns = xuc.last_shoot_tx_ns();
        if (tx_ns > 0) {
          xuc_pending_shot_time = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(tx_ns));
        }
      }
    } else {
      cboard.send(command);
    }

    if (!competition_headless) {
      // 绘制识别与预测结果
      if (!targets.empty() && tracker_converged && yaw_motion_model_trusted) {
        auto target = targets.front();

        // 绘制跟踪到的所有装甲板位置（绿色）
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 255, 0});
        }

        // 绘制预测的瞄准位置（红色）
        if (aimer.debug_aim_point.valid) {
          Eigen::Vector4d aim_xyza = aimer.debug_aim_point.xyza;
          auto image_points =
            solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 0, 255});
        }
      }

      cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      // 相机输出为 BGR，imshow 直接可用
      cv::imshow("reprojection", img);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    }

    // 🔍 性能监控：打印各环节耗时
    auto loop_end = std::chrono::steady_clock::now();
    static int perf_counter = 0;
    if (++perf_counter % 50 == 0) {
      auto detect_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      auto track_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
      auto total_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
      tools::logger()->info("[PERF] 总耗时={:.1f}ms (检测={:.1f}ms, 跟踪={:.1f}ms), FPS={:.1f}",
        total_ms, detect_ms, track_ms, 1000.0 / total_ms);
    }
  }

  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  visualizer.reset();
  rclcpp::shutdown();
  tools::logger()->info("[ROS2] Shutdown complete");
#endif

  return 0;
}
