// XUC虚拟下位机双向链路测试
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <filesystem>

#include "io/command.hpp"
#include "io/xuc.hpp"

static bool near(double value, double expected, double tolerance)
{
  return std::fabs(value - expected) <= tolerance;
}

// Virtual-port only: the emulator supplies stationary, moving, yaw-wrap,
// interrupted, and recovered phases. Never transmits motor/shoot commands.
static int test_pose_history(io::XucSender & xuc)
{
  using namespace std::chrono;
  bool stationary = false, moving = false, wrapped = true, missing = false, recovered = false;
  bool paired = true, bounded_bracket = true;
  int checked = 0, wrap_checked = 0;
  const auto start = steady_clock::now();
  while (steady_clock::now() - start < milliseconds(3300)) {
    const auto now = steady_clock::now();
    const double elapsed = duration<double>(now - start).count();
    const auto target = now - milliseconds(20);
    io::XucSender::ImuPose pose;
    const bool valid = xuc.imu_at(target, pose);
    const bool stable = xuc.imu_stable_at(target, milliseconds(200), .001);
    if (elapsed > .4 && elapsed < .65 && valid && stable) stationary = true;
    if (elapsed > .9 && elapsed < 1.15 && valid) {
      ++checked;
      bounded_bracket = bounded_bracket && pose.timestamp == target && pose.bracket_ms > 0 && pose.bracket_ms <= 30;
      paired = paired && near(pose.pitch, .02 - .2 * (pose.yaw - .01), 1e-6);
      moving = moving || !stable;
    }
    if (elapsed > 1.4 && elapsed < 1.55 && valid) {
      ++wrap_checked;
      wrapped = wrapped && std::abs(std::remainder(pose.yaw - .01, 2 * M_PI)) > 3.1;
    }
    if (elapsed > 1.9 && elapsed < 2.1 && !valid && !stable) missing = true;
    if (elapsed > 2.8 && valid && stable) recovered = true;
    std::this_thread::sleep_for(milliseconds(10));
  }
  io::XucSender::ImuPose unused;
  const bool outside = !xuc.imu_at(start - seconds(5), unused) &&
                       !xuc.imu_at(steady_clock::now() + seconds(5), unused);
  const bool passed = stationary && moving && wrapped && missing && recovered &&
                      checked > 5 && wrap_checked > 5 && paired && bounded_bracket && outside;
  std::printf("POSE_HISTORY stationary=%d moving=%d wrap=%d gap=%d recovery=%d pair=%d bracket=%d outside=%d samples=%d\n",
    stationary, moving, wrapped, missing, recovered, paired, bounded_bracket, outside, checked);
  std::printf("POSE_HISTORY_RESULT=%s\n", passed ? "PASS" : "FAIL");
  return passed ? 0 : 3;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf("用法: %s /dev/pts/N [--safety|--pose-history] [--imu-unit=deg|rad]\n", argv[0]);
    return 1;
  }

  const std::string port = argv[1];
  if (port.rfind("/dev/pts/", 0) != 0 || !std::filesystem::is_character_file(port)) {
    std::printf("[FAIL] This test requires a virtual /dev/pts/* port\n");
    return 1;
  }
  bool safety_profile = false;
  bool pose_history = false;
  std::string imu_angle_unit = "rad";
  for (int i = 2; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--safety") safety_profile = true;
    else if (option == "--pose-history") pose_history = true;
    else if (option == "--imu-unit=deg") imu_angle_unit = "deg";
    else if (option == "--imu-unit=rad") imu_angle_unit = "rad";
    else return 1;
  }
  const std::string cfg_path = "/tmp/xuc_link_test.yaml";

  {
    std::ofstream config(cfg_path);
    config
      << "xuc_enable: true\n"
      << "xuc_serial_port: \"" << port << "\"\n"
      << "xuc_serial_baud: 460800\n"
      << "xuc_imu_angle_unit: " << imu_angle_unit << "\n"
      << "xuc_imu_yaw_sign: " << (pose_history ? "-1" : "1") << "\n"
      << "xuc_imu_yaw_offset_rad: " << (pose_history ? "0.01" : "0") << "\n"
      << "xuc_imu_pitch_offset_rad: " << (pose_history ? "0.02" : "0") << "\n"
      << "xuc_yaw_sign: 1\n"
      << "xuc_pitch_sign: -1\n"
      << "xuc_allow_control: " << (pose_history ? "false" : "true") << "\n"
      << "xuc_allow_shoot: " << (safety_profile || pose_history ? "false" : "true") << "\n"
      << "xuc_require_feedback_for_control: "
      << (safety_profile ? "true" : "false") << "\n"
      << "xuc_require_auto_mode_for_control: "
      << (safety_profile ? "true" : "false") << "\n"
      << "xuc_max_yaw_excursion_rad: "
      << (safety_profile ? "0.05" : "0.0") << "\n"
      << "xuc_lock_pitch_for_control: "
      << (safety_profile ? "true" : "false") << "\n";
  }
  std::printf("TEST_PROFILE=%s\n", safety_profile ? "SAFETY" : "PROTOCOL");

  io::XucSender xuc(cfg_path);

  if (!xuc.enabled()) {
    std::printf("[FAIL] XUC串口无法打开\n");
    std::remove(cfg_path.c_str());
    return 2;
  }
  if (pose_history) {
    const int code = test_pose_history(xuc);
    std::remove(cfg_path.c_str());
    return code;
  }

  io::Command command{};
  command.control = true;
  command.shoot = true;
  command.yaw = 0.30;
  command.pitch = -0.15;

  bool accepted_bad_crc = false;
  bool saw_initial_valid = false;
  bool saw_timeout_invalid = false;
  bool saw_recovery_valid = false;
  bool fields_valid = true;

  const auto start = std::chrono::steady_clock::now();

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - start).count();

    if (elapsed_ms >= 4000) break;

    // Exercise a brief detector/tracker dropout while AUTO feedback remains
    // active.  A bounded session must keep its original yaw anchor.
    command.control =
      !(safety_profile && elapsed_ms >= 1150 && elapsed_ms < 1300);
    xuc.send(command);

    const bool valid = xuc.rx_valid();

    if (elapsed_ms < 600 && valid) {
      accepted_bad_crc = true;
    }

    if (elapsed_ms >= 900 &&
        elapsed_ms < 1450 &&
        valid) {
      saw_initial_valid = true;
    }

    if (elapsed_ms >= 2100 &&
        elapsed_ms < 2350 &&
        !valid) {
      saw_timeout_invalid = true;
    }

    if (elapsed_ms >= 2700 && valid) {
      saw_recovery_valid = true;
    }

    if (valid) {
      fields_valid =
        fields_valid &&
        (safety_profile ? xuc.mode() <= 1 : xuc.mode() == 1) &&
        xuc.robot_id() == 3 &&
        near(xuc.bullet_speed(), 14.5, 0.01) &&
        xuc.bullet_count() > 0 &&
        near(xuc.imu_pitch(), 0.10, 0.001) &&
        (near(xuc.raw_imu_yaw(), 0.50, 0.001) ||
         near(xuc.raw_imu_yaw(), 0.55, 0.001)) &&
        (near(xuc.imu_yaw(), 0.50, 0.001) ||
         near(xuc.imu_yaw(), 0.55, 0.001));
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(50));
  }

  auto result = [](bool ok, const char * name) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  };

  result(!accepted_bad_crc, "错误CRC反馈未被接受");
  result(saw_initial_valid, "正常反馈能够被解析");
  result(fields_valid, "反馈字段和IMU数值正确");
  result(saw_timeout_invalid, "反馈中断500ms后状态失效");
  result(saw_recovery_valid, "反馈恢复后链路重新有效");
  result(xuc.rx_bytes() > 0, "接收字节计数有效");
  result(xuc.crc_errors() > 0, "错误CRC已计数");
  result(xuc.valid_packets() > 0, "合法反馈包已计数");

  const bool passed =
    !accepted_bad_crc &&
    saw_initial_valid &&
    fields_valid &&
    saw_timeout_invalid &&
    saw_recovery_valid &&
    xuc.rx_bytes() > 0 &&
    xuc.crc_errors() > 0 &&
    xuc.valid_packets() > 0;

  std::printf(
    "XUC_LINK_RESULT=%s\n",
    passed ? "PASS" : "FAIL");

  std::remove(cfg_path.c_str());
  return passed ? 0 : 3;
}
