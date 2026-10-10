// XUC 上位机→下位机 电机目标角 测试探针（对应电控仓库 zj78848108/new_infantry）。
//
// 这块板子的契约（已按电控仓库源码逐行核对）
// ------------------------------------------
//   * 拨杆 (左 DOWN, 右 DOWN) → ctrl.mode = AUTOAIM，反馈 mode=1；只有这一档 XUC 生效。
//   * AUTOAIM 分支（RC.cpp）: if (xuc.control_enable && xuc.Fresh()) SetTargetPos(xuc.target_yaw / target_pitch)
//     ⇒ 帧里的 yaw/pitch 是**达妙电机位置目标（弧度）**，不是 IMU 角、不是世界系角。
//   * 电机限幅: YAW ∈ [-3.14, +3.14] rad；PITCH ∈ [-0.38, +0.34] rad。
//   * 执行: setPos 以 0.01 rad / 2 ms = 5 rad/s 逼近 targetPos。
//   * 刷新窗口: Fresh() 默认 100 ms ⇒ 必须以 ≥10 Hz 持续发送。
//   * 失联（control=0 或超时）: AUTOAIM 分支什么都不做，电机保持最后目标。
//   * 反馈 mode = (ctrl.mode==AUTOAIM)?1:0；robot_id = 真实机器人 ID（裁判系统未接=0）；
//     bullet_speed/bullet_count 恒为 0（该工程未接发射遥测）；imu_* 是 CH010 角度（度）。
//
// 安全设计
// --------
//   * 默认 dry-run：只打印与只读基线，绝不置 control=1。
//   * --execute 才真正下发；序列固定为  基线 → 回零(park) → 小幅阶跃 → 回零 → 撤控，
//     结束时**必定**停在 0 位（电机零点，也是本固件上电默认位）。
//   * 阶跃有硬上限 --step-rad ≤ 0.20 rad（≈11.5°）。
//   * 全程监测：mode 掉 1、反馈持续失效 → 立即撤控退出。
//   * 本工具不下发 shoot；临时配置里 xuc_allow_shoot 恒为 false，正式配置不被修改。
//
// ⚠ 已知风险（必须现场确认）
// --------------------------
//   下位机反馈里**没有电机当前位置**。若云台曾被遥控器在其它模式下移动过，targetPos
//   可能已经偏离 0，那么"回零"这一步会以最高 5 rad/s（≈286°/s）冲向零点。
//   因此执行前必须：云台活动范围清空、线缆留足余量、人离开、随时能拨回安全档。
//   另外 AUTOAIM 与 FIRE 只差右拨杆一档（左 DOWN 时 右 UP=FIRE 会让摩擦轮起转）。
//
// 用法（小电脑）：
//   # 1) 只读：确认 mode=1 与基线
//   ./build/xuc_motion_probe -c=configs/standard.yaml
//   # 2) 真正下发（现场确认清场后）
//   ./build/xuc_motion_probe -c=configs/standard.yaml --execute --step-rad=0.05

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "io/xuc.hpp"

namespace
{
constexpr double RAD2DEG = 57.29577951308232;
// 该固件的电机限幅
constexpr double YAW_MIN = -3.14, YAW_MAX = 3.14;
constexpr double PITCH_MIN = -0.38, PITCH_MAX = 0.34;
// 工具自身更保守的阶跃上限
constexpr double MAX_STEP_RAD = 0.20;

struct Options
{
  std::string config = "configs/standard.yaml";
  double step_rad = 0.05;   // 默认 ≈2.9°
  double step_seconds = 2.0;
  double settle_seconds = 1.5;
  std::string axis = "yaw";  // yaw | pitch
  bool execute = false;
  bool help = false;
  // 单目标模式：直接把 yaw/pitch 当电机位置目标下发（度，便于现场口述）
  bool single_target = false;
  double target_yaw_deg = 0.0;
  double target_pitch_deg = 0.0;
  double hold_seconds = 2.0;
  bool park_after = false;
};

struct Sample
{
  double imu_yaw_deg = 0, imu_pitch_deg = 0;
  int count = 0;
  void add(double y, double p)
  {
    imu_yaw_deg += y;
    imu_pitch_deg += p;
    ++count;
  }
  double yaw() const { return count ? imu_yaw_deg / count : 0.0; }
  double pitch() const { return count ? imu_pitch_deg / count : 0.0; }
  std::string str() const
  {
    return count ? fmt::format("yaw {:+.3f}°  pitch {:+.3f}°  ({} 样本)", yaw(), pitch(), count)
                 : std::string("无样本");
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto after = [&](const char * p) {
      return a.rfind(p, 0) == 0 ? a.substr(std::strlen(p)) : std::string();
    };
    if (a == "--execute") opt.execute = true;
    else if (a == "--park-after") opt.park_after = true;
    else if (a == "--help" || a == "-h") opt.help = true;
    else if (!after("--target-yaw-deg=").empty()) {
      opt.target_yaw_deg = std::stod(after("--target-yaw-deg="));
      opt.single_target = true;
    } else if (!after("--target-pitch-deg=").empty()) {
      opt.target_pitch_deg = std::stod(after("--target-pitch-deg="));
      opt.single_target = true;
    } else if (!after("--target-yaw-rad=").empty()) {
      opt.target_yaw_deg = std::stod(after("--target-yaw-rad=")) * RAD2DEG;
      opt.single_target = true;
    } else if (!after("--target-pitch-rad=").empty()) {
      opt.target_pitch_deg = std::stod(after("--target-pitch-rad=")) * RAD2DEG;
      opt.single_target = true;
    } else if (!after("--hold-seconds=").empty()) opt.hold_seconds = std::stod(after("--hold-seconds="));
    else if (!after("--config=").empty()) opt.config = after("--config=");
    else if (!after("--axis=").empty()) opt.axis = after("--axis=");
    else if (!after("--step-rad=").empty()) opt.step_rad = std::stod(after("--step-rad="));
    else if (!after("--step-seconds=").empty()) opt.step_seconds = std::stod(after("--step-seconds="));
    else if (!after("--settle-seconds=").empty())
      opt.settle_seconds = std::stod(after("--settle-seconds="));
    else if (a.rfind("-c=", 0) == 0) opt.config = a.substr(3);
    else {
      fmt::print(stderr, "未知参数: {}\n", a);
      return 2;
    }
  }
  if (opt.help) {
    fmt::print(
      "用法: xuc_motion_probe [-c=配置] [--axis=yaw|pitch] [--step-rad=0.05]\n"
      "                       [--step-seconds=2] [--settle-seconds=1.5] [--execute]\n"
      "      单目标: xuc_motion_probe -c=配置 --target-yaw-deg=0.5 [--target-pitch-deg=..]\n"
      "              或 --target-yaw-rad=0.5 [--target-pitch-rad=..]\n"
      "                       [--hold-seconds=2] [--park-after] [--execute]\n"
      "  yaw/pitch 按本固件契约下发为「达妙电机位置目标(rad)」；yaw ∈ [-3.14,3.14]，\n"
      "  pitch ∈ [-0.38,0.34]。默认 dry-run；--execute 才真正下发。\n");
    return 0;
  }
  if (opt.axis != "yaw" && opt.axis != "pitch") {
    fmt::print(stderr, "[FAIL] --axis 只能是 yaw 或 pitch\n");
    return 2;
  }
  if (opt.step_rad <= 0.0 || opt.step_rad > MAX_STEP_RAD) {
    fmt::print(stderr, "[FAIL] --step-rad 必须位于 (0, {:.2f}]\n", MAX_STEP_RAD);
    return 2;
  }

  const std::string cfg_path = "/tmp/xuc_motion_probe.yaml";
  {
    auto yaml = YAML::LoadFile(opt.config);
    yaml["xuc_enable"] = true;
    yaml["xuc_allow_control"] = true;
    yaml["xuc_allow_shoot"] = false;  // 本工具永不涉及发射
    yaml["xuc_require_feedback_for_control"] = true;
    yaml["xuc_require_auto_mode_for_control"] = false;
    // 发送端不再做角度钳位：本固件的目标是电机位置，钳位交给电机层
    yaml["xuc_max_yaw_excursion_rad"] = 0.0;
    yaml["xuc_lock_pitch_for_control"] = false;
    std::ofstream out(cfg_path);
    out << yaml;
  }

  fmt::print(
    "模式: {}  轴: {}  阶跃: {:.4f} rad ({:.2f}°)\n",
    opt.execute ? "EXECUTE（会真实下发 control=1 与电机目标角）" : "DRY-RUN（只看不动）", opt.axis,
    opt.step_rad, opt.step_rad * RAD2DEG);
  fmt::print(
    "固件契约: yaw/pitch = 电机位置(rad)；yaw ∈ [{:.2f},{:.2f}]，pitch ∈ [{:.2f},{:.2f}]；"
    "slew 5 rad/s；刷新窗口 100 ms\n",
    YAW_MIN, YAW_MAX, PITCH_MIN, PITCH_MAX);

  io::XucSender xuc(cfg_path);
  if (!xuc.enabled()) {
    fmt::print(stderr, "[FAIL] 串口无法打开\n");
    std::remove(cfg_path.c_str());
    return 3;
  }

  const auto wait_start = std::chrono::steady_clock::now();
  while (
    !xuc.rx_valid() &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() < 3.0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!xuc.rx_valid()) {
    fmt::print(stderr, "[FAIL] 3 秒内没有收到有效反馈\n");
    std::remove(cfg_path.c_str());
    return 4;
  }
  if (xuc.mode() != 1) {
    fmt::print(
      stderr,
      "[FAIL] 下位机不在 AUTOAIM（mode={}，需要 1）。请把左右拨杆都打到 DOWN 再试。\n",
      static_cast<unsigned>(xuc.mode()));
    std::remove(cfg_path.c_str());
    return 5;
  }
  fmt::print(
    "下位机: mode=1(AUTOAIM)  robot_id={}  imu_pitch={:+.3f}°  imu_yaw={:+.3f}°  "
    "有效包={}  CRC错误={}\n",
    static_cast<unsigned>(xuc.robot_id()), xuc.imu_pitch() * RAD2DEG, xuc.imu_yaw() * RAD2DEG,
    xuc.valid_packets(), xuc.crc_errors());

  io::Command cmd{};
  cmd.control = false;
  cmd.shoot = false;
  cmd.yaw = 0.0;
  cmd.pitch = 0.0;

  auto run_phase = [&](const char * name, double seconds, bool control, double yaw_target,
                       double pitch_target) {
    Sample s;
    const auto t0 = std::chrono::steady_clock::now();
    int lost = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
      cmd.control = control && opt.execute;
      cmd.yaw = yaw_target;
      cmd.pitch = pitch_target;
      xuc.send(cmd);
      if (!xuc.rx_valid()) ++lost;
      if (xuc.mode() != 1) ++lost;
      // 只在阶段后半段取样，避开运动过程
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > seconds * 0.5)
        s.add(xuc.imu_yaw() * RAD2DEG, xuc.imu_pitch() * RAD2DEG);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    fmt::print(
      "  [{:<6}] t={:4.1f}s  target(yaw={:+.4f}, pitch={:+.4f})  →  {}{}\n", name, seconds,
      yaw_target, pitch_target, s.str(), lost ? fmt::format("  异常样本 {}", lost) : "");
    return s;
  };

  const double step_yaw = (opt.axis == "yaw") ? opt.step_rad : 0.0;
  const double step_pitch = (opt.axis == "pitch") ? opt.step_rad : 0.0;

  if (opt.single_target) {
    // ---- 单目标模式：把 yaw/pitch 直接当电机位置目标下发 ----
    const double ty = opt.target_yaw_deg / RAD2DEG;
    const double tp = opt.target_pitch_deg / RAD2DEG;
    if (opt.target_yaw_deg < YAW_MIN * RAD2DEG || opt.target_yaw_deg > YAW_MAX * RAD2DEG) {
      fmt::print(stderr, "[FAIL] target-yaw-deg 超出电机限幅 [{:.1f}, {:.1f}]°\n", YAW_MIN * RAD2DEG,
                 YAW_MAX * RAD2DEG);
      std::remove(cfg_path.c_str());
      return 2;
    }
    if (opt.target_pitch_deg < PITCH_MIN * RAD2DEG || opt.target_pitch_deg > PITCH_MAX * RAD2DEG) {
      fmt::print(stderr, "[FAIL] target-pitch-deg 超出电机限幅 [{:.1f}, {:.1f}]°\n",
                 PITCH_MIN * RAD2DEG, PITCH_MAX * RAD2DEG);
      std::remove(cfg_path.c_str());
      return 2;
    }

    fmt::print(
      "\n--- 单目标模式 ---\n  电机位置目标: yaw {:+.4f} rad ({:+.3f}°)  pitch {:+.4f} rad "
      "({:+.3f}°)\n",
      ty, opt.target_yaw_deg, tp, opt.target_pitch_deg);
    const Sample base = run_phase("基线", opt.settle_seconds, false, 0.0, 0.0);
    Sample held;
    if (opt.execute) {
      held = run_phase("目标", opt.hold_seconds, true, ty, tp);
      if (opt.park_after) run_phase("回零", opt.settle_seconds, true, 0.0, 0.0);
    } else {
      fmt::print("  (dry-run：未下发 control=1)\n");
    }
    // 撤控（目标会保留在下位机里，电机继续停在最后目标）
    cmd.control = false;
    cmd.shoot = false;
    cmd.yaw = 0.0;
    cmd.pitch = 0.0;
    for (int i = 0; i < 40; ++i) {
      xuc.send(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    fmt::print("\n===== 结果 =====\n");
    fmt::print("  基线        {}\n", base.str());
    if (opt.execute) {
      fmt::print("  目标保持后  {}\n", held.str());
      fmt::print(
        "\n  IMU 实测变化: yaw {:+.3f}°  pitch {:+.3f}°（目标 yaw {:+.3f}°）\n",
        held.yaw() - base.yaw(), held.pitch() - base.pitch(), opt.target_yaw_deg);
      const double dy = held.yaw() - base.yaw();
      if (std::abs(dy) < 0.15) {
        fmt::print("  ⇒ IMU 几乎没动：指令可能没被接受，或目标与当前电机位置几乎相同。\n");
      } else if (dy * opt.target_yaw_deg > 0) {
        fmt::print("  ⇒ 同向跟随，通路成立。\n");
      } else {
        fmt::print("  ⇒ 反向跟随，上位机需要取反。\n");
      }
      fmt::print(
        "  电机停在最后目标（yaw {:+.3f}°{}）。\n", opt.park_after ? 0.0 : opt.target_yaw_deg,
        opt.park_after ? "，已回零" : "，未回零");
    } else {
      fmt::print("  DRY-RUN：未下发任何 control=1。\n");
    }
    fmt::print(
      "  有效包={}  CRC错误={}  字段错误={}  RX字节={}\n", xuc.valid_packets(), xuc.crc_errors(),
      xuc.field_errors(), xuc.rx_bytes());
    std::remove(cfg_path.c_str());
    return 0;
  }

  fmt::print("\n--- 序列开始 ---\n");
  const Sample base = run_phase("基线", opt.settle_seconds, false, 0.0, 0.0);
  Sample park, step, back;
  if (opt.execute) {
    park = run_phase("回零", opt.settle_seconds, true, 0.0, 0.0);
    step = run_phase("阶跃", opt.step_seconds, true, step_yaw, step_pitch);
    back = run_phase("回零", opt.settle_seconds, true, 0.0, 0.0);
  } else {
    fmt::print("  (dry-run：跳过 回零/阶跃/回零 三个阶段)\n");
  }

  // 收尾：撤控（多次防丢包）
  cmd.control = false;
  cmd.shoot = false;
  cmd.yaw = 0.0;
  cmd.pitch = 0.0;
  for (int i = 0; i < 40; ++i) {
    xuc.send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  fmt::print("\n===== 结果 =====\n");
  fmt::print("  基线      {}\n", base.str());
  if (opt.execute) {
    fmt::print("  回零后    {}\n", park.str());
    fmt::print("  阶跃后    {}\n", step.str());
    fmt::print("  再回零    {}\n", back.str());
    const double d_yaw = step.yaw() - park.yaw();
    const double d_pitch = step.pitch() - park.pitch();
    fmt::print(
      "\n  电机目标 {:+.4f} rad ({:+.2f}°)  →  IMU 实测变化  yaw {:+.3f}°  pitch {:+.3f}°\n",
      opt.step_rad, opt.step_rad * RAD2DEG, d_yaw, d_pitch);
    if (opt.axis == "yaw") {
      const double ratio = (opt.step_rad * RAD2DEG) != 0 ? d_yaw / (opt.step_rad * RAD2DEG) : 0.0;
      fmt::print("  比值 (IMU变化 / 目标变化) = {:+.3f}\n", ratio);
      if (std::abs(d_yaw) < 0.15) {
        fmt::print("  ⇒ IMU 几乎没变化：指令可能没被接受（检查 mode=1、刷新率、control 位）。\n");
      } else if (ratio > 0) {
        fmt::print("  ⇒ 同向跟随，通路成立；比值≈1 表示电机位置与 IMU yaw 近似 1:1。\n");
      } else {
        fmt::print("  ⇒ 反向！电机位置与 IMU yaw 符号相反，上位机必须取反。\n");
      }
      if (std::abs(std::abs(d_yaw) - opt.step_rad * RAD2DEG) > 0.5 && std::abs(d_yaw) >= 0.15) {
        fmt::print(
          "     注意：|IMU变化|={:.2f}° 与目标 {:.2f}° 不等 ⇒ 中间有减速比或非线性，"
          "需要标定映射后再闭环。\n",
          std::abs(d_yaw), opt.step_rad * RAD2DEG);
      }
    }
    fmt::print(
      "  （结束时已发 control=0；电机停在最后目标，即 0 位。）\n");
  } else {
    fmt::print("  DRY-RUN：未下发任何 control=1，电机未被指令驱动。\n");
  }
  fmt::print(
    "  有效包={}  CRC错误={}  字段错误={}  RX字节={}\n", xuc.valid_packets(), xuc.crc_errors(),
    xuc.field_errors(), xuc.rx_bytes());

  std::remove(cfg_path.c_str());
  return 0;
}
