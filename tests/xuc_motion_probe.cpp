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
#include <ctime>
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
// 命令单位：2026-10-11 实测确认，电控新固件把命令的 yaw/pitch 解释为
// **绝对 IMU 角，单位＝度**（与反馈 imu_pitch/imu_yaw 同帧同号）。
// 实测证据：下发 yaw=0.0877 / pitch=0.0279 后，IMU 落在 +0.080° / +0.183°，
// 即原始数值被当作“度”使用；若按弧度解释应落在 5.02° / 1.60°，与实测不符。
//
// 关于范围：新固件 AUTOAIM 分支**没有 IMU 角限幅**，只有电机层
// SetTargetPos 的 pos_min/pos_max 钳位：
//   YAW   ∈ [-3.14, +3.14] rad = [-180°, +180°]（宽松）
//   PITCH ∈ [-0.38, +0.34] rad = [-21.8°, +19.5°]（很窄）
// 因此这里**不再限制目标绝对值**（IMU yaw 本来就可能在 ±180° 内的任意处，
// 例如 +27°；限制绝对值会误拦正常指令），只限制**单次偏移量**，
// 并对 pitch 的最终目标做越界检查（因为它的限位确实很窄）。
constexpr double MAX_REL_YAW_DEG = 30.0;
constexpr double MAX_REL_PITCH_DEG = 10.0;
// pitch 最终目标的安全范围（留 ~1° 余量给电机限位 [-21.8, +19.5]）
constexpr double PITCH_TARGET_MIN_DEG = -20.0;
constexpr double PITCH_TARGET_MAX_DEG = 18.5;

struct Options
{
  std::string config = "configs/standard.yaml";
  double step_deg = 5.0;  // 序列模式的阶跃量（度）
  double step_seconds = 2.0;
  double settle_seconds = 1.5;
  std::string axis = "yaw";  // yaw | pitch
  bool execute = false;
  bool help = false;
  // 单目标模式：yaw/pitch 作为绝对 IMU 角（度）下发
  bool single_target = false;
  double target_yaw_deg = 0.0;
  double target_pitch_deg = 0.0;
  double hold_seconds = 2.0;
  bool park_after = false;
  // 相对偏移模式：目标 = 当前 IMU 显示角 + 偏移（度）
  bool relative = false;
  double rel_yaw_deg = 0.0;
  double rel_pitch_deg = 0.0;
  // 纯监测模式：只读、100 Hz 记录，不发任何指令，用于观察控制结束时刻的姿态
  bool monitor = false;
  double monitor_seconds = 60.0;
  // 逐帧轨迹：把控制全过程与“控制结束后”的姿态都记录在同一进程里，
  // 避免另开监测进程抢占串口（那样用户的 --execute 会打不开串口）。
  bool trace = false;
  double tail_seconds = 3.0;  // 撤控后再记录多久
  std::string trace_path;     // --trace 时的落盘路径（为空则自动生成）
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
    else if (a == "--trace") opt.trace = true;
    else if (!after("--trace-path=").empty()) {
      opt.trace_path = after("--trace-path=");
      opt.trace = true;
    }
    else if (!after("--tail-seconds=").empty()) opt.tail_seconds = std::stod(after("--tail-seconds="));
    else if (a == "--monitor") opt.monitor = true;
    else if (!after("--monitor-seconds=").empty())
      opt.monitor_seconds = std::stod(after("--monitor-seconds="));
    else if (a == "--park-after") opt.park_after = true;
    else if (a == "--relative") opt.relative = true;
    else if (a == "--help" || a == "-h") opt.help = true;
    else if (!after("--rel-yaw-deg=").empty()) {
      opt.rel_yaw_deg = std::stod(after("--rel-yaw-deg="));
      opt.relative = true;
    } else if (!after("--rel-pitch-deg=").empty()) {
      opt.rel_pitch_deg = std::stod(after("--rel-pitch-deg="));
      opt.relative = true;
    } else if (!after("--target-yaw-deg=").empty()) {
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
    else if (!after("--step-deg=").empty()) opt.step_deg = std::stod(after("--step-deg="));
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
      "用法: xuc_motion_probe [-c=配置] [--axis=yaw|pitch] [--step-deg=5]\n"
      "                       [--step-seconds=2] [--settle-seconds=1.5] [--execute]\n"
      "      语义判定: xuc_motion_probe -c=配置 --rel-yaw-deg=5 [--rel-pitch-deg=2]\n"
      "                       [--hold-seconds=2] [--execute]\n"
      "        目标 = 当前 IMU 角 + 偏移；比值≈1 ⇒ 新语义(IMU角)，比值≈0.55 ⇒ 旧语义(电机位置)\n"
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
  if (opt.step_deg <= 0.0 || opt.step_deg > MAX_REL_YAW_DEG) {
    fmt::print(stderr, "[FAIL] --step-deg 必须位于 (0, {:.0f}]\n", MAX_REL_YAW_DEG);
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
    // 新语义下命令是绝对 IMU 角（度），发送端不做额外钳位
    yaml["xuc_max_yaw_excursion_rad"] = 0.0;
    yaml["xuc_lock_pitch_for_control"] = false;
    std::ofstream out(cfg_path);
    out << yaml;
  }

  fmt::print(
    "模式: {}\n",
    opt.execute ? "EXECUTE（会真实下发 control=1）" : "DRY-RUN（只看不动）");
  fmt::print(
    "固件契约（2026-10-11 实测确认）: yaw/pitch = **绝对 IMU 角，单位=度**；"
    "单次偏移上限 yaw ±{:.0f}°、pitch ±{:.0f}°；pitch 目标限 [{:.1f},{:.1f}]°；刷新窗口 100 ms\n",
    MAX_REL_YAW_DEG, MAX_REL_PITCH_DEG, PITCH_TARGET_MIN_DEG, PITCH_TARGET_MAX_DEG);

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
  if (opt.monitor) {
    // ---- 纯监测模式：只读、高频记录，绝不发送任何指令 ----
    // 用途：观察外部控制（工具 --execute 或遥控器）结束那一刻的姿态与后续漂移。
    fmt::print(
      "\n--- 监测模式（只读，{:.0f}s，约 100 Hz）---\n"
      "  字段: t_s mode yaw_deg pitch_deg d_pitch_deg(相对首帧) valid\n"
      "  监测期间本程序不发送任何字节。\n",
      opt.monitor_seconds);
    const auto t0 = std::chrono::steady_clock::now();
    double first_pitch = 0.0;
    bool have_first = false;
    int n = 0;
    uint8_t last_mode = 255;
    while (true) {
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (t >= opt.monitor_seconds) break;
      const bool valid = xuc.rx_valid();
      const double yaw = xuc.imu_yaw() * RAD2DEG;
      const double pitch = xuc.imu_pitch() * RAD2DEG;
      const uint8_t m = xuc.mode();
      if (!have_first && valid) {
        first_pitch = pitch;
        have_first = true;
      }
      // mode 变化单独标记，便于定位“控制结束/拨杆切换”的时刻
      if (m != last_mode) {
        fmt::print(">>> mode {} -> {}  @ t={:.2f}s\n", static_cast<unsigned>(last_mode),
                   static_cast<unsigned>(m), t);
        last_mode = m;
      }
      fmt::print(
        "{:7.3f} {} {:9.3f} {:9.3f} {:+8.3f} {}\n", t, static_cast<unsigned>(m), yaw, pitch,
        pitch - first_pitch, valid ? 1 : 0);
      ++n;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    fmt::print(
      "\n监测结束: {} 帧；首帧 pitch {:+.3f}°  末帧 pitch {:+.3f}°  净变化 {:+.3f}°\n", n,
      first_pitch, xuc.imu_pitch() * RAD2DEG, xuc.imu_pitch() * RAD2DEG - first_pitch);
    std::remove(cfg_path.c_str());
    return 0;
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

  const auto trace_t0 = std::chrono::steady_clock::now();
  // 轨迹落盘：--trace 时同时写文件，避免“终端输出没保存”导致事后无法复查。
  std::ofstream trace_file;
  if (opt.trace) {
    if (opt.trace_path.empty()) {
      const auto now = std::chrono::system_clock::now();
      const std::time_t tt = std::chrono::system_clock::to_time_t(now);
      char stamp[32]{};
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&tt));
      opt.trace_path = fmt::format("/tmp/xuc_motion_trace_{}.csv", stamp);
    }
    trace_file.open(opt.trace_path);
    if (trace_file) {
      trace_file << "t_s,phase,control,raw_imu_yaw_deg,raw_imu_pitch_deg,valid,mode\n";
      fmt::print("轨迹落盘: {}\n", opt.trace_path);
    } else {
      fmt::print(stderr, "[warning] 轨迹文件无法写入: {}\n", opt.trace_path);
    }
  }

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
      // 逐帧轨迹（可选）：把控制全过程与结束后的姿态都记下来，避免另开进程抢串口
      if (opt.trace) {
        const double t =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - trace_t0).count();
        const int ctl = (control && opt.execute) ? 1 : 0;
        // 记录**原始值**（与固件比较用的同一个量），避免事后被显示符号误导
        const double raw_yaw = xuc.raw_imu_yaw() * RAD2DEG;
        const double raw_pitch = xuc.raw_imu_pitch() * RAD2DEG;
        fmt::print(
          "T {:8.3f} {:>7} {:>8} {:9.3f} {:9.3f} {}\n", t, name, ctl ? "ctl=1" : "ctl=0", raw_yaw,
          raw_pitch, xuc.rx_valid() ? 1 : 0);
        if (trace_file) {
          trace_file << fmt::format(
            "{:.3f},{},{},{:.4f},{:.4f},{},{}\n", t, name, ctl, raw_yaw, raw_pitch,
            xuc.rx_valid() ? 1 : 0, static_cast<unsigned>(xuc.mode()));
          trace_file.flush();
        }
      }
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

  if (opt.relative) {
    // ---- 相对偏移模式（新语义：绝对 IMU 角，单位＝度）----
    // 目标 = 当前 IMU 显示角 + 偏移（度），直接把该数值作为 cmd.yaw/pitch 下发。
    // 实测（2026-10-11）：下发 0.0877 → IMU 落在 +0.080°，证实原始数值按“度”使用。
    if (std::abs(opt.rel_yaw_deg) > MAX_REL_YAW_DEG ||
        std::abs(opt.rel_pitch_deg) > MAX_REL_PITCH_DEG) {
      fmt::print(stderr, "[FAIL] 相对偏移过大（yaw ≤{:.0f}°，pitch ≤{:.0f}°）\n",
                 MAX_REL_YAW_DEG, MAX_REL_PITCH_DEG);
      std::remove(cfg_path.c_str());
      return 2;
    }
    // ⚠ 基准必须用 raw_imu_yaw()（下位机原始值），不能用 imu_yaw()。
    // 配置里 xuc_imu_yaw_sign = -1，imu_yaw() 返回的是翻转后的显示值；而固件
    // 是用 GetAngleYaw()（原始值）做 err = target - raw 的比较。
    // 2026-10-12 实测教训：用显示值当基准会引入 2×raw 的误差 —— 当时 raw=-29.07°、
    // 显示 +29.07°，发出 +29.07° 后固件算出 err=+58.14°，云台猛转 57.8°。
    // pitch 因为 xuc_imu_pitch_sign=+1 恰好未暴露该问题。
    const double base_yaw_deg = xuc.raw_imu_yaw() * RAD2DEG;
    const double base_pitch_deg = xuc.raw_imu_pitch() * RAD2DEG;
    const double cmd_yaw_deg = base_yaw_deg + opt.rel_yaw_deg;
    const double cmd_pitch_deg = base_pitch_deg + opt.rel_pitch_deg;

    // 目标绝对值保护：只对 pitch 做（其电机限位很窄 [-21.8°, +19.5°]）。
    // yaw 电机限位是 ±180°，IMU yaw 本来就可能在任何角度，限制绝对值会误拦。
    // 注意：越界时电机被钳在限位上、误差持续存在 ⇒ 表现为“发了指令但电机不动”且持续堵转。
    if (cmd_pitch_deg < PITCH_TARGET_MIN_DEG || cmd_pitch_deg > PITCH_TARGET_MAX_DEG) {
      fmt::print(
        stderr,
        "[FAIL] 目标 pitch {:.3f}° 超出安全范围 [{:.1f}, {:.1f}]°（当前 {:.3f}° + 偏移 {:+.2f}°）\n"
        "  新固件无 IMU 角限幅，越界会被电机限位钳住并持续堵转。\n"
        "  若当前 pitch 已接近上限，请先用遥控器把它降下来再测。\n",
        cmd_pitch_deg, PITCH_TARGET_MIN_DEG, PITCH_TARGET_MAX_DEG, base_pitch_deg,
        opt.rel_pitch_deg);
      std::remove(cfg_path.c_str());
      return 2;
    }

    fmt::print(
      "\n--- 相对偏移模式（新语义：绝对 IMU 角，单位=度）---\n"
      "  当前 IMU: yaw {:+.3f}°  pitch {:+.3f}°\n"
      "  下发目标: yaw {:+.3f}（按度解释）  pitch {:+.3f}（按度解释）\n"
      "  期望: IMU 变化≈({:+.2f}°, {:+.2f}°)\n",
      base_yaw_deg, base_pitch_deg, cmd_yaw_deg, cmd_pitch_deg, opt.rel_yaw_deg, opt.rel_pitch_deg);

    const Sample base = run_phase("基线", opt.settle_seconds, false, 0.0, 0.0);
    Sample held;
    if (opt.execute) {
      held = run_phase("目标", opt.hold_seconds, true, cmd_yaw_deg, cmd_pitch_deg);
      // 控制结束后继续记录：这一段正是观察“摩擦轮停转后 pitch 是否还在动”的关键窗口
      if (opt.trace && opt.tail_seconds > 0.0) {
        run_phase("撤控后", opt.tail_seconds, false, 0.0, 0.0);
      }
    } else {
      fmt::print("  (dry-run：未下发 control=1)\n");
    }
    cmd.control = false;
    cmd.shoot = false;
    cmd.yaw = 0.0;
    cmd.pitch = 0.0;
    for (int i = 0; i < 40; ++i) {
      xuc.send(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const double d_yaw = held.yaw() - base.yaw();
    const double d_pitch = held.pitch() - base.pitch();
    fmt::print("\n===== 结果 =====\n");
    fmt::print("  基线      {}\n", base.str());
    if (opt.execute) {
      fmt::print("  目标保持后 {}\n", held.str());
      fmt::print(
        "\n  实测 IMU 变化: yaw {:+.3f}°  pitch {:+.3f}°  （下发偏移 yaw {:+.2f}° pitch {:+.2f}°）\n",
        d_yaw, d_pitch, opt.rel_yaw_deg, opt.rel_pitch_deg);
      if (std::abs(opt.rel_yaw_deg) > 1.0) {
        const double ratio = d_yaw / opt.rel_yaw_deg;
        fmt::print("  yaw 比值 (实测变化/下发偏移) = {:+.3f}\n", ratio);
        if (std::abs(d_yaw) < 0.2) {
          fmt::print("  ⇒ 几乎没动：静摩擦、指令未被接受，或该轴被其它模式占用。\n");
        } else if (ratio > 0.75 && ratio < 1.25) {
          fmt::print("  ⇒ 比值≈1 ⇒ **新语义（IMU 绝对角/度）确认**，可以接视觉闭环。\n");
        } else if (ratio < 0) {
          fmt::print("  ⇒ 反向！命令到 IMU 显示存在负号，需修正符号约定。\n");
        } else {
          fmt::print("  ⇒ 比值异常，需人工判断（可能含限幅、传动比或非线性）。\n");
        }
      }
      if (std::abs(opt.rel_pitch_deg) > 1.0) {
        fmt::print("  pitch 比值 = {:+.3f}\n", d_pitch / opt.rel_pitch_deg);
      }
    } else {
      fmt::print("  DRY-RUN：未下发任何 control=1。\n");
    }
    fmt::print(
      "  有效包={}  CRC错误={}  字段错误={}\n", xuc.valid_packets(), xuc.crc_errors(),
      xuc.field_errors());
    std::remove(cfg_path.c_str());
    return 0;
  }

  if (opt.single_target) {
    // ---- 单目标模式：把 yaw/pitch 作为绝对 IMU 角（度）下发 ----
    const double ty = opt.target_yaw_deg;
    const double tp = opt.target_pitch_deg;
    // yaw 电机限位 ±180°，不做绝对值限制；pitch 限位窄，需检查
    if (tp < PITCH_TARGET_MIN_DEG || tp > PITCH_TARGET_MAX_DEG) {
      fmt::print(stderr, "[FAIL] target-pitch-deg 超出安全范围 [{:.1f}, {:.1f}]°\n",
                 PITCH_TARGET_MIN_DEG, PITCH_TARGET_MAX_DEG);
      std::remove(cfg_path.c_str());
      return 2;
    }

    fmt::print(
      "\n--- 单目标模式（绝对 IMU 角，单位=度）---\n  yaw 目标 {:+.3f}°   pitch 目标 {:+.3f}°\n",
      ty, tp);
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

  fmt::print("\n--- 序列开始（绝对 IMU 角，单位=度；基准取下位机原始值）---\n");
  // 同 --rel-* 模式：目标必须基于 raw 值构造（见该处注释）
  const double anchor_yaw_deg = xuc.raw_imu_yaw() * RAD2DEG;
  const double anchor_pitch_deg = xuc.raw_imu_pitch() * RAD2DEG;
  const double step_yaw = anchor_yaw_deg + ((opt.axis == "yaw") ? opt.step_deg : 0.0);
  const double step_pitch = anchor_pitch_deg + ((opt.axis == "pitch") ? opt.step_deg : 0.0);
  fmt::print(
    "  锚点 yaw {:+.3f}°  pitch {:+.3f}°；阶跃 {:.2f}°\n", anchor_yaw_deg, anchor_pitch_deg,
    opt.step_deg);
  const Sample base = run_phase("基线", opt.settle_seconds, false, 0.0, 0.0);
  Sample park, step, back;
  if (opt.execute) {
    park = run_phase("锚点", opt.settle_seconds, true, anchor_yaw_deg, anchor_pitch_deg);
    step = run_phase("阶跃", opt.step_seconds, true, step_yaw, step_pitch);
    back = run_phase("回锚点", opt.settle_seconds, true, anchor_yaw_deg, anchor_pitch_deg);
  } else {
    fmt::print("  (dry-run：跳过 锚点/阶跃/回锚点 三个阶段)\n");
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
    fmt::print("  锚点后    {}\n", park.str());
    fmt::print("  阶跃后    {}\n", step.str());
    fmt::print("  回锚点后  {}\n", back.str());
    const double d_yaw = step.yaw() - park.yaw();
    const double d_pitch = step.pitch() - park.pitch();
    fmt::print(
      "\n  IMU 角目标偏移 {:+.2f}°  →  IMU 实测变化  yaw {:+.3f}°  pitch {:+.3f}°\n",
      opt.step_deg, d_yaw, d_pitch);
    if (opt.axis == "yaw") {
      const double ratio = opt.step_deg != 0 ? d_yaw / opt.step_deg : 0.0;
      fmt::print("  比值 (IMU变化 / 目标偏移) = {:+.3f}\n", ratio);
      if (std::abs(d_yaw) < 0.15) {
        fmt::print("  ⇒ IMU 几乎没变化：指令可能没被接受（检查 mode=1、刷新率、control 位）。\n");
      } else if (ratio > 0.75 && ratio < 1.25) {
        fmt::print("  ⇒ 比值≈1，新语义（绝对 IMU 角）成立。\n");
      } else if (ratio < 0) {
        fmt::print("  ⇒ 反向！符号约定需要修正。\n");
      } else {
        fmt::print("  ⇒ 比值异常（{:.2f}），需人工判断。\n", ratio);
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
