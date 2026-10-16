#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在目标机 standard.cpp 上精确插入三批改动（幂等，可重复运行）：
  A) [SHOOT] 监控显示：开火请求计数、inhibit 计数、配置开关与稳定门
  B) [SHOOT-HOLD] 持续 shoot 诊断模式（环境变量 SPV_SHOOT_HOLD_MS 控制）
  C) 开火节奏回退模式（xuc_fire_cadence_fallback_ms）：不依赖拨弹遥测，定时重复开火
只做局部锚点替换，不整体覆盖，保留其他 agent 的修改。

重要：小电脑工程实际编译的是 `src/apps/standard.cpp`
（CMakeLists: add_executable(standard src/apps/standard.cpp)）。
默认自动探测：优先 src/apps/standard.cpp，不存在则回退 src/standard.cpp。
"""
import os
import sys

CANDIDATES = ['src/apps/standard.cpp', 'src/standard.cpp']
P = sys.argv[1] if len(sys.argv) > 1 else next(
    (c for c in CANDIDATES if os.path.exists(c)), None)
if P is None:
    raise SystemExit('找不到 standard.cpp（尝试过: %s）' % ', '.join(CANDIDATES))
print('目标文件:', P)

s = open(P, encoding='utf-8').read()
orig = s
log = []


def sub(tag, old, new, marker):
    """marker 已存在则跳过；否则做一次替换。
    注意：marker 必须是**新增内容独有**的字符串，否则重复运行会重复插入
    （2026-10-13 曾因 marker 取变量名导致重复声明、编译失败）。
    """
    global s
    if marker in s:
        log.append('%s 已存在' % tag)
    elif old in s:
        s = s.replace(old, new, 1)
        log.append('%s OK' % tag)
    else:
        log.append('%s FAIL 锚点未找到' % tag)


# ---------- A1: 配置读取（含 C 的配置项）----------
sub('A1 配置读取',
    '''  const bool xuc_fire_guard_enabled = integration_yaml["xuc_fire_guard_enabled"] &&
    integration_yaml["xuc_fire_guard_enabled"].as<bool>();''',
    '''  const bool xuc_fire_guard_enabled = integration_yaml["xuc_fire_guard_enabled"] &&
    integration_yaml["xuc_fire_guard_enabled"].as<bool>();
  // 仅用于监控窗口显示"开火总开关是否已解除"；真正生效位置在驱动 io/xuc.cpp。
  const bool xuc_allow_shoot_configured = integration_yaml["xuc_allow_shoot"] &&
    integration_yaml["xuc_allow_shoot"].as<bool>();
  // 开火节奏回退间隔（秒）。>0 时不依赖拨弹遥测，改为固定间隔重复开火。
  // 0 = 保持原遥测模式。见 fire cadence gate 处注释。
  const double xuc_fire_cadence_fallback_s =
    integration_yaml["xuc_fire_cadence_fallback_ms"]
      ? std::clamp(
          integration_yaml["xuc_fire_cadence_fallback_ms"].as<double>(), 0.0, 5000.0) / 1000.0
      : 0.0;''',
    'xuc_fire_cadence_fallback_s')

# ---------- A2/B: 计数器与时刻变量 ----------
sub('A2 计数器变量',
    '  bool xuc_gyro_cadence_hold_logged = false;',
    '''  bool xuc_gyro_cadence_hold_logged = false;
  long xuc_fire_inhibit_count = 0;   // 被拨弹/节奏门控拦下的累计次数（监控窗口用）
  // 【诊断】持续 shoot 模式的计数与结束标记；仅用于 SPV_SHOOT_HOLD_MS 诊断，验收后删除
  long xuc_shoot_hold_frames = 0;
  bool xuc_shoot_hold_ended_logged = false;
  // 最近一次实际开出 shoot=1 的时刻；供“定时回退”节奏门控使用（见 cadence gate）
  std::chrono::steady_clock::time_point xuc_last_shot_issue_time{};''',
    'xuc_last_shot_issue_time')

# ---------- C: 节奏门控改双模式 ----------
sub('C 节奏门控双模式',
    '''      const bool feedback_cadence_gate_open =
        !xuc.enabled() ||
        (!xuc_pending_shot && xuc_active_shot_id == 0 &&
         (!xuc_feeder_activity_seen ||
          (!xuc_feeder_active &&
           std::chrono::duration<double>(t - xuc_feeder_last_active).count() >=
             xuc_gyro_post_feed_recovery_s)));''',
    '''      // 【节奏门控】两种模式：
      //  (a) 遥测模式（默认，xuc_fire_cadence_fallback_ms == 0）：依赖下位机上报的
      //      拨弹转速来判定“上一发已结束”。前提是固件已接入 feeder 遥测。
      //  (b) 定时回退模式（xuc_fire_cadence_fallback_ms > 0）：不依赖拨弹遥测，
      //      改为“距上次开火至少间隔 N 毫秒”。
      //      启用原因（2026-10-13 实测）：电控固件把 bullet_speed/bullet_count 写死 0，
      //      遥测模式下永远等不到拨弹反馈，开火被反复 inhibit（30 秒仅 7 发）；
      //      同一实测证明单帧 shoot=1（约 10ms）即可触发拨弹动作，故固定间隔可行。
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
                  xuc_gyro_post_feed_recovery_s))));''',
    'cadence_uses_fallback')

# ---------- C: 日志文案区分模式 ----------
sub('C 门控日志文案',
    '''          tools::logger()->info(
            "[XUC][FIRE] request inhibited until feeder is available");
          xuc_gyro_cadence_hold_logged = true;
        }
      } else if (xuc_gyro_cadence_hold_logged) {
        tools::logger()->info(
          "[XUC][FIRE] feeder available; fresh qualification required");
        xuc_gyro_cadence_hold_logged = false;
      }''',
    '''          tools::logger()->info(
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
      }''',
    'cadence fallback interval elapses')

# ---------- A4: SHOOT 显示 ----------
sub('A4 SHOOT 显示',
    '''    } else {
      command.shoot = native_shoot;
      if (measurement_fallback) command.shoot = false;
    }''',
    '''    } else {
      command.shoot = native_shoot;
      if (measurement_fallback) command.shoot = false;
    }

    // 开火请求计数与显示：把"上位机决定开火"的次数与当前帧状态打到监控窗口。
    // 用途：试射时直接看到开火请求有没有发出去、发了多少次，以及被哪一级门控拦下。
    // 注意：这只是上位机的决策计数，不代表下位机真的拨弹。
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
          "允许开火(配置)={} 稳定门(ms)={:.0f}  节奏={}\\n",
          command.shoot ? 1 : 0, command.control ? 1 : 0, shoot_request_count,
          xuc_fire_inhibit_count, shoot_frames_total, xuc_allow_shoot_configured ? 1 : 0,
          xuc_fire_stable_s * 1000.0,
          cadence_uses_fallback
            ? fmt::format("定时 {:.0f}ms", xuc_fire_cadence_fallback_s * 1000.0)
            : std::string("遥测"));
      }
    }''',
    '[SHOOT] 本帧')

# ---------- B + C: send 段（持续 shoot + 时刻记录 + pending 条件）----------
sub('B/C send 段',
    '''    if (xuc.enabled()) {
      xuc.send(command);
      if (command.shoot) {
        const int64_t tx_ns = xuc.last_shoot_tx_ns();
        if (tx_ns > 0) {
          xuc_pending_shot_time = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(tx_ns));
        }
      }
    } else {''',
    '''    if (xuc.enabled()) {
      // 【诊断模式】持续 shoot：SPV_SHOOT_HOLD_MS 指定时长内无视算法决策，强制
      // shoot=1 连续下发。用途：电控反馈"shoot 只有一帧、持续极短，拨弹轮没动"，
      // 需要区分"指令太短"还是"下位机不执行"。
      // 用法：SPV_SHOOT_HOLD_MS=3000 ./build/standard -c=configs/standard.yaml
      // 安全：仍要求 control=1 且配置允许开火；结束后自动恢复算法决策。
      // 仅诊断用，验收后应删除。
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
            "[SHOOT-HOLD] 持续下发中：帧={} 剩余={:.1f}s\\n", xuc_shoot_hold_frames,
            std::chrono::duration<double>(shoot_hold_deadline - hold_now).count());
        }
      } else if (shoot_hold_ms > 0 && xuc_shoot_hold_frames > 0 &&
                 !xuc_shoot_hold_ended_logged) {
        tools::logger()->warn(
          "[SHOOT-HOLD] 持续 shoot 结束：共下发 {} 帧（{} ms）", xuc_shoot_hold_frames,
          shoot_hold_ms);
        xuc_shoot_hold_ended_logged = true;
      }
      xuc.send(command);
      // 记录本次实际开出 shoot=1 的时刻，供“定时回退”节奏门控计算间隔。
      if (command.shoot) {
        xuc_last_shot_issue_time = std::chrono::steady_clock::now();
      }
      // 定时回退与诊断模式均不记录 pending：这两种模式不依赖拨弹反馈，
      // 若记录 pending 会因遥测为 0 而超时循环，反而阻塞节奏。
      if (command.shoot && !shoot_hold_active && xuc_fire_cadence_fallback_s <= 0.0) {
        const int64_t tx_ns = xuc.last_shoot_tx_ns();
        if (tx_ns > 0) {
          xuc_pending_shot_time = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(tx_ns));
        }
      }
    } else {''',
    '[SHOOT-HOLD]')

if s != orig:
    open(P, 'w', encoding='utf-8').write(s)
    log.append('已写回文件')
print('\n'.join(log))
