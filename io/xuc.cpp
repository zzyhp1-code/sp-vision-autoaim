#include "io/xuc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
bool XucSender::imu_at(
  std::chrono::steady_clock::time_point timestamp, ImuPose & pose,
  std::chrono::milliseconds max_gap) const
{
  if (!enabled_ || max_gap.count() <= 0) return false;
  std::unique_lock<std::mutex> lock(imu_history_mutex_);
  imu_history_changed_.wait_for(lock, std::chrono::milliseconds(30), [&] {
    return !imu_history_.empty() && imu_history_.back().timestamp >= timestamp;
  });
  if (imu_history_.empty() || timestamp < imu_history_.front().timestamp ||
      timestamp > imu_history_.back().timestamp) return false;
  auto hi = std::lower_bound(imu_history_.begin(), imu_history_.end(), timestamp,
    [](const ImuPose & sample, auto t) { return sample.timestamp < t; });
  if (hi == imu_history_.end()) return false;
  if (hi->timestamp == timestamp) {
    pose = *hi;
  } else {
    if (hi == imu_history_.begin()) return false;
    const auto lo = std::prev(hi);
    const auto gap = hi->timestamp - lo->timestamp;
    if (gap > max_gap || gap <= std::chrono::steady_clock::duration::zero()) return false;
    const double a = std::chrono::duration<double>(timestamp - lo->timestamp).count() /
                     std::chrono::duration<double>(gap).count();
    pose.timestamp = timestamp;
    pose.yaw = lo->yaw + a * std::remainder(hi->yaw - lo->yaw, 2 * M_PI);
    pose.pitch = lo->pitch + a * std::remainder(hi->pitch - lo->pitch, 2 * M_PI);
    pose.bracket_ms = std::chrono::duration<double, std::milli>(gap).count();
  }
  pose.yaw = imu_yaw_sign_ * pose.yaw + imu_yaw_offset_rad_;
  pose.pitch = imu_pitch_sign_ * pose.pitch + imu_pitch_offset_rad_;
  return true;
}

bool XucSender::imu_stable_at(
  std::chrono::steady_clock::time_point timestamp,
  std::chrono::milliseconds window, double max_range_rad) const
{
  if (window.count() <= 0 || !std::isfinite(max_range_rad) || max_range_rad <= 0) return false;
  std::lock_guard<std::mutex> lock(imu_history_mutex_);
  const auto begin = timestamp - window;
  if (imu_history_.empty() || imu_history_.front().timestamp > begin ||
      imu_history_.back().timestamp < timestamp) return false;
  auto it = std::lower_bound(imu_history_.begin(), imu_history_.end(), begin,
    [](const ImuPose & sample, auto t) { return sample.timestamp < t; });
  if (it != imu_history_.begin()) --it;
  const double y0 = it->yaw, p0 = it->pitch;
  double ymin = 0, ymax = 0, pmin = 0, pmax = 0;
  auto previous = it->timestamp;
  for (; it != imu_history_.end(); ++it) {
    if (it->timestamp - previous > std::chrono::milliseconds(30)) return false;
    const double y = std::remainder(it->yaw - y0, 2 * M_PI);
    const double p = std::remainder(it->pitch - p0, 2 * M_PI);
    ymin = std::min(ymin, y); ymax = std::max(ymax, y);
    pmin = std::min(pmin, p); pmax = std::max(pmax, p);
    previous = it->timestamp;
    if (it->timestamp >= timestamp) break;
  }
  return std::abs(imu_yaw_sign_) * (ymax - ymin) <= max_range_rad &&
         std::abs(imu_pitch_sign_) * (pmax - pmin) <= max_range_rad;
}

XucSender::XucSender(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  if (yaml["xuc_enable"]) enabled_ = yaml["xuc_enable"].as<bool>();
  if (!enabled_) {
    tools::logger()->info("[XUC] disabled (xuc_enable=false)");
    return;
  }

  std::string port = "/dev/ttyUSB0";
  uint32_t baud = 460800;
  if (yaml["xuc_serial_port"]) port = yaml["xuc_serial_port"].as<std::string>();
  if (yaml["xuc_serial_baud"]) baud = yaml["xuc_serial_baud"].as<uint32_t>();
  const std::string imu_angle_unit = yaml["xuc_imu_angle_unit"]
    ? yaml["xuc_imu_angle_unit"].as<std::string>() : "rad";
  if (imu_angle_unit == "deg") {
    imu_angle_to_rad_ = M_PI / 180.0;
  } else if (imu_angle_unit != "rad") {
    throw std::invalid_argument("xuc_imu_angle_unit must be rad or deg");
  }

  // 命令方向的线上单位。电控 2026-10-12 固件（new_infantry @ 73ea451）的 xuc.h
  // 明确把 yaw/pitch 标注为 deg，且 RC.cpp 直接与 GetAngleYaw()（度）相减；
  // 上位机内部一律弧度，因此默认按度发送。缺失该键时默认 deg，与当前固件一致。
  const std::string command_angle_unit = yaml["xuc_command_angle_unit"]
    ? yaml["xuc_command_angle_unit"].as<std::string>() : "deg";
  if (command_angle_unit == "deg") {
    command_rad_to_wire_ = 180.0 / M_PI;
  } else if (command_angle_unit == "rad") {
    command_rad_to_wire_ = 1.0;
  } else {
    throw std::invalid_argument("xuc_command_angle_unit must be rad or deg");
  }
  if (yaml["xuc_yaw_sign"]) yaw_sign_ = yaml["xuc_yaw_sign"].as<double>();
  if (yaml["xuc_pitch_sign"]) pitch_sign_ = yaml["xuc_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_sign"]) imu_yaw_sign_ = yaml["xuc_imu_yaw_sign"].as<double>();
  if (yaml["xuc_imu_pitch_sign"]) imu_pitch_sign_ = yaml["xuc_imu_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_offset_rad"]) {
    imu_yaw_offset_rad_ = yaml["xuc_imu_yaw_offset_rad"].as<double>();
  }
  if (yaml["xuc_imu_pitch_offset_rad"]) {
    imu_pitch_offset_rad_ = yaml["xuc_imu_pitch_offset_rad"].as<double>();
  }
  if (yaml["xuc_allow_control"]) allow_control_ = yaml["xuc_allow_control"].as<bool>();
  if (yaml["xuc_allow_shoot"]) allow_shoot_ = yaml["xuc_allow_shoot"].as<bool>();
  if (yaml["xuc_require_feedback_for_control"]) {
    require_feedback_for_control_ = yaml["xuc_require_feedback_for_control"].as<bool>();
  }
  if (yaml["xuc_require_auto_mode_for_control"]) {
    require_auto_mode_for_control_ = yaml["xuc_require_auto_mode_for_control"].as<bool>();
  }
  if (yaml["xuc_lock_pitch_for_control"]) {
    lock_pitch_for_control_ = yaml["xuc_lock_pitch_for_control"].as<bool>();
  }
  if (yaml["xuc_max_yaw_excursion_rad"]) {
    max_yaw_excursion_rad_ =
      std::max(0.0, yaml["xuc_max_yaw_excursion_rad"].as<double>());
  }
  if (yaml["xuc_feeder_active_rpm_threshold"]) {
    feeder_active_rpm_threshold_ = std::clamp(
      yaml["xuc_feeder_active_rpm_threshold"].as<double>(), 50.0, 3000.0);
  }

  try {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(timeout);
    serial_.open();
    // 启动接收线程：解析新版 TxPacket_TJ
    recv_thread_ = std::thread([this] { recv_loop(); });
    tools::logger()->info(
      "[XUC] serial opened {} @ {} baud, command_unit={}, imu_wire_unit={}, imu_output=rad, yaw_sign={}, pitch_sign={}, "
      "imu_yaw_sign={}, imu_pitch_sign={}, allow_control={}, allow_shoot={}, "
      "require_feedback={}, require_auto_mode={}, max_yaw_excursion_deg={:.2f}, "
      "lock_pitch={}",
      port, baud, command_angle_unit, imu_angle_unit, yaw_sign_, pitch_sign_, imu_yaw_sign_, imu_pitch_sign_, allow_control_,
      allow_shoot_, require_feedback_for_control_, require_auto_mode_for_control_,
      max_yaw_excursion_rad_ * 180.0 / M_PI, lock_pitch_for_control_);
    if (!allow_control_) {
      tools::logger()->warn("[XUC][SAFE] control output is forced to zero");
    }
    if (!allow_shoot_) {
      tools::logger()->warn("[XUC][SAFE] shoot output is forced to zero");
    }
  } catch (const serial::IOException & e) {
    enabled_ = false;
    tools::logger()->warn("[XUC] failed to open serial port {}: {}", port, e.what());
  }
}

XucSender::~XucSender()
{
  recv_quit_.store(true, std::memory_order_relaxed);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

bool XucSender::rx_valid() const
{
  if (!rx_valid_.load(std::memory_order_relaxed)) return false;

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_ns = last_rx_ns_.load(std::memory_order_relaxed);

  return last_ns > 0 && now_ns - last_ns <= RX_TIMEOUT_NS;
}

XucSender::FeederFeedback XucSender::feeder_feedback() const
{
  std::lock_guard<std::mutex> lock(feeder_feedback_mutex_);
  return feeder_feedback_;
}

void XucSender::send(const Command & cmd)
{
  if (!enabled_ || !serial_.isOpen()) return;

  XucRxPacket pkt{};
  pkt.head[0] = 'S';
  pkt.head[1] = 'P';
  const bool link_ready = control_ready();
  const bool control_requested = link_ready && cmd.control;
  pkt.control = control_requested ? 1 : 0;
  pkt.shoot = (control_requested && allow_shoot_ && cmd.shoot) ? 1 : 0;

  double target_yaw_raw = yaw_sign_ * cmd.yaw;
  double target_pitch_raw = pitch_sign_ * cmd.pitch;
  const bool bounded_session =
    max_yaw_excursion_rad_ > 0.0 || lock_pitch_for_control_;

  // This diagnostic profile intentionally permits only one yaw origin per
  // process.  Detector dropouts, link recovery, or a chattering feedback mode
  // must never move that origin.  A new physical test requires stopping and
  // restarting the process, which is also the safest explicit re-arm boundary.
  if (control_requested && bounded_session) {
    if (!control_session_active_) {
      control_yaw_anchor_raw_ =
        static_cast<double>(imu_yaw_.load(std::memory_order_relaxed));
      control_session_active_ = true;
      tools::logger()->warn(
        "[XUC][BOUNDED] control session anchored at raw yaw {:.2f} deg",
        control_yaw_anchor_raw_ * 180.0 / M_PI);
    }

    if (max_yaw_excursion_rad_ > 0.0) {
      double delta = target_yaw_raw - control_yaw_anchor_raw_;
      delta = std::remainder(delta, 2.0 * M_PI);
      delta = std::clamp(delta, -max_yaw_excursion_rad_, max_yaw_excursion_rad_);
      target_yaw_raw = control_yaw_anchor_raw_ + delta;
    }
    if (lock_pitch_for_control_) {
      target_pitch_raw =
        static_cast<double>(imu_pitch_.load(std::memory_order_relaxed));
    }
  } else if (!bounded_session) {
    control_session_active_ = false;
  }

  // 内部一律弧度；写帧前按 xuc_command_angle_unit 转成线上单位。
  // 电控 2026-10-12 固件（73ea451）期望度，直接与 GetAngleYaw()（度）相减。
  pkt.yaw = static_cast<float>(target_yaw_raw * command_rad_to_wire_);
  pkt.pitch = static_cast<float>(target_pitch_raw * command_rad_to_wire_);

  // CRC16 对前 12 字节计算，低字节在前
  uint16_t crc = tools::get_crc16(reinterpret_cast<const uint8_t *>(&pkt),
                                  offsetof(XucRxPacket, crc16));
  uint8_t buf[sizeof(XucRxPacket)]{};
  std::memcpy(buf, &pkt, offsetof(XucRxPacket, crc16));
  buf[offsetof(XucRxPacket, crc16)] = static_cast<uint8_t>(crc & 0xff);
  buf[offsetof(XucRxPacket, crc16) + 1] = static_cast<uint8_t>((crc >> 8) & 0xff);

  try {
    if (pkt.shoot != 0) {
      const auto tx_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      last_shoot_tx_ns_.store(tx_ns, std::memory_order_release);
    }
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception & e) {
    tools::logger()->warn("[XUC] write failed: {}", e.what());
  }
}

// 接收线程：读串口字节 → 扫描 'S''P' + CRC16 校验 → 解析 TxPacket_TJ（20 字节）
void XucSender::recv_loop()
{
  uint8_t scratch[sizeof(XucTxPacket)];
  std::vector<uint8_t> buf;
  buf.reserve(sizeof(XucTxPacket) * 2);

  while (!recv_quit_.load(std::memory_order_relaxed)) {
    size_t n = 0;
    try {
      // The parser leaves less than one packet in buf. Request only its
      // missing bytes: read(64) otherwise waits until timeout even after a
      // complete 20-byte feedback packet has arrived, batching timestamps.
      n = serial_.read(scratch, sizeof(XucTxPacket) - buf.size());
    } catch (const std::exception &) {
      break;
    }
    if (n == 0) continue;  // 超时无数据
    rx_bytes_.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
    buf.insert(buf.end(), scratch, scratch + n);

    size_t i = 0;
    while (i + sizeof(XucTxPacket) <= buf.size()) {
      if (buf[i] == 'S' && buf[i + 1] == 'P') {
        header_candidates_.fetch_add(1, std::memory_order_relaxed);
        uint16_t crc = tools::get_crc16(&buf[i], offsetof(XucTxPacket, crc16));
        if (static_cast<uint8_t>(crc & 0xff) == buf[i + offsetof(XucTxPacket, crc16)] &&
            static_cast<uint8_t>((crc >> 8) & 0xff) == buf[i + offsetof(XucTxPacket, crc16) + 1]) {
          XucTxPacket pkt{};
          std::memcpy(&pkt, &buf[i], sizeof(XucTxPacket));

          const bool fields_valid =
            pkt.mode <= 3 &&
            std::isfinite(pkt.bullet_speed) &&
            std::isfinite(pkt.imu_pitch) &&
            std::isfinite(pkt.imu_yaw);

          if (fields_valid) {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
            const bool feeder_active =
              std::abs(static_cast<double>(pkt.bullet_speed)) >=
              feeder_active_rpm_threshold_;
            {
              std::lock_guard<std::mutex> lock(feeder_feedback_mutex_);
              if (feeder_active != feeder_feedback_.active) {
                feeder_feedback_.active = feeder_active;
                feeder_feedback_.transition_time_ns = now_ns;
                feeder_feedback_.rpm = static_cast<double>(pkt.bullet_speed);
                ++feeder_feedback_.transition_sequence;
              }
            }
            mode_.store(pkt.mode, std::memory_order_relaxed);
            robot_id_.store(pkt.robot_id, std::memory_order_relaxed);
            bullet_speed_.store(pkt.bullet_speed, std::memory_order_relaxed);
            bullet_count_.store(pkt.bullet_count, std::memory_order_relaxed);
            imu_pitch_.store(static_cast<float>(pkt.imu_pitch * imu_angle_to_rad_), std::memory_order_relaxed);
            imu_yaw_.store(static_cast<float>(pkt.imu_yaw * imu_angle_to_rad_), std::memory_order_relaxed);

            {
              std::lock_guard<std::mutex> lock(imu_history_mutex_);
              imu_history_.push_back({
                std::chrono::steady_clock::time_point(std::chrono::nanoseconds(now_ns)),
                pkt.imu_yaw * imu_angle_to_rad_, pkt.imu_pitch * imu_angle_to_rad_, 0.0});
              while (imu_history_.size() > 1 &&
                     now_ns - std::chrono::duration_cast<std::chrono::nanoseconds>(
                       imu_history_.front().timestamp.time_since_epoch()).count() > 2000000000LL) {
                imu_history_.pop_front();
              }
            }
            imu_history_changed_.notify_all();
            last_rx_ns_.store(now_ns, std::memory_order_relaxed);
            valid_packets_.fetch_add(1, std::memory_order_relaxed);
            rx_valid_.store(true, std::memory_order_relaxed);
          } else {
            field_errors_.fetch_add(1, std::memory_order_relaxed);
          }

          i += sizeof(XucTxPacket);
          continue;
        }
        crc_errors_.fetch_add(1, std::memory_order_relaxed);
      }
      ++i;
    }
    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(i));
    // 防止残留缓冲无限增长（最多保留一个半包，供后续拼包）
    if (buf.size() > sizeof(XucTxPacket) + 2) {
      buf.erase(buf.begin(),
                buf.begin() + static_cast<std::ptrdiff_t>(buf.size() - sizeof(XucTxPacket) - 2));
    }
  }

  rx_valid_.store(false, std::memory_order_relaxed);
}

}  // namespace io
