#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

void Aimer::set_delay_adjustment(double adjustment_s)
{
  delay_adjustment_time_ = std::clamp(adjustment_s, -0.050, 0.050);
}

AimPredictionModel Aimer::prediction_model(const Target & target, AimGeometry requested) const
{
  AimPredictionModel model;
  const auto & raw = target.ekf().x;
  const auto & covariance = target.ekf().P;
  const bool raw_healthy = raw.size() == 11 && raw.allFinite() &&
    covariance.rows() == 11 && covariance.cols() == 11 && covariance.allFinite() &&
    raw[8] > 0.05 && raw[8] < 0.5 &&
    raw[8] + raw[9] > 0.05 && raw[8] + raw[9] < 0.5;
  // Resolve once at the image boundary. In particular, do not let YawESKF's
  // rate decide the delay or trust gate for raw-EKF geometry.
  if (requested == AimGeometry::whole_ekf && target.use_whole_ekf_geometry(decision_speed_)) {
    model.geometry = AimGeometry::whole_ekf;
    model.state = raw;
  } else {
    model.state = target.ekf_x();
  }
  model.finite = raw_healthy && model.state.size() == 11 && model.state.allFinite();
  if (!model.finite) return model;
  model.angular_velocity_rad_s = model.state[7];
  // convergened() latches a flag, so query a copy instead of modifying Tracker.
  auto target_copy = target;
  // Geometry ownership does not replace the independent motion-health gate.
  // Tracker convergence only counts updates and plausible radii; it does not
  // establish that a newly acquired raw-EKF rotation estimate is trustworthy.
  model.trusted = target_copy.convergened() && target.yaw_eskf_trusted();
  return model;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, AimPredictionTrace * trace, AimGeometry geometry,
  const FixedCenterAimModel * fixed_center)
{
  if (trace) *trace = AimPredictionTrace{};
  debug_aim_point = {false, Eigen::Vector4d::Zero()};
  const auto fail = [&](AimPredictionFailure failure) -> io::Command {
    if (trace) { trace->valid = false; trace->failure = failure; }
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  };
  if (targets.empty()) return fail(AimPredictionFailure::no_target);
  auto target = targets.front();

  const auto model = prediction_model(target, geometry);
  if (!model.finite) return fail(AimPredictionFailure::non_finite);
  const bool whole_ekf_geometry = model.geometry == AimGeometry::whole_ekf;
  const auto image_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    timestamp.time_since_epoch()).count();
  const auto checked_ns = to_now ? std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count() : image_ns;
  if (fixed_center && (!whole_ekf_geometry || target.armor_num() != 4 || !target.jumped ||
      fixed_center->context.target_id != static_cast<int>(target.name) ||
      !fixed_center->valid_at(checked_ns) || fixed_center->model.reference_ns > image_ns))
    return fail(AimPredictionFailure::stale_model);
  const double prediction_rate = fixed_center
    ? fixed_center->model.state.omega_rad_s : model.angular_velocity_rad_s;
  if (trace) {
    trace->geometry = model.geometry;
    trace->whole_ekf_geometry = whole_ekf_geometry;
    trace->angular_velocity_rad_s = prediction_rate;
    trace->fixed_center = fixed_center != nullptr;
    if (fixed_center) {
      trace->fixed_center_epoch = fixed_center->epoch;
      trace->fixed_center_revision = fixed_center->revision;
      trace->model_reference_ns = fixed_center->model.reference_ns;
      trace->model_valid_until_ns = fixed_center->valid_until_ns;
    }
  }
  double delay_time =
    (std::abs(prediction_rate) > decision_speed_
       ? high_speed_delay_time_
       : low_speed_delay_time_) + delay_adjustment_time_;

  // Low-speed friction-wheel benches can be well below the competition-speed
  // default.  Preserve the fallback only for missing/invalid telemetry so the
  // trajectory and flight-time predictor can use an explicitly calibrated value.
  if (!std::isfinite(bullet_speed) || bullet_speed < 5 || bullet_speed > 35) {
    bullet_speed = 23;
  }
  if (trace) {
    trace->delay_s = delay_time;
    trace->speed_mps = bullet_speed;
  }

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  if (trace) trace->base_prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    future.time_since_epoch()).count();
  // For high-speed whole-body prediction each solve owns one physical face.
  // An invisible face at the base time may become selectable by impact time.
  const bool fixed_faces = fixed_center || (whole_ekf_geometry && target.jumped);
  struct Candidate
  {
    AimPredictionTrace trace;
    AimPoint point{false, Eigen::Vector4d::Zero()};
    double pitch = 0.0;
    AimPredictionFailure failure = AimPredictionFailure::none;
  };
  const auto solve = [&](int fixed_id) {
    Candidate result;
    if (trace) result.trace = *trace;
    result.trace.fixed_face = fixed_id >= 0;
    const auto point_at = [&](const Target & predicted, std::int64_t time_ns) -> AimPoint {
      if (fixed_center) {
        const auto face = fixed_center->face_at(time_ns, fixed_id);
        result.trace.face_id = fixed_id;
        return {true, Eigen::Vector4d(face[0], face[1], face[2], face[3])};
      }
      if (fixed_id < 0)
        return choose_aim_point(predicted, whole_ekf_geometry, &result.trace.face_id);
      const auto faces = predicted.whole_ekf_armor_xyza_list();
      if (fixed_id >= static_cast<int>(faces.size()))
        return {false, Eigen::Vector4d::Zero()};
      result.trace.face_id = fixed_id;
      return {true, faces[fixed_id]};
    };
    result.point = point_at(target, std::chrono::duration_cast<std::chrono::nanoseconds>(
      future.time_since_epoch()).count());
    if (!result.point.valid) {
      result.failure = AimPredictionFailure::initial_selection;
      return result;
    }
    const auto trajectory_at = [&](const AimPoint & point, double & flight, double & pitch) {
      if (!point.xyza.allFinite()) return AimPredictionFailure::non_finite;
      tools::Trajectory trajectory(
        bullet_speed, point.xyza.head<2>().norm(), point.xyza[2]);
      if (trajectory.unsolvable) return AimPredictionFailure::trajectory;
      if (!std::isfinite(trajectory.fly_time) || !std::isfinite(trajectory.pitch) ||
          trajectory.fly_time < 0.0) return AimPredictionFailure::non_finite;
      flight = trajectory.fly_time;
      pitch = trajectory.pitch;
      return AimPredictionFailure::none;
    };
    double previous_flight = 0.0;
    result.failure = trajectory_at(result.point, previous_flight, result.pitch);
    if (result.failure != AimPredictionFailure::none) return result;
    for (int iter = 0; iter < 10; ++iter) {
      const auto predict_time = future + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(previous_flight));
      auto predicted = target;
      predicted.predict(predict_time);
      result.trace.prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        predict_time.time_since_epoch()).count();
      result.point = point_at(predicted, result.trace.prediction_ns);
      result.trace.flight_time_used_s = previous_flight;
      result.trace.iterations = iter + 1;
      result.trace.aim_xyza = result.point.xyza;
      if (!result.point.valid) {
        result.failure = AimPredictionFailure::iterative_selection;
        return result;
      }
      double next_flight = 0.0;
      result.failure = trajectory_at(result.point, next_flight, result.pitch);
      if (result.failure != AimPredictionFailure::none) return result;
      result.trace.flight_time_s = next_flight;
      if (std::abs(next_flight - previous_flight) < 0.001) {
        result.trace.converged = true;
        auto selection_state = predicted.ekf().x;
        if (fixed_center) {
          selection_state[0] = fixed_center->model.state.center_x;
          selection_state[2] = fixed_center->model.state.center_y;
          selection_state[7] = fixed_center->model.state.omega_rad_s;
        }
        if (fixed_id >= 0 &&
            !rotation_face_selectable(predicted, selection_state, result.point.xyza))
          result.failure = AimPredictionFailure::final_selection;
        return result;
      }
      previous_flight = next_flight;
    }
    result.failure = AimPredictionFailure::not_converged;
    return result;
  };

  Candidate selected;
  if (fixed_faces) {
    std::optional<Candidate> unconverged, final_ineligible;
    for (int face = 0; face < target.armor_num(); ++face) {
      auto candidate = solve(face);
      if (candidate.failure == AimPredictionFailure::not_converged && !unconverged)
        unconverged = candidate;
      if (candidate.failure == AimPredictionFailure::final_selection && !final_ineligible)
        final_ineligible = candidate;
      selected = std::move(candidate);
      if (selected.failure == AimPredictionFailure::none) break;
    }
    if (selected.failure != AimPredictionFailure::none) {
      // Keep diagnostics from a candidate that actually has the reported
      // failure. An unconverged face cannot be ruled out by final selection.
      if (unconverged) selected = std::move(*unconverged);
      else if (final_ineligible) selected = std::move(*final_ineligible);
      if (trace) *trace = selected.trace;
      return fail(selected.failure);
    }
  } else {
    selected = solve(-1);
    if (trace) *trace = selected.trace;
    if (selected.failure != AimPredictionFailure::none) return fail(selected.failure);
  }
  debug_aim_point = selected.point;
  if (trace) *trace = selected.trace;

  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  if (fixed_center && to_now && !fixed_center->valid_at(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()))
    return fail(AimPredictionFailure::stale_model);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(selected.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) return fail(AimPredictionFailure::non_finite);
  if (trace) {
    trace->valid = std::isfinite(yaw) && std::isfinite(pitch);
  }
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now, AimPredictionTrace * trace, AimGeometry geometry,
  const FixedCenterAimModel * fixed_center)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now, trace, geometry, fixed_center);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;
  if (!std::isfinite(command.yaw) || !std::isfinite(command.pitch)) {
    if (trace) { trace->valid = false; trace->failure = AimPredictionFailure::non_finite; }
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  return command;
}

AimPoint Aimer::choose_aim_point(
  const Target & target, bool whole_ekf_geometry, int * face_id)
{
  Eigen::VectorXd ekf_x = whole_ekf_geometry ? target.ekf().x : target.ekf_x();
  const bool prefer_direct_motion =
    !whole_ekf_geometry && std::abs(ekf_x[7]) <= decision_speed_ &&
    target.name != ArmorName::outpost;
  std::vector<Eigen::Vector4d> armor_xyza_list =
    whole_ekf_geometry ? target.whole_ekf_armor_xyza_list()
                       : target.armor_xyza_list(prefer_direct_motion);
  auto armor_num = armor_xyza_list.size();
  const auto select = [&](bool valid, int id) -> AimPoint {
    if (face_id) *face_id = valid ? id : -1;
    return {valid, armor_xyza_list[id]};
  };
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return select(true, 0);

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (prefer_direct_motion) {
    // At low rotation rate the currently observed physical plate is the most
    // reliable target. Prefer its fresh CV track instead of re-selecting among
    // several model faces at every ballistic iteration; the latter produced
    // red-box jumps and corresponding gimbal reversals near a face hand-over.
    if (target.last_id >= 0 && target.last_id < static_cast<int>(armor_num) &&
        target.armor_motion_trusted(target.last_id) &&
        std::abs(delta_angle_list[target.last_id]) <= 75.0 / 57.3) {
      lock_id_ = target.last_id;
      return select(true, target.last_id);
    }

    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return select(false, 0);
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return select(true, static_cast<int>(lock_id_));
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return select(true, id_list[0]);
  }

  // 小陀螺时选板（2026-10-19 借鉴 Fujian_AutoAim 26.10.18）：
  // 一侧的装甲板不断出现、另一侧不断消失，前者被打中的概率更高。
  // 所以按 ω 的符号挑"正在进入视野"的那块，而不是挑 delta_angle 最接近 0 的。
  // choose_aim_point 的 target 已预测到弹着时刻，delta_angle 就是弹着时刻的角度。
  for (int i = 0; i < armor_num; i++) {
    if (rotation_face_selectable(target, ekf_x, armor_xyza_list[i]))
      return select(true, i);
  }

  return select(false, 0);
}

bool Aimer::rotation_face_selectable(
  const Target & target, const Eigen::VectorXd & state, const Eigen::Vector4d & face) const
{
  if (state.size() != 11 || !state.allFinite() || !face.allFinite()) return false;
  const double coming = target.name == ArmorName::outpost ? 70.0 / 57.3 : comming_angle_;
  const double leaving = target.name == ArmorName::outpost ? 30.0 / 57.3 : leaving_angle_;
  const double delta = tools::limit_rad(face[3] - std::atan2(state[2], state[0]));
  return std::abs(delta) <= coming &&
    ((state[7] > 0.0 && delta < leaving) || (state[7] < 0.0 && delta > -leaving));
}

}  // namespace auto_aim
