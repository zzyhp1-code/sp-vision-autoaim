#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <functional>
#include <limits>
#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ROS2 headers (仅在 ROS2 可用时编译)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth},
  association_max_cost_(2.0),
  batch_max_observations_(4),
  batch_min_omega_rad_s_(3.0),
  primary_detection_index_(-1),
  primary_face_id_(-1)
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  if (yaml["tracker_association_max_cost"]) {
    association_max_cost_ = std::clamp(
      yaml["tracker_association_max_cost"].as<double>(), 0.05, 20.0);
  }
  if (yaml["tracker_batch_max_observations"]) {
    batch_max_observations_ = std::clamp(
      yaml["tracker_batch_max_observations"].as<int>(), 1, 4);
  }
  if (yaml["tracker_batch_min_omega_rad_s"]) {
    // Upper bound raised from 20 to 1000 so a config can disable joint updates
    // outright (setting it above any physically reachable yaw rate).  The old
    // ceiling of 20 silently capped such a value at 20.
    batch_min_omega_rad_s_ = std::clamp(
      yaml["tracker_batch_min_omega_rad_s"].as<double>(), 0.0, 1000.0);
  }
  if (yaml["tracker_target_switch_min_frames"]) {
    target_switch_min_frames_ = std::clamp(
      yaml["tracker_target_switch_min_frames"].as<int>(), 1, 30);
  }
  if (yaml["yaw_eskf_angular_jerk_variance"]) {
    yaw_eskf_config_.angular_jerk_variance = std::clamp(
      yaml["yaw_eskf_angular_jerk_variance"].as<double>(), 0.1, 5000.0);
  }
  if (yaml["yaw_eskf_measurement_floor_deg"]) {
    const double sigma_rad = std::clamp(
      yaml["yaw_eskf_measurement_floor_deg"].as<double>(), 0.1, 20.0) *
      M_PI / 180.0;
    yaw_eskf_config_.measurement_variance_floor = sigma_rad * sigma_rad;
  }
  if (yaml["yaw_eskf_nis_reject_threshold"]) {
    yaw_eskf_config_.nis_reject_threshold = std::clamp(
      yaml["yaw_eskf_nis_reject_threshold"].as<double>(), 1.0, 100.0);
  }
  if (yaml["yaw_eskf_min_trusted_updates"]) {
    yaw_eskf_config_.min_trusted_updates = std::clamp(
      yaml["yaw_eskf_min_trusted_updates"].as<int>(), 2, 100);
  }
  if (yaml["yaw_eskf_trusted_rate_std_rad_s"]) {
    const double stddev = std::clamp(
      yaml["yaw_eskf_trusted_rate_std_rad_s"].as<double>(), 0.1, 10.0);
    yaw_eskf_config_.trusted_rate_variance = stddev * stddev;
  }
  if (yaml["yaw_eskf_max_trusted_abs_rate_rad_s"]) {
    yaw_eskf_config_.max_trusted_abs_rate_rad_s = std::clamp(
      yaml["yaw_eskf_max_trusted_abs_rate_rad_s"].as<double>(), 0.5, 30.0);
  }
  if (yaml["yaw_eskf_max_trusted_abs_acceleration_rad_s2"]) {
    yaw_eskf_config_.max_trusted_abs_acceleration_rad_s2 = std::clamp(
      yaml["yaw_eskf_max_trusted_abs_acceleration_rad_s2"].as<double>(),
      1.0, 500.0);
  }
  if (yaml["whole_state_iekf_enabled"])
    iterated_update_config_.enabled = yaml["whole_state_iekf_enabled"].as<bool>();
  if (yaml["whole_state_iekf_max_iterations"])
    iterated_update_config_.max_iterations = std::clamp(
      yaml["whole_state_iekf_max_iterations"].as<int>(), 1, 10);
  if (yaml["whole_state_iekf_stop_threshold"])
    iterated_update_config_.stop_threshold = std::clamp(
      yaml["whole_state_iekf_stop_threshold"].as<double>(), 1e-8, 0.1);
  if (yaml["whole_state_iekf_huber_threshold"])
    iterated_update_config_.huber_threshold = std::clamp(
      yaml["whole_state_iekf_huber_threshold"].as<double>(), 0.5, 10.0);
  // 发散确认帧数：目标快速移动时单帧 EKF 半径抖动不应立即判丢。
  if (yaml["tracker_diverge_confirm_frames"]) {
    diverge_confirm_frames_ = std::clamp(
      yaml["tracker_diverge_confirm_frames"].as<int>(), 1, 100);
  }
}

std::string Tracker::state() const { return state_; }

void Tracker::reset()
{
  target_ = Target{};
  state_ = pre_state_ = "lost";
  detect_count_ = 0;
  temp_lost_count_ = 0;
  max_temp_lost_count_ = normal_temp_lost_count_;
  diverge_streak_ = 0;
  switch_candidate_name_ = ArmorName::not_armor;
  switch_candidate_frames_ = 0;
  omni_target_priority_ = ArmorPriority::fifth;
  last_timestamp_ = std::chrono::steady_clock::time_point{};
  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;
}

const std::vector<Tracker::ArmorAssociation> & Tracker::last_associations() const
{
  return last_associations_;
}

int Tracker::primary_detection_index() const { return primary_detection_index_; }

int Tracker::primary_face_id() const { return primary_face_id_; }

void Tracker::solve_observations(std::list<Armor> & armors)
{
  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;
  const auto detection_count = armors.size();
  armors.remove_if([this](Armor & armor) { return !solver_.solve(armor); });
  if (armors.size() != detection_count) {
    tools::logger()->debug(
      "[Tracker][PNP] rejected={} detections={}", detection_count - armors.size(), detection_count);
  }
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.2) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
  solve_observations(armors);

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  } else if (state_ == "tracking" && should_switch_target(armors)) {
    found = set_target(armors, t);
    tools::logger()->info(
      "[Tracker] stable target switch to {} after {} frames",
      ARMOR_NAMES[target_.name], target_switch_min_frames_);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测（连续 diverge_confirm_frames_ 帧才确认丢失，避免单帧抖动跟丢）
  if (state_ != "lost" && target_.diverged()) {
    ++diverge_streak_;
    if (diverge_streak_ >= diverge_confirm_frames_) {
      tools::logger()->debug(
        "[Tracker] Target diverged! ({}/{} consecutive frames)",
        diverge_streak_, diverge_confirm_frames_);
      state_ = "lost";
      diverge_streak_ = 0;
      return {};
    }
    tools::logger()->debug(
      "[Tracker] Target diverged {}/{}; holding target", diverge_streak_,
      diverge_confirm_frames_);
  } else if (state_ != "lost") {
    diverge_streak_ = 0;   // 恢复正常即清零
  }
  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    // Print detailed EKF diagnostics to help debug convergence issues
    try {
      auto & ekf = target_.ekf();
      double recent_rate = ekf.data.at("recent_nis_failures");
      double nis = ekf.data.at("nis");
      double nees = ekf.data.at("nees");
      double ry = ekf.data.at("residual_yaw");
      double rp = ekf.data.at("residual_pitch");
      double rd = ekf.data.at("residual_distance");
      double ra = ekf.data.at("residual_angle");

      tools::logger()->debug(
        "[Target] Bad Converge Found! recent_rate={:.3f} last_nis={:.3f} nees={:.3f} resid=(yaw={:.3f},pitch={:.3f},dist={:.3f},ang={:.3f})",
        recent_rate, nis, nees, ry, rp, rd, ra);
    } catch (const std::exception & e) {
      tools::logger()->debug("[Target] Bad Converge Found! (failed to read EKF data: {})", e.what());
    }

    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.2) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  solve_observations(armors);

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测（连续 diverge_confirm_frames_ 帧才确认丢失，避免单帧抖动跟丢）
  if (state_ != "lost" && target_.diverged()) {
    ++diverge_streak_;
    if (diverge_streak_ >= diverge_confirm_frames_) {
      tools::logger()->debug(
        "[Tracker] Target diverged! ({}/{} consecutive frames)",
        diverge_streak_, diverge_confirm_frames_);
      state_ = "lost";
      diverge_streak_ = 0;
      return {switch_target, {}};  // 返回switch_target和空的targets
    }
    tools::logger()->debug(
      "[Tracker] Target diverged {}/{}; holding target", diverge_streak_,
      diverge_confirm_frames_);
  } else if (state_ != "lost") {
    diverge_streak_ = 0;
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;
  if (armors.empty()) return false;

  Armor * selected = select_target(armors);
  if (selected == nullptr) return false;
  auto & armor = *selected;
  int selected_detection_index = 0;
  for (auto it = armors.begin(); it != armors.end() && &*it != selected; ++it)
    ++selected_detection_index;
  switch_candidate_name_ = ArmorName::not_armor;
  switch_candidate_frames_ = 0;

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(
      armor, t, 0.2, 2, P0_dig, yaw_eskf_config_, iterated_update_config_);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.2765, 3, P0_dig, yaw_eskf_config_, iterated_update_config_);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.3205, 3, P0_dig, yaw_eskf_config_, iterated_update_config_);
  }

  else {
    // x = [cx, vx, cy, vy, cz, vz, yaw, yaw_rate, r, l, h]
    //
    // Priors for the three geometric states (r, l, h) are tightened together
    // because none of them is well observable from the on-axis face alone.  The
    // observation is p = c - r*[cos, sin], so shifting the centre along that
    // direction while adding the same amount to r leaves the image barely
    // changed; with the old variance of 1.0 (std 1 m) the estimate was free to
    // trade r against position and the field log showed r reaching -0.558 and
    // 0.653 m (a negative radius is meaningless).  `diverged()` then fired on
    // r < 0.05 and reset an otherwise working track.
    //
    //   l = r2 - r1 : measured chassis value 0.04 m, prior std 5 cm
    //   h = z2 - z1 : measured 0.00 m, prior std 5 cm
    //   r           : 0.24..0.28 m measured (centre to armour), prior std 3 cm
    //
    // The radius itself is passed to the constructor (0.26 m) so that the centre
    // is derived from it consistently -- changing r after construction would
    // move every predicted face.
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.0009, 0.0025, 0.0025}};
    target_ = Target(
      armor, t, 0.26, 4, P0_dig, yaw_eskf_config_, iterated_update_config_);
  }

  // Seed the geometric offsets from the measured chassis instead of leaving them
  // at zero.  The EKF cannot observe l/h from the on-axis face alone, so the
  // initial value is what the whole track is based on until an off-axis face
  // happens to be associated.  Starting from the real values keeps the predicted
  // faces 1/3 close to where they actually are, which is what lets the
  // association cost for those faces fall inside the gate.
  //
  // `l` is r2 - r1 with faces 1 and 3 on the +l radius.  From h_armor_xyz():
  // angle = yaw + i*90 deg and position = centre - r_i*[cos, sin], so face 1 is
  // always on the -y side and face 3 always on the +y side; `l` only changes how
  // far each sits from the centre, it cannot swap the two sides.  An earlier
  // attempt to use a negative l to "swap" the predictions came from a mistaken
  // reading of the per-face cost breakdown and has been reverted.
  target_.seed_geometry(0.04, 0.00);
  ++model_generation_;

  last_associations_.push_back({selected_detection_index, 0, 0.0});
  primary_detection_index_ = selected_detection_index;
  primary_face_id_ = 0;

  return true;
}

Armor * Tracker::select_target(std::list<Armor> & armors)
{
  Armor * best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  for (auto & armor : armors) {
    const double center_score = std::hypot(armor.center_norm.x, armor.center_norm.y);
    const double range_score = armor.xyz_in_world.norm() / 10.0;
    // Priority remains dominant.  Distance and image-center position break
    // ties, matching SHtech's fixed-target selection without target chatter.
    const double score = 10.0 * static_cast<int>(armor.priority) +
      0.65 * center_score + 0.35 * range_score -
      0.20 * std::clamp(armor.confidence, 0.0, 1.0);
    if (score < best_score) {
      best_score = score;
      best = &armor;
    }
  }
  return best;
}

bool Tracker::should_switch_target(const std::list<Armor> & armors)
{
  const auto candidate = std::find_if(
    armors.begin(), armors.end(), [this](const Armor & armor) {
      return armor.priority < target_.priority && armor.name != target_.name;
    });
  if (candidate == armors.end()) {
    switch_candidate_name_ = ArmorName::not_armor;
    switch_candidate_frames_ = 0;
    return false;
  }
  if (candidate->name != switch_candidate_name_) {
    switch_candidate_name_ = candidate->name;
    switch_candidate_frames_ = 1;
  } else {
    ++switch_candidate_frames_;
  }
  return switch_candidate_frames_ >= target_switch_min_frames_;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;

  // Associate in the image plane against every rigid-body face, then update
  // the EKF once with the best observation.  The old loop updated the same EKF
  // sequentially with every same-class detection in one frame.  During a face
  // handover that could pull body yaw toward two different armor ids at the
  // same timestamp and create a long/short phase-crossing pair.
  // Association is a measurement update: retain its established filtered
  // phase reference. Switching it to raw EKF phase based on an instantaneous
  // speed threshold adds a feedback discontinuity during reacquisition.
  // Aimer independently keeps one geometry throughout each future solve.
  const auto association_state = target_.ekf_x();
  const auto predicted_faces = target_.armor_xyza_list();
  struct Candidate
  {
    Armor * armor = nullptr;
    int detection_index = -1;
    int face_id = -1;
    double cost = std::numeric_limits<double>::infinity();
    double alignment_rad = std::numeric_limits<double>::infinity();
  };
  std::vector<Candidate> candidates;
  int detection_index = 0;
  // Diagnostic counters: how many detections were dropped before the cost gate
  // even ran, and the best rejected cost.  Without this it is impossible to tell
  // "the gate is too tight" from "the other detection is a different armor".
  int dropped_by_name = 0;
  int dropped_by_type = 0;
  int rejected_by_cost = 0;
  double best_rejected_cost = std::numeric_limits<double>::infinity();
  int best_rejected_det = -1;
  int best_rejected_face = -1;

  // Per-term breakdown of the best rejected candidate, so a systematic bias can
  // be attributed to the centre term (predicted position wrong) or the angle
  // term (predicted normal wrong) instead of just seeing the lumped cost.
  double best_detail_cost = std::numeric_limits<double>::infinity();
  int best_detail_det = -1;
  int best_detail_face = -1;
  double best_detail_center = 0.0;
  double best_detail_angle = 0.0;
  double best_detail_range = 0.0;
  double best_detail_conf = 0.0;
  double best_detail_px = 0.0;
  double best_detail_scale = 1.0;

  // Full detection x face cost matrix for the frame, so the whole picture is
  // visible at once instead of only the single best near-miss.  Each cell keeps
  // the lumped cost plus the signed angular difference, which is what
  // distinguishes "faces 1 and 3 swapped" from "body yaw is off by a constant".
  struct MatrixCell
  {
    bool valid = false;
    double cost = std::numeric_limits<double>::infinity();
    double angle_deg = 0.0;
    double pixels = 0.0;
    double range_m = 0.0;
  };
  // At most a handful of detections and 4 faces; fixed bounds keep this cheap.
  constexpr int kMaxDiagDet = 4;
  MatrixCell cost_matrix[kMaxDiagDet][4];
  double det_range[kMaxDiagDet] = {0, 0, 0, 0};
  double det_yaw_deg[kMaxDiagDet] = {0, 0, 0, 0};
  for (auto & armor : armors) {
    const int current_detection_index = detection_index++;
    if (armor.name != target_.name) {
      ++dropped_by_name;
      continue;
    }
    if (armor.type != target_.armor_type) {
      ++dropped_by_type;
      continue;
    }

    const double observed_range_m = std::max(0.05, armor.ypd_in_world[2]);
    const double detection_scale_px = std::max(
      30.0, std::sqrt(std::max(1.0, static_cast<double>(armor.box.area()))));
    cv::Point2f observed_center = armor.center;
    if (!armor.points.empty()) {
      observed_center = cv::Point2f(0.0F, 0.0F);
      for (const auto & point : armor.points) observed_center += point;
      observed_center *= 1.0F / static_cast<float>(armor.points.size());
    }

    if (current_detection_index < kMaxDiagDet) {
      det_range[current_detection_index] = observed_range_m;
      det_yaw_deg[current_detection_index] =
        armor.ypr_in_world[0] * 180.0 / M_PI;
    }

    for (std::size_t id = 0; id < predicted_faces.size(); ++id) {
      const auto & face = predicted_faces[id];
      const auto projected = solver_.reproject_armor(
        face.head(3), face[3], target_.armor_type, target_.name);
      if (projected.empty()) continue;

      cv::Point2f projected_center(0.0F, 0.0F);
      for (const auto & point : projected) projected_center += point;
      projected_center *= 1.0F / static_cast<float>(projected.size());

      const double center_error =
        cv::norm(observed_center - projected_center) / detection_scale_px;
      const double angle_error = std::abs(tools::limit_rad(
        armor.ypr_in_world[0] - face[3])) / (20.0 * CV_PI / 180.0);
      const double predicted_range_m = std::max(
        0.05, tools::xyz2ypd(face.head(3))[2]);
      const double range_error =
        std::abs(std::log(observed_range_m / predicted_range_m)) / 0.25;
      const double confidence_bonus = 0.10 * std::clamp(armor.confidence, 0.0, 1.0);
      const double cost = center_error * center_error +
        0.20 * angle_error * angle_error +
        0.20 * range_error * range_error - confidence_bonus;

      // Record this (detection, face) pair for the full-matrix dump.  The signed
      // angular difference is what reveals a swapped face pair or a yaw bias.
      if (current_detection_index < kMaxDiagDet && id < 4) {
        auto & cell = cost_matrix[current_detection_index][id];
        cell.valid = true;
        cell.cost = cost;
        cell.angle_deg =
          std::remainder(armor.ypr_in_world[0] - face[3], 2.0 * M_PI) * 180.0 / M_PI;
        cell.pixels = cv::norm(observed_center - projected_center);
        cell.range_m = predicted_range_m;
      }

      // Keep the per-term breakdown of the best near-miss so a systematic bias
      // can be attributed: a large centre term means the predicted image
      // position is off (radius / l / h / centre), a large angle term means the
      // predicted face normal is off (body yaw), a large range term means the
      // solved distance disagrees with the prediction.
      if (std::isfinite(cost) && cost > association_max_cost_ &&
          cost < best_detail_cost) {
        best_detail_cost = cost;
        best_detail_det = current_detection_index;
        best_detail_face = static_cast<int>(id);
        best_detail_center = center_error;
        best_detail_angle = angle_error;
        best_detail_range = range_error;
        best_detail_conf = confidence_bonus;
        best_detail_px = cv::norm(observed_center - projected_center);
        best_detail_scale = detection_scale_px;
      }

      if (std::isfinite(cost) && cost <= association_max_cost_) {
        const auto & state = association_state;
        const double center_yaw = std::atan2(state[2], state[0]);
        candidates.push_back({
          &armor, current_detection_index, static_cast<int>(id), cost,
          std::abs(tools::limit_rad(face[3] - center_yaw))});
      } else if (std::isfinite(cost)) {
        ++rejected_by_cost;
        if (cost < best_rejected_cost) {
          best_rejected_cost = cost;
          best_rejected_det = current_detection_index;
          best_rejected_face = static_cast<int>(id);
        }
      }
    }
  }

  // Solve the complete one-to-one assignment.  A greedy lowest-pair-first
  // choice can consume the only plausible detection for one face and force a
  // bad handover on another.  There are at most four faces, so an exhaustive
  // minimum-cost assignment is both deterministic and inexpensive.
  std::vector<bool> detection_used(static_cast<std::size_t>(detection_index), false);
  std::vector<Candidate> current_assignment;
  std::vector<Candidate> selected;
  double best_assignment_cost = std::numeric_limits<double>::infinity();
  std::function<void(int, double)> assign_face = [&](int face_id, double total_cost) {
    if (face_id >= static_cast<int>(predicted_faces.size()) ||
        static_cast<int>(current_assignment.size()) >= batch_max_observations_) {
      if (current_assignment.size() > selected.size() ||
          (current_assignment.size() == selected.size() &&
           total_cost < best_assignment_cost)) {
        selected = current_assignment;
        best_assignment_cost = total_cost;
      }
      return;
    }

    // A face can be out of view; skipping it is a valid branch.
    assign_face(face_id + 1, total_cost);
    for (const auto & candidate : candidates) {
      if (candidate.face_id != face_id || candidate.detection_index < 0) continue;
      const auto detection_slot = static_cast<std::size_t>(candidate.detection_index);
      if (detection_used[detection_slot]) continue;
      detection_used[detection_slot] = true;
      current_assignment.push_back(candidate);
      assign_face(face_id + 1, total_cost + candidate.cost);
      current_assignment.pop_back();
      detection_used[detection_slot] = false;
    }
  };
  assign_face(0, 0.0);

  if (selected.empty()) return false;
  const auto primary_it = std::min_element(
    selected.begin(), selected.end(), [](const Candidate & a, const Candidate & b) {
      if (std::abs(a.alignment_rad - b.alignment_rad) > 1e-6) {
        return a.alignment_rad < b.alignment_rad;
      }
      return a.cost < b.cost;
    });
  primary_detection_index_ = primary_it->detection_index;
  primary_face_id_ = primary_it->face_id;
  // Copy the primary before `selected` is possibly rewritten below; primary_it
  // is an iterator into `selected` and would dangle afterwards.
  Candidate primary = *primary_it;

  // ---------------------------------------------------------------------------
  // Geometric plausibility gate for the off-axis faces.
  //
  // Faces 1 and 3 are the ones offset along the chassis short axis, so they are
  // the only observations that constrain `l = r2 - r1` and `h`.  Those two states
  // receive no process noise, so a single mis-association can drive them far from
  // zero in one step.  Field logs caught exactly that: `l` went 0.000 -> 0.435 m
  // in one frame, `r + l` left diverged()'s 0.05..0.5 band, and the track reset.
  //
  // A real 4-armor chassis has |r2 - r1| of a few centimetres.  An association
  // implying much more is a mis-pairing, not a measurement, so the off-axis face
  // is dropped and the on-axis observation keeps the track alive.
  {
    constexpr double kMaxPlausibleRadiusDelta = 0.10;  // metres, |r2 - r1|
    constexpr double kMaxPlausibleHeightDelta = 0.10;  // metres, |h|

    const auto & state = association_state;
    if (state.size() >= 11) {
      const bool geometry_already_broken =
        std::abs(state[9]) > kMaxPlausibleRadiusDelta ||
        std::abs(state[10]) > kMaxPlausibleHeightDelta;

      const bool has_off_axis = std::any_of(
        selected.begin(), selected.end(), [](const Candidate & c) {
          return c.face_id == 1 || c.face_id == 3;
        });

      if (has_off_axis && geometry_already_broken) {
        std::vector<Candidate> filtered;
        filtered.reserve(selected.size());
        for (const auto & association : selected) {
          if (association.face_id == 1 || association.face_id == 3) {
            tools::logger()->info(
              "[ASSOC] off-axis face{} rejected: geometry implausible "
              "(l={:.3f} h={:.3f}, limits {:.2f}/{:.2f})",
              association.face_id, state[9], state[10],
              kMaxPlausibleRadiusDelta, kMaxPlausibleHeightDelta);
            continue;
          }
          filtered.push_back(association);
        }
        selected = filtered.empty()
          ? std::vector<Candidate>{primary}
          : std::move(filtered);
      }
    }
  }

  // The geometry filter may remove the old primary. Re-select from the actual
  // surviving assignment before exposing IDs or choosing the single update.
  primary = *std::min_element(selected.begin(), selected.end(), [](const Candidate & a, const Candidate & b) {
    if (std::abs(a.alignment_rad - b.alignment_rad) > 1e-6)
      return a.alignment_rad < b.alignment_rad;
    return a.cost < b.cost;
  });
  primary_detection_index_ = primary.detection_index;
  primary_face_id_ = primary.face_id;

  // Preserve the proven single-observation path on fixed and slow targets.
  // Joint observations are enabled only after the target is clearly in gyro
  // motion, where two simultaneously visible faces materially improve the
  // rigid-body centre, radius and phase estimate.
  // Never let an association-induced rate spike enable the more aggressive
  // multi-face update.  It is useful only after the independent yaw ESKF has
  // both converged and passed its configured physical rate/acceleration bounds.
  const bool use_batch = target_.yaw_eskf_trusted() &&
    std::abs(target_.yaw_eskf_rate()) >= batch_min_omega_rad_s_;
  std::vector<std::pair<Armor, int>> batch;
  batch.reserve(use_batch ? selected.size() : 1);
  for (const auto & association : selected) {
    if (use_batch || association.detection_index == primary_detection_index_) {
      // These are the observations passed into Target::update[_batch]. The EKF
      // records NIS diagnostically; it has no per-observation acceptance gate.
      last_associations_.push_back({
        association.detection_index, association.face_id, association.cost});
      batch.emplace_back(*association.armor, association.face_id);
    }
  }

  static int association_log_counter = 0;
  if (++association_log_counter % 10 == 0) {
    tools::logger()->info(
      "[ASSOC] primary_face={} matches={} cost={:.3f} detections={} confidence={:.2f}",
      primary_face_id_, batch.size(), primary.cost, armors.size(),
      primary.armor->confidence);
  }

  // Diagnostic: dump every surviving candidate so a face-association jump can be
  // told apart from a weak-observability drift.  Sampled more often than the
  // summary line because a face hand-over can complete within a few frames.
  if (association_log_counter % 10 == 0) {
    static const char * kFaceNames[4] = {"face0", "face1", "face2", "face3"};
    for (const auto & c : candidates) {
      const char * fname = (c.face_id >= 0 && c.face_id < 4) ? kFaceNames[c.face_id] : "face?";
      tools::logger()->info(
        "[ASSOC_D] det={} {} cost={:.3f} align={:.1f}deg conf={:.2f}",
        c.detection_index, fname, c.cost, c.alignment_rad * 180.0 / M_PI,
        c.armor ? c.armor->confidence : 0.0);
    }
    // Why the other detections never became candidates.  This is the key split
    // between "cost gate too tight" and "they are simply a different armor".
    tools::logger()->info(
      "[ASSOC_D] gate: total_det={} cand={} drop_name={} drop_type={} reject_cost={} "
      "best_rejected=(det{},face{},cost={:.2f}) limit={:.2f}",
      armors.size(), candidates.size(), dropped_by_name, dropped_by_type,
      rejected_by_cost, best_rejected_det, best_rejected_face,
      std::isfinite(best_rejected_cost) ? best_rejected_cost : -1.0,
      association_max_cost_);

    // Term-by-term attribution of the closest rejected candidate.
    if (std::isfinite(best_detail_cost)) {
      tools::logger()->info(
        "[ASSOC_D] near-miss det{} face{} cost={:.2f} = centre {:.2f}^2->{:.2f} "
        "+ angle {:.2f}^2*0.20->{:.2f} + range {:.2f}^2*0.20->{:.2f} - conf {:.2f} "
        "| pixels={:.1f} scale={:.1f} angle_deg={:.1f}",
        best_detail_det, best_detail_face, best_detail_cost,
        best_detail_center, best_detail_center * best_detail_center,
        best_detail_angle, 0.20 * best_detail_angle * best_detail_angle,
        best_detail_range, 0.20 * best_detail_range * best_detail_range,
        best_detail_conf,
        best_detail_px, best_detail_scale, best_detail_angle * 20.0);
    }
    // Which face the assignment actually committed, and the runner-up, so a
    // near-tie between adjacent faces is visible.
    if (selected.size() > 1) {
      std::string line = "  selected:";
      for (const auto & s : selected) {
        line += fmt::format(" det{}->face{} ({:.3f})", s.detection_index, s.face_id, s.cost);
      }
      tools::logger()->info("[ASSOC_D]{}", line);
    }
    // State the estimator believes, for correlating with the r/l jumps.
    // Flag any excursion of l/h away from 0: those two states are only written
    // when face 1 or 3 is associated, so a nonzero value pinpoints the frame at
    // which an odd face was accepted.
    const auto & st = association_state;
    if (st.size() >= 11) {
      tools::logger()->info(
        "[ASSOC_D] ekf r={:.3f} l={:.3f} h={:.3f} yaw={:.1f}deg omega={:+.3f}{}",
        st[8], st[9], st[10], st[6] * 180.0 / M_PI, st[7],
        (std::abs(st[9]) > 0.05 || std::abs(st[10]) > 0.05) ? "  <<< L/H NONZERO" : "");
    }

    // Full detection x face matrix.  One line per detection so the pairing can be
    // read directly: which face each detection prefers, and by how much.
    for (int d = 0; d < kMaxDiagDet; ++d) {
      bool any = false;
      for (int f = 0; f < 4; ++f) any = any || cost_matrix[d][f].valid;
      if (!any) continue;
      std::string row = fmt::format(
        "[ASSOC_M] det{} range={:.2f}m yaw={:+.1f}deg |", d,
        det_range[d], det_yaw_deg[d]);
      for (int f = 0; f < 4; ++f) {
        const auto & c = cost_matrix[d][f];
        if (c.valid) {
          row += fmt::format(
            " f{}:cost={:.2f} ang={:+.0f} px={:.0f} pred_r={:.2f} |", f, c.cost,
            c.angle_deg, c.pixels, c.range_m);
        } else {
          row += fmt::format(" f{}: proj_fail |", f);
        }
      }
      tools::logger()->info("{}", row);
    }
  }

  if (use_batch) {
    target_.update_batch(batch, primary_face_id_);
  } else {
    target_.update(*primary.armor, primary_face_id_);
  }
  return true;
}

#ifdef AMENT_CMAKE_FOUND
void Tracker::set_ros2_node(std::shared_ptr<rclcpp::Node> node)
{
  ros_node_ = node;
  if (ros_node_) {
    // 创建 publisher 并存储为 void*（类型擦除）
    auto pub = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_markers", 10);
    marker_pub_ = std::static_pointer_cast<void>(pub);
    tools::logger()->info("[Tracker] ROS2 marker publisher initialized on topic: armor_markers");
  }
}

void Tracker::publish_markers(
  const std::list<Target> & targets,
  const rclcpp::Time & timestamp,
  const Eigen::Vector4d & aim_xyza,
  bool has_aim_point)
{
  if (!ros_node_ || !marker_pub_) {
    return;  // ROS2 未初始化，直接返回
  }

  if (targets.empty()) {
    return;  // 没有目标，不发布
  }

  // 将 void* 转换回正确的类型
  auto pub = std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>(
    marker_pub_);

  visualization_msgs::msg::MarkerArray marker_array;

  // 遍历所有目标
  for (const auto & target : targets) {
    // 获取所有装甲板位姿
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

    // 为每个装甲板创建一个 marker
    int marker_id = 0;
    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";  // 根据你的坐标系修改
      marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      marker.ns = "armor_plates";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;  // 改为球体
      marker.action = visualization_msgs::msg::Marker::ADD;

      // 设置位置 (x, y, z 在世界坐标系中)
      marker.pose.position.x = xyza[0];
      marker.pose.position.y = xyza[1];
      marker.pose.position.z = xyza[2];

      // 设置姿态 (从 yaw 角度转换为四元数)
      double yaw = xyza[3];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin(yaw / 2.0);
      marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 球体（直径）
      double diameter;
      if (target.armor_type == ArmorType::small) {
        diameter = 0.08;  // 小装甲板：80mm 直径
      } else {
        diameter = 0.12;  // 大装甲板：120mm 直径
      }
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = diameter;

      // 设置颜色 (绿色，半透明)
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.7;

      marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(marker);
    }

    // 如果有瞄准点，也创建一个 marker（红色球体）
    if (has_aim_point) {
      visualization_msgs::msg::Marker aim_marker;
      aim_marker.header.frame_id = "world";
      aim_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      aim_marker.ns = "aim_point";
      aim_marker.id = 1000;
      aim_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 球体
      aim_marker.action = visualization_msgs::msg::Marker::ADD;

      aim_marker.pose.position.x = aim_xyza[0];
      aim_marker.pose.position.y = aim_xyza[1];
      aim_marker.pose.position.z = aim_xyza[2];

      double yaw = aim_xyza[3];
      aim_marker.pose.orientation.x = 0.0;
      aim_marker.pose.orientation.y = 0.0;
      aim_marker.pose.orientation.z = std::sin(yaw / 2.0);
      aim_marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 立着的长方体
      if (target.armor_type == ArmorType::small) {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      } else {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      }

      // 红色
      aim_marker.color.r = 1.0;
      aim_marker.color.g = 0.0;
      aim_marker.color.b = 0.0;
      aim_marker.color.a = 0.9;

      aim_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(aim_marker);
    }

    // 🆕 添加敌车中心的蓝色球体
    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = "world";
    center_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
    center_marker.ns = "vehicle_center";
    center_marker.id = 2000;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;

    // 从 EKF 状态获取敌车中心位置
    Eigen::VectorXd ekf_x = target.ekf_x();
    center_marker.pose.position.x = ekf_x[0];  // 中心 X
    center_marker.pose.position.y = ekf_x[2];  // 中心 Y
    center_marker.pose.position.z = ekf_x[4];  // 中心 Z

    // 姿态（可选，球体不受姿态影响）
    center_marker.pose.orientation.x = 0.0;
    center_marker.pose.orientation.y = 0.0;
    center_marker.pose.orientation.z = 0.0;
    center_marker.pose.orientation.w = 1.0;

    double center_diameter = 0.08;  // 80mm 直径
    center_marker.scale.x = center_diameter;
    center_marker.scale.y = center_diameter;
    center_marker.scale.z = center_diameter;

    // 蓝色
    center_marker.color.r = 0.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 1.0;
    center_marker.color.a = 0.8;

    center_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

    marker_array.markers.push_back(center_marker);
  }

  // 发布 marker array
  pub->publish(marker_array);
}
#endif

}  // namespace auto_aim
