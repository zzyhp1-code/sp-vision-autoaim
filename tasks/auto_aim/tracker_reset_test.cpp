#include "tracker.hpp"

#include <cassert>
#include <chrono>
#include <iostream>

int main(int argc, char ** argv)
{
  assert(argc == 2);  // Offline fixture config; no camera, serial or motor.
  auto_aim::Solver solver(argv[1]);
  auto_aim::Tracker tracker(argv[1], solver);
  auto now = std::chrono::steady_clock::now();
  const auto detection = [&]() {
    auto points = solver.reproject_armor(
      Eigen::Vector3d(3.0, 0.0, 0.2), 0.0, auto_aim::ArmorType::small,
      auto_aim::ArmorName::two);
    auto_aim::Armor armor(10, 0.99f, cv::boundingRect(points), points, auto_aim::YOLOVersion::YOLO26);
    armor.center_norm = cv::Point2f(0, 0);
    return std::list<auto_aim::Armor>{armor};
  };
  const auto track = [&](bool visible) {
    now += std::chrono::milliseconds(16);
    auto armors = visible ? detection() : std::list<auto_aim::Armor>{};
    return tracker.track(armors, now);
  };

  auto targets = track(true);
  assert(tracker.model_generation() == 1 && tracker.state() == "detecting");
  for (int frame = 0; frame < 30; ++frame) targets = track(true);
  assert(!targets.empty() && tracker.state() == "tracking");
  assert(targets.front().convergened() && targets.front().yaw_eskf_trusted());
  assert(targets.front().yaw_eskf_accepted_updates() > 0);
  const double old_rate_variance = targets.front().ekf().P(7, 7);

  targets = track(false);
  assert(tracker.state() == "temp_lost" && !targets.empty());
  targets = track(true);
  assert(tracker.state() == "tracking" && tracker.model_generation() == 1);

  tracker.reset();
  assert(tracker.state() == "lost" && tracker.last_associations().empty());
  assert(tracker.primary_detection_index() == -1 && tracker.primary_face_id() == -1);
  assert(track(false).empty());
  assert(tracker.model_generation() == 1);
  targets = track(true);
  assert(tracker.state() == "detecting" && tracker.model_generation() == 2);
  assert(!targets.front().convergened() && !targets.front().yaw_eskf_trusted());
  assert(targets.front().yaw_eskf_accepted_updates() == 0);
  assert(targets.front().ekf().P(7, 7) > old_rate_variance);
  for (int frame = 0; frame < 30; ++frame) targets = track(true);
  assert(tracker.state() == "tracking" && targets.front().convergened());
  assert(targets.front().yaw_eskf_trusted());

  // A planned reset also works with uninterrupted visible detections.
  tracker.reset();
  targets = track(true);
  assert(tracker.state() == "detecting" && tracker.model_generation() == 3);
  assert(targets.front().yaw_eskf_accepted_updates() == 0);
  assert(!targets.front().convergened());
  std::cout << "tracker_reset_test: PASS (temp_lost continuity + fresh EKF/YawESKF)\n";
}
