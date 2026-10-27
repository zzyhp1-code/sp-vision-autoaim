#include "solver.hpp"
#include "tracker.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

namespace
{
struct Fixture
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("sp_solver_validation_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");

  Fixture()
  {
    std::ofstream file(path);
    file <<
      "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
      "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
      "t_camera2gimbal: [0,0,0]\n"
      "camera_matrix: [1000,0,640,0,1000,512,0,0,1]\n"
      "distort_coeffs: [0,0,0,0,0]\n"
      "enemy_color: blue\nmin_detect_count: 3\nmax_temp_lost_count: 5\n"
      "outpost_max_temp_lost_count: 8\ntracker_batch_min_omega_rad_s: 1000\n";
    assert(file.good());
  }

  ~Fixture()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};

// Generate pixels independently with OpenCV's forward projection, not Solver's
// inverse/reprojection path. No detector, camera, serial port or field YAML.
auto_aim::Armor detection(
  const Eigen::Vector3d & position = {3.0, 0.0, 0.2}, double yaw = 0.0,
  auto_aim::ArmorType type = auto_aim::small,
  auto_aim::ArmorName name = auto_aim::two)
{
  const double width = type == auto_aim::big ? 0.230 : 0.135;
  const std::vector<cv::Point3f> corners{
    {0, static_cast<float>(width / 2), 0.028F},
    {0, static_cast<float>(-width / 2), 0.028F},
    {0, static_cast<float>(-width / 2), -0.028F},
    {0, static_cast<float>(width / 2), -0.028F}};
  const Eigen::Matrix3d camera_to_world =
    (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
  const double pitch = (name == auto_aim::outpost ? -15.0 : 15.0) * CV_PI / 180.0;
  const Eigen::Matrix3d armor_to_world =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
  cv::Mat rotation;
  cv::eigen2cv(Eigen::Matrix3d(camera_to_world.transpose() * armor_to_world), rotation);
  cv::Vec3d rvec;
  cv::Rodrigues(rotation, rvec);
  const Eigen::Vector3d translation = camera_to_world.transpose() * position;
  const cv::Mat camera = (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 512, 0, 0, 1);
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(
    corners, rvec, cv::Vec3d(translation[0], translation[1], translation[2]),
    camera, cv::Mat::zeros(1, 5, CV_64F), pixels);
  auto_aim::Armor armor(10, 0.99F, cv::boundingRect(pixels), pixels, auto_aim::YOLOVersion::YOLO26);
  armor.type = type;
  armor.name = name;
  armor.center_norm = cv::Point2f((armor.center.x - 640) / 1000, (armor.center.y - 512) / 1000);
  return armor;
}

void assert_invalid_pose(const auto_aim::Armor & armor)
{
  assert(armor.xyz_in_gimbal.array().isNaN().all());
  assert(armor.xyz_in_world.array().isNaN().all());
  assert(armor.ypr_in_gimbal.array().isNaN().all());
  assert(armor.ypr_in_world.array().isNaN().all());
  assert(armor.ypd_in_world.array().isNaN().all());
  assert(std::isnan(armor.yaw_raw));
}

void test_solver(auto_aim::Solver & solver)
{
  for (const auto type : {auto_aim::small, auto_aim::big}) {
    for (const double degrees : {-34.0, 0.0, 13.0, 55.0}) {
      const Eigen::Vector3d expected(4.0, 0.12, 0.25);
      auto armor = detection(expected, degrees * CV_PI / 180.0, type);
      assert(solver.solve(armor));
      assert((armor.xyz_in_world - expected).norm() < 1e-4);
      assert(std::abs(armor.ypr_in_world[0] - degrees * CV_PI / 180.0) < 0.02);
      assert(armor.ypr_in_gimbal.allFinite() && std::isfinite(armor.yaw_raw));
    }
  }
  auto balance = detection({4.0, 0.1, 0.2}, 0.2, auto_aim::big, auto_aim::three);
  assert(solver.solve(balance) && std::isfinite(balance.yaw_raw));
  auto outpost = detection({4.0, 0.1, 0.2}, 0.2, auto_aim::small, auto_aim::outpost);
  assert(solver.solve(outpost));
  assert(std::isfinite(solver.oupost_reprojection_error(outpost, -15 * CV_PI / 180.0)));

  const auto reject = [&](std::vector<cv::Point2f> points) {
    auto armor = detection();
    assert(solver.solve(armor));  // Seed a previously valid/cached pose.
    armor.points = std::move(points);
    assert(!solver.solve(armor));
    assert_invalid_pose(armor);
    assert(std::isinf(solver.oupost_reprojection_error(armor, 0.0)));
  };
  auto pixels = detection().points;
  reject({});
  reject({pixels[0]});
  reject({pixels[0], pixels[1], pixels[2]});
  auto extra = pixels;
  extra.push_back(pixels[0]);
  reject(extra);
  auto duplicate = pixels;
  duplicate[2] = duplicate[1];
  reject(duplicate);
  reject({{0, 0}, {1, 0}, {2, 0}, {3, 0}});
  reject({{0, 0}, {1, 1e-8F}, {2, 0}, {3, -1e-8F}});
  reject({pixels[0], pixels[2], pixels[1], pixels[3]});  // Crossing order.
  reject({{0, 0}, {10, 0}, {1, 1}, {0, 10}});  // Concave.
  for (const float bad : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity()}) {
    auto nonfinite = pixels;
    nonfinite[1].x = bad;
    reject(nonfinite);
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const cv::Vec3d zero(0, 0, 0);
  assert(auto_aim::Solver::valid_camera_pose(zero, {0, 0, 3}, auto_aim::small));
  assert(!auto_aim::Solver::valid_camera_pose(zero, {0, 0, -3}, auto_aim::small));
  assert(!auto_aim::Solver::valid_camera_pose(zero, {0, 0, 0}, auto_aim::small));
  assert(!auto_aim::Solver::valid_camera_pose(zero, {0, 0, 0.01}, auto_aim::small));
  assert(!auto_aim::Solver::valid_camera_pose({CV_PI / 2, 0, 0}, {0, 0, 0.1}, auto_aim::big));
  assert(!auto_aim::Solver::valid_camera_pose({nan, 0, 0}, {0, 0, 3}, auto_aim::small));
  assert(!auto_aim::Solver::valid_camera_pose(zero, {infinity, 0, 3}, auto_aim::small));
  auto stale = detection();
  assert(solver.solve(stale));
  solver.set_R_gimbal2world(Eigen::Quaterniond(nan, 0, 0, 0));
  assert(!solver.solve(stale));
  assert_invalid_pose(stale);
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  assert(solver.reproject_armor({-3, 0, 0}, 0, auto_aim::small, auto_aim::two).empty());
  assert(solver.reproject_armor({3, 0, 0}, nan, auto_aim::small, auto_aim::two).empty());
}

void test_tracker(const std::string & config, auto_aim::Solver & solver, bool omni)
{
  auto_aim::Tracker tracker(config, solver);
  auto now = std::chrono::steady_clock::now();
  const auto track = [&](std::list<auto_aim::Armor> & armors) {
    now += std::chrono::milliseconds(16);
    if (omni) {
      return std::get<1>(tracker.track(
        std::vector<omniperception::DetectionResult>{}, armors, now));
    }
    return tracker.track(armors, now);
  };
  const auto invalid_detection = [&]() {
    auto armor = detection();
    assert(solver.solve(armor));
    armor.points.resize(3);
    return armor;
  };
  std::list<auto_aim::Armor> bad{invalid_detection()};
  assert(track(bad).empty() && bad.empty());
  assert(tracker.state() == "lost" && tracker.model_generation() == 0);
  assert(tracker.last_associations().empty());

  std::list<auto_aim::Armor> visible{detection()};
  auto targets = track(visible);
  assert(tracker.state() == "detecting" && tracker.model_generation() == 1);
  bad = {invalid_detection()};
  assert(track(bad).empty() && tracker.state() == "lost");
  assert(tracker.last_associations().empty() && tracker.primary_detection_index() == -1);
  for (int frame = 0; frame < 35; ++frame) {
    visible = {detection()};
    targets = track(visible);
  }
  assert(tracker.state() == "tracking" && !targets.empty());
  assert(tracker.last_associations().size() == 1);
  const auto generation = tracker.model_generation();
  // Bad high-priority observations cannot start a target-switch streak.
  for (int frame = 0; frame < 4; ++frame) {
    auto high_priority = invalid_detection();
    high_priority.priority = auto_aim::first;
    high_priority.name = auto_aim::one;
    visible = {high_priority, detection()};
    targets = track(visible);
    assert(visible.size() == 1 && tracker.model_generation() == generation);
    assert(targets.front().name == auto_aim::two);
    assert(tracker.last_associations().size() == 1);
    assert(tracker.primary_detection_index() == 0);
  }
  auto prediction_only = targets.front();
  prediction_only.predict(now + std::chrono::milliseconds(16));
  bad = {invalid_detection()};
  targets = track(bad);
  assert(tracker.state() == "temp_lost" && !targets.empty() && bad.empty());
  assert(tracker.last_associations().empty());
  assert(tracker.primary_detection_index() == -1 && tracker.primary_face_id() == -1);
  assert(targets.front().ekf_x().isApprox(prediction_only.ekf_x(), 1e-12));
  assert(targets.front().ekf().P.isApprox(prediction_only.ekf().P, 1e-12));
  assert(targets.front().yaw_eskf_accepted_updates() == prediction_only.yaw_eskf_accepted_updates());
  visible = {detection()};
  targets = track(visible);
  assert(tracker.state() == "tracking" && tracker.model_generation() == generation);
  for (int frame = 0; frame < 6; ++frame) {
    bad = {invalid_detection()};
    targets = track(bad);
  }
  assert(tracker.state() == "lost" && targets.empty());
}
}  // namespace

int main()
{
  Fixture fixture;
  auto_aim::Solver solver(fixture.path.string());
  test_solver(solver);
  test_tracker(fixture.path.string(), solver, false);
  test_tracker(fixture.path.string(), solver, true);
  std::cout << "solver_validation_test: PASS (synthetic PnP, invalid/cache/depth guards, both Tracker paths)\n";
}
