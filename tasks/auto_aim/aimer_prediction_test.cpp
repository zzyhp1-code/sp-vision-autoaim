#include "aimer.hpp"
#include "solver.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <yaml-cpp/yaml.h>

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
  std::filesystem::path path;
  bool temporary = false;

  explicit Fixture(const char * supplied)
  {
    if (supplied) {
      path = supplied;
      return;
    }
    temporary = true;
    path = std::filesystem::temp_directory_path() /
      ("sp_aimer_prediction_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    std::ofstream file(path);
    file <<
      "yaw_offset: 0\npitch_offset: 0\n"
      "comming_angle: 60\nleaving_angle: 20\n"
      "high_speed_delay_time: 0.170\nlow_speed_delay_time: 0.165\ndecision_speed: 1\n"
      "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
      "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\nt_camera2gimbal: [0,0,0]\n"
      "camera_matrix: [1000,0,640,0,1000,512,0,0,1]\ndistort_coeffs: [0,0,0,0,0]\n";
    assert(file.good());
  }

  ~Fixture()
  {
    if (!temporary) return;
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  assert(argc <= 2);  // Optional synthetic fixture; no device access or field YAML.
  const Fixture fixture(argc == 2 ? argv[1] : nullptr);
  auto_aim::Aimer aimer(fixture.path.string());
  const auto yaml = YAML::LoadFile(fixture.path.string());
  assert(yaml["comming_angle"].as<double>() == 60.0);
  assert(yaml["leaving_angle"].as<double>() == 20.0);
  assert(yaml["decision_speed"].as<double>() > 0.5 &&
         yaml["decision_speed"].as<double>() < 3.0);
  const double base_high = 0.005 + yaml["high_speed_delay_time"].as<double>();
  auto_aim::AimPredictionTrace trace;
  const auto image = std::chrono::steady_clock::time_point{};
  const auto predict = [&](std::list<auto_aim::Target> targets) {
    for (const auto & tracked : targets) {
      const auto selected = aimer.prediction_model(tracked, auto_aim::AimGeometry::whole_ekf);
      assert((selected.geometry == auto_aim::AimGeometry::whole_ekf) ==
        tracked.use_whole_ekf_geometry(yaml["decision_speed"].as<double>()));
    }
    return aimer.aim(targets, image, 20.0, false, &trace, auto_aim::AimGeometry::whole_ekf);
  };
  const auto observation = [](double range, double phase, double radius) {
    auto_aim::Armor armor(10, 1.0F, cv::Rect(0, 0, 10, 10),
      {{0, 0}, {10, 0}, {10, 10}, {0, 10}}, auto_aim::YOLOVersion::YOLO26);
    armor.name = auto_aim::ArmorName::two;
    armor.type = auto_aim::ArmorType::small;
    armor.priority = auto_aim::ArmorPriority::first;
    armor.confidence = 1.0;
    armor.xyz_in_world = {range - radius * std::cos(phase), -radius * std::sin(phase), 0.0};
    armor.ypr_in_world = {phase, 0.0, 0.0};
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    return armor;
  };
  const auto target = [&](double range, double rate, double phase = 0.0, double radius = 0.26) {
    auto_aim::Target t(
      observation(range, phase, radius), image, radius, 4, Eigen::VectorXd::Ones(11));
    // The fixture sets the main filter directly; no camera, device or Tracker
    // association is involved. Use the fully initialized production constructor.
    const_cast<tools::ExtendedKalmanFilter &>(t.ekf()).x[7] = rate;
    t.jumped = true;
    t.last_id = 0;
    return t;
  };
  auto command = predict({target(3.0, 0.0)});
  assert(command.control && trace.valid && trace.converged && aimer.debug_aim_point.valid);
  assert(!trace.whole_ekf_geometry && !trace.fixed_face && trace.face_id == 0);
  assert(std::abs(trace.delay_s - yaml["low_speed_delay_time"].as<double>()) < 1e-12);
  assert(trace.failure == auto_aim::AimPredictionFailure::none);
  command = predict({});
  assert(!command.control && !trace.valid && !aimer.debug_aim_point.valid);
  assert(trace.failure == auto_aim::AimPredictionFailure::no_target);

  // At the delay-only instant all four faces fail the 60/20 selection rule.
  // By impact, face 3 (positive rate) or face 1 (negative rate) is eligible.
  for (double sign : {1.0, -1.0}) {
    const double rate = sign * 5.0;
    const auto rotating = target(4.0, rate, sign * 25.0 / 57.3 - rate * base_high);
    auto base = rotating;
    base.predict(image + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(base_high)));
    for (const auto & face : base.whole_ekf_armor_xyza_list()) {
      const double delta = tools::limit_rad(face[3]);
      assert(!(std::abs(delta) <= 60.0 / 57.3 &&
        ((rate > 0 && delta < 20.0 / 57.3) || (rate < 0 && delta > -20.0 / 57.3))));
    }
    command = predict({rotating});
    assert(command.control && trace.valid && trace.converged && trace.fixed_face);
    assert(trace.whole_ekf_geometry && trace.geometry == auto_aim::AimGeometry::whole_ekf);
    assert(trace.face_id == (sign > 0 ? 3 : 1));
    assert(trace.angular_velocity_rad_s == rate);
    assert(std::abs(trace.delay_s - yaml["high_speed_delay_time"].as<double>()) < 1e-12);
    // Independently reconstruct the same physical face at the reported time.
    auto impact = rotating;
    impact.predict(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(trace.prediction_ns)));
    const auto face = impact.whole_ekf_armor_xyza_list().at(trace.face_id);
    assert(face.isApprox(trace.aim_xyza, 1e-9));
    assert(face.isApprox(aimer.debug_aim_point.xyza, 1e-9));
    tools::Trajectory ballistic(20.0, face.head<2>().norm(), face[2]);
    assert(!ballistic.unsolvable);
    assert(std::abs(ballistic.fly_time - trace.flight_time_used_s) < 0.001);
    assert(std::abs(trace.flight_time_s - ballistic.fly_time) < 1e-12);
    assert(std::abs(tools::limit_rad(command.yaw - std::atan2(face[1], face[0]) -
      yaml["yaw_offset"].as<double>() / 57.3)) < 1e-12);
    // The supplied Tracker object stays at image time.
    assert(rotating.ekf().x[6] == sign * 25.0 / 57.3 - rate * base_high);
  }

  // A candidate may be visible initially, then enter the final selection gap.
  // Fixed-face solving must finish and report final_selection, not reuse the
  // initially visible point or change the physical face within an iteration.
  command = predict({target(3.0, 3.0, -3.0 * base_high)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::final_selection);
  assert(!trace.valid && !aimer.debug_aim_point.valid);

  // Preserve the existing-geometry selection stages for legacy callers.
  command = aimer.aim({target(3.0, 5.0, 25.0 / 57.3 - 5.0 * base_high)},
    image, 20.0, false, &trace, auto_aim::AimGeometry::existing);
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::initial_selection);
  command = aimer.aim({target(3.0, 3.0, -3.0 * base_high)},
    image, 20.0, false, &trace, auto_aim::AimGeometry::existing);
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::iterative_selection);
  assert(trace.iterations > 0 && !aimer.debug_aim_point.valid);

  // Deliberately noncontracting high-rate geometry: every face exhausts the
  // ten-iteration budget while remaining finite. It must not authorize control.
  command = predict({target(1.0, 400.0, -400.0 * base_high, 0.49)});
  assert(!command.control && !trace.valid && !trace.converged);
  assert(trace.failure == auto_aim::AimPredictionFailure::not_converged);
  assert(trace.iterations == 10 && !aimer.debug_aim_point.valid);

  // The same requirement also applies to low-speed/direct-motion prediction.
  auto receding = target(1.0, 0.0);
  const_cast<tools::ExtendedKalmanFilter &>(receding.ekf()).x[1] = 18.0;
  receding.jumped = false;
  command = predict({receding});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::not_converged);
  assert(!trace.whole_ekf_geometry && !trace.converged && trace.iterations == 10);

  auto low_speed = target(3.0, 0.3);
  command = predict({low_speed});
  assert(command.control && trace.converged && !trace.whole_ekf_geometry && !trace.fixed_face);
  auto unjumped = target(3.0, 3.0);
  unjumped.jumped = false;
  command = predict({unjumped});
  assert(command.control && trace.converged && trace.whole_ekf_geometry && !trace.fixed_face);
  assert(trace.face_id == 0);
  auto outpost = target(3.0, 0.0);
  outpost.name = auto_aim::ArmorName::outpost;
  outpost.jumped = false;
  command = predict({outpost});
  assert(command.control && trace.converged && !trace.whole_ekf_geometry);

  // Make both filters mature but disagree about rotation: raw EKF is 5 rad/s,
  // YawESKF is stationary. Geometry, speed branch and trust must agree on raw.
  auto_aim::YawESKF::Config eskf_config;
  eskf_config.min_trusted_updates = 4;
  eskf_config.trusted_rate_variance = 1000.0;
  const auto measured = observation(3.0, 0.0, 0.26);
  auto_aim::Target mixed(measured, image, 0.26, 4, Eigen::VectorXd::Ones(11), eskf_config);
  for (int i = 0; i < 4; ++i) mixed.update(measured, 0);
  const_cast<tools::ExtendedKalmanFilter &>(mixed.ekf()).x[7] = 5.0;
  assert(mixed.yaw_eskf_trusted() && std::abs(mixed.ekf_x()[7]) < 1e-12);
  const auto whole_model = aimer.prediction_model(mixed, auto_aim::AimGeometry::whole_ekf);
  const auto existing_model = aimer.prediction_model(mixed, auto_aim::AimGeometry::existing);
  assert(whole_model.finite && whole_model.trusted && whole_model.angular_velocity_rad_s == 5.0);
  assert(whole_model.geometry == auto_aim::AimGeometry::whole_ekf);
  assert(mixed.use_whole_ekf_geometry(yaml["decision_speed"].as<double>()));
  assert(existing_model.finite && existing_model.trusted && existing_model.angular_velocity_rad_s == 0.0);
  command = predict({mixed});
  assert(trace.whole_ekf_geometry && trace.angular_velocity_rad_s == 5.0);
  assert(std::abs(trace.delay_s - yaml["high_speed_delay_time"].as<double>()) < 1e-12);

  // Four updates and plausible radii alone must not grant rotation authority.
  // Keep raw-EKF prediction geometry while the independent observer warms up.
  auto cautious_config = eskf_config;
  cautious_config.min_trusted_updates = 20;
  auto_aim::Target warming(measured, image, 0.26, 4, Eigen::VectorXd::Ones(11), cautious_config);
  for (int i = 0; i < 4; ++i) warming.update(measured, 0);
  const_cast<tools::ExtendedKalmanFilter &>(warming.ekf()).x[7] = 5.0;
  assert(warming.convergened() && !warming.yaw_eskf_trusted());
  const auto warming_model = aimer.prediction_model(warming);
  assert(warming_model.finite && !warming_model.trusted);
  assert(warming_model.geometry == auto_aim::AimGeometry::whole_ekf);
  assert(warming_model.angular_velocity_rad_s == 5.0);
  const_cast<tools::ExtendedKalmanFilter &>(mixed.ekf()).P(0, 0) =
    std::numeric_limits<double>::quiet_NaN();
  assert(!aimer.prediction_model(mixed).finite && !aimer.prediction_model(mixed).trusted);

  command = predict({target(1000.0, 0.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::trajectory);
  command = predict({target(std::numeric_limits<double>::quiet_NaN(), 0.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::non_finite);

  // Fixed-centre invocation: every iteration and the final eligibility check
  // use one immutable geometry despite an intentionally translating raw EKF.
  const auto fixed_image = std::chrono::steady_clock::time_point(std::chrono::seconds(1));
  for (double sign : {-1.0, 1.0}) {
    const double rate = sign * 5.0;
    auto raw = target(4.2, rate);
    auto & state = const_cast<tools::ExtendedKalmanFilter &>(raw.ekf()).x;
    state[1] = 1.5; state[3] = -0.7;
    const auto original_state = raw.ekf().x;
    auto_aim::FixedCenterAimModel fixed;
    fixed.context = {7, static_cast<int>(raw.name), 1, 1, 2, sign > 0 ? 1 : -1,
      20.0, 0.17, rate, 4.0};
    fixed.model = {1000000000LL, {4.0, 0.0, sign * 25.0 / 57.3 - rate * base_high,
      rate, {0.22, 0.27}, {0.02, 0.08}}};
    fixed.epoch = 12; fixed.revision = 3; fixed.valid_until_ns = 1250000000LL;
    command = aimer.aim({raw}, fixed_image, 20.0, false, &trace,
      auto_aim::AimGeometry::whole_ekf, &fixed);
    assert(command.control && trace.valid && trace.converged && trace.fixed_center && trace.fixed_face);
    assert(trace.fixed_center_epoch == 12 && trace.fixed_center_revision == 3);
    assert(trace.model_valid_until_ns == fixed.valid_until_ns && trace.angular_velocity_rad_s == rate);
    const auto face = fixed.face_at(trace.prediction_ns, trace.face_id);
    assert(trace.aim_xyza.isApprox(Eigen::Vector4d(face[0], face[1], face[2], face[3]), 1e-12));
    tools::Trajectory ballistic(20.0, std::hypot(face[0], face[1]), face[2]);
    assert(!ballistic.unsolvable && std::abs(ballistic.fly_time - trace.flight_time_used_s) < 0.001);
    assert(std::abs(command.yaw - std::atan2(face[1], face[0])) < 1e-12);
    assert(raw.ekf().x.isApprox(original_state, 0.0));
    auto expired = fixed;
    expired.valid_until_ns = 999999999;
    command = aimer.aim({raw}, fixed_image, 20.0, false, &trace,
      auto_aim::AimGeometry::whole_ekf, &expired);
    assert(!command.control && !trace.valid && trace.failure == auto_aim::AimPredictionFailure::stale_model);
    auto wrong_target = fixed; ++wrong_target.context.target_id;
    command = aimer.aim({raw}, fixed_image, 20.0, false, &trace,
      auto_aim::AimGeometry::whole_ekf, &wrong_target);
    assert(!command.control && !aimer.debug_aim_point.valid);
    command = aimer.aim({target(3.0, 0.0)}, fixed_image, 20.0, false, &trace,
      auto_aim::AimGeometry::whole_ekf, &fixed);
    assert(!command.control && !trace.fixed_center);  // Stationary path cannot take over.
    // Removing the optional snapshot retains the original healthy path.
    command = predict({target(4.0, rate, sign * 25.0 / 57.3 - rate * base_high)});
    assert(command.control && !trace.fixed_center);
  }

  // The send-time projection overload must use the supplied feedback pose
  // without modifying Solver's image-time pose used by Tracker.
  auto_aim::Solver solver(fixture.path.string());
  const Eigen::Vector3d xyz(3.0, 0.0, 0.2);
  const auto original = solver.R_gimbal2world();
  const Eigen::Matrix3d rotated = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto pixels = solver.reproject_armor(xyz, 0.0, auto_aim::small, auto_aim::two, &rotated);
  assert(solver.R_gimbal2world().isApprox(original));
  solver.set_R_gimbal2world(Eigen::Quaterniond(rotated));  // Fixture's IMU extrinsic is identity.
  const auto reference = solver.reproject_armor(xyz, 0.0, auto_aim::small, auto_aim::two);
  assert(pixels.size() == 4 && reference.size() == pixels.size());
  for (std::size_t i = 0; i < pixels.size(); ++i) assert(cv::norm(pixels[i] - reference[i]) < 1e-4);
  solver.set_R_gimbal2world(Eigen::Quaterniond(original));
  std::cout << "aimer_prediction_test: PASS (final fixed faces, both directions, convergence, model source, legacy + feedback projection)\n";
}
