#include "aimer.hpp"
#include "solver.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>

int main(int argc, char ** argv)
{
  assert(argc == 2);  // Synthetic, narrow selection fixture; no device access.
  auto_aim::Aimer aimer(argv[1]);
  auto_aim::AimPredictionTrace trace;
  const auto image = std::chrono::steady_clock::time_point{};
  const auto predict = [&](std::list<auto_aim::Target> targets) {
    return aimer.aim(targets, image, 20.0, false, &trace, auto_aim::AimGeometry::whole_ekf);
  };
  const auto target = [](double range, double rate) {
    auto_aim::Target t(range, rate, 0.26, 0.0);
    t.name = auto_aim::ArmorName::two;
    t.armor_type = auto_aim::ArmorType::small;
    t.priority = auto_aim::ArmorPriority::first;
    t.jumped = true;
    t.last_id = 0;
    return t;
  };
  auto command = predict({target(3.0, 0.0)});
  assert(command.control && trace.valid && aimer.debug_aim_point.valid);
  assert(trace.failure == auto_aim::AimPredictionFailure::none);
  command = predict({});
  assert(!command.control && !trace.valid && !aimer.debug_aim_point.valid);
  assert(trace.failure == auto_aim::AimPredictionFailure::no_target);

  // The fixture's 10-degree selection cone has a valid initial face at 3 rad/s,
  // but none when the projectile arrives. A different target starts off-cone.
  command = predict({target(3.0, 100.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::initial_selection);
  command = predict({target(3.0, 3.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::iterative_selection);
  assert(trace.iterations > 0 && !aimer.debug_aim_point.valid);
  command = predict({target(1000.0, 0.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::trajectory);
  command = predict({target(std::numeric_limits<double>::quiet_NaN(), 0.0)});
  assert(!command.control && trace.failure == auto_aim::AimPredictionFailure::non_finite);

  // The send-time projection overload must use the supplied feedback pose
  // without modifying Solver's image-time pose used by Tracker.
  auto_aim::Solver solver(argv[1]);
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
  std::cout << "aimer_prediction_test: PASS (five failure stages + feedback projection)\n";
}
