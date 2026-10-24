#include "aimer.hpp"
#include "fire_window.hpp"
#include "model_rebuild.hpp"
#include "stable_rotation_rate.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace
{
// The test owns its synthetic config so unrelated tracker fixtures cannot
// change face-cone or delay assumptions. It lives in the system temp directory
// and is removed by the destructor, including on an assert failure.
class TemporaryAimerConfig
{
public:
  TemporaryAimerConfig()
  {
    path_ = std::filesystem::temp_directory_path() /
      ("spv_rotation_prediction_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    std::ofstream out(path_);
    out << "yaw_offset: 0.0\n"
           "pitch_offset: 0.0\n"
           "comming_angle: 60\n"
           "leaving_angle: 20\n"
           "high_speed_delay_time: 0.0\n"
           "low_speed_delay_time: 0.0\n"
           "decision_speed: 0.8\n";
    out.close();
  }
  ~TemporaryAimerConfig() { std::error_code ignored; std::filesystem::remove(path_, ignored); }
  std::string path() const { return path_.string(); }
private:
  std::filesystem::path path_;
};
}  // namespace

int main()
{
  const TemporaryAimerConfig config;
  auto_aim::Aimer off(config.path()), production(config.path()), shadow(config.path());
  auto_aim::FireCandidateWindow off_candidate, shadow_candidate;
  auto_aim::ModelRebuildCounter off_rebuild(15), shadow_rebuild(15);
  const auto origin = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
  int valid = 0, compared = 0, changed = 0;
  std::uint64_t requests = 0;
  for (int frame = 0; frame < 400; ++frame) {
    const auto image = origin + std::chrono::milliseconds(frame * 16);
    const double omega = frame < 200 ? -4.4 : 4.4;
    Eigen::VectorXd state(11);
    state << 4.0, 0.2, 0.1, -0.1, 0.0, 0.0, omega * frame * 0.016, omega, 0.26, 0.04, 0.0;
    state[6] = std::remainder(state[6], 2 * M_PI);
    const auto target = auto_aim::Target::prediction_snapshot(
      state, image, auto_aim::ArmorName::two, auto_aim::ArmorType::small, true, 0);
    const auto old_state = target.ekf().x;
    auto_aim::AimPredictionContext clock;
    clock.evaluation_time = image + std::chrono::milliseconds(25);
    auto_aim::AimPredictionTrace a, b, s;
    const auto ca = off.aim({target}, image, 19.8, true, &a, auto_aim::AimGeometry::whole_ekf, clock);
    auto_aim::RotationPredictionEpoch shadow_epoch;
    assert(!shadow_epoch.update(auto_aim::RotationPredictionMode::shadow, frame % 2, frame));
    const auto cb = production.aim({target}, image, 19.8, true, &b, auto_aim::AimGeometry::whole_ekf, clock);
    auto option = clock; option.rotation_rate_rad_s = omega * 1.03;
    const auto cs = shadow.aim({target}, image, 19.8, true, &s, auto_aim::AimGeometry::whole_ekf, option);
    (void)cs;
    assert(ca.control == cb.control && ca.shoot == cb.shoot);
    assert(ca.yaw == cb.yaw && ca.pitch == cb.pitch);
    assert(a.face_id == b.face_id && a.prediction_ns == b.prediction_ns);
    assert(a.evaluation_ns == s.evaluation_ns && a.base_prediction_ns == s.base_prediction_ns);
    assert(target.ekf().x.isApprox(old_state, 0.0));
    assert(s.rotation_override_used);
    if (a.valid) ++valid;
    if (a.valid && s.valid) {
      ++compared;
      assert(std::abs(s.rotation_rate_rad_s - *option.rotation_rate_rad_s) < 1e-12);
      const double horizon = (s.prediction_ns - std::chrono::duration_cast<std::chrono::nanoseconds>(
        image.time_since_epoch()).count()) * 1e-9;
      const double expected_phase = state[6] + *option.rotation_rate_rad_s * horizon + s.face_id * M_PI / 2;
      assert(std::abs(std::remainder(shadow.debug_aim_point.xyza[3] - expected_phase, 2 * M_PI)) < 1e-9);
      if (std::abs(ca.yaw - cs.yaw) > 1e-6) ++changed;
    }
    const std::uint64_t generation = frame / 80 + 1;
    off_rebuild.observe_model(generation); shadow_rebuild.observe_model(generation);
    off_rebuild.update_trust(true, true, true); shadow_rebuild.update_trust(true, true, true);
    off_candidate.update(a.valid, false, generation, a.face_id, a.whole_ekf_geometry,
      image, off.debug_aim_point.xyza[3], 0);
    shadow_candidate.update(b.valid, false, generation, b.face_id, b.whole_ekf_geometry,
      image, production.debug_aim_point.xyza[3], 0);
    const bool fire_a = a.valid && off_candidate.frames >= 4 && !off_candidate.sent;
    const bool fire_b = b.valid && shadow_candidate.frames >= 4 && !shadow_candidate.sent;
    assert(fire_a == fire_b);
    if (fire_a) {
      ++requests;
      assert(off_rebuild.record_request(requests, true) == shadow_rebuild.record_request(requests, true));
      off_candidate.sent = shadow_candidate.sent = true;
    }
    assert(off_rebuild.sent_requests() == shadow_rebuild.sent_requests());
    assert(off_rebuild.pending() == shadow_rebuild.pending());
    // Full translation and the online model must be identical for both copies.
    auto legacy_copy = target, stable_copy = target;
    legacy_copy.predict(image + std::chrono::milliseconds(400));
    stable_copy.predict_for_aim(image + std::chrono::milliseconds(400), omega * 1.03);
    for (int index : {0, 1, 2, 3, 4, 5, 8, 9, 10})
      assert(legacy_copy.ekf().x[index] == stable_copy.ekf().x[index]);
    assert(legacy_copy.ekf().P.isApprox(stable_copy.ekf().P, 0.0));
  }
  assert(valid > 100 && compared > 100 && changed > 100 && requests > 0);
  // Static/low-speed and the legacy geometry must ignore the optional gyro rate.
  Eigen::VectorXd stationary(11); stationary << 4, 0, 0, 0, 0, 0, 0, 0, .26, 0, 0;
  auto target = auto_aim::Target::prediction_snapshot(
    stationary, origin, auto_aim::ArmorName::two, auto_aim::ArmorType::small, true, 0);
  auto_aim::AimPredictionContext option; option.evaluation_time = origin;
  option.rotation_rate_rad_s = 4.5;
  auto_aim::AimPredictionTrace trace;
  off.aim({target}, origin, 19.8, true, &trace, auto_aim::AimGeometry::whole_ekf, option);
  assert(!trace.rotation_override_used);
  std::cout << "rotation_prediction_test: PASS (independent shadow, frozen clock/rate, translation, gates/counters)\n";
}
