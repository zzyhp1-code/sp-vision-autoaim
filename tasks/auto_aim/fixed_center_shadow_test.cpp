#include "fixed_center_shadow.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using Shadow = auto_aim::FixedCenterShadow;
constexpr double pi = 3.14159265358979323846;
int checks = 0;
void require(bool condition, const char * message)
{
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
bool near(double a, double b, double tolerance = 1e-5) { return std::abs(a - b) < tolerance; }
std::int64_t ns(int ms) { return static_cast<std::int64_t>(ms) * 1000000; }
Shadow::Context context(double omega = 5.0)
{
  Shadow::Context c;
  c.model_generation = 1;
  c.target_id = 3;
  c.geometry = 1;
  c.source = 1;
  c.speed_mode = 1;
  c.rotation_sign = omega > 0.0 ? 1 : -1;
  c.bullet_speed_mps = 20.0;
  c.base_delay_s = 0.16;
  c.omega_rad_s = omega;
  c.center_range_m = std::hypot(4.0, 1.0);
  return c;
}
Shadow::Seed seed(int time_ms, double omega = 5.0)
{
  // Deliberately imperfect EKF seed. The shadow must fit the observations.
  return {4.08, 0.96, 0.4 + omega * (time_ms - 1000) / 1000.0,
    omega + (omega > 0.0 ? 0.15 : -0.15), {0.23, 0.27}, {0.2, 0.2}};
}
Shadow::Xyz truth(int time_ms, int face, double omega = 5.0, double dx = 0.0, double dy = 0.0)
{
  const double phase = 0.3 + omega * (time_ms - 1000) / 1000.0 + face * pi / 2.0;
  const double radius = face % 2 ? 0.26 : 0.22;
  return {4.0 + dx - radius * std::cos(phase), 1.0 + dy - radius * std::sin(phase),
    face % 2 ? 0.18 : 0.15};
}
std::vector<Shadow::Observation> visible_observations(int time_ms, double omega,
  double dx = 0.0, double dy = 0.0)
{
  std::vector<Shadow::Observation> out;
  for (int face = 0; face < 4; ++face) {
    const double phase = 0.3 + omega * (time_ms - 1000) / 1000.0 + face * pi / 2.0;
    const double alignment = std::remainder(phase - std::atan2(1.0, 4.0), 2.0 * pi);
    if (std::abs(alignment) < 0.95) out.push_back({face, truth(time_ms, face, omega, dx, dy)});
  }
  return out;
}
Shadow::Output run(Shadow & shadow, double omega = 5.0, bool move_holdout = false,
  bool change_holdout_seed = false)
{
  const auto c = context(omega);
  Shadow::Output out;
  for (int time_ms = 1000; time_ms <= 4000; time_ms += 20) {
    const bool displaced = move_holdout && time_ms > 3500;
    auto current_seed = seed(time_ms, omega);
    if (change_holdout_seed && time_ms > 3500) {
      current_seed.center_x += 1.0;
      current_seed.center_y -= 0.8;
      current_seed.phase_rad += 1.0;
    }
    out = shadow.update(ns(time_ms), c, current_seed,
      visible_observations(time_ms, omega, displaced ? 0.2 : 0.0, displaced ? 0.12 : 0.0));
  }
  return out;
}

void fit_and_holdout_tests()
{
  Shadow shadow;
  const auto out = run(shadow);
  require(out.fitted && out.evaluable && out.revision > 0, "chronological shadow fit converges");
  require(out.train_samples >= 32 && out.heldout_samples >= 8 && out.heldout_face_mask != 0,
    "training and later holdout have explicit coverage");
  require(out.heldout_yaw_p90_rad < 1e-5 && out.heldout_position_p90_m < 1e-4,
    "stationary rotating target predicts chronological heldout observations");
  require(near(out.model.state.center_x, 4.0) && near(out.model.state.center_y, 1.0) &&
    near(out.model.state.radius_m[0], 0.22) && near(out.model.state.radius_m[1], 0.26),
    "shared centre and both radii recovered from partial visibility");
  require(near(out.model.state.omega_rad_s, 5.0) && near(out.model.state.height_m[0], 0.15) &&
    near(out.model.state.height_m[1], 0.18), "rate and parity heights recovered");
  const auto point = shadow.predict(ns(4250), 3);
  const auto actual = truth(4250, 3);
  require(point && near((*point)[0], actual[0]) && near((*point)[1], actual[1]) &&
    near((*point)[2], actual[2]), "future same-face forecast uses one rigid geometry");
  require(!shadow.predict(ns(4000), 0) && !shadow.predict(ns(7000), 0) &&
    !shadow.predict(ns(4250), 4), "future forecast time and physical face bounds enforced");
  const auto forecast = shadow.forecast(ns(4005), ns(4250), 3);
  require(forecast.available && forecast.stationary_supported &&
    forecast.reason == Shadow::Reason::ready && near(forecast.xyz[0], actual[0]),
    "online forecast checks actual issue time and predicts the requested face");
  require(!shadow.forecast(ns(3990), ns(4250), 0).available &&
    shadow.forecast(ns(4005), ns(4000), 0).reason == Shadow::Reason::invalid_time &&
    shadow.forecast(ns(4005), ns(4250), 4).reason == Shadow::Reason::invalid_face,
    "online forecast rejects invalid issue/future time and physical face");
  require(!shadow.forecast(ns(4260), ns(4500), 0).available,
    "time passing without update cannot keep an old candidate available");
  shadow.update(ns(4220), context(), seed(4220), {});
  require(shadow.forecast(ns(4260), ns(4500), 0).reason == Shadow::Reason::stale_observation,
    "fresh frame with no new observations cannot hide stale observations");

  Shadow shifted;
  const auto bad = run(shifted, 5.0, true);
  require(bad.evaluable && bad.heldout_position_p90_m > 0.2 && bad.heldout_yaw_p90_rad > 0.01,
    "moving heldout centre produces large diagnostic error");
  require(near(bad.model.state.center_x, 4.0) && near(bad.model.state.center_y, 1.0),
    "heldout points never leak into the trained candidate");
  require(bad.heldout_position_p90_m > out.heldout_position_p90_m + 0.1,
    "score distinguishes supported and violated fixed-centre assumptions");
  const auto unsupported = shifted.forecast(ns(4005), ns(4250), 0);
  require(unsupported.available && !unsupported.stationary_supported,
    "large heldout error stays available for unbiased parallel scoring");

  Shadow drifting;
  for (int ms = 1000; ms <= 4000; ms += 20)
    drifting.update(ns(ms), context(), seed(ms),
      visible_observations(ms, 5.0, ms > 3500 ? 0.8 * (ms - 3500) / 1000.0 : 0.0));
  const auto moving = drifting.forecast(ns(4005), ns(4250), 0);
  require(moving.available && !moving.stationary_supported &&
    moving.reason == Shadow::Reason::moving_center && moving.fit.heldout_center_speed_mps > 0.7,
    "heldout centre drift is labelled and retained for future error scoring");

  Shadow::Config short_age;
  short_age.max_prediction_age_ns = ns(10);
  Shadow expires(short_age);
  run(expires);
  require(expires.forecast(ns(4020), ns(4300), 0).reason == Shadow::Reason::stale_model,
    "model age uses issue time even with a recent accepted source observation");

  Shadow altered_seed;
  const auto different_seed = run(altered_seed, 5.0, false, true);
  require(different_seed.evaluable &&
    near(different_seed.model.state.center_x, out.model.state.center_x) &&
    near(different_seed.model.state.omega_rad_s, out.model.state.omega_rad_s) &&
    near(different_seed.heldout_position_p90_m, out.heldout_position_p90_m),
    "heldout-era EKF seed cannot influence training initializer or validation score");

  Shadow reverse;
  const auto reversed = run(reverse, -5.0);
  require(reversed.evaluable && reversed.heldout_position_p90_m < 1e-4 &&
    near(reversed.model.state.omega_rad_s, -5.0), "negative rotation fitted without face relabeling");
}

void isolation_and_storage_tests()
{
  const auto c = context();
  Shadow shadow;
  const auto before = run(shadow);
  auto replacement = c;
  replacement.model_generation++;
  auto out = shadow.update(ns(4020), replacement, seed(4020), visible_observations(4020, 5.0));
  require(out.epoch > before.epoch && !out.evaluable && out.samples <= 2 &&
    out.revision == 0, "tracker generation isolates shadow parameters and history");
  require(!shadow.predict(ns(4300), 0), "unmatured replacement cannot emit a forecast");
  const auto epoch = out.epoch;
  out = shadow.update(ns(4010), replacement, seed(4010), visible_observations(4010, 5.0));
  require(out.epoch > epoch && !out.evaluable, "backward capture time resets shadow");

  Shadow source_change;
  const auto source_before = run(source_change);
  replacement = c;
  replacement.source++;
  out = source_change.update(ns(4020), replacement, seed(4020), visible_observations(4020, 5.0));
  require(out.epoch > source_before.epoch && !out.evaluable, "predictor ownership change resets");

  Shadow loss;
  run(loss);
  for (int time_ms = 4020; time_ms <= 4320; time_ms += 20)
    out = loss.update(ns(time_ms), c, seed(time_ms), {});
  require(!out.evaluable && !loss.predict(ns(4500), 0), "stale observation/model cannot predict");

  Shadow missing_face;
  for (int time_ms = 1000; time_ms <= 5000; time_ms += 20) {
    out = missing_face.update(ns(time_ms), c, seed(time_ms), {{0, truth(time_ms, 0)}});
  }
  require(!out.evaluable && out.revision == 0, "unobserved faces cannot supply a complete model");

  Shadow duplicates;
  const auto sample = Shadow::Observation{0, truth(1000, 0)};
  out = duplicates.update(ns(1000), c, seed(1000), {sample, sample});
  require(out.samples == 0, "duplicate same-face associations are discarded");
  auto invalid = sample;
  invalid.xyz[0] = std::numeric_limits<double>::quiet_NaN();
  out = duplicates.update(ns(1020), c, seed(1020), {invalid});
  require(out.samples == 0, "non-finite solved observation discarded");

  Shadow::Config bounded;
  bounded.max_samples = 40;
  Shadow small_window(bounded);
  out = run(small_window);
  require(out.samples <= 40, "sample memory bounded independently of camera frame count");

  Shadow::Config invalid_config;
  invalid_config.holdout_ns = invalid_config.window_ns;
  bool threw = false;
  try { Shadow invalid_shadow(invalid_config); } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "invalid temporal partition rejected");

  Shadow failed;
  run(failed);
  bool failure_seen = false;
  for (int ms = 4020; ms <= 6500; ms += 20) {
    auto bad_points = visible_observations(ms, 5.0);
    for (auto & o : bad_points) { o.xyz[0] += 50.0; o.xyz[1] -= 30.0; }
    out = failed.update(ns(ms), c, seed(ms), bad_points);
    if (out.fit_attempted && out.reason == Shadow::Reason::fit_failed) {
      failure_seen = true;
      require(!out.evaluable && !failed.forecast(ns(ms + 1), ns(ms + 200), 0).available,
        "a newer failed fit immediately revokes the previous successful candidate");
      break;
    }
  }
  require(failure_seen && failed.counters().fit_failures > 0,
    "contradictory geometry exercises numerical fit failure and lifetime failure count");
}
}  // namespace

int main()
{
  try {
    fit_and_holdout_tests();
    isolation_and_storage_tests();
    std::cout << "fixed_center_shadow_test: PASS (" << checks << " checks)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & e) {
    std::cerr << "fixed_center_shadow_test: FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
