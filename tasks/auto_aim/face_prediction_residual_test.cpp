#include "face_prediction_residual.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using Audit = auto_aim::FacePredictionResidual;
constexpr double pi = 3.14159265358979323846;
int checks = 0;

void require(bool condition, const char * message)
{
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
bool near(double a, double b, double tolerance = 1e-10)
{
  return std::abs(a - b) <= tolerance;
}
std::int64_t ns(int ms) { return static_cast<std::int64_t>(ms) * 1000000; }
Audit::Context context()
{
  Audit::Context c;
  c.model_generation = 1;
  c.target_id = 3;
  c.geometry = 1;
  c.source = 1;
  c.speed_mode = 1;
  c.rotation_sign = 1;
  c.bullet_speed_mps = 20.0;
  c.base_delay_s = 0.16;
  c.omega_rad_s = 2.0 * pi;
  c.center_range_m = 4.0;
  return c;
}
Audit::Observation observation(int face = 0, Audit::Xyz xyz = {4.0, 0.0, 0.0})
{
  return {face, xyz};
}
Audit::Prediction prediction(
  int issued_ms, int impact_ms, int face = 0, Audit::Xyz xyz = {4.0, 0.0, 0.0},
  const Audit::Context & c = context())
{
  return {ns(issued_ms), ns(impact_ms), face, xyz, c};
}

void bracket_and_identity_tests()
{
  const auto c = context();
  Audit audit;
  audit.observe(ns(1000), c, {observation()});
  require(audit.enqueue(prediction(1000, 1100, 0, {4.0, 0.15, 0.3})), "enqueue valid");
  audit.observe(ns(1090), c, {observation(0, {4.0, 0.0, 0.1})});
  require(audit.observe(ns(1110), c, {observation(0, {4.0, 0.2, 0.3})}).empty(),
    "must wait for full allowed reference window");
  const auto matches = audit.observe(ns(1130), c, {});
  require(matches.size() == 1, "same face bracket matched");
  const auto & m = matches.front();
  require(m.left_ns == ns(1090) && m.right_ns == ns(1110), "reference timestamps preserved");
  require(near(m.observed_xyz[1], 0.1) && near(m.observed_xyz[2], 0.2),
    "interpolate XYZ before computing bearing");
  require(near(m.yaw_error_rad, std::atan2(0.15, 4.0) - std::atan2(0.1, 4.0)),
    "raw predicted minus observed bearing");
  require(near(m.position_error_m, std::hypot(0.05, 0.1)), "position residual");
  require(m.context.model_generation == 1 && m.context.source == 1 && m.passage_id == 1,
    "forecast provenance retained");
  require(audit.summary(0).completed_cycles == 0, "one matched frame is not independent evidence");

  for (int test = 0; test < 4; ++test) {
    Audit missing;
    missing.observe(ns(1000), c, {observation()});
    require(missing.enqueue(prediction(1000, 1100)), "queue missing-reference case");
    if (test == 0) {  // Same-face reference on the left only.
      missing.observe(ns(1090), c, {observation()});
    } else if (test == 1) {  // A different face cannot supply the right endpoint.
      missing.observe(ns(1090), c, {observation()});
      missing.observe(ns(1110), c, {observation(1)});
    } else if (test == 2) {  // Both endpoints exist but the left is too far away.
      missing.observe(ns(1060), c, {observation()});
      missing.observe(ns(1110), c, {observation()});
    } else {  // An isolated exact-time observation is not a full local bracket.
      missing.observe(ns(1100), c, {observation()});
    }
    require(missing.observe(ns(1130), c, {}).empty(), "missing local bracket never extrapolated");
    require(missing.summary(0).matched == 0 && missing.summary(0).unmatched == 1,
      "missing references counted explicitly");
  }

  Audit exact;
  exact.observe(ns(1000), c, {observation()});
  exact.enqueue(prediction(1000, 1100));
  exact.observe(ns(1090), c, {observation()});
  exact.observe(ns(1100), c, {observation()});
  require(exact.observe(ns(1130), c, {}).size() == 1, "exact endpoint with local second sample");

  Audit wrap;
  wrap.observe(ns(1000), c, {observation()});
  wrap.enqueue(prediction(1000, 1100, 0, {-4.0, 0.04, 0.0}));
  wrap.observe(ns(1090), c, {observation(0, {-4.0, -0.04, 0.0})});
  wrap.observe(ns(1110), c, {observation(0, {-4.0, -0.04, 0.0})});
  const auto wrapped = wrap.observe(ns(1130), c, {});
  require(wrapped.size() == 1 && std::abs(wrapped.front().yaw_error_rad) < 0.03,
    "bearing error wraps across pi");
}

void context_and_input_tests()
{
  const auto c = context();
  std::vector<Audit::Context> changed(10, c);
  changed[0].model_generation++;
  changed[1].target_id++;
  changed[2].geometry++;
  changed[3].source++;
  changed[4].speed_mode++;
  changed[5].rotation_sign = -1;
  changed[5].omega_rad_s = -c.omega_rad_s;
  changed[6].bullet_speed_mps += 0.6;
  changed[7].base_delay_s += 0.002;
  changed[8].omega_rad_s += 1.1;
  changed[9].center_range_m += 0.6;
  for (const auto & next : changed) {
    Audit audit;
    audit.observe(ns(1000), c, {observation()});
    audit.enqueue(prediction(1000, 1100));
    const auto epoch = audit.summary(0).epoch;
    require(audit.observe(ns(1090), next, {observation()}).empty(), "context reset no stale match");
    require(audit.summary(0).epoch > epoch && audit.summary(0).pending == 0,
      "context keys and anchored numeric changes reset evidence");
  }
  Audit drift;
  drift.observe(ns(1000), c, {observation()});
  drift.enqueue(prediction(1000, 1100));
  auto slowly_changed = c;
  slowly_changed.base_delay_s += 0.0006;
  drift.observe(ns(1010), slowly_changed, {observation()});
  require(drift.summary(0).pending == 1, "small jitter keeps anchor");
  slowly_changed.base_delay_s += 0.0006;
  drift.observe(ns(1020), slowly_changed, {observation()});
  require(drift.summary(0).pending == 0, "gradual context drift compared to original anchor");

  for (int clock_case = 0; clock_case < 3; ++clock_case) {
    Audit audit;
    audit.observe(ns(1000), c, {observation()});
    audit.enqueue(prediction(1000, 1100));
    const auto epoch = audit.summary(0).epoch;
    const int next = clock_case == 0 ? 1000 : (clock_case == 1 ? 990 : 1300);
    audit.observe(ns(next), c, {});
    require(audit.summary(0).pending == 0 && audit.summary(0).epoch > epoch,
      "duplicate/backward/gapped capture clocks reset");
  }

  Audit ambiguous;
  ambiguous.observe(ns(1000), c, {observation(), observation()});
  require(!ambiguous.enqueue(prediction(1000, 1100)), "duplicate face IDs are not accepted evidence");
  ambiguous.observe(ns(1010), c, {observation()});
  require(ambiguous.enqueue(prediction(1010, 1110)), "unique association accepted");
  ambiguous.observe(ns(1020), c, {});
  require(!ambiguous.enqueue(prediction(1020, 1120)), "coast-only frame cannot enqueue");
  ambiguous.observe(ns(1030), c, {observation(0, {4.0, NAN, 0.0})});
  require(!ambiguous.enqueue(prediction(1030, 1130)), "non-finite observation rejected");
  ambiguous.observe(ns(1040), c, {observation()});
  auto invalid = prediction(1040, 1140);
  invalid.xyz[2] = std::numeric_limits<double>::infinity();
  require(!ambiguous.enqueue(invalid), "non-finite forecast rejected");
  invalid = prediction(1040, 1040);
  require(!ambiguous.enqueue(invalid), "non-future forecast rejected");
  invalid = prediction(1040, 4000);
  require(!ambiguous.enqueue(invalid), "forecast age bounded");

  Audit horizon;
  horizon.observe(ns(1000), c, {observation()});
  horizon.enqueue(prediction(1000, 1100));
  require(!horizon.enqueue(prediction(1000, 1300)) && horizon.summary(0).pending == 0,
    "changed forecast horizon resets evidence");

  Audit::Config invalid_config;
  invalid_config.bracket_ns = 0;
  bool threw = false;
  try { Audit invalid_audit(invalid_config); } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "invalid audit limits rejected");
}

void bounded_pending_and_order_tests()
{
  Audit::Config config;
  config.max_pending = 2;
  Audit audit(config);
  const auto c = context();
  audit.observe(ns(1000), c, {observation()});
  require(audit.enqueue(prediction(1000, 1200)), "queue later target first");
  require(audit.enqueue(prediction(1000, 1100)), "out-of-order impact targets sorted");
  require(audit.enqueue(prediction(1000, 1150)), "pending remains bounded");
  require(audit.summary(0).pending == 2 && audit.summary(0).unmatched == 1,
    "capacity eviction counted");
  audit.observe(ns(1140), c, {observation()});
  audit.observe(ns(1160), c, {observation()});
  const auto matches = audit.observe(ns(1180), c, {});
  require(matches.size() == 1 && matches.front().prediction_ns == ns(1150),
    "earliest retained impact matures first");
  audit.observe(ns(1190), c, {observation()});
  audit.observe(ns(1210), c, {observation()});
  require(audit.observe(ns(1230), c, {}).size() == 1 && audit.summary(0).pending == 0,
    "later retained impact matures independently");
}

// Keep capture frames continuous while a physical face is out of view.
void advance_idle(Audit & audit, const Audit::Context & c, int & time_ms, int until_ms)
{
  while (time_ms < until_ms) {
    time_ms = std::min(time_ms + 100, until_ms);
    audit.observe(ns(time_ms), c, {});
  }
}
void passage(Audit & audit, const Audit::Context & c, int & time_ms, int start_ms,
  double bias_rad, int forecast_stride_ms)
{
  advance_idle(audit, c, time_ms, start_ms - 10);
  for (int offset = 0; offset <= 300; offset += 10) {
    time_ms = start_ms + offset;
    audit.observe(ns(time_ms), c, {observation()});
    if (offset <= 150 && offset % forecast_stride_ms == 0) {
      require(audit.enqueue(prediction(time_ms, time_ms + 100, 0,
        {4.0 * std::cos(bias_rad), 4.0 * std::sin(bias_rad), 0.0}, c)),
        "passage forecast accepted");
    }
  }
  advance_idle(audit, c, time_ms, start_ms + 460);
}

void independent_passage_tests()
{
  auto c = context();
  Audit audit;
  int time_ms = 990;
  passage(audit, c, time_ms, 1000, 0.01, 10);
  require(audit.summary(0).completed_cycles == 1, "adjacent frames yield one completed passage");
  passage(audit, c, time_ms, 2000, 0.02, 50);
  passage(audit, c, time_ms, 3000, 0.03, 50);
  const auto s = audit.summary(0);
  require(s.completed_cycles == 3 && s.matched == 24, "three distinct turns give three samples");
  require(near(s.bias_rad, 0.02) && near(s.scatter_rad, 0.01) && near(s.p90_abs_rad, 0.03),
    "cycles weighted equally despite different frame counts");
  require(audit.summary(1).completed_cycles == 0 && std::isnan(audit.summary(1).bias_rad),
    "evidence never borrowed from another face");
  auto replaced = c;
  replaced.model_generation++;
  audit.observe(ns(time_ms + 10), replaced, {observation()});
  require(audit.summary(0).completed_cycles == 0 && audit.summary(0).matched == 0,
    "new tracker generation clears learned passages");

  Audit split;
  time_ms = 990;
  passage(split, c, time_ms, 1000, 0.01, 50);
  passage(split, c, time_ms, 1500, 0.02, 50);
  require(split.summary(0).completed_cycles == 1,
    "visibility dropout within one revolution is not independent evidence");
  passage(split, c, time_ms, 2000, 0.03, 50);
  require(split.summary(0).completed_cycles == 2,
    "subsequent full revolution can add independent evidence");

  Audit wrapped;
  time_ms = 990;
  passage(wrapped, c, time_ms, 1000, pi - 0.01, 50);
  passage(wrapped, c, time_ms, 2000, -pi + 0.01, 50);
  require(std::abs(wrapped.summary(0).bias_rad) > 3.1 &&
    near(wrapped.summary(0).scatter_rad, 0.01), "circular cycle median retains large errors");

  Audit::Config short_history;
  short_history.history_age_ns = ns(500);
  Audit expiry(short_history);
  time_ms = 990;
  passage(expiry, c, time_ms, 1000, 0.01, 50);
  require(expiry.summary(0).completed_cycles == 1, "recent cycle kept");
  advance_idle(expiry, c, time_ms, 1800);
  require(expiry.summary(0).completed_cycles == 0, "old cycle statistics expire");

  c.rotation_sign = -1;
  c.omega_rad_s = -2.0 * pi;
  Audit reverse;
  time_ms = 990;
  passage(reverse, c, time_ms, 1000, -0.02, 50);
  require(reverse.summary(0).completed_cycles == 1 && near(reverse.summary(0).bias_rad, -0.02),
    "reverse rotation uses absolute period but signed residual");
}
}  // namespace

int main()
{
  try {
    bracket_and_identity_tests();
    context_and_input_tests();
    bounded_pending_and_order_tests();
    independent_passage_tests();
    std::cout << "face_prediction_residual_test: PASS (" << checks << " checks)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & e) {
    std::cerr << "face_prediction_residual_test: FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
