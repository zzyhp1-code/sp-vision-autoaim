#include "fixed_center_prediction_audit.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
using Audit = auto_aim::FixedCenterPredictionAudit;
using Shadow = auto_aim::FixedCenterShadow;
using Residual = auto_aim::FacePredictionResidual;
constexpr double pi = 3.14159265358979323846;
int checks = 0;
void require(bool condition, const char * message)
{
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
std::int64_t ns(int ms) { return static_cast<std::int64_t>(ms) * 1000000; }
Residual::Context context()
{
  return {1, 3, 1, 1, 1, 1, 20.0, 0.17, 5.0, std::hypot(4.0, 1.0)};
}
Residual::Xyz truth(int ms, int face)
{
  const double phase = 0.3 + 5.0 * (ms - 1000) / 1000.0 + face * pi / 2.0;
  const double radius = face % 2 ? 0.26 : 0.22;
  return {4.0 - radius * std::cos(phase), 1.0 - radius * std::sin(phase),
    face % 2 ? 0.18 : 0.15};
}
std::vector<Residual::Observation> observations(int ms, double dx = 0.0)
{
  std::vector<Residual::Observation> out;
  for (int face = 0; face < 4; ++face) {
    auto xyz = truth(ms, face); xyz[0] += dx; out.push_back({face, xyz});
  }
  return out;
}
void train(Shadow & shadow, bool moving = false)
{
  for (int ms = 1000; ms <= 4000; ms += 20) {
    Shadow::Seed seed{4.05, 0.96, 0.35 + 5.0 * (ms - 1000) / 1000.0,
      5.1, {0.23, 0.27}, {0.2, 0.2}};
    shadow.update(ns(ms), context(), seed,
      observations(ms, moving && ms > 3500 ? 0.8 * (ms - 3500) / 1000.0 : 0.0));
  }
  require(shadow.output().evaluable, "synthetic training produces a converged candidate");
}
Residual::Prediction prediction(int ms, int horizon = 200, int face = 0)
{
  auto xyz = truth(ms + horizon, face); xyz[1] += 0.03;
  return {ns(ms + 1), ns(ms + horizon), face, xyz, context()};
}

void paired_future_tests()
{
  Shadow shadow; train(shadow);
  Audit audit;
  std::vector<Audit::Pair> pairs;
  for (int ms = 4000; ms <= 4420; ms += 20) {
    const auto matched = audit.observe(ns(ms), context(), observations(ms));
    pairs.insert(pairs.end(), matched.begin(), matched.end());
    if (ms <= 4040) require(audit.enqueue(prediction(ms), shadow).enqueued,
      "same frame and future face enqueue both predictions together");
    if (ms == 4100) shadow.reset();
  }
  require(pairs.size() == 3 && audit.counters().matched == 3,
    "issued evidence survives later candidate reset and all pairs match");
  for (const auto & pair : pairs) {
    require(pair.main.observed_xyz == pair.fixed.observed_xyz &&
      pair.main.left_ns == pair.fixed.left_ns && pair.main.right_ns == pair.fixed.right_ns &&
      pair.main.prediction_ns == pair.fixed.prediction_ns && pair.main.face_id == pair.fixed.face_id,
      "both errors use the exact same physical face and visual time bracket");
    require(std::abs(pair.fixed.yaw_error_rad) < 1e-4 &&
      std::abs(pair.main.yaw_error_rad) > 0.005 && pair.metadata.stationary_supported,
      "independent fixed-centre forecast beats deliberately biased baseline");
  }
  for (int ms = 4440; ms <= 4620; ms += 20) audit.observe(ns(ms), context(), {});
  const auto main = audit.main_summary(0), fixed = audit.fixed_summary(0);
  require(main.completed_cycles == 1 && fixed.completed_cycles == 1 &&
    main.p90_abs_rad > fixed.p90_abs_rad + 0.005,
    "paired summaries have identical independent-cycle weighting");
  require(audit.counters().pairing_errors == 0 && audit.counters().no_reference == 0,
    "well bracketed cohort has no unmatched or pairing failures");
}

void coverage_and_reset_tests()
{
  Shadow shadow; train(shadow);
  Audit audit;
  audit.observe(ns(4000), context(), observations(4000));
  require(audit.enqueue(prediction(4000), shadow).enqueued, "enqueue before temporary loss");
  for (int ms = 4020; ms <= 4260; ms += 20) audit.observe(ns(ms), context(), {});
  require(audit.counters().no_reference == 1 && audit.main_summary(0).unmatched == 1 &&
    audit.fixed_summary(0).unmatched == 1, "missing future observations count against both cohorts");

  Audit isolated;
  isolated.observe(ns(4000), context(), observations(4000));
  isolated.enqueue(prediction(4000), shadow);
  auto changed = context(); ++changed.model_generation;
  isolated.observe(ns(4020), changed, observations(4020));
  require(isolated.counters().discarded_reset == 1 && isolated.main_summary(0).pending == 0,
    "model replacement discards pending pair and records evidence loss");
  const auto wrong_model = isolated.enqueue({ns(4021), ns(4220), 0, truth(4220, 0), changed}, shadow);
  require(!wrong_model.enqueued && wrong_model.forecast.reason == Shadow::Reason::context_changed,
    "a candidate from another model cannot enter a paired cohort");
  isolated.skip_main();
  require(isolated.counters().main_skipped == 1, "invalid native predictions retain coverage denominator");

  Audit stale;
  stale.observe(ns(4260), context(), observations(4260));
  require(!stale.enqueue(prediction(4260), shadow).enqueued && stale.counters().available == 0,
    "cached successful model expires at actual issue time");

  Audit source;
  source.observe(ns(4000), context(), {});
  require(!source.enqueue(prediction(4000), shadow).enqueued && source.counters().source_rejected == 1,
    "coast-only source cannot generate paired evidence");

  Residual::Config bounded; bounded.max_pending = 2;
  Audit small(bounded);
  small.observe(ns(4000), context(), observations(4000));
  for (int i = 0; i < 3; ++i) {
    auto p = prediction(4000); p.issued_ns += i; p.prediction_ns += i;
    require(small.enqueue(p, shadow).enqueued, "bounded paired queue accepts valid input");
  }
  require(small.counters().overflow == 1 && small.main_summary(0).pending == 2 &&
    small.fixed_summary(0).pending == 2, "overflow evicts the same forecast from both cohorts");
  small.reset(); small.reset();
  require(small.counters().discarded_reset == 2, "repeated no-target reset is idempotent");

  Audit horizon;
  horizon.observe(ns(4000), context(), observations(4000));
  horizon.enqueue(prediction(4000), shadow);
  horizon.observe(ns(4020), context(), observations(4020));
  require(!horizon.enqueue(prediction(4020, 400), shadow).enqueued &&
    horizon.counters().discarded_reset == 1 && horizon.counters().pairing_errors == 0,
    "horizon regime change isolates both scorers and counts discarded pending evidence");
}

void unsupported_candidates_still_score()
{
  Shadow moving; train(moving, true);
  Audit audit;
  audit.observe(ns(4000), context(), observations(4000, 0.4));
  const auto result = audit.enqueue(prediction(4000), moving);
  require(result.enqueued && !result.forecast.stationary_supported &&
    result.forecast.reason == Shadow::Reason::moving_center,
    "moving candidate is labelled but not removed from the comparison");
  std::vector<Audit::Pair> pairs;
  for (int ms = 4020; ms <= 4260; ms += 20) {
    const auto matched = audit.observe(ns(ms), context(), observations(ms, 1.0));
    pairs.insert(pairs.end(), matched.begin(), matched.end());
  }
  require(pairs.size() == 1 && pairs[0].fixed.position_error_m > 0.9 &&
    !pairs[0].metadata.stationary_supported && audit.counters().stationary_matched == 0,
    "large future errors and movement labels survive matching without trimming");
}
}  // namespace

int main()
{
  try {
    paired_future_tests(); coverage_and_reset_tests(); unsupported_candidates_still_score();
    std::cout << "fixed_center_prediction_audit_test: PASS (" << checks << " checks)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & e) {
    std::cerr << "fixed_center_prediction_audit_test: FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
