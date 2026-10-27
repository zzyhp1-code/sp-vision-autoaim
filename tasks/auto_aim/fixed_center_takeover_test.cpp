#include "fixed_center_takeover.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
using Shadow = auto_aim::FixedCenterShadow;
using Policy = auto_aim::FixedCenterTakeover;
int checks = 0;
void require(bool condition, const char * message)
{ ++checks; if (!condition) throw std::runtime_error(message); }
std::int64_t ns(int ms) { return static_cast<std::int64_t>(ms) * 1000000; }
Shadow::Context context(double rate = 5.0)
{ return {7, 3, 1, 1, 2, rate > 0 ? 1 : -1, 20.0, 0.17, rate, std::hypot(4.0, 1.0)}; }
Shadow::Seed seed(int ms, double rate)
{ return {4.02, 0.97, 0.32 + rate * (ms - 1000) / 1000.0, rate, {0.23, 0.27}, {0.1, 0.2}}; }
std::vector<Shadow::Observation> observations(int ms, double rate, double dx = 0, double dy = 0)
{
  std::vector<Shadow::Observation> out;
  for (int f = 0; f < 4; ++f) {
    const double phase = 0.3 + rate * (ms - 1000) / 1000.0 + f * 1.57079632679489661923;
    const double r = f % 2 ? 0.26 : 0.22;
    out.push_back({f, {4.0 + dx - r * std::cos(phase), 1.0 + dy - r * std::sin(phase),
      f % 2 ? 0.18 : 0.15}});
  }
  return out;
}
void train(Shadow & shadow, double rate = 5.0, int violation = 0)
{
  for (int ms = 1000; ms <= 4000; ms += 20) {
    const double dx = violation == 1 && ms > 3500 ? 0.8 * (ms - 3500) / 1000.0 : 0.0;
    const double dy = violation == 2 && ms > 3500 ? 0.4 : 0.0;
    shadow.update(ns(ms), context(rate), seed(ms, rate), observations(ms, rate, dx, dy));
  }
  require(shadow.output().evaluable, "training model converges before policy evaluation");
}
void stability_and_fallback(double rate)
{
  Shadow shadow; train(shadow, rate);
  Policy policy;
  auto d = policy.update(true, true, ns(4000), ns(4001), context(rate), shadow);
  require(!d.use_fixed && std::string(d.reason) == "settling", "one successful revision cannot take over");
  Policy cached;
  cached.update(true, true, ns(4000), ns(4001), context(rate), shadow);
  d = cached.update(true, true, ns(4000), ns(4220), context(rate), shadow);
  require(!d.use_fixed, "repeated cached success cannot mature the policy");
  for (int ms = 4020; ms <= 4260; ms += 20) {
    shadow.update(ns(ms), context(rate), seed(ms, rate), observations(ms, rate));
    d = policy.update(true, true, ns(ms), ns(ms + 1), context(rate), shadow);
    if (ms < 4200) require(!d.use_fixed, "200 ms and three new revisions are both required");
  }
  require(d.use_fixed && d.snapshot.valid_at(ns(4261)), "stable supported candidate can take over");
  auto malformed = d.snapshot; malformed.revision = 0;
  require(!malformed.valid_at(ns(4261)), "unversioned snapshot cannot acquire authority");
  malformed = d.snapshot; malformed.context.rotation_sign = 2;
  require(!malformed.valid_at(ns(4261)), "invalid rotation ownership is rejected");
  const auto face = d.snapshot.face_at(ns(4600), 3);
  const auto truth = observations(4600, rate)[3].xyz;
  require(std::hypot(face[0] - truth[0], face[1] - truth[1]) < 1e-5,
    "snapshot preserves exact same-face future geometry in both directions");
  require(!d.snapshot.valid_at(d.snapshot.valid_until_ns + 1), "send-time expiry is strict");
  d = policy.update(true, true, ns(4280), ns(4281), context(rate), shadow, false);
  require(!d.use_fixed && std::string(d.reason) == "no_observation",
    "brief observation gap cannot keep fixed-centre authority");
  d = policy.update(true, true, ns(4280), ns(4281), context(rate), shadow, true);
  require(d.use_fixed, "fresh recovery can reuse still-valid model quality, with request maturity isolated separately");
  d = policy.update(true, false, ns(4280), ns(4281), context(rate), shadow);
  require(!d.use_fixed && std::string(d.reason) == "ineligible",
    "lost observation/trust/mode closes this frame's request");
  d = policy.update(true, true, ns(4280), ns(4281), context(rate), shadow);
  require(d.use_fixed,
    "restored eligibility reuses preserved model quality instead of re-settling");
  d = policy.update(false, true, ns(4280), ns(4281), context(rate), shadow);
  require(!d.use_fixed && std::string(d.reason) == "disabled", "configuration switch always returns to baseline");
  d = policy.update(true, true, ns(4280), ns(4281), context(rate), shadow);
  require(!d.use_fixed && std::string(d.reason) == "settling",
    "an explicit disable is the only kind of frame that drops the quality history");
  d = policy.update(true, true, ns(4600), ns(4601), context(rate), shadow);
  require(!d.use_fixed, "expired fit cannot retain takeover authority");
  auto changed = context(rate); ++changed.model_generation;
  shadow.update(ns(4620), changed, seed(4620, rate), observations(4620, rate));
  d = policy.update(true, true, ns(4620), ns(4621), changed, shadow);
  require(!d.use_fixed, "new tracker generation cannot use old fit or stability history");
}
void quality_failures()
{
  for (int violation : {1, 2}) {
    Shadow shadow; train(shadow, 5.0, violation); Policy policy;
    const auto d = policy.update(true, true, ns(4000), ns(4001), context(), shadow);
    require(!d.use_fixed && d.forecast.available && !d.forecast.stationary_supported,
      "movement or heldout error falls back without blocking baseline prediction");
  }
  Shadow shadow; train(shadow); Policy policy;
  auto wrong = context(); wrong.source = 9;
  const auto d = policy.update(true, true, ns(4000), ns(4001), wrong, shadow);
  require(!d.use_fixed && std::string(d.reason) == "context_changed", "model ownership checked before use");
}
}  // namespace
int main()
{
  try {
    stability_and_fallback(5.0); stability_and_fallback(-5.0); quality_failures();
    std::cout << "fixed_center_takeover_test: PASS (" << checks << " checks)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & e) {
    std::cerr << "fixed_center_takeover_test: FAIL: " << e.what() << '\n'; return EXIT_FAILURE;
  }
}
