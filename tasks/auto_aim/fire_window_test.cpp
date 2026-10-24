#include "fire_window.hpp"

#include <array>
#include <cassert>
#include <iostream>
#include <string>

namespace
{
constexpr double deg = 3.14159265358979323846 / 180.0;
const auto_aim::FireWindowSelectionConfig selection{0.050, 0.5 * deg};

void test_bounded_selection()
{
  // Both rotation directions: four frames confirm eligibility, then let the
  // approaching plate get closer instead of issuing on the first mature frame.
  for (const double direction : {-1.0, 1.0}) {
    auto_aim::FireWindowSelector selector;
    const auto policy = auto_aim::make_fire_window_policy(false, true, true, 4);
    const std::array<double, 7> errors{2.9, 2.5, 2.1, 1.7, 1.2, 0.7, 0.3};
    for (int i = 0; i < static_cast<int>(errors.size()); ++i) {
      const auto decision = selector.update(
        true, policy.ready(true, i + 1, false, false), false,
        direction * errors[i] * deg, i * 0.016, selection);
      assert(decision.ready == (i == 6));
      if (i == 3) {
        assert(decision.started);
        assert(std::string(decision.reason) == "waiting");
      }
      if (i == 6) {
        assert(std::string(decision.reason) == "near_center");
        assert(decision.waited_s < selection.max_wait_s);
      }
    }
    assert(!selector.update(true, true, true, 0.2 * deg, 0.112, selection).ready);
  }

  // No local minimum inside the yaw gate: the first fresh frame after the
  // deadline still issues once, instead of recreating the old indefinite veto.
  {
    auto_aim::FireWindowSelector selector;
    for (int i = 0; i < 8; ++i) {
      const auto d = selector.update(
        true, i >= 3, false, (2.8 - 0.2 * i) * deg, i * 0.016, selection);
      assert(d.ready == (i == 7));
      if (i == 7) {
        assert(std::string(d.reason) == "deadline");
        assert(d.waited_s >= selection.max_wait_s);
        assert(d.waited_s < selection.max_wait_s + 0.016);
      }
    }
  }

  // Already moving away, or crossing between frames with a worse next sample:
  // issue immediately at maturity; there is no mandatory 50-ms extra delay.
  for (const double direction : {-1.0, 1.0}) {
    for (const std::array<double, 2> errors :
         {std::array<double, 2>{1.0, 1.3}, std::array<double, 2>{0.7, -0.8}}) {
      auto_aim::FireWindowSelector selector;
      assert(!selector.update(true, false, false, direction * errors[0] * deg, 0.0, selection).ready);
      const auto d = selector.update(true, true, false, direction * errors[1] * deg, 0.016, selection);
      assert(d.ready && std::string(d.reason) == "turning");
      assert(d.waited_s == 0.0);
    }
  }

  // The deadline cannot override a closed gate, AUTO/measurement loss, invalid
  // geometry, or an already-latched pulse. Reopening gets a new timing budget.
  {
    auto_aim::FireWindowSelector selector;
    assert(!selector.update(true, true, false, 2.0 * deg, 0.0, selection).ready);
    assert(!selector.update(false, true, false, 0.1 * deg, 0.080, selection).ready);
    auto d = selector.update(true, true, false, 1.5 * deg, 0.096, selection);
    assert(!d.ready && d.started && d.waited_s == 0.0);
    assert(!std::isfinite(d.next_error_rad));
    assert(!selector.update(true, true, false, NAN, 0.160, selection).ready);
    assert(!selector.update(true, true, false, 1.4 * deg, 0.176, selection).ready);
    assert(!selector.update(true, true, true, 0.1 * deg, 0.240, selection).ready);
    assert(!selector.update(true, false, false, 0.1 * deg, 0.256, selection).ready);
  }

  // Face changes reset both the derivative and the timer. A stale sample or a
  // discontinuous clock must not make the next face look like a turning point.
  for (const double next_time : {0.016, 0.0, -0.016, 0.200}) {
    auto_aim::FireWindowSelector selector;
    selector.update(true, true, false, 1.0 * deg, 0.0, selection);
    if (next_time == 0.016) selector.reset();  // Caller detected a face switch.
    const auto d = selector.update(true, true, false, 2.0 * deg, next_time, selection);
    assert(!d.ready && d.started && d.waited_s == 0.0);
    assert(!std::isfinite(d.next_error_rad));
  }

  // Small derivative noise does not masquerade as a turning point.
  {
    auto_aim::FireWindowSelector selector;
    selector.update(true, false, false, 1.0 * deg, 0.0, selection);
    assert(!selector.update(true, true, false, 1.01 * deg, 0.016, selection).ready);
    assert(selector.update(true, true, false, 1.01 * deg, 0.080, selection).ready);
  }

  // A zero budget restores the previous four-frame behavior for A/B testing,
  // but still never accepts a closed or immature candidate.
  {
    auto_aim::FireWindowSelector selector;
    auto disabled = selection;
    disabled.max_wait_s = 0.0;
    assert(!selector.update(true, false, false, 2.0 * deg, 0.0, disabled).ready);
    assert(selector.update(true, true, false, 2.0 * deg, 0.016, disabled).ready);
    assert(!selector.update(false, true, false, 2.0 * deg, 0.032, disabled).ready);
  }
}
}  // namespace

int main()
{
  // Regress the field failure: all angle/feedback gates can stay open while
  // the error decreases until the next face handoff, without a local minimum.
  // An unrelated PLL solution must not divert this stable window into a wait.
  for (const bool schedule_valid : {false, true}) {
    for (const bool interval_safe : {false, true}) {
      const auto policy = auto_aim::make_fire_window_policy(
        false, schedule_valid, interval_safe, 4);
      assert(policy.timing == auto_aim::FireTiming::stable_window);
      assert(!policy.uses_event_clock());
      const std::array<double, 6> errors_deg{2.9, 2.5, 2.1, 1.7, 1.3, 0.9};
      bool latched = false;
      int requests = 0;
      for (int frame = 0; frame < static_cast<int>(errors_deg.size()); ++frame) {
        const bool candidate_valid = errors_deg[frame] <= 3.0;
        const bool ready = policy.ready(candidate_valid, frame + 1, latched, false);
        if (frame < 3) assert(!ready);
        if (frame == 3) assert(ready);
        if (ready) {
          ++requests;
          latched = true;
        }
      }
      assert(requests == 1);
      // Leaving AUTO, stale measurements or a closed cadence gate arrive as
      // candidate_valid=false and must still veto an otherwise mature window.
      assert(!policy.ready(false, 20, false, true));
      assert(!policy.ready(true, 20, true, true));
    }
  }

  // With phase gating explicitly enabled, its impact-time condition remains
  // mandatory. An unsafe/absent event solution never owns the final send wait.
  for (const bool schedule_valid : {false, true}) {
    const auto policy = auto_aim::make_fire_window_policy(true, schedule_valid, false, 4);
    assert(policy.timing == auto_aim::FireTiming::impact_phase);
    assert(!policy.uses_event_clock());
    assert(!policy.ready(true, 4, false, false));
    assert(policy.ready(true, 4, false, true));
  }

  // An explicitly enabled, safe event route retains its one-frame window and
  // phase qualification. Readiness and final-send timing use the same object.
  {
    const auto policy = auto_aim::make_fire_window_policy(true, true, true, 4);
    assert(policy.uses_event_clock());
    assert(!policy.ready(true, 1, false, false));
    assert(policy.ready(true, 1, false, true));
    assert(!policy.ready(false, 1, false, true));
  }

  // Malformed frame counts cannot create a zero-frame request.
  for (const int configured_frames : {-3, 0}) {
    const auto policy = auto_aim::make_fire_window_policy(false, true, true, configured_frames);
    assert(!policy.ready(true, 0, false, false));
    assert(policy.ready(true, 1, false, false));
  }

  test_bounded_selection();
  std::cout << "fire_window_test: PASS (policy + bounded selection)\n";
}
