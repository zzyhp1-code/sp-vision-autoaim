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

void test_temporary_loss()
{
  using namespace std::chrono;
  using auto_aim::FireClock;
  const auto origin = FireClock::time_point{} + seconds(1);
  auto_aim::FireObservationClock clock;
  auto_aim::FireCandidateWindow candidate;
  auto_aim::FireRequestGates physical;
  physical.target = physical.model = physical.prediction = physical.control = true;
  physical.auto_mode = physical.feedback = physical.convergence = physical.motion = true;
  physical.allow_shoot = true;
  physical.cadence = physical.yaw = physical.visibility = physical.pitch = true;
  physical.aim = true;
  physical.alignment = true;
  physical.armor_jump = true;
  physical.one_face = physical.phase = physical.timing = true;
  const auto gates = [&](FireClock::time_point now, bool gap, bool inhibit = false,
                         std::uint64_t generation = 1, int face = 2, bool geometry = true) {
    return auto_aim::apply_fire_observation_gates(
      physical, candidate, clock, now, generation, face, geometry, !gap, gap,
      inhibit, !gap, 50.0, 4);
  };
  auto_aim::FireWindowSelector selector;
  const auto policy = auto_aim::make_fire_window_policy(false, true, true, 4);
  // Fourth *measured* frame issues immediately, with no extra selection wait.
  for (int frame = 0; frame < 4; ++frame) {
    const auto image = origin + milliseconds(16 * frame);
    clock.observe(1, image, true);
    candidate.update(gates(image, false).candidate_allowed(), false, 1, 2, true,
                     image, 0.2, 3);
    const auto ready = gates(image, false).request_allowed();
    assert(ready == (frame == 3));
    const auto decision = selector.update(true, candidate.frames >= policy.required_frames,
      false, 2.0 * deg, duration<double>(image.time_since_epoch()).count(), {0.0, 0.5 * deg});
    assert(decision.ready == ready);
  }
  const auto last_image = clock.image_time;
  // Prediction must not refresh observation time or grow the candidate count.
  for (int age : {16, 32, 49, 50}) {
    const auto now = last_image + milliseconds(age);
    clock.observe(1, now, false);
    const auto g = gates(now, true);
    assert(g.request_allowed() && g.candidate_allowed(true));
    candidate.update(g.candidate_allowed(true), true, 1, 2, true, now, 0.3, 3);
    assert(candidate.frames == 4 && clock.image_time == last_image);
  }
  // Inclusive 50 ms boundary; processing and send delay are both in the age.
  assert(gates(last_image + milliseconds(50), true).request_allowed());
  assert(!gates(last_image + milliseconds(50) + nanoseconds(1), true).request_allowed());
  const auto qualified = gates(last_image + milliseconds(49), true);
  assert(qualified.request_allowed());
  assert(!gates(last_image + milliseconds(51), true).request_allowed());
  assert(!gates(last_image + milliseconds(20), true, true).request_allowed());
  for (int frames = 0; frames < 4; ++frames) {
    auto immature = candidate;
    immature.frames = frames;
    assert(!immature.can_continue(1, 2, true, 4));
    const auto g = auto_aim::apply_fire_observation_gates(
      physical, immature, clock, last_image + milliseconds(16), 1, 2, true,
      false, true, false, false, 50, 4);
    assert(!g.candidate_allowed(true) && !g.request_allowed());
    immature.update(false, true, 1, 2, true, last_image, 0.2, 3);
    assert(!immature.active && immature.frames == 0);
  }
  // A changed face, model generation or geometry cannot inherit qualification.
  assert(!gates(last_image, true, false, 2).request_allowed());
  assert(!gates(last_image, true, false, 1, 3).request_allowed());
  assert(!gates(last_image, true, false, 1, 2, false).request_allowed());
  auto changed = candidate;
  changed.update(true, false, 1, 3, true, last_image, 0.5, 3);
  assert(changed.frames == 1 && changed.face == 3 && !changed.sent);
  changed = candidate;
  changed.update(true, false, 2, 2, true, last_image, 0.5, 3);
  assert(changed.frames == 1 && changed.generation == 2);
  candidate.sent = true;
  assert(!gates(last_image, true).request_allowed());
  assert(!candidate.can_continue(1, 2, true, 4));
  candidate.sent = false;
  // Every physical gate, including prediction/trust/feedback and rebuild, is
  // authoritative for candidate, ordinary send, pulse and diagnostic paths.
  for (auto member : {&auto_aim::FireRequestGates::target, &auto_aim::FireRequestGates::model,
       &auto_aim::FireRequestGates::prediction, &auto_aim::FireRequestGates::feedback,
       &auto_aim::FireRequestGates::yaw, &auto_aim::FireRequestGates::visibility,
       &auto_aim::FireRequestGates::pitch}) {
    physical.*member = false;
    const auto g = gates(last_image, true);
    assert(!g.request_allowed() && !g.candidate_allowed(true));
    physical.*member = true;
  }
  // Cooldown and "aim exists right now" gate the SEND only. Qualification must
  // keep accumulating through a cooldown, and an absent plate must not silently
  // bank qualification that is then reused for a shot.
  for (const char * gate : {"cadence", "aim", "alignment", "armor_jump"}) {
    const std::string name(gate);
    auto & flag = name == "cadence" ? physical.cadence
                : name == "aim" ? physical.aim
                : name == "alignment" ? physical.alignment
                : physical.armor_jump;
    flag = false;
    const auto g = gates(last_image, true);
    const auto reasons = g.closed();
    assert(!g.request_allowed());
    assert(g.candidate_allowed(true));
    assert(reasons.find(name) != std::string::npos);
    flag = true;
  }
  // Qualification cannot be banked across a stall in its own evidence either.
  assert(gates(last_image + milliseconds(150), true).mature == false);
  auto all_closed = gates(last_image, true);
  all_closed.prediction = all_closed.model = all_closed.feedback = false;
  const auto reasons = all_closed.closed();
  assert(reasons.find("prediction") != std::string::npos);
  assert(reasons.find("model") != std::string::npos);
  assert(reasons.find("feedback") != std::string::npos);
  candidate.reset();  // Full loss or entry to rebuild discards the window.
  assert(!gates(last_image, true).request_allowed());
  clock.observe(2, last_image + milliseconds(16), false);
  assert(!clock.within(last_image + milliseconds(20), 2, 50.0));
  clock.observe(2, last_image + milliseconds(32), true);
  assert(clock.within(last_image + milliseconds(33), 2, 50.0));
}

void test_rotating_candidate_continuity()
{
  using namespace std::chrono;
  const auto image = auto_aim::FireClock::time_point(seconds(10));
  const std::int64_t prediction = 10400000000LL;
  for (const double rate : {-4.5, 4.5}) {
    auto_aim::FireCandidateWindow window;
    // A retained window can skip two unqualified frames. At this field rate,
    // 135 ms advances a real face by 34.8 degrees, without any face switch.
    const double start_phase = rate > 0 ? 3.0 : -3.0;
    window.update(true, false, 7, 2, true, image, start_phase, 3, prediction, rate);
    for (int i = 1; i < 4; ++i) {
      window.update(false, false, 7, 2, true, image + milliseconds(135 * i - 90), 0, 3);
      window.update(false, false, 7, 2, true, image + milliseconds(135 * i - 45), 0, 3);
      const auto next_prediction = prediction + 135000000LL * i;
      const double next_phase = std::remainder(start_phase + rate * 0.135 * i, 2 * M_PI);
      assert(std::abs(std::remainder(next_phase - window.face_yaw_rad, 2 * M_PI)) > 20 * deg);
      assert(window.phase_discontinuity(next_phase, next_prediction) < 1e-12);
      // A true phase jump is still rejected even when the model retains its ID.
      assert(window.phase_discontinuity(next_phase + 40 * deg, next_prediction) > 20 * deg);
      window.update(true, false, 7, 2, true, image + milliseconds(135 * i), next_phase, 3,
        next_prediction, rate);
      assert(window.frames == i + 1);
    }
    assert(window.can_continue(7, 2, true, 4));
    auto changed = window;
    changed.update(true, false, 7, 3, true, image + milliseconds(450), 0, 3,
      prediction + 450000000, rate);
    assert(changed.frames == 1 && changed.face == 3);
    // Flight-time changes move the prediction timestamp as well as image time.
    const auto shifted = window.phase_prediction_ns + 65000000;
    assert(window.phase_discontinuity(window.face_yaw_rad + rate * 0.065, shifted) < 1e-12);
    window.reset();
    assert(!window.active && window.phase_prediction_ns == 0);
  }
}

}  // namespace

int main()
{
  {
    using namespace std::chrono;
    const auto image = auto_aim::FireClock::time_point(seconds(10));
    auto_aim::FireCandidateWindow window;
    for (int i = 0; i < 4; ++i)
      window.update(true, false, 7, 2, true, image + milliseconds(40 * i), 0, 3);
    assert(window.can_continue(7, 2, true, 4));
    window.update(true, false, 7, 2, true, image + milliseconds(160), 0, 3, 0, 5.0, 13);
    assert(window.frames == 1 && window.matches(7, 2, true, 13));
    assert(!window.matches(7, 2, true) && !window.can_continue(7, 2, true, 3, 13));
    window.update(true, false, 7, 2, true, image + milliseconds(200), 0, 3, 0, 5.0, 13);
    assert(window.frames == 2);
    window.update(true, false, 7, 2, true, image + milliseconds(240), 0, 3, 0, 5.0, 14);
    assert(window.frames == 1 && window.prediction_source == 14);
    window.update(true, true, 7, 2, true, image + milliseconds(280), 0, 3, 0, 5.0, 0);
    assert(!window.active);  // Fallback in loss cannot inherit fixed-centre maturity.
  }
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
  test_temporary_loss();
  test_rotating_candidate_continuity();
  std::cout << "fire_window_test: PASS (policy + selection + temporary-loss/send gates)\n";
}
