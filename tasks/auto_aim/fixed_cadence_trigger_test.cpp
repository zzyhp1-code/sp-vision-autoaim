// Offline checks for FixedCadenceTrigger.
//
// The trigger exists because a per-frame alignment test cannot request shots on a
// fast gyro: one face passes in (pi/2)/|omega| and the usable window is a small
// slice of it. These tests pin the properties the field depends on: the clock
// only anchors on a real face centre, it requests once per N faces, missed slots
// are skipped rather than burst, and calibration tracks a small speed error
// without jumping.
#include "gyro_phase_scheduler.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace
{
constexpr double kPi = M_PI;

// Walk a synthetic target at a constant rate so the trigger sees the same folded
// phase sequence the tracker would supply.
struct Sim
{
  double omega;
  double phase = 0.0;

  // Advance by dt and return the folded phase.
  double step(double dt)
  {
    phase += omega * dt;
    return std::remainder(phase, kPi / 2.0);
  }
};

void test_anchor_requires_face_centre()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;

  // 0.50 rad (~28.6 deg) is outside the default 20 deg anchor tolerance, so a
  // caller that only has an off-centre observation must not start the clock.
  auto solution = trigger.observe(0.0, 0.0, 0.50, omega, 0.41, 2, 0.1);
  assert(!solution.valid);
  assert(!trigger.anchored());

  // A near-centre phase (~1.1 deg) anchors.
  solution = trigger.observe(0.05, 0.05, 0.02, omega, 0.41, 2, 0.1);
  assert(trigger.anchored());
  assert(solution.valid);
  assert(solution.face_period_s > 0.0);
}

void test_interval_matches_stride()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  const double face_period = (kPi / 2.0) / omega;

  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 2, 0.1);
  assert(trigger.anchored());
  // Two faces per request.
  const double expected = 2.0 * face_period;
  assert(std::abs(trigger.command_interval_s() - expected) < 1e-9);

  auto_aim::FixedCadenceTrigger single;
  single.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.1);
  assert(std::abs(single.command_interval_s() - face_period) < 1e-9);
}

void test_direction_and_setting_change_reset()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.1);
  assert(trigger.anchored());
  const double first_interval = trigger.command_interval_s();

  // A direction change must drop the old clock and re-anchor from scratch, so the
  // new interval reflects the new settings rather than the previous ones.
  trigger.observe(0.05, 0.05, 0.0, -omega, 0.2, 1, 0.1);
  assert(trigger.anchored());
  assert(std::abs(trigger.command_interval_s() - first_interval) < 1e-9);

  // Changing the stride also re-anchors, and the interval doubles.
  trigger.observe(0.10, 0.10, 0.0, -omega, 0.2, 2, 0.1);
  assert(trigger.anchored());
  assert(std::abs(trigger.command_interval_s() - 2.0 * first_interval) < 1e-9);

  // An off-centre observation with changed settings cannot anchor on its own.
  auto_aim::FixedCadenceTrigger needs_centre;
  needs_centre.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.1);
  assert(needs_centre.anchored());
  needs_centre.observe(0.05, 0.05, 0.5, -omega, 0.2, 1, 0.1);
  assert(!needs_centre.anchored());
}

// The rate comes from a live estimator, so it jitters every frame. Keying the
// clock on that would reset it continuously and calibration would never
// accumulate; field log 2026-10-31_04-15-44 showed calib stuck at 0 for 87 s with
// anchored dropping to 0 about half the time for exactly this reason.
void test_rate_jitter_does_not_reset_clock()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.2);
  assert(trigger.anchored());
  const double interval_at_anchor = trigger.command_interval_s();

  // +-8% one-frame jitter, as the EKF produces, must keep the same clock.
  const double jitter[] = {+0.08, -0.05, +0.03, -0.08, +0.06, -0.02};
  double now = 0.0;
  const double dt = 1.0 / 60.0;
  for (int round = 0; round < 3; ++round) {
    for (double factor : jitter) {
      now += dt;
      trigger.observe(now, now, 0.0, omega * (1.0 + factor), 0.2, 1, 0.2);
      assert(trigger.anchored());
    }
  }
  // Same reference rate, so the nominal interval is unchanged.
  assert(std::abs(trigger.command_interval_s() - interval_at_anchor) < 1e-9);

  // A large departure re-seeds the reference rate without dropping the anchor.
  // The learned period is deliberately kept: calibration slews it in bounded
  // steps, and re-seeding only widens the bound that calibration is allowed to
  // reach, so a stale seed cannot freeze the clock on the wrong period.
  trigger.observe(now + dt, now + dt, 0.0, omega * 1.40, 0.2, 1, 0.2);
  assert(trigger.anchored());
  assert(std::isfinite(trigger.command_interval_s()));
  assert(trigger.command_interval_s() > 0.0);
}

// A reversal is a genuinely different target state and must still drop the clock.
void test_reversal_still_resets()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.2);
  assert(trigger.anchored());
  // Off centre, so it cannot immediately re-anchor.
  trigger.observe(0.05, 0.05, 0.4, -omega, 0.2, 1, 0.2);
  assert(!trigger.anchored());
}

// The phase diagnostic must report a real offset, not the identically-zero value
// produced by comparing a phase against the clock's own extrapolation. A target
// running slightly behind the configured rate has to show up as a growing residual.
void test_crossing_residual_reports_real_offset()
{
  // Fixed-period mode with the true rate a little slower than configured. Over time
  // each measured crossing should land later than the clock predicts.
  auto_aim::FixedCadenceTrigger::Config config;
  config.calibrate_period = false;   // keep the anchored period, only measure
  auto_aim::FixedCadenceTrigger trigger(config);

  const double omega = 4.3458;
  const double actual_omega = omega * 0.97;   // 3% slow
  const double face_period = (kPi / 2.0) / actual_omega;

  double phase = 0.0;
  double now = 0.0;
  const double dt = 1.0 / 60.0;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.2);
  assert(trigger.anchored());

  // Run several faces so the lag accumulates well past the noise floor.
  const int steps = static_cast<int>(8.0 * face_period / dt);
  for (int i = 0; i < steps; ++i) {
    now += dt;
    phase += actual_omega * dt;
    const double folded = std::remainder(phase, kPi / 2.0);
    trigger.observe(now, now, folded, omega, 0.2, 1, 0.2);
  }

  assert(trigger.crossing_count() >= 4);
  const double residual = trigger.crossing_residual_s();
  assert(std::isfinite(residual));
  // The target is slower than the clock expects, so crossings arrive late.
  assert(residual > 0.0);
  // And the magnitude must be meaningful, not a rounding artefact.
  assert(residual > 1e-3);
}

void test_stale_observation_drops_clock()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.2, 1, 0.05);
  assert(trigger.anchored());

  // Observation older than max_age_s: no anchoring, and the clock resets.
  auto solution = trigger.observe(0.5, 0.0, 0.0, omega, 0.2, 1, 0.05);
  assert(!solution.valid);
  assert(!trigger.anchored());
}

void test_missed_slots_are_skipped_not_burst()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  const double face_period = (kPi / 2.0) / omega;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.0, 1, 0.1);
  const auto first_due = trigger.observe(0.0, 0.0, 0.0, omega, 0.0, 1, 0.1).command_due_time_s;

  // Ten periods later, with a fresh in-time observation: the due time must have
  // advanced to the current neighbourhood, never remain ten faces behind.
  const double now = first_due + 10.0 * face_period + 0.001;
  const auto solution = trigger.observe(now, now, 0.0, omega, 0.0, 1, 0.1);
  assert(solution.command_due_time_s > now - 0.05);
  assert(solution.command_due_time_s < now + face_period + 0.05);
}

void test_mark_fired_advances_one_period()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  trigger.observe(0.0, 0.0, 0.0, omega, 0.0, 1, 0.1);
  const double before = trigger.command_interval_s();
  const auto due_before = trigger.observe(0.0, 0.0, 0.0, omega, 0.0, 1, 0.1).command_due_time_s;
  trigger.mark_fired();
  const auto due_after = trigger.observe(0.0, 0.0, 0.0, omega, 0.0, 1, 0.1).command_due_time_s;
  assert(std::abs((due_after - due_before) - before) < 1e-9);
}

void test_calibration_tracks_slower_target()
{
  // Feed a target that actually spins 2% slower than the configured rate. The
  // clock must converge toward the observed period instead of drifting forever.
  auto_aim::FixedCadenceTrigger::Config config;
  config.max_period_step_ratio = 0.02;  // let a short test converge quickly
  auto_aim::FixedCadenceTrigger trigger(config);

  const double nominal_omega = 4.56;
  const double actual_omega = nominal_omega * 0.98;
  Sim sim{actual_omega};
  sim.phase = 0.0;

  double now = 0.0;
  const double dt = 1.0 / 60.0;
  for (int i = 0; i < 60 * 12; ++i) {
    now += dt;
    const double folded = sim.step(dt);
    // Report the nominal rate, as a caller with a fixed-speed assumption would.
    trigger.observe(now, now, folded, nominal_omega, 0.2, 1, 0.1);
  }
  assert(trigger.calibration_updates() > 0);
  const double nominal_face = (kPi / 2.0) / nominal_omega;
  const double actual_face = (kPi / 2.0) / actual_omega;
  const double learned = trigger.command_interval_s();
  // Must move toward the true period and stay inside the configured gate.
  assert(learned > nominal_face * 0.999);
  assert(learned < actual_face * 1.05);
}

void test_negative_offset_is_supported()
{
  auto_aim::FixedCadenceTrigger trigger;
  const double omega = 4.56;
  auto solution = trigger.observe(0.0, 0.0, 0.0, omega, -0.15, 1, 0.1);
  assert(trigger.anchored());
  assert(solution.valid);
  assert(std::isfinite(solution.command_due_time_s));
}

void test_invalid_inputs_do_not_anchor()
{
  auto_aim::FixedCadenceTrigger trigger;
  assert(!trigger.observe(NAN, 0.0, 0.0, 4.56, 0.2, 1, 0.1).valid);
  assert(!trigger.observe(0.0, 0.0, 0.0, 0.0, 0.2, 1, 0.1).valid);
  assert(!trigger.observe(0.0, 0.0, 0.0, 4.56, 0.2, 0, 0.1).valid);
  assert(!trigger.anchored());
}

}  // namespace

int main()
{
  test_anchor_requires_face_centre();
  test_interval_matches_stride();
  test_direction_and_setting_change_reset();
  test_stale_observation_drops_clock();
  test_missed_slots_are_skipped_not_burst();
  test_mark_fired_advances_one_period();
  test_calibration_tracks_slower_target();
  test_negative_offset_is_supported();
  test_invalid_inputs_do_not_anchor();
  test_rate_jitter_does_not_reset_clock();
  test_reversal_still_resets();
  test_crossing_residual_reports_real_offset();
  std::cout << "fixed_cadence_trigger_test: PASS (anchor, stride, reset, stale, "
               "skip, advance, calibrate, offset, invalid, jitter, reversal, residual)\n";
  return 0;
}
