#include "stable_rotation_rate.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using auto_aim::StableRotationRate;

int main()
{
  const double pi = std::acos(-1.0);
  const auto ns = [](double t) { return static_cast<std::int64_t>(std::llround(t * 1e9)); };
  const auto fill = [&](StableRotationRate & filter, double omega, std::uint64_t generation = 1) {
    for (int i = 0; i <= 120; ++i) {
      const double t = i / 60.0;
      const double noise = 0.025 * std::sin(2.0 * pi * t / 0.35);
      filter.observe(generation, ns(t), std::remainder(omega * t + noise, 2.0 * pi),
        omega + 0.4 * std::cos(2.0 * pi * t / 0.35), true);
    }
  };
  for (double omega : {4.5, -4.5}) {
    StableRotationRate filter;
    fill(filter, omega);
    auto e = filter.estimate(ns(2.0), omega);
    assert(e.ready && e.samples >= 30 && e.span_s >= 1.5);
    assert(std::abs(e.rate_rad_s - omega) < 0.02);
    // Prediction-only frames cannot grow or refresh the trusted-image history.
    filter.observe(1, ns(2.02), 0.0, omega, false);
    const auto held = filter.estimate(ns(2.05), omega);
    assert(held.ready && held.samples == e.samples && held.image_ns == e.image_ns);
    assert(!filter.estimate(ns(2.05) + 1, omega).ready);
    assert(!filter.estimate(ns(1.99), omega).ready);
    filter.observe(1, ns(2.0), 10.0, omega, true);
    assert(filter.estimate(ns(2.0), omega).samples == e.samples);
    // A 90-degree anchor jump must not be folded away as a face change.
    filter.observe(1, ns(2.016), omega * 2.016 + pi / 2.0, omega, true);
    assert(!filter.estimate(ns(2.016), omega).ready);
    assert(filter.estimate(ns(2.016), omega).epoch != e.epoch);
  }
  StableRotationRate filter;
  fill(filter, 4.5);
  filter.observe(2, ns(2.016), 0.0, 4.5, true);
  assert(filter.estimate(ns(2.016), 4.5).samples == 1);
  filter.reset(); fill(filter, 4.5);
  filter.observe(1, ns(2.101), 4.5 * 2.101, 4.5, true);
  assert(filter.estimate(ns(2.101), 4.5).samples == 1);
  filter.reset(); fill(filter, 4.5);
  filter.observe(1, ns(2.016), 9.0, -4.5, true);
  assert(!filter.estimate(ns(2.016), -4.5).ready);
  filter.observe(1, ns(1.0), 4.5, 4.5, true);
  assert(filter.estimate(ns(1.0), 4.5).samples == 1);
  filter.observe(1, ns(1.02), NAN, 4.5, true);
  assert(filter.estimate(ns(1.02), 4.5).samples == 0);

  // Smooth acceleration can remain finite but is outside the stable-speed policy.
  filter.reset();
  for (int i = 0; i <= 120; ++i) {
    const double t = i / 60.0;
    filter.observe(3, ns(t), 3.0 * t + 0.8 * t * t, 3.0 + 1.6 * t, true);
  }
  assert(!filter.estimate(ns(2.0), 6.2).ready);
  filter.reset(); fill(filter, 4.5);
  assert(!filter.estimate(ns(2.0), -4.5).ready);
  StableRotationRate::Config bounded;
  bounded.maximum_abs_rate = 4.0;
  StableRotationRate slow_limit(bounded); fill(slow_limit, 4.5);
  assert(!slow_limit.estimate(ns(2.0), 4.5).ready);

  using Mode = auto_aim::RotationPredictionMode;
  assert(auto_aim::rotation_prediction_mode("off") == Mode::off);
  assert(auto_aim::rotation_prediction_mode("shadow") == Mode::shadow);
  assert(auto_aim::rotation_prediction_mode("active") == Mode::active);
  bool rejected = false;
  try { auto_aim::rotation_prediction_mode("typo"); } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  auto_aim::RotationPredictionEpoch route;
  for (int i = 0; i < 100; ++i) {
    assert(!route.update(Mode::off, i % 2, i));
    assert(!route.update(Mode::shadow, i % 2, i));
  }
  assert(!route.update(Mode::active, false, 1));
  assert(route.update(Mode::active, true, 1));
  assert(!route.update(Mode::active, true, 1));
  assert(route.update(Mode::active, true, 2));
  assert(route.update(Mode::active, false, 2));

  // Override is possible only in reviewed active mode while the tracker is in
  // high-speed rotation with a ready estimate; off/shadow must never qualify.
  for (const bool high_speed : {false, true}) {
    for (const bool ready : {false, true}) {
      assert(!auto_aim::uses_stable_rotation(Mode::off, high_speed, ready));
      assert(!auto_aim::uses_stable_rotation(Mode::shadow, high_speed, ready));
    }
  }
  assert(!auto_aim::uses_stable_rotation(Mode::active, false, true));
  assert(!auto_aim::uses_stable_rotation(Mode::active, true, false));
  assert(auto_aim::uses_stable_rotation(Mode::active, true, true));

  std::cout << "stable_rotation_rate_test: PASS\n";
}
