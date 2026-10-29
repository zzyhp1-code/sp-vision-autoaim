#include "model_rebuild.hpp"

#include <cassert>
#include <iostream>

namespace
{
void test_requests_and_generations()
{
  auto_aim::ModelRebuildCounter rebuild(15);
  assert(!rebuild.record_request(1, true));  // No model yet.
  assert(rebuild.observe_model(1));
  assert(!rebuild.fire_allowed());
  assert(!rebuild.update_trust(false, true, true));
  assert(!rebuild.update_trust(true, false, true));
  assert(!rebuild.update_trust(true, true, false));
  assert(rebuild.update_trust(true, true, true));
  for (std::uint64_t id = 1; id <= 14; ++id) {
    assert(!rebuild.record_request(id, false));  // Failed/cancelled TX.
    assert(!rebuild.record_request(id, true));
    assert(!rebuild.record_request(id, true));  // Repeated/held request.
    assert(rebuild.sent_requests() == static_cast<int>(id));
    assert(!rebuild.pending());
    assert(!rebuild.observe_model(1));  // temp_lost -> tracking, same model.
    assert(rebuild.sent_requests() == static_cast<int>(id));
  }
  assert(rebuild.record_request(15, true));
  assert(rebuild.pending() && !rebuild.fire_allowed());
  assert(!rebuild.record_request(16, true));
  assert(rebuild.sent_requests() == 15 && rebuild.last_shot_id() == 15);

  rebuild.reset_started();
  assert(!rebuild.pending() && rebuild.waiting_for_trust());
  assert(rebuild.sent_requests() == 0 && !rebuild.fire_allowed());
  assert(rebuild.observe_model(2));
  assert(!rebuild.record_request(16, true));
  assert(rebuild.update_trust(true, true, true));
  assert(!rebuild.record_request(15, true));  // Old IDs cannot cross models.
  assert(!rebuild.record_request(16, true));
  assert(rebuild.sent_requests() == 1);
  assert(rebuild.observe_model(3));  // Natural full reacquisition resets count.
  assert(rebuild.sent_requests() == 0 && !rebuild.fire_allowed());

  auto_aim::ModelRebuildCounter disabled(0);
  disabled.observe_model(1);
  for (std::uint64_t id = 1; id <= 40; ++id) {
    assert(!disabled.record_request(id, true));
    assert(disabled.fire_allowed() && !disabled.pending());
  }
  assert(disabled.sent_requests() == 0);

  auto_aim::ModelRebuildCounter every_request(1);
  every_request.observe_model(1);
  every_request.update_trust(true, true, true);
  assert(every_request.record_request(1, true));
}

// A fresh estimator can be "trusted" while still carrying a steady yaw bias, so
// trust must persist for settle_ms before a request is allowed.
void test_settle_window()
{
  constexpr std::int64_t ms = 1000000;
  auto_aim::ModelRebuildCounter settling(0, 3000.0);  // no rebuild, 3 s settle
  assert(settling.enabled());
  assert(!settling.fire_allowed());
  settling.observe_model(1);
  assert(!settling.fire_allowed());

  // First trust starts the window instead of opening the gate.
  assert(!settling.update_trust(true, true, true, 0));
  assert(settling.settling() && !settling.fire_allowed());

  // Still settling one millisecond before the deadline.
  assert(!settling.update_trust(true, true, true, 3000 * ms - 1));
  assert(!settling.fire_allowed());

  // Deadline reached: gate opens exactly once.
  assert(settling.update_trust(true, true, true, 3000 * ms));
  assert(!settling.settling() && settling.fire_allowed());

  // A later loss of trust does not reopen the gate permanently, and being
  // already settled is not reported as a new readiness event.
  assert(!settling.update_trust(true, true, true, 4000 * ms));
  assert(settling.fire_allowed());

  // Losing trust during the window restarts it in full.
  auto_aim::ModelRebuildCounter restarted(0, 1000.0);
  restarted.observe_model(1);
  assert(!restarted.update_trust(true, true, true, 0));
  assert(!restarted.update_trust(true, true, false, 500 * ms));  // trust lost
  assert(!restarted.fire_allowed());
  // Window now measures from the recovery instant, not from the first trust.
  assert(!restarted.update_trust(true, true, true, 1400 * ms));
  assert(!restarted.fire_allowed());
  assert(restarted.update_trust(true, true, true, 1500 * ms));
  assert(restarted.fire_allowed());

  // A new generation forces a fresh settle even when the old one had settled.
  assert(restarted.observe_model(2));
  assert(!restarted.fire_allowed());
  assert(!restarted.update_trust(true, true, true, 1600 * ms));
  assert(!restarted.fire_allowed());

  // A full estimator reset (planned rebuild) also re-arms the window.
  auto_aim::ModelRebuildCounter after_reset(25, 2000.0);
  after_reset.observe_model(1);
  assert(!after_reset.update_trust(true, true, true, 0));
  assert(after_reset.update_trust(true, true, true, 2000 * ms));
  assert(after_reset.fire_allowed());
  after_reset.reset_started();
  assert(!after_reset.fire_allowed());

  // settle_ms = 0 keeps the historical behaviour: with neither rebuild nor
  // settle configured the counter is inert, so fire is allowed immediately and
  // update_trust is not a readiness event.
  auto_aim::ModelRebuildCounter instant(0, 0.0);
  assert(!instant.enabled());
  instant.observe_model(1);
  assert(instant.fire_allowed() && !instant.settling());
  assert(!instant.update_trust(true, true, true, 0));
  assert(instant.fire_allowed());
}

}  // namespace

int main()
{
  test_requests_and_generations();
  test_settle_window();
  std::cout << "model_rebuild_test: PASS (15 requests + trust + settle + generations)\n";
}
