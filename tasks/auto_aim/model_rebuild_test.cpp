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

}  // namespace

int main()
{
  test_requests_and_generations();
  std::cout << "model_rebuild_test: PASS (15 requests + trust + generations)\n";
}
