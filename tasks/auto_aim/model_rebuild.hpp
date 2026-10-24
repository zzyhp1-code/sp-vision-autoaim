#ifndef AUTO_AIM__MODEL_REBUILD_HPP
#define AUTO_AIM__MODEL_REBUILD_HPP

#include <algorithm>
#include <cstdint>

namespace auto_aim
{
// Count unique, matched host requests in the current estimator generation.
// This is deliberately not a muzzle/projectile counter.
class ModelRebuildCounter
{
public:
  explicit ModelRebuildCounter(int after_requests)
  : after_requests_(std::max(0, after_requests)) {}

  bool enabled() const { return after_requests_ > 0; }
  int sent_requests() const { return sent_requests_; }
  std::uint64_t last_shot_id() const { return last_counted_shot_id_; }
  bool pending() const { return pending_; }
  bool waiting_for_trust() const { return waiting_for_trust_; }
  bool fire_allowed() const { return !enabled() || (!waiting_for_trust_ && !pending_); }

  bool observe_model(std::uint64_t generation)
  {
    if (generation == generation_) return false;
    generation_ = generation;
    sent_requests_ = 0;
    pending_ = false;
    waiting_for_trust_ = enabled();
    return true;
  }

  bool update_trust(bool tracking, bool converged, bool yaw_trusted)
  {
    if (!waiting_for_trust_ || !tracking || !converged || !yaw_trusted) return false;
    waiting_for_trust_ = false;
    return true;
  }

  bool record_request(std::uint64_t shot_id, bool tx_call_matches)
  {
    if (!enabled() || !fire_allowed() || generation_ == 0 || !tx_call_matches ||
        shot_id == 0 || shot_id <= last_counted_shot_id_) return false;
    last_counted_shot_id_ = shot_id;
    pending_ = ++sent_requests_ >= after_requests_;
    return pending_;
  }

  void reset_started()
  {
    sent_requests_ = 0;
    pending_ = false;
    waiting_for_trust_ = enabled();
  }

private:
  int after_requests_ = 0;
  int sent_requests_ = 0;
  std::uint64_t generation_ = 0;
  std::uint64_t last_counted_shot_id_ = 0;
  bool pending_ = false;
  bool waiting_for_trust_ = false;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__MODEL_REBUILD_HPP
