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
  // settle_ms: after the estimator first looks trusted, keep refusing requests
  // for this long. Field logs showed the first generation of a run sitting on a
  // steady ~6.4 deg yaw bias (equivalent to ~26 ms of lead) while still passing
  // tracking+converged+yaw_trusted, so trust alone is not stable enough.
  explicit ModelRebuildCounter(int after_requests, double settle_ms = 0.0)
  : after_requests_(std::max(0, after_requests)),
    settle_ns_(static_cast<std::int64_t>(std::max(0.0, settle_ms) * 1e6)),
    waiting_for_trust_(after_requests_ > 0 || settle_ns_ > 0) {}

  bool enabled() const { return after_requests_ > 0 || settle_ns_ > 0; }
  int sent_requests() const { return sent_requests_; }
  std::uint64_t last_shot_id() const { return last_counted_shot_id_; }
  bool pending() const { return pending_; }
  bool waiting_for_trust() const { return waiting_for_trust_; }
  bool settling() const { return settling_; }
  bool fire_allowed() const { return (!enabled() || (!waiting_for_trust_ && !pending_)) && !settling_; }

  bool observe_model(std::uint64_t generation)
  {
    if (generation == generation_) return false;
    generation_ = generation;
    sent_requests_ = 0;
    pending_ = false;
    waiting_for_trust_ = enabled();
    settling_ = false;
    trusted_since_ns_ = 0;
    return true;
  }

  // now_ns is a steady-clock timestamp in nanoseconds.
  bool update_trust(bool tracking, bool converged, bool yaw_trusted, std::int64_t now_ns = 0)
  {
    if (waiting_for_trust_) {
      if (!tracking || !converged || !yaw_trusted) return false;
      waiting_for_trust_ = false;
      trusted_since_ns_ = now_ns;
      if (settle_ns_ <= 0) return true;   // no settling window requested
      settling_ = true;
      return false;
    }
    if (settling_) {
      // Any loss of trust restarts the settle window; it only ends on a full
      // settle_ns of uninterrupted trust.
      if (!tracking || !converged || !yaw_trusted) {
        trusted_since_ns_ = now_ns;
        return false;
      }
      if (now_ns - trusted_since_ns_ >= settle_ns_) {
        settling_ = false;
        return true;
      }
    }
    return false;
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
    settling_ = false;
    trusted_since_ns_ = 0;
  }

private:
  int after_requests_ = 0;
  std::int64_t settle_ns_ = 0;
  int sent_requests_ = 0;
  std::uint64_t generation_ = 0;
  std::uint64_t last_counted_shot_id_ = 0;
  bool pending_ = false;
  bool waiting_for_trust_ = false;
  bool settling_ = false;
  std::int64_t trusted_since_ns_ = 0;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__MODEL_REBUILD_HPP
