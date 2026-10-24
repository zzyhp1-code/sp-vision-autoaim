#ifndef AUTO_AIM__FIRE_WINDOW_HPP
#define AUTO_AIM__FIRE_WINDOW_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace auto_aim
{
// A time grid anchored by the first successful transmission. Poll every frame,
// including target loss: a tick is consumed even when qualification is closed.
class FixedFireInterval
{
public:
  struct Tick
  {
    bool ready = false;
    std::int64_t due_ns = 0;
    std::int64_t skipped = 0;
  };

  static constexpr std::int64_t max_lateness_ns = 50000000;

  Tick poll(std::int64_t now_ns, std::int64_t interval_ns)
  {
    Tick tick;
    if (interval_ns <= 0 || now_ns < 0) return tick;
    if (!have_sent_) {
      tick.ready = true;  // Initial acquisition; sent() anchors the grid once.
      return tick;
    }
    if (now_ns < next_due_ns_) return tick;
    const auto elapsed_ticks = (now_ns - next_due_ns_) / interval_ns + 1;
    tick.due_ns = next_due_ns_ + (elapsed_ticks - 1) * interval_ns;
    next_due_ns_ += elapsed_ticks * interval_ns;
    // Do not issue stale ticks after a processing stall or replay missed slots.
    tick.ready = elapsed_ticks == 1 && within_tick(now_ns, tick.due_ns);
    tick.skipped = elapsed_ticks - (tick.ready ? 1 : 0);
    return tick;
  }

  static bool within_tick(std::int64_t now_ns, std::int64_t due_ns)
  {
    return due_ns == 0 ||
      (now_ns >= due_ns && now_ns - due_ns <= max_lateness_ns);
  }

  void sent(std::int64_t tx_call_ns, std::int64_t interval_ns)
  {
    if (interval_ns <= 0 || tx_call_ns < 0 || (have_sent_ && tx_call_ns < last_sent_ns_)) return;
    if (!have_sent_) next_due_ns_ = tx_call_ns + interval_ns;
    last_sent_ns_ = tx_call_ns;
    have_sent_ = true;
  }

  bool have_sent() const { return have_sent_; }
  std::int64_t last_sent_ns() const { return last_sent_ns_; }
  std::int64_t next_due_ns() const { return next_due_ns_; }

private:
  bool have_sent_ = false;
  std::int64_t last_sent_ns_ = 0;
  std::int64_t next_due_ns_ = 0;
};

enum class FireTiming { stable_window, impact_phase, event_clock };

// One policy is shared by candidate qualification and the final send stage.
// A disabled phase gate must not silently retain event-clock timing.
struct FireWindowPolicy
{
  FireTiming timing = FireTiming::stable_window;
  int required_frames = 1;

  bool uses_event_clock() const { return timing == FireTiming::event_clock; }

  const char * name() const
  {
    if (timing == FireTiming::event_clock) return "event";
    if (timing == FireTiming::impact_phase) return "phase";
    return "stable";
  }

  bool ready(
    bool candidate_valid, int candidate_frames, bool request_latched,
    bool phase_moment_ready) const
  {
    return candidate_valid && candidate_frames >= required_frames && !request_latched &&
           (timing == FireTiming::stable_window || phase_moment_ready);
  }
};

inline FireWindowPolicy make_fire_window_policy(
  bool phase_gate_active, bool event_schedule_valid, bool event_interval_safe,
  int stable_window_frames)
{
  if (!phase_gate_active) {
    return {FireTiming::stable_window, std::max(1, stable_window_frames)};
  }
  if (event_schedule_valid && event_interval_safe) {
    return {FireTiming::event_clock, 1};
  }
  return {FireTiming::impact_phase, std::max(1, stable_window_frames)};
}

struct FireWindowSelectionConfig
{
  double max_wait_s = 0.0;  // Zero preserves immediate mature-window qualification.
  double near_error_rad = 0.0;
  double trend_epsilon_rad = 0.02 * 3.14159265358979323846 / 180.0;
};

struct FireWindowDecision
{
  bool ready = false;
  bool started = false;
  const char * reason = "inactive";
  double waited_s = 0.0;
  double next_error_rad = std::numeric_limits<double>::quiet_NaN();
};

// A bounded preference within the existing yaw-error gate, not another phase
// gate. Call once per fresh frame; the caller continues tracking and sending
// aim commands while ready=false. No timer may bypass a closed candidate gate.
class FireWindowSelector
{
public:
  void reset()
  {
    previous_valid_ = false;
    waiting_ = false;
  }

  FireWindowDecision update(
    bool candidate_valid, bool mature, bool request_latched, double error_rad,
    double now_s, const FireWindowSelectionConfig & config)
  {
    FireWindowDecision decision;
    if (!candidate_valid || request_latched || !std::isfinite(error_rad) ||
        !std::isfinite(now_s)) {
      reset();
      decision.reason = request_latched ? "latched" : "invalid";
      return decision;
    }

    // A detector gap, repeated timestamp or clock reversal cannot supply a
    // derivative or an already-expired deadline for a new observation.
    if (previous_valid_ && (now_s <= previous_time_s_ || now_s - previous_time_s_ > 0.10)) {
      reset();
    }
    if (previous_valid_) {
      constexpr double two_pi = 6.28318530717958647692;
      decision.next_error_rad = std::remainder(
        error_rad + std::remainder(error_rad - previous_error_rad_, two_pi), two_pi);
    }
    previous_error_rad_ = error_rad;
    previous_time_s_ = now_s;
    previous_valid_ = true;

    if (!mature) {
      waiting_ = false;
      decision.reason = "warmup";
      return decision;
    }
    if (config.max_wait_s <= 0.0) {
      decision.ready = true;
      decision.reason = "disabled";
      return decision;
    }
    if (!waiting_) {
      waiting_ = true;
      mature_since_s_ = now_s;
      decision.started = true;
    }
    decision.waited_s = now_s - mature_since_s_;
    if (std::abs(error_rad) <= config.near_error_rad) {
      decision.ready = true;
      decision.reason = "near_center";
    } else if (std::isfinite(decision.next_error_rad) &&
               std::abs(decision.next_error_rad) >
                 std::abs(error_rad) + config.trend_epsilon_rad) {
      decision.ready = true;
      decision.reason = "turning";
    } else if (decision.waited_s >= config.max_wait_s) {
      // Checked on the next fresh frame at/after the deadline. This is not a
      // sleep, and the actual elapsed time is logged rather than called exact.
      decision.ready = true;
      decision.reason = "deadline";
    } else {
      decision.reason = "waiting";
    }
    return decision;
  }

private:
  bool previous_valid_ = false;
  bool waiting_ = false;
  double previous_error_rad_ = 0.0;
  double previous_time_s_ = 0.0;
  double mature_since_s_ = 0.0;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__FIRE_WINDOW_HPP
