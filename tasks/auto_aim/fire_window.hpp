#ifndef AUTO_AIM__FIRE_WINDOW_HPP
#define AUTO_AIM__FIRE_WINDOW_HPP

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

namespace auto_aim
{
using FireClock = std::chrono::steady_clock;

// Only a trusted, associated camera observation advances this clock. Prediction
// frames and processing time never move the origin of a temporary-loss budget.
struct FireObservationClock
{
  std::uint64_t generation = 0;
  FireClock::time_point image_time{};

  void observe(std::uint64_t model, FireClock::time_point image, bool trusted_association)
  {
    if (model != generation) { generation = model; image_time = {}; }
    if (trusted_association && image > image_time) image_time = image;
  }

  double age_ms(FireClock::time_point now) const
  {
    return image_time == FireClock::time_point{} ? std::numeric_limits<double>::infinity()
      : std::chrono::duration<double, std::milli>(now - image_time).count();
  }

  bool within(FireClock::time_point now, std::uint64_t model, double max_age_ms) const
  {
    return model == generation && image_time != FireClock::time_point{} && now >= image_time &&
      now - image_time <= std::chrono::duration_cast<FireClock::duration>(
        std::chrono::duration<double, std::milli>(max_age_ms));
  }
};

struct FireCandidateWindow
{
  bool active = false, sent = false;
  int frames = 0, misses = 0, face = -1;
  std::uint64_t generation = 0;
  bool whole_ekf = false;
  std::uint64_t prediction_source = 0;  // 0: original; fixed centre: epoch + 1.
  double face_yaw_rad = INFINITY;
  std::int64_t phase_prediction_ns = 0;
  double phase_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
  FireClock::time_point since{};
  // Image time of the newest frame that kept this window alive. Qualification may
  // span a cooldown, but never a stall in the evidence that produced it.
  FireClock::time_point last_eligible{};

  void reset() { *this = FireCandidateWindow{}; }
  bool matches(std::uint64_t model, int candidate_face, bool geometry, std::uint64_t source = 0) const
  {
    return active && generation == model && face == candidate_face && face >= 0 &&
      whole_ekf == geometry && prediction_source == source;
  }
  bool can_continue(std::uint64_t model, int candidate_face, bool geometry, int required,
    std::uint64_t source = 0) const
  {
    return matches(model, candidate_face, geometry, source) && frames >= std::max(4, required) && !sent;
  }
  double phase_discontinuity(double face_yaw, std::int64_t prediction_ns) const
  {
    if (!std::isfinite(face_yaw) || !std::isfinite(face_yaw_rad)) return INFINITY;
    double expected_advance = 0.0;
    if (phase_prediction_ns > 0 && prediction_ns > 0 && std::isfinite(phase_rate_rad_s)) {
      const double dt = (static_cast<double>(prediction_ns) -
        static_cast<double>(phase_prediction_ns)) / 1e9;
      expected_advance = phase_rate_rad_s * dt;
    }
    return std::abs(std::remainder(
      face_yaw - face_yaw_rad - expected_advance, 6.2831853071795864769));
  }
  void update(
    bool eligible, bool temporary_loss, std::uint64_t model, int candidate_face, bool geometry,
    FireClock::time_point image, double face_yaw, int hold_frames,
    std::int64_t prediction_ns = 0,
    double angular_velocity_rad_s = std::numeric_limits<double>::quiet_NaN(),
    std::uint64_t source = 0)
  {
    if (active && !matches(model, candidate_face, geometry, source)) reset();
    if (temporary_loss) {
      // A gap can preserve an eligible mature window, never create/grow one.
      if (!eligible || !can_continue(model, candidate_face, geometry, 4, source)) reset();
      return;
    }
    if (eligible) {
      if (!active) { active = true; since = image; frames = 0; }
      last_eligible = image;
      generation = model; face = candidate_face; whole_ekf = geometry;
      prediction_source = source;
      face_yaw_rad = face_yaw; misses = 0; ++frames;
      phase_prediction_ns = prediction_ns;
      phase_rate_rad_s = angular_velocity_rad_s;
    } else if (!active || ++misses > hold_frames) {
      reset();
    }
  }
};

// Candidate accumulation and every send path use the same named gates. Keeping
// all failures (rather than one short-circuited reason) makes rejection auditable.
struct FireRequestGates
{
  bool target = false, tracker = false, observation = false, model = false;
  bool prediction = false, generation = false, face = false;
  bool control = false, auto_mode = false, feedback = false, allow_shoot = false;
  bool convergence = false, motion = false, cadence = false;
  bool yaw = false, visibility = false, pitch = false, one_face = false, phase = false;
  bool mature = false, unsent = false, timing = false;
  // "An aim point exists on THIS frame". Send-only: a plate rotating out of the
  // shootable cone is a missing opportunity, not a failed model, so it must not
  // erase qualification already accumulated for this model.
  bool aim = false;

  std::string closed(bool qualification_only = false) const
  {
    std::string out;
    const auto add = [&](bool open, const char * name) {
      if (!open) { if (!out.empty()) out += ','; out += name; }
    };
    add(target, "target"); add(tracker, "tracker"); add(observation, "observation");
    add(model, "model"); add(prediction, "prediction"); add(generation, "generation");
    add(face, "face"); add(control, "control"); add(auto_mode, "auto");
    add(feedback, "feedback"); add(convergence, "convergence"); add(motion, "motion");
    add(allow_shoot, "allow_shoot");
    add(yaw, "yaw"); add(visibility, "visibility");
    add(pitch, "pitch"); add(one_face, "one_face"); add(phase, "phase");
    if (!qualification_only) {
      // Cooldown and "aim exists right now" belong to the SEND decision only.
      // Keeping cadence in the qualification set made a 750 ms cooldown also
      // stop frame confirmation, so every shot paid cooldown + maturity again.
      add(cadence, "cadence"); add(aim, "aim");
      add(mature, "mature"); add(unsent, "unsent"); add(timing, "timing");
    }
    return out;
  }
  bool candidate_allowed(bool temporary_loss = false) const
  { return closed(true).empty() && (!temporary_loss || (mature && unsent)); }
  bool request_allowed() const { return closed().empty(); }
};

inline FireRequestGates apply_fire_observation_gates(
  FireRequestGates gates, const FireCandidateWindow & candidate,
  const FireObservationClock & observation, FireClock::time_point now,
  std::uint64_t generation, int face, bool whole_ekf, bool tracking, bool temporary_loss,
  bool inhibit_temporary_loss, bool fresh_association, double max_age_ms, int required_frames,
  std::uint64_t prediction_source = 0)
{
  const bool same = candidate.matches(generation, face, whole_ekf, prediction_source);
  gates.tracker = tracking || (temporary_loss && !inhibit_temporary_loss);
  gates.observation = temporary_loss ? observation.within(now, generation, max_age_ms)
                                    : fresh_association;
  gates.generation = !candidate.active || candidate.generation == generation;
  gates.face = face >= 0 && (!candidate.active || same);
  // Maturity may be reached during a cooldown, so it must also be *current*:
  // a window whose newest evidence is older than ~3 frame periods is not mature.
  const bool evidence_current = candidate.last_eligible != FireClock::time_point{} &&
    now >= candidate.last_eligible &&
    now - candidate.last_eligible <= std::chrono::milliseconds(100);
  gates.mature = same && evidence_current && candidate.frames >=
    (temporary_loss ? std::max(4, required_frames) : required_frames);
  gates.unsent = !candidate.sent;
  return gates;
}

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
