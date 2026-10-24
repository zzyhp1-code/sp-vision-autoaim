#ifndef AUTO_AIM__STABLE_ROTATION_RATE_HPP
#define AUTO_AIM__STABLE_ROTATION_RATE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace auto_aim
{
enum class RotationPredictionMode { off, shadow, active };

inline RotationPredictionMode rotation_prediction_mode(const std::string & value)
{
  if (value == "off") return RotationPredictionMode::off;
  if (value == "shadow") return RotationPredictionMode::shadow;
  if (value == "active") return RotationPredictionMode::active;
  throw std::invalid_argument("xuc_rotation_prediction_mode must be off, shadow or active");
}

// History contains posterior MAIN-EKF body phases, once per trusted image.
// Neither a Target prediction copy nor a temporary-loss frame may append here.
class StableRotationRate
{
  static constexpr double pi = 3.14159265358979323846;
public:
  struct Config
  {
    double window_s = 2.0, minimum_span_s = 1.5;
    int minimum_samples = 30;
    double maximum_gap_s = 0.100, maximum_age_s = 0.050;
    double maximum_innovation_rad = 30.0 * pi / 180.0;
    double maximum_rms_rad = 5.0 * pi / 180.0;
    double maximum_half_rate_difference = 0.5;
    double maximum_abs_rate = 8.0;
  };
  struct Estimate
  {
    bool ready = false;
    const char * reason = "warmup";
    double rate_rad_s = NAN, rms_rad = NAN, span_s = 0.0, age_s = INFINITY;
    std::size_t samples = 0;
    std::uint64_t epoch = 0, generation = 0;
    std::int64_t image_ns = 0;
  };

  StableRotationRate() = default;
  explicit StableRotationRate(const Config & config) : config_(config) {}

  void reset()
  {
    samples_.clear();
    generation_ = 0;
    ++epoch_;
  }

  void observe(
    std::uint64_t generation, std::int64_t image_ns, double phase, double raw_rate,
    bool trusted_association)
  {
    if (generation != generation_) { reset(); generation_ = generation; }
    if (!trusted_association) return;
    if (!std::isfinite(phase) || !std::isfinite(raw_rate)) { reset(); return; }
    if (!samples_.empty() && image_ns == samples_.back().ns) return;
    double unwrapped = phase;
    if (!samples_.empty()) {
      const auto & previous = samples_.back();
      const double dt = (image_ns - previous.ns) * 1e-9;
      const double delta = std::remainder(phase - previous.phase, 2.0 * pi);
      if (dt <= 0.0 || image_ns - previous.ns > std::llround(config_.maximum_gap_s * 1e9) ||
          raw_rate * previous.raw_rate < 0.0 ||
          std::abs(delta - previous.raw_rate * dt) > config_.maximum_innovation_rad) {
        samples_.clear();
        ++epoch_;
      } else {
        unwrapped = previous.unwrapped + delta;
      }
    }
    samples_.push_back({image_ns, phase, unwrapped, raw_rate});
    while (!samples_.empty() &&
           (image_ns - samples_.front().ns) * 1e-9 > config_.window_s + 1e-9)
      samples_.pop_front();
  }

  Estimate estimate(std::int64_t now_ns, double current_raw_rate) const
  {
    Estimate out;
    out.epoch = epoch_; out.generation = generation_; out.samples = samples_.size();
    if (samples_.empty()) return out;
    out.image_ns = samples_.back().ns;
    out.age_s = (now_ns - out.image_ns) * 1e-9;
    out.span_s = (out.image_ns - samples_.front().ns) * 1e-9;
    if (now_ns < out.image_ns ||
        now_ns - out.image_ns > std::llround(config_.maximum_age_s * 1e9)) {
      out.reason = "stale"; return out;
    }
    if (out.samples < static_cast<std::size_t>(config_.minimum_samples) ||
        out.span_s + 1e-9 < config_.minimum_span_s) return out;
    const auto all = fit(samples_.front().ns, samples_.back().ns, true);
    out.rate_rad_s = all.rate; out.rms_rad = all.rms;
    const auto middle = samples_.front().ns + (out.image_ns - samples_.front().ns) / 2;
    const auto early = fit(samples_.front().ns, middle, false);
    const auto late = fit(middle, samples_.back().ns, true);
    if (!std::isfinite(all.rate) || early.count < 6 || late.count < 6 ||
        !std::isfinite(early.rate) || !std::isfinite(late.rate) ||
        all.rms > config_.maximum_rms_rad ||
        std::abs(early.rate - late.rate) > config_.maximum_half_rate_difference) {
      out.reason = "unstable"; return out;
    }
    if (!std::isfinite(current_raw_rate) || all.rate * current_raw_rate <= 0.0 ||
        std::abs(all.rate) > config_.maximum_abs_rate) {
      out.reason = "direction_or_rate"; return out;
    }
    out.ready = true; out.reason = "ready";
    return out;
  }

private:
  struct Sample { std::int64_t ns; double phase, unwrapped, raw_rate; };
  struct Fit { double rate = NAN, rms = NAN; std::size_t count = 0; };
  Config config_;
  std::deque<Sample> samples_;
  std::uint64_t generation_ = 0, epoch_ = 0;

  Fit fit(std::int64_t begin, std::int64_t end, bool include_end) const
  {
    Fit out;
    double mt = 0.0, my = 0.0;
    const auto inside = [&](const Sample & s) {
      return s.ns >= begin && (include_end ? s.ns <= end : s.ns < end);
    };
    for (const auto & s : samples_) if (inside(s)) {
      mt += (s.ns - begin) * 1e-9; my += s.unwrapped; ++out.count;
    }
    if (out.count < 2) return out;
    mt /= out.count; my /= out.count;
    double numerator = 0.0, denominator = 0.0;
    for (const auto & s : samples_) if (inside(s)) {
      const double dt = (s.ns - begin) * 1e-9 - mt;
      numerator += dt * (s.unwrapped - my); denominator += dt * dt;
    }
    if (denominator <= 1e-12) return out;
    out.rate = numerator / denominator;
    double squared = 0.0;
    for (const auto & s : samples_) if (inside(s)) {
      const double residual = s.unwrapped - my - out.rate * ((s.ns - begin) * 1e-9 - mt);
      squared += residual * residual;
    }
    out.rms = std::sqrt(squared / out.count);
    return out;
  }
};

// Only the reviewed active mode, in high-speed rotation, may replace the
// production angular rate. Off and shadow must stay byte-identical to legacy.
inline bool uses_stable_rotation(
  RotationPredictionMode mode, bool high_speed_mode, bool estimate_ready)
{
  return mode == RotationPredictionMode::active && high_speed_mode && estimate_ready;
}
// Off/shadow never change the production route. Active route changes revoke
// old candidate maturity; ordinary frame-to-frame fit updates do not.
class RotationPredictionEpoch
{
public:
  bool update(RotationPredictionMode mode, bool override_used, std::uint64_t epoch)
  {
    if (mode != RotationPredictionMode::active) return false;
    const auto key = override_used ? epoch + 1 : 0;
    const bool changed = initialized_ && key != previous_;
    initialized_ = true; previous_ = key;
    return changed;
  }
private:
  bool initialized_ = false;
  std::uint64_t previous_ = 0;
};
}  // namespace auto_aim
#endif
