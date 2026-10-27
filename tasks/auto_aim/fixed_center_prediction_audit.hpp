#ifndef AUTO_AIM__FIXED_CENTER_PREDICTION_AUDIT_HPP
#define AUTO_AIM__FIXED_CENTER_PREDICTION_AUDIT_HPP

#include "fixed_center_shadow.hpp"

namespace auto_aim
{

// Two read-only forecasts, issued together and scored with the exact same
// future visual bracket. This class has no command or tracker mutation API.
class FixedCenterPredictionAudit
{
public:
  using Residual = FacePredictionResidual;
  using Shadow = FixedCenterShadow;
  struct Metadata
  {
    std::int64_t issued_ns = 0, prediction_ns = 0, reference_ns = 0;
    int face_id = -1;
    std::uint64_t model_epoch = 0, revision = 0;
    bool stationary_supported = false;
    Shadow::Reason reason = Shadow::Reason::warmup;
    double center_speed_mps = 0.0, heldout_yaw_p90_rad = 0.0;
  };
  struct Pair { Residual::Match main, fixed; Metadata metadata; };
  struct Counters
  {
    std::uint64_t attempted = 0, available = 0, enqueued = 0, main_skipped = 0;
    std::uint64_t matched = 0, stationary_matched = 0, no_reference = 0;
    std::uint64_t discarded_reset = 0, overflow = 0, source_rejected = 0, pairing_errors = 0;
    std::array<std::uint64_t, 14> reasons{};
  };
  struct EnqueueResult { Shadow::Forecast forecast; bool enqueued = false; };

  FixedCenterPredictionAudit() : FixedCenterPredictionAudit(Residual::Config{}) {}
  explicit FixedCenterPredictionAudit(const Residual::Config & config)
  : config_(config), main_(config), fixed_(config) {}

  // Lifetime counters survive resets so loss of pending evidence stays visible.
  void reset()
  {
    if (!active_) return;
    counters_.discarded_reset += pending_.size();
    pending_.clear(); main_.reset(); fixed_.reset(); active_ = false;
  }
  void skip_main() { ++counters_.main_skipped; }
  const Counters & counters() const { return counters_; }
  Residual::Summary main_summary(int face) const { return main_.summary(face); }
  Residual::Summary fixed_summary(int face) const { return fixed_.summary(face); }

  std::vector<Pair> observe(std::int64_t frame_ns, const Residual::Context & context,
    const std::vector<Residual::Observation> & observations)
  {
    const auto epoch = main_.summary(0).epoch;
    const auto main_matches = main_.observe(frame_ns, context, observations);
    const auto fixed_matches = fixed_.observe(frame_ns, context, observations);
    active_ = true;
    if (main_.summary(0).epoch != epoch) {
      counters_.discarded_reset += pending_.size();
      pending_.clear();
    }
    std::vector<Pair> pairs;
    if (main_matches.size() != fixed_matches.size()) {
      ++counters_.pairing_errors; reset(); return pairs;
    }
    for (std::size_t i = 0; i < main_matches.size(); ++i) {
      const auto & a = main_matches[i];
      const auto & b = fixed_matches[i];
      const auto metadata = std::find_if(pending_.begin(), pending_.end(),
        [&](const Metadata & m) { return same_key(m, a); });
      if (metadata == pending_.end() || a.issued_ns != b.issued_ns ||
          a.prediction_ns != b.prediction_ns || a.face_id != b.face_id ||
          a.left_ns != b.left_ns || a.right_ns != b.right_ns ||
          a.observed_xyz != b.observed_xyz || a.passage_id != b.passage_id) {
        ++counters_.pairing_errors; reset(); return {};
      }
      pairs.push_back({a, b, *metadata});
      ++counters_.matched;
      if (metadata->stationary_supported) ++counters_.stationary_matched;
      pending_.erase(metadata);
    }
    // Both scorers have now consumed all predictions due at this frame,
    // including those for which no same-face visual bracket was available.
    while (!pending_.empty() && pending_.front().prediction_ns <= frame_ns - config_.bracket_ns) {
      ++counters_.no_reference;
      pending_.pop_front();
    }
    return pairs;
  }

  EnqueueResult enqueue(const Residual::Prediction & prediction, const Shadow & shadow)
  {
    ++counters_.attempted;
    EnqueueResult result;
    result.forecast = shadow.forecast(prediction.issued_ns, prediction.prediction_ns, prediction.face_id);
    if (!shadow.matches_context(prediction.context)) {
      result.forecast.available = result.forecast.stationary_supported = false;
      result.forecast.reason = Shadow::Reason::context_changed;
    }
    ++counters_.reasons[static_cast<std::size_t>(result.forecast.reason)];
    if (!result.forecast.available) return result;
    ++counters_.available;
    auto alternative = prediction;
    alternative.xyz = result.forecast.xyz;
    const auto epoch = main_.summary(0).epoch;
    const bool a = main_.enqueue(prediction), b = fixed_.enqueue(alternative);
    if (main_.summary(0).epoch != epoch) {
      counters_.discarded_reset += pending_.size(); pending_.clear();
    }
    if (a != b) {
      ++counters_.pairing_errors; reset(); return result;
    }
    if (!a) { ++counters_.source_rejected; return result; }
    if (pending_.size() >= config_.max_pending) {
      pending_.pop_front(); ++counters_.overflow;
    }
    const auto & fit = result.forecast.fit;
    Metadata metadata{prediction.issued_ns, prediction.prediction_ns, fit.model.reference_ns,
      prediction.face_id, fit.epoch, fit.revision, result.forecast.stationary_supported,
      result.forecast.reason, fit.heldout_center_speed_mps, fit.heldout_yaw_p90_rad};
    pending_.insert(std::upper_bound(pending_.begin(), pending_.end(), metadata.prediction_ns,
      [](std::int64_t time, const Metadata & m) { return time < m.prediction_ns; }), metadata);
    ++counters_.enqueued; result.enqueued = true;
    return result;
  }

private:
  static bool same_key(const Metadata & metadata, const Residual::Match & match)
  {
    return metadata.issued_ns == match.issued_ns && metadata.prediction_ns == match.prediction_ns &&
      metadata.face_id == match.face_id;
  }
  Residual::Config config_;
  Residual main_, fixed_;
  std::deque<Metadata> pending_;
  Counters counters_;
  bool active_ = false;
};

}  // namespace auto_aim
#endif
