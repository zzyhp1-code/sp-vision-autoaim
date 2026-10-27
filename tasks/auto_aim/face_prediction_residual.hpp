#ifndef AUTO_AIM__FACE_PREDICTION_RESIDUAL_HPP
#define AUTO_AIM__FACE_PREDICTION_RESIDUAL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace auto_aim
{

// Read-only audit of raw forecasts against later accepted, same-face visual
// observations. It does not measure projectile impacts or authorize a request.
// Face IDs must remain physical IDs within one tracker model generation.
class FacePredictionResidual
{
public:
  using Xyz = std::array<double, 3>;

  struct Config
  {
    std::int64_t bracket_ns = 30000000;       // Each side of the reference bracket.
    std::int64_t visibility_gap_ns = 150000000;
    std::int64_t frame_gap_ns = 250000000;
    std::int64_t history_age_ns = 10000000000LL;
    double max_horizon_s = 2.0;
    double max_omega_change_rad_s = 1.0;
    double max_speed_change_mps = 0.5;
    double max_delay_change_s = 0.001;
    double max_range_change_m = 0.5;
    double max_horizon_change_s = 0.12;
    double independent_turn_fraction = 0.75;
    std::size_t max_pending = 256;
    std::size_t max_observations_per_face = 256;
    std::size_t max_samples_per_passage = 256;
    std::size_t max_cycles_per_face = 32;
    std::size_t min_samples_per_cycle = 3;
  };

  struct Context
  {
    std::uint64_t model_generation = 0;
    int target_id = -1;
    int geometry = 0;       // Caller-defined, e.g. existing / whole EKF.
    int source = 0;         // Caller-defined estimator/predictor ownership.
    int speed_mode = 0;     // Explicit mode, not a frame-by-frame threshold here.
    int rotation_sign = 0;
    double bullet_speed_mps = 0.0;
    double base_delay_s = 0.0;
    double omega_rad_s = 0.0;
    double center_range_m = 0.0;
  };

  struct Observation
  {
    int face_id = -1;
    Xyz xyz{};  // Solved world position from an accepted tracker association.
  };

  struct Prediction
  {
    std::int64_t issued_ns = 0;
    std::int64_t prediction_ns = 0;
    int face_id = -1;
    Xyz xyz{};  // Raw final predicted world position, before aim trims.
    Context context;
  };

  struct Match
  {
    std::uint64_t epoch = 0;
    Context context;
    int face_id = -1;
    std::uint64_t passage_id = 0;
    std::int64_t issued_ns = 0, prediction_ns = 0, left_ns = 0, right_ns = 0;
    Xyz predicted_xyz{}, observed_xyz{}, error_xyz{};
    double yaw_error_rad = 0.0;  // predicted minus interpolated observation.
    double position_error_m = 0.0;
  };

  struct Summary
  {
    std::uint64_t epoch = 0, matched = 0, unmatched = 0;
    std::size_t pending = 0, completed_cycles = 0;
    // All summary statistics weight completed independent passages equally.
    // p90_abs_rad is the P90 of passage P90 absolute errors, not hit probability.
    double bias_rad = std::numeric_limits<double>::quiet_NaN();
    double scatter_rad = std::numeric_limits<double>::quiet_NaN();
    double p90_abs_rad = std::numeric_limits<double>::quiet_NaN();
  };

  FacePredictionResidual() : FacePredictionResidual(Config{}) {}
  explicit FacePredictionResidual(const Config & config) : config_(config)
  {
    if (config_.bracket_ns <= 0 || config_.visibility_gap_ns <= config_.bracket_ns ||
        config_.frame_gap_ns <= 0 || config_.history_age_ns <= 0 ||
        !positive(config_.max_horizon_s) || !positive(config_.max_omega_change_rad_s) ||
        !positive(config_.max_speed_change_mps) || !positive(config_.max_delay_change_s) ||
        !positive(config_.max_range_change_m) || !positive(config_.max_horizon_change_s) ||
        !positive(config_.independent_turn_fraction) ||
        config_.independent_turn_fraction > 1.0 || config_.max_pending == 0 ||
        config_.max_observations_per_face < 2 || config_.max_samples_per_passage == 0 ||
        config_.max_cycles_per_face == 0 || config_.min_samples_per_cycle == 0 ||
        config_.min_samples_per_cycle > config_.max_samples_per_passage) {
      throw std::invalid_argument("Invalid face prediction residual limits");
    }
  }

  void reset()
  {
    faces_ = {};
    pending_.clear();
    context_.reset();
    horizon_anchor_s_.reset();
    last_frame_ns_ = 0;
    last_accepted_ns_ = 0;
    matched_ = 0;
    unmatched_ = 0;
    ++epoch_;
  }

  // Call once per captured frame, even when no observations were accepted.
  // Repeated/backward clocks, long processing gaps and changed contexts clear
  // the audit. Duplicate accepted IDs are ambiguous and neither is retained.
  std::vector<Match> observe(
    std::int64_t frame_ns, const Context & context,
    const std::vector<Observation> & observations)
  {
    std::vector<Match> out;
    if (frame_ns <= 0 || !valid_context(context)) {
      reset();
      return out;
    }
    if ((last_frame_ns_ && (frame_ns <= last_frame_ns_ ||
         frame_ns - last_frame_ns_ > config_.frame_gap_ns)) ||
        (context_ && !same_context(*context_, context))) {
      reset();
    }
    if (!context_) context_ = context;
    last_frame_ns_ = frame_ns;

    std::array<int, 4> counts{};
    for (const auto & observation : observations) {
      if (valid_face(observation.face_id)) ++counts[observation.face_id];
    }
    for (const auto & observation : observations) {
      if (!valid_face(observation.face_id) || counts[observation.face_id] != 1 ||
          !valid_xyz(observation.xyz)) continue;
      auto & face = faces_[observation.face_id];
      if (face.passages.empty() ||
          frame_ns - face.passages.back().last_ns > config_.visibility_gap_ns) {
        face.passages.push_back(Passage{++face.next_passage, frame_ns, frame_ns, {}});
      }
      auto & passage = face.passages.back();
      passage.last_ns = frame_ns;
      face.observations.push_back({frame_ns, observation.xyz, passage.id});
      while (face.observations.size() > config_.max_observations_per_face ||
             (!face.observations.empty() &&
              (frame_ns - face.observations.front().time_ns) / 1e9 >
                config_.max_horizon_s + 2.0 * config_.bracket_ns / 1e9)) {
        face.observations.pop_front();
      }
      last_accepted_ns_ = frame_ns;
    }

    // Wait until the complete allowed right-hand reference window elapsed.
    // Missing brackets count as unmatched; no other face or extrapolation is
    // allowed, and even an exact sample uses two distinct bracketing times.
    while (!pending_.empty() &&
           pending_.front().prediction_ns <= frame_ns - config_.bracket_ns) {
      const auto prediction = pending_.front();
      pending_.pop_front();
      const auto reference = interpolate(faces_[prediction.face_id], prediction.prediction_ns);
      if (!reference) {
        ++unmatched_;
        continue;
      }
      Match match;
      match.epoch = epoch_;
      match.context = prediction.context;
      match.face_id = prediction.face_id;
      match.passage_id = reference->passage_id;
      match.issued_ns = prediction.issued_ns;
      match.prediction_ns = prediction.prediction_ns;
      match.left_ns = reference->left_ns;
      match.right_ns = reference->right_ns;
      match.predicted_xyz = prediction.xyz;
      match.observed_xyz = reference->xyz;
      for (std::size_t axis = 0; axis < 3; ++axis)
        match.error_xyz[axis] = prediction.xyz[axis] - reference->xyz[axis];
      match.yaw_error_rad = std::remainder(
        std::atan2(prediction.xyz[1], prediction.xyz[0]) -
          std::atan2(reference->xyz[1], reference->xyz[0]), 2.0 * pi);
      match.position_error_m = std::hypot(
        std::hypot(match.error_xyz[0], match.error_xyz[1]), match.error_xyz[2]);
      if (!std::isfinite(match.yaw_error_rad) || !std::isfinite(match.position_error_m)) {
        ++unmatched_;
        continue;
      }
      ++matched_;
      auto & passages = faces_[prediction.face_id].passages;
      const auto passage = std::find_if(passages.begin(), passages.end(),
        [&](const Passage & p) { return p.id == reference->passage_id; });
      if (passage != passages.end() && passage->errors.size() < config_.max_samples_per_passage)
        passage->errors.push_back(match.yaw_error_rad);
      out.push_back(match);  // Large errors are evidence too; never trim them.
    }
    for (auto & face : faces_) {
      while (!face.passages.empty() &&
             frame_ns - face.passages.front().last_ns > config_.visibility_gap_ns) {
        close_passage(face, face.passages.front());
        face.passages.pop_front();
      }
      while (!face.cycles.empty() &&
             frame_ns - face.cycles.front().time_ns > config_.history_age_ns)
        face.cycles.pop_front();
    }
    return out;
  }

  // Enqueue only a valid, converged prediction. Requiring an accepted source
  // frame prevents unobserved coast-only forecasts from creating evidence.
  bool enqueue(const Prediction & prediction)
  {
    if (!context_ || !valid_context(prediction.context) ||
        !same_context(*context_, prediction.context) ||
        !valid_face(prediction.face_id) || !valid_xyz(prediction.xyz) ||
        last_accepted_ns_ != last_frame_ns_ || prediction.issued_ns < last_frame_ns_ ||
        prediction.issued_ns - last_frame_ns_ > config_.frame_gap_ns ||
        prediction.prediction_ns <= prediction.issued_ns) return false;
    const double horizon_s = (prediction.prediction_ns - prediction.issued_ns) / 1e9;
    if (horizon_s > config_.max_horizon_s) return false;
    if (horizon_anchor_s_ &&
        std::abs(horizon_s - *horizon_anchor_s_) > config_.max_horizon_change_s) {
      reset();
      return false;
    }
    if (!horizon_anchor_s_) horizon_anchor_s_ = horizon_s;
    if (pending_.size() >= config_.max_pending) {
      pending_.pop_front();
      ++unmatched_;
    }
    pending_.insert(std::upper_bound(pending_.begin(), pending_.end(), prediction.prediction_ns,
      [](std::int64_t time, const Prediction & p) { return time < p.prediction_ns; }), prediction);
    return true;
  }

  Summary summary(int face_id) const
  {
    Summary out;
    out.epoch = epoch_;
    out.matched = matched_;
    out.unmatched = unmatched_;
    out.pending = pending_.size();
    if (!valid_face(face_id)) return out;
    const auto & cycles = faces_[face_id].cycles;
    out.completed_cycles = cycles.size();
    if (cycles.empty()) return out;
    std::vector<double> biases, bounds;
    for (const auto & cycle : cycles) {
      biases.push_back(cycle.bias_rad);
      bounds.push_back(cycle.p90_abs_rad);
    }
    out.bias_rad = circular_median(biases);
    for (auto & bias : biases) bias = std::abs(std::remainder(bias - out.bias_rad, 2.0 * pi));
    out.scatter_rad = median(biases);
    out.p90_abs_rad = quantile90(bounds);
    return out;
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  struct TimedObservation
  {
    std::int64_t time_ns;
    Xyz xyz;
    std::uint64_t passage_id;
  };
  struct Reference
  {
    Xyz xyz;
    std::int64_t left_ns, right_ns;
    std::uint64_t passage_id;
  };
  struct Passage
  {
    std::uint64_t id;
    std::int64_t first_ns, last_ns;
    std::vector<double> errors;
  };
  struct Cycle { std::int64_t time_ns; double bias_rad, p90_abs_rad; };
  struct Face
  {
    std::deque<TimedObservation> observations;
    std::deque<Passage> passages;
    std::deque<Cycle> cycles;
    std::optional<std::int64_t> last_independent_ns;
    std::uint64_t next_passage = 0;
  };
  Config config_;
  std::array<Face, 4> faces_;
  std::deque<Prediction> pending_;
  std::optional<Context> context_;
  std::optional<double> horizon_anchor_s_;
  std::int64_t last_frame_ns_ = 0, last_accepted_ns_ = 0;
  std::uint64_t epoch_ = 0, matched_ = 0, unmatched_ = 0;

  static bool positive(double v) { return std::isfinite(v) && v > 0.0; }
  static bool valid_face(int id) { return id >= 0 && id < 4; }
  static bool valid_xyz(const Xyz & xyz)
  {
    return std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]) &&
      std::isfinite(std::hypot(xyz[0], xyz[1])) && std::hypot(xyz[0], xyz[1]) > 1e-9;
  }
  static bool valid_context(const Context & c)
  {
    return c.model_generation != 0 && c.target_id >= 0 && c.geometry >= 0 && c.source >= 0 &&
      c.speed_mode >= 0 && c.rotation_sign >= -1 && c.rotation_sign <= 1 &&
      positive(c.bullet_speed_mps) && std::isfinite(c.base_delay_s) && c.base_delay_s >= 0.0 &&
      std::isfinite(c.omega_rad_s) && positive(c.center_range_m) &&
      ((c.rotation_sign == 0 && std::abs(c.omega_rad_s) <= 1e-6) ||
       c.rotation_sign * c.omega_rad_s > 0.0);
  }
  bool same_context(const Context & a, const Context & b) const
  {
    return a.model_generation == b.model_generation && a.target_id == b.target_id &&
      a.geometry == b.geometry && a.source == b.source && a.speed_mode == b.speed_mode &&
      a.rotation_sign == b.rotation_sign &&
      std::abs(a.bullet_speed_mps - b.bullet_speed_mps) <= config_.max_speed_change_mps &&
      std::abs(a.base_delay_s - b.base_delay_s) <= config_.max_delay_change_s &&
      std::abs(a.omega_rad_s - b.omega_rad_s) <= config_.max_omega_change_rad_s &&
      std::abs(a.center_range_m - b.center_range_m) <= config_.max_range_change_m;
  }
  static double median(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : values[n / 2 - 1] / 2.0 + values[n / 2] / 2.0;
  }
  static double quantile90(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(std::ceil(0.9 * values.size())) - 1];
  }
  static double circular_median(std::vector<double> values)
  {
    const double anchor = values.front();
    for (auto & value : values) value = anchor + std::remainder(value - anchor, 2.0 * pi);
    return std::remainder(median(values), 2.0 * pi);
  }
  std::optional<Reference> interpolate(const Face & face, std::int64_t time_ns) const
  {
    auto right = std::lower_bound(face.observations.begin(), face.observations.end(), time_ns,
      [](const TimedObservation & o, std::int64_t time) { return o.time_ns < time; });
    if (right == face.observations.end()) return {};
    if (right == face.observations.begin()) {
      if (right->time_ns != time_ns || std::next(right) == face.observations.end()) return {};
      ++right;
    }
    const auto left = std::prev(right);
    if (left->passage_id != right->passage_id ||
        time_ns - left->time_ns > config_.bracket_ns ||
        right->time_ns - time_ns > config_.bracket_ns) return {};
    const double weight = static_cast<double>(time_ns - left->time_ns) /
      static_cast<double>(right->time_ns - left->time_ns);
    if (weight < 0.0 || weight > 1.0) return {};
    Reference reference{{}, left->time_ns, right->time_ns, left->passage_id};
    for (std::size_t axis = 0; axis < 3; ++axis)
      reference.xyz[axis] = left->xyz[axis] + weight * (right->xyz[axis] - left->xyz[axis]);
    if (!valid_xyz(reference.xyz)) return {};
    return reference;
  }
  void close_passage(Face & face, const Passage & passage)
  {
    if (passage.errors.size() < config_.min_samples_per_cycle || !context_ ||
        std::abs(context_->omega_rad_s) <= 0.1) return;
    const std::int64_t center_ns = passage.first_ns + (passage.last_ns - passage.first_ns) / 2;
    const double separation_ns = config_.independent_turn_fraction * 2.0 * pi /
      std::abs(context_->omega_rad_s) * 1e9;
    if (face.last_independent_ns && center_ns - *face.last_independent_ns < separation_ns) return;
    auto absolute_errors = passage.errors;
    for (auto & value : absolute_errors) value = std::abs(value);
    face.cycles.push_back({center_ns, circular_median(passage.errors), quantile90(absolute_errors)});
    face.last_independent_ns = center_ns;
    while (face.cycles.size() > config_.max_cycles_per_face) face.cycles.pop_front();
  }
};

}  // namespace auto_aim

#endif  // AUTO_AIM__FACE_PREDICTION_RESIDUAL_HPP
