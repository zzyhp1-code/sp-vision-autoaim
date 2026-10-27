#ifndef AUTO_AIM__CENTER_FOLLOW_LEAD_HPP
#define AUTO_AIM__CENTER_FOLLOW_LEAD_HPP

#include <algorithm>
#include <cmath>

namespace auto_aim
{

// Yaw correction for the vehicle-centre follow path.
//
// Why this is NOT an omega * flight_time lead
// -------------------------------------------
// A spinning vehicle turns about its own centre, so the rotation centre does
// not move.  Aiming at the centre therefore does not fall behind the way
// aiming at a translating target would: there is no omega * flight_time term
// to add.  Concretely, the armour sits at radius r from the centre, so in the
// gimbal frame the front plate's bearing is at most atan(r / d) away from the
// centre bearing (2.3 deg at r = 0.26 m, d = 4.5 m); it never swings with the
// full body angle.  Adding omega * flight_time would command a lead roughly an
// order of magnitude larger than the geometry can produce.
//
// What actually needs correcting
// -------------------------------
// The centre is tracked from the rotation model, which is fitted from a short
// recent window.  If the body also drifts laterally, the model lags the true
// centre and the gimbal points slightly behind the plate that is about to be
// faced.  The measurable symptom is a persistent offset between the camera
// bearing of the visible plate and the estimated centre bearing.  This helper
// converts a centre position into its gimbal azimuth and returns the bounded
// correction from the estimated centre toward the (Kalman-predicted) future
// centre.
//
// The envelope is deliberately small: it must stay well inside atan(r / d),
// because anything larger would command the gimbal off the plate instead of
// onto it.

// Bearing of a centre position in the plane, as atan2(y, x).
inline double center_azimuth_rad(double x_m, double y_m)
{
  return std::atan2(y_m, x_m);
}

// Shortest signed angular difference (to - from), wrapped to (-pi, pi].
inline double angular_difference_rad(double to_rad, double from_rad)
{
  return std::remainder(to_rad - from_rad, 2.0 * M_PI);
}

// Bounded correction that moves the centre estimate toward the predicted
// centre.  Returns 0 when either azimuth is unusable.
inline double center_follow_lead_rad(
  double predicted_center_azimuth_rad, double estimated_center_azimuth_rad,
  double max_lead_rad)
{
  if (!std::isfinite(predicted_center_azimuth_rad) ||
      !std::isfinite(estimated_center_azimuth_rad)) {
    return 0.0;
  }
  const double limit = std::max(0.0, max_lead_rad);
  const double delta =
    angular_difference_rad(predicted_center_azimuth_rad, estimated_center_azimuth_rad);
  return std::clamp(delta, -limit, limit);
}

}  // namespace auto_aim

#endif
