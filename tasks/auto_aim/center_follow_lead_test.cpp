#include "center_follow_lead.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{
constexpr double kDeg = M_PI / 180.0;

// Exact replica of Target::h_armor_xyz (tasks/auto_aim/target.cpp): the armour
// sits at radius r from the centre, at body phase + i * 90 deg.
double armor_azimuth_deg(double d, double r, double body_phase_deg, int i)
{
  const double cx = d;
  const double cy = 0.0;
  const double ang = body_phase_deg * kDeg + i * M_PI / 2.0;
  return std::atan2(cy - r * std::sin(ang), cx - r * std::cos(ang)) / kDeg;
}
}

int main()
{
  // --- 1. The geometry that invalidates an omega * flight_time lead. -------
  // A vehicle spinning about its own centre keeps that centre fixed, so the
  // front plate's bearing never swings with the full body angle: it stays
  // within atan(r / d) of the centre.  Adding omega * flight_time would command
  // a lead an order of magnitude larger than the geometry can produce.
  {
    const double d = 4.5;
    const double r = 0.26;
    double worst = 0.0;
    for (int deg = 0; deg < 360; deg += 5) {
      double best = 1e9;
      for (int i = 0; i < 4; ++i) {
        const double az = std::abs(armor_azimuth_deg(d, r, deg, i));
        if (az < best) best = az;
      }
      if (best > worst) worst = best;
    }
    const double bound_deg = std::atan(r / d) / kDeg;
    // The front plate never leaves a window slightly wider than atan(r/d).
    assert(worst < 1.1 * bound_deg);
    assert(worst < 3.5);

    // And the omega * flight_time "lead" is far outside that window.
    const double omega_lead_deg = 1.5 * (d / 18.0) / kDeg;
    assert(omega_lead_deg > 8.0 * worst);
  }

  // --- 2. A stationary centre owes no lead. --------------------------------
  {
    const double az = auto_aim::center_azimuth_rad(4.5, 0.0);
    const double lead = auto_aim::center_follow_lead_rad(az, az, 3.0 * kDeg);
    assert(std::abs(lead) < 1e-12);
  }

  // --- 3. A translating centre produces a bounded, signed correction. ------
  {
    // 4.5 m down +x, moving 0.6 m/s in +y over a 0.25 s flight => +0.15 m.
    const double now_az = auto_aim::center_azimuth_rad(4.5, 0.0);
    const double future_az = auto_aim::center_azimuth_rad(4.5, 0.15);
    const double lead = auto_aim::center_follow_lead_rad(future_az, now_az, 3.0 * kDeg);
    assert(lead > 0.0);
    assert(std::abs(lead - future_az) < 1e-12);
    assert(lead < 3.0 * kDeg);

    // The mirrored motion gives the mirrored correction.
    const double future_neg = auto_aim::center_azimuth_rad(4.5, -0.15);
    const double lead_neg = auto_aim::center_follow_lead_rad(future_neg, now_az, 3.0 * kDeg);
    assert(lead_neg < 0.0);
    assert(std::abs(lead + lead_neg) < 1e-12);
  }

  // --- 4. The envelope clamps instead of chasing a wild estimate. ----------
  {
    const double now_az = auto_aim::center_azimuth_rad(4.5, 0.0);
    const double wild = auto_aim::center_azimuth_rad(1.0, 3.0);  // ~72 deg away
    const double lead = auto_aim::center_follow_lead_rad(wild, now_az, 3.0 * kDeg);
    assert(std::abs(lead - 3.0 * kDeg) < 1e-12);
    const double lead_neg = auto_aim::center_follow_lead_rad(-wild, now_az, 3.0 * kDeg);
    assert(std::abs(lead_neg + 3.0 * kDeg) < 1e-12);
    // A zero envelope disables the correction outright.
    assert(auto_aim::center_follow_lead_rad(wild, now_az, 0.0) == 0.0);
  }

  // --- 5. Wrap-around is handled: crossing +/-180 deg must stay small. -----
  {
    const double to = (179.0) * kDeg;
    const double from = (-179.0) * kDeg;
    const double diff = auto_aim::angular_difference_rad(to, from);
    assert(std::abs(diff - (-2.0 * kDeg)) < 1e-12);
    const double lead = auto_aim::center_follow_lead_rad(to, from, 3.0 * kDeg);
    assert(std::abs(lead - diff) < 1e-12);
  }

  // --- 6. Non-finite inputs produce no correction. -------------------------
  {
    const double lim = 3.0 * kDeg;
    assert(auto_aim::center_follow_lead_rad(NAN, 0.0, lim) == 0.0);
    assert(auto_aim::center_follow_lead_rad(0.0, NAN, lim) == 0.0);
    assert(auto_aim::center_follow_lead_rad(INFINITY, 0.0, lim) == 0.0);
  }

  std::cout << "center_follow_lead_test: PASS\n";
  return 0;
}
