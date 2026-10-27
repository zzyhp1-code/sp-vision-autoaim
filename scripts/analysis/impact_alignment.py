#!/usr/bin/env python3
"""Impact-time alignment audit for a single field log.

For every sent request, interpolate the *later measured* armour pose at the
Aimer's own prediction timestamp and compare its normal against the gimbal
bore.  This answers one question only: when the projectile is predicted to
arrive, is a plate actually on the bore line?

Fold convention
---------------
The four armour plates are 90 deg apart, so the bearing residual is folded to
+/-45 deg.  That makes the metric insensitive to which physical plate the model
called "face i".  A plate can only be hit while its centre is within roughly
atan(half_width / range) of the bore, i.e. about 1 deg at 4.5 m, which is why
both the median and the spread matter:

  * a large |median| that does not scale with omega is a bore/baseline offset;
  * a large spread is timing error, and `median * 1/omega` is its size in ms.

Usage: python3 impact_alignment.py LOG [LOG ...]
"""

import bisect
import json
import math
import statistics
import sys
from pathlib import Path

MAX_BRACKET_NS = 40_000_000  # interpolate only across a short real observation gap


def wrap(angle: float, period: float = 2.0 * math.pi) -> float:
    return (angle + period / 2.0) % period - period / 2.0


def load(path: Path):
    predictions, observations = [], []
    for line in path.read_text(errors="replace").splitlines():
        if "[AIMPRED] {" in line:
            try:
                predictions.append(json.loads(line.split("[AIMPRED] ", 1)[1]))
            except ValueError:
                pass
        elif "[AIMOBS] {" in line:
            try:
                observations.append(json.loads(line.split("[AIMOBS] ", 1)[1]))
            except ValueError:
                pass
    observations = [
        o for o in observations
        if o.get("pose_fresh") and o.get("measured_world_yaw_rad") is not None
    ]
    observations.sort(key=lambda o: o["image_ns"])
    return predictions, observations


def audit(path: Path) -> dict:
    predictions, observations = load(path)
    times = [o["image_ns"] for o in observations]
    residuals, horizons, omegas = [], [], []
    for p in predictions:
        if not p.get("valid") or not times:
            continue
        target_ns = p["prediction_ns"]
        index = bisect.bisect_left(times, target_ns)
        if index < 1 or index >= len(observations):
            continue
        lo, hi = observations[index - 1], observations[index]
        span = hi["image_ns"] - lo["image_ns"]
        if span <= 0 or span > MAX_BRACKET_NS:
            continue
        f = (target_ns - lo["image_ns"]) / span
        plate = lo["measured_world_yaw_rad"] + f * wrap(
            hi["measured_world_yaw_rad"] - lo["measured_world_yaw_rad"])
        gimbal = lo["gimbal_yaw_rad"] + f * wrap(hi["gimbal_yaw_rad"] - lo["gimbal_yaw_rad"])
        # The plate normal faces the bore, hence the pi offset before folding.
        residuals.append(math.degrees(wrap(plate - gimbal - math.pi, math.pi / 2.0)))
        horizons.append((target_ns - p["image_ns"]) / 1e6)
        raw = p.get("raw_state") or []
        if len(raw) >= 8:
            omegas.append(abs(raw[7]))
    if not residuals:
        return {"log": path.name, "paired": 0}

    absolute = sorted(abs(r) for r in residuals)
    omega = statistics.median(omegas) if omegas else float("nan")
    median = statistics.median(residuals)
    return {
        "log": path.name,
        "paired": len(residuals),
        "median_deg": round(median, 2),
        "median_abs_deg": round(statistics.median(absolute), 2),
        "p90_abs_deg": round(absolute[int(0.9 * (len(absolute) - 1))], 2),
        "within_1deg_pct": round(100.0 * sum(d <= 1.0 for d in absolute) / len(absolute), 1),
        "within_3deg_pct": round(100.0 * sum(d <= 3.0 for d in absolute) / len(absolute), 1),
        "within_5deg_pct": round(100.0 * sum(d <= 5.0 for d in absolute) / len(absolute), 1),
        "omega_median_rad_s": round(omega, 2),
        # A pure timing error would need this much extra lead; a bore offset would not.
        # residual[deg] / omega[rad/s] -> seconds requires converting degrees to rad.
        "equivalent_timing_ms": round(math.radians(median) / omega * 1000.0, 1)
        if omega and math.isfinite(omega) and omega > 0.1 else None,
        "horizon_median_ms": round(statistics.median(horizons), 1),
    }


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    rows = [audit(Path(arg)) for arg in argv[1:]]
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    paired = [r for r in rows if r.get("paired")]
    if len(paired) > 1:
        pooled = statistics.mean(r["median_abs_deg"] for r in paired)
        print(json.dumps({"pooled_median_abs_deg": round(pooled, 2)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
