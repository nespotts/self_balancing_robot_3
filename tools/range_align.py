#!/usr/bin/env python3
"""Range head alignment from a flat wall (issue #7). Runs on the Pi (stdlib only).

Setup: robot square to a flat wall 30-60 cm in front (axle parallel to the wall), motors off,
rng.servo_hold_deg -1, rng.sweep_deg 90, rng.steps 19 (5 deg apart), wait ~20 s, then:

    python3 tools/range_align.py --seconds 15

Fits the front sensor's readings to a flat wall:  r + 12.9 = D / cos(k * a + phi)
(a = reported head angle). The wall is perpendicular to robot forward, so:
  phi -> head zero error -> new rng.servo_trim_deg
  k   -> real degrees per reported degree -> new rng.servo_deg_per_cmd
A wall can't tell left from right: check rng.servo_dir by eye (hold 175 must turn sensor 0 LEFT).
"""
import argparse
import glob
import json
import math
import os
import statistics

SENSOR_RADIUS_MM = 12.9


def load(path, seconds):
    rows, schema = [], None
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r[0] == "schema":
                schema = r[1]
            else:
                rows.append(r)
    tend = max(r[1] for r in rows if isinstance(r[1], (int, float)))
    params = {}
    for r in rows:
        if r[0] == "param":
            params[r[3]] = r[4]
    cols = schema["rng"]
    rng = [dict(zip(cols, r[2:])) for r in rows if r[0] == "rng" and r[1] >= tend - seconds]
    return rng, params


def fit(pts):
    """pts: [(a_deg, r_mm)] -> (rms, D, k, phi)"""
    best = None
    for ki in range(-40, 41):
        k = 1.0 + ki * 0.005
        for pi in range(-300, 301):
            phi = pi * 0.1
            g = [1.0 / math.cos(math.radians(k * a + phi)) for a, _ in pts]
            if any(x <= 0 for x in g):
                continue
            D = sum(r * x for (_, r), x in zip(pts, g)) / sum(x * x for x in g)
            e = sum((r - D * x) ** 2 for (_, r), x in zip(pts, g))
            if best is None or e < best[0]:
                best = (e, D, k, phi)
    e, D, k, phi = best
    return math.sqrt(e / len(pts)), D, k, phi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--file")
    ap.add_argument("--max-angle", type=float, default=50, help="use readings within +- this of forward")
    a = ap.parse_args()
    path = a.file or sorted(glob.glob(os.path.expanduser("~/sbr3-logs/sbr3-*.jsonl")))[-1]
    rng, params = load(path, a.seconds)
    bins = {}
    for d in rng:
        if d["sensor_id"] == 0 and d["valid"] and abs(d["sensor_angle_deg"]) <= a.max_angle:
            bins.setdefault(round(d["sensor_angle_deg"], 2), []).append(d["range_mm"] + SENSOR_RADIUS_MM)
    pts = sorted((ang, statistics.median(v)) for ang, v in bins.items() if len(v) >= 2)
    print("%s: %d readings, %d angles" % (os.path.basename(path), len(rng), len(pts)))
    if len(pts) < 5:
        print("not enough front readings (need a sweep across the wall)")
        return
    for _ in range(2):  # drop points that don't sit on the wall, refit
        rms, D, k, phi = fit(pts)
        res = [abs(r - D / math.cos(math.radians(k * ang + phi))) for ang, r in pts]
        lim = max(15.0, 3 * statistics.median(res))
        keep = [p for p, e in zip(pts, res) if e <= lim]
        if len(keep) == len(pts) or len(keep) < 5:
            break
        pts = keep
    for ang, r in pts:
        print("  a %7.2f  r %6.0f  fit %6.0f" % (ang, r, D / math.cos(math.radians(k * ang + phi))))
    trim = params.get("rng.servo_trim_deg", 0.0)
    scale = params.get("rng.servo_deg_per_cmd", 1.0)
    sdir = -1 if params.get("rng.servo_dir", 1.0) < 0 else 1
    a0 = -phi / k  # reported angle where the head really points straight at the wall
    print("wall %.0f mm, rms %.1f mm, k %.3f, head zero at reported %+.1f deg" % (D, rms, k, a0))
    print("now:      rng.servo_trim_deg %.1f  rng.servo_deg_per_cmd %.3f  rng.servo_dir %d" % (trim, scale, sdir))
    print("suggest:  rng.servo_trim_deg %.1f  rng.servo_deg_per_cmd %.3f" % (trim + sdir * a0 / scale, scale * k))


if __name__ == "__main__":
    main()
