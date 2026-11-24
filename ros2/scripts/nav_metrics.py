#!/usr/bin/env python3
"""Compare Nav2 runs (sim_e2e_goals.sh output dirs) on the same goals: a table + plots.

  python3 ros2/scripts/nav_metrics.py --map MAP.yaml --run "#96 RPP=DIR1" --run "MPPI=DIR2" [--box 2000,100,300,300]
      [--png out.png] [--md out.md]
Per run (all goals): time to goal, path length (true path, sim), min clearance (robot centre to the nearest true
wall, sim), stops (speed down to ~0 and up again before the goal), reversals (backing up), peak commanded accel
and decel (/cmd_vel into the adapter), lean change (95th percentile of |pitch - median|, what a balancing robot feels).
"""
import argparse
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", ".."))
from map_png import load_map  # noqa: E402

COLORS = ["#888888", "#2a7fff", "#ee7700", "#22aa44", "#aa33cc"]


def rows(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def fl(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def smooth(xs, n):
    out = []
    for i in range(len(xs)):
        w = xs[max(0, i - n + 1):i + 1]
        out.append(sum(w) / len(w))
    return out


def run_metrics(d, room):
    goals = []
    i = 1
    while os.path.exists(os.path.join(d, "goal%d.json" % i)):
        g = json.load(open(os.path.join(d, "goal%d.json" % i)))
        tr = g.get("track") or []
        goals.append({"goal": g["goal"], "result": g["result"], "seconds": g["seconds"],
                      "t0": tr[0][0] if tr else None, "t1": tr[-1][0] if tr else None})
        i += 1
    truth = [(float(r["t"]), float(r["X"]) / 1000, float(r["Y"]) / 1000) for r in rows(os.path.join(d, "truth.csv"))]
    tk = [r for r in rows(os.path.join(d, "track.csv"))]
    res = {"goals": goals, "time_s": 0.0, "path_m": 0.0, "min_clear_cm": 1e9, "stops": 0, "reversals": 0,
           "peak_acc": 0.0, "peak_dec": 0.0, "peak_lean": 0.0, "succeeded": 0, "paths": [], "speed": []}
    t_first = goals[0]["t0"] if goals and goals[0]["t0"] else 0
    pitches = [fl(r["pitch"]) for r in tk if fl(r["pitch"]) is not None]
    p_med = sorted(pitches)[len(pitches) // 2] if pitches else 0.0
    for g in goals:
        if g["t0"] is None:
            continue
        res["succeeded"] += g["result"] == "succeeded"
        res["time_s"] += g["seconds"]
        t0, t1 = g["t0"], g["t0"] + g["seconds"]
        P = [(t, x, y) for t, x, y in truth if t0 <= t <= t1]
        res["paths"].append(P)
        res["path_m"] += sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(P, P[1:]))
        for t, x, y in P:
            res["min_clear_cm"] = min(res["min_clear_cm"], room.clearance(x * 1000, y * 1000) / 10.0)
        seg = [r for r in tk if t0 <= float(r["t"]) <= t1]
        ts = [float(r["t"]) for r in seg]
        cv = [fl(r["cmd_v"]) or 0.0 for r in seg]
        cvs = smooth(cv, 4)                                      # 0.2 s at 20 Hz
        for a, b, ta, tb in zip(cvs, cvs[1:], ts, ts[1:]):
            if tb > ta:
                acc = (b - a) / (tb - ta)
                res["peak_acc"] = max(res["peak_acc"], acc)
                res["peak_dec"] = max(res["peak_dec"], -acc)
        moving, low = False, False
        for v in cv:                                             # stop = fast -> ~0 -> fast again
            if v >= 0.06:
                if low:
                    res["stops"] += 1
                moving, low = True, False
            elif moving and abs(v) <= 0.01:
                low = True
        rev = False
        for v in cv:
            if v < -0.02 and not rev:
                res["reversals"] += 1
            rev = v < -0.02
        for r in seg:
            p = fl(r["pitch"])
            if p is not None:
                res.setdefault("leans", []).append(abs(p - p_med))
        res["speed"].append([(t - t_first, v) for t, v in zip(ts, cv)])
    ls = sorted(res.pop("leans", []))           # 95th percentile: the sim's floor bumps make the max noisy
    res["peak_lean"] = ls[int(0.95 * (len(ls) - 1))] if ls else 0.0
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--run", action="append", required=True, help="LABEL=DIR")
    ap.add_argument("--box", action="append", default=[])
    ap.add_argument("--png", default="nav_compare.png")
    ap.add_argument("--md", default=None)
    a = ap.parse_args()
    from station.sim.sim import Room
    from station.sim.extras import parse_box
    room = Room()
    for b in a.box:
        room.base += parse_box(b)
    room.segs = list(room.base)
    runs = []
    for spec in a.run:
        label, d = spec.split("=", 1)
        runs.append((label, run_metrics(d, room)))
    hdr = "| | goals reached | time (s) | path (m) | min clearance (cm, centre) | stops | reversals | " \
          "peak accel / decel (m/s², cmd) | lean change p95 (deg) |"
    lines = [hdr, "|" + "---|" * 9]
    for label, r in runs:
        lines.append("| %s | %d / %d | %.0f | %.2f | %.0f | %d | %d | %.2f / %.2f | %.1f |" % (
            label, r["succeeded"], len(r["goals"]), r["time_s"], r["path_m"], r["min_clear_cm"], r["stops"],
            r["reversals"], r["peak_acc"], r["peak_dec"], r["peak_lean"]))
    table = "\n".join(lines)
    print(table)
    if a.md:
        open(a.md, "w").write(table + "\n")

    img, res, origin = load_map(a.map)
    w, h = img.size
    fig = plt.figure(figsize=(15, 6.2), dpi=110)
    ax = fig.add_subplot(1, 2, 1)
    ax.imshow(img, cmap="gray", extent=[origin[0], origin[0] + w * res, origin[1], origin[1] + h * res],
              origin="upper", vmin=0, vmax=255, interpolation="nearest")
    for b in a.box:
        segs = parse_box(b)
        for s in segs:
            ax.plot([s[0] / 1000, s[2] / 1000], [s[1] / 1000, s[3] / 1000], color="#d00", lw=2)
    ax.plot([], [], color="#d00", lw=2, label="box not on the map")
    for k, (label, r) in enumerate(runs):
        for j, P in enumerate(r["paths"]):
            ax.plot([p[1] for p in P], [p[2] for p in P], color=COLORS[k % len(COLORS)], lw=1.8, alpha=0.9,
                    label=label if j == 0 else None)
    for g in runs[0][1]["goals"]:
        gx, gy, gyaw = g["goal"]
        ax.plot(gx, gy, "*", color="#dd0000", ms=13)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("true paths (sim), same goals (red stars), same start (0, 0)", fontsize=10)
    n = len(runs)
    for k, (label, r) in enumerate(runs):
        bx = fig.add_subplot(n, 2, 2 * (k + 1))
        for seg in r["speed"]:
            bx.plot([s[0] for s in seg], [s[1] for s in seg], color=COLORS[k % len(COLORS)], lw=1.2)
        bx.set_ylim(-0.12, 0.28)
        bx.axhline(0, color="#999", lw=0.6)
        bx.set_ylabel("m/s", fontsize=8)
        bx.text(0.01, 0.82, "%s: cmd speed" % label, transform=bx.transAxes, fontsize=9)
        bx.tick_params(labelsize=8)
        if k == n - 1:
            bx.set_xlabel("s since the first goal")
    fig.tight_layout()
    fig.savefig(a.png)
    print("wrote", a.png)


if __name__ == "__main__":
    main()
