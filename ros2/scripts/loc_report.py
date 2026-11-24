#!/usr/bin/env python3
"""Localization test report (sim_e2e_localize.sh): ROS map pose (pose_logger track.csv) vs the sim's truth
(truth.csv; the map frame is the sim world when the map was made from the sim's start pose).

  python3 ros2/scripts/loc_report.py OUT_DIR --map MAP.yaml [--title amcl] [--png OUT_DIR/localize.png]
Prints the numbers (and writes OUT_DIR/report.json): time to relocalize at start and after the kidnap, pose errors
while standing / driving, goal results.
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_png import load_map  # noqa: E402

GOOD_M, GOOD_DEG, HOLD_S = 0.10, 5.0, 2.0


def read_csv(p):
    with open(p) as f:
        return [r for r in csv.DictReader(f)]


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def interp_truth(truth, t):
    ts = [r[0] for r in truth]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    a, b = truth[i - 1], truth[i]
    if b[0] - a[0] > 1.0:
        return None
    u = (t - a[0]) / (b[0] - a[0])
    dth = math.remainder(b[3] - a[3], 2 * math.pi)
    return a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * dth


def errors(track, truth):
    out = []
    for r in track:
        t, x, y, yaw = fnum(r["t"]), fnum(r["x"]), fnum(r["y"]), fnum(r["yaw"])
        tr = interp_truth(truth, t)
        if tr is None:
            continue
        if x is None:
            out.append((t, None, None, fnum(r.get("q")), tr, None))
            continue
        e = math.hypot(x - tr[0], y - tr[1])
        eh = abs(math.degrees(math.remainder(yaw - tr[2], 2 * math.pi)))
        out.append((t, e, eh, fnum(r.get("q")), tr, (x, y, yaw)))
    return out


def settle_time(errs, t0, t1=None):
    """First time after t0 from which the error stays < GOOD for HOLD_S."""
    good_since = None
    for t, e, eh, *_ in errs:
        if t < t0 or (t1 is not None and t > t1):
            continue
        ok = e is not None and e < GOOD_M and eh < GOOD_DEG
        if ok:
            good_since = good_since or t
            if t - good_since >= HOLD_S:
                return good_since - t0
        else:
            good_since = None
    return None


def stats(errs, t0, t1):
    es = [e for t, e, *_ in errs if t0 <= t <= t1 and e is not None]
    hs = [eh for t, e, eh, *_ in errs if t0 <= t <= t1 and eh is not None]
    if not es:
        return None
    es.sort()
    return {"n": len(es), "mean_cm": round(100 * sum(es) / len(es), 1), "median_cm": round(100 * es[len(es) // 2], 1),
            "max_cm": round(100 * es[-1], 1), "mean_deg": round(sum(hs) / len(hs), 1), "max_deg": round(max(hs), 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--map", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()
    O = a.out
    truth = [(float(r["t"]), float(r["X"]) / 1000, float(r["Y"]) / 1000, math.radians(float(r["TH"])))
             for r in read_csv(os.path.join(O, "truth.csv"))]
    track = read_csv(os.path.join(O, "track.csv"))
    errs = errors(track, truth)
    rd = lambda n: float(open(os.path.join(O, n)).read()) if os.path.exists(os.path.join(O, n)) else None
    t_arm, t_kid = rd("t_armed"), rd("t_kidnap")
    goals = {}
    for i in (1, 2):
        p = os.path.join(O, "goal%d.json" % i)
        if os.path.exists(p):
            g = json.load(open(p))
            tr = g.get("track") or []
            goals[i] = {"goal": g["goal"], "result": g["result"], "seconds": g["seconds"],
                        "t0": tr[0][0] if tr else None, "t1": tr[-1][0] if tr else None}
    rep = {"title": a.title}
    t_end = errs[-1][0] if errs else 0
    if t_arm:
        rep["start_relocalize_s"] = settle_time(errs, t_arm, t_kid)
        g1 = goals.get(1, {}).get("t0") or t_kid or t_end
        rep["standing_after_start"] = stats(errs, g1 - 5, g1)
    for i, g in goals.items():
        if g.get("t0"):
            rep["driving_goal%d" % i] = stats(errs, g["t0"], g["t1"])
        rep["goal%d" % i] = {k: g[k] for k in ("goal", "result", "seconds")}
    if t_kid:
        rep["kidnap_recover_s"] = settle_time(errs, t_kid)
        k = [e for t, e, *_ in errs if t_kid <= t <= t_kid + 3 and e is not None]
        rep["kidnap_error_m"] = round(max(k), 2) if k else None
        g2 = goals.get(2, {}).get("t0") or t_end
        rep["standing_after_kidnap"] = stats(errs, g2 - 5, g2)
    log = os.path.join(O, "launch.log")
    if os.path.exists(log):
        rep["monitor"] = [ln.split("]: ", 1)[-1].strip() for ln in open(log, errors="replace")
                          if "loc_monitor" in ln and ("global match" in ln or "initial pose" in ln or "lost" in ln)][-6:]
    json.dump(rep, open(os.path.join(O, "report.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))

    # ---- plot: map + tracks, error over time
    img, res, origin = load_map(a.map)
    w, h = img.size
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(14, 5.6), dpi=110, gridspec_kw={"width_ratios": [1.15, 1]})
    ax.imshow(img, cmap="gray", extent=[origin[0], origin[0] + w * res, origin[1], origin[1] + h * res],
              origin="upper", vmin=0, vmax=255, interpolation="nearest")
    t0 = t_arm or (errs[0][0] if errs else 0)
    tt = [r for r in truth if r[0] >= t0 - 1]
    segs, cur = [], []
    for i, r in enumerate(tt):                          # break the truth line at the teleport
        if cur and math.hypot(r[1] - cur[-1][1], r[2] - cur[-1][2]) > 0.3:
            segs.append(cur)
            cur = []
        cur.append(r)
    segs.append(cur)
    for i, s in enumerate(segs):
        ax.plot([r[1] for r in s], [r[2] for r in s], color="#2a7fff", lw=2.2, label="truth" if i == 0 else None)
    est = [(t, p) for t, e, eh, q, tr, p in errs if p is not None and t >= t0]
    ax.plot([p[0] for t, p in est], [p[1] for t, p in est], ".", color="#ff8800", ms=2.2, label="ROS map pose")
    for i, s in enumerate(segs):
        ax.plot(s[0][1], s[0][2], "o", color="#2a7fff", ms=8)
        if i:
            ax.annotate("kidnapped here", (s[0][1], s[0][2]), xytext=(8, 8), textcoords="offset points", fontsize=8)
    ax.annotate("start (standing up)", (segs[0][0][1], segs[0][0][2]), xytext=(8, -12), textcoords="offset points",
                fontsize=8)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("%s on the saved map: truth vs estimate" % a.title, fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    xs = [t - t0 for t, e, *_ in errs if e is not None]
    bx.plot(xs, [100 * e for t, e, *_ in errs if e is not None], color="#cc3311", lw=1.3, label="position error (cm)")
    bx.set_yscale("log")
    bx.set_ylim(0.5, 600)
    bx.axhline(100 * GOOD_M, color="#999", ls=":", lw=1)
    bx.set_xlabel("s since armed")
    bx.set_ylabel("position error (cm, log)")
    qx = bx.twinx()
    qq = [(t - t0, q) for t, e, eh, q, *_ in errs if q is not None]
    qx.plot([v[0] for v in qq], [v[1] for v in qq], color="#33aa55", lw=1, alpha=0.8, label="scan fit (quality)")
    qx.set_ylim(0, 1.05)
    qx.set_ylabel("scan fit (share of beams within 10 cm)")
    for i, g in goals.items():
        if g.get("t0"):
            bx.axvspan(g["t0"] - t0, g["t1"] - t0, color="#ffcc66", alpha=0.25, lw=0)
            bx.text(g["t0"] - t0 + 1, 300, "goal %d" % i, fontsize=8)
    if t_kid:
        bx.axvline(t_kid - t0, color="#000", lw=1)
        bx.text(t_kid - t0 + 1, 150, "kidnap", fontsize=8)
    s1, s2 = rep.get("start_relocalize_s"), rep.get("kidnap_recover_s")
    bx.set_title("relocalized %s s after arming, %s s after the kidnap" % (
        "?" if s1 is None else "%.1f" % s1, "?" if s2 is None else "%.1f" % s2), fontsize=10)
    h1, l1 = bx.get_legend_handles_labels()
    h2, l2 = qx.get_legend_handles_labels()
    bx.legend(h1 + h2, l1 + l2, loc="lower right", fontsize=8)
    fig.tight_layout()
    png = a.png or os.path.join(O, "localize.png")
    fig.savefig(png)
    print("wrote", png)


if __name__ == "__main__":
    main()
