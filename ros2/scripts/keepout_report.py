#!/usr/bin/env python3
"""Keep-out test report (sim_e2e_keepout.sh): the three runs' plans and true paths over the map + keep-out mask.

  python3 ros2/scripts/keepout_report.py OUT_DIR --map MAP.yaml
Checks that no true path point entered a virtual wall's keep-out band while it was there.
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
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sbr3_ros"))
from map_png import load_map  # noqa: E402
from sbr3_ros import keepout as ko  # noqa: E402

WALL1 = {"id": "w1", "kind": "wall", "pts": [[1500, -1600], [1500, -200]]}
WALL2 = {"id": "w2", "kind": "wall", "pts": [[1500, -250], [1500, 750]]}
WALL3 = {"id": "w3", "kind": "wall", "pts": [[1500, 1050], [1500, 2100]]}
RUNS = [("1: wall, goes around", [WALL1], "#2a7fff"), ("2: closed, unreachable", [WALL1, WALL2, WALL3], "#dd2222"),
        ("3: walls deleted", [], "#22aa44")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--map", required=True)
    a = ap.parse_args()
    with open(os.path.join(a.out, "truth.csv")) as f:
        tr = [(float(r["t"]), float(r["X"]) / 1000, float(r["Y"]) / 1000) for r in csv.DictReader(f)]
    img, res, origin = load_map(a.map)
    w, h = img.size
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.2), dpi=105)
    rep = []
    for i, ((label, zones, col), ax) in enumerate(zip(RUNS, axs), 1):
        p = os.path.join(a.out, "goal%d.json" % i)
        g = json.load(open(p)) if os.path.exists(p) else {"result": "missing", "goal": [0, 0, 0], "track": []}
        ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
        ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=255, interpolation="nearest")
        if zones:
            mask, _ = ko.build_mask(zones, (0.0, 0.0, 0.0), (origin[0], origin[1]), res, w, h, 0.15)
            rgba = np.zeros((h, w, 4))
            rgba[..., 0], rgba[..., 1], rgba[..., 2] = 1.0, 0.1, 0.1
            rgba[..., 3] = np.where(mask >= 100, 0.45, mask / 100.0 * 0.35)
            ax.imshow(rgba, extent=ext, origin="lower", interpolation="nearest")
            for z in zones:
                ax.plot([q[0] / 1000 for q in z["pts"]], [q[1] / 1000 for q in z["pts"]], color="#cc0000", lw=2.5)
        tk = g.get("track") or []
        t0, t1 = (tk[0][0], tk[-1][0]) if tk else (0, 0)
        path = [(x, y) for t, x, y in tr if t0 <= t <= t1]
        inside = 0
        if zones and path:
            for x, y in path:
                for z in zones:
                    s, _ = ko.segments(z, (0.0, 0.0, 0.0))
                    if min(ko._seg_dist(np.array(x), np.array(y), q) for q in s) < 0.12:
                        inside += 1
        plan = g.get("first_plan") or []
        if plan:
            ax.plot([q[0] for q in plan], [q[1] for q in plan], "--", color="#22bb33", lw=1.5, label="Nav2 plan")
        if path:
            ax.plot([q[0] for q in path], [q[1] for q in path], color=col, lw=2.2, label="robot (truth)")
        gx, gy, _ = g["goal"]
        ax.plot(gx, gy, "*", color="#dd0000", ms=15, label="goal")
        ln = float(np.hypot(*np.diff(np.array(path), axis=0).T).sum()) if len(path) > 1 else 0.0
        ax.set_title("%s\n%s after %.0f s, path %.1f m, %d points on a wall" % (
            label, g["result"], g.get("seconds", 0), ln, inside), fontsize=9)
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=7)
        rep.append({"run": label, "result": g["result"], "seconds": g.get("seconds"), "path_m": round(float(ln), 2),
                    "points_on_wall": inside, "plan_points": len(plan)})
    fig.tight_layout()
    png = os.path.join(a.out, "keepout.png")
    fig.savefig(png)
    json.dump(rep, open(os.path.join(a.out, "report.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))
    print("wrote", png)


if __name__ == "__main__":
    main()
