#!/usr/bin/env python3
"""Render a map_saver map (.yaml + .pgm) to a PNG, optionally with the sim room's true walls and a Nav2 run.

  python3 ros2/scripts/map_png.py ~/maps/room.yaml docs/img/ros2/map.png [--truth] [--nav run.json] [--title "..."]
--truth: overlay the simulator room (station/sim/sim.py Room, world = map frame when slam_toolbox starts at the
sim's start pose) as thin blue lines. --nav: goal_test.py JSON (first plan in green, the driven track in orange).
Needs matplotlib (python3-matplotlib, pulled in by ros-jazzy-desktop) and Pillow.
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402


def load_map(yaml_path):
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]
    return Image.open(img_path).convert("L"), res, origin


def seg_dist(px, py, s):
    xa, ya, xb, yb = s
    ex, ey = xb - xa, yb - ya
    L2 = ex * ex + ey * ey
    u = max(0.0, min(1.0, ((px - xa) * ex + (py - ya) * ey) / L2)) if L2 else 0.0
    return math.hypot(xa + u * ex - px, ya + u * ey - py)


def wall_error(img, res, origin, segs, occ=50):
    """(occupied cells, % within 0.1 m of a true wall, median distance m) for a map in the sim's world frame."""
    w, h = img.size
    px = img.load()
    ds = []
    for j in range(h):
        for i in range(w):
            if px[i, j] < occ:
                x = origin[0] + (i + 0.5) * res
                y = origin[1] + (h - j - 0.5) * res
                ds.append(min(seg_dist(x, y, s) for s in segs))
    if not ds:
        return None
    ds.sort()
    return len(ds), 100.0 * sum(1 for d in ds if d <= 0.1) / len(ds), ds[len(ds) // 2]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("png")
    ap.add_argument("--truth", action="store_true")
    ap.add_argument("--nav", default=None)
    ap.add_argument("--title", default="")
    a = ap.parse_args(argv)
    img, res, origin = load_map(a.yaml)
    w, h = img.size
    x0, y0 = origin[0], origin[1]
    ext = [x0, x0 + w * res, y0, y0 + h * res]
    fig, ax = plt.subplots(figsize=(8, 8 * h / max(w, 1) + 0.6), dpi=120)
    ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=255, interpolation="nearest")
    title = a.title
    if a.truth:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
        from station.sim.sim import Room
        segs = [tuple(v / 1000.0 for v in s) for s in Room().segs]
        for i, (xa, ya, xb, yb) in enumerate(segs):
            ax.plot([xa, xb], [ya, yb], color="#2a7fff", lw=0.8, alpha=0.8,
                    label="sim walls (truth)" if i == 0 else None)
        q = wall_error(img, res, origin, segs)
        if q:
            print("occupied cells: %d, %.0f %% within 0.10 m of a true wall, median %.3f m" % q)
            title = (title + "\n" if title else "") + "%d wall cells, %.0f %% within 10 cm of the true walls, " \
                "median %.1f cm" % (q[0], q[1], q[2] * 100)
    if a.nav:
        with open(a.nav) as f:
            run = json.load(f)
        plan = run.get("first_plan") or run.get("plan") or []
        if plan:
            ax.plot([p[0] for p in plan], [p[1] for p in plan], color="#22bb33", lw=2, label="Nav2 plan")
        tr = run.get("track") or []
        if tr:
            ax.plot([p[1] for p in tr], [p[2] for p in tr], color="#ff8800", lw=2, label="robot (map frame)")
            ax.plot(tr[0][1], tr[0][2], "o", color="#ff8800", ms=7)
        gx, gy, gyaw = run["goal"]
        ax.plot(gx, gy, "*", color="#dd0000", ms=16, label="goal (%s)" % run.get("result"))
        ax.arrow(gx, gy, 0.3 * math.cos(gyaw), 0.3 * math.sin(gyaw), color="#dd0000", width=0.02)
    if a.truth or a.nav:
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    if title:
        ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(a.png)
    print("wrote", a.png, "%dx%d cells at %.3f m" % (w, h, res))


if __name__ == "__main__":
    main()
