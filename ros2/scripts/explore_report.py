#!/usr/bin/env python3
"""Explore run report (sim tests): coverage vs the sim's true free area, time, distance, goals; map PNGs.

  python3 ros2/scripts/explore_report.py OUT_DIR [--maze] [--title "..."]
Reads OUT_DIR/maps/<name>.yaml + .explore.json (the explorer's auto-save), status.jsonl, snap_*.npz and, with
--maze, zones.json (the sim's maze walls). Writes OUT_DIR/summary.json, explore_final.png, explore_snap_<t>s.png.
Truth: the sim room's walls (station/sim/sim.py Room, + the maze), the free cells (> 5 cm from a wall) connected to
the start; world = the map frame (slam_toolbox starts at the sim's start pose). Coverage = those cells that the map
shows free / all of them; "drivable" coverage: the same for cells > 14 cm from a wall (where the robot fits).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "..", "sbr3_ros"))
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

from sbr3_ros import frontier as fr  # noqa: E402


def load_pgm_map(yaml_path):
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    img_path = os.path.join(os.path.dirname(yaml_path), meta["image"])
    with open(img_path, "rb") as f:
        data = f.read()
    parts, i = [], 0
    while len(parts) < 4:                      # P5, width, height, maxval (skip # comments)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    i += 1
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)[::-1, :]    # row 0 = origin y
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[img >= 250] = 0
    grid[img <= 50] = 100
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")][:2]
    return grid, float(meta["resolution"]), origin


def truth_segments(maze_zones):
    from station.sim.sim import Room
    segs = list(Room(empty=maze_zones is not None).segs)
    if maze_zones is not None:
        from station.bridge.zones import wall_segments
        segs += [tuple(float(v) for v in s[:4]) for s in wall_segments(maze_zones.get("items") or [])]
    return [tuple(v / 1000.0 for v in s) for s in segs]


def seg_dist_grid(xs, ys, segs):
    X, Y = np.meshgrid(xs, ys)
    best = np.full(X.shape, np.inf)
    for xa, ya, xb, yb in segs:
        ex, ey = xb - xa, yb - ya
        L2 = ex * ex + ey * ey
        u = np.clip(((X - xa) * ex + (Y - ya) * ey) / L2, 0, 1) if L2 else np.zeros(X.shape)
        best = np.minimum(best, np.hypot(xa + u * ex - X, ya + u * ey - Y))
    return best


def truth_free(segs, res=0.05, start=(0.0, 0.0), margin=0.05):
    xs_all = [v for s in segs for v in (s[0], s[2])]
    ys_all = [v for s in segs for v in (s[1], s[3])]
    x0, x1, y0, y1 = min(xs_all) - 0.2, max(xs_all) + 0.2, min(ys_all) - 0.2, max(ys_all) + 0.2
    xs = np.arange(x0 + res / 2, x1, res)
    ys = np.arange(y0 + res / 2, y1, res)
    d = seg_dist_grid(xs, ys, segs)
    lab, _ = ndimage.label(d > margin)
    sr, sc = int((start[1] - y0) / res), int((start[0] - x0) / res)
    free = lab == lab[sr, sc]
    return free, xs, ys


def sample(grid, res, origin, xs, ys):
    ci = np.floor((xs - origin[0]) / res).astype(int)
    ri = np.floor((ys - origin[1]) / res).astype(int)
    out = np.full((len(ys), len(xs)), -1, dtype=np.int16)
    okc = (ci >= 0) & (ci < grid.shape[1])
    okr = (ri >= 0) & (ri < grid.shape[0])
    out[np.ix_(okr, okc)] = grid[np.ix_(ri[okr], ci[okc])]
    return out


CMAP = ListedColormap(["#9a9a9a", "#f4f4f4", "#1a1a1a"])     # unknown, free, wall


def draw_map(ax, grid, res, origin):
    st = fr.classify(grid, fr.Params())
    disp = np.choose(st, [0, 1, 2])
    H, W = grid.shape
    ax.imshow(disp, cmap=CMAP, vmin=0, vmax=2, origin="lower", interpolation="nearest",
              extent=[origin[0], origin[0] + W * res, origin[1], origin[1] + H * res])


def draw_truth(ax, segs):
    for i, (xa, ya, xb, yb) in enumerate(segs):
        ax.plot([xa, xb], [ya, yb], color="#2a7fff", lw=0.9, alpha=0.75, label="true walls" if i == 0 else None)


def frame(ax, segs):
    xs = [v for s in segs for v in (s[0], s[2])]
    ys = [v for s in segs for v in (s[1], s[3])]
    ax.set_xlim(min(xs) - 0.3, max(xs) + 0.3)
    ax.set_ylim(min(ys) - 0.3, max(ys) + 0.3)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--maze", action="store_true")
    ap.add_argument("--title", default="")
    a = ap.parse_args(argv)
    out = a.out
    zones = None
    if a.maze:
        with open(os.path.join(out, "zones.json")) as f:
            zones = json.load(f)
    segs = truth_segments(zones)
    ymls = sorted(glob.glob(os.path.join(out, "maps", "*.yaml")))
    if not ymls:
        sys.exit("no saved map in %s/maps" % out)
    yml = ymls[-1]
    grid, res, origin = load_pgm_map(yml)
    with open(yml[:-5] + ".explore.json") as f:
        ex = json.load(f)
    status = []
    if os.path.exists(os.path.join(out, "status.jsonl")):
        with open(os.path.join(out, "status.jsonl")) as f:
            status = [json.loads(line) for line in f if line.strip()]
    free, xs, ys = truth_free(segs, res)
    m = sample(grid, res, origin, xs, ys)
    mapped = free & (m >= 0) & (m <= 25)
    cov = 100.0 * mapped.sum() / max(1, free.sum())
    # where the robot itself fits (> robot radius + 2 cm from walls, connected to the start): e.g. not the 35 cm
    # slots between the maze and the room walls, which the LD19 only sees at a glancing angle from their ends
    drv, _, _ = truth_free(segs, res, margin=0.14)
    cov_drv = 100.0 * (drv & (m >= 0) & (m <= 25)).sum() / max(1, drv.sum())
    wall_wrong = int(((m >= 65) & free & (seg_dist_grid(xs, ys, segs) > 0.15)).sum())
    st = ex["status"]
    goals = ex["goals"]
    states = [s["state"] for s in status]
    final = status[-1] if status else st
    summary = {"world": "maze" if a.maze else "room", "map": os.path.basename(yml)[:-5],
               "coverage_pct": round(cov, 1), "true_free_m2": round(free.sum() * res * res, 2),
               "coverage_drivable_pct": round(cov_drv, 1), "drivable_m2": round(drv.sum() * res * res, 2),
               "mapped_free_m2": round(mapped.sum() * res * res, 2), "map_free_m2": st["area_m2"],
               "false_wall_cells": wall_wrong, "elapsed_s": st["elapsed_s"], "dist_m": st["dist_m"],
               "goals_ok": st["goals_ok"], "goals_failed": st["goals_failed"], "goals": len(goals),
               "goal_results": {r: sum(1 for g in goals if g["result"].split(":")[0] == r)
                                for r in sorted(set(g["result"].split(":")[0] for g in goals))},
               "end_state": final.get("state"), "end_reason": st["reason"],
               "plans": ex.get("plans"), "plan_ms_max": ex.get("plan_ms_max"),
               "blacklisted": len(ex.get("blacklist") or []),
               "states_seen": sorted(set(states))}
    # loops: the same viewpoint (within 0.3 m) chosen again and again
    vps = [g["vp"] for g in goals]
    rep = max([sum(1 for w in vps if math.hypot(w[0] - v[0], w[1] - v[1]) < 0.3) for v in vps] or [0])
    summary["max_repeats_same_spot"] = rep
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))

    # final map
    fig, ax = plt.subplots(figsize=(8, 6.4), dpi=110)
    draw_map(ax, grid, res, origin)
    draw_truth(ax, segs)
    tr = np.array(ex["track"])
    if len(tr):
        ax.plot(tr[:, 0], tr[:, 1], color="#ff8800", lw=1.6, label="robot path (%.1f m)" % st["dist_m"])
        ax.plot(tr[0, 0], tr[0, 1], "s", color="#ff8800", ms=8, label="start")
    for i, g in enumerate(goals):
        r = g["result"].split(":")[0]
        c = {"reached": "#22aa33", "seen": "#22aa33", "switched": "#ccaa00"}.get(r, "#dd2222")
        ax.plot(g["vp"][0], g["vp"][1], "o", mfc="none", mec=c, mew=1.6, ms=8,
                label={"#22aa33": "goal (reached / frontier seen)", "#ccaa00": "goal (switched)",
                       "#dd2222": "goal (failed)"}[c])
    h, l_ = ax.get_legend_handles_labels()
    seen = {}
    for hh, ll in zip(h, l_):
        seen.setdefault(ll, hh)
    ax.legend(seen.values(), seen.keys(), loc="upper right", fontsize=7, framealpha=0.85)
    frame(ax, segs)
    title = a.title or ("Explore, sim %s" % summary["world"])
    ax.set_title("%s: %.0f %% of the true free area mapped, %.0f s, %.1f m, %d goals (%d failed)" % (
        title, cov, st["elapsed_s"], st["dist_m"], len(goals), st["goals_failed"]), fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "explore_final.png"))
    plt.close(fig)

    # snapshots, with the frontiers the explorer would see
    for path in sorted(glob.glob(os.path.join(out, "snap_*s.npz"))):
        d = np.load(path)
        g, r, o = d["grid"], float(d["res"]), list(d["origin"])
        trk = d["track"]
        fig, ax = plt.subplots(figsize=(6.4, 5.2), dpi=100)
        draw_map(ax, g, r, o)
        draw_truth(ax, segs)
        if len(trk):
            ax.plot(trk[:, 0], trk[:, 1], color="#ff8800", lw=1.4, label="robot path")
            ax.plot(trk[-1, 0], trk[-1, 1], "o", color="#ff8800", ms=7)
            fs, _ = fr.find_frontiers(g, r, o, tuple(trk[-1]))
            stf = fr.classify(g, fr.Params())
            fm = fr.frontier_mask(stf)
            yy, xx = np.nonzero(fm)
            ax.plot(o[0] + (xx + 0.5) * r, o[1] + (yy + 0.5) * r, ",", color="#cc33cc")
            for k, f in enumerate(fs):
                ax.plot(*f.viewpoint, "^", color="#cc33cc", ms=6 if k else 9, mec="k" if k == 0 else "none",
                        label=("frontier viewpoints (best: big)" if k == 0 else None))
        t = d["target"]
        if np.isfinite(t).all():
            ax.plot(t[0], t[1], "*", color="#dd0000", ms=14, label="current goal")
        ax.legend(loc="upper right", fontsize=7, framealpha=0.85)
        frame(ax, segs)
        ax.set_title("%s at %.0f s: frontiers (magenta) and the goal" % (title, float(d["elapsed"])), fontsize=9)
        fig.tight_layout()
        fig.savefig(path[:-4].replace("snap_", "explore_snap_") + ".png")
        plt.close(fig)


if __name__ == "__main__":
    main()
