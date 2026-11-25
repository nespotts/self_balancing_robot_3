"""Global relocalisation evaluation (issue #105): kidnap the robot anywhere on a mapped room, at any heading, and see
whether reloc.find() finds it; CPU time on this PC with a Pi Zero 2W estimate (x --pi-scale).

  python -m station.sim.reloc_eval                         the sim room (5 x 3.5 m) and the 10 x 10 m bench room
  python -m station.sim.reloc_eval --worlds big --trials 40 --seeds 1 2 3
  python -m station.sim.reloc_eval --worlds sym            a symmetric room: must say "ambiguous", never a wrong pose

Maps are built from noisy ray-cast turns (450 points, 8 mm noise) on a lattice of poses, as a tour would; a kidnap
scan is taken at a random free pose / heading (clearance >= 250 mm) and placed in the map with a wrong pose. Found =
ok and within 100 mm / 3 deg of the truth; wrong = ok but farther (a false accept: must be 0).
"""
import argparse
import math
import random
import time

from station.bridge import lidarmatch as lm
from station.bridge import reloc
from station.bridge.occgrid import Grid
from station.sim.sim import Room, _box

WORLDS = ("room", "room5", "big", "sym")


class Segs:
    def __init__(self, segs):
        self.segs = list(segs)

    raycast = Room.raycast
    clearance = Room.clearance


def world(name):
    """(room, (xmin, ymin, xmax, ymax) of the floor)."""
    if name == "room":
        return Room(), (-1000, -1500, 4000, 2000)
    if name == "room5":                     # 5 x 5 m, a few boxes, a cabinet, a partition
        s = _box(0, 0, 5000, 5000) + _box(-1200, 1300, 700, 500) + _box(1300, -900, 450, 450) \
            + _box(800, 1700, 300, 900) + [(-2500, -1500, -1700, -2500), (300, -2500, 300, -1600)]
        return Segs(s), (-2500, -2500, 2500, 2500)
    if name == "big":                       # loop_bench's 10 x 10 m room
        s = _box(4000, 0, 10000, 10000) + _box(4500, 2000, 600, 600) + _box(5500, -500, 500, 500) \
            + _box(2000, -1500, 800, 300) + [(6000, 5000, 6000, 4300), (9000, -1000, 8300, -1000),
                                             (-1000, 2000, -400, 2000)]
        return Segs(s), (-1000, -5000, 9000, 5000)
    if name == "sym":                       # an empty 4 x 3 m rectangle: every pose has a 180 deg twin
        return Segs(_box(0, 0, 4000, 3000)), (-2000, -1500, 2000, 1500)
    raise ValueError(name)


def turn(room, x, y, h, rng, n=450, noise=8.0, max_d=8000.0):
    pts = []
    for k in range(n):
        a = math.radians(h) + 2 * math.pi * k / n
        d = room.raycast(x, y, a, max_d)
        if d is not None:
            d += rng.gauss(0, noise)
            pts.append((x + d * math.cos(a), y + d * math.sin(a)))
    return pts


def place(pts, true, guess):
    """Points seen at the true pose, put in the map with the guessed pose (what a wrong pose does)."""
    tx, ty, tt = true
    gx, gy, gt = guess
    c, s = math.cos(math.radians(-tt)), math.sin(math.radians(-tt))
    rel = [(c * (x - tx) - s * (y - ty), s * (x - tx) + c * (y - ty)) for x, y in pts]
    c, s = math.cos(math.radians(gt)), math.sin(math.radians(gt))
    return [(c * x - s * y + gx, s * x + c * y + gy) for x, y in rel]


def free_pose(room, box, rng, clear=250.0):
    while True:
        x, y = rng.uniform(box[0], box[2]), rng.uniform(box[1], box[3])
        if room.clearance(x, y) >= clear and _inside(room, x, y):
            return x, y, rng.uniform(-180.0, 180.0)


def _inside(room, x, y):
    """Inside the closed outer walls (a ray in every direction hits something within 12 m) and not in a box."""
    hits = [room.raycast(x, y, a, 12000.0) for a in (0.3, 1.9, 3.5, 5.1)]
    if any(h is None for h in hits):
        return False
    # in a box: a ray out of it crosses an odd number of walls ... a cheap stand-in: the box interiors are small, so
    # look for a wall within 700 mm on all four sides
    near = [room.raycast(x, y, a, 700.0) for a in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)]
    return not all(h is not None for h in near)


def build_map(room, box, rng, step=700.0, turns=2):
    """A map from noisy turns on a lattice of free poses (as a tour that covered the room)."""
    g = Grid()
    x = box[0] + step / 2
    while x < box[2]:
        y = box[1] + step / 2
        while y < box[3]:
            if room.clearance(x, y) >= 200 and _inside(room, x, y):
                for _ in range(turns):
                    g.integrate_scan(x, y, [turn(room, x, y, rng.uniform(0, 360), rng)], margin_mm=25)
                g.clear_disc(x, y, 50.0)
            y += step
        x += step
    return g


def kidnap(fz, pyr, room, box, rng, trials, quiet=True):
    rows = []
    for i in range(trials):
        true = free_pose(room, box, rng)
        guess = (rng.uniform(box[0], box[2]), rng.uniform(box[1], box[3]), rng.uniform(-180, 180))
        pts = place(turn(room, true[0], true[1], true[2], rng), true, guess)
        t0 = time.perf_counter()
        r = reloc.find(fz, pyr, [pts], guess[0], guess[1])
        cpu = time.perf_counter() - t0
        e = math.hypot(r["x"] - true[0], r["y"] - true[1])
        eh = abs((guess[2] + r["dth"] - true[2] + 180.0) % 360.0 - 180.0)
        good = e < 100.0 and eh < 3.0
        row = {"ok": r["ok"], "why": r["why"], "good": good, "err_mm": round(e), "err_deg": round(eh, 2),
               "q": r["q"], "score": r["score"], "margin": r["margin"], "nodes": r["nodes"], "rots": r["rots"],
               "cpu_s": cpu, "ms": r["ms"]}
        rows.append(row)
        if not quiet:
            print("  ", i, row, flush=True)
    return rows


def evaluate(name, seed, trials, pi_scale=9.0, quiet=True):
    rng = random.Random(seed)
    room, box = world(name)
    t0 = time.perf_counter()
    g = build_map(room, box, rng)
    build_s = time.perf_counter() - t0
    f = lm.Field()
    f.sync(g)
    fz = reloc.Frozen(f)
    t0 = time.perf_counter()
    pyr = reloc.Pyramid(fz)
    pyr_s = time.perf_counter() - t0
    rows = kidnap(fz, pyr, room, box, rng, trials, quiet)
    n = len(rows)
    found = sum(1 for r in rows if r["ok"] and r["good"])
    wrong = sum(1 for r in rows if r["ok"] and not r["good"])
    amb = sum(1 for r in rows if r["why"] == "ambiguous")
    cpu = sorted(r["cpu_s"] for r in rows)
    out = {"world": name, "seed": seed, "trials": n, "found": found, "wrong": wrong, "ambiguous": amb,
           "other": n - found - wrong - amb, "found_pct": round(100.0 * found / n), "walls": f.walls,
           "map_m": [round((pyr.Wc * pyr.R) / 1000.0, 1), round((pyr.Hc * pyr.R) / 1000.0, 1)],
           "pyramid_ms": round(1000 * pyr_s, 1), "pyramid_kb": round(pyr.mem_bytes() / 1024),
           "cpu_ms_median": round(1000 * cpu[n // 2], 1), "cpu_ms_p90": round(1000 * cpu[int(0.9 * (n - 1))], 1),
           "cpu_ms_max": round(1000 * cpu[-1], 1),
           "nodes_median": sorted(r["nodes"] for r in rows)[n // 2], "rots": rows[0]["rots"],
           "pi_s_median": round(pi_scale * cpu[n // 2], 2), "pi_s_max": round(pi_scale * cpu[-1], 2),
           "pi_pyramid_s": round(pi_scale * pyr_s, 2), "map_build_s": round(build_s, 1),
           "misses": [(r["why"], r["err_mm"], r["err_deg"], r["q"], r["margin"]) for r in rows if not (r["ok"] and r["good"])]}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds", nargs="+", default=["room", "big"], choices=WORLDS)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--pi-scale", type=float, default=9.0, help="Pi Zero 2W / this PC, per core")
    ap.add_argument("-v", action="store_true", help="every trial")
    a = ap.parse_args(argv)
    for w in a.worlds:
        for s in a.seeds:
            print(evaluate(w, s, a.trials, a.pi_scale, quiet=not a.v), flush=True)


if __name__ == "__main__":
    main()
