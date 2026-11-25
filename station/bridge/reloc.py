"""Global relocalisation (issue #105): where on the whole map is the robot, at any heading?

LidarMatch (lidarmatch.py) runs this in its worker thread when the pose is lost for a while, after a Teensy reboot /
pose reset while a map exists, after a map is loaded, by hand ("Find myself"), and in a small window around the pose a
user gave with "Set position".

  1. Pyramid (cached while the map does not change): the matcher's likelihood field (25 mm cells) max-pooled into
     COARSE_F x 25 = 100 mm cells, cropped to the mapped tiles, padded by the scan's reach, then levels 1 .. LEVELS:
     level h holds the max over the 2^h x 2^h block of coarse cells starting at each cell (Cartographer's precomputation
     grids). Plus a "seen free" mask with the same levels: a block where no cell was ever seen free cannot hold the
     robot (it never stood inside a wall or out in the unknown).
  2. Branch and bound (Cartographer's fast correlative scan matcher, Hess et al. 2016): for every heading in steps of
     ~1 coarse cell at the scan's typical range, the scan's points are rotated once; a node = (heading, a block of
     2^h x 2^h robot cells) has an upper bound: the sum of level h at the points. The top-level nodes are sorted and
     searched depth first, best child first; a node whose bound is below max(min score, PRUNE_F x the best leaf so far)
     is cut. All leaves within PRUNE_F of the best are kept: the ambiguity check needs the best pose elsewhere too.
  3. Candidates: the leaves clustered (NEAR_MM / NEAR_DEG), the best K_REFINE refined with lidarmatch.match() on the
     fine field (pose to one 25 mm cell / 0.25 deg; score and inliers as the normal matcher computes them).
  4. Verdict: the best must fit well (ACCEPT: score, inliers, share of points on mapped cells) and beat the best
     candidate NOT near it by ACCEPT["margin"]; else "ambiguous" (a symmetric room: two poses fit) or "no fit".
     LidarMatch then puts the robot there and confirms the pose over 2 more scans before it maps again.

Stdlib only. find() runs in a worker thread and yields (Yielder): the long C calls (max pooling) are cut into chunks
that hold the GIL ~0.3 ms on a PC (~3 ms on the Pi Zero 2W) each, so the bridge's event loop does not stall (#102).
"""
import math
import operator
import time

from . import lidarmatch as lm

COARSE_F = 4               # coarse cell = 4 x 4 field cells (100 mm)
LEVELS = 5                 # top level blocks: 2^5 = 32 coarse cells (3.2 m)
R_MAX_MM = 6000.0          # scan points farther than this are left out (and the padding is sized for it)
FREE_BYTE = 121            # a grid byte below this was seen free at least once
POINTS_COARSE = 80         # points for the branch and bound
POINTS_FINE = 150          # ... for the refinement (the normal matcher's count)
STEP_MIN_DEG, STEP_MAX_DEG = 1.0, 3.0   # heading step: ~1 coarse cell at the 80th percentile range, clamped
MIN_COARSE = 0.3           # a leaf must reach this mean (0..1) on the coarse field
PRUNE_F = 0.85             # nodes within this share of the best leaf are searched (ambiguity check)
NEAR_MM, NEAR_DEG = 300.0, 10.0   # leaves closer than this are the same candidate
K_REFINE = 5               # candidates refined on the fine field
REFINE_MM, REFINE_DEG = 100.0, 2.0
MAX_NODES = 600000         # node budget (a search that would take longer gives up: "too slow")
ACCEPT = {"score": 0.55, "inliers": 0.55, "known": 0.5, "margin": 0.15}
CHUNK = 2048               # bytes per max-pooling call (GIL hold)
SLICE_S, PAUSE_S = 0.003, 0.0015   # worker: after this much work, sleep this long (the loop gets the GIL)


class Cancelled(Exception):
    pass


class Yielder:
    """Called often from the worker; every SLICE_S it sleeps PAUSE_S (the event loop takes the GIL) and checks cancel."""

    def __init__(self, slice_s=None, pause_s=None, cancel=None):
        self.slice_s = SLICE_S if slice_s is None else slice_s
        self.pause_s = PAUSE_S if pause_s is None else pause_s
        self.cancel = cancel
        self.clock = time.perf_counter
        self.end = self.clock() + self.slice_s
        self.pauses = 0

    def __call__(self):
        if self.clock() >= self.end:
            if self.cancel is not None and self.cancel():
                raise Cancelled()
            time.sleep(self.pause_s)
            self.pauses += 1
            self.end = self.clock() + self.slice_s


def _nothing():
    pass


class _G:
    """The grid parts match() reads, frozen."""
    __slots__ = ("res", "x0", "y0", "W", "H", "T", "cells")

    def __init__(self, g):
        self.res, self.x0, self.y0, self.W, self.H, self.T = g.res, g.x0, g.y0, g.W, g.H, g.T
        self.cells = bytes(g.cells)


class Frozen:
    """A copy of a lidarmatch.Field (and its grid) for a worker thread: the loop may rebuild the real one meanwhile.
    Copying is a memcpy (~1 ms on the Pi for a 20 x 20 m grid)."""

    def __init__(self, field):
        self.grid = _G(field.grid)
        self.f = bytes(field.f)
        self.Wp, self.Hp, self.walls = field.Wp, field.Hp, field.walls
        bb = field.grid.bbox_tiles()
        self.bbox = bb                   # touched tiles (tx0, ty0, tx1, ty1), grid tile indices


def _row(arr, w, h, y, x_lo, x_hi, lut=None):
    """Bytes [x_lo, x_hi) of row y of a w x h array, zeros outside it, optionally through lut."""
    n = x_hi - x_lo
    if y < 0 or y >= h:
        return bytes(n)
    a, b = max(0, x_lo), min(w, x_hi)
    if b <= a:
        return bytes(n)
    s = bytes(arr[y * w + a:y * w + b])
    if lut is not None:
        s = s.translate(lut)
    return bytes(a - x_lo) + s + bytes(x_hi - b)


def _pool(rows, F):
    """F rows of F*n bytes -> n bytes: the max of each F x F block."""
    v = bytes(map(max, *rows)) if len(rows) > 1 else rows[0]
    return bytes(map(max, *[v[k::F] for k in range(F)])) if F > 1 else v


def _level(a, s, Wq, lo, hi_i, hi_j, y):
    """Next pyramid level: out[i] = max of a over the 2s x 2s block starting at i (a is the s-block level). Only
    blocks that reach the data, (I, J) in (lo - 2s, hi_i) x (lo - 2s, hi_j), can be non-zero: the padding is skipped."""
    n = len(a)
    ext = a + bytes(s * Wq + s)
    out = bytearray(n)
    sw = s * Wq
    i0 = max(0, lo - 2 * s + 1)
    w = hi_i - i0
    for J in range(max(0, lo - 2 * s + 1), hi_j):
        i = J * Wq + i0
        j = i + w
        out[i:j] = bytes(map(max, ext[i:j], ext[i + s:j + s], ext[i + sw:j + sw], ext[i + sw + s:j + sw + s]))
        y()
    return bytes(out)


class Pyramid:
    """Max-pooled likelihood field + seen-free mask, LEVELS + 1 levels, COARSE_F field cells per coarse cell.
    Padded coarse cell (I, J) is index J * Wq + I; its lower left corner is at world (x0 + I * R, y0 + J * R)."""

    def __init__(self, fz, y=_nothing, levels=LEVELS, F=COARSE_F, reach_mm=R_MAX_MM):
        t0 = time.perf_counter()
        g = fz.grid
        PADF = lm.PAD
        res = g.res
        T = g.T
        bb = fz.bbox or (0, 0, g.W // T - 1, g.H // T - 1)
        # the touched tiles, in padded field cells (+ the kernel's reach)
        px_lo, py_lo = bb[0] * T + PADF - lm.KERNEL_R, bb[1] * T + PADF - lm.KERNEL_R
        px_hi, py_hi = (bb[2] + 1) * T + PADF + lm.KERNEL_R, (bb[3] + 1) * T + PADF + lm.KERNEL_R
        Wc, Hc = -(-(px_hi - px_lo) // F), -(-(py_hi - py_lo) // F)
        R = res * F
        P = int(math.ceil(reach_mm / R)) + (1 << levels) + 3
        Wq, Hq = Wc + 2 * P, Hc + 2 * P
        self.R, self.P, self.Wq, self.Hq, self.Wc, self.Hc, self.L, self.F = R, P, Wq, Hq, Wc, Hc, levels, F
        self.x0 = g.x0 - PADF * res + (px_lo - P * F) * res
        self.y0 = g.y0 - PADF * res + (py_lo - P * F) * res
        c0, fr0 = bytearray(Wq * Hq), bytearray(Wq * Hq)
        free_lut = bytes(1 if v < FREE_BYTE else 0 for v in range(256))
        f, Wp, Hp, cells, W, H = fz.f, fz.Wp, fz.Hp, g.cells, g.W, g.H
        n = Wc * F
        lo_i, lo_j, hi_i, hi_j = Wq, Hq, -1, -1
        for J in range(Hc):
            o = (J + P) * Wq + P
            c0[o:o + Wc] = _pool([_row(f, Wp, Hp, py_lo + J * F + r, px_lo, px_lo + n) for r in range(F)], F)
            fr = _pool([_row(cells, W, H, py_lo - PADF + J * F + r, px_lo - PADF, px_lo - PADF + n, free_lut)
                        for r in range(F)], F)
            fr0[o:o + Wc] = fr
            a = fr.find(1)
            if a >= 0:
                lo_j, hi_j = min(lo_j, J + P), max(hi_j, J + P)
                lo_i, hi_i = min(lo_i, a + P), max(hi_i, fr.rfind(1) + P)
            y()
        self.free_box = (lo_i, lo_j, hi_i, hi_j) if hi_i >= 0 else None   # padded coarse cells, inclusive
        self.f, self.free = [bytes(c0)], [bytes(fr0)]
        for h in range(1, levels + 1):
            s = 1 << (h - 1)
            self.f.append(_level(self.f[-1], s, Wq, P, P + Wc, P + Hc, y))
            self.free.append(_level(self.free[-1], s, Wq, P, P + Wc, P + Hc, y))
        self.walls = fz.walls
        self.ms = (time.perf_counter() - t0) * 1000.0

    def cell(self, x, y):
        return int(math.floor((x - self.x0) / self.R)), int(math.floor((y - self.y0) / self.R))

    def centre(self, I, J):
        return self.x0 + (I + 0.5) * self.R, self.y0 + (J + 0.5) * self.R

    def mem_bytes(self):
        return sum(len(b) for b in self.f + self.free)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def bnb(pyr, rel, rots, box, min_score=MIN_COARSE, prune_f=PRUNE_F, max_nodes=MAX_NODES, y=_nothing):
    """Branch and bound over rotations rots (rad, added to the points' heading) x robot cells box = (I0, J0, I1, J1)
    (padded coarse cells, inclusive). rel: points (mm) relative to the robot. Returns (leaves [(sum, k, I, J)] within
    prune_f of the best, nodes scored, complete)."""
    Wq, R, L = pyr.Wq, pyr.R, pyr.L
    n = len(rel)
    inv = 1.0 / R
    mvs = [memoryview(b) for b in pyr.f]
    free = pyr.free
    S = 1 << L
    I0, J0, I1, J1 = box
    getters = []
    top = []
    big = 1 << 16                    # int() of a positive number is its floor: floor(0.5 + x) = int(x + B) - big
    B = big + 0.5
    shift = big + big * Wq
    rc = [(u * inv, v * inv) for u, v in rel]
    try:
        mvL, frL = mvs[L], free[L]
        for k, th in enumerate(rots):
            c, s = math.cos(th), math.sin(th)
            offs = [int(c * u - s * v + B) + Wq * int(s * u + c * v + B) for u, v in rc]
            m0 = min(offs)
            om = m0 - shift
            g = operator.itemgetter(*[o - m0 for o in offs])
            getters.append((g, om))
            for J in range(J0, J1 + 1, S):
                b0 = J * Wq
                for I in range(I0, I1 + 1, S):
                    b = b0 + I
                    if frL[b]:
                        top.append((sum(g(mvL[b + om:])), k, I, J))
            y()
        nodes = len(top)
        # sort the top level by bound, best first (ints sort fast: the GIL is held through the sort)
        keys = sorted((t[0] << 22) | i for i, t in enumerate(top))
        keys.reverse()
        y()
        thr_min = min_score * 255.0 * n
        best = 0
        thr = thr_min
        leaves = []
        mask = (1 << 22) - 1
        for key in keys:
            sc, k, I, J = top[key & mask]
            if sc < thr:
                break
            g, om = getters[k]
            stack = [(sc, I, J, L)]
            while stack:
                sc, I, J, h = stack.pop()
                if sc < thr:
                    continue
                if h == 0:
                    leaves.append((sc, k, I, J))
                    if sc > best:
                        best = sc
                        thr = max(thr_min, prune_f * best)
                    continue
                h -= 1
                s = 1 << h
                mv, fr = mvs[h], free[h]
                ch = []
                for I2, J2 in ((I, J), (I + s, J), (I, J + s), (I + s, J + s)):
                    if I2 > I1 or J2 > J1:
                        continue
                    b = J2 * Wq + I2
                    if not fr[b]:
                        continue
                    v = sum(g(mv[b + om:]))
                    nodes += 1
                    if v >= thr:
                        ch.append((v, I2, J2, h))
                if ch:
                    ch.sort()
                    stack.extend(ch)
                if nodes > max_nodes:
                    return [l for l in leaves if l[0] >= thr], nodes, False
                y()
        return [l for l in leaves if l[0] >= thr], nodes, True
    finally:
        for m in mvs:
            m.release()


def rotations(step_deg, centre=0.0, half_deg=None):
    """Headings to try (rad): the whole circle, or centre +- half_deg."""
    if half_deg is None or half_deg >= 180.0:
        n = int(math.ceil(360.0 / step_deg))
        return [2 * math.pi * k / n for k in range(n)]
    n = int(math.ceil(half_deg / step_deg))
    return [math.radians(centre + step_deg * k) for k in range(-n, n + 1)]


def heading_step(rel, R):
    """~1 coarse cell at the 80th percentile range, clamped to STEP_MIN_DEG .. STEP_MAX_DEG."""
    d = sorted(math.hypot(u, v) for u, v in rel)
    r = d[int(0.8 * (len(d) - 1))] if d else 1000.0
    return max(STEP_MIN_DEG, min(STEP_MAX_DEG, math.degrees(R / max(r, 1.0))))


def refine(fz, rel, x, y, th, min_curv=0.05):
    """One candidate (robot at (x, y), points rotated by th rad) refined on the fine field with lidarmatch.match()."""
    c, s = math.cos(th), math.sin(th)
    pts = [(x + c * u - s * v, y + s * u + c * v) for u, v in rel]
    r = lm.match(fz, pts, x, y, window_mm=REFINE_MM, window_deg=REFINE_DEG, min_score=0.0, min_inliers=0.0,
                 min_curv=min_curv, min_margin=0.0)
    r["x"], r["y"], r["th"] = x + r["dx"], y + r["dy"], th + math.radians(r["dth"])
    r["q"] = r["score"] * r["known"] / max(1, r["n"])      # mean fit over all points (unmapped ones count 0)
    return r


def _near(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) < NEAR_MM and abs(_wrap(a[2] - b[2])) < math.radians(NEAR_DEG)


def find(fz, pyr, runs, cx, cy, around=None, min_curv=0.05, accept=None, y=_nothing, max_nodes=MAX_NODES):
    """Where is the robot? runs: one turn's point runs (map frame, current correction), (cx, cy) the robot's map
    position they were taken at. around (Set position): (x, y, half_box_mm, half_deg): search only there, headings
    within +- half_deg of the points' current one. Returns a dict:
      ok, why ("" | "ambiguous" | "no fit" | "map too small" | "too few points" | "too slow" | "outside the map"),
      x, y, dth (deg: the rotation to add to the robot's heading about its own position), score, inliers, known, n, q
      (fit of all points), margin (vs the best candidate elsewhere), second (its q), cands (refined: [x, y, dth, q]),
      nodes, rots, step_deg, ms."""
    t0 = time.perf_counter()
    A = dict(ACCEPT, **(accept or {}))
    out = {"ok": False, "why": "", "x": cx, "y": cy, "dth": 0.0, "score": 0.0, "inliers": 0, "known": 0, "n": 0,
           "q": 0.0, "margin": 0.0, "second": 0.0, "cands": [], "nodes": 0, "rots": 0, "step_deg": 0.0}

    def done(why=""):
        out["why"] = why
        out["ms"] = round((time.perf_counter() - t0) * 1000.0, 1)
        return out
    if pyr.free_box is None or pyr.walls < lm.MIN_WALL_CELLS:
        return done("map too small")
    rel = [(x - cx, yy - cy) for x, yy in lm.thin_runs(runs, POINTS_FINE)]
    rel = [p for p in rel if p[0] * p[0] + p[1] * p[1] < R_MAX_MM * R_MAX_MM]
    if len(rel) < lm.MIN_POINTS:
        return done("too few points")
    stride = max(1, int(round(len(rel) / float(POINTS_COARSE))))
    rel_c = rel[::stride]
    y()
    step = heading_step(rel_c, pyr.R)
    box = pyr.free_box
    if around is None:
        rots = rotations(step)
    else:
        ax, ay, half_mm, half_deg = around
        rots = rotations(step, 0.0, half_deg)
        i0, j0 = pyr.cell(ax - half_mm, ay - half_mm)
        i1, j1 = pyr.cell(ax + half_mm, ay + half_mm)
        box = (max(box[0], i0), max(box[1], j0), min(box[2], i1), min(box[3], j1))
        if box[0] > box[2] or box[1] > box[3]:
            return done("outside the map")
    out["rots"], out["step_deg"] = len(rots), round(step, 2)
    leaves, nodes, complete = bnb(pyr, rel_c, rots, box, max_nodes=max_nodes, y=y)
    out["nodes"] = nodes
    if not complete:
        return done("too slow")
    if not leaves:
        return done("no fit")
    # candidates: the best leaf, then the best one not near any kept one
    leaves.sort(reverse=True)
    cands = []
    for sc, k, I, J in leaves:
        x, yy = pyr.centre(I, J)
        p = (x, yy, rots[k])
        if any(_near(p, c[1:]) for c in cands):
            continue
        cands.append((sc, x, yy, rots[k]))
        if len(cands) >= K_REFINE:
            break
    refs = []
    for sc, x, yy, th in cands:
        refs.append(refine(fz, rel, x, yy, th, min_curv))
        y()
    best = max(refs, key=lambda r: r["q"])
    bp = (best["x"], best["y"], best["th"])
    others = [r["q"] for r in refs if r is not best and not _near((r["x"], r["y"], r["th"]), bp)]
    second = max(others) if others else 0.0
    out.update(x=round(best["x"], 1), y=round(best["y"], 1), dth=round(math.degrees(_wrap(best["th"])), 3),
               score=best["score"], inliers=best["inliers"], known=best["known"], n=best["n"], q=round(best["q"], 3),
               second=round(second, 3), margin=round((best["q"] - second) / best["q"], 3) if best["q"] > 0 else 0.0,
               cands=[[round(r["x"]), round(r["y"]), round(math.degrees(_wrap(r["th"])), 1), round(r["q"], 3)]
                      for r in sorted(refs, key=lambda r: -r["q"])])
    fit = (best["score"] >= A["score"] and best["inliers"] >= A["inliers"] * max(1, best["known"])
           and best["known"] >= A["known"] * best["n"])
    if not fit:
        return done("no fit")
    if out["margin"] < A["margin"]:
        return done("ambiguous")
    out["ok"] = True
    return done()
