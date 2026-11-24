"""Real-time LD19 scan-to-map matching (issue #78, SLAM step 2 of #71): the map pose stops drifting.

Every match_every_n-th LD19 turn (lidar.py, after deskew and the filters, before the turn goes into the grid):
  1. Field: a likelihood field of the map's walls with the grid's cells: 255 on a wall cell (byte > OCC_BYTE), a
     Gaussian (sigma 1.25 cells = 31 mm) down to 0 three cells away. Kept up to date tile by tile: the grid flags the
     tiles it changes (grid.fdirty), a tile's wall mask is compared with the last one, new wall cells are added
     (max of kernels) and only a tile that lost a wall cell is recomputed. Known walls cost nothing.
  2. Points: the turn's points (map frame, current correction) thinned to ~match_points, evenly along each run (>= 50 mm
     apart, centred, so the scan direction does not bias the heading).
  3. Search, correlative and coarse to fine, around the predicted pose (odometry + correction): rotations in 1 deg steps
     over +-match_window_deg x shifts in 2-cell (50 mm) steps over +-match_window_mm, then hill climbing in 0.25 deg /
     1 cell steps from the best one, then least-squares parabolas through 5 samples per axis (sub-cell). A rotation
     turns the points once; a shift is then only an index offset, so a pose's score is sum(field[i + offset]) through
     operator.itemgetter on a memoryview: C speed (~20 ns per point on a PC), stdlib only.
  4. Acceptance: enough points; score (mean field value of the points that land in mapped cells) and inliers (points
     within ~37 mm of a wall); no second pose that fits about as well (margin, coarse against coarse); a clear peak, not
     at the window edge. Degeneracy: the score's curvature at the peak in x / y (2 x 2 Hessian, eigen directions) and in
     theta (+-1 deg). A direction with almost no curvature (along a long featureless corridor; nothing at all in an
     empty space) is not corrected, the others are.
  5. Correction (LidarMatch._accept): the median of the last 5 accepted results, x match_gain, nothing under the dead
     band (match_deadband_mm / _deg: the matcher's own noise would otherwise walk the map off), at most match_step_mm /
     match_step_deg per match (rate limit), applied to Mapper.corr (map pose = corr o odometry pose) as a rotation about
     the robot, then a shift: the map, planner (to_odom, routes resent: Planner.on_drift), places and browser all use
     corr already. A change bigger than match_jump_mm / match_jump_deg is a jump: applied only once JUMP_CONFIRM matches
     in a row agree on it.
  6. Mapping (LidarMatch._map_now): the turn goes into the map at the corrected pose, but only every match_map_mm /
     match_map_deg / match_map_s (more often on new ground) and only with a trusted pose: drawing every turn lets the
     walls follow the pose error. Turning faster than match_max_turn_dps: neither matched nor mapped.
  7. Map-only start: no matching for the first match_warmup turns and while the map has under MIN_WALL_CELLS wall cells,
     so early bad matches cannot bend the map.Not here (next version): loop closure, relocalisation (a kidnapped / reset pose is out of the window: rejected).
"""
import bisect
import collections
import itertools
import math
import operator
import time

OCC_BYTE = 150             # grid byte above which a cell is a wall for the field (log-odds +1.1: two net hits)
SIGMA_CELLS = 1.25
KERNEL_R = 3
PAD = 40                   # zero cells around the field: room for the shifts and for clamped far points (2K + 1)
FINE_DEG = 0.25            # fine rotation step; the coarse one is COARSE_K of them
COARSE_K = 4
COARSE_CELLS = 2           # coarse shift step (cells)
TH_PARA = 2                # the heading parabola samples every 2nd fine step (+-1 deg)
FINE_EXTRA = 6             # cells the hill climb may go past the coarse window
MIN_WALL_CELLS = 150       # map-only until the map has this many wall cells
MIN_POINTS = 40            # thinned points needed
MIN_KNOWN = 0.35           # ... of which this share must land in mapped cells (else: new ground, map only)
MIN_INLIERS = 25
INLIER_VALUE = 128         # field value of an inlier: ~1.2 sigma (37 mm) from a wall
FAR_CELLS = 3.5            # "another pose" for the second-peak margin: this far in a constrained direction ...
FAR_DEG = 1.5              # ... or this far in heading
THIN_MM = 50.0
JUMP_CONFIRM = 3           # matches in a row that must agree on a jump
JUMP_AGREE_MM, JUMP_AGREE_DEG = 40.0, 1.5
FILTER_N, FILTER_MIN, FILTER_S = 5, 3, 3.0   # median of the last 5 results (at least 3, none older than 3 s)
NEW_GROUND, NEW_GROUND_F = 0.8, 0.4   # a turn with over 20 % of its points on unmapped ground: map 2.5 x as often
DOUBT_MAP_S = 10.0         # a pose the matcher keeps rejecting is mapped anyway after this long
HISTORY_N = 40

OCC_LUT = bytes(1 if v > OCC_BYTE else 0 for v in range(256))
_KERNEL = [(di, dj, int(round(255 * math.exp(-(di * di + dj * dj) / (2 * SIGMA_CELLS ** 2)))))
           for dj in range(-KERNEL_R, KERNEL_R + 1) for di in range(-KERNEL_R, KERNEL_R + 1)]
_KERNEL = [k for k in _KERNEL if k[2] >= 8]


def compose(delta, corr, cx, cy):
    """New correction = delta o corr; delta = rotate delta[2] deg about (cx, cy) (map frame), then shift delta[:2]."""
    dx, dy, dt = delta
    tx, ty, tt = corr
    c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))
    qx, qy = tx - cx, ty - cy
    return [c * qx - s * qy + cx + dx, s * qx + c * qy + cy + dy, tt + dt]


def apply(corr, x, y):
    """Odometry (x, y) -> map frame with this correction."""
    c, s = math.cos(math.radians(corr[2])), math.sin(math.radians(corr[2]))
    return c * x - s * y + corr[0], s * x + c * y + corr[1]


def inverse(corr, x, y):
    """Map frame (x, y) -> odometry frame."""
    c, s = math.cos(math.radians(-corr[2])), math.sin(math.radians(-corr[2]))
    return c * (x - corr[0]) - s * (y - corr[1]), s * (x - corr[0]) + c * (y - corr[1])


def thin_runs(runs, cap, spacing=THIN_MM):
    """Runs of points (scan order) -> at most ~cap points, evenly spaced along each run (>= spacing apart) and centred on
    it: symmetric, so the scan direction does not matter (a thinning that always keeps the first point of a run but
    rarely the last one turns the match by ~0.04 deg, every turn the same way)."""
    hypot = math.hypot
    lens = []
    total = 0.0
    runs = [r for r in runs if r]
    for run in runs:
        cum = [0.0]
        cum += itertools.accumulate([hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(run, run[1:])])
        lens.append(cum)
        total += cum[-1] + spacing
    step = max(spacing, total / max(1, cap))
    out = []
    for run, cum in zip(runs, lens):
        L = cum[-1]
        n = int(L / step) + 1
        s = 0.5 * (L - (n - 1) * step)
        last = len(cum) - 1
        for k in range(n):
            target = s + k * step
            i = bisect.bisect_left(cum, target)         # the point nearest to target along the run
            if i > last or (i > 0 and target - cum[i - 1] <= cum[i] - target):
                i -= 1
            out.append(run[i])
    return out


class Field:
    """Likelihood field of the grid's walls, padded by PAD zero cells: index (iy + PAD) * Wp + ix + PAD."""

    def __init__(self):
        self.grid = None
        self.cells = None
        self.f = None
        self.walls = 0
        self.rebuilds = 0

    def invalidate(self):
        self.grid = None

    def sync(self, grid):
        """Bring the field up to date with the grid (cheap when the walls did not change)."""
        if grid is not self.grid or grid.cells is not self.cells:
            self._rebuild(grid)
            return
        tiles, grid.fdirty = grid.fdirty, set()
        for t in tiles:
            self._tile(t)

    def _rebuild(self, grid):
        self.grid, self.cells = grid, grid.cells
        self.W, self.H = grid.W, grid.H
        self.Wp, self.Hp = grid.W + 2 * PAD, grid.H + 2 * PAD
        self.f = bytearray(self.Wp * self.Hp)
        self.kern = [(dj * self.Wp + di, v) for di, dj, v in _KERNEL]
        self.masks = {}
        self.walls = 0
        self.rebuilds += 1
        self.zero = bytes(grid.T * grid.T)
        tiles = grid.touched | grid.dirty | grid.fdirty
        grid.fdirty = set()
        for t in tiles:
            self._tile(t)

    def _splat(self, ix, iy):
        f = self.f
        b = (iy + PAD) * self.Wp + ix + PAD
        for o, v in self.kern:
            i = b + o
            if f[i] < v:
                f[i] = v

    def _tile(self, t):
        g = self.grid
        tx, ty = t
        if not (0 <= tx < g.ntx and 0 <= ty < g.nty):
            return
        T, W, cells = g.T, g.W, g.cells
        x0, y0 = tx * T, ty * T
        new = b"".join([cells[(y0 + j) * W + x0:(y0 + j) * W + x0 + T] for j in range(T)]).translate(OCC_LUT)
        old = self.masks.get(t, self.zero)
        if old == new:
            return
        added, removed = [], False
        for j in range(T):
            a, b = old[j * T:(j + 1) * T], new[j * T:(j + 1) * T]
            if a == b:
                continue
            for i in range(T):
                if a[i] != b[i]:
                    if b[i]:
                        added.append((x0 + i, y0 + j))
                    else:
                        removed = True
        n_new = new.count(1)
        self.walls += n_new - old.count(1)
        if n_new:
            self.masks[t] = new
        else:
            self.masks.pop(t, None)
        if removed:
            self._recompute(x0 - KERNEL_R, y0 - KERNEL_R, x0 + T + KERNEL_R, y0 + T + KERNEL_R)
        else:
            for ix, iy in added:
                self._splat(ix, iy)

    def _recompute(self, i0, j0, i1, j1):
        """Field cells in [i0, i1) x [j0, j1) (grid cells) from the grid's walls (a wall cell went away)."""
        g, f, Wp = self.grid, self.f, self.Wp
        i0, j0, i1, j1 = max(0, i0), max(0, j0), min(g.W, i1), min(g.H, j1)
        if i1 <= i0 or j1 <= j0:
            return
        blank = bytes(i1 - i0)
        for j in range(j0, j1):
            a = (j + PAD) * Wp + i0 + PAD
            f[a:a + i1 - i0] = blank
        R = KERNEL_R
        a0, a1 = max(0, i0 - R), min(g.W, i1 + R)
        W, cells = g.W, g.cells
        for j in range(max(0, j0 - R), min(g.H, j1 + R)):
            row = cells[j * W + a0:j * W + a1].translate(OCC_LUT)
            k = row.find(1)
            while k >= 0:
                self._splat(a0 + k, j)
                k = row.find(1, k + 1)


class Search:
    """Scores of one thinned scan at rotations (fine steps) x shifts (cells) around (cx, cy), cached."""

    def __init__(self, field, pts, cx, cy, K):
        g = field.grid
        self.fld, self.K = field, K
        self.res = g.res
        self.gx0, self.gy0 = g.x0 - PAD * g.res, g.y0 - PAD * g.res     # world mm of padded cell (0, 0)
        self.cx, self.cy = cx, cy
        self.rel = [(x - cx, y - cy) for x, y in pts]
        inv = 1.0 / g.res
        self.rels = [(qx * inv, qy * inv) for qx, qy in self.rel]           # in cells
        self.n = len(pts)
        self.mv = memoryview(field.f)
        self.getters = {}
        self.cache = {}
        # every rotation of every point lands in [K, W - K) x [K, H - K) of the padded field: no clamping needed
        r = max(math.hypot(u, v) for u, v in self.rels) + 1.0
        ax, ay = (cx - self.gx0) * inv, (cy - self.gy0) * inv
        self.safe = K <= ax - r and ax + r < field.Wp - K - 1 and K <= ay - r and ay + r < field.Hp - K - 1

    def getter(self, k):
        gt = self.getters.get(k)
        if gt is None:
            th = math.radians(k * FINE_DEG)
            c, s = math.cos(th), math.sin(th)
            K, Wp = self.K, self.fld.Wp
            ax, ay = (self.cx - self.gx0) / self.res, (self.cy - self.gy0) / self.res
            base = K * Wp + K
            if self.safe:
                idx = [int(c * u - s * v + ax) + Wp * int(s * u + c * v + ay) - base for u, v in self.rels]
            else:
                hi_i, hi_j = Wp - K - 1, self.fld.Hp - K - 1
                idx = []
                for u, v in self.rels:
                    i = int(c * u - s * v + ax)
                    j = int(s * u + c * v + ay)
                    i = K if i < K else (hi_i if i > hi_i else i)        # off the map: clamped into the zero padding
                    j = K if j < K else (hi_j if j > hi_j else j)
                    idx.append(j * Wp + i - base)
            gt = self.getters[k] = operator.itemgetter(*idx)
        return gt
    def score(self, k, sx, sy):
        key = (k, sx, sy)
        v = self.cache.get(key)
        if v is None:
            K = self.K
            v = self.cache[key] = sum(self.getter(k)(self.mv[(sy + K) * self.fld.Wp + sx + K:]))
        return v

    def values(self, k, sx, sy):
        K = self.K
        return self.getter(k)(self.mv[(sy + K) * self.fld.Wp + sx + K:])

    def release(self):
        self.mv.release()


def _vertex5(y):
    """Peak offset (-1 .. 1 steps) of the least-squares parabola through samples at -2 .. 2 steps."""
    b = (-2 * y[0] - y[1] + y[3] + 2 * y[4]) / 10.0
    a = (4 * y[0] + y[1] + y[3] + 4 * y[4] - 2 * sum(y)) / 14.0
    return 0.0 if a >= 0 else max(-1.0, min(1.0, -b / (2 * a)))


def match(field, pts, cx, cy, window_mm=100.0, window_deg=3.0, min_score=0.4, min_inliers=0.5, min_curv=0.05,
          min_margin=0.05):
    """Match thinned map-frame points (current correction) against the field around the robot at (cx, cy).
    Returns a dict: ok, why, dx, dy (mm), dth (deg; rotate about (cx, cy), then shift), score (0..1), n, known, inliers,
    margin, curv [lambda_max, lambda_min, theta], along (degenerate direction, unit vector, or None), fix ("xy", "x'",
    "th" ...: what is corrected), ms."""
    t0 = time.perf_counter()
    out = {"ok": False, "why": "", "dx": 0.0, "dy": 0.0, "dth": 0.0, "score": 0.0, "n": len(pts), "known": 0,
           "inliers": 0, "margin": 0.0, "curv": None, "along": None, "fix": ""}

    def done(why=""):
        out["why"] = why
        out["ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
        return out
    if len(pts) < MIN_POINTS:
        return done("too few points (%d)" % len(pts))
    g = field.grid
    res = g.res
    nxy = max(1, int(math.ceil(window_mm / (COARSE_CELLS * res) - 1e-9)))
    nth = max(1, int(math.ceil(window_deg / (COARSE_K * FINE_DEG) - 1e-9)))
    K = min((PAD - 3) // 2, nxy * COARSE_CELLS + FINE_EXTRA)       # shifts reach +-(K + 1): PAD >= 2K + 3
    nxy = min(nxy, (K - 2) // COARSE_CELLS)
    se = Search(field, pts, cx, cy, K)
    try:
        coarse = []
        for kc in range(-nth, nth + 1):
            k = kc * COARSE_K
            for a in range(-nxy, nxy + 1):
                for b in range(-nxy, nxy + 1):
                    coarse.append((se.score(k, a * COARSE_CELLS, b * COARSE_CELLS), k, a * COARSE_CELLS, b * COARSE_CELLS))
        best = max(coarse)
        if best[0] <= 0:
            return done("no wall in reach")
        # fine: hill climbing on the 0.25 deg x 1 cell lattice (first step covers the coarse cell)
        _, k, sx, sy = best
        kmax, smax = nth * COARSE_K + COARSE_K, K - 1
        cur, cur_s = (k, sx, sy), best[0]
        span = ((-2, -1, 0, 1, 2), (-1, 0, 1))
        for _ in range(12):
            nb = cur
            for dk in span[0]:
                for dx in span[1]:
                    for dy in span[1]:
                        q = (cur[0] + dk, cur[1] + dx, cur[2] + dy)
                        if abs(q[0]) > kmax or abs(q[1]) > smax or abs(q[2]) > smax:
                            continue
                        v = se.score(*q)
                        if v > cur_s:
                            nb, cur_s = q, v
            if nb == cur:
                break
            cur = nb
            span = ((-1, 0, 1), (-1, 0, 1))
        k, sx, sy = cur
        S0 = cur_s
        sc = se.score
        # curvature (relative to the peak): x / y Hessian per cell^2, theta over +-1 deg (the score is a staircase in
        # small rotations: +-0.25 deg would often read flat)
        hxx = -(sc(k, sx + 1, sy) - 2 * S0 + sc(k, sx - 1, sy)) / S0
        hyy = -(sc(k, sx, sy + 1) - 2 * S0 + sc(k, sx, sy - 1)) / S0
        hxy = -(sc(k, sx + 1, sy + 1) - sc(k, sx + 1, sy - 1) - sc(k, sx - 1, sy + 1) + sc(k, sx - 1, sy - 1)) / (4 * S0)
        hth = -(sc(k + COARSE_K, sx, sy) - 2 * S0 + sc(k - COARSE_K, sx, sy)) / S0
        tr, dif = (hxx + hyy) / 2, (hxx - hyy) / 2
        disc = math.sqrt(dif * dif + hxy * hxy)
        l1, l2 = tr + disc, tr - disc
        if abs(hxy) > 1e-12:
            ux, uy = l2 - hyy, hxy                      # eigenvector of the small eigenvalue (the loose direction)
            un = math.hypot(ux, uy)
            ux, uy = ux / un, uy / un
        else:
            ux, uy = (1.0, 0.0) if hxx < hyy else (0.0, 1.0)
        out["curv"] = [round(l1, 3), round(l2, 3), round(hth, 4)]
        # sub-cell / sub-step peak: least-squares parabolas through 5 samples (a wall on a cell border is a 2-cell
        # plateau: a 3-point parabola from one end of it would be half a cell off)
        fx = _vertex5([sc(k, sx + i, sy) for i in (-2, -1, 0, 1, 2)])
        fy = _vertex5([sc(k, sx, sy + i) for i in (-2, -1, 0, 1, 2)])
        fk = TH_PARA * _vertex5([sc(k + i * TH_PARA, sx, sy) for i in (-2, -1, 0, 1, 2)])
        dx, dy, dth = (sx + fx) * res, (sy + fy) * res, (k + fk) * FINE_DEG
        # what is constrained
        xy_ok, th_ok = l1 >= min_curv, hth >= min_curv
        along = None
        if xy_ok and l2 < min_curv:                     # a corridor: no correction along (ux, uy)
            along = (ux, uy)
            p = dx * ux + dy * uy
            dx, dy = dx - p * ux, dy - p * uy
            out["along"] = [round(ux, 3), round(uy, 3)]
        elif not xy_ok:
            dx = dy = 0.0
        if not th_ok:
            dth = 0.0
        out["fix"] = ("xy" if xy_ok and along is None else "x'" if xy_ok else "") + (" th" if th_ok else "")
        out.update(dx=round(dx, 1), dy=round(dy, 1), dth=round(dth, 3))
        # score + inliers at the peak (only points that land in mapped cells count: new ground is not a mismatch)
        vals = se.values(k, sx, sy)
        cells, W, H = g.cells, g.W, g.H
        c, s = math.cos(math.radians(k * FINE_DEG)), math.sin(math.radians(k * FINE_DEG))
        known = inl = tot = 0
        for (qx, qy), v in zip(se.rel, vals):
            i = int((c * qx - s * qy + cx - g.x0) // res) + sx
            j = int((s * qx + c * qy + cy - g.y0) // res) + sy
            if 0 <= i < W and 0 <= j < H and cells[j * W + i] != 128:
                known += 1
                tot += v
                if v >= INLIER_VALUE:
                    inl += 1
        out["known"], out["inliers"] = known, inl
        out["score"] = round(tot / (255.0 * known), 3) if known else 0.0
        # second peak: the best coarse pose clearly elsewhere (in a constrained direction / heading)
        far = 0
        for v, kk, ax, ay in coarse:
            ex, ey = ax - sx, ay - sy
            if along is not None:
                p = ex * along[0] + ey * along[1]
                ex, ey = ex - p * along[0], ey - p * along[1]
            if (xy_ok and math.hypot(ex, ey) > FAR_CELLS) or (th_ok and abs(kk - k) * FINE_DEG > FAR_DEG):
                far = max(far, v)
        # (coarse against coarse: a repeating structure between the coarse steps would otherwise look unique)
        out["margin"] = round((best[0] - far) / best[0], 3)
        if not (xy_ok or th_ok):
            return done("no structure")
        if l2 < -0.5 * min_curv:                        # not a peak in some direction: the best is elsewhere
            return done("no clear peak")
        if abs(k) >= kmax or abs(sx) >= smax or abs(sy) >= smax:
            # at the edge of the search: the best pose may be beyond it (along a corridor the climb may wander: fine)
            px, py = (sx - (sx * along[0] + sy * along[1]) * along[0], sy - (sx * along[0] + sy * along[1]) * along[1]) \
                if along is not None else (sx, sy)
            if (th_ok and abs(k) >= kmax) or abs(px) >= smax - 1 or abs(py) >= smax - 1:
                return done("at the search limit")
        if known < MIN_KNOWN * len(pts):
            return done("mostly new ground (%d of %d points mapped)" % (known, len(pts)))
        if inl < MIN_INLIERS or inl < min_inliers * known:
            return done("too few points fit the map (%d of %d)" % (inl, known))
        if out["score"] < min_score:
            return done("low score (%.2f)" % out["score"])
        if out["margin"] < min_margin:
            return done("ambiguous (margin %.0f %%)" % (100 * out["margin"]))
        out["ok"] = True
        return done("corridor: not along it" if along is not None else "")
    finally:
        se.release()


class LidarMatch:
    """Mapper.lmatch: lidar.py hands every LD19 turn to on_scan() before it is mapped."""

    def __init__(self, mapper):
        self.m = mapper
        self.field = Field()
        self.history = collections.deque(maxlen=HISTORY_N)     # recent results: (ok, why) for the matched share
        self.reset()

    def reset(self):
        self.field.invalidate()
        self.turns = 0
        self.k = 0
        self.n = {"matched": 0, "rejected": 0, "jumps": 0}
        self.last = None             # the last result (match() dict + applied step)
        self.state = "off"
        self.jump = None             # [target corr, matches agreeing]
        self.raw = collections.deque(maxlen=FILTER_N)   # recent accepted results [dx, dy, dth, time] (median filter)
        self.cost_ms = 0.0
        self.cost_avg_ms = 0.0
        self.history.clear()
        self.moved = [0.0, 0.0]      # total applied: mm, deg
        self.mapped_at = None        # map pose + time of the last turn that went into the map
        self.prev_h = None           # (map heading, time) of the last turn (turn rate)
        self.doubt_since = None

    def on_scan(self, ox, oy, mx, my, mh, runs, pts):
        """One turn about to be mapped: ox, oy the lidar origin, (mx, my, mh) the robot's map pose at mid-turn, runs /
        pts the points (map frame, current correction). Returns them, moved to the corrected pose when a match was
        applied; runs is None when this turn should not go into the map (see _map_now)."""
        S = self.m.settings
        self.turns += 1
        if not S.get("lidar_match"):
            self.state = "off"
            return ox, oy, mx, my, runs, pts
        if self.turns <= int(S["match_warmup"]):
            self.state = "building the map"
            self.mapped_at = (mx, my, mh, self._now())
            return ox, oy, mx, my, runs, pts
        now = self._now()
        ph, self.prev_h = self.prev_h, (mh, now)
        if ph is not None and now > ph[1] and \
                abs((mh - ph[0] + 180.0) % 360.0 - 180.0) / (now - ph[1]) > S["match_max_turn_dps"]:
            # turning fast: a few ms of lidar / telemetry timing error turn the whole scan (100 deg/s x 3 ms = 0.3 deg):
            # neither matched nor mapped
            self.state = "turning"
            return ox, oy, mx, my, None, pts
        self.k += 1
        if self.k % max(1, int(S["match_every_n"])):
            return ox, oy, mx, my, (runs if self._map_now(mx, my, mh, None, S) else None), pts
        t0 = time.perf_counter()
        step = None
        try:
            try:
                self.field.sync(self.m.grid)
                if self.field.walls < MIN_WALL_CELLS:
                    res = {"ok": False, "why": "map too small (%d wall cells)" % self.field.walls, "new": True}
                else:
                    sel = thin_runs(runs, max(MIN_POINTS, int(S["match_points"])))
                    res = match(self.field, sel, mx, my, S["match_window_mm"], S["match_window_deg"],
                                S["match_min_score"], S["match_min_inliers"], S["match_min_curv"], S["match_min_margin"])
            except Exception as e:      # a matcher bug must never stop the mapping
                self.field.invalidate()
                res = {"ok": False, "why": "matcher error: %s: %s" % (type(e).__name__, e), "new": True}
            if res["ok"]:
                step = self._accept(res, mx, my, S)
            else:
                self.jump = None
            self.n["matched" if step else "rejected"] += 1
            self.history.append(step is not None)
            res["applied"] = [round(v, 2) for v in step] if step else None
            self.last = res
            self.state = ("matched" if any(step) else "matched: pose fine") if step else "kept: " + res["why"]
        finally:
            self.cost_ms = (time.perf_counter() - t0) * 1000.0
            self.cost_avg_ms = self.cost_ms if not self.cost_avg_ms else 0.9 * self.cost_avg_ms + 0.1 * self.cost_ms
        if step and any(step):          # the turn goes to the corrected pose
            dx, dy, dt = step
            c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))

            def tf(x, y):
                qx, qy = x - mx, y - my
                return c * qx - s * qy + mx + dx, s * qx + c * qy + my + dy
            runs = [[tf(x, y) for x, y in run] for run in runs]
            pts = [p for run in runs for p in run] if sum(len(r) for r in runs) == len(pts) else [tf(x, y) for x, y in pts]
            ox, oy = tf(ox, oy)
            mx, my, mh = mx + dx, my + dy, mh + dt
        trust = step is not None or res.get("new") or res["why"].startswith(("too few points (", "mostly new ground",
                                                                               "no wall", "no structure"))
        return ox, oy, mx, my, (runs if self._map_now(mx, my, mh, trust, S) else None), pts

    def _now(self):
        return getattr(self.m.lidar, "clock", time.monotonic)()

    def _map_now(self, x, y, h, trust, S):
        """Map this turn? While matching, the map is the reference: drawing every turn into it lets it follow the pose
        error (a wall drawn a little turned is matched by the next turn, then drawn again ...). So a turn is mapped only
        once the robot moved match_map_mm / turned match_map_deg since the last mapped one, or after match_map_s (as
        Hector SLAM's map update thresholds), and only with a trusted pose: matched, or nothing to match against (new
        ground). A pose the matcher doubts (a jump, ambiguous, does not fit) is not drawn; after DOUBT_MAP_S of that
        the map takes it anyway (moved furniture). trust None: a turn between matches (the last match decides)."""
        now = self._now()
        if trust is None:
            trust = self.last is None or self.last.get("applied") is not None or self.last.get("new")
        if trust:
            self.doubt_since = None
        elif self.doubt_since is None:
            self.doubt_since = now
        a = self.mapped_at
        L = self.last or {}
        f = NEW_GROUND_F if L.get("new") or (L.get("n") and L.get("known", 0) < NEW_GROUND * L["n"]) else 1.0
        due = a is None or math.hypot(x - a[0], y - a[1]) >= f * S["match_map_mm"] or \
            abs((h - a[2] + 180.0) % 360.0 - 180.0) >= f * S["match_map_deg"] or now - a[3] >= f * S["match_map_s"]
        if not due or not (trust or now - self.doubt_since >= DOUBT_MAP_S):
            return False
        self.mapped_at = (x, y, h, now)
        return True
    def _accept(self, res, cx, cy, S):
        """Filters an accepted match into the correction step applied: (dx, dy, dth) (zeros: within the dead band),
        or None (not accepted: an unconfirmed jump)."""
        m = self.m
        raw = (res["dx"], res["dy"], res["dth"])
        if math.hypot(raw[0], raw[1]) > S["match_jump_mm"] or abs(raw[2]) > S["match_jump_deg"]:
            # a jump (wheel slip, a bump): only once a few matches in a row agree on where the robot really is
            target = compose(raw, m.corr, cx, cy)
            ox, oy = inverse(m.corr, cx, cy)
            j = self.jump
            if j is not None:
                ax, ay = apply(target, ox, oy)
                bx, by = apply(j[0], ox, oy)
                if math.hypot(ax - bx, ay - by) < JUMP_AGREE_MM and abs(target[2] - j[0][2]) < JUMP_AGREE_DEG:
                    j[0], j[1] = target, j[1] + 1
                else:
                    j = None
            if j is None:
                j = self.jump = [target, 1]
                self.raw.clear()                # the odometry error just changed: older results are stale
            if j[1] < JUMP_CONFIRM:
                res["why"] = "jump %.0f mm / %.1f deg: waiting for %d matches to agree" % (
                    math.hypot(raw[0], raw[1]), raw[2], JUMP_CONFIRM)
                return None
            if j[1] == JUMP_CONFIRM:
                self.n["jumps"] += 1
        else:
            self.jump = None
        # filter: the median of the last FILTER_N results (a single turn's heading is good to ~0.1 deg only), then a
        # dead band: a change within the matcher's own noise / bias (a few mm, ~0.05 deg: the grid's 25 mm cells) is
        # left alone, else matching every turn against a map drawn with those very corrections walks the map off
        now = self._now()
        rh = self.raw
        while rh and now - rh[0][3] > FILTER_S:
            rh.popleft()
        rh.append([raw[0], raw[1], raw[2], now])
        if self.jump is not None and self.jump[1] >= JUMP_CONFIRM:
            f = raw                                     # a confirmed jump: no need to wait for the median
        elif len(rh) < FILTER_MIN:
            return 0.0, 0.0, 0.0
        else:
            f = [sorted(e[i] for e in rh)[len(rh) // 2] for i in range(3)]
        g = S["match_gain"]
        gxy = g if math.hypot(f[0], f[1]) > S["match_deadband_mm"] else 0.0
        gth = g if abs(f[2]) > S["match_deadband_deg"] else 0.0
        dx, dy, dt = f[0] * gxy, f[1] * gxy, f[2] * gth
        d = math.hypot(dx, dy)
        lim = S["match_step_mm"]
        if d > lim:
            dx, dy = dx * lim / d, dy * lim / d
        lim = S["match_step_deg"]
        dt = max(-lim, min(lim, dt))
        if not (dx or dy or dt):
            return 0.0, 0.0, 0.0
        if m.places.origin is None:       # the default Home (the odometry origin) stays where it is in the map
            m.places.origin = [round(v, 1) for v in m.map_pose(0.0, 0.0, 0.0)[:2]]
        m.corr = compose((dx, dy, dt), m.corr, cx, cy)
        for e in rh:                      # the filtered results were relative to the old correction
            e[0] -= dx
            e[1] -= dy
            e[2] -= dt
        self.moved[0] += math.hypot(dx, dy)
        self.moved[1] += abs(dt)
        for f in m.corr_listeners:
            f()
        return dx, dy, dt

    def stats(self):
        """For the map's stats line (browsers): state, last score, correction, cost."""
        L = self.last or {}
        h = self.history
        return {"on": bool(self.m.settings.get("lidar_match")), "lidar": self.m.lidar is not None,
                "state": self.state, "score": L.get("score"), "fix": L.get("fix"), "applied": L.get("applied"),
                "ok_pct": round(100.0 * sum(h) / len(h)) if h else None, "ms": round(self.cost_avg_ms, 1),
                "matched": self.n["matched"], "rejected": self.n["rejected"], "jumps": self.n["jumps"],
                "moved_mm": round(self.moved[0]), "moved_deg": round(self.moved[1], 1)}
