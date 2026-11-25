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
     so early bad matches cannot bend the map.
  8. Wheel slip (#79, match_slip): the matched lidar poses are compared with the wheels' travel; when the body does not
     follow the wheels the position is held (heading: gyro) and the lidar takes over. A pose the matcher keeps
     doubting is "lost": not mapped, searched wider (to match_wide_mm / _deg) until a confident match. A slip where
     the body stays put while the robot drives is "stuck" (the bridge tells the navigator). See LidarMatch.
  9. Relocalisation (#105, match_global; reloc.py): lost for GLOBAL_AFTER_S, a Teensy reboot / pose reset seen while a
     map exists (the odometry jumped back to ~0, 0, 0), a map loaded, or "Find myself": a branch-and-bound search over
     the whole map and all headings in the worker thread. A clear, unique best puts the robot there (still lost: not
     mapped) until 2 strict matches confirm it; an ambiguous one (a symmetric room) is not taken: "use Set position".
     Set position (set_pose): the user's pose at once, then the same search in a small window around it.
Not here (next version): loop closure.
"""
import asyncio
import bisect
import collections
import concurrent.futures
import itertools
import math
import operator
import time

from . import link_protocol as lp
from . import loopprof

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
DOUBT_MAP_S = 10.0         # match_slip off (#78 behaviour): a pose the matcher keeps rejecting is mapped after this long
HISTORY_N = 40
# wheel slip / lost / stuck (#79, LidarMatch modes)
GAP_S, GAP_MM = 0.5, 100.0  # no turn for 0.5 s while the wheels went 100 mm: check the pose (the lost search) ...
CHECK_S = 3.0              # ... and lost if that does not confirm it within 3 s
ODO_JUMP_MM = 300.0        # the odometry moved more than this between two turns: a pose reset / gap, not travel
SLIP_REF_S = 1.5           # slip check: against the matched turns of the last 1.5 s
SLIP_SHORT_MM, SLIP_SHORT_DEF, SLIP_SHORT_F = 30.0, 25.0, 0.6   # wheels >= 30 mm: slip if the body is >= 25 mm and 60 % short
SLIP_LONG_MM, SLIP_LONG_DEF, SLIP_LONG_F = 80.0, 40.0, 0.4      # wheels >= 80 mm: ... >= 40 mm and 40 % short (half slip)
GRIP_OK = 0.7              # slipping: the body made >= 70 % of the wheels' travel on 2 matched turns in a row: over
SLIP_IDLE_S, SLIP_IDLE_MM = 1.0, 15.0   # slipping and the wheels moved < 15 mm in the last second: over (nothing to hold)
SLIP_LOST_S = 1.5          # slipping and no match for this long: lost
LOCK_MAX_MM = 200.0        # a slip lock takes at most this much in one go
LOST_AFTER = 3             # matches in a row that doubt the pose (an unconfirmed jump counts half): lost
NEUTRAL = ("too few points (", "mostly new ground", "no wall", "no structure", "map too small", "matcher error")
WIDE_LEVELS = 3            # lost: the window grows to match_wide_mm / _deg in this many steps ...
WIDE_TRIES = 2             # ... after this many failed searches per step
WIDE_MAX_MM = 400.0        # the field's padding allows no more (PAD)
WIDE_POINTS = 100          # points per wide search (and the coarse grid scores every 2nd)
LOST_SLOW_S = 20.0         # lost this long at the widest: one search a second
LOCK_CONFIRM = 2           # lost: strict matches in a row that must agree on the pose
ENV_CONFIRM, ENV_MIN_S = 8, 4.0   # ... or this many looser ones over >= 4 s (the room changed: the way out)
REMAP_S = 3.0              # re-anchored in a changed room / trusted by hand: every turn is mapped this long
ENV_MAX_MM, ENV_MAX_DEG = 60.0, 2.0   # ... and only for a pose that needs no bigger fix than this
LOST_CENTRE_MM, LOST_CENTRES = 600.0, 4   # lost: further centres every 600 mm back along the travel (at most 4)
LOST_CENTRE_MIN_MM = 200.0  # ... when the last lidar fix is this far behind the prediction
CAND_STALE_S = 3.0         # a candidate pose no match agreed with for this long is dropped
STRICT = {"score": 0.55, "inliers": 0.6, "margin": 0.15}
RELAX = {"score": 0.3, "inliers": 0.35, "margin": 0.05}
# a pose the whole-map search found clearly unique at a lower fit (reloc.UNIQUE, #110) is confirmed at that fit
CONFIRM_RELAXED = {"score": 0.45, "inliers": 0.45, "margin": 0.15}
STUCK_MM = 60.0            # lidar stuck: the body moved less than this ...
STUCK_WHEEL_MM = 150.0     # ... while the wheels turned at least this much, for match_stuck_s
STUCK_MODES = (lp.MODE_GOTO, lp.MODE_EXPLORE)

# relocalisation (#105, reloc.py)
GLOBAL_AFTER_S = 5.0       # lost this long: search the whole map ...
GLOBAL_RETRY_S = 12.0      # ... and again this often while still lost (ambiguous / no fit / not confirmed)
CONFIRM_S = 8.0            # a pose found / set waits this long for its 2 strict matches, then counts as unconfirmed
FOUND_SHOW_S = 10.0        # the map info line says "found" this long
SET_BOX_MM, SET_DEG = 600.0, 25.0   # Set position: searched within +-600 mm / +-25 deg of the tapped pose ...
SET_ACCEPT = {"margin": 0.05}       # ... where a runner-up close by is no reason to doubt it
SET_HOLD_S = 60.0          # after a Set position the automatic whole-map search waits this long (the user's word)
RESET_NEAR_MM, RESET_NEAR_DEG = 500.0, 30.0   # a jump of the odometry to within this of 0, 0, 0: a pose reset / reboot

FIELD_BUDGET_S = 0.01      # field update per turn (#102: Pi time); the tiles left over are done with the next turn
#                            (#110: 10 ms, the other chains' steps - match, grid parts, link - share its loop round)
WORKER_DRIVE_SHARE = 0.35  # (#110) the lost / whole-map search's share of a core while the robot drives
DRIVE_MMS = 40.0           # ... "drives": GOTO / EXPLORE, or the wheels faster than this
_POOL = []                 # the lost search's worker thread (#102), made on first use


def _pool():
    if not _POOL:
        _POOL.append(concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="lidar-lost"))
    return _POOL[0]


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
        self.carried = 0           # syncs that ran out of FIELD_BUDGET_S
        self.ver = 0               # bumps on every change (the global search's pyramid cache, #105)

    def invalidate(self):
        self.grid = None

    def sync(self, grid):
        """Bring the field up to date with the grid (cheap when the walls did not change)."""
        if grid is not self.grid or grid.cells is not self.cells:
            if not (grid is self.grid and self._shift(grid)):
                self._rebuild(grid)
                return
        tiles, grid.fdirty = grid.fdirty, set()
        clock = time.perf_counter
        end = clock() + FIELD_BUDGET_S
        it = iter(tiles)
        for t in it:
            self._tile(t)
            if clock() > end:                 # a wall moved in many tiles (#102): the rest with the next turn
                grid.fdirty.update(it)
                self.carried += 1
                break

    def _shift(self, grid):
        """The grid grew (exploring: a tile of new ground at its edge, Grid._reshape): the old cells sit unchanged at
        an offset in the new array, so the field is the old one moved by that offset (#110: no full rebuild, which
        was up to ~150 ms on the Pi, every time the map grew). False if that is not what happened (a reset, a load)."""
        W, H = self.W, self.H
        dx, dy = getattr(self, "cx0", grid.cx0) - grid.cx0, getattr(self, "cy0", grid.cy0) - grid.cy0
        W2, H2, new, T = grid.W, grid.H, grid.cells, grid.T
        if getattr(grid, "gen", None) != getattr(self, "gen", -1) or getattr(grid, "reshapes", 0) == \
                getattr(self, "reshapes", -1):
            return False                     # a reset (Grid._start) or the cells swapped some other way
        if dx < 0 or dy < 0 or dx + W > W2 or dy + H > H2 or dx % T or dy % T:
            return False
        # (cells the growing turn changed in the old area are in grid.fdirty: sync() goes over them next)
        Wp2, Hp2 = W2 + 2 * PAD, H2 + 2 * PAD
        f, Wp, Hp = self.f, self.Wp, self.Hp
        f2 = bytearray(Wp2 * Hp2)
        for j in range(Hp):
            o = (j + dy) * Wp2 + dx
            f2[o:o + Wp] = f[j * Wp:(j + 1) * Wp]
        self.f, self.cells, self.W, self.H, self.Wp, self.Hp = f2, new, W2, H2, Wp2, Hp2
        self.cx0, self.cy0 = grid.cx0, grid.cy0
        self.reshapes = grid.reshapes
        self.kern = [(dj * Wp2 + di, v) for di, dj, v in _KERNEL]
        dtx, dty = dx // T, dy // T
        self.masks = {(a + dtx, b + dty): m for (a, b), m in self.masks.items()}
        self.ver += 1
        self.shifts = getattr(self, "shifts", 0) + 1
        return True

    def _rebuild(self, grid):
        self.grid, self.cells = grid, grid.cells
        self.cx0, self.cy0 = grid.cx0, grid.cy0
        self.gen, self.reshapes = getattr(grid, "gen", None), getattr(grid, "reshapes", 0)
        self.W, self.H = grid.W, grid.H
        self.Wp, self.Hp = grid.W + 2 * PAD, grid.H + 2 * PAD
        self.f = bytearray(self.Wp * self.Hp)
        self.kern = [(dj * self.Wp + di, v) for di, dj, v in _KERNEL]
        self.masks = {}
        self.walls = 0
        self.rebuilds += 1
        self.ver += 1
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
        self.ver += 1
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
        # every rotation of every point lands in [K + 1, W - K - 1) x [K + 1, H - K - 1) of the padded field: no clamping
        # needed (shifts reach +-(K + 1): the climb's edge and the 5-point fits)
        r = max(math.hypot(u, v) for u, v in self.rels) + 1.0
        ax, ay = (cx - self.gx0) * inv, (cy - self.gy0) * inv
        self.safe = K + 1 <= ax - r and ax + r < field.Wp - K - 2 and K + 1 <= ay - r and ay + r < field.Hp - K - 2

    def getter(self, k):
        gt = self.getters.get(k)
        if gt is None:
            th = math.radians(k * FINE_DEG)
            c, s = math.cos(th), math.sin(th)
            K, Wp = self.K, self.fld.Wp
            ax, ay = (self.cx - self.gx0) / self.res, (self.cy - self.gy0) / self.res
            base = (K + 1) * Wp + K + 1     # slices start at (sy + K + 1, sx + K + 1) >= 0 (a negative one wraps)
            if self.safe:
                idx = [int(c * u - s * v + ax) + Wp * int(s * u + c * v + ay) - base for u, v in self.rels]
            else:
                lo, hi_i, hi_j = K + 1, Wp - K - 2, self.fld.Hp - K - 2   # (a shift of +-(K + 1) stays inside, #79)
                idx = []
                for u, v in self.rels:
                    i = int(c * u - s * v + ax)
                    j = int(s * u + c * v + ay)
                    i = lo if i < lo else (hi_i if i > hi_i else i)      # off the map: clamped into the zero padding
                    j = lo if j < lo else (hi_j if j > hi_j else j)
                    idx.append(j * Wp + i - base)
            gt = self.getters[k] = operator.itemgetter(*idx)
        return gt
    def score(self, k, sx, sy):
        key = (k, sx, sy)
        v = self.cache.get(key)
        if v is None:
            K = self.K
            v = self.cache[key] = sum(self.getter(k)(self.mv[(sy + K + 1) * self.fld.Wp + sx + K + 1:]))
        return v

    def values(self, k, sx, sy):
        K = self.K
        return self.getter(k)(self.mv[(sy + K + 1) * self.fld.Wp + sx + K + 1:])

    def release(self):
        self.mv.release()


def _nothing():
    pass


def _vertex5(y):
    """Peak offset (-1 .. 1 steps) of the least-squares parabola through samples at -2 .. 2 steps."""
    b = (-2 * y[0] - y[1] + y[3] + 2 * y[4]) / 10.0
    a = (4 * y[0] + y[1] + y[3] + 4 * y[4] - 2 * sum(y)) / 14.0
    return 0.0 if a >= 0 else max(-1.0, min(1.0, -b / (2 * a)))


def _drain(gen):
    """Run a step generator (match_iter, LidarMatch.on_scan_iter) to its end: its return value."""
    try:
        while True:
            next(gen)
    except StopIteration as e:
        return e.value


def match(*args, **kw):
    """match_iter() in one go."""
    return _drain(match_iter(*args, **kw))


def match_iter(field, pts, cx, cy, window_mm=100.0, window_deg=3.0, min_score=0.4, min_inliers=0.5, min_curv=0.05,
               min_margin=0.05, coarse_stride=1, y=None):
    """Match thinned map-frame points (current correction) against the field around the robot at (cx, cy).
    y: called between steps when the match runs in a worker thread (the lost search: loopprof.Throttle, #110).
    Returns a dict: ok, why, dx, dy (mm), dth (deg; rotate about (cx, cy), then shift), score (0..1), n, known, inliers,
    margin, curv [lambda_max, lambda_min, theta], along (degenerate direction, unit vector, or None), fix ("xy", "x'",
    "th" ...: what is corrected), ms. coarse_stride n: the coarse grid scores every n-th point only (a wide search,
    #79: its cost is in the coarse grid); the climb and the checks use them all."""
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
    sc_c = Search(field, pts[::coarse_stride], cx, cy, K) if coarse_stride > 1 else se
    try:
        coarse = []
        csc = sc_c.score
        y = y or _nothing
        for kc in range(-nth, nth + 1):
            k = kc * COARSE_K
            for a in range(-nxy, nxy + 1):
                y()
                for b in range(-nxy, nxy + 1):
                    coarse.append((csc(k, a * COARSE_CELLS, b * COARSE_CELLS), k, a * COARSE_CELLS, b * COARSE_CELLS))
            yield                                       # (#110) a step: the bridge may let the loop run here
        best = max(coarse)
        if best[0] <= 0:
            return done("no wall in reach")
        # fine: hill climbing on the 0.25 deg x 1 cell lattice (first step covers the coarse cell)
        _, k, sx, sy = best
        kmax, smax = nth * COARSE_K + COARSE_K, K - 1
        cur, cur_s = (k, sx, sy), se.score(k, sx, sy)
        span = ((-2, -1, 0, 1, 2), (-1, 0, 1))
        for _ in range(12):
            y()
            yield
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
        yield
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
        if sc_c is not se:
            sc_c.release()


class LidarMatch:
    """Mapper.lmatch: lidar.py hands every LD19 turn to on_scan() before it is mapped.

    Modes (#79, setting match_slip; off = the #78 behaviour: track only, a doubted pose is mapped after 10 s):
      track  normal: the #78 filter (median, gain, rate limit, jumps after 3 agreeing matches).
      slip   the wheels turned but the matched lidar pose did not follow (see _observe): their travel is not trusted.
             Each turn the correction takes the odometry's move back out (hold the position, heading from the gyro),
             every turn is matched around that held pose and a match is taken as it is (the lidar is the odometry
             now). Nothing is mapped. Over once the body follows the wheels again (or the wheels stop).
      lost   matches kept doubting the pose (or none while slipping): nothing is mapped; the search widens step by
             step to match_wide_mm / match_wide_deg, runs less often and on fewer points, and the pose is only taken
             back after 2 strict matches agree (or, the way out for a changed room, 8 looser ones over 4 s). The map
             menu's "Trust the pose" ends it by hand.
    Stuck: slipping, wheels turning but the lidar pose still for match_stuck_s in GOTO / EXPLORE -> stuck_listeners
    (the bridge sends EVT_LINK_STUCK: the navigator backs off like its own stuck check)."""

    def __init__(self, mapper):
        self.m = mapper
        self.field = Field()
        self.history = collections.deque(maxlen=HISTORY_N)     # recent results: (ok, why) for the matched share
        self.stuck_listeners = []    # callables(): lidar stuck (#79), the bridge tells the robot
        self.tlm_mode = None         # robot mode from telemetry (on_tlm)
        self.tlm_speed = 0.0         # ... and its speed (mm/s): driving -> the searches take less CPU (#110)
        self.reset()

    def reset(self):
        self.field.invalidate()
        self.gen = getattr(self, "gen", 0) + 1      # a lost search still running in its thread is dropped (#102)
        self.job = None
        self.turns = 0
        self.k = 0
        self.n = {"matched": 0, "rejected": 0, "jumps": 0, "slips": 0, "lost": 0, "found": 0, "stuck": 0, "checks": 0,
                  "global": 0, "relocated": 0}
        self.pyr = None              # (key, reloc.Pyramid) of the last global search (#105): reused while the map is the same
        self.reloc = None            # the global search / Set position state for the browsers (#105, _reloc_stats)
        self.last_global = None      # the last reloc.find() result
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
        self._reset_recover()

    def _reset_recover(self):
        self.mode = "track"          # track | slip | lost (#79)
        self.prev_odo = None         # (odometry x, y, time) of the last turn
        self.odo_path = 0.0          # wheel travel so far (sum of the odometry's moves between turns), mm
        self.wheels = collections.deque(maxlen=40)      # (time, odo_path) per turn: are the wheels turning?
        self.obs = collections.deque(maxlen=24)         # matched turns: (time, odo x, y, lidar map x, y)
        self.suspect = None          # time of the reference turn a first slip sign was measured against
        self.grip = None             # body travel / wheel travel, last measured
        self.grip_ok = 0
        self.fails = 0.0             # matches in a row that doubt the pose (track)
        self.slip_t0 = self.slip_seen = 0.0
        self.held_mm = 0.0           # wheel travel held back in this slip
        self.stuck_ref = None        # (time, lidar x, y, odo_path) where the body was last seen to move
        self.lost_t0 = 0.0
        self.level = 0               # lost: wide search level 1 .. WIDE_LEVELS
        self.tries = 0
        self.cand = None             # lost: [target corr, agreeing, strict in a row, since, last agreeing, centre]
        self.offs, self.off_i = [(0.0, 0.0)], 0   # lost: search centres (offsets from the prediction), next one
        self.cands = {}              # lost: candidate per centre index
        self.cand_i = 0
        self.last_fix = None         # (lidar map x, y, time) of the last full x / y fix
        self.last_pose = None        # (map x, y, heading, time) of the last turn (pose reset: keep the robot there)
        self.remap_until = 0.0       # re-anchored: map every turn until then
        self.checking = False        # lost only to check the pose after a lidar gap (quiet; lost after CHECK_S)
        self.lost_k = 0              # self.k when it got lost
        self.glob_req = None         # {"why", "by", "around"}: a global search starts with the next turn (#105)
        self.glob_t = None           # lost: when the last global search ended (or a Set position holds them off until)
        self.glob_running = False    # the job in the worker thread is a global search
        self.confirm_t0 = None       # a pose found / set by hand waits for its confirmation since

    # ------------------------------------------------------------ inputs
    def prepare(self):
        """The field up to date with the grid: the bridge runs this as its own loop callback before on_scan (#102), whose
        own sync then has little left to do."""
        if self.m.settings.get("lidar_match") and self.job is None:
            with getattr(self.m, "prof", loopprof.NULL).part("match.field"):
                self.field.sync(self.m.grid)

    def on_tlm(self, d, t):
        self.tlm_mode = d.get("mode")
        self.tlm_speed = d.get("speed_mms", 0.0)

    def _driving(self):
        return self.tlm_mode in STUCK_MODES or abs(getattr(self, "tlm_speed", 0.0) or 0.0) > DRIVE_MMS

    def _worker_share(self):
        """(#110) The lost / whole-map search's share of a core: less while the robot drives (the loop needs the Pi
        then: obstacles for the navigator, the link); standing still it may take what the loop leaves."""
        return WORKER_DRIVE_SHARE if self._driving() else 1.0

    def on_scan(self, ox, oy, mx, my, mh, runs, pts):
        """on_scan_iter() in one go."""
        return _drain(self.on_scan_iter(ox, oy, mx, my, mh, runs, pts))

    def on_scan_iter(self, ox, oy, mx, my, mh, runs, pts):
        """(A step generator, #110: the bridge runs its steps in separate loop callbacks, ~20 ms of Pi time each.)
        One turn about to be mapped: ox, oy the lidar origin, (mx, my, mh) the robot's map pose at mid-turn, runs /
        pts the points (map frame, current correction). Returns them, moved to the corrected pose when a match was
        applied; runs is None when this turn should not go into the map (see _map_now)."""
        S = self.m.settings
        self.turns += 1
        recover = bool(S.get("match_slip", True))
        if self.mode != "track" and not (recover and S.get("lidar_match")):
            self.mode, self.suspect = "track", None        # switched off meanwhile
        if not S.get("lidar_match"):
            self.state = "off"
            return ox, oy, mx, my, runs, pts
        now = self._now()
        odo = inverse(self.m.corr, mx, my)              # where the wheels say the robot is (odometry frame)
        oh = mh - self.m.corr[2]                        # ... and its odometry (gyro) heading
        prev, self.prev_odo = self.prev_odo, (odo[0], odo[1], now, oh)
        gap = None
        if not recover:
            self.glob_req = None
        if prev is not None and recover and self.turns > int(S["match_warmup"]):
            d = math.hypot(odo[0] - prev[0], odo[1] - prev[1])
            if self._reset_seen(prev, odo, oh, S):
                # the odometry jumped back to ~0, 0, 0 (a Teensy reboot, or a pose reset nobody logged): the map pose
                # jumped to the old origin. Search the whole map for where the robot really is (#105)
                self.request_global("the odometry jumped to %d, %d, %d deg: a Teensy reboot / pose reset"
                                    % (round(odo[0]), round(odo[1]), round(oh)), by="reset")
            elif now - prev[2] > GAP_S and d > GAP_MM:
                gap = (now - prev[2], d)
        if prev is not None and (now - prev[2] > 1.0 or math.hypot(odo[0] - prev[0], odo[1] - prev[1]) > ODO_JUMP_MM):
            prev = None                                  # a gap / pose reset: no wheel travel to compare
        if prev is not None:
            self.odo_path += math.hypot(odo[0] - prev[0], odo[1] - prev[1])
        self.wheels.append((now, self.odo_path))
        if self.turns <= int(S["match_warmup"]):
            self.state = "building the map"
            self.mapped_at = (mx, my, mh, now)
            self.last_pose = (mx, my, mh, now)
            return ox, oy, mx, my, runs, pts
        if recover and self.mode == "slip":
            if prev is not None:
                ox, oy, mx, my, runs, pts = self._hold(odo, prev, ox, oy, mx, my, runs, pts)
            if self._wheels_idle(now):
                self._end_slip(now, "the wheels stopped")
        if gap is not None and self.mode != "lost":
            # no turn for a while (lidar out, tilted) but the wheels went on: whatever they did (slip?) nobody saw,
            # and a match in the normal window can snap onto the wrong wall. Check the pose with the lost search
            # (not mapped until 2 strict matches agree); a real "lost" only if that takes over CHECK_S.
            self._lose(now, "no lidar turn for %.1f s while the wheels went %d mm" % gap, pred=(mx, my), check=True)
        self.last_pose = (mx, my, mh, now)
        ph, self.prev_h = self.prev_h, (mh, now)
        if ph is not None and now > ph[1] and \
                abs((mh - ph[0] + 180.0) % 360.0 - 180.0) / (now - ph[1]) > S["match_max_turn_dps"]:
            # turning fast: a few ms of lidar / telemetry timing error turn the whole scan (100 deg/s x 3 ms = 0.3 deg):
            # neither matched nor mapped
            self.state = "turning" if self.mode == "track" else self.state
            return ox, oy, mx, my, None, pts
        self.k += 1
        if recover and self.glob_req is not None and self.mode != "lost":
            self._lose(now, self.glob_req["why"], quiet=True)    # not mapped until the pose is found and confirmed
        lost = recover and self.mode == "lost"
        if lost and self.job is None and self._global_due(now, S):
            if self._global_start(runs, mx, my, now, S):         # whole-map search (#105) with this turn
                return ox, oy, mx, my, None, pts
        if lost:
            self._confirm_check(now)
            every = self._wide_every(now)
            if self.checking and now - self.lost_t0 > CHECK_S:
                self.checking = False                    # the check did not confirm it: lost
                self.n["lost"] += 1
                self.m.say("lidar: pose lost (not confirmed %.0f s after a lidar gap): searching wider, not mapping"
                           % CHECK_S)
        elif recover and (self.mode == "slip" or self.suspect is not None):
            every = 1                                    # slipping / a first sign of it: every turn
        else:
            every = max(1, int(S["match_every_n"]))
        if (self.k - self.lost_k - 1 if lost else self.k) % every:
            if lost or (recover and self.mode == "slip"):
                return ox, oy, mx, my, None, pts
            return ox, oy, mx, my, (runs if self._map_now(mx, my, mh, None, S) else None), pts
        if lost and self.job is not None:
            return ox, oy, mx, my, None, pts              # the last lost search is still running in its thread
        t0 = time.perf_counter()
        step = None
        try:
            try:
                P = getattr(self.m, "prof", loopprof.NULL).part
                with P("match.field"):
                    self.field.sync(self.m.grid)
                if self.field.walls < MIN_WALL_CELLS:
                    res = {"ok": False, "why": "map too small (%d wall cells)" % self.field.walls, "new": True}
                elif lost:
                    mm, deg = self._wide(self.level, S)
                    off = self._lost_centre()
                    if off != (0.0, 0.0):
                        mm, deg = self._wide(WIDE_LEVELS, S)
                    sel = thin_runs(runs, WIDE_POINTS)
                    if off != (0.0, 0.0):
                        sel = [(x + off[0], y + off[1]) for x, y in sel]
                    args = (self.field, sel, mx + off[0], my + off[1], mm, deg, RELAX["score"], RELAX["inliers"],
                            S["match_min_curv"], RELAX["margin"])
                    loop = self._loop()
                    if loop is not None:
                        # the wide search (~150-600 ms on the Pi) in a worker thread (#102): nothing is mapped while
                        # lost, so its turn waits for nothing; the result goes through _recover() when it is in
                        self._lost_job(loop, args, off, mm, deg, odo, mx, my, now)
                        return ox, oy, mx, my, None, pts
                    res = self._lost_res(match(*args, coarse_stride=2), off, mm, deg)
                else:
                    sel = thin_runs(runs, max(MIN_POINTS, int(S["match_points"])))
                    res = yield from match_iter(self.field, sel, mx, my, S["match_window_mm"], S["match_window_deg"],
                                S["match_min_score"], S["match_min_inliers"], S["match_min_curv"], S["match_min_margin"])
            except Exception as e:      # a matcher bug must never stop the mapping
                self.field.invalidate()
                res = {"ok": False, "why": "matcher error: %s: %s" % (type(e).__name__, e), "new": True}
            step = self._result(res, recover, odo, mx, my, now, S)
        finally:
            self.cost_ms = (time.perf_counter() - t0) * 1000.0
            self.cost_avg_ms = self.cost_ms if not self.cost_avg_ms else 0.9 * self.cost_avg_ms + 0.1 * self.cost_ms
        if step and any(step):          # the turn goes to the corrected pose
            dx, dy, dt = step
            c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))

            def tf(x, y):
                qx, qy = x - mx, y - my
                return c * qx - s * qy + mx + dx, s * qx + c * qy + my + dy
            ox, oy, runs, pts = self._move(tf, ox, oy, runs, pts)
            mx, my, mh = mx + dx, my + dy, mh + dt
            self.last_pose = (mx, my, mh, now)
        if recover and self.mode != "track":
            return ox, oy, mx, my, None, pts             # slipping / lost: the pose is not good enough to draw with
        trust = step is not None or res.get("new") or res["why"].startswith(("too few points (", "mostly new ground",
                                                                               "no wall", "no structure"))
        return ox, oy, mx, my, (runs if self._map_now(mx, my, mh, trust, S) else None), pts

    def _result(self, res, recover, odo, mx, my, now, S):
        """A match result -> the correction step applied (or None); counters, history, state."""
        if recover:
            step = self._recover(res, odo, mx, my, now, S)
        elif res["ok"]:
            step = self._accept(res, mx, my, S)
        else:
            step = None
            self.jump = None
        self.n["matched" if step else "rejected"] += 1
        self.history.append(step is not None)
        res["applied"] = [round(v, 2) for v in step] if step else None
        self.last = res
        self.state = self._state_text(step, res)
        return step

    @staticmethod
    def _lost_res(res, off, mm, deg):
        res["dx"], res["dy"] = res["dx"] + off[0], res["dy"] + off[1]   # from the prediction
        res["window"] = [round(mm), round(deg, 1)]
        res["off"] = [round(off[0]), round(off[1])]
        return res

    def _loop(self):
        """The running event loop when the bridge runs the lost search in a thread (lidar.defer), else None."""
        if not getattr(self.m.lidar, "defer", False):
            return None
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return None

    def _lost_job(self, loop, args, off, mm, deg, odo, mx, my, now):
        """Run one lost search in the worker thread; its result is handled on the loop (_lost_done). The field is not
        touched meanwhile: nothing is mapped while lost, and no other search starts before this one is done."""
        gen = self.gen
        prof = getattr(self.m, "prof", loopprof.NULL)
        share = self._worker_share()

        def work():
            with prof.thread("match.lost"):
                try:
                    return match(*args, coarse_stride=2, y=loopprof.Throttle(share))
                except Exception as e:
                    return {"ok": False, "why": "matcher error: %s: %s" % (type(e).__name__, e), "new": True,
                            "dx": 0.0, "dy": 0.0, "dth": 0.0}

        def done(fut):              # (worker thread)
            try:
                loop.call_soon_threadsafe(self._lost_done, fut, gen, off, mm, deg, odo, mx, my, now)
            except RuntimeError:    # the loop is gone (shutdown)
                pass
        self.job = _pool().submit(work)
        self.job.add_done_callback(done)

    def _lost_done(self, fut, gen, off, mm, deg, odo, mx, my, now):
        if gen != self.gen:
            return                  # reset / another map meanwhile
        self.job = None
        S = self.m.settings
        if self.mode != "lost" or not S.get("lidar_match"):
            return                  # found / trusted by hand / switched off meanwhile
        try:
            res = fut.result()
        except Exception as e:
            res = {"ok": False, "why": "matcher error: %s: %s" % (type(e).__name__, e), "new": True, "dx": 0.0, "dy": 0.0}
        t0 = time.perf_counter()
        with getattr(self.m, "prof", loopprof.NULL).part("lidar.match"):
            self._result(self._lost_res(res, off, mm, deg), True, odo, mx, my, now, S)
        self.cost_ms = res.get("ms", 0.0) + (time.perf_counter() - t0) * 1000.0

    def _now(self):
        return getattr(self.m.lidar, "clock", time.monotonic)()

    @staticmethod
    def _move(tf, ox, oy, runs, pts):
        n = sum(len(r) for r in runs)
        runs = [[tf(x, y) for x, y in run] for run in runs]
        pts = [p for run in runs for p in run] if n == len(pts) else [tf(x, y) for x, y in pts]
        ox, oy = tf(ox, oy)
        return ox, oy, runs, pts

    def _map_now(self, x, y, h, trust, S):
        """Map this turn? While matching, the map is the reference: drawing every turn into it lets it follow the pose
        error (a wall drawn a little turned is matched by the next turn, then drawn again ...). So a turn is mapped only
        once the robot moved match_map_mm / turned match_map_deg since the last mapped one, or after match_map_s (as
        Hector SLAM's map update thresholds), and only with a trusted pose: matched, or nothing to match against (new
        ground). A pose the matcher doubts (a jump, ambiguous, does not fit) is not drawn; without match_slip, after
        DOUBT_MAP_S of that the map takes it anyway (with it, the matcher goes "lost" instead: see _recover).
        trust None: a turn between matches (the last match decides)."""
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
        late = not S.get("match_slip", True) and now - self.doubt_since >= DOUBT_MAP_S if not trust else False
        if now < self.remap_until:                 # re-anchored (#79): every turn, whatever the matcher says
            due, trust = True, True
        if not due or not (trust or late):
            return False
        self.mapped_at = (x, y, h, now)
        return True

    # ------------------------------------------------------------ track: the #78 filter
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
        return self._apply(dx, dy, dt, cx, cy)

    def _apply(self, dx, dy, dt, cx, cy):
        """Move the correction by (dx, dy, dt) (rotate dt about the robot at (cx, cy), then shift)."""
        m = self.m
        if m.places.origin is None:       # the default Home (the odometry origin) stays where it is in the map
            m.places.origin = [round(v, 1) for v in m.map_pose(0.0, 0.0, 0.0)[:2]]
        m.corr = compose((dx, dy, dt), m.corr, cx, cy)
        for e in self.raw:                # the filtered results were relative to the old correction
            e[0] -= dx
            e[1] -= dy
            e[2] -= dt
        self.moved[0] += math.hypot(dx, dy)
        self.moved[1] += abs(dt)
        for f in m.corr_listeners:
            f()
        return dx, dy, dt

    # ------------------------------------------------------------ #79: slip, lost, stuck
    def _recover(self, res, odo, mx, my, now, S):
        """A match result -> the correction step applied, or None; switches between track / slip / lost."""
        if self.mode == "lost":
            return self._lost_result(res, mx, my, now, S)
        full = res["ok"] and res.get("along") is None and res.get("fix", "").startswith("xy")
        verdict = self._observe(now, odo, (mx + res["dx"], my + res["dy"])) if full else ""
        if self.mode == "slip":
            if not res["ok"]:
                if now - self.slip_seen > SLIP_LOST_S:
                    self._lose(now, "no match while the wheels slip")
                return None
            self.slip_seen = now
            step = self._lock(res, mx, my, S)
            if full:
                self._stuck_check(now, (mx + res["dx"], my + res["dy"]), S)
                self.grip_ok = self.grip_ok + 1 if verdict == "grip" else 0
                if self.grip_ok >= 2:
                    self._end_slip(now, "grip again")
            return step
        if verdict == "slip":                      # track: the wheels ran off without the body
            self._start_slip(now)
            return self._lock(res, mx, my, S)
        if res["ok"]:
            step = self._accept(res, mx, my, S)
            self.fails = 0.0 if step is not None else self.fails + 0.5   # an unconfirmed jump: half a doubt
        else:
            self.jump = None
            if not res.get("new") and not res["why"].startswith(NEUTRAL):
                self.fails += 1.0
            step = None
        if now < self.remap_until:
            self.fails = 0.0                       # re-anchored: the new walls are still going into the map
        if self.fails >= LOST_AFTER:
            self._lose(now, res["why"])
        return step

    def _observe(self, now, odo, L):
        """A matched turn with a full x / y fix: odo the odometry position, L the lidar's (map frame). Compares the
        body's travel (lidar) with the wheels' (odometry) against the matched turns of the last SLIP_REF_S:
        'slip' (confirmed: a second turn measured against the same or an older reference also says so, so one bad
        match cannot start it), 'suspect' (a first sign), 'grip' (body >= GRIP_OK of the wheels) or ''."""
        hist = self.obs
        while hist and now - hist[0][0] > SLIP_REF_S:
            hist.popleft()
        th = math.radians(self.m.corr[2])
        c, s = math.cos(th), math.sin(th)
        ref = None
        grip = None
        confirmed = False
        for t, ax, ay, lx, ly in reversed(hist):
            dx, dy = odo[0] - ax, odo[1] - ay
            d = math.hypot(dx, dy)
            if d < SLIP_SHORT_MM:
                continue
            ux, uy = (c * dx - s * dy) / d, (s * dx + c * dy) / d      # wheel travel direction, map frame
            along = (L[0] - lx) * ux + (L[1] - ly) * uy
            if grip is None:
                grip = along / d
            short = d - along >= max(SLIP_SHORT_DEF, SLIP_SHORT_F * d)
            far = d >= SLIP_LONG_MM and d - along >= max(SLIP_LONG_DEF, SLIP_LONG_F * d)
            if short or far:
                if ref is None:
                    ref = t
                if self.suspect is not None and t <= self.suspect + 1e-9:
                    confirmed = True
                    break
        hist.append((now, odo[0], odo[1], L[0], L[1]))
        self.last_fix = (L[0], L[1], now)
        if grip is not None:
            self.grip = max(0.0, min(1.5, grip))
        if confirmed:
            self.suspect = None
            return "slip"
        self.suspect = ref
        if ref is not None:
            return "suspect"
        return "grip" if grip is not None and grip >= GRIP_OK else ""

    def _hold(self, odo, prev, ox, oy, mx, my, runs, pts):
        """Slipping: the correction takes the wheels' move since the last turn back out (map pose held; the heading
        is the gyro's, untouched)."""
        th = math.radians(self.m.corr[2])
        c, s = math.cos(th), math.sin(th)
        ex, ey = odo[0] - prev[0], odo[1] - prev[1]
        dx, dy = -(c * ex - s * ey), -(s * ex + c * ey)
        if not (dx or dy):
            return ox, oy, mx, my, runs, pts
        cr = self.m.corr
        self.m.corr = [cr[0] + dx, cr[1] + dy, cr[2]]
        self.held_mm += math.hypot(dx, dy)
        for e in self.raw:
            e[0] -= dx
            e[1] -= dy
        for f in self.m.corr_listeners:
            f()
        ox, oy, runs, pts = self._move(lambda x, y: (x + dx, y + dy), ox, oy, runs, pts)
        return ox, oy, mx + dx, my + dy, runs, pts

    def _lock(self, res, cx, cy, S):
        """Slipping / found again: take the match as it is (no median, gain or jump wait; heading <= match_step_deg)."""
        dx, dy = res["dx"], res["dy"]
        d = math.hypot(dx, dy)
        if d > LOCK_MAX_MM:
            dx, dy = dx * LOCK_MAX_MM / d, dy * LOCK_MAX_MM / d
        lim = S["match_step_deg"]
        dt = max(-lim, min(lim, res["dth"]))
        self.raw.clear()
        self.jump = None
        if not (dx or dy or dt):
            return 0.0, 0.0, 0.0
        return self._apply(dx, dy, dt, cx, cy)

    def _wheels_idle(self, now):
        w = self.wheels
        old = None
        for t, p in w:
            if now - t <= SLIP_IDLE_S:
                old = p
                break
        return old is not None and w and now - w[0][0] >= SLIP_IDLE_S and self.odo_path - old < SLIP_IDLE_MM

    def _start_slip(self, now):
        self.mode = "slip"
        self.slip_t0 = self.slip_seen = now
        self.held_mm = 0.0
        self.grip_ok = 0
        self.stuck_ref = None
        self.fails = 0.0
        self.n["slips"] += 1
        self.m.say("lidar: wheel slip (the body made %d %% of the wheels' travel): holding the position, not mapping"
                   % round(100 * (self.grip or 0.0)))

    def _end_slip(self, now, why):
        self.mode = "track"
        self.suspect = None
        self.raw.clear()
        self.jump = None
        self.fails = 0.0
        self.m.say("lidar: wheel slip over after %.1f s (%s), %d mm of wheel travel left out"
                   % (now - self.slip_t0, why, round(self.held_mm)))

    def _stuck_check(self, now, L, S):
        """Slipping in GOTO / EXPLORE: the wheels keep turning but the lidar pose stays put for match_stuck_s."""
        lim = S.get("match_stuck_s", 0.0)
        r = self.stuck_ref
        if r is None or math.hypot(L[0] - r[1], L[1] - r[2]) > STUCK_MM:
            self.stuck_ref = (now, L[0], L[1], self.odo_path)
            return False
        if lim <= 0 or now - r[0] < lim or self.odo_path - r[3] < STUCK_WHEEL_MM or self.tlm_mode not in STUCK_MODES:
            return False
        self.stuck_ref = (now, L[0], L[1], self.odo_path)       # again after another match_stuck_s
        self.n["stuck"] += 1
        self.m.say("lidar: stuck - the wheels turned %d mm in %.1f s but the robot did not move: robot told to back off"
                   % (round(self.odo_path - r[3]), now - r[0]))
        for f in self.stuck_listeners:
            try:
                f()
            except Exception:
                pass
        return True

    def _lose(self, now, why, pred=None, check=False, quiet=False):
        """Lost (check: only checking the pose after a lidar gap, from the +-300 mm level, quietly; quiet: the caller
        says why, #105)."""
        self.mode = "lost"
        self.checking = check
        self.lost_t0 = now
        self.glob_t = None
        self.confirm_t0 = None
        self.lost_k = self.k                      # the next turn searches at once
        self.level, self.tries = (2 if check else 1), 0
        self.cand = None
        # where to look: slip puts the robot behind its odometry, somewhere back along the wheels' travel since the
        # last lidar fix; the window is +-match_wide_mm, so beyond that the search also tries centres back along it
        # (the last fix itself may be a bad match next to the prediction: one centre more than the gap needs)
        self.offs, self.off_i, self.cands = [(0.0, 0.0)], 0, {}
        f, p = self.last_fix, pred or self.last_pose
        if f is not None and p is not None:
            vx, vy = f[0] - p[0], f[1] - p[1]
            d = math.hypot(vx, vy)
            if d >= LOST_CENTRE_MIN_MM:
                n = min(LOST_CENTRES, int(math.ceil(d / LOST_CENTRE_MM)) + 1)
                self.offs += [(vx / d * LOST_CENTRE_MM * k, vy / d * LOST_CENTRE_MM * k) for k in range(1, n + 1)]
        self.suspect = None
        self.raw.clear()
        self.jump = None
        if check:
            self.n["checks"] += 1
            return
        self.n["lost"] += 1
        if not quiet:
            self.m.say("lidar: pose lost (%s): searching wider, not mapping until it is found" % why)

    def _found(self, now, how, remap=False):
        """Back to track. remap (re-anchored in a changed room / trusted by hand): every turn is mapped for REMAP_S,
        so the new walls are in the map before the matcher doubts the pose again."""
        self.mode = "track"
        self.fails = 0.0
        self.remap_until = now + REMAP_S if remap else 0.0
        self.cand = None
        self.cands = {}
        self.offs = [(0.0, 0.0)]
        self.obs.clear()
        self.suspect = None
        self.mapped_at = None                     # map at once from the pose found
        self.glob_req = None
        rl = self.reloc
        if rl is not None and rl["phase"] in ("confirming", "set", "set_poor", "unconfirmed"):
            # a pose the global search found / the user set, now confirmed by the matcher (#105)
            self.n["relocated"] += 1
            rl.update(phase="found", t=now)
            how = ("%s, found by the whole-map search" if rl.get("by") not in ("set",) else
                   "%s, from the position set by hand") % how
        elif rl is not None and rl["phase"] != "found":
            self.reloc = None
        self.confirm_t0 = None
        if self.checking:
            self.checking = False
            if "moved" in how and not how.endswith(("moved 0 mm", "moved 1 mm")):
                self.m.say("lidar: pose checked after a lidar gap (%s)" % how)
            return
        self.n["found"] += 1
        self.m.say("lidar: pose found again after %.1f s (%s)" % (now - self.lost_t0, how))

    @staticmethod
    def _wide(level, S):
        """Search window of a lost level: from the normal one up to match_wide_mm / _deg in WIDE_LEVELS steps."""
        f = level / float(WIDE_LEVELS)
        mm = S["match_window_mm"] + f * max(0.0, S["match_wide_mm"] - S["match_window_mm"])
        deg = S["match_window_deg"] + f * max(0.0, S["match_wide_deg"] - S["match_window_deg"])
        return min(mm, WIDE_MAX_MM), deg

    def _wide_every(self, now):
        """Lost: search every n-th turn, wider = rarer (CPU); after LOST_SLOW_S at the widest, once a second."""
        if self.level >= WIDE_LEVELS and now - self.lost_t0 > LOST_SLOW_S:
            return 10
        return 1 + self.level

    def _lost_centre(self):
        """The search centre's offset from the prediction for this lost search (map frame): the centre of a candidate
        that had a strict match (to confirm it), else the next of self.offs in turn."""
        for i, c in self.cands.items():
            if c[2] >= 1:
                self.cand = c
                return self.offs[i]
        i = self.off_i % len(self.offs)
        self.off_i += 1
        self.cand = self.cands.get(i)
        self.cand_i = i
        return self.offs[i]

    def _centre_index(self, res):
        off = tuple(res.get("off") or (0, 0))
        for i, o in enumerate(self.offs):
            if (round(o[0]), round(o[1])) == off:
                return i
        return 0

    def _lost_result(self, res, mx, my, now, S):
        """Lost: a wide search result. The pose is taken back after LOCK_CONFIRM strict matches in a row agree on it
        (score / inliers / margin above STRICT), or ENV_CONFIRM looser ones over ENV_MIN_S (the room changed: what still
        fits pins the pose, the rest gets mapped). Otherwise the next search is wider."""
        m = self.m
        self.tries += 1
        if res["ok"]:
            target = compose((res["dx"], res["dy"], res["dth"]), m.corr, mx, my)
            rl = self.reloc
            st = CONFIRM_RELAXED if rl and rl.get("phase") == "confirming" and rl.get("relaxed") else STRICT
            strict = (res["score"] >= max(S["match_min_score"], st["score"]) and
                      res["inliers"] >= max(S["match_min_inliers"], st["inliers"]) * res["known"] and
                      res["margin"] >= max(S["match_min_margin"], st["margin"]))
            c = self.cand
            if c is not None and now - c[4] > CAND_STALE_S:
                c = None
            if c is not None:
                ox, oy = inverse(m.corr, mx, my)
                ax, ay = apply(target, ox, oy)
                bx, by = apply(c[0], ox, oy)
                if math.hypot(ax - bx, ay - by) < JUMP_AGREE_MM and abs(target[2] - c[0][2]) < JUMP_AGREE_DEG:
                    c[0], c[1], c[2], c[4] = target, c[1] + 1, (c[2] + 1 if strict else 0), now
                else:
                    c = None
            if c is None:
                c = self.cand = [target, 1, 1 if strict else 0, now, now, self._centre_index(res)]
                self.cands[c[5]] = c
            # the way out for a changed room: the looser matches must agree with the pose as it is (a small fix); a
            # consistent but far pose that never fits strictly is a wrong one (a wall snapped onto its neighbour)
            near = math.hypot(res["dx"], res["dy"]) <= ENV_MAX_MM and abs(res["dth"]) <= ENV_MAX_DEG
            if c[2] >= LOCK_CONFIRM or (near and c[1] >= ENV_CONFIRM and now - c[3] >= ENV_MIN_S):
                how = ("%d matches agree" % c[2]) if c[2] >= LOCK_CONFIRM else \
                    "%d looser matches agree over %.0f s: re-anchored, the room may have changed" % (c[1], now - c[3])
                d = math.hypot(res["dx"], res["dy"])
                lim = 3.0 * S["match_step_deg"]
                step = self._apply(res["dx"], res["dy"], max(-lim, min(lim, res["dth"])), mx, my)
                self._found(now, "%s, moved %d mm" % (how, round(d)), remap=c[2] < LOCK_CONFIRM)
                return step
            res["why"] = "%s match %d of %d" % ("strict" if strict else "loose", c[2] if strict else c[1],
                                                LOCK_CONFIRM if strict else ENV_CONFIRM)
            return None
        c = self.cands.get(self._centre_index(res))
        if c is not None:
            c[2] = 0               # strict matches must come in a row; the looser count goes on (a changed room also
            #                        gives "ambiguous" now and then): a candidate expires after CAND_STALE_S instead
        if self.tries >= WIDE_TRIES and self.level < WIDE_LEVELS:
            self.level += 1
            self.tries = 0
        return None

    def trust_pose(self, who="?"):
        """Lost, by hand (map menu "Trust the pose"): take the pose as it is and map from here."""
        if self.mode != "lost":
            return False
        self.reloc = None
        self._found(self._now(), "%s: the pose was trusted as it is" % who, remap=True)
        return True

    def on_pose_reset(self):
        """The robot's odometry went back to (0, 0, 0) where it stands: with a recent lidar pose, the correction keeps
        it at that map pose (else the map would see it jump to the old origin and stay lost)."""
        lp_ = self.last_pose
        if self.job is not None:     # a search on the old odometry frame: its result would be wrong now
            self.gen += 1
            self.job = None
            self.glob_running = False
        self.prev_odo = None
        self.obs.clear()
        self.wheels.clear()
        self.suspect = None
        self.prev_h = None
        if self.mode == "slip":
            self.mode = "track"
        if not self.m.settings.get("lidar_match") or not self.m.settings.get("match_slip", True):
            return False
        if lp_ is None or self._now() - lp_[3] > 1.0:
            # no recent lidar pose to keep (the lidar was off / the Teensy rebooted, which also stops its LD19 bytes):
            # with a map, find the robot on it (#105)
            if self.field.walls >= MIN_WALL_CELLS:
                self.request_global("pose reset with no recent lidar pose", by="reset")
            return False
        self.m.corr = [lp_[0], lp_[1], lp_[2]]
        self.raw.clear()
        self.jump = None
        for f in self.m.corr_listeners:
            f()
        return True

    # ------------------------------------------------------------ #105: relocalisation, Set position
    def request_global(self, why, by="auto", around=None):
        """Search the whole map (around: (x, y, half_mm, half_deg) only there) with the next turn: lost (not mapped)
        until the pose is found and confirmed. by: "auto" | "reset" | "load" | "user" | "set"."""
        self.glob_req = {"why": why, "by": by, "around": around}
        self.prev_h = None                         # the map heading may jump: not "turning fast"
        if by != "set":
            self.reloc = {"phase": "pending", "by": by, "why": why, "t": self._now()}
        self.m.say("lidar: %s - searching the whole map for the robot" % why if around is None else
                   "lidar: %s - matching the scan around it" % why)

    def relocalize(self, who="?"):
        """"Find myself" (map menu / API): a whole-map search now. Returns an error text or None."""
        S = self.m.settings
        if self.m.lidar is None:
            return "no lidar: nothing to search the map with"
        if not (S.get("lidar_match") and S.get("match_slip", True)):
            return "lidar scan matching (and its wheel slip handling) is off"
        if self.field.grid is not None and self.field.walls < MIN_WALL_CELLS:
            return "the map is too small to search (%d wall cells)" % self.field.walls
        if self.job is not None and self.glob_running:
            return None                            # one is running already
        self.request_global("%s: find myself" % who, by="user")
        return None

    def set_pose(self, x, y, th, who="?", odom=None):
        """Set position (map menu tool / API): the robot is at (x, y) mm, heading th deg, map frame. The correction is set
        so the current odometry pose maps there at once; with LD19 matching the pose is then matched in a window around
        it (SET_BOX_MM / SET_DEG: a tap is not exact) and confirmed by 2 strict matches before anything is mapped
        again; a poor fit keeps the user's pose, unconfirmed (the map info line says so). odom: the robot's odometry
        (x, y, heading) now (the bridge's telemetry), else the lidar's pose history. Returns an error text or None."""
        if odom is None:
            hist = getattr(self.m.lidar, "hist", None)
            items = hist.items() if hist is not None else []
            if items:
                odom = items[-1][1:4]
            elif self.last_pose is not None:
                lx, ly, lh = self.last_pose[:3]
                odom = inverse(self.m.corr, lx, ly) + (lh - self.m.corr[2],)
        if odom is None:
            return "no robot pose yet (no telemetry)"
        x, y, th = float(x), float(y), float(th)
        if not all(math.isfinite(v) for v in (x, y, th)):
            return "set_pose: x, y, th must be numbers"
        ox, oy, oh = (float(v) for v in odom[:3])
        if self.job is not None:     # a search still running would overwrite the user's pose with its own: dropped
            self.gen += 1
            self.job = None
            self.glob_running = False
        dt = th - oh
        c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))
        self._set_corr([x - (c * ox - s * oy), y - (s * ox + c * oy), dt])
        S = self.m.settings
        now = self._now()
        self.reloc = {"phase": "set", "by": "set", "who": who, "t": now, "pose": [round(x), round(y), round(th, 1)]}
        if self.m.lidar is None or not (S.get("lidar_match") and S.get("match_slip", True)):
            self.reloc["phase"] = "found"         # nothing to check it with: taken as it is
            self.m.say("%s: position set to %d, %d mm, %d deg" % (who, round(x), round(y), round(th)))
            return None
        if self.mode != "lost":
            self._lose(now, "position set", quiet=True)
        else:
            self.lost_t0, self.checking = now, False
        self._confirm_start(now)
        self.request_global("%s: position set to %d, %d mm, %d deg" % (who, round(x), round(y), round(th)), by="set",
                            around=(x, y, SET_BOX_MM, SET_DEG))
        return None

    def _set_corr(self, corr):
        """A new correction from relocalisation / Set position (a jump, not a filtered step)."""
        m = self.m
        if m.places.origin is None:               # the default Home (the odometry origin) stays where it is in the map
            m.places.origin = [round(v, 1) for v in m.map_pose(0.0, 0.0, 0.0)[:2]]
        m.corr = [float(v) for v in corr]
        self.raw.clear()
        self.jump = None
        self.prev_h = None
        self.obs.clear()
        self.last_fix = None
        self.last_pose = None
        for f in m.corr_listeners:
            f()

    def _confirm_start(self, now):
        """Lost, a pose to confirm (found / set): the normal window around it again, no old candidates."""
        self.level, self.tries = 0, 0
        self.cand, self.cands = None, {}
        self.offs, self.off_i = [(0.0, 0.0)], 0
        self.confirm_t0 = now

    def _confirm_check(self, now):
        rl = self.reloc
        if rl is not None and self.confirm_t0 is not None and now - self.confirm_t0 > CONFIRM_S and \
                rl["phase"] in ("confirming", "set"):
            rl["phase"] = "unconfirmed" if rl["phase"] == "confirming" else "set_poor"
            self.confirm_t0 = None
            self.m.say("lidar: the pose %s was not confirmed by the scans within %.0f s: still not mapping"
                       % ("found" if rl["phase"] == "unconfirmed" else "set by hand", CONFIRM_S))

    def _reset_seen(self, prev, odo, oh, S):
        """The odometry jumped to ~(0, 0, 0) from somewhere else: a Teensy reboot / unlogged pose reset (#105)."""
        if not S.get("match_global", True) or self.field.walls < MIN_WALL_CELLS or len(prev) < 4:
            return False
        d = math.hypot(odo[0] - prev[0], odo[1] - prev[1])
        dh = abs((oh - prev[3] + 180.0) % 360.0 - 180.0)
        near = math.hypot(odo[0], odo[1]) < RESET_NEAR_MM and abs((oh + 180.0) % 360.0 - 180.0) < RESET_NEAR_DEG
        return near and (d > ODO_JUMP_MM or dh > 45.0)

    def _global_due(self, now, S):
        if self.glob_req is not None:
            return True
        if not S.get("match_global", True) or self.checking or self.m.lidar is None:
            return False
        if self.reloc is not None and self.reloc["phase"] in ("confirming", "set"):
            return False
        if now - self.lost_t0 < GLOBAL_AFTER_S:
            return False
        return self.glob_t is None or now >= self.glob_t + GLOBAL_RETRY_S

    def _pyr_key(self):
        g = self.m.grid
        return id(g), id(g.cells), self.field.ver, len(g.touched | g.dirty)

    def _global_start(self, runs, mx, my, now, S):
        """Start a whole-map (or Set position window) search with this turn: in the worker thread under the bridge,
        right here without a loop (tests, offline sims). False: not started (map too small)."""
        from . import reloc
        req, self.glob_req = self.glob_req or {"why": "lost for %.0f s" % (now - self.lost_t0), "by": "auto",
                                               "around": None}, None
        self.checking = False                      # (a gap check overtaken by a reboot / request: really lost now)
        P = getattr(self.m, "prof", loopprof.NULL)
        with P.part("match.field"):
            self.field.sync(self.m.grid)
        if self.field.walls < MIN_WALL_CELLS:
            if req["by"] != "set":
                self.reloc = {"phase": "none", "by": req["by"], "why": "map too small", "t": now}
            self.glob_t = now
            return False
        with P.part("match.global"):
            fz = reloc.Frozen(self.field)          # (copies: the loop may change the real field meanwhile)
        key = self._pyr_key()
        pyr = self.pyr[1] if self.pyr is not None and self.pyr[0] == key else None
        around = req.get("around")
        accept = SET_ACCEPT if around is not None else None
        corr0, gen = list(self.m.corr), self.gen
        min_curv = S["match_min_curv"]
        prof = getattr(self.m, "prof", loopprof.NULL)
        if req["by"] != "set":
            self.reloc = {"phase": "searching", "by": req["by"], "why": req["why"], "t": now}
        self.state = "set position: matching the scan around it" if around is not None else \
            "lost: searching the whole map"
        self.glob_running = True

        def work(y=reloc._nothing):
            with prof.thread("match.global"):
                try:
                    p = pyr or reloc.Pyramid(fz, y)
                    r = reloc.find(fz, p, runs, mx, my, around, min_curv, accept, y)
                    r["pyr_ms"] = 0.0 if pyr is not None else round(p.ms, 1)
                    return r, p
                except reloc.Cancelled:
                    return None, None
                except Exception as e:      # a search bug must never stop the bridge
                    return {"ok": False, "why": "error: %s: %s" % (type(e).__name__, e)}, None
        loop = self._loop()
        if loop is None:
            res, p = work()
            self._global_done(res, p, key, gen, req, mx, my, corr0)
            return True

        share = self._worker_share()

        def run():
            return work(reloc.Yielder(cancel=lambda: gen != self.gen, share=share))

        def done(fut):              # (worker thread)
            try:
                res, p = fut.result()
            except Exception as e:
                res, p = {"ok": False, "why": "error: %s: %s" % (type(e).__name__, e)}, None
            try:
                loop.call_soon_threadsafe(self._global_done, res, p, key, gen, req, mx, my, corr0)
            except RuntimeError:    # the loop is gone (shutdown)
                pass
        self.job = _pool().submit(run)
        self.job.add_done_callback(done)
        return True

    def _global_done(self, res, pyr, key, gen, req, mx, my, corr0):
        """(loop) A global search result: a clear, unique pose -> the correction jumps there and the matcher confirms it
        (2 strict matches) before mapping; else the state says why (ambiguous: use Set position)."""
        if gen != self.gen:
            return                  # reset / another map meanwhile
        self.job = None
        self.glob_running = False
        if res is None:
            return
        if pyr is not None:
            self.pyr = (key, pyr)
        now = self._now()
        S = self.m.settings
        self.n["global"] += 1
        self.last_global = res
        by, around = req["by"], req.get("around")
        self.glob_t = now + (SET_HOLD_S if by == "set" else 0.0)
        if self.mode != "lost" or not S.get("lidar_match"):
            return                  # trusted by hand / switched off meanwhile
        info = {"score": res.get("score"), "margin": res.get("margin"), "ms": res.get("ms"), "q": res.get("q"),
                "cands": len(res.get("cands") or ())}
        if res["ok"]:
            # rotate dth about where the robot was (mx, my) in the turn's frame (corr0), then shift it to (x, y)
            target = compose((res["x"] - mx, res["y"] - my, res["dth"]), corr0, mx, my)
            moved = math.hypot(res["x"] - mx, res["y"] - my)
            self._set_corr(target)
            self._confirm_start(now)
            rl = self.reloc if by == "set" and self.reloc else {}
            rl.update(info, phase="confirming", by=by, why=req["why"], t=now, relaxed=bool(res.get("relaxed")))
            self.reloc = rl
            self.state = "found on the map: confirming"
            self.m.say("lidar: %s at %d, %d mm (moved %d mm, %+.0f deg; fit %d %%%s, margin %d %%, %.1f s): confirming"
                       % ("pose matched" if by == "set" else "found on the map", round(res["x"]), round(res["y"]),
                          round(moved), res["dth"], round(100 * res["score"]),
                          " - low but unique" if res.get("relaxed") else "", round(100 * res["margin"]),
                          res["ms"] / 1000.0))
            return
        why = res.get("why") or "no fit"
        if by == "set":
            rl = self.reloc or {"by": "set"}
            rl.update(info, phase="set_poor", why=why, t=now)
            self.reloc = rl
            self.state = "set position: %s - kept, not mapping" % why
            self.m.say("lidar: the scan does not fit the map well around the position set (%s, fit %d %%): kept, "
                       "not mapping until a match confirms it" % (why, round(100 * (res.get("score") or 0))))
            return
        phase = "ambiguous" if why == "ambiguous" else "none"
        self.reloc = dict(info, phase=phase, by=by, why=why, t=now)
        self.state = "lost: ambiguous - use Set position" if phase == "ambiguous" else \
            "lost: not found on the map (%s)" % why
        self.m.say("lidar: whole-map search: %s (best fit %d %%, margin %d %%, %d candidates): %s" % (
            why, round(100 * (res.get("score") or 0)), round(100 * (res.get("margin") or 0)), info["cands"],
            "use Set position" if phase == "ambiguous" else "still lost, searching again in %.0f s" % GLOBAL_RETRY_S))

    def _reloc_stats(self, now):
        rl = self.reloc
        if rl is None:
            return None
        if rl["phase"] == "found" and now - rl.get("t", now) > FOUND_SHOW_S:
            self.reloc = None
            return None
        out = {k: rl.get(k) for k in ("phase", "by", "why", "score", "margin", "ms", "cands")}
        out["age_s"] = round(now - rl.get("t", now), 1)
        out["running"] = self.glob_running
        return out

    # ------------------------------------------------------------ browsers
    def _state_text(self, step, res):
        if self.mode == "slip":
            return "wheel slip: holding the position (grip %d %%)" % round(100 * min(1.0, self.grip or 0.0))
        if self.mode == "lost":
            w = res.get("window") or [0, 0]
            ph = (self.reloc or {}).get("phase")
            if ph in ("confirming", "set", "set_poor", "unconfirmed"):    # #105: a pose found / set, being confirmed
                lead = {"confirming": "found on the map: confirming", "set": "set position: confirming",
                        "set_poor": "set position: poor fit, not mapping",
                        "unconfirmed": "found on the map: not confirmed"}[ph]
                return lead + " (%s)" % res["why"] if res.get("why") else lead
            if ph == "ambiguous":
                return "lost: ambiguous - use Set position (searching +-%d mm / %.0f deg)" % (w[0], w[1])
            if ph == "none":
                return "lost: not found on the map (searching +-%d mm / %.0f deg)" % (w[0], w[1])
            return ("checking the pose after a lidar gap: +-%d mm / %.0f deg" if self.checking else
                    "lost: searching +-%d mm / %.0f deg") % (w[0], w[1]) + (" (%s)" % res["why"] if res.get("why") else "")
        return ("matched" if any(step) else "matched: pose fine") if step else "kept: " + res["why"]

    def stats(self):
        """For the map's stats line (browsers): state, last score, correction, cost."""
        L = self.last or {}
        h = self.history
        now = self._now() if self.m.lidar is not None else 0.0
        return {"on": bool(self.m.settings.get("lidar_match")), "lidar": self.m.lidar is not None,
                "state": self.state, "score": L.get("score"), "fix": L.get("fix"), "applied": L.get("applied"),
                "ok_pct": round(100.0 * sum(h) / len(h)) if h else None, "ms": round(self.cost_avg_ms, 1),
                "matched": self.n["matched"], "rejected": self.n["rejected"], "jumps": self.n["jumps"],
                "moved_mm": round(self.moved[0]), "moved_deg": round(self.moved[1], 1),
                "mode": self.mode, "grip": None if self.grip is None else round(min(1.0, self.grip), 2),
                "slip_s": round(now - self.slip_t0, 1) if self.mode == "slip" else None,
                "lost_s": round(now - self.lost_t0, 1) if self.mode == "lost" else None,
                "window": [round(v, 1) for v in self._wide(self.level, self.m.settings)] if self.mode == "lost" else None,
                "slips": self.n["slips"], "lost": self.n["lost"], "found": self.n["found"], "stuck": self.n["stuck"],
                "checking": self.checking, "checks": self.n["checks"],
                "reloc": self._reloc_stats(self._now()), "global": self.n["global"], "relocated": self.n["relocated"]}
