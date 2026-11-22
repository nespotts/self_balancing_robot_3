"""Scan-to-map pose correction. Map pose = corr o odometry pose (Mapper.corr), so the map stays put and
the browser shifts it under the robot.

Matcher (issue #12, setting "correct", off by default): continuous. Readings are held back for one sweep
(~32 readings, ~1.3 s). The sweep's hit points are then tried against the grid at a small set of pose
offsets around the robot (dx, dy in +-25 mm, dtheta in +-1 deg); the offset whose hits land best on
occupied cells (3x3-ish smoothed log-odds, small penalty for moving) updates Mapper.corr, and the sweep
is integrated with the new pose. Did not beat odometry while driving (the sweep smears).

StopScan (issue #71, setting "stop_scan"): only while the robot stands still. Readings are held back
while the pose stays put; after two full sweeps (SCAN_N readings, ~2.6 s) they are matched against the
map built so far with match_scan() (correlative search over x / y / theta on a distance field, every
reading scored on the arc its ~25 deg cone could have hit), and a correction is applied only with strong
evidence: enough points on known walls, a clear best pose (an ambiguous scan, e.g. a long featureless
corridor, is rejected) and a bounded size (MAX_XY_MM / MAX_TH_DEG). Then the readings go into the map with
the corrected pose. Triggers: any stop (HOLD, arrived, dwelling, explore pauses at frontier goals) while
"stop_scan" is on, or an explicit request (the Map's "Scan & correct", works with it off).
"""
import bisect
import collections
import itertools
import math
import time

SWEEP_N = 32               # readings per match (one full sweep with two sensors at 16 steps)
WARMUP_SWEEPS = 3          # first sweeps just build the map
MIN_HITS = 12
XY_STEPS = (-25, 0, 25)
TH_STEPS = (-1.0, 0.0, 1.0)
MOVE_PENALTY = 6.0         # score per mm of offset (keeps it still when the match is flat)
MIN_GAIN = 200             # score gain over "no change" needed to move (+ 5 % of |base|)

# log-odds byte -> score: only occupied evidence counts (hits taken at a slant land in front of the
# wall, in cells other readings saw free: penalising those pulls the match off)
SCORE = [max(0, v - 128) for v in range(256)]


class Matcher:
    def __init__(self, mapper):
        self.m = mapper
        self.reset()

    def reset(self):
        self.buf = []
        self.sweeps = 0
        self.last = None           # {"dx", "dy", "dth", "gain", "hits"} of the last match (stats)
        self.applied = 0

    def add(self, d):
        self.buf.append(d)
        if len(self.buf) >= SWEEP_N:
            self._match()
            self.flush()

    def flush(self):
        buf, self.buf = self.buf, []
        for d in buf:
            self.m.integrate(d)
        if buf:
            self.sweeps += 1

    def _hits(self):
        """Hit points (map frame, current correction) of the buffered sweep."""
        m = self.m
        S = m.settings
        pts = []
        for d in self.buf:
            r = float(d["range_mm"])
            if not d["valid"] or r >= 8000 or r > S["max_hit_mm"]:
                continue
            x, y, hd = m.map_pose(d["x_mm"], d["y_mm"], d["heading_deg"])
            h = math.radians(hd)
            a = h + math.radians(d["sensor_angle_deg"])
            rr = r + 12.9
            pts.append((x + 26.0 * math.cos(h) + rr * math.cos(a), y + 26.0 * math.sin(h) + rr * math.sin(a)))
        return pts

    def _match(self):
        if self.sweeps < WARMUP_SWEEPS:
            return
        pts = self._hits()
        if len(pts) < MIN_HITS:
            return
        m, g = self.m, self.m.grid
        last = self.buf[-1]
        cx, cy, _ = m.map_pose(last["x_mm"], last["y_mm"], last["heading_deg"])
        cells, W, res, x0, y0 = g.cells, g.W, g.res, g.x0, g.y0
        n = len(cells)
        S = SCORE

        def score(dx, dy, c, s):
            tot = 0
            for px, py in pts:
                qx, qy = px - cx, py - cy
                ix = int((c * qx - s * qy + cx + dx - x0) // res)
                iy = int((s * qx + c * qy + cy + dy - y0) // res)
                i = iy * W + ix
                if W < i < n - W - 1:
                    tot += 2 * S[cells[i]] + S[cells[i - 1]] + S[cells[i + 1]] + S[cells[i - W]] + S[cells[i + W]]
            return tot

        base = score(0, 0, 1.0, 0.0)
        best = (base, 0, 0, 0.0)
        for th in TH_STEPS:
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            for dx in XY_STEPS:
                for dy in XY_STEPS:
                    if not (dx or dy or th):
                        continue
                    v = score(dx, dy, c, s) - MOVE_PENALTY * (abs(dx) + abs(dy) + 25 * abs(th))
                    if v > best[0]:
                        best = (v, dx, dy, th)
        v, dx, dy, th = best
        self.last = {"dx": dx, "dy": dy, "dth": th, "gain": round(v - base), "hits": len(pts)}
        if v - base < MIN_GAIN + 0.05 * abs(base) or v <= 0:
            self.last["dx"] = self.last["dy"] = self.last["dth"] = 0
            return
        # new corr = delta o corr, delta = rotate th about (cx, cy) then shift (dx, dy)
        tx, ty, tt = m.corr
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        qx, qy = tx - cx, ty - cy
        m.corr = [c * qx - s * qy + cx + dx, s * qx + c * qy + cy + dy, tt + th]
        self.applied += 1


# ====================================================================== stop-and-scan (#71)
# Beam model: a VL53L0X reading is the distance to the NEAREST wall anywhere in its ~25 deg cone. For every
# candidate robot position the map is ray-cast from the sensor axis in 1 deg steps (polar range table), the
# table's sliding minimum over +-CONE_HALF_DEG is the range each reading should have seen, and a pose scores
# sum(exp(-(r - expected)^2 / 2 sigma^2)). Rotation = shifting the table by whole / interpolated degrees.
# (A plain point-to-wall distance field cannot tell where in the cone the hit was: it lets the scan slide
# +-10 deg along every wall, and it found neither the heading nor the position in the sim.)
RAY_STOP = 45              # a ray stops where its summed occupancy evidence (cell byte - 128, occupied cells only)
                           # reaches this: one sure wall cell, or the middle of a band of weak ones (walls seen from
                           # afar are weak; cone smear in front of walls seen at a slant too: a fixed threshold
                           # either misses the first or stops at the second)
CONE_HALF_DEG = 8          # half the effective beam width (VL53L0X field of view ~25 deg; weak far targets: less)
SIGMA_MM = 25.0            # range residual width: map cell + noise ...
SIGMA_FRAC = 0.02          # ... + this fraction of the range
MAX_XY_MM = 150.0          # search window = largest correction (bounded size)
MAX_TH_DEG = 8.0
STEP_XY_MM = 50.0          # coarse steps; then 12.5 mm / 0.25 deg around the best, then a parabola fit
STEP_TH_DEG = 1.0
FAR_XY_MM, FAR_TH_DEG = 80.0, 2.5   # "another pose" for the ambiguity margin
MIN_POINTS = 20            # valid hits in the scan
MIN_INLIERS = 16           # ... of which this many fit the map within 2 sigma (and >= MIN_INLIER_FRAC of them)
MIN_INLIER_FRAC = 0.5
MIN_MARGIN = 0.08          # best score must beat any pose FAR away by this fraction (corridor: ~0)
MIN_GAIN = 0.02            # ... and the zero correction by this fraction (else: pose already fine)
MIN_APPLY_MM, MIN_APPLY_DEG = 25.0, 0.75 # smaller corrections are not applied: about the matcher's own error
                                         # (sim: ~20 mm, ~1 deg); below it a correction adds noise, not accuracy
AXIS_FWD_MM, SENSOR_R_MM = 26.0, 12.9    # servo axis ahead of the robot centre; sensor out from the axis
RAY_BIAS = 0.5             # cells: a ray stops where it enters a wall cell, the wall is ~ half a cell further

_RAYS = {}                 # (w, n_cells) -> per degree: (flat cell offsets, distances in cells) at half-cell steps


def _rays(w, n):
    key = (w, n)
    if key not in _RAYS:
        out = []
        for deg in range(360):
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            offs, dist, last = [], [], None
            for k in range(1, 2 * n + 1):
                q = 0.5 * k
                o = int(math.floor(0.5 + q * s)) * w + int(math.floor(0.5 + q * c))
                if o != last:
                    offs.append(o)
                    dist.append(q)
                    last = o
            out.append((offs, dist))
        if len(_RAYS) > 8:
            _RAYS.clear()
        _RAYS[key] = out
    return _RAYS[key]


class ScanScorer:
    """A scan around ref against the map: score(th, dx, dy) (higher = better), see the beam model above."""

    def __init__(self, grid, scan, ref, heading_deg):
        self.n = len(scan)
        self.cx, self.cy = ref
        res = self.res = grid.res
        h = math.radians(heading_deg)
        self.ax, self.ay = self.cx + AXIS_FWD_MM * math.cos(h), self.cy + AXIS_FWD_MM * math.sin(h)
        # per reading: beam direction (deg, map frame), range from the axis, axis offset projected on the beam
        self.rd = []
        rmax = 0.0
        for ox, oy, a, r in scan:
            ca, sa = math.cos(a), math.sin(a)
            bx, by = ox - SENSOR_R_MM * ca, oy - SENSOR_R_MM * sa       # this reading's servo axis
            proj = (bx - self.ax) * ca + (by - self.ay) * sa              # its offset along the beam
            self.rd.append((math.degrees(a), r + SENSOR_R_MM + proj, SIGMA_MM + SIGMA_FRAC * r))
            rmax = max(rmax, r)
        self.reach = rmax + SENSOR_R_MM + 3 * SIGMA_MM + 50.0              # rays this long
        n = self.ncell = int(math.ceil(self.reach / res)) + 1
        box = n + int(math.ceil((MAX_XY_MM + 40.0) / res)) + 2
        # local occupancy window (1 = wall) centred on the axis
        gi = int(math.floor((self.ax - grid.x0) / res)) - box
        gj = int(math.floor((self.ay - grid.y0) / res)) - box
        self.w = w = 2 * box + 1
        self.wx0, self.wy0 = grid.x0 + gi * res, grid.y0 + gj * res
        lut = bytes(max(0, v - 128) for v in range(256))
        occ = bytearray(w * w)
        cells, GW, GH = grid.cells, grid.W, grid.H
        for j in range(w):
            gy = gj + j
            if 0 <= gy < GH:
                a, b = max(0, -gi), min(w, GW - gi)
                if a < b:
                    occ[j * w + a:j * w + b] = bytes(cells[gy * GW + gi + a:gy * GW + gi + b]).translate(lut)
        self.occ = bytes(occ)
        self.rays = _rays(w, n)
        self.tables = {}

    def table(self, dx, dy):
        """Expected range (mm from the axis) per degree for the axis moved by (dx, dy): nearest wall in the cone."""
        key = (round(dx, 1), round(dy, 1))
        t = self.tables.get(key)
        if t is not None:
            return t
        res, w, occ = self.res, self.w, self.occ
        X, Y = self.ax + dx, self.ay + dy
        fi, fj = (X - self.wx0) / res, (Y - self.wy0) / res
        i0, j0 = int(fi), int(fj)
        o0 = j0 * w + i0
        ex, ey = (fi - i0 - 0.5) * res, (fj - j0 - 0.5) * res         # true origin vs the cell centre
        inf = 1e9
        polar = []
        T = RAY_STOP
        for deg, (offs, dist) in enumerate(self.rays):
            k = bisect.bisect_left(list(itertools.accumulate([occ[o0 + o] for o in offs])), T)
            if k >= len(offs):
                polar.append(inf)
            else:
                c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
                polar.append((dist[k] + RAY_BIAS) * res - (ex * c + ey * s))
        C = int(CONE_HALF_DEG)
        ext = polar[-C:] + polar + polar[:C] if C else polar
        t = [min(ext[d:d + 2 * C + 1]) for d in range(360)]
        self.tables[key] = t
        return t

    def score(self, th, dx=0.0, dy=0.0, detail=False):
        t = self.table(dx, dy)
        tot, inl = 0.0, 0
        exp = math.exp
        for deg, r, sg in self.rd:
            q = deg + th
            b = math.floor(q)
            f = q - b
            b = int(b) % 360
            e0, e1 = t[b], t[(b + 1) % 360]
            e = e0 + f * (e1 - e0) if e0 < 1e8 and e1 < 1e8 else min(e0, e1)
            z = (r - e) / sg
            if z * z < 18.0:
                tot += exp(-0.5 * z * z)
                if z * z < 4.0:
                    inl += 1
        return (tot, inl) if detail else tot


def match_scan(grid, scan, ref, heading_deg=0.0):
    """Correlative scan-to-map match. scan: [(ox, oy, a_rad, r_mm)] sensor position + beam angle in the map
    frame (current correction), valid hits only; ref: (x, y) the rotation centre (robot), heading_deg its
    map heading. Returns a dict: ok (apply it), why (rejection reason), dx, dy (mm), dth (deg: rotate about
    ref, then shift), n, inliers, margin, gain, score (0..1 at the best pose), ms (time taken).
    Search: coarse grid over the whole window, then a fine grid around the best, then a parabola fit."""
    t0 = time.perf_counter()
    out = {"ok": False, "why": "", "dx": 0.0, "dy": 0.0, "dth": 0.0, "n": len(scan), "inliers": 0,
           "margin": 0.0, "gain": 0.0, "score": 0.0}

    def done(why):
        out["why"] = why
        out["ms"] = round((time.perf_counter() - t0) * 1000.0)
        return out
    if len(scan) < MIN_POINTS:
        return done("too few readings (%d)" % len(scan))
    sc = ScanScorer(grid, scan, ref, heading_deg)
    n = sc.n
    nxy = int(round(MAX_XY_MM / STEP_XY_MM))
    nth = int(round(MAX_TH_DEG / STEP_TH_DEG))
    coarse = []                        # (score, dx mm, dy mm, th)
    for kx in range(-nxy, nxy + 1):
        for ky in range(-nxy, nxy + 1):
            dx, dy = kx * STEP_XY_MM, ky * STEP_XY_MM
            for kt in range(-nth, nth + 1):
                coarse.append((sc.score(kt * STEP_TH_DEG, dx, dy), dx, dy, kt * STEP_TH_DEG))
    best = max(coarse)
    zero_s = sc.score(0.0)
    # ambiguity, heading and position apart: the best pose with another heading (any position) / at another
    # position (any heading). A corridor pins the heading but not the position along it.
    far_th = max((c[0] for c in coarse if abs(c[3] - best[3]) > FAR_TH_DEG), default=0.0)
    far_xy = max((c[0] for c in coarse if math.hypot(c[1] - best[1], c[2] - best[2]) > FAR_XY_MM), default=0.0)
    fs, ft = STEP_XY_MM / 4.0, STEP_TH_DEG / 4.0

    def para(lo, mid, hi):             # sub-step peak offset of a parabola through three samples
        den = lo - 2 * mid + hi
        return 0.0 if den >= 0 else max(-0.5, min(0.5, 0.5 * (lo - hi) / den))

    def refine(c, turn=True, move=True):
        """Fine grid (a quarter of the coarse steps, +-2 of them) around coarse pose c, then a parabola fit:
        (score, dx, dy, dth, inliers). turn / move False: that part stays as in c."""
        fine = {}
        for i in (range(-2, 3) if move else (0,)):
            for j in (range(-2, 3) if move else (0,)):
                for kt in (range(-3, 4) if turn else (0,)):
                    fine[(kt, i, j)] = sc.score(c[3] + ft * kt, c[1] + fs * i, c[2] + fs * j)
        kb = max(fine, key=fine.get)
        bs = fine[kb]
        kt, i, j = kb
        g = fine.get
        th = c[3] + ft * (kt + (para(g((kt - 1, i, j), bs), bs, g((kt + 1, i, j), bs)) if turn else 0.0))
        x = c[1] + fs * (i + (para(g((kt, i - 1, j), bs), bs, g((kt, i + 1, j), bs)) if move else 0.0))
        y = c[2] + fs * (j + (para(g((kt, i, j - 1), bs), bs, g((kt, i, j + 1), bs)) if move else 0.0))
        tot, inl = sc.score(c[3] + ft * kt, c[1] + fs * i, c[2] + fs * j, detail=True)
        return bs, x, y, th, inl, tot

    mth = (best[0] - far_th) / best[0] if best[0] > 0 else 0.0
    mxy = (best[0] - far_xy) / best[0] if best[0] > 0 else 0.0
    out["margin"] = round(min(mth, mxy), 3)
    out["margin_th"], out["margin_xy"] = round(mth, 3), round(mxy, 3)
    if best[0] <= 0:
        return done("no fit")
    turn, move = mth >= MIN_MARGIN, mxy >= MIN_MARGIN
    if turn and move:
        c, part = best, ""
    elif move:
        # position clear, heading not (the wide beam often leaves it loose): the position at the odometry
        # heading only, if the best pose there agrees with the overall best
        c = max(k for k in coarse if k[3] == 0.0)
        part = "ambiguous heading (margin %.0f %%): position only" % (100 * mth)
        if math.hypot(c[1] - best[1], c[2] - best[2]) > STEP_XY_MM:
            return done("ambiguous heading (margin %.0f %%)" % (100 * mth))
    elif turn:
        # heading clear, position not (a corridor): the heading at the odometry position only, if it agrees
        c = max(k for k in coarse if k[1] == 0 and k[2] == 0)
        part = "ambiguous position (margin %.0f %%): heading only" % (100 * mxy)
        if abs(c[3] - best[3]) > 1.5 * STEP_TH_DEG:
            return done("ambiguous position (margin %.0f %%)" % (100 * mxy))
    else:
        return done("ambiguous (margin %.0f %%)" % (100 * out["margin"]))
    bs, dx, dy, dth, inl, tot = refine(c, turn=turn, move=move)
    out.update(dx=round(dx, 1), dy=round(dy, 1), dth=round(dth, 2), inliers=inl, score=round(tot / n, 3))
    out["gain"] = round((bs - zero_s) / bs, 3) if bs > 0 else 0.0
    if part:
        out["part"] = part
    if inl < max(MIN_INLIERS, MIN_INLIER_FRAC * n):
        return done("too few readings fit the map (%d of %d)" % (inl, n))
    if abs(dx) > MAX_XY_MM - 10 or abs(dy) > MAX_XY_MM - 10 or abs(dth) > MAX_TH_DEG - 0.5:
        out.update(dx=max(-MAX_XY_MM, min(MAX_XY_MM, out["dx"])), dy=max(-MAX_XY_MM, min(MAX_XY_MM, out["dy"])),
                   dth=max(-MAX_TH_DEG, min(MAX_TH_DEG, out["dth"])))
        return done("correction at the search limit")
    if out["gain"] < MIN_GAIN or (math.hypot(dx, dy) < MIN_APPLY_MM and abs(dth) < MIN_APPLY_DEG):
        return done("pose already fine" + (" (%s)" % part if part else ""))
    out["ok"] = True
    return done(part)

SCAN_N = 64                # readings per stop-and-scan: two full sweeps with both sensors (~2.6 s)
STILL_MM, STILL_DEG = 60.0, 4.0   # standing still = within this of where it stopped (it sways +-25 mm balancing;
                                  # every reading carries its own odometry pose, so a little creep does not matter)
PITCH_SPREAD_DEG = 2.0     # scan readings this far from the scan's median pitch are not matched
REQUEST_GIVE_UP = 300      # readings (~12 s) an explicit request waits for the robot to stand still
HISTORY_N = 20


def thread_runner(fn, cb):
    """Run fn() in a worker thread when an asyncio loop is running (the bridge: the match must not stall
    the 10 Hz heartbeat), else inline (offline sim, tests); cb(result or exception) runs on the caller's side."""
    import asyncio
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = None
    if loop is None:
        try:
            r = fn()
        except Exception as e:     # a matcher bug must never take the bridge down
            r = e
        cb(r)
        return
    fut = loop.run_in_executor(None, fn)
    fut.add_done_callback(lambda f: cb(f.exception() or f.result()))


class StopScan:
    """Stop-and-scan pose correction (see the module doc). Mapper routes readings here while "stop_scan" is
    on or a request is pending; add() holds them back while the robot stands still and integrates them after
    the match (or as soon as it moves)."""

    def __init__(self, mapper, runner=thread_runner):
        self.m = mapper
        self.runner = runner
        self.listeners = []        # callables(entry): a correction was applied (planner: resend the route)
        self.history = collections.deque(maxlen=HISTORY_N)
        self.gen = 0
        self.reset()

    def reset(self):
        self.gen += 1              # a match running for the old map is dropped
        self.buf = []
        self.anchor = None         # odometry pose where the robot stopped
        self.done = False          # this stop has been matched
        self.matching = False
        self.request = None        # who asked for a scan (explicit), or None
        self.req_wait = 0
        self.applied = 0
        self.seq = 0               # bumps with every history entry
        self.last = None
        self.history.clear()

    def busy(self):
        return bool(self.buf or self.request or self.matching)

    def ask(self, who="?"):
        """Explicit "Scan & correct": match the next two sweeps taken standing still (also with stop_scan off)."""
        if self.matching:
            return
        self.flush()
        self.anchor, self.done = None, False
        self.request, self.req_wait = who, 0

    def flush(self):
        if self.matching:
            return
        buf, self.buf = self.buf, []
        for d in buf:
            self.m.integrate(d)

    def add(self, d):
        if self.matching:              # held back until the result is in (then mapped with the new pose)
            self.buf.append(d)
            return
        p = (d["x_mm"], d["y_mm"], d["heading_deg"])
        a = self.anchor
        if a is None or math.hypot(p[0] - a[0], p[1] - a[1]) > STILL_MM or \
                abs((p[2] - a[2] + 180.0) % 360.0 - 180.0) > STILL_DEG:
            self.flush()               # moving: what was held back goes into the map unmatched
            self.anchor, self.done = p, False
        if self.request:
            self.req_wait += 1
            if self.req_wait > REQUEST_GIVE_UP:
                self._entry({"ok": False, "why": "the robot did not stand still"}, self.request)
                self.request = None
        if self.done and not self.request or not (self.m.settings.get("stop_scan") or self.request):
            self.flush()
            self.m.integrate(d)
            return
        self.buf.append(d)
        if len(self.buf) >= SCAN_N:
            self._match()

    def _scan(self):
        m, S = self.m, self.m.settings
        pitches = sorted(d["pitch_deg"] for d in self.buf)
        med = pitches[len(pitches) // 2]
        scan = []
        for d in self.buf:
            r = float(d["range_mm"])
            if not d["valid"] or r >= 8000 or r > S["max_hit_mm"] or r < S["min_mm"] or \
                    abs(d["pitch_deg"] - med) > PITCH_SPREAD_DEG:
                continue
            x, y, hd = m.map_pose(d["x_mm"], d["y_mm"], d["heading_deg"])
            h = math.radians(hd)
            a = h + math.radians(d["sensor_angle_deg"])
            scan.append((x + 26.0 * math.cos(h) + 12.9 * math.cos(a), y + 26.0 * math.sin(h) + 12.9 * math.sin(a),
                         a, r))
        last = self.buf[-1]
        ref = m.map_pose(last["x_mm"], last["y_mm"], last["heading_deg"])
        return scan, ref

    def _match(self):
        scan, ref = self._scan()
        who = self.request or "auto"
        gen, grid = self.gen, self.m.grid
        self.matching = True

        def finish(res):
            if gen != self.gen:
                return
            self.matching = False
            if isinstance(res, Exception):
                res = {"ok": False, "why": "matcher error: %s" % res}
            if res.get("ok"):
                self._apply(res, ref)
            self.request = None
            self.done = True
            self._entry(res, who)
            self.flush()
            if res.get("ok"):
                for f in self.listeners:
                    f(self.last)
        self.runner(lambda: match_scan(grid, scan, ref[:2], ref[2]), finish)

    def _apply(self, res, ref):
        """new corr = delta o corr; delta = rotate dth about ref, then shift (dx, dy)."""
        m = self.m
        cx, cy = ref[:2]
        tx, ty, tt = m.corr
        c, s = math.cos(math.radians(res["dth"])), math.sin(math.radians(res["dth"]))
        qx, qy = tx - cx, ty - cy
        m.corr = [c * qx - s * qy + cx + res["dx"], s * qx + c * qy + cy + res["dy"], tt + res["dth"]]
        self.applied += 1

    def _entry(self, res, who):
        e = {k: res.get(k) for k in ("ok", "why", "dx", "dy", "dth", "n", "inliers", "margin", "gain", "score", "ms")
             if res.get(k) is not None}
        e["mm"] = round(math.hypot(res.get("dx", 0.0), res.get("dy", 0.0)), 1)
        e["who"], e["time"] = who, int(time.time())
        self.last = e
        self.history.appendleft(e)
        self.seq += 1
        m = self.m
        if e["ok"]:
            m.say("pose corrected %.0f mm / %.1f deg (%s)" % (e["mm"], e["dth"], who))
        elif who != "auto":
            m.say("scan & correct: no correction: %s" % e["why"])