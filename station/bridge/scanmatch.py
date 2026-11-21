"""Scan-to-map pose correction (issue #12, setting "correct", off by default).

Readings are held back for one sweep (~32 readings, ~1.3 s). The sweep's hit points are then
tried against the grid at a small set of pose offsets around the robot (dx, dy in +-25 mm,
dtheta in +-1 deg); the offset whose hits land best on occupied cells (3x3-ish smoothed log-odds,
small penalty for moving) updates Mapper.corr, and the sweep is integrated with the new pose.
Map pose = corr o odometry pose, so the map stays put and the browser shifts it under the robot.
"""
import math

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
