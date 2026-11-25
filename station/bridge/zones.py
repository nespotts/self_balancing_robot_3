"""Virtual walls, no-go zones, a keep-in fence, slow zones and mazes on the map (issue #70).

Stored with the map (<map>.zones.json next to <map>.json, like places), map frame, mm.

  wall   polyline: never crossed           nogo   polygon: never entered
  fence  one polygon: never left           slow   polygon + speed cap (mm/s)
  maze   generated wall set (perfect maze, entrance + exit) + metadata
  bump   "invisible obstacle" mark (#104): where the robot touched something below the lidar's scan plane (a small
         disc, x / y / r + its polygon pts); the robot adds them (MSG_CONTACT), a person clears them

Who enforces what:
- planner (CostMap): mark_lines() puts walls / zone edges into the grid as occupied cells BEFORE inflation
  (so they get radius + margin like real walls, squeeze plans too); mark_areas() then blocks no-go insides and
  everything outside the fence, and makes slow zones cost more. Frontiers there are never chosen.
- bridge watchdog: watch() per tick (footprint inside a zone / out of the fence / on or across a wall, and only while
  moving into it, #114), and manual_filter() on every forwarded cmd_vel. The firmware only knows real range obstacles.
- escape (#114): escape() - the planner's first move when the robot stands at / over an edge (after a watchdog stop,
  a pose jump into a zone): straight away from it, backwards when the edge is ahead.
- bump marks count for the planner only (their edges are walls before inflation, like no-go zones), never for the
  watchdog: the robot is standing right at one when it is made and has to back away from it.
"""
import json
import math
import os
import random
import time

FORMAT = "sbr3-zones-1"
KINDS = ("wall", "nogo", "fence", "slow", "maze", "bump")
NAMES = {"wall": "virtual wall", "nogo": "no-go zone", "fence": "fence", "slow": "slow zone", "maze": "maze wall",
         "bump": "bump mark"}
MAX_ITEMS = 100              # drawn zones (bump marks have their own limit)
BUMP_R_MM = 70.0             # bump mark radius: about the body's half width (76)
BUMP_AHEAD_MM = 35.0         # its centre: this far past the contact point (the obstacle is beyond the body's edge)
BUMP_MERGE_MM = 100.0        # a contact this close to a mark is the same obstacle: the mark moves, hits + 1
BUMP_SIDES = 12              # polygon for the planner / ROS keepout
MAX_BUMPS = 200              # the oldest go first
MAX_PTS = 64
MAX_UNDO = 30
MIN_AREA_MM2 = 100.0 * 100.0
SLOW_DEFAULT, SLOW_LIMITS = 120.0, (30.0, 500.0)
MAZE_CELL_DEFAULT, MAZE_CELL_LIMITS = 700.0, (400.0, 1500.0)   # 700: corners stay > rng.obstacle_mm from a real wall
MAZE_SIZE_DEFAULT = 3000.0
MAZE_MAX_CELLS = 12
SLOW_COST = 40               # costmap step cost in a slow zone (x5: routes go round when it is not much longer)
SLOW_GROW_MM = 150.0         # autonomous: the speed cap starts this far before the zone
FP_TOL_MM = 20.0             # watchdog footprint = planner robot radius - this (pose noise; routes keep radius + margin)
# direction-aware watchdog + escape (#114)
MOVING_MMS = 20.0            # slower than this: the direction comes from the pose change, not heading x speed sign
INTO_STEP_MM = 30.0          # "into a zone" = 30 mm further along the motion the footprint is ...
INTO_GROW_MM = 2.0           # ... this much deeper (> ~4 deg into the edge; along it or away is fine)
ESCAPE_NEAR_MM = 30.0        # a plan starting with the footprint this close to a zone edge escapes first
ESCAPE_CLEAR_MM = 60.0       # ... straight out until the footprint is this far clear of every edge
ESCAPE_MIN_MM = 150.0        # ... and at least this far (a shorter first leg the navigator counts as passed at once)
ESCAPE_MAX_MM = 500.0        # ... at most this far
ESCAPE_STEP_MM = 10.0
STRAIGHT_WITHIN_DEG = 75.0   # straight back / ahead along the heading (no turn beside the edge) is tried first when
                             # within this of the way out


# ---------------------------------------------------------------- geometry
def seg_dist(px, py, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    L2 = ex * ex + ey * ey
    u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / L2)) if L2 else 0.0
    return math.hypot(x1 + u * ex - px, y1 + u * ey - py)


def inside(px, py, pts):
    c, j = False, len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def edges(pts, closed=True):
    n = len(pts)
    return [(pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1]) for i in range(n if closed else n - 1)]


def edge_dist(px, py, segs):
    return min((seg_dist(px, py, *s) for s in segs), default=1e18)


def crosses(ax, ay, bx, by, s):
    """Does segment a-b intersect segment s (touching counts)?"""
    cx, cy, dx, dy = s

    def orient(px, py, qx, qy, rx, ry):
        v = (qx - px) * (ry - py) - (qy - py) * (rx - px)
        return 0 if abs(v) < 1e-9 else (1 if v > 0 else -1)
    o1, o2 = orient(ax, ay, bx, by, cx, cy), orient(ax, ay, bx, by, dx, dy)
    o3, o4 = orient(cx, cy, dx, dy, ax, ay), orient(cx, cy, dx, dy, bx, by)
    if o1 != o2 and o3 != o4:
        return True
    return (o1 == 0 and seg_dist(cx, cy, ax, ay, bx, by) < 1e-6) or (o2 == 0 and seg_dist(dx, dy, ax, ay, bx, by) < 1e-6) \
        or (o3 == 0 and seg_dist(ax, ay, cx, cy, dx, dy) < 1e-6) or (o4 == 0 and seg_dist(bx, by, cx, cy, dx, dy) < 1e-6)


def area(pts):
    return 0.5 * sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in edges(pts))


def _num(v, lo, hi, default=None):
    try:
        v = float(v)
    except (TypeError, ValueError):
        if default is None:
            raise ValueError("bad number")
        return default
    if not math.isfinite(v):
        if default is None:
            raise ValueError("bad number")
        return default
    return max(lo, min(hi, v))


def clean_pts(pts, kind):
    out = []
    for p in pts or []:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            raise ValueError("bad point")
        x, y = float(p[0]), float(p[1])
        if not (abs(x) < 1e7 and abs(y) < 1e7):
            raise ValueError("bad point")
        if out and abs(x - out[-1][0]) + abs(y - out[-1][1]) < 1.0:
            continue
        out.append([round(x, 1), round(y, 1)])
    if kind != "wall" and len(out) > 2 and abs(out[0][0] - out[-1][0]) + abs(out[0][1] - out[-1][1]) < 1.0:
        out.pop()                                  # closed by repeating the first point
    if len(out) < (2 if kind == "wall" else 3):
        raise ValueError("a %s needs at least %d points" % (NAMES[kind], 2 if kind == "wall" else 3))
    if len(out) > MAX_PTS:
        raise ValueError("%d points at most" % MAX_PTS)
    if kind != "wall" and abs(area(out)) < MIN_AREA_MM2:
        raise ValueError("that %s is too small" % NAMES[kind])
    return out


# ---------------------------------------------------------------- maze
def make_maze(cx, cy, w, h, cell, seed=None, toward=None):
    """Random perfect maze (every cell reachable, one way between any two) centred on (cx, cy), axis-aligned
    in the map frame. Entrance on the side facing `toward` (the robot), exit on the opposite side.
    Returns the maze item (no id)."""
    cell = _num(cell, *MAZE_CELL_LIMITS)
    cols = max(2, min(MAZE_MAX_CELLS, int(_num(w, 0, 1e5) // cell)))
    rows = max(2, min(MAZE_MAX_CELLS, int(_num(h, 0, 1e5) // cell)))
    seed = random.randrange(1 << 30) if seed is None else int(seed)
    rnd = random.Random(seed)
    x0, y0 = cx - cols * cell / 2, cy - rows * cell / 2
    east = [[False] * cols for _ in range(rows)]    # passage from (r, c) to (r, c + 1)
    north = [[False] * cols for _ in range(rows)]   # passage from (r, c) to (r + 1, c)
    seen = [[False] * cols for _ in range(rows)]
    start = (rnd.randrange(rows), rnd.randrange(cols))
    seen[start[0]][start[1]] = True
    stack = [start]
    while stack:                                    # iterative depth-first backtracker
        r, c = stack[-1]
        nb = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 0 <= r + dr < rows and 0 <= c + dc < cols and not seen[r + dr][c + dc]]
        if not nb:
            stack.pop()
            continue
        nr, nc = rnd.choice(nb)
        if nr != r:
            north[min(r, nr)][c] = True
        else:
            east[r][min(c, nc)] = True
        seen[nr][nc] = True
        stack.append((nr, nc))
    # entrance: the side facing the robot, the cell nearest to it; exit: a random cell on the opposite side
    tx, ty = toward if toward else (x0 - cell, cy)
    dx, dy = (tx - cx) / (cols * cell), (ty - cy) / (rows * cell)
    side = ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("N" if dy > 0 else "S")
    opp = {"E": "W", "W": "E", "N": "S", "S": "N"}[side]

    def along(s, k):          # cell (r, c) at index k along side s
        return {"W": (k, 0), "E": (k, cols - 1), "S": (0, k), "N": (rows - 1, k)}[s]

    n_side = rows if side in "WE" else cols
    k_in = int(max(0, min(n_side - 1, ((ty - y0) if side in "WE" else (tx - x0)) // cell)))
    k_out = rnd.randrange(rows if opp in "WE" else cols)

    def gap(s, k):            # the outer wall segment of that cell on side s
        r, c = along(s, k)
        if s == "W":
            return (x0, y0 + r * cell, x0, y0 + (r + 1) * cell)
        if s == "E":
            return (x0 + cols * cell, y0 + r * cell, x0 + cols * cell, y0 + (r + 1) * cell)
        if s == "S":
            return (x0 + c * cell, y0, x0 + (c + 1) * cell, y0)
        return (x0 + c * cell, y0 + rows * cell, x0 + (c + 1) * cell, y0 + rows * cell)

    def centre(s, k, out=0.0):
        r, c = along(s, k)
        x, y = x0 + (c + 0.5) * cell, y0 + (r + 0.5) * cell
        ox, oy = {"W": (-1, 0), "E": (1, 0), "S": (0, -1), "N": (0, 1)}[s]
        return [round(x + ox * out, 1), round(y + oy * out, 1)]

    g_in, g_out = gap(side, k_in), gap(opp, k_out)
    # wall runs: horizontal lines y = y0 + r * cell, vertical x = x0 + c * cell; merged into long segments
    walls = []
    for r in range(rows + 1):
        run = None
        for c in range(cols + 1):
            wall = c < cols and (r in (0, rows) or not north[r - 1][c])
            seg = (x0 + c * cell, y0 + r * cell, x0 + (c + 1) * cell, y0 + r * cell) if c < cols else None
            if wall and seg not in (g_in, g_out):
                run = [seg[0], seg[1], seg[2], seg[3]] if run is None else [run[0], run[1], seg[2], seg[3]]
            else:
                if run:
                    walls.append(run)
                run = None
    for c in range(cols + 1):
        run = None
        for r in range(rows + 1):
            wall = r < rows and (c in (0, cols) or not east[r][c - 1])
            seg = (x0 + c * cell, y0 + r * cell, x0 + c * cell, y0 + (r + 1) * cell) if r < rows else None
            if wall and seg not in (g_in, g_out):
                run = [seg[0], seg[1], seg[2], seg[3]] if run is None else [run[0], run[1], seg[2], seg[3]]
            else:
                if run:
                    walls.append(run)
                run = None
    return {"kind": "maze", "x0": round(x0, 1), "y0": round(y0, 1), "cols": cols, "rows": rows, "cell": round(cell, 1),
            "seed": seed, "walls": [[round(v, 1) for v in s] for s in walls],
            "entry": centre(side, k_in, 0.9 * cell), "entry_in": centre(side, k_in),
            "exit": centre(opp, k_out), "exit_out": centre(opp, k_out, 0.9 * cell),
            "exit_gap": [round(v, 1) for v in g_out], "best_s": None}


def maze_cells_connected(item):
    """Every cell reachable from the entrance cell and the exit cell among them (walls as stored)."""
    x0, y0, cols, rows, cell = item["x0"], item["y0"], item["cols"], item["rows"], item["cell"]
    segs = [tuple(s) for s in item["walls"]] + [tuple(item["exit_gap"])]

    def blocked(ax, ay, bx, by):
        return any(crosses(ax, ay, bx, by, s) for s in segs)

    def cell_of(p):
        return (int((p[1] - y0) // cell), int((p[0] - x0) // cell))
    start, goal = cell_of(item["entry_in"]), cell_of(item["exit"])
    seen, q = {start}, [start]
    while q:
        r, c = q.pop()
        ax, ay = x0 + (c + 0.5) * cell, y0 + (r + 0.5) * cell
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in seen and \
                    not blocked(ax, ay, ax + dc * cell, ay + dr * cell):
                seen.add((nr, nc))
                q.append((nr, nc))
    return len(seen) == rows * cols and goal in seen


def bump_pts(x, y, r=BUMP_R_MM):
    """A bump mark's polygon (BUMP_SIDES corners round (x, y))."""
    return [[round(x + r * math.cos(2 * math.pi * k / BUMP_SIDES), 1), round(y + r * math.sin(2 * math.pi * k / BUMP_SIDES), 1)]
            for k in range(BUMP_SIDES)]


def wall_segments(items):
    """Every virtual wall and maze wall as (x1, y1, x2, y2): what the simulator builds for real (sim --maze)."""
    out = []
    for it in items:
        if it["kind"] == "wall":
            out += edges(it["pts"], closed=False)
        elif it["kind"] == "maze":
            out += [tuple(s) for s in it["walls"]]
    return out


# ---------------------------------------------------------------- snapshot (planner + watchdog)
class ZoneView:
    """Read-only snapshot of the layers. close_exit: a maze id whose exit gap counts as a wall (Solve: the only
    way to the exit cell is through the maze, not round it and in through the exit)."""

    def __init__(self, items, close_exit=None):
        self.walls = []        # (key, [segments], kind)  key = item id
        self.nogo = []         # (key, pts, edges)
        self.fence = None      # (key, pts, edges)
        self.slow = []         # (key, pts, edges, speed)
        self.bumps = []        # (key, pts, edges): bump marks (#104), planner only
        for it in items:
            k = it["kind"]
            if k == "bump":
                self.bumps.append((it["id"], it["pts"], edges(it["pts"])))
            elif k == "wall":
                self.walls.append((it["id"], edges(it["pts"], closed=False), "wall"))
            elif k == "maze":
                segs = [tuple(s) for s in it["walls"]]
                if close_exit is not None and it["id"] == close_exit:
                    segs.append(tuple(it["exit_gap"]))
                self.walls.append((it["id"], segs, "maze"))
            elif k == "nogo":
                self.nogo.append((it["id"], it["pts"], edges(it["pts"])))
            elif k == "fence":
                self.fence = (it["id"], it["pts"], edges(it["pts"]))
            elif k == "slow":
                self.slow.append((it["id"], it["pts"], edges(it["pts"]), float(it.get("speed") or SLOW_DEFAULT)))
        self.n = len(items)

    def __bool__(self):
        return self.n > 0

    def bbox(self):
        xs, ys = [], []
        for _k, segs, _kind in self.walls:
            for s in segs:
                xs += [s[0], s[2]]
                ys += [s[1], s[3]]
        for z in self.nogo + self.slow + self.bumps + ([self.fence] if self.fence else []):
            xs += [p[0] for p in z[1]]
            ys += [p[1] for p in z[1]]
        return (min(xs), min(ys), max(xs), max(ys)) if xs else None

    def all_lines(self):
        """Every segment that becomes an occupied cell before inflation (bump marks too: an obstacle the lidar can't
        see, kept off like a wall)."""
        out = []
        for _k, segs, _kind in self.walls:
            out += segs
        for z in self.nogo + self.bumps + ([self.fence] if self.fence else []):
            out += z[2]
        return out

    # ------------------------------------------------------------ planner raster
    def mark_lines(self, cm):
        """Occupied (state 2) along every wall and zone edge, as a 4-connected line (no diagonal leak)."""
        w, h, res, x0, y0, st = cm.w, cm.h, cm.res, cm.x0, cm.y0, cm.state
        for x1, y1, x2, y2 in self.all_lines():
            n = max(1, int(math.ceil(math.hypot(x2 - x1, y2 - y1) / (res / 4.0))))
            prev = None
            for k in range(n + 1):
                cx = int(math.floor((x1 + (x2 - x1) * k / n - x0) / res))
                cy = int(math.floor((y1 + (y2 - y1) * k / n - y0) / res))
                if prev and prev[0] != cx and prev[1] != cy and 0 <= cx < w and 0 <= prev[1] < h:
                    st[prev[1] * w + cx] = 2
                if 0 <= cx < w and 0 <= cy < h:
                    st[cy * w + cx] = 2
                prev = (cx, cy)

    @staticmethod
    def _spans(cm, pts):
        """{row: [(a, b), ...]} cells whose centres lie inside the polygon (b exclusive)."""
        res, x0, y0, w = cm.res, cm.x0, cm.y0, cm.w
        ys = [p[1] for p in pts]
        r0 = max(0, int((min(ys) - y0) // res) - 1)
        r1 = min(cm.h - 1, int((max(ys) - y0) // res) + 1)
        out = {}
        es = edges(pts)
        for r in range(r0, r1 + 1):
            yc = y0 + (r + 0.5) * res
            xs = sorted(x1 + (yc - y1) * (x2 - x1) / (y2 - y1) for x1, y1, x2, y2 in es if (y1 <= yc < y2) or (y2 <= yc < y1))
            sp = []
            for i in range(0, len(xs) - 1, 2):
                a = max(0, int(math.ceil((xs[i] - x0) / res - 0.5)))
                b = min(w, int(math.floor((xs[i + 1] - x0) / res - 0.5)) + 1)
                if b > a:
                    sp.append((a, b))
            if sp:
                out[r] = sp
        return out

    def mark_areas(self, cm, blocked, margin):
        """After inflation: no-go insides and the outside of the fence are blocked + occupied; slow zones cost more."""
        w, st, cost = cm.w, cm.state, cm.cost
        for _k, pts, _e in self.nogo:
            for r, sp in self._spans(cm, pts).items():
                for a, b in sp:
                    st[r * w + a:r * w + b] = b"\x02" * (b - a)
                    cost[r * w + a:r * w + b] = bytes([blocked]) * (b - a)
        if self.fence:
            spans = self._spans(cm, self.fence[1])
            for r in range(cm.h):
                x = 0
                for a, b in spans.get(r, []) + [(w, w)]:
                    if a > x:
                        st[r * w + x:r * w + a] = b"\x02" * (a - x)
                        cost[r * w + x:r * w + a] = bytes([blocked]) * (a - x)
                    x = b
        lut = bytes(max(v, SLOW_COST) if v < margin else v for v in range(256))
        for _k, pts, _e, _s in self.slow:
            for r, sp in self._spans(cm, pts).items():
                for a, b in sp:
                    cost[r * w + a:r * w + b] = bytes(cost[r * w + a:r * w + b]).translate(lut)

    def point_blocked(self, x, y):
        """Why a goal there is refused (inside a no-go zone / outside the fence), or None."""
        for _k, pts, _e in self.nogo:
            if inside(x, y, pts):
                return "that spot is inside a no-go zone"
        if self.fence and not inside(x, y, self.fence[1]):
            return "that spot is outside the fence"
        return None

    # ------------------------------------------------------------ watchdog
    def depths(self, x, y, fp, prev=None, jump_mm=300.0):
        """Violations of a footprint of radius fp at (x, y): {key: (depth mm, text)}; depth > 0 = how far the
        footprint is in. prev: the last pose; a wall crossed since then counts too (unless it jumped > jump_mm:
        a pose reset)."""
        out = {}
        for k, segs, kind in self.walls:
            d = edge_dist(x, y, segs)
            crossed = prev is not None and math.hypot(x - prev[0], y - prev[1]) < jump_mm and \
                any(crosses(prev[0], prev[1], x, y, s) for s in segs)
            if crossed or d < fp:
                out[k] = (fp + d if crossed else fp - d, ("crossed a " if crossed else "at a ") + NAMES[kind], kind)
        for k, pts, es in self.nogo:
            d = edge_dist(x, y, es)
            dep = fp + d if inside(x, y, pts) else fp - d
            if dep > 0:
                out[k] = (dep, "in a no-go zone", "nogo")
        if self.fence:
            k, pts, es = self.fence
            d = edge_dist(x, y, es)
            dep = fp - d if inside(x, y, pts) else fp + d
            if dep > 0:
                out[k] = (dep, "at the fence" if inside(x, y, pts) else "outside the fence", "fence")
        return out

    def slow_cap(self, x, y, grow=0.0):
        """Lowest speed cap (mm/s) of the slow zones at (x, y) (or within grow mm of them), or None."""
        caps = [s for _k, pts, es, s in self.slow if inside(x, y, pts) or (grow > 0 and edge_dist(x, y, es) < grow)]
        return min(caps) if caps else None

    def manual_filter(self, x, y, h_deg, speed, turn, fp, v_now=0.0, horizon=0.3, block=True):
        """Joystick cmd_vel at pose (x, y, h): -> (speed, turn, why). Slow zones cap the speed; with block, a speed
        that would take the footprint further into a zone (or across a wall) within `horizon` s becomes 0.
        Turning on the spot never moves the (round) footprint, so turn is kept; driving out is always allowed."""
        why = None
        look = max(abs(speed), abs(v_now)) * horizon + 30.0
        cap = self.slow_cap(x, y, grow=look)
        if cap is not None and abs(speed) > cap:
            speed = math.copysign(cap, speed)
            why = "slow zone: %d mm/s" % cap
        if not block or abs(speed) < 1.0:
            return speed, turn, why
        a = math.radians(h_deg + turn * horizon / 2.0)
        d = math.copysign(look, speed)
        nx, ny = x + d * math.cos(a), y + d * math.sin(a)
        now = self.depths(x, y, fp)
        nxt = self.depths(nx, ny, fp, prev=(x, y))
        for k, (dep, _text, kind) in nxt.items():
            if dep > now.get(k, (0.0,))[0] + 0.5:
                return 0.0, turn, "blocked: %s ahead" % NAMES[kind]
        return speed, turn, why

    # ------------------------------------------------------------ direction (#114)
    def into(self, x, y, ux, uy, fp, step=INTO_STEP_MM):
        """Keys of the zones that motion along the unit vector (ux, uy) from (x, y) takes the footprint further into
        (or into at all): parallel to an edge or away from it is not "into"."""
        now = self.depths(x, y, fp)
        nxt = self.depths(x + ux * step, y + uy * step, fp, prev=(x, y))
        return {k for k, (d, _t, _kind) in nxt.items() if d > now.get(k, (0.0,))[0] + INTO_GROW_MM}

    def watch(self, x, y, h_deg, v, fp, prev, hold, again, active=True, jump_mm=300.0):
        """The bridge watchdog's 25 Hz check (#70, direction-aware #114). (x, y, h): the pose, v: its speed (mm/s,
        < 0 = backwards), prev: the last pose, hold: {key: least depth since the last stop}. -> (why or None, hold).
        A stop (active only: GOTO / EXPLORE, armed) only for motion into a zone or across a wall - never for driving
        along an edge or away from it (an escape). The direction: the heading x the speed's sign, or the pose change
        when it barely moves. Again for the same zone only if it got `again` mm deeper in than its least depth since
        (or once it was `again` mm clear of it). A pose jump (> jump_mm: a
        relocalisation / Set position) that lands in a zone is no violation: why = "pose moved into ..." (escape)."""
        jumped = prev is not None and math.hypot(x - prev[0], y - prev[1]) >= jump_mm
        v_now = self.depths(x, y, fp, prev)
        # a held zone is let go once the footprint is `again` mm clear of it (rocking back to its edge is no new stop)
        band = self.depths(x, y, fp + again) if hold else {}
        hold = {k: min(d, band[k][0] - again) for k, d in hold.items() if k in band}
        if not active or not v_now:
            return None, hold
        new = [k for k, (d, _t, _kind) in v_now.items() if k not in hold or d > hold[k] + again]
        if not new:
            return None, hold
        if jumped:
            why = "pose moved: now " + v_now[new[0]][1]
        else:
            u = None
            if abs(v) >= MOVING_MMS:
                a = math.radians(h_deg)
                sg = 1.0 if v > 0 else -1.0
                u = (sg * math.cos(a), sg * math.sin(a))
            elif prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                L = math.hypot(dx, dy)
                if L >= 3.0:
                    u = (dx / L, dy / L)
            ins = self.into(x, y, u[0], u[1], fp) if u else set()
            bad = [k for k in new if k in ins or v_now[k][1].startswith("crossed")]
            if not bad:
                return None, hold
            why = v_now[bad[0]][1]
        for k, (d, _t, _kind) in v_now.items():
            hold[k] = d
        return why, hold

    def _normal(self, x, y, fp):
        """{key: (depth, (ux, uy) or None, text)} for the zones within fp of (x, y): the unit vector points away
        from the zone (out of a no-go zone, into the fence, off a wall; None: right on a wall line)."""
        out = {}

        def off(segs, sign):
            best = None
            for x1, y1, x2, y2 in segs:
                ex, ey = x2 - x1, y2 - y1
                L2 = ex * ex + ey * ey
                u = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / L2)) if L2 else 0.0
                cx, cy = x1 + u * ex, y1 + u * ey
                d = math.hypot(x - cx, y - cy)
                if best is None or d < best[0]:
                    best = (d, cx, cy)
            d, cx, cy = best
            return d, (((x - cx) / d * sign, (y - cy) / d * sign) if d > 1.0 else None)
        for k, segs, kind in self.walls:
            for i, s in enumerate(segs):               # each wall segment (a maze corner: both of its walls)
                d, n = off([s], 1.0)
                if d < fp:
                    out[(k, i)] = (fp - d, n, "at a " + NAMES[kind])
        for k, pts, es in self.nogo:
            ins = inside(x, y, pts)
            d, n = off(es, -1.0 if ins else 1.0)
            dep = fp + d if ins else fp - d
            if dep > 0:
                out[k] = (dep, n, "in a no-go zone")
        if self.fence:
            k, pts, es = self.fence
            ins = inside(x, y, pts)
            d, n = off(es, -1.0 if ins else 1.0)
            n = (-n[0], -n[1]) if n else None          # the fence keeps in: away = inwards
            dep = fp - d if ins else fp + d
            if dep > 0:
                out[k] = (dep, n, "at the fence" if ins else "outside the fence")
        return out

    def escape(self, x, y, h_deg, fp, clear=ESCAPE_CLEAR_MM, near=ESCAPE_NEAR_MM, max_mm=ESCAPE_MAX_MM, ok=None):
        """#114: the robot at (x, y, heading h) stands at / over a zone edge (its footprint fp within `near` mm of one):
        a short straight move away from it first. -> None (not at an edge), or {"x", "y": where to, "dist": mm,
        "drive": "back" / "ahead" (straight along the heading: no turn beside the edge) or "turn" (turn on the spot,
        then drive there), "reverse": drive == "back", "why": the zone}, or {"boxed": True, "why": ...} when no straight
        move gets clear (and the robot is really in a zone). Clear = the footprint `clear` mm off every edge, and at
        least ESCAPE_MIN_MM; on the way it never gets deeper into any zone or crosses a wall. ok(x, y): extra check
        of the end point (the planner: not inside a real obstacle)."""
        near_now = self._normal(x, y, fp + near)
        if not near_now:
            return None
        worst = max(near_now.values(), key=lambda e: e[0])
        sx = sum(n[0] * (dep + 1.0) for dep, n, _t in near_now.values() if n)
        sy = sum(n[1] * (dep + 1.0) for dep, n, _t in near_now.values() if n)
        h = math.radians(h_deg)
        ahead, back = math.atan2(math.sin(h), math.cos(h)), math.atan2(-math.sin(h), -math.cos(h))
        base = math.atan2(sy, sx) if math.hypot(sx, sy) > 1e-6 else back
        within = math.cos(math.radians(STRAIGHT_WITHIN_DEG))
        cands = [(a, k) for a, k in ((back, "back"), (ahead, "ahead")) if math.cos(a - base) > within]
        cands += [(base + math.radians(s * a), "turn") for a in range(0, 91, 15) for s in ((1,) if a == 0 else (1, -1))]
        start = self.depths(x, y, fp)
        segs = [s for _k, ss, _kind in self.walls for s in ss if seg_dist(x, y, *s) > 1.0]
        for a, how in cands:
            ux, uy = math.cos(a), math.sin(a)
            d, good = 0.0, None
            while d < max_mm:
                d = min(max_mm, d + ESCAPE_STEP_MM)
                qx, qy = x + ux * d, y + uy * d
                now = self.depths(qx, qy, fp)
                if any(dep > start.get(k, (0.0,))[0] + 1.0 for k, (dep, _t, _kind) in now.items()):
                    break                                  # deeper into something (or into another zone)
                if not self._normal(qx, qy, fp + clear) and (ok is None or ok(qx, qy)):
                    good = (qx, qy, d)                     # clear here; on to ESCAPE_MIN_MM while it stays clear
                    if d >= ESCAPE_MIN_MM:
                        break
                elif good:
                    break
            if good and not any(crosses(x, y, good[0], good[1], s) for s in segs):
                return {"x": round(good[0], 1), "y": round(good[1], 1), "dist": round(good[2], 1), "drive": how,
                        "reverse": how == "back", "why": worst[2]}
        if start:
            return {"boxed": True, "why": worst[2]}
        return None                                        # only near edges, none touched: plan as usual

# ---------------------------------------------------------------- the store
class Zones:
    def __init__(self):
        self.items = []          # replaced, never mutated in place: a ZoneView / worker thread sees a consistent list
        self.history = []        # earlier item lists (undo)
        self.n = 0               # id counter
        self.version = 0
        self._view = (None, None)

    def _id(self, kind):
        self.n += 1
        return "%s%d" % (kind[0], self.n)

    def _commit(self, items):
        self.history.append(self.items)
        del self.history[:-MAX_UNDO]
        self.items = items
        self.version += 1

    def find(self, zid):
        return next((it for it in self.items if it["id"] == zid), None)

    def view(self, close_exit=None):
        if close_exit is not None:
            return ZoneView(self.items, close_exit)
        v, cached = self._view
        if v != self.version:
            cached = ZoneView(self.items)
            self._view = (self.version, cached)
        return cached

    # ------------------------------------------------------------ changes (raise ValueError)
    def _drawn(self, items):
        return sum(1 for x in items if x["kind"] != "bump")

    def add(self, kind, pts, speed=None):
        if kind not in KINDS or kind in ("maze", "bump"):
            raise ValueError("unknown zone kind %r" % kind)
        it = {"id": self._id(kind), "kind": kind, "pts": clean_pts(pts, kind)}
        if kind == "slow":
            it["speed"] = round(_num(speed, *SLOW_LIMITS, default=SLOW_DEFAULT))
        items = [x for x in self.items if not (kind == "fence" and x["kind"] == "fence")]   # one fence
        if self._drawn(items) >= MAX_ITEMS:
            raise ValueError("%d zones is the limit" % MAX_ITEMS)
        self._commit(items + [it])
        return it["id"]

    # ------------------------------------------------------------ bump marks (#104)
    def bumps(self):
        return [x for x in self.items if x["kind"] == "bump"]

    def add_bump(self, x, y, heading=None, what="bump", when=None, undo=False):
        """The robot touched something the lidar can't see at (x, y) (map frame), driving toward heading (deg): a mark
        BUMP_AHEAD_MM past it. Within BUMP_MERGE_MM of a mark it is the same obstacle: that mark moves a third of the
        way there and counts the hit. undo False (the robot's report): no undo step. -> (id, merged)."""
        x, y = _num(x, -1e7, 1e7), _num(y, -1e7, 1e7)
        if heading is not None:
            h = math.radians(_num(heading, -1e6, 1e6))
            x, y = x + BUMP_AHEAD_MM * math.cos(h), y + BUMP_AHEAD_MM * math.sin(h)
        what = what if what in ("bump", "stuck", "manual") else "bump"
        when = when or time.strftime("%Y-%m-%dT%H:%M:%S")
        near = min(self.bumps(), key=lambda b: math.hypot(b["x"] - x, b["y"] - y), default=None)
        if near is not None and math.hypot(near["x"] - x, near["y"] - y) < BUMP_MERGE_MM:
            nx, ny = near["x"] + (x - near["x"]) / 3.0, near["y"] + (y - near["y"]) / 3.0
            it = dict(near, x=round(nx, 1), y=round(ny, 1), pts=bump_pts(nx, ny, near.get("r", BUMP_R_MM)),
                      hits=int(near.get("hits", 1)) + 1, t=when, what=what)
            items = [it if b is near else b for b in self.items]
            merged = True
        else:
            it = {"id": self._id("bump"), "kind": "bump", "x": round(x, 1), "y": round(y, 1), "r": BUMP_R_MM,
                  "pts": bump_pts(x, y), "what": what, "hits": 1, "t": when}
            items = self.items + [it]
            old = [b for b in items if b["kind"] == "bump"]
            if len(old) > MAX_BUMPS:                    # the oldest go first
                drop = {b["id"] for b in old[:len(old) - MAX_BUMPS]}
                items = [b for b in items if b["id"] not in drop]
            merged = False
        if undo:
            self._commit(items)
        else:
            self.items = items
            self.version += 1
        return it["id"], merged

    def update(self, zid, pts=None, speed=None):
        old = self.find(zid)
        if not old:
            raise ValueError("no zone %s" % zid)
        if old["kind"] == "maze":
            raise ValueError("a maze is not edited point by point: clear it and generate another")
        if old["kind"] == "bump":
            raise ValueError("a bump mark is not edited: delete it")
        it = dict(old)
        if pts is not None:
            it["pts"] = clean_pts(pts, it["kind"])
        if speed is not None and it["kind"] == "slow":
            it["speed"] = round(_num(speed, *SLOW_LIMITS))
        self._commit([it if x is old else x for x in self.items])
        return zid

    def delete(self, zid):
        if not self.find(zid):
            raise ValueError("no zone %s" % zid)
        self._commit([x for x in self.items if x["id"] != zid])
        return zid

    def clear(self, kind=None):
        self._commit([x for x in self.items if kind and x["kind"] != kind])

    def undo(self):
        if not self.history:
            raise ValueError("nothing to undo")
        snap = self.history.pop()
        # the robot's bump marks made since that step (no undo steps of their own) stay; a cleared / deleted one is back
        cur = {b["id"]: b for b in self.bumps()}
        self.items = [cur.pop(x["id"], x) if x["kind"] == "bump" else x for x in snap] + list(cur.values())
        self.version += 1

    def add_maze(self, cx, cy, w=MAZE_SIZE_DEFAULT, h=MAZE_SIZE_DEFAULT, cell=MAZE_CELL_DEFAULT, seed=None, toward=None):
        if self._drawn(self.items) >= MAX_ITEMS:
            raise ValueError("%d zones is the limit" % MAX_ITEMS)
        it = make_maze(_num(cx, -1e7, 1e7), _num(cy, -1e7, 1e7), w, h, cell, seed, toward)
        it = dict(it, id=self._id("maze"))
        self._commit(self.items + [it])
        return it["id"]

    def maze_best(self, zid, seconds):
        """A solve finished: keep the best time with the maze (no undo step)."""
        it = self.find(zid)
        if it and it["kind"] == "maze" and (it.get("best_s") is None or seconds < it["best_s"]):
            self.items = [dict(x, best_s=round(seconds, 1)) if x is it else x for x in self.items]
            self.version += 1

    # ------------------------------------------------------------ files
    def to_json(self):
        return json.dumps({"format": FORMAT, "items": self.items}, indent=1)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        if not isinstance(d, dict) or d.get("format") != FORMAT:
            raise ValueError("not a zones file")
        z = cls()
        items = []
        for it in d.get("items") or []:
            try:
                k = it["kind"]
                if k == "maze":
                    good = dict(it, walls=[[float(v) for v in s][:4] for s in it["walls"]],
                                exit_gap=[float(v) for v in it["exit_gap"]][:4])
                    for key in ("entry", "entry_in", "exit", "exit_out"):
                        good[key] = [float(v) for v in it[key]][:2]
                    if any(len(s) != 4 for s in good["walls"]) or len(good["exit_gap"]) != 4:
                        continue
                elif k == "bump":
                    bx, by = _num(it["x"], -1e7, 1e7), _num(it["y"], -1e7, 1e7)
                    br = _num(it.get("r"), 20.0, 500.0, default=BUMP_R_MM)
                    good = {"id": str(it["id"]), "kind": k, "x": bx, "y": by, "r": br, "pts": bump_pts(bx, by, br),
                            "what": it.get("what") if it.get("what") in ("bump", "stuck", "manual") else "bump",
                            "hits": max(1, int(_num(it.get("hits"), 1, 1e6, default=1))), "t": str(it.get("t") or "")[:19]}
                elif k in KINDS:
                    good = {"id": str(it["id"]), "kind": k, "pts": clean_pts(it["pts"], k)}
                    if k == "slow":
                        good["speed"] = round(_num(it.get("speed"), *SLOW_LIMITS, default=SLOW_DEFAULT))
                else:
                    continue
                good["id"] = str(it["id"])[:12]
                if any(x["id"] == good["id"] for x in items) or (k == "fence" and any(x["kind"] == "fence" for x in items)):
                    continue
                items.append(good)
                try:
                    z.n = max(z.n, int(good["id"][1:]))
                except ValueError:
                    pass
            except (KeyError, TypeError, ValueError):
                continue                      # one bad entry must not lose the others
        drawn = [it for it in items if it["kind"] != "bump"][:MAX_ITEMS]
        z.items = drawn + [it for it in items if it["kind"] == "bump"][-MAX_BUMPS:]
        return z

    @staticmethod
    def write(path, z):
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(z.to_json())
        os.replace(tmp, path)

    @classmethod
    def read(cls, path):
        try:
            with open(path, encoding="utf-8") as f:
                return cls.from_json(f.read())
        except (OSError, ValueError):
            return cls()
