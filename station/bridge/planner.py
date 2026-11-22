"""Path planning and frontier exploration on the occupancy grid (issue #12 follow-up). Runs in the bridge.

CostMap (pure, testable): the grid's touched area at 50 mm cells, obstacles inflated by the robot
radius + a safety margin (hard: never planned through) plus a soft band (costs more, so routes keep
to the middle of free space). A* (8-connected, octile) and line-of-sight simplification turn a goal
into a few waypoints; a Dijkstra flood from the robot finds the nearest reachable frontier (free
cells next to unknown ones) for exploring.

Planner (bridge side): "go here" = plan -> MSG_GOTO (replace) -> MODE_GOTO; replans when the robot
reports BLOCKED or the map grows an obstacle on the remaining route. Explore = repeatedly plan to the
nearest frontier in MODE_EXPLORE until none is left. Stops on E-stop, disarm, link loss.
Planning work runs in a worker thread (the event loop keeps the 10 Hz heartbeat going).
"""
import asyncio
import concurrent.futures
import heapq
import json
import math
import os
import time

from . import link_protocol as lp
from .occgrid import UNKNOWN, UNIT
from .places import HOME

SQ2 = math.sqrt(2.0)
BLOCKED = 255             # within robot radius + 30 mm of an obstacle: never
MARGIN = 250              # within radius + margin: only to drive out of it near the start
OCC_P, FREE_P = 0.62, 0.40
OCC_BYTE = UNKNOWN + math.log(OCC_P / (1 - OCC_P)) / UNIT     # ~138
FREE_BYTE = UNKNOWN + math.log(FREE_P / (1 - FREE_P)) / UNIT  # ~120
# fine byte -> class bits: 2 = occupied, 1 = free, 0 = unknown (OR-able: occupied wins)
CLASS_LUT = bytes(2 if v > OCC_BYTE else (1 if v < FREE_BYTE else 0) for v in range(256))
# squeeze plans (#62): only sure walls count (~0.8). The sensor cone draws weak stray hits into a doorway
# (just over OCC_BYTE) from beams that clip a jamb; those count as free there (seen, not a frontier).
# Only for maps without range weighting (mapping "range_weight" off): with it, far hits are weak by design
# (a wall seen from 1-2 m sits at ~140-155 and would vanish from a sure-walls map, letting squeeze routes and
# frontiers through it), and close free readings already clear far stray hits out of doorways (#63).
SURE_OCC_BYTE = 155
SURE_LUT = bytes(2 if v > SURE_OCC_BYTE else (1 if v < FREE_BYTE or v > OCC_BYTE else 0) for v in range(256))
FINAL_LUT = bytes([0, 1, 2, 2] + [0] * 252)

DEFAULTS = {
    "robot_radius_mm": 80.0,    # footprint radius the planner keeps off obstacles
    "margin_mm": 180.0,         # + this much (the robot sways, the map is fuzzy)
    "soft_mm": 300.0,           # + a band that costs extra (keeps routes in the middle)
    "squeeze_mm": 35.0,         # no route with the margin: try again keeping only radius + this (narrow gaps)
    "unknown_ok": True,         # "go here" may route through unmapped space (explore never does)
    "frontier_min_mm": 200.0,   # ignore frontier bits smaller than this
    "scan_pause": True,         # explore: stand still at each frontier goal for a stop-and-scan (#71; only
                                # while the map's stop_scan correction is on)
    "zones_block_manual": True,  # bridge: joystick cmd_vel that would drive into a zone / across a virtual wall -> 0 (#70)
}
LIMITS = {"robot_radius_mm": (20, 400), "margin_mm": (0, 500), "soft_mm": (0, 1000), "squeeze_mm": (0, 200),
          "frontier_min_mm": (50, 2000)}
STUCK_BOOST_MM = 100.0    # the robot reported "nav: stuck": plan with this much more margin ...
STUCK_BOOST_MAX = 200.0
BOOST_DECAY_MM = 50.0     # ... less by this after each goal reached
SETTINGS_FILE = "planner.json"   # in the map folder (~/sbr3-maps)
SQUEEZE_TRIES = 6         # explore: squeeze goals in a row that showed nothing new before it is done
SQUEEZE_GROW = 160        # ... "something new" = this many more free 25 mm cells (0.1 m^2)
SQUEEZE_CHECKS = 400      # explore, squeezing: frontier spots checked (100 mm blocks, nearest first)
SCAN_WAIT_S = 6.0         # explore: wait at most this long at a frontier goal for the stop-and-scan (#71)
SCAN_EVERY_S = 10.0       # explore: a new frontier goal after this long without a scan: stop and scan first


def clean_settings(s, base):
    """Copy of base with the known keys of s applied (clamped to LIMITS, wrong types skipped)."""
    out = dict(base)
    for k, v in (s if isinstance(s, dict) else {}).items():
        if k not in DEFAULTS:
            continue
        if isinstance(DEFAULTS[k], bool):
            out[k] = bool(v)
            continue
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(v):
            lo, hi = LIMITS.get(k, (-1e9, 1e9))
            out[k] = max(lo, min(hi, v))
    return out


class CostMap:
    def __init__(self, grid, settings=None, res_mm=50.0, margin_m=1.0, sure_walls=False, specks=None, zones=None):
        """sure_walls: only sure walls count (SURE_LUT); specks: fill unknown specks (default: = sure_walls).
        zones: a zones.ZoneView (#70): virtual walls / zone edges become walls before inflation, no-go insides and
        the outside of the fence are blocked after it, slow zones cost more."""
        S = dict(DEFAULTS, **(settings or {}))
        self.lut = SURE_LUT if sure_walls else CLASS_LUT
        self.g = grid
        f = max(1, int(round(res_mm / grid.res)))
        self.f, self.res = f, grid.res * f
        bb = grid.bbox_tiles()
        T = grid.T
        if bb is None:
            bb = (grid.ntx // 2, grid.nty // 2, grid.ntx // 2, grid.nty // 2)
        zb = zones.bbox() if zones else None
        if zb:                                 # the costmap covers the zones too (a maze drawn on unmapped floor)
            tm = grid.res * T
            zt = [int((zb[0] - grid.x0) // tm), int((zb[1] - grid.y0) // tm), int((zb[2] - grid.x0) // tm),
                  int((zb[3] - grid.y0) // tm)]
            zt = [max(0, min(grid.ntx - 1, zt[0])), max(0, min(grid.nty - 1, zt[1])),
                  max(0, min(grid.ntx - 1, zt[2])), max(0, min(grid.nty - 1, zt[3]))]
            bb = (min(bb[0], zt[0]), min(bb[1], zt[1]), max(bb[2], zt[2]), max(bb[3], zt[3]))
        m = int(math.ceil(margin_m * 1000.0 / (grid.res * T)))
        tx0, ty0 = max(0, bb[0] - m), max(0, bb[1] - m)
        tx1, ty1 = min(grid.ntx - 1, bb[2] + m), min(grid.nty - 1, bb[3] + m)
        self.fx0, self.fy0 = tx0 * T, ty0 * T                     # fine cell of coarse (0, 0)
        self.w = ((tx1 + 1) * T - self.fx0) // f
        self.h = ((ty1 + 1) * T - self.fy0) // f
        self.x0 = grid.x0 + self.fx0 * grid.res                    # world mm of the coarse (0, 0) corner
        self.y0 = grid.y0 + self.fy0 * grid.res
        self._classify()
        if sure_walls if specks is None else specks:
            self._fill_specks()
        self.centre = False          # squeeze maps (set by the Planner): shortcuts keep to the gap centre line (#68)
        self.hard_mm = S["robot_radius_mm"] + S["margin_mm"]
        self.inner_r = (S["robot_radius_mm"] + min(30.0, S["margin_mm"])) / self.res
        if zones:
            zones.mark_lines(self)
        self._inflate(S["robot_radius_mm"] + S["margin_mm"], S["soft_mm"])
        if zones:
            zones.mark_areas(self, BLOCKED, MARGIN)
        self.unknown_ok = bool(S["unknown_ok"])

    # ---------------------------------------------------------- build
    def _classify(self):
        g, W, f, w = self.g.cells, self.g.W, self.f, self.w
        nb = w * f
        state = bytearray()
        for cy in range(self.h):
            acc = 0
            for k in range(f):
                j = self.fy0 + cy * f + k
                s = j * W + self.fx0
                acc |= int.from_bytes(bytes(g[s:s + nb]).translate(self.lut), "big")
            for k in range(1, f):                  # OR each byte with the f-1 before it
                acc |= acc >> (8 * k)
            row = acc.to_bytes(nb, "big")[f - 1::f]
            state += row.translate(FINAL_LUT)
        self.state = state                          # 0 unknown, 1 free, 2 occupied

    def _fill_specks(self):
        """Squeeze maps: an unknown cell with 3+ free 4-neighbours counts as free (a weakly seen speck in a
        doorway would cut the only way through; explore never travels through unknown)."""
        w, n, st = self.w, len(self.state), self.state
        fill = []
        i = st.find(0)
        while i >= 0:
            ix = i % w
            k = 0
            for o in (1, -1, w, -w):
                j = i + o
                if 0 <= j < n and abs(j % w - ix) <= 1 and st[j] == 1:
                    k += 1
            if k >= 3:
                fill.append(i)
            i = st.find(0, i + 1)
        for i in fill:
            st[i] = 1

    def _inflate(self, hard_mm, soft_mm):
        w, h = self.w, self.h
        cost = bytearray(w * h)
        rh = hard_mm / self.res
        rs = (hard_mm + soft_mm) / self.res
        occ = []
        st = self.state
        i = st.find(2)
        while i >= 0:
            occ.append(i)
            i = st.find(2, i + 1)
        self.n_occ = len(occ)
        for r, val in ((rs, 20), (rh, MARGIN), (self.inner_r, BLOCKED)):
            ri = int(math.floor(r))
            spans = [(dy, int(math.floor(math.sqrt(max(0.0, r * r - dy * dy))))) for dy in range(-ri, ri + 1)]
            fill = bytes([val]) * (2 * ri + 3)
            for i in occ:
                cx, cy = i % w, i // w
                for dy, dx in spans:
                    y = cy + dy
                    if 0 <= y < h:
                        a, b = max(0, cx - dx), min(w, cx + dx + 1)
                        cost[y * w + a:y * w + b] = fill[:b - a]
        self.cost = cost

    # ---------------------------------------------------------- coordinates
    def cell(self, x, y):
        cx, cy = int((x - self.x0) // self.res), int((y - self.y0) // self.res)
        return cy * self.w + cx if 0 <= cx < self.w and 0 <= cy < self.h else None

    def center(self, i):
        return self.x0 + (i % self.w + 0.5) * self.res, self.y0 + (i // self.w + 0.5) * self.res

    def passable(self, i, unknown_ok):
        return self.cost[i] < MARGIN and (unknown_ok or self.state[i] == 1)

    def _step_cost(self, i, unknown_ok):
        c = 1.0 + self.cost[i] / 10.0          # soft band: x3
        if self.state[i] == 0:
            c += 1.0                            # unknown: x2
        return c

    def nearest_passable(self, i, unknown_ok, max_mm=500.0):
        """i, or the closest passable cell within max_mm (BFS), or None."""
        if i is None:
            return None
        if self.passable(i, unknown_ok):
            return i
        w, n = self.w, len(self.cost)
        lim = int(max_mm / self.res)
        cx0, cy0 = i % w, i // w
        seen, q = {i}, [i]
        while q:
            nq = []
            for j in q:
                for d in (1, -1, w, -w):
                    k = j + d
                    if k < 0 or k >= n or k in seen or abs(k % w - j % w) > 1:
                        continue
                    if abs(k % w - cx0) > lim or abs(k // w - cy0) > lim:
                        continue
                    if self.passable(k, unknown_ok):
                        return k
                    seen.add(k)
                    nq.append(k)
            q = nq
        return None

    def _neighbours(self):
        w = self.w
        return ((1, 1.0), (-1, 1.0), (w, 1.0), (-w, 1.0), (w + 1, SQ2), (w - 1, SQ2), (-w + 1, SQ2), (-w - 1, SQ2))

    # ---------------------------------------------------------- search
    def astar(self, start_xy, goal_xy, unknown_ok=None, escape_mm=350.0):
        """World path [(x, y), ...] from start to goal (cell centres), or None. The robot may start
        inside the inflated zone (it can drive out within escape_mm of the start)."""
        unknown_ok = self.unknown_ok if unknown_ok is None else unknown_ok
        s = self.cell(*start_xy)
        g = self.nearest_passable(self.cell(*goal_xy), unknown_ok)
        if s is None or g is None:
            return None
        w, n, cost, state = self.w, len(self.cost), self.cost, self.state
        sx, sy = s % w, s // w
        esc = (escape_mm / self.res) ** 2
        gx, gy = g % w, g // w
        nbrs = self._neighbours()
        best = {s: 0.0}
        parent = {s: None}
        heap = [(0.0, 0.0, s)]
        while heap:
            _, d, i = heapq.heappop(heap)
            if i == g:
                break
            if d > best.get(i, 1e18):
                continue
            ix = i % w
            for off, step in nbrs:
                k = i + off
                if k < 0 or k >= n:
                    continue
                kx = k % w
                if abs(kx - ix) > 1:
                    continue
                c = cost[k]
                ky = k // w
                if c >= MARGIN or (state[k] != 1 and not unknown_ok):
                    d2 = (kx - sx) ** 2 + (ky - sy) ** 2
                    if d2 > esc or state[k] == 2 or (c == BLOCKED and d2 > 4):
                        continue
                    c = 60                       # escaping the inflated zone near the start
                nd = d + step * (1.0 + c / 10.0 + (1.0 if state[k] == 0 else 0.0))
                if nd < best.get(k, 1e18):
                    best[k] = nd
                    parent[k] = i
                    dx, dy = abs(kx - gx), abs(ky - gy)
                    hh = (dx + dy) + (SQ2 - 2) * min(dx, dy)
                    heapq.heappush(heap, (nd + hh, nd, k))
        if g not in parent:
            return None
        cells = []
        i = g
        while i is not None:
            cells.append(i)
            i = parent[i]
        cells.reverse()
        return self.simplify(cells, unknown_ok)

    def clear_line(self, a, b, unknown_ok, allow_start_zone=False, escape_mm=350.0):
        """True if the straight segment between cells a and b crosses no blocked (or, unless
        unknown_ok, unknown) cell. allow_start_zone: a is the robot; blocked cells within escape_mm
        of it are fine (it is already in the inflated zone and has to drive out)."""
        w = self.w
        ax, ay, bx, by = a % w, a // w, b % w, b // w
        dx, dy = bx - ax, by - ay
        n = max(abs(dx), abs(dy))
        esc = (escape_mm / self.res) ** 2 if allow_start_zone and self.cost[a] >= MARGIN else -1
        for k in range(1, n + 1):
            ox, oy = round(k * dx / n), round(k * dy / n)
            i = (ay + oy) * w + ax + ox
            c = self.cost[i]
            if c >= MARGIN and self.state[i] != 2 and ox * ox + oy * oy <= (esc if c == MARGIN else min(esc, 4)):
                continue
            if c >= MARGIN:
                return False
            if self.state[i] == 2 or (not unknown_ok and self.state[i] == 0):
                return False
        return True

    def _clearance(self, i, cache, R):
        """Distance (cells, capped at R) from cell i to the nearest occupied cell."""
        v = cache.get(i)
        if v is None:
            w, st = self.w, self.state
            cx, cy = i % w, i // w
            best = R * R
            for y in range(max(0, cy - int(R)), min(self.h, cy + int(R) + 1)):
                row = y * w
                for x in range(max(0, cx - int(R)), min(w, cx + int(R) + 1)):
                    if st[row + x] == 2:
                        d = (x - cx) ** 2 + (y - cy) ** 2
                        if d < best:
                            best = d
            v = cache[i] = math.sqrt(best)
        return v

    def _centred_line(self, a, b, floor, cache, R):
        """Squeeze maps: the straight segment a-b never gets closer to a wall than `floor` cells."""
        w = self.w
        ax, ay, bx, by = a % w, a // w, b % w, b // w
        dx, dy = bx - ax, by - ay
        n = max(abs(dx), abs(dy))
        for k in range(n + 1):
            if self._clearance((ay + round(k * dy / n)) * w + ax + round(k * dx / n), cache, R) < floor:
                return False
        return True

    def simplify(self, cells, unknown_ok):
        """Line-of-sight pruning: keep a cell only when the next one can't be seen from the last kept.
        Squeeze maps (#68): a shortcut may not pass closer to a wall than the route itself does between its
        ends (less half a cell), so legs stay on the gap's centre line instead of cutting the jamb."""
        if len(cells) <= 2:
            return [self.center(i) for i in cells]
        cache, R = {}, (self.hard_mm + 80.0) / self.res
        must = set()
        if self.centre:                  # per narrow stretch: its centre cell + a lead-in / lead-out stay waypoints
            lead = int(round(150.0 / self.res))
            tight = (self.hard_mm + 40.0) / self.res
            run = []
            for k, c in enumerate(cells + [None]):
                if c is not None and self._clearance(c, cache, R) < tight:
                    run.append(k)
                elif run:
                    must.update((max(0, run[0] - lead), run[len(run) // 2], min(len(cells) - 1, run[-1] + lead)))
                    run = []
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = min([m for m in must if m > i] + [len(cells) - 1])    # never skip over a must-keep cell
            while j > i + 1:
                if self.clear_line(cells[i], cells[j], unknown_ok, allow_start_zone=(i == 0)):
                    if not self.centre:
                        break
                    floor = min(self._clearance(c, cache, R) for c in cells[i:j + 1]) - 0.5
                    if self._centred_line(cells[i], cells[j], floor, cache, R):
                        break
                j -= 1
            out.append(cells[j])
            i = j
        return [self.center(i) for i in out]
    def path_clear(self, pts, unknown_ok=True):
        """Is a world polyline still free of obstacles (inflated)? The first point (the robot) may be
        in the inflated zone."""
        cells = [self.cell(x, y) for x, y in pts]
        if any(c is None for c in cells):
            return False
        return all(self.clear_line(a, b, unknown_ok, allow_start_zone=(k == 0))
                   for k, (a, b) in enumerate(zip(cells, cells[1:])))

    def _opens_up(self, i, open_cm, look_mm=300.0):
        """Squeezing: does frontier cell i lead into open space of open_cm? Either it lies there itself, or
        straight on into the unknown beside it (up to look_mm, about the robot's radius + the normal margin)
        there is unknown space outside open_cm's wall margin. Seen through a narrow doorway at an angle, the
        far room shows only a strip along its own wall (all inside the margin) - the room itself is beyond."""
        j = open_cm.cell(*self.center(i))
        if j is None or open_cm.state[j] == 2:
            return False                        # a weak wall hit (free here, a wall in the normal map)
        if open_cm.cost[j] < MARGIN:
            return True
        w, n, st = self.w, len(self.state), self.state
        ix, iy = i % w, i // w
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for k in range(1, int(look_mm / self.res) + 1):
                x, y = ix + dx * k, iy + dy * k
                if not (0 <= x < w and 0 <= y * w + x < n) or st[y * w + x] != 0:
                    break                       # known space or a wall ends the look (the first step must be unknown)
                j = open_cm.cell(*self.center(y * w + x))
                if j is None or open_cm.state[j] == 2:
                    break                       # a wall in the normal map: not through it
                if open_cm.cost[j] < MARGIN:
                    return True
        return False

    def _squeeze_goal(self, front, dist, parent, blacklist, black_r_mm, open_cm):
        """Squeezing (#62): the nearest frontier cell that leads into open space (outside the wall margin of
        open_cm, the normal costmap; see _opens_up) - i.e. only reachable through a narrow gap. Frontier bits
        along walls (inside the margin) are not worth squeezing for. One check per 100 mm block."""
        w, br2 = self.w, black_r_mm ** 2
        blocks = set()
        for i in sorted(set(front), key=lambda i: dist[i]):
            b = (i % w // 4, i // w // 4)
            if b in blocks:
                continue
            blocks.add(b)
            if len(blocks) > SQUEEZE_CHECKS:
                break
            cx, cy = self.center(i)
            if any((cx - bx) ** 2 + (cy - by) ** 2 < br2 for bx, by in blacklist):
                continue
            if not self._opens_up(i, open_cm):
                continue
            cells = []
            k = i
            while k is not None:
                cells.append(k)
                k = parent[k]
            cells.reverse()
            return [(dist[i] * self.res, (cx, cy), self.simplify(cells, False), 1)]
        return []

    def frontiers(self, start_xy, min_mm=200.0, blacklist=(), black_r_mm=350.0, escape_mm=350.0, open_cm=None):
        """Reachable frontier clusters, nearest first: [(dist_mm, goal_xy, path_xy, size_cells)].
        Frontier = free, unblocked cell with an unknown 4-neighbour. Travel only through free cells.
        open_cm (#62, squeezing): only the nearest frontier cell in open space of open_cm (see _squeeze_goal)."""
        s = self.cell(*start_xy)
        if s is None:
            return []
        w, n, cost, state = self.w, len(self.cost), self.cost, self.state
        sx, sy = s % w, s // w
        esc = (escape_mm / self.res) ** 2
        nbrs = self._neighbours()
        dist = {s: 0.0}
        parent = {s: None}
        heap = [(0.0, s)]
        front = []
        while heap:
            d, i = heapq.heappop(heap)
            if d > dist[i]:
                continue
            ix = i % w
            if cost[i] < MARGIN and state[i] == 1:
                for o in (1, -1, w, -w):
                    k = i + o
                    if 0 <= k < n and abs(k % w - ix) <= 1 and state[k] == 0:
                        front.append(i)
                        break
            for off, step in nbrs:
                k = i + off
                if k < 0 or k >= n or abs(k % w - ix) > 1:
                    continue
                c = cost[k]
                if state[k] != 1 or c >= MARGIN:
                    # near the start the robot's own footprint is unseen / inflated: let it out
                    kx, ky = k % w, k // w
                    d2 = (kx - sx) ** 2 + (ky - sy) ** 2
                    if state[k] == 2 or d2 > esc or (c == BLOCKED and d2 > 4):
                        continue
                    c = max(c, 60) if c >= MARGIN else c
                nd = d + step * (1.0 + c / 10.0)
                if nd < dist.get(k, 1e18):
                    dist[k] = nd
                    parent[k] = i
                    heapq.heappush(heap, (nd, k))
        if open_cm is not None:
            return self._squeeze_goal(front, dist, parent, blacklist, black_r_mm, open_cm)
        # cluster (8-connected) the frontier cells
        fset = set(front)
        clusters = []
        while fset:
            seed = fset.pop()
            comp, q = [seed], [seed]
            while q:
                i = q.pop()
                ix = i % w
                for off, _ in nbrs:
                    k = i + off
                    if k in fset and abs(k % w - ix) <= 1:
                        fset.discard(k)
                        comp.append(k)
                        q.append(k)
            clusters.append(comp)
        out = []
        min_cells = max(1, int(min_mm / self.res))
        br2 = black_r_mm ** 2
        for comp in clusters:
            if len(comp) < min_cells:
                continue
            ok = [i for i in comp if not any((self.center(i)[0] - bx) ** 2 + (self.center(i)[1] - by) ** 2 < br2
                                              for bx, by in blacklist)]
            if not ok:
                continue
            goal = min(ok, key=lambda i: dist[i])
            cells = []
            i = goal
            while i is not None:
                cells.append(i)
                i = parent[i]
            cells.reverse()
            out.append((dist[goal] * self.res, self.center(goal), self.simplify(cells, False), len(comp)))
        out.sort(key=lambda t: t[0])
        return out


class Planner:
    """Bridge side: jobs, replanning, explore loop. bridge: the Bridge (send / handle_command / tlm / nav)."""

    def __init__(self, bridge, nav_names, clock=time.monotonic, sync=False):
        self.b = bridge
        self.clock = clock            # sim time in offline runs
        self.sync = sync              # run jobs inline (offline runs / tests)
        self.nav_names = nav_names
        self.settings = self._load_settings()
        self.boost = 0.0              # extra margin after the robot got stuck (#62), decays on success
        self.replan_at = None         # stuck: replan once the robot has backed off
        self.squeeze = False          # the current route keeps only radius + squeeze_mm (narrow gap)
        self.squeeze_left = SQUEEZE_TRIES   # explore: squeeze goals left before "done" (see SQUEEZE_TRIES)
        self.sq_free = None           # free cells in the squeeze map at the last squeeze plan
        self._sq_free_now = None      # (set by the worker with each squeeze explore plan)
        self.mode = "idle"            # idle | goto | explore
        self.goals = []               # goto: user goals (map frame)
        self.path = []                # current route, map frame (first point = robot when planned)
        self.sent_at = 0.0
        self.started_at = 0.0
        self.job = None
        self.last_check = 0.0
        self.last_plan = 0.0
        self.blocked_plans = 0
        self.blacklist = []           # explore: frontier goals given up on (map frame)
        self.goal = None              # explore: current frontier goal
        self.map_changed = False
        self.status = ""
        self.gen = 0                  # bumps with every user command (drops stale job results)
        self.arm = False              # the next mode request carries the user's arm request (see command)
        self.pending = None
        self.dest = None              # goto: name of the saved place being driven to (#53)
        self.scan_wait = None         # explore: (since, stopscan.seq) while waiting for a stop-and-scan (#71)
        self.holding = False          # explore: stopped on the way for a scan (then it plans again)
        self.sent_goal = None         # explore: the frontier goal of the route last sent
        self.scan_seen, self.last_scan_t = -1, 0.0
        ss = getattr(getattr(bridge, "mapper", None), "stopscan", None)
        if ss is not None:
            ss.listeners.append(self.on_correction)
        self.solve = None             # goto: the maze being solved (its exit gap counts as a wall, #70)
        self.solve_t0 = 0.0
        self.zone_hits = 0            # watchdog stops in this job (#70)

    # ---------------------------------------------------------- frames
    def robot_map_pose(self):
        t = self.b.tlm
        return self.b.mapper.map_pose(t["x_mm"], t["y_mm"], t["heading_deg"]) if t else None

    def to_odom(self, pts):
        dx, dy, dt = self.b.mapper.corr
        c, s = math.cos(math.radians(-dt)), math.sin(math.radians(-dt))
        return [(round(c * (x - dx) - s * (y - dy), 1), round(s * (x - dx) + c * (y - dy), 1)) for x, y in pts]

    def costmap(self, kind=None):
        """kind: "boost" (margin + the stuck boost), "normal", "squeeze" (radius + squeeze_mm only);
        None = what the current route was planned with."""
        if kind is None:
            kind = "squeeze" if self.squeeze else ("boost" if self.boost > 0 else "normal")
        s = dict(self.settings)
        g = self.b.mapper.grid
        zones = self.zone_view()
        if kind == "boost":
            s["margin_mm"] += self.boost
        elif kind == "squeeze":
            # at the map's own 25 mm cells (at 50 mm a 280 mm doorway rounds shut); sure walls only when the
            # map is not range-weighted (see SURE_LUT)
            s["margin_mm"] = min(s["margin_mm"], s["squeeze_mm"])
            rw = self.b.mapper.settings.get("range_weight", False)
            cm = CostMap(g, s, res_mm=g.res, sure_walls=not rw, specks=True, zones=zones)
            cm.centre = True
            return cm
        return CostMap(g, s, zones=zones)

    def zone_view(self):
        """The map's zones (#70) as the planner sees them (Solve: that maze's exit closed), or None."""
        z = getattr(self.b.mapper, "zones", None)
        if z is None or not z.items:
            return None
        return z.view(close_exit=self.solve) if self.solve else z.view()

    def zone_stop(self, why):
        """The bridge's watchdog stopped the robot (HOLD): it is in a no-go zone / at a virtual wall / the fence.
        Replan in a moment (the route keeps the margin again); give up after 3 in one job."""
        if self.mode == "idle":
            return
        self.zone_hits += 1
        if self.zone_hits > 3:
            self.stop("stopped: %s (3 times - gave up)" % why)
            return
        self.replan_at = self.clock() + 1.5
        self.last_check = self.clock()
        self._set_status("stopped: %s - replanning" % why)

    def zones_changed(self):
        """Layers edited: check the route against them at the next tick."""
        self.last_check = 0.0
        self.last_plan = min(self.last_plan, self.clock() - 1.5)

    def _costmaps(self):
        """Costmaps to try in turn: boosted (after a stuck), normal, then squeezed (narrow gaps)."""
        kinds = (["boost"] if self.boost > 0 else []) + ["normal"]
        if self.settings["squeeze_mm"] < self.settings["margin_mm"]:
            kinds.append("squeeze")
        for k in kinds:
            yield k, self.costmap(k)

    # ---------------------------------------------------------- settings (persisted, #62)
    def _settings_path(self):
        d = getattr(getattr(self.b, "mapper", None), "dir", None)
        return os.path.join(d, SETTINGS_FILE) if d else None

    def _load_settings(self):
        p = self._settings_path()
        try:
            with open(p, encoding="utf-8") as f:
                return clean_settings(json.load(f), DEFAULTS)
        except (OSError, ValueError, TypeError):
            return dict(DEFAULTS)

    def _save_settings(self):
        p = self._settings_path()
        if not p:
            return
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                json.dump(self.settings, f, indent=1)
        except OSError as e:
            self.b.say("planner: settings not saved: %s" % e)

    # ---------------------------------------------------------- robot log (#62)
    def on_log(self, text):
        """A robot log line. "nav: stuck ...": replan with a larger margin once it has backed off."""
        if not text.startswith("nav: stuck") or self.mode == "idle":
            return
        self.boost = min(STUCK_BOOST_MAX, self.boost + STUCK_BOOST_MM)
        self.squeeze = False
        self.replan_at = self.clock() + 5.0       # its back-off ends within 5 s
        self._set_status("robot stuck: backing off, replanning with +%d mm margin" % self.boost)

    def _success(self):
        """A goal reached: the stuck boost decays."""
        self.boost = max(0.0, self.boost - BOOST_DECAY_MM)

    # ---------------------------------------------------------- messages
    def msg(self):
        return json.dumps({"t": "plan", "mode": self.mode, "status": self.status,
                           "dest": self.dest if self.mode == "goto" else None,
                           "solve": {"maze": self.solve, "elapsed": round(self.clock() - self.solve_t0, 1)}
                           if self.solve and self.mode == "goto" else None,
                           "path": [[round(x), round(y)] for x, y in self.to_odom(self.path)],
                           "goals": [[round(x), round(y)] for x, y in self.to_odom(self.goals)],
                           "settings": self.settings, "boost": self.boost, "squeeze": self.squeeze},
                          separators=(",", ":"))

    def _push(self):
        self.b.broadcast(self.msg())

    def _set_status(self, text, log=True):
        if text != self.status:
            self.status = text
            if log and text:
                self.b.say("planner: " + text)
        self._push()

    # ---------------------------------------------------------- commands
    def command(self, m, who="?"):
        """{"t": "plan", "action": "goto", "points": [[x, y], ...], "go": true}   (odometry frame, like goto)
           {"t": "plan", "action": "explore"} | {"action": "stop"} | {"action": "set", "settings": {...}}
           goto / explore with "arm": true: the user confirmed arming (no remote): the planner's first
           mode request carries the arm request; later ones never arm."""
        act = m.get("action")
        if act == "set":
            new = clean_settings(m.get("settings"), self.settings)
            if new != self.settings:
                self.settings = new
                self._save_settings()
            self._push()
            return None
        if act == "stop":
            self.stop("stopped by %s" % who, idle_robot=False)
            return None
        if not self.b.tlm:
            return {"t": "error", "text": "no robot pose yet"}
        if act == "solve":
            # Solve a maze (#70): drive to its exit cell; its exit gap counts as a wall, so the route goes through
            # the maze from the entrance (not round it); the time is shown and the best one kept with the maze
            z = getattr(self.b.mapper, "zones", None)
            it = z.find(str(m.get("maze"))) if z else None
            if not it or it["kind"] != "maze":
                return {"t": "error", "text": "no maze %s" % m.get("maze")}
            self._begin_goto([tuple(it["exit"])], "maze exit", bool(m.get("arm")), solve=it["id"])
            return None
        if act == "home" or (act == "goto" and m.get("place")):
            # a saved place (#53): {"action": "goto", "place": "Couch"} or {"action": "home"}
            name = HOME if act == "home" else str(m.get("place"))
            xy = self.b.mapper.place_xy(name)
            if xy is None:
                return {"t": "error", "text": "no place called %s" % name}
            zv = self.zone_view()
            bad = zv.point_blocked(*xy) if zv else None
            if bad:
                return {"t": "error", "text": "%s: %s" % (name, bad)}
            self._begin_goto([xy], HOME if name.lower() == HOME.lower() else self.b.mapper.places.find(name)["name"],
                             bool(m.get("arm")))
            return None
        if act == "goto":
            dx, dy, dt = self.b.mapper.corr
            c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))
            goals = []
            for p in m.get("points") or []:
                try:
                    x, y = float(p[0]), float(p[1])
                except (TypeError, ValueError, IndexError):
                    continue
                goals.append((c * x - s * y + dx, s * x + c * y + dy))
            if not goals:
                return {"t": "error", "text": "no goal"}
            zv = self.zone_view()
            bad = next((r for r in (zv.point_blocked(*g) for g in goals) if r), None) if zv else None
            if bad:
                return {"t": "error", "text": "not sent: " + bad}
            self._begin_goto(goals, None, bool(m.get("arm")))
            return None
        if act == "explore":
            self.mode, self.goals, self.blacklist, self.goal = "explore", [], [], None
            self.replan_at, self.squeeze = None, False
            self.squeeze_left, self.sq_free = SQUEEZE_TRIES, None
            self.started_at = self.last_scan_t = self.clock()
            self.holding, self.sent_goal, self.scan_wait = False, None, None
            self.blocked_plans = 0
            self.solve, self.zone_hits = None, 0
            self._set_status("exploring")
            if self.b.tlm["mode"] != lp.MODE_EXPLORE:     # arm now (no remote): the sweep runs while armed
                self.b.handle_command({"t": "set_mode", "mode": lp.MODE_EXPLORE, "arm": bool(m.get("arm"))},
                                      "planner")
            self.arm = False
            self._start_job(self._plan_explore, go=True, force=True)
            return None
        return {"t": "error", "text": "unknown plan action %r" % act}

    def _begin_goto(self, goals, dest, arm, solve=None):
        """Plan + drive to map-frame goals. dest: the saved place's name (shown on the map), or None.
        solve: a maze id (Solve, #70)."""
        self.mode, self.goals, self.blacklist = "goto", goals[:20], []
        self.replan_at, self.squeeze = None, False
        self.dest = dest
        self.solve, self.solve_t0, self.zone_hits = solve, self.clock(), 0
        self.arm = arm
        self.started_at = self.clock()
        self.blocked_plans = 0
        self._start_job(self._plan_goto, go=True, force=True)

    def robot_go_home(self, target_odom):
        """The robot just started its own GO_HOME (the remote's Actions > Go home, or the station's event):
        a straight line to target_odom (first waypoint or the origin) in GOTO. When the bridge can plan, it
        takes over with a route around obstacles to the saved Home (else to that same target). It never arms:
        only an armed robot (the firmware requested GOTO) and no E-stop latch."""
        b = self.b
        t = b.tlm
        if b.estop or getattr(b, "robot_estop", False) or not t or not (t["status"] & lp.TLM_ARMED):
            return                               # the robot ignored the request (latched / disarmed)
        if not b.mapper.grid.touched:
            self.stop()
            b.say("planner: GO_HOME from the robot: no map yet, it drives the straight line")
            return
        xy = b.mapper.places.find(HOME)
        goal = (xy["x"], xy["y"]) if xy else b.mapper.map_pose(target_odom[0], target_odom[1], 0.0)[:2]
        b.say("planner: GO_HOME from the robot: planning a route to %s" % ("Home" if xy else "its target"))
        self._begin_goto([goal], HOME if xy else "Home", False)

    def stop(self, why="", idle_robot=False):
        """Forget the job. The robot keeps its last waypoints unless idle_robot (E-stop does that)."""
        if self.mode == "idle" and not self.path:
            return
        was = self.mode
        self.gen += 1
        self.pending = None
        self.arm = False
        self.replan_at = None
        self.squeeze = False
        self.holding, self.scan_wait = False, None
        self.solve = None
        self.mode, self.path, self.goals, self.goal = "idle", [], [], None
        if was != "idle" and why:
            self._set_status(why)
        else:
            self._push()

    def cancel_if_user_goto(self, who):
        """A plain goto / clear from a client takes over: the planner lets go."""
        if who != "planner" and self.mode != "idle":
            self.stop("manual waypoints from %s" % who)

    # ---------------------------------------------------------- jobs (worker thread)
    def _start_job(self, fn, go=False, check=False, force=False):
        """Run fn(pose) in the worker; force (a user command): bump the generation so results of
        older jobs are dropped, and queue it if a job is still running."""
        if force:
            self.gen += 1
        if self.job and not self.job.done():
            if force:
                self.pending = (fn, go)
            return False
        pose = self.robot_map_pose()
        if pose is None:
            return False
        gen = self.gen
        if not check:
            self.last_plan = self.clock()
        if self.sync:
            fut = concurrent.futures.Future()
            try:
                fut.set_result(fn(pose))
            except Exception as e:
                fut.set_exception(e)
            self._job_done(fut, go, gen)
            return True
        loop = asyncio.get_running_loop()
        self.job = loop.run_in_executor(None, fn, pose)
        self.job.add_done_callback(lambda fut: self._job_done(fut, go, gen))
        return True

    def _job_done(self, fut, go, gen):
        if self.pending:
            fn, pgo = self.pending
            self.pending = None
            self.job = None
            if self.mode != "idle":
                self._start_job(fn, pgo)
            return
        if gen != self.gen:
            return                    # a newer command came in meanwhile
        try:
            res = fut.result()
        except Exception as e:      # a planner bug must never take the bridge down
            import traceback
            traceback.print_exc()
            res = ("fail", "planner error: %s" % e)
        if self.mode == "idle":
            return
        kind = res[0]
        if kind == "path":
            _, path, status, cmk = res
            squeeze = cmk == "squeeze"
            if squeeze and not self.squeeze:
                # no speed field in MSG_GOTO: the robot's side check (nav.side_min_mm) slows it in the gap
                self.b.say("planner: narrow gap - squeezing through with %d mm clearance (the robot slows "
                           "down beside the walls)" % self.settings["squeeze_mm"])
            self.squeeze = squeeze
            if self.mode == "explore" and squeeze:
                now_free = self._sq_free_now
                grew = self.sq_free is not None and now_free is not None and now_free - self.sq_free > SQUEEZE_GROW
                self.squeeze_left = SQUEEZE_TRIES if grew else self.squeeze_left - 1
                self.sq_free = now_free
            elif self.mode == "explore":
                self.squeeze_left, self.sq_free = SQUEEZE_TRIES, None
            if self.mode == "explore" and self._scan_on() and self.sent_goal is not None and \
                    self.goal != self.sent_goal and self.clock() - self.last_scan_t > SCAN_EVERY_S:
                # a new frontier goal and no scan for a while: stop here and scan first, then plan again (#71)
                t = self.b.tlm
                self.b.handle_command({"t": "goto", "points": [[t["x_mm"], t["y_mm"]]], "replace": True}, "planner")
                self.sent_at, self.sent_n = self.clock(), 1
                self.path, self.holding, self.scan_wait, self.sent_goal = [], True, None, None
                self._set_status("exploring: stopped to scan and check the pose", log=False)
                return
            self.path = path
            self.sent_goal = self.goal
            self._send(path[1:] or path[-1:], go)
            if squeeze:
                status += " (squeezing through a narrow gap)"
            elif cmk == "boost":
                status += " (+%d mm margin after a stuck)" % self.boost
            self._set_status(status, log=False)
        elif kind == "ok":
            pass
        elif kind == "wait":
            self.path = []
            self._set_status(res[1], log=False)
        elif kind == "blacklist":     # explore: that frontier is unreachable / done; try again soon
            self.blacklist.append(res[1])
            self.last_plan = 0.0
        elif kind == "done":
            self.mode, self.path, self.goal = "idle", [], None
            self.b.handle_command({"t": "set_mode", "mode": lp.MODE_BALANCE}, "planner")
            self._set_status(res[1])
        else:   # fail
            self.mode, self.path, self.goal = "idle", [], None
            if self.solve:
                self.solve = None
                res = (res[0], "maze: " + res[1])
            self._set_status(res[1])

    def on_correction(self, entry):
        """A stop-and-scan moved the map frame (#71): the robot's waypoints are in its odometry frame, so the
        rest of the route goes again, converted with the new correction (the robot is standing still)."""
        rest = self._remaining() if self.mode != "idle" else []
        if rest:
            pose = self.robot_map_pose()
            if pose:
                self.path = [pose[:2]] + rest
                self._send(rest, False)
        self._push()

    def _scan_on(self):
        ss = getattr(self.b.mapper, "stopscan", None)
        return ss is not None and self.settings["scan_pause"] and bool(self.b.mapper.settings.get("stop_scan"))

    def _scan_hold(self, now):
        """Explore, stopped (at a frontier goal, or on the way): True while waiting for the stop-and-scan
        (scan_pause, #71)."""
        if not self._scan_on():
            return False
        ss = self.b.mapper.stopscan
        if self.scan_wait is None:
            if ss.done and not ss.matching:
                return False          # this stop was matched already
            self.scan_wait = (now, ss.seq)
            self._set_status("exploring: scanning to check the pose", log=False)
        t0, seq = self.scan_wait
        return ss.seq == seq and now - t0 < SCAN_WAIT_S

    def _send(self, pts, go):
        self.scan_wait = None
        odom = self.to_odom(pts)
        self.b.handle_command({"t": "goto", "points": [list(p) for p in odom], "replace": True}, "planner")
        self.sent_at = self.clock()
        self.sent_n = len(odom)
        if go:
            mode = lp.MODE_GOTO if self.mode == "goto" else lp.MODE_EXPLORE
            if not self.b.tlm or self.b.tlm["mode"] != mode:
                self.b.handle_command({"t": "set_mode", "mode": mode, "arm": self.arm}, "planner")
            self.arm = False

    def _plan_goto(self, pose):
        start = pose[:2]
        fail = None
        for kind, cm in self._costmaps():
            route = [start]
            for gpt in self.goals:
                leg = cm.astar(route[-1], gpt)
                if leg is None:
                    fail = fail or gpt
                    route = None
                    break
                leg[-1] = gpt if cm.passable(cm.cell(*gpt) or 0, True) else leg[-1]
                route += leg[1:]
            if route:
                n = len(route) - 1
                return ("path", route, "route: %d waypoint%s" % (n, "" if n == 1 else "s"), kind)
        return ("fail", "no route to %.1f, %.1f m" % (fail[0] / 1000, fail[1] / 1000))

    def _plan_explore(self, pose):
        fr, kind, normal, held = None, "normal", None, None
        for kind, cm in self._costmaps():
            if kind == "squeeze" and self.squeeze_left <= 0:
                break                 # tried enough narrow spots: the rest is done
            fr = cm.frontiers(pose[:2], self.settings["frontier_min_mm"], self.blacklist,
                              open_cm=normal if kind == "squeeze" else None)
            if kind == "normal":
                normal = cm
            if fr and kind != "squeeze" and self.squeeze:
                i = cm.cell(*pose[:2])
                if i is None or cm.cost[i] >= MARGIN:
                    # squeezing and still in the gap: the wide-margin map only gets out of it through its
                    # escape zone, whose first leg may cut a jamb corner - get through on the squeeze map first
                    held, fr = held or (fr, kind), None
                    continue
            if fr:
                if kind == "squeeze":
                    self._sq_free_now = cm.state.count(1)
                break
        if not fr and held:
            fr, kind = held
        if not fr:
            if self.clock() - self.started_at < 8.0:
                return ("wait", "exploring: looking around")
            return ("done", "explore done: no reachable frontier left")
        dist, goal, path, size = fr[0]
        if dist < 120 or len(path) < 2:
            # standing at it and it is still unknown beyond: the sensors can't see it; skip it
            return ("blacklist", goal)
        self.goal = goal
        path[0] = pose[:2]
        return ("path", path, "exploring: frontier %.1f m away (%d left)" % (dist / 1000, len(fr)), kind)

    def _check(self, pose):
        """Periodic: is the remaining route still clear? explore: is the goal still a frontier?"""
        cm = self.costmap()
        rest = self._remaining()
        # (a squeeze goal may be a spot near a doorway to look from, not a frontier cell itself: drive there)
        if self.mode == "explore" and self.goal is not None and not self.squeeze:
            i = cm.cell(*self.goal)
            still = i is not None and cm.state[i] == 1 and cm.cost[i] < MARGIN and any(
                0 <= i + o < len(cm.state) and cm.state[i + o] == 0 for o in (1, -1, cm.w, -cm.w))
            if not still:
                return self._plan_explore(pose)
        if rest and not cm.path_clear([pose[:2]] + rest, unknown_ok=self.mode == "goto"):
            return self._plan_goto(pose) if self.mode == "goto" else self._plan_explore(pose)
        return ("ok",)

    def _remaining(self):
        nav = self.b.nav or {}
        k = nav.get("nav_index", 0) if nav.get("waypoint_count") == getattr(self, "sent_n", -1) else 0
        return self.path[1 + k:] if self.path else []

    # ---------------------------------------------------------- tick (25 Hz from the bridge)
    def tick(self, now):
        if self.mode == "idle":
            return
        b = self.b
        alive = b.link and now - b.tlm_time < 1.0
        estop = b.estop and b.estop_time > self.started_at
        if estop or not alive:
            self.stop("stopped: %s" % ("E-stop" if estop else "robot link lost"))
            return
        t = b.tlm
        if now - self.started_at > 3.0 and t and t["mode"] == lp.MODE_IDLE:
            self.stop("stopped: robot disarmed")
            return
        if self.job and not self.job.done():
            return
        ss = getattr(b.mapper, "stopscan", None)
        if ss is not None and ss.seq != self.scan_seen:
            self.scan_seen, self.last_scan_t = ss.seq, now
        if self.mode == "explore" and self.holding:
            if self._scan_hold(now):
                return
            self.holding = False
            self._start_job(self._plan_explore, go=True)
            return
        nav = b.nav or {}
        fresh = now - self.sent_at > 0.6 and nav.get("waypoint_count") == getattr(self, "sent_n", -1)
        state = self.nav_names[nav["state"]] if fresh and nav.get("state", 99) < len(self.nav_names) else ""
        if self.replan_at is not None:        # stuck (#62): let it back off, then plan with the boost
            if now < self.replan_at:
                return
            self.replan_at = None
            self._start_job(self._plan_goto if self.mode == "goto" else self._plan_explore, go=True)
            return
        if state == "blocked" and now - self.last_plan > 2.0:
            self.blocked_plans += 1
            if self.blocked_plans > 6:
                if self.mode == "explore" and self.goal:
                    self.blacklist.append(self.goal)
                    self.blocked_plans = 0
                    self._start_job(self._plan_explore, go=True)
                else:
                    self.stop("gave up: blocked")
                return
            self._start_job(self._plan_goto if self.mode == "goto" else self._plan_explore, go=True)
            return
        if state == "arrived" and nav.get("nav_index", 0) >= nav.get("waypoint_count", 0):
            if self.mode == "goto":
                dest, solve = self.dest, self.solve
                self.mode, self.path, self.solve = "idle", [], None
                self._success()
                if solve:
                    secs = self.clock() - self.solve_t0
                    z = self.b.mapper.zones
                    z.maze_best(solve, secs)
                    if hasattr(self.b.mapper, "zones_changed"):
                        self.b.mapper.zones_changed()
                    self._set_status("maze solved in %.1f s" % secs)
                    return
                self._set_status("arrived" if not dest else "arrived at " + dest)
                return
            if self._scan_hold(now):
                return
            if now - self.last_plan > 0.8:
                if self.goal:
                    self.blacklist.append(self.goal)     # we're there; don't come back to it
                    self._success()
                self._start_job(self._plan_explore, go=True)
            return
        if self.mode == "explore" and not self.path and now - self.last_plan > 1.0:
            self._start_job(self._plan_explore, go=True)
            return
        # no progress for a while (wedged on a corner the map does not show, wheels slipping): replan
        t = b.tlm
        if t:
            p = (t["x_mm"], t["y_mm"], t["heading_deg"])
            lp_ = getattr(self, "progress", None)
            if lp_ is None or math.hypot(p[0] - lp_[0], p[1] - lp_[1]) > 40 or abs(p[2] - lp_[2]) > 8:
                self.progress = p
                self.progress_t = now
            elif now - self.progress_t > 6.0 and now - self.last_plan > 6.0:
                self.progress_t = now
                self.blocked_plans += 1
                if self.mode == "explore" and self.goal and self.blocked_plans > 2:
                    self.blacklist.append(self.goal)
                    self.blocked_plans = 0
                self._start_job(self._plan_goto if self.mode == "goto" else self._plan_explore, go=True)
                return
        # ~100-250 ms of a Pi core per check (worker thread): every 2.5 s while driving
        if now - self.last_check > 2.5 and now - self.last_plan > 1.5 and state != "blocked":
            self.last_check = now
            self._start_job(self._check, go=True, check=True)
