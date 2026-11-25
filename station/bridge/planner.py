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
import re
import time

from . import link_protocol as lp
from . import zones as zm
from .loopprof import worker_yield as _yield     # (#110) long loops in the worker thread give way to the event loop
from .occgrid import UNKNOWN, UNIT
from .places import HOME

SQ2 = math.sqrt(2.0)
BLOCKED = 255             # within robot radius + 30 mm of an obstacle: never
MARGIN = 250              # within radius + margin: only to drive out of it near the start
OCC_P, FREE_P = 0.62, 0.40
OCC_BYTE = UNKNOWN + math.log(OCC_P / (1 - OCC_P)) / UNIT     # ~138
FREE_BYTE = UNKNOWN + math.log(FREE_P / (1 - FREE_P)) / UNIT  # ~120
# fine byte -> class bits: 2 = occupied, 1 = free, 4 = unknown (OR-able: occupied wins, then free; the unknown bit
# tells whether any fine cell of a coarse one was unknown, #108)
CLASS_LUT = bytes(2 if v > OCC_BYTE else (1 if v < FREE_BYTE else 4) for v in range(256))
# squeeze plans (#62): only sure walls count (~0.8). The sensor cone draws weak stray hits into a doorway
# (just over OCC_BYTE) from beams that clip a jamb; those count as free there (seen, not a frontier).
# Only for maps without range weighting (mapping "range_weight" off): with it, far hits are weak by design
# (a wall seen from 1-2 m sits at ~140-155 and would vanish from a sure-walls map, letting squeeze routes and
# frontiers through it), and close free readings already clear far stray hits out of doorways (#63).
SURE_OCC_BYTE = 155
SURE_LUT = bytes(2 if v > SURE_OCC_BYTE else (1 if v < FREE_BYTE or v > OCC_BYTE else 4) for v in range(256))
FINAL_LUT = bytes(2 if v & 2 else (1 if v & 1 else 0) for v in range(256))     # OR of classes -> 0 / 1 / 2
UNK_BIT = bytes(1 if v & 4 else 0 for v in range(256))
OCC_ONLY = bytes([0, 0, 1] + [0] * 253)       # state -> 1 where occupied
FREE_ONE = bytes([0, 1] + [0] * 254)          # state -> 1 where free
UNKNOWN_ONE = bytes([1] + [0] * 255)          # state -> 1 where unknown
THREE_UP = bytes(1 if v >= 3 else 0 for v in range(256))
# #108: a thin wall end is drawn short. Beams that pass just beside it sweep their free wedge (cone_deg)
# over its last cells, so the end fades into unknown cells (a 260 mm doorway's jamb: last occupied cell 45 mm short,
# the two cells after it 137 / 121 - unknown). Unknown cells within this of a wall count as wall for the hard bands (both maps; at 50 mm: a cell with
# any unknown fine cell). Where the end is erased to strongly free the map has no trace of it: see the doorway sweep.
END_GROW_MM = 50.0
CENTRE_LEAD_MM = 150.0    # squeeze routes: lead-in / lead-out on the gap's centre line, this far beyond its narrow part
CENTRE_REACH_MM = 100.0   # ... a waypoint moves at most this far sideways onto the centre line
GAP_SIDE_MM = 100.0       # ... a gap: walls within radius + margin + this on both sides of the route

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
DRIFT_RESEND_MM = 20.0    # LD19 scan matching (#78): resend the route once its next waypoints moved this much
CONTACT_REPLAN_S = 2.5    # a contact (#104): replan this long after (the back-off of nav.stuck_backoff_mm is done;
                          # GOTO waits 4 s for the new route before it tries the old one again)
# zone escape (#114): the watchdog stopped it at a zone edge, or a plan starts at one -> a short straight move away first
ZONE_SETTLE_S = 0.5       # after a watchdog stop (HOLD): plan the escape this much later, once it stands still
ZONE_STILL_MMS = 30.0     # ... (HOLD rolls it back to where it stopped: a fast robot overshoots first) ...
ZONE_STILL_S = 0.4        # ... for this long ...
ZONE_SETTLE_MAX_S = 3.0   # ... or after this long anyway
ZONE_MAX_STOPS = 10       # watchdog stops in one job before it gives up anyway (each one escapes first)
BACK_SPEED_MMS = 80.0     # escape straight back (the edge is ahead): STATION cmd_vel at this speed ...
BACK_MAX_S = 6.0          # ... for at most this long
BACK_MODE_WAIT_S = 1.0    # the robot did not switch to STATION: the escape point goes as a waypoint instead
# #111: stopped again and again at one spot (BLOCKED -> back-off -> the same route -> BLOCKED ...): blocks, stucks and
# contacts within LOOP_R_MM of each other in LOOP_WINDOW_S, LOOP_COUNT of them (a block counts only when close, LOOP_BLOCK_MM)
# (3 stops of any kind) -> a bump mark where it was blocked, more margin (the stuck boost),
# explore gives that frontier goal up, and it plans another way
LOOP_R_MM = 150.0
LOOP_WINDOW_S = 30.0
LOOP_WEIGHT = {"block": 1, "stuck": 1, "wall": 1, "contact": 1}   # (wall: a contact on a mapped wall, no mark)
LOOP_COUNT = 3
LOOP_AHEAD_MM = 39.0      # a block's obstacle: its distance (from the sensor head, front_) + this from the axle
LOOP_BLOCK_MM = 160.0     # a block counts only at this distance or closer (the stop distance, live rng.obstacle_mm;
                          # 2026-11-25 loop: 44-152 mm, a doorway squeeze: 260-300)
LOOP_REPLAN_S = 1.0
LOOP_SAME_S = 0.3         # reports this close in time (and 50 mm) are one stop (a stuck that is also a contact)
BLOCKED_RE = re.compile(r"nav: blocked, obstacle at (\d+) mm")
# stale routes: a plan starts from the pose when the job began, and on the Pi a plan takes seconds (the worker gives
# way to the loop, #110) - meanwhile the robot drives on along the old route. The new route goes from where it is now:
ON_ROUTE_MM = 60.0        # the robot within this of the route: it is on it (joins at the next point ahead)
STALE_MM = 150.0          # moved this far while planning and the join leg is not clear: plan again from here (once)
SAME_ROUTE_MM = 100.0     # the new route's points all within this of the ones it follows: not sent (no restart at 0)


def project_on_route(path, xy):
    """Where the robot at xy is on the route path (path[0] = where it was planned from): (k, (px, py), off) -
    nearest to leg path[k-1] -> path[k] at (px, py), off mm from it, so path[k] is the next point ahead. The first
    leg within ON_ROUTE_MM wins (a route that comes back past here later is not skipped to); a projection past the
    end of a leg (it reached that point) moves on to the next leg."""
    best, first = None, None
    for k in range(1, len(path)):
        (ax, ay), (bx, by) = path[k - 1], path[k]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((xy[0] - ax) * dx + (xy[1] - ay) * dy) / L2))
        p = (ax + u * dx, ay + u * dy)
        hit = (k + 1 if u >= 1.0 and k + 1 < len(path) else k, p, math.hypot(xy[0] - p[0], xy[1] - p[1]))
        if best is None or hit[2] < best[2]:
            best = hit
        if hit[2] <= ON_ROUTE_MM:
            first = hit
            break
    return first or best


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
    def __init__(self, grid, settings=None, res_mm=50.0, margin_m=1.0, sure_walls=False, specks=None, zones=None,
                 grow_mm=0.0):
        """sure_walls: only sure walls count (SURE_LUT); specks: fill unknown specks (default: = sure_walls).
        zones: a zones.ZoneView (#70): virtual walls / zone edges become walls before inflation, no-go insides and
        the outside of the fence are blocked after it, slow zones cost more.
        grow_mm (#108): (partly) unknown cells this close to a wall at the edge of seen-free space are wall too (hard
        bands only): thin wall ends the sensor cone drew short (see _grow)."""
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
        self.radius_mm = S["robot_radius_mm"]
        self._ccache = {}            # cell -> clearance (cells), see _clearance
        if zones:
            zones.mark_lines(self)
        self._inflate(S["robot_radius_mm"] + S["margin_mm"], S["soft_mm"], grow_mm)
        if zones:
            zones.mark_areas(self, BLOCKED, MARGIN)
        self.unknown_ok = bool(S["unknown_ok"])

    # ---------------------------------------------------------- build
    def _classify(self):
        g, W, f, w = self.g.cells, self.g.W, self.f, self.w
        nb = w * f
        state, part = bytearray(), bytearray()
        for cy in range(self.h):
            _yield()
            acc = 0
            for k in range(f):
                j = self.fy0 + cy * f + k
                s = j * W + self.fx0
                acc |= int.from_bytes(bytes(g[s:s + nb]).translate(self.lut), "big")
            for k in range(1, f):                  # OR each byte with the f-1 before it
                acc |= acc >> (8 * k)
            row = acc.to_bytes(nb, "big")[f - 1::f]
            state += row.translate(FINAL_LUT)
            part += row.translate(UNK_BIT)
        self.state = state                          # 0 unknown, 1 free, 2 occupied
        self.part_unknown = part                    # 1: a fine cell of it is unknown (see _grow)

    def _fill_specks(self):
        """Squeeze maps: an unknown cell with 3+ free 4-neighbours counts as free (a weakly seen speck in a
        doorway would cut the only way through; explore never travels through unknown)."""
        # (#108: counted for all cells at once with whole-map integer shifts - one byte per cell, a count <= 4 never
        # carries into the next byte; the cell-by-cell loop over the unknown was most of a squeeze map's build)
        w, n, st = self.w, len(self.state), self.state
        full = (1 << (8 * n)) - 1
        free = int.from_bytes(st.translate(FREE_ONE), "big")
        if not hasattr(CostMap, "_colmask") or CostMap._colmask[0] != (w, n):
            first = (b"\x00" + b"\x01" * (w - 1)) * (n // w)      # 0 in column 0
            last = (b"\x01" * (w - 1) + b"\x00") * (n // w)       # 0 in the last column
            CostMap._colmask = ((w, n), int.from_bytes(first, "big"), int.from_bytes(last, "big"))
        _k, not_first, not_last = CostMap._colmask
        # big-endian: cell i+1's byte shifted left by 8 lands on cell i
        cnt = (((free << 8) & full) & not_last) + ((free >> 8) & not_first) + \
            ((free << (8 * w)) & full) + (free >> (8 * w))
        both = (int.from_bytes(cnt.to_bytes(n, "big").translate(THREE_UP), "big") &
                int.from_bytes(st.translate(UNKNOWN_ONE), "big")).to_bytes(n, "big")
        i = both.find(1)
        while i >= 0:
            st[i] = 1                # (part_unknown keeps them for the wall-end growth, #108)
            i = both.find(1, i + 1)

    @staticmethod
    def _paint(buf, w, h, cells, r, val):
        """buf[...] = val on a disc of radius r (cells) round each cell."""
        ri = int(math.floor(r))
        spans = [(dy, int(math.floor(math.sqrt(max(0.0, r * r - dy * dy))))) for dy in range(-ri, ri + 1)]
        fill = bytes([val]) * (2 * ri + 3)
        yl = _yield
        for i in cells:
            yl()
            cx, cy = i % w, i // w
            for dy, dx in spans:
                y = cy + dy
                if 0 <= y < h:
                    a, b = max(0, cx - dx), min(w, cx + dx + 1)
                    buf[y * w + a:y * w + b] = fill[:b - a]

    def _grow(self, occ, grow_mm):
        """(#108) Cells within grow_mm of an occupied one that are (partly) unknown - at 50 mm cells: any of its fine
        cells - and lie in or next to seen-free space: where a wall end the sensor drew short may still go on (filled
        specks count: they were unknown). Cells deep in the unknown behind a wall add nothing (its own bands cover
        them), so only the ones at the edge of seen space count."""
        w, h, st = self.w, self.h, self.state
        r = grow_mm / self.res
        if r < 1.0 or not occ:
            return []
        near = bytearray(w * h)
        self._paint(near, w, h, occ, r, 1)
        n = w * h
        both = (int.from_bytes(near, "big") & int.from_bytes(self.part_unknown, "big")).to_bytes(n, "big")
        out = []
        i = both.find(1)
        while i >= 0:
            s = st[i]
            if s == 1:
                out.append(i)
            elif s == 0:
                ix = i % w
                if (ix + 1 < w and st[i + 1] == 1) or (ix > 0 and st[i - 1] == 1) or \
                        (i + w < n and st[i + w] == 1) or (i >= w and st[i - w] == 1):
                    out.append(i)
            i = both.find(1, i + 1)
        return out

    def _inflate(self, hard_mm, soft_mm, grow_mm=0.0):
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
        self.grown = self._grow(occ, grow_mm) if grow_mm > 0 else []
        self._solid = None
        self._paint(cost, w, h, occ, rs, 20)
        for r, val in ((rh, MARGIN), (self.inner_r, BLOCKED)):
            self._paint(cost, w, h, occ, r, val)
            self._paint(cost, w, h, self.grown, r, val)    # (their soft band: the wall's own one covers it)
        self.cost = cost

    def solid(self):
        """1 = a wall for clearances: occupied (zones' blocked areas too) or a grown wall end (#108)."""
        if self._solid is None:
            s = bytearray(self.state.translate(OCC_ONLY))
            for i in self.grown:
                s[i] = 1
            self._solid = s
        return self._solid

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
        yl = _yield
        while heap:
            _, d, i = heapq.heappop(heap)
            yl()
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
        """Distance (cells, capped at R) from cell i to the nearest wall cell (occupied or a grown wall end)."""
        v = cache.get(i)
        if v is None:
            w, so = self.w, self.solid()
            cx, cy = i % w, i // w
            best = R * R
            ri = int(R)
            x0, x1 = max(0, cx - ri), min(w, cx + ri + 1)
            for y in range(max(0, cy - ri), min(self.h, cy + ri + 1)):
                row = y * w
                j = so.find(1, row + x0, row + x1)
                while j >= 0:
                    d = (j - row - cx) ** 2 + (y - cy) ** 2
                    if d < best:
                        best = d
                    j = so.find(1, j + 1, row + x1)
            v = cache[i] = math.sqrt(best)
        return v

    def _walls_near(self, x, y, r_mm):
        """Centres (world mm) of the wall cells in the square of half side r_mm round (x, y)."""
        w, h, so, res = self.w, self.h, self.solid(), self.res
        cx, cy = int((x - self.x0) // res), int((y - self.y0) // res)
        k = int(r_mm / res) + 1
        x0, x1 = max(0, cx - k), min(w, cx + k + 1)
        pts = []
        for yy in range(max(0, cy - k), min(h, cy + k + 1)):
            row = yy * w
            j = so.find(1, row + x0, row + x1)
            while j >= 0:
                pts.append((self.x0 + (j - row + 0.5) * res, self.y0 + (yy + 0.5) * res))
                j = so.find(1, j + 1, row + x1)
        return pts

    @staticmethod
    def _dist2(x, y, pts, cap2):
        best = cap2
        for px, py in pts:
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best:
                best = d
        return best

    def wall_clearance(self, pts, step_mm=10.0):
        """Smallest distance (mm) from the polyline pts to a wall, to the wall cells' near edges (a wall cell's
        hit is anywhere inside it: centre - half a cell); capped at radius + margin + 80. #108: what a squeeze
        route really keeps - the doorway's jambs as the map has them, grown wall ends included."""
        cap = self.hard_mm + 80.0
        R = cap / self.res
        best = cap
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(bx - ax, by - ay) / step_mm))
            for k in range(n + 1):
                _yield()
                x, y = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                i = self.cell(x, y)
                if i is None:
                    continue
                c = self._clearance(i, self._ccache, R) * self.res   # cell-centre distance: within ~0.7 cell
                if c - self.res > best:
                    continue
                d = math.sqrt(self._dist2(x, y, self._walls_near(x, y, min(cap, c + self.res)), cap * cap))
                best = min(best, d - self.res / 2.0)
        return max(0.0, best)

    def _centre_xy(self, p, t, unknown_ok):
        """Squeeze maps (#108): move p sideways (across direction t, at most CENTRE_REACH_MM) to where it is
        furthest from the walls on both sides - the gap's centre line, off the 25 mm cell grid. None if a side is
        open (the farthest point is at the reach limit) or there is no wall near."""
        nx, ny = -t[1], t[0]
        reach = CENTRE_REACH_MM
        look = reach + self.hard_mm + 80.0
        pts = self._walls_near(p[0], p[1], look)
        if not pts:
            return None
        cap2 = look * look
        step = 5.0
        kmax = int(reach / step)
        best, bk = -1.0, None
        for k in sorted(range(-kmax, kmax + 1), key=abs):       # ties: the smallest move
            d = self._dist2(p[0] + nx * k * step, p[1] + ny * k * step, pts, cap2)
            if d > best + 1e-6:
                best, bk = d, k
        if bk is None or abs(bk) == kmax or best >= cap2:
            return None
        q = (p[0] + nx * bk * step, p[1] + ny * bk * step)
        i = self.cell(*q)
        if i is None or not self.passable(i, unknown_ok):
            return None
        return q

    def _gap_axis(self, m, t):
        """Direction through the gap at its centre m: across the line between the nearest wall cells on either side
        of the route (a doorway: through it, square to the jambs), signed along t."""
        nx, ny = -t[1], t[0]
        pts = self._walls_near(m[0], m[1], CENTRE_REACH_MM + self.hard_mm + 80.0)
        a = b = None
        for px, py in pts:
            s = (px - m[0]) * nx + (py - m[1]) * ny
            d = (px - m[0]) ** 2 + (py - m[1]) ** 2
            if s < 0 and (a is None or d < a[0]):
                a = (d, px, py)
            elif s > 0 and (b is None or d < b[0]):
                b = (d, px, py)
        if a is None or b is None:
            return t
        gx, gy = b[1] - a[1], b[2] - a[2]
        L = math.hypot(gx, gy)
        ux, uy = gy / L, -gx / L
        if ux * t[0] + uy * t[1] < 0:
            ux, uy = -ux, -uy
        return ux, uy

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
        cache, R = self._ccache, (self.hard_mm + 80.0) / self.res
        must = set()
        runs = []
        if self.centre:                  # per narrow stretch: its centre cell + a lead-in / lead-out stay waypoints
            lead = int(round(CENTRE_LEAD_MM / self.res))
            tight = (self.hard_mm + 40.0) / self.res
            last = len(cells) - 1

            def triple(r, pinch=False):
                mid = r[len(r) // 2]
                if pinch:     # a gap: its narrowest cell (a doorway: in the doorway, not where the stretch is centred)
                    inner = [k for k in r if 0 < k < last] or r       # (not the robot / the goal themselves)
                    mid = min(inner, key=lambda k: (round(self._clearance(cells[k], cache, R), 1), abs(k - mid)))
                return max(0, r[0] - lead), mid, min(last, r[-1] + lead)
            run, gap = [], []
            for k, c in enumerate(cells + [None]):
                t = c is not None and self._clearance(c, cache, R) < tight
                if t:
                    run.append(k)
                elif run:
                    must.update(triple(run))
                    run = []
                # #108: the gap itself = where walls are close on both sides (a narrow stretch also runs on along a
                # wall beyond a doorway: its middle is not the doorway's)
                if t and self._two_sided(cells, k, self.hard_mm + GAP_SIDE_MM):
                    gap.append(k)
                elif gap:
                    ks = triple(gap, pinch=True)
                    must.update(ks)
                    runs.append(ks)
                    gap = []
            for kin, kmid, kout in runs:     # a gap: only its lead-in, middle and lead-out must stay (#108)
                must -= {k for k in must if kin < k < kout and k != kmid}
        out = [0]                        # kept indices into cells
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
            out.append(j)
            i = j
        pts = [self.center(cells[k]) for k in out]
        if runs:
            self._centre_runs(cells, out, pts, runs, unknown_ok)
        return pts

    def _centre_runs(self, cells, out, pts, runs, unknown_ok):
        """Squeeze maps (#108): put each narrow stretch's waypoints on the gap's true centre line. The A* cells only
        know the gap to a cell (25 mm: a 240 mm doorway's centre lies on a cell edge) and, with the soft band flat
        inside the gap, take any cell of the hard corridor. The middle waypoint moves across to the point furthest
        from both sides, the lead-in / lead-out onto the line through it square to the jambs (the robot drives
        straight through). A moved point is kept only when its legs stay clear."""
        pos = {k: n for n, k in enumerate(out)}
        last = len(cells) - 1
        for kin, kmid, kout in runs:
            if kmid not in pos or kmid in (0, last):
                continue
            a = self.center(cells[max(0, kmid - 2)])
            b = self.center(cells[min(last, kmid + 2)])
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L < 1e-6:
                continue
            t = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
            n = pos[kmid]
            if pts[n] is None:
                continue
            m = self._centre_xy(pts[n], t, unknown_ok)
            if m is None or not self._legs_clear(pts, n, m, unknown_ok):
                continue
            pts[n] = m
            u = self._gap_axis(m, t)
            for j in range(pos.get(kin, n) + 1, pos.get(kout, n)):      # other waypoints in the gap: onto the line
                if j == n or pts[j] is None:
                    continue
                p = pts[j]
                s = (p[0] - m[0]) * u[0] + (p[1] - m[1]) * u[1]
                p0, p1 = self._beside(pts, j, -1), self._beside(pts, j, 1)
                s0 = (p0[0] - m[0]) * u[0] + (p0[1] - m[1]) * u[1]
                s1 = (p1[0] - m[0]) * u[0] + (p1[1] - m[1]) * u[1]
                if not s0 + 20.0 < s < s1 - 20.0:
                    continue                  # would step back (or bunch up) along the line: leave it
                q = (m[0] + u[0] * s, m[1] + u[1] * s)
                i = self.cell(*q)
                if i is not None and self.passable(i, unknown_ok) and self._legs_clear(pts, j, q, unknown_ok):
                    pts[j] = q
            for k, sgn in ((kin, -1.0), (kout, 1.0)):
                if k not in pos or k in (0, last) or k == kmid or pts[pos[k]] is None:
                    continue                  # the robot / the goal stay where they are
                j = pos[k]
                p = pts[j]
                d = max(CENTRE_LEAD_MM, sgn * ((p[0] - m[0]) * u[0] + (p[1] - m[1]) * u[1]))
                o = self._beside(pts, j, -1 if sgn < 0 else 1)   # the waypoint before the lead-in / after the lead-out
                so = sgn * ((o[0] - m[0]) * u[0] + (o[1] - m[1]) * u[1])
                for f in (1.0, 0.66, 0.4):
                    if d * f > so - 50.0:
                        continue                  # not 50 mm beyond the waypoint before / after it: no step back
                    q = (m[0] + sgn * u[0] * d * f, m[1] + sgn * u[1] * d * f)
                    i = self.cell(*q)
                    if i is not None and self.passable(i, unknown_ok) and self._legs_clear(pts, j, q, unknown_ok):
                        pts[j] = q
                        break
                else:
                    # no room for it (the robot is about at the gap already): straight on to the middle, if clear
                    a, b = self._beside(pts, j, -1), self._beside(pts, j, 1)
                    ca, cb = self.cell(*a), self.cell(*b)
                    if ca is not None and cb is not None and self.clear_line(
                            ca, cb, unknown_ok, allow_start_zone=(a is pts[0])):
                        pts[j] = None
        pts[:] = [p for p in pts if p is not None]

    @staticmethod
    def _beside(pts, j, step):
        """The nearest waypoint before (step -1) / after (+1) j that is still there."""
        k = j + step
        while pts[k] is None:
            k += step
        return pts[k]

    def _two_sided(self, cells, k, r_mm):
        """Route cell k has walls within r_mm on both sides (left and right of the route there)."""
        last = len(cells) - 1
        return self._two_sided_xy(self.center(cells[k]), self.center(cells[max(0, k - 2)]),
                                  self.center(cells[min(last, k + 2)]), r_mm)

    def _two_sided_xy(self, p, a, b, r_mm):
        """Walls within r_mm of p on both sides of the direction a -> b."""
        nx, ny = a[1] - b[1], b[0] - a[0]
        L = math.hypot(nx, ny)
        if L < 1e-6:
            return False
        nx, ny = nx / L, ny / L
        r2, lo, hi = r_mm * r_mm, False, False
        half = self.res / 2.0
        for ox, oy in self._walls_near(p[0], p[1], r_mm):
            dx, dy = ox - p[0], oy - p[1]
            if dx * dx + dy * dy <= r2:
                s = dx * nx + dy * ny
                if s < -half:
                    lo = True
                elif s > half:
                    hi = True
                if lo and hi:
                    return True
        return False

    def recentre(self, pts, unknown_ok, min_mm=10.0):
        """Squeeze maps (#108): a route planned earlier, its waypoints in a gap (walls close on both sides) moved
        onto the gap's centre line as this (newer) map has it - the robot sees a doorway's jambs better as it
        comes closer. The first point (the robot) and the last (the goal) stay. None if nothing moved min_mm."""
        out = list(pts)
        moved = False
        for j in range(1, len(out) - 1):
            p, a, b = out[j], out[j - 1], out[j + 1]
            if not self._two_sided_xy(p, a, b, self.hard_mm + GAP_SIDE_MM):
                continue
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L < 1e-6:
                continue
            q = self._centre_xy(p, ((b[0] - a[0]) / L, (b[1] - a[1]) / L), unknown_ok)
            if q is None or math.hypot(q[0] - p[0], q[1] - p[1]) < min_mm or not self._legs_clear(out, j, q, unknown_ok):
                continue
            out[j] = q
            moved = True
        return out if moved else None

    def _legs_clear(self, pts, j, q, unknown_ok):
        """Would the route stay clear with waypoint j moved to q?"""
        c = self.cell(*q)
        if c is None:
            return False
        if j > 0:
            p = self._beside(pts, j, -1)
            a = self.cell(*p)
            if a is None or not self.clear_line(a, c, unknown_ok, allow_start_zone=(p is pts[0])):
                return False
        if j + 1 < len(pts):
            b = self.cell(*self._beside(pts, j, 1))
            if b is None or not self.clear_line(c, b, unknown_ok):
                return False
        return True

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
        yl = _yield
        while heap:
            d, i = heapq.heappop(heap)
            yl()
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
            _yield()
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
        self._sq_clear_now = None     # (set by the worker: a squeeze route's clearance in mm, #108)
        self._sq_cm = None
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
        cl = getattr(getattr(bridge, "mapper", None), "corr_listeners", None)
        if cl is not None:
            cl.append(self.on_drift)          # LD19 scan matching moved the map frame (#78)
        self.sent_corr = None         # mapper.corr when the route was last sent (in the odometry frame)
        self.solve = None             # goto: the maze being solved (its exit gap counts as a wall, #70)
        self.solve_t0 = 0.0
        self.zone_hits = 0            # watchdog stops in this job (#70)
        self.zone_settle = False      # replan_at is a zone stop's (#114): also wait until it stands still ...
        self.zone_still = None        # ... (since when)
        self.no_station = False       # this job: the robot did not take STATION for an escape - escape by waypoint
        self.backing = None           # zone escape straight back (#114): {"x0", "y0", "h0" (odometry), "dist", "t0", ...}
        self._esc_now = None          # (set by the worker with each plan: its zone escape, #114)
        self.spots = []               # #111: recent stops: (time, map x, y, weight, what stopped it (odometry x, y,
                                      # heading) or None: a contact, marked by the bridge already)
        self.loop_marks = 0           # ... loops marked in this job
        self._cms = {}                # costmap kind -> the last one built (set by the worker)
        self._stale_retry = False     # the plan running is the one more after a stale route that could not be joined

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
        if kind == "squeeze":
            # at the map's own 25 mm cells (at 50 mm a 280 mm doorway rounds shut); sure walls only when the
            # map is not range-weighted (see SURE_LUT)
            s["margin_mm"] = min(s["margin_mm"], s["squeeze_mm"])
            rw = self.b.mapper.settings.get("range_weight", False)
            cm = CostMap(g, s, res_mm=g.res, sure_walls=not rw, specks=True, zones=zones, grow_mm=END_GROW_MM)
            cm.centre = True
        else:
            cm = CostMap(g, s, zones=zones, grow_mm=END_GROW_MM)
        self._cms[kind] = cm          # the last one of each kind: a route's join leg is checked on it (no rebuild)
        return cm

    def zone_view(self):
        """The map's zones (#70) as the planner sees them (Solve: that maze's exit closed), or None."""
        z = getattr(self.b.mapper, "zones", None)
        if z is None or not z.items:
            return None
        return z.view(close_exit=self.solve) if self.solve else z.view()

    def zone_fp(self):
        """The watchdog's footprint radius (mm): the planner's robot radius less the pose noise."""
        return max(20.0, self.settings["robot_radius_mm"] - zm.FP_TOL_MM)

    def zone_escape(self, pose, cm=None):
        """#114: the escape move for a plan starting at pose (map frame), or None (not at a zone edge, or no zones);
        {"boxed": True, ...} when no straight move gets the footprint clear. cm: end points inside real obstacles
        are refused."""
        zv = self.zone_view()
        if not zv:
            return None

        def ok(x, y):
            i = cm.cell(x, y)
            return i is None or cm.cost[i] < BLOCKED
        return zv.escape(pose[0], pose[1], pose[2], self.zone_fp(), ok=ok if cm is not None else None)

    def zone_stop(self, why):
        """The bridge's watchdog stopped the robot (HOLD): it drove into a no-go zone / at a virtual wall / the fence
        (or its pose moved into one). Escape first (#114): in a moment the next plan starts with a short straight move
        away from the edge (backwards when the edge is ahead), then the route. Gives up only when boxed in (no straight
        way out) or after ZONE_MAX_STOPS in one job."""
        if self.mode == "idle":
            return
        self.zone_hits += 1
        if self.zone_hits > ZONE_MAX_STOPS:
            self.stop("stopped: %s (%d times - check the zones)" % (why, ZONE_MAX_STOPS))
            return
        pose = self.robot_map_pose()
        esc = self.zone_escape(pose) if pose else None
        if esc and esc.get("boxed"):
            self.stop("stopped: %s - boxed in by zones - edit them" % why)
            return
        self._back_end(send_stop=True)
        self.squeeze = False
        self.last_check = self.clock()
        if esc and esc["drive"] != "turn" and self.b.tlm and not self.no_station:
            # straight back / ahead along the heading: at once (it brakes and reverses in one go - waiting for HOLD
            # to settle after a fast stop only rocks it to and fro); the route is planned once it is clear
            self.replan_at = None
            self._back_start(esc, None, None, why=why)
            return
        self.replan_at, self.zone_settle = self.clock() + ZONE_SETTLE_S, True
        self._set_status("stopped: %s - %s" % (why, "backing away from it" if esc and esc["drive"] == "back"
                                               else "moving away from it" if esc else "replanning"))

    def _vel(self, v):
        send = getattr(self.b, "send", None)
        if send:
            send(lp.encode_cmd_vel(v, 0.0))

    def _back_start(self, esc, path, status, why=None):
        """#114: move straight away from the zone edge along the heading - backwards when the edge is ahead (STATION,
        cmd_vel straight back / ahead, checked clear by ZoneView.escape; no turning beside the edge), then drive the
        rest of path (path[1] = the escape point), or (path None: after a watchdog stop) plan the route from there."""
        t = self.b.tlm
        sign = -1.0 if esc["drive"] == "back" else 1.0
        self.backing = {"x0": t["x_mm"], "y0": t["y_mm"], "h0": t["heading_deg"], "dist": esc["dist"], "sign": sign,
                        "t0": self.clock(), "path": path, "status": status}
        self.path = path or []
        if path is not None:
            self.sent_goal = self.goal
        self.b.handle_command({"t": "set_mode", "mode": lp.MODE_STATION, "arm": self.arm}, "planner")
        self.arm = False
        self._vel(sign * BACK_SPEED_MMS)
        move = "%s %d mm straight away from %s" % ("backing" if sign < 0 else "driving", esc["dist"],
                                                   "it" if why else "the zone edge")
        self._set_status("stopped: %s - %s, then replanning" % (why, move) if why else "%s, then %s" % (move, status))

    def _back_tick(self, now):
        bk = self.backing
        t = self.b.tlm
        if t["mode"] != lp.MODE_STATION and now - bk["t0"] > BACK_MODE_WAIT_S:
            # the robot did not take STATION: the escape point goes as the first waypoint (it turns, drives out)
            self.backing, self.no_station = None, True
            self.b.say("planner: robot not in STATION - driving to the zone escape point as a waypoint")
            if bk["path"] is None:
                self._start_job(self._plan_goto if self.mode == "goto" else self._plan_explore, go=True)
                return
            self.path = bk["path"]
            self._send(bk["path"][1:], True)
            self._set_status(bk["status"], log=False)
            return
        h = math.radians(bk["h0"])
        done = bk["sign"] * ((t["x_mm"] - bk["x0"]) * math.cos(h) + (t["y_mm"] - bk["y0"]) * math.sin(h))
        if done >= bk["dist"] - 5.0 or now - bk["t0"] > BACK_MAX_S:
            self._back_end(send_stop=True)
            self.b.say("planner: %s %d mm away from the zone edge" % ("backed" if bk["sign"] < 0 else "drove",
                                                                     max(0.0, done)))
            if bk["path"] is None:                     # after a watchdog stop: the route from here
                self._start_job(self._plan_goto if self.mode == "goto" else self._plan_explore, go=True)
                return
            pose = self.robot_map_pose()
            rest = bk["path"][2:]
            self.path = [pose[:2]] + rest
            self._send(rest, True)                     # GOTO / EXPLORE again with the rest of the route
            self._set_status(bk["status"], log=False)
            return
        self._vel(bk["sign"] * BACK_SPEED_MMS)
    def _back_end(self, send_stop=False):
        if self.backing is not None:
            self.backing = None
            if send_stop:
                self._vel(0.0)

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

    def reload_settings(self):
        """The mapper moved to another robot profile (#97): that folder's planner.json."""
        self.settings = self._load_settings()
        self.boost = 0.0

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
        """A robot log line. "nav: stuck ...": replan with a larger margin once it has backed off.
        "nav: blocked ..." / "nav: stuck ...": a stop at this spot (#111, see _spot)."""
        if self.mode == "idle":
            return
        m = BLOCKED_RE.match(text)
        if m:
            # (only a block at something this close counts: inside / at the lidar's dead zone, where the map can't keep
            # the route off it. Farther it is mapped, and a robot squeezing through a narrow doorway stops in front of
            # it a few times while the route is recentred: marking that would shut the doorway)
            if float(m.group(1)) <= LOOP_BLOCK_MM:
                self._spot("block", float(m.group(1)) + LOOP_AHEAD_MM)
            return
        if not text.startswith("nav: stuck"):
            return
        self.boost = min(STUCK_BOOST_MAX, self.boost + STUCK_BOOST_MM)
        self.squeeze = False
        self.replan_at = self.clock() + 5.0       # its back-off ends within 5 s
        self._set_status("robot stuck: backing off, replanning with +%d mm margin" % self.boost)
        self._spot("stuck", LOOP_AHEAD_MM + 11.0)

    def on_contact(self, mark):
        """The robot touched something the lidar can't see (#104): the bridge put a bump mark there (a wall for the
        costmap). The robot backs off and waits (GOTO: 4 s, EXPLORE: for a new list): replan round the mark
        once it has backed off (no extra margin - the mark is the obstacle). mark None (#111): it touched a wall the
        map has (no mark): replan, and it counts as a block for the same-spot rule."""
        if self.mode == "idle":
            return
        self.squeeze = False
        self.replan_at = self.clock() + CONTACT_REPLAN_S
        self.last_check = self.clock()
        if mark is None:
            self._set_status("robot touched a wall the map has, replanning")
            self._spot("wall", None)
            return
        self._set_status("robot touched something the lidar can't see: bump mark %s, replanning" % mark["id"])
        self._spot("contact", None)

    def _spot(self, kind, ahead_mm):
        """#111: the robot stopped here (a block, a stuck, a contact). Again and again at one spot (LOOP_*): the route
        keeps leading into something the map does not keep it off (an obstacle inside the lidar's dead zone, under its
        scan plane, a blocked back-off): mark it (a bump mark ahead_mm in front of the robot; a contact's own mark is
        there already), plan with the stuck boost, explore gives that goal up, and plan another way."""
        t = self.b.tlm
        if not t:
            return
        now = self.clock()
        x, y, h = t["x_mm"], t["y_mm"], t["heading_deg"]       # odometry frame
        mx, my, _ = self.b.mapper.map_pose(x, y, h)
        a = math.radians(h)
        obst = (x + ahead_mm * math.cos(a), y + ahead_mm * math.sin(a), h) if ahead_mm is not None else None
        self.spots = [s for s in self.spots if now - s[0] <= LOOP_WINDOW_S]
        w = LOOP_WEIGHT[kind]
        same = [s for s in self.spots if now - s[0] < LOOP_SAME_S and math.hypot(s[1] - mx, s[2] - my) < 50.0]
        if same:                                      # one stop, two reports (a stuck that is also a contact)
            self.spots.remove(same[-1])
            w = max(w, same[-1][3])
            obst = obst or same[-1][4]
        self.spots.append((now, mx, my, w, obst))
        near = [s for s in self.spots if math.hypot(s[1] - mx, s[2] - my) <= LOOP_R_MM]
        n = len(near)
        if sum(s[3] for s in near) < LOOP_COUNT:
            return
        self.spots = [s for s in self.spots if s not in near]  # (each stop counts once)
        mark = None
        for s in near:                                # what stopped it there (a contact is marked already)
            if s[4] is not None and hasattr(self.b.mapper, "add_contact"):
                mark, _merged = self.b.mapper.add_contact({"x_mm": s[4][0], "y_mm": s[4][1], "heading_deg": s[4][2],
                                                            "name": "stuck"})
        if self.mode == "explore" and self.goal is not None:
            self.blacklist.append(self.goal)                     # that frontier: not through here again
        self.loop_marks += 1
        self.boost = min(STUCK_BOOST_MAX, self.boost + STUCK_BOOST_MM)
        self.squeeze = False
        self.replan_at = now + LOOP_REPLAN_S
        self.last_check = now
        self._set_status("stopped %d times at the same spot (%.2f, %.2f m)%s: planning another way (+%d mm margin)" % (
            n, mx / 1000.0, my / 1000.0, " - bump mark %s" % mark["id"] if mark else "", self.boost))

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
            self.replan_at, self.squeeze, self.no_station = None, False, False
            self._back_end(send_stop=True)
            self.squeeze_left, self.sq_free = SQUEEZE_TRIES, None
            self.started_at = self.last_scan_t = self.clock()
            self.holding, self.sent_goal, self.scan_wait = False, None, None
            self.blocked_plans = 0
            self.solve, self.zone_hits = None, 0
            self.spots, self.loop_marks = [], 0
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
        self.replan_at, self.squeeze, self.no_station = None, False, False
        self._back_end(send_stop=True)
        self.dest = dest
        self.solve, self.solve_t0, self.zone_hits = solve, self.clock(), 0
        self.spots, self.loop_marks = [], 0
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
        self._back_end(send_stop=True)
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
        gen, t0 = self.gen, self.clock()
        if force:
            self._stale_retry = False
        if not check:
            self.last_plan = t0
        if self.sync:
            fut = concurrent.futures.Future()
            try:
                fut.set_result(fn(pose))
            except Exception as e:
                fut.set_exception(e)
            self._job_done(fut, go, gen, fn, t0)
            return True
        loop = asyncio.get_running_loop()
        self.job = loop.run_in_executor(None, self._run_job, fn, pose)
        self.job.add_done_callback(lambda fut: self._job_done(fut, go, gen, fn, t0))
        return True

    def _run_job(self, fn, pose):
        """Worker thread: the job, counted in the bridge's loop profile as thread time (#102: it competes for the GIL)."""
        prof = getattr(self, "prof", None)
        if prof is None:
            return fn(pose)
        with prof.thread("planner" + getattr(fn, "__name__", ".job").replace("_plan_", ".").replace("_", ".", 1)):
            return fn(pose)

    def _job_done(self, fut, go, gen, fn=None, t0=None):
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
            clear = self._sq_clear_now if squeeze else None
            if squeeze and not self.squeeze:
                # no speed field in MSG_GOTO: the robot's side check (nav.side_min_mm) slows it in the gap.
                # The clearance is the route's own (#108), not the squeeze_mm setting it was planned with
                self.b.say("planner: narrow gap - squeezing through with %s clearance (the robot slows "
                           "down beside the walls)" % ("~%d mm" % clear if clear is not None else "little"))
            self.squeeze = squeeze
            if self.mode == "explore" and squeeze:
                now_free = self._sq_free_now
                grew = self.sq_free is not None and now_free is not None and now_free - self.sq_free > SQUEEZE_GROW
                self.squeeze_left = SQUEEZE_TRIES if grew else self.squeeze_left - 1
                self.sq_free = now_free
            elif self.mode == "explore":
                self.squeeze_left, self.sq_free = SQUEEZE_TRIES, None
            esc = self._esc_now
            if esc and esc["drive"] != "turn" and len(path) > 2 and not self.no_station:
                # #114: straight back (the zone edge is ahead) / ahead away from it (STATION cmd_vel), then the route
                self._back_start(esc, path, status)
                return
            if esc:
                status += " (first %d mm straight away from the zone edge)" % esc["dist"]
            if not esc and self.mode == "explore" and self._scan_on() and self.sent_goal is not None and \
                    self.goal != self.sent_goal and self.clock() - self.last_scan_t > SCAN_EVERY_S:
                # a new frontier goal and no scan for a while: stop here and scan first, then plan again (#71)
                t = self.b.tlm
                self.b.handle_command({"t": "goto", "points": [[t["x_mm"], t["y_mm"]]], "replace": True}, "planner")
                self.sent_at, self.sent_n = self.clock(), 1
                self.path, self.holding, self.scan_wait, self.sent_goal = [], True, None, None
                self._set_status("exploring: stopped to scan and check the pose", log=False)
                return
            retry, self._stale_retry = self._stale_retry, False
            if not esc:
                path, note = self._fresh(path, self._cms.get(cmk), t0, replan=fn if not retry else None, go=go)
                if path is None:
                    return            # planning again from here
                if note == "same":
                    self.sent_goal = self.goal
                    self._set_status(status, log=False)
                    return
            self.path = path
            self.sent_goal = self.goal
            self._send(path[1:] or path[-1:], go)
            if squeeze:
                status += " (squeezing through a narrow gap%s)" % (", ~%d mm clearance" % clear if clear is not None
                                                                   else "")
            elif cmk == "boost":
                status += " (+%d mm margin after a stuck)" % self.boost
            self._set_status(status, log=False)
        elif kind == "ok":
            pass
        elif kind == "recentre":      # the same route, its gap waypoints moved onto the centre line (#108)
            self.path = self._fresh(res[1], self._cms.get("squeeze"), t0, dedupe=False)[0]
            self._send(self.path[1:], False)
            self._push()
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

    def _fresh(self, path, cm, t0, replan=None, go=False, dedupe=True):
        """A route planned from the pose when its job began (path[0], map frame) against where the robot is now (it
        drove on meanwhile: plans take seconds on the Pi): the points it passed are dropped and it joins at the next
        one ahead - straight there when it is on the route or that leg is clear on cm, else back onto the route first.
        Moved more than STALE_MM and no clear join: plans again from here (replan: the job's function; None = not
        again). Returns (path, note): path None = planning again; note "same" = the route it follows already (dedupe:
        then nothing is sent - a resend restarts its list at 0). Plan time and the move go to the log."""
        pose = self.robot_map_pose()
        if not pose or len(path) < 2:
            return path, ""
        cur = (pose[0], pose[1])
        moved = math.hypot(cur[0] - path[0][0], cur[1] - path[0][1])
        k, proj, off = project_on_route(path, cur)
        how = ""
        if off <= ON_ROUTE_MM:
            new = [cur] + path[k:]
        else:
            a = cm.cell(*cur) if cm is not None else None
            b = cm.cell(*path[k]) if cm is not None else None
            if a is not None and b is not None and cm.clear_line(a, b, self.mode == "goto", allow_start_zone=True):
                new = [cur] + path[k:]
                how = ", joins it from %d mm off" % off
            elif moved > STALE_MM and replan is not None:
                if replan == self._check:
                    replan = self._plan_goto if self.mode == "goto" else self._plan_explore
                self._plan_note(t0, moved, "%d mm off the route, the way onto it is not clear - planning again"
                                % off)
                self._stale_retry = True
                self._start_job(replan, go)
                return None, ""
            else:
                new = [cur, proj] + path[k:]
                how = ", back onto it (%d mm off) first" % off
        if dedupe and self._following(new[1:]):
            self._plan_note(t0, moved, "the route it is on already - not sent again")
            return path, "same"
        what = ("dropped %d passed point%s" % (k - 1, "" if k == 2 else "s") if k > 1 else "route from here") + how
        if k > 1 or how or moved >= 50.0 or (t0 is not None and self.clock() - t0 >= 0.5):
            self._plan_note(t0, moved, what)
        return new, ""

    def _following(self, pts):
        """Is the robot driving the route pts (map frame) already - the rest of the one sent, each point within
        SAME_ROUTE_MM?"""
        if self.holding or self.backing is not None or not self.path:
            return False
        t, nav = self.b.tlm, self.b.nav or {}
        if not t or t["mode"] != (lp.MODE_GOTO if self.mode == "goto" else lp.MODE_EXPLORE) or \
                nav.get("waypoint_count") != getattr(self, "sent_n", -1):
            return False
        st = nav.get("state", 99)
        if st >= len(self.nav_names) or self.nav_names[st] not in ("turning", "driving"):
            return False
        rest = self._remaining()
        return len(rest) == len(pts) and all(math.hypot(a[0] - b[0], a[1] - b[1]) <= SAME_ROUTE_MM
                                             for a, b in zip(rest, pts))

    def _plan_note(self, t0, moved, what):
        dur = self.clock() - t0 if t0 is not None else 0.0
        self.b.say("planner: plan took %.1f s, robot moved %d mm meanwhile - %s" % (dur, moved, what))

    def on_correction(self, entry):
        """A stop-and-scan moved the map frame (#71): the robot's waypoints are in its odometry frame, so the
        rest of the route goes again, converted with the new correction (the robot is standing still)."""
        rest = self._remaining() if self.mode != "idle" and self.backing is None else []
        if rest:
            pose = self.robot_map_pose()
            if pose:
                self.path = [pose[:2]] + rest
                self._send(rest, False)
        self._push()

    def on_drift(self):
        """The LD19 scan matcher moved the map frame a little (#78, many times a second): the route the robot follows
        is in its odometry frame, so once the next waypoints are off by DRIFT_RESEND_MM the rest goes again (<= 1 / s)."""
        if self.mode == "idle" or not self.path or self.sent_corr is None or self.clock() - self.sent_at < 1.0:
            return
        if self.replan_at is not None or self.backing is not None:   # backing off / away: the route follows after
            return
        rest = self._remaining()
        if not rest:
            return
        dx, dy, dt = self.sent_corr
        c, s = math.cos(math.radians(-dt)), math.sin(math.radians(-dt))
        off = 0.0
        for (x, y), (nx, ny) in zip(rest[:2], self.to_odom(rest[:2])):
            off = max(off, math.hypot(c * (x - dx) - s * (y - dy) - nx, s * (x - dx) + c * (y - dy) - ny))
        if off < DRIFT_RESEND_MM:
            return
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
        self.sent_corr = list(self.b.mapper.corr)
        odom = self.to_odom(pts)
        self.b.handle_command({"t": "goto", "points": [list(p) for p in odom], "replace": True}, "planner")
        self.sent_at = self.clock()
        self.sent_n = len(odom)
        if go:
            mode = lp.MODE_GOTO if self.mode == "goto" else lp.MODE_EXPLORE
            if not self.b.tlm or self.b.tlm["mode"] != mode:
                self.b.handle_command({"t": "set_mode", "mode": mode, "arm": self.arm}, "planner")
            self.arm = False

    def _start_escape(self, pose, cm):
        """Escape first (#114): (route head, start of the real route, None) - [robot, escape point] and the escape
        point when the plan starts at a zone edge, else ([robot], robot); (None, None, why) when boxed in."""
        esc = self.zone_escape(pose, cm)
        self._esc_now = esc if esc and not esc.get("boxed") else None
        if esc and esc.get("boxed"):
            return None, None, "boxed in by zones (%s) - edit them" % esc["why"]
        if esc:
            e = (esc["x"], esc["y"])
            return [pose[:2], e], e, None
        return [pose[:2]], pose[:2], None

    def _plan_goto(self, pose):
        fail = None
        for kind, cm in self._costmaps():
            route, start, boxed = self._start_escape(pose, cm)
            if boxed:
                return ("fail", boxed)
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
                self._sq_clear_now = self._route_clear(cm, route[1:] if self._esc_now else route) \
                    if kind == "squeeze" else None
                return ("path", route, "route: %d waypoint%s" % (n, "" if n == 1 else "s"), kind)
        return ("fail", "no route to %.1f, %.1f m" % (fail[0] / 1000, fail[1] / 1000))

    def _plan_explore(self, pose):
        fr, kind, normal, held = None, "normal", None, None
        head, start, esc_cm = [pose[:2]], pose[:2], None
        for kind, cm in self._costmaps():
            if kind == "squeeze" and self.squeeze_left <= 0:
                break                 # tried enough narrow spots: the rest is done
            if esc_cm is None:        # escape first (#114), worked out once on the first map
                esc_cm = cm
                head, start, boxed = self._start_escape(pose, cm)
                if boxed:
                    return ("fail", "explore: " + boxed)
            fr = cm.frontiers(start, self.settings["frontier_min_mm"], self.blacklist,
                              open_cm=normal if kind == "squeeze" else None)
            if kind == "normal":
                normal = cm
            if fr and kind != "squeeze" and self.squeeze:
                i = cm.cell(*start)
                if i is None or cm.cost[i] >= MARGIN:
                    # squeezing and still in the gap: the wide-margin map only gets out of it through its
                    # escape zone, whose first leg may cut a jamb corner - get through on the squeeze map first
                    held, fr = held or (fr, kind), None
                    continue
            if fr:
                if kind == "squeeze":
                    self._sq_free_now = cm.state.count(1)
                    self._sq_cm = cm
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
        path[0] = start
        self._sq_clear_now = self._route_clear(self._sq_cm, path) if kind == "squeeze" else None
        path = head[:-1] + path
        return ("path", path, "exploring: frontier %.1f m away (%d left)" % (dist / 1000, len(fr)), kind)

    @staticmethod
    def _route_clear(cm, route):
        """A squeeze route's clearance (mm, beyond the robot radius) as the map has the walls (#108). Not counted:
        the first 100 mm from the robot (it may start beside a wall: that is where it is, not where it squeezes)."""
        if len(route) < 2:
            return None
        (ax, ay), (bx, by) = route[0], route[1]
        L = math.hypot(bx - ax, by - ay)
        f = min(0.5, 100.0 / L) if L > 0 else 0.0
        pts = [(ax + (bx - ax) * f, ay + (by - ay) * f)] + list(route[1:])
        return max(0.0, cm.wall_clearance(pts) - cm.radius_mm)

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
        if rest and self.squeeze:
            # #108: the gap's jambs show up better as the robot comes closer: keep the route on its centre line
            new = cm.recentre([pose[:2]] + rest, self.mode == "goto")
            if new:
                self._sq_clear_now = self._route_clear(cm, new)
                return ("recentre", new)
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
        if self.backing is not None:          # zone escape: backing straight away from the edge (#114)
            self._back_tick(now)
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
            if self.zone_settle and now < self.replan_at + ZONE_SETTLE_MAX_S:
                # zone stop (#114): HOLD brings it to a stand (back to where it stopped, after an overshoot)
                if abs((t or {}).get("speed_mms", 0.0)) > ZONE_STILL_MMS:
                    self.zone_still = None
                    return
                self.zone_still = self.zone_still or now
                if now - self.zone_still < ZONE_STILL_S:
                    return
            self.replan_at, self.zone_settle, self.zone_still = None, False, None
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
