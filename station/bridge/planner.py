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
import time

from . import link_protocol as lp
from .occgrid import UNKNOWN, UNIT

SQ2 = math.sqrt(2.0)
BLOCKED = 255             # within robot radius + 30 mm of an obstacle: never
MARGIN = 250              # within radius + margin: only to drive out of it near the start
OCC_P, FREE_P = 0.62, 0.40
OCC_BYTE = UNKNOWN + math.log(OCC_P / (1 - OCC_P)) / UNIT     # ~138
FREE_BYTE = UNKNOWN + math.log(FREE_P / (1 - FREE_P)) / UNIT  # ~120
# fine byte -> class bits: 2 = occupied, 1 = free, 0 = unknown (OR-able: occupied wins)
CLASS_LUT = bytes(2 if v > OCC_BYTE else (1 if v < FREE_BYTE else 0) for v in range(256))
FINAL_LUT = bytes([0, 1, 2, 2] + [0] * 252)

DEFAULTS = {
    "robot_radius_mm": 80.0,    # footprint radius the planner keeps off obstacles
    "margin_mm": 120.0,         # + this much (the robot sways, the map is fuzzy)
    "soft_mm": 200.0,           # + a band that costs extra (keeps routes in the middle)
    "unknown_ok": True,         # "go here" may route through unmapped space (explore never does)
    "frontier_min_mm": 200.0,   # ignore frontier bits smaller than this
}
LIMITS = {"robot_radius_mm": (20, 400), "margin_mm": (0, 500), "soft_mm": (0, 1000),
          "frontier_min_mm": (50, 2000)}


class CostMap:
    def __init__(self, grid, settings=None, res_mm=50.0, margin_m=1.0):
        S = dict(DEFAULTS, **(settings or {}))
        self.g = grid
        f = max(1, int(round(res_mm / grid.res)))
        self.f, self.res = f, grid.res * f
        bb = grid.bbox_tiles()
        T = grid.T
        if bb is None:
            bb = (grid.ntx // 2, grid.nty // 2, grid.ntx // 2, grid.nty // 2)
        m = int(math.ceil(margin_m * 1000.0 / (grid.res * T)))
        tx0, ty0 = max(0, bb[0] - m), max(0, bb[1] - m)
        tx1, ty1 = min(grid.ntx - 1, bb[2] + m), min(grid.nty - 1, bb[3] + m)
        self.fx0, self.fy0 = tx0 * T, ty0 * T                     # fine cell of coarse (0, 0)
        self.w = ((tx1 + 1) * T - self.fx0) // f
        self.h = ((ty1 + 1) * T - self.fy0) // f
        self.x0 = grid.x0 + self.fx0 * grid.res                    # world mm of the coarse (0, 0) corner
        self.y0 = grid.y0 + self.fy0 * grid.res
        self._classify()
        self.inner_r = (S["robot_radius_mm"] + 30.0) / self.res
        self._inflate(S["robot_radius_mm"] + S["margin_mm"], S["soft_mm"])
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
                acc |= int.from_bytes(bytes(g[s:s + nb]).translate(CLASS_LUT), "big")
            for k in range(1, f):                  # OR each byte with the f-1 before it
                acc |= acc >> (8 * k)
            row = acc.to_bytes(nb, "big")[f - 1::f]
            state += row.translate(FINAL_LUT)
        self.state = state                          # 0 unknown, 1 free, 2 occupied

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

    def simplify(self, cells, unknown_ok):
        """Line-of-sight pruning: keep a cell only when the next one can't be seen from the last kept."""
        if len(cells) <= 2:
            return [self.center(i) for i in cells]
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.clear_line(cells[i], cells[j], unknown_ok, allow_start_zone=(i == 0)):
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

    def frontiers(self, start_xy, min_mm=200.0, blacklist=(), black_r_mm=350.0, escape_mm=350.0):
        """Reachable frontier clusters, nearest first: [(dist_mm, goal_xy, path_xy, size_cells)].
        Frontier = free, unblocked cell with an unknown 4-neighbour. Travel only through free cells."""
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
                if state[k] != 1:
                    continue
                if c >= MARGIN:
                    kx, ky = k % w, k // w
                    d2 = (kx - sx) ** 2 + (ky - sy) ** 2
                    if d2 > esc or (c == BLOCKED and d2 > 4):
                        continue
                    c = 60
                nd = d + step * (1.0 + c / 10.0)
                if nd < dist.get(k, 1e18):
                    dist[k] = nd
                    parent[k] = i
                    heapq.heappush(heap, (nd, k))
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
        self.settings = dict(DEFAULTS)
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

    # ---------------------------------------------------------- frames
    def robot_map_pose(self):
        t = self.b.tlm
        return self.b.mapper.map_pose(t["x_mm"], t["y_mm"], t["heading_deg"]) if t else None

    def to_odom(self, pts):
        dx, dy, dt = self.b.mapper.corr
        c, s = math.cos(math.radians(-dt)), math.sin(math.radians(-dt))
        return [(round(c * (x - dx) - s * (y - dy), 1), round(s * (x - dx) + c * (y - dy), 1)) for x, y in pts]

    def costmap(self):
        return CostMap(self.b.mapper.grid, self.settings)

    # ---------------------------------------------------------- messages
    def msg(self):
        return json.dumps({"t": "plan", "mode": self.mode, "status": self.status,
                           "path": [[round(x), round(y)] for x, y in self.to_odom(self.path)],
                           "goals": [[round(x), round(y)] for x, y in self.to_odom(self.goals)],
                           "settings": self.settings}, separators=(",", ":"))

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
           {"t": "plan", "action": "explore"} | {"action": "stop"} | {"action": "set", "settings": {...}}"""
        act = m.get("action")
        if act == "set":
            for k, v in (m.get("settings") or {}).items():
                if k in DEFAULTS:
                    if isinstance(DEFAULTS[k], bool):
                        self.settings[k] = bool(v)
                    else:
                        try:
                            v = float(v)
                        except (TypeError, ValueError):
                            continue
                        lo, hi = LIMITS.get(k, (-1e9, 1e9))
                        self.settings[k] = max(lo, min(hi, v)) if math.isfinite(v) else self.settings[k]
            self._push()
            return None
        if act == "stop":
            self.stop("stopped by %s" % who, idle_robot=False)
            return None
        if not self.b.tlm:
            return {"t": "error", "text": "no robot pose yet"}
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
            self.mode, self.goals, self.blacklist = "goto", goals[:20], []
            self.started_at = self.clock()
            self.blocked_plans = 0
            self._start_job(self._plan_goto, go=True)
            return None
        if act == "explore":
            self.mode, self.goals, self.blacklist, self.goal = "explore", [], [], None
            self.started_at = self.clock()
            self.blocked_plans = 0
            self._set_status("exploring")
            self._start_job(self._plan_explore, go=True)
            return None
        return {"t": "error", "text": "unknown plan action %r" % act}

    def stop(self, why="", idle_robot=False):
        """Forget the job. The robot keeps its last waypoints unless idle_robot (E-stop does that)."""
        if self.mode == "idle" and not self.path:
            return
        was = self.mode
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
    def _start_job(self, fn, go=False, check=False):
        if self.job and not self.job.done():
            return False
        pose = self.robot_map_pose()
        if pose is None:
            return False
        if not check:
            self.last_plan = self.clock()
        if self.sync:
            fut = concurrent.futures.Future()
            try:
                fut.set_result(fn(pose))
            except Exception as e:
                fut.set_exception(e)
            self._job_done(fut, go)
            return True
        loop = asyncio.get_running_loop()
        self.job = loop.run_in_executor(None, fn, pose)
        self.job.add_done_callback(lambda fut: self._job_done(fut, go))
        return True

    def _job_done(self, fut, go):
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
            _, path, status = res
            self.path = path
            self._send(path[1:] or path[-1:], go)
            self._set_status(status, log=False)
        elif kind == "ok":
            pass
        elif kind == "blacklist":     # explore: that frontier is unreachable / done; try again soon
            self.blacklist.append(res[1])
            self.last_plan = 0.0
        elif kind == "done":
            self.mode, self.path, self.goal = "idle", [], None
            self.b.handle_command({"t": "set_mode", "mode": lp.MODE_BALANCE}, "planner")
            self._set_status(res[1])
        else:   # fail
            self.mode, self.path, self.goal = "idle", [], None
            self._set_status(res[1])

    def _send(self, pts, go):
        odom = self.to_odom(pts)
        self.b.handle_command({"t": "goto", "points": [list(p) for p in odom], "replace": True}, "planner")
        self.sent_at = self.clock()
        self.sent_n = len(odom)
        if go:
            mode = lp.MODE_GOTO if self.mode == "goto" else lp.MODE_EXPLORE
            if not self.b.tlm or self.b.tlm["mode"] != mode:
                self.b.handle_command({"t": "set_mode", "mode": mode}, "planner")

    def _plan_goto(self, pose):
        cm = self.costmap()
        start = pose[:2]
        route = [start]
        for gpt in self.goals:
            leg = cm.astar(route[-1], gpt)
            if leg is None:
                return ("fail", "no route to %.1f, %.1f m" % (gpt[0] / 1000, gpt[1] / 1000))
            leg[-1] = gpt if cm.passable(cm.cell(*gpt) or 0, True) else leg[-1]
            route += leg[1:]
        n = len(route) - 1
        return ("path", route, "route: %d waypoint%s" % (n, "" if n == 1 else "s"))

    def _plan_explore(self, pose):
        cm = self.costmap()
        fr = cm.frontiers(pose[:2], self.settings["frontier_min_mm"], self.blacklist)
        if not fr:
            return ("done", "explore done: no reachable frontier left")
        dist, goal, path, size = fr[0]
        if dist < 120 or len(path) < 2:
            # standing at it and it is still unknown beyond: the sensors can't see it; skip it
            return ("blacklist", goal)
        self.goal = goal
        path[0] = pose[:2]
        return ("path", path, "exploring: frontier %.1f m away (%d left)" % (dist / 1000, len(fr)))

    def _check(self, pose):
        """Periodic: is the remaining route still clear? explore: is the goal still a frontier?"""
        cm = self.costmap()
        rest = self._remaining()
        if self.mode == "explore" and self.goal is not None:
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
        nav = b.nav or {}
        fresh = now - self.sent_at > 0.6 and nav.get("waypoint_count") == getattr(self, "sent_n", -1)
        state = self.nav_names[nav["state"]] if fresh and nav.get("state", 99) < len(self.nav_names) else ""
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
                self.mode, self.path = "idle", []
                self._set_status("arrived")
                return
            if now - self.last_plan > 0.8:
                if self.goal:
                    self.blacklist.append(self.goal)     # we're there; don't come back to it
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
        if now - self.last_check > 1.5 and now - self.last_plan > 1.5 and state != "blocked":
            self.last_check = now
            self._start_job(self._check, go=True, check=True)
