"""Retrace steps when the pose is lost: drive back along the path just driven to where the map is rich, so the
relocalisation finds the robot there (the planner's job is paused meanwhile and resumed once the pose is back).

  1. Breadcrumbs (on_scan, every LD19 turn): the robot's ODOMETRY pose every SPACING_MM of travel, the last KEEP_MM
     of path. A crumb laid while the matcher tracks the pose ("fine": track mode, no doubts) is trusted and keeps the
     correction of the moment (its map pose) and a richness: the distinct known wall cells (grid byte > OCC_BYTE) the
     turn's points land on from there (every RICH_STRIDE-th point, ~150 grid lookups per crumb: cheap on the Pi).
     Crumbs laid while lost only mark the way back (odometry is still good over a few metres); none while the wheels
     slip, and a jump in the odometry (Teensy reboot / pose reset) or a new map drops them all.
  2. Trigger (tick, from Planner.tick): a goto / explore job, the matcher lost for AFTER_S (its own wide search and the
     lidar gap check had their go) and no pose to confirm: the job pauses and the robot drives back along the crumbs,
     newest first, as short GOTO lists in the odometry frame (the map pose is not trusted; the navigator's speed falls
     with the distance to the last point, so ~AHEAD_MM ahead keeps it slow, ~150 mm/s). The route ends at the richest
     trusted crumb within the setting retrace_max_mm of path; on the way it stops at every rich one (>= RICH_F of the
     best, at least RICH_MIN cells, STOP_GAP_MM apart).
  3. Safety: the live lidar turn (odometry frame) must show nothing in the corridor ahead (robot radius + SIDE_MM, up to
     the next points): else the next crumbs are skipped (up to SKIP_MAX), else it stops and waits, then gives up. No
     fresh turn: it stops. The robot's own obstacle stop, contact and stuck handling, and the bridge's zone watchdog
     (a stop there ends the retrace) stay as they are.
  4. At a rich crumb: stand still, then the whole-map search restricted to a BOX_MM / BOX_DEG window around where the
     crumb's trusted correction puts the robot (lidarmatch.request_global(by="retrace", around=...)), confirmed by the
     matcher's 2 strict matches as any found pose. Found (track again): the paused job is planned again from here.
  5. Gives up when the crumbs run out, after retrace_max_mm of driving, or when blocked: it stops, the whole-map search
     runs again and the map says to use Set position; the job stays paused until the pose is found (or the user stops
     it). Planner setting retrace_lost (default on) turns it off.
"""
import collections
import math

from . import link_protocol as lp
from .lidarmatch import OCC_BYTE, apply

SPACING_MM = 150.0         # a crumb every 150 mm of odometry travel ...
KEEP_MM = 20000.0          # ... the last 20 m of path
JUMP_MM = 600.0            # the odometry moved more than this between two turns: a reset / reboot, the crumbs are void
RICH_STRIDE = 3            # richness: every 3rd point of the turn
AFTER_S = 4.0              # lost this long (the in-place search first): retrace
AHEAD_MM = 250.0           # the GOTO list reaches this far ahead (the navigator slows to ~kp x distance: ~150 mm/s)
REACH_MM = 120.0           # a crumb on the way counts as passed this close
STOP_REACH_MM = 80.0       # ... a stop (rich crumb) as reached
SIDE_MM = 30.0             # corridor half width = the planner's robot radius + this
NEAR_SELF_MM = 100.0       # scan points this close to the axle are the robot itself (cage posts) / handled by contact
SKIP_MAX = 3               # an obstacle on the way: try up to this many crumbs further back
SCAN_STALE_S = 0.6         # no lidar turn this long: stop and wait ...
SCAN_GIVE_UP_S = 4.0       # ... and give up after this long
BLOCK_WAIT_S = 2.0         # blocked: stand this long, then look again ...
BLOCK_TRIES = 3            # ... this many times
NO_PROGRESS_S = 8.0        # nearer the next crumb by < 40 mm in this long: blocked
SETTLE_S, SETTLE_MAX_S, STILL_MMS = 0.4, 2.5, 30.0   # at a stop: still (< 30 mm/s) for 0.4 s, or after 2.5 s anyway
CHECK_S = 8.0              # at a stop: wait this long for the local search (+ CONFIRM_EXTRA_S once a pose is confirming)
CONFIRM_EXTRA_S = 8.0
SET_WAIT_S = 12.0          # Set position while retracing: stand this long for its confirmation, then the job goes on
BOX_MM, BOX_DEG = 600.0, 20.0   # local search window around the crumb's map pose
RICH_F, RICH_MIN = 0.75, 25     # a stop: >= 75 % of the best richness on the route, >= 25 cells
STOP_GAP_MM = 500.0        # stops at least this far apart along the route
TRAIL_MAX = 60             # crumbs sent to the browsers
PER_JOB = 3                # retraces per job at most (a spot that keeps losing the pose: the job goes on as before)


Crumb = collections.namedtuple("Crumb", "x y h path corr rich")   # odometry pose; path mm; corr None: untrusted


def richness(z, corr, grid, stride=RICH_STRIDE):
    """Distinct known wall cells the turn's points (complex, odometry frame) land on with the correction corr."""
    cells, W, H, res, x0, y0 = grid.cells, grid.W, grid.H, grid.res, grid.x0, grid.y0
    th = math.radians(corr[2])
    c, s = math.cos(th), math.sin(th)
    tx, ty = corr[0], corr[1]
    seen = set()
    for p in z[::stride]:
        x, y = p.real, p.imag
        ix = int((c * x - s * y + tx - x0) // res)
        iy = int((s * x + c * y + ty - y0) // res)
        if 0 <= ix < W and 0 <= iy < H:
            i = iy * W + ix
            if cells[i] > OCC_BYTE:
                seen.add(i)
    return len(seen)


def corridor_hit(z, pts, half, near_self=NEAR_SELF_MM):
    """The first scan point (complex, odometry frame) within half of the polyline pts [(x, y)] (robot first), or None.
    Points within near_self of the robot are left out; around each later point a disc of half counts too."""
    if len(pts) < 2:
        return None
    rx, ry = pts[0]
    ns2, h2 = near_self * near_self, half * half
    legs = []
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L > 1.0:
            legs.append((ax, ay, bx, by, dx / L, dy / L, L))
    for p in z:
        x, y = p.real, p.imag
        if (x - rx) ** 2 + (y - ry) ** 2 < ns2:
            continue
        for ax, ay, bx, by, ux, uy, L in legs:
            qx, qy = x - ax, y - ay
            t = qx * ux + qy * uy
            if (0.0 <= t <= L and abs(qy * ux - qx * uy) <= half) or (x - bx) ** 2 + (y - by) ** 2 <= h2:
                return x, y
    return None


class Retracer:
    """Planner.retrace. on_scan: a lidar scan listener (crumbs, the live obstacle view); tick: from Planner.tick."""

    def __init__(self, planner):
        self.p = planner
        self.crumbs = collections.deque()
        self.ver = None              # mapper.version the crumbs belong to
        self.last_odo = None         # (x, y) of the last turn: odometry jumps
        self.scan = None             # (lidar clock, [complex points, odometry frame]) of the last turn
        self.st = None               # retracing: see _start
        self.done_lost = None        # lost_t0 of the lost episode already handled (retraced / given up / cancelled)
        self.n = {"started": 0, "found": 0, "gave_up": 0}
        self.job_n = 0               # retraces in this job
        lid = getattr(getattr(planner.b, "mapper", None), "lidar", None)
        if lid is not None:
            lid.scan_listeners.append(self.on_scan)

    # ---------------------------------------------------------- inputs
    def _lm(self):
        return self.p.b.mapper.lmatch

    def _trusted(self, lm):
        S = self.p.b.mapper.settings
        return (lm.mode == "track" and lm.fails < 1.0 and lm.jump is None and lm.suspect is None
                and lm.turns > int(S.get("match_warmup", 0)) and lm.state.startswith("matched"))

    def on_scan(self, scan):
        turn = getattr(scan, "turn", None)
        if not turn or turn.tilted or not turn.z:
            return
        m = self.p.b.mapper
        lid = m.lidar
        self.scan = (lid.clock() if lid is not None else self.p.clock(), turn.z)
        if not m.settings.get("lidar_match"):
            return
        if self.ver != m.version:
            self.crumbs.clear()
            self.ver = m.version
        x, y, h = turn.mid
        lo, self.last_odo = self.last_odo, (x, y)
        if lo is not None and math.hypot(x - lo[0], y - lo[1]) > JUMP_MM:
            self.crumbs.clear()             # a pose reset / Teensy reboot: another odometry frame
        lm = m.lmatch
        if lm.mode == "slip":
            return                          # the wheels' travel is not the body's
        c = self.crumbs[-1] if self.crumbs else None
        d = math.hypot(x - c.x, y - c.y) if c else 0.0
        if c is not None and d < SPACING_MM:
            return
        trusted = self._trusted(lm)
        if lm.mode != "track" and not trusted and (c is None or self.st is not None):
            return                          # lost: no crumbs to start a trail with, nor while driving back
        corr = list(m.corr) if trusted else None
        rich = richness(turn.z, corr, m.grid) if trusted else 0
        self.crumbs.append(Crumb(x, y, h, (c.path + d) if c else 0.0, corr, rich))
        while self.crumbs and self.crumbs[-1].path - self.crumbs[0].path > KEEP_MM:
            self.crumbs.popleft()

    def scan_age(self):
        lid = self.p.b.mapper.lidar
        if self.scan is None or lid is None:
            return 1e9
        return lid.clock() - self.scan[0]

    # ---------------------------------------------------------- the planner's side
    def active(self):
        return self.st is not None

    def reset(self):
        """The job ended / a new one: stop retracing (the robot keeps what the caller sends next); not again in this
        lost episode."""
        lm = getattr(getattr(self.p.b, "mapper", None), "lmatch", None)
        if self.st is not None:
            self.done_lost = self.st["lost_t0"]
        elif lm is not None and lm.mode == "lost":
            self.done_lost = lm.lost_t0
        self.st = None
        self.job_n = 0

    def msg(self):
        st = self.st
        if st is None:
            return None
        route = st["route"]
        k = st["j"]
        trail = [[round(c.x), round(c.y)] for c in route[k:k + TRAIL_MAX]]
        stops = [i - k for i in st["stops"] if k <= i < k + TRAIL_MAX]
        return {"phase": st["phase"], "trail": trail, "stops": stops, "back_mm": round(st["driven"]),
                "max_mm": round(self.p.settings["retrace_max_mm"])}

    def tick(self, now):
        """True while retracing / waiting for the pose: the planner's job waits (paused)."""
        if self.st is None:
            if not self._due(now):
                return False
            self._start(now)
            return self.st is not None
        lm = self._lm()
        if not self.p.settings.get("retrace_lost", True):
            self._resume(now, "still lost, retracing switched off")
            return True
        if lm.mode != "lost":
            self._resume(now, "found again" if lm.mode == "track" else lm.mode)
            return True
        st = self.st
        t = self.p.b.tlm
        if t:
            if st["odo"] is not None:
                st["driven"] += math.hypot(t["x_mm"] - st["odo"][0], t["y_mm"] - st["odo"][1])
            st["odo"] = (t["x_mm"], t["y_mm"])
        ph = st["phase"]
        if (lm.reloc or {}).get("phase") in ("set", "set_poor") and ph != "hold":
            self._stop_robot()               # Set position: the user's pose, confirmed standing
            self._enter(now, "hold", "lost - position set by hand: confirming it")
            return True
        if ph == "hold":
            if now - st["since"] > SET_WAIT_S:
                self._resume(now, "set by hand (not confirmed yet)")
            return True
        # (a pose the whole-map search found is confirmed by the turns while it drives on: no stop for it)
        if ph == "drive":
            self._drive(now)
        elif ph == "wait":
            if now >= st["until"]:
                self._enter(now, "drive", "lost - retracing steps")
                st["sent"] = None
        elif ph == "settle":
            self._settle(now)
        elif ph == "check":
            self._check(now)
        return True

    def _due(self, now):
        p = self.p
        if not p.settings.get("retrace_lost", True) or p.mode not in ("goto", "explore") or self.job_n >= PER_JOB:
            return False
        if p.backing is not None or p.replan_at is not None or not p.b.tlm:
            return False
        m = getattr(p.b, "mapper", None)
        lm = getattr(m, "lmatch", None)
        if getattr(m, "lidar", None) is None or lm is None:
            return False
        if lm.mode != "lost" or lm.checking or lm.lost_t0 == self.done_lost:
            return False
        if (lm.reloc or {}).get("phase") in ("set", "set_poor"):
            return False                     # the user's pose: no driving off
        return lm._now() - lm.lost_t0 >= AFTER_S

    # ---------------------------------------------------------- start / end
    def _route(self, t):
        """Crumbs newest first within retrace_max_mm of path from the robot, and the stops on them."""
        cap = self.p.settings["retrace_max_mm"]
        cs = list(self.crumbs)
        if not cs:
            return [], []
        x, y = t["x_mm"], t["y_mm"]
        head = math.hypot(cs[-1].x - x, cs[-1].y - y)
        top = cs[-1].path
        route = [c for c in reversed(cs) if head + top - c.path <= cap]
        trusted = [(i, c) for i, c in enumerate(route) if c.corr is not None]
        if not trusted:
            return route, []
        best = max(c.rich for _, c in trusted)
        lim = max(RICH_MIN, RICH_F * best)
        stops = []
        for i, c in trusted:
            if c.rich < lim:
                continue
            if stops and route[stops[-1]].path - c.path < STOP_GAP_MM:
                if c.rich > route[stops[-1]].rich:
                    stops[-1] = i            # two close together: the richer one
                continue
            stops.append(i)
        if not stops:                        # nothing rich: head for the best trusted one anyway
            stops = [max(trusted, key=lambda ic: ic[1].rich)[0]]
        return route[:stops[-1] + 1], stops

    def _start(self, now):
        p = self.p
        lm = self._lm()
        t = p.b.tlm
        route, stops = self._route(t)
        self.n["started"] += 1
        self.job_n += 1
        p.gen += 1                           # a planner job still running: its result is dropped
        p.pending = None
        p.holding, p.scan_wait = False, None
        self.st = {"lost_t0": lm.lost_t0, "route": route, "stops": stops, "j": 0, "stop_k": 0, "phase": "drive",
                   "since": now, "sent": None, "sent_at": -1e9, "sent_n": 0, "driven": 0.0,
                   "odo": (t["x_mm"], t["y_mm"]), "blocks": 0, "prog": None, "asked": None, "until": 0.0,
                   "scan_wait": None, "job": p.mode}
        if not stops:
            self._give_up(now, "no well-mapped spot on the way back (%d crumbs)" % len(route))
            return
        p.path = []
        c = route[stops[-1]]
        p.b.say("planner: pose lost for %.0f s - pausing the %s, retracing %d steps (%.1f m) back to a well-mapped "
                "spot (%d wall cells seen there)" % (lm._now() - lm.lost_t0, "route" if p.mode == "goto" else
                                                      "exploring", stops[-1] + 1,
                                                      (math.hypot(route[0].x - t["x_mm"], route[0].y - t["y_mm"])
                                                       + route[0].path - c.path) / 1000.0, c.rich))
        self._enter(now, "drive", "lost - retracing steps")
        self._drive(now)

    def _enter(self, now, phase, status):
        self.st["phase"], self.st["since"] = phase, now
        self.p._set_status(status, log=phase != "drive")

    def _resume(self, now, how, log=True):
        """Retracing is over (found, given up, switched off): the paused job is planned again from here."""
        p = self.p
        st, self.st = self.st, None
        self.done_lost = st["lost_t0"]
        if self._lm().mode == "track":
            self.n["found"] += 1
        if log:
            p.b.say("planner: pose %s after %.1f m of retracing - resuming the %s" % (
                how, st["driven"] / 1000.0, "route" if p.mode == "goto" else "exploring"))
        p.progress = None
        p.blocked_plans = 0
        p.last_check = now
        if p.mode == "explore":
            p.started_at = now               # (explore "looking around" grace, as at a start)
        p._start_job(p._plan_goto if p.mode == "goto" else p._plan_explore, go=True, force=True)

    def _give_up(self, now, why):
        """Back to the behaviour without retracing: the whole-map search (now, then every 12 s), the map's Set
        position prompt, and the job goes on with the pose it has."""
        p = self.p
        lm = self._lm()
        self.n["gave_up"] += 1
        self._stop_robot()
        if lm.mode == "lost" and lm.m.settings.get("match_global", True) and not lm.glob_running:
            lm.request_global("retracing gave up (%s)" % why, by="auto")
        p.b.say("planner: lost - retracing gave up: %s after %.1f m; the whole-map search goes on (use Set position "
                "if it does not find the robot), the %s goes on" % (why, self.st["driven"] / 1000.0,
                                                                    "route" if p.mode == "goto" else "exploring"))
        self._resume(now, why, log=False)

    # ---------------------------------------------------------- driving
    def _stop_robot(self):
        t = self.p.b.tlm
        if t:
            self.p.b.handle_command({"t": "goto", "points": [[t["x_mm"], t["y_mm"]]], "replace": True}, "planner")
            self.st["sent"], self.st["sent_n"], self.st["sent_at"] = None, 1, self.p.clock()

    def _nav_state(self, now):
        st = self.st
        nav = self.p.b.nav or {}
        if now - st["sent_at"] <= 0.6 or nav.get("waypoint_count") != st["sent_n"]:
            return ""
        s = nav.get("state", 99)
        names = self.p.nav_names
        return names[s] if s < len(names) else ""

    def _blocked(self, now, why):
        st = self.st
        st["blocks"] += 1
        if st["blocks"] > BLOCK_TRIES:
            self._give_up(now, "blocked on the way back (%s)" % why)
            return
        self._stop_robot()
        st["until"] = now + BLOCK_WAIT_S
        st["prog"] = None
        self._enter(now, "wait", "lost - retracing steps: %s, waiting" % why)

    def _drive(self, now):
        p, st = self.p, self.st
        t = p.b.tlm
        route, stops = st["route"], st["stops"]
        cap = p.settings["retrace_max_mm"]
        if st["driven"] > cap + 300.0:
            self._give_up(now, "drove %.1f m back" % (st["driven"] / 1000.0))
            return
        age = self.scan_age()
        if age > SCAN_STALE_S:               # no live lidar view: do not drive blind
            if st["scan_wait"] is None:
                st["scan_wait"] = now
                self._stop_robot()
            if now - st["scan_wait"] > SCAN_GIVE_UP_S:
                self._give_up(now, "no lidar data")
            return
        st["scan_wait"] = None
        x, y = t["x_mm"], t["y_mm"]
        stop_i = stops[st["stop_k"]]
        while st["j"] < stop_i and math.hypot(route[st["j"]].x - x, route[st["j"]].y - y) < REACH_MM:
            st["j"] += 1
        j = st["j"]
        c = route[stop_i]
        if j == stop_i and (math.hypot(c.x - x, c.y - y) < STOP_REACH_MM or
                            (self._nav_state(now) == "arrived" and math.hypot(c.x - x, c.y - y) < 2 * REACH_MM)):
            self._stop_robot()
            self._enter(now, "settle", "lost - retracing steps: checking the pose here")
            st["still"] = None
            return
        if self._nav_state(now) == "blocked":
            self._blocked(now, "the robot reports blocked")
            return
        # the GOTO list: crumbs from j on, AHEAD_MM of path, never past the next stop
        pts, d, px, py = [], 0.0, x, y
        for k in range(j, stop_i + 1):
            d += math.hypot(route[k].x - px, route[k].y - py)
            pts.append(k)
            px, py = route[k].x, route[k].y
            if d >= AHEAD_MM:
                break
        z = self.scan[1]
        half = p.settings["robot_radius_mm"] + SIDE_MM
        hit = corridor_hit(z, [(x, y)] + [(route[k].x, route[k].y) for k in pts[:2]], half)
        if hit is not None:
            # something on the way back that was not there: skip to a crumb further back the lidar sees clear
            for k in range(j + 1, min(stop_i, j + SKIP_MAX) + 1):
                if corridor_hit(z, [(x, y), (route[k].x, route[k].y)], half) is None:
                    p.b.say("planner: retracing - obstacle at %d, %d (odometry): skipping %d step%s" % (
                        hit[0], hit[1], k - j, "" if k - j == 1 else "s"))
                    st["j"] = j = k
                    pts = [i for i in pts if i >= k] or [k]
                    hit = None
                    break
            if hit is not None:
                self._blocked(now, "something %d mm ahead" % math.hypot(hit[0] - x, hit[1] - y))
                return
        # progress: nearer the next crumb
        dn = math.hypot(route[j].x - x, route[j].y - y)
        pr = st["prog"]
        if pr is None or pr[0] != j or dn < pr[1] - 40.0:
            st["prog"] = (j, dn, now)
        elif now - pr[2] > NO_PROGRESS_S:
            self._blocked(now, "no progress")
            return
        if st["sent"] != pts[0] or now - st["sent_at"] > 2.0:
            moved = st["sent"] != pts[0]
            self._send([(route[k].x, route[k].y) for k in pts])
            st["sent"] = pts[0]
            if moved:
                p._push()                    # the browsers' trail

    def _send(self, pts):
        p, st = self.p, self.st
        p.b.handle_command({"t": "goto", "points": [[round(a, 1), round(b, 1)] for a, b in pts], "replace": True},
                           "planner")
        st["sent_at"], st["sent_n"] = p.clock(), len(pts)
        mode = lp.MODE_GOTO if p.mode == "goto" else lp.MODE_EXPLORE
        t = p.b.tlm
        if t and t["mode"] != mode and t["mode"] in (lp.MODE_GOTO, lp.MODE_EXPLORE, lp.MODE_BALANCE):
            p.b.handle_command({"t": "set_mode", "mode": mode, "arm": False}, "planner")

    def _settle(self, now):
        st = self.st
        t = self.p.b.tlm
        if abs((t or {}).get("speed_mms", 0.0)) > STILL_MMS:
            st["still"] = None
        else:
            st["still"] = st.get("still") or now
        if (st["still"] is not None and now - st["still"] >= SETTLE_S) or now - st["since"] > SETTLE_MAX_S:
            self._ask(now)

    def _ask(self, now):
        """At a stop: the whole-map search in a window around where the crumb's trusted correction puts the robot."""
        st = self.st
        lm = self._lm()
        c = st["route"][st["stops"][st["stop_k"]]]
        t = self.p.b.tlm
        x, y = apply(c.corr, t["x_mm"], t["y_mm"])
        if lm.m.settings.get("match_global", True):
            lm.request_global("retracing: back at a well-mapped spot (%d wall cells)" % c.rich, by="retrace",
                              around=(x, y, BOX_MM, BOX_DEG))
        st["asked"] = now
        self._enter(now, "check", "lost - retracing steps: checking the pose here")

    def _check(self, now):
        st = self.st
        lm = self._lm()
        rl = lm.reloc or {}
        ph = rl.get("phase")
        limit = CHECK_S + (CONFIRM_EXTRA_S if ph == "confirming" else 0.0)
        answered = (st["asked"] is not None and rl.get("by") == "retrace" and rl.get("t", -1e9) >= st["asked"]
                    and ph in ("none", "ambiguous", "unconfirmed"))
        if not answered and now - st["since"] < limit:
            return
        st["stop_k"] += 1                    # no fix here: on to the next well-mapped spot
        if st["stop_k"] >= len(st["stops"]):
            self._give_up(now, "no fix at %d well-mapped spot%s" % (len(st["stops"]),
                                                                   "" if len(st["stops"]) == 1 else "s"))
            return
        st["sent"], st["prog"], st["blocks"] = None, None, 0
        self._enter(now, "drive", "lost - retracing steps")

    # ---------------------------------------------------------- events while retracing
    def on_zone_stop(self, why):
        """The bridge's zone watchdog stopped the robot (HOLD): the map pose is not trusted, so no escape move - stop
        retracing there."""
        self._give_up(self.p.clock(), "stopped by the zone watchdog (%s)" % why)

    def on_bump(self, why):
        """Contact / the robot's stuck back-off: it backs off by itself; look again in a moment."""
        if self.st["phase"] in ("drive", "wait"):
            self._blocked(self.p.clock(), why)
