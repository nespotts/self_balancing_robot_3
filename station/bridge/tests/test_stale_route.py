"""Stale routes: a plan starts from the pose when its job began, and on the Pi a plan takes seconds - the robot drove on
meanwhile. The route it gets must start ahead of it, not at the corner it passed (session 2026-11-25 15:58, t=493.24:
it turned round, drove ~500 mm back to that corner and came forward again). Run: python -m unittest discover -s station -t ."""
import concurrent.futures
import math
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.bridge.planner import DEFAULTS, CostMap, Planner, project_on_route
from station.bridge.tests.test_zones import free_grid
from station.sim.tour import DriftRobot, heartbeat, plan_offline

DRIVING = lp.NAV_STATE_NAMES.index("driving")


class FakeBridge:
    def __init__(self, x, y, mode=lp.MODE_EXPLORE):
        self.mapper = Mapper(None)
        self.tlm = {"x_mm": x, "y_mm": y, "heading_deg": 70.0, "mode": mode, "speed_mms": 250.0}
        self.nav = {}
        self.cmds, self.lines = [], []
        self.now = 100.0

    def say(self, text):
        self.lines.append(text)

    def broadcast(self, text, droppable=False):
        pass

    def handle_command(self, m, who="?"):
        self.cmds.append(m)


def planner(b, mode):
    p = Planner(b, lp.NAV_STATE_NAMES, clock=lambda: b.now, sync=True)
    p.mode = mode
    p.started_at = b.now - 60.0
    return p


def done(p, res, t0, fn=None):
    fut = concurrent.futures.Future()
    fut.set_result(res)
    p._job_done(fut, True, p.gen, fn, t0)


def costmap(walls=()):
    g = free_grid(walls=walls)
    g.touched = {(tx, ty) for tx in range(g.ntx) for ty in range(g.nty)}
    return CostMap(g, DEFAULTS)


def gotos(b):
    return [[tuple(q) for q in m["points"]] for m in b.cmds if m["t"] == "goto"]


# the 493.24 plan: started at 489.3 from (14650, 16915), before the corner (14770, 16731) it passed at 489.81
STALE = [(14650.0, 16915.0), (14770.1, 16731.3), (15028.8, 17377.9), (14282.3, 17638.0), (14232.3, 17638.7)]
NOW = (15007.0, 17223.0)          # where the robot was when it came back (~150 mm short of its next point)


class TestProjection(unittest.TestCase):
    def test_project(self):
        k, p, off = project_on_route(STALE, NOW)
        self.assertEqual(k, 2)                                    # next point ahead: (15028.8, 17377.9)
        self.assertLess(off, 60.0)
        self.assertEqual(project_on_route(STALE, STALE[0])[0], 1)  # standing at the start: the first point
        self.assertEqual(project_on_route(STALE, STALE[1])[0], 2)  # at a corner: the one after it
        # a route that comes back past here later: the first leg it is on wins
        back = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 30.0), (0.0, 30.0)]
        self.assertEqual(project_on_route(back, (400.0, 5.0))[0], 1)


class TestStaleStart(unittest.TestCase):
    def test_493_first_point_behind_is_dropped(self):
        b = FakeBridge(*NOW)
        p = planner(b, "explore")
        done(p, ("path", list(STALE), "exploring: frontier 1.2 m away (3 left)", "normal"), t0=b.now - 3.9)
        sent = gotos(b)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0], STALE[2:])                     # not the corner 500 mm behind it
        self.assertEqual(p.path[0], NOW)
        self.assertTrue(any("plan took 3.9 s" in s and "dropped 1 passed point" in s for s in b.lines), b.lines)

    def test_same_route_not_sent_again(self):
        # it follows the 491.67 list (its next point 15027.6, 17374.2): the 493.24 plan is the same route
        b = FakeBridge(*NOW)
        p = planner(b, "explore")
        p.path = [(14980.0, 16800.0), (15027.6, 17374.2), (14279.8, 17630.8), (14229.8, 17631.3)]
        p.sent_n = 3
        b.nav = {"state": DRIVING, "waypoint_count": 3, "nav_index": 0}
        done(p, ("path", list(STALE), "exploring", "normal"), t0=b.now - 3.9)
        self.assertEqual(gotos(b), [])                            # no restart at index 0
        self.assertEqual(p.path[1], (15027.6, 17374.2))
        self.assertTrue(any("not sent again" in s for s in b.lines), b.lines)
        # a different route is sent
        other = STALE[:2] + [(15028.8, 17377.9), (14600.0, 18200.0)]
        done(p, ("path", other, "exploring", "normal"), t0=b.now - 1.0)
        self.assertEqual(gotos(b), [other[2:]])

    def test_off_route_join(self):
        # it drove off the planned line (1200 mm): straight to the next point when that leg is clear ...
        b = FakeBridge(1000.0, -1200.0, mode=lp.MODE_GOTO)
        p = planner(b, "goto")
        p._cms["normal"] = costmap()
        done(p, ("path", [(0.0, 0.0), (2000.0, 0.0)], "route", "normal"), t0=b.now - 3.0)
        self.assertEqual(gotos(b), [[(2000.0, 0.0)]])
        self.assertTrue(any("joins it" in s for s in b.lines), b.lines)

    def test_off_route_blocked_replans_from_here(self):
        # ... a wall between it and the route: plan again from where it is (once)
        b = FakeBridge(1000.0, -1200.0, mode=lp.MODE_GOTO)
        p = planner(b, "goto")
        p._cms["normal"] = costmap(walls=[(-500.0, -600.0, 2500.0, -600.0)])
        calls = []

        def plan(pose):
            calls.append(pose[:2])
            return ("path", [pose[:2], (2900.0, -1200.0), (2900.0, 0.0), (2000.0, 0.0)], "route", "normal")
        done(p, ("path", [(0.0, 0.0), (2000.0, 0.0)], "route", "normal"), t0=b.now - 3.0, fn=plan)
        self.assertEqual(calls, [(1000.0, -1200.0)])
        self.assertEqual(gotos(b), [[(2900.0, -1200.0), (2900.0, 0.0), (2000.0, 0.0)]])
        self.assertTrue(any("planning again" in s for s in b.lines), b.lines)

    def test_recentre_trimmed(self):
        b = FakeBridge(*NOW)
        p = planner(b, "explore")
        done(p, ("recentre", list(STALE)), t0=b.now - 2.0)
        self.assertEqual(gotos(b), [STALE[2:]])


class TestSlowPlannerSim(unittest.TestCase):
    def test_explore_with_slow_planner_never_backtracks(self):
        """Every plan takes 2 s of sim time (the robot drives on its old route meanwhile, as on the Pi). No route
        the planner sends may start at a point the robot already passed."""
        r = DriftRobot(seed=2)
        m = Mapper(tempfile.mkdtemp())
        orig = Planner._plan_explore
        trail, bad, sends = [], [], [0]

        def slow(self, pose):
            res = orig(self, pose)
            rob, b = self.b.robot, self.b
            t_end = rob.t + 2.0
            while rob.t < t_end:
                heartbeat(rob)
                for _ in range(40):
                    rob.step()
                b.update()
                trail.append((b.tlm["x_mm"], b.tlm["y_mm"]))
            return res

        def watch(robot, b):
            trail.append((b.tlm["x_mm"], b.tlm["y_mm"]))
            orig_hc = getattr(b, "_orig_hc", None)
            if orig_hc is None:
                b._orig_hc = b.handle_command

                def hc(msg, who="?"):
                    if msg["t"] == "goto" and who == "planner" and len(msg["points"]) > 1:
                        sends[0] += 1
                        fx, fy = msg["points"][0]
                        x, y = b.tlm["x_mm"], b.tlm["y_mm"]
                        # passed in the last few seconds (not long ago: explore may head back that way)
                        passed = any(math.hypot(px - fx, py - fy) < 150 for px, py in trail[-80:-15])
                        if passed and math.hypot(x - fx, y - fy) > 300:
                            bad.append((round(robot.t, 1), (round(x), round(y)), msg["points"][0]))
                    b._orig_hc(msg, who)
                b.handle_command = hc

        Planner._plan_explore = slow
        try:
            s, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=150, on_step=watch)
        finally:
            Planner._plan_explore = orig
        self.assertGreater(sends[0], 3)
        self.assertEqual(bad, [])
        self.assertTrue(any("plan took 2.0 s" in s for s in b.lines), b.lines[-10:])
        _occ, free = m.grid.counts()
        self.assertGreater(free * 0.025 * 0.025, 5.0)              # it still explores


if __name__ == "__main__":
    unittest.main()
