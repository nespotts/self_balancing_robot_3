"""Navigation in the simulator (issue #8): the Navigator port of src/navigator.cpp driving the simulated robot.
Run: python -m unittest discover -s station -t ."""
import math
import os
import re
import unittest

from station.bridge import link_protocol as lp
from station.sim import sim as simmod
from station.sim.sim import SimRobot

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
HB = lp.encode_heartbeat(0, 1)[4:-2]     # payloads (strip sync/type/len and crc)


def payload(frame):
    return frame[4:-2]


class NavRun:
    """Steps the sim with Pi heartbeats (no remote: SET_MODE arms) and collects what it sends."""

    def __init__(self, seed=1):
        self.r = SimRobot(seed=seed)
        self.logs, self.navs, self.tlm = [], [], []

    def frame(self, f):
        self.r.on_frame(f[2], payload(f))

    def run(self, seconds, until=None):
        r = self.r
        end = r.t + seconds
        while r.t + r.dt <= end:
            r.on_frame(lp.MSG_HEARTBEAT, HB)
            r.run_until(min(end, r.t + 0.05), max_steps=100000)
            for f in r.out:
                t, p = f[2], payload(f)
                if t == lp.MSG_LOG:
                    self.logs.append(lp.decode_log(p))
                elif t == lp.MSG_NAV_STATUS:
                    self.navs.append(lp.decode_nav_status(p))
                elif t == lp.MSG_TELEMETRY:
                    self.tlm.append(lp.decode_telemetry(p))
            r.out = []
            if until and until(self):
                return True
        return False

    def stand_up(self, mode):
        """Balance first (the sim's 'hand' that stands it up throws it around), put it back at the origin
        facing where it faced at boot, then switch to `mode`."""
        r = self.r
        th0 = r.TH
        self.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        self.assertTrue(self.run(10.0, lambda s: s.r.ctl.upright and s.r.t > 3.0 and abs(s.r.speed_f) < 40))
        r.ctl.heading_sp += th0 - r.TH
        r.X = r.Y = 0.0
        r.TH = th0
        r.nav.pts = []
        self.run(1.0)
        self.frame(lp.encode_set_mode(mode))

    def assertTrue(self, v):
        if not v:
            raise AssertionError("condition not reached")

    def pos(self):
        x, y, _ = self.r.odom()
        return x, y


class TestParamsMatchFirmware(unittest.TestCase):
    def test_same_table(self):
        with open(os.path.join(ROOT, "src", "params.h")) as fh:
            src = fh.read()
        rows = re.findall(r'X\(\s*\w+,\s*"([^"]+)",\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f\)', src)
        self.assertEqual([r[0] for r in rows], [p[0] for p in simmod.PARAMS])
        for (name, d, lo, hi), (sn, sd, slo, shi) in zip(rows, simmod.PARAMS):
            if name != "rng.enable":       # the sim starts with the sweep on
                self.assertAlmostEqual(float(d), sd, places=2, msg=name)
            self.assertAlmostEqual(float(lo), slo, places=4, msg=name)
            self.assertAlmostEqual(float(hi), shi, places=4, msg=name)

    def test_state_names(self):
        self.assertEqual(lp.NAV_STATE_NAMES[simmod.NAV_HOLDING], "holding")


class TestGoto(unittest.TestCase):
    def test_three_waypoints_arrive_and_stop(self):
        n = NavRun(seed=5)
        pts = [(800.0, 0.0), (800.0, 500.0), (0.0, 500.0)]
        n.frame(lp.encode_goto(pts))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(90.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED), "never arrived: %s" % n.logs[-5:])
        reached = [l for l in n.logs if l.startswith("nav: reached")]
        self.assertEqual(len(reached), 3, n.logs)
        self.assertIn("nav: arrived", n.logs)
        x, y = n.pos()
        self.assertLess(math.hypot(x - 0.0, y - 500.0), 2 * n.r.P("nav.arrive_mm"))
        n.run(2.0)                                      # and it stays stopped
        self.assertLess(abs(n.r.speed_f), 60.0)                       # (balancing sway)
        x2, y2 = n.pos()
        self.assertLess(math.hypot(x2 - x, y2 - y), 40.0)
        # status: prompt + every state seen, the final one ARRIVED with nav_index = count
        states = [m["state"] for m in n.navs]
        for st in (lp.NAV_TURNING, lp.NAV_DRIVING, lp.NAV_ARRIVED):
            self.assertIn(st, states)
        self.assertEqual((n.navs[-1]["state"], n.navs[-1]["nav_index"], n.navs[-1]["waypoint_count"]),
                         (lp.NAV_ARRIVED, 3, 3))
        t = n.tlm[-1]
        self.assertTrue(t["status"] & lp.TLM_NAV_DONE)
        self.assertEqual((t["nav_state"], t["nav_index"]), (lp.NAV_ARRIVED, 3))
        self.assertGreater(n.r.room.clearance(n.r.X, n.r.Y), simmod.ROBOT_RADIUS)

    def test_end_heading(self):
        n = NavRun(seed=6)
        n.r.set_param(n.r.pidx["nav.end_heading_deg"], 90.0)
        n.frame(lp.encode_goto([(500.0, 0.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(40.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED))
        _, _, th = n.r.odom()
        self.assertLess(abs(simmod._wrap180(th - 90.0)), 6.0)

    def test_obstacle_stops_and_resumes_when_clear(self):
        # second leg runs along y = 900 straight into the box (x 1200..1800, y 700..1100)
        n = NavRun(seed=7)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED), n.logs[-5:])
        n.run(1.5)
        x0 = n.r.X
        n.run(3.0)
        r = n.r
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        self.assertLess(abs(r.speed_f), 80.0)                          # (balancing sway)
        self.assertLess(abs(r.X - x0), 80.0)                           # stays put
        self.assertLess(r.X, 1200.0 - simmod.ROBOT_RADIUS)            # stopped short of the box
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        blocked = [m for m in n.navs if m["state"] == lp.NAV_BLOCKED]
        self.assertTrue(blocked)
        self.assertGreater(blocked[-1]["obstacle_mm"], 150.0)
        self.assertLess(blocked[-1]["obstacle_mm"], r.P("rng.obstacle_mm") + r.P("nav.clear_mm"))
        self.assertTrue(n.tlm[-1]["status"] & lp.TLM_OBSTACLE)
        self.assertTrue(any(l.startswith("nav: blocked") for l in n.logs))
        # GOTO waits for the way to clear: take the box away
        box = simmod._box(1500, 900, 600, 400)
        r.room.segs = [s for s in r.room.segs if s not in box]
        self.assertTrue(n.run(20.0, lambda s: s.r.nav.state == lp.NAV_DRIVING))
        self.assertTrue(any(l.startswith("nav: clear") for l in n.logs))

    def test_backoff(self):
        n = NavRun(seed=8)
        n.r.set_param(n.r.pidx["nav.backoff_mm"], 120.0)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED))
        x0 = n.r.X
        n.run(6.0)
        self.assertEqual(n.r.nav.state, lp.NAV_BLOCKED)    # backed off, still blocked (no oscillation)
        self.assertLess(n.r.X, x0 - 60.0)


class TestExplore(unittest.TestCase):
    def test_blocked_waits_for_new_list(self):
        n = NavRun(seed=9)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_EXPLORE)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED))
        box = simmod._box(1500, 900, 600, 400)            # even when the way clears ...
        n.r.room.segs = [s for s in n.r.room.segs if s not in box]
        n.run(4.0)
        self.assertEqual(n.r.nav.state, lp.NAV_BLOCKED)    # ... EXPLORE waits for the planner
        n.frame(lp.encode_goto([(600.0, 300.0)]))          # new list (replace) -> resumes
        self.assertTrue(n.run(40.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED))
        x, y = n.pos()
        self.assertLess(math.hypot(x - 600.0, y - 300.0), 100.0)


class TestHold(unittest.TestCase):
    def test_comes_back_after_a_push(self):
        n = NavRun(seed=10)
        n.stand_up(lp.MODE_HOLD)
        n.run(2.0)
        hx, hy = n.r.nav.hold
        self.assertEqual(n.r.nav.state, lp.NAV_HOLDING)
        n.r.X += 250.0                                    # shoved 25 cm forward
        n.run(8.0)
        x, y = n.pos()
        self.assertLess(math.hypot(x - hx, y - hy), 60.0)


class TestStation(unittest.TestCase):
    def test_cmd_vel_obstacle_stop(self):
        n = NavRun(seed=11)
        n.r.TH = -90.0                                    # facing the wall at y = -1500
        n.stand_up(lp.MODE_STATION)
        cmd = lp.encode_cmd_vel(200.0, 0.0)               # the station keeps driving forward
        end = n.r.t + 15.0
        while n.r.t + n.r.dt <= end:
            n.frame(cmd)
            n.run(0.1)
        r = n.r
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        self.assertGreater(r.Y, -1500.0 + simmod.ROBOT_RADIUS)
        self.assertLess(r.Y, -800.0)                      # it did drive
        self.assertLess(abs(r.speed_f), 40.0)
        self.assertTrue(n.tlm[-1]["status"] & lp.TLM_OBSTACLE)


if __name__ == "__main__":
    unittest.main()
