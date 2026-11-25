"""Contact detection (issue #104) in the simulator: low boxes the lidar and the range sweep look over, which stop the body
(wheel stall + a pitch kick on impact). The navigator (port of src/navigator.cpp + src/contact.cpp) must find them by
touch, back off and report them; the bridge's planner replans round the bump mark; normal driving never trips it.
Run: python -m unittest discover -s station -t ."""
import math
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.sim import contact as cm
from station.sim import tour
from station.sim.sim import SimRobot

LIVE = {"nav.max_speed": 250.0, "rng.obstacle_mm": 160.0, "nav.slow_mm": 450.0, "nav.arrive_mm": 120.0,
        "nav.dwell_ms": 100.0, "nav.stop_speed": 90.0, "nav.stop_turn_dps": 45.0, "nav.align_deg": 20.0}
BOX = (1300.0, 0.0, 200.0, 300.0, 60.0)    # low box: its face at x 1200, 60 mm high


def logs(r):
    out = [lp.decode_log(f[4:-2]) for f in r.out if f[2] == lp.MSG_LOG]
    r.out = []
    return out


class Drive:
    """A SimRobot standing up, then GOTO through its own waypoint list (no bridge), heartbeats from the Pi."""

    def __init__(self, seed=1, params=None, box=BOX, wiggle=None):
        self.r = r = SimRobot(seed=seed)
        if box:
            r.room.add_low(*box)
        if wiggle:
            r.wiggle = (wiggle, 2.0)
        for k, v in (params or {}).items():
            r.set_param(r.pidx[k], v)
        r.set_mode(lp.MODE_BALANCE, arm=True)
        self.logs = []
        self.run(3.0)

    def goto(self, points, mode=lp.MODE_GOTO):
        tour.set_list(self.r, points)
        self.r.set_mode(mode)

    def run(self, seconds, until=None):
        r = self.r
        end = r.t + seconds
        while r.t < end:
            if r.step_n % 40 == 0:
                tour.heartbeat(r)
            r.step()
            if r.step_n % 40 == 0:
                self.logs += logs(r)
                if until and until(self):
                    return True
        self.logs += logs(r)
        return False


class TestDetect(unittest.TestCase):
    def test_bump_at_cruise_speed(self):
        d = Drive(params=LIVE)
        d.goto([(2500.0, 0.0)])
        self.assertTrue(d.run(20.0, lambda s: s.r.contacts))
        r = d.r
        c = r.contacts[0]
        t_hit = r.impacts[0][0]
        self.assertGreater(r.impacts[0][1], 200.0)                       # it really hit it at speed
        self.assertEqual(c["kind"], lp.LINK_CONTACT_BUMP, d.logs[-3:])
        self.assertGreaterEqual(c["value"], cm.BUMP_RATE)
        self.assertLess(r.t - t_hit, 0.25)                               # detected within ~0.15 s
        self.assertAlmostEqual(c["x_mm"], 1200.0 - 90.0 + 50.0, delta=30.0)   # on the body's front edge, at the box
        self.assertAlmostEqual(c["y_mm"], 0.0, delta=40.0)
        self.assertTrue(any(l.startswith("nav: contact bump at") for l in d.logs), d.logs[-5:])
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        d.run(0.6)
        self.assertLess(r.nav.speed, 0.0)                                # backing off (rear corridor checked)
        x0 = r.X
        d.run(3.0)
        self.assertGreater(1200.0 - 90.0 - r.X, 60.0)                    # backed off ~nav.stuck_backoff_mm
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)                    # GOTO waits for a new list ...
        self.assertTrue(r.nav.contact)
        self.assertLess(abs(r.X - x0), 120.0)

    def test_stuck_when_too_slow_for_a_bump(self):
        for speed in (150.0, 110.0):
            d = Drive(params=dict(LIVE, **{"nav.max_speed": speed}))
            d.goto([(2500.0, 0.0)])
            self.assertTrue(d.run(25.0, lambda s: s.r.contacts), speed)
            r = d.r
            c = r.contacts[0]
            t_hit = r.impacts[0][0]
            self.assertEqual(c["kind"], lp.LINK_CONTACT_STUCK, (speed, d.logs[-3:]))
            self.assertLess(r.t - t_hit, 0.6, speed)                      # the 0.3-0.5 s target (vs 2.5 s before)
            self.assertAlmostEqual(c["x_mm"], 1160.0, delta=40.0)
            self.assertTrue(any(l.startswith("nav: contact stuck at") for l in d.logs))

    def test_explore_waits_for_a_new_list(self):
        d = Drive(params=LIVE)
        d.goto([(2500.0, 0.0)], lp.MODE_EXPLORE)
        self.assertTrue(d.run(20.0, lambda s: s.r.contacts))
        d.run(6.0)
        self.assertEqual(d.r.nav.state, lp.NAV_BLOCKED)                  # (GOTO would have tried again by now)
        self.assertEqual(len(d.r.contacts), 1)

    def test_station_reported_only(self):
        d = Drive(params=LIVE)
        r = d.r
        r.set_mode(lp.MODE_STATION)
        end = r.t + 10.0
        while r.t < end:                                                 # the joystick: 250 mm/s straight on
            r.on_frame(lp.MSG_CMD_VEL, lp.encode_cmd_vel(250.0, 0.0)[4:-2])
            d.run(0.05)
        self.assertTrue(r.contacts)
        self.assertTrue(all(c["flags"] & lp.LINK_CONTACT_REPORT for c in r.contacts))
        self.assertTrue(all(c["mode"] == lp.MODE_STATION for c in r.contacts))
        self.assertLessEqual(len(r.contacts), 4)                          # at most one per 3 s
        self.assertTrue(any("reported only" in l for l in d.logs))
        self.assertNotEqual(r.nav.state, lp.NAV_BLOCKED)

    def test_hold_pushed_into_it_never_marks(self):
        for side in (1.0, -1.0):
            d = Drive(params=LIVE, box=None)
            r = d.r
            r.set_mode(lp.MODE_HOLD)
            d.run(1.0)
            hx = r.X                                                     # the hold point (heading ~0)
            r.room.add_low(hx + side * 200.0, r.Y, 100.0, 300.0)         # a low box between it and ...
            r.X = hx + side * 450.0                                      # ... where it is carried: it drives back
            d.run(8.0)                                                   # into the box and keeps pushing
            self.assertTrue(r.impacts, side)
            self.assertEqual(r.contacts, [], side)                       # HOLD: never a mark

    def test_off_with_stuck_ms_0(self):
        d = Drive(params=dict(LIVE, **{"nav.stuck_ms": 0.0}))
        d.goto([(2500.0, 0.0)])
        d.run(15.0)
        self.assertTrue(d.r.impacts)
        self.assertEqual(d.r.contacts, [])


class TestNoFalsePositives(unittest.TestCase):
    """Starts, stops, turns in place, slow-downs at waypoints: never a contact (several seeds; and with a 2 Hz pitch
    wiggle like the real robot's, ~18 deg/s RMS, with its live nav settings)."""

    def tour_run(self, seed, params=None, wiggle=None, seconds=150.0):
        d = Drive(seed=seed, params=params, box=None, wiggle=wiggle)
        d.goto(tour.TOUR)
        d.run(seconds, lambda s: s.r.nav.state == lp.NAV_ARRIVED and s.r.nav.idx >= len(tour.TOUR))
        return d

    def test_tours(self):
        for seed, params, wiggle in ((1, None, None), (2, None, None), (3, LIVE, None), (4, LIVE, 15.0)):
            d = self.tour_run(seed, params, wiggle)
            self.assertEqual(d.r.nav.state, lp.NAV_ARRIVED, (seed, d.logs[-4:]))
            self.assertEqual(d.r.contacts, [], (seed, [l for l in d.logs if "contact" in l]))

    def test_explore(self):
        tmp = tempfile.mkdtemp()
        try:
            for seed in (5, 6):
                r = SimRobot(seed=seed)
                for k, v in LIVE.items():
                    r.set_param(r.pidx[k], v)
                secs, b = tour.plan_offline(r, Mapper(tmp + "/%d" % seed), {"t": "plan", "action": "explore"}, max_s=90.0)
                self.assertGreater(secs, 20.0)
                self.assertEqual(r.contacts, [], (seed, [l for l in b.robot_logs if "contact" in l]))
                self.assertEqual(b.contacts, [])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestReplan(unittest.TestCase):
    def test_marked_and_avoided(self):
        """GOTO through the planner straight at a low box: bump, mark, replan round it, arrive - without a second hit."""
        tmp = tempfile.mkdtemp()
        try:
            r = SimRobot(seed=1)
            for k, v in LIVE.items():
                r.set_param(r.pidx[k], v)
            r.room.add_low(*BOX)
            m = Mapper(tmp)
            track = []
            secs, b = tour.plan_offline(r, m, {"t": "plan", "action": "goto", "points": [[2500, 0]], "go": True},
                                        max_s=90.0, on_step=lambda rr, bb: track.append((rr.t, rr.X, rr.Y)))
            self.assertEqual(b.planner.status, "arrived", b.lines)
            self.assertEqual(len(b.contacts), 1, b.lines)                 # one contact, then the route went round
            marks = m.zones.bumps()
            self.assertEqual(len(marks), 1)
            mk = marks[0]
            self.assertAlmostEqual(mk["x"], 1200.0, delta=40.0)          # at the box's face
            self.assertAlmostEqual(mk["y"], 0.0, delta=40.0)
            self.assertTrue(any(x > 1500.0 for t, x, y in track))        # it got past the box ...
            self.assertTrue(any(abs(y) > 150.0 for t, x, y in track if 1100.0 < x < 1500.0))   # ... round it
            self.assertTrue(any(l.startswith("planner: robot touched something") for l in b.lines))
            x, y, _ = r.odom()
            self.assertLess(math.hypot(x - 2500.0, y), 200.0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
