"""Contact detection (issue #104) in the simulator: low boxes the lidar and the range sweep look over, which stop the body
(wheel stall + a pitch kick on impact). The navigator (port of src/navigator.cpp + src/contact.cpp) must find them by
touch, back off and report them; the bridge's planner replans round the bump mark; normal driving never trips it.
Run: python -m unittest discover -s station -t ."""
import math
import os
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.sim import contact as cm
from station.sim import sim as ctl_mod
from station.sim import tour
from station.sim.sim import SimRobot
from station.tools import contact_replay

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

    def test_slow_tours(self):
        """#111: the stall rule watches slow driving (from 60 mm/s) and back-offs: the tour at 60 and 80 mm/s."""
        for seed, speed in ((1, 60.0), (2, 80.0)):
            d = self.tour_run(seed, dict(LIVE, **{"nav.max_speed": speed}), seconds=260.0)
            self.assertEqual(d.r.nav.state, lp.NAV_ARRIVED, (seed, speed, d.logs[-4:]))
            self.assertEqual(d.r.contacts, [], (seed, speed, [l for l in d.logs if "contact" in l]))

    def test_explore(self):
        tmp = tempfile.mkdtemp()
        try:
            for seed in (5, 6):
                r = SimRobot(seed=seed)
                for k, v in LIVE.items():
                    r.set_param(r.pidx[k], v)
                secs, b = tour.plan_offline(r, Mapper(tmp + "/%d" % seed), {"t": "plan", "action": "explore"}, max_s=90.0)
                self.assertGreater(secs, 20.0)
                # no FALSE contact: each one had a real impact just before. (Seed 5, since the #103 pass-through
                # changed the explore path: a planner leg 71 mm from a thin wall's end (2300, 1100), the 90 mm body
                # clips it at 230 mm/s - a true bump, which it must report.)
                false = [t for t in r.contact_times if not any(t - 0.6 <= ti <= t for ti, v in r.impacts)]
                self.assertEqual(false, [], (seed, [l for l in b.robot_logs if "contact" in l]))
                self.assertLessEqual(len(r.contacts), 1, seed)
                self.assertEqual(len(b.contacts), len(r.contacts))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def simmod_wrap(a):
    return (a + 180.0) % 360.0 - 180.0


REPLAY = os.path.join(os.path.dirname(__file__), "data", "contact_replay_20261125.csv")
# the real robot's session of 2026-11-25 (v8): where it really was blocked while commanded to move (log time, s)
REPLAY_B = os.path.join(os.path.dirname(__file__), "data", "contact_replay_20261125b.csv")   # 18:36 (push-detect)
REAL_TRUE = [(358.20, 361.10, True), (386.55, 389.30, True), (389.30, 401.60, False), (413.40, 417.40, False),
             (436.90, 439.70, True), (546.60, 549.20, False)]     # (..., must: a push v8 missed)


class TestRealReplay(unittest.TestCase):
    """#111: the real log (its telemetry rebuilt into the detector's inputs, station/tools/contact_replay.py) through the
    Python port; test/host/contact_replay.cpp does the same with the C++ detector."""

    def test_missed_pushes_found_nothing_else(self):
        rows = contact_replay.read_csv(REPLAY)
        self.assertGreater(len(rows), 10000)
        hits = contact_replay.replay(rows)
        for t0, t1, must in REAL_TRUE:
            if must:
                self.assertTrue(any(t0 <= h[0] <= t1 and h[2] < 0 for h in hits), (t0, hits))   # found, rear
        false = [h for h in hits if not any(t0 <= h[0] <= t1 for t0, t1, _ in REAL_TRUE)]
        self.assertEqual(false, [])
        first = [next(h[0] for h in hits if t0 <= h[0] <= t1) for t0, t1, must in REAL_TRUE if must]
        print("\nreal log replay: the 3 missed pushes found at %s s (%d detections, all in blocked stretches)"
              % (", ".join("%.2f" % t for t in first), len(hits)))

    def test_slow_push_before_the_last_point(self):
        """18:36 session (push-detect on the robot): GOTO at 40 mm/s into a low box 135 mm before the last point - not
        cruising, under 60 mm/s: missed. The wheels at 0 from 132.9 s, leaning in for 9 s. Dead still: found in 1.5 s,
        and nothing anywhere else in that session."""
        rows = contact_replay.read_csv(REPLAY_B)
        self.assertGreater(len(rows), 10000)
        hits = contact_replay.replay(rows)
        self.assertTrue(hits)
        near = [h for h in hits if 132.8 <= h[0] <= 150.3]
        self.assertTrue(near and near[0][0] <= 132.9 + 1.5, hits)
        self.assertGreater(near[0][2], 0)                               # in front
        # the rest: real forward contacts in a pocket (drove in at 100-250 mm/s, stopped dead), and ~20 back-offs that
        # rocked after the bounce which the robot called "rear, holding": no rear contact at all
        ok = [(132.8, 150.3), (376.0, 420.0), (950.0, 965.0)]
        self.assertEqual([h for h in hits if not any(a <= h[0] <= b for a, b in ok)], [])
        self.assertEqual([h for h in hits if h[2] < 0], [])
        print("\nreal log replay (18:36): the slow push before the last point found at %.2f s (still from 132.9); "
              "%d front contacts, no rear one (the robot made 21 'rear, holding')" % (near[0][0], len(hits)))


class TestSlowAndBackoff(unittest.TestCase):
    """#111: low boxes met slowly (40-80 mm/s) and while backing off: found, marked, no fall, no pushing on."""

    def test_slow_push_found_marked(self):
        for speed in (40.0, 60.0, 80.0):
            tmp = tempfile.mkdtemp()
            try:
                r = SimRobot(seed=1)
                for k, v in dict(LIVE, **{"nav.max_speed": speed}).items():
                    r.set_param(r.pidx[k], v)
                m = Mapper(tmp)
                fell, at = [], {}

                def go(b, r=r, at=at):
                    # once it stands: a low box, its face 0.45 m ahead (a short approach: the sim's balancer at
                    # 40 mm/s can stand in its motor dead band in free space for seconds too)
                    at["face"], at["y"] = face, _ = r.X + 90.0 + 450.0, r.Y
                    r.room.add_low(face + 100.0, r.Y, 200.0, 300.0)
                    return b.planner.command({"t": "plan", "action": "goto", "points": [[r.X + 1800.0, r.Y]],
                                              "go": True}, "test")
                secs, b = tour.plan_offline(r, m, go, max_s=90.0,
                                            on_step=lambda rr, bb: fell.append(rr.t) if not rr.ctl.upright else None)
                face = at["face"]
                self.assertTrue(r.impacts, speed)
                # marked at the box: by a contact (80: the stall rule ~0.9 s after the first touch; 40, 60: dead still, once
                # the sim's robot stops bouncing off it, ~1.4 s), or the planner's same-spot rule after 3 stops there
                at_box = [z for z in m.zones.bumps() if abs(z["x"] - face) < 50.0 and abs(z["y"] - at["y"]) < 60.0]
                self.assertTrue(at_box, (speed, m.zones.bumps(), b.robot_logs[-6:]))
                c = r.contacts[0] if r.contacts else None
                if speed >= cm.STALL_DRIVE_MIN_CMD + 20.0:
                    self.assertEqual(c["kind"], lp.LINK_CONTACT_STUCK, speed)
                    self.assertAlmostEqual(c["x_mm"], face - 90.0 + 50.0, delta=40.0)       # at the box
                    self.assertLess(r.contact_times[0] - r.impacts[0][0], 2.0, speed)
                self.assertEqual(fell, [], speed)                       # never fell over
                how = ("contact %.1f s after the first touch" % (r.contact_times[0] - r.impacts[0][0]) if c
                       and abs(c["x_mm"] - (face - 40.0)) < 40.0 else "same-spot rule")
                print("\n  slow push %3.0f mm/s: marked at the box (%s), %d mark(s), %s" % (
                    speed, how, len(m.zones.bumps()), b.planner.status))
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    def test_push_before_the_last_point(self):
        """The 18:36 miss: GOTO to a point just past a low box (the axle stops 135 mm short of it: still driving, slowing
        for the stop, 40 mm/s). Dead still -> a contact at the box, not 'arrived' while pinned; never fell."""
        d = Drive(params=LIVE)
        r = d.r
        face = r.X + 90.0 + 400.0
        r.room.add_low(face + 100.0, r.Y, 200.0, 300.0)
        d.goto([(face - 90.0 + 135.0, r.Y)])
        self.assertTrue(d.run(40.0, lambda s: s.r.contacts or s.r.nav.state == lp.NAV_ARRIVED), d.logs[-5:])
        self.assertTrue(r.impacts)
        self.assertTrue(r.contacts, d.logs[-5:])
        c = r.contacts[0]
        self.assertEqual(c["kind"], lp.LINK_CONTACT_STUCK)
        self.assertAlmostEqual(c["x_mm"], face - 40.0, delta=40.0)
        dt = r.contact_times[0] - r.impacts[0][0]
        self.assertLess(dt, 4.0)        # (the sim's robot bounces off it ~2.3 s first; then still: 0.8 s)
        self.assertTrue(r.ctl.upright)
        print("\n  push 135 mm before the last point: contact %.2f s after the first touch" % dt)

    def test_stops_and_arrivals_no_contact(self):
        """Normal stops: GOTO to single points (short, long, slow), every arrival without a contact."""
        for seed, speed, pts in ((1, 250.0, [(600.0, 0.0)]), (2, 40.0, [(400.0, 0.0)]), (3, 80.0, [(1500.0, 300.0)]),
                                 (4, 250.0, [(150.0, 0.0)])):
            d = Drive(seed=seed, params=dict(LIVE, **{"nav.max_speed": speed}), box=None)
            d.goto(pts)
            self.assertTrue(d.run(60.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED), (seed, d.logs[-4:]))
            d.run(3.0)
            self.assertEqual(d.r.contacts, [], (seed, speed, [l for l in d.logs if "contact" in l]))

    def test_backoff_into_a_box_holds(self):
        """A bump at the box ahead, the back-off hits a low box behind: a rear contact (heading + 180), then it holds
        (no back-off into it, no drive back into the first one), upright; the velocity loop's lean was reset."""
        d = Drive(params=LIVE)
        r = d.r
        d.goto([(2500.0, 0.0)])
        self.assertTrue(d.run(25.0, lambda s: s.r.contacts), d.logs[-5:])
        r.room.add_low(r.X - 90.0 - 40.0 - 100.0, 0.0, 200.0, 300.0)       # a low box 40 mm behind it
        self.assertTrue(d.run(6.0, lambda s: len(s.r.contacts) >= 2), d.logs[-5:])
        front, rear = r.contacts[0], r.contacts[1]
        self.assertAlmostEqual(abs(front["heading_deg"]), 0.0, delta=10.0)
        self.assertAlmostEqual(abs(rear["heading_deg"]), 180.0, delta=10.0)       # behind it
        self.assertLess(rear["x_mm"], r.X)
        # (rear: 1 s settle after the back-off starts, the rocking died down, 0.8 s still)
        self.assertLess(r.contact_times[1] - r.contact_times[0], 3.5)
        self.assertTrue(any(", rear" in l and "holding" in l for l in d.logs), d.logs[-5:])
        x0 = r.X
        d.run(3.0)                                                               # (GOTO tries again after 4 s)
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        self.assertTrue(r.ctl.upright)
        self.assertLess(abs(r.X - x0), 30.0)                                     # holds there
        self.assertEqual(len(r.contacts), 2)
        self.assertLess(abs(r.ctl.vel_i - r.ctl.free_i), 0.3)                    # stood up: no lean against either

    def test_backoff_at_40_into_a_box(self):
        """BLOCKED by a box the range sweep sees, the back-off (nav.backoff_mm, 40 mm/s) into a low box behind: a rear
        contact, then it holds (the 2026-11-25 BLOCKED loop: backing off at -40 into something, again and again)."""
        from station.sim.tests.test_nav import BOX_ROUTE, NavRun
        n = NavRun(seed=8)
        r = n.r
        r.set_param(r.pidx["nav.backoff_mm"], 120.0)
        n.frame(lp.encode_goto(BOX_ROUTE))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED and s.r.nav.backing))
        h = math.radians(r.TH)
        back = 90.0 + 30.0 + 100.0                                               # its face 30 mm behind the body
        r.room.add_low(r.X - back * math.cos(h), r.Y - back * math.sin(h), 200.0, 200.0)
        # (the sim's robot rocks against the box behind for most of the back-off: a rear contact needs it still for
        # 0.8 s with the rocking died down - 18:36: rocking back-offs were ~20 false rear contacts - so here the
        # back-off's own 5 s limit may be what stops it. Either way: it stops, no pushing on, upright)
        n.run(6.0, lambda s: s.r.contacts)
        for c in r.contacts:
            self.assertEqual(c["kind"], lp.LINK_CONTACT_STUCK)
            self.assertLess(abs(abs(simmod_wrap(c["heading_deg"] - r.odom()[2])) - 180.0), 15.0)   # behind it
        n.run(2.0)
        self.assertTrue(r.ctl.upright)
        self.assertLessEqual(len(r.contacts), 1)
        self.assertFalse(r.nav.backing)
        self.assertLess(abs(r.nav.speed), 1.0)                                   # stopped: no pushing on
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)


class TestAntiWindup(unittest.TestCase):
    """#111: pushing against it the velocity loop's lean is held; driving and balancing never meet the limit."""

    def test_lean_held_when_blocked(self):
        """STATION (the joystick) pushes it into a low box at 120 mm/s for 4 s (contacts are only reported there): with the
        limit the integrator leans 1 deg past where the wheels stopped and no further."""
        for aw in (True, False):
            d = Drive(params=LIVE)
            r = d.r
            r.ctl.anti_windup = aw
            r.set_mode(lp.MODE_STATION)
            t_hit = None
            while r.t < 40.0 and (t_hit is None or r.t < t_hit + 4.0):
                r.on_frame(lp.MSG_CMD_VEL, lp.encode_cmd_vel(120.0, 0.0)[4:-2])
                d.run(0.05)
                if r.impacts and t_hit is None:
                    t_hit = r.impacts[0][0]
            lean = r.ctl.vel_i - r.ctl.blk_i                 # past the lean when the wheels stopped
            self.assertTrue(r.ctl.upright)
            if aw:
                self.assertLessEqual(lean, ctl_mod.BLK_LEAN_DEG + 0.02)
                self.assertTrue(r.ctl.held)
                held = lean
            else:
                self.assertGreater(lean, held + 1.0)    # without it the lean keeps growing
                print("\nanti-windup, 4 s pushed into a box at 120 mm/s: integrator +%.2f deg past where the wheels "
                      "stopped (without: +%.2f)" % (held, lean))

    def test_normal_driving_unchanged(self):
        """The same tour / balancing with the anti-windup off: bit-identical runs (it never engaged), same metrics."""
        out = {}
        for aw in (True, False):
            d = Drive(seed=3, params=LIVE, box=None)
            d.r.ctl.anti_windup = aw
            d.goto(tour.TOUR)
            th, sp, held = [], [], 0
            end = d.r.t + 80.0
            while d.r.t < end and not (d.r.nav.state == lp.NAV_ARRIVED and d.r.nav.idx >= len(tour.TOUR)):
                d.run(0.05)
                th.append(math.degrees(d.r.theta))
                sp.append(d.r.speed_f - d.r.ctl.speed_sp)
                held += d.r.ctl.held
            rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
            m = sum(th) / len(th)
            out[aw] = (round(d.r.t, 3), round(d.r.X, 3), round(d.r.Y, 3), rms([x - m for x in th]), rms(sp), held,
                       len(d.r.contacts))
        self.assertEqual(out[True], out[False])
        self.assertEqual(out[True][5], 0)
        print("\nbalance with / without anti-windup: tour %.1f s, pitch rms %.3f deg, speed error rms %.1f mm/s - "
              "identical" % (out[True][0], out[True][3], out[True][4]))


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
