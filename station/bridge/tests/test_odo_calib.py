"""Calibrate odometry (issue #14): the maths, the state machine against a fake robot, and the whole procedure
against the simulator with a deliberately wrong wheel diameter / track / left-right ratio (the tool must recover
the true values from what the robot reports and what 'the tape measure' says).
Run: python -m unittest discover -s station -t ."""
import json
import math
import os
import unittest

from station.bridge import link_protocol as lp
from station.bridge import odo_calib as oc
from station.bridge.odo_calib import OdoCalib
from station.sim import sim as simmod
from station.sim.sim import SimRobot
from station.sim.tests.test_nav import HB, NavRun, payload

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


# ---------------------------------------------------------------------------------- the maths
class TestMaths(unittest.TestCase):
    def test_distance_scale(self):
        self.assertAlmostEqual(oc.distance_scale(2000.0, 2090.0), 1.045)
        with self.assertRaises(ValueError):
            oc.distance_scale(0.0, 2000.0)

    def test_route_and_local_frame(self):
        pts = oc.route("cw", 1000.0, 500.0, 200.0, 90.0)      # facing +y: ahead = +y, right = +x
        self.assertEqual(pts, [(500.0, 1200.0), (1500.0, 1200.0), (1500.0, 200.0), (500.0, 200.0)])
        pts = oc.route("ccw", 1000.0, 0.0, 0.0, 0.0)
        self.assertEqual(pts, [(1000.0, 0.0), (1000.0, 1000.0), (0.0, 1000.0), (0.0, 0.0)])
        self.assertEqual(oc.route("straight", 2000.0, 0.0, 0.0, 180.0)[0][0], -2000.0)
        x, y = oc.to_local(0.0, 100.0, 90.0)                  # 10 cm north while facing north = ahead
        self.assertAlmostEqual(x, 100.0)
        self.assertAlmostEqual(y, 0.0)
        x, y = oc.to_local(100.0, 0.0, 90.0)                  # east while facing north = to the right
        self.assertAlmostEqual(y, -100.0)

    def test_fit_track_and_lr(self):
        T, eps = 131.0, 0.012
        runs = []
        for psi, S in ((0.0, 2000.0), (-1.5 * math.pi, 4000.0), (1.5 * math.pi, 4000.0)):
            runs.append({"X": T * psi + eps * S, "S": S, "psi": psi})
        t, e = oc.fit_track_lr(runs)
        self.assertAlmostEqual(t, T, places=6)
        self.assertAlmostEqual(e, eps, places=9)
        t, e = oc.fit_track_lr(runs[:1])                       # only a straight run: eps only
        self.assertIsNone(t)
        self.assertAlmostEqual(e, eps, places=9)
        t, e = oc.fit_track_lr(runs[1:2])                      # one direction cannot split track from eps
        self.assertIsNone(t)
        self.assertIsNone(e)
        self.assertEqual(oc.fit_track_lr([]), (None, None))

    def test_lr_from_eps(self):
        self.assertAlmostEqual(oc.lr_from_eps(1.0, 0.0), 1.0)
        self.assertLess(oc.lr_from_eps(1.0, 0.02), 1.0)        # right wheel over-reads: its diameter is smaller

    def test_drift_per_min(self):
        ts = [i * 0.04 for i in range(1500)]
        g = [0.05 * t for t in ts]                            # 3 deg/min
        w = [0.0 for _ in ts]
        d, gp, wp = oc.drift_per_min(ts, g, w)
        self.assertAlmostEqual(d, 3.0, places=6)
        self.assertAlmostEqual(gp, 3.0, places=6)
        d, _, _ = oc.drift_per_min(ts, g, [0.05 * t for t in ts])   # the robot counter-steered: wheels turned too
        self.assertAlmostEqual(d, 0.0, places=6)
        self.assertEqual(oc.verdict(0.2), "excellent")
        self.assertEqual(oc.verdict(-6.0), "poor")

    @staticmethod
    def umb_run(cw, b_nom, e_b, e_d, side, n=2000):
        """A differential robot that dead-reckons its heading from the wheels (the UMBmark assumption)."""
        dl = 2.0 / (e_d + 1.0)
        dr = e_d * dl
        b = e_b * b_nom
        x = y = th = 0.0
        sgn = -1.0 if cw else 1.0

        def move(cl, cr):
            nonlocal x, y, th
            sl, sr = cl * dl, cr * dr
            for _ in range(n):
                dth = (sr - sl) / b / n
                d = (sr + sl) / 2.0 / n
                x += d * math.cos(th + dth / 2)
                y += d * math.sin(th + dth / 2)
                th += dth
        for _ in range(4):
            move(side, side)
            a = sgn * math.pi / 2.0 * b_nom / 2.0
            move(-a, a)
        return x, y

    def test_umbmark_recovers_track_and_diameter_ratio(self):
        for e_b, e_d in ((1.02, 1.0), (1.0, 1.01), (0.98, 0.99)):
            cw = self.umb_run(True, 126.8, e_b, e_d, 1000.0)
            ccw = self.umb_run(False, 126.8, e_b, e_d, 1000.0)
            u = oc.umbmark(cw, ccw, 1000.0, 126.8)
            self.assertAlmostEqual(u["E_b"], e_b, delta=0.008, msg=(e_b, e_d))     # first-order formulas
            self.assertAlmostEqual(u["E_d"], e_d, delta=0.004, msg=(e_b, e_d))
            self.assertAlmostEqual(u["track_mm"], e_b * 126.8, delta=1.0)

    def test_umbmark_perfect_robot(self):
        u = oc.umbmark((0.0, 0.0), (0.0, 0.0), 1000.0, 126.8)
        self.assertAlmostEqual(u["E_b"], 1.0)
        self.assertAlmostEqual(u["E_d"], 1.0)
        self.assertIsNone(u["R_mm"])


# ---------------------------------------------------------------------------------- fake bridge on the sim
class SimBridge:
    """What OdoCalib uses of the bridge, wired to a SimRobot (no network): commands become link frames."""

    def __init__(self, nav):
        self.n = nav
        self.r = nav.r
        self.link_state, self.estop, self.cmd_active = "connected", False, False
        self.tlm, self.tlm_time = None, 0.0
        self.said, self.sent, self.cmds = [], [], []
        self.seen = 0

    @property
    def params(self):
        return {i: {"id": i, "name": p[0], "value": p[1], "min": p[2], "max": p[3]}
                for i, p in enumerate(self.r.params)}

    def say(self, text):
        self.said.append(text)

    def broadcast(self, text, droppable=False):
        self.sent.append(json.loads(text))

    def handle_command(self, m, who="?"):
        self.cmds.append(m)
        t = m["t"]
        if t == "goto":
            for f in lp.goto_frames([tuple(p) for p in m["points"]], m.get("replace", True)):
                self.n.frame(f)
            if m.get("go"):
                self.n.frame(lp.encode_set_mode(lp.MODE_GOTO))
        elif t == "set_mode":
            self.n.frame(lp.encode_set_mode(m["mode"]))
        elif t == "event":
            self.n.frame(lp.encode_event(m["event"]))
        elif t == "set_param":
            self.n.frame(lp.encode_set_param(m["id"], m["value"]))
        return None


class CalibRig:
    def __init__(self, true_d=68.9, true_track=130.0, true_lr=1.0, bias=0.0, seed=3):
        self.nav = NavRun(seed=seed)
        self.r = self.nav.r
        self.r.set_hardware(true_d, true_track, true_lr, bias)
        self.b = SimBridge(self.nav)
        self.c = OdoCalib(self.b)
        self.nav.stand_up(lp.MODE_BALANCE)
        self.nav.run(2.0)
        self.pump(0.2)

    def pump(self, seconds, until=None):
        """Step the sim and feed every telemetry frame to the tool; True when `until()` held."""
        end = self.r.t + seconds
        while self.r.t < end:
            self.nav.run(0.1)
            for d in self.nav.tlm[self.b.seen:]:
                now = d["t_us"] / 1e6
                self.b.tlm, self.b.tlm_time = d, self.r.t
                self.c.on_tlm(d, now)
            self.b.seen = len(self.nav.tlm)
            self.c.tick(self.r.t)
            if until and until():
                return True
        return False

    def run_test(self, test, **kw):
        self.c.command(dict(kw, action="start", test=test), "test", now=self.r.t)
        ok = self.pump(300.0, lambda: not self.c.active())
        self.assert_ok(ok, "run did not end")
        state = self.c.state
        self.pump(1.5)                       # back to BALANCE, standing still (the owner takes a while to measure)
        return state

    @staticmethod
    def assert_ok(v, text=""):
        if not v:
            raise AssertionError(text)

    def truth_offset(self, x0, y0, th0):
        """What a tape measure shows: the robot's true position vs where it started, in the start frame."""
        return oc.to_local(self.r.X - x0, self.r.Y - y0, th0)

    def start_truth(self):
        return self.r.X, self.r.Y, self.r.TH


class TestSimHardware(unittest.TestCase):
    def test_default_sim_unchanged(self):
        r = SimRobot(seed=1)
        self.assertIsNone(r.hw)
        self.assertEqual(r.wheel_scale(), (1.0, 1.0))

    def test_wrong_params_scale_the_pose(self):
        r = SimRobot(seed=1)
        r.set_hardware(wheel_d=69.3, track=126.8, lr=1.0)    # real wheels 5 % bigger than odo.wheel_d_mm 66
        sl, sr = r.wheel_scale()
        self.assertAlmostEqual(sl, 66.0 / 69.3)
        self.assertAlmostEqual(sr, 66.0 / 69.3)


class TestRunsInSim(unittest.TestCase):
    def test_straight_then_squares_recover_the_true_wheels(self):
        rig = CalibRig(true_d=69.3, true_track=131.0, true_lr=1.015)
        c = rig.c
        self.assertIsNone(c.why_not("straight", rig.r.t))
        slow = c._val("nav.max_speed")

        # --- straight run, 0.8 m
        st0 = rig.start_truth()
        n0 = len(rig.nav.tlm)
        self.assertEqual(rig.run_test("straight", length_mm=800.0), "measure")
        self.assertEqual(c._val("nav.max_speed"), slow)                        # speed limit put back
        self.assertLessEqual(max(abs(d["speed_mms"]) for d in rig.nav.tlm[n0:]), oc.SPEED_LIMIT_MMS + 40.0)
        fwd, side = rig.truth_offset(*st0)
        rep = c.data["straight"]["reported_mm"]
        self.assertAlmostEqual(rep / 800.0, 1.0, delta=0.06)                  # it thinks it drove ~0.8 m
        self.assertGreater(fwd, rep * 1.03)                                    # but it drove ~5 % further
        c.command({"t": "calib", "action": "measured", "test": "straight", "forward_cm": fwd / 10.0,
                   "side_cm": side / 10.0}, "test", now=rig.r.t)
        self.assertEqual(c.state, "idle")
        rows = {r["name"]: r for r in c.proposal()["rows"]}
        self.assertAlmostEqual(rows["odo.wheel_d_mm"]["to"], 69.3, delta=0.35)

        # --- both squares (0.5 m, small: the sim room is tight)
        offsets = {}
        for test in ("cw", "ccw"):
            st = rig.start_truth()
            self.assertEqual(rig.run_test(test, length_mm=500.0), "measure")
            x, y = rig.truth_offset(*st)
            offsets[test] = (x, y)
            c.command({"t": "calib", "action": "measured", "test": test, "x_cm": x / 10.0, "y_cm": y / 10.0},
                      "test", now=rig.r.t)
            self.assertLess(math.hypot(x, y), 150.0)                           # closes: the gyro keeps it square
            self.assertEqual(c.data[test]["length_mm"], 500.0)
        prop = c.proposal()
        rows = {r["name"]: r for r in prop["rows"]}
        self.assertAlmostEqual(rows["odo.wheel_d_mm"]["to"], 69.3, delta=0.35)
        self.assertAlmostEqual(rows["odo.track_mm"]["to"], 131.0, delta=3.0)
        self.assertAlmostEqual(rows["odo.lr_ratio"]["to"], 1.015, delta=0.006)
        self.assertEqual(set(prop["closing_cm"]), {"cw", "ccw"})
        self.assertIsNotNone(prop["umbmark"])

        # --- apply, undo, apply again
        c.command({"t": "calib", "action": "apply"}, "test", now=rig.r.t)
        self.assertAlmostEqual(c._val("odo.wheel_d_mm"), rows["odo.wheel_d_mm"]["to"], places=2)
        self.assertTrue(c.msg(rig.r.t)["changed"])
        c.command({"t": "calib", "action": "revert"}, "test", now=rig.r.t)
        self.assertAlmostEqual(c._val("odo.wheel_d_mm"), 66.0, places=3)
        self.assertAlmostEqual(c._val("odo.track_mm"), 126.8, places=3)
        self.assertFalse(c.msg(rig.r.t)["changed"])

    def test_calibrated_odometry_is_right(self):
        """With the true values in the params the straight run reads the tape distance."""
        rig = CalibRig(true_d=69.3, true_track=131.0, true_lr=1.015)
        for name, v in (("odo.wheel_d_mm", 69.3), ("odo.track_mm", 131.0), ("odo.lr_ratio", 1.015)):
            rig.b.handle_command({"t": "set_param", "id": rig.r.pidx[name], "value": v})
        rig.nav.run(0.5)
        st = rig.start_truth()
        rig.run_test("straight", length_mm=800.0)
        fwd, _ = rig.truth_offset(*st)
        self.assertAlmostEqual(rig.c.data["straight"]["reported_mm"], fwd, delta=fwd * 0.02)
        sl, sr = rig.c.data["straight"]["Sl"], rig.c.data["straight"]["Sr"]
        self.assertAlmostEqual(sr / sl, 1.0, delta=0.01)

    def test_gyro_drift_at_rest(self):
        rig = CalibRig(true_d=66.0, true_track=126.8, bias=0.1)              # 6 deg/min
        rig.c.command({"t": "calib", "action": "start", "test": "gyro", "secs": 40}, "test", now=rig.r.t)
        rig.assert_ok(rig.pump(80.0, lambda: not rig.c.active()), "gyro run did not end")
        g = rig.c.data["gyro"]
        self.assertAlmostEqual(g["drift_dpm"], 6.0, delta=0.8)
        self.assertEqual(oc.verdict(g["drift_dpm"]), "poor")
        self.assertEqual(rig.c.state, "idle")

    def test_stop_and_disarm_end_the_run(self):
        rig = CalibRig()
        rig.c.command({"t": "calib", "action": "start", "test": "straight", "length_mm": 1500}, "test", now=rig.r.t)
        rig.pump(6.0)
        self.assertTrue(rig.c.active())
        self.assertEqual(rig.c._val("nav.max_speed"), 120.0)
        rig.c.command({"t": "calib", "action": "stop"}, "test", now=rig.r.t)
        self.assertEqual(rig.c.state, "aborted")
        self.assertEqual(rig.c._val("nav.max_speed"), 250.0)                   # restored
        rig.pump(3.0)
        self.assertEqual(rig.r.mode, lp.MODE_BALANCE)                         # back to standing, still armed
        self.assertTrue(rig.r.armed)
        # a second run, ended by disarming: the tool must not re-engage the robot
        rig.c.command({"t": "calib", "action": "start", "test": "straight", "length_mm": 1500}, "test", now=rig.r.t)
        rig.pump(4.0)
        rig.nav.frame(lp.encode_set_mode(lp.MODE_IDLE))
        rig.pump(2.0)
        self.assertEqual(rig.c.state, "aborted")
        self.assertEqual(rig.r.mode, lp.MODE_IDLE)
        self.assertFalse(rig.r.armed)


# ---------------------------------------------------------------------------------- the state machine alone
class FakeBridge:
    def __init__(self, mode=lp.MODE_BALANCE, armed=True, with_params=True):
        self.params = {}
        if with_params:
            for i, (n, v, lo, hi) in enumerate([("odo.wheel_d_mm", 66.0, 40, 100), ("odo.track_mm", 126.8, 80, 200),
                                                ("odo.lr_ratio", 1.0, 0.9, 1.1), ("nav.max_speed", 250.0, 0, 1500)]):
                self.params[i] = {"id": i, "name": n, "value": v, "min": lo, "max": hi}
        self.link_state, self.estop, self.cmd_active = "connected", False, False
        self.tlm = {"status": (lp.TLM_ARMED if armed else 0) | lp.TLM_UPRIGHT, "mode": mode, "speed_mms": 0.0,
                    "x_mm": 0.0, "y_mm": 0.0, "heading_deg": 0.0, "nav_state": 0, "wheel_left_mms": 0.0,
                    "wheel_right_mms": 0.0, "turn_rate_dps": 0.0}
        self.tlm_time = 0.0
        self.said, self.sent, self.cmds = [], [], []

    def say(self, text):
        self.said.append(text)

    def broadcast(self, text, droppable=False):
        self.sent.append(json.loads(text))

    def handle_command(self, m, who="?"):
        self.cmds.append(m)
        if m["t"] == "set_param":
            self.params[m["id"]]["value"] = m["value"]


class TestStateMachine(unittest.TestCase):
    def test_refuses_when_not_ready(self):
        for kw, text in (({"armed": False}, "Arm"), ({"mode": lp.MODE_IDLE}, "BALANCE"), ({"with_params": False}, "odo")):
            fb = FakeBridge(**kw)
            c = OdoCalib(fb)
            with self.assertRaises(ValueError) as cm:
                c.command({"action": "start", "test": "straight"}, "t", now=0.1)
            self.assertIn(text, str(cm.exception))
            self.assertEqual(fb.cmds, [])                                        # nothing was sent
        fb = FakeBridge()
        fb.estop = True
        with self.assertRaises(ValueError):
            OdoCalib(fb).command({"action": "start", "test": "cw"}, "t", now=0.1)
        fb = FakeBridge()
        fb.tlm["speed_mms"] = 200.0
        with self.assertRaises(ValueError):
            OdoCalib(fb).command({"action": "start", "test": "cw"}, "t", now=0.1)
        fb = FakeBridge(mode=lp.MODE_STATION)
        with self.assertRaises(ValueError):                                      # gyro needs BALANCE / HOLD
            OdoCalib(fb).command({"action": "start", "test": "gyro"}, "t", now=0.1)

    def test_start_sends_a_slow_route_and_never_arms(self):
        fb = FakeBridge()
        c = OdoCalib(fb)
        fb.tlm.update(x_mm=100.0, y_mm=50.0, heading_deg=90.0)
        c.command({"action": "start", "test": "straight", "length_mm": 1000}, "t", now=0.1)
        goto = [m for m in fb.cmds if m["t"] == "goto"][0]
        self.assertTrue(goto["go"])
        self.assertNotIn("arm", goto)
        self.assertEqual(goto["points"], [(100.0, 1050.0)])
        self.assertEqual(fb.params[3]["value"], 120.0)                           # <= 150 mm/s
        with self.assertRaises(ValueError):
            c.command({"action": "start", "test": "cw"}, "t", now=0.2)           # already running
        with self.assertRaises(ValueError):
            c.command({"action": "start", "test": "straight", "length_mm": 99999}, "t", now=0.2)

    def test_aborts_restore_the_speed_limit(self):
        for what in ("disarm", "fall", "estop", "speed", "lost", "stop", "mode"):
            fb = FakeBridge()
            c = OdoCalib(fb)
            c.command({"action": "start", "test": "cw"}, "t", now=0.0)
            d = dict(fb.tlm, mode=lp.MODE_GOTO)
            fb.tlm = d
            fb.tlm_time = 0.5
            c.on_tlm(d, 0.5)
            self.assertTrue(c.active())
            n_before = len(fb.cmds)
            if what == "disarm":
                d = dict(d, status=lp.TLM_UPRIGHT)
            elif what == "fall":
                d = dict(d, status=lp.TLM_ARMED)
            elif what == "estop":
                fb.estop = True
            elif what == "speed":
                d = dict(d, speed_mms=400.0)
            elif what == "mode":
                d = dict(d, mode=lp.MODE_BALANCE)
            if what == "lost":
                c.tick(5.0)
            elif what == "stop":
                c.command({"action": "stop"}, "t", now=4.0)
            else:
                c.on_tlm(d, 4.0)
            self.assertEqual(c.state, "aborted", what)
            self.assertEqual(fb.params[3]["value"], 250.0, what)
            sent = fb.cmds[n_before:]
            self.assertTrue(any(m.get("event") == lp.EVT_CLEAR_WAYPOINTS for m in sent) or what == "mode", what)
            if what in ("disarm", "estop", "fall"):
                self.assertFalse(any(m["t"] == "set_mode" for m in sent), what)  # never re-engages a stopped robot

    def test_measured_needs_a_run_and_sane_numbers(self):
        c = OdoCalib(FakeBridge())
        with self.assertRaises(ValueError):
            c.command({"action": "measured", "test": "straight", "forward_cm": 200}, "t")
        c.data["straight"] = {"reported_mm": 2000.0, "end_local": (2000.0, 0.0), "psi": 0.0, "Sl": 2000.0,
                              "Sr": 2000.0, "length_mm": 2000.0}
        with self.assertRaises(ValueError):
            c.command({"action": "measured", "test": "straight", "forward_cm": "abc"}, "t")
        with self.assertRaises(ValueError):
            c.command({"action": "measured", "test": "straight", "forward_cm": 5}, "t")
        c.command({"action": "measured", "test": "straight", "forward_cm": 209.0, "side_cm": 0}, "t")
        row = c.proposal()["rows"][0]
        self.assertEqual(row["name"], "odo.wheel_d_mm")
        self.assertAlmostEqual(row["to"], 66.0 * 209.0 / 200.0, places=3)
        c.command({"action": "apply"}, "t")
        self.assertAlmostEqual(c._val("odo.wheel_d_mm"), 68.97, places=2)
        c.command({"action": "revert"}, "t")
        self.assertEqual(c._val("odo.wheel_d_mm"), 66.0)
        with self.assertRaises(ValueError):
            c.command({"action": "apply"}, "t")                                  # nothing left to apply

    def test_state_message_is_json(self):
        fb = FakeBridge()
        c = OdoCalib(fb)
        m = c.msg(0.1)
        self.assertEqual(m["t"], "calib_state")
        self.assertEqual(set(m["tests"]), {"straight", "cw", "ccw", "gyro"})
        self.assertIsNone(m["tests"]["straight"]["ready"])
        json.dumps(m)


class TestUi(unittest.TestCase):
    def test_card_and_wiring_exist(self):
        with open(os.path.join(WEB, "index.html"), encoding="utf-8") as f:
            html = f.read()
        self.assertTrue('id="cal-card"' in html, "cal-card missing in index.html")
        with open(os.path.join(WEB, "js", "tune.js"), encoding="utf-8") as f:
            self.assertTrue("calib.js" in f.read(), "tune.js does not import calib.js")
        self.assertTrue(os.path.exists(os.path.join(WEB, "js", "calib.js")))


if __name__ == "__main__":
    unittest.main()
