"""Black-box fall recorder (issue #50): detection, cause tags, real fixtures from the robot, simulator, files.
Run: python -m unittest discover -s station -t ."""
import math
import os
import shutil
import tempfile
import unittest

from station.bridge import falls as F
from station.bridge import link_protocol as lp
from station.tools import falls_scan

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE = os.path.join(HERE, "data", "falls_20261121.jsonl.gz")
PARAMS = {"bal.cutoff_deg": 45.0, "bal.offset_deg": 3.15, "rng.obstacle_mm": 300.0, "ctl.rate_hz": 400.0}


def frame(t, pitch=-3.1, **kw):
    d = dict(t_us=int(t * 1e6), mode=lp.MODE_MANUAL, status=lp.TLM_ARMED | lp.TLM_UPRIGHT, error_code=0, radio_ok=1,
             pitch_deg=pitch + 0.02 * math.sin(t * 377.0), pitch_rate_dps=0.0, roll_deg=0.0, heading_deg=0.0, yaw_rate_dps=0.0, x_mm=0.0, y_mm=0.0,
             speed_mms=0.0, turn_rate_dps=0.0, cmd_speed_mms=0.0, cmd_turn_dps=0.0, wheel_left_mms=0.0,
             wheel_right_mms=0.0, balance_target_deg=-3.1, motor_left=0.05, motor_right=0.05, battery_v=12.0,
             loop_hz=400.0, bal_p=0.0, bal_i=0.0, bal_d=0.0, steer_out=0.0, vel_out_deg=0.0, step=int(t * 400),
             remote_flags=0, nav_state=0, fault_flags=0)
    d.update(kw)
    return d


def fallen(d):
    """The frame as the firmware reports it after a tip-over / cut: not upright, motors off."""
    d["status"] &= ~lp.TLM_UPRIGHT
    d["motor_left"] = d["motor_right"] = 0.0
    return d


class Harness:
    """Feeds scenario(t) -> frame dict into a recorder at `hz`, with a manual clock."""

    def __init__(self, tmp, **kw):
        self.t = 0.0
        self.rec = F.FallRecorder(tmp, params=lambda: dict(PARAMS), session=lambda: "sbr3-test.jsonl", threaded=False,
                                  clock=lambda: self.t, wall=lambda: 1.8e9 + self.t, **kw)
        self.saved = []

    def run(self, scenario, t_end, hz=200.0, t_start=0.0):
        n = int((t_end - t_start) * hz)
        for i in range(n):
            self.t = t_start + i / hz
            self.rec.on_tlm(scenario(self.t), self.t)
            self.saved += self.rec.tick(self.t)
        return self

    def finish(self, extra=10.0):
        self.t += extra
        self.saved += self.rec.tick(self.t)
        return self.saved


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def tags(self, meta):
        return [t["tag"] for t in meta["tags"]]


class TestDetection(Base):
    def test_balancing_without_falling_records_nothing(self):
        h = Harness(self.tmp).run(lambda t: frame(t, pitch=-3.1 + 2 * math.sin(t * 20)), 10)
        self.assertEqual(h.finish(), [])
        self.assertEqual(h.rec.stats()["count"], 0)

    def test_tip_over_is_saved_with_window(self):
        def sc(t):
            if t < 8.0:
                return frame(t)
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
        h = Harness(self.tmp).run(sc, 12)
        (m,) = h.finish()
        self.assertEqual(m["trigger"]["kind"], "tip")
        self.assertTrue(m["tipped"])
        self.assertTrue(F.NAME_RE.match(m["id"]))
        meta, rows, events, ranges = F.record_rows(os.path.join(self.tmp, m["id"]))
        self.assertAlmostEqual(rows[0]["t"], -5.0, delta=0.02)
        self.assertAlmostEqual(rows[-1]["t"], 2.0, delta=0.02)
        self.assertEqual(sorted(r["t"] for r in rows), [r["t"] for r in rows])
        self.assertEqual(meta["params"]["bal.cutoff_deg"], 45.0)
        self.assertEqual(meta["session"], "sbr3-test.jsonl")
        self.assertLess(m["bytes"], 200000)       # 7 s at 200 Hz
        self.assertEqual(h.rec.list()[0]["id"], m["id"])
        self.assertNotIn("params", h.rec.list()[0])

    def test_user_stop_is_not_a_fall(self):
        h = Harness(self.tmp)

        def sc(t):
            if t == 0:
                pass
            return frame(t) if t < 3.0 else fallen(frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED))
        h.rec.user_stop("pc", 2.95)
        h.run(sc, 8)
        self.assertEqual(h.finish(), [])
        self.assertEqual(h.rec.stats()["ignored"], 1)

    def test_remote_estop_and_remote_disarm_are_not_falls(self):
        h = Harness(self.tmp).run(lambda t: frame(t) if t < 3 else fallen(
            frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED, remote_flags=lp.LINK_RF_ESTOP)), 6)
        self.assertEqual(h.finish(), [])
        h2 = Harness(self.tmp).run(lambda t: frame(t) if t < 3 else frame(t, mode=lp.MODE_IDLE, status=0, motor_left=0,
                                                                         motor_right=0), 6)
        self.assertEqual(h2.finish(), [])
        self.assertEqual(h2.rec.stats()["ignored"], 1)

    def test_cut_that_did_not_tip_is_not_a_fall(self):
        # mode drops to IDLE, nothing explains it, and the robot is caught: not a fall
        h = Harness(self.tmp).run(lambda t: frame(t) if t < 3 else fallen(frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED)), 8)
        self.assertEqual(h.finish(), [])
        self.assertEqual(h.rec.stats()["ignored"], 1)

    def test_a_second_fall_inside_the_window_extends_it(self):
        def sc(t):
            if 3.0 <= t < 3.5 or t >= 5.0:
                return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
            return frame(t)
        h = Harness(self.tmp).run(sc, 12)
        saved = h.finish()
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]["trigger"]["extra"][0][1], "tip")
        # a fall long after is a new record
        h.run(lambda t: frame(t) if t < 30 else fallen(frame(t, pitch=-50, status=lp.TLM_ARMED)), 36, t_start=12)
        self.assertEqual(len(h.finish()), 2)

    def test_link_lost_while_balancing(self):
        h = Harness(self.tmp).run(lambda t: frame(t), 4)
        h.rec.on_link_lost(h.t + 0.1)
        saved = h.finish()
        self.assertEqual([m["cause"] for m in saved], ["link_lost"])

    def test_file_cap(self):
        h = Harness(self.tmp, max_files=3)
        for k in range(6):
            t0 = 20.0 * k
            h.run(lambda t, t0=t0: frame(t) if t < t0 + 6 else fallen(frame(t, pitch=-50, status=lp.TLM_ARMED)),
                  t0 + 14, t_start=t0)
            h.finish(1.0)
        self.assertEqual(h.rec.stats()["count"], 3)
        self.assertEqual(h.rec.stats()["saved"], 6)

    def test_threaded_writer(self):
        rec = F.FallRecorder(self.tmp, threaded=True, clock=lambda: self.t, wall=lambda: 1.8e9)
        self.t = 0.0
        for i in range(2000):
            self.t = i / 200.0
            rec.on_tlm(frame(self.t) if self.t < 5 else fallen(frame(self.t, pitch=-50, status=lp.TLM_ARMED)), self.t)
            rec.tick(self.t)
        rec.flush()
        done = rec.tick(self.t + 5)
        self.assertEqual(len(done), 1)
        self.assertIsNone(rec.error)


class TestTags(Base):
    def one(self, scenario, t_end=12, **kw):
        h = Harness(self.tmp, **kw).run(scenario, t_end)
        saved = h.finish()
        self.assertEqual(len(saved), 1, saved)
        return saved[0]

    def test_saturated(self):
        def sc(t):
            if t < 6.0:
                return frame(t, pitch=-3.1)
            if t < 6.5:   # leaning, motors at full power
                return frame(t, pitch=-3.1 - 40 * (t - 6.0), motor_left=-1.0, motor_right=-1.0)
            return fallen(frame(t, pitch=-60, status=lp.TLM_ARMED))
        m = self.one(sc)
        self.assertEqual(m["cause"], "saturated")
        self.assertIn("full power", m["tags"][0]["detail"])

    def test_radio_dropout_then_mode_drop(self):
        def sc(t):
            if t < 6.0:
                return frame(t)
            if t < 7.0:
                return frame(t, radio_ok=0)
            if t < 7.2:       # radio is back, the mode fell to IDLE, the robot is still upright for a moment
                return fallen(frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED, pitch=-4))
            return fallen(frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED, pitch=-4 - 100 * (t - 7.2)))
        m = self.one(sc)
        self.assertEqual(m["cause"], "radio")
        self.assertEqual(m["trigger"]["kind"], "cut")
        self.assertTrue(m["tipped"])

    def test_unexplained_cut_that_tips_is_motors_cut(self):
        def sc(t):
            if t < 6.0:
                return frame(t)
            return fallen(frame(t, mode=lp.MODE_IDLE, status=lp.TLM_ARMED, pitch=-4 - 100 * (t - 6.0)))
        self.assertEqual(self.one(sc)["cause"], "mode_drop")

    def test_battery_sag(self):
        def sc(t):
            v = 12.0 - 0.9 * min(1.0, max(0.0, (t - 4.5) / 1.5))
            if t < 6.5:
                return frame(t, battery_v=v)
            return fallen(frame(t, battery_v=v, pitch=-50, status=lp.TLM_ARMED))
        m = self.one(sc)
        self.assertEqual(m["cause"], "battery")
        self.assertGreater(m["metrics"]["battery_drop_v"], 0.6)

    def test_loop_rate_dip(self):
        def sc(t):
            if t < 6.5:
                return frame(t, loop_hz=120.0 if t > 6.0 else 400.0)
            return fallen(frame(t, loop_hz=120.0, pitch=-50, status=lp.TLM_ARMED))
        self.assertEqual(self.one(sc)["cause"], "loop")

    def test_wheel_slip(self):
        def sc(t):
            if t < 6.5:
                slip = 5.6 < t < 6.3
                return frame(t, wheel_left_mms=700.0 if slip else 20.0, wheel_right_mms=20.0)
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
        self.assertEqual(self.one(sc)["cause"], "slip")

    def test_saturation_spin_up_is_not_slip(self):
        def sc(t):
            if t < 6.0:
                return frame(t)
            if t < 6.5:
                return frame(t, motor_left=-1.0, motor_right=-1.0, wheel_left_mms=-2500 * (t - 6.0), wheel_right_mms=-2500 * (t - 6.0))
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
        tags = self.tags(self.one(sc))
        self.assertNotIn("slip", tags)
        self.assertIn("saturated", tags)

    def test_obstacle(self):
        h = Harness(self.tmp)
        for i in range(60):
            t = 5.0 + i * 0.05
            h.rec.on_range({"t_us": int(t * 1e6), "range_mm": 180.0, "sensor_id": 0, "sensor_angle_deg": 0.0,
                            "valid": 1}, t)
        h.run(lambda t: frame(t, speed_mms=400.0) if t < 7.0 else fallen(frame(t, speed_mms=400.0, pitch=-50, status=lp.TLM_ARMED)), 12)
        (m,) = h.finish()
        self.assertEqual(m["cause"], "obstacle")
        meta, rows, events, ranges = F.record_rows(os.path.join(self.tmp, m["id"]))
        self.assertTrue(ranges)

    def test_growing_oscillation(self):
        def sc(t):
            if t < 8.0:
                a = 0.4 + 1.8 * max(0.0, t - 4.0)           # grows from 0.4 to ~7 deg over 4 s
                return frame(t, pitch=-3.1 + a * math.sin(2 * math.pi * 5.0 * t), motor_left=0.3, motor_right=0.3)
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
        m = self.one(sc)
        self.assertEqual(m["cause"], "oscillation")
        self.assertGreater(m["metrics"]["osc_hz"], 4.0)

    def test_steady_chatter_is_not_oscillation(self):
        def sc(t):
            if t < 8.0:
                return frame(t, pitch=-3.1 + 2.0 * math.sin(2 * math.pi * 5.0 * t))
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED, motor_left=-1.0, motor_right=-1.0))
        self.assertNotIn("oscillation", self.tags(self.one(sc)))

    def test_push_with_motors_not_saturated(self):
        def sc(t):
            if t < 7.0:
                return frame(t)
            if t < 7.3:
                return frame(t, pitch=-3.1 + 80 * (t - 7.0), pitch_rate_dps=400.0, motor_left=0.2, motor_right=0.2)
            return fallen(frame(t, pitch=-50, status=lp.TLM_ARMED))
        self.assertEqual(self.one(sc)["cause"], "pushed")

    def test_priority_puts_upstream_causes_first(self):
        # battery sag AND saturated: the sag is the cause, saturation the symptom
        def sc(t):
            v = 12.0 - 0.9 * min(1.0, max(0.0, (t - 4.5) / 1.0))
            if t < 6.0:
                return frame(t, battery_v=v)
            if t < 6.5:
                return frame(t, battery_v=v, motor_left=1.0, motor_right=1.0)
            return fallen(frame(t, battery_v=v, pitch=-50, status=lp.TLM_ARMED))
        tags = self.tags(self.one(sc))
        self.assertEqual(tags[:2], ["battery", "saturated"])


class TestRealFalls(Base):
    """Trimmed session log of 2026-11-21 (robot on carpet, MANUAL mode from the RC remote): falls the owner saw."""

    def test_fixture(self):
        saved = falls_scan.scan(FIXTURE, self.tmp)
        rec = falls_scan.scan.recorder
        causes = [(m["cause"], m["trigger"]["kind"]) for m in saved]
        self.assertEqual(causes, [
            ("pushed", "tip"),      # 02:56:38  pitch rate spike -110 deg/s, then 0.33 s at full power, tipped past 45 deg
            ("radio", "cut"),       # 02:57:16  RC dropped out 1.1 s, came back, mode fell to IDLE, then it fell over
            ("radio", "cut"),       # 03:01:02  RC flapping, mode fell to IDLE (the firmware bug fixed since)
            ("runaway", "tip"),     # 03:08:27  lean target ran to -23 deg, motors at full power for 1.3 s
        ])
        self.assertEqual(rec.ignored_total, 1)       # 03:01:?? the owner pressed IDLE from the PC: not a fall
        self.assertIn("saturated", self.tags(saved[0]))
        self.assertIn("saturated", self.tags(saved[3]))
        self.assertTrue(all(m["tipped"] for m in saved))
        self.assertIn("1.1", saved[1]["tags"][0]["detail"])      # longest dropout 1.10 s
        # the saved file reads back with the params active then
        meta, rows, events, ranges = F.record_rows(os.path.join(self.tmp, saved[1]["id"]))
        self.assertEqual(meta["params"]["bal.cutoff_deg"], 45.0)
        res = F.analyze(rows, meta["params"], events, ranges, meta["trigger"])
        self.assertEqual(res["cause"], "radio")


class TestSim(Base):
    def run_sim(self, push_deg, secs=6.0, stop_after_push=False):
        from station.sim.sim import SimRobot
        r = SimRobot(seed=3)
        r.pi_link = True
        r.requested, r.armed = lp.MODE_BALANCE, True
        rec = F.FallRecorder(self.tmp, params=lambda: {p[0]: p[1] for p in r.params}, threaded=False,
                             clock=lambda: r.t, wall=lambda: 1.8e9 + r.t)
        parser, saved, pushed = lp.FrameParser(), [], False
        while r.t < secs:
            r.step()
            for fr in r.out:
                for t, p in parser.feed(fr):
                    if t == lp.MSG_TELEMETRY:
                        rec.on_tlm(lp.decode_telemetry(p), r.t)
                        saved += rec.tick(r.t)
            r.out.clear()
            if not pushed and r.t > 2.0:
                pushed = True
                r.push_robot(push_deg)
                if stop_after_push:
                    rec.user_stop("test", r.t)
                    r.requested = lp.MODE_IDLE
                    r.armed = False
        saved += rec.tick(r.t + 10)
        return saved, rec

    def test_small_push_is_recovered(self):
        saved, rec = self.run_sim(60)
        self.assertEqual(saved, [])

    def test_big_push_topples_the_sim_robot(self):
        saved, rec = self.run_sim(800)
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]["cause"], "pushed")
        self.assertTrue(saved[0]["tipped"])

    def test_estop_during_a_fall_is_not_a_fall(self):
        saved, rec = self.run_sim(800, stop_after_push=True)
        self.assertEqual(saved, [])


if __name__ == "__main__":
    unittest.main()
