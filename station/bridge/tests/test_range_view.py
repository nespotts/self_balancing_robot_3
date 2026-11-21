"""Range sensor view (issue #19): bridge range entries + rates, and the simulator's sweep model.
Run: python -m unittest discover -s station -t ."""
import math
import os
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge import bridge as bridge_mod
from station.bridge.bridge import Bridge
from station.sim.sim import SimRobot


def range_payload(**kw):
    d = dict(t_us=1000, x_mm=0.0, y_mm=0.0, heading_deg=0.0, pitch_deg=-3.0, sensor_angle_deg=0.0,
             range_mm=500, sensor_id=0, valid=1)
    d.update(kw)
    return lp.encode_range(d)[4:-2]   # strip sync/type/len and crc -> payload


class TestBridgeRanges(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.b = Bridge("tcp://127.0.0.1:1", self.tmp, self.tmp, quiet=True)

    def tearDown(self):
        self.b.log.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_entry_fields(self):
        b = self.b
        b._on_frame(lp.MSG_RANGE, range_payload(sensor_angle_deg=90.0, range_mm=1000, sensor_id=1))
        e = b.ranges[-1]
        self.assertEqual(len(e), 11)
        i, hx, hy, rx, ry, valid, tilt, t_ms, sid, ang, mm = e
        self.assertEqual((i, valid, tilt, sid, ang, mm), (0, 1, 0, 1, 90.0, 1000))
        # hit = servo axis (26 mm ahead of the axle) + (range + 12.9 mm) along the sensor direction
        self.assertEqual((rx, ry), (26, 0))
        self.assertEqual((hx, hy), (26, int(1000 + bridge_mod.SENSOR_RADIUS_MM)))

    def test_invalid_keeps_raw_range(self):
        b = self.b
        b._on_frame(lp.MSG_RANGE, range_payload(range_mm=8190, valid=0, sensor_angle_deg=-45.25, pitch_deg=20))
        e = b.ranges[-1]
        self.assertEqual(e[5], 0)            # invalid
        self.assertEqual(e[6], 1)            # tilted
        self.assertEqual(e[9], -45.2)        # angle rounded to 0.1 deg
        self.assertEqual(e[10], 8190)        # raw mm as the robot sent it
        d = math.hypot(e[1] - e[3], e[2] - e[4])
        self.assertAlmostEqual(d, 2000 + bridge_mod.SENSOR_RADIUS_MM, delta=2)   # map ray: 2 m

    def test_rates_per_sensor(self):
        b = self.b
        for k in range(30):
            b._on_frame(lp.MSG_RANGE, range_payload(sensor_id=k % 3 and 1))   # 10 x sensor 0, 20 x sensor 1
        b._tick(b.last_rate + 2.0)
        self.assertEqual(b.status()["rng_hz"], [5.0, 10.0])
        b._tick(b.last_rate + 1.0)
        self.assertEqual(b.status()["rng_hz"], [0.0, 0.0])

    def test_ranges_since(self):
        b = self.b
        for k in range(5):
            b._on_frame(lp.MSG_RANGE, range_payload(range_mm=100 + k))
        self.assertEqual([e[10] for e in b.ranges_since(3, 100)], [103, 104])
        self.assertEqual([e[10] for e in b.ranges_since(0, 2)], [103, 104])


class TestSimSweep(unittest.TestCase):
    def frames(self, robot, seconds):
        robot.out = []
        robot.run_until(robot.t + seconds, max_steps=100000)
        out = []
        for f in robot.out:
            if f[2] == lp.MSG_RANGE:
                out.append(lp.decode_range(f[4:-2]))
        return out

    def set(self, robot, name, v):
        robot.set_param(robot.pidx[name], v)

    def test_sweep_both_sensors(self):
        robot = SimRobot(seed=2)
        fr = self.frames(robot, 6.0)[-100:]          # (first move settles up to 1.5 s)
        by = {0: [f for f in fr if f["sensor_id"] == 0], 1: [f for f in fr if f["sensor_id"] == 1]}
        self.assertEqual(abs(len(by[0]) - len(by[1])) <= 1, True)
        a0 = [f["sensor_angle_deg"] for f in by[0]]
        self.assertAlmostEqual(min(a0), -85.0, delta=0.01)    # sweep 170, 16 steps
        self.assertAlmostEqual(max(a0), 85.0, delta=0.01)
        for f0, f1 in zip(by[0], by[1]):                      # read together, back to back
            self.assertAlmostEqual(abs(lp_wrap(f1["sensor_angle_deg"] - f0["sensor_angle_deg"])), 180, delta=0.01)
        dt = (fr[-1]["t_us"] - fr[0]["t_us"]) / 1e6
        self.assertAlmostEqual(len(fr) / dt, 25, delta=3)     # ~25 readings/s (both sensors)

    def test_park_and_trim(self):
        robot = SimRobot(seed=3, head_error_deg=8.0)
        self.set(robot, "rng.servo_hold_deg", 90)
        fr = self.frames(robot, 4.0)[-50:]
        self.assertTrue(all(abs(f["sensor_angle_deg"]) < 1e-3 for f in fr if f["sensor_id"] == 0))
        self.assertTrue(all(abs(abs(f["sensor_angle_deg"]) - 180) < 1e-3 for f in fr if f["sensor_id"] == 1))
        dt = (fr[-1]["t_us"] - fr[0]["t_us"]) / 1e6
        self.assertAlmostEqual(len(fr) / dt, 50, delta=5)     # parked: no settle -> ~50/s
        self.assertAlmostEqual(robot.rs_true_head, 8.0)       # mounted 8 deg off ...
        self.set(robot, "rng.servo_trim_deg", -8.0)
        self.frames(robot, 0.5)
        self.assertAlmostEqual(robot.rs_true_head, 0.0)       # ... trim -8 fixes it; reported angle stays 0
        self.set(robot, "rng.servo_hold_deg", 0)              # clamped to servo_min_deg 5 -> head -85
        fr = self.frames(robot, 1.0)
        self.assertAlmostEqual(fr[-1]["sensor_angle_deg"] if fr[-1]["sensor_id"] == 0
                               else lp_wrap(fr[-1]["sensor_angle_deg"] + 180), -85.0 + 8.0, delta=0.01)

    def test_enable_and_limits(self):
        robot = SimRobot(seed=4)
        self.set(robot, "rng.enable", 0)
        self.assertEqual(self.frames(robot, 1.0), [])
        self.set(robot, "rng.enable", 1)
        self.set(robot, "rng.max_mm", 50)                     # everything is farther: valid 0, raw mm kept
        fr = self.frames(robot, 3.0)
        self.assertTrue(fr)
        self.assertFalse(any(f["valid"] for f in fr))
        self.assertTrue(any(50 < f["range_mm"] < 8190 for f in fr))


def lp_wrap(a):
    return (a + 180.0) % 360.0 - 180.0


if __name__ == "__main__":
    unittest.main()
