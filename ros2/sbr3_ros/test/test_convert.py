"""sbr3_ros.convert: bridge ROS stream -> ROS values, cmd_vel gate. No ROS needed:
  python -m unittest discover -s ros2/sbr3_ros/test      (Windows or WSL)
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))   # repo: station.*

from sbr3_ros import convert as cv  # noqa: E402

try:
    from station.bridge import ros_stream as rs   # the bridge side, to test both ends of the format
except ImportError:                                # the package alone (installed copy in ~/sbr3_ws)
    rs = None

HELLO = {"t": "ros_hello", "v": 1, "subscribed": True,
         "lidar": {"x_mm": 10.0, "y_mm": -5.0, "height_mm": 208.0, "yaw_deg": 0.0, "min_range_mm": 120.0,
                   "max_range_mm": 8000.0},
         "scan": {"bins": 480, "angle_min": -math.pi, "angle_increment": 2 * math.pi / 480, "hz": 10},
         "odom": {"cols": ["ts", "x_mm", "y_mm", "heading_deg", "speed_mms", "turn_rate_dps", "pitch_deg",
                           "pitch_rate_dps", "roll_deg", "yaw_rate_dps"]}}
STATE = {"t": "ros_state", "link": "connected", "alive": True, "mode": "STATION", "armed": True, "estop": False,
         "robot_estop": False}


def quat_to_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class TestClock(unittest.TestCase):
    def test_min_latency_offset(self):
        c = cv.ClockSync()
        self.assertIsNone(c.to_ros(1.0))
        c.update(100.0, 5000.030)            # 30 ms latency
        c.update(100.5, 5000.505)            # 5 ms: better
        c.update(101.0, 5001.090)            # a slow one does not move it (much)
        self.assertAlmostEqual(c.offset, 4900.005, places=3)
        self.assertAlmostEqual(c.to_ros(102.0), 5002.005, places=3)

    def test_creeps_and_resets(self):
        c = cv.ClockSync(creep=1e-3)
        c.update(0.0, 1000.0)
        for k in range(1, 101):              # the bridge clock runs 0.05 % slow: the offset follows
            c.update(k * 0.9995, 1000.0 + k)
        self.assertAlmostEqual(c.offset, 1000.0 + 100 * 0.0005, delta=0.01)
        c.update(3.0, 1101.0)                # bridge restarted: monotonic clock elsewhere
        self.assertEqual(c.resets, 1)
        self.assertAlmostEqual(c.offset, 1098.0)

    def test_split_stamp(self):
        self.assertEqual(cv.split_stamp(12.25), (12, 250000000))
        self.assertEqual(cv.split_stamp(3.9999999999), (4, 0))


class TestGeometry(unittest.TestCase):
    def test_quaternions(self):
        for r, p, y in [(0, 0, 0), (0.1, -0.2, 1.0), (0, 0, math.pi * 0.99), (-0.05, 0.3, -2.5)]:
            q = cv.quat_from_rpy(r, p, y)
            self.assertAlmostEqual(sum(v * v for v in q), 1.0)
            for a, b in zip(quat_to_rpy(q), (r, p, y)):
                self.assertAlmostEqual(a, b)
        self.assertEqual(cv.quat_yaw(0.0), (0.0, 0.0, 0.0, 1.0))

    def test_corr_to_tf(self):
        x, y, yaw = cv.corr_to_tf([120.0, -40.0, 370.0])
        self.assertEqual((x, y), (0.12, -0.04))
        self.assertAlmostEqual(yaw, math.radians(10.0))

    def test_corr_matches_map_pose(self):
        """map -> odom from corr, applied to an odometry pose, gives Mapper.map_pose()."""
        corr = [300.0, -200.0, 15.0]
        tx, ty, tyaw = cv.corr_to_tf(corr)
        ox, oy, oh = 1.0, 0.5, 30.0                  # odometry pose (m, m, deg)
        mx = tx + math.cos(tyaw) * ox - math.sin(tyaw) * oy
        my = ty + math.sin(tyaw) * ox + math.cos(tyaw) * oy
        c, s = math.cos(math.radians(15)), math.sin(math.radians(15))
        self.assertAlmostEqual(mx * 1000, c * 1000 - s * 500 + 300, places=6)   # mapping.Mapper.map_pose
        self.assertAlmostEqual(my * 1000, s * 1000 + c * 500 - 200, places=6)

    def test_laser_mount(self):
        self.assertEqual(cv.laser_mount(HELLO), (0.01, -0.005, 0.208, 0.0))
        self.assertEqual(cv.laser_mount({"lidar": None})[2], 0.208)


class TestScan(unittest.TestCase):
    def test_layout(self):
        L = cv.scan_layout(HELLO)
        self.assertEqual(L["bins"], 480)
        self.assertAlmostEqual(L["angle_max"], math.pi - 2 * math.pi / 480)
        self.assertEqual((L["range_min"], L["range_max"]), (0.12, 8.0))
        self.assertAlmostEqual(L["scan_time"], 0.1)

    def test_ranges(self):
        import base64
        import struct
        raw = [0, 100, 1500, 9000, 65535] + [2000] * 475
        msg = {"r": base64.b64encode(struct.pack("<480H", *raw)).decode()}
        out = cv.scan_ranges(msg, 0.12, 8.0)
        self.assertEqual(len(out), 480)
        self.assertEqual(out[0], cv.INF)                 # no return
        self.assertEqual(out[1], cv.INF)                 # inside the cage
        self.assertEqual(out[2], 1.5)
        self.assertEqual(out[3], cv.INF)                 # beyond range_max
        self.assertEqual(out[5], 2.0)

    @unittest.skipIf(rs is None, "station.bridge not importable")
    def test_same_format_as_the_bridge(self):
        rr = [0, 120, 3456, 8000] * 120
        out = cv.scan_ranges({"r": rs.encode_ranges(rr)}, 0.12, 8.0)
        self.assertEqual(out[:4], [cv.INF, 0.12, 3.456, 8.0])
        self.assertEqual(HELLO["odom"]["cols"], rs.ODOM_COLS)

    def test_keep_scan(self):
        self.assertTrue(cv.keep_scan({"tilt": 2.0, "tilt_pk": 0}, 6.0))
        self.assertFalse(cv.keep_scan({"tilt": 7.0, "tilt_pk": 0}, 6.0))
        self.assertFalse(cv.keep_scan({"tilt": 5.0, "tilt_pk": 2}, 6.0))   # packets were left out
        self.assertFalse(cv.keep_scan({"tilt": 3.0, "tilt_pk": 0}, 2.5))


class TestOdom(unittest.TestCase):
    def test_rows(self):
        m = {"rows": [[12.5, 1000.0, -250.0, 450.0, 200.0, -90.0, -3.0, 10.0, 1.0, -90.0]]}
        r, = cv.odom_rows(m, HELLO["odom"]["cols"])
        self.assertEqual((r["t"], r["x"], r["y"], r["v"]), (12.5, 1.0, -0.25, 0.2))
        self.assertAlmostEqual(r["yaw"], math.radians(90.0))           # 450 deg wrapped
        self.assertAlmostEqual(r["w"], -math.pi / 2)
        self.assertAlmostEqual(r["pitch"], math.radians(-3.0))
        self.assertAlmostEqual(r["yaw_rate"], -math.pi / 2)

    def test_short_or_reordered_rows(self):
        cols = ["ts", "heading_deg", "x_mm"]
        r, = cv.odom_rows({"rows": [[1.0, 180.0, 5.0]]}, cols)
        self.assertEqual((r["x"], r["y"]), (0.005, 0.0))
        self.assertAlmostEqual(abs(r["yaw"]), math.pi)


class TestCmdGate(unittest.TestCase):
    def test_drivable(self):
        self.assertEqual(cv.drivable(STATE), (True, ""))
        for k, v, word in [("mode", "BALANCE", "BALANCE"), ("mode", "GOTO", "GOTO"), ("armed", False, "disarmed"),
                           ("estop", True, "E-stop"), ("robot_estop", True, "E-stop"),
                           ("alive", False, "not connected"), ("link", "waiting", "not connected")]:
            ok, why = cv.drivable(dict(STATE, **{k: v}))
            self.assertFalse(ok, k)
            self.assertIn(word, why)
        self.assertFalse(cv.drivable(None)[0])
        self.assertFalse(cv.drivable(dict(STATE, _rx=0.0), now_s=5.0)[0])    # stale state

    def test_caps_and_slew(self):
        g = cv.CmdGate(max_speed=0.25, max_turn=1.0, max_accel=0.5, max_turn_accel=2.0)
        g.on_twist(2.0, -5.0, 0.0)                     # capped to 0.25 m/s, -1 rad/s
        self.assertEqual(g.target, (0.25, -1.0))
        outs = []
        for k in range(0, 30):
            g.on_twist(2.0, -5.0, 0.05 * k)
            outs.append(g.step(0.05 * k, STATE))
        self.assertEqual(outs[0], {"t": "cmd_vel", "speed": 25, "turn": -5.7})   # 0.5 m/s^2 * 50 ms
        speeds = [o["speed"] for o in outs]
        self.assertEqual(max(speeds), 250)
        self.assertTrue(all(b - a <= 26 for a, b in zip(speeds, speeds[1:])))
        self.assertEqual(outs[-1]["turn"], -57.3)

    def test_deadman(self):
        g = cv.CmdGate(deadman_s=0.5, max_accel=100.0, max_turn_accel=100.0)
        g.on_twist(0.2, 0.0, 0.0)
        self.assertEqual(g.step(0.1, STATE)["speed"], 200)
        self.assertEqual(g.step(0.45, STATE)["speed"], 200)
        self.assertEqual(g.step(0.55, STATE), {"t": "cmd_vel", "speed": 0, "turn": 0})   # one stop ...
        self.assertIsNone(g.step(0.6, STATE))                                          # ... then silence
        self.assertIsNone(g.step(5.0, STATE))
        g.on_twist(0.1, 0.0, 5.0)
        self.assertEqual(g.step(5.05, STATE)["speed"], 100)

    def test_not_drivable_sends_nothing_but_one_stop(self):
        g = cv.CmdGate(max_accel=100.0)
        g.on_twist(0.2, 0.0, 0.0)
        self.assertIsNone(g.step(0.05, dict(STATE, mode="BALANCE")))
        self.assertIn("BALANCE", g.why)
        self.assertEqual(g.step(0.1, STATE)["speed"], 200)
        self.assertEqual(g.step(0.15, dict(STATE, estop=True)), {"t": "cmd_vel", "speed": 0, "turn": 0})
        self.assertIsNone(g.step(0.2, dict(STATE, estop=True)))
        self.assertEqual(g.out, (0.0, 0.0))
        self.assertEqual(g.step(0.25, STATE)["speed"], 200)    # back in STATION: slews from 0 (here: fast)

    def test_never_arms_or_changes_mode(self):
        g = cv.CmdGate()
        sent = []
        for k in range(200):
            g.on_twist(0.3 * math.sin(k / 7.0), 0.5, k * 0.05)
            st = dict(STATE, mode=["STATION", "IDLE", "BALANCE"][k % 3], armed=bool(k % 2))
            o = g.step(k * 0.05 + 0.01, st)
            if o:
                sent.append(o)
        self.assertTrue(sent)
        self.assertEqual({o["t"] for o in sent}, {"cmd_vel"})

    def test_bad_numbers(self):
        g = cv.CmdGate()
        g.on_twist(float("nan"), float("inf"), 0.0)
        self.assertEqual(g.target, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
