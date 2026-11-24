"""LD19 obstacles for the robot's navigator (issue #77): MSG_LIDAR_OBSTACLES encoding and the bridge's send gating
(lidar_obstacles.py), on the simulated LD19 + clock of test_lidar.Rig."""
import math
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge.lidar_obstacles import MAX_SCAN_AGE_S, LidarObstacles
from station.bridge.tests.test_lidar import Rig

N = lp.LINK_LIDAR_SECTORS


def payload(frame):
    return frame[4:-2]


class TestEncoding(unittest.TestCase):
    def test_round_trip(self):
        pts = [None] * N
        pts[0] = (1234.4, -5.6)
        pts[9] = (-3.0, 870.0)
        pts[35] = (40000.0, -40000.0)          # clamped to int16
        pts[18] = (0.2, -0.3)                  # a return that rounds to (0, 0) must not read as "none"
        f = lp.encode_lidar_obstacles(300, 12.6, pts)
        self.assertEqual(f[2], lp.MSG_LIDAR_OBSTACLES)
        self.assertEqual(len(payload(f)), lp.LIDAR_OBST_SIZE)
        self.assertEqual(lp.LIDAR_OBST_SIZE, 4 + 4 * N)
        (t, p), = lp.FrameParser().feed(f)
        d = lp.decode(t, p)
        self.assertEqual((d["seq"], d["sectors"], d["age_ms"]), (300 & 0xFF, N, 13))
        self.assertEqual(d["pts"][0], (1234, -6))
        self.assertEqual(d["pts"][9], (-3, 870))
        self.assertEqual(d["pts"][35], (32767, -32768))
        self.assertEqual(d["pts"][18], (1, 0))
        self.assertEqual(d["pts"][1], (0, 0))
        self.assertEqual(sum(1 for x, y in d["pts"] if x or y), 4)

    def test_bad_frames(self):
        with self.assertRaises(ValueError):
            lp.encode_lidar_obstacles(0, 0, [None] * 16)
        p = payload(lp.encode_lidar_obstacles(0, 0, [None] * N))
        with self.assertRaises(ValueError):
            lp.decode_lidar_obstacles(p[:-2])  # short: the firmware drops it (len != sizeof)
        bad = bytes([p[0], 16]) + p[2:]
        with self.assertRaises(ValueError):
            lp.decode_lidar_obstacles(bad)     # other sector count
        self.assertEqual(lp.decode_lidar_obstacles(p)["age_ms"], 0)
        big = payload(lp.encode_lidar_obstacles(0, 1e9, [None] * N))
        self.assertEqual(lp.decode_lidar_obstacles(big)["age_ms"], 0xFFFF)

    def test_fits_a_frame(self):
        self.assertLessEqual(lp.LIDAR_OBST_SIZE, lp.LINK_MAX_PAYLOAD)
        self.assertEqual(lp.MSG_NAMES[lp.MSG_LIDAR_OBSTACLES], "MSG_LIDAR_OBSTACLES")


def expected_sectors(room, x, y, th, mask=((87.0, 93.0), (267.0, 273.0)), rmin=120.0, rmax=8000.0):
    """Nearest wall per sector by ray casting from the robot (lidar at the axle), robot frame, 0.25 deg rays."""
    best = [None] * N
    for k in range(1440):
        a = k * 0.25
        if any(lo <= a <= hi for lo, hi in mask):
            continue
        d = room.raycast(x, y, math.radians(th + a), rmax)
        if d is None or d < rmin:
            continue
        i = int(math.floor(a / 10.0 + 0.5)) % N
        if best[i] is None or d < best[i]:
            best[i] = d
    return best


class Feed:
    """Rig + LidarObstacles; frames decoded as sent."""

    def __init__(self, tmp, link=True, **kw):
        self.rig = Rig(tmp, **kw)
        self.frames = []
        self.link = link
        self.obst = LidarObstacles(self.rig.lidar, self.send, clock=lambda: self.rig.r.t)

    def send(self, frame):
        if not self.link:
            return False
        (t, p), = lp.FrameParser().feed(frame)
        self.frames.append((self.rig.r.t, lp.decode(t, p)))
        return True


class TestGating(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def check_room(self, f, d, tol_frac=0.02):
        r = f.rig.r
        exp = expected_sectors(r.room, r.X, r.Y, r.TH)
        for i, (x, y) in enumerate(d["pts"]):
            if exp[i] is None:
                self.assertEqual((x, y), (0, 0), i)
                continue
            self.assertNotEqual((x, y), (0, 0), i)
            rng = math.hypot(x, y)
            self.assertLess(abs(rng - exp[i]), max(40.0, tol_frac * exp[i]), (i, rng, exp[i]))
            a = math.degrees(math.atan2(y, x)) % 360.0
            self.assertLess(abs((a - i * 10.0 + 180.0) % 360.0 - 180.0), 5.01, (i, a))   # inside its own sector

    def test_frames_match_the_room(self):
        f = Feed(self.tmp)
        f.rig.run(1.5)
        self.assertGreaterEqual(len(f.frames), 12)                          # ~10 Hz
        self.assertEqual(f.obst.n["sent"], len(f.frames))
        self.assertEqual([d["seq"] for _t, d in f.frames], list(range(len(f.frames))))
        for _t, d in f.frames:
            self.assertLessEqual(d["age_ms"], 15)                           # telemetry at 100 Hz
        self.check_room(f, f.frames[-1][1])
        x, y = f.frames[-1][1]["pts"][0]
        self.assertAlmostEqual(x, 4000, delta=60)                           # far wall straight ahead
        self.assertEqual(f.obst.status()["sent"], len(f.frames))

    def test_robot_frame_when_turned(self):
        f = Feed(self.tmp)
        f.rig.r.X, f.rig.r.Y, f.rig.r.TH = 500.0, 300.0, 90.0
        f.rig.run(1.0)
        self.assertGreater(len(f.frames), 5)
        self.check_room(f, f.frames[-1][1])
        x, y = f.frames[-1][1]["pts"][0]                                    # ahead = +Y in the world: wall y = 2000
        self.assertAlmostEqual(x, 1700, delta=60)

    def test_moving_robot_is_deskewed_to_the_newest_pose(self):
        f = Feed(self.tmp, tlm_hz=100)
        f.rig.mover = lambda r, dt: setattr(r, "X", r.X + 400.0 * dt)     # 400 mm/s toward the far wall
        f.rig.run(1.2)
        self.assertGreater(len(f.frames), 6)
        t, d = f.frames[-1]
        # the frame is in the robot frame of the newest telemetry pose, age_ms ago
        x_then = f.rig.r.X - 400.0 * (f.rig.r.t - t) - 400.0 * d["age_ms"] / 1000.0
        self.assertAlmostEqual(d["pts"][0][0], 4000 - x_then, delta=40)
        self.assertAlmostEqual(d["pts"][18][0], -1000 - x_then, delta=40)

    def test_mapping_switches_do_not_matter(self):
        f = Feed(self.tmp, every_n=3, ray_step=4)
        f.rig.mapper.settings["enabled"] = False
        f.rig.run(1.5)
        self.assertGreaterEqual(len(f.frames), 12)                          # every scan, even with the map off

    def test_tilted_scans_are_not_sent(self):
        f = Feed(self.tmp, pitch=lambda r: 8.0 if 1.0 < r.t < 1.6 else 0.0)  # > pitch_max_deg (6)
        f.rig.run(1.0)
        n0 = len(f.frames)
        self.assertGreater(n0, 6)
        f.rig.run(0.6)                                                      # leaning: nothing
        self.assertLessEqual(len(f.frames) - n0, 1)                         # (a scan that ended before the lean)
        self.assertGreater(f.obst.n["tilt"], 3)
        n1 = len(f.frames)
        f.rig.run(1.0)
        self.assertGreater(len(f.frames) - n1, 6)                           # upright again: frames again

    def test_a_single_tilted_packet_drops_the_scan(self):
        f = Feed(self.tmp, pitch=lambda r: 8.0 if 0.5 <= r.t % 1.0 < 0.51 else 0.0)   # one telemetry sample a second
        f.rig.run(3.0)
        self.assertGreaterEqual(f.obst.n["tilt"], 2)
        self.assertGreater(len(f.frames), 15)
        self.assertLess(len(f.frames), 29)

    def test_leaning_to_speed_up_still_sends(self):
        # 4 deg is past the map's gate (2.5) but inside the navigator's (pitch_max_deg 6): sent, floor clipped
        f = Feed(self.tmp, pitch=lambda r: 4.0 if r.t > 1.0 else 0.0, height_mm=200.0)
        f.rig.run(1.0)
        n0 = len(f.frames)
        f.rig.run(0.6)
        self.assertGreater(len(f.frames) - n0, 4)
        self.assertGreater(f.rig.lidar.n["tilt"], 0)                        # the map dropped them
        d = f.frames[-1][1]
        floor = 0.9 * 200.0 / math.tan(math.radians(4.0))                  # ~2.6 m: the far walls ahead / behind
        x, y = d["pts"][0]
        self.assertEqual((x, y), (0, 0))                                    # far wall ahead (4 m) clipped, not a hit
        self.assertNotEqual(d["pts"][9], (0, 0))                            # the sides (no tilt there) still seen
        self.assertGreater(floor, 2000)

    def test_lying_down_nothing(self):
        f = Feed(self.tmp, pitch=lambda r: 55.0)
        f.rig.run(1.5)
        self.assertEqual(f.frames, [])
        self.assertGreater(f.obst.n["tilt"], 0)

    def test_silent_lidar_sends_nothing(self):
        f = Feed(self.tmp)
        f.rig.run(1.0)
        n0 = len(f.frames)
        step = f.rig.sim.step
        f.rig.sim.step = lambda robot, active=True: step(robot, active=False)   # unplugged: no packets
        f.rig.run(1.0)
        self.assertEqual(len(f.frames), n0)

    def test_no_pose_no_frames(self):
        f = Feed(self.tmp, tlm_hz=0.001)                                    # one telemetry frame at the start, then none
        f.rig.run(1.5)
        self.assertTrue(all(t < 0.004 + lp.LINK_LIDAR_MAX_AGE_MS / 1000.0 for t, _d in f.frames), f.frames)
        self.assertGreater(f.obst.n["no_pose"], 8)

    def test_old_pose_no_frames(self):
        f = Feed(self.tmp, tlm_hz=2.5)                                      # telemetry every 400 ms > 300 ms max age
        f.rig.run(2.0)
        self.assertGreater(f.obst.n["no_pose"], 0)
        for _t, d in f.frames:
            self.assertLessEqual(d["age_ms"], lp.LINK_LIDAR_MAX_AGE_MS)

    def test_disabled(self):
        f = Feed(self.tmp, enabled=False)
        f.rig.run(1.0)
        self.assertEqual(f.frames, [])
        self.assertGreater(f.obst.n["off"], 0)

    def test_no_link(self):
        f = Feed(self.tmp, link=False)
        f.rig.run(1.0)
        self.assertGreater(f.obst.n["no_link"], 5)
        self.assertEqual(f.obst.n["sent"], 0)
        self.assertIsNone(f.obst.status()["age_s"])

    def test_late_scan_is_not_sent(self):
        f = Feed(self.tmp)
        got = []
        f.rig.lidar.scan_listeners.append(got.append)
        f.rig.run(0.5)
        self.assertTrue(got)
        sc = got[-1]
        self.assertIsNone(f.obst.build(sc, now=sc.t1 + MAX_SCAN_AGE_S + 0.01))
        self.assertGreater(f.obst.n["stale"], 0)

    def test_listener_error_does_not_stop_the_mapping(self):
        f = Feed(self.tmp)
        f.rig.lidar.scan_listeners.insert(0, lambda sc: 1 / 0)
        f.rig.run(1.2)
        self.assertGreater(f.rig.lidar.n["used"], 5)
        self.assertGreater(len(f.frames), 5)
        self.assertIn("ZeroDivisionError", f.rig.lidar.reader.error)


if __name__ == "__main__":
    unittest.main()
