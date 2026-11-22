"""Stop-and-scan pose correction (#71): the matcher on synthetic sweeps, ambiguity rejection, the StopScan
flow in the Mapper, the planner hooks, and an end-to-end sim run with odometry drift."""
import math
import random
import shutil
import tempfile
import unittest

from station.bridge import scanmatch
from station.bridge.mapping import Mapper
from station.bridge.occgrid import Grid
from station.sim.sim import Room, _box
from station.sim.tour import DriftRobot, plan_offline


class Walls:
    """Line-segment world + a sweep generator like the sim (two sensors back to back, 16 servo steps over
    170 deg, the nearest wall within +-cone counts, range noise 8 mm + 1 %)."""

    def __init__(self, segs, seed=1):
        self.room = Room()
        self.room.segs = segs
        self.rng = random.Random(seed)

    def ray(self, ox, oy, a_deg, cone=8.0):
        best = None
        for e in (-cone, -cone / 2, 0.0, cone / 2, cone):
            d = self.room.raycast(ox, oy, math.radians(a_deg + e))
            if d is not None and (best is None or d < best):
                best = d
        return best

    def sweep(self, x, y, h, err=(0.0, 0.0, 0.0), sweeps=2, noise=1.0):
        """Readings taken standing at the true pose (x, y, h); odometry reports it off by err (world mm, deg)."""
        out = []
        for k in range(16 * sweeps):
            step = k % 16 if (k // 16) % 2 == 0 else 15 - k % 16
            a = -85.0 + step * 170.0 / 15
            for sid in (0, 1):
                ang = a + 180.0 * sid
                hr = math.radians(h)
                d = self.ray(x + 26.0 * math.cos(hr), y + 26.0 * math.sin(hr), h + ang)
                ok = d is not None and d - 12.9 <= 2000
                r = d - 12.9 + self.rng.gauss(0, (8 + 0.01 * d) * noise) if ok else 8190
                out.append({"t_us": 0, "x_mm": x + err[0], "y_mm": y + err[1], "heading_deg": h + err[2],
                            "pitch_deg": -3.0, "sensor_angle_deg": ang if ang <= 180 else ang - 360,
                            "range_mm": r, "sensor_id": sid, "valid": 1 if ok else 0})
        return out


def room_segs():
    """The sim room, moved half a cell so its walls run through cell centres (on a cell edge a perfect raster
    would put a wall half a cell off on one side of the room)."""
    return [(x1 + 12.5, y1 + 12.5, x2 + 12.5, y2 + 12.5) for x1, y1, x2, y2 in Room().segs]


def raster(segs, size_m=20.0):
    """A perfect map: free everywhere, sure walls on the segments."""
    g = Grid(size_m)
    g.cells[:] = bytes([90]) * len(g.cells)
    for x1, y1, x2, y2 in segs:
        n = int(math.hypot(x2 - x1, y2 - y1) / 10) + 1
        for k in range(n + 1):
            i = g.index(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n)
            if i is not None:
                g.cells[i] = 198
    g.touched.add((0, 0))
    return g


def scan_of(mapper, readings):
    ss = mapper.stopscan
    ss.buf = list(readings)
    scan, ref = ss._scan()
    ss.buf = []
    return scan, ref


class TestMatcher(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_known_offset_recovered(self):
        w = Walls(room_segs(), seed=3)
        g = raster(w.room.segs)
        cases = [((500, 300, 30), (60, -40, 2.0)), ((2000, 0, 10), (-50, 30, -1.5)),
                 ((0, 1000, -60), (40, 60, 3.0)), ((3300, 500, 120), (-70, -20, -2.5))]
        for pose, err in cases:
            scan, ref = scan_of(self.m, w.sweep(*pose, err=err))
            r = scanmatch.match_scan(g, scan, ref[:2], ref[2])
            self.assertTrue(r["ok"], (pose, r))
            # odometry = truth + err: the correction is -err (rotation about the robot)
            self.assertLess(math.hypot(r["dx"] + err[0], r["dy"] + err[1]), 40, (pose, r))
            self.assertLess(abs(r["dth"] + err[2]), 1.0, (pose, r))

    def test_correct_pose_is_left_alone(self):
        w = Walls(room_segs(), seed=4)
        g = raster(w.room.segs)
        scan, ref = scan_of(self.m, w.sweep(1500, -500, 90))
        r = scanmatch.match_scan(g, scan, ref[:2], ref[2])
        self.assertLess(math.hypot(r["dx"], r["dy"]), 25, r)
        self.assertLess(abs(r["dth"]), 0.8, r)

    def test_long_featureless_corridor_is_rejected(self):
        # 1.2 m wide, 14 m long: nothing tells where along it the robot is (the sensors reach 2 m)
        segs = [(-7000, -612.5, 7000, -612.5), (-7000, 612.5, 7000, 612.5)]
        w = Walls(segs, seed=5)
        g = raster(segs)
        scan, ref = scan_of(self.m, w.sweep(0, 0, 0, err=(80, 0, 0)))
        r = scanmatch.match_scan(g, scan, ref[:2], ref[2])
        self.assertIn("ambiguous", r["why"])
        self.assertEqual((r["dx"], r["dy"]), (0.0, 0.0), r)                   # never a position correction
        self.assertLess(abs(r["dth"]), 1.0, r)

    def test_unknown_area_is_rejected(self):
        w = Walls(room_segs())
        scan, ref = scan_of(self.m, w.sweep(500, 300, 0))
        r = scanmatch.match_scan(Grid(20.0), scan, ref[:2], ref[2])         # empty map
        self.assertFalse(r["ok"])
        self.assertTrue("fit the map" in r["why"] or "no fit" in r["why"], r)

    def test_too_few_readings(self):
        r = scanmatch.match_scan(Grid(20.0), [(0, 0, 0.0, 500.0)] * 5, (0, 0), 0.0)
        self.assertFalse(r["ok"])
        self.assertIn("too few", r["why"])

    def test_correction_is_bounded(self):
        w = Walls(room_segs(), seed=6)
        g = raster(w.room.segs)
        scan, ref = scan_of(self.m, w.sweep(1000, 500, 45, err=(400, 0, 0)))    # far outside the window
        r = scanmatch.match_scan(g, scan, ref[:2], ref[2])
        self.assertLessEqual(abs(r["dx"]), scanmatch.MAX_XY_MM)
        self.assertLessEqual(abs(r["dth"]), scanmatch.MAX_TH_DEG)
        self.assertFalse(r["ok"], r)


class TestStopScanFlow(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)
        self.m.settings["stop_scan"] = True
        self.w = Walls(room_segs(), seed=7)
        self.m.grid = raster(self.w.room.segs)                         # a known map (the matcher has its own tests)
        self.m.n["used"] = 100

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_still_readings_are_held_back_matched_then_mapped(self):
        ss = self.m.stopscan
        rd = self.w.sweep(500, 300, 20, err=(50, -30, 1.5))
        used = self.m.n["used"]
        for d in rd[:scanmatch.SCAN_N - 1]:
            ss.add(d)
        self.assertEqual(len(ss.buf), scanmatch.SCAN_N - 1)
        self.assertEqual(self.m.n["used"], used)                       # held back
        ss.add(rd[scanmatch.SCAN_N - 1])
        self.assertEqual(ss.seq, 1)
        self.assertEqual(self.m.n["used"], used + scanmatch.SCAN_N)    # mapped after the match
        e = ss.last
        self.assertTrue(e["ok"], e)
        self.assertEqual(ss.applied, 1)
        # the map pose of the odometry pose is now close to the truth
        x, y, h = self.m.map_pose(550, 270, 21.5)
        self.assertLess(math.hypot(x - 500, y - 300), 35)
        self.assertLess(abs(h - 20), 1.6)                              # (may be a position-only correction)
        self.assertTrue(ss.done)
        ss.add(rd[0])                                                  # still standing: mapped right away
        self.assertEqual(len(ss.buf), 0)
        msg = self.m.scan_msg()
        self.assertIn('"applied":1', msg)

    def test_moving_flushes_without_a_match(self):
        ss = self.m.stopscan
        for k, d in enumerate(self.w.sweep(500, 300, 0)[:40]):
            d = dict(d, x_mm=500 + 10 * k)                             # driving: 10 mm per reading
            ss.add(d)
            self.assertLess(len(ss.buf), 10)
        self.assertEqual(ss.seq, 0)

    def test_off_maps_directly_and_request_works(self):
        self.m.settings["stop_scan"] = False
        used = self.m.n["used"]
        for d in self.w.sweep(500, 300, 0)[:10]:
            self.m.on_range(d)
        self.assertGreater(self.m.n["used"], used)                     # mapped right away
        self.assertEqual(self.m.stopscan.buf, [])
        self.assertIsNone(self.m.command({"t": "map", "action": "scan"}, "test"))
        for d in self.w.sweep(500, 300, 0, err=(40, 40, 0), sweeps=4):
            self.m.on_range(d)
        self.assertEqual(self.m.stopscan.seq, 1)
        self.assertEqual(self.m.stopscan.last["who"], "test")
        self.assertIsNone(self.m.stopscan.request)

    def test_reset_clears(self):
        ss = self.m.stopscan
        for d in self.w.sweep(500, 300, 0, err=(40, 40, 0)):
            ss.add(d)
        self.m.reset()
        self.assertEqual((ss.seq, ss.applied, ss.buf, list(ss.history)), (0, 0, [], []))
        self.assertEqual(self.m.corr, [0.0, 0.0, 0.0])


class TestPlannerHooks(unittest.TestCase):
    def test_explore_pauses_for_the_scan_and_routes_follow_corrections(self):
        tmp = tempfile.mkdtemp()
        try:
            r = DriftRobot(seed=2, drift_deg_min=3.0, drift_scale=0.02)
            m = Mapper(tmp)
            m.settings["stop_scan"] = True
            stats = {"sent_after_corr": 0, "holding": 0}

            def watch(robot, b):
                stats["holding"] += b.planner.holding or b.planner.scan_wait is not None
            s, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=150, on_step=watch)
            self.assertGreater(m.stopscan.seq, 2)                      # it stopped and scanned
            self.assertGreater(stats["holding"], 10)
            for e in m.stopscan.history:
                if e["ok"]:
                    self.assertLessEqual(e["mm"], scanmatch.MAX_XY_MM * 1.5)
                    self.assertLessEqual(abs(e["dth"]), scanmatch.MAX_TH_DEG)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_correction_resends_the_rest_of_the_route_in_the_new_frame(self):
        tmp = tempfile.mkdtemp()
        try:
            from station.sim.tour import OfflineBridge
            r = DriftRobot(seed=1)
            m = Mapper(tmp)
            b = OfflineBridge(r, m)
            b.update()
            p = b.planner
            p.mode, p.path, p.sent_n = "goto", [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)], 2
            b.nav = {"nav_index": 0, "waypoint_count": 2}
            m.corr = [100.0, 0.0, 0.0]                                 # map = odometry + 100 mm in x
            p.on_correction({"ok": True})
            self.assertEqual(b.waypoints, [(900.0, 0.0), (900.0, 1000.0)])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
