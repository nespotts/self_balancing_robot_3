"""LD19 -> occupancy grid: masks, filters, pitch gating, deskew, dense scan fill, cost (issue #76)."""
import json
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import ld19, lidar as lidar_mod
from station.bridge.mapping import Mapper
from station.bridge.occgrid import UNKNOWN, Grid
from station.bridge.tests.test_ld19 import FakeRobot
from station.sim.lidar_sim import LidarSim


class Rig:
    """FakeRobot + LidarSim + Ld19Stream + ScanAssembler + Lidar + Mapper, all on the simulated clock."""

    def __init__(self, tmp, sim=None, robot=None, pitch=lambda r: 0.0, tlm_hz=100, **settings):
        self.r = robot or FakeRobot()
        self.sim = sim or LidarSim(seed=1, posts=True, dropout=0.01)
        self.stream = ld19.Ld19Stream()
        self.asm = ld19.ScanAssembler()
        self.mapper = Mapper(map_dir=tmp)
        self.lidar = lidar_mod.Lidar(self.mapper, "tcp://x", clock=lambda: self.r.t)
        self.lidar.apply_settings(settings, save=False)
        self.pitch = pitch
        self.tlm_dt = 1.0 / tlm_hz
        self.next_tlm = 0.0
        self.scans = 0
        self.cost = []
        self.mover = None            # callable(robot, dt): moves the robot

    def run(self, seconds, dt=0.004):
        end = self.r.t + seconds
        while self.r.t < end:
            if self.mover:
                self.mover(self.r, dt)
            self.r.t += dt
            if self.r.t >= self.next_tlm:
                self.next_tlm = self.r.t + self.tlm_dt
                self.lidar.on_tlm({"x_mm": self.r.X, "y_mm": self.r.Y, "heading_deg": self.r.TH,
                                   "pitch_deg": self.pitch(self.r)}, self.r.t)
            for p in self.stream.feed(self.sim.step(self.r), now=self.r.t):
                sc = self.asm.add(p)
                if sc:
                    self.scans += 1
                    t0 = time.perf_counter()
                    self.lidar.on_scan(sc)
                    self.cost.append(time.perf_counter() - t0)


def cell_max(grid, x, y, r=40.0):
    """Largest stored byte within r mm of (x, y) (128 = unknown, higher = occupied)."""
    best = 0
    for dx in (-r, 0, r):
        for dy in (-r, 0, r):
            best = max(best, grid.value(x + dx, y + dy))
    return best


class TestSettings(unittest.TestCase):
    def test_clean_settings(self):
        s = lidar_mod.clean_settings({"yaw_deg": "12.5", "height_mm": -5, "every_n": 2.6, "mask": [[10, 20], ["x"], [370, -10]],
                                      "tilt_mode": "bogus", "enabled": 0, "nope": 1, "min_range_mm": float("nan")},
                                     lidar_mod.DEFAULTS)
        self.assertEqual(s["yaw_deg"], 12.5)
        self.assertEqual(s["height_mm"], 20)                         # clamped
        self.assertEqual(s["every_n"], 3)
        self.assertEqual(s["mask"], [[10.0, 20.0], [10.0, 350.0]])   # junk skipped, angles taken mod 360
        self.assertEqual(s["tilt_mode"], "skip")
        self.assertIs(s["enabled"], False)
        self.assertEqual(s["min_range_mm"], lidar_mod.DEFAULTS["min_range_mm"])
        self.assertNotIn("nope", s)

    def test_persisted_like_the_other_settings(self):
        tmp = tempfile.mkdtemp()
        try:
            m = Mapper(map_dir=tmp)
            lid = lidar_mod.Lidar(m, "tcp://x")
            self.assertEqual(lid.settings["mask"], lidar_mod.DEFAULTS["mask"])
            lid.command({"action": "set", "settings": {"yaw_deg": 90, "mask": [[80, 100]], "x_mm": -30}})
            self.assertTrue(os.path.exists(os.path.join(tmp, "lidar.json")))
            lid2 = lidar_mod.Lidar(Mapper(map_dir=tmp), "tcp://x")
            self.assertEqual((lid2.settings["yaw_deg"], lid2.settings["x_mm"], lid2.settings["mask"]), (90, -30, [[80, 100]]))
            self.assertNotIn("lidar", [x["name"] for x in m.list()])   # not listed as a saved map
            r = lid2.command({"action": "defaults"})
            self.assertEqual(r["settings"]["yaw_deg"], 0.0)
            self.assertEqual(lid2.command({"action": "zzz"})["t"], "error")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_mask_table(self):
        t = lidar_mod.mask_table([[87, 93], [350, 10]])
        on = lambda deg: t[int(deg * 2)]
        self.assertTrue(on(90) and on(87.2) and on(92.8) and on(355) and on(0) and on(9.9))
        self.assertFalse(on(86.5) or on(93.5) or on(180) or on(270) or on(10.5) or on(349))
        self.assertEqual(sum(lidar_mod.mask_table([[5, 5]])), 0)


class TestGridFill(unittest.TestCase):
    def test_polygon_fill_circle(self):
        g = Grid()
        n = 360
        runs = [[(3000 * math.cos(2 * math.pi * k / n), 3000 * math.sin(2 * math.pi * k / n)) for k in range(n)]]
        g.integrate_scan(0.0, 0.0, runs, margin_mm=25.0)
        free = [g.value(x, y) for x in range(-2800, 2801, 200) for y in range(-2800, 2801, 200) if math.hypot(x, y) < 2850]
        self.assertTrue(all(v < UNKNOWN for v in free))
        self.assertGreater(len(free), 300)
        for k in range(0, 360, 10):                                  # the ring is occupied
            a = math.radians(k)
            self.assertGreater(cell_max(g, 3000 * math.cos(a), 3000 * math.sin(a), 25), UNKNOWN + 10)
        self.assertEqual(g.value(3500, 0), UNKNOWN)                  # outside: untouched
        self.assertEqual(g.value(0, 3500), UNKNOWN)

    def test_gap_leaves_unknown_and_far_points_connect(self):
        g = Grid()
        # two runs: a wall segment at y = 2000 seen from x -1000..-100 and 100..1000, 80 mm apart points (3 cells)
        run_a = [(x, 2000.0) for x in range(-1000, -90, 80)]
        run_b = [(x, 2000.0) for x in range(100, 1001, 80)]
        g.integrate_scan(0.0, 0.0, [run_a, run_b], margin_mm=25.0)
        self.assertLess(g.value(-500, 1000), UNKNOWN)               # inside the fans: free
        self.assertEqual(g.value(0, 1000), UNKNOWN)                 # between the runs: not seen
        for x in range(-1000, -100, 25):                            # the wall is a continuous line, not dots
            self.assertGreater(cell_max(g, x, 2000, 12), UNKNOWN + 10, x)

    def test_hit_counts_once_per_scan(self):
        g = Grid()
        run = [(2000.0 + 0.1 * k, 0.0) for k in range(50)]          # 50 points in one cell
        g.integrate_scan(0.0, 0.0, [run], margin_mm=25.0)
        self.assertEqual(g.value(2000.0, 0.0) - UNKNOWN, round(0.85 / 0.05))

    def test_grows_and_survives_a_point_outside_the_memory_cap(self):
        g = Grid(size_m=2.0, max_cells=200 * 200)
        g.integrate_scan(0.0, 0.0, [[(5000.0, 0.0), (5000.0, 100.0), (50.0, 300.0)]])    # must not raise
        g2 = Grid(size_m=2.0)
        g2.integrate_scan(0.0, 0.0, [[(5000.0, 0.0), (5000.0, 100.0), (5000.0, 200.0)]])
        self.assertGreater(g2.value(5000, 0), UNKNOWN)


class TestMapping(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_room_fills_in(self):
        rig = Rig(self.tmp)
        rig.run(1.5)
        g = rig.mapper.grid
        L = rig.lidar
        self.assertGreaterEqual(L.n["used"], 12)
        self.assertGreater(cell_max(g, 4000, 0), UNKNOWN + 30)       # far wall straight ahead (x = 4000)
        self.assertGreater(cell_max(g, -1000, 0), UNKNOWN + 30)      # wall behind
        self.assertGreater(cell_max(g, 500, 2000), UNKNOWN + 30)     # left wall (x = 0 is the masked cage post)
        self.assertGreater(cell_max(g, 2000, -1500), UNKNOWN + 30)   # right wall (the couch hides it at x = 500)
        self.assertGreater(cell_max(g, 1200, 900), UNKNOWN + 30)     # the box's near face
        for x, y in ((500, 0), (2500, 0), (1000, -600), (1000, 1500), (-500, 700)):
            self.assertLess(g.value(x, y), UNKNOWN - 10, (x, y))     # free floor
        self.assertEqual(g.value(6000, 0), UNKNOWN)                  # outside the room: never seen
        self.assertLess(g.value(1500, 900), UNKNOWN + 60)            # the box's inside is not 'free' proof, just not a ray target
        # a wall cell is a thin line: nothing occupied a metre in front of the far wall
        self.assertLess(g.value(3000, 0), UNKNOWN)
        self.assertGreater(L.status()["cost_ms"], 0)
        self.assertTrue(L.flowing())

    def test_mask_and_range_filters(self):
        rig = Rig(self.tmp, mask=[], min_range_mm=0)                  # no mask, no min range: the cage posts show up
        rig.run(1.0)
        g = rig.mapper.grid
        self.assertGreater(cell_max(g, 0, 70, 20), UNKNOWN + 20)     # post at robot +90 deg, 55 mm out (cell 70 mm covers it)
        tmp2 = tempfile.mkdtemp()
        try:
            rig2 = Rig(tmp2, mask=[[85, 95], [265, 275]], min_range_mm=0)   # masked: gone
            rig2.run(1.0)
            self.assertLess(cell_max(rig2.mapper.grid, 0, 70, 20), UNKNOWN + 5)
            self.assertGreater(rig2.lidar.n["mask"], 0)
            tmp3 = tempfile.mkdtemp()
            rig3 = Rig(tmp3, mask=[], min_range_mm=120)                      # the min range alone also drops them
            rig3.run(1.0)
            self.assertLess(cell_max(rig3.mapper.grid, 0, 70, 20), UNKNOWN + 5)
            self.assertGreater(rig3.lidar.n["range"], 0)
            shutil.rmtree(tmp3, ignore_errors=True)
        finally:
            shutil.rmtree(tmp2, ignore_errors=True)

    def test_max_range_and_intensity(self):
        rig = Rig(self.tmp, max_range_mm=2500, min_intensity=0)
        rig.run(1.0)
        g = rig.mapper.grid
        self.assertEqual(g.value(3900, 0), UNKNOWN)                  # the far wall (4 m) is past max_range
        self.assertGreater(rig.lidar.n["range"], 0)
        tmp = tempfile.mkdtemp()
        try:
            rig = Rig(tmp, min_intensity=255)                        # nothing is that bright
            rig.run(0.5)
            self.assertEqual(rig.lidar.n["used"], 0)
            self.assertGreater(rig.lidar.n["intensity"], 1000)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_mount_offset_yaw_and_flip(self):
        # sensor turned 90 deg on the robot (its front points left) and 40 mm forward: matching sim + settings
        for flip in (False, True):
            tmp = tempfile.mkdtemp()
            try:
                sim = LidarSim(seed=1, yaw_deg=90.0, x_mm=40.0, y_mm=-25.0, flip=flip, posts=False)
                rig = Rig(tmp, sim=sim, yaw_deg=90.0, x_mm=40.0, y_mm=-25.0, flip=flip, mask=[])
                rig.run(1.0)
                self.assertGreater(cell_max(rig.mapper.grid, 4000, 0), UNKNOWN + 30, flip)
                self.assertGreater(cell_max(rig.mapper.grid, 0, 2000), UNKNOWN + 30, flip)
                # a wrong yaw setting puts the walls in the wrong place (so the test would notice)
                tmp2 = tempfile.mkdtemp()
                bad = Rig(tmp2, sim=LidarSim(seed=1, yaw_deg=90.0, x_mm=40.0, y_mm=-25.0, flip=flip, posts=False), yaw_deg=0.0, mask=[])
                bad.run(1.0)
                shutil.rmtree(tmp2, ignore_errors=True)
                self.assertLess(bad.mapper.grid.value(0, 2000), UNKNOWN + 30 if bad.lidar.n["used"] else 1000)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    def test_clockwise_convention(self):
        """A feature on the robot's LEFT (+y) must land on +y: the LD19 counts clockwise, the map is counter-clockwise."""
        from station.sim.sim import Room
        room = Room(empty=True)
        room.segs = [(-4000, -4000, 4000, -4000), (4000, -4000, 4000, 4000), (4000, 4000, -4000, 4000), (-4000, 4000, -4000, -4000),
                     (500, 300, 500, 900)]            # a short wall ahead-left only
        r = FakeRobot(room)
        rig = Rig(self.tmp, robot=r, sim=LidarSim(seed=1, posts=False), mask=[])
        rig.run(1.0)
        g = rig.mapper.grid
        self.assertGreater(cell_max(g, 500, 600), UNKNOWN + 30)
        self.assertLess(cell_max(g, 500, -600), UNKNOWN)             # the mirror image is free, not wall

    def test_pitch_gate_skip(self):
        rig = Rig(self.tmp, pitch=lambda r: 4.0, pitch_auto_ref=False, pitch_ref_deg=0.0)     # leaning 4 deg > 2.5
        rig.run(1.0)
        self.assertEqual(rig.lidar.n["used"], 0)
        self.assertGreater(rig.lidar.n["tilt"], 3000)
        self.assertEqual(rig.mapper.n["scans"], 0)
        tmp = tempfile.mkdtemp()
        try:
            rig = Rig(tmp, pitch=lambda r: 1.0, pitch_auto_ref=False, pitch_ref_deg=0.0)       # 1 deg is fine
            rig.run(1.0)
            self.assertGreater(rig.lidar.n["used"], 5)
            self.assertEqual(rig.lidar.n["tilt"], 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_pitch_ref_follows_the_typical_pitch(self):
        """The IMU reads -3 deg when balanced: a constant -3.15 must not gate anything."""
        rig = Rig(self.tmp, pitch=lambda r: -3.15)
        rig.run(1.0)
        self.assertGreater(rig.lidar.n["used"], 5)
        self.assertAlmostEqual(rig.lidar.tilt_ref, -3.15, places=2)
        tmp = tempfile.mkdtemp()
        try:                                                           # lying down / carried: never
            rig = Rig(tmp, pitch=lambda r: 55.0)
            rig.run(0.5)
            self.assertEqual(rig.lidar.n["used"], 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_sway_drops_only_the_tilted_packets(self):
        """A robot rocking +-4 deg at 2 Hz: packets near the extremes go, the ones near upright map."""
        rig = Rig(self.tmp, pitch=lambda r: 4.0 * math.sin(2 * math.pi * 2 * r.t))
        rig.run(2.0)
        L = rig.lidar
        share = L.n["tilt"] / max(1, L.n["points"])
        self.assertGreater(share, 0.15)
        self.assertLess(share, 0.75)
        self.assertGreater(L.n["used"], 8)

    def test_floor_hits_are_clipped(self):
        """Robot leaning 2 deg (inside the 2.5 deg gate), scan plane 100 mm up: the front beams meet the floor at
        100 / tan(2 deg) = 2.9 m. Without the floor clip that is a 'wall' across the room."""
        def lean(r):
            r.theta = math.radians(2.0)
            return 2.0
        for clip, expect_wall in ((True, False), (False, True)):
            tmp = tempfile.mkdtemp()
            try:
                rig = Rig(tmp, sim=LidarSim(seed=1, height_mm=100.0, posts=False, dropout=0.0), pitch=lean,
                          pitch_auto_ref=False, height_mm=100.0, floor_clip=clip, mask=[])
                rig.run(1.0)
                seen = cell_max(rig.mapper.grid, 2860, 0, 25) > UNKNOWN + 20
                self.assertEqual(seen, expect_wall, "floor_clip=%s" % clip)
                self.assertGreater(rig.lidar.n["used"], 5)
                if clip:
                    self.assertGreater(rig.lidar.n["floor"], 100)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    def test_project_mode_keeps_tilted_packets(self):
        rig = Rig(self.tmp, pitch=lambda r: 4.0, pitch_auto_ref=False, tilt_mode="project", height_mm=2000.0)
        rig.run(1.0)
        self.assertGreater(rig.lidar.n["used"], 5)                    # 4 deg is under pitch_max_deg
        self.assertEqual(rig.lidar.n["tilt"], 0)

    def test_deskew_sharpens_a_moving_scan(self):
        def spin(r, dt):                      # 120 deg/s turn while driving 300 mm/s: ~12 deg smear per scan
            r.TH += 120.0 * dt
            r.X += 300.0 * dt * math.cos(math.radians(r.TH))
            r.Y += 300.0 * dt * math.sin(math.radians(r.TH))
        err = {}
        for deskew in (True, False):
            tmp = tempfile.mkdtemp()
            try:
                r = FakeRobot()
                r.X, r.Y = 500.0, 200.0
                rig = Rig(tmp, robot=r, sim=LidarSim(seed=1, posts=False, dropout=0.0, noise_mm=0.0), deskew=deskew, mask=[],
                          min_range_mm=0)
                rig.mover = spin
                rig.run(0.6)
                pts = rig.lidar.last_pts
                self.assertGreater(len(pts), 300)
                err[deskew] = sum(r.room.clearance(x, y) for x, y in pts) / len(pts)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
        self.assertLess(err[True], 25.0)                              # points sit on the walls (to ~cell size)
        self.assertGreater(err[False], 2 * err[True])                 # the smear is gone with deskew

    def test_decimation_settings(self):
        rig = Rig(self.tmp, every_n=3, ray_step=2)
        rig.run(1.2)
        n = rig.lidar.n
        self.assertEqual(n["scans"], rig.scans)
        self.assertAlmostEqual(n["used"], rig.scans / 3.0, delta=1.01)
        self.assertEqual(n["used"] + n["every_n"], n["scans"])
        self.assertGreater(cell_max(rig.mapper.grid, 4000, 0, 150), UNKNOWN + 20)

    def test_no_pose_no_map(self):
        rig = Rig(self.tmp, tlm_hz=0.001)                              # no telemetry reaches the lidar
        rig.next_tlm = 1e9
        rig.run(0.5)
        self.assertEqual(rig.lidar.n["used"], 0)
        self.assertGreater(rig.lidar.n["no_pose"], 2)

    def test_sweep_is_ignored_while_the_lidar_flows(self):
        rig = Rig(self.tmp)
        d = {"pitch_deg": 0.0, "range_mm": 800, "valid": 1, "x_mm": 0.0, "y_mm": 0.0, "heading_deg": 0.0,
             "sensor_angle_deg": 0.0}
        m = rig.mapper
        m.on_range(d)                                                  # nothing flows yet: the sweep draws
        self.assertEqual(m.n["used"], 1)
        rig.run(0.6)
        used = m.n["used"]
        m.on_range(d)
        self.assertEqual(m.n["used"], used)
        self.assertEqual(m.n["sweep_skipped"], 1)
        m.apply_settings({"lidar_replaces_sweep": False}, save=False)  # the switch
        m.on_range(d)
        self.assertEqual(m.n["used"], used + 1)
        rig.r.t += 5.0                                                 # the lidar stopped: the sweep takes over again
        m.apply_settings({"lidar_replaces_sweep": True}, save=False)
        m.on_range(d)
        self.assertEqual(m.n["used"], used + 2)

    def test_cost_per_scan(self):
        rig = Rig(self.tmp)
        rig.run(2.0)
        c = sorted(rig.cost[5:])                                       # skip the first scans (grid growth)
        ms = 1000.0 * sum(c) / len(c)
        print("\n[lidar] %d scans, mean %.2f ms, median %.2f ms, max %.2f ms per scan (every_n=1, ray_step=1)"
              % (len(c), ms, 1000 * c[len(c) // 2], 1000 * c[-1]))
        self.assertLess(ms, 60.0)
        rig2 = Rig(tempfile.mkdtemp(), ray_step=2)
        rig2.run(2.0)
        c2 = sorted(rig2.cost[5:])
        print("[lidar] ray_step=2: mean %.2f ms" % (1000.0 * sum(c2) / len(c2)))
        shutil.rmtree(rig2.mapper.dir, ignore_errors=True)


class TestPoseJump(unittest.TestCase):
    """#99: an odometry jump (pose reset; its log line may come after the telemetry) clears the pose history, so a scan
    is never interpolated across it."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.L = Rig(self.tmp).lidar

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def tlm(self, t, x, y, h, w=0.0):
        self.L.on_tlm({"x_mm": x, "y_mm": y, "heading_deg": h, "pitch_deg": 0.0, "yaw_rate_dps": w}, t)

    def test_jumps_clear_the_history(self):
        L = self.L
        for i in range(50):                             # driving 1 m/s and turning 200 deg/s at 100 Hz: no jump
            self.tlm(i * 0.01, 1500.0 + 10.0 * i, 0.0, 2.0 * i, 200.0)
        self.assertEqual(len(L.hist.items()), 50)
        self.assertEqual(L.n["pose_jumps"], 0)
        self.tlm(0.50, 0.0, 0.0, 0.0)                   # pose reset 2 m away
        self.assertEqual(L.n["pose_jumps"], 1)
        self.assertEqual(L.hist.items(), [(0.50, 0.0, 0.0, 0.0, 0.0)])
        for i in range(1, 10):
            self.tlm(0.50 + i * 0.01, 2.0 * i, 0.0, 0.0)
        self.tlm(0.60, 20.0, 0.0, 120.0)                # heading step the yaw rate (0) does not explain: a reset
        self.assertEqual(L.n["pose_jumps"], 2)
        self.assertEqual(len(L.hist.items()), 1)
        self.tlm(0.61, 21.0, 0.0, 380.0)                # +260 = -100 wrapped: still a jump
        self.assertEqual(L.n["pose_jumps"], 3)
        self.tlm(1.62, 300.0, 0.0, 380.0)               # 1 s gap, 279 mm: travel, not a jump
        self.assertEqual(L.n["pose_jumps"], 3)
        self.assertEqual(len(L.hist.items()), 2)


if __name__ == "__main__":
    unittest.main()
