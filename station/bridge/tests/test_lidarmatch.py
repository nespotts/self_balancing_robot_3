"""LD19 scan-to-map matching (issue #78): likelihood field, correlative match, degeneracy, filters, Mapper / lidar wiring,
and a short sim run with drifting odometry (the long evaluation: python -m station.sim.lidar_eval)."""
import math
import random
import shutil
import tempfile
import unittest

from station.bridge import lidarmatch as lm
from station.bridge.mapping import DEFAULTS, Mapper
from station.bridge.occgrid import Grid
from station.sim.sim import Room


class Segs:
    """A room made of wall segments (raycast like sim.Room)."""

    def __init__(self, segs):
        self.segs = segs

    raycast = Room.raycast


def corridor():
    return Segs([(-12000, -500, 12000, -500), (-12000, 500, 12000, 500)])     # its ends are out of reach


def scan(room, x, y, h, rng, n=450, noise=8.0, max_d=8000):
    """One 360 deg turn from pose (x, y, h deg): world points in angle order (CCW)."""
    pts = []
    for k in range(n):
        a = math.radians(h) + 2 * math.pi * k / n
        d = room.raycast(x, y, a, max_d)
        if d is not None:
            d += rng.gauss(0, noise)
            pts.append((x + d * math.cos(a), y + d * math.sin(a)))
    return pts


def seen_from_guess(pts, true, guess=(0.0, 0.0, 0.0)):
    """Points measured at the true pose, placed in the map with the guessed pose (what odometry error does)."""
    tx, ty, tt = true
    gx, gy, gt = guess
    c, s = math.cos(math.radians(-tt)), math.sin(math.radians(-tt))
    rel = [(c * (x - tx) - s * (y - ty), s * (x - tx) + c * (y - ty)) for x, y in pts]
    c, s = math.cos(math.radians(gt)), math.sin(math.radians(gt))
    return [(c * x - s * y + gx, s * x + c * y + gy) for x, y in rel]


def built_grid(room, rng, poses=((0, 0, 0),), n=8):
    g = Grid()
    for x, y, h in poses:
        for _ in range(n):
            g.integrate_scan(x, y, [scan(room, x, y, h, rng)], margin_mm=25)
    return g


class TestField(unittest.TestCase):
    def test_values_and_incremental_updates_equal_a_rebuild(self):
        g = Grid(size_m=4.0)
        for x in range(-1000, 1000, 10):
            for _ in range(3):
                g.integrate(0, 0, [], hit=(x, 800.0))       # a wall along y = 800
        f = lm.Field()
        f.sync(g)
        self.assertGreater(f.walls, 50)
        i = g.index(0, 800)
        wp = f.Wp

        def fv(x, y):
            ix, iy = g.cell_of(x, y)
            return f.f[(iy + lm.PAD) * wp + ix + lm.PAD]
        self.assertEqual(fv(0, 810), 255)
        self.assertTrue(255 > fv(0, 840) > fv(0, 870) > 0)            # falls off away from the wall
        self.assertEqual(fv(0, 1000), 0)
        self.assertIsNotNone(i)
        # new walls: added in place, only changed tiles looked at
        for y in range(-1000, 1000, 10):
            for _ in range(3):
                g.integrate(0, 0, [], hit=(-600.0, y))
        f.sync(g)
        # a wall goes away (observed free): that tile is recomputed
        for x in range(-1000, 1000, 10):
            for _ in range(12):
                g.integrate(x, 700.0, [(x, 900.0)])
        f.sync(g)
        ref = lm.Field()
        ref.sync(g)
        self.assertEqual(f.walls, ref.walls)
        self.assertEqual(bytes(f.f), bytes(ref.f))
        self.assertEqual(fv(0, 810), 0)                              # erased
        self.assertEqual(f.rebuilds, 1)

    def test_grid_reset_or_load_rebuilds(self):
        rng = random.Random(1)
        g = built_grid(Room(), rng, n=3)
        f = lm.Field()
        f.sync(g)
        w = f.walls
        g.reset()
        f.sync(g)
        self.assertEqual((f.walls, f.rebuilds), (0, 2))
        g2 = Grid.from_json(built_grid(Room(), rng, n=3).to_json())
        f.sync(g2)
        self.assertEqual(f.rebuilds, 3)
        self.assertAlmostEqual(f.walls, w, delta=0.1 * w)


class TestThin(unittest.TestCase):
    def test_even_spacing_and_direction_does_not_matter(self):
        rng = random.Random(2)
        run = [(x * 7.0 + rng.uniform(-2, 2), 0.0) for x in range(300)]     # 2.1 m, ~7 mm apart
        a = lm.thin_runs([run], 1000)
        b = lm.thin_runs([run[::-1]], 1000)
        self.assertEqual(sorted(a), sorted(b))
        xs = sorted(p[0] for p in a)
        gaps = [q - p for p, q in zip(xs, xs[1:])]
        self.assertTrue(all(abs(gp - 50) <= 9 for gp in gaps), gaps)
        self.assertAlmostEqual(xs[0] - run[0][0], run[-1][0] - xs[-1], delta=9)         # centred
        self.assertLessEqual(len(lm.thin_runs([run], 20)), 21)        # the cap widens the spacing


class TestMatch(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(4)

    def match_at(self, room, g, true, max_d=8000, **kw):
        f = lm.Field()
        f.sync(g)
        p = seen_from_guess(scan(room, true[0], true[1], true[2], self.rng, max_d=max_d), true)
        return lm.match(f, lm.thin_runs([p], 150), 0.0, 0.0, **kw)

    def test_recovers_a_known_offset(self):
        room = Room()
        g = built_grid(room, self.rng)
        for true in ((40, -30, 1.5), (-70, 60, -2.2), (20, 10, 0.5), (0, 0, 0)):
            r = self.match_at(room, g, true)
            self.assertTrue(r["ok"], r)
            self.assertLess(math.hypot(r["dx"] - true[0], r["dy"] - true[1]), 12, r)
            self.assertLess(abs(r["dth"] - true[2]), 0.4, r)
            self.assertIsNone(r["along"])
            self.assertGreater(r["score"], 0.8)
            self.assertLess(r["ms"], 50)

    def test_corridor_is_corrected_across_only(self):
        room = corridor()
        g = Grid()                                   # mapped along it, farther than the probe turn reaches
        for x in range(-3000, 3001, 500):
            for _ in range(3):
                g.integrate_scan(x, 0, [scan(room, x, 0, 0, self.rng, max_d=5000)], margin_mm=25)
        r = self.match_at(room, g, (60, -30, 1.0), max_d=5000)
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["why"], "corridor: not along it")
        self.assertAlmostEqual(abs(r["along"][0]), 1.0, delta=0.05)    # loose along x
        self.assertLess(abs(r["dx"]), 5)                                # not "corrected" along the corridor
        self.assertAlmostEqual(r["dy"], -30, delta=8)
        self.assertAlmostEqual(r["dth"], 1.0, delta=0.3)

    def test_out_of_the_window_is_not_a_match(self):
        room = Room()
        g = built_grid(room, self.rng)
        r = self.match_at(room, g, (300, 0, 0))
        self.assertFalse(r["ok"], r)

    def test_unmapped_ground_is_not_matched(self):
        g = built_grid(Room(), self.rng, n=2)
        f = lm.Field()
        f.sync(g)
        far = Segs([(-2000 + 9000, -1500, 4000 + 9000, -1500), (4000 + 9000, -1500, 4000 + 9000, 2000)])
        p = scan(far, 9000, 0, 0, self.rng)
        r = lm.match(f, lm.thin_runs([p], 150), 9000.0, 0.0)
        self.assertFalse(r["ok"])
        self.assertTrue(r["why"].startswith(("mostly new ground", "no wall")), r["why"])

    def test_featureless_open_space_is_rejected(self):
        # one short wall stub far away: nothing to pin x / y / heading on
        room = Segs([(1500, -60, 1500, 60)])
        g = built_grid(room, self.rng)
        r = self.match_at(room, g, (20, 0, 0))
        self.assertFalse(r["ok"], r)

    def test_ambiguous_repeating_structure_is_rejected(self):
        # a grating: walls every 100 mm across y, seen as points (no raycast): any 100 mm shift fits as well
        g = Grid()
        pts = [(x, y) for y in range(-1000, 1001, 100) for x in range(-1500, 1500, 20)]
        for _ in range(4):
            for x, y in pts:
                g.integrate(0, 0, [], hit=(x + 0.5, y + 0.5))
        f = lm.Field()
        f.sync(g)
        probe = [(x + 10.0, y + 45.0) for x, y in pts[::7]]
        r = lm.match(f, probe[:400], 0.0, 0.0, window_mm=150)
        self.assertFalse(r["ok"], r)


class TestLidarMatch(unittest.TestCase):
    """The filter / acceptance layer and its Mapper wiring, fed turns directly."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(map_dir=self.tmp)
        self.m.apply_settings({"match_every_n": 1}, save=False)
        self.room = Room()
        self.rng = random.Random(7)
        self.t = 0.0

        class L:                                   # stands in for lidar.Lidar (the matcher uses its clock)
            clock = staticmethod(lambda: self.t)
        self.m.lidar = L()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def turn(self, true, guess):
        """One turn measured at the true pose, placed with corr o guess (odometry); mapped like lidar.py does."""
        m = self.m
        self.t += 0.1
        gx, gy, gh = m.map_pose(*guess)
        pts = seen_from_guess(scan(self.room, true[0], true[1], true[2], self.rng), true, (gx, gy, gh))
        ox, oy, mx, my, runs, pts = m.lmatch.on_scan(gx, gy, gx, gy, gh, [pts], pts)
        if runs is not None:
            m.integrate_scan(ox, oy, runs, (mx, my))
        return runs is not None

    def test_warm_up_then_drift_is_pulled_back(self):
        m = self.m
        for _ in range(DEFAULTS["match_warmup"]):
            self.assertTrue(self.turn((0, 0, 0), (0, 0, 0)))              # warm-up: map only
        self.assertEqual(m.corr, [0.0, 0.0, 0.0])
        self.assertEqual(m.lmatch.state, "building the map")
        # odometry now 40 mm / 1.2 deg off (and stays so): the correction converges on it
        for _ in range(40):
            self.turn((0, 0, 0), (40.0, -20.0, 1.2))
        x, y, h = m.map_pose(40.0, -20.0, 1.2)
        self.assertLess(math.hypot(x, y), 12, m.corr)
        self.assertLess(abs(h), 0.25, m.corr)
        st = m.stats()["lmatch"]
        self.assertTrue(st["on"])
        self.assertGreater(st["matched"], 30)
        self.assertTrue(st["state"].startswith("matched"), st)
        self.assertIsNotNone(m.places.origin)                            # the default Home stays put in the map
        self.assertAlmostEqual(m.places.origin[0], 0.0, delta=1)

    def test_correct_pose_is_left_alone(self):
        m = self.m
        for _ in range(40):
            self.turn((0, 0, 0), (0, 0, 0))
        self.assertLess(math.hypot(m.corr[0], m.corr[1]), 3)
        self.assertLess(abs(m.corr[2]), 0.05)

    def test_a_jump_waits_for_agreeing_matches_and_steps_are_rate_limited(self):
        m = self.m
        for _ in range(12):
            self.turn((0, 0, 0), (0, 0, 0))
        moved = []
        for _ in range(8):
            self.turn((0, 0, 0), (90.0, 0.0, 0.0))                       # 90 mm wheel slip
            moved.append(m.corr[0])
        self.assertEqual(moved[:2], [0.0, 0.0])                          # not on the first matches ...
        self.assertLess(moved[2], -1)                                    # ... on the 3rd that agrees
        steps = [a - b for a, b in zip(moved, moved[1:])]
        self.assertLessEqual(max(steps), DEFAULTS["match_step_mm"] + 0.5)
        self.assertEqual(m.lmatch.n["jumps"], 1)
        self.assertLess(abs(m.map_pose(90.0, 0, 0)[0]), 25)

    def test_map_updates_only_after_moving_or_a_while(self):
        m = self.m
        for _ in range(12):
            self.turn((0, 0, 0), (0, 0, 0))
        mapped = [self.turn((0, 0, 0), (0, 0, 0)) for _ in range(30)]     # 3 s standing still
        self.assertLessEqual(sum(mapped), 1)
        mapped = [self.turn((k * 30.0, 0, 0), (k * 30.0, 0, 0)) for k in range(1, 21)]   # driving 600 mm
        self.assertGreaterEqual(sum(mapped), 2)
        self.assertLessEqual(sum(mapped), 6)

    def test_off_maps_every_turn_and_does_not_touch_the_pose(self):
        m = self.m
        m.apply_settings({"lidar_match": False}, save=False)
        for _ in range(20):
            self.assertTrue(self.turn((0, 0, 0), (40.0, 0, 1.0)))
        self.assertEqual(m.corr, [0.0, 0.0, 0.0])
        self.assertEqual(m.lmatch.state, "off")

    def test_settings_persist_and_reset(self):
        m = self.m
        m.command({"action": "set", "settings": {"lidar_match": False, "match_every_n": 3.4, "match_window_mm": 9999}})
        m2 = Mapper(map_dir=self.tmp)
        self.assertIs(m2.settings["lidar_match"], False)
        self.assertEqual(m2.settings["match_every_n"], 3)
        self.assertEqual(m2.settings["match_window_mm"], 300)
        for _ in range(15):
            self.turn((0, 0, 0), (0, 0, 0))
        m.reset()
        self.assertEqual((m.lmatch.turns, m.lmatch.n["matched"], m.lmatch.last), (0, 0, None))


class TestPlannerDrift(unittest.TestCase):
    def test_the_route_is_resent_once_the_correction_moved_it(self):
        from station.sim.tour import DriftRobot, OfflineBridge
        tmp = tempfile.mkdtemp()
        try:
            r = DriftRobot(seed=1)
            m = Mapper(tmp)
            b = OfflineBridge(r, m)
            b.update()
            p = b.planner
            p.mode, p.path = "goto", [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
            p._send(p.path[1:], False)
            b.nav = {"nav_index": 0, "waypoint_count": 2}
            r.t += 2.0
            m.corr = [10.0, 0.0, 0.0]
            for f in m.corr_listeners:
                f()
            self.assertEqual(b.waypoints, [(1000.0, 0.0), (1000.0, 1000.0)])     # 10 mm: not worth a resend
            m.corr = [30.0, 0.0, 0.0]
            p.on_drift()
            self.assertEqual(b.waypoints, [(970.0, 0.0), (970.0, 1000.0)])       # in the new odometry frame
            m.corr = [80.0, 0.0, 0.0]
            p.on_drift()
            self.assertEqual(b.waypoints[0], (970.0, 0.0))                       # at most once a second
            p.mode = "idle"
            r.t += 2.0
            p.on_drift()
            self.assertEqual(b.waypoints[0], (970.0, 0.0))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestSimDrift(unittest.TestCase):
    """Sim end to end: real LD19 packets (lidar_sim), the bridge's lidar.py + matcher + planner, odometry drifting
    6 deg/min and 3 % long; the robot drives part of the tour. With matching the pose error vs the sim's truth stays
    small and the walls stay thin. (Numbers for the whole tour: python -m station.sim.lidar_eval.)"""

    def drive(self, match):
        from station.sim.lidar_eval import TOUR, Run, wall_stats
        e = Run(5, match, drift=6.0, scale=0.03)
        try:
            e.stand_up()
            for p in TOUR[:4]:
                e.drive(lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False), max_s=40.0)
            fit, n_occ, _ = wall_stats(e.m, e.r.room, e.F)
            return max(e.errs), e.herrs[-1], fit, n_occ, e.m.lmatch, e.match_ms
        finally:
            e.close()

    def test_matching_bounds_the_drift_and_keeps_the_map_sharp(self):
        off = self.drive(False)
        on = self.drive(True)
        self.assertGreater(off[0], 80)                      # odometry alone: off by ~0.1 m and 5 deg in ~40 s
        self.assertLess(on[0], 50, on[:4])                  # matched: bounded
        self.assertLess(on[0], off[0] / 2)
        self.assertLess(on[1], 1.0)
        self.assertLess(on[1], off[1] / 3)
        self.assertGreater(on[2], 0.9)                      # walls where the real walls are ...
        self.assertGreater(on[2], off[2] + 0.2)
        self.assertLess(on[3], off[3])                      # ... and thinner
        lmt = on[4]
        self.assertGreater(lmt.n["matched"], 10 * max(1, lmt.n["rejected"]))
        self.assertLess(sorted(on[5])[len(on[5]) // 2], 25.0)   # ms per match on this PC (Pi: ~10 x)


if __name__ == "__main__":
    unittest.main()
