"""Relocalisation (issue #105): the whole-map branch-and-bound search (reloc.py), its triggers in LidarMatch (lost > 5 s,
a Teensy reboot / pose reset, a map loaded, "Find myself"), the ambiguity check (a symmetric room), Set position, and
the worker thread keeping the event loop free. (Success rate / CPU over many kidnaps: python -m station.sim.reloc_eval.)"""
import asyncio
import math
import os
import random
import shutil
import sys
import tempfile
import time
import unittest

from station.bridge import lidarmatch as lm
from station.bridge import reloc
from station.bridge.mapping import Mapper
from station.bridge.tests.test_lidarmatch import Segs, Turns, built_grid, scan, seen_from_guess
from station.sim.sim import Room, _box


def ahead(p, d):
    return p[0] + d * math.cos(math.radians(p[2])), p[1] + d * math.sin(math.radians(p[2]))


class TestPyramid(unittest.TestCase):
    def test_levels_are_block_maxima_and_the_free_mask_is_where_the_robot_was(self):
        g = built_grid(Room(), random.Random(1), n=3)
        f = lm.Field()
        f.sync(g)
        pyr = reloc.Pyramid(reloc.Frozen(f))
        rng = random.Random(2)
        Wq, P = pyr.Wq, pyr.P
        for h in range(1, pyr.L + 1):
            s = 1 << h
            for _ in range(300):
                I, J = rng.randrange(P - s, P + pyr.Wc), rng.randrange(P - s, P + pyr.Hc)
                want = max(pyr.f[0][(J + dj) * Wq + I + di] for dj in range(s) for di in range(s))
                self.assertEqual(pyr.f[h][J * Wq + I], want, (h, I, J))
        # coarse cells hold the max of their 4 x 4 field cells
        i, j = pyr.cell(4000.0, 0.0)                  # on the far wall
        self.assertGreater(pyr.f[0][j * Wq + i], 200)
        i, j = pyr.cell(0.0, 0.0)                     # the robot's spot: free, nothing to match
        self.assertEqual(pyr.free[0][j * Wq + i], 1)
        self.assertLess(pyr.f[0][j * Wq + i], 50)
        i, j = pyr.cell(9000.0, 0.0)                  # out beyond the walls: never seen
        self.assertEqual(pyr.free[0][j * Wq + i], 0)

    def test_branch_and_bound_finds_the_exhaustive_best(self):
        g = built_grid(Room(), random.Random(3), n=3)
        f = lm.Field()
        f.sync(g)
        fz = reloc.Frozen(f)
        pyr = reloc.Pyramid(fz)
        rng = random.Random(4)
        true = (300.0, -200.0, 35.0)
        pts = seen_from_guess(scan(Room(), *true, rng), true, (0.0, 0.0, 0.0))
        rel = lm.thin_runs([pts], 60)
        rots = reloc.rotations(6.0)
        box = pyr.free_box
        leaves, nodes, complete = reloc.bnb(pyr, rel, rots, box, min_score=0.0, prune_f=1.0)
        self.assertTrue(complete)
        best = max(leaves)
        # brute force over the same lattice
        Wq, R = pyr.Wq, pyr.R
        top = 0
        for k, th in enumerate(rots):
            c, s = math.cos(th), math.sin(th)
            offs = [int(math.floor(0.5 + (c * u - s * v) / R)) + Wq * int(math.floor(0.5 + (s * u + c * v) / R))
                    for u, v in rel]
            for J in range(box[1], box[3] + 1):
                for I in range(box[0], box[2] + 1):
                    if pyr.free[0][J * Wq + I]:
                        top = max(top, sum(pyr.f[0][J * Wq + I + o] for o in offs))
        self.assertEqual(best[0], top)
        n_all = len(rots) * (box[2] - box[0] + 1) * (box[3] - box[1] + 1)
        self.assertLess(nodes, n_all / 20, "the bound prunes most of the lattice")

    def test_yielder_cancels(self):
        n = [0]

        def cancel():
            n[0] += 1
            return n[0] > 2
        y = reloc.Yielder(slice_s=0.0, pause_s=0.0, cancel=cancel)
        with self.assertRaises(reloc.Cancelled):
            for _ in range(10):
                y()


class TestFind(unittest.TestCase):
    """reloc.find() on a map, kidnapped to several poses and headings."""

    def setUp(self):
        self.room = Room()
        self.rng = random.Random(7)
        g = built_grid(self.room, self.rng, poses=((0, 0, 0), (2600, 200, 90), (-300, -700, 0)), n=3)
        f = lm.Field()
        f.sync(g)
        self.fz = reloc.Frozen(f)
        self.pyr = reloc.Pyramid(self.fz)

    def test_kidnapped_anywhere_at_any_heading(self):
        for true in ((2700, 300, 140), (-500, 0, -100), (1000, -500, 10), (3400, 1500, 200), (200, 1600, -45)):
            guess = (1234.0, -321.0, 77.0)
            pts = seen_from_guess(scan(self.room, *true, self.rng), true, guess)
            r = reloc.find(self.fz, self.pyr, [pts], guess[0], guess[1])
            self.assertTrue(r["ok"], (true, r))
            self.assertLess(math.hypot(r["x"] - true[0], r["y"] - true[1]), 40, (true, r))
            self.assertLess(abs((guess[2] + r["dth"] - true[2] + 180) % 360 - 180), 1.0, (true, r))
            self.assertGreaterEqual(r["margin"], reloc.ACCEPT["margin"])

    def test_symmetric_room_is_ambiguous(self):
        room = Segs(_box(0, 0, 4000, 3000))
        rng = random.Random(8)
        g = built_grid(room, rng, poses=((0, 0, 0), (-1000, 500, 0)), n=3)
        f = lm.Field()
        f.sync(g)
        fz = reloc.Frozen(f)
        pyr = reloc.Pyramid(fz)
        for true in ((700, -400, 30), (-1200, 600, 170), (0, 0, 90)):
            pts = seen_from_guess(scan(room, *true, rng), true, (0.0, 0.0, 0.0))
            r = reloc.find(fz, pyr, [pts], 0.0, 0.0)
            self.assertFalse(r["ok"], r)
            self.assertEqual(r["why"], "ambiguous", r)
            self.assertGreaterEqual(len(r["cands"]), 2)
            # the twins: 180 deg apart, mirrored through the room's centre
            a, b = r["cands"][:2]
            self.assertLess(math.hypot(a[0] + b[0], a[1] + b[1]), 80, r["cands"])
            self.assertAlmostEqual(abs((a[2] - b[2]) % 360 - 180), 0, delta=2)

    def test_set_position_window(self):
        true = (2700, 300, 140)
        guess = (2550.0, 420.0, 155.0)                # the user's tap: 190 mm / 15 deg off
        pts = seen_from_guess(scan(self.room, *true, self.rng), true, guess)
        r = reloc.find(self.fz, self.pyr, [pts], guess[0], guess[1], around=(guess[0], guess[1], 600, 25),
                       accept={"margin": 0.05})
        self.assertTrue(r["ok"], r)
        self.assertLess(math.hypot(r["x"] - true[0], r["y"] - true[1]), 40, r)
        self.assertLess(r["rots"], 40)                # only +-25 deg
        far = reloc.find(self.fz, self.pyr, [pts], guess[0], guess[1], around=(-500, 1500, 600, 25),
                         accept={"margin": 0.05})
        self.assertFalse(far["ok"], far)              # nothing like it there


class Reloc(Turns):
    """LidarMatch with the #105 triggers, turns fed straight in (Turns, test_lidarmatch)."""

    def walk(self, a, b, steps):
        """True pose = odometry from a to b (both (x, y, deg)) in steps turns."""
        for k in range(1, steps + 1):
            f = k / float(steps)
            p = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]))
            self.turn(p, p)
        return b

    def head_err(self, true, guess):
        return abs((self.m.map_pose(*guess)[2] - true[2] + 180) % 360 - 180)

    def build_room(self):
        self.build(25)
        p = self.walk((0, 0, 0), (1500, 200, 60), 30)
        p = self.walk(p, (2600, 300, 120), 25)
        return p


class TestTriggers(Reloc):
    def test_carried_away_is_found_after_5_s(self):
        self.build_room()
        mapped0 = self.mapped
        true, odo = (-400.0, -200.0, -120.0), (2600.0, 300.0, 120.0)     # carried 3 m and turned (odometry: still)
        modes, t_lost, t_found = [], None, None
        for _ in range(150):
            self.turn(true, odo)
            modes.append(self.lm.mode)
            if t_lost is None and self.lm.mode == "lost":
                t_lost = self.t
            if t_lost is not None and t_found is None and self.lm.mode == "track":
                t_found = self.t
                mapped_found = self.mapped
        self.assertIsNotNone(t_found, (modes, self.said))
        self.assertGreaterEqual(t_found - t_lost, lm.GLOBAL_AFTER_S)       # the local search had its go first
        self.assertLess(t_found - t_lost, lm.GLOBAL_AFTER_S + 2.0, self.said)
        self.assertLessEqual(mapped_found - mapped0, 1, "nothing mapped from the wrong pose or before the "
                                                        "confirmation (only the confirming turn, at the pose found)")
        self.assertLess(self.err(true, odo), 30)
        self.assertLess(self.head_err(true, odo), 1.0)
        self.assertEqual(self.lm.mode, "track")
        self.assertEqual((self.lm.n["global"], self.lm.n["relocated"]), (1, 1))
        st = self.m.stats()["lmatch"]["reloc"]
        self.assertEqual((st["phase"], st["by"]), ("found", "auto"))
        self.assertTrue(any("found on the map" in s for s in self.said), self.said)
        self.assertTrue(any("found by the whole-map search" in s for s in self.said), self.said)
        self.assertGreater(self.mapped, mapped_found, "mapping again after the confirmation")

    def test_teensy_reboot_resets_the_odometry_elsewhere(self):
        p = self.build_room()                            # the robot stands at (2600, 300, 120), odometry = truth
        self.t += 4.0                                    # rebooting: no telemetry, no LD19 bytes (they come via the Teensy)
        t0 = self.t
        modes = []
        for _ in range(40):
            self.turn(p, (0.0, 0.0, 0.0))                # back: the odometry reads 0, 0, 0 where it stands
            modes.append(self.lm.mode)
        self.assertTrue(any("Teensy reboot" in s for s in self.said), self.said)
        self.assertEqual(self.lm.mode, "track", (modes, self.said))
        self.assertLess(modes.index("track", modes.index("lost")) * 0.1, 1.0, modes)   # found at once, not after 5 s
        self.assertLess(self.err(p, (0, 0, 0)), 30)
        self.assertLess(self.head_err(p, (0, 0, 0)), 1.0)
        self.assertEqual(self.m.stats()["lmatch"]["reloc"]["by"], "reset")
        self.assertGreater(self.t, t0)

    def test_pose_reset_without_a_recent_lidar_pose(self):
        p = self.build_room()
        self.t += 3.0                                     # the lidar was out: nothing to keep the pose by
        self.m.on_pose_reset()
        self.assertIsNotNone(self.lm.glob_req)
        for _ in range(30):
            self.turn(p, (0.0, 0.0, 0.0))
        self.assertEqual(self.lm.mode, "track", self.said)
        self.assertLess(self.err(p, (0, 0, 0)), 30)

    def test_find_myself_and_its_errors(self):
        p = self.build_room()
        self.assertIsNone(self.m.command({"action": "relocalize"}, "test"))
        self.assertEqual(self.lm.glob_req["by"], "user")
        self.turn(p, p)
        self.assertEqual(self.lm.n["global"], 1)
        for _ in range(10):
            self.turn(p, p)
        self.assertEqual(self.lm.mode, "track")          # the same pose, confirmed
        self.assertLess(self.err(p, p), 25)
        lid = self.m.lidar
        self.m.lidar = None
        self.assertIn("no lidar", self.m.command({"action": "relocalize"})["text"])
        self.m.lidar = lid
        self.m.apply_settings({"lidar_match": False}, save=False)
        self.assertEqual(self.m.command({"action": "relocalize"})["t"], "error")

    def test_off_setting(self):
        self.m.apply_settings({"match_global": False}, save=False)
        self.build_room()
        for _ in range(100):
            self.turn((-400.0, -200.0, -120.0), (2600.0, 300.0, 120.0))
        self.assertEqual((self.lm.mode, self.lm.n["global"]), ("lost", 0))

    def test_map_load_finds_the_robot_on_it(self):
        p = self.build_room()
        self.m.save("room")
        m2 = Mapper(map_dir=self.tmp)
        m2.lidar = self.m.lidar
        m2.say = self.said.append
        m2.apply_settings({"match_every_n": 1}, save=False)
        self.m = m2
        self.lm = m2.lmatch
        m2.load("room")
        self.assertEqual(m2.lmatch.glob_req["by"], "load")
        true, odo = (1500.0, 200.0, 60.0), (-50.0, 30.0, 4.0)     # another session: its own odometry frame
        modes = []
        for _ in range(30):
            self.turn(true, odo)
            modes.append(m2.lmatch.mode)
        self.assertEqual(m2.lmatch.mode, "track", (modes, self.said))
        self.assertLess(self.err(true, odo), 30)
        self.assertLess(self.head_err(true, odo), 1.0)
        self.assertTrue(any("map room loaded" in s for s in self.said), self.said)


class TestSymmetric(Reloc):
    def test_symmetric_room_asks_for_set_position(self):
        room = Segs(_box(1000, 0, 4000, 3000))          # empty: every pose has a 180 deg twin
        self.room = room
        self.build(25)
        p = self.walk((0, 0, 0), (900, 300, 45), 20)
        mapped0, corr0 = self.mapped, list(self.m.corr)
        true = (1000.0 + 1000.0, -600.0, 200.0)
        for _ in range(110):
            self.turn(true, p)                           # carried
        st = self.m.stats()["lmatch"]
        self.assertEqual(self.lm.mode, "lost")
        self.assertEqual(st["reloc"]["phase"], "ambiguous", (st, self.said))
        self.assertGreaterEqual(self.lm.n["global"], 1)
        self.assertEqual(self.lm.n["relocated"], 0)
        self.assertEqual(self.mapped, mapped0, "never mapped")
        self.assertEqual(self.m.corr, corr0, "the pose was not moved to either twin")
        self.assertIn("ambiguous", self.lm.state)
        self.assertTrue(any("use Set position" in s for s in self.said), self.said)
        # the user says which one: confirmed, mapping again
        self.assertIsNone(self.m.command({"action": "set_pose", "x": true[0] - 100, "y": true[1] + 80,
                                          "th": true[2] + 6}, "test", odom=p))
        for _ in range(20):
            self.turn(true, p)
        self.assertEqual(self.lm.mode, "track", self.said)
        self.assertLess(self.err(true, p), 30)
        self.assertLess(self.head_err(true, p), 1.0)


class TestSetPosition(Reloc):
    def test_set_position_refines_and_confirms(self):
        self.m.apply_settings({"match_global": False}, save=False)   # the user alone
        self.build_room()
        odo = (2600.0, 300.0, 120.0)
        true = (-400.0, -200.0, -120.0)
        for _ in range(20):
            self.turn(true, odo)
        self.assertEqual(self.lm.mode, "lost")
        mapped0 = self.mapped
        tap = (true[0] + 150, true[1] - 120, true[2] - 12)           # a tap 190 mm / 12 deg off
        self.assertIsNone(self.m.command({"action": "set_pose", "x": tap[0], "y": tap[1], "th": tap[2]}, "test",
                                         odom=odo))
        x, y, h = self.m.map_pose(*odo)
        self.assertLess(math.hypot(x - tap[0], y - tap[1]), 1e-6)    # at once where the user put it
        self.assertAlmostEqual(h, tap[2], places=6)
        self.assertEqual(self.m.stats()["lmatch"]["reloc"]["phase"], "set")
        self.assertTrue(self.m.full_pending)
        for _ in range(12):
            self.turn(true, odo)
        self.assertEqual(self.lm.mode, "track", self.said)
        self.assertLess(self.err(true, odo), 30)
        self.assertLess(self.head_err(true, odo), 1.0)
        st = self.m.stats()["lmatch"]["reloc"]
        self.assertEqual((st["phase"], st["by"]), ("found", "set"))
        self.assertTrue(any("from the position set by hand" in s for s in self.said), self.said)
        self.assertLessEqual(self.mapped - mapped0, 12)

    def test_poor_fit_keeps_the_users_pose_unconfirmed(self):
        self.build_room()
        odo = (2600.0, 300.0, 120.0)
        true = (-400.0, -200.0, -120.0)
        mapped0 = self.mapped
        wrong = (1800.0, 1500.0, 0.0)
        self.assertIsNone(self.m.command({"action": "set_pose", "x": wrong[0], "y": wrong[1], "th": wrong[2]},
                                         "test", odom=odo))
        for _ in range(30):
            self.turn(true, odo)
        st = self.m.stats()["lmatch"]["reloc"]
        self.assertEqual(self.lm.mode, "lost")
        self.assertEqual(st["phase"], "set_poor", (st, self.said))
        self.assertIn("poor fit", self.lm.state)
        self.assertEqual(self.mapped, mapped0)
        x, y, _ = self.m.map_pose(*odo)
        self.assertLess(math.hypot(x - wrong[0], y - wrong[1]), 450, "kept (the lost search may nudge it a little)")
        # the automatic whole-map search holds off for a while: the user's word
        self.assertEqual(self.lm.n["relocated"], 0)

    def test_api(self):
        m = self.m
        self.assertIn("no robot pose", m.command({"action": "set_pose", "x": 1, "y": 2, "th": 3})["text"])
        self.assertIn("needed", m.command({"action": "set_pose", "x": "a", "y": 2}, odom=(0, 0, 0))["text"])
        # without lidar matching: just taken
        m.apply_settings({"lidar_match": False}, save=False)
        self.assertIsNone(m.command({"action": "set_pose", "x": 1000, "y": -500, "th": 90}, odom=(10, 20, 30)))
        x, y, h = m.map_pose(10, 20, 30)
        self.assertAlmostEqual(x, 1000)
        self.assertAlmostEqual(y, -500)
        self.assertAlmostEqual(h, 90)
        self.assertEqual(m.lmatch.mode, "track")
        # frame "odom": the coordinates the browser draws (odometry frame, current correction)
        want = m.map_pose(-200.0, 300.0, 45.0)
        self.assertIsNone(m.command({"action": "set_pose", "x": -200, "y": 300, "th": 45, "frame": "odom"},
                                    odom=(10, 20, 30)))
        for a, b in zip(m.map_pose(10, 20, 30), want):
            self.assertAlmostEqual(a, b, places=6)


class TestSimReloc(unittest.TestCase):
    """Sim end to end: real LD19 packets (lidar_sim), the bridge's lidar.py + matcher; the robot drives 3 tour legs
    (the map), then is carried off / its Teensy reboots. (More seeds and poses: python -m station.sim.lidar_eval
    --reloc.)"""

    def test_kidnap_and_reboot_are_found(self):
        from station.sim.lidar_eval import run_kidnap
        k = run_kidnap(1, "kidnap", (-300.0, 1200.0, -100.0))
        self.assertIsNotNone(k["found_s"], k)
        self.assertLess(k["found_s"], lm.GLOBAL_AFTER_S + 3.0, k)
        self.assertLess(k["err_end_mm"], 40, k)
        self.assertLess(k["head_end_deg"], 1.0, k)
        self.assertEqual((k["mode"], k["relocated"]), ("track", 1), k)
        self.assertGreater(k["wall_fit"], 0.97, k)       # the map is not smeared by the wrong pose
        r = run_kidnap(2, "reboot")
        self.assertIsNotNone(r["found_s"], r)
        self.assertLess(r["found_s"], 3.0 + 2.0, r)      # the 3 s gap + found at once (no 5 s wait)
        self.assertLess(r["err_end_mm"], 40, r)
        self.assertEqual((r["mode"], r["relocated"]), ("track", 1), r)
        self.assertGreater(r["wall_fit"], 0.97, r)


class TestNoStall(unittest.TestCase):
    """The search runs in the worker thread: the event loop keeps turning while it runs."""

    def test_loop_keeps_running_during_a_whole_map_search(self):
        tmp = tempfile.mkdtemp()
        old = sys.getswitchinterval()
        sys.setswitchinterval(0.001)                      # as the bridge (#102)
        try:
            m = Mapper(map_dir=tmp)
            room = Segs(_box(4000, 0, 10000, 10000) + _box(4500, 2000, 600, 600) + _box(5500, -500, 500, 500)
                        + [(6000, 5000, 6000, 4300), (9000, -1000, 8300, -1000), (-1000, 2000, -400, 2000)])
            rng = random.Random(5)
            m.grid = built_grid(room, rng, poses=[(x, y, 0) for x in (0, 3000, 6000) for y in (-3000, 0, 3000)], n=2)
            clock = [100.0]

            class L:
                defer = True
                hist = None

                @staticmethod
                def clock():
                    return clock[0]
            m.lidar = L()
            lmt = m.lmatch
            lmt.turns = 10 ** 6
            true = (6500.0, -2500.0, 75.0)
            pts = seen_from_guess(scan(room, *true, rng), true, (0.0, 0.0, 0.0))
            gaps = []

            async def run():
                lmt.request_global("test", by="user")
                lmt.on_scan(0.0, 0.0, 0.0, 0.0, 0.0, [pts], pts)
                self.assertTrue(lmt.glob_running)
                self.assertIsNotNone(lmt.job)
                t_prev = time.perf_counter()
                t0 = t_prev
                while lmt.job is not None and time.perf_counter() - t0 < 30:
                    await asyncio.sleep(0)
                    t = time.perf_counter()
                    gaps.append(t - t_prev)
                    t_prev = t
                return time.perf_counter() - t0
            dur = asyncio.run(run())
            gaps.sort()
            self.assertIsNone(lmt.job)
            r = lmt.last_global
            self.assertTrue(r["ok"], r)
            self.assertLess(math.hypot(r["x"] - true[0], r["y"] - true[1]), 40)
            print("\n[reloc] 10 x 10 m: search %.0f ms in the thread (pyramid %s ms), loop gaps median %.2f ms, "
                  "p99 %.2f, max %.1f ms (%d loop rounds)" % (1000 * dur, r.get("pyr_ms"), 1000 * gaps[len(gaps) // 2],
                                                            1000 * gaps[int(0.99 * len(gaps))], 1000 * gaps[-1], len(gaps)))
            self.assertGreater(len(gaps), 50, "the loop ran meanwhile")
            # (PC: p99 ~1 ms, max ~8 ms; lenient for a busy test machine. The Pi numbers: loop_bench --kidnap)
            self.assertLess(gaps[int(0.99 * len(gaps))], 0.020, "the worker hands the GIL back")
            self.assertLess(gaps[-1], 0.150, "no long piece of work holds the GIL")
        finally:
            sys.setswitchinterval(old)
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
