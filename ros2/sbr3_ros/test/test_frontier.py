"""Frontier search for the explorer (no ROS; needs numpy + scipy, else skipped)."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
try:
    import numpy as np
    import scipy  # noqa: F401
    from sbr3_ros import frontier as fr
except ImportError:            # Windows Python without numpy / scipy: the WSL run covers it
    fr = None

RES = 0.05


def room(w_m=6.0, h_m=4.0):
    """All unknown, origin (0, 0)."""
    return np.full((int(h_m / RES), int(w_m / RES)), -1, dtype=np.int16)


def rc(x, y):
    return int(y / RES), int(x / RES)


def fill(g, x0, y0, x1, y1, v):
    r0, c0 = rc(x0, y0)
    r1, c1 = rc(x1, y1)
    g[r0:r1, c0:c1] = v


@unittest.skipIf(fr is None, "needs numpy + scipy")
class FrontierTest(unittest.TestCase):
    def test_closed_room_has_no_frontier(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, 0)
        fs, info = fr.find_frontiers(g, RES, (0.0, 0.0), (1.5, 1.5))
        self.assertEqual(fs, [])
        self.assertTrue(info["reachable"])
        self.assertAlmostEqual(info["free_m2"], 4.9 * 2.9, delta=0.2)

    def test_half_seen_room(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, -1)
        fill(g, 0.55, 0.55, 3.0, 3.45, 0)                # the left part seen, the right part unknown
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.5, 2.0))
        self.assertEqual(len(fs), 1)
        f = fs[0]
        self.assertTrue(f.opens)
        self.assertLess(f.viewpoint[0], 3.0)             # in known free space, not in the unknown
        self.assertGreater(f.viewpoint[0], 2.0)          # near the frontier at x = 3
        self.assertGreaterEqual(f.clearance, 0.29)
        self.assertAlmostEqual(f.dist_m, abs(f.viewpoint[0] - 1.5) + 0.0, delta=0.6)
        self.assertGreater(f.size_m, 2.5)

    def test_doorway_beats_wall_strip(self):
        g = room(8.0, 4.0)
        fill(g, 0.5, 0.5, 7.5, 3.5, -1)
        fill(g, 0.5, 0.5, 4.0, 3.5, 100)                 # room A walls ...
        fill(g, 0.55, 0.55, 3.95, 3.45, 0)               # ... and its floor, all seen
        fill(g, 3.95, 1.6, 4.0, 2.4, 0)                  # a 0.8 m doorway in the right wall into unknown room B
        fill(g, 0.55, 3.35, 1.5, 3.45, -1)               # a 10 cm unseen strip along the top wall (shadow)
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 1.0))
        self.assertGreaterEqual(len(fs), 2)
        door = [f for f in fs if f.point[0] > 3.5]
        strip = [f for f in fs if f.point[1] > 3.0]
        self.assertTrue(door and strip)
        self.assertTrue(door[0].opens)
        self.assertFalse(strip[0].opens)
        self.assertIs(fs[0], door[0])                    # the doorway first, though farther away

    def test_unreachable_and_blacklist(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 2.5, 3.45, 0)                # robot's room
        fill(g, 3.0, 0.55, 4.5, 3.45, 0)                 # a seen island behind the wall at 2.5..3.0
        fill(g, 4.5, 0.55, 5.45, 3.45, -1)               # its frontier at x = 4.5: not reachable
        fill(g, 0.55, 0.55, 1.0, 1.5, -1)                # a reachable frontier in the robot's room
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.8, 2.0))
        self.assertTrue(all(f.point[0] < 2.5 for f in fs), [f.as_dict() for f in fs])
        self.assertEqual(len(fs), 1)
        fs2, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.8, 2.0), blacklist=[fs[0].point, fs[0].viewpoint],
                                   p=fr.Params(black_r=1.5))
        self.assertEqual(fs2, [])

    def test_costmap_keeps_viewpoints_off_inflation(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, -1)
        fill(g, 0.55, 0.55, 3.0, 3.45, 0)
        cost = np.zeros(g.shape, dtype=np.int16)
        fill(cost, 2.6, 0.55, 3.0, 3.45, 80)            # e.g. a keep-out band near the frontier
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 2.0), cost=cost)
        self.assertEqual(len(fs), 1)
        self.assertLess(fs[0].viewpoint[0], 2.6)
        fill(cost, 2.0, 0.55, 3.0, 3.45, 80)            # all of view_r costly: the cell with the most clearance
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 2.0), cost=cost)
        self.assertEqual(len(fs), 1)
        self.assertGreater(fs[0].clearance, 0.5)

    def test_speck_is_not_a_frontier(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, 0)
        g[rc(2.0, 2.0)] = -1                             # one unseen cell in the middle of the floor
        fill(g, 3.0, 1.0, 3.1, 1.1, -1)                  # a 2 x 2 hole: frontier of 4-ish cells < min_size
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 1.0))
        self.assertEqual(fs, [])

    def test_sparse_wall_is_no_frontier(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, 0)
        for x in (1.0, 1.1, 1.25, 2.0, 2.05, 3.0):        # 5-10 cm holes in the top wall, unknown behind
            fill(g, x, 3.45, x + 0.05, 3.5, 0)
        fill(g, 0.5, 3.5, 5.5, 4.0, -1)
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 1.0))
        self.assertEqual(fs, [])
        fill(g, 2.5, 3.45, 3.3, 3.5, 0)                  # an 80 cm doorway is one
        fs, _ = fr.find_frontiers(g, RES, (0.0, 0.0), (1.0, 1.0))
        self.assertEqual(len(fs), 1)
        self.assertTrue(fs[0].opens)

    def test_robot_in_inflation_can_leave(self):
        g = room()
        fill(g, 0.5, 0.5, 5.5, 3.5, 100)
        fill(g, 0.55, 0.55, 5.45, 3.45, -1)
        fill(g, 0.55, 0.55, 3.0, 3.45, 0)
        fs, info = fr.find_frontiers(g, RES, (0.0, 0.0), (0.68, 2.0))     # 0.13 m from the wall
        self.assertTrue(info["reachable"])
        self.assertEqual(len(fs), 1)

    def test_resample_and_write_map(self):
        src = np.arange(12, dtype=np.int16).reshape(3, 4)
        out = fr.resample(src, 0.1, (0.0, 0.0), (6, 8), 0.05, (0.0, 0.0))
        self.assertEqual(out[0, 0], 0)
        self.assertEqual(out[5, 7], 11)
        out2 = fr.resample(src, 0.1, (0.0, 0.0), (2, 2), 0.1, (0.3, 0.2))
        self.assertEqual(out2[0, 0], 11)
        self.assertEqual(out2[1, 1], -1)
        g = np.array([[-1, 0, 100], [10, 70, 30]], dtype=np.int16)
        with tempfile.TemporaryDirectory() as d:
            img = fr.write_map(os.path.join(d, "m"), g, 0.05, (-1.0, 2.0))
            self.assertEqual(img.tolist(), [[254, 0, 205], [205, 254, 0]])     # flipped: top row = max y
            with open(os.path.join(d, "m.pgm"), "rb") as fh:
                data = fh.read()
            self.assertTrue(data.startswith(b"P5\n"))
            self.assertTrue(data.endswith(bytes([254, 0, 205, 205, 254, 0])))
            with open(os.path.join(d, "m.yaml")) as fh:
                y = fh.read()
            self.assertIn("image: m.pgm", y)
            self.assertIn("origin: [-1.000, 2.000, 0]", y)
            try:
                fr.map_png(os.path.join(d, "m.png"), g, 0.05, (-1.0, 2.0), [(-0.9, 2.0), (-0.9, 2.05)],
                           [(-0.95, 2.02, True)])
                self.assertTrue(os.path.getsize(os.path.join(d, "m.png")) > 0)
            except ImportError:
                pass

    def test_big_map_is_fast(self):
        import time
        g = room(25.0, 25.0)
        fill(g, 0.5, 0.5, 24.5, 24.5, 100)
        fill(g, 0.55, 0.55, 24.45, 24.45, 0)
        for x in range(3, 24, 3):                        # walls with doors, unknown patches
            fill(g, x, 0.55, x + 0.05, 24.45, 100)
            fill(g, x, 5.0 + (x % 7), x + 0.05, 5.8 + (x % 7), 0)
            fill(g, x + 1.0, 18.0, x + 2.0, 19.0, -1)
        t = time.time()
        fs, info = fr.find_frontiers(g, RES, (0.0, 0.0), (1.5, 1.5))
        self.assertLess(time.time() - t, 3.0)
        self.assertGreaterEqual(len(fs), 5)


if __name__ == "__main__":
    unittest.main()
