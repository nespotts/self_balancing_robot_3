"""Station link (#106): grid encoding, plan simplification, frames, commands, Nav2 status (no ROS)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sbr3_ros import stationlink as sl  # noqa: E402


class GridTest(unittest.TestCase):
    def test_roundtrip(self):
        data = [-1] * 300 + [0] * 5 + [100] * 2 + [50] + [-1] * 200 + [10] * 140
        s = sl.encode_grid(data)
        self.assertEqual(sl.decode_grid(s), [sl.grid_class(v) for v in data])
        self.assertLess(len(s), 40)                       # a handful of runs, varint counts

    def test_classes(self):
        self.assertEqual([sl.grid_class(v) for v in (-1, 0, 25, 26, 64, 65, 100)],
                         [sl.UNKNOWN, sl.FREE, sl.FREE, sl.MAYBE, sl.MAYBE, sl.OCC, sl.OCC])

    def test_map_msg(self):
        m = sl.map_msg(4, 2, 0.05, (-1.0, -2.0, 0.0), [0, 0, 100, -1, -1, -1, 0, 0], 3)
        self.assertEqual((m["t"], m["w"], m["h"], m["seq"], m["enc"]), ("ros_map", 4, 2, 3, "rle4"))
        self.assertEqual(sl.decode_grid(m["rle"]), [1, 1, 2, 0, 0, 0, 1, 1])
        self.assertEqual(m["origin"], [-1.0, -2.0, 0.0])

    def test_digest(self):
        a = sl.grid_digest(2, 1, 0.05, (0, 0, 0), [0, 100])
        self.assertEqual(a, sl.grid_digest(2, 1, 0.05, (0, 0, 0), [0, 100]))
        self.assertNotEqual(a, sl.grid_digest(2, 1, 0.05, (0, 0, 0), [0, 99]))
        self.assertNotEqual(a, sl.grid_digest(2, 1, 0.05, (0.1, 0, 0), [0, 100]))


class PlanTest(unittest.TestCase):
    def test_simplify_line(self):
        pts = [(i * 0.05, 0.001 * (i % 2)) for i in range(41)]
        s = sl.simplify(pts, 0.03)
        self.assertEqual(s, [pts[0], pts[-1]])

    def test_simplify_corner(self):
        pts = [(i * 0.1, 0.0) for i in range(11)] + [(1.0, i * 0.1) for i in range(1, 11)]
        s = sl.simplify(pts, 0.02)
        self.assertEqual(s, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])

    def test_plan_msg(self):
        pts = [(math.cos(a / 100.0), math.sin(a / 100.0)) for a in range(0, 315)]
        m = sl.plan_msg(pts, tol=0.0001, max_pts=20)
        self.assertLessEqual(len(m["pts"]), 20)
        self.assertAlmostEqual(m["len_m"], 3.14, places=1)
        self.assertEqual(sl.plan_msg([])["pts"], [])


class FrameTest(unittest.TestCase):
    def test_station_map_roundtrip(self):
        T = (1.0, -0.5, math.radians(90))
        x, y = sl.station_to_map(T, 1000.0, 0.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.5)
        sx, sy = sl.map_to_station(T, x, y)
        self.assertAlmostEqual(sx, 1000.0, places=6)
        self.assertAlmostEqual(sy, 0.0, places=6)
        x, y, a = sl.station_to_map(T, 0, 0, 0.0)
        self.assertAlmostEqual(a, math.radians(90))

    def test_explore_fill(self):
        T = (1.0, 0.0, 0.0)
        m = sl.explore_fill({"state": "exploring", "target_map": [2.0, 0.5], "frontiers": 4}, T)
        self.assertEqual((m["t"], m["target"], m["frontiers"]), ("ros_explore", [1000, 500], 4))
        m = sl.explore_fill({"state": "exploring", "target": [5, 6], "target_map": [2.0, 0.5]}, T)
        self.assertEqual(m["target"], [5, 6])               # the explorer's own station target wins
        m = sl.explore_fill({"state": "idle"}, None)
        self.assertIsNone(m["target"])


class CommandTest(unittest.TestCase):
    def test_parse_pose(self):
        self.assertEqual(sl.parse_pose({"x": 1, "y": 2}), ("map", 1.0, 2.0, None))
        f, x, y, yaw = sl.parse_pose({"x": 1, "y": 2, "yaw": 4.0, "frame": "station"})
        self.assertEqual(f, "station")
        self.assertAlmostEqual(yaw, 4.0 - 2 * math.pi)
        for bad in ({"x": "nan", "y": 0}, {"y": 1}, {"x": 0, "y": 0, "frame": "odom"}):
            with self.assertRaises((ValueError, KeyError)):
                sl.parse_pose(bad)

    def test_parse_patrol(self):
        self.assertEqual(sl.parse_patrol({"places": "Kitchen, Couch ,", "loops": 2, "dwell": 5}),
                         (["Kitchen", "Couch"], 2, 5.0))
        self.assertEqual(sl.parse_patrol({"places": ["A"]}), (["A"], 1, 3.0))
        with self.assertRaises(ValueError):
            sl.parse_patrol({"places": []})

    def test_map_name(self):
        self.assertEqual(sl.clean_map_name("explore 2026/11"), "explore-2026-11")
        self.assertIsNone(sl.clean_map_name("  "))


class NavTrackerTest(unittest.TestCase):
    def test_goal(self):
        n = sl.NavTracker()
        self.assertEqual(n.msg()["state"], "idle")
        n.start("place", (1, 2, 0), "Kitchen", now=10.0)
        n.feedback(2.345, 12.4)
        m = n.msg(now=13.0)
        self.assertEqual((m["state"], m["kind"], m["place"], m["dist_m"], m["eta_s"], m["elapsed_s"]),
                         ("navigating", "place", "Kitchen", 2.35, 12, 3.0))
        n.finish("failed", "no valid path", now=20.0)
        m = n.msg(now=30.0)
        self.assertEqual((m["state"], m["reason"], m["elapsed_s"], m["dist_m"]), ("failed", "no valid path", 10.0, None))
        self.assertFalse(n.busy())

    def test_nav2_reason(self):
        self.assertEqual(sl.nav2_reason(0, ""), "")
        self.assertEqual(sl.nav2_reason(102), "pose not available in time (TF)")
        self.assertEqual(sl.nav2_reason(208, ""), "no path to the goal")
        self.assertEqual(sl.nav2_reason(999), "Nav2 error 999")
        self.assertEqual(sl.nav2_reason(102, " custom "), "custom")

    def test_patrol(self):
        p = {"places": ["A", "B"], "i": 0, "loop": 0, "loops": 2, "dwell": 3.0, "missed": [], "dwell_end": None}
        stops = []
        cur = (0, 0)
        while cur is not None:
            stops.append(cur)
            p["i"], p["loop"] = cur
            cur = sl.patrol_next(p)
        self.assertEqual(stops, [(0, 0), (1, 0), (0, 1), (1, 1)])
        p.update(i=1, loop=5, loops=0)
        self.assertEqual(sl.patrol_next(p), (0, 6))          # loops 0 = forever
        n = sl.NavTracker()
        n.patrol = dict(p, i=0, loop=0)
        n.start("patrol", (0, 0, 0), "A", now=0.0, keep_patrol=True)
        n.dwell(until=5.0)
        m = n.msg(now=2.0)
        self.assertEqual((m["state"], m["patrol"]["dwell_left"]), ("dwell", 3.0))


if __name__ == "__main__":
    unittest.main()
