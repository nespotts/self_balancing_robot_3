"""Retracing steps in the simulator (retrace.py): real LD19 packets, the bridge's lidar.py + matcher, the planner and the
sim navigator. After 3 tour legs (the map) a goto drives into a sparse spot (the lidar gets ~7 % of its points there);
the robot is bumped 300 mm sideways during a 1 s lidar gap, so the pose is lost where nothing can find it. The planner
pauses the goto, drives back along its breadcrumbs out of the sparse spot, the pose is found and confirmed there, and the
goto is resumed and finished. (Off vs on, more seeds: python -m station.sim.lidar_eval --retrace --seeds 1 2 3.)
Run: python -m unittest discover -s station -t ."""
import time
import unittest

from station.bridge import retrace as rt
from station.sim.lidar_eval import SPARSE_X, run_retrace


class TestRetraceSim(unittest.TestCase):
    def test_lost_in_a_sparse_spot_retraces_relocalizes_and_resumes(self):
        t0 = time.time()
        o = run_retrace(2)
        info = (o, time.time() - t0)
        self.assertIsNotNone(o["lost_s"], info)                    # the gap check failed there: lost
        self.assertIsNotNone(o["retrace_s"], info)
        self.assertGreaterEqual(o["retrace_s"] - o["lost_s"], rt.AFTER_S - 1.5, info)   # the in-place search first
        self.assertIn("lost - retracing steps", o["statuses"], info)
        self.assertGreaterEqual(o["trail"], 10, info)              # the trail for the map, while it retraced
        self.assertGreater(o["x_back"], SPARSE_X, info)            # it drove back out of the sparse spot ...
        self.assertGreater(o["back_mm"], 500, info)
        self.assertLessEqual(o["back_mm"], 5300, info)             # ... no farther than retrace_max_mm
        self.assertIsNotNone(o["found_s"], info)                   # ... and was found there
        self.assertLess(o["err_found_mm"], 50, info)
        # every retrace ended with the pose found (by the matcher's own search at a stop, the window search there or
        # the whole-map search), none gave up. (The goal is in the sparse spot: a lidar gap check may doubt the pose
        # again just before it arrives, the pose itself is right: err_end_mm.)
        self.assertGreaterEqual(o["n"]["started"], 1, info)
        self.assertEqual(o["n"]["found"], o["n"]["started"], info)
        self.assertEqual(o["n"]["gave_up"], 0, info)
        self.assertEqual(o["status"], "arrived", info)              # the goto was resumed and finished
        self.assertLess(o["err_end_mm"], 50, info)
        self.assertLess(o["goal_mm"], 150, info)
        self.assertGreater(o["wall_fit"], 0.97, info)              # nothing mapped from the wrong pose
        self.assertTrue(any("retracing" in ln and "back to a well-mapped spot" in ln for ln in o["lines"]), info)
        self.assertTrue(any("resuming the route" in ln for ln in o["lines"]), info)


if __name__ == "__main__":
    unittest.main()
