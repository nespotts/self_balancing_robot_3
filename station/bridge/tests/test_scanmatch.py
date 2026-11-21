"""scanmatch.Matcher: sweeps are held back, flushed into the map, and the pose correction stays bounded (#49)."""
import math
import shutil
import tempfile
import unittest

from station.bridge import scanmatch
from station.bridge.mapping import Mapper


def reading(i, x=0.0, wall=1000.0, valid=True):
    a = -30 + 60 * (i % 16) / 15.0                            # one sweep = 16 steps, two sensors = 32 readings
    return {"range_mm": wall / math.cos(math.radians(a)) - 40 if valid else 8190, "valid": valid,
            "x_mm": x, "y_mm": 0.0, "heading_deg": 0.0, "sensor_angle_deg": a}


class TestScanMatch(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(map_dir=self.tmp)
        self.m.settings["correct"] = True

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_a_sweep_is_held_back_then_integrated(self):
        mt = self.m.matcher
        for i in range(scanmatch.SWEEP_N - 1):
            mt.add(reading(i))
        self.assertEqual((len(mt.buf), self.m.n["used"], mt.sweeps), (scanmatch.SWEEP_N - 1, 0, 0))
        mt.add(reading(scanmatch.SWEEP_N - 1))
        self.assertEqual((len(mt.buf), self.m.n["used"], mt.sweeps), (0, scanmatch.SWEEP_N, 1))

    def test_a_consistent_wall_leaves_the_correction_alone(self):
        mt = self.m.matcher
        for i in range(scanmatch.SWEEP_N * 8):
            mt.add(reading(i))
        self.assertEqual(mt.sweeps, 8)
        self.assertEqual(self.m.corr, [0.0, 0.0, 0.0])
        self.assertIsNotNone(mt.last)
        self.assertEqual(mt.applied, 0)

    def test_odometry_drift_is_pulled_back_within_one_step_per_sweep(self):
        mt = self.m.matcher
        for i in range(scanmatch.SWEEP_N * 5):                 # build the map at the true pose
            mt.add(reading(i))
        for i in range(scanmatch.SWEEP_N * 4):                 # then odometry claims +20 mm forward
            mt.add(reading(i, x=20.0))
        dx = self.m.corr[0]
        self.assertLessEqual(abs(self.m.corr[1]), 25 * 4)
        self.assertLessEqual(abs(dx), 25 * 4)
        self.assertLessEqual(abs(self.m.corr[2]), 4.0)
        self.assertLessEqual(mt.applied, 4)

    def test_no_targets_means_no_match(self):
        mt = self.m.matcher
        for i in range(scanmatch.SWEEP_N * 6):
            mt.add(reading(i, valid=False))
        self.assertEqual(self.m.corr, [0.0, 0.0, 0.0])
        self.assertEqual(mt.applied, 0)

    def test_reset_clears_state(self):
        mt = self.m.matcher
        for i in range(scanmatch.SWEEP_N * 2 + 5):
            mt.add(reading(i))
        mt.reset()
        self.assertEqual((mt.buf, mt.sweeps, mt.last, mt.applied), ([], 0, None, 0))

    def test_flush_with_correction_off_integrates_what_is_buffered(self):
        mt = self.m.matcher
        for i in range(5):
            mt.add(reading(i))
        mt.flush()
        self.assertEqual(self.m.n["used"], 5)


if __name__ == "__main__":
    unittest.main()