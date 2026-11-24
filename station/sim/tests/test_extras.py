"""Sim extras for the ROS 2 tests (#98): start pose, kidnapping, truth row, extra boxes."""
import math
import unittest

from station.sim import extras
from station.sim.sim import Room, SimRobot
from station.sim.tour import DriftRobot


class ExtrasTest(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(extras.parse_pose("1200,-400,90"), (1200.0, -400.0, 90.0))
        self.assertEqual(extras.parse_pose("5 6"), (5.0, 6.0, 0.0))
        for bad in ("", "1", "1,2,3,4", "a,b", "nan,1,2"):
            with self.assertRaises(ValueError):
                extras.parse_pose(bad)
        self.assertEqual(len(extras.parse_box("0,0,200,100")), 4)
        with self.assertRaises(ValueError):
            extras.parse_box("0,0,-1,100")

    def test_start_pose_reads_zero_odometry(self):
        r = SimRobot()
        extras.teleport(r, 1200, -400, 90, reset_odom=True)
        self.assertEqual((r.X, r.Y, r.TH), (1200.0, -400.0, 90.0))
        for v in r.odom():
            self.assertAlmostEqual(v, 0.0)

    def test_kidnap_keeps_odometry(self):
        for robot in (SimRobot(), DriftRobot(drift_deg_min=3.0, drift_scale=0.02)):
            r = robot
            extras.teleport(r, 300, 200, 30, reset_odom=True)
            before = r.odom()
            extras.teleport(r, 2500, 1000, -120)
            self.assertEqual((r.X, r.Y, r.TH), (2500.0, 1000.0, -120.0))
            for a, b in zip(before, r.odom()):
                self.assertAlmostEqual(a, b, places=6)
            r.step()                                       # the jump is not integrated as motion
            for a, b in zip(before, r.odom()):
                self.assertAlmostEqual(a, b, delta=1.0)

    def test_kidnap_then_drive_moves_odometry_along_odom_heading(self):
        r = SimRobot()
        extras.teleport(r, 0, 0, 0, reset_odom=True)
        extras.teleport(r, 1000, 1000, 90)                 # true heading +Y, odometry still says +X
        r.X += 100.0                                       # whatever the true motion: odometry follows the odom frame
        ox, oy, _ = r.odom()
        self.assertAlmostEqual(math.hypot(ox, oy), 100.0, places=6)

    def test_boxes_survive_extra_walls(self):
        r = SimRobot()
        n = len(r.room.segs)
        extras.add_boxes(r, ["2200,-300,300,300"])
        self.assertEqual(len(r.room.segs), n + 4)
        r.room.set_extra([(0, 0, 100, 0)])
        self.assertEqual(len(r.room.segs), n + 5)
        self.assertLess(r.room.clearance(2200, -140), 20.0)
        self.assertEqual(len(Room().segs), n)

    def test_truth_row(self):
        r = SimRobot()
        extras.teleport(r, 100, 200, 45, reset_odom=True)
        row = extras.truth_row(r, now=12.5).strip().split(",")
        self.assertEqual([float(v) for v in row[:4]], [12.5, 100.0, 200.0, 45.0])


if __name__ == "__main__":
    unittest.main()
