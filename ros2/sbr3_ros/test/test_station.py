"""Station frame and places (no ROS)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sbr3_ros import convert as cv  # noqa: E402


class StationFrameTest(unittest.TestCase):
    def test_station_frame_is_inverse_of_corr(self):
        corr = [120.0, -40.0, 10.0]                       # station map pose = corr o odometry pose
        odom_pt = (1.0, 0.5, 0.0)
        station_pt = cv.pose_compose(cv.corr_to_tf(corr), odom_pt)
        back = cv.pose_compose(cv.station_frame_tf(corr), station_pt)    # odom <- station_map
        for a, b in zip(odom_pt, back):
            self.assertAlmostEqual(a, b)
        self.assertEqual(cv.station_frame_tf(None), (-0.0, -0.0, 0.0))

    def test_places(self):
        m = {"t": "places", "map": "lab", "items": [{"name": "Home", "x": 0, "y": 0, "home": True, "default": True},
                                                    {"name": "Kitchen", "x": 2400, "y": -300}, {"bad": 1}]}
        p = cv.places_from_msg(m)
        self.assertEqual([q["name"] for q in p], ["Home", "Kitchen"])
        self.assertAlmostEqual(p[1]["x"], 2.4)
        self.assertEqual(cv.find_place(p, "  kitchen ")["name"], "Kitchen")
        self.assertEqual(cv.find_place(p, "HOME")["name"], "Home")
        self.assertIsNone(cv.find_place(p, "garage"))

    def test_route_yaws(self):
        y = cv.route_yaws([(1, 0), (1, 1), (1, 1), (0, 1)], start=(0, 0))
        self.assertAlmostEqual(y[0], 0.0)
        self.assertAlmostEqual(y[1], math.pi / 2)
        self.assertAlmostEqual(y[2], math.pi / 2)     # same point: keeps the heading
        self.assertAlmostEqual(abs(y[3]), math.pi)


if __name__ == "__main__":
    unittest.main()
