"""Station zones -> keepout filter mask (no ROS)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sbr3_ros import keepout as ko  # noqa: E402


class KeepoutTest(unittest.TestCase):
    RES = 0.05

    def cell(self, mask, origin, x, y):
        return int(mask[int((y - origin[1]) / self.RES), int((x - origin[0]) / self.RES)])

    def test_nogo_wall_fence(self):
        items = [{"id": "n1", "kind": "nogo", "pts": [[1000, 1000], [2000, 1000], [2000, 2000], [1000, 2000]]},
                 {"id": "w2", "kind": "wall", "pts": [[0, -1000], [0, 1000]]},
                 {"id": "s3", "kind": "slow", "pts": [[3000, 0], [4000, 0], [4000, 1000]], "speed": 100}]
        T = (0.0, 0.0, 0.0)
        origin, w, h = ko.mask_extent((-2.0, -2.0, self.RES, 120, 100), items, T)
        mask, used = ko.build_mask(items, T, origin, self.RES, w, h, inflate=0.15, soft=0)
        self.assertEqual(used, 2)                                  # slow zones are not keep-out
        c = lambda x, y: self.cell(mask, origin, x, y)
        self.assertEqual(c(1.5, 1.5), 100)                         # inside the no-go zone
        self.assertEqual(c(0.9, 1.5), 100)                         # within inflate of its edge
        self.assertEqual(c(0.7, 1.5), 0)
        self.assertEqual(c(0.0, 0.0), 100)                         # on the wall
        self.assertEqual(c(0.12, 0.0), 100)
        self.assertEqual(c(0.3, 0.0), 0)
        self.assertEqual(c(0.0, 1.4), 0)                           # past the wall's end
        self.assertEqual(c(3.7, 0.3), 0)                           # slow zone: free
        fence = [{"id": "f1", "kind": "fence", "pts": [[-1000, -1000], [1000, -1000], [1000, 1000], [-1000, 1000]]}]
        mask, _ = ko.build_mask(fence, T, origin, self.RES, w, h, inflate=0.15, soft=0)
        self.assertEqual(self.cell(mask, origin, 0.0, 0.0), 0)
        self.assertEqual(self.cell(mask, origin, 0.9, 0.0), 100)  # near the fence, inside
        self.assertEqual(self.cell(mask, origin, 1.5, 0.0), 100)  # outside

    def test_transform_and_maze(self):
        items = [{"id": "w1", "kind": "wall", "pts": [[1000, 0], [1000, 500]]},
                 {"id": "m2", "kind": "maze", "walls": [[0, 2000, 500, 2000]]}]
        T = (0.5, 0.0, math.pi / 2)                                # station_map is turned 90 deg in the map
        origin, w, h = ko.mask_extent(None, items, T)
        mask, used = ko.build_mask(items, T, origin, self.RES, w, h, inflate=0.1, soft=0)
        self.assertEqual(used, 2)
        # wall (1, 0)-(1, 0.5) station -> map: x = 0.5 - y, y = x -> (0.5, 1)-(0, 1)
        self.assertEqual(self.cell(mask, origin, 0.25, 1.0), 100)
        self.assertEqual(self.cell(mask, origin, 0.25, 0.5), 0)
        # maze wall (0, 2)-(0.5, 2) -> (-1.5, 0)-(-1.5, 0.5)
        self.assertEqual(self.cell(mask, origin, -1.5, 0.25), 100)

    def test_soft_band(self):
        items = [{"id": "w1", "kind": "wall", "pts": [[0, -1000], [0, 1000]]}]
        T = (0.0, 0.0, 0.0)
        origin, w, h = ko.mask_extent(None, items, T)
        mask, _ = ko.build_mask(items, T, origin, self.RES, w, h, inflate=0.15, soft=0.30, soft_max=60)
        c = lambda x: self.cell(mask, origin, x, 0.0)
        self.assertEqual(c(0.1), 100)
        v = [c(x) for x in (0.22, 0.32, 0.42)]
        self.assertTrue(60 >= v[0] > v[1] > v[2] > 0, v)      # falls off with distance, never lethal
        self.assertEqual(c(0.6), 0)

    def test_signature_ignores_undo(self):
        a = [{"id": "n1", "kind": "nogo", "pts": [[0, 0], [1, 0], [1, 1]]}]
        self.assertEqual(ko.zones_signature(a), ko.zones_signature(list(a) + [{"id": "s", "kind": "slow"}]))


if __name__ == "__main__":
    unittest.main()
