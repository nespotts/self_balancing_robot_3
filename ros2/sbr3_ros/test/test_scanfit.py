"""scanfit: map distance field, scan fit, global relocalization (no ROS)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sbr3_ros import scanfit as sf  # noqa: E402

# a 5 x 3.5 m room with two boxes and a partition (like the sim room), metres
SEGS = []


def box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


SEGS += box(1.5, 0.25, 5.0, 3.5) + box(1.5, 0.9, 0.6, 0.4) + box(2.9, -0.8, 0.45, 0.45) + box(0.7, -1.3, 1.0, 0.4)
SEGS += [(2.3, 2.0, 2.3, 1.1), (-1.0, 1.2, -0.3, 2.0)]
RES = 0.05
ORIGIN = (-1.5, -2.0)
W, H = 140, 110


def seg_dist(px, py, s):
    xa, ya, xb, yb = s
    ex, ey = xb - xa, yb - ya
    L2 = ex * ex + ey * ey
    u = max(0.0, min(1.0, ((px - xa) * ex + (py - ya) * ey) / L2)) if L2 else 0.0
    return math.hypot(xa + u * ex - px, ya + u * ey - py)


def grid():
    data = []
    for r in range(H):
        for c in range(W):
            x, y = ORIGIN[0] + (c + 0.5) * RES, ORIGIN[1] + (r + 0.5) * RES
            inside = -1.0 < x < 4.0 and -1.5 < y < 2.0
            d = min(seg_dist(x, y, s) for s in SEGS)
            data.append(100 if d <= RES * 0.75 else (0 if inside else -1))
    return data


def raycast(x, y, a):
    dx, dy = math.cos(a), math.sin(a)
    best = None
    for x1, y1, x2, y2 in SEGS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best if best is not None else float("inf")


def scan_at(pose, n=480):
    inc = 2 * math.pi / n
    return [raycast(pose[0], pose[1], pose[2] + (-math.pi + i * inc)) for i in range(n)], -math.pi, inc


class ScanFitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.field = sf.MapField(grid(), W, H, RES, ORIGIN)

    def test_field(self):
        f = self.field
        self.assertGreater(f.n_occ, 300)
        self.assertLess(float(f.dist_at([-1.0], [0.0])[0]), 0.05)      # on the west wall
        self.assertAlmostEqual(float(f.dist_at([0.0], [0.0])[0]), f.max_d, places=3)   # > 0.4 m from walls

    def test_fit_true_vs_wrong_pose(self):
        pose = (0.3, 0.2, 0.4)
        r, amin, inc = scan_at(pose)
        pts = sf.scan_points(r, amin, inc)
        self.assertGreater(sf.fit(self.field, pts, pose), 0.95)
        self.assertLess(sf.fit(self.field, pts, (1.0, -0.5, 1.5)), 0.4)

    def test_global_search_finds_pose(self):
        for pose in [(0.3, 0.2, 0.4), (3.3, 1.3, -2.0), (1.9, -0.4, 3.0)]:
            r, amin, inc = scan_at(pose)
            pts = sf.scan_points(r, amin, inc)
            res = sf.global_search(self.field, pts)
            s, p = res[0]
            self.assertLess(math.hypot(p[0] - pose[0], p[1] - pose[1]), 0.06, (pose, p))
            self.assertLess(abs(math.remainder(p[2] - pose[2], 2 * math.pi)), math.radians(2.0), (pose, p))
            self.assertGreater(s, 0.8)
            if len(res) > 1:
                self.assertLess(res[1][0], s)

    def test_compose_inverse(self):
        a = (1.0, 2.0, 0.7)
        b = (0.1, -0.05, 0.2)
        c = sf.compose(sf.compose(a, b), sf.inverse(b))
        for u, v in zip(a, c):
            self.assertAlmostEqual(u, v)


if __name__ == "__main__":
    unittest.main()
