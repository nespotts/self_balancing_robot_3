"""Scan-to-map fit and global relocalization on an occupancy grid (numpy, no ROS: unit tested on Windows).

- MapField: distance to the nearest occupied cell for every cell of a nav_msgs/OccupancyGrid-like grid;
- fit(): share of the scan's end points within `tol` of a wall (how well the pose explains the scan);
- global_search(): brute-force correlative match of one scan over the whole free map (coarse grid of positions
  x headings, then a fine search around the best few): where is the robot, with no prior.

Poses are (x m, y m, yaw rad) of the LASER in the map frame; the caller converts base_link <-> laser.
"""
import math

import numpy as np


class MapField:
    def __init__(self, data, width, height, res, origin_xy, max_d=0.4, occ_thresh=65, free_thresh=25):
        """data: row-major int values (OccupancyGrid: -1 unknown, 0..100), row 0 at origin_xy (bottom)."""
        g = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.w, self.h, self.res = int(width), int(height), float(res)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.occ = g >= occ_thresh
        self.free = (g >= 0) & (g <= free_thresh)
        self.max_d = float(max_d)
        R = int(math.ceil(max_d / self.res))
        dist = np.full((self.h, self.w), self.max_d, dtype=np.float32)
        occ = self.occ
        pad = np.zeros((self.h + 2 * R, self.w + 2 * R), dtype=bool)
        pad[R:R + self.h, R:R + self.w] = occ
        for di in range(-R, R + 1):
            for dj in range(-R, R + 1):
                d = math.hypot(di, dj) * self.res
                if d > self.max_d:
                    continue
                sh = pad[R + di:R + di + self.h, R + dj:R + dj + self.w]
                np.minimum(dist, np.where(sh, np.float32(d), np.float32(self.max_d)), out=dist)
        self.dist = dist
        self.n_occ = int(occ.sum())

    @classmethod
    def from_msg(cls, m, **kw):
        i = m.info
        return cls(m.data, i.width, i.height, i.resolution, (i.origin.position.x, i.origin.position.y), **kw)

    def cells(self, x, y):
        """world m (arrays) -> (row, col, inside)"""
        c = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        r = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        inside = (c >= 0) & (c < self.w) & (r >= 0) & (r < self.h)
        return np.clip(r, 0, self.h - 1), np.clip(c, 0, self.w - 1), inside

    def dist_at(self, x, y):
        r, c, inside = self.cells(x, y)
        return np.where(inside, self.dist[r, c], self.max_d)


def scan_points(ranges, angle_min, angle_inc, rmin=0.05, rmax=8.0, max_n=None):
    """LaserScan ranges -> (N, 2) end points in the laser frame (valid returns only), optionally evenly thinned."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    r, a = r[ok], a[ok]
    if max_n and len(r) > max_n:
        idx = np.linspace(0, len(r) - 1, max_n).round().astype(int)
        r, a = r[idx], a[idx]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def transform(pts, pose):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return pts[:, 0] * c - pts[:, 1] * s + x, pts[:, 0] * s + pts[:, 1] * c + y


def fit(field, pts, pose, tol=0.10):
    """Share (0..1) of the scan end points within tol of an occupied cell, for a laser pose in the map frame."""
    if len(pts) == 0:
        return 0.0
    x, y = transform(pts, pose)
    return float(np.mean(field.dist_at(x, y) <= tol))


def score(field, pts, pose, sigma=0.05):
    """Smooth match score (0..1): mean of exp(-d^2 / 2 sigma^2) over the end points."""
    if len(pts) == 0:
        return 0.0
    x, y = transform(pts, pose)
    d = field.dist_at(x, y)
    return float(np.mean(np.exp(-0.5 * (d / sigma) ** 2)))


def _scores_grid(field, pts, xs, ys, yaws, sigma):
    """scores[k, i] for heading yaws[k] at position (xs[i], ys[i])."""
    lut_d = field.dist
    out = np.empty((len(yaws), len(xs)), dtype=np.float32)
    for k, t in enumerate(yaws):
        c, s = math.cos(t), math.sin(t)
        px = pts[:, 0] * c - pts[:, 1] * s
        py = pts[:, 0] * s + pts[:, 1] * c
        X = xs[:, None] + px[None, :]
        Y = ys[:, None] + py[None, :]
        cc = np.floor((X - field.ox) / field.res).astype(np.int64)
        rr = np.floor((Y - field.oy) / field.res).astype(np.int64)
        inside = (cc >= 0) & (cc < field.w) & (rr >= 0) & (rr < field.h)
        d = np.where(inside, lut_d[np.clip(rr, 0, field.h - 1), np.clip(cc, 0, field.w - 1)], field.max_d)
        out[k] = np.mean(np.exp(-0.5 * (d / sigma) ** 2), axis=1)
    return out


def global_search(field, pts, xy_step=0.10, yaw_step_deg=3.0, sigma=0.06, keep=12, fine=True, max_beams=120,
                  candidates=None):
    """Best laser poses for this scan anywhere in the map's free space.
    -> list of (score, (x, y, yaw)), best first, distinct (>= 0.4 m or >= 20 deg apart).
    candidates: optional (xs, ys) arrays of positions to try instead of the free-space grid."""
    if len(pts) == 0:
        return []
    if max_beams and len(pts) > max_beams:
        pts_c = pts[np.linspace(0, len(pts) - 1, max_beams).round().astype(int)]
    else:
        pts_c = pts
    if candidates is None:
        step = max(1, int(round(xy_step / field.res)))
        rr, cc = np.nonzero(field.free[::step, ::step])
        rr, cc = rr * step, cc * step
        # not right next to a wall: the robot body is there (r ~ 0.12 m)
        ok = field.dist[rr, cc] >= 0.10
        rr, cc = rr[ok], cc[ok]
        xs = field.ox + (cc + 0.5) * field.res
        ys = field.oy + (rr + 0.5) * field.res
    else:
        xs, ys = (np.asarray(v, dtype=np.float64) for v in candidates)
    if len(xs) == 0:
        return []
    yaws = np.radians(np.arange(-180.0, 180.0, yaw_step_deg))
    S = _scores_grid(field, pts_c, xs, ys, yaws, sigma * 2)          # coarse: a wider kernel
    flat = np.argsort(S, axis=None)[::-1]
    picks = []
    for f in flat[:5000]:
        k, i = divmod(int(f), len(xs))
        p = (float(xs[i]), float(ys[i]), float(yaws[k]))
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) >= 0.4 or
               abs(math.remainder(p[2] - q[2], 2 * math.pi)) >= math.radians(20) for q in picks):
            picks.append(p)
        if len(picks) >= keep:
            break
    out = []
    for p in picks:
        out.append(refine(field, pts, p, sigma) if fine else (score(field, pts, p, sigma), p))
    out.sort(key=lambda v: -v[0])
    return out


def refine(field, pts, pose, sigma=0.05, rounds=((0.10, 0.025, 4.0, 1.0), (0.03, 0.01, 1.0, 0.25))):
    """Local grid search around pose: -> (score, pose)."""
    best = (score(field, pts, pose, sigma), tuple(pose))
    for span, step, aspan, astep in rounds:
        x0, y0, t0 = best[1]
        d = np.arange(-span, span + 1e-9, step)
        X, Y = np.meshgrid(x0 + d, y0 + d)
        yaws = t0 + np.radians(np.arange(-aspan, aspan + 1e-9, astep))
        S = _scores_grid(field, pts, X.ravel(), Y.ravel(), yaws, sigma)
        k, i = divmod(int(np.argmax(S)), X.size)
        if S[k, i] > best[0]:
            best = (float(S[k, i]), (float(X.ravel()[i]), float(Y.ravel()[i]), float(math.remainder(yaws[k], 2 * math.pi))))
    return best


# ------------------------------------------------------------------ base_link <-> laser
def compose(a, b):
    """a o b for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.remainder(a[2] + b[2], 2 * math.pi))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])
