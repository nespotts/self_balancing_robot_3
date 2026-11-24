"""Station zones (station/bridge/zones.py: virtual walls, no-go zones, the fence, mazes) -> a Nav2 keepout filter
mask (OccupancyGrid values: 100 keep out, 0 free) in the ROS map frame. numpy, no ROS: unit tested on Windows.

Zones come in the station's map frame (mm); T = map <- station_map (x m, y m, yaw rad) puts them in the ROS map.
Every keep-out shape grows by `inflate` (the robot radius + a margin) and gets a soft cost band around that: Nav2
costmap filters are applied after the inflation layer, so the mask itself must keep the robot's body out, as the
station's own planner does.
Slow zones are not keep-out (left out here).
"""
import math

import numpy as np

KEEPOUT_KINDS = ("wall", "nogo", "fence", "maze")


def _xf(pts_mm, T):
    c, s = math.cos(T[2]), math.sin(T[2])
    out = []
    for p in pts_mm:
        x, y = p[0] / 1000.0, p[1] / 1000.0
        out.append((c * x - s * y + T[0], s * x + c * y + T[1]))
    return out


def segments(item, T):
    """Keep-out item -> (segments [(x1, y1, x2, y2)] in the map frame, polygon [(x, y)] or None, kind)."""
    k = item.get("kind")
    if k == "maze":
        segs = []
        for w in item.get("walls") or []:
            (a, b) = _xf([(w[0], w[1]), (w[2], w[3])], T)
            segs.append((a[0], a[1], b[0], b[1]))
        return segs, None
    pts = _xf(item.get("pts") or [], T)
    if k == "wall":
        return [(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])], None
    ring = pts + pts[:1]
    return [(a[0], a[1], b[0], b[1]) for a, b in zip(ring, ring[1:])], pts


def _seg_dist(X, Y, s):
    xa, ya, xb, yb = s
    ex, ey = xb - xa, yb - ya
    L2 = ex * ex + ey * ey
    if L2 <= 0:
        return np.hypot(X - xa, Y - ya)
    u = np.clip(((X - xa) * ex + (Y - ya) * ey) / L2, 0.0, 1.0)
    return np.hypot(xa + u * ex - X, ya + u * ey - Y)


def _inside(X, Y, poly):
    """Even-odd rule, vectorised over the cells."""
    inside = np.zeros(X.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        cond = (Y >= min(y1, y2)) & (Y < max(y1, y2))
        xc = x1 + (Y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (X < xc)
    return inside


def build_mask(items, T, origin_xy, res, width, height, inflate=0.15, soft=0.30, soft_max=60):
    """-> (int8 array (height, width), row 0 at origin_xy; number of items used).
    100 (lethal) within `inflate` of a keep-out shape; then a soft band `soft` m wide falling from soft_max to 0
    (like the inflation layer, which costmap filters skip): plans keep a margin from the zone instead of grazing
    its lethal edge, where the controller's look-ahead would see a collision."""
    xs = origin_xy[0] + (np.arange(width) + 0.5) * res
    ys = origin_xy[1] + (np.arange(height) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    dist = np.full((height, width), np.inf)
    used = 0
    for it in items or []:
        k = it.get("kind")
        if k not in KEEPOUT_KINDS:
            continue
        try:
            segs, poly = segments(it, T)
        except (TypeError, ValueError, IndexError):
            continue
        if not segs:
            continue
        used += 1
        d = np.full((height, width), np.inf)
        for s in segs:
            np.minimum(d, _seg_dist(X, Y, s), out=d)
        if k == "nogo" and poly and len(poly) >= 3:
            d[_inside(X, Y, poly)] = 0.0
        elif k == "fence" and poly and len(poly) >= 3:
            d[~_inside(X, Y, poly)] = 0.0
        np.minimum(dist, d, out=dist)
    lethal = dist <= inflate + 0.5 * res
    out = np.zeros((height, width), dtype=np.int8)
    if soft > 0:
        band = (~lethal) & (dist < inflate + soft)
        out[band] = np.round(soft_max * (1.0 - (dist[band] - inflate) / soft)).astype(np.int8)
    out[lethal] = 100
    return out, used


def mask_extent(map_info, items, T, margin=1.0):
    """Mask grid covering the map and every zone (+ margin): (origin_xy, width, height) at the map's resolution.
    map_info: (origin_x, origin_y, res, width, height) or None."""
    xs, ys = [], []
    if map_info:
        ox, oy, res, w, h = map_info
        xs += [ox, ox + w * res]
        ys += [oy, oy + h * res]
    else:
        res = 0.05
    for it in items or []:
        if it.get("kind") not in KEEPOUT_KINDS:
            continue
        segs, _ = segments(it, T)
        for s in segs:
            xs += [s[0], s[2]]
            ys += [s[1], s[3]]
    if not xs:
        return None
    x0 = math.floor((min(xs) - margin) / res) * res
    y0 = math.floor((min(ys) - margin) / res) * res
    w = int(math.ceil((max(xs) + margin - x0) / res))
    h = int(math.ceil((max(ys) + margin - y0) / res))
    return (x0, y0), w, h


def zones_signature(items):
    """What matters for the mask (ids, kinds and points): ignore undo counters etc."""
    return repr([(it.get("id"), it.get("kind"), it.get("pts"), it.get("walls")) for it in items or []
                 if it.get("kind") in KEEPOUT_KINDS])
