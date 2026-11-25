"""Frontier search for the explore node (issue #107), numpy + scipy, no ROS.

  find_frontiers(grid, res, origin, robot_xy, cost=None, blacklist=(), p=Params()) -> (frontiers, info)

grid: the /map data as an (H, W) int array (-1 unknown, 0..100 occupancy), row 0 at origin y.
cost: the Nav2 global costmap resampled onto the same cells (-1 unknown, 0..100; 99 inscribed, 100 lethal) or None.

1. Cells: free (<= free_max), occupied (>= occ_min), unknown; unknown specks with 3+ free 4-neighbours count as free;
   gaps in walls up to close_gaps wide count as wall (a far wall seen sparsely is no row of tiny frontiers).
2. Travel: known free cells at least robot_radius + travel_margin from a wall (and not inscribed / lethal in the
   costmap); near the robot everything but walls (its own footprint is often unseen or inflated).
   Path lengths from the robot by Dijkstra (scipy csgraph), a little dearer near walls.
3. Frontier cells: free cells with an unknown 4-neighbour, clustered (8-connected); clusters shorter than min_size
   or within black_r of a blacklisted point are dropped.
4. Viewpoint per cluster: a reachable cell within view_r of the cluster with goal_clear from walls (costmap cost <=
   goal_max_cost); if there is none, the reachable cell there with the most clearance (>= robot_radius + 2 cm).
   No viewpoint: the cluster is not reachable (yet) and is left out.
5. Score: information gain (the share of a square around the cluster in the unknown regions it borders) + size - distance, and a bonus when
   the frontier "opens up" into new space (unknown deeper than open_depth just beyond it; a doorway into an unseen
   room, not a strip of shadow along a wall - like station/bridge/planner.py's _opens_up for squeeze goals).
"""
import math
import os
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

UNKNOWN, FREE, OCC = 0, 1, 2
N8 = np.ones((3, 3), dtype=bool)
N4 = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)


@dataclass
class Params:
    free_max: int = 25               # map_saver free_thresh 0.25
    occ_min: int = 65                # map_saver occupied_thresh 0.65
    close_gaps: float = 0.20         # gaps in walls up to this wide count as wall (no frontier through them)
    robot_radius: float = 0.12       # nav2.yaml robot_radius
    travel_margin: float = 0.02      # travel cells: >= robot_radius + this from a wall
    escape_r: float = 0.35           # around the robot: anything but a wall is passable
    wall_cost: float = 1.5           # path steps cost (1 + wall_cost * costmap / 100)
    min_size: float = 0.30           # m of frontier (cells x res) or the cluster is ignored
    min_unknown: float = 0.05        # m^2: ... and the unknown region behind it must be at least this big
    view_r: float = 0.80             # viewpoint within this of the frontier cluster
    goal_clear: float = 0.30         # viewpoint clearance from walls (m), when there is room
    goal_max_cost: int = 60          # ... and costmap cost at most this (inflation 0.45 m: 60 ~ 0.3 m from a wall)
    gain_r: float = 1.50             # information gain: unknown share of a (2 gain_r)^2 square
    open_look: float = 0.50          # opens up: look this far beyond the frontier ...
    open_depth: float = 0.25         # ... for unknown this deep (distance to the nearest known cell)
    black_r: float = 0.40            # blacklisted points block frontier cells within this
    w_gain: float = 2.0
    w_size: float = 0.5
    w_open: float = 1.0
    w_dist: float = 0.6


@dataclass
class Frontier:
    cells: int
    size_m: float
    centroid: tuple                  # (x, y) m, map frame
    point: tuple                     # the cluster cell nearest the viewpoint
    viewpoint: tuple
    dist_m: float                    # path length robot -> viewpoint
    gain: float                      # 0..1
    opens: bool
    score: float = 0.0
    clearance: float = 0.0
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        r = lambda v: [round(v[0], 3), round(v[1], 3)]
        return {"cells": self.cells, "size_m": round(self.size_m, 2), "point": r(self.point),
                "viewpoint": r(self.viewpoint), "dist_m": round(self.dist_m, 2), "gain": round(self.gain, 3),
                "opens": self.opens, "score": round(self.score, 3)}


def classify(grid, p):
    g = np.asarray(grid)
    st = np.full(g.shape, UNKNOWN, dtype=np.uint8)
    st[(g >= 0) & (g <= p.free_max)] = FREE
    st[g >= p.occ_min] = OCC
    # an unknown speck inside free space (a weakly seen cell) is free
    f = (st == FREE).astype(np.int8)
    n = np.zeros(g.shape, dtype=np.int8)
    n[1:, :] += f[:-1, :]
    n[:-1, :] += f[1:, :]
    n[:, 1:] += f[:, :-1]
    n[:, :-1] += f[:, 1:]
    st[(st == UNKNOWN) & (n >= 3)] = FREE
    return st


def frontier_mask(st):
    unk = st == UNKNOWN
    nb = np.zeros(st.shape, dtype=bool)
    nb[1:, :] |= unk[:-1, :]
    nb[:-1, :] |= unk[1:, :]
    nb[:, 1:] |= unk[:, :-1]
    nb[:, :-1] |= unk[:, 1:]
    return (st == FREE) & nb


def path_lengths(trav, start, step_cost, res):
    """Dijkstra over 8-connected travel cells from start (row, col): (H, W) metres, inf where unreachable."""
    H, W = trav.shape
    idx = np.full(trav.shape, -1, dtype=np.int64)
    n = int(trav.sum())
    out = np.full(trav.shape, np.inf)
    if n == 0 or not trav[start]:
        return out
    idx[trav] = np.arange(n)
    rows, cols, wts = [], [], []
    for dy, dx, L in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
        # pairs (r, c) - (r + dy, c + dx)
        a_sl = (slice(0, H - dy), slice(max(0, -dx), W - max(0, dx)))
        b_sl = (slice(dy, H), slice(max(0, dx), W - max(0, -dx)))
        a = trav[a_sl]
        b = trav[b_sl]
        both = a & b
        ia = idx[a_sl][both]
        ib = idx[b_sl][both]
        c = 0.5 * (step_cost[a_sl][both] + step_cost[b_sl][both]) * L * res
        rows.append(ia)
        cols.append(ib)
        wts.append(c)
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    wts = np.concatenate(wts)
    G = coo_matrix((wts, (rows, cols)), shape=(n, n)).tocsr()
    d = dijkstra(G, directed=False, indices=int(idx[start]))
    out[trav] = d
    return out


def find_frontiers(grid, res, origin, robot_xy, cost=None, blacklist=(), p=None):
    p = p or Params()
    grid = np.asarray(grid)
    H, W = grid.shape
    ox, oy = origin[0], origin[1]
    info = {"free_m2": 0.0, "reachable": False, "clusters": 0}
    st = classify(grid, p)
    info["free_m2"] = float((st == FREE).sum()) * res * res
    if p.close_gaps > 0:                                # walls seen sparsely (far, at a glance): close their gaps
        occ = st == OCC
        n = max(1, int(round(p.close_gaps / res / 2)))
        st[ndimage.binary_closing(occ, structure=N8, iterations=n) & ~occ] = OCC
    to_rc = lambda x, y: (int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res)))
    to_xy = lambda r, c: (ox + (c + 0.5) * res, oy + (r + 0.5) * res)
    rr, rc = to_rc(*robot_xy)
    if not (0 <= rr < H and 0 <= rc < W):
        info["why"] = "robot off the map"
        return [], info
    clear = ndimage.distance_transform_edt(st != OCC) * res
    cost = None if cost is None else np.asarray(cost)
    trav = (st == FREE) & (clear >= p.robot_radius + p.travel_margin)
    if cost is not None:
        trav &= ~((cost >= 99))
    yy, xx = np.ogrid[:H, :W]
    near = (yy - rr) ** 2 + (xx - rc) ** 2 <= (p.escape_r / res) ** 2
    trav |= near & (st != OCC) & (clear >= 0.5 * p.robot_radius)
    trav[rr, rc] = st[rr, rc] != OCC or trav[rr, rc]
    step = np.ones(grid.shape)
    if cost is not None:
        step += p.wall_cost * np.clip(cost, 0, 100) / 100.0
    if not trav[rr, rc]:
        info["why"] = "robot cell blocked"
        return [], info
    dist = path_lengths(trav, (rr, rc), step, res)
    reach = np.isfinite(dist)
    info["reachable"] = True
    info["reach_m2"] = float(reach.sum()) * res * res
    # viewpoint candidates
    ok_cost = np.ones(grid.shape, dtype=bool) if cost is None else (cost <= p.goal_max_cost)
    good = reach & (clear >= p.goal_clear) & ok_cost
    fair = reach & (clear >= p.robot_radius + 0.02) & ~near
    if cost is not None:
        fair &= cost < 99
    front = frontier_mask(st)
    lab, nlab = ndimage.label(front, structure=N8)
    info["clusters"] = int(nlab)
    if nlab == 0:
        return [], info
    unk = (st == UNKNOWN)
    ulab, _ = ndimage.label(unk, structure=N4)          # unknown regions (4-connected: not through wall corners)
    usize = np.bincount(ulab.ravel())
    usize[0] = 0
    udepth = ndimage.distance_transform_edt(unk) * res
    min_unk = max(1, int(round(p.min_unknown / (res * res))))
    min_cells = max(1, int(round(p.min_size / res)))
    bl = [tuple(b) for b in blacklist]
    gr = int(round(p.gain_r / res))
    vr = int(math.ceil(p.view_r / res))
    look = int(math.ceil(p.open_look / res))
    out = []
    for k, sl in enumerate(ndimage.find_objects(lab), start=1):
        if sl is None:
            continue
        r0, r1 = sl[0].start, sl[0].stop
        c0, c1 = sl[1].start, sl[1].stop
        cr, cc = np.nonzero(lab[sl] == k)
        cr = cr + r0
        cc = cc + c0
        if bl:
            xs = ox + (cc + 0.5) * res
            ys = oy + (cr + 0.5) * res
            keep = np.ones(len(cr), dtype=bool)
            for bx, by in bl:
                keep &= (xs - bx) ** 2 + (ys - by) ** 2 >= p.black_r ** 2
            cr, cc = cr[keep], cc[keep]
        if len(cr) < min_cells:
            continue
        # the unknown regions this frontier borders: a hole of a few cells in the floor is no frontier
        nl = np.concatenate([ulab[np.clip(cr + dy, 0, H - 1), np.clip(cc + dx, 0, W - 1)]
                             for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1))])
        nl = np.unique(nl[nl > 0])
        if len(nl) == 0 or usize[nl].max() < min_unk:
            continue
        # a window around the cluster: viewpoint, opens-up
        wr0, wr1 = max(0, cr.min() - max(vr, look)), min(H, cr.max() + max(vr, look) + 1)
        wc0, wc1 = max(0, cc.min() - max(vr, look)), min(W, cc.max() + max(vr, look) + 1)
        m = np.zeros((wr1 - wr0, wc1 - wc0), dtype=bool)
        m[cr - wr0, cc - wc0] = True
        dfront, (ir, ic) = ndimage.distance_transform_edt(~m, return_indices=True)
        dfront = dfront * res
        win = (slice(wr0, wr1), slice(wc0, wc1))
        cand = good[win] & (dfront <= p.view_r)
        if bl:
            wy, wx = np.mgrid[wr0:wr1, wc0:wc1]
            wxs, wys = ox + (wx + 0.5) * res, oy + (wy + 0.5) * res
            for bx, by in bl:
                cand &= (wxs - bx) ** 2 + (wys - by) ** 2 >= (0.75 * p.black_r) ** 2
        if cand.any():
            # nearest to the frontier; among those within 10 cm of that, the shortest path
            dd = np.where(cand, dfront, np.inf)
            best = dd.min()
            sel = cand & (dfront <= best + 0.10)
            score = np.where(sel, dist[win], np.inf)
        else:
            cand = fair[win] & (dfront <= p.view_r)
            if bl:
                for bx, by in bl:
                    cand &= (wxs - bx) ** 2 + (wys - by) ** 2 >= (0.75 * p.black_r) ** 2
            if not cand.any():
                continue
            score = np.where(cand, -clear[win] + 0.2 * dfront, np.inf)
        j = np.unravel_index(int(np.argmin(score)), score.shape)
        vr_, vc_ = j[0] + wr0, j[1] + wc0
        pr, pc = ir[j] + wr0, ic[j] + wc0
        # information gain around the cluster's middle
        mr, mc = int(round(cr.mean())), int(round(cc.mean()))
        gr0, gr1 = max(0, mr - gr), min(H, mr + gr + 1)
        gc0, gc1 = max(0, mc - gr), min(W, mc + gr + 1)
        gwin = ulab[gr0:gr1, gc0:gc1]
        gain = float(np.isin(gwin, nl).sum()) / float((2 * gr + 1) ** 2)
        # opens up: unknown deeper than open_depth within open_look beyond the frontier, reached through the
        # unknown only (straight into it, not over a wall into the unknown behind it)
        m_unk = unk[win]
        grow = ndimage.binary_dilation(m, structure=N4, iterations=look, mask=m_unk | m) & m_unk
        opens = bool(grow.any() and udepth[win][grow].max() >= p.open_depth)
        f = Frontier(cells=int(len(cr)), size_m=len(cr) * res, centroid=to_xy(cr.mean(), cc.mean()),
                     point=to_xy(pr, pc), viewpoint=to_xy(vr_, vc_), dist_m=float(dist[vr_, vc_]),
                     gain=float(gain), opens=opens, clearance=float(clear[vr_, vc_]))
        f.score = (p.w_gain * f.gain + p.w_size * min(f.size_m, 3.0) + (p.w_open if opens else 0.0)
                   - p.w_dist * f.dist_m)
        out.append(f)
    out.sort(key=lambda f: -f.score)
    return out, info


def resample(src, src_res, src_origin, dst_shape, dst_res, dst_origin, fill=-1):
    """Nearest-cell resample of grid src onto the dst cell centres (both row 0 at their origin y)."""
    src = np.asarray(src)
    H, W = dst_shape
    xs = dst_origin[0] + (np.arange(W) + 0.5) * dst_res
    ys = dst_origin[1] + (np.arange(H) + 0.5) * dst_res
    ci = np.floor((xs - src_origin[0]) / src_res).astype(np.int64)
    ri = np.floor((ys - src_origin[1]) / src_res).astype(np.int64)
    out = np.full(dst_shape, fill, dtype=src.dtype)
    okc = (ci >= 0) & (ci < src.shape[1])
    okr = (ri >= 0) & (ri < src.shape[0])
    out[np.ix_(okr, okc)] = src[np.ix_(ri[okr], ci[okc])]
    return out


# ---------------------------------------------------------------- map files (map_saver format)
def write_map(path_base, grid, res, origin, free_thresh=0.25, occ_thresh=0.65):
    """NAME.pgm + NAME.yaml like nav2 map_saver (trinary: free 254, occupied 0, unknown 205)."""
    g = np.asarray(grid)
    img = np.full(g.shape, 205, dtype=np.uint8)
    img[(g >= 0) & (g <= free_thresh * 100)] = 254
    img[g >= occ_thresh * 100] = 0
    img = img[::-1, :]                                  # pgm row 0 = the top (highest y)
    H, W = img.shape
    with open(path_base + ".pgm", "wb") as f:
        f.write(b"P5\n# CREATOR: sbr3_ros explore %.3f m/pix\n%d %d\n255\n" % (res, W, H))
        f.write(img.tobytes())
    with open(path_base + ".yaml", "w") as f:
        f.write("image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n"
                "occupied_thresh: %.2f\nfree_thresh: %.2f\n" % (os.path.basename(path_base), res, origin[0],
                                                               origin[1], occ_thresh, free_thresh))
    return img


def map_png(path, grid, res, origin, track=(), goals=(), scale=4):
    """A quick PNG of the map (+ the driven track, goals: green reached, red failed). Needs Pillow."""
    from PIL import Image, ImageDraw
    g = np.asarray(grid)
    img = np.full(g.shape, 160, dtype=np.uint8)
    img[(g >= 0) & (g <= 25)] = 245
    img[g >= 65] = 20
    H, W = g.shape
    im = Image.fromarray(img[::-1, :]).convert("RGB").resize((W * scale, H * scale), Image.NEAREST)
    d = ImageDraw.Draw(im)
    px = lambda x, y: ((x - origin[0]) / res * scale, (H - (y - origin[1]) / res) * scale)
    if len(track) > 1:
        d.line([px(x, y) for x, y in track], fill=(255, 136, 0), width=max(1, scale // 2))
    for gx, gy, ok in goals:
        x, y = px(gx, gy)
        c = (40, 170, 60) if ok else (220, 30, 30)
        r = scale + 3
        d.ellipse([x - r, y - r, x + r, y + r], outline=c, width=2)
    if track:
        x, y = px(*track[0])
        d.rectangle([x - 4, y - 4, x + 4, y + 4], outline=(40, 90, 255), width=2)
    im.save(path)
