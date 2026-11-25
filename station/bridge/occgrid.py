"""Occupancy grid (issue #12): log-odds cells, integer ray casting, PNG tiles, save / load.

Stdlib only: it runs inside the bridge on the Pi.

- One byte per cell in a bytearray: 128 = unknown (log-odds 0), +1 = UNIT nats. Updates go through
  256-entry lookup tables (add + clamp in one index), so a ray costs one list comprehension
  (integer Bresenham) and one `g[i] = lut[g[i]]` per cell.
- The grid starts at 20 x 20 m (25 mm cells, centred on the odometry origin), is split into 1 m tiles
  and grows by whole tiles when a reading reaches past its edge (one reallocation, rare; the hot path
  stays one flat array). max_cells caps the memory (default 64 M cells = 64 MB, 200 x 200 m): past it
  the reading is clipped as before and `warning` says so. Tile indices (dirty, touched, bbox_tiles,
  tile_png) are relative to the grid's corner; tiles_msg() sends browsers absolute ones (world
  mm / (tile size), stable while the grid grows). Saved files hold the initial area plus the touched tiles.
- Range weighting (integrate_wedge rw=True): a hit up close is strong and narrow, a far one weak (a
  25 degree beam only tells which cone the target is in, so a far hit must not wall off a doorway
  next to it); a close reading's free space counts double, so it clears cells that far hits marked.
- Tiles go to browsers as 8-bit greyscale PNGs (unknown grey, free white, occupied black).
"""
import base64
import json
import math
import struct
import zlib

UNIT = 0.05            # nats per stored step
UNKNOWN = 128
FORMAT = "sbr3-occgrid-1"
MAX_CELLS = 64_000_000   # memory cap: one byte per cell
# range weighting of a hit: weight (x the hit step) and arc width scale fall / rise linearly near -> far
RW = {"near": 500.0, "far": 2000.0, "w_near": 1.6, "w_far": 0.4, "arc_near": 0.7, "arc_far": 1.0,
      "arc_cap_mm": 120.0, "free_near_mm": 800.0}
RW_LEVELS = 8


def _clamp_byte(v):
    return 1 if v < 1 else (255 if v > 255 else int(v))


def prob(v):
    """Stored byte -> occupancy probability."""
    return 1.0 / (1.0 + math.exp(-(v - UNKNOWN) * UNIT))


# byte -> display grey (255 white = free, 0 black = occupied, 128 unknown)
GREY = bytes(min(255, max(0, round(255 * (1.0 - prob(v))))) for v in range(256))   # GREY[128] = 128


def _png_chunk(kind, data):
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def png_grey(w, h, rows):
    """8-bit greyscale PNG from h rows of w bytes (top row first)."""
    raw = b"".join(b"\x00" + r for r in rows)
    return (b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + _png_chunk(b"IDAT", zlib.compress(raw, 6)) + _png_chunk(b"IEND", b""))


class Grid:
    """size_m: side of the square map; res_mm: cell size; tile_m: tile side.
    hit / free: log-odds added for an occupied / free observation (nats); lo / hi: clamp (nats)."""

    def __init__(self, size_m=20.0, res_mm=25.0, tile_m=1.0, hit=0.85, free=0.4, lo=-2.0, hi=3.5,
                 max_cells=MAX_CELLS):
        self.res = float(res_mm)
        self.T = max(4, int(round(tile_m * 1000.0 / self.res)))         # cells per tile side
        n = max(1, int(math.ceil(size_m * 1000.0 / (self.res * self.T))))
        self.init_n = (n, n)                  # tiles of the initial (centred) area
        self.max_cells = int(max_cells)
        self.warning = None                   # set when the memory cap stopped the map from growing
        self.updates = 0                      # rays integrated
        self.set_model(hit, free, lo, hi)
        self._start()

    def _start(self):
        """(Re)allocate the initial area, centred on the origin."""
        nx, ny = self.init_n
        self._set_area(-(nx // 2) * self.T, -(ny // 2) * self.T, nx * self.T, ny * self.T,
                       bytearray([UNKNOWN]) * (nx * self.T * ny * self.T))
        self.dirty = set()        # tiles changed since the last take_dirty() (relative to the corner)
        self.touched = set()      # tiles ever changed (what a full update sends)
        self.fdirty = set()       # tiles changed since the scan matcher's likelihood field last looked (#78)
        self.warning = None

    def _set_area(self, cx0, cy0, W, H, cells):
        """cx0, cy0: world cell (floor(mm / res)) of the grid's cell (0, 0); a multiple of the tile size."""
        self.cx0, self.cy0, self.W, self.H, self.cells = cx0, cy0, W, H, cells
        self.ntx, self.nty = W // self.T, H // self.T
        self.x0, self.y0 = cx0 * self.res, cy0 * self.res                 # world mm of the (0, 0) corner
        self.bx1, self.by1 = self.x0 + W * self.res, self.y0 + H * self.res

    def tile_origin(self):
        """Absolute tile index of the grid's tile (0, 0) (what tiles_msg adds)."""
        return self.cx0 // self.T, self.cy0 // self.T

    def mem_mb(self):
        return len(self.cells) / 1e6

    def _reshape(self, nx0, ny0, nx1, ny1):
        """Grow to the world-cell rectangle [nx0, nx1) x [ny0, ny1) (tile aligned, contains the old one)."""
        W, H, T = self.W, self.H, self.T
        nW, nH = nx1 - nx0, ny1 - ny0
        dx, dy = self.cx0 - nx0, self.cy0 - ny0
        new = bytearray([UNKNOWN]) * (nW * nH)
        old = self.cells
        for j in range(H):
            o = (j + dy) * nW + dx
            new[o:o + W] = old[j * W:(j + 1) * W]
        dtx, dty = dx // T, dy // T
        self.dirty = {(a + dtx, b + dty) for a, b in self.dirty}
        self.touched = {(a + dtx, b + dty) for a, b in self.touched}
        self.fdirty = {(a + dtx, b + dty) for a, b in self.fdirty}
        self._set_area(nx0, ny0, nW, nH, new)

    def _grow(self, xmin, xmax, ymin, ymax):
        """Make the grid cover this world box (mm), with a tile of slack on the sides that grow.
        False (and `warning` set) when that would pass max_cells."""
        if not (math.isfinite(xmin) and math.isfinite(xmax) and math.isfinite(ymin) and math.isfinite(ymax)):
            return False
        r, T = self.res, self.T
        a0, a1 = int(math.floor(xmin / r)), int(math.floor(xmax / r))
        b0, b1 = int(math.floor(ymin / r)), int(math.floor(ymax / r))
        ex0, ex1, ey0, ey1 = self.cx0, self.cx0 + self.W, self.cy0, self.cy0 + self.H
        for slack in (1, 0):
            nx0 = ex0 if a0 >= ex0 else (a0 // T - slack) * T
            nx1 = ex1 if a1 < ex1 else (a1 // T + 1 + slack) * T
            ny0 = ey0 if b0 >= ey0 else (b0 // T - slack) * T
            ny1 = ey1 if b1 < ey1 else (b1 // T + 1 + slack) * T
            if (nx1 - nx0) * (ny1 - ny0) <= self.max_cells:
                self._reshape(nx0, ny0, nx1, ny1)
                return True
        if not self.warning:
            self.warning = "map memory limit reached (%d MB, %.0f x %.0f m): readings beyond it are not mapped" % (
                self.max_cells // 1000000, self.W * r / 1000.0, self.H * r / 1000.0)
        return False

    def _cover(self, xs, ys):
        """Grow the grid to hold these world points when they reach past its edge."""
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        if x0 < self.x0 or x1 >= self.bx1 or y0 < self.y0 or y1 >= self.by1:
            self._grow(x0, x1, y0, y1)

    # ------------------------------------------------------------ sensor model
    def set_model(self, hit=0.85, free=0.4, lo=-2.0, hi=3.5):
        self.model = dict(hit=hit, free=free, lo=lo, hi=hi)
        LO = _clamp_byte(UNKNOWN + round(lo / UNIT))
        HI = _clamp_byte(UNKNOWN + round(hi / UNIT))
        h, f = max(1, round(hit / UNIT)), max(1, round(free / UNIT))
        fw = max(1, round(free / 2 / UNIT))
        # values already beyond the clamp (a map saved with other limits) are not pulled back
        self.hit_lut = bytes(v if v >= HI else min(HI, v + h) for v in range(256))
        self.free_lut = bytes(v if v <= LO else max(LO, v - f) for v in range(256))
        self.free2_lut = bytes(v if v <= LO else max(LO, v - 2 * f) for v in range(256))
        self.weak_lut = bytes(v if v <= LO else max(LO, v - fw) for v in range(256))
        self.LO, self.HI = LO, HI
        # range-weighted hits: level 0 (near) .. RW_LEVELS (far) -> (hit lut, arc width scale)
        self.rw_levels = []
        for k in range(RW_LEVELS + 1):
            t = k / RW_LEVELS
            hk = max(1, round(h * (RW["w_near"] + (RW["w_far"] - RW["w_near"]) * t)))
            self.rw_levels.append((bytes(v if v >= HI else min(HI, v + hk) for v in range(256)),
                                   RW["arc_near"] + (RW["arc_far"] - RW["arc_near"]) * t))

    # ------------------------------------------------------------ geometry
    def cell_of(self, x, y):
        """World mm -> (ix, iy) (may be outside the grid)."""
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def inside(self, ix, iy):
        return 0 <= ix < self.W and 0 <= iy < self.H

    def index(self, x, y):
        ix, iy = self.cell_of(x, y)
        return iy * self.W + ix if self.inside(ix, iy) else None

    def value(self, x, y):
        i = self.index(x, y)
        return UNKNOWN if i is None else self.cells[i]

    def _clip(self, ax, ay, bx, by):
        """Liang-Barsky: segment in cell units clipped to the grid; None if it misses."""
        t0, t1 = 0.0, 1.0
        dx, dy = bx - ax, by - ay
        hi_x, hi_y = self.W - 1e-6, self.H - 1e-6
        for p, q in ((-dx, ax), (dx, hi_x - ax), (-dy, ay), (dy, hi_y - ay)):
            if p == 0:
                if q < 0:
                    return None
                continue
            r = q / p
            if p < 0:
                if r > t1:
                    return None
                t0 = max(t0, r)
            else:
                if r < t0:
                    return None
                t1 = min(t1, r)
        return ax + t0 * dx, ay + t0 * dy, ax + t1 * dx, ay + t1 * dy, t1 >= 1.0

    def line(self, x0, y0, x1, y1):
        """Cell indices on the segment between two world points (mm), both ends included, clipped to
        the grid. Returns (indices, end_inside)."""
        r = self.res
        c = self._clip((x0 - self.x0) / r, (y0 - self.y0) / r, (x1 - self.x0) / r, (y1 - self.y0) / r)
        if c is None:
            return [], False
        ax, ay, bx, by, end_in = c
        i0, j0, i1, j1 = int(ax), int(ay), int(bx), int(by)
        return self.line_cells(i0, j0, i1, j1), end_in

    def line_cells(self, i0, j0, i1, j1):
        """Integer Bresenham (one cell per step of the major axis), as flat indices."""
        W = self.W
        dx, dy = i1 - i0, j1 - j0
        n = max(abs(dx), abs(dy))
        base = j0 * W + i0
        if n == 0:
            return [base]
        n2 = 2 * n
        if abs(dx) >= abs(dy):
            s = 1 if dx > 0 else -1
            return [base + k * s + ((2 * k * dy + n) // n2) * W for k in range(n + 1)]
        s = W if dy > 0 else -W
        return [base + k * s + (2 * k * dx + n) // n2 for k in range(n + 1)]

    def _mark(self, xs, ys):
        """Tiles covered by the bounding box of these world points -> dirty / touched."""
        self._mark_box(min(xs), max(xs), min(ys), max(ys))

    def _mark_box(self, xmin, xmax, ymin, ymax):
        r = self.res * self.T
        tx0 = max(0, int((xmin - self.x0) // r))
        tx1 = min(self.ntx - 1, int((xmax - self.x0) // r))
        ty0 = max(0, int((ymin - self.y0) // r))
        ty1 = min(self.nty - 1, int((ymax - self.y0) // r))
        d, f = self.dirty, self.fdirty
        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                d.add((tx, ty))
                f.add((tx, ty))

    # ------------------------------------------------------------ updates
    def integrate(self, ox, oy, free_ends, hit=None, weak=False):
        """One reading. ox, oy: beam origin (mm). free_ends: world points; every cell on the segment
        origin -> point is observed free. hit: world point of the target (its cell is observed
        occupied; never also marked free), or None. weak: half-strength free update ("no target")."""
        free = set()
        xs, ys = [ox], [oy]
        for ex, ey in free_ends:
            xs.append(ex)
            ys.append(ey)
        if hit is not None:
            xs.append(hit[0])
            ys.append(hit[1])
        self._cover(xs, ys)
        for ex, ey in free_ends:
            idx, _ = self.line(ox, oy, ex, ey)
            free.update(idx)
        g = self.cells
        hi = None
        if hit is not None:
            hi = self.index(hit[0], hit[1])
            if hi is not None:
                free.discard(hi)
        lut = self.weak_lut if weak else self.free_lut
        for i in free:
            g[i] = lut[g[i]]
        if hi is not None:
            g[hi] = self.hit_lut[g[hi]]
        if free or hi is not None:
            self._mark(xs, ys)
            self.updates += 1

    def wedge_spans(self, ox, oy, a, half, r):
        """Cells of the triangle origin -> (r, a - half) -> (r, a + half) (world mm / rad) as row spans
        [(start, stop), ...] of flat indices, clipped to the grid. A cell is in when the horizontal
        line through its row's centre crosses it inside the triangle (the thin tip at the sensor may be
        skipped: it is the robot's own spot)."""
        res, W, H = self.res, self.W, self.H
        p = [((ox - self.x0) / res, (oy - self.y0) / res)]
        for s in (-half, half):
            p.append(((ox + r * math.cos(a + s) - self.x0) / res, (oy + r * math.sin(a + s) - self.y0) / res))
        p.sort(key=lambda q: q[1])
        (x0, y0), (x1, y1), (x2, y2) = p
        j_lo = max(0, int(math.floor(y0)))
        j_hi = min(H - 1, int(math.floor(y2)))
        if j_lo > j_hi or max(x0, x1, x2) < 0 or min(x0, x1, x2) >= W:
            return []
        k02 = (x2 - x0) / (y2 - y0) if y2 > y0 else 0.0
        k01 = (x1 - x0) / (y1 - y0) if y1 > y0 else 0.0
        k12 = (x2 - x1) / (y2 - y1) if y2 > y1 else 0.0
        spans = []
        for j in range(j_lo, j_hi + 1):
            yc = j + 0.5
            if yc < y0:
                yc = y0
            elif yc > y2:
                yc = y2
            xa = x0 + (yc - y0) * k02
            xb = x0 + (yc - y0) * k01 if yc < y1 else x1 + (yc - y1) * k12
            if xa > xb:
                xa, xb = xb, xa
            i0 = int(xa) if xa >= 0 else 0
            i1 = int(xb) if xb < W else W - 1
            if i0 <= i1:
                b = j * W
                spans.append((b + i0, b + i1 + 1))
        return spans

    def integrate_wedge(self, ox, oy, a, half, r_free, hit=None, weak=False, hit_half=0.0, rw=False):
        """One reading with a beam cone: cells in the wedge (half-angle `half` rad, out to r_free mm)
        are observed free. hit = range (mm) of the target on the beam centre, or None: the cells on
        the arc at that range, +-hit_half rad wide, are observed occupied (and never also freed).
        rw: range-weighted (see the module doc): hit evidence and arc width fall off with range, and the
        free space counts double when the reading is closer than RW['free_near_mm']."""
        r = max(r_free, hit or 0.0)
        if r <= 0:
            return
        w = max(half, hit_half)
        xs = [ox, ox + r * math.cos(a - w), ox + r * math.cos(a + w), ox + r * math.cos(a)]
        ys = [oy, oy + r * math.sin(a - w), oy + r * math.sin(a + w), oy + r * math.sin(a)]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        if x0 < self.x0 or x1 >= self.bx1 or y0 < self.y0 or y1 >= self.by1:
            self._grow(x0, x1, y0, y1)
        g = self.cells
        hits = []
        hl = self.hit_lut
        if hit is not None:
            if rw:
                t = (hit - RW["near"]) / (RW["far"] - RW["near"])
                hl, scale = self.rw_levels[int(RW_LEVELS * (0.0 if t < 0.0 else 1.0 if t > 1.0 else t) + 0.5)]
                hit_half = min(hit_half * scale, RW["arc_cap_mm"] / hit)
            n = int(hit * hit_half / self.res)
            step = self.res / hit if hit > 0 else 0.0
            seen = set()
            for k in range(-n, n + 1):
                i = self.index(ox + hit * math.cos(a + k * step), oy + hit * math.sin(a + k * step))
                if i is not None and i not in seen:
                    seen.add(i)
                    hits.append((i, g[i]))
        spans = self.wedge_spans(ox, oy, a, half, r_free) if r_free > 0 else []
        lut = self.weak_lut if weak else self.free_lut
        if rw and not weak and r_free <= RW["free_near_mm"]:
            lut = self.free2_lut                       # a close reading: its free space counts double
        for s, e in spans:
            g[s:e] = g[s:e].translate(lut)
        for i, v in hits:
            g[i] = hl[v]
        if spans or hits:
            self._mark_box(x0, x1, y0, y1)
            self.updates += 1

    def integrate_scan(self, ox, oy, runs, margin_mm=0.0, margin_frac=0.02, seen=None):
        """One spinning-lidar revolution (#76), dense. ox, oy: sensor origin; runs: lists of world points (mm) in
        rotation order, each list a stretch with no angular gap (the points of one run are neighbours, so the
        space between them is seen too).
        - free space: each run is the polygon origin -> its points, every point pulled back towards the origin by
          margin_mm + margin_frac * range (range noise: do not erase the wall cell), filled by scanline: one
          span per row, `translate` per span, so a 3 m scan costs a few hundred row operations, not 450 rays x 120 cells.
        - hits: the cell of every point, plus a line between neighbours 2 - 5 cells apart (125 mm: a wall 8 m away has points 112 mm apart) (a wall seen from far
          away is a line, not dots); each cell counts once per scan however many points land in it.
        Hits win over free space (applied after it). Returns the number of cells marked occupied.
        seen: a set of cells already hit by this revolution (a turn integrated in parts, #102: the runs are cut with one
        shared point, so the fans meet edge to edge; its cell counts once): skipped here, and this call's hits added."""
        pts = [p for run in runs for p in run]
        if not pts:
            return 0
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        xs.append(ox)
        ys.append(oy)
        self._cover(xs, ys)
        res, W, H, g, T = self.res, self.W, self.H, self.cells, self.T
        x0, y0 = self.x0, self.y0
        cox, coy = (ox - x0) / res, (oy - y0) / res
        changed = set()           # tiles whose bytes really changed (#102: not the whole bounding box of the turn)
        floor, ceil, hypot = math.floor, math.ceil, math.hypot
        lut = self.free_lut
        hit = set()
        for run in runs:
            poly = [(cox, coy)]
            prev = None
            for x, y in run:
                dx, dy = x - ox, y - oy
                r = hypot(dx, dy)
                k = 1.0 - (margin_mm + margin_frac * r) / r if r > 1e-6 else 0.0
                k = k if k > 0.0 else 0.0
                poly.append(((ox + dx * k - x0) / res, (oy + dy * k - y0) / res))
                i, j = int(floor((x - x0) / res)), int(floor((y - y0) / res))
                if 0 <= i < W and 0 <= j < H:
                    hit.add(j * W + i)
                    if prev is not None and 2 <= max(abs(i - prev[0]), abs(j - prev[1])) <= 5:
                        hit.update(self.line_cells(prev[0], prev[1], i, j))
                    prev = (i, j)
                else:
                    prev = None
            if len(poly) < 3:
                continue
            rows = {}
            poly.append(poly[0])
            for (xa, ya), (xb, yb) in zip(poly, poly[1:]):
                if ya == yb:
                    continue
                if ya > yb:
                    xa, ya, xb, yb = xb, yb, xa, ya
                j0, j1 = max(0, int(ceil(ya - 0.5))), min(H, int(ceil(yb - 0.5)))
                if j1 <= j0:
                    continue
                slope = (xb - xa) / (yb - ya)
                for j in range(j0, j1):
                    c = xa + (j + 0.5 - ya) * slope
                    if j in rows:
                        rows[j].append(c)
                    else:
                        rows[j] = [c]
            for j, cs in rows.items():
                cs.sort()
                b = j * W
                ty = j // T
                for k in range(0, len(cs) - 1, 2):
                    i0, i1 = max(0, int(ceil(cs[k] - 0.5))), min(W, int(ceil(cs[k + 1] - 0.5)))
                    if i1 > i0:
                        old = g[b + i0:b + i1]
                        new = old.translate(lut)
                        if new != old:            # free space seen again (already at the clamp) changes nothing
                            g[b + i0:b + i1] = new
                            for tx in range(i0 // T, (i1 - 1) // T + 1):
                                changed.add((tx, ty))
        if seen is not None:
            hit -= seen
            seen |= hit
        hl = self.hit_lut
        for i in hit:
            v = g[i]
            nv = hl[v]
            if nv != v:
                g[i] = nv
                changed.add(((i % W) // T, i // W // T))
        if changed:
            self.dirty |= changed
            self.fdirty |= changed
        self.updates += 1
        return len(hit)

    def clear_disc(self, x, y, r):
        """Cells whose centre is within r mm of (x, y) are observed free: the robot's own body stands there
        (a doorway it drove through is open, whatever far cone hits drew into it)."""
        self._cover((x - r, x + r), (y - r, y + r))
        res, W, g, lut = self.res, self.W, self.cells, self.free_lut
        ix0, iy0 = self.cell_of(x - r, y - r)
        ix1, iy1 = self.cell_of(x + r, y + r)
        n = 0
        for iy in range(max(0, iy0), min(self.H - 1, iy1) + 1):
            dy = self.y0 + (iy + 0.5) * res - y
            for ix in range(max(0, ix0), min(W - 1, ix1) + 1):
                dx = self.x0 + (ix + 0.5) * res - x
                if dx * dx + dy * dy <= r * r:
                    i = iy * W + ix
                    g[i] = lut[g[i]]
                    n += 1
        if n:
            self._mark_box(x - r, x + r, y - r, y + r)
            self.updates += 1

    def reset(self):
        self._start()
        self.updates = 0
    # ------------------------------------------------------------ tiles / images
    def take_dirty(self):
        d, self.dirty = self.dirty, set()
        self.touched |= d
        return sorted(d)

    def tile_raw(self, tx, ty):
        """One tile's cell bytes, bottom row first (what the change check compares, #102)."""
        T, W, g = self.T, self.W, self.cells
        x0 = tx * T
        return b"".join([g[j * W + x0: j * W + x0 + T] for j in range(ty * T, ty * T + T)])

    def png_of_raw(self, raw):
        """tile_raw() bytes -> the tile's PNG (top row first, greyscale)."""
        T = self.T
        grey = raw.translate(GREY)
        return png_grey(T, T, [grey[k * T:(k + 1) * T] for k in range(T - 1, -1, -1)])

    def tile_rows(self, tx, ty, lut=GREY):
        """Rows of one tile, top (high y) first, mapped through lut."""
        T, W, g = self.T, self.W, self.cells
        x0 = tx * T
        return [bytes(g[j * W + x0: j * W + x0 + T]).translate(lut)
                for j in range(ty * T + T - 1, ty * T - 1, -1)]

    def tile_png(self, tx, ty):
        return self.png_of_raw(self.tile_raw(tx, ty))

    def tiles_msg(self, tiles):
        """[[tx, ty, base64 png], ...] with absolute tile indices (world mm / tile size, floor): they
        stay valid while the grid grows. Pass tiles from take_dirty() / touched of the same moment."""
        ox, oy = self.tile_origin()
        return [[tx + ox, ty + oy, base64.b64encode(self.tile_png(tx, ty)).decode("ascii")] for tx, ty in tiles]

    def info(self):
        return {"res_mm": self.res, "x0_mm": self.x0, "y0_mm": self.y0, "w": self.W, "h": self.H,
                "tile": self.T, "ntx": self.ntx, "nty": self.nty, "tox": self.cx0 // self.T,
                "toy": self.cy0 // self.T, "mem_mb": round(self.mem_mb(), 1), "warning": self.warning}

    def bbox_tiles(self):
        """(tx0, ty0, tx1, ty1) of the touched tiles, or None."""
        t = self.touched | self.dirty
        if not t:
            return None
        return min(a for a, _ in t), min(b for _, b in t), max(a for a, _ in t), max(b for _, b in t)

    def png(self, scale=1):
        """Whole touched area as one PNG (for downloads / the report); scale = pixels per cell."""
        bb = self.bbox_tiles()
        if bb is None:
            return png_grey(1, 1, [b"\x80"])
        tx0, ty0, tx1, ty1 = bb
        T, W, g = self.T, self.W, self.cells
        x0, x1 = tx0 * T, (tx1 + 1) * T
        rows = []
        for j in range((ty1 + 1) * T - 1, ty0 * T - 1, -1):
            r = bytes(g[j * W + x0: j * W + x1]).translate(GREY)
            if scale > 1:
                r = bytes(b for b in r for _ in range(scale))
            rows += [r] * scale
        return png_grey((x1 - x0) * scale, len(rows), rows)

    def counts(self, occ=0.65, free=0.35):
        """Cells above occ / below free probability (for stats)."""
        o_th = UNKNOWN + math.log(occ / (1 - occ)) / UNIT
        f_th = UNKNOWN + math.log(free / (1 - free)) / UNIT
        hist = [0] * 256
        for tx, ty in self.touched | self.dirty:
            for r in self.tile_rows(tx, ty, lut=bytes(range(256))):
                for v in r:
                    hist[v] += 1
        n_occ = sum(c for v, c in enumerate(hist) if v > o_th)
        n_free = sum(c for v, c in enumerate(hist) if v < f_th)
        return n_occ, n_free

    def snapshot(self):
        """A frozen copy to save from a worker thread (#102): the cells are copied now (a few ms on the Pi for a 20 m map),
        so the thread never sees a half-updated grid; to_json() / png() work on it as on the grid itself."""
        g = Grid.__new__(Grid)
        g.__dict__.update(self.__dict__)
        g.cells = bytes(self.cells)
        g.touched = self.touched | self.dirty
        g.dirty, g.fdirty = set(), set()
        return g

    # ------------------------------------------------------------ save / load
    def to_json(self, extra=None):
        """The initial area plus everything touched (a grown map is cropped to that, so a rarely
        used huge allocation does not make saving slow)."""
        T, W, ox0, oy0 = self.T, self.W, self.cx0, self.cy0
        ix0, iy0 = -(self.init_n[0] // 2) * T, -(self.init_n[1] // 2) * T
        x0, y0, x1, y1 = ix0, iy0, ix0 + self.init_n[0] * T, iy0 + self.init_n[1] * T
        bb = self.bbox_tiles()
        if bb:
            x0, y0 = min(x0, ox0 + bb[0] * T), min(y0, oy0 + bb[1] * T)
            x1, y1 = max(x1, ox0 + (bb[2] + 1) * T), max(y1, oy0 + (bb[3] + 1) * T)
        x0, y0, x1, y1 = max(x0, ox0), max(y0, oy0), min(x1, ox0 + W), min(y1, oy0 + self.H)
        if (x0, y0, x1, y1) == (ox0, oy0, ox0 + W, oy0 + self.H):
            cells = bytes(self.cells)
        else:
            cells = b"".join(bytes(self.cells[(j - oy0) * W + x0 - ox0:(j - oy0) * W + x1 - ox0]) for j in range(y0, y1))
        dtx, dty = (x0 - ox0) // T, (y0 - oy0) // T
        d = {"format": FORMAT, "res_mm": self.res, "tile": T, "ntx": (x1 - x0) // T, "nty": (y1 - y0) // T,
             "x0_mm": x0 * self.res, "y0_mm": y0 * self.res, "unit": UNIT, "model": self.model,
             "init_n": list(self.init_n),
             "touched": sorted((a - dtx, b - dty) for a, b in self.touched | self.dirty),
             "cells": base64.b64encode(zlib.compress(cells, 6)).decode("ascii")}
        if extra:
            d.update(extra)
        return json.dumps(d, separators=(",", ":"))

    @classmethod
    def from_json(cls, text, max_cells=MAX_CELLS):
        d = json.loads(text)
        if d.get("format") != FORMAT:
            raise ValueError("not an sbr3 occupancy grid")
        g = cls.__new__(cls)
        g.res = float(d["res_mm"])
        g.T = int(d["tile"])
        ntx, nty = int(d["ntx"]), int(d["nty"])
        W, H = ntx * g.T, nty * g.T
        cx0, cy0 = int(round(float(d["x0_mm"]) / g.res)), int(round(float(d["y0_mm"]) / g.res))
        cells = zlib.decompress(base64.b64decode(d["cells"]))
        if len(cells) != W * H:
            raise ValueError("cell data size mismatch")
        g.max_cells = int(max_cells)
        g.warning = None
        g.updates = 0
        n = d.get("init_n") or [ntx, nty]
        g.init_n = (int(n[0]), int(n[1]))
        g.dirty = set()
        g.touched = set(tuple(t) for t in d.get("touched", []))
        g.fdirty = set(g.touched)
        m = d.get("model") or {}
        g.set_model(**{k: float(m[k]) for k in ("hit", "free", "lo", "hi") if k in m})
        g._set_area(cx0, cy0, W, H, bytearray(cells))
        if cx0 % g.T or cy0 % g.T or W % g.T:       # a map saved with tiles not aligned to the world: pad it
            T = g.T
            old_touched = g.touched
            g.touched = set()
            g._reshape((cx0 // T) * T, (cy0 // T) * T, -((-(cx0 + W)) // T) * T, -((-(cy0 + H)) // T) * T)
            g.touched = {(x, y) for x in range(g.ntx) for y in range(g.nty)} if old_touched else set()
        g.meta = {k: v for k, v in d.items() if k not in ("cells", "touched")}
        return g