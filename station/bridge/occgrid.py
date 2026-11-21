"""Occupancy grid (issue #12): log-odds cells, integer ray casting, PNG tiles, save / load.

Stdlib only: it runs inside the bridge on the Pi.

- One byte per cell in a bytearray: 128 = unknown (log-odds 0), +1 = UNIT nats. Updates go through
  256-entry lookup tables (add + clamp in one index), so a ray costs one list comprehension
  (integer Bresenham) and one `g[i] = lut[g[i]]` per cell.
- The grid is fixed (default 20 x 20 m, 25 mm cells, centred on the odometry origin) and split
  into tiles (1 m); changed tiles are tracked so browsers only get what changed.
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

    def __init__(self, size_m=20.0, res_mm=25.0, tile_m=1.0, hit=0.85, free=0.4, lo=-2.0, hi=3.5):
        self.res = float(res_mm)
        self.T = max(4, int(round(tile_m * 1000.0 / self.res)))         # cells per tile side
        self.ntx = self.nty = max(1, int(math.ceil(size_m * 1000.0 / (self.res * self.T))))
        self.W, self.H = self.ntx * self.T, self.nty * self.T
        self.x0 = -self.W * self.res / 2.0                                # world mm of the cell (0, 0) corner
        self.y0 = -self.H * self.res / 2.0
        self.cells = bytearray([UNKNOWN]) * (self.W * self.H)
        self.dirty = set()        # tiles changed since the last take_dirty()
        self.touched = set()      # tiles ever changed (what a full update sends)
        self.updates = 0          # rays integrated
        self.set_model(hit, free, lo, hi)

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
        self.weak_lut = bytes(v if v <= LO else max(LO, v - fw) for v in range(256))
        self.LO, self.HI = LO, HI

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
        r = self.res * self.T
        tx0 = max(0, int((min(xs) - self.x0) // r))
        tx1 = min(self.ntx - 1, int((max(xs) - self.x0) // r))
        ty0 = max(0, int((min(ys) - self.y0) // r))
        ty1 = min(self.nty - 1, int((max(ys) - self.y0) // r))
        d = self.dirty
        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                d.add((tx, ty))

    # ------------------------------------------------------------ updates
    def integrate(self, ox, oy, free_ends, hit=None, weak=False):
        """One reading. ox, oy: beam origin (mm). free_ends: world points; every cell on the segment
        origin -> point is observed free. hit: world point of the target (its cell is observed
        occupied; never also marked free), or None. weak: half-strength free update ("no target")."""
        free = set()
        xs, ys = [ox], [oy]
        for ex, ey in free_ends:
            idx, _ = self.line(ox, oy, ex, ey)
            free.update(idx)
            xs.append(ex)
            ys.append(ey)
        g = self.cells
        hi = None
        if hit is not None:
            hi = self.index(hit[0], hit[1])
            xs.append(hit[0])
            ys.append(hit[1])
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
        line through its row's centre crosses it inside the triangle; every row the triangle covers
        gets at least the cell(s) under that line, so the apex (the sensor) is always included."""
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

    def integrate_wedge(self, ox, oy, a, half, r_free, hit=None, weak=False, hit_half=0.0):
        """One reading with a beam cone: cells in the wedge (half-angle `half` rad, out to r_free mm)
        are observed free. hit = range (mm) of the target on the beam centre, or None: the cells on
        the arc at that range, +-hit_half rad wide, are observed occupied (and never also freed)."""
        g = self.cells
        hits = []
        if hit is not None:
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
        for s, e in spans:
            g[s:e] = g[s:e].translate(lut)
        hl = self.hit_lut
        for i, v in hits:
            g[i] = hl[v]
        if spans or hits:
            r = max(r_free, hit or 0.0)
            w = max(half, hit_half)
            self._mark([ox, ox + r * math.cos(a - w), ox + r * math.cos(a + w), ox + r * math.cos(a)],
                       [oy, oy + r * math.sin(a - w), oy + r * math.sin(a + w), oy + r * math.sin(a)])
            self.updates += 1
    def reset(self):
        self.cells[:] = bytearray([UNKNOWN]) * len(self.cells)
        self.dirty.clear()
        self.touched.clear()
        self.updates = 0

    # ------------------------------------------------------------ tiles / images
    def take_dirty(self):
        d, self.dirty = self.dirty, set()
        self.touched |= d
        return sorted(d)

    def tile_rows(self, tx, ty, lut=GREY):
        """Rows of one tile, top (high y) first, mapped through lut."""
        T, W, g = self.T, self.W, self.cells
        x0 = tx * T
        return [bytes(g[j * W + x0: j * W + x0 + T]).translate(lut)
                for j in range(ty * T + T - 1, ty * T - 1, -1)]

    def tile_png(self, tx, ty):
        return png_grey(self.T, self.T, self.tile_rows(tx, ty))

    def tiles_msg(self, tiles):
        """[[tx, ty, base64 png], ...]"""
        return [[tx, ty, base64.b64encode(self.tile_png(tx, ty)).decode("ascii")] for tx, ty in tiles]

    def info(self):
        return {"res_mm": self.res, "x0_mm": self.x0, "y0_mm": self.y0, "w": self.W, "h": self.H,
                "tile": self.T, "ntx": self.ntx, "nty": self.nty}

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

    # ------------------------------------------------------------ save / load
    def to_json(self, extra=None):
        d = {"format": FORMAT, "res_mm": self.res, "tile": self.T, "ntx": self.ntx, "nty": self.nty,
             "x0_mm": self.x0, "y0_mm": self.y0, "unit": UNIT, "model": self.model,
             "touched": sorted(self.touched | self.dirty),
             "cells": base64.b64encode(zlib.compress(bytes(self.cells), 6)).decode("ascii")}
        if extra:
            d.update(extra)
        return json.dumps(d, separators=(",", ":"))

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        if d.get("format") != FORMAT:
            raise ValueError("not an sbr3 occupancy grid")
        g = cls.__new__(cls)
        g.res = float(d["res_mm"])
        g.T, g.ntx, g.nty = int(d["tile"]), int(d["ntx"]), int(d["nty"])
        g.W, g.H = g.ntx * g.T, g.nty * g.T
        g.x0, g.y0 = float(d["x0_mm"]), float(d["y0_mm"])
        cells = zlib.decompress(base64.b64decode(d["cells"]))
        if len(cells) != g.W * g.H:
            raise ValueError("cell data size mismatch")
        g.cells = bytearray(cells)
        g.dirty = set()
        g.touched = set(tuple(t) for t in d.get("touched", []))
        g.updates = 0
        m = d.get("model") or {}
        g.set_model(**{k: float(m[k]) for k in ("hit", "free", "lo", "hi") if k in m})
        g.meta = {k: v for k, v in d.items() if k not in ("cells", "touched")}
        return g
