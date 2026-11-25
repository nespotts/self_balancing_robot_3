"""Occupancy grid, mapper, planner (issue #12). Run: python -m unittest discover -s station -t ."""
import asyncio
import base64
import json
import math
import os
import shutil
import struct
import tempfile
import time
import unittest
import zlib

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.bridge.occgrid import GREY, UNKNOWN, Grid, png_grey
from station.bridge.planner import BLOCKED, DEFAULTS, END_GROW_MM, MARGIN, CostMap
from station.sim.sim import ROBOT_RADIUS, SimRobot, _box
from station.sim.tour import TOUR, DriftRobot, OfflineBridge, drive_offline, plan_offline


def png_size_and_pixels(data):
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", data[16:24])
    pos, idat = 8, b""
    while pos < len(data):
        n = struct.unpack(">I", data[pos:pos + 4])[0]
        kind = data[pos + 4:pos + 8]
        if kind == b"IDAT":
            idat += data[pos + 8:pos + 8 + n]
        pos += 12 + n
    raw = zlib.decompress(idat)
    rows = [raw[r * (w + 1) + 1:(r + 1) * (w + 1)] for r in range(h)]
    return w, h, rows


def reading(x=0.0, y=0.0, heading=0.0, angle=0.0, r=1000, valid=1, pitch=-3.0, sid=0):
    return {"t_us": 0, "x_mm": x, "y_mm": y, "heading_deg": heading, "pitch_deg": pitch,
            "sensor_angle_deg": angle, "range_mm": r, "sensor_id": sid, "valid": valid}


def doorway_run(args):
    """One explore of the two-room doorway sim (#62 / #68 / #108): width mm, seed. Returns a dict: closest approach
    to any wall, got through, and for every squeeze route the planner sent through the doorway (a fresh plan or a
    re-centred one): where it crosses the wall line against the centre of the gap as the squeeze map had it then, and
    the clearance it reported. Module level: the sweep runs it in worker processes."""
    from station.bridge import planner as P
    width, seed = args
    c = 600.0
    r = SimRobot(seed=seed)
    r.room.segs = _box(750, 0, 3500, 2000) + [(1000, -1000, 1000, c - width / 2.0), (1000, c + width / 2.0, 1000, 1000)]
    m = Mapper(tempfile.mkdtemp())
    out = {"width": width, "seed": seed, "x": -1e9, "close": 1e9, "jamb": 1e9, "cross": []}
    ends = [(1000, c - width / 2.0 - 150, 1000, c - width / 2.0), (1000, c + width / 2.0, 1000, c + width / 2.0 + 150)]

    def note(planner, path):
        cm = planner.costmap("squeeze")
        so = cm.solid()
        lo, hi = None, None
        for x in (987.5, 1012.5):
            for k in range(-12, 13):
                y = c + 12.5 + 25 * k
                i = cm.cell(x, y)
                if so[i]:
                    if y < c:
                        lo = y if lo is None else max(lo, y)
                    else:
                        hi = y if hi is None else min(hi, y)
        if lo is None or hi is None or hi - lo > 2 * (cm.hard_mm + 40.0):
            return                                     # one side open on the map: no gap centre to keep to
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            if (ax - 1000.0) * (bx - 1000.0) < 0:
                y = ay + (by - ay) * (1000.0 - ax) / (bx - ax)
                if lo < y < hi:
                    out["cross"].append((round(planner.clock(), 1), round(y - (lo + hi) / 2.0, 1),
                                         planner._sq_clear_now, (hi - lo) / 2.0 - cm.res / 2.0 - cm.radius_mm))

    orig_e, orig_c = P.Planner._plan_explore, P.Planner._check

    def plan_explore(self, pose):
        res = orig_e(self, pose)
        if res[0] == "path" and res[3] == "squeeze":
            note(self, res[1])
        return res

    def check(self, pose):
        res = orig_c(self, pose)
        if res[0] == "recentre" or (res[0] == "path" and res[3] == "squeeze"):
            note(self, res[1])
        return res

    def watch(robot, b):
        out["x"] = max(out["x"], robot.X)
        out["close"] = min(out["close"], robot.room.clearance(robot.X, robot.Y))
        out["jamb"] = min(out["jamb"], robot.room._seg_clearance(robot.X, robot.Y, ends))     # the doorway's jambs
    P.Planner._plan_explore, P.Planner._check = plan_explore, check
    try:
        plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=400, on_step=watch)
    finally:
        P.Planner._plan_explore, P.Planner._check = orig_e, orig_c
    out["through"] = out["x"] > 1000 + ROBOT_RADIUS + 20
    out["touch"] = out["close"] <= ROBOT_RADIUS + 1.0
    out["jamb_touch"] = out["jamb"] <= ROBOT_RADIUS + 1.0
    return out


def wall_grid(occ_x_mm=1000, y_range=(-1000, 1000), free_box=(-1500, -1500, 1500, 1500)):
    """A grid with a known free box and a vertical wall (occupied) at x = occ_x_mm."""
    g = Grid(size_m=8)
    for j in range(g.H):
        for i in range(g.W):
            x, y = g.x0 + (i + 0.5) * g.res, g.y0 + (j + 0.5) * g.res
            if free_box[0] <= x <= free_box[2] and free_box[1] <= y <= free_box[3]:
                g.cells[j * g.W + i] = g.LO
    for y in range(y_range[0], y_range[1], 10):
        for dx in (0, 25):
            g.cells[g.index(occ_x_mm + dx, y)] = g.HI
    g._mark([free_box[0], free_box[2]], [free_box[1], free_box[3]])
    g.take_dirty()
    return g


class TestGrid(unittest.TestCase):
    def test_line_cells_endpoints_and_steps(self):
        g = Grid(size_m=4)
        for (i0, j0, i1, j1) in [(5, 5, 40, 12), (40, 12, 5, 5), (10, 50, 13, 3), (7, 7, 7, 7), (3, 30, 60, 30)]:
            cells = g.line_cells(i0, j0, i1, j1)
            self.assertEqual(cells[0], j0 * g.W + i0)
            self.assertEqual(cells[-1], j1 * g.W + i1)
            self.assertEqual(len(cells), max(abs(i1 - i0), abs(j1 - j0)) + 1)
            for a, b in zip(cells, cells[1:]):      # 8-connected, one step at a time
                self.assertLessEqual(abs(a % g.W - b % g.W), 1)
                self.assertLessEqual(abs(a // g.W - b // g.W), 1)

    def test_line_clipped_to_grid(self):
        g = Grid(size_m=2)
        idx, end_in = g.line(0, 0, 5000, 0)
        self.assertFalse(end_in)
        self.assertEqual(len(idx), g.W // 2)          # from the centre to the edge
        self.assertTrue(all(0 <= i < g.W * g.H for i in idx))
        self.assertEqual(g.line(5000, 5000, 6000, 6000)[0], [])

    def test_wedge_covers_cone(self):
        g = Grid(size_m=4)
        spans = g.wedge_spans(0, 0, 0.0, math.radians(6), 1000)
        cells = set()
        for s, e in spans:
            cells.update(range(s, e))
        self.assertIn(g.index(150, 0), cells)                 # near the apex (the sensor)
        self.assertIn(g.index(950, 0), cells)                 # along the beam
        self.assertIn(g.index(900, 85), cells)                # inside the cone edge (tan 6 deg * 900 = 95)
        self.assertNotIn(g.index(900, 160), cells)            # outside it
        self.assertNotIn(g.index(1100, 0), cells)             # beyond the range
        self.assertNotIn(g.index(-100, 0), cells)             # behind the sensor

    def test_log_odds_update_and_clamp(self):
        g = Grid(size_m=4)
        g.integrate_wedge(0, 0, 0.0, math.radians(6), 900, 1000)
        self.assertLess(g.value(500, 0), UNKNOWN)             # free
        self.assertGreater(g.value(1000, 0), UNKNOWN)         # hit
        self.assertEqual(g.value(-500, 0), UNKNOWN)           # untouched
        for _ in range(200):
            g.integrate_wedge(0, 0, 0.0, math.radians(6), 900, 1000)
        self.assertEqual(g.value(500, 0), g.LO)               # clamped: it can still change its mind
        self.assertEqual(g.value(1000, 0), g.HI)
        # a hit cell is never also freed by its own reading's wedge
        g2 = Grid(size_m=4)
        g2.integrate_wedge(0, 0, 0.0, math.radians(10), 1000, 1000)
        self.assertGreater(g2.value(1000, 0), UNKNOWN)
        # weak ("no target") free is half strength
        g3 = Grid(size_m=4)
        g3.integrate_wedge(0, 0, 0.0, 0.1, 1000, weak=True)
        g4 = Grid(size_m=4)
        g4.integrate_wedge(0, 0, 0.0, 0.1, 1000)
        self.assertGreater(g3.value(500, 0), g4.value(500, 0))
        self.assertLess(g3.value(500, 0), UNKNOWN)

    def test_dirty_tiles_and_png(self):
        g = Grid(size_m=6)
        g.integrate_wedge(100, 100, math.radians(30), math.radians(6), 1500, 1600)
        tiles = g.take_dirty()
        self.assertTrue(tiles)
        self.assertEqual(g.take_dirty(), [])
        self.assertTrue(set(tiles) <= g.touched)
        tx, ty = g.cell_of(500, 330)
        tile = (tx // g.T, ty // g.T)
        self.assertIn(tile, tiles)
        w, h, rows = png_size_and_pixels(g.tile_png(*tile))
        self.assertEqual((w, h), (g.T, g.T))
        # the free cell at (500, 330) is white-ish in its tile; row 0 is the top (high y)
        self.assertGreater(rows[g.T - 1 - ty % g.T][tx % g.T], 140)       # one free reading: p 0.4
        self.assertEqual(GREY[UNKNOWN], 128)
        w, h, rows = png_size_and_pixels(g.png())
        self.assertEqual(w % g.T, 0)
        msg = g.tiles_msg([tile])
        self.assertEqual(base64.b64decode(msg[0][2]), g.tile_png(*tile))
        self.assertEqual(png_size_and_pixels(png_grey(3, 2, [b"\x00\x80\xff", b"\x01\x02\x03"]))[2][1], b"\x01\x02\x03")

    def test_save_load_roundtrip(self):
        g = Grid(size_m=6)
        for k in range(20):
            g.integrate_wedge(0, 0, k * 0.3, math.radians(6), 1200, 1300)
        g2 = Grid.from_json(g.to_json({"corr": [1, 2, 3]}))
        self.assertEqual(bytes(g2.cells), bytes(g.cells))
        self.assertEqual(g2.touched, g.touched | g.dirty)
        self.assertEqual((g2.W, g2.H, g2.x0, g2.res), (g.W, g.H, g.x0, g.res))
        self.assertEqual(g2.meta["corr"], [1, 2, 3])
        with self.assertRaises(ValueError):
            Grid.from_json(json.dumps({"format": "other"}))


class TestMapper(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_filters(self):
        m = self.m
        for _ in range(20):
            m.on_range(reading(r=800))
        self.assertGreater(m.grid.value(26 + 12.9 + 800, 0), UNKNOWN)
        used = m.n["used"]
        m.on_range(reading(angle=90, r=800, pitch=12.0))                 # tilted: dropped
        self.assertEqual(m.n["used"], used)
        self.assertEqual(m.grid.value(39, 800), UNKNOWN)
        m.on_range(reading(angle=90, r=8190, valid=0))                    # no target: ignored by default
        self.assertEqual(m.grid.value(39, 500), UNKNOWN)
        m.apply_settings({"no_target_mm": 1000}, save=False)
        m.on_range(reading(angle=90, r=8190, valid=0))
        self.assertLess(m.grid.value(39, 500), UNKNOWN)                   # weak free out to 1 m
        self.assertEqual(m.grid.value(39, 1300), UNKNOWN)
        m.on_range(reading(angle=180, r=20))                              # too close
        self.assertEqual(m.n["short"], 1)

    def test_messages_and_files(self):
        m = self.m
        for k in range(32):
            m.on_range(reading(angle=-85 + k * 5.3, r=1200))
        full = json.loads(m.full_msg())
        self.assertTrue(full["full"] and full["tiles"] and full["w"] == m.grid.W)
        out = m.poll(time.monotonic())
        self.assertEqual(len(out), 1)
        d = json.loads(out[0])
        self.assertFalse(d["full"])
        self.assertTrue(d["tiles"])
        self.assertEqual(m.poll(time.monotonic()), [])                     # throttled
        self.assertEqual(m.poll(time.monotonic() + 1), [])                 # nothing changed
        r = m.command({"action": "save", "name": "kitchen"})
        self.assertEqual(r["t"], "map_saved")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "kitchen.png")))
        self.assertEqual(m.command({"action": "save", "name": "../x"})["t"], "error")
        before = bytes(m.grid.cells)
        m.command({"action": "reset"})
        self.assertEqual(m.grid.value(1000, 0), UNKNOWN)
        self.assertTrue(json.loads(m.poll(time.monotonic() + 2)[0])["full"])
        m.command({"action": "load", "name": "kitchen"})
        self.assertEqual(bytes(m.grid.cells), before)
        self.assertEqual([x["name"] for x in m.command({"action": "list"})["maps"]], ["kitchen"])
        m.command({"action": "delete", "name": "kitchen"})
        self.assertEqual(m.list(), [])
        self.assertEqual(m.command({"action": "load", "name": "nope"})["t"], "error")
        m.command({"action": "set", "settings": {"pitch_limit_deg": 99, "correct": True, "bogus": 1}})
        self.assertEqual(m.settings["pitch_limit_deg"], 45)
        self.assertTrue(m.settings["correct"])
        self.assertTrue(Mapper(self.tmp).settings["correct"])               # persisted


class TestCostMap(unittest.TestCase):
    def test_route_around_wall(self):
        g = wall_grid()
        cm = CostMap(g)
        self.assertEqual(cm.cost[cm.cell(1000, 0)], BLOCKED)
        self.assertGreaterEqual(cm.cost[cm.cell(1150, 0)], MARGIN)          # inflated
        path = cm.astar((0, 0), (1400, 0))
        self.assertIsNotNone(path)
        self.assertLessEqual(len(path), 5)                                    # simplified
        # every leg keeps clear of the wall: sample the polyline
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            for k in range(21):
                x, y = ax + (bx - ax) * k / 20, ay + (by - ay) * k / 20
                if 900 < x < 1150:
                    self.assertGreater(abs(y), 1000 + 80)
        self.assertTrue(cm.path_clear(path))
        self.assertFalse(cm.path_clear([(0, 0), (1400, 0)]))
        # free space only (explore): the unknown beyond the box is not usable
        self.assertIsNone(cm.astar((0, 0), (2500, 0), unknown_ok=False))

    def test_squeeze_through_narrow_gap(self):
        # a wall across the free box (x = 1000) with a 300 mm gap at y = 0: shut with the 180 mm margin,
        # open for the squeeze plan (radius 80 + 35 at 25 mm cells, #62)
        g = wall_grid(y_range=(-1500, -150))
        for y in range(150, 1500, 10):
            for dx in (0, 25):
                g.cells[g.index(1000 + dx, y)] = g.HI
        self.assertIsNone(CostMap(g).astar((0, 0), (1400, 0), unknown_ok=False))
        s = dict(DEFAULTS, margin_mm=DEFAULTS["squeeze_mm"])
        path = CostMap(g, s, res_mm=g.res, sure_walls=True).astar((0, 0), (1400, 0), unknown_ok=False)
        self.assertIsNotNone(path)
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            if ax < 1000 <= bx or bx < 1000 <= ax:
                y = ay + (by - ay) * (1000 - ax) / (bx - ax)
                self.assertLess(abs(y), 150 - 76)                     # through the gap, body clear of the ends

    def test_thin_wall_end_keeps_margin(self):
        # issue #108: the range sweep draws a thin wall's end short - its last cells fade to unknown (a beam passing
        # just beside the end sweeps its free wedge over them). A wall x 1000..1050 ending at y = 0 whose last 50 mm
        # are unknown: legs passing the end still keep radius + margin from the real end (to a cell's rounding), on
        # the normal map and on the squeeze map; without the wall-end growth they cut ~50 mm closer.
        g = Grid(size_m=8)
        for j in range(g.H):
            for i in range(g.W):
                x, y = g.x0 + (i + 0.5) * g.res, g.y0 + (j + 0.5) * g.res
                if -1500 <= x <= 2500 and -1500 <= y <= 1500:
                    g.cells[j * g.W + i] = g.LO
        for y in range(-1500, 0, 5):
            for dx in (0, 25):
                g.cells[g.index(1000 + dx, y)] = g.HI if y < -50 else UNKNOWN
        g._mark([-1500, 2500], [-1500, 1500])
        g.take_dirty()

        def wall_dist(path):
            best = 1e9
            for (ax, ay), (bx, by) in zip(path, path[1:]):
                n = max(1, int(math.hypot(bx - ax, by - ay) / 5))
                for k in range(n + 1):
                    x, y = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                    best = min(best, math.hypot(max(1000 - x, 0, x - 1050), max(-1500 - y, 0, y)))
            return best
        squeeze = {"settings": dict(DEFAULTS, margin_mm=DEFAULTS["squeeze_mm"]), "res_mm": 25, "specks": True}
        for kw in ({}, squeeze):
            for start, goal in (((500, -700), (1600, -700)), ((500, -300), (1500, -300)), ((700, 400), (1300, -600))):
                worst = {}
                for grow in (0.0, END_GROW_MM):
                    cm = CostMap(g, grow_mm=grow, **kw)
                    path = cm.astar(start, goal, unknown_ok=False)
                    self.assertIsNotNone(path)
                    worst[grow] = wall_dist(path)
                need = cm.hard_mm - cm.res * 0.71
                self.assertGreaterEqual(worst[END_GROW_MM], need, (cm.res, start, goal))
        self.assertLess(worst[0.0], need)                     # (the fade does cut in without it)

    def test_squeeze_centres_waypoints_in_gap(self):
        # issue #108: a 250 mm gap (wall x 1000..1050, jamb cells end at y 475 / start at 725). Its centre (600) lies on
        # a 25 mm cell edge, so the A* cells pass 12.5 mm off it at best (and the flat soft band lets them take any
        # cell of the corridor); the route's waypoints are moved onto the centre line, the robot enters square to it.
        g = wall_grid(y_range=(-1500, 475))
        for y in range(725, 1500, 10):
            for dx in (0, 25):
                g.cells[g.index(1000 + dx, y)] = g.HI
        cm = CostMap(g, dict(DEFAULTS, margin_mm=DEFAULTS["squeeze_mm"]), res_mm=g.res, specks=True,
                     grow_mm=END_GROW_MM)
        cm.centre = True
        for start, goal in (((300, 100), (1700, 1100)), ((300, 1100), (1700, 0)), ((400, 600), (1600, 600))):
            path = cm.astar(start, goal, unknown_ok=False)
            self.assertIsNotNone(path)
            ys = [ay + (by - ay) * (1025 - ax) / (bx - ax) for (ax, ay), (bx, by) in zip(path, path[1:])
                  if (ax - 1025) * (bx - 1025) < 0]
            self.assertEqual(len(ys), 1, path)
            self.assertAlmostEqual(ys[0], 600, delta=6, msg=path)
            self.assertGreaterEqual(cm.wall_clearance(path), 120)    # 137.5 to the jamb cells, less half a cell

    def test_frontiers(self):
        g = wall_grid()
        cm = CostMap(g)
        fr = cm.frontiers((0, 0))
        self.assertTrue(fr)
        d, goal, path, size = fr[0]
        self.assertAlmostEqual(abs(goal[0]) + abs(goal[1]), 1475, delta=200)   # the box edge (wall side blocked)
        self.assertGreaterEqual(size, 4)
        fr2 = cm.frontiers((0, 0), blacklist=[goal], black_r_mm=5000)
        self.assertEqual(fr2, [])


class TestSimMapping(unittest.TestCase):
    """End-to-end without network: the sim robot drives its loop, the mapper builds the map."""

    def wall_fit(self, g, room, occ_byte=150):
        d = []
        for i, v in enumerate(g.cells):
            if v > occ_byte:
                d.append(room.clearance(g.x0 + (i % g.W + 0.5) * g.res, g.y0 + (i // g.W + 0.5) * g.res))
        return d

    def test_tour_map_converges_to_walls(self):
        r = DriftRobot(seed=1)
        m = Mapper(tempfile.mkdtemp())
        for _ in range(2):
            drive_offline(r, m.on_range)
        d = self.wall_fit(m.grid, r.room)
        self.assertGreater(len(d), 300)
        self.assertGreater(sum(1 for v in d if v < 75) / len(d), 0.85)     # occupied cells sit on walls
        # the walls along the route show up: outer wall samples have an occupied cell within 75 mm
        hits = tot = 0
        for x in range(-800, 3900, 100):
            for y in (-1500, 2000):
                if 200 < x < 1200 and y < 0 or 2300 < x < 2350:
                    continue                                                   # behind the couch / partition
                tot += 1
                hits += any(m.grid.value(x + dx, y + dy) > 150 for dx in (-50, 0, 50) for dy in (-75, -50, -25, 0, 25, 50, 75))
        self.assertGreater(hits / tot, 0.45)      # two loops of one route don't face every wall
        # free space inside the room, unknown outside
        self.assertLess(m.grid.value(1000, 300), UNKNOWN)
        self.assertEqual(m.grid.value(-3000, 300), UNKNOWN)

    def test_drift_option(self):
        r = DriftRobot(seed=1, drift_deg_min=6, drift_scale=0.02)
        drive_offline(r, lambda d: None, points=TOUR[:3])
        err_mm, err_deg = r.drift_error()
        self.assertGreater(err_mm, 30)
        self.assertGreater(err_deg, 1)

    def test_explore_maps_room(self):
        r = DriftRobot(seed=2)
        m = Mapper(tempfile.mkdtemp())
        s, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=400)
        self.assertEqual(b.planner.mode, "idle")
        self.assertIn("explore done", b.planner.status)
        _occ, free = m.grid.counts()
        self.assertGreater(free * 0.025 * 0.025, 11.0)          # m^2 seen free (room: ~14.5 m^2 minus boxes)
        d = self.wall_fit(m.grid, r.room)
        self.assertGreater(sum(1 for v in d if v < 75) / len(d), 0.8)

    def test_route_behind_wall(self):
        r = DriftRobot(seed=4)
        m = Mapper(tempfile.mkdtemp())
        drive_offline(r, m.on_range)                              # map the room first
        close = [1e9]

        def watch(robot, b):
            close[0] = min(close[0], robot.room.clearance(robot.X, robot.Y))
        # from near the start to behind the partition wall (x = 2300, y 1100..2000)
        s, b = plan_offline(r, m, {"t": "plan", "action": "goto", "points": [[2900, 1700]]}, max_s=120,
                            on_step=watch)
        self.assertEqual(b.planner.status, "arrived", b.lines)
        x, y, _ = r.odom()
        self.assertLess(math.hypot(x - 2900, y - 1700), 150)
        self.assertGreater(close[0], 100)                          # never scraped a wall

    def test_explore_through_narrow_doorway(self):
        # issue #62: two rooms (x -1000..1000 and 1000..2500) joined by a 280 mm doorway near a corner: the wall
        # margin (180) shuts it, the squeeze plan (radius + 35, 25 mm cells) gets through. Thin-beam robot: with
        # the sensor cone (DriftRobot) the map itself often draws a doorway this narrow shut.
        for width, seed in ((280.0, 5), (260.0, 3)):
            r = SimRobot(seed=seed)
            c = 600.0
            r.room.segs = _box(750, 0, 3500, 2000) + [(1000, -1000, 1000, c - width / 2), (1000, c + width / 2, 1000, 1000)]
            m = Mapper(tempfile.mkdtemp())
            seen = {"x": -1e9, "close": 1e9}

            def watch(robot, b):
                seen["x"] = max(seen["x"], robot.X)
                seen["close"] = min(seen["close"], robot.room.clearance(robot.X, robot.Y))
            s, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=400, on_step=watch)
            self.assertIn("explore done", b.planner.status, b.lines[-5:])
            self.assertTrue(any("narrow gap" in l for l in b.lines), b.lines)
            self.assertGreater(seen["x"], 1000 + ROBOT_RADIUS + 20, width)  # all of it went through
            self.assertGreater(seen["close"], ROBOT_RADIUS, width)     # without touching a jamb
            free = sum(1 for x in range(1200, 2400, 100) for y in range(-900, 1000, 100)
                       if m.grid.value(x, y) < UNKNOWN)
            self.assertGreater(free / (12 * 19), 0.9, width)          # the far room is mapped

    def test_squeeze_stays_centred_in_doorway(self):
        # issue #68: route simplification used to cut the jamb corner at the squeeze limit (sim runs touched a
        # jamb: clearance == body radius). These (width, seed) runs all did; squeeze legs now stay on the centre line.
        for width, seed in ((260, 4), (260, 7), (280, 1), (280, 6), (280, 7), (300, 2), (300, 8)):
            r = SimRobot(seed=seed)
            c = 600.0
            r.room.segs = _box(750, 0, 3500, 2000) + [(1000, -1000, 1000, c - width / 2.0), (1000, c + width / 2.0, 1000, 1000)]
            m = Mapper(tempfile.mkdtemp())
            seen = {"x": -1e9, "close": 1e9}

            def watch(robot, b):
                seen["x"] = max(seen["x"], robot.X)
                seen["close"] = min(seen["close"], robot.room.clearance(robot.X, robot.Y))
            s, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=400, on_step=watch)
            self.assertGreater(seen["x"], 1000 + ROBOT_RADIUS + 20, (width, seed))
            self.assertGreater(seen["close"], ROBOT_RADIUS + 1.0, (width, seed))

    def test_squeeze_doorway_sweep(self):
        # issue #108: doorways 240-320 mm x 10 seeds (50 explores, in worker processes).
        # - Planner: every squeeze route through a gap the map shows on both sides crosses it on the centre line
        #   (within half a 25 mm cell of the middle between the jamb cells; it used to take any cell of the hard
        #   corridor: up to 62 mm off), and the clearance it reports is the route's own (<= what the gap gives).
        # - Robot: touching a jamb is rare. Not never: the range sweep's free wedge (cone_deg) of beams passing just
        #   beside a thin wall end erases the end on the map (strongly free, 60-150 mm of it at times), and no route
        #   can keep off a jamb the map does not have. master (749889b): 7 of these 50 runs touched a jamb, 2 never got
        #   through. (Touching other walls - squeeze legs along a wall keep only radius + 35 - is not counted here.)
        import multiprocessing
        jobs = [(w, s) for w in (240, 260, 280, 300, 320) for s in range(1, 11)]
        with multiprocessing.Pool(max(1, min(8, (os.cpu_count() or 2) - 1))) as pool:
            runs = pool.map(doorway_run, jobs)
        crossings = [(r["width"], r["seed"], c) for r in runs for c in r["cross"]]
        self.assertGreater(len(crossings), 20)
        for w, s, (t, off, clear, half) in crossings:
            self.assertLessEqual(abs(off), 13.0, (w, s, t, off))
            if clear is not None:
                self.assertLessEqual(clear, half + 25.0, (w, s, t, clear, half))
        jamb = [(r["width"], r["seed"]) for r in runs if r["jamb_touch"]]
        stuck = [(r["width"], r["seed"]) for r in runs if not r["through"]]
        self.assertLessEqual(len(jamb), 5, jamb)          # (now 4: 240/9, 240/10, 260/2, 300/6 - each a jamb end 45-150 mm
        self.assertLessEqual(len(stuck), 2, stuck)        #  short or missing on the map when the route was planned; now 0)


class TestPlannerSettings(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def bridge(self):
        return OfflineBridge(SimRobot(seed=1), Mapper(self.tmp))

    def test_defaults_and_persisted(self):
        p = self.bridge().planner
        self.assertEqual((p.settings["margin_mm"], p.settings["soft_mm"]), (180.0, 300.0))
        self.assertIsNone(p.command({"t": "plan", "action": "set", "settings": {"margin_mm": 150, "soft_mm": 9999,
                                                                               "squeeze_mm": "x", "bogus": 1}}))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "planner.json")))
        p2 = self.bridge().planner                                 # bridge restart
        self.assertEqual((p2.settings["margin_mm"], p2.settings["soft_mm"]), (150.0, 1000.0))
        self.assertEqual(p2.settings["squeeze_mm"], DEFAULTS["squeeze_mm"])
        self.assertNotIn("bogus", p2.settings)
        self.assertEqual(Mapper(self.tmp).list(), [])              # not listed as a saved map
        with open(os.path.join(self.tmp, "planner.json"), "w") as f:
            f.write("{broken")
        self.assertEqual(self.bridge().planner.settings, DEFAULTS)

    def test_stuck_boosts_margin_then_decays(self):
        b = self.bridge()
        p = b.planner
        p.on_log("nav: stuck, no progress driving for 2.5 s (waypoint 1/2)")
        self.assertEqual(p.boost, 0.0)                             # not planning: ignored
        p.mode = "explore"
        p.on_log("nav: stuck, no progress driving for 2.5 s (waypoint 1/2)")
        self.assertEqual(p.boost, 100.0)
        self.assertIsNotNone(p.replan_at)
        self.assertIn("stuck", p.status)
        self.assertEqual([k for k, _cm in p._costmaps()], ["boost", "normal", "squeeze"])
        for _ in range(3):
            p.on_log("nav: stuck, no progress turning for 2.5 s (waypoint 1/2)")
        self.assertEqual(p.boost, 200.0)                           # capped
        p._success()
        self.assertEqual(p.boost, 150.0)
        p.on_log("nav: blocked, obstacle at 287 mm (waypoint 2/2)")
        self.assertEqual(p.boost, 150.0)


def corridor(robot, x1, half_w=1500):
    """Replace the sim room by a long corridor: x -1000 .. x1, y +-half_w."""
    from station.sim.sim import _box
    robot.room.segs = _box((x1 - 1000) / 2.0, 0, x1 + 1000, 2 * half_w)


def door_room(robot, gap):
    """Two rooms (x -1000..2000, 2000..5000; y +-1500) with a doorway `gap` mm wide in the wall at x = 2000."""
    from station.sim.sim import _box
    robot.room.segs = _box(2000, 0, 6000, 3000) + [(2000, -1500, 2000, -gap / 2.0), (2000, gap / 2.0, 2000, 1500)]


class TestBigMap(unittest.TestCase):
    """The grid grows with the robot (issue #63) and narrow doorways stay open."""

    def test_grid_grows_every_way(self):
        g = Grid(size_m=4)
        w0 = g.W
        g.integrate_wedge(500, 500, 0.0, math.radians(6), 800, 900)
        v0 = g.value(1000, 500)
        self.assertLess(v0, UNKNOWN)
        g.integrate_wedge(15000, -9000, math.radians(200), math.radians(6), 1500, 1600)   # far right / below
        self.assertGreater(g.W, w0)
        self.assertGreater(g.value(15000 + 1600 * math.cos(math.radians(200)), -9000 + 1600 * math.sin(math.radians(200))), UNKNOWN)
        self.assertEqual(g.value(1000, 500), v0)                          # old cells kept (they moved inside the array)
        g.integrate_wedge(-12000, 7000, 0.0, math.radians(6), 300)        # far left / above
        self.assertEqual(g.value(1000, 500), v0)
        self.assertLess(g.value(-12000 + 150, 7000), UNKNOWN)
        self.assertEqual(g.x0 % (g.T * g.res), 0)                          # tile aligned
        # browsers get absolute tile indices (world mm // 1000), whatever the grid's corner is
        tiles = {(tx, ty) for tx, ty, _ in g.tiles_msg(g.take_dirty())}
        self.assertIn((0, 0), tiles)                                       # (500, 500), dirty before the grid grew
        self.assertIn((-12, 7), tiles)
        self.assertIn((13, -10), tiles)
        full = {(tx, ty) for tx, ty, _ in g.tiles_msg(sorted(g.touched))}
        self.assertEqual(full, tiles)

    def test_memory_cap_warns(self):
        g = Grid(size_m=4, max_cells=200 * 200 * 4)          # 5 m x 5 m
        w0 = g.W
        g.integrate_wedge(0, 0, 0.0, math.radians(6), 800, 900)
        g.integrate_wedge(30000, 0, 0.0, math.radians(6), 800, 900)
        self.assertEqual(g.W, w0)
        self.assertIn("memory limit", g.warning)
        self.assertEqual(g.value(30500, 0), UNKNOWN)
        self.assertGreater(g.value(900, 0), UNKNOWN)         # the map itself keeps working
        m = Mapper(tempfile.mkdtemp(), max_cells=200 * 200 * 4)
        lines = []
        m.say = lines.append
        m.on_range(reading(x=60000, r=800))
        m.poll(time.monotonic())
        self.assertTrue(any("memory limit" in x for x in lines))
        self.assertTrue(m.stats()["warning"])

    def test_save_load_grown_and_old_files(self):
        g = Grid(size_m=4)
        g.integrate_wedge(0, 0, 0.0, math.radians(6), 800, 900)
        g.integrate_wedge(12000, 5000, 1.0, math.radians(6), 1500, 1600)
        text = g.to_json()
        g2 = Grid.from_json(text)
        self.assertEqual(g2.touched, g.touched | g.dirty)
        for x, y in ((500, 0), (900, 0), (12000 + 800 * math.cos(1.0), 5000 + 800 * math.sin(1.0)), (3000, 3000)):
            self.assertEqual(g2.value(x, y), g.value(x, y))
        self.assertEqual(g2.tile_origin()[0] * g2.T * g2.res, g2.x0)
        # a map with a grown but untouched margin is saved without it
        g3 = Grid(size_m=4)
        g3.integrate_wedge(500, 500, 0.0, math.radians(6), 800, 900)
        g3.integrate_wedge(9000, 0, 0.0, math.radians(6), 100, 200)
        g3.reset()
        g3.integrate_wedge(500, 500, 0.0, math.radians(6), 800, 900)
        self.assertEqual(Grid.from_json(g3.to_json()).W, g3.W)
        # an old file (no init_n; a grid whose corner is not on a tile edge) loads and stays put
        d = json.loads(Grid(size_m=4).to_json())
        d.pop("init_n")
        d.update(ntx=2, nty=2, x0_mm=-30 * 25.0, y0_mm=-30 * 25.0)
        cells = bytearray([UNKNOWN]) * (80 * 80)
        cells[(30 + 4) * 80 + 30 + 10] = 200                               # world cell (10, 4)
        d["cells"] = base64.b64encode(zlib.compress(bytes(cells))).decode("ascii")
        d["touched"] = [[0, 0]]
        g4 = Grid.from_json(json.dumps(d))
        self.assertEqual(g4.value(10 * 25 + 5, 4 * 25 + 5), 200)
        self.assertEqual(g4.x0 % (g4.T * g4.res), 0)
        self.assertTrue(g4.touched)
        # and a default-size old file keeps its place
        old = Grid(size_m=20)
        old.integrate_wedge(1000, 1000, 0.0, math.radians(6), 800, 900)
        self.assertEqual(Grid.from_json(old.to_json()).value(1500, 1000), old.value(1500, 1000))

    def test_robot_far_from_the_start_maps_correctly(self):
        r = DriftRobot(seed=3)
        corridor(r, 18000)
        m = Mapper(tempfile.mkdtemp())
        w0 = m.grid.W
        drive_offline(r, m.on_range, points=[(15000, 0)], max_s=200)
        self.assertGreater(r.X, 14000)                                       # it really went 15 m out
        self.assertGreater(m.grid.W, w0)                                     # the 20 m grid had to grow
        walls = [v for i, v in enumerate(m.grid.cells) if v > 150]
        near = sum(1 for i, v in enumerate(m.grid.cells) if v > 150 and r.room.clearance(
            m.grid.x0 + (i % m.grid.W + 0.5) * m.grid.res, m.grid.y0 + (i // m.grid.W + 0.5) * m.grid.res) < 75)
        self.assertGreater(len(walls), 150)
        self.assertGreater(near / len(walls), 0.85)
        seen = sum(1 for x in range(11000, 16000, 250) for y in (-1500, 1500)
                   if any(m.grid.value(x + dx, y + dy) > 150 for dx in (-50, 0, 50) for dy in (-75, -50, -25, 0, 25, 50, 75)))
        self.assertGreater(seen, 6)                                          # of 40 wall samples out there (a straight drive sees the walls at few angles)
        # seen free along the drive out there (#103: one cell, (14000, 0), was luck: the sweep's beams leave a few
        # unknown cells on the track either way)
        track = [m.grid.value(x, 0) for x in range(13000, 15001, 250)]
        self.assertGreater(sum(1 for v in track if v < UNKNOWN), len(track) // 2, track)
        # browsers: full message covers the far tiles with absolute indices, the planner can use the grid
        full = json.loads(m.full_msg())
        self.assertTrue(any(t[0] >= 14 for t in full["tiles"]))
        cm = CostMap(m.grid)
        self.assertIsNotNone(cm.cell(15000, 0))
        path = cm.astar((0, 0), (14000, 0), unknown_ok=False)
        self.assertIsNotNone(path)
        # save / load keeps it
        m.save("far")
        before = bytes(m.grid.cells)
        m2 = Mapper(m.dir)
        m2.load("far")
        self.assertEqual(m2.grid.value(14000, 0), m.grid.value(14000, 0))
        self.assertEqual(m2.grid.value(14000, 1500), m.grid.value(14000, 1500))

    def test_narrow_doorway_stays_open(self):
        from station.bridge.planner import OCC_BYTE
        for gap in (260, 300):
            for seed, pts in ((1, [(1500, 0), (2600, 0)]), (2, [(1400, -900), (1400, 900)]), (3, [(1500, 0), (2600, 0)])):
                r = DriftRobot(seed=seed)
                door_room(r, gap)
                m = Mapper(tempfile.mkdtemp())
                drive_offline(r, m.on_range, points=pts, max_s=120)
                g = m.grid
                slot = [g.value(x, y) for x in (1975, 2000, 2025) for y in range(-50, 51, 25)]
                label = "gap %d seed %d: %s" % (gap, seed, slot)
                self.assertLessEqual(sum(1 for v in slot if v > OCC_BYTE), 3, label)         # open (the frame ends are 140 mm out)
                self.assertLessEqual(g.value(2000, 0), OCC_BYTE, label)

    def test_body_spot_is_free(self):
        # a far cone hit drawn into a doorway goes once the robot drives over it (the readings look sideways)
        from station.bridge.planner import OCC_BYTE
        m = Mapper(tempfile.mkdtemp())
        m.grid.integrate_wedge(0, 0, 0.0, math.radians(6), 1150, 1200, hit_half=math.radians(2.5), rw=True)
        self.assertGreater(m.grid.value(1200, 0), OCC_BYTE)
        for x in range(1000, 1401, 10):
            m.on_range(reading(x=x, heading=0.0, angle=90.0, r=600))
        self.assertLessEqual(m.grid.value(1200, 0), OCC_BYTE)
        self.assertLess(m.grid.value(1200, -40), UNKNOWN)                  # (the beams look the other way)
        self.assertEqual(m.grid.value(1200, -100), UNKNOWN)               # only the body's own spot

    def test_range_weight_marks(self):
        # near hits are strong and narrow, far hits weak; a close free reading clears a far hit
        g = Grid(size_m=8)
        g.integrate_wedge(0, 0, 0.0, math.radians(6), 300, 400, hit_half=math.radians(2.5), rw=True)
        near = g.value(400, 0) - UNKNOWN
        g2 = Grid(size_m=8)
        g2.integrate_wedge(0, 0, 0.0, math.radians(6), 1900, 2000, hit_half=math.radians(2.5), rw=True)
        far = g2.value(2000, 0) - UNKNOWN
        g3 = Grid(size_m=8)
        g3.integrate_wedge(0, 0, 0.0, math.radians(6), 1900, 2000, hit_half=math.radians(2.5))
        plain = g3.value(2000, 0) - UNKNOWN
        self.assertGreater(near, plain)
        self.assertLess(far, plain)
        self.assertGreater(far, 0)
        g2.integrate_wedge(0, 0, 0.0, math.radians(6), 700, 800, hit_half=math.radians(2.5), rw=True)    # close, same line
        self.assertLessEqual(g2.value(1500, 0), UNKNOWN)       # (not cleared here: the wedge ends at 700) ...
        g2.integrate_wedge(300, 0, 0.0, math.radians(6), 1900, 2000, rw=True)
        self.assertLess(g2.value(1000, 0), UNKNOWN)

class TestBridgeMapApi(unittest.TestCase):
    def test_ws(self):
        asyncio.run(self._run())

    async def _run(self):
        from station.bridge.bridge import Bridge
        from station.bridge.httpws import HttpServer, WsClient
        from station.sim.sim import SimServer
        tmp = tempfile.mkdtemp()
        robot = DriftRobot(seed=1)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        phys = asyncio.ensure_future(sim.physics())
        web = os.path.join(os.path.dirname(__file__), "..", "..", "web")
        b = Bridge("tcp://127.0.0.1:%d" % srv.sockets[0].getsockname()[1], web, tmp, "127.0.0.1", 0,
                   release_file=os.path.join(tmp, "release"), quiet=True, map_dir=os.path.join(tmp, "maps"))
        http = HttpServer(web, [("GET", "/api/map", b.api_map), ("GET", "/api/map/", b.api_map)], "/ws", b.ws_handler)
        hs = await http.start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]

        async def wait_for(ws, pred, timeout=8.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                m = json.loads(await asyncio.wait_for(ws.recv(), end - time.monotonic()))
                if pred(m):
                    return m
            raise AssertionError("timeout")
        try:
            ws = await WsClient.connect("127.0.0.1", port)
            full = await wait_for(ws, lambda m: m["t"] == "map")
            self.assertTrue(full["full"])
            self.assertEqual(full["res_mm"], 25.0)
            await wait_for(ws, lambda m: m["t"] == "plan" and m["mode"] == "idle")
            # robot boots lying down: arm it so the sweep sees the room, tiles start coming
            ws.send(json.dumps({"t": "set_mode", "mode": lp.MODE_BALANCE, "arm": True}))
            d = await wait_for(ws, lambda m: m["t"] == "map" and not m["full"] and m["tiles"])
            self.assertTrue(d["tiles"][0][2])
            ws.send(json.dumps({"t": "map", "action": "save", "name": "t1"}))
            r = await wait_for(ws, lambda m: m["t"] == "map_saved")
            self.assertEqual(r["maps"][0]["name"], "t1")
            loop = asyncio.get_running_loop()
            import urllib.request
            png = await loop.run_in_executor(None, lambda: urllib.request.urlopen(
                "http://127.0.0.1:%d/api/map/current.png" % port, timeout=5).read())
            self.assertEqual(png[:4], b"\x89PNG")
            # explore starts (and drives), STOP ends it
            ws.send(json.dumps({"t": "plan", "action": "explore"}))
            await wait_for(ws, lambda m: m["t"] == "plan" and m["mode"] == "explore")
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["mode"] == lp.MODE_EXPLORE)
            ws.send(json.dumps({"t": "estop"}))
            m = await wait_for(ws, lambda m: m["t"] == "plan" and m["mode"] == "idle")
            self.assertIn("E-stop", m["status"])
            ws.send(json.dumps({"t": "map", "action": "reset"}))
            await wait_for(ws, lambda m: m["t"] == "map" and m["full"] and m["v"] == full["v"] + 1)
            ws.close()
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
