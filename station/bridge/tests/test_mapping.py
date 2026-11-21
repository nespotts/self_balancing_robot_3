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
from station.bridge.planner import BLOCKED, MARGIN, CostMap
from station.sim.tour import TOUR, DriftRobot, drive_offline, plan_offline


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
