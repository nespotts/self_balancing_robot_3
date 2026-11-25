"""Event-loop stalls with the LD19 (issue #102): tiles only when their bytes change, the encoding budget carries the rest
over (every change still arrives), autosave off the loop on a snapshot, the turn in parts, the loop profile."""
import asyncio
import base64
import json
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import lidar as lidar_mod
from station.bridge import loopprof, mapping
from station.bridge.mapping import AUTOSAVE_NAME, Mapper
from station.bridge.occgrid import Grid

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


def ring(cx, cy, r, n=450, a0=0.0, a1=2 * math.pi):
    """One run of points on a circle (a round room seen from inside)."""
    return [[(cx + r * math.cos(a0 + (a1 - a0) * k / n), cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n)]]


def tiles_of(msgs):
    out = {}
    for text in msgs:
        m = json.loads(text)
        if m.get("t") == "map":
            for tx, ty, b64 in m["tiles"]:
                out[(tx, ty)] = b64
    return out


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TestDirtyOnChange(unittest.TestCase):
    def test_only_changed_tiles_are_dirty(self):
        g = Grid(20, 25)
        runs = ring(0, 0, 2500)
        g.integrate_scan(0, 0, runs, margin_mm=25)
        first = set(g.take_dirty())
        # the bounding box of the turn is 6 x 6 tiles; the ring's corners are never touched
        x0, y0 = g.cell_of(-2500, -2500)
        x1, y1 = g.cell_of(2500, 2500)
        box = {(tx, ty) for tx in range(x0 // g.T, x1 // g.T + 1) for ty in range(y0 // g.T, y1 // g.T + 1)}
        self.assertTrue(first)
        self.assertTrue(first < box, "only the tiles the turn changed, not its whole bounding box")
        # the same turn again and again: hits and free space reach their clamps, then nothing changes
        for _ in range(12):
            g.integrate_scan(0, 0, runs, margin_mm=25)
            g.take_dirty()
        before = bytes(g.cells)
        g.integrate_scan(0, 0, runs, margin_mm=25)
        self.assertEqual(bytes(g.cells), before)
        self.assertEqual(g.take_dirty(), [], "a turn that changes no byte dirties no tile")
        self.assertEqual(g.fdirty & set(), set())

    def test_unchanged_dirty_tile_is_not_resent(self):
        m = Mapper(None)
        m.integrate_scan(0, 0, ring(0, 0, 1500), (0, 0))
        sent = tiles_of(m.poll(1.0))
        self.assertTrue(sent)
        m.grid.dirty |= set(m.grid.touched)          # marked (e.g. a clear_disc bounding box) but nothing changed
        out = m.poll(2.0)
        self.assertEqual(tiles_of(out), {})
        self.assertGreaterEqual(m.tile_stats["same"], len(sent))


class TestBudget(unittest.TestCase):
    def setUp(self):
        self.budget = mapping.TILE_BUDGET_S

    def tearDown(self):
        mapping.TILE_BUDGET_S = self.budget

    def test_carry_over_sends_every_changed_tile_nearest_first(self):
        mapping.TILE_BUDGET_S = 0.0                   # one tile per poll
        m = Mapper(None)
        m.body_at = (2600.0, 0.0)                     # the robot stands in absolute tile (2, 0) (1 m tiles)
        m.integrate_scan(0, 0, ring(0, 0, 4000, n=900), (2600.0, 0.0))
        want = {(tx + m.grid.tile_origin()[0], ty + m.grid.tile_origin()[1]) for tx, ty in m.grid.dirty}
        self.assertGreater(len(want), 20)
        got, order = {}, []
        t = 1.0
        for _ in range(400):
            msgs = m.poll(t)
            new = tiles_of(msgs)
            for k in sorted(new):
                order.append(k)
            got.update(new)
            t += mapping.TILE_AGAIN_S
            if not m.tile_queue and t > 3:
                break
        self.assertFalse(m.tile_queue, "the backlog drains")
        self.assertEqual(set(got), want)
        self.assertEqual(order[0], (2, 0), "the robot's own tile first")
        # what the browsers have is the grid now
        ox, oy = m.grid.tile_origin()
        for (tx, ty), b64 in got.items():
            self.assertEqual(base64.b64decode(b64), m.grid.tile_png(tx - ox, ty - oy))

    def test_tile_changed_again_while_queued_or_after_sending(self):
        mapping.TILE_BUDGET_S = 0.0
        m = Mapper(None)
        m.integrate_scan(0, 0, ring(0, 0, 1500), (0, 0))
        got = {}
        t = 1.0
        for _ in range(100):
            got.update(tiles_of(m.poll(t)))
            t += mapping.TILE_AGAIN_S
        m.integrate_scan(0, 0, ring(300, 0, 1500), (300, 0))    # the same tiles change again
        for _ in range(200):
            got.update(tiles_of(m.poll(t)))
            t += mapping.TILE_AGAIN_S
        ox, oy = m.grid.tile_origin()
        for tx, ty in m.grid.touched:
            self.assertEqual(base64.b64decode(got[(tx + ox, ty + oy)]), m.grid.tile_png(tx, ty))

    def test_full_message_uses_current_bytes_and_cache(self):
        m = Mapper(None)
        m.integrate_scan(0, 0, ring(0, 0, 1500), (0, 0))
        m.poll(1.0)
        m.integrate_scan(0, 0, ring(200, 100, 1200), (200, 100))   # changed, not sent yet
        full = json.loads(m.full_msg())
        ox, oy = m.grid.tile_origin()
        for tx, ty, b64 in full["tiles"]:
            self.assertEqual(base64.b64decode(b64), m.grid.tile_png(tx - ox, ty - oy))
        # a new client's full map does not count as sent to everyone: the queued tiles still go out
        self.assertTrue(tiles_of(m.poll(2.0)))

    def test_reset_and_load_drop_the_queue(self):
        mapping.TILE_BUDGET_S = 0.0
        m = Mapper(None)
        m.integrate_scan(0, 0, ring(0, 0, 3000), (0, 0))
        m.poll(1.0)
        self.assertTrue(m.tile_queue)
        m.reset()
        self.assertFalse(m.tile_queue)
        self.assertFalse(m.tile_sent)


class TestAutosaveOffLoop(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_snapshot_saved_while_the_grid_changes(self):
        m = Mapper(self.tmp)
        m.integrate_scan(0, 0, ring(0, 0, 1500), (0, 0))
        m.places.add("Couch", 500.0, 200.0)
        snap = bytes(m.grid.cells)
        prof = loopprof.LoopProf()
        m.prof = prof

        async def run():
            t0 = time.perf_counter()
            self.assertTrue(m.autosave())
            self.assertLess(time.perf_counter() - t0, 0.5)
            self.assertTrue(m.saving, "the files are written in a worker thread")
            self.assertFalse(m.autosave(), "one at a time")
            for k in range(30):                            # the loop keeps mapping meanwhile
                m.integrate_scan(0, 0, ring(400 + 10 * k, 0, 900), (400, 0))
            while m.saving:
                await asyncio.sleep(0.01)
        asyncio.run(run())
        self.assertNotEqual(bytes(m.grid.cells), snap)
        with open(os.path.join(self.tmp, AUTOSAVE_NAME + ".json"), encoding="utf-8") as f:
            saved = Grid.from_json(f.read())
        self.assertEqual(bytes(saved.cells), snap, "the file holds the grid as it was when the save started")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, AUTOSAVE_NAME + ".png")))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, AUTOSAVE_NAME + ".places.json")))
        self.assertIn("map.autosave", prof.thread_s)
        self.assertFalse([n for n in os.listdir(self.tmp) if ".tmp" in n])

    def test_poll_autosaves_without_a_loop_too(self):
        m = Mapper(self.tmp)
        m.integrate_scan(0, 0, ring(0, 0, 1500), (0, 0))
        m.last_autosave = -1e9
        m.poll(100.0)
        self.assertTrue(os.path.exists(os.path.join(self.tmp, AUTOSAVE_NAME + ".json")))
        self.assertFalse(m.saving)


class TestTurnInParts(unittest.TestCase):
    def test_split_runs_shares_one_point(self):
        run = [(float(i), 0.0) for i in range(400)]
        parts = lidar_mod.split_runs([run, run[:50]], 160)
        self.assertTrue(all(sum(len(r) for r in p) <= 161 for p in parts))
        flat = [r for p in parts for r in p]
        self.assertEqual(flat[0][-1], flat[1][0])           # cut with a shared point
        self.assertEqual(sum(len(r) for r in flat), 400 + 2 + 50)

    def test_parts_map_like_one_turn(self):
        runs = ring(0, 0, 2200, n=450)
        a, b = Grid(20, 25), Grid(20, 25)
        for _ in range(3):
            a.integrate_scan(0, 0, runs, margin_mm=25)
            seen = set()
            for part in lidar_mod.split_runs(runs, 160):
                b.integrate_scan(0, 0, part, margin_mm=25, seen=seen)
        diff = sum(1 for x, y in zip(a.cells, b.cells) if x != y)
        self.assertLessEqual(diff, 6, "the parts' free-space fans meet edge to edge; each hit cell counts once")

    def test_deferred_turn_runs_in_separate_callbacks(self):
        from station.bridge.lidar import Lidar
        m = Mapper(None)
        m.settings["lidar_match"] = False
        lid = Lidar(m, None, clock=time.monotonic)
        lid.defer = True
        prof = loopprof.LoopProf()
        lid.prof = m.prof = prof
        calls = []
        orig = m.integrate_scan

        def spy(*a, **kw):
            calls.append(kw.get("last", True))
            return orig(*a, **kw)
        m.integrate_scan = spy

        async def run():
            lid._match_map(0.0, 0.0, 0.0, 0.0, 0.0, ring(0, 0, 2000, n=450), [], 0.0, m.version)
            self.assertEqual(calls, [], "nothing in the calling callback")
            for _ in range(10):
                await asyncio.sleep(0)
        asyncio.run(run())
        self.assertEqual(calls, [False, False, True])
        self.assertEqual(m.n["scans"], 1)


class TestLoopProf(unittest.TestCase):
    def test_exclusive_parts_stall_line_and_status(self):
        clk = FakeClock()
        p = loopprof.LoopProf(clock=clk)
        p.tick(0.0, 0.0)
        with p.part("map.poll"):
            clk.t += 0.010
            with p.part("map.tiles"):
                clk.t += 0.300
            clk.t += 0.005
        with p.part("link:lidar_raw"):
            clk.t += 0.002
            with p.part("lidar.match"):
                clk.t += 0.120
        with p.part("health"):
            clk.t += 0.001
        self.assertAlmostEqual(p.since["map.poll"], 0.015)
        self.assertAlmostEqual(p.since["map.tiles"], 0.300)
        line = p.tick(0.5, 0.42)
        self.assertIn("stalled for 420 ms", line)
        self.assertIn("top: map.tiles 300 ms, lidar.match 120 ms, map.poll 15 ms", line)
        self.assertNotIn("health", line)
        self.assertIsNone(p.tick(0.54, 0.0), "an on-time tick logs nothing")
        st = p.status()
        self.assertEqual(st["stall_max_ms"], 420)
        self.assertEqual(st["stalls"], 1)
        p.tick(11.0, 0.0)                                  # the 10 s window closes
        st = p.status()
        self.assertEqual(st["stall_max_ms"], 420)
        self.assertEqual(st["top"][0][0], "map.tiles")
        self.assertAlmostEqual(st["chunk_max_ms"], 315.0, places=3)   # map.poll with its tiles: one piece of work
        p.tick(22.0, 0.0)
        self.assertEqual(p.status()["stall_max_ms"], 0, "the next window without a stall")

    def test_worker_thread_named_in_the_stall_line(self):
        clk = FakeClock()
        p = loopprof.LoopProf(clock=clk)
        p.tick(0.0, 0.0)
        with p.thread("planner.explore"):
            clk.t += 0.2
        line = p.tick(0.3, 0.26)
        self.assertIn("worker thread running: planner.explore", line)
        self.assertIn("no timed part", line)

    def test_gc_pauses_are_a_part(self):
        import gc
        p = loopprof.LoopProf()
        p.install_gc()
        try:
            with p.part("x"):
                gc.collect()
        finally:
            p.uninstall_gc()
        self.assertIn("gc", p.since)

    def test_health_pi_tile_shows_the_loop(self):
        from station.bridge import health as H
        from station.bridge.tests.test_health import ctx, tile
        ok = tile(H.evaluate(ctx(loop={"stall_max_ms": 30, "busy_pct": 41.0, "top": [["lidar.match", 8.0]]})), "pi")
        self.assertEqual(ok["level"], "ok")
        self.assertIn("max stall 30 ms", ok["why"])
        bad = tile(H.evaluate(ctx(loop={"stall_max_ms": 420, "busy_pct": 70.0, "top": [["map.tiles", 90.0]]})), "pi")
        self.assertEqual(bad["level"], "warn")
        self.assertIn("stalled 420 ms (top: map.tiles 90 ms/s)", bad["why"])

    def test_bridge_status_has_the_loop(self):
        from station.bridge.bridge import Bridge
        tmp = tempfile.mkdtemp()
        try:
            b = Bridge("tcp://127.0.0.1:9", WEB, tmp, quiet=True, map_dir=os.path.join(tmp, "m"),
                       preset_dir=os.path.join(tmp, "p"), falls_dir=os.path.join(tmp, "f"))
            st = b.status()["loop"]
            for k in ("stall_max_ms", "busy_pct", "chunk_max_ms", "top", "threads", "stalls"):
                self.assertIn(k, st)
            self.assertIs(b.mapper.prof, b.prof)
            self.assertIn("loop", b.health_ctx())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
