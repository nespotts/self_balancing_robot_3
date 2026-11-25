"""Issue #110: the Pi bridge's per-turn cost and timing.

- RobotClock: telemetry stamped with the robot's clock (least latency anchor, u32 wrap, reboot).
- Real robot rows (2026-11-25, tests/data/tlm_burst_20261125.json): after loop stalls the telemetry arrived in bursts
  (up to ~200 ms late). Stamped with its arrival time the pose history put the robot up to ~90 mm / ~20 deg off at a
  lidar turn's time, enough to fake a wheel slip; stamped with the robot's clock it is exact, and the slip detector
  says nothing while the body follows the wheels.
- The shared deskew (Lidar.deskew) gives the same points as the old per-point code; the map frame is applied when the
  turn is matched (a correction made meanwhile counts).
- worker_yield: a worker naps while the loop is inside a timed part.
- FrameParser: a big backlog with junk parses the same.
Run: python -m unittest discover -s station -t .
"""
import json
import math
import os
import shutil
import tempfile
import threading
import time
import unittest

from station.bridge import ld19, lidar as lidar_mod, link_protocol as lp, loopprof
from station.bridge.lidarmatch import LidarMatch
from station.bridge.mapping import Mapper
from station.bridge.tests.test_ld19 import FakeRobot
from station.sim.lidar_sim import LidarSim

DATA = os.path.join(os.path.dirname(__file__), "data", "tlm_burst_20261125.json")


class TestRobotClock(unittest.TestCase):
    def test_least_latency_wrap_and_reboot(self):
        c = lidar_mod.RobotClock()
        self.assertAlmostEqual(c.map(1_000_000, 50.020), 50.020)          # first frame: its own latency
        self.assertAlmostEqual(c.map(1_010_000, 50.025), 50.025)          # 5 ms less late: the new anchor ...
        self.assertAlmostEqual(c.map(1_020_000, 50.200), 50.035, places=3)   # a late burst: stamped when measured
        self.assertAlmostEqual(c.map(1_030_000, 50.201), 50.045, places=3)
        t = c.map((1 << 32) - 5000, 60.0)                                  # just before the u32 wrap ...
        t2 = c.map(5000, 60.010)                                           # ... and after it: 10 ms later
        self.assertAlmostEqual(t2 - t, 0.010, places=4)
        self.assertAlmostEqual(c.map(1000, 70.0), 70.0)                    # back in time: rebooted, anchored afresh
        self.assertEqual(c.map(None, 71.5), 71.5)                          # no robot time: the arrival time


def load_rows():
    with open(DATA, encoding="utf-8") as f:
        d = json.load(f)
    return [dict(zip(d["cols"], r)) for r in d["rows"]]


def replay(rows, robot_clock):
    """Every 100 ms turn: the bridge's pose (its history interpolated at the turn's time, handled once that moment's
    telemetry has arrived) against the truth (the odometry when the turn was measured). Returns (max mm, max deg,
    slip / suspect verdicts of LidarMatch._observe with the body following the wheels)."""
    off = min(r["t"] - r["t_us"] / 1e6 for r in rows)
    for r in rows:
        r["rt"] = r["t_us"] / 1e6 + off
    tmp = tempfile.mkdtemp()
    try:
        m = Mapper(map_dir=tmp)
        L = lidar_mod.Lidar(m, "teensy", clock=lambda: 0.0)
        lm = LidarMatch(m)
        i = j = k = 0
        worst_mm = worst_deg = 0.0
        verdicts = []
        t_turn = rows[0]["rt"] + 0.5
        while t_turn < rows[-1]["rt"] - 0.2:
            while j + 1 < len(rows) and rows[j]["rt"] < t_turn + 0.05:
                j += 1
            handled = max(t_turn + 0.05, rows[j]["t"])
            while i < len(rows) and rows[i]["t"] <= handled:
                r = rows[i]
                d = {kk: r[kk] for kk in ("x_mm", "y_mm", "heading_deg", "pitch_deg", "yaw_rate_dps", "mode")}
                if robot_clock:
                    d["t_us"] = r["t_us"]
                L.on_tlm(d, r["t"])
                i += 1
            hist = L.hist.items()
            _, ex, ey, eh, _ = lidar_mod.interp(hist, lidar_mod._seek(hist, t_turn), t_turn)
            while k + 1 < len(rows) and rows[k + 1]["rt"] <= t_turn:
                k += 1
            a, b = rows[k], rows[k + 1]
            f = (t_turn - a["rt"]) / (b["rt"] - a["rt"]) if b["rt"] > a["rt"] else 0.0
            tx, ty = a["x_mm"] + f * (b["x_mm"] - a["x_mm"]), a["y_mm"] + f * (b["y_mm"] - a["y_mm"])
            th = a["heading_deg"] + f * ((b["heading_deg"] - a["heading_deg"] + 180) % 360 - 180)
            worst_mm = max(worst_mm, math.hypot(ex - tx, ey - ty))
            worst_deg = max(worst_deg, abs((eh - th + 180) % 360 - 180))
            v = lm._observe(t_turn, (ex, ey), (tx, ty))
            if v in ("slip", "suspect"):
                verdicts.append(v)
            t_turn += 0.1
        return worst_mm, worst_deg, verdicts
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


class TestRealBurstTiming(unittest.TestCase):
    def test_robot_clock_pose_history_on_the_real_rows(self):
        rows = load_rows()
        late = max(r["t"] - r["t_us"] / 1e6 for r in rows) - min(r["t"] - r["t_us"] / 1e6 for r in rows)
        self.assertGreater(late, 0.15)                              # the rows hold the bursts of a stalled loop
        old_mm, old_deg, old_v = replay([dict(r) for r in rows], robot_clock=False)
        new_mm, new_deg, new_v = replay([dict(r) for r in rows], robot_clock=True)
        print("\n[#110 real rows] pose at a turn: arrival stamps max %.0f mm / %.1f deg (%d slip verdicts), "
              "robot clock max %.1f mm / %.2f deg (%d)" % (old_mm, old_deg, len(old_v), new_mm, new_deg, len(new_v)))
        self.assertGreater(old_mm, 25.0)                             # the old stamping: the pose a burst late
        self.assertLess(new_mm, 3.0)
        self.assertLess(new_deg, 0.5)
        self.assertEqual(new_v, [], "no wheel slip while the body follows the wheels")


def old_map_points(L, scan):
    """The pre-#110 per-point mapping code (process_scan, map frame = odometry frame, no ray_step): the reference."""
    S = L.settings
    hist = L.hist.items()
    ref = lidar_mod.pitch_ref(hist, S)
    limit = S["pitch_limit_deg"]
    sgn = 1.0 if S["flip"] else -1.0
    out = []
    idx = 0
    for pk in scan.packets:
        idx, px, py, ph, pp = lidar_mod.interp(hist, idx, pk.t)
        dp = pp - ref
        if abs(dp) > limit:
            continue
        hr = math.radians(ph)
        ox, oy = px + math.cos(hr) * S["x_mm"] - math.sin(hr) * S["y_mm"], py + math.sin(hr) * S["x_mm"] + \
            math.cos(hr) * S["y_mm"]
        dpr = math.radians(dp)
        for a, d, q in pk.pts:
            if d <= 0 or d < S["min_range_mm"] or d > S["max_range_mm"] or q < S["min_intensity"]:
                continue
            b = S["yaw_deg"] + sgn * a
            if L._mask[int((b % 360.0) * 2.0)]:
                continue
            rb = math.radians(b)
            if dpr:
                e = dpr * math.cos(rb)
                ae = abs(e)
                if S["floor_clip"] and ae > 1e-4 and d > 0.9 * S["height_mm"] / math.tan(ae):
                    continue
            out.append((ox + d * math.cos(hr + rb), oy + d * math.sin(hr + rb)))
    return out


class TestSharedDeskew(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def scans(self, n=6, lean=1.0):
        r = FakeRobot()
        r.X, r.Y = 300.0, -200.0
        sim = LidarSim(seed=3, dropout=0.05)
        st, asm = ld19.Ld19Stream(), ld19.ScanAssembler()
        L = lidar_mod.Lidar(Mapper(map_dir=self.tmp), "teensy", clock=lambda: r.t)
        L.apply_settings({"x_mm": 30.0, "y_mm": -10.0, "yaw_deg": -90.0}, save=False)
        out = []
        while len(out) < n:
            r.t += 0.004
            r.TH += 40.0 * 0.004                            # turning 40 deg/s while driving 300 mm/s
            r.X += 1.2 * math.cos(math.radians(r.TH))
            r.Y += 1.2 * math.sin(math.radians(r.TH))
            r.theta = math.radians(lean * math.sin(2 * math.pi * 2.0 * r.t))
            L.on_tlm({"x_mm": r.X, "y_mm": r.Y, "heading_deg": r.TH, "pitch_deg": math.degrees(r.theta),
                      "yaw_rate_dps": 40.0}, r.t)
            for p in st.feed(sim.step(r), r.t):
                sc = asm.add(p)
                if sc:
                    out.append(sc)
        return L, out

    def test_same_points_as_the_per_point_code(self):
        L, scans = self.scans()
        for sc in scans[1:]:
            ref = old_map_points(L, sc)
            turn = L.deskew(sc)
            new = [z for i0, i1 in turn.runs for z in turn.z[i0:i1]]
            self.assertEqual(len(new), len(ref))
            self.assertEqual(turn.map_n, len(ref))
            worst = max(abs(complex(*p) - z) for p, z in zip(ref, new))
            self.assertLess(worst, 2.0, "horizontal range (cos of <= 1 deg) and float order only")

    def test_map_frame_uses_the_correction_at_match_time(self):
        L, scans = self.scans(3)
        turn = L.deskew(scans[-1])
        a = L._map_frame(turn)
        L.mapper.corr = [100.0, -50.0, 2.0]                  # a match of the turn before moved the map frame meanwhile
        b = L._map_frame(turn)
        c, s = math.cos(math.radians(2.0)), math.sin(math.radians(2.0))
        for (x, y), (u, v) in zip(a[6][:50], b[6][:50]):
            self.assertAlmostEqual(u, c * x - s * y + 100.0, places=6)
            self.assertAlmostEqual(v, s * x + c * y - 50.0, places=6)
        self.assertAlmostEqual(b[2], c * a[2] - s * a[3] + 100.0, places=6)   # the robot's map pose too


class TestFieldGrows(unittest.TestCase):
    def test_grown_grid_shifts_the_field(self):
        from station.bridge import lidarmatch as lm
        from station.bridge.occgrid import Grid

        def ring(cx, cy, r, n=400):
            return [[(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]]
        g = Grid(size_m=4.0)
        g.integrate_scan(0.0, 0.0, ring(0, 0, 1500), margin_mm=25.0)
        f = lm.Field()
        f.sync(g)
        W0 = g.W
        g.integrate_scan(2500.0, -1800.0, ring(2500, -1800, 1200), margin_mm=25.0)   # new ground past two edges
        self.assertGreater(g.W, W0)
        f.sync(g)
        self.assertEqual(getattr(f, "shifts", 0), 1)            # moved, not rebuilt
        fresh = lm.Field()
        fresh.sync(g)
        self.assertEqual(bytes(f.f), bytes(fresh.f))
        self.assertEqual(f.walls, fresh.walls)
        g.reset()                                               # a reset is not a growth: rebuilt
        f.sync(g)
        self.assertEqual(f.walls, 0)


class TestMatchInSteps(unittest.TestCase):
    def test_a_match_runs_in_several_loop_callbacks(self):
        import asyncio
        m = Mapper(None)
        m.apply_settings({"match_warmup": 0, "match_every_n": 1}, save=False)

        def room():
            pts = []
            for k in range(450):                         # a 4 x 3 m box round the robot
                a = 2 * math.pi * k / 450
                c, s = math.cos(a), math.sin(a)
                t = min(2000.0 / abs(c) if c else 1e9, 1500.0 / abs(s) if s else 1e9)
                pts.append((t * c, t * s))
            return [pts]
        for _ in range(4):
            m.integrate_scan(0.0, 0.0, room(), (0.0, 0.0))
        lid = lidar_mod.Lidar(m, None, clock=time.monotonic)
        lid.defer = True
        prof = loopprof.LoopProf()
        lid.prof = m.prof = prof
        old = lidar_mod.MATCH_SLICE_S
        lidar_mod.MATCH_SLICE_S = 0.0                   # one step per callback
        steps = []
        orig = lid._match_step

        def spy(*a):
            steps.append(1)
            return orig(*a)
        lid._match_step = spy

        async def run():
            runs = room()
            lid._match_map(0.0, 0.0, 0.0, 0.0, 0.0, runs, runs[0], 0.0, m.version)
            self.assertTrue(lid._matching, "not done in the calling callback")
            for _ in range(60):
                await asyncio.sleep(0)
        try:
            asyncio.run(run())
        finally:
            lidar_mod.MATCH_SLICE_S = old
        self.assertFalse(lid._matching)
        self.assertGreater(len(steps), 5)
        self.assertEqual(m.lmatch.n["matched"] + m.lmatch.n["rejected"], 1)
        self.assertTrue(m.lmatch.last["ok"], m.lmatch.last)


class TestWorkerYield(unittest.TestCase):
    def tearDown(self):
        loopprof.set_active(None)

    def test_worker_naps_while_the_loop_works(self):
        prof = loopprof.LoopProf()
        loopprof.set_active(prof)
        counts = {"busy": 0, "idle": 0}
        phase = {"p": "idle", "stop": False}

        def worker():
            while not phase["stop"]:
                loopprof.worker_yield()
                counts[phase["p"]] += 1
        th = threading.Thread(target=worker, daemon=True)
        th.start()
        time.sleep(0.1)
        with prof.part("lidar.scan"):                       # the loop has work for 100 ms
            phase["p"] = "busy"
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 0.1:
                pass
            phase["p"] = "idle"
        time.sleep(0.1)
        phase["stop"] = True
        th.join(2.0)
        self.assertGreater(counts["idle"], 50 * max(1, counts["busy"]))
        loopprof.worker_yield()                             # on the loop's own thread: a no-op


class TestFrameParserBacklog(unittest.TestCase):
    def test_backlog_with_junk(self):
        frames = [lp.encode_lidar_raw(i & 0xFF, bytes([i & 0xFF]) * 170) for i in range(300)]
        data = b"\x00\xA5junk" + b"".join(f if i % 50 else b"\xA5\x5A\xFF" + f for i, f in enumerate(frames)) + b"\xA5"
        p = lp.FrameParser()
        got = p.feed(data)
        self.assertEqual(len(got), 300)
        self.assertEqual(got[123][1][0], 123)
        self.assertEqual(bytes(p.buf), b"\xA5")              # a trailing sync half is kept
        self.assertEqual(p.feed(b"\x5A"), [])               # ... and waits for the rest of its frame
        p2 = lp.FrameParser()
        out = []
        for k in range(0, len(data), 7):                     # the same bytes in small reads
            out += p2.feed(data[k:k + 7])
        self.assertEqual([t for t, _ in out], [t for t, _ in got])
        self.assertEqual(p.crc_errors, p2.crc_errors)


if __name__ == "__main__":
    unittest.main()
