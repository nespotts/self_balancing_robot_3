"""Health page (#51): rules, battery minutes model, Pi stats, and the bridge <-> sim end to end.
Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest
import urllib.request

from station.bridge import health as H
from station.bridge import link_protocol as lp
from station.bridge.tests.test_bridge_sim import wait_for
from station.bridge.tests.test_hardening import Rig


def tlm(**kw):
    d = {"t_us": 0, "mode": 0, "status": 0, "error_code": 0, "radio_ok": 0, "battery_v": 12.4, "loop_hz": 400.0,
         "motor_left": 0.0, "motor_right": 0.0, "remote_flags": 0}
    d.update(kw)
    return d


def ctx(**kw):
    c = {"link": "connected", "robot_alive": True, "tlm": tlm(), "tlm_hz": 100.0, "rng_hz": [12.5, 12.5], "crc_errors": 0,
         "params": {"tlm.hz": 100.0, "ctl.rate_hz": 400.0, "rng.enable": 1.0, "batt.low_cell": 3.4},
         "hello": {"firmware": "sim v4", "link_version": lp.LINK_PROTOCOL_VERSION}, "estop": False, "robot_estop": False,
         "remote_seen": False, "clients": 1, "log_error": "",
         "pi": {"cpu_pct": 20.0, "temp_c": 45.0, "mem_pct": 30.0, "disk_free_gb": 40.0, "log_mb": 120},
         "battery": {"state": "ok", "minutes": 18, "pct": 62}, "failsafes": 0, "radio": None}
    c.update(kw)
    return c


def tile(out, id):
    return next(t for t in out["tiles"] if t["id"] == id)


def item(out, id):
    return next((i for i in out["preflight"]["items"] if i["id"] == id), None)


class TestRules(unittest.TestCase):
    def test_healthy_robot_is_green(self):
        out = H.evaluate(ctx())
        self.assertEqual(out["preflight"]["level"], "green", out["preflight"])
        self.assertEqual(out["preflight"]["text"], "Ready to arm")
        self.assertEqual([t["id"] for t in out["tiles"] if t["level"] != "ok"], ["radio"])     # no remote: grey, not a problem
        self.assertEqual(tile(out, "batt")["value"], "12.4 V  62%  ~18 min")

    def test_imu_and_encoder_faults_are_red(self):
        for code, tid in ((1, "imu"), (2, "imu"), (3, "encl"), (4, "encr")):
            out = H.evaluate(ctx(tlm=tlm(error_code=code)))
            self.assertEqual(tile(out, tid)["level"], "bad")
            self.assertEqual(out["preflight"]["level"], "red")
        self.assertEqual(tile(H.evaluate(ctx(tlm=tlm(error_code=3))), "encr")["level"], "ok")

    def test_loop_rate(self):
        for hz, lvl in ((400, "ok"), (370, "ok"), (300, "warn"), (150, "bad")):
            out = H.evaluate(ctx(tlm=tlm(loop_hz=hz)))
            self.assertEqual(tile(out, "loop")["level"], lvl, hz)
            self.assertEqual(item(out, "loop")["level"], lvl)
        # the target follows ctl.rate_hz
        c = ctx(tlm=tlm(loop_hz=200.0))
        c["params"]["ctl.rate_hz"] = 200.0
        self.assertEqual(tile(H.evaluate(c), "loop")["level"], "ok")

    def test_range_sensors_only_checked_when_enabled(self):
        c = ctx(rng_hz=[12.0, 0.0])
        out = H.evaluate(c)
        self.assertEqual((tile(out, "rngf")["level"], tile(out, "rngr")["level"]), ("ok", "bad"))
        self.assertEqual(item(out, "rng")["level"], "bad")
        c["params"]["rng.enable"] = 0.0
        out = H.evaluate(c)
        self.assertEqual(tile(out, "rngr")["level"], "na")
        self.assertIsNone(item(out, "rng"))
        self.assertEqual(out["preflight"]["level"], "green")

    def test_estop_latch_blocks_preflight(self):
        for k in ("estop", "robot_estop"):
            out = H.evaluate(ctx(**{k: True}))
            self.assertEqual(item(out, "estop")["level"], "bad")
            self.assertEqual(out["preflight"]["level"], "red")

    def test_control_remote_or_station(self):
        out = H.evaluate(ctx(clients=0))
        self.assertEqual(item(out, "ctl")["level"], "warn")
        self.assertEqual(out["preflight"]["level"], "amber")
        out = H.evaluate(ctx(remote_seen=True, tlm=tlm(radio_ok=1)))
        self.assertEqual((item(out, "ctl")["level"], out["preflight"]["level"]), ("ok", "green"))
        out = H.evaluate(ctx(remote_seen=True, tlm=tlm(radio_ok=0), failsafes=2))
        self.assertEqual((tile(out, "radio")["level"], item(out, "ctl")["level"]), ("bad", "bad"))
        self.assertIn("2", tile(out, "radio")["why"])

    def test_radio_quality_and_gaps(self):
        r = H.parse_radio("radio: 49.0 pkt/s, gaps>100ms 0 (max 32 ms), bad 0, multi 0, ack fail 0, poll max 61 us")
        out = H.evaluate(ctx(remote_seen=True, tlm=tlm(radio_ok=1), radio=r))
        self.assertEqual((tile(out, "radio")["level"], tile(out, "radio")["value"]), ("ok", "98%"))
        r["gaps"], r["max_gap_ms"] = 3, 180
        out = H.evaluate(ctx(remote_seen=True, tlm=tlm(radio_ok=1), radio=r))
        self.assertEqual(tile(out, "radio")["level"], "warn")
        self.assertIn("gaps", tile(out, "radio")["why"])
        self.assertEqual(tile(H.evaluate(ctx()), "radio")["level"], "na")      # no remote: not a problem
        self.assertIsNone(H.parse_radio("IMU ok"))

    def test_battery_levels(self):
        low = H.evaluate(ctx(tlm=tlm(status=lp.TLM_LOW_BATT, battery_v=10.0)))
        self.assertEqual((tile(low, "batt")["level"], low["preflight"]["level"]), ("bad", "red"))
        near = H.evaluate(ctx(tlm=tlm(battery_v=10.5)))            # 3.5 V/cell, limit 3.4
        self.assertEqual(tile(near, "batt")["level"], "warn")
        short = H.evaluate(ctx(battery={"state": "ok", "minutes": 3, "pct": 20}))
        self.assertEqual(tile(short, "batt")["level"], "warn")
        bench = H.evaluate(ctx(tlm=tlm(battery_v=0.0), battery={"state": "bench"}))
        self.assertEqual((tile(bench, "batt")["value"], tile(bench, "batt")["level"]), ("bench supply", "ok"))
        self.assertEqual(bench["preflight"]["level"], "green")

    def test_no_robot(self):
        for link, why in (("waiting", "no robot"), ("released", "released")):
            out = H.evaluate(ctx(link=link, tlm=None, robot_alive=False))
            self.assertIn(why, tile(out, "link")["value"])
            self.assertEqual(tile(out, "imu")["level"], "na")
            self.assertEqual(out["preflight"]["level"], "red")
            self.assertEqual(item(out, "link")["level"], "bad" if link == "waiting" else "warn")
        out = H.evaluate(ctx(robot_alive=False))
        self.assertEqual(tile(out, "link")["value"], "silent")

    def test_pi_and_disk(self):
        out = H.evaluate(ctx(pi={"cpu_pct": 30.0, "temp_c": 75.0, "disk_free_gb": 40.0, "log_mb": 5}))
        self.assertEqual(tile(out, "pi")["level"], "warn")
        out = H.evaluate(ctx(pi={"cpu_pct": 30.0, "temp_c": 85.0, "disk_free_gb": 0.1, "log_mb": 5}))
        self.assertEqual((tile(out, "pi")["level"], tile(out, "disk")["level"]), ("bad", "bad"))
        out = H.evaluate(ctx(pi={}))
        self.assertEqual(tile(out, "pi")["level"], "na")
        self.assertEqual(out["preflight"]["level"], "green")      # a PC without /proc is not a robot fault
        out = H.evaluate(ctx(log_error="disk full"))
        self.assertEqual(tile(out, "disk")["level"], "bad")

    def test_firmware_link_version(self):
        out = H.evaluate(ctx(hello={"firmware": "old", "link_version": 2}))
        self.assertEqual(tile(out, "fw")["level"], "warn")


class TestBatteryModel(unittest.TestCase):
    def run_model(self, seconds, slope, sag=0.0, v0=12.4, load_fn=lambda t: 0.0):
        m = H.BatteryModel()
        for t in range(int(seconds)):
            ld = load_fn(t)
            m.add(float(t), v0 + slope * t + sag * ld, ld)
        return m

    def test_minutes_from_trend(self):
        # 12.4 V falling 0.12 V/min; limit 3 x 3.4 = 10.2 V: 18.3 min from the start
        m = self.run_model(300, -0.002)
        e = m.estimate(299.0)
        self.assertEqual(e["state"], "ok")
        self.assertAlmostEqual(e["minutes"], (12.4 - 0.002 * 299 - 10.2) / 0.002 / 60, delta=1.5)

    def test_load_sag_is_not_a_trend(self):
        # the motors pulse on and off (sag 0.8 V at full duty): minutes must stay near the true trend
        load = lambda t: 0.6 if (t // 20) % 2 else 0.0
        m = self.run_model(400, -0.002, sag=-0.8, load_fn=load)
        e = m.estimate(399.0)
        want = (12.4 - 0.002 * 399 - 0.8 * 0.3 - 10.2) / 0.002 / 60     # at the mean load 0.3
        self.assertEqual(e["state"], "ok")
        self.assertAlmostEqual(e["minutes"], want, delta=3)
        self.assertGreater(e["pct"], 0)

    def test_learning_then_steady(self):
        m = self.run_model(60, -0.002)
        self.assertEqual(m.estimate(59.0)["state"], "learning")
        self.assertIsNotNone(m.estimate(59.0)["pct"])
        m = self.run_model(300, 0.0)
        e = m.estimate(299.0)
        self.assertEqual((e["state"], e["minutes"]), ("steady", None))

    def test_below_limit_is_empty(self):
        m = self.run_model(300, -0.002, v0=10.5)
        self.assertEqual(m.estimate(299.0)["minutes"], 0)

    def test_bench_supply(self):
        m = H.BatteryModel()
        for t in range(300):
            m.add(float(t), 0.0, 0.0)
        e = m.estimate(299.0)
        self.assertEqual((e["state"], e["minutes"], e["v"]), ("bench", None, None))

    def test_gap_resets(self):
        m = self.run_model(200, -0.002)
        m.add(400.0, 12.4, 0.0)            # away for 200 s: rest voltage recovery would fake a trend
        self.assertEqual(len(m.bins), 0)
        self.assertEqual(m.estimate(400.0)["state"], "learning")


class TestPiStats(unittest.TestCase):
    def test_reads_proc_and_sys(self):
        tmp = tempfile.mkdtemp()
        try:
            proc, th = os.path.join(tmp, "proc"), os.path.join(tmp, "thermal")
            os.makedirs(proc)
            os.makedirs(os.path.join(th, "thermal_zone0"))
            os.makedirs(os.path.join(th, "thermal_zone1"))
            with open(os.path.join(th, "thermal_zone0", "temp"), "w") as f:
                f.write("51578\n")
            with open(os.path.join(th, "thermal_zone1", "temp"), "w") as f:
                f.write("48000\n")
            with open(os.path.join(proc, "meminfo"), "w") as f:
                f.write("MemTotal:  1000000 kB\nMemFree: 1 kB\nMemAvailable:   250000 kB\n")

            def stat(user, idle):
                with open(os.path.join(proc, "stat"), "w") as f:
                    f.write("cpu  %d 0 0 %d 0 0 0 0 0 0\ncpu0 1 2 3\n" % (user, idle))
            stat(1000, 9000)
            ps = H.PiStats(tmp, proc, th)
            first = ps.read()
            self.assertIsNone(first["cpu_pct"])              # needs two samples
            stat(1250, 9750)                                 # 250 busy of 1000 ticks
            r = ps.read()
            self.assertEqual(r["cpu_pct"], 25.0)
            self.assertEqual(r["temp_c"], 51.6)              # the hottest zone
            self.assertEqual(r["mem_pct"], 75.0)
            self.assertGreater(r["disk_free_gb"], 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_missing_proc_is_none_not_a_crash(self):
        ps = H.PiStats(".", "/nonexistent/proc", "/nonexistent/thermal")
        r = ps.read()
        self.assertEqual((r["cpu_pct"], r["temp_c"], r["mem_pct"]), (None, None, None))


class TestHealthObject(unittest.TestCase):
    def test_tick_counts_failsafes_and_uses_radio_line(self):
        tmp = tempfile.mkdtemp()
        try:
            h = H.Health(tmp, "/nonexistent/proc", "/nonexistent/thermal")
            c = ctx(remote_seen=True)
            h.on_tlm(tlm(radio_ok=1), 1.0)
            h.on_tlm(tlm(radio_ok=0), 1.1)
            h.on_log("radio: 50.0 pkt/s, gaps>100ms 0 (max 20 ms), bad 0, multi 0, ack fail 0, poll max 60 us", 1.2)
            c["tlm"] = tlm(radio_ok=1)
            out = h.tick(2.0, c)
            self.assertEqual(out["t"], "health")
            self.assertEqual(out["failsafes"], 1)
            self.assertEqual(tile(out, "radio")["value"], "100%")
            self.assertIn("1 failsafe", tile(out, "radio")["why"])
            out = h.tick(2.0 + H.RADIO_FRESH_S + 1, c)          # the radio line is old now: no percentage
            self.assertEqual(tile(out, "radio")["value"], "ok")
            json.dumps(out)                                     # serialisable
            h.on_lost()
            h.on_tlm(tlm(radio_ok=0), 50.0)
            self.assertEqual(h.failsafes, 1)                    # a reconnect is not a failsafe
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_tick_is_cheap(self):
        h = H.Health(".", "/nonexistent/proc", "/nonexistent/thermal")
        c = ctx()
        t0 = time.perf_counter()
        for i in range(200):
            h.on_tlm(tlm(), float(i))
            h.tick(float(i), c)
        self.assertLess((time.perf_counter() - t0) / 200, 0.005)    # well under 5 ms a second


class TestHealthWithSim(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_end_to_end(self):
        asyncio.run(self._run())

    async def _run(self):
        rig = await Rig(self.tmp).start()
        try:
            b, robot = rig.b, rig.robot
            b.health.battery.BIN_S, b.health.battery.MIN_SPAN_S = 2.0, 20.0     # a short "learning" time for the test
            robot.drain = 0.003                                                  # V/s: ~11 min to the limit
            ws = await rig.ws()
            h = await wait_for(ws, lambda m: m["t"] == "health" and m["tiles"] and m["preflight"]["level"] != "red", 8)     # range rates need a first second
            ids = [t["id"] for t in h["tiles"]]
            self.assertEqual(ids, ["link", "loop", "imu", "encl", "encr", "rngf", "rngr", "radio", "pi", "disk", "batt", "fw", "estop"])
            self.assertIn(h["preflight"]["level"], ("green", "amber"), h["preflight"])
            self.assertEqual(tile(h, "estop")["level"], "ok")
            self.assertEqual(tile(h, "radio")["level"], "na")                    # sim without a remote
            self.assertEqual(tile(h, "imu")["level"], "ok")
            # a late-joining client gets the last message at once, and the Kindle poll has it too
            ws2 = await rig.ws()
            await wait_for(ws2, lambda m: m["t"] == "health", 2)
            st = json.loads(await asyncio.get_running_loop().run_in_executor(None, lambda: urllib.request.urlopen(
                "http://127.0.0.1:%d/api/state" % rig.port, timeout=5).read()))
            self.assertEqual(st["health"]["t"], "health")
            api = json.loads(await asyncio.get_running_loop().run_in_executor(None, lambda: urllib.request.urlopen(
                "http://127.0.0.1:%d/api/health" % rig.port, timeout=5).read()))
            self.assertEqual(len(api["tiles"]), 13)
            # the battery model learns the sim's discharge and gives minutes
            h = await wait_for(ws, lambda m: m["t"] == "health" and m["battery"]["minutes"] is not None, 40)
            self.assertEqual(h["battery"]["state"], "ok")
            self.assertTrue(5 < h["battery"]["minutes"] < 14, h["battery"])
            self.assertIn("min", tile(h, "batt")["value"])
            # faults flow through
            robot.fault = 2
            h = await wait_for(ws, lambda m: m["t"] == "health" and tile(m, "imu")["level"] == "bad", 4)
            self.assertEqual(h["preflight"]["level"], "red")
            robot.fault = 0
            # E-stop
            ws.send(json.dumps({"t": "estop"}))
            h = await wait_for(ws, lambda m: m["t"] == "health" and tile(m, "estop")["level"] == "bad", 4)
            self.assertEqual(item(h, "estop")["level"], "bad")
            # it never commanded anything: still disarmed, IDLE
            self.assertFalse(robot.armed)
            self.assertEqual(robot.mode, lp.MODE_IDLE)
        finally:
            await rig.stop()

    def test_bench_and_radio_in_sim(self):
        asyncio.run(self._bench())

    async def _bench(self):
        rig = await Rig(self.tmp).start()
        try:
            rig.robot.bench = True
            rig.robot.remote = True
            rig.robot.radio_log_at = 0.0                                          # print the radio line now
            ws = await rig.ws()
            h = await wait_for(ws, lambda m: m["t"] == "health" and tile(m, "batt")["value"] == "bench supply"
                               and tile(m, "radio")["value"].endswith("%") and m["preflight"]["level"] != "red", 10)
            self.assertEqual(tile(h, "batt")["level"], "ok")
            self.assertIsNone(h["battery"]["minutes"])
            self.assertEqual(tile(h, "radio")["value"], "100%")
            self.assertIn(h["preflight"]["level"], ("green", "amber"))
        finally:
            await rig.stop()

    def test_robot_link_down(self):
        asyncio.run(self._nolink())

    async def _nolink(self):
        rig = await Rig(self.tmp, link=False).start()
        try:
            ws = await rig.ws()
            h = await wait_for(ws, lambda m: m["t"] == "health" and m["tiles"], 5)
            self.assertEqual(tile(h, "link")["level"], "bad")
            self.assertEqual(tile(h, "imu")["level"], "na")
            self.assertEqual(h["preflight"]["level"], "red")
            self.assertEqual(item(h, "link")["level"], "bad")
        finally:
            await rig.stop()


if __name__ == "__main__":
    unittest.main()