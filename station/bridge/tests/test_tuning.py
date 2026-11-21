"""Tune page tools (issue #16): scoring, presets, motor fit (real 2026-11-21 test data), sim motor test,
and the bridge commands end to end. Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import math
import os
import random
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge import motor_fit as mf
from station.bridge import presets, scoring
from station.bridge.session_log import SessionLog

DATA = os.path.join(os.path.dirname(__file__), "data")
REAL_MT = os.path.join(DATA, "motor_test_20261121.jsonl")   # wheels-up test 4 on the real robot (Pi log lines)


def synth_rows(seconds=20.0, hz=400, osc_hz=1.8, amp=1.5, push_at=None, push_deg=6.0, seed=1):
    """Balancing telemetry: pitch error = amp * sin(2 pi osc_hz t) (+ an optional push that decays with a
    small overshoot), around a balance target of -3.15 deg."""
    rnd = random.Random(seed)
    rows = []
    for i in range(int(seconds * hz)):
        t = i / hz
        e = amp * math.sin(2 * math.pi * osc_hz * t)
        if push_at is not None and t >= push_at:
            s = t - push_at
            e += push_deg * math.exp(-s / 0.25) * math.cos(2 * math.pi * 2.0 * s)
        rows.append((t, {"t_us": int(t * 1e6), "mode": 1, "status": 3, "pitch_deg": -3.15 + e,
                         "balance_target_deg": -3.15, "pitch_rate_dps": 2 * math.pi * osc_hz * amp * math.cos(2 * math.pi * osc_hz * t),
                         "motor_left": 0.1 * math.sin(2 * math.pi * osc_hz * t), "motor_right": 0.1 * math.sin(2 * math.pi * osc_hz * t),
                         "bal_p": 250 * e, "bal_i": 50.0, "bal_d": 5 * rnd.gauss(0, 1), "loop_hz": 400.0,
                         "wheel_left_mms": 10.0, "wheel_right_mms": 10.0}))
    return rows


class TestScoring(unittest.TestCase):
    def test_oscillation(self):
        s = scoring.score(synth_rows(osc_hz=1.8, amp=1.5), 20)
        self.assertTrue(s["ok"])
        self.assertAlmostEqual(s["osc_hz"], 1.8, delta=0.06)
        self.assertAlmostEqual(s["err_rms"], 1.5 / math.sqrt(2), delta=0.02)
        self.assertAlmostEqual(s["motor_rms"], 0.1 / math.sqrt(2), delta=0.002)
        self.assertEqual(s["sat_pct"], 0)
        self.assertEqual(s["push_n"], 0)
        self.assertAlmostEqual(s["tlm_hz"], 400, delta=1)
        self.assertEqual(scoring.format_score(s, 20)[-1], "no pushes (>3 deg) in window")

    def test_push_settle(self):
        s = scoring.score(synth_rows(osc_hz=1.0, amp=0.2, push_at=5.0, push_deg=6.0), 20)
        self.assertEqual(s["push_n"], 1)
        p = s["pushes"][0]
        self.assertGreater(p["peak"], 5.0)
        self.assertIsNotNone(p["settle"])
        self.assertLess(p["settle"], 0.5)
        self.assertGreater(p["over"], 0.0)          # the decaying cosine swings to the other side
        self.assertEqual(s["settle_mean"], p["settle"])
        self.assertIn("push +", "\n".join(scoring.format_score(s)))

    def test_not_enough(self):
        rows = synth_rows(seconds=0.3)
        s = scoring.score(rows)
        self.assertFalse(s["ok"])
        for _t, r in rows:
            r["status"] = 1                           # armed but not upright
        self.assertEqual(scoring.score(rows)["up"], 0)

    def test_read_window_from_session_log(self):
        tmp = tempfile.mkdtemp()
        try:
            log = SessionLog(tmp)
            log.start()
            log.t0 = time.monotonic()
            base = {k: 0.0 for k in lp.TELEMETRY_FIELDS}
            fake_t = [0.0]
            log._t = lambda: round(fake_t[0], 3)
            for t, r in synth_rows(seconds=30, hz=500, osc_hz=2.0):   # 2 ms steps: exact in the log's 3 decimals
                fake_t[0] = t
                d = dict(base, **r)
                log.tlm(d)
                if abs(t - 10.0) < 1e-9:
                    log.record("param", 2, "bal.kd", 7.15, 0, 50)
            log.flush()
            path = os.path.join(tmp, log.name)
            with open(path, "a") as f:
                f.write('["tlm",31.5,1,2')        # half-written last line (live file)
            rows, other = scoring.read_window(path, last_s=20)
            self.assertAlmostEqual(rows[0][0], 9.998, delta=1e-6)
            self.assertAlmostEqual(rows[-1][0], 29.998, delta=1e-6)
            self.assertEqual(len(rows), 10001)
            self.assertEqual([o[3] for o in other], ["bal.kd"])
            rows2, _ = scoring.read_window(path, t0=5.0, t1=7.5)
            self.assertEqual(len(rows2), 1251)
            s = scoring.score(rows2, 2.5)
            self.assertAlmostEqual(s["osc_hz"], 2.0, delta=0.15)
            log.stop()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


PARAMS = [{"id": i, "name": n, "value": v, "min": lo, "max": hi} for i, (n, v, lo, hi) in enumerate([
    ("bal.kp", 250.0, 0, 2000), ("bal.ki", 1500.0, 0, 20000), ("bal.kd", 7.15, 0, 50), ("bal.d_src", 0.0, 0, 1),
    ("vel.kp", 0.006, 0, 1), ("mot.left_scale", 0.95, 0.5, 1.5), ("mot.test", 0.0, 0, 4), ("mot.drive_mode", 0.0, 0, 0),
    ("cfg.save", 0.0, 0, 1), ("tlm.hz", 400.0, 1, 1000), ("mot.model", 0.0, 0, 1)])]


class TestPresets(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.store = presets.PresetStore(os.path.join(self.tmp, "p"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_builtins(self):
        names = [p["name"] for p in self.store.list()]
        self.assertEqual(names[:2], ["v3 known good", "2026-11-21 (kd 7.15, ki 1500)"])
        v3 = self.store.get("v3 known good")["values"]
        self.assertEqual((v3["bal.kp"], v3["bal.ki"], v3["bal.kd"], v3["bal.d_src"]), (250.0, 2010.0, 4.63, 0.0))
        today = self.store.get("2026-11-21 (kd 7.15, ki 1500)")["values"]
        self.assertEqual((today["bal.ki"], today["bal.kd"]), (1500.0, 7.15))
        for p in presets.BUILTIN:
            self.assertFalse(set(p["values"]) & presets.NEVER)
        with self.assertRaises(ValueError):
            self.store.delete("v3 known good")
        with self.assertRaises(ValueError):
            self.store.save("v3 known good", {"bal.kp": 1})

    def test_save_list_delete(self):
        snap = presets.snapshot(PARAMS)
        self.assertNotIn("mot.test", snap)
        self.assertNotIn("mot.drive_mode", snap)
        self.assertNotIn("cfg.save", snap)
        self.assertNotIn("tlm.hz", snap)
        self.assertEqual(snap["bal.kd"], 7.15)
        self.store.save("soft / carpet", snap, "note")
        p = self.store.get("soft / carpet")
        self.assertEqual(p["values"], snap)
        self.assertFalse(p["builtin"])
        self.assertTrue(os.path.isfile(os.path.join(self.tmp, "p", "soft_carpet.json")))
        self.store.delete("soft / carpet")
        self.assertIsNone(self.store.get("soft / carpet"))
        with self.assertRaises(ValueError):
            self.store.save("  ", snap)

    def test_plan_and_diff(self):
        plan = presets.plan_apply({"bal.ki": 2010.0, "bal.kd": 4.63, "bal.kp": 250.0, "mot.test": 4, "mot.drive_mode": 1,
                                   "cfg.save": 1, "bal.nope": 3, "mot.left_scale": 9.0}, PARAMS)
        self.assertEqual([(c["name"], c["to"]) for c in plan["changes"]],
                         [("bal.ki", 2010.0), ("bal.kd", 4.63), ("mot.left_scale", 1.5)])   # clamped to max
        self.assertEqual(plan["same"], 1)
        self.assertEqual(plan["skipped"], ["bal.nope", "cfg.save", "mot.drive_mode", "mot.test"])
        rows = presets.diff({"bal.kp": 250.0, "bal.kd": 4.63}, {"bal.kd": 7.15, "bal.kp": 250.0, "vel.kp": 0.006},
                            order=["bal.kp", "bal.kd"])
        self.assertEqual([(r["name"], r["same"]) for r in rows], [("bal.kp", True), ("bal.kd", False), ("vel.kp", False)])
        self.assertIsNone(rows[2]["a"])


class TestMotorFit(unittest.TestCase):
    def test_real_run(self):
        out = mf.fit_items(mf.read_file(REAL_MT))
        self.assertEqual(len(out["runs"]), 1)
        self.assertEqual(out["runs"][0]["status"], "ok")
        self.assertEqual(out["runs"][0]["n"], {"a": 8, "b": 64, "c": 12})
        R = out["results"][0]
        self.assertEqual(R["cfg_name"], "0:36621")
        a = {(r["wheel"], r["dir"]): r for r in R["a"]["rows"]}
        self.assertEqual(a[("left", "fwd")]["start_duty"], 2380)
        self.assertAlmostEqual(a[("left", "fwd")]["start_v"], 6.81, delta=0.01)
        self.assertAlmostEqual(a[("right", "fwd")]["stop_v"], 5.95, delta=0.01)
        self.assertAlmostEqual(R["b"]["lr"]["left_pct"], -18.4, delta=0.1)
        P = R["proposal"]["params"]
        self.assertEqual((P["mot.dz_l_fwd"], P["mot.dz_l_rev"], P["mot.dz_r_fwd"], P["mot.dz_r_rev"]), (5.70, 5.60, 5.95, 5.95))
        self.assertEqual((P["mot.left_scale"], P["mot.right_scale"]), (1.0, 0.856))
        self.assertEqual((P["mot.model"], P["mot.vcomp"], P["mot.v_ref"]), (1.0, 1.0, 12.0))
        self.assertNotIn("mot.drive_mode", P)
        self.assertAlmostEqual(R["proposal"]["gain_ratio"], 1.18, delta=0.01)
        self.assertEqual(len(R["b"]["points"]["left_fwd"]), 16)
        json.dumps(out)                                       # goes to the browser as is
        text = "\n".join(mf.report(R))
        self.assertIn("mot.dz_l_fwd = 5.70 V", text)
        self.assertIn("left is -18.4%", text)
        self.assertIn("mot.right_scale=0.856", text)

    def test_live_state(self):
        st = mf.LiveState()
        texts = [it[2] for it in mf.read_file(REAL_MT) if it[0] == "log"]
        seen = []
        for i, t in enumerate(texts):
            st.feed(t, float(i))
            seen.append((st.state, st.wait))
            if t.startswith("mt,b,") and st.counts["b"] == 32:
                self.assertEqual(st.state, "running")
                self.assertAlmostEqual(st.progress(), (1 + 0.5) / 3, delta=0.01)
        self.assertIn(("waiting", "arm_the_remote_in_IDLE"), seen)
        self.assertIn(("waiting", "mode_not_IDLE"), seen)
        self.assertEqual(st.state, "done")
        self.assertEqual(st.progress(), 1.0)
        st.feed("mt,request,1", 1.0)
        st.feed("mt,begin,1,max=4095,vb=11.6,pitch=1.0,hz=36621,drive=0", 2.0)
        st.feed("mt,abort,pitch_rate,part=a,t=38.0,vb=11.56", 3.0)
        self.assertEqual((st.state, st.result.split(",")[0]), ("aborted", "pitch_rate"))


class TestSimMotorTest(unittest.TestCase):
    def run_sim(self, test, seconds, remote=True):
        from station.sim.sim import SimRobot
        r = SimRobot(remote=remote, seed=3)
        r.set_param(r.pidx["mot.test"], float(test))
        par, lines = lp.FrameParser(), []
        for _ in range(int(seconds * 400)):
            r.step()
            if r.out:
                for t, p in par.feed(b"".join(r.out)):
                    if t == lp.MSG_LOG:
                        lines.append(lp.decode_log(p))
                r.out = []
        return r, lines

    def test_all(self):
        r, lines = self.run_sim(4, 80)
        self.assertIn("sim: robot on the stand, wheels in the air", lines)
        self.assertTrue(any(l.startswith("mt,end,4,ok") for l in lines), lines[-5:])
        self.assertEqual(r.P("mot.test"), 0.0)                 # back to 0 when done (and echoed)
        R = mf.fit_items(mf.mt_items(lines))["results"][0]
        for row in R["a"]["rows"]:                            # TT dead zone: ~6 V to start, ~5.6 V to keep turning
            self.assertAlmostEqual(row["start_v"], 6.05, delta=0.2)
            self.assertAlmostEqual(row["stop_v"], 5.6, delta=0.15)
        self.assertAlmostEqual(R["b"]["lr"]["left_pct"], -18.0, delta=2.0)
        self.assertEqual(len(R["c"]["rows"]), 4)
        self.assertGreater(R["proposal"]["params"]["mot.left_scale"], R["proposal"]["params"]["mot.right_scale"])

    def test_needs_remote(self):
        _r, lines = self.run_sim(1, 3, remote=False)
        self.assertIn("mt,wait,arm_the_remote_in_IDLE", lines)
        self.assertFalse(any(l.startswith("mt,begin") for l in lines))


class TestBridgeTools(unittest.TestCase):
    """Presets, scoring and the motor test through the bridge (sim robot with the RC remote on)."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_commands(self):
        asyncio.run(self._run())

    async def _run(self):
        from station.bridge.bridge import Bridge
        from station.bridge.httpws import HttpServer, WsClient
        from station.sim.sim import SimRobot, SimServer
        from station.bridge.tests.test_bridge_sim import WEB, wait_for
        robot = SimRobot(remote=True, seed=2)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % srv.sockets[0].getsockname()[1], WEB, os.path.join(self.tmp, "logs"),
                   "127.0.0.1", 0, release_file=os.path.join(self.tmp, "rel"), quiet=True,
                   preset_dir=os.path.join(self.tmp, "presets"))
        hs = await HttpServer(WEB, [], "/ws", b.ws_handler).start("127.0.0.1", 0)
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        send = None
        try:
            ws = await WsClient.connect("127.0.0.1", hs.sockets[0].getsockname()[1])
            send = lambda **m: ws.send(json.dumps(m))
            pl = await wait_for(ws, lambda m: m["t"] == "presets")
            self.assertEqual(pl["list"][0]["name"], "v3 known good")
            await wait_for(ws, lambda m: m["t"] == "params" and len(m["list"]) >= 62)
            names = {p["name"]: p for p in b.params.values()}
            send(t="set_param", id=names["tlm.hz"]["id"], value=400)
            send(t="set_mode", mode=lp.MODE_BALANCE)          # remote armed: the mode follows
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["status"] & lp.TLM_UPRIGHT and m["mode"] == lp.MODE_BALANCE, 8)

            # preset: save the live values, then apply the v3 one (armed -> needs confirm)
            send(t="preset", action="save", name="sim now")
            await wait_for(ws, lambda m: m["t"] == "presets" and any(p["name"] == "sim now" for p in m["list"]))
            send(t="preset", action="plan", name="2026-11-21 (kd 7.15, ki 1500)")
            plan = await wait_for(ws, lambda m: m["t"] == "preset_plan")
            self.assertTrue(plan["armed"])
            self.assertEqual(sorted(c["name"] for c in plan["changes"]), ["bal.kd", "bal.ki"])
            send(t="preset", action="apply", name="2026-11-21 (kd 7.15, ki 1500)")
            err = await wait_for(ws, lambda m: m["t"] == "error")
            self.assertTrue(err.get("need_confirm"))
            t_apply = b.tools._t()
            send(t="preset", action="apply", name="2026-11-21 (kd 7.15, ki 1500)", confirm=True)
            await wait_for(ws, lambda m: m["t"] == "preset_applied")
            await wait_for(ws, lambda m: m["t"] == "params" and any(p["name"] == "bal.kd" and p["value"] == 7.15 for p in m["list"]))
            self.assertEqual(robot.P("bal.ki"), 1500.0)
            send(t="preset", action="diff", a="", b="sim now")
            d = await wait_for(ws, lambda m: m["t"] == "preset_diff")
            self.assertEqual(sorted(r["name"] for r in d["rows"] if not r["same"]), ["bal.kd", "bal.ki"])

            # scored runs: a marked window and "last N s"; params at the time come along
            send(t="score", action="start")
            await wait_for(ws, lambda m: m["t"] == "score_state" and m["running"])
            await asyncio.sleep(2.0)
            send(t="score", action="stop")
            runs = await wait_for(ws, lambda m: m["t"] == "score_runs" and m["runs"], 10)
            run = runs["runs"][-1]
            self.assertTrue(run["score"]["ok"], run["score"])
            self.assertGreater(run["score"]["tlm_hz"], 300)
            self.assertEqual(run["params"]["bal.kd"], 7.15)
            self.assertTrue(runs["metrics"])
            send(t="score", action="last", seconds=4)
            runs = await wait_for(ws, lambda m: m["t"] == "score_runs" and len(m["runs"]) == 2, 10)
            last = runs["runs"][-1]
            self.assertEqual(last["label"], "last 4 s")
            if last["t0"] < t_apply:                            # the window saw the preset applied
                self.assertTrue(any(c["name"] == "bal.ki" for c in last["changed"]))

            # motor test: start needs confirm; robot waits (mode not IDLE), then runs in IDLE; cancel stops it
            send(t="motor", action="start", test=1)
            err = await wait_for(ws, lambda m: m["t"] == "error")
            self.assertTrue(err.get("need_confirm"))
            send(t="motor", action="start", test=1, confirm=True)
            st = await wait_for(ws, lambda m: m["t"] == "motor_state" and m["state"] == "waiting" and m["wait"])
            self.assertEqual(st["wait"], "mode_not_IDLE")
            send(t="set_mode", mode=lp.MODE_IDLE)
            await wait_for(ws, lambda m: m["t"] == "motor_state" and m["state"] == "running", 10)
            await wait_for(ws, lambda m: m["t"] == "motor_state" and m["counts"]["a"] >= 1, 20)
            send(t="motor", action="cancel")
            st = await wait_for(ws, lambda m: m["t"] == "motor_state" and m["state"] == "aborted")
            self.assertTrue(st["result"].startswith("cancelled"))
            send(t="motor", action="fit")
            fit = await wait_for(ws, lambda m: m["t"] == "motor_fit")
            self.assertEqual(fit["runs"][-1]["status"][:7], "aborted")
            # apply: only motor-fit params
            send(t="motor", action="apply", params={"mot.drive_mode": 1})
            err = await wait_for(ws, lambda m: m["t"] == "error")
            self.assertIn("not a motor-fit param", err["text"])
            send(t="motor", action="apply", params={"mot.dz_l_fwd": 5.7, "mot.right_scale": 0.86})
            err = await wait_for(ws, lambda m: m["t"] == "error")    # remote armed in IDLE: still armed
            self.assertTrue(err.get("need_confirm"))
            send(t="motor", action="apply", params={"mot.dz_l_fwd": 5.7, "mot.right_scale": 0.86}, confirm=True)
            ap = await wait_for(ws, lambda m: m["t"] == "motor_applied")
            self.assertEqual(len(ap["changes"]), 2)
            await asyncio.sleep(0.3)
            self.assertAlmostEqual(robot.P("mot.dz_l_fwd"), 5.7, places=5)
            ws.close()
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
