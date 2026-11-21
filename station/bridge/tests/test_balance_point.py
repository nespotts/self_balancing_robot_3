"""Find balance point (issue #52): the estimator, the state machine against a fake plant (fast, fake clock), and
the whole thing through the bridge API against the simulator (real time, deliberately wrong bal.offset_deg).
Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import os
import random
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge.balance_point import BalancePoint, Estimator, propose

TRUE_OFFSET = 3.15
WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


class FakeBridge:
    """Just what BalancePoint uses; the 'robot' leans vel_out = offset - TRUE (lagging, plus noise)."""

    def __init__(self, offset=1.0, true=TRUE_OFFSET, noise=0.05, tau=2.0, seed=1):
        self.params = {0: {"id": 0, "name": "bal.offset_deg", "value": offset, "min": -15.0, "max": 15.0},
                       1: {"id": 1, "name": "ctl.loops", "value": 3.0, "min": 0.0, "max": 3.0},
                       2: {"id": 2, "name": "vel.limit_deg", "value": 20.0, "min": 0.0, "max": 45.0}}
        self.true, self.noise, self.tau = true, noise, tau
        self.rnd = random.Random(seed)
        self.lean = offset - true
        self.link_state, self.estop, self.cmd_active = "connected", False, False
        self.tlm_time = 0.0
        self.tlm = self._frame()
        self.said, self.sent, self.cmds = [], [], []
        self.speed = 0.0
        self.bal_i = 0.0

    def _frame(self):
        return {"status": lp.TLM_ARMED | lp.TLM_UPRIGHT, "mode": lp.MODE_BALANCE, "speed_mms": 0.0,
                "pitch_deg": -3.15, "balance_target_deg": -3.15, "cmd_speed_mms": 0.0, "cmd_turn_dps": 0.0,
                "vel_out_deg": 0.0, "bal_i": 0.0}

    def say(self, text):
        self.said.append(text)

    def broadcast(self, text, droppable=False):
        self.sent.append(json.loads(text))

    def handle_command(self, m, who="?"):
        self.cmds.append(m)
        if m["t"] == "set_param":
            p = self.params[m["id"]]
            p["value"] = max(p["min"], min(p["max"], m["value"]))

    def offset(self):
        return self.params[0]["value"]


def run(bp, fb, seconds, t0, dt=0.04, hook=None):
    """Advance the fake robot + bridge by `seconds` (25 Hz telemetry); returns the new time."""
    t = t0
    end = t0 + seconds
    while t < end:
        t += dt
        target_lean = fb.offset() - fb.true
        fb.lean += (target_lean - fb.lean) * min(1.0, dt / fb.tau)
        d = fb._frame()
        d.update(vel_out_deg=fb.lean + fb.rnd.gauss(0, fb.noise), speed_mms=fb.speed + fb.rnd.gauss(0, 5),
                 bal_i=fb.bal_i, balance_target_deg=fb.lean - fb.offset())
        d["pitch_deg"] = d["balance_target_deg"] + fb.rnd.gauss(0, 0.3)       # holding its target
        if hook:
            hook(d, t)
        fb.tlm, fb.tlm_time = d, t
        bp.on_tlm(d, t)
        bp.tick(t)
        if bp.state in ("done", "aborted", "idle") and bp.round and not hook:
            break
    return t


class TestEstimator(unittest.TestCase):
    def test_mean_and_halves(self):
        e = Estimator()
        for i in range(100):
            e.add(i * 0.1, -2.0 if i < 50 else -1.0, 3.0, 10.0, 0.0)
        s = e.stats()
        self.assertEqual(s["n"], 100)
        self.assertAlmostEqual(s["mean"], -1.5)
        self.assertAlmostEqual(s["first"], -2.0)
        self.assertAlmostEqual(s["second"], -1.0)
        self.assertAlmostEqual(s["dur"], 9.9)
        self.assertEqual(Estimator().stats()["n"], 0)

    def test_propose(self):
        self.assertAlmostEqual(propose(3.15, 0.42), 2.73)            # lean forward (+): balance point is lower
        self.assertAlmostEqual(propose(1.5, -1.65), 3.15)
        self.assertAlmostEqual(propose(1.0, -9.0, max_step=2.0), 3.0)  # at most max_step per round
        self.assertAlmostEqual(propose(14.5, -3.0, hi=15.0), 15.0)     # inside the param range


class TestRoutine(unittest.TestCase):
    OPTS = {"settle_s": 4.0, "window_s": 6.0, "rounds": 3}

    def start(self, fb, **kw):
        bp = BalancePoint(fb)
        fb.tlm_time = 0.0
        m = dict(self.OPTS, **kw)
        self.assertEqual(bp.command(dict(m, action="start"), "test", now=0.0)["t"], "ok")
        return bp

    def test_auto_converges_and_never_changes_mode(self):
        fb = FakeBridge(offset=1.0)
        bp = self.start(fb, auto=True)
        run(bp, fb, 120, 0.0)
        self.assertEqual(bp.state, "done", bp.text)
        self.assertAlmostEqual(fb.offset(), TRUE_OFFSET, delta=0.15)
        self.assertLessEqual(bp.round, 3)
        self.assertTrue(all(c["t"] == "set_param" and c["id"] == 0 for c in fb.cmds))   # only bal.offset_deg
        self.assertIn("(was 1.00)", bp.text)
        self.assertEqual(bp.history[0]["offset"], 1.0)
        self.assertAlmostEqual(bp.history[0]["mean"], 1.0 - TRUE_OFFSET, delta=0.5)      # still settling in round 1

    def test_slew_is_gradual(self):
        fb = FakeBridge(offset=1.0)
        bp = self.start(fb, auto=True)
        run(bp, fb, 120, 0.0)
        vals = [c["value"] for c in fb.cmds]
        self.assertGreater(len(vals), 5)
        self.assertLess(max(abs(b - a) for a, b in zip(vals, vals[1:])), 0.2)            # no jumps

    def test_manual_propose_apply_verify(self):
        fb = FakeBridge(offset=2.0)
        bp = self.start(fb)
        run(bp, fb, 60, 0.0)
        self.assertEqual(bp.state, "proposal", bp.text)
        self.assertAlmostEqual(bp.proposal, TRUE_OFFSET, delta=0.2)
        self.assertEqual(fb.offset(), 2.0)                       # nothing applied without Apply
        self.assertEqual(fb.cmds, [])
        t = fb.tlm_time
        bp.command({"action": "apply"}, "test", now=t)
        run(bp, fb, 60, t)
        self.assertEqual(bp.state, "done", bp.text)
        self.assertAlmostEqual(fb.offset(), TRUE_OFFSET, delta=0.2)
        self.assertEqual(len(bp.history), 2)                     # measured, applied, measured again

    def test_already_balanced(self):
        fb = FakeBridge(offset=TRUE_OFFSET)
        bp = self.start(fb)
        run(bp, fb, 60, 0.0)
        self.assertEqual(bp.state, "done", bp.text)
        self.assertEqual(fb.cmds, [])
        self.assertIn("is right", bp.text)

    def test_step_limited_per_round(self):
        fb = FakeBridge(offset=1.0, true=9.0)                    # 8 deg off: the lean would hit vel.limit_deg
        bp = self.start(fb, max_step_deg=2.0)
        fb.params[2]["value"] = 45.0
        run(bp, fb, 60, 0.0)
        self.assertEqual(bp.state, "proposal", bp.text)
        self.assertAlmostEqual(bp.proposal, 3.0, delta=0.05)     # 1 + 2 (clamped), not 9

    def test_far_off_hits_velocity_limit(self):
        fb = FakeBridge(offset=1.0, true=25.0, tau=0.5)
        fb.params[0]["max"] = 30.0
        bp = self.start(fb)
        run(bp, fb, 60, 0.0)
        self.assertEqual(bp.state, "aborted")
        self.assertIn("velocity limit", bp.text)

    def test_refuses_to_start(self):
        fb = FakeBridge()
        bp = BalancePoint(fb)
        fb.tlm_time = 0.0
        for patch, word in (({"status": lp.TLM_UPRIGHT}, "Arm"), ({"status": lp.TLM_ARMED}, "upright"),
                            ({"mode": lp.MODE_STATION}, "BALANCE"), ({"speed_mms": 200.0}, "moving"),
                            ({"cmd_speed_mms": 100.0}, "driven"), ({"pitch_deg": 5.0}, "leaning")):
            fb.tlm = dict(fb._frame(), **patch)
            with self.assertRaises(ValueError, msg=word) as cm:
                bp.command({"action": "start"}, "test", now=0.5)
            self.assertIn(word, str(cm.exception))
        fb.tlm = fb._frame()
        fb.cmd_active = True
        with self.assertRaises(ValueError):
            bp.command({"action": "start"}, "test", now=0.5)
        fb.cmd_active = False
        fb.params[1]["value"] = 1.0                              # velocity loop off: vel_out would read 0
        with self.assertRaises(ValueError) as cm:
            bp.command({"action": "start"}, "test", now=0.5)
        self.assertIn("velocity loop", str(cm.exception))
        fb.params[1]["value"] = 3.0
        with self.assertRaises(ValueError):                      # telemetry stale
            bp.command({"action": "start"}, "test", now=5.0)
        self.assertEqual(bp.state, "idle")
        self.assertEqual(fb.cmds, [])

    def abort_case(self, hook=None, tick_only=None, word=""):
        fb = FakeBridge(offset=1.0)
        bp = self.start(fb, auto=True)
        end = run(bp, fb, 8, 0.0, hook=lambda d, t: None)
        self.assertIn(bp.state, ("settle", "measure"))
        before = fb.offset()
        if hook:
            run(bp, fb, 2, end, hook=hook)
        if tick_only:
            tick_only(fb, bp, end + 0.04)
        self.assertEqual(bp.state, "aborted", bp.text)
        self.assertIn(word, bp.text)
        if "someone" not in word:
            self.assertEqual(fb.offset(), before)                # an abort leaves the offset alone
        return fb, bp

    def test_aborts(self):
        self.abort_case(hook=lambda d, t: d.update(speed_mms=400.0), word="pushed")
        self.abort_case(hook=lambda d, t: d.update(pitch_deg=d["balance_target_deg"] + 12), word="pushed")
        self.abort_case(hook=lambda d, t: d.update(status=lp.TLM_UPRIGHT), word="disarmed")
        self.abort_case(hook=lambda d, t: d.update(mode=lp.MODE_IDLE), word="mode")
        self.abort_case(hook=lambda d, t: d.update(status=lp.TLM_ARMED), word="fell")
        self.abort_case(tick_only=lambda fb, bp, t: (setattr(fb, "cmd_active", True), bp.tick(t)), word="driving")
        self.abort_case(tick_only=lambda fb, bp, t: (setattr(fb, "estop", True), bp.tick(t)), word="E-stop")
        self.abort_case(tick_only=lambda fb, bp, t: bp.tick(t + 5), word="Lost")                # telemetry went quiet
        self.abort_case(tick_only=lambda fb, bp, t: (setattr(fb, "link_state", "waiting"), bp.tick(t)), word="Lost")
        self.abort_case(tick_only=lambda fb, bp, t: bp.on_lost(), word="Lost")
        self.abort_case(tick_only=lambda fb, bp, t: (fb.params[0].update(value=fb.offset() + 0.5), bp.tick(t)),
                        word="someone else")
        _fb, bp = self.abort_case(tick_only=lambda fb, bp, t: bp.command({"action": "stop"}, "test", now=t), word="Stopped")

    def test_drifting_is_not_a_measurement(self):
        fb = FakeBridge(offset=1.0)
        fb.speed = 120.0                                          # rolling steadily (slope?): no valid balance point
        bp = self.start(fb)
        run(bp, fb, 60, 0.0)
        self.assertEqual(bp.state, "aborted")
        self.assertIn("drifting", bp.text)

    def test_unsettled_window_is_measured_again(self):
        fb = FakeBridge(offset=1.0, true=6.0, tau=12.0)                      # slow integrator: the first window still moves
        fb.lean = 0.0
        bp = self.start(fb)
        run(bp, fb, 80, 0.0)
        self.assertTrue(any("measuring again" in s for s in fb.said))

    def test_revert(self):
        fb = FakeBridge(offset=1.0)
        bp = self.start(fb, auto=True)
        t = run(bp, fb, 120, 0.0)
        self.assertGreater(fb.offset(), 2.5)
        bp.command({"action": "revert"}, "test", now=t)
        run(bp, fb, 5, t, hook=lambda d, t: None)
        self.assertAlmostEqual(fb.offset(), 1.0, places=2)
        self.assertEqual(bp.state, "idle")

    def test_knob_trim_warning(self):
        fb = FakeBridge(offset=TRUE_OFFSET)
        bp = self.start(fb)
        run(bp, fb, 60, 0.0, hook=lambda d, t: d.update(balance_target_deg=d["vel_out_deg"] - TRUE_OFFSET - 0.6))
        self.assertTrue(any("Knob trim" in w for w in bp.warn), bp.warn)


# ---------------------------------------------------------------- simulator through the bridge API

async def wait_for(ws, pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = json.loads(await asyncio.wait_for(ws.recv(), max(0.01, end - time.monotonic())))
        if pred(m):
            return m
    raise AssertionError("timeout")


class TestWithSim(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_converges_in_the_sim(self):
        asyncio.run(self._run())

    async def _run(self):
        from station.bridge.bridge import Bridge
        from station.bridge.httpws import HttpServer, WsClient
        from station.sim.sim import SimRobot, SimServer
        robot = SimRobot(seed=5)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % srv.sockets[0].getsockname()[1], WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True,
                   preset_dir=os.path.join(self.tmp, "presets"))
        http = HttpServer(WEB, [], "/ws", b.ws_handler)
        hs = await http.start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        try:
            ws = await WsClient.connect("127.0.0.1", port)
            send = lambda **m: ws.send(json.dumps(m))
            params = await wait_for(ws, lambda m: m["t"] == "params" and len(m["list"]) >= 72)
            pid = next(p["id"] for p in params["list"] if p["name"] == "bal.offset_deg")
            send(t="balpoint", action="status")
            first = await wait_for(ws, lambda m: m["t"] == "balpoint_state")
            self.assertEqual(first["state"], "idle")

            send(t="set_param", id=pid, value=1.0)        # the robot's real balance point is 3.15 deg
            send(t="set_mode", mode=lp.MODE_BALANCE, arm=True)
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["status"] & lp.TLM_UPRIGHT, 8)
            # not before it stands (the routine refuses to start on a moving robot), then run on its own
            end = time.monotonic() + 15
            while True:
                send(t="balpoint", action="start", auto=True, settle_s=6, window_s=8, rounds=4)
                m = await wait_for(ws, lambda m: m["t"] in ("error", "balpoint_state", "ok"), 3)
                if m["t"] != "error":
                    break
                self.assertLess(time.monotonic(), end, m)
                await asyncio.sleep(0.5)
            modes = set()
            while True:
                m = await wait_for(ws, lambda m: m["t"] in ("balpoint_state", "tlm"), 150)
                if m["t"] == "tlm":
                    modes.add(m["mode"])
                    continue
                if m["state"] in ("done", "aborted"):
                    break
            self.assertEqual(m["state"], "done", m["text"])
            self.assertEqual(modes, {lp.MODE_BALANCE})                  # never changed the mode
            self.assertLessEqual(m["round"], 4)
            offset = next(p["value"] for p in b.params.values() if p["name"] == "bal.offset_deg")
            self.assertAlmostEqual(offset, TRUE_OFFSET, delta=0.3, msg=str(m["history"]))
            self.assertAlmostEqual(m["offset"], offset, places=2)
            self.assertEqual(m["orig"], 1.0)
            ws.close()
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
