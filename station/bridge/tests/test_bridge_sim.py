"""End-to-end: simulator <-> bridge over TCP, browser side over WebSocket/HTTP.
Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest
import urllib.request

from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import WsClient
from station.sim.sim import SimRobot, SimServer

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


async def wait_for(ws, pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = json.loads(await asyncio.wait_for(ws.recv(), end - time.monotonic()))
        if pred(m):
            return m
    raise AssertionError("timeout")


class TestBridgeWithSim(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_end_to_end(self):
        asyncio.run(self._run())

    async def _run(self):
        robot = SimRobot(seed=1)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        sim_port = srv.sockets[0].getsockname()[1]
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % sim_port, WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True)
        from station.bridge.httpws import HttpServer
        routes = [("GET", "/api/state", b.api_state), ("*", "/api/cmd", b.api_cmd),
                  ("GET", "/api/logs", b.api_logs), ("*", "/api/logs/", b.api_logs),
                  ("*", "/api/serial", b.api_serial)]
        http = HttpServer(WEB, routes, "/ws", b.ws_handler)
        hs = await http.start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        loop = asyncio.get_running_loop()

        def http_get(path, data=None):
            req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=data)
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.read()
        try:
            ws = await WsClient.connect("127.0.0.1", port)
            hello = await wait_for(ws, lambda m: m["t"] == "hello")
            self.assertEqual(hello["modes"][3], "STATION")
            params = await wait_for(ws, lambda m: m["t"] == "params" and len(m["list"]) >= 11)
            names = {p["name"]: p for p in params["list"]}
            self.assertEqual(names["bal.kp"]["value"], 250.0)
            self.assertEqual(names["vel.kd"]["value"], 0.00007)
            tlm = await wait_for(ws, lambda m: m["t"] == "tlm")
            self.assertEqual(tlm["mode"], lp.MODE_IDLE)
            await wait_for(ws, lambda m: m["t"] == "ranges" and len(m["pts"]) > 0)

            # drive in STATION
            ws.send(json.dumps({"t": "set_mode", "mode": lp.MODE_STATION}))
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["mode"] == lp.MODE_STATION)
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["status"] & lp.TLM_UPRIGHT)
            for _ in range(25):
                ws.send(json.dumps({"t": "cmd_vel", "speed": 300, "turn": 0}))
                await asyncio.sleep(0.05)
            m = b.tlm  # (the ws queue holds older messages we have not read yet)
            self.assertGreater(m["speed_mms"], 100)
            self.assertGreater(m["x_mm"], 50)

            # stop sending -> bridge stops cmd_vel after the timeout -> robot slows down
            await asyncio.sleep(1.2)
            m = b.tlm
            self.assertLess(abs(m["cmd_speed_mms"]), 1)
            self.assertFalse(b.cmd_active)

            # set a param, echo comes back
            pid = names["bal.kp"]["id"]
            ws.send(json.dumps({"t": "set_param", "id": pid, "value": 260}))
            await wait_for(ws, lambda m: m["t"] == "params" and
                           any(p["id"] == pid and p["value"] == 260 for p in m["list"]))

            # goto
            ws.send(json.dumps({"t": "goto", "points": [[600, 0], [600, 400]], "replace": True, "go": True}))
            await wait_for(ws, lambda m: m["t"] == "waypoints" and len(m["points"]) == 2)
            await wait_for(ws, lambda m: m["t"] == "nav" and m["waypoint_count"] == 2)
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["mode"] == lp.MODE_GOTO)

            # e-stop -> IDLE
            ws.send(json.dumps({"t": "estop"}))
            await wait_for(ws, lambda m: m["t"] == "status" and m["estop"])
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["mode"] == lp.MODE_IDLE, 2)

            # HTTP polling endpoint (Kindle)
            st = json.loads(await loop.run_in_executor(None, http_get, "/api/state?r=0&params=1"))
            self.assertEqual(st["status"]["link"], "connected")
            self.assertTrue(st["ranges"])
            self.assertIn("pitch_deg", st["tlm"])
            self.assertTrue(st["params"])
            r = json.loads(await loop.run_in_executor(None, http_get, "/api/cmd?t=set_mode&mode=1"))
            self.assertEqual(r["t"], "ok")
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["mode"] == lp.MODE_BALANCE)

            # session logs
            logs = json.loads(await loop.run_in_executor(None, http_get, "/api/logs"))
            self.assertEqual(len(logs["sessions"]), 1)
            name = logs["sessions"][0]["name"]
            csv = (await loop.run_in_executor(None, http_get, "/api/logs/%s?csv=tlm" % name)).decode()
            lines = csv.splitlines()
            self.assertTrue(lines[0].startswith("t_s,t_us,mode"))
            self.assertGreater(len(lines), 100)
            raw = (await loop.run_in_executor(None, http_get, "/api/logs/" + name)).decode()
            self.assertIn('["rng"', raw)

            # static files
            html = (await loop.run_in_executor(None, http_get, "/")).decode()
            self.assertIn("<html", html.lower())

            # release the serial port (flashing) and re-attach
            await loop.run_in_executor(None, http_get, "/api/serial?release=1", b"")
            await asyncio.sleep(1.3)
            self.assertEqual(b.link_state, "released")
            await loop.run_in_executor(None, http_get, "/api/serial?release=0", b"")
            await asyncio.sleep(1.3)
            self.assertEqual(b.link_state, "connected")
            ws.close()
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
