"""Fall recorder inside the bridge: simulator knocked over -> a record appears (WebSocket "fall" message, /api/falls)."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest
import urllib.error
import urllib.request

from station.bridge import falls as F
from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import HttpServer, WsClient
from station.sim.sim import SimRobot, SimServer

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


async def wait_for(ws, pred, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = json.loads(await asyncio.wait_for(ws.recv(), end - time.monotonic()))
        if pred(m):
            return m
    raise AssertionError("timeout")


class TestBridgeFalls(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_sim_fall_end_to_end(self):
        asyncio.run(self._run())

    async def _run(self):
        robot = SimRobot(seed=2)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        sim_port = srv.sockets[0].getsockname()[1]
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % sim_port, WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True,
                   preset_dir=os.path.join(self.tmp, "presets"), falls_dir=os.path.join(self.tmp, "falls"))
        routes = [("*", "/api/cmd", b.api_cmd), ("*", "/api/falls", b.api_falls), ("*", "/api/falls/", b.api_falls)]
        http = HttpServer(WEB, routes, "/ws", b.ws_handler)
        hs = await http.start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        loop = asyncio.get_running_loop()

        def http_get(path, method="GET"):
            req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), method=method)
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.read()

        async def get_json(path, method="GET"):
            return json.loads(await loop.run_in_executor(None, http_get, path, method))
        try:
            ws = await WsClient.connect("127.0.0.1", port)
            await wait_for(ws, lambda m: m["t"] == "tlm")
            empty = await get_json("/api/falls")
            self.assertEqual(empty["falls"], [])
            self.assertIn("saturated", empty["tags"])

            # arm (no remote: the station arms), balance, a small shove is recovered
            ws.send(json.dumps({"t": "set_mode", "mode": lp.MODE_STATION}))
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["status"] & lp.TLM_UPRIGHT)
            await asyncio.sleep(2.5)
            robot.push_robot(60.0)
            await asyncio.sleep(1.5)
            self.assertEqual((await get_json("/api/falls"))["falls"], [])

            # a big shove topples it: the recorder saves it ~2 s later and tells the browsers
            robot.push_robot(800.0)
            m = await wait_for(ws, lambda m: m["t"] == "fall", 10)
            fall = m["fall"]
            self.assertEqual(fall["cause"], "pushed")
            self.assertTrue(fall["tipped"])
            self.assertTrue(fall["session"].startswith("sbr3-"))
            lst = await get_json("/api/falls")
            self.assertEqual([f["id"] for f in lst["falls"]], [fall["id"]])
            self.assertEqual(lst["falls"][0]["tags"][0]["tag"], "pushed")
            self.assertEqual(lst["stats"]["saved"], 1)
            self.assertTrue(lst["falls"][0]["spark"]["pitch"])

            rec = await get_json("/api/falls/" + fall["id"])
            self.assertEqual(rec["cols"][0], "t")
            self.assertGreater(len(rec["rows"]), 100)
            pitch = [r[rec["cols"].index("pitch_deg")] for r in rec["rows"]]
            self.assertGreater(max(abs(p) for p in pitch), 40)
            self.assertEqual(rec["meta"]["params"]["bal.cutoff_deg"], 45.0)
            self.assertTrue(rec["rng"])                                   # the sim sweeps its range sensors
            raw = (await loop.run_in_executor(None, http_get, "/api/falls/%s?raw=1" % fall["id"])).decode()
            self.assertTrue(raw.startswith('["fall",'))
            self.assertIn('["tlm",', raw)

            # E-stop while it is up again: the stop is not a fall
            ws.send(json.dumps({"t": "set_mode", "mode": lp.MODE_STATION}))
            await wait_for(ws, lambda m: m["t"] == "tlm" and m["status"] & lp.TLM_UPRIGHT, 10)
            await asyncio.sleep(1.0)
            ws.send(json.dumps({"t": "estop"}))
            await asyncio.sleep(4.0)
            self.assertEqual(len((await get_json("/api/falls"))["falls"]), 1)

            # bad names are refused; delete works
            with self.assertRaises(urllib.error.HTTPError):
                await loop.run_in_executor(None, http_get, "/api/falls/..%2f..%2fsecret")
            await get_json("/api/falls/" + fall["id"], "DELETE")
            self.assertEqual((await get_json("/api/falls"))["falls"], [])
            ws.close()
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.falls.flush()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
