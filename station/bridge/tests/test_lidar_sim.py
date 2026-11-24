"""End to end (issue #76): simulated Teensy + simulated LD19 (real 47-byte packets over a fake serial port = TCP) +
the real bridge: parser, scan assembly, pose interpolation, filters, mapping, Health tile, browser messages."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import WsClient
from station.bridge.occgrid import UNKNOWN
from station.sim.lidar_sim import LidarSim
from station.sim.sim import SimRobot, SimServer
from station.bridge.tests.test_bridge_sim import WEB, until, wait_for
from station.bridge.tests.test_lidar import cell_max


class TestLidarEndToEnd(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_map_fills_from_the_simulated_ld19(self):
        asyncio.run(self._run())

    async def _run(self):
        robot = SimRobot(seed=3)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True, lidar=LidarSim(seed=3), lidar_port=0)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        lsrv = await asyncio.start_server(sim._lidar_client, "127.0.0.1", 0)
        sim_port, lidar_port = srv.sockets[0].getsockname()[1], lsrv.sockets[0].getsockname()[1]
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % sim_port, WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True,
                   preset_dir=os.path.join(self.tmp, "presets"), lidar_port="tcp://127.0.0.1:%d" % lidar_port)
        hs = await b.build_http().start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task()),
                 asyncio.ensure_future(b.lidar.run())]
        try:
            ws = await WsClient.connect("127.0.0.1", port)
            hello = await wait_for(ws, lambda m: m["t"] == "lidar")             # settings + status on connect
            self.assertEqual(hello["settings"]["mask"], [[87.0, 93.0], [267.0, 273.0]])
            self.assertIn("defaults", hello)
            # stand the robot up (it boots lying on its face: the lidar sees the floor and must not map it)
            await until(lambda: b.tlm is not None)
            await asyncio.sleep(0.6)
            self.assertEqual(b.lidar.n["used"], 0)                              # lying down: nothing mapped
            self.assertGreater(b.lidar.n["scans"], 0)
            b.handle_command({"t": "set_mode", "mode": lp.MODE_STATION, "arm": True}, "test")
            await until(lambda: b.tlm["status"] & lp.TLM_UPRIGHT, 4.0)
            self.assertTrue(b.tlm["status"] & lp.TLM_UPRIGHT)
            await until(lambda: b.lidar.n["used"] >= 25, 6.0)
            self.assertGreaterEqual(b.lidar.n["used"], 25)

            st = b.lidar.status()
            self.assertEqual(st["state"], "ok")
            self.assertAlmostEqual(st["rpm"], 600, delta=3)
            self.assertAlmostEqual(st["pps"], 375, delta=40)
            self.assertEqual(st["crc_errors"], 0)
            self.assertTrue(st["flowing"])

            g = b.mapper.grid
            self.assertGreater(cell_max(g, 4000, 0, 150), UNKNOWN + 30)         # far wall
            self.assertGreater(cell_max(g, -1000, 0, 150), UNKNOWN + 30)        # wall behind
            self.assertLess(g.value(2000, -200), UNKNOWN - 5)                   # open floor
            self.assertEqual(g.value(6500, 0), UNKNOWN)
            # the sparse VL53L0X sweep no longer draws into the map while the lidar flows
            await until(lambda: b.mapper.n["sweep_skipped"] > 3, 3.0)
            self.assertGreater(b.mapper.n["sweep_skipped"], 3)

            # status + health + browser messages
            s = await wait_for(ws, lambda m: m["t"] == "status" and m["lidar"] and m["lidar"]["state"] == "ok")
            self.assertGreater(s["lidar"]["packets"], 100)
            h = b.health.tick(time.monotonic(), b.health_ctx())
            tile = [t for t in h["tiles"] if t["id"] == "lidar"][0]
            self.assertEqual(tile["level"], "ok", tile)
            sc = await wait_for(ws, lambda m: m["t"] == "lidar_scan")
            self.assertGreater(len(sc["pts"]), 300)
            self.assertEqual(len(sc["pts"]) % 2, 0)
            # settings over the command channel
            ws.send(json.dumps({"t": "lidar", "action": "set", "settings": {"yaw_deg": 5, "every_n": 2}}))
            r = await wait_for(ws, lambda m: m["t"] == "lidar" and m["settings"]["yaw_deg"] == 5)
            self.assertEqual(r["settings"]["every_n"], 2)
            ws.send(json.dumps({"t": "lidar", "action": "defaults"}))
            await wait_for(ws, lambda m: m["t"] == "lidar" and m["settings"]["yaw_deg"] == 0)
            self.assertTrue(os.path.exists(os.path.join(self.tmp, "maps", "lidar.json")))
        finally:
            for t in tasks + [phys]:
                t.cancel()
            b.lidar.reader.close()
            hs.close()
            srv.close()
            lsrv.close()

    def test_bridge_without_lidar_is_unchanged(self):
        async def go():
            b = Bridge("tcp://127.0.0.1:1", WEB, self.tmp, "127.0.0.1", 0, quiet=True)
            self.assertIsNone(b.lidar)
            self.assertIsNone(b.status()["lidar"])
            self.assertNotIn("lidar", [t["id"] for t in b.health.tick(time.monotonic(), b.health_ctx())["tiles"]])
            self.assertEqual(b.handle_command({"t": "lidar", "action": "get"})["t"], "error")
        asyncio.run(go())

    def test_missing_lidar_port_is_quiet_and_health_is_grey(self):
        async def go():
            b = Bridge("tcp://127.0.0.1:1", WEB, self.tmp, "127.0.0.1", 0, quiet=True, lidar_port="/dev/ttyS5-nope")
            b.lidar.reader.RETRY_S = 0.01
            before = len(b.logs)
            t = asyncio.ensure_future(b.lidar.run())
            await asyncio.sleep(0.3)
            t.cancel()
            self.assertEqual(len(b.logs) - before, 1)                           # one line, not one per retry
            tile = [x for x in b.health.tick(time.monotonic(), b.health_ctx())["tiles"] if x["id"] == "lidar"][0]
            self.assertEqual(tile["level"], "na")
            self.assertEqual(b.status()["lidar"]["state"], "waiting")
        asyncio.run(go())


if __name__ == "__main__":
    unittest.main()
