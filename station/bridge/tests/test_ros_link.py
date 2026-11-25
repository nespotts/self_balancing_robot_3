"""ROS inside the station (issue #106): station/bridge/ros_link.py relays the adapter's ROS map / pose / Nav2 status
to the station clients and forwards the station's goals; plus one run through the real bridge + sim over WebSocket."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import health
from station.bridge import link_protocol as lp
from station.bridge import ros_link as rl
from station.bridge.bridge import Bridge
from station.bridge.httpws import WsClient
from station.bridge.tests.test_bridge_sim import WEB, until, wait_for
from station.sim.lidar_sim import LidarSim
from station.sim.sim import SimRobot, SimServer


class FakeWs:
    def __init__(self, backlog=0):
        self.sent = []
        self.closed = False
        self.backlog = backlog
        self.peer = ("127.0.0.1", 1)

    def send(self, text):
        self.sent.append(text)

    def buffered(self):
        return self.backlog

    def msgs(self, t):
        return [m for m in (json.loads(x) for x in self.sent) if m["t"] == t]


class FakeStream:
    def __init__(self):
        self.clients = set()


class FakeBridge:
    def __init__(self):
        self.clients = set()
        self.ros = FakeStream()
        self.tlm, self.tlm_time = None, -10.0
        self.link_state, self.estop, self.robot_estop = "connected", False, False
        self.lines, self.status_pushes = [], 0

    def say(self, text):
        self.lines.append(text)

    def push_status(self):
        self.status_pushes += 1


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def rig():
    clk = Clock()
    b = FakeBridge()
    link = rl.RosLink(b, clock=clk)
    adapter, phone, slow = FakeWs(), FakeWs(), FakeWs(backlog=10 ** 6)
    b.clients |= {adapter, phone, slow}
    b.ros.clients.add(adapter)
    return clk, b, link, adapter, phone, slow


def drive_ready(b, clk, mode=lp.MODE_STATION, armed=True):
    b.tlm = {"mode": mode, "status": lp.TLM_ARMED if armed else 0, "x_mm": 0.0, "y_mm": 0.0}
    b.tlm_time = clk.t


class RosLinkTest(unittest.TestCase):
    def test_nothing_without_adapter(self):
        clk, b, link, adapter, phone, _ = rig()
        self.assertIsNone(link.status())
        self.assertIsNone(link.health())
        r = link.command({"t": "ros", "action": "goal", "x": 1, "y": 2}, "phone")
        self.assertEqual(r["code"], "ros_offline")
        link.welcome(phone)
        self.assertEqual(phone.sent, [])
        self.assertEqual(adapter.sent, [])

    def test_only_subscribers_speak_for_ros(self):
        clk, b, link, adapter, phone, _ = rig()
        link.from_adapter(phone, {"t": "ros_status", "loc": None})        # a browser cannot pretend to be ROS
        self.assertIsNone(link.status())
        self.assertEqual(phone.sent, [])

    def test_relay_cache_welcome_timeout(self):
        clk, b, link, adapter, phone, slow = rig()
        link.from_adapter(adapter, {"t": "ros_status", "loc": {"state": "ok", "quality": 0.9},
                                    "nodes": {"nav2": "active"}, "drivable": False})
        link.from_adapter(adapter, {"t": "ros_map", "w": 2, "h": 1, "rle": "AQI="})
        link.from_adapter(adapter, {"t": "ros_pose", "p": [1, 2, 0], "T": [0, 0, 0]})
        self.assertEqual([m["t"] for m in map(json.loads, phone.sent)], ["ros_status", "ros_map", "ros_pose"])
        self.assertEqual(adapter.sent, [])                                 # not echoed back to ROS
        self.assertEqual([m["t"] for m in map(json.loads, slow.sent)], ["ros_status", "ros_map"])   # pose droppable
        st = link.status()
        self.assertTrue(st["connected"])
        self.assertEqual((st["loc"], st["nav2"]), ("ok", "active"))
        late = FakeWs()
        link.welcome(late)
        self.assertEqual([m["t"] for m in map(json.loads, late.sent)], ["ros_status", "ros_map", "ros_pose"])
        self.assertEqual(health.ros_tile(link.health())["value"], "Nav2 ready")
        clk.t += rl.TIMEOUT_S + 0.1
        n = b.status_pushes
        link.tick(clk.t)
        self.assertFalse(link.status()["connected"])
        self.assertGreater(b.status_pushes, n)
        tile = health.ros_tile(link.health())
        self.assertEqual((tile["level"], tile["value"]), (health.BAD, "offline"))
        self.assertEqual(link.command({"t": "ros", "action": "cancel"}, "phone")["code"], "ros_offline")
        late2 = FakeWs()
        link.welcome(late2)
        self.assertEqual(late2.sent, [])

    def test_commands_need_manual(self):
        clk, b, link, adapter, phone, _ = rig()
        link.from_adapter(adapter, {"t": "ros_status"})
        drive_ready(b, clk, mode=lp.MODE_BALANCE)
        r = link.command({"t": "ros", "action": "goal", "x": 1, "y": 2, "yaw": 0.5}, "phone")
        self.assertEqual((r["code"], r["text"]), ("not_manual", rl.MANUAL_TEXT))
        drive_ready(b, clk, armed=False)
        self.assertIn("disarmed", link.command({"t": "ros", "action": "place", "name": "Kitchen"}, "phone")["text"])
        b.estop = True
        self.assertIn("E-stop", link.command({"t": "ros", "action": "patrol", "places": ["A"]}, "phone")["text"])
        b.estop = False
        self.assertEqual(adapter.sent, [])
        # cancel / initial pose / explore stop / save map never need Manual
        for m in ({"action": "cancel"}, {"action": "initial_pose", "x": 0, "y": 0, "yaw": 0},
                  {"action": "explore", "run": False}, {"action": "save_map", "name": "x"}):
            self.assertEqual(link.command(dict(m, t="ros"), "phone")["t"], "ros_sent")
        self.assertEqual([m["cmd"] for m in adapter.msgs("ros_cmd")], ["cancel", "initial_pose", "explore", "save_map"])
        self.assertFalse(b.tlm["status"] & lp.TLM_ARMED)                  # nothing armed anything

    def test_forward_goal(self):
        clk, b, link, adapter, phone, _ = rig()
        link.from_adapter(adapter, {"t": "ros_status"})
        drive_ready(b, clk)
        r = link.command({"t": "ros", "action": "goal", "x": 1.5, "y": -0.5, "yaw": 1.0, "id": "x", "cmd": "spoof"},
                         "phone")
        self.assertEqual(r["t"], "ros_sent")
        c = adapter.msgs("ros_cmd")[-1]
        self.assertEqual((c["cmd"], c["x"], c["y"], c["yaw"], c["who"], c["id"]), ("goal", 1.5, -0.5, 1.0, "phone", r["id"]))
        r = link.command({"t": "ros", "action": "patrol", "places": ["A", "B"], "loops": 2, "dwell": 4}, "phone")
        self.assertEqual(adapter.msgs("ros_cmd")[-1]["places"], ["A", "B"])
        self.assertEqual(link.command({"t": "ros", "action": "fly"}, "phone")["t"], "error")
        self.assertEqual(link.command({"t": "ros", "action": "place", "name": " "}, "phone")["t"], "error")

    def test_station_pose_hook(self):
        clk, b, link, adapter, phone, _ = rig()
        self.assertFalse(link.station_pose_set(100, 200, 90))
        link.from_adapter(adapter, {"t": "ros_status"})
        self.assertTrue(link.station_pose_set(100, 200, 90))
        c = adapter.msgs("ros_cmd")[-1]
        self.assertEqual((c["cmd"], c["frame"], c["x"], c["y"]), ("initial_pose", "station", 100.0, 200.0))
        self.assertAlmostEqual(c["yaw"], 1.5708, places=3)

    def test_adapter_drop(self):
        clk, b, link, adapter, phone, _ = rig()
        link.from_adapter(adapter, {"t": "ros_nav", "state": "navigating"})
        self.assertEqual(link.status()["nav"], "navigating")
        link.drop(phone)
        self.assertTrue(link.connected())
        link.drop(adapter)
        self.assertFalse(link.connected())

    def test_explore_relayed_never_refused(self):
        clk, b, link, adapter, phone, _ = rig()
        link.from_adapter(adapter, {"t": "ros_explore", "state": "exploring", "target": [100, 200], "frontiers": 3,
                                    "area_m2": 12.5, "reason": ""})
        self.assertEqual(phone.msgs("ros_explore")[0]["frontiers"], 3)
        self.assertTrue(link.status()["explore"])
        self.assertEqual(adapter.sent, [])


class TestBridgeRosLink(unittest.TestCase):
    """Real bridge + sim: a fake adapter (subscribed to the ROS stream) talks ROS; a phone sees it and sends goals."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_over_websocket(self):
        asyncio.run(self._run())

    async def _run(self):
        robot = SimRobot(seed=5)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True, lidar=LidarSim(seed=5), lidar_port=0)
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
            phone = await WsClient.connect("127.0.0.1", port)
            st = await wait_for(phone, lambda m: m["t"] == "status")
            self.assertIsNone(st["ros"])                                   # no adapter: nothing about ROS
            await until(lambda: b.tlm is not None)
            phone.send(json.dumps({"t": "ros", "action": "goal", "x": 1, "y": 0}))
            e = await wait_for(phone, lambda m: m["t"] == "error")
            self.assertEqual(e["code"], "ros_offline")
            ad = await WsClient.connect("127.0.0.1", port)
            ad.send(json.dumps({"t": "ros", "action": "subscribe"}))
            await wait_for(ad, lambda m: m["t"] == "ros_hello")
            ad.send(json.dumps({"t": "ros_status", "loc": {"state": "ok", "quality": 0.8},
                                "nodes": {"nav2": "active", "slam": True}, "tf": True}))
            ad.send(json.dumps({"t": "ros_map", "w": 2, "h": 1, "res": 0.05, "origin": [0, 0, 0], "rle": "AQI="}))
            m = await wait_for(phone, lambda m: m["t"] == "ros_map")
            self.assertEqual(m["w"], 2)
            st = await wait_for(phone, lambda m: m["t"] == "status" and m["ros"] and m["ros"]["connected"], 3.0)
            self.assertEqual(st["ros"]["loc"], "ok")
            h = await wait_for(phone, lambda m: m["t"] == "health" and any(t["id"] == "ros" for t in m["tiles"]), 3.0)
            self.assertEqual(next(t for t in h["tiles"] if t["id"] == "ros")["value"], "Nav2 ready")
            # not in Manual: refused, nothing reaches ROS
            phone.send(json.dumps({"t": "ros", "action": "place", "name": "Kitchen"}))
            e = await wait_for(phone, lambda m: m["t"] == "error" and m.get("code") == "not_manual")
            self.assertIn("Manual", e["text"])
            b.handle_command({"t": "set_mode", "mode": lp.MODE_STATION, "arm": True}, "test")   # the owner on the phone
            await until(lambda: b.tlm and b.tlm["mode"] == lp.MODE_STATION and b.tlm["status"] & lp.TLM_ARMED, 6.0)
            phone.send(json.dumps({"t": "ros", "action": "goal", "x": 1.2, "y": 0.3, "yaw": 0.0}))
            await wait_for(phone, lambda m: m["t"] == "ros_sent")
            c = await wait_for(ad, lambda m: m["t"] == "ros_cmd")
            self.assertEqual((c["cmd"], c["x"], c["y"]), ("goal", 1.2, 0.3))
            # the HTTP API path is the same command
            self.assertEqual(b.handle_command({"t": "ros", "action": "cancel"}, "http")["t"], "ros_sent")
            c = await wait_for(ad, lambda m: m["t"] == "ros_cmd")
            self.assertEqual(c["cmd"], "cancel")
            ad.send(json.dumps({"t": "ros_nav", "state": "cancelled", "reason": "cancelled from the station"}))
            n = await wait_for(phone, lambda m: m["t"] == "ros_nav")
            self.assertEqual(n["state"], "cancelled")
            ad.close()
            st = await wait_for(phone, lambda m: m["t"] == "status" and m["ros"] and not m["ros"]["connected"], 3.0)
            self.assertFalse(st["ros"]["connected"])
        finally:
            for t in tasks + [phys]:
                t.cancel()
            b.lidar.reader.close()
            hs.close()
            srv.close()
            lsrv.close()


if __name__ == "__main__":
    unittest.main()
