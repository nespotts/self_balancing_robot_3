"""ROS stream for the ROS 2 adapter (issue #96): station/bridge/ros_stream.py, on the simulated LD19 of test_lidar.Rig,
plus one end-to-end run through the real bridge + sim over WebSocket."""
import asyncio
import json
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge import ros_stream as rs
from station.bridge.bridge import Bridge
from station.bridge.httpws import WsClient
from station.bridge.tests.test_bridge_sim import WEB, until, wait_for
from station.bridge.tests.test_lidar import Rig
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


class FakeBridge:
    """What RosStream reads from the Bridge."""

    def __init__(self, lidar, mapper):
        self.lidar, self.mapper = lidar, mapper
        self.tlm, self.tlm_time = None, -10.0
        self.link_state, self.estop, self.robot_estop = "connected", False, False
        self.lines = []

    def say(self, text):
        self.lines.append(text)


class Feed:
    """Rig + RosStream with one subscriber; telemetry goes to both the lidar and the stream, ticks at 25 Hz."""

    def __init__(self, tmp, subscribe=True, ws=None, **kw):
        self.rig = Rig(tmp, **kw)
        self.b = FakeBridge(self.rig.lidar, self.rig.mapper)
        self.ros = rs.RosStream(self.b, clock=lambda: self.rig.r.t)
        self.ws = ws or FakeWs()
        if subscribe:
            self.hello = self.ros.command(self.ws, {"t": "ros", "action": "subscribe"})
        lidar_on_tlm = self.rig.lidar.on_tlm

        def on_tlm(d, t):
            d = dict(d, speed_mms=0.0, turn_rate_dps=0.0, pitch_rate_dps=0.0, roll_deg=0.0, yaw_rate_dps=0.0,
                     mode=lp.MODE_STATION, status=lp.TLM_ARMED | lp.TLM_UPRIGHT, battery_v=12.1, radio_ok=0)
            lidar_on_tlm(d, t)
            self.b.tlm, self.b.tlm_time = d, t
            self.ros.on_tlm(d, t)
        self.rig.lidar.on_tlm = on_tlm
        self.next_tick = 0.0
        self.rig_mover = None

        def tick_mover(r, dt):
            if self.rig_mover:
                self.rig_mover(r, dt)
            if r.t >= self.next_tick:
                self.next_tick = r.t + 0.04
                self.ros.tick(r.t)
        self.rig.mover = tick_mover


def ranges(msg):
    return rs.decode_ranges(msg["r"])


def bin_angle(i, bins=rs.SCAN_BINS):
    return -math.pi + (i + 0.5) * 2 * math.pi / bins


class TestStream(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_encoding_round_trip(self):
        rr = [0, 1, 1234, 65535, 70000, -5]
        self.assertEqual(rs.decode_ranges(rs.encode_ranges(rr)), [0, 1, 1234, 65535, 65535, 0])

    def test_nothing_without_subscribers(self):
        f = Feed(self.tmp, subscribe=False)
        f.rig.run(1.0)
        self.assertEqual(f.ws.sent, [])
        self.assertEqual(f.ros.n["scans"], 0)          # no work at all: the bin pass is skipped
        self.assertEqual(f.ros.odom_rows, [])

    def test_hello(self):
        f = Feed(self.tmp, height_mm=208.0, yaw_deg=90.0)
        h = f.hello
        self.assertEqual(h["t"], "ros_hello")
        self.assertTrue(h["subscribed"])
        self.assertEqual(h["lidar"]["height_mm"], 208.0)
        self.assertEqual(h["lidar"]["yaw_deg"], 0.0)  # the mount yaw is applied in the bridge
        self.assertEqual(h["lidar"]["gate_deg"], 6.0)
        self.assertEqual(h["scan"]["bins"], rs.SCAN_BINS)
        self.assertAlmostEqual(h["scan"]["angle_increment"] * rs.SCAN_BINS, 2 * math.pi)
        self.assertEqual(h["odom"]["cols"][0], "ts")
        self.assertEqual(f.ros.command(f.ws, {"action": "unsubscribe"})["subscribed"], False)
        self.assertEqual(f.ros.clients, set())
        self.assertEqual(f.ros.command(f.ws, {"action": "bogus"})["t"], "error")

    def check_room(self, msg, x, y, th, room, min_ok=0.9):
        """Every bin with a return lies on a wall of the sim room (ray cast from the pose at the bin's angle)."""
        rr = ranges(msg)
        ok = n = 0
        for i, r in enumerate(rr):
            if not r:
                continue
            n += 1
            exp = room.raycast(x, y, math.radians(th) + bin_angle(i), 9000.0)
            if exp is not None and abs(r - exp) < max(40.0, 0.02 * exp):
                ok += 1
        self.assertGreater(n, 300)
        self.assertGreater(ok / n, min_ok, (ok, n))

    def test_scans_match_the_room_in_the_laser_frame(self):
        f = Feed(self.tmp)
        r = f.rig.r
        r.X, r.Y, r.TH = 500.0, 300.0, 90.0
        f.rig.run(1.5)
        scans = f.ws.msgs("ros_scan")
        self.assertGreaterEqual(len(scans), 12)                    # ~10 Hz
        self.assertEqual([m["seq"] for m in scans], list(range(1, len(scans) + 1)))
        m = scans[-1]
        self.assertEqual(len(ranges(m)), rs.SCAN_BINS)
        self.assertEqual(m["tilt_pk"], 0)
        self.assertLess(m["tilt"], 0.01)
        self.assertLess(m["t0"], m["ts"])
        self.assertAlmostEqual(m["ts"] - m["t0"], 0.1, delta=0.02)
        self.check_room(m, r.X, r.Y, r.TH, r.room)
        # straight ahead (bin around angle 0) = +Y in the world: wall y = 2000
        i0 = rs.SCAN_BINS // 2
        ahead = [v for v in ranges(m)[i0 - 2:i0 + 2] if v]
        self.assertTrue(ahead)
        self.assertAlmostEqual(min(ahead), 1700, delta=40)
        # the cage posts (mask 87-93 deg / 267-273 deg in the robot frame) stay empty
        for deg in (90.0, -90.0):
            i = int((math.radians(deg) + math.pi) / (2 * math.pi) * rs.SCAN_BINS)
            self.assertEqual(ranges(m)[i], 0, deg)

    def test_moving_robot_is_deskewed_to_the_scan_end(self):
        f = Feed(self.tmp)
        f.rig_mover = lambda r, dt: setattr(r, "X", r.X + 400.0 * dt)      # 400 mm/s toward the far wall
        f.rig.run(1.2)
        m = f.ws.msgs("ros_scan")[-1]
        x_end = f.rig.r.X - 400.0 * (f.rig.r.t - m["ts"])                   # the pose at the stamp
        self.check_room(m, x_end, 0.0, 0.0, f.rig.r.room)
        i0 = rs.SCAN_BINS // 2
        ahead = min(v for v in ranges(m)[i0 - 2:i0 + 2] if v)
        self.assertAlmostEqual(ahead, 4000 - x_end, delta=30)

    def test_turning_robot_is_deskewed(self):
        f = Feed(self.tmp)
        f.rig_mover = lambda r, dt: setattr(r, "TH", r.TH + 90.0 * dt)      # 90 deg/s
        f.rig.run(1.2)
        m = f.ws.msgs("ros_scan")[-1]
        th_end = f.rig.r.TH - 90.0 * (f.rig.r.t - m["ts"])
        self.check_room(m, 0.0, 0.0, th_end, f.rig.r.room, min_ok=0.85)

    def test_tilted_scans_are_flagged(self):
        f = Feed(self.tmp, pitch=lambda r: 8.0 if (r.t % 2.0) > 1.0 else 0.0)  # 8 deg lean every other second
        f.rig.run(4.0)
        scans = f.ws.msgs("ros_scan")
        tilted = [m for m in scans if m["tilt_pk"]]
        level = [m for m in scans if m["tilt"] < 0.01]
        self.assertTrue(tilted and level)
        for m in tilted:
            self.assertGreater(m["tilt"], 6.0)

    def test_odometry_rows_and_state(self):
        f = Feed(self.tmp, tlm_hz=100)
        f.rig_mover = lambda r, dt: setattr(r, "X", r.X + 200.0 * dt)
        f.rig.run(2.0)
        od = f.ws.msgs("ros_odom")
        rows = [r for m in od for r in m["rows"]]
        self.assertAlmostEqual(len(rows) / 2.0, 50, delta=4)               # 100 Hz telemetry -> <= 50 rows/s
        self.assertGreater(len(od), 40)                                     # batched per 25 Hz tick
        ts = [r[0] for r in rows]
        self.assertEqual(ts, sorted(ts))
        self.assertEqual(len(rows[0]), len(rs.ODOM_COLS))
        self.assertAlmostEqual(rows[-1][1], f.rig.r.X, delta=20)
        st = f.ws.msgs("ros_state")
        self.assertAlmostEqual(len(st), 4, delta=1)                         # 2 Hz
        s = st[-1]
        self.assertEqual((s["mode"], s["armed"], s["estop"], s["link"]), ("STATION", True, False, "connected"))
        self.assertEqual(s["battery_v"], 12.1)
        self.assertEqual(s["corr"], [0.0, 0.0, 0.0])
        self.assertIn("kbps", s["stats"])

    def test_correction_rides_along(self):
        f = Feed(self.tmp)
        f.rig.mapper.corr = [120.0, -40.0, 2.5]
        f.rig.lidar.settings["enabled"] = True
        f.rig.run(0.6)
        self.assertEqual(f.ws.msgs("ros_scan")[-1]["corr"], [120.0, -40.0, 2.5])

    def test_slow_client_skips_and_bandwidth(self):
        slow = FakeWs(backlog=rs.WS_SLOW_BYTES + 1)
        f = Feed(self.tmp, ws=slow)
        f.rig.run(1.0)
        self.assertEqual(slow.sent, [])
        self.assertGreater(f.ros.n["skipped_slow"], 5)
        f2 = Feed(self.tmp)
        f2.rig.run(5.5)
        total = sum(len(x) for x in f2.ws.sent)
        kbps = total / 5.5 / 1024.0
        self.assertLess(kbps, 25.0)                                        # fine over Wi-Fi
        self.assertLess(max(len(x) for x in f2.ws.sent), 2000)
        self.assertGreater(f2.ros.kbps, 5.0)                               # its own estimate (5 s window)


class TestBridgeEndToEnd(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_subscribe_over_websocket(self):
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
            other = await WsClient.connect("127.0.0.1", port)               # a phone: never gets ROS messages
            ws = await WsClient.connect("127.0.0.1", port)
            await until(lambda: b.tlm is not None)
            ws.send(json.dumps({"t": "ros", "action": "subscribe"}))
            h = await wait_for(ws, lambda m: m["t"] == "ros_hello")
            self.assertEqual(h["scan"]["bins"], rs.SCAN_BINS)
            b.handle_command({"t": "set_mode", "mode": lp.MODE_STATION, "arm": True}, "test")
            od = await wait_for(ws, lambda m: m["t"] == "ros_odom")
            self.assertTrue(od["rows"])
            st = await wait_for(ws, lambda m: m["t"] == "ros_state" and m["armed"], 6.0)
            self.assertEqual(st["mode"], "STATION")
            sc = await wait_for(ws, lambda m: m["t"] == "ros_scan" and m["tilt"] < 6.0, 8.0)
            self.assertGreater(sum(1 for r in rs.decode_ranges(sc["r"]) if r), 250)
            self.assertLessEqual(sc["ts"], sc["now"])
            # the phone never saw any of it
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                try:
                    m = json.loads(await asyncio.wait_for(other.recv(), 0.2))
                except asyncio.TimeoutError:
                    continue
                self.assertFalse(m["t"].startswith("ros_"), m["t"])
            ws.close()
            await until(lambda: not b.ros.clients, 3.0)
            self.assertEqual(b.ros.clients, set())
        finally:
            for t in tasks + [phys]:
                t.cancel()
            b.lidar.reader.close()
            hs.close()
            srv.close()
            lsrv.close()


if __name__ == "__main__":
    unittest.main()
