"""The Android app's native stop-on-background sequence (issue #32), against the sim and a real bridge.

The app (android/src/.../AppStop.java) cannot be paused here, so this checks the other half: the HTTP sequence it
sends when the app is left (cmd_vel 0, plan stop, read /api/state, set_mode BALANCE if GOTO/EXPLORE) really stops a
running Explore / Go to, keeps the robot balancing, and never E-stops, idles or arms. When a JDK is installed the
app's own Java code is compiled and run against the bridge; the same sequence is replayed from Python otherwise.
Run: python -m unittest discover -s station -t ."""
import asyncio
import glob
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import unittest
import urllib.request

from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import HttpServer
from station.sim.sim import SimRobot, SimServer

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")
JAVA_SRC = os.path.join(os.path.dirname(__file__), "..", "..", "android", "src", "com", "sbr3", "station", "AppStop.java")


def find_jdk():
    for pat in (r"C:\Program Files\Microsoft\jdk-*", r"C:\Program Files\Android\jdk", r"C:\Program Files\Java\jdk*"):
        for d in glob.glob(pat):
            if os.path.exists(os.path.join(d, "bin", "javac.exe")):
                return d
    j = shutil.which("javac")
    return os.path.dirname(os.path.dirname(os.path.realpath(j))) if j else None


RUNNER = """package com.sbr3.station;
public class RunStop { public static void main(String[] a) { System.out.println(AppStop.stop(a[0])); } }
"""


class TestAppStop(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_constants_match_the_bridge(self):
        with open(JAVA_SRC, encoding="utf-8") as f:
            src = f.read()
        mode = dict(re.findall(r"MODE_([A-Z]+) = (\d+)", src))
        for i, name in enumerate(lp.MODE_NAMES):
            self.assertEqual(int(mode[name]), i, name)
        self.assertIn("TLM_ARMED = 0x%02X" % lp.TLM_ARMED, src)
        self.assertIn("TLM_UPRIGHT = 0x%02X" % lp.TLM_UPRIGHT, src)
        self.assertIn('{\\"t\\":\\"cmd_vel\\",\\"speed\\":0,\\"turn\\":0}', src)
        self.assertIn('{\\"t\\":\\"plan\\",\\"action\\":\\"stop\\"}', src)

    def test_stop_sequence_on_sim(self):
        asyncio.run(self._run())

    async def _run(self):
        robot = SimRobot(seed=3)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % srv.sockets[0].getsockname()[1], WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True, map_dir=os.path.join(self.tmp, "maps"),
                   preset_dir=os.path.join(self.tmp, "presets"))
        routes = [("GET", "/api/state", b.api_state), ("*", "/api/cmd", b.api_cmd), ("GET", "/api/status", b.api_status)]
        hs = await HttpServer(WEB, routes, "/ws", b.ws_handler).start("127.0.0.1", 0)
        port = hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        loop = asyncio.get_running_loop()
        cmds = []

        def post(m):
            cmds.append(m)
            req = urllib.request.Request("http://127.0.0.1:%d/api/cmd" % port, data=json.dumps(m).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read())

        def get(path):
            with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=3) as r:
                return json.loads(r.read())

        async def apost(m):
            return await loop.run_in_executor(None, post, m)

        async def until(pred, what, timeout=8.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if pred():
                    return
                await asyncio.sleep(0.03)
            raise AssertionError("timeout: " + what)

        def python_stop():
            post({"t": "cmd_vel", "speed": 0, "turn": 0})
            post({"t": "plan", "action": "stop"})
            tlm = get("/api/state?max=0&r=2000000000")["tlm"]
            if tlm["mode"] in (lp.MODE_GOTO, lp.MODE_EXPLORE) and tlm["status"] & lp.TLM_ARMED and tlm["status"] & lp.TLM_UPRIGHT:
                post({"t": "set_mode", "mode": lp.MODE_BALANCE})
            post({"t": "cmd_vel", "speed": 0, "turn": 0})

        jdk = find_jdk()
        java_dir = None
        if jdk:
            java_dir = tempfile.mkdtemp(dir=self.tmp)
            os.makedirs(os.path.join(java_dir, "src"))
            runner = os.path.join(java_dir, "RunStop.java")
            with open(runner, "w") as f:
                f.write(RUNNER)
            r = subprocess.run([os.path.join(jdk, "bin", "javac"), "-encoding", "UTF-8", "--release", "8", "-Xlint:-options",
                                "-d", java_dir, JAVA_SRC, runner], capture_output=True, text=True)
            if r.returncode != 0:
                java_dir = None
                print("javac failed, replaying the sequence from Python:", r.stderr[:300])

        def app_stop():
            if not java_dir:
                python_stop()
                return "python"
            r = subprocess.run([os.path.join(jdk, "bin", "java"), "-cp", java_dir, "com.sbr3.station.RunStop",
                                "127.0.0.1:%d" % port], capture_output=True, text=True, timeout=20)
            self.assertEqual(r.returncode, 0, r.stderr)
            return r.stdout.strip()

        try:
            await until(lambda: b.tlm is not None, "telemetry")
            # arm (no remote: the explicit arm request + a mode) and stand up, like the app's Arm step
            await apost({"t": "set_mode", "mode": lp.MODE_STATION, "arm": True})
            await until(lambda: b.tlm["mode"] == lp.MODE_STATION and b.tlm["status"] & lp.TLM_UPRIGHT, "STATION upright")

            # ---- Explore from the phone, then the app is left
            await apost({"t": "plan", "action": "explore"})
            await until(lambda: b.planner.mode == "explore" and b.tlm["mode"] == lp.MODE_EXPLORE, "explore running")
            await asyncio.sleep(0.5)
            out = await loop.run_in_executor(None, app_stop)
            await until(lambda: b.planner.mode == "idle", "planner stopped")
            await until(lambda: b.tlm["mode"] == lp.MODE_BALANCE, "robot back in BALANCE")
            self.assertTrue(b.tlm["status"] & lp.TLM_ARMED, "still armed")
            self.assertFalse(b.estop, "never an E-stop")
            await asyncio.sleep(0.6)
            self.assertEqual(b.planner.mode, "idle", "planner stays stopped")
            self.assertEqual(b.tlm["mode"], lp.MODE_BALANCE, "mode stays BALANCE (" + out + ")")
            self.assertFalse(b.cmd_active, "no cmd_vel forwarding")
            self.assertLess(abs(b.tlm["cmd_speed_mms"]), 1)
            if java_dir:
                self.assertIn("set_mode", out, "the app's Java code sent the BALANCE step: " + out)

            # ---- Go to
            await apost({"t": "plan", "action": "goto", "points": [[1500, 0]]})
            await until(lambda: b.planner.mode == "goto" and b.tlm["mode"] == lp.MODE_GOTO, "goto running")
            await loop.run_in_executor(None, app_stop)
            await until(lambda: b.planner.mode == "idle" and b.tlm["mode"] == lp.MODE_BALANCE, "goto stopped, BALANCE")

            # ---- a plain waypoint goto (no planner): back to BALANCE as well
            await apost({"t": "goto", "points": [[600, 0], [600, 400]], "replace": True, "go": True})
            await until(lambda: b.tlm["mode"] == lp.MODE_GOTO, "goto mode")
            await loop.run_in_executor(None, app_stop)
            await until(lambda: b.tlm["mode"] == lp.MODE_BALANCE, "plain goto stopped, BALANCE")

            # ---- driving in STATION: the zero cmd_vel is all it takes, the robot keeps standing in STATION
            await apost({"t": "set_mode", "mode": lp.MODE_STATION})
            await until(lambda: b.tlm["mode"] == lp.MODE_STATION, "STATION")
            for _ in range(5):
                await apost({"t": "cmd_vel", "speed": 300, "turn": 0})
                await asyncio.sleep(0.05)
            self.assertTrue(b.cmd_active)
            await loop.run_in_executor(None, app_stop)
            await until(lambda: b.cmd == (0, 0), "cmd_vel zeroed")
            self.assertEqual(b.tlm["mode"], lp.MODE_STATION)
            self.assertTrue(b.tlm["status"] & lp.TLM_ARMED)

            # ---- disarmed robot: never armed by the stop
            await apost({"t": "set_mode", "mode": lp.MODE_IDLE})
            await until(lambda: not b.tlm["status"] & lp.TLM_ARMED, "disarmed")
            await loop.run_in_executor(None, app_stop)
            await asyncio.sleep(0.4)
            self.assertEqual(b.tlm["mode"], lp.MODE_IDLE)
            self.assertFalse(b.tlm["status"] & lp.TLM_ARMED, "the stop did not arm the robot")
            # the race the old rule had: a late / stray set_mode BALANCE (what AppStop sends) never arms
            await apost({"t": "set_mode", "mode": lp.MODE_BALANCE})
            await asyncio.sleep(0.4)
            self.assertEqual(b.tlm["mode"], lp.MODE_IDLE)
            self.assertFalse(b.tlm["status"] & lp.TLM_ARMED, "a plain set_mode did not arm the robot")
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
