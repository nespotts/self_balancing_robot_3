"""Android app inputs (web/js/native.js) in headless Chrome with a fake window.SBR3Native, against sim + bridge.

Checks the safety contract: tilt cmd_vel flows only while the dead-man is held, a zero goes out at once on
release, and driving stops within 300 ms when the tilt sensor goes quiet or the page disconnects. Gamepad stick
drives, B = E-stop. Plain browsers get none of it. Also the app's start screen (android/assets/find.html). Skipped when no Chrome / Edge is installed.
Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import os
import shutil
import socket
import subprocess
import tempfile
import time
import unittest
import urllib.parse
import urllib.request

from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import WsClient
from station.sim.sim import SimRobot, SimServer
from station.tools.screenshots import BROWSERS, Cdp

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")
BROWSER = next((b for b in BROWSERS if os.path.exists(b)), None)

FAKE_NATIVE = r"""
window.__calls = []; window.__sent = [];
const _send = WebSocket.prototype.send;
WebSocket.prototype.send = function (d) {
  try { const m = JSON.parse(d); if (m.t !== "hi") window.__sent.push([performance.now(), m]); } catch (e) {}
  return _send.call(this, d);
};
window.__tilt = { pitch: -35, roll: 2, on: true, rot: 0 };
window.SBR3Native = {
  appInfo: () => JSON.stringify({ version: "test", tilt: true, orientation: "auto" }),
  tiltStart: (hz) => __calls.push(["tiltStart", hz]), tiltStop: () => __calls.push(["tiltStop"]),
  vibrate: (p) => __calls.push(["vibrate", p]), keepAwake: (on) => __calls.push(["keepAwake", on]),
  gamepads: () => '{"list":[]}', setOrientation: (m) => __calls.push(["orient", m]), openFinder: () => {},
  lastAddress: () => "", gestureRects: (j) => __calls.push(["rects", j]),
};
setInterval(() => {
  if (__tilt.on && window.__sbr3) __sbr3.emit("tilt", JSON.stringify({ pitch: __tilt.pitch, roll: __tilt.roll, rot: __tilt.rot, t: 0 }));
}, 33);
"""

FIND = os.path.join(os.path.dirname(__file__), "..", "..", "android", "assets", "find.html")
FAKE_FIND = r"""
window.__calls = [];
window.SBR3Native = {
  appInfo: () => "{}", lastAddress: () => "192.168.0.209:8080",
  findStart: () => {
    __calls.push(["findStart"]);
    setTimeout(() => {
      const R = { addr: "192.168.0.209:8080", kind: "robot", alive: true }, S = { addr: "192.168.0.149:8080", kind: "sim", alive: true };
      if (__scenario === "found") __sbr3.emit("findDone", JSON.stringify({ hits: [R, S], auto: R.addr }));
      else if (__scenario === "two") __sbr3.emit("findDone", JSON.stringify({ hits: [R, { ...R, addr: "192.168.0.50:8080" }, S], auto: null }));
      else if (__scenario === "sim") __sbr3.emit("findDone", JSON.stringify({ hits: [S], auto: null }));
      else { __sbr3.emit("find", '{"wifi":true,"subnet":"192.168.0.x","total":3,"done":3}'); __sbr3.emit("findDone", "{}"); }
    }, 100);
  },
  findStop: () => __calls.push(["findStop"]), probe: (a) => __calls.push(["probe", a]),
  connect: (a) => __calls.push(["connect", a]),
};
"""


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@unittest.skipIf(BROWSER is None or os.environ.get("SBR3_FAST"), "no Chrome / Edge, or SBR3_FAST=1")
class TestNativeDrive(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_tilt_and_gamepad(self):
        asyncio.run(self._run())

    async def ev(self, js):
        r = await self.cdp.call("Runtime.evaluate", expression=js, returnByValue=True, awaitPromise=True)
        if "exceptionDetails" in r:
            raise AssertionError("JS error: %s" % r["exceptionDetails"])
        return r["result"].get("value")

    async def until(self, js, timeout=10.0, what=""):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if await self.ev(js):
                return
            await asyncio.sleep(0.05)
        raise AssertionError("timeout: " + (what or js))

    async def touch(self, kind, x=0, y=0):
        pts = [] if kind == "touchEnd" else [{"x": x, "y": y}]
        await self.cdp.call("Input.dispatchTouchEvent", type=kind, touchPoints=pts)

    async def cmds(self, since):
        """cmd_vel sent by the page after page-time `since` (ms): [[t, speed, turn], ...]"""
        return await self.ev("__sent.filter(([t, m]) => m.t === 'cmd_vel' && t > %f).map(([t, m]) => [t, m.speed, m.turn])" % since)

    async def wait_active(self, b, timeout=3.0):
        """Poll for the bridge's cmd_active: it drops after any 300 ms gap between cmd_vel frames, and the
        page's 100 ms timer can be delayed while headless Chrome is busy, so never assert the instant value (#28)."""
        end = time.monotonic() + timeout
        while not b.cmd_active and time.monotonic() < end:
            await asyncio.sleep(0.02)
        self.assertTrue(b.cmd_active, "bridge never saw cmd_vel")

    async def load(self, url):
        await self.cdp.call("Page.navigate", url="about:blank")
        await asyncio.sleep(0.3)
        await self.cdp.call("Page.navigate", url=url)
        await self.until("!!(window.sbr3 && sbr3.state.connected && sbr3.state.tlm && sbr3.state.hello)", what="page connected")

    async def station(self, b):
        await self.ev("sbr3.send({t: 'set_mode', mode: %d, arm: true})" % lp.MODE_STATION)
        await self.until("sbr3.state.tlm.mode === %d && (sbr3.state.tlm.status & %d)" % (lp.MODE_STATION, lp.TLM_ARMED),
                         what="STATION + armed")
        await self.until("!document.getElementById('joy').classList.contains('locked')", what="joystick unlocked")

    async def find_page(self):
        """The app's start screen (android/assets/find.html): auto-connect, not found + manual entry, load error."""
        url = "file:///" + os.path.abspath(FIND).replace("\\", "/")

        async def open_with(scenario, hash_=""):
            sid = (await self.cdp.call("Page.addScriptToEvaluateOnNewDocument",
                                       source="window.__scenario = %s;" % json.dumps(scenario) + FAKE_FIND))["identifier"]
            await self.cdp.call("Page.navigate", url="about:blank")
            await asyncio.sleep(0.2)
            await self.cdp.call("Page.navigate", url=url + hash_)
            await self.until("document.readyState === 'complete' && !!window.__calls")
            await self.cdp.call("Page.removeScriptToEvaluateOnNewDocument", identifier=sid)

        title = "document.getElementById('title').textContent"
        await open_with("found")
        await self.until("__calls.some(c => c[0] === 'connect' && c[1] === '192.168.0.209:8080')", what="auto-connect")
        self.assertIn("Robot", await self.ev(title))

        # issue #27: two robots (+ a sim) -> a list, robots first, nothing opened until tapped
        hits = "[...document.querySelectorAll('#hits button')].map(b => b.textContent)"
        await open_with("two")
        await self.until(title + " === 'Pick a robot'")
        self.assertEqual(await self.ev(hits), ["Robot · 192.168.0.209last used", "Robot · 192.168.0.50", "Simulator · 192.168.0.149"])
        await asyncio.sleep(0.5)
        self.assertFalse(await self.ev("__calls.some(c => c[0] === 'connect')"), "no auto-connect with two robots")
        await self.ev("document.querySelectorAll('#hits button')[2].click()")
        await self.until("__calls.some(c => c[0] === 'connect' && c[1] === '192.168.0.149:8080')", what="tap the sim")

        # only a simulator answered: ask, don't open it
        await open_with("sim")
        await self.until(title + " === 'Only a simulator found'")
        self.assertEqual(await self.ev(hits), ["Simulator · 192.168.0.149"])
        self.assertFalse(await self.ev("__calls.some(c => c[0] === 'connect')"), "a lone sim is not opened by itself")

        await open_with("none")
        await self.until(title + " === 'Robot not found'")
        self.assertTrue(await self.ev("!document.getElementById('tips').hidden && !document.getElementById('retry').closest('.row').hidden"))
        await self.ev("document.getElementById('show-manual').click(); document.getElementById('addr').value = '192.168.0.50';"
                      "document.getElementById('go').click()")
        self.assertTrue(await self.ev("__calls.some(c => c[0] === 'probe' && c[1] === '192.168.0.50')"))
        await self.ev("__sbr3.emit('probe', JSON.stringify({addr: '192.168.0.50:8080', ok: false, error: 'No answer from 192.168.0.50:8080'}))")
        self.assertIn("No answer", await self.ev("document.getElementById('err').textContent"))
        self.assertFalse(await self.ev("__calls.some(c => c[0] === 'connect')"))
        await self.ev("document.getElementById('go').click();"
                      "__sbr3.emit('probe', JSON.stringify({addr: '192.168.0.50:8080', ok: true, status: {}}))")
        await self.until("__calls.some(c => c[0] === 'connect' && c[1] === '192.168.0.50:8080')", what="manual connect")

        await open_with("none", "#error=net%3A%3AERR_CONNECTION_REFUSED&addr=192.168.0.209%3A8080")
        await self.until(title + " === \"Couldn't open the robot\"")
        self.assertFalse(await self.ev("document.getElementById('manual').hidden"))
        self.assertEqual(await self.ev("document.getElementById('addr').value"), "192.168.0.209")
        self.assertFalse(await self.ev("__calls.some(c => c[0] === 'findStart')"), "no auto-scan after a load error")

    async def _run(self):
        robot = SimRobot(seed=2)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % srv.sockets[0].getsockname()[1], WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True)
        hs = await b.build_http().start("127.0.0.1", 0)     # real server: Chrome sends Origin on the WebSocket
        url = "http://127.0.0.1:%d/#drive" % hs.sockets[0].getsockname()[1]
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        dport = free_port()
        prof = tempfile.mkdtemp(prefix="sbr3-native-")
        proc = subprocess.Popen([BROWSER, "--headless=new", "--disable-gpu", "--no-first-run", "--hide-scrollbars",
                                 "--remote-debugging-port=%d" % dport, "--user-data-dir=" + prof, "about:blank"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            page = None
            for _ in range(75):
                try:
                    lst = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/json/list" % dport, timeout=1).read())
                    page = [t for t in lst if t["type"] == "page"][0]
                    break
                except Exception:
                    await asyncio.sleep(0.2)
            self.assertIsNotNone(page, "browser did not start")
            self.cdp = Cdp(await WsClient.connect("127.0.0.1", dport, urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path))
            await self.cdp.call("Page.enable")
            await self.cdp.call("Emulation.setDeviceMetricsOverride", width=412, height=915, deviceScaleFactor=1, mobile=True)
            await self.cdp.call("Emulation.setTouchEmulationEnabled", enabled=True)

            # ---- a plain browser: nothing changes
            await self.load(url)
            self.assertFalse(await self.ev("document.body.classList.contains('native') || !!document.getElementById('input-seg')"))

            # ---- the app: fake SBR3Native before any page script
            sid = (await self.cdp.call("Page.addScriptToEvaluateOnNewDocument", source=FAKE_NATIVE))["identifier"]
            await self.load(url)
            self.assertTrue(await self.ev("document.body.classList.contains('native')"))
            self.assertTrue(await self.ev("!document.getElementById('input-seg').hidden"))
            await self.ev("document.querySelector('#input-seg [data-v=tilt]').click()")
            self.assertTrue(await self.ev("__calls.some(c => c[0] === 'tiltStart')"))
            self.assertTrue(await self.ev("document.getElementById('joy').classList.contains('tilt')"))
            await self.station(b)
            x, y = (await self.ev("(() => { const r = document.getElementById('joy').getBoundingClientRect();"
                                  " return [r.left + r.width / 2, r.top + r.height / 2]; })()"))

            # not held: tilting does nothing
            t0 = await self.ev("performance.now()")
            await self.ev("__tilt.pitch = -20")
            await asyncio.sleep(1.0)
            self.assertEqual(await self.cmds(t0), [], "cmd_vel without the dead-man")

            # held + tilted forward 15 deg (from the pitch at the press): forward cmd_vel at ~10 Hz
            await self.ev("__tilt.pitch = -35")
            await asyncio.sleep(0.1)
            t0 = await self.ev("performance.now()")
            await self.touch("touchStart", x, y)
            await self.ev("__tilt.pitch = -20")
            await asyncio.sleep(1.0)
            c = await self.cmds(t0)
            self.assertGreaterEqual(len(c), 8, c)
            self.assertGreater(c[-1][1], 50, "forward speed")
            self.assertEqual(c[-1][2], 0, "no turn with roll inside the dead-band")
            await self.wait_active(b)
            self.assertFalse(await self.ev("__calls.some(c => c[0] === 'vibrate')"), "no haptics")
            # roll right -> turn right (negative = clockwise)
            await self.ev("__tilt.roll = 20")
            await asyncio.sleep(0.4)
            c = await self.cmds(t0)
            self.assertLess(c[-1][2], -20, "right turn")

            # release: an immediate zero, nothing after it
            # (latency is measured from the page's own touchend event, not from before the CDP round trip, which
            # can itself take 100+ ms while the machine is busy - #66)
            await self.ev("window.__rel = null; document.addEventListener('touchend', () => { window.__rel = performance.now(); }, "
                          "{capture: true, once: true})")
            rel0 = await self.ev("performance.now()")
            await self.touch("touchEnd")
            await asyncio.sleep(0.6)
            c = await self.cmds(rel0)
            rel = await self.ev("window.__rel")
            self.assertIsNotNone(rel, "the page saw the touch end")
            # rel0 was taken before the CDP round trip: while the touchEnd waited behind a busy browser the held pad
            # legitimately kept sending its ~10 Hz cmd_vel. Those leading frames must come before the page's own touchend;
            # from the first zero on, everything below is checked as before (#98)
            while len(c) > 1 and c[0][1:] != [0, 0]:
                self.assertLessEqual(c[0][0], rel, "drive frame after the touch end")
                c = c[1:]
            self.assertTrue(c, "a stop after release")
            self.assertEqual(c[0][1:], [0, 0])
            self.assertLess(c[0][0] - rel, 150, "zero within 150 ms of release")
            self.assertTrue(all(s == 0 and w == 0 for _, s, w in c), c)
            self.assertFalse(b.cmd_active, "bridge stopped forwarding")
            self.assertFalse(await self.ev("document.getElementById('joy').classList.contains('held')"))

            # tilt sensor goes quiet while held -> stop within 300 ms (+ one 100 ms tick)
            await self.ev("__tilt.roll = 2; __tilt.pitch = -35")
            await asyncio.sleep(0.1)
            await self.touch("touchStart", x, y)
            await self.ev("__tilt.pitch = -20")
            await asyncio.sleep(0.5)
            quiet = await self.ev("__tilt.on = false, performance.now()")
            await asyncio.sleep(0.8)
            c = await self.cmds(quiet)
            moving = [t for t, s, w in c if s or w]
            self.assertTrue(not moving or moving[-1] - quiet <= 400, c)
            self.assertEqual(c[-1][1:], [0, 0], c)
            self.assertFalse(await self.ev("document.getElementById('joy').classList.contains('held')"))
            await self.touch("touchEnd")
            await self.ev("__tilt.on = true")

            # the display turns 180 deg (ROTATION_90 -> 270) while held: samples are in the other frame -> let go (issue #31)
            await self.ev("__tilt.roll = 2; __tilt.pitch = -35")
            await asyncio.sleep(0.1)
            await self.touch("touchStart", x, y)
            self.assertTrue(await self.ev("document.getElementById('joy').classList.contains('held')"))
            t0 = await self.ev("performance.now()")
            await self.ev("__tilt.rot = 2")
            await asyncio.sleep(0.3)
            self.assertFalse(await self.ev("document.getElementById('joy').classList.contains('held')"), "released on rotation change")
            c = await self.cmds(t0)
            self.assertTrue(all(s == 0 and w == 0 for _, s, w in c), c)
            await self.touch("touchEnd")
            # the rotation event alone (no sample yet) also lets go; a new press in the new frame works
            await self.touch("touchStart", x, y)
            self.assertTrue(await self.ev("document.getElementById('joy').classList.contains('held')"), "new press in the new frame")
            await self.ev("__sbr3.emit('rotation', JSON.stringify({rot: 0}))")
            await asyncio.sleep(0.1)
            self.assertFalse(await self.ev("document.getElementById('joy').classList.contains('held')"), "released on rotation event")
            await self.touch("touchEnd")
            await self.ev("__tilt.rot = 0")
            # the pad reports its rectangle so the Back swipe does not start there
            rects = await self.ev("(__calls.filter(c => c[0] === 'rects').pop() || [])[1] || \"\"")
            self.assertTrue(rects.startswith("[[") and rects.count(",") == 3, rects)

            # page goes away while held (app killed / Wi-Fi gone) -> bridge stops within 300 ms
            await asyncio.sleep(0.2)
            await self.touch("touchStart", x, y)
            await self.ev("__tilt.pitch = -20")
            await asyncio.sleep(0.5)
            await self.wait_active(b)
            await self.cdp.call("Page.navigate", url="about:blank")
            # Page.navigate returns before the old page is torn down, and on a busy machine the old page keeps sending
            # cmd_vel for a long while after that: "gone" is the moment the new document is really there, not the
            # moment of the call (the suite flaked on it, #98).
            await self.until("location.href === 'about:blank'", what="old page gone")
            gone = time.monotonic()            # same clock as the bridge (time.monotonic)
            moving = lambda: b.cmd_active and b.cmd != (0, 0)
            # The bridge's 40 ms tick runs on this very event loop, so a stall of the loop (this process starved of CPU)
            # delays its stop by as much: measure the longest stall of our own 5 ms poll and allow exactly that.
            stall, prev = 0.0, time.monotonic()
            while moving() and time.monotonic() - gone < 2:
                await asyncio.sleep(0.005)
                now = time.monotonic()
                stall, prev = max(stall, now - prev - 0.005), now
            stopped = time.monotonic()
            # Whichever comes first: the page's pagehide zero, the socket closing, or the bridge's 300 ms timeout
            # after the last cmd_vel it got (+ one 40 ms bridge tick). From the moment the page goes, one 100 ms
            # page tick may already be on its way.
            self.assertLess(stopped - b.cmd_time, 0.3 + 0.04 + 0.03 + stall, "bridge kept forwarding after its timeout")
            self.assertLess(stopped - gone, 0.1 + 0.3 + 0.04 + 0.03 + stall, "bridge still forwarding cmd_vel")
            await self.touch("touchEnd")

            # ---- gamepad: stick drives in STATION, goes stale -> stop, B = E-stop
            await self.load(url)
            await self.station(b)
            t0 = await self.ev("performance.now()")
            for _ in range(10):
                await self.ev("__sbr3.emit('gamepad', JSON.stringify({lx: 0, ly: -0.8, rx: 0, ry: 0, hx: 0, hy: 0, lt: 0, rt: 0}))")
                await asyncio.sleep(0.1)
            c = await self.cmds(t0)
            self.assertGreaterEqual(len(c), 7, c)
            # (not c[-1]: this emit loop runs through CDP round trips; when they stall past the stick's staleness
            # limit the last frame is already the stale zero - the forward drive is what is checked, #98)
            self.assertGreater(max(s for _, s, _ in c), 50, "gamepad forward")
            last = await self.ev("performance.now()")
            await asyncio.sleep(0.8)
            c = await self.cmds(last)
            moving = [t for t, s, w in c if s or w]
            self.assertTrue(not moving or moving[-1] - last <= 500, c)
            self.assertEqual(c[-1][1:], [0, 0], c)
            await self.ev("__sbr3.emit('gamepadButton', JSON.stringify({name: 'B', down: true}))")
            await self.until("__sent.some(([t, m]) => m.t === 'estop')", what="estop sent")
            end = time.monotonic() + 2
            while not b.estop and time.monotonic() < end:
                await asyncio.sleep(0.02)
            self.assertTrue(b.estop)
            await self.cdp.call("Page.removeScriptToEvaluateOnNewDocument", identifier=sid)
            await self.find_page()
        finally:
            if getattr(self, "cdp", None):
                self.cdp.ws.close()
            proc.terminate()
            try:
                proc.wait(5)
            except Exception:
                proc.kill()
            shutil.rmtree(prof, ignore_errors=True)
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            hs.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
