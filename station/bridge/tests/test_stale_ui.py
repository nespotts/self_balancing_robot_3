"""Stale data and no-link commands in the web app (issue #48), in headless Chrome at phone width against sim + bridge.

When the robot goes silent (the serial port is released, as for flashing) the Drive readouts grey out with a
"no data" badge, the mode tiles stop claiming a mode and cannot be tapped, the Sensors cards say no data, and
a command sent anyway comes back as a visible error instead of pretending. GOTO / EXPLORE need a second tap.
Skipped when no Chrome / Edge is installed. SBR3_SAVE_SHOTS=1 also writes the screenshots to station/docs/screenshots/.
Run: python -m unittest discover -s station -t ."""
import asyncio
import base64
import json
import os
import shutil
import subprocess
import tempfile
import time
import unittest
import urllib.parse
import urllib.request

from station.bridge import link_protocol as lp
from station.bridge.httpws import WsClient
from station.bridge.tests.test_hardening import Rig
from station.bridge.tests.test_native_drive import free_port
from station.tools.screenshots import BROWSERS, Cdp

LABEL = {"GOTO": "Go to", "EXPLORE": "Explore"}
BROWSER = next((b for b in BROWSERS if os.path.exists(b)), None)
SHOTS = os.path.join(os.path.dirname(__file__), "..", "..", "docs", "screenshots")
HOOK = r"""
window.__sent = [];
const _send = WebSocket.prototype.send;
WebSocket.prototype.send = function (d) {
  try { const m = JSON.parse(d); if (m.t !== "hi" && m.t !== "cmd_vel") window.__sent.push(m); } catch (e) {}
  return _send.call(this, d);
};
"""


@unittest.skipIf(BROWSER is None or os.environ.get("SBR3_FAST"), "no Chrome / Edge, or SBR3_FAST=1")
class TestStaleUi(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_stale_readouts_and_no_link_errors(self):
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

    async def shot(self, name):
        if not os.environ.get("SBR3_SAVE_SHOTS"):
            return
        r = await self.cdp.call("Page.captureScreenshot", format="png")
        os.makedirs(SHOTS, exist_ok=True)
        with open(os.path.join(SHOTS, name), "wb") as f:
            f.write(base64.b64decode(r["data"]))

    async def _run(self):
        rig = await Rig(self.tmp).start()
        dport, prof = free_port(), tempfile.mkdtemp(prefix="sbr3-stale-")
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
            await self.cdp.call("Emulation.setDeviceMetricsOverride", width=412, height=915, deviceScaleFactor=2, mobile=True)
            await self.cdp.call("Page.addScriptToEvaluateOnNewDocument", source=HOOK)
            await self.cdp.call("Page.navigate", url="http://127.0.0.1:%d/#drive" % rig.port)
            await self.until("!!(window.sbr3 && sbr3.state.connected && sbr3.state.tlm && sbr3.state.hello)", what="page connected")
            tiles = "document.querySelector('#view-drive .tiles')"
            badge = "getComputedStyle(%s.querySelector('.nodata')).display" % tiles
            modes = "[...document.querySelectorAll('#modes button')]"

            # live: normal readouts, tappable mode tiles, the current mode lit
            await self.until("!%s.classList.contains('stale')" % tiles, what="live readouts")
            self.assertEqual(await self.ev(badge), "none")
            self.assertTrue(await self.ev("%s.every(b => !b.disabled)" % modes))
            self.assertEqual(await self.ev("%s.filter(b => b.classList.contains('on')).map(b => b.dataset.name)" % modes), ["IDLE"])

            # arm (no RC remote: the explicit arm step), then GOTO / EXPLORE need two taps
            await self.ev("sbr3.send({t: 'set_mode', mode: %d, arm: true})" % lp.MODE_STATION)
            await self.until("sbr3.state.tlm.mode === %d" % lp.MODE_STATION, what="STATION")
            await self.ev("__sent.length = 0")
            tap = "document.querySelector('#modes [data-name=%s]')"
            for name, mode in (("GOTO", lp.MODE_GOTO), ("EXPLORE", lp.MODE_EXPLORE)):
                await self.ev(tap % name + ".click()")
                self.assertEqual(await self.ev("__sent.filter(m => m.t === 'set_mode').length"), 0, name + " went on one tap")
                self.assertTrue(await self.ev(tap % name + ".classList.contains('confirm')"))
                self.assertEqual(await self.ev(tap % name + ".textContent"), "Tap again")
                await self.ev(tap % name + ".click()")
                self.assertEqual(await self.ev("__sent.filter(m => m.t === 'set_mode').map(m => m.mode)"), [mode])
                self.assertEqual(await self.ev(tap % name + ".textContent"), LABEL[name])
                await self.ev("__sent.length = 0")
            await self.ev(tap % "GOTO" + ".click()")                       # armed, then left alone: it forgets
            await asyncio.sleep(3.3)
            self.assertEqual(await self.ev(tap % "GOTO" + ".textContent"), "Go to")
            self.assertEqual(await self.ev("__sent.filter(m => m.t === 'set_mode').length"), 0)
            # one tap modes (IDLE / STATION ...) still go at once
            await self.ev(tap % "IDLE" + ".click()")
            self.assertEqual(await self.ev("__sent.filter(m => m.t === 'set_mode').map(m => m.mode)"), [lp.MODE_IDLE])
            await self.until("sbr3.state.tlm.mode === %d" % lp.MODE_IDLE, what="IDLE")

            # the robot goes silent (serial port released, like flashing)
            pid = next(p["id"] for p in rig.b.params.values() if p["name"] == "bal.kp")
            await self.shot("phone-live-drive.png")
            rig.b.released_api = True
            await self.until("%s.classList.contains('stale')" % tiles, 6, "readouts stale within ~1 s")
            self.assertNotEqual(await self.ev(badge), "none")
            self.assertEqual(await self.ev("document.querySelector('#joy-lock-text').textContent"), "Robot not responding")
            self.assertTrue(await self.ev("%s.every(b => b.disabled)" % modes))
            self.assertEqual(await self.ev("%s.filter(b => b.classList.contains('on')).length" % modes), 0)
            self.assertLess(await self.ev("parseFloat(getComputedStyle(%s.querySelector('b')).opacity)" % tiles), 1)
            await self.shot("phone-stale-drive.png")

            # sensors: the cards say so too
            await self.ev("sbr3.show('sensors')")
            await self.until("document.querySelector('.sens-card.stale') && getComputedStyle("
                             "document.querySelector('.sens-card .nodata')).display !== 'none'", 4, "sensor cards stale")
            await asyncio.sleep(0.4)
            await self.shot("phone-stale-sensors.png")
            await self.ev("sbr3.show('drive')")

            # a command with no link: refused with a visible error, the value on screen goes back
            await self.ev("sbr3.send({t: 'set_param', id: %d, value: 321})" % pid)
            await self.until("!document.getElementById('toast').hidden && /not connected/i.test(document.getElementById('toast').textContent)",
                             4, "error toast")
            await self.until("sbr3.state.params.find(p => p.id === %d).value === 250" % pid, 3, "value back to the robot's")
            self.assertEqual(rig.b.params[pid]["value"], 250.0)
            await self.ev("document.getElementById('stop').dispatchEvent(new PointerEvent('pointerdown', {bubbles: true}))")
            await self.until("/STOP/.test(document.getElementById('toast').textContent)", 4, "STOP-may-not-have-arrived toast")
            await self.shot("phone-stale-stop.png")

            # back: live again
            rig.b.released_api = False
            await self.until("!%s.classList.contains('stale')" % tiles, 8, "readouts live again")
            self.assertEqual(await self.ev(badge), "none")
        finally:
            proc.terminate()
            try:
                proc.wait(5)
            except Exception:
                proc.kill()
            shutil.rmtree(prof, ignore_errors=True)
            await rig.stop()


if __name__ == "__main__":
    unittest.main()
