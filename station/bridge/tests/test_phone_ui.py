"""The phone as the remote (issue #64), in headless Chrome at phone size against sim + bridge: the Control screen.

No RC remote: Arm (two taps) -> Disarm button next to STOP, mode buttons, who is in charge; Disarm really disarms; STOP
latches and Release E-stop clears it; no "RC LOST" anywhere and Health says "RC remote: off". With a remote: its name shows
and the pill / Health behave as before. Portrait and landscape both fit the top bar. Skipped without Chrome / Edge.
SBR3_SAVE_SHOTS=1 also writes station/docs/screenshots/phone-controller*.png.
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
from station.bridge.tests.test_native_drive import free_port
from station.bridge.tests.test_phone_remote import PhoneRig
from station.tools.screenshots import BROWSERS, Cdp

BROWSER = next((b for b in BROWSERS if os.path.exists(b)), None)
SHOTS = os.path.join(os.path.dirname(__file__), "..", "..", "docs", "screenshots")
HOOK = r"""
window.confirm = () => true; window.__sent = [];
const _send = WebSocket.prototype.send;
WebSocket.prototype.send = function (d) {
  try { const m = JSON.parse(d); if (m.t !== "hi" && m.t !== "cmd_vel") window.__sent.push(m); } catch (e) {}
  return _send.call(this, d);
};
"""


@unittest.skipIf(BROWSER is None or os.environ.get("SBR3_FAST"), "no Chrome / Edge, or SBR3_FAST=1")
class TestPhoneUi(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_controller_without_remote(self):
        asyncio.run(self._run(remote=False))

    def test_controller_with_remote(self):
        asyncio.run(self._run(remote=True))

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

    async def size(self, w, h):
        await self.cdp.call("Emulation.setDeviceMetricsOverride", width=w, height=h, deviceScaleFactor=2, mobile=True)
        await asyncio.sleep(0.3)

    def vis(self, sel):
        return "!document.querySelector('%s').hidden && getComputedStyle(document.querySelector('%s')).display !== 'none'" % (sel, sel)

    async def fits(self, what):
        """The top bar must not push anything off the screen."""
        over = await self.ev("(() => { const t = document.querySelector('.topbar'); return t.scrollWidth - t.clientWidth })()")
        self.assertLessEqual(over, 0, "top bar overflows by %s px: %s" % (over, what))
        r = await self.ev("(() => { const r = document.getElementById('stop').getBoundingClientRect(); return [r.left, r.right, innerWidth] })()")
        self.assertTrue(0 <= r[0] and r[1] <= r[2] + 0.5, "STOP is off screen %s: %s" % (r, what))

    async def _run(self, remote):
        rig = await PhoneRig(self.tmp, remote=remote).start()
        dport, prof = free_port(), tempfile.mkdtemp(prefix="sbr3-phone-")
        proc = subprocess.Popen([BROWSER, "--headless=new", "--disable-gpu", "--no-first-run", "--hide-scrollbars",
                                 "--remote-debugging-port=%d" % dport, "--user-data-dir=" + prof, "about:blank"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            page = None
            for _ in range(75):
                try:
                    with urllib.request.urlopen("http://127.0.0.1:%d/json" % dport, timeout=1) as r:
                        page = next((p for p in json.loads(r.read()) if p.get("type") == "page"), None)
                    if page:
                        break
                except Exception:
                    pass
                await asyncio.sleep(0.2)
            self.assertIsNotNone(page, "browser did not start")
            self.cdp = Cdp(await WsClient.connect("127.0.0.1", dport, urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path))
            await self.cdp.call("Page.enable")
            await self.size(412, 915)
            await self.cdp.call("Page.addScriptToEvaluateOnNewDocument", source=HOOK)
            await self.cdp.call("Page.navigate", url="http://127.0.0.1:%d/#drive" % rig.port)
            await self.until("!!(window.sbr3 && sbr3.state.connected && sbr3.state.tlm && sbr3.state.hello)", what="page connected")
            await self.until("document.querySelector('#view-drive .tiles') && !document.querySelector('#view-drive .tiles').classList.contains('stale')")
            top = "document.querySelector('#pill-link .txt').textContent"
            who = "document.getElementById('ctl-who').textContent"
            names = "[...document.querySelectorAll('#modes button')].filter(b => !b.hidden).map(b => b.textContent)"

            if not remote:
                # ---- disarmed: Arm is the one big button, no Disarm, no mode buttons, the phone is in charge
                self.assertTrue(await self.ev(self.vis("#arm-btn")))
                self.assertFalse(await self.ev(self.vis("#disarm")))
                self.assertFalse(await self.ev(self.vis("#modes")))
                self.assertEqual(await self.ev(who), "Controlled by: this phone")
                self.assertEqual(await self.ev(top), "LINK")                       # never "RC LOST" with no remote
                await self.ev("document.getElementById('arm-btn').click()")
                self.assertEqual(await self.ev("document.getElementById('arm-btn').textContent"), "Tap again to arm")
                self.assertFalse(await self.ev("sbr3.state.tlm.status & %d" % lp.TLM_ARMED))   # one tap does not arm
                await self.ev("document.getElementById('arm-btn').click()")
                await self.until("(sbr3.state.tlm.status & %d) !== 0" % lp.TLM_ARMED, what="armed")
            else:
                self.assertEqual(await self.ev(who), "Controlled by: RC remote (its switch arms)")
                self.assertEqual(await self.ev("document.getElementById('arm-btn').hidden"), True)   # its switch arms, not this page
                await self.ev("document.querySelector('#modes [data-name=BALANCE]').click()")

            # ---- armed: Disarm next to STOP, the five mode buttons, nothing overflowing
            await self.until("sbr3.state.tlm.mode === %d" % lp.MODE_BALANCE, what="BALANCE")
            await self.until(self.vis("#disarm"), what="Disarm visible while armed")
            self.assertEqual(await self.ev(names), ["Balance", "Hold", "Manual", "Go to", "Explore"] + (["RC drive"] if remote else []))
            await self.fits("portrait 412, armed")
            await self.size(360, 740)
            await self.fits("portrait 360, armed")
            await self.size(412, 915)
            self.assertEqual(await self.ev(top), "LINK")

            # drive from the phone: Manual (joystick) mode, then the Control screen shot
            await self.ev("document.querySelector('#modes [data-name=STATION]').click()")
            await self.until("sbr3.state.tlm.mode === %d" % lp.MODE_STATION, what="Manual")
            await self.until("sbr3.state.tlm.status & %d" % lp.TLM_UPRIGHT, 15, "upright")
            await asyncio.sleep(3.6)                                   # (the Arming toast goes away)
            await self.shot("phone-controller.png")
            await self.size(915, 412)
            await self.fits("landscape 915, armed")
            await asyncio.sleep(0.4)
            await self.shot("phone-controller-landscape.png")
            await self.size(844, 390)
            await self.fits("landscape 844, armed")
            await self.size(412, 915)

            # ---- Disarm: one tap, SET_MODE IDLE, no latch
            await self.ev("__sent.length = 0; document.getElementById('disarm').click()")
            await self.until("sbr3.state.tlm.mode === %d" % lp.MODE_IDLE, what="IDLE")
            self.assertEqual(await self.ev("__sent.filter(m => m.t === 'set_mode').map(m => [m.mode, !!m.arm])"), [[lp.MODE_IDLE, False]])
            self.assertFalse(rig.b.estop)
            if not remote:
                await self.until("(sbr3.state.tlm.status & %d) === 0" % lp.TLM_ARMED, what="disarmed")
                await self.until(self.vis("#arm-btn"), what="Arm is back")
                await self.until("document.getElementById('disarm').hidden", what="Disarm gone once disarmed")
            else:
                # the RC remote's switch still says armed: nothing left to turn off, the button goes away
                self.assertTrue(await self.ev("(sbr3.state.tlm.status & %d) !== 0" % lp.TLM_ARMED))
                await self.until("document.getElementById('disarm').hidden", what="Disarm hidden at IDLE with a remote")

            # ---- STOP latches; Disarm is gone; Release E-stop (confirm) clears it
            if not remote:
                await self.ev("document.getElementById('arm-btn').click(); document.getElementById('arm-btn').click()")
                await self.until(self.vis("#disarm"), what="armed again")
            await self.ev("document.getElementById('stop').dispatchEvent(new PointerEvent('pointerdown', {bubbles: true}))")
            await self.until("!document.getElementById('estop-bar').hidden", what="E-stop bar")
            self.assertEqual(await self.ev("document.querySelector('#pill-armed .txt').textContent"), "E-STOP")
            await self.until("document.getElementById('disarm').hidden", what="no Disarm while latched")
            await self.ev("document.getElementById('estop-release').click()")
            await self.until("document.getElementById('estop-bar').hidden", what="released")
            if not remote:
                await self.until(self.vis("#arm-btn"), what="Arm again after the release (release never arms)")

            # ---- Health: the RC remote line is neutral when there is none
            await self.ev("sbr3.show('health')")
            await self.until("document.querySelector('.hl-tile.lv-na, .hl-tile') && [...document.querySelectorAll('.hl-tile')].some(t => /RC remote/.test(t.textContent))", what="health tiles")
            tile = await self.ev("[...document.querySelectorAll('.hl-tile')].find(t => /RC remote/.test(t.textContent)).className + '|' + "
                                 "[...document.querySelectorAll('.hl-tile')].find(t => /RC remote/.test(t.textContent)).querySelector('.hl-v').textContent")
            if not remote:
                self.assertEqual(tile.split("|")[1], "off")
                self.assertIn("lv-na", tile)
                self.assertNotIn("lv-bad", tile)
            self.assertNotIn("RC LOST", await self.ev("document.body.innerText"))
        finally:
            proc.terminate()
            shutil.rmtree(prof, ignore_errors=True)
            await rig.stop()


if __name__ == "__main__":
    unittest.main()
