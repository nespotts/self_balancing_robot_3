"""Health view (#51) in headless Chrome at phone width against sim + bridge: tiles, preflight, battery minutes, and
that the page never sends a command. Skipped when no Chrome / Edge is installed.
SBR3_SAVE_SHOTS=1 also writes the screenshots to station/docs/screenshots/.
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

from station.bridge.httpws import WsClient
from station.bridge.tests.test_hardening import Rig
from station.bridge.tests.test_native_drive import free_port
from station.bridge.tests.test_stale_ui import BROWSER, HOOK, SHOTS
from station.tools.screenshots import Cdp


@unittest.skipIf(BROWSER is None, "no Chrome / Edge")
class TestHealthUi(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

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

    def test_health_view(self):
        asyncio.run(self._health())

    async def _health(self):
        rig = await Rig(self.tmp).start()
        dport, prof = free_port(), tempfile.mkdtemp(prefix="sbr3-health-")
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

            rig.b.health.battery.BIN_S, rig.b.health.battery.MIN_SPAN_S = 2.0, 20.0   # short learning time
            rig.robot.drain = 0.0019                          # V/s: ~18 min to the limit
            rig.robot.remote, rig.robot.radio_log_at = True, 0.0     # RC remote on (switch off): radio tile + line
            rig.robot.remote_armed = False
            await self.cdp.call("Page.navigate", url="http://127.0.0.1:%d/#health" % rig.port)
            await self.until("!!(window.sbr3 && sbr3.state.health && sbr3.state.health.battery.minutes != null)", 60, "minutes learned")
            await self.until("document.querySelectorAll('.hl-tile').length === 14", 5, "14 tiles")
            await self.until("sbr3.state.health.preflight.level !== 'red'", 8, "preflight not red")
            self.assertEqual(await self.ev("document.body.dataset.view"), "health")
            self.assertEqual(await self.ev("document.getElementById('hl-dot').className.includes('lv-ok')"), True)
            self.assertEqual(await self.ev("document.getElementById('hl-text').textContent"), "Ready to arm")
            self.assertRegex(await self.ev("document.getElementById('hl-batt').textContent"), r"about \d+ min left")
            self.assertRegex(await self.ev("document.querySelector('#pill-batt .txt').textContent"), r"^\d+\.\d V ~\d+m$")
            self.assertEqual(await self.ev("document.documentElement.scrollWidth <= innerWidth"), True)    # no sideways scroll
            # tapping a good tile opens its reason
            self.assertEqual(await self.ev("getComputedStyle(document.querySelector('.hl-tile .hl-w')).display"), "none")
            await self.ev("document.querySelector('.hl-tile').click()")
            self.assertEqual(await self.ev("getComputedStyle(document.querySelector('.hl-tile .hl-w')).display"), "block")
            await self.ev("document.querySelector('.hl-tile').click()")
            await self.shot("phone-health.png")

            # a fault turns the checklist red and names the part
            rig.robot.fault = 4
            await self.until("document.getElementById('hl-dot').className.includes('lv-bad')", 5, "red banner")
            self.assertRegex(await self.ev("document.getElementById('hl-text').textContent"), r"^Not ready")
            self.assertEqual(await self.ev("[...document.querySelectorAll('.hl-tile.lv-bad .hl-l')].map(e => e.textContent)"), ["Enc R"])
            await self.shot("phone-health-fault.png")
            rig.robot.fault = 0

            # the robot goes away: nothing is claimed
            rig.b.released_api = True
            await self.until("document.getElementById('hl-text').textContent.includes('released')", 6, "released")
            rig.b.released_api = False

            # the page only looks: nothing but the hello went out
            self.assertEqual(await self.ev("__sent.length"), 0)
            self.assertFalse(rig.robot.armed)
        finally:
            proc.terminate()
            await rig.stop()
            shutil.rmtree(prof, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()