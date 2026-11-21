"""Browser console smoke test (#49): every tab of the web app loads against sim + bridge in headless Chrome with no
uncaught exception, no console error and no horizontal overflow, at phone and desktop width.
Skipped when no Chrome / Edge is installed, or when SBR3_FAST=1 (quick run without the browser tests)."""
import asyncio
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
from station.tools.screenshots import BROWSERS, Cdp

BROWSER = next((b for b in BROWSERS if os.path.exists(b)), None)
TABS = ["drive", "map", "sensors", "plots", "tune", "logs"]
SIZES = [(360, 740), (1440, 900)]


class EventCdp(Cdp):
    """Cdp that keeps the events it passes while waiting for a reply."""
    def __init__(self, ws):
        super().__init__(ws)
        self.events = []

    async def call(self, method, **params):
        self.n += 1
        my = self.n
        self.ws.send(json.dumps({"id": my, "method": method, "params": params}))
        while True:
            m = json.loads(await self.ws.recv())
            if m.get("id") == my:
                if "error" in m:
                    raise RuntimeError("%s: %s" % (method, m["error"]))
                return m.get("result", {})
            if "method" in m:
                self.events.append(m)

    async def pump(self, sec):
        """Collect events for a while (a call that is answered at once reads whatever arrived before it)."""
        end = time.monotonic() + sec
        while time.monotonic() < end:
            try:
                m = json.loads(await asyncio.wait_for(self.ws.recv(), max(0.01, end - time.monotonic())))
                if "method" in m:
                    self.events.append(m)
            except asyncio.TimeoutError:
                pass


def problems(events):
    out = []
    for e in events:
        p, m = e.get("params", {}), e["method"]
        if m == "Runtime.exceptionThrown":
            d = p["exceptionDetails"]
            out.append("exception: %s" % ((d.get("exception") or {}).get("description") or d.get("text")))
        elif m == "Runtime.consoleAPICalled" and p.get("type") == "error":
            out.append("console.error: %s" % " ".join(str(a.get("value", a.get("description", ""))) for a in p["args"]))
        elif m == "Log.entryAdded" and p["entry"].get("level") == "error":
            out.append("log error: %s %s" % (p["entry"].get("text"), p["entry"].get("url", "")))
    return out


@unittest.skipIf(BROWSER is None or os.environ.get("SBR3_FAST"), "no Chrome / Edge, or SBR3_FAST=1")
class TestConsoleSmoke(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_every_tab_loads_clean(self):
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

    async def _run(self):
        rig = await Rig(self.tmp).start()
        dport, prof = free_port(), tempfile.mkdtemp(prefix="sbr3-smoke-")
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
            self.cdp = EventCdp(await WsClient.connect("127.0.0.1", dport, urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path))
            for d in ("Page", "Runtime", "Log"):
                await self.cdp.call(d + ".enable")
            bad = []
            for w, h in SIZES:
                await self.cdp.call("Emulation.setDeviceMetricsOverride", width=w, height=h, deviceScaleFactor=1, mobile=w < 600)
                await self.cdp.call("Page.navigate", url="http://127.0.0.1:%d/#drive" % rig.port)
                await self.until("!!(window.sbr3 && sbr3.state.connected && sbr3.state.tlm && sbr3.state.hello)", what="page connected")
                for tab in TABS:
                    await self.ev("document.querySelector('#tabs [data-view=%s]').click()" % tab)
                    await self.until("!document.getElementById('view-%s').hidden && getComputedStyle(document.getElementById('view-%s')).display !== 'none'" % (tab, tab),
                                     what="tab %s shown" % tab)
                    await self.cdp.pump(0.4)
                    over = await self.ev("document.documentElement.scrollWidth - document.documentElement.clientWidth")
                    if over > 0:
                        bad.append("%s @%dx%d: horizontal overflow %dpx" % (tab, w, h, over))
                bad += ["@%dx%d %s" % (w, h, p) for p in problems(self.cdp.events)]
                self.cdp.events.clear()
            self.assertEqual(bad, [])
        finally:
            if getattr(self, "cdp", None):
                self.cdp.ws.close()
            proc.terminate()
            try:
                proc.wait(5)
            except Exception:
                proc.kill()
            shutil.rmtree(prof, ignore_errors=True)
            await rig.stop()


if __name__ == "__main__":
    unittest.main()