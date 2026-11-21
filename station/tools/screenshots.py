"""Capture UI screenshots with headless Chrome/Edge (DevTools protocol, stdlib only).

  python -m station.sim --http-port 8090            (in another terminal)
  python -m station.tools.screenshots --url http://localhost:8090/ --prep

--prep drives the simulated robot along a route first so the map has a track and range points.
--prep-map explores the sim room (occupancy map), then plans a route behind the partition wall;
  shoot the map-* shots after it:  ... --prep-map phone-occmap desktop-occmap phone-map-menu desktop-route
"""
import argparse
import asyncio
import base64
import json
import os
import shutil
import subprocess
import tempfile
import time
import urllib.parse
import urllib.request

from station.bridge.httpws import WsClient

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "screenshots")
BROWSERS = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            "/usr/bin/chromium", "/usr/bin/google-chrome"]

PHONE = (412, 915, 2.0, True)
DESKTOP = (1440, 900, 1.0, False)
async def rect(cdp, sel):
    r = await cdp.call("Runtime.evaluate", returnByValue=True, expression=(
        "(()=>{const r=document.querySelector(%s).getBoundingClientRect();"
        "return [r.left+r.width/2, r.top+r.height/2, r.width, r.height]})()" % json.dumps(sel)))
    return r["result"]["value"]


async def touch(cdp, kind, x, y):
    pts = [] if kind == "touchEnd" else [{"x": x, "y": y}]
    await cdp.call("Input.dispatchTouchEvent", type=kind, touchPoints=pts)


async def act_drive(cdp):
    """Take control, push the joystick up-left and hold it while the screenshot is taken."""
    await cdp.call("Runtime.evaluate", expression="document.getElementById(\"joy-take\").click()")
    await asyncio.sleep(1.5)
    x, y, w, _h = await rect(cdp, "#joy")
    await touch(cdp, "touchStart", x, y)
    for i in range(1, 11):
        await touch(cdp, "touchMove", x - w * 0.012 * i, y - w * 0.03 * i)
        await asyncio.sleep(0.05)
    for _ in range(25):   # keep moving slightly so the hold looks real (and cmd_vel keeps flowing)
        await touch(cdp, "touchMove", x - w * 0.12, y - w * 0.3)
        await asyncio.sleep(0.1)
    return lambda: touch(cdp, "touchEnd", 0, 0)


async def act_waypoints(cdp):
    """Tap three new waypoints on the map (not sent: they show as pending with the Go button)."""
    x, y, w, h = await rect(cdp, "#map")
    for dx, dy in ((0.25, -0.2), (0.3, 0.05), (-0.2, 0.15)):
        await touch(cdp, "touchStart", x + w * dx, y + h * dy)
        await touch(cdp, "touchEnd", 0, 0)
        await asyncio.sleep(0.2)
    return None


FULL_RATE = "document.getElementById('tc-full').classList.contains('on')||document.getElementById('tc-full').click()"

SHOTS = [  # name, device, view, js to run after the view is shown, extra wait s[, action]
    ("phone-map", PHONE, "map", "", 1),
    ("phone-waypoints", PHONE, "map", "", 0.5, act_waypoints),
    ("phone-driving", PHONE, "drive", "", 0, act_drive),
    ("phone-drive", PHONE, "drive", "", 1),
    ("phone-sensors", PHONE, "sensors", "", 1.5),
    ("phone-sensors-setup", PHONE, "sensors", "document.getElementById('sens-more').click()", 1),
    ("phone-plots", PHONE, "plots", "", 1),
    ("phone-tune", PHONE, "tune", FULL_RATE, 6),
    ("phone-logs", PHONE, "logs", "", 1.5),
    ("desktop-map", DESKTOP, "map", "", 1),
    ("desktop-sensors", DESKTOP, "sensors", "", 1.5),
    ("desktop-plots", DESKTOP, "plots", "", 1),
    ("desktop-tune", DESKTOP, "tune", FULL_RATE, 6),
    ("desktop-logs", DESKTOP, "logs", "", 1.5),
    # occupancy map / route / explore (issue #12): run with --prep-map
    ("desktop-occmap", DESKTOP, "map", "", 1.5),
    ("desktop-route", DESKTOP, "map", "sbr3.send({t:'plan',action:'goto',points:[[2900,1650]]})", 4),
    ("phone-occmap", PHONE, "map", "", 1.5),
    ("phone-map-menu", PHONE, "map", "document.getElementById('map-more').click()", 1),
    ("phone-explore", PHONE, "map", "document.getElementById('map-menu').hidden=true;sbr3.send({t:'map',action:'reset'});sbr3.send({t:'plan',action:'explore'})", 14),
    # last: disarm, then the first tap of the two-tap "Arm & Balance"
    ("phone-arm", PHONE, "drive", "sbr3.send({t:'set_mode',mode:0});"
     "setTimeout(()=>document.getElementById('arm-btn').click(),1800)", 2.5),
]


class Cdp:
    def __init__(self, ws):
        self.ws, self.n = ws, 0

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


async def prep(url):
    u = urllib.parse.urlsplit(url)
    ws = await WsClient.connect(u.hostname, u.port or 80)
    send = lambda **m: ws.send(json.dumps(m))
    async def drain(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            try:
                await asyncio.wait_for(ws.recv(), max(0.01, end - time.monotonic()))
            except asyncio.TimeoutError:
                pass
    await drain(1)
    send(t="set_mode", mode=1)
    await drain(2)
    send(t="goto", points=[[1200, 0], [1800, -600], [2400, 300], [3300, 300]], replace=True, go=True)
    await drain(22)
    send(t="goto", points=[[3300, 700], [2000, 400], [600, 500]], replace=True, go=True)
    await drain(5)
    ws.close()


async def prep_map(url, max_s=150):
    """Explore the sim room through the bridge (like pressing Explore), then drive to the far side."""
    u = urllib.parse.urlsplit(url)
    ws = await WsClient.connect(u.hostname, u.port or 80)
    send = lambda **m: ws.send(json.dumps(m))
    end = time.monotonic() + max_s
    await asyncio.sleep(2)
    send(t="map", action="reset")
    send(t="plan", action="explore")
    mode = None
    while time.monotonic() < end:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 1))
        except asyncio.TimeoutError:
            continue
        if m.get("t") == "plan":
            print("plan:", m.get("mode"), m.get("status"))
            if mode == "explore" and m.get("mode") == "idle":
                break
            mode = m.get("mode")
    send(t="plan", action="goto", points=[[0, 0]])        # back near the start for the shots
    await asyncio.sleep(12)
    ws.close()


async def shoot(url, browser, names, port=9333):
    prof = tempfile.mkdtemp(prefix="sbr3-shot-")
    proc = subprocess.Popen([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--no-first-run",
                             "--remote-debugging-port=%d" % port, "--user-data-dir=" + prof, "about:blank"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(50):
            try:
                targets = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/json/list" % port, timeout=1).read())
                page = [t for t in targets if t["type"] == "page"][0]
                break
            except Exception:
                await asyncio.sleep(0.2)
        path = urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path
        cdp = Cdp(await WsClient.connect("127.0.0.1", port, path))
        await cdp.call("Page.enable")
        os.makedirs(OUT, exist_ok=True)
        current = None
        for name, (w, h, dpr, mobile), view, js, wait, *action in SHOTS:
            if names and name not in names:
                continue
            if current != (w, h, dpr, mobile):
                await cdp.call("Emulation.setDeviceMetricsOverride", width=w, height=h, deviceScaleFactor=dpr, mobile=mobile)
                await cdp.call("Emulation.setTouchEmulationEnabled", enabled=mobile)
                await cdp.call("Page.navigate", url="about:blank")   # fresh page (a hash change alone keeps state)
                await asyncio.sleep(0.5)
                await cdp.call("Page.navigate", url=url + "#" + view)
                await asyncio.sleep(7)   # connect + fill the plot buffers
                current = (w, h, dpr, mobile)
            await cdp.call("Runtime.evaluate", expression="sbr3.show(%s); %s" % (json.dumps(view), js))
            after = await action[0](cdp) if action else None
            await asyncio.sleep(wait)
            shot = await cdp.call("Page.captureScreenshot", format="png")
            if after:
                await after()
            fn = os.path.join(OUT, name + ".png")
            with open(fn, "wb") as f:
                f.write(base64.b64decode(shot["data"]))
            print("saved", fn)
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
        shutil.rmtree(prof, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8090/")
    ap.add_argument("--prep", action="store_true", help="drive the simulated robot first")
    ap.add_argument("--prep-map", action="store_true", help="explore the sim room first (map shots)")
    ap.add_argument("--browser", default=next((b for b in BROWSERS if os.path.exists(b)), None))
    ap.add_argument("--cdp-port", type=int, default=9333, help="browser debugging port")
    ap.add_argument("names", nargs="*", help="only these shots")
    a = ap.parse_args()

    async def run():
        if a.prep:
            await prep(a.url)
        if a.prep_map:
            await prep_map(a.url)
        await shoot(a.url, a.browser, a.names, a.cdp_port)
    asyncio.run(run())


if __name__ == "__main__":
    main()
