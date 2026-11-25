"""Screenshots of ROS inside the station (#106), against the sim stack of ros2/scripts/sim_e2e_station.sh (HOLD=1).

  wsl -d Ubuntu-24.04 -- env HOLD=1 bash ros2/scripts/sim_e2e_station.sh simroom      (leave it running)
  python -m station.tools.ros_shots --url http://localhost:8095/

SIMULATOR ONLY: it arms the sim robot and taps Manual (as the owner would), so it refuses any non-local bridge.
Headless Chrome / Edge over the DevTools protocol (station/tools/screenshots.py); PNGs in station/docs/screenshots.
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
from station.tools.screenshots import BROWSERS, DESKTOP, OUT, PHONE, Cdp

MODES = ["IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"]


class Station:
    def __init__(self, ws):
        self.ws, self.last = ws, {}
        self.task = asyncio.ensure_future(self.reader())

    async def reader(self):
        while True:
            text = await self.ws.recv()
            if text is None:
                return
            m = json.loads(text)
            self.last[m.get("t")] = m

    def send(self, **m):
        self.ws.send(json.dumps(m))

    async def until(self, pred, secs, what):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if pred():
                return
            await asyncio.sleep(0.1)
        raise TimeoutError(what)

    def nav(self):
        return self.last.get("ros_nav") or {}


class Browser:
    def __init__(self, browser, port=9334):
        self.browser, self.port, self.dev = browser, port, None

    async def start(self):
        self.prof = tempfile.mkdtemp(prefix="sbr3-ros-shot-")
        self.proc = subprocess.Popen([self.browser, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                                      "--no-first-run", "--remote-debugging-port=%d" % self.port,
                                      "--user-data-dir=" + self.prof, "about:blank"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(50):
            try:
                targets = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/json/list" % self.port, timeout=1).read())
                page = [t for t in targets if t["type"] == "page"][0]
                break
            except Exception:
                await asyncio.sleep(0.2)
        path = urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path
        self.cdp = Cdp(await WsClient.connect("127.0.0.1", self.port, path))
        await self.cdp.call("Page.enable")

    async def open(self, url, dev, view="map", ros_view=True, wait=6.0):
        w, h, dpr, mobile = dev
        await self.cdp.call("Emulation.setDeviceMetricsOverride", width=w, height=h, deviceScaleFactor=dpr, mobile=mobile)
        await self.cdp.call("Emulation.setTouchEmulationEnabled", enabled=mobile)
        await self.cdp.call("Page.navigate", url=url)
        await asyncio.sleep(1.0)
        await self.js("localStorage.setItem('sbr3.ros.view', %s)" % json.dumps(json.dumps(ros_view)))
        await self.cdp.call("Page.navigate", url="about:blank")
        await asyncio.sleep(0.3)
        await self.cdp.call("Page.navigate", url=url + "#" + view)
        await asyncio.sleep(wait)
        self.dev = dev

    async def js(self, expr):
        r = await self.cdp.call("Runtime.evaluate", expression=expr, returnByValue=True)
        return r.get("result", {}).get("value")

    async def tap(self, x, y):
        await self.cdp.call("Input.dispatchTouchEvent", type="touchStart", touchPoints=[{"x": x, "y": y}])
        await asyncio.sleep(0.08)
        await self.cdp.call("Input.dispatchTouchEvent", type="touchEnd", touchPoints=[])

    async def shot(self, name):
        s = await self.cdp.call("Page.captureScreenshot", format="png")
        fn = os.path.join(OUT, name + ".png")
        with open(fn, "wb") as f:
            f.write(base64.b64decode(s["data"]))
        print("saved", fn, flush=True)

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(5)
        except Exception:
            self.proc.kill()
        shutil.rmtree(self.prof, ignore_errors=True)


async def run(url, browser):
    u = urllib.parse.urlsplit(url)
    if u.hostname not in ("localhost", "127.0.0.1"):
        raise SystemExit("ros_shots: refusing %s - local simulator only" % url)
    S = Station(await WsClient.connect(u.hostname, u.port or 80))
    S.send(t="hi", name="ros_shots")
    await S.until(lambda: (S.last.get("status") or {}).get("ros") and S.last["status"]["ros"]["connected"], 60, "ROS")
    t = S.last.get("tlm") or {}
    if not (t and MODES[t["mode"]] == "STATION" and t["status"] & 1):
        S.send(t="set_mode", mode=MODES.index("BALANCE"), arm=True)
        await asyncio.sleep(3)
        S.send(t="set_mode", mode=MODES.index("STATION"))
    await S.until(lambda: S.last.get("tlm") and MODES[S.last["tlm"]["mode"]] == "STATION", 10, "STATION")
    await S.until(lambda: ((S.last.get("ros_status") or {}).get("loc") or {}).get("state") == "ok", 60, "localized")
    os.makedirs(OUT, exist_ok=True)
    B = Browser(browser)
    await B.start()
    try:
        S.send(t="ros", action="cancel")
        await asyncio.sleep(2)
        def far_goal():                                  # the far side of the room
            p = (S.last.get("ros_pose") or {}).get("p") or [0, 0, 0]
            gx, gy, gyaw = (0.0, 0.0, 3.14) if (p[0] - 3.3) ** 2 + (p[1] - 1.3) ** 2 < 4.0 else (3.3, 1.3, 0.0)
            S.send(t="ros", action="goal", frame="map", x=gx, y=gy, yaw=gyaw)

        def busy(kind=None, dist=0.6):
            n = S.nav()
            return n.get("state") == "navigating" and (kind is None or n.get("kind") == kind) and \
                (n.get("dist_m") or 0) > dist and len((S.last.get("ros_plan") or {}).get("pts") or []) > 1

        async def moving_shot(name, dev, start, kind=None):
            """a shot while Nav2 drives (a loaded PC can make Nav2 abort a goal: start it again)"""
            for _ in range(4):
                if not busy(kind):
                    start()
                    try:
                        await S.until(lambda: busy(kind, 1.0), 25, name)
                    except TimeoutError:
                        continue
                await B.open(url, dev, wait=4.0)
                await frame(dev)
                if busy(kind, 0.3):
                    await B.shot(name)
                    return
            raise TimeoutError("no Nav2 run for " + name)

        async def frame(dev):                            # the whole sim room in view
            cx, cy, s = (1650, 150, 0.074) if dev is PHONE else (1450, -150, 0.16)
            await B.js("document.getElementById(\"map-follow\").click(); Object.assign(sbr3.map().view, "
                       "{cx: %d, cy: %d, s: %s})" % (cx, cy, s))
            await asyncio.sleep(1.5)

        # 1. a Nav2 goal on the way: ROS view, plan, status line (phone + desktop)
        await moving_shot("phone-ros-nav", PHONE, far_goal)
        await moving_shot("desktop-ros-nav", DESKTOP, far_goal)
        await S.until(lambda: S.nav().get("state") in ("succeeded", "failed", "cancelled"), 150, "goal end")
        await asyncio.sleep(1.5)
        # 2. tap for a goal (pending, heading arrow) on the phone
        await B.open(url, PHONE, wait=4.0)
        await frame(PHONE)
        await B.tap(412 / 2 - 40, 915 / 2 + 40)
        await asyncio.sleep(1.0)
        await B.shot("phone-ros-goal")
        await B.js("document.getElementById('ros-clear').click()")
        # 3. a place: Go here (ROS)
        await B.js("sbr3.openPlace('Couch')")
        await asyncio.sleep(1.0)
        await B.shot("phone-ros-place")
        await B.js("document.getElementById('ps-close').click()")
        # 4. the patrol picker, then a patrol running (desktop)
        await B.js("document.getElementById('ros-patrol').click()")
        await asyncio.sleep(0.8)
        await B.shot("phone-ros-patrol")
        await B.js("document.getElementById('rp-close').click()")
        await moving_shot("desktop-ros-patrol", DESKTOP, lambda: S.send(
            t="ros", action="patrol", places=["Window", "Couch", "Kitchen"], loops=2, dwell=4), kind="patrol")
        # 5. Health with the ROS tile
        ros_tile = ("(() => { const t = [...document.querySelectorAll(\".hl-tile\")].find((b) => "
                    "b.querySelector(\".hl-l\").textContent === \"ROS\"); if (t) { t.click(); "
                    "t.scrollIntoView({block: \"center\"}); } return !!t; })()")
        for name, dev in (("phone-ros-health", PHONE), ("desktop-ros-health", DESKTOP)):
            await B.open(url, dev, view="health", wait=4.0)
            await B.js(ros_tile)
            await asyncio.sleep(1.0)
            await B.shot(name)
        # 6. the robot leaves Manual during the patrol: the station says why
        S.send(t="set_mode", mode=MODES.index("BALANCE"))
        await B.open(url, PHONE, wait=1.5)
        await frame(PHONE)
        await B.shot("phone-ros-manual")
        S.send(t="set_mode", mode=MODES.index("STATION"))
        S.send(t="ros", action="cancel")
    finally:
        B.stop()
        S.task.cancel()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8095/")
    ap.add_argument("--browser", default=next((b for b in BROWSERS if os.path.exists(b)), None))
    a = ap.parse_args()
    asyncio.run(run(a.url, a.browser))


if __name__ == "__main__":
    main()
