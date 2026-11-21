"""Screenshots of the Android app screens in headless Chrome, with a fake window.SBR3Native:

  python -m station.sim --http-port 8090            (in another terminal)
  python -m station.tools.android_shots --url http://localhost:8090/

android-find (searching), android-notfound, android-drive-tilt (dead-man held, tilted), android-landscape.
"""
import argparse
import asyncio
import base64
import json
import os
import shutil
import subprocess
import tempfile
import urllib.parse
import urllib.request

from station.bridge.httpws import WsClient
from station.tools.screenshots import BROWSERS, OUT, Cdp

HERE = os.path.dirname(os.path.abspath(__file__))
FIND = os.path.join(os.path.dirname(HERE), "android", "assets", "find.html")

FAKE = r"""
window.__tilt = { pitch: -35, roll: 2 };
window.SBR3Native = {
  appInfo: () => JSON.stringify({ version: "2026.11.21", tilt: true, orientation: "auto" }),
  lastAddress: () => "192.168.0.209:8080",
  findStart: () => {
    const sc = window.__scenario || "search";
    let d = 0;
    const tick = () => {
      d += 24;
      __sbr3.emit("find", JSON.stringify({ phase: "scan", subnet: "192.168.0.x", total: 253, done: Math.min(d, 253), wifi: true }));
      if (sc === "search" && d >= 120) return;
      if (d >= 253) { __sbr3.emit("findDone", "{}"); return; }
      setTimeout(tick, 30);
    };
    setTimeout(tick, 50);
  },
  findStop: () => {}, probe: () => {}, connect: () => {}, openFinder: () => {},
  tiltStart: () => {}, tiltStop: () => {}, vibrate: () => {}, keepAwake: () => {}, setOrientation: () => {},
  gamepads: () => JSON.stringify({ list: [{ id: 7, name: "Xbox Wireless Controller" }] }),
};
setInterval(() => { if (window.__sbr3) __sbr3.emit("tilt", JSON.stringify({ pitch: __tilt.pitch, roll: __tilt.roll, t: 0 })); }, 33);
"""


async def main_async(url, browser, names):
    port = 9334
    prof = tempfile.mkdtemp(prefix="sbr3-ashot-")
    proc = subprocess.Popen([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--no-first-run",
                             "--allow-file-access-from-files", "--remote-debugging-port=%d" % port,
                             "--user-data-dir=" + prof, "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        page = None
        for _ in range(75):
            try:
                lst = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/json/list" % port, timeout=1).read())
                page = [t for t in lst if t["type"] == "page"][0]
                break
            except Exception:
                await asyncio.sleep(0.2)
        cdp = Cdp(await WsClient.connect("127.0.0.1", port, urllib.parse.urlsplit(page["webSocketDebuggerUrl"]).path))
        await cdp.call("Page.enable")
        await cdp.call("Page.addScriptToEvaluateOnNewDocument", source=FAKE)
        async def ev(js):
            r = await cdp.call("Runtime.evaluate", expression=js, returnByValue=True)
            if "exceptionDetails" in r:
                raise RuntimeError(json.dumps(r["exceptionDetails"])[:600])
            return r

        async def metrics(w, h):
            await cdp.call("Emulation.setDeviceMetricsOverride", width=w, height=h, deviceScaleFactor=2, mobile=True)
            await cdp.call("Emulation.setTouchEmulationEnabled", enabled=True)

        async def save(name):
            shot = await cdp.call("Page.captureScreenshot", format="png")
            fn = os.path.join(OUT, name + ".png")
            with open(fn, "wb") as f:
                f.write(base64.b64decode(shot["data"]))
            print("saved", fn)

        async def nav(u, wait):
            await cdp.call("Page.navigate", url="about:blank")
            await asyncio.sleep(0.3)
            await cdp.call("Page.navigate", url=u)
            await asyncio.sleep(wait)

        want = lambda n: not names or n in names
        os.makedirs(OUT, exist_ok=True)
        find_url = "file:///" + FIND.replace("\\", "/")
        await metrics(412, 915)
        if want("android-find"):
            await cdp.call("Page.addScriptToEvaluateOnNewDocument", source="window.__scenario='search'")
            await nav(find_url, 1.2)
            await save("android-find")
        if want("android-notfound"):
            await cdp.call("Page.addScriptToEvaluateOnNewDocument", source="window.__scenario='none'")
            await nav(find_url, 1.5)
            await save("android-notfound")
        if want("android-drive-tilt") or want("android-landscape"):
            await nav(url + "#drive", 5)
            await ev("document.querySelector('#input-seg [data-v=tilt]').click();"
                     "sbr3.send({t:'set_mode', mode: sbr3.state.hello.modes.indexOf('STATION')})")
            await asyncio.sleep(2.5)
        if want("android-drive-tilt"):
            r = (await ev("(()=>{const r=document.getElementById('joy').getBoundingClientRect();"
                          "return [r.left+r.width/2, r.top+r.height/2]})()"))["result"]["value"]
            await cdp.call("Input.dispatchTouchEvent", type="touchStart", touchPoints=[{"x": r[0], "y": r[1]}])
            for i in range(1, 9):      # tilt forward and a little right, gradually, while held (cmd_vel flows)
                await ev("__tilt.pitch = %f; __tilt.roll = %f" % (-35 + 1.6 * i, 2 + 0.9 * i))
                await asyncio.sleep(0.12)
            await asyncio.sleep(1.0)
            await save("android-drive-tilt")
            await cdp.call("Input.dispatchTouchEvent", type="touchEnd", touchPoints=[])
            await ev("__tilt.pitch = -35; __tilt.roll = 2")
            await asyncio.sleep(0.5)
        if want("android-drive-hold"):
            await save("android-drive-hold")
        if want("android-landscape"):
            await metrics(915, 412)
            await asyncio.sleep(1.0)
            await ev("sbr3.show('drive')")
            await asyncio.sleep(0.8)
            await save("android-landscape")
            await metrics(870, 412)              # cutout side letterboxed: the phone (tabs) layout
            await asyncio.sleep(1.0)
            await save("android-landscape-narrow")
        await ev("sbr3.send({t:'set_mode', mode: 0})")
        await asyncio.sleep(0.3)
        cdp.ws.close()
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
    ap.add_argument("--browser", default=next((b for b in BROWSERS if os.path.exists(b)), None))
    ap.add_argument("names", nargs="*")
    a = ap.parse_args()
    asyncio.run(main_async(a.url, a.browser, a.names))


if __name__ == "__main__":
    main()
