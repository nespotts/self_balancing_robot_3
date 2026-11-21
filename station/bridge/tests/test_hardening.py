"""Ground-station hardening (issues #44-#48): who may command the robot, event-loop stalls, tlm.hz after Tune,
bad input, half-open clients, stale / no-link commands. Real bridge + sim over loopback sockets.
Run: python -m unittest discover -s station -t ."""
import asyncio
import importlib.util
import json
import os
import shutil
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from types import SimpleNamespace
from unittest import mock

from station.bridge import bridge as bridge_mod
from station.bridge import httpws
from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.httpws import HttpServer, WsClient, check_origin, host_ok
from station.sim.sim import SimRobot, SimServer

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")
TUNE_PY = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "tune.py")


class Rig:
    """sim <-> bridge <-> real HTTP/WebSocket server on a free loopback port (link=False: no robot at all)."""

    def __init__(self, tmp, link=True, **kw):
        self.tmp, self.link, self.kw = tmp, link, kw

    async def start(self):
        self.robot = SimRobot(seed=1)
        sim = SimServer(self.robot, "127.0.0.1", 0, quiet=True)
        self.srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        self.phys = asyncio.ensure_future(sim.physics())
        sim_port = self.srv.sockets[0].getsockname()[1] if self.link else 1
        self.b = Bridge("tcp://127.0.0.1:%d" % sim_port, WEB, os.path.join(self.tmp, "logs"), "127.0.0.1", 0,
                        release_file=os.path.join(self.tmp, "release"), quiet=True, map_dir=os.path.join(self.tmp, "maps"),
                        preset_dir=os.path.join(self.tmp, "presets"), **self.kw)
        self.hs = await self.b.build_http().start("127.0.0.1", 0)
        self.port = self.hs.sockets[0].getsockname()[1]
        self.tasks = [asyncio.ensure_future(self.b.link_task()), asyncio.ensure_future(self.b.tick_task())]
        return self

    async def stop(self):
        for t in self.tasks + [self.phys]:
            t.cancel()
        self.srv.close()
        self.hs.close()
        self.b.log.stop()

    async def until(self, pred, what, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return
            await asyncio.sleep(0.03)
        raise AssertionError("timeout: " + what)

    async def ws(self, headers=None):
        return await WsClient.connect("127.0.0.1", self.port, headers=headers)


async def raw(port, data, until_close=True, timeout=5.0):
    """Send raw bytes, return (status, head, body). until_close=False: stop after the first chunk (WebSocket 101)."""
    r, w = await asyncio.open_connection("127.0.0.1", port)
    w.write(data)
    await w.drain()
    buf = b""
    try:
        while True:
            chunk = await asyncio.wait_for(r.read(65536), timeout)
            if not chunk:
                break
            buf += chunk
            if not until_close:
                break
    finally:
        w.close()
    head, _, body = buf.partition(b"\r\n\r\n")
    status = int(head.split(b" ", 2)[1]) if head else 0
    return status, head.decode("latin-1"), body


def request(port, method, path, headers=None, body=b"", host=None):
    h = {"Host": host or "127.0.0.1:%d" % port, "Connection": "close"}
    h.update(headers or {})
    if body:
        h.setdefault("Content-Length", str(len(body)))
    return ("%s %s HTTP/1.1\r\n%s\r\n\r\n" % (method, path, "\r\n".join("%s: %s" % kv for kv in h.items()))).encode("latin-1") + body


WS_UP = {"Upgrade": "websocket", "Connection": "Upgrade", "Sec-WebSocket-Key": "dGhlIHNhbXBsZSBub25jZQ==",
         "Sec-WebSocket-Version": "13"}
EVIL = "http://evil.example"


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


# ---------------------------------------------------------------------------------------------- #44
class TestOrigin(Base):
    def test_host_ok(self):
        for h in ("127.0.0.1:8080", "192.168.0.209:8080", "[::1]:8080", "localhost:8080", "localhost", "sbr3:8080",
                  "robot.local:8080", "pi.lan", "10.0.0.5"):
            self.assertTrue(host_ok(h), h)
        for h in ("evil.example", "evil.example:8080", "a.b.c.d.com:80", "", "1.2.3.4.example"):
            self.assertFalse(host_ok(h), h)
        self.assertTrue(host_ok("pi.example.org:8080", ["pi.example.org"]))
        self.assertTrue(host_ok("pi.example.org:8080", ["pi.example.org:8080"]))

    def test_check_origin_matrix(self):
        def chk(**h):
            return check_origin(SimpleNamespace(headers={k.lower().replace("_", "-"): v for k, v in h.items()}))
        host = "192.168.0.209:8080"
        # the Android WebView loads http://<robot>:8080/ -> same origin; tools and the app's native stop send no Origin
        self.assertIsNone(chk(host=host, origin="http://" + host))
        self.assertIsNone(chk(host=host, origin="http://192.168.0.209:8080/"))
        self.assertIsNone(chk(host=host))
        self.assertIsNone(chk(host=host, sec_fetch_site="same-origin"))
        self.assertIsNone(chk(host=host, sec_fetch_site="none"))
        self.assertIsNone(chk(host=host, referer="http://%s/#drive" % host))
        self.assertIsNone(chk())                                            # HTTP/1.0 tool: no Host, no Origin
        for bad in (dict(origin=EVIL), dict(origin="null"), dict(origin="http://192.168.0.209:9999"),
                    dict(origin="https://" + host.replace(":8080", "")), dict(sec_fetch_site="cross-site"),
                    dict(sec_fetch_site="same-site"), dict(referer=EVIL + "/page")):
            r = chk(host=host, **bad)
            self.assertEqual(r.status if r else None, 403, bad)
        self.assertEqual(chk(host="evil.example", origin="http://evil.example").status, 403)   # DNS rebinding
        self.assertEqual(chk(origin=EVIL).status, 403)                       # Origin but no Host: refuse
        self.assertIsNone(check_origin(SimpleNamespace(headers={"host": host, "origin": "http://tablet.example:3000"}),
                                       allow_origins=["http://tablet.example:3000"]))

    def test_over_the_wire(self):
        asyncio.run(self._wire())

    async def _wire(self):
        rig = await Rig(self.tmp, link=False).start()
        p, loop = rig.port, asyncio.get_running_loop()
        me = "http://127.0.0.1:%d" % p
        try:
            async def ws_status(**h):
                return (await raw(p, request(p, "GET", "/ws", dict(WS_UP, **h)), until_close=False))[0]

            self.assertEqual(await ws_status(), 101)                         # no Origin: tools
            self.assertEqual(await ws_status(Origin=me), 101)                # the page the bridge served
            self.assertEqual(await ws_status(Origin=EVIL), 403)              # any other site
            self.assertEqual(await ws_status(Origin="null"), 403)
            self.assertEqual(await ws_status(Origin="http://127.0.0.1:1"), 403)   # another port = another site
            self.assertEqual(await ws_status(**{"Sec-Fetch-Site": "cross-site"}), 403)
            r = await raw(p, request(p, "GET", "/ws", dict(WS_UP, Origin="http://evil.example"), host="evil.example"))
            self.assertEqual(r[0], 403)                                      # rebinding: Host is not the robot

            async def get(path, **h):
                return await raw(p, request(p, "GET", path, h))
            st, _, body = await get("/api/cmd?t=ping")
            self.assertEqual((st, json.loads(body)["t"]), (200, "pong"))     # old Kindle path / curl
            self.assertEqual((await get("/api/cmd?t=ping", Origin=me))[0], 200)
            self.assertEqual((await get("/api/cmd?t=ping", Referer=me + "/k.html"))[0], 200)
            self.assertEqual((await get("/api/cmd?t=ping", Origin=EVIL))[0], 403)
            self.assertEqual((await get("/api/cmd?t=ping", **{"Sec-Fetch-Site": "cross-site"}))[0], 403)   # <img src=...>
            self.assertEqual((await get("/api/cmd?t=set_mode&mode=1", Referer=EVIL + "/x"))[0], 403)
            self.assertEqual((await get("/api/state", Origin=EVIL))[0], 403)
            self.assertEqual((await get("/", Origin=EVIL))[0], 200)         # the static page itself is public

            def post(body, ctype="application/json", **h):
                hh = dict(h)
                if ctype:
                    hh["Content-Type"] = ctype
                return raw(p, request(p, "POST", "/api/cmd", hh, body))
            ping = b'{"t":"ping"}'
            self.assertEqual((await post(ping))[0], 200)
            self.assertEqual((await post(ping, "application/json; charset=utf-8"))[0], 200)
            self.assertEqual((await post(ping, Origin=me))[0], 200)
            self.assertEqual((await post(ping, "text/plain"))[0], 415)       # CORS "simple request": no preflight
            self.assertEqual((await post(ping, "application/x-www-form-urlencoded"))[0], 415)
            self.assertEqual((await post(ping, None))[0], 415)
            self.assertEqual((await post(ping, Origin=EVIL))[0], 403)
            self.assertEqual((await post(ping, "text/plain", Origin=EVIL))[0], 403)
            self.assertEqual((await raw(p, request(p, "POST", "/api/serial?release=1", {"Origin": EVIL})))[0], 403)
            self.assertFalse(rig.b.released_api)

            # tools/tune.py (runs on the Pi, curl-like) and the station's own page keep working
            spec = importlib.util.spec_from_file_location("sbr3_tune", TUNE_PY)
            tune = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(tune)
            tune.BRIDGE = me
            self.assertEqual(json.loads(await loop.run_in_executor(None, tune.api_cmd, {"t": "ping", "id": 7}))["id"], 7)
            self.assertIn("status", await loop.run_in_executor(None, tune.api_state))
        finally:
            await rig.stop()

    def test_relaxed(self):
        asyncio.run(self._relaxed())

    async def _relaxed(self):
        for kw, h, expect in (
                (dict(allow_any_origin=True), dict(Origin=EVIL), 200),
                (dict(allow_origins=["http://tablet.example:3000"]), dict(Origin="http://tablet.example:3000"), 200),
                (dict(allow_origins=["http://tablet.example:3000"]), dict(Origin=EVIL), 403),
                (dict(), dict(Origin=EVIL), 403)):
            rig = await Rig(self.tmp, link=False, **kw).start()
            try:
                self.assertEqual((await raw(rig.port, request(rig.port, "GET", "/api/cmd?t=ping", h)))[0], expect, (kw, h))
            finally:
                await rig.stop()
        rig = await Rig(self.tmp, link=False, allow_hosts=["pi.example.org"]).start()
        try:
            r = await raw(rig.port, request(rig.port, "GET", "/api/cmd?t=ping", host="pi.example.org:8080"))
            self.assertEqual(r[0], 200)
        finally:
            await rig.stop()


# ---------------------------------------------------------------------------------------------- #45
def make_big_log(path, mb):
    """A tlm-heavy session: exporting its range rows scans every line without a match."""
    tlm = '["tlm",12.345,' + ",".join(["1.2345"] * 24) + "]\n"
    block = (tlm * (4 * 1024 * 1024 // len(tlm))).encode()
    with open(path, "wb") as f:
        f.write(b'["schema",{"tlm":[],"rng":[]}]\n')
        for i in range(mb // 4):
            f.write(block)
            if i in (1, mb // 8):
                f.write(b'["rng",5.0,1,2,3]\n')
        f.write(b'["rng",9.0,4,5,6]\n')


class TestExportDoesNotStall(Base):
    def test_big_csv_export_keeps_the_loop_and_the_heartbeat_alive(self):
        asyncio.run(self._run())

    async def _run(self):
        rig = await Rig(self.tmp).start()
        loop = asyncio.get_running_loop()
        name = "sbr3-20200101-000000.jsonl"
        try:
            await rig.until(lambda: rig.b.tlm and rig.b.tlm["status"] & lp.TLM_PI_LINK, "robot heartbeat seen")
            make_big_log(os.path.join(rig.b.log.dir, name), 80)
            url = "http://127.0.0.1:%d" % rig.port
            lat, done, out = [], threading.Event(), {}

            def poll_status():                   # a browser / the status line asking while the export runs
                while not done.is_set():
                    t0 = time.perf_counter()
                    with urllib.request.urlopen(url + "/api/status", timeout=10) as r:
                        r.read()
                    lat.append(time.perf_counter() - t0)
                    time.sleep(0.02)

            def export():
                with urllib.request.urlopen("%s/api/logs/%s?csv=rng" % (url, name), timeout=60) as r:
                    out["csv"] = r.read().decode()
                done.set()
            gaps, pi_link_lost = [], []

            async def ticker():
                last = time.perf_counter()
                while not done.is_set():
                    await asyncio.sleep(0.005)
                    now = time.perf_counter()
                    gaps.append(now - last)
                    last = now
                    if rig.b.tlm and not rig.b.tlm["status"] & lp.TLM_PI_LINK:
                        pi_link_lost.append(now)
            poller = threading.Thread(target=poll_status, daemon=True)
            poller.start()
            tk = asyncio.ensure_future(ticker())
            await loop.run_in_executor(None, export)
            await tk
            poller.join(5)
            self.assertEqual(out["csv"].count("\n"), 4)            # header + 3 range rows
            self.assertGreater(len(lat), 3, "status was polled during the export")
            self.assertLess(max(gaps), 0.1, "event loop blocked for %d ms" % (max(gaps) * 1000))
            self.assertLess(max(lat), 0.15, "/api/status took %d ms during the export" % (max(lat) * 1000))
            self.assertFalse(pi_link_lost, "robot dropped the Pi link (heartbeat stalled)")
        finally:
            await rig.stop()

    def test_stall_is_reported(self):
        asyncio.run(self._warn())

    async def _warn(self):
        rig = await Rig(self.tmp, link=False).start()
        try:
            await asyncio.sleep(0.2)
            time.sleep(0.5)                                          # block the loop on purpose
            await rig.until(lambda: any("stalled" in x["text"] for x in rig.b.logs), "stall warning in the log")
        finally:
            await rig.stop()

    def test_csv_stream_and_sync_helpers(self):
        # the chunk helper joins whatever the generator yields and ends with b""
        it = iter(["a,b\n", "1,2\n"])
        self.assertEqual(httpws._next_chunk(it), b"a,b\n1,2\n")
        self.assertEqual(httpws._next_chunk(it), b"")


# ---------------------------------------------------------------------------------------------- #46
class TestTlmHz(Base):
    def _setup_hz(self, rig, ws, value):
        pid = next(p["id"] for p in rig.b.params.values() if p["name"] == "tlm.hz")
        ws.send(json.dumps({"t": "set_param", "id": pid, "value": value}))

    def test_restored_when_tune_closes(self):
        asyncio.run(self._restore())

    async def _restore(self):
        rig = await Rig(self.tmp).start()
        try:
            rig.b.hz_grace_s = 0.3
            await rig.until(lambda: len(rig.b.params) > 40, "params")
            self.assertEqual(rig.robot.P("tlm.hz"), 100)
            ws = await rig.ws()
            ws.send(json.dumps({"t": "sub", "tune": True}))
            self._setup_hz(rig, ws, 400)
            await rig.until(lambda: rig.robot.P("tlm.hz") == 400 and rig.b.status()["tlm_boost"], "400 Hz on")
            self.assertEqual(rig.b.status()["tlm_base"], 100)
            await asyncio.sleep(0.8)                                 # still subscribed: left alone
            self.assertEqual(rig.robot.P("tlm.hz"), 400)
            ws.close()                                               # tab closed / phone slept
            await rig.until(lambda: rig.robot.P("tlm.hz") == 100, "tlm.hz back to 100")
            self.assertFalse(rig.b.status()["tlm_boost"])
            await rig.until(lambda: rig.b.status()["tlm_hz"] < 150, "telemetry back near 100 Hz")
        finally:
            await rig.stop()

    def test_not_restored_when_saved_or_set_by_a_tool(self):
        asyncio.run(self._keep())

    async def _keep(self):
        rig = await Rig(self.tmp).start()
        try:
            rig.b.hz_grace_s = 0.2
            await rig.until(lambda: len(rig.b.params) > 40, "params")
            # the user saved with 400 on purpose: that is the robot's default now
            ws = await rig.ws()
            ws.send(json.dumps({"t": "sub", "tune": True}))
            self._setup_hz(rig, ws, 400)
            await rig.until(lambda: rig.b.hz_base == 100, "boost noted")
            ws.send(json.dumps({"t": "cfg", "action": "save"}))
            await rig.until(lambda: rig.b.hz_base is None, "save adopts the rate")
            ws.close()
            await asyncio.sleep(0.8)
            self.assertEqual(rig.robot.P("tlm.hz"), 400)
            # tools/tune.py style: HTTP set_param is final, never "temporary"
            loop = asyncio.get_running_loop()
            pid = next(p["id"] for p in rig.b.params.values() if p["name"] == "tlm.hz")

            def post(m):
                req = urllib.request.Request("http://127.0.0.1:%d/api/cmd" % rig.port, data=json.dumps(m).encode(),
                                             headers={"Content-Type": "application/json"})
                return urllib.request.urlopen(req, timeout=3).read()
            await loop.run_in_executor(None, post, {"t": "set_param", "id": pid, "value": 100})
            await loop.run_in_executor(None, post, {"t": "set_param", "id": pid, "value": 300})
            await asyncio.sleep(0.8)
            self.assertEqual(rig.robot.P("tlm.hz"), 300)
            # a browser that never opened Tune raises it (a script): left alone too
            ws = await rig.ws()
            self._setup_hz(rig, ws, 400)
            await asyncio.sleep(0.8)
            self.assertEqual(rig.robot.P("tlm.hz"), 400)
            ws.close()
        finally:
            await rig.stop()

    def test_reopening_tune_cancels_the_restore(self):
        asyncio.run(self._reopen())

    async def _reopen(self):
        rig = await Rig(self.tmp).start()
        try:
            rig.b.hz_grace_s = 0.6
            await rig.until(lambda: len(rig.b.params) > 40, "params")
            ws = await rig.ws()
            ws.send(json.dumps({"t": "sub", "tune": True}))
            self._setup_hz(rig, ws, 400)
            await rig.until(lambda: rig.robot.P("tlm.hz") == 400, "400")
            ws.send(json.dumps({"t": "sub", "tune": False}))     # switched tab away
            await asyncio.sleep(0.25)
            ws.send(json.dumps({"t": "sub", "tune": True}))      # ... and back within the grace period
            await asyncio.sleep(1.0)
            self.assertEqual(rig.robot.P("tlm.hz"), 400)
            ws.send(json.dumps({"t": "sub", "tune": False}))
            await rig.until(lambda: rig.robot.P("tlm.hz") == 100, "restored after leaving for good")
            ws.close()
        finally:
            await rig.stop()


# ---------------------------------------------------------------------------------------------- #47
FUZZ = [
    {"t": "goto", "points": "abc"}, {"t": "goto", "points": [[1]]}, {"t": "goto", "points": [[None, None]]},
    {"t": "goto", "points": [["a", "b"]]}, {"t": "goto", "points": 5}, {"t": "goto", "points": [[1, 2]] * 400},
    {"t": "set_mode"}, {"t": "set_mode", "mode": "x"}, {"t": "set_mode", "mode": [1]}, {"t": "set_mode", "mode": 1e308},
    {"t": "set_param", "id": "x"}, {"t": "set_param", "id": 1, "value": None}, {"t": "set_param", "id": [1]},
    {"t": "event", "event": "a"}, {"t": "event"}, {"t": "event", "event": None},
    {"t": "cfg", "action": 5}, {"t": "cfg", "action": ["save"]}, {"t": "cfg"},
    {"t": "preset"}, {"t": "preset", "action": "save", "name": 5}, {"t": "preset", "action": "save", "name": "../../x"},
    {"t": "preset", "action": "apply", "name": None}, {"t": "preset", "action": "apply", "name": ["a"]},
    {"t": "preset", "action": "plan", "name": {}}, {"t": "preset", "action": "diff", "a": 1, "b": []},
    {"t": "score", "action": "last", "seconds": "x"}, {"t": "score", "action": "last", "seconds": None},
    {"t": "score", "action": 5}, {"t": "score"},
    {"t": "motor"}, {"t": "motor", "action": "start"}, {"t": "motor", "action": "start", "test": 5},
    {"t": "motor", "action": "fit", "session": 5}, {"t": "motor", "action": "fit", "session": "../../etc/passwd"},
    {"t": "motor", "action": "apply", "params": "x"}, {"t": "motor", "action": "apply", "params": [1]},
    {"t": "motor", "action": "apply", "params": {"a": "b"}},
    {"t": "map"}, {"t": "map", "action": "save", "name": 5}, {"t": "map", "action": "load", "name": None},
    {"t": "map", "action": "load", "name": "../../x"}, {"t": "map", "action": "delete", "name": ["x"]},
    {"t": "map", "action": "set", "settings": "x"}, {"t": "map", "action": "set", "settings": 5},
    {"t": "map", "action": "set", "key": "x", "value": "y"},
    {"t": "plan"}, {"t": "plan", "action": "goto"}, {"t": "plan", "action": "goto", "points": "x"},
    {"t": "plan", "action": "goto", "points": [[1e308, 1e308]]}, {"t": "plan", "action": "goto", "points": [[None]]},
    {"t": "plan", "action": "set", "settings": 5}, {"t": "plan", "action": "set", "settings": "x"},
    {"t": "plan", "action": "set", "settings": [1]}, {"t": "plan", "action": "set", "key": 5},
    {"t": "plan", "action": "explore", "x": 1}, {"t": "cmd_vel", "speed": "x", "turn": [1]},
    {"t": "cmd_vel", "speed": float("nan")}, {"t": "sub"}, {"t": "hi", "name": 5}, {"t": "hi", "name": {"a": 1}},
    {"t": "ping"}, {"t": 5}, {"t": None}, {"t": ["a"]}, {"t": {"a": 1}}, [], 5, "x", None, True, [{"t": "ping"}],
]


class TestBadInput(Base):
    def test_fuzz_keeps_every_socket_open(self):
        asyncio.run(self._fuzz())

    async def _fuzz(self):
        rig = await Rig(self.tmp).start()
        try:
            await rig.until(lambda: len(rig.b.params) > 40 and rig.b.tlm, "robot")
            ws = await rig.ws()
            texts = [json.dumps(c) for c in FUZZ] + ["{bad json", "", "[" * 30000, "null", '{"t":"ping"} x', "é€"]
            for i, text in enumerate(texts):
                ws.send(text)
                ws.send(json.dumps({"t": "ping", "id": "p%d" % i}))
                end = time.monotonic() + 8
                while True:
                    m = await asyncio.wait_for(ws.recv(), max(0.1, end - time.monotonic()))
                    self.assertIsNotNone(m, "socket closed after %r" % text[:80])
                    if '"p%d"' % i in m and '"pong"' in m:
                        break
            self.assertEqual(len(rig.b.clients), 1)
            self.assertFalse(any(t.done() for t in rig.tasks), "a bridge task died")
        finally:
            await rig.stop()

    def test_http_cmd_never_500(self):
        asyncio.run(self._http())

    async def _http(self):
        rig = await Rig(self.tmp, link=False).start()
        try:
            for body in (b'{"t":"map","action":"set","settings":"x"}', b'{"t":"plan","action":"set","settings":5}',
                         b'[1,2]', b'5', b'{"t":["a"]}', b'{"t": "goto", "points": 5}', b'{bad', b"[" * 30000):
                st, _, out = await raw(rig.port, request(rig.port, "POST", "/api/cmd", {"Content-Type": "application/json"}, body))
                self.assertIn(st, (200, 400), body[:40])
                self.assertIn("t", json.loads(out))
        finally:
            await rig.stop()

    def test_http_framing(self):
        asyncio.run(self._framing())

    async def _framing(self):
        rig = await Rig(self.tmp, link=False).start()
        p = rig.port
        try:
            js = {"Content-Type": "application/json"}
            for cl in ("abc", "-5", "1e3", "²", "12 34"):
                h = dict(js, **{"Content-Length": cl})
                st = (await raw(p, request(p, "POST", "/api/cmd", h)))[0]
                self.assertEqual(st, 400, cl)
            self.assertEqual((await raw(p, request(p, "POST", "/api/cmd", dict(js, **{"Content-Length": "99999999"}))))[0], 413)
            self.assertEqual((await raw(p, request(p, "POST", "/api/cmd", dict(js, **{"Transfer-Encoding": "chunked"}))))[0], 400)
            many = {"X-%d" % i: "v" for i in range(httpws.MAX_HEADER_LINES + 10)}
            self.assertEqual((await raw(p, request(p, "GET", "/api/status", many)))[0], 431)
            self.assertEqual((await raw(p, request(p, "GET", "/api/status", {"X-Big": "v" * 70000})))[0], 431)
            self.assertEqual((await raw(p, b"GET /" + b"a" * 70000 + b" HTTP/1.1\r\n\r\n"))[0], 431)
            self.assertEqual((await raw(p, request(p, "GET", "/api/status")))[0], 200)       # still serving
            # slow-loris: a request that never finishes is dropped after READ_TIMEOUT_S, not held forever
            with mock.patch.object(httpws, "READ_TIMEOUT_S", 0.3):
                for part in (b"GET /api/status HTTP/1.1\r\nHost: x\r\n",                      # headers never end
                             b"POST /api/cmd HTTP/1.1\r\nContent-Length: 50\r\n\r\n{\"t\""):  # body never arrives
                    t0 = time.monotonic()
                    r, w = await asyncio.open_connection("127.0.0.1", p)
                    w.write(part)
                    await w.drain()
                    self.assertEqual(await asyncio.wait_for(r.read(100), 3), b"")
                    self.assertLess(time.monotonic() - t0, 2.0)
                    w.close()
        finally:
            await rig.stop()

    def test_static_dir_sibling(self):
        web, old = os.path.join(self.tmp, "web"), os.path.join(self.tmp, "web-old")
        for d, f in ((web, "index.html"), (old, "secret.txt")):
            os.makedirs(d)
            with open(os.path.join(d, f), "w") as fh:
                fh.write("x")
        srv = HttpServer(web, [])
        self.assertEqual(srv._static("/index.html").status, 200)
        self.assertEqual(srv._static("/").status, 200)
        self.assertEqual(srv._static("/../web-old/secret.txt").status, 403)
        self.assertEqual(srv._static("/../web-old").status, 403)
        self.assertEqual(srv._static("/../../etc/passwd").status, 403)


class TestLinkTaskSurvives(Base):
    def test_unwritable_log_dir_does_not_end_the_bridge(self):
        asyncio.run(self._run())

    async def _run(self):
        rig = Rig(self.tmp)
        with mock.patch.object(bridge_mod, "LOG_RETRY_S", 0.3):
            await rig.start()
            try:
                good = rig.b.log.dir
                await rig.until(lambda: rig.b.link_state == "connected", "connected")
                # the disk fills up / the dir goes read-only: a path that cannot be a directory
                blocker = os.path.join(self.tmp, "blocker")
                with open(blocker, "w") as f:
                    f.write("x")
                rig.b.log.stop()
                rig.b.log.dir = os.path.join(blocker, "logs")
                rig.b.link.writer.close()                              # robot link drops and comes back
                await rig.until(lambda: rig.b.link_state != "connected", "link lost")
                await rig.until(lambda: rig.b.link_state == "connected" and rig.b.log_failed, "reconnected without logging")
                self.assertIsNone(rig.b.log.f)
                self.assertTrue(rig.b.status()["log_error"])
                self.assertFalse(any(t.done() for t in rig.tasks), "link_task / tick_task ended")
                await rig.until(lambda: rig.b.tlm_seq > 0, "telemetry flows without a log")
                self.assertTrue(any("cannot write the session log" in x["text"] for x in rig.b.logs))
                rig.b.log.dir = good                                   # the disk is back: logging resumes by itself
                await rig.until(lambda: rig.b.log.f is not None, "log retried")
                self.assertEqual(rig.b.status()["log_error"], "")
                # an exception in the post-connect step is reported, not fatal
                with mock.patch.object(rig.b.tools, "on_session", side_effect=RuntimeError("boom")):
                    rig.b.link.writer.close()
                    await rig.until(lambda: rig.b.link_state != "connected", "link lost 2")
                    await rig.until(lambda: rig.b.link_state == "connected", "reconnected 2")
                self.assertFalse(any(t.done() for t in rig.tasks))
            finally:
                await rig.stop()

    def test_task_loop_survives_an_exception(self):
        asyncio.run(self._loop())

    async def _loop(self):
        rig = await Rig(self.tmp, link=False).start()
        try:
            calls = []

            async def boom():
                calls.append(1)
                raise OSError("disk exploded")
            with mock.patch.object(rig.b, "_link_step", boom):
                await asyncio.sleep(1.0)
            self.assertGreaterEqual(len(calls), 2)
            self.assertFalse(rig.tasks[0].done())
        finally:
            await rig.stop()


class FakeClient:
    def __init__(self, queued=0, idle=0.0, since_ping=0.0):
        now = time.monotonic()
        self.closed, self.peer = False, ("10.0.0.9", 1)
        self.queued, self.last_rx, self.last_ping = queued, now - idle, now - since_ping
        self.pings, self.aborted = 0, False

    def buffered(self):
        return self.queued

    def ping(self):
        self.pings += 1
        self.last_ping = time.monotonic()

    def abort(self):
        self.aborted = self.closed = True

    def send(self, text):
        return True


class TestHalfOpenClients(Base):
    def _bridge(self):
        return Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True,
                      map_dir=os.path.join(self.tmp, "maps"), preset_dir=os.path.join(self.tmp, "p"))

    def test_reaping(self):
        b = self._bridge()
        fresh, hog, idle, due = FakeClient(), FakeClient(queued=bridge_mod.WS_DROP_BYTES + 1), \
            FakeClient(idle=bridge_mod.WS_IDLE_S + 1), FakeClient(since_ping=bridge_mod.WS_PING_S + 1)
        b.clients.update([fresh, hog, idle, due])
        b._reap_clients(time.monotonic())
        self.assertTrue(hog.aborted and idle.aborted)
        self.assertFalse(fresh.aborted or fresh.pings)
        self.assertEqual((due.pings, due.aborted), (1, False))
        b._reap_clients(time.monotonic())
        self.assertEqual({fresh, due}, set(b.clients))               # the dropped ones are forgotten
        self.assertEqual(due.pings, 1)                               # not pinged again straight away

    def test_real_socket_ping_pong_and_drop(self):
        asyncio.run(self._real())

    async def _real(self):
        rig = await Rig(self.tmp, link=False).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: len(rig.b.clients) == 1, "client")
            c = next(iter(rig.b.clients))
            c.last_ping = c.last_rx = time.monotonic() - (bridge_mod.WS_PING_S + 1)
            before = c.last_rx
            rig.b._reap_clients(time.monotonic())                    # sends a ping
            await rig.until(lambda: c.last_ping > before, "ping sent")
            end = time.monotonic() + 1.5
            while time.monotonic() < end and c.last_rx <= before:     # the client answers with a pong (WsClient.recv)
                try:
                    await asyncio.wait_for(ws.recv(), 0.3)
                except asyncio.TimeoutError:
                    pass
            await rig.until(lambda: c.last_rx > before, "pong refreshed last_rx")
            c.last_rx = time.monotonic() - (bridge_mod.WS_IDLE_S + 5)   # now it has been silent for too long
            rig.b._reap_clients(time.monotonic())
            await rig.until(lambda: not rig.b.clients, "silent client dropped")
            ws.close()
        finally:
            await rig.stop()


# ---------------------------------------------------------------------------------------------- #48
class TestNoLinkCommands(Base):
    def _bridge(self):
        b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True,
                   map_dir=os.path.join(self.tmp, "maps"), preset_dir=os.path.join(self.tmp, "p"))
        b.params = {7: {"id": 7, "name": "bal.kp", "value": 250.0, "min": 0.0, "max": 1000.0},
                    9: {"id": 9, "name": "cfg.save", "value": 0.0, "min": 0.0, "max": 1.0}}
        return b

    def test_commands_that_need_the_robot_are_refused(self):
        b = self._bridge()
        sent = []
        b.send = lambda f: sent.append(f) or False                  # (what send() does with no link)
        for m in ({"t": "set_mode", "mode": 3}, {"t": "set_param", "id": 7, "value": 300}, {"t": "cfg", "action": "save"},
                  {"t": "goto", "points": [[1, 2]], "go": True}, {"t": "event", "event": 1}, {"t": "get_params"},
                  {"t": "preset", "action": "apply", "name": "x"}, {"t": "motor", "action": "start"},
                  {"t": "plan", "action": "goto", "points": [[1, 2]]}, {"t": "plan", "action": "explore"}):
            r = b.handle_command(m, "test")
            self.assertEqual((r or {}).get("code"), "no_link", m)
            self.assertEqual(r["t"], "error")
            self.assertIn("not connected", r["text"])
        self.assertEqual(sent, [], "nothing is even attempted")
        self.assertEqual(b.params[7]["value"], 250.0, "no optimistic value")
        self.assertFalse(b.params_dirty)
        self.assertEqual(b.waypoints, [])
        self.assertFalse(any("mode ->" in x["text"] for x in b.logs))
        # commands that do not touch the robot still work
        self.assertEqual(b.handle_command({"t": "ping", "id": 1}, "t")["t"], "pong")
        self.assertIsNone(b.handle_command({"t": "plan", "action": "stop"}, "t"))
        self.assertIsNone(b.handle_command({"t": "plan", "action": "set", "settings": {"margin_mm": 100}}, "t"))
        self.assertIsNone(b.handle_command({"t": "cmd_vel", "speed": 0, "turn": 0}, "t"))
        self.assertEqual(b.handle_command({"t": "preset", "action": "list"}, "t")["t"], "presets")
        # STOP: the latch / planner stop still happen locally, but the user is told it may not have arrived
        r = b.handle_command({"t": "estop"}, "test")
        self.assertTrue(b.estop)
        self.assertEqual(r["code"], "no_link")
        self.assertIn("STOP", r["text"])

    def test_with_a_link_they_go_through(self):
        asyncio.run(self._linked())

    async def _linked(self):
        rig = await Rig(self.tmp).start()
        try:
            await rig.until(lambda: len(rig.b.params) > 40, "params")
            pid = next(p["id"] for p in rig.b.params.values() if p["name"] == "bal.kp")
            self.assertIsNone(rig.b.handle_command({"t": "set_param", "id": pid, "value": 260}, "t"))
            self.assertIsNone(rig.b.handle_command({"t": "estop"}, "t"))
        finally:
            await rig.stop()

    def test_link_goes_down_mid_session(self):
        asyncio.run(self._down())

    async def _down(self):
        rig = await Rig(self.tmp).start()
        try:
            await rig.until(lambda: len(rig.b.params) > 40 and rig.b.tlm, "robot")
            pid = next(p["id"] for p in rig.b.params.values() if p["name"] == "bal.kp")
            ws = await rig.ws()
            rig.b.released_api = True                                # flashing: the bridge lets go of the port
            await rig.until(lambda: rig.b.link_state == "released", "released")
            ws.send(json.dumps({"t": "set_param", "id": pid, "value": 321}))
            end, err, params = time.monotonic() + 5, None, None
            while time.monotonic() < end and not (err and params):
                m = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if m["t"] == "error" and m.get("code") == "no_link":
                    err = m
                if err and m["t"] == "params":                       # the page gets the real values back
                    params = m
            self.assertTrue(err and params)
            self.assertEqual(next(p for p in params["list"] if p["id"] == pid)["value"], 250.0)
            self.assertEqual(rig.b.params[pid]["value"], 250.0)
            ws.close()
        finally:
            await rig.stop()


if __name__ == "__main__":
    unittest.main()
