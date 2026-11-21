"""Ground-station bridge: Teensy link <-> browsers (WebSocket JSON + HTTP polling), session logs."""
import asyncio
import collections
import json
import math
import os
import tempfile
import time

from . import link_protocol as lp
from .httpws import HttpServer, Response
from .session_log import SessionLog, to_csv
from .transport import LinkError, open_link

BRIDGE_VERSION = "sbr3-bridge 1"
# NavStatusFrame.state is robot-defined; these are the names the simulator uses (and a
# suggestion for the firmware). Unknown values show as "state N" in the UI.
NAV_STATE_NAMES = ["idle", "turning", "driving", "blocked", "arrived"]
TICK_S = 0.04            # 25 Hz: browser telemetry rate and cmd_vel resend rate
HEARTBEAT_S = 0.1        # 10 Hz MSG_HEARTBEAT
CMD_TIMEOUT_S = lp.LINK_CMD_VEL_TIMEOUT_MS / 1000.0
ROBOT_SILENT_S = 1.0     # no telemetry for this long -> robot shown as not responding
RANGE_RING = 4000
RANGE_ON_CONNECT = 2500
TILT_DEG = 10.0          # range points taken while pitched more than this are flagged
WS_SLOW_BYTES = 256 * 1024
POLLER_TIMEOUT_S = 3.0


def _now():
    return time.monotonic()


def _num(v, default=0.0, lo=-1e9, hi=1e9):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(v):
        return default
    return max(lo, min(hi, v))


def _sig(v):
    """float32 -> short decimal (4.630000114 -> 4.63)."""
    return float("%.6g" % v)


def _round_dict(d):
    return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()}


class Bridge:
    def __init__(self, port, web_dir, log_dir, http_host="0.0.0.0", http_port=8080, baud=115200,
                 release_file=None, log_max_mb=1000, quiet=False):
        self.port = port
        self.baud = baud
        self.web_dir = web_dir
        self.http_host, self.http_port = http_host, http_port
        self.release_file = release_file or os.path.join(tempfile.gettempdir(), "sbr3-bridge.release")
        self.quiet = quiet
        self.log = SessionLog(log_dir, log_max_mb)
        self.t_start = _now()

        self.link = None
        self.link_state = "waiting"     # waiting | connected | released
        self.link_error = ""
        self.released_api = False
        self.parser = lp.FrameParser()

        self.hello = None
        self.params = {}                # id -> dict
        self.params_dirty = False
        self.last_get_params = -10.0
        self.tlm = None
        self.tlm_seq = 0
        self.tlm_sent_seq = 0
        self.tlm_time = -10.0
        self.tlm_count = 0
        self.tlm_hz = 0.0
        self.nav = None
        self.ranges = collections.deque(maxlen=RANGE_RING)  # [i, hx, hy, rx, ry, valid, tilt, t_ms]
        self.range_next = 0
        self.range_sent = 0
        self.logs = collections.deque(maxlen=200)          # {"i","t","src","text"}
        self.log_next = 0
        self.waypoints = []                                 # what the station last sent (x, y)
        self.range_listeners = []                           # callables(range dict) e.g. mapping (#12)

        self.cmd = (0.0, 0.0)
        self.cmd_time = -10.0
        self.cmd_active = False
        self.estop = False
        self.estop_time = -10.0
        self.estop_repeats = []

        self.clients = set()
        self.pollers = {}               # ip -> last poll time (Kindle polling page, #11)
        self.status_cache = None
        self.last_hb = 0.0
        self.last_status = 0.0
        self.last_rate = _now()
        self.client_ids = 0

    # ------------------------------------------------------------------ util
    def say(self, text):
        """Bridge-side log line: shown in the UI and written to the session log."""
        if not self.quiet:
            print("[bridge]", text, flush=True)
        self._add_log("bridge", text)
        self.log.record("bridge", text)

    def _add_log(self, src, text):
        item = {"i": self.log_next, "t": round(_now() - self.t_start, 2), "src": src, "text": text}
        self.log_next += 1
        self.logs.append(item)
        self.broadcast(json.dumps(dict(t="log", **{k: v for k, v in item.items() if k != "t"}, ts=item["t"])))

    def pi_ms(self):
        return int((_now() - self.t_start) * 1000)

    def client_count(self):
        now = _now()
        return len(self.clients) + sum(1 for t in self.pollers.values() if now - t < POLLER_TIMEOUT_S)

    # ------------------------------------------------------------------ link
    def released(self):
        return self.released_api or os.path.exists(self.release_file)

    async def link_task(self):
        while True:
            if self.released():
                if self.link:
                    self.say("serial port released (flashing?)")
                    self.link.close()
                    self._on_lost(quiet=True)
                self._set_link_state("released")
            elif not self.link:
                try:
                    self.link = await open_link(self.port, self._on_data, self._on_lost, self.baud)
                except LinkError as e:
                    if str(e) != self.link_error:
                        self.link_error = str(e)
                        self.say("waiting for %s: %s" % (self.port, e))
                    self._set_link_state("waiting")
                else:
                    self.link_error = ""
                    self.parser = lp.FrameParser()
                    self._set_link_state("connected")
                    self.log.start()
                    self.say("connected to %s, session %s" % (self.port, self.log.name))
                    self.send(lp.encode_get_params())
                    self.last_get_params = _now()
            await asyncio.sleep(1.0)

    def _set_link_state(self, s):
        if s != self.link_state:
            self.link_state = s
            self.push_status()

    def _on_lost(self, quiet=False):
        if self.link is None:
            return
        self.link = None
        self.log.stop()
        self.tlm_time = -10.0
        if not quiet:
            self.say("lost %s (Teensy reset/unplugged?) - reconnecting" % self.port)
        self._set_link_state("released" if self.released() else "waiting")

    def send(self, frame):
        if self.link:
            return self.link.write(frame)
        return False

    def _on_data(self, data):
        for t, payload in self.parser.feed(data):
            try:
                self._on_frame(t, payload)
            except ValueError as e:  # short payload etc.
                self.parser.crc_errors += 1
                if not self.quiet:
                    print("[bridge] bad frame", lp.MSG_NAMES.get(t, t), e)

    def _on_frame(self, t, p):
        if t == lp.MSG_TELEMETRY:
            d = lp.decode_telemetry(p)
            self.tlm = d
            self.tlm_seq += 1
            self.tlm_count += 1
            self.tlm_time = _now()
            self.log.tlm(d)
            if self.estop and d["mode"] != lp.MODE_IDLE and _now() - self.estop_time > 0.5:
                self.estop = False
                self.push_status()
        elif t == lp.MSG_RANGE:
            d = lp.decode_range(p)
            self.log.rng(d)
            self._add_range(d)
        elif t == lp.MSG_LOG:
            text = lp.decode_log(p)
            self.log.record("log", text)
            self._add_log("robot", text)
        elif t == lp.MSG_PARAM_INFO:
            d = lp.decode_param_info(p)
            for k in ("value", "min", "max"):
                d[k] = _sig(d[k])
            self.params[d["id"]] = d
            self.params_dirty = True
            self.log.record("param", d["id"], d["name"], d["value"], d["min"], d["max"])
        elif t == lp.MSG_NAV_STATUS:
            d = _round_dict(lp.decode_nav_status(p))
            d.pop("reserved", None)
            self.nav = d
            self.log.record("nav", d)
            self.broadcast(json.dumps(dict(t="nav", **d)))
        elif t == lp.MSG_HELLO:
            d = lp.decode_hello(p)
            changed = self.hello is None or d["firmware"] != self.hello.get("firmware")
            self.hello = d
            self.log.record("hello", d)
            if changed:
                self.params = {}
                self.params_dirty = True
                self.say("robot: firmware '%s', link v%d, radio v%d, %d params" % (
                    d["firmware"], d["link_version"], d["radio_version"], d["param_count"]))
                if d["link_version"] != lp.LINK_PROTOCOL_VERSION:
                    self.say("WARNING: robot link protocol v%d, bridge speaks v%d" % (
                        d["link_version"], lp.LINK_PROTOCOL_VERSION))
            self.broadcast(json.dumps({"t": "robot", "robot": d}))
            # HELLO comes at boot (ask for params) and in reply to GET_PARAMS (don't loop)
            if _now() - self.last_get_params > 2.0:
                self.send(lp.encode_get_params())
                self.last_get_params = _now()

    def _add_range(self, d):
        r = d["range_mm"]
        a = math.radians(d["heading_deg"] + d["sensor_angle_deg"])
        rr = r if (d["valid"] and r < 8000) else 2000
        hx = d["x_mm"] + rr * math.cos(a)
        hy = d["y_mm"] + rr * math.sin(a)
        tilt = 1 if abs(d["pitch_deg"]) > TILT_DEG else 0
        self.ranges.append([self.range_next, int(hx), int(hy), int(d["x_mm"]), int(d["y_mm"]),
                            int(d["valid"]), tilt, self.pi_ms()])
        self.range_next += 1
        for f in self.range_listeners:
            try:
                f(d)
            except Exception as e:
                print("[bridge] range listener error", e)

    def ranges_since(self, since, limit):
        if not self.ranges:
            return []
        first = self.ranges[0][0]
        start = max(since, first, self.range_next - limit)
        return list(self.ranges)[start - first:] if start < self.range_next else []

    # ------------------------------------------------------------------ periodic
    async def tick_task(self):
        while True:
            await asyncio.sleep(TICK_S)
            now = _now()
            try:
                self._tick(now)
            except Exception:
                import traceback
                traceback.print_exc()

    def _tick(self, now):
        if self.link:
            if now - self.last_hb >= HEARTBEAT_S - 0.005:
                self.last_hb = now
                self.send(lp.encode_heartbeat(self.pi_ms(), self.client_count()))
            # safety: only forward cmd_vel while a client keeps sending it
            if self.cmd_active:
                if now - self.cmd_time > CMD_TIMEOUT_S:
                    self.cmd_active = False
                    self.send(lp.encode_cmd_vel(0.0, 0.0))  # one explicit stop, then silence
                    self.push_status()
                else:
                    self.send(lp.encode_cmd_vel(*self.cmd))
            while self.estop_repeats and now >= self.estop_repeats[0]:
                self.estop_repeats.pop(0)
                self.send(lp.encode_set_mode(lp.MODE_IDLE))
        # browsers: latest telemetry, new range points, params
        if self.tlm_seq != self.tlm_sent_seq and self.tlm:
            self.tlm_sent_seq = self.tlm_seq
            d = _round_dict(self.tlm)
            d["t"] = "tlm"
            d["pt"] = self.pi_ms()
            self.broadcast(json.dumps(d, separators=(",", ":")), droppable=True)
        if self.range_next != self.range_sent:
            pts = self.ranges_since(self.range_sent, 500)
            self.range_sent = self.range_next
            if pts:
                self.broadcast(json.dumps({"t": "ranges", "pts": pts}, separators=(",", ":")), droppable=True)
        if self.params_dirty:
            self.params_dirty = False
            self.broadcast(self.params_msg())
        if now - self.last_status >= 1.0:
            dt = now - self.last_rate
            self.tlm_hz = self.tlm_count / dt if dt > 0 else 0.0
            self.tlm_count = 0
            self.last_rate = now
            self.push_status()
            self.log.flush()

    def status(self):
        now = _now()
        return {
            "t": "status", "link": self.link_state, "port": self.port, "link_error": self.link_error,
            "robot_alive": now - self.tlm_time < ROBOT_SILENT_S, "tlm_hz": round(self.tlm_hz, 1),
            "crc_errors": self.parser.crc_errors, "frames": self.parser.frames_ok,
            "clients": self.client_count(), "session": self.log.name if self.log.f else None,
            "estop": self.estop, "cmd_active": self.cmd_active,
            "uptime_s": int(now - self.t_start), "release_file": self.release_file,
        }

    def push_status(self):
        self.last_status = _now()
        self.broadcast(json.dumps(self.status()))

    def params_msg(self):
        return json.dumps({"t": "params", "list": sorted(self.params.values(), key=lambda d: d["id"])})

    def broadcast(self, text, droppable=False):
        for c in list(self.clients):
            if c.closed:
                self.clients.discard(c)
                continue
            if droppable and c.buffered() > WS_SLOW_BYTES:
                continue  # slow client: skip telemetry, keep commands/status
            c.send(text)

    # ------------------------------------------------------------------ commands
    def handle_command(self, m, who="?"):
        """One client command (dict). Returns an optional reply dict."""
        t = m.get("t")
        if t == "cmd_vel":
            self.cmd = (_num(m.get("speed"), 0, -3000, 3000), _num(m.get("turn"), 0, -1000, 1000))
            self.cmd_time = _now()
            if not self.cmd_active:
                self.cmd_active = True
                self.push_status()
            self.send(lp.encode_cmd_vel(*self.cmd))
            return None
        if t == "estop":
            self.do_estop(who)
            return None
        self.log.record("cmd", dict(m, who=who))
        if t == "set_mode":
            mode = int(_num(m.get("mode"), -1))
            if not 0 <= mode < lp.MODE_COUNT:
                return {"t": "error", "text": "bad mode"}
            self.send(lp.encode_set_mode(mode))
            if mode != lp.MODE_IDLE:
                self.estop = False
            self.say("%s: mode -> %s" % (who, lp.MODE_NAMES[mode]))
        elif t == "set_param":
            pid = int(_num(m.get("id"), -1))
            if pid not in self.params:
                return {"t": "error", "text": "unknown param %s" % pid}
            p = self.params[pid]
            v = _num(m.get("value"), p["value"], p["min"], p["max"])
            self.send(lp.encode_set_param(pid, v))
            p["value"] = v  # optimistic; a ParamInfo echo from the robot overrides it
            self.params_dirty = True
        elif t == "get_params":
            self.send(lp.encode_get_params())
            self.last_get_params = _now()
        elif t == "goto":
            pts = [(_num(p[0]), _num(p[1])) for p in (m.get("points") or [])
                   if isinstance(p, (list, tuple)) and len(p) >= 2][:255]
            replace = bool(m.get("replace", True))
            for f in lp.goto_frames(pts, replace):
                self.send(f)
            self.waypoints = (pts if replace else self.waypoints + pts)
            self.broadcast(json.dumps({"t": "waypoints", "points": self.waypoints}))
            if m.get("go"):
                self.send(lp.encode_set_mode(lp.MODE_GOTO))
                self.estop = False
            self.say("%s: %d waypoint(s)%s" % (who, len(pts), " -> GOTO" if m.get("go") else ""))
        elif t == "event":
            ev = int(_num(m.get("event"), -1))
            if not 0 < ev < lp.EVT_COUNT:
                return {"t": "error", "text": "bad event"}
            self.send(lp.encode_event(ev))
            if ev == lp.EVT_CLEAR_WAYPOINTS:
                self.waypoints = []
                self.broadcast(json.dumps({"t": "waypoints", "points": []}))
            elif ev == lp.EVT_MARK_WAYPOINT and self.tlm:
                self.waypoints.append((round(self.tlm["x_mm"], 1), round(self.tlm["y_mm"], 1)))
                self.broadcast(json.dumps({"t": "waypoints", "points": self.waypoints}))
            elif ev == lp.EVT_RESET_POSE:
                self.ranges.clear()
                self.waypoints = []
                self.broadcast(json.dumps({"t": "reset"}))
            self.say("%s: event %s" % (who, lp.EVENT_NAMES[ev]))
        elif t == "ping":
            return {"t": "pong", "id": m.get("id")}
        else:
            return {"t": "error", "text": "unknown command %r" % t}
        return None

    def do_estop(self, who):
        self.cmd = (0.0, 0.0)
        self.cmd_active = False
        self.send(lp.encode_set_mode(lp.MODE_IDLE))
        self.send(lp.encode_cmd_vel(0.0, 0.0))
        now = _now()
        self.estop_repeats = [now + 0.1, now + 0.25]  # belt and braces
        self.estop = True
        self.estop_time = now
        self.log.record("cmd", {"t": "estop", "who": who})
        self.say("E-STOP from %s" % who)
        self.push_status()

    # ------------------------------------------------------------------ clients
    def hello_msg(self):
        return json.dumps({
            "t": "hello", "bridge": BRIDGE_VERSION, "modes": lp.MODE_NAMES, "events": lp.EVENT_NAMES,
            "nav_states": NAV_STATE_NAMES, "tlm_bits": lp.TLM_BITS, "robot": self.hello,
            "cmd_vel_timeout_ms": lp.LINK_CMD_VEL_TIMEOUT_MS, "waypoints": self.waypoints,
        })

    async def ws_handler(self, ws):
        self.client_ids += 1
        who = "client%d@%s" % (self.client_ids, ws.peer[0] if ws.peer else "?")
        ws.send(self.hello_msg())
        ws.send(json.dumps(self.status()))
        ws.send(self.params_msg())
        if self.nav:
            ws.send(json.dumps(dict(t="nav", **self.nav)))
        pts = self.ranges_since(0, RANGE_ON_CONNECT)
        if pts:
            ws.send(json.dumps({"t": "ranges", "pts": pts}, separators=(",", ":")))
        for item in list(self.logs)[-60:]:
            ws.send(json.dumps(dict(t="log", **{k: v for k, v in item.items() if k != "t"}, ts=item["t"])))
        self.clients.add(ws)
        self.say("%s connected (%d clients)" % (who, len(self.clients)))
        try:
            while True:
                text = await ws.recv()
                if text is None:
                    break
                try:
                    m = json.loads(text)
                    if isinstance(m, dict):
                        if m.get("t") == "hi" and m.get("name"):
                            who = "%s@%s" % (str(m["name"])[:20], ws.peer[0] if ws.peer else "?")
                            continue
                        reply = self.handle_command(m, who)
                        if reply:
                            ws.send(json.dumps(reply))
                except (ValueError, TypeError, KeyError) as e:
                    ws.send(json.dumps({"t": "error", "text": "bad message: %s" % e}))
        finally:
            self.clients.discard(ws)
            # a driving client went away: stop immediately rather than waiting for the timeout
            if self.cmd_active:
                self.cmd_active = False
                self.send(lp.encode_cmd_vel(0.0, 0.0))
            self.say("%s disconnected (%d clients)" % (who, len(self.clients)))

    # ------------------------------------------------------------------ HTTP API
    def api_state(self, req):
        """Polling endpoint for browsers without WebSockets (Kindle Fire, issue #11).
        GET /api/state?r=<next range index>&l=<next log index>"""
        self.pollers[req.peer[0] if req.peer else "?"] = _now()
        r = int(_num(req.query.get("r"), self.range_next, 0))
        li = int(_num(req.query.get("l"), 0, 0))
        st = self.status()
        out = {"status": st, "tlm": _round_dict(self.tlm) if self.tlm else None, "nav": self.nav,
               "robot": self.hello, "waypoints": self.waypoints,
               "ranges": self.ranges_since(r, int(_num(req.query.get("max"), 300, 0, 2000))),
               "r": self.range_next, "logs": [x for x in self.logs if x["i"] >= li][-20:], "l": self.log_next,
               "modes": lp.MODE_NAMES}
        if req.query.get("params"):
            out["params"] = sorted(self.params.values(), key=lambda d: d["id"])
        return Response.json(out)

    def api_cmd(self, req):
        """POST /api/cmd with a JSON command, or GET /api/cmd?t=set_mode&mode=0 (old browsers)."""
        if req.method == "POST" and req.body:
            try:
                m = req.json()
            except ValueError:
                return Response.json({"t": "error", "text": "bad json"}, 400)
        else:
            m = dict(req.query)
            if "points" in m:  # "x,y;x,y"
                m["points"] = [p.split(",") for p in m["points"].split(";") if "," in p]
        who = "http@%s" % (req.peer[0] if req.peer else "?")
        self.pollers[req.peer[0] if req.peer else "?"] = _now()
        reply = self.handle_command(m, who) if isinstance(m, dict) else {"t": "error"}
        return Response.json(reply or {"t": "ok"})

    def api_logs(self, req):
        name = req.path[len("/api/logs/"):] if req.path.startswith("/api/logs/") else ""
        if not name:
            return Response.json({"sessions": self.log.list(), "dir": self.log.dir})
        path = self.log.path(name)
        if not path:
            return Response.text("no such session", 404)
        if req.method == "DELETE":
            if self.log.f and name == self.log.name:
                return Response.text("session is active", 400)
            os.remove(path)
            return Response.json({"t": "ok"})
        kind = req.query.get("csv")
        if kind in ("tlm", "rng"):
            self.log.flush()
            r = Response(200, ctype="text/csv; charset=utf-8", headers={
                "Content-Disposition": 'attachment; filename="%s-%s.csv"' % (name[:-6], kind)})
            r.gen = to_csv(path, kind)
            return r
        self.log.flush()
        return Response(200, ctype="application/x-ndjson", file=path,
                        headers={"Content-Disposition": 'attachment; filename="%s"' % name})

    def api_serial(self, req):
        """POST /api/serial?release=1 frees the serial port (for flashing); release=0 re-attaches."""
        if req.method == "POST" or "release" in req.query:
            v = req.query.get("release")
            if v is None and req.body:
                try:
                    v = req.json().get("release")
                except ValueError:
                    v = None
            self.released_api = str(v).lower() in ("1", "true", "yes", "on")
            self.say("serial %s via API" % ("release requested" if self.released_api else "re-attach requested"))
        return Response.json({"link": self.link_state, "released": self.released(),
                              "release_file": self.release_file})

    def api_status(self, req):
        return Response.json(self.status())

    # ------------------------------------------------------------------ run
    async def run(self):
        routes = [
            ("GET", "/api/state", self.api_state),
            ("*", "/api/cmd", self.api_cmd),
            ("GET", "/api/status", self.api_status),
            ("*", "/api/serial", self.api_serial),
            ("GET", "/api/logs", self.api_logs),
            ("*", "/api/logs/", self.api_logs),
        ]
        http = HttpServer(self.web_dir, routes, "/ws", self.ws_handler)
        await http.start(self.http_host, self.http_port)
        self.say("web UI on http://%s:%d/  (robot link: %s)" % (
            "localhost" if self.http_host in ("0.0.0.0", "") else self.http_host, self.http_port, self.port))
        await asyncio.gather(self.link_task(), self.tick_task())
