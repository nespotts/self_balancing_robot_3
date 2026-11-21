"""Ground-station bridge: Teensy link <-> browsers (WebSocket JSON + HTTP polling), session logs."""
import asyncio
import collections
import json
import math
import os
import tempfile
import time

from . import link_protocol as lp
from .httpws import HttpServer, Response, check_origin
from .mapping import Mapper
from .planner import Planner
from .session_log import SessionLog, to_csv
from .transport import LinkError, open_link
from .tuning_api import TuningTools

BRIDGE_VERSION = "sbr3-bridge 3"   # 3: ranges carry sensor_id / angle / raw mm, status rng_hz
# NavStatusFrame.state: NavState in link_protocol.h (idle, turning, driving, blocked, arrived, holding)
NAV_STATE_NAMES = lp.NAV_STATE_NAMES
TICK_S = 0.04            # 25 Hz: browser telemetry rate and cmd_vel resend rate
HEARTBEAT_S = 0.1        # 10 Hz MSG_HEARTBEAT
CMD_TIMEOUT_S = lp.LINK_CMD_VEL_TIMEOUT_MS / 1000.0
ROBOT_SILENT_S = 1.0     # no telemetry for this long -> robot shown as not responding
RANGE_RING = 4000
RANGE_ON_CONNECT = 2500
TILT_DEG = 10.0          # range points taken while pitched more than this are flagged
                         # (IMU pitch, which reads about -3 deg when balanced)
SENSOR_FWD_MM = 26.0     # range-sensor servo axis, forward of the axle (CAD)
SENSOR_RADIUS_MM = 12.9  # each VL53L0X sits this far out from the servo axis, facing outwards (CAD)
WS_SLOW_BYTES = 256 * 1024
WS_DROP_BYTES = 4 * 1024 * 1024   # a client whose send buffer grows beyond this is not reading: drop it (#47)
WS_PING_S = 15.0                  # ping idle clients; a client silent (no pong either) for WS_IDLE_S is dropped
WS_IDLE_S = 45.0
LOG_RETRY_S = 30.0                # session log could not be opened: try again this often
TICK_LATE_S = 0.2                 # warn in the log when the event loop was blocked this long (#45)
TLM_HZ_GRACE_S = 10.0             # Tune closed this long ago -> put tlm.hz back (#46)
# Commands that act on the robot: refused with an error while there is no link (#48)
NEEDS_LINK = {"set_mode", "set_param", "cfg", "get_params", "goto", "event", "estop_release"}  # release: the robot latches too
NEEDS_LINK_ACTION = {"preset": ("apply",), "motor": ("start", "apply"), "plan": ("goto", "explore")}
NO_LINK_TEXT = "robot not connected"
POLLER_TIMEOUT_S = 3.0
# Flashing: while this file exists the bridge lets go of the serial port (checked 4x a second).
# A file older than RELEASE_STALE_S is ignored (the flash script died without removing it);
# touch it again to hold the port longer. See README.md "Flashing while the bridge runs".
DEFAULT_RELEASE_FILE = "/tmp/sbr3-bridge.release" if os.name == "posix" else \
    os.path.join(tempfile.gettempdir(), "sbr3-bridge.release")
RELEASE_STALE_S = 15 * 60
LINK_POLL_S = 0.25
DEFAULT_LOG_MAX_MB = 2048
PRUNE_EVERY_S = 60.0
# Tuning stream: every telemetry frame (up to tlm.hz = 400) batched per tick, for subscribed clients.
TUNE_COLS = ["rt", "pitch_deg", "balance_target_deg", "bal_p", "bal_i", "bal_d", "motor_left", "motor_right",
             "steer_out", "vel_out_deg", "step"]
TUNE_BUF_MAX = 4000
CFG_ACTIONS = {"save": "cfg.save", "defaults": "cfg.defaults"}


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


def _flag(v):
    """A boolean field from JSON (true / 1) or a query string ("1", "true")."""
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)


def _sig(v):
    """float32 -> short decimal (4.630000114 -> 4.63)."""
    return float("%.6g" % v)


def _round_dict(d):
    return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()}


class Bridge:
    def __init__(self, port, web_dir, log_dir, http_host="0.0.0.0", http_port=8080, baud=115200,
                 release_file=None, log_max_mb=DEFAULT_LOG_MAX_MB, quiet=False, map_dir=None,
                 preset_dir="~/sbr3-presets", allow_any_origin=False, allow_hosts=(), allow_origins=()):
        self.port = port
        self.baud = baud
        self.web_dir = web_dir
        self.http_host, self.http_port = http_host, http_port
        self.release_file = release_file or DEFAULT_RELEASE_FILE
        self.release_stale_warned = False
        self.last_open_try = -10.0
        self.last_log_try = -10.0
        self.log_failed = False
        self.last_tick = None
        self.pruning = False
        self.last_prune = _now()
        self.quiet = quiet
        self.allow_any_origin = allow_any_origin    # --allow-any-origin: no Origin / Host checks (#44)
        self.allow_hosts = list(allow_hosts)
        self.allow_origins = list(allow_origins)
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
        self.get_params_at = None       # re-request the param list at this time (after cfg.defaults)
        self.remote_seen = False        # RC remote reported live at least once since connecting
        self.tune_clients = set()
        self.tune_buf = []
        self.hz_base = None             # tlm.hz before a Tune client raised it (None = not raised); see _hz_*
        self.hz_restore_at = None
        self.hz_armed = False           # a Tune subscriber has been around since the boost: restore when it leaves
        self.hz_grace_s = TLM_HZ_GRACE_S
        self.rt_offset = None           # robot t_us (ms) -> Pi ms
        self.tlm = None
        self.tlm_seq = 0
        self.tlm_sent_seq = 0
        self.tlm_time = -10.0
        self.tlm_count = 0
        self.tlm_hz = 0.0
        self.nav = None
        # [i, hx, hy, rx, ry, valid, tilt, t_ms, sensor_id, sensor_angle_deg, range_mm]: see _add_range
        self.ranges = collections.deque(maxlen=RANGE_RING)
        self.range_next = 0
        self.range_sent = 0
        self.rng_count = [0, 0]                             # readings per sensor since the last status
        self.rng_hz = [0.0, 0.0]
        self.logs = collections.deque(maxlen=200)          # {"i","t","src","text"}
        self.log_next = 0
        self.track = collections.deque(maxlen=5000)         # robot path [x, y] (every 20 mm)
        self.waypoints = []                                 # what the station last sent (x, y)
        self.range_listeners = []                           # callables(range dict) e.g. mapping (#12)
        self.tools = TuningTools(self, preset_dir)          # Tune page: presets, scored runs, motor test (#16)
        self.mapper = Mapper(map_dir or os.path.join(self.log.dir, "maps"), say=self.say)   # main: ~/sbr3-maps
        self.range_listeners.append(self.mapper.on_range)
        self.planner = Planner(self, NAV_STATE_NAMES)           # route planning + explore on the map

        self.cmd = (0.0, 0.0)
        self.cmd_time = -10.0
        self.cmd_active = False
        self.estop = False              # station E-stop latched (cleared only by "estop_release")
        self.estop_time = -10.0
        self.estop_repeats = []
        self.robot_estop = False        # robot reports its link E-stop latch (remote_flags, v4 firmware)

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
    def release_file_active(self):
        try:
            age = time.time() - os.stat(self.release_file).st_mtime
        except OSError:
            self.release_stale_warned = False
            return False
        if age > RELEASE_STALE_S:
            if not self.release_stale_warned:
                self.release_stale_warned = True
                self.say("ignoring stale %s (%d min old)" % (self.release_file, age // 60))
            return False
        return True

    def released(self):
        return self.released_api or self.release_file_active()

    async def link_task(self):
        while True:
            try:
                await self._link_step()
            except Exception as e:     # one bad moment (full disk, odd port error) must not end the bridge
                import traceback
                traceback.print_exc()
                try:
                    self.say("link task error: %s: %s (carrying on)" % (type(e).__name__, e))
                except Exception:
                    pass
            await asyncio.sleep(LINK_POLL_S)

    async def _link_step(self):
        if self.released():
            if self.link:
                self.say("serial port released (flashing?)")
                self.link.close()
                self._on_lost(quiet=True)
            self._set_link_state("released")
        elif not self.link and _now() - self.last_open_try >= 1.0:
            self.last_open_try = _now()
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
                self.remote_seen = False
                self.rt_offset = None
                self._set_link_state("connected")
                self._start_log()
                try:
                    self.tools.on_session()
                except Exception as e:
                    self.say("tuning tools: %s" % e)
                self.say("connected to %s, session %s" % (self.port, self.log.name or "(not logging)"))
                self.send(lp.encode_get_params())
                self.last_get_params = _now()
        elif self.link and not self.log.f and _now() - self.last_log_try >= LOG_RETRY_S:
            self._start_log()          # the disk was full / read-only: try again, quietly

    def _start_log(self):
        self.last_log_try = _now()
        if self.log.start():
            if self.log_failed:
                self.log_failed = False
                self.say("session log back: %s" % self.log.name)
            return True
        if not self.log_failed:       # say it once; the retry stays quiet
            self.log_failed = True
            self.say("WARNING: cannot write the session log in %s (%s) - running without logging, retrying every %d s"
                     % (self.log.dir, self.log.error, LOG_RETRY_S))
        return False

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
        self.tools.on_lost()
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
            except Exception as e:  # short payload etc.: drop the frame, keep the link
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
            if d["radio_ok"] and not self.remote_seen:
                self.remote_seen = True
                self.push_status()
            self.tools.on_tlm(d, self.tlm_time)           # find balance point (#52)
            if self.tune_clients:
                self._add_tune(d)
            last = self.track[-1] if self.track else None
            if last is None or abs(d["x_mm"] - last[0]) + abs(d["y_mm"] - last[1]) > 20:
                self.track.append([int(d["x_mm"]), int(d["y_mm"])])
            rf = (d["remote_flags"] & lp.LINK_RF_LINK_ESTOP) != 0
            if rf != self.robot_estop:          # the robot's own latch (v4 firmware)
                self.robot_estop = rf
                self.push_status()
        elif t == lp.MSG_RANGE:
            d = lp.decode_range(p)
            self.log.rng(d)
            self._add_range(d)
        elif t == lp.MSG_LOG:
            text = lp.decode_log(p)
            self.log.record("log", text)
            self._add_log("robot", text)
            self.tools.on_log(text)
        elif t == lp.MSG_PARAM_INFO:
            d = lp.decode_param_info(p)
            for k in ("value", "min", "max"):
                d[k] = _sig(d[k])
            self.tools.param_changed(d["name"], self.params.get(d["id"], {}).get("value"), d["value"])
            if d["name"] == "tlm.hz" and self.hz_base is not None and abs(d["value"] - self.hz_base) < 1e-6:
                self.hz_base = None          # the robot is back at its own rate (reset / restored)
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
            if self.estop:                     # Teensy rebooted while latched here: latch it there again
                self.send(lp.encode_event(lp.EVT_LINK_ESTOP))

    def _add_tune(self, d):
        # robot time (exact step spacing) mapped onto the Pi clock the browser plots use
        ms = d["t_us"] / 1000.0
        pi = self.pi_ms()
        if self.rt_offset is None or abs(ms + self.rt_offset - pi) > 500:   # first sample / reboot / wrap
            self.rt_offset = pi - ms
        row = [round(ms + self.rt_offset, 1)] + [round(d[k], 3) for k in TUNE_COLS[1:-1]] + [d["step"]]
        self.tune_buf.append(row)
        if len(self.tune_buf) > TUNE_BUF_MAX:
            del self.tune_buf[:len(self.tune_buf) - TUNE_BUF_MAX]

    def _add_range(self, d):
        """One RangeFrame -> ring entry (what browsers get in "ranges" messages):
        [i, hit x, hit y, sensor x, sensor y (servo axis), valid, tilt, Pi ms,
         sensor_id (0 front / 1 rear), sensor_angle_deg (robot frame, + = CCW), range_mm (raw)]
        Invalid readings get a hit point 2 m out (map rays); the raw range_mm is kept as sent."""
        r = d["range_mm"]
        h = math.radians(d["heading_deg"])
        ox = d["x_mm"] + SENSOR_FWD_MM * math.cos(h)    # sensor position (servo axis)
        oy = d["y_mm"] + SENSOR_FWD_MM * math.sin(h)
        a = h + math.radians(d["sensor_angle_deg"])
        rr = (r if (d["valid"] and r < 8000) else 2000) + SENSOR_RADIUS_MM
        hx = ox + rr * math.cos(a)
        hy = oy + rr * math.sin(a)
        tilt = 1 if abs(d["pitch_deg"]) > TILT_DEG else 0
        sid = d["sensor_id"] & 1
        self.ranges.append([self.range_next, int(hx), int(hy), int(ox), int(oy),
                            int(d["valid"]), tilt, self.pi_ms(),
                            sid, round(d["sensor_angle_deg"], 1), int(r)])
        self.range_next += 1
        self.rng_count[sid] += 1
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
            late = now - self.last_tick - TICK_S if self.last_tick is not None else 0.0
            self.last_tick = now
            if late > TICK_LATE_S:      # something blocked the loop: the robot drops the link after 500 ms of this
                self.say("WARNING: event loop stalled for %d ms" % round(late * 1000))
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
                self.send(lp.encode_event(lp.EVT_LINK_ESTOP))
                self.send(lp.encode_set_mode(lp.MODE_IDLE))
            if self.get_params_at is not None and now >= self.get_params_at:
                self.get_params_at = None
                self.send(lp.encode_get_params())
                self.last_get_params = now
        if self.tune_buf:
            rows, self.tune_buf = self.tune_buf, []
            text = json.dumps({"t": "tune", "cols": TUNE_COLS, "rows": rows}, separators=(",", ":"))
            for c in list(self.tune_clients):
                if c.closed:
                    self.tune_clients.discard(c)
                elif c.buffered() < WS_SLOW_BYTES:
                    c.send(text)
        if now - self.last_prune >= PRUNE_EVERY_S and not self.pruning:
            self.last_prune = now
            self._prune_async()
        self._tlm_hz_tick(now)
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
        for text in self.mapper.poll(now):                # occupancy grid tiles, ~2 Hz (#12)
            self.broadcast(text)
        self.planner.tick(now)
        self.tools.tick(now)
        if self.params_dirty:
            self.params_dirty = False
            self.broadcast(self.params_msg())
        if now - self.last_status >= 1.0:
            self._reap_clients(now)
            dt = now - self.last_rate
            self.tlm_hz = self.tlm_count / dt if dt > 0 else 0.0
            self.tlm_count = 0
            self.rng_hz = [c / dt if dt > 0 else 0.0 for c in self.rng_count]
            self.rng_count = [0, 0]
            self.last_rate = now
            self.push_status()
            self.log.flush()

    def _prune_async(self):
        """Delete old sessions in a worker thread (stat / remove of big files must not stall the loop)."""
        def work():
            try:
                self.log.prune()
            except Exception:
                pass
            finally:
                self.pruning = False
        self.pruning = True
        try:
            asyncio.get_running_loop().run_in_executor(None, work)
        except RuntimeError:            # no loop (unit tests calling _tick directly)
            work()

    def _reap_clients(self, now):
        """Half-open phones (left the Wi-Fi without a FIN) must not pile up buffered data forever (#47)."""
        for c in list(self.clients):
            if c.closed:
                self.clients.discard(c)
            elif c.buffered() > WS_DROP_BYTES:
                self.say("dropping client %s: not reading (%d KB queued)" % (c.peer[0] if c.peer else "?", c.buffered() // 1024))
                c.abort()
            elif now - c.last_rx > WS_IDLE_S:
                self.say("dropping client %s: no answer for %d s" % (c.peer[0] if c.peer else "?", WS_IDLE_S))
                c.abort()
            elif now - c.last_ping >= WS_PING_S:
                c.ping()

    # ---- tlm.hz: the Tune page raises it to full rate; the bridge puts it back (#46)
    def _tlm_hz_param(self):
        return next((p for p in self.params.values() if p["name"] == "tlm.hz"), None)

    def _hz_note_set(self, p, v, who):
        """A client set tlm.hz. Raising it from a browser is temporary (Tune open); lowering back or any
        other source (tune.py on the Pi, the planner) is final."""
        if who.startswith("http@") or who == "planner":
            self.hz_base = None
        elif self.hz_base is None:
            if v > p["value"]:
                self.hz_base = p["value"]      # remembered; restored when the last Tune subscriber leaves
                self.hz_armed = bool(self.tune_clients)
        elif v <= self.hz_base:
            self.hz_base = None
        if self.hz_base is None:
            self.hz_restore_at = None
            self.hz_armed = False

    def _tlm_hz_tick(self, now):
        if self.hz_base is None:
            return
        if self.tune_clients:
            self.hz_armed = True
        if self.tune_clients or not self.hz_armed or not self.link:
            self.hz_restore_at = None
            return
        if self.hz_restore_at is None:
            self.hz_restore_at = now + self.hz_grace_s
        elif now >= self.hz_restore_at:
            p, base = self._tlm_hz_param(), self.hz_base
            self.hz_base = self.hz_restore_at = None
            self.hz_armed = False
            if p and p["value"] != base:
                self.send(lp.encode_set_param(p["id"], base))
                p["value"] = base
                self.params_dirty = True
                self.say("tlm.hz back to %g Hz (Tune closed)" % base)
                self.push_status()

    def status(self):
        now = _now()
        return {
            "t": "status", "link": self.link_state, "port": self.port, "link_error": self.link_error,
            "robot_alive": now - self.tlm_time < ROBOT_SILENT_S, "tlm_hz": round(self.tlm_hz, 1),
            "rng_hz": [round(h, 1) for h in self.rng_hz],     # range readings/s, sensor 0 and 1
            "crc_errors": self.parser.crc_errors, "frames": self.parser.frames_ok,
            "clients": self.client_count(), "session": self.log.name if self.log.f else None,
            "estop": self.estop, "robot_estop": self.robot_estop,
            "cmd_active": self.cmd_active, "remote_seen": self.remote_seen,
            "uptime_s": int(now - self.t_start), "release_file": self.release_file,
            "tlm_boost": self.hz_base is not None,         # tlm.hz raised by the Tune page, restored when it closes
            "tlm_base": self.hz_base,                      # ... and what it goes back to
            "log_error": self.log.error,                   # why the session log is off ("" = fine)
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
        if not self.link and self._needs_link(t, m.get("action")):
            # nothing would reach the robot: say so instead of pretending (no optimistic state either)
            return {"t": "error", "code": "no_link", "text": NO_LINK_TEXT + " - %s not sent" % t}
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
            if not self.link:        # the flags are set, but the robot cannot have heard it
                return {"t": "error", "code": "no_link", "text": "robot link down - STOP may not have reached the robot"}
            return None
        self.log.record("cmd", dict(m, who=who))
        if t == "estop_release":
            return self.do_estop_release(who)
        if t == "set_mode":
            mode = int(_num(m.get("mode"), -1))
            if not 0 <= mode < lp.MODE_COUNT:
                return {"t": "error", "text": "bad mode"}
            if mode != lp.MODE_IDLE and self.estop:
                return {"t": "error", "code": "estop", "text": "E-stop is on: release it first"}
            self.send_mode(mode, _flag(m.get("arm")))
            if who != "planner" and mode not in (lp.MODE_GOTO, lp.MODE_EXPLORE):
                self.planner.stop("stopped: mode %s from %s" % (lp.MODE_NAMES[mode], who))
            self.say("%s: mode -> %s%s" % (who, lp.MODE_NAMES[mode], " (arm)" if _flag(m.get("arm")) else ""))
        elif t == "set_param":
            pid = int(_num(m.get("id"), -1))
            if pid not in self.params:
                return {"t": "error", "text": "unknown param %s" % pid}
            p = self.params[pid]
            if p["name"].startswith("cfg."):    # actions (read back 0): use {"t": "cfg"}
                return self.handle_command({"t": "cfg", "action": p["name"][4:]}, who) \
                    if _num(m.get("value")) >= 0.5 else None
            v = _num(m.get("value"), p["value"], p["min"], p["max"])
            self.send(lp.encode_set_param(pid, v))
            self.tools.param_changed(p["name"], p["value"], v)
            if p["name"] == "tlm.hz":
                self._hz_note_set(p, v, who)
            p["value"] = v  # optimistic; a ParamInfo echo from the robot overrides it
            self.params_dirty = True
        elif t == "cfg":
            # {"t": "cfg", "action": "save"}     -> cfg.save = 1 (EEPROM write, waits for motors off)
            # {"t": "cfg", "action": "defaults"} -> cfg.defaults = 1 (defaults, not saved)
            name = CFG_ACTIONS.get(m.get("action"))
            pid = next((i for i, p in self.params.items() if p["name"] == name), None) if name else None
            if pid is None:
                return {"t": "error", "text": "robot has no %s param" % (name or m.get("action"))}
            self.send(lp.encode_set_param(pid, 1.0))
            if name == "cfg.save":
                self.hz_base = self.hz_restore_at = None      # saved on purpose: that rate is now the default
                self.hz_armed = False
            if name == "cfg.defaults":
                self.get_params_at = _now() + 0.5   # the robot re-streams; ask again in case frames were lost
            self.say("%s: %s" % (who, "save params to EEPROM" if name == "cfg.save" else "restore default params"))
        elif t == "get_params":
            self.send(lp.encode_get_params())
            self.last_get_params = _now()
        elif t == "goto":
            pts = [(_num(p[0]), _num(p[1])) for p in (m.get("points") or [])
                   if isinstance(p, (list, tuple)) and len(p) >= 2][:255]
            replace = bool(m.get("replace", True))
            self.planner.cancel_if_user_goto(who)
            for f in lp.goto_frames(pts, replace):
                self.send(f)
            self.waypoints = (pts if replace else self.waypoints + pts)
            self.broadcast(json.dumps({"t": "waypoints", "points": self.waypoints}))
            if m.get("go"):
                if self.estop:
                    return {"t": "error", "code": "estop", "text": "waypoints sent; E-stop is on: release it first"}
                self.send_mode(lp.MODE_GOTO, _flag(m.get("arm")))
            if who != "planner":                       # (the planner re-sends routes often)
                self.say("%s: %d waypoint(s)%s" % (who, len(pts), " -> GOTO" if m.get("go") else ""))
        elif t == "event":
            ev = int(_num(m.get("event"), -1))
            if not 0 < ev < lp.EVT_COUNT:
                return {"t": "error", "text": "bad event"}
            self.send(lp.encode_event(ev))
            if ev == lp.EVT_CLEAR_WAYPOINTS:
                self.planner.cancel_if_user_goto(who)
                self.waypoints = []
                self.broadcast(json.dumps({"t": "waypoints", "points": []}))
            elif ev == lp.EVT_MARK_WAYPOINT and self.tlm:
                self.waypoints.append((round(self.tlm["x_mm"], 1), round(self.tlm["y_mm"], 1)))
                self.broadcast(json.dumps({"t": "waypoints", "points": self.waypoints}))
            elif ev == lp.EVT_RESET_POSE:
                self.ranges.clear()
                self.track.clear()
                self.waypoints = []
                self.broadcast(json.dumps({"t": "reset"}))
            self.say("%s: event %s" % (who, lp.EVENT_NAMES[ev]))
        elif t in ("preset", "score", "motor", "balpoint"):
            return self.tools.handle(m, who)
        elif t == "map":       # occupancy grid: reset / save / load / list / delete / set (#12)
            return self.mapper.command(m, who)
        elif t == "plan":      # route planning / explore: goto / explore / stop / set
            if self.estop and m.get("action") in ("goto", "explore"):
                return {"t": "error", "code": "estop", "text": "E-stop is on: release it first"}
            return self.planner.command(m, who)
        elif t == "ping":
            return {"t": "pong", "id": m.get("id")}
        else:
            return {"t": "error", "text": "unknown command %r" % t}
        return None

    def send_mode(self, mode, arm=False):
        """SET_MODE. arm (an explicit user arm step, e.g. the two-tap Arm & Balance): EVT_LINK_ARM first, so
        with no remote present this SET_MODE may arm the robot. Without it a SET_MODE never arms (v4 firmware)."""
        if arm and mode != lp.MODE_IDLE:
            self.send(lp.encode_event(lp.EVT_LINK_ARM))
        self.send(lp.encode_set_mode(mode))

    @staticmethod
    def _needs_link(t, action):
        try:
            return t in NEEDS_LINK or action in NEEDS_LINK_ACTION.get(t, ())
        except TypeError:        # unhashable junk in "t" / "action"
            return False

    def do_estop(self, who):
        """Station E-stop: latched in the robot (EVT_LINK_ESTOP, v4) and here; SET_MODE IDLE for older firmware.
        Stays on until "estop_release" (then the robot still needs arming)."""
        self.cmd = (0.0, 0.0)
        self.cmd_active = False
        self.send(lp.encode_event(lp.EVT_LINK_ESTOP))
        self.send(lp.encode_set_mode(lp.MODE_IDLE))
        self.send(lp.encode_cmd_vel(0.0, 0.0))
        now = _now()
        self.estop_repeats = [now + 0.1, now + 0.25]  # belt and braces
        self.planner.stop("stopped: E-stop")
        self.estop = True
        self.estop_time = now
        self.log.record("cmd", {"t": "estop", "who": who})
        self.say("E-STOP from %s" % who)
        self.push_status()

    def do_estop_release(self, who):
        """Clear the station E-stop latch. The robot stays IDLE and disarmed: arm again (remote ARMED
        off -> on, or the station's Arm step)."""
        if not self.estop and not self.robot_estop:
            return None
        self.estop_repeats = []          # a pending E-stop repeat must not re-latch it after the release
        self.estop = False
        self.send(lp.encode_event(lp.EVT_LINK_ESTOP_RELEASE))
        self.say("E-stop released by %s (robot stays disarmed until armed again)" % who)
        self.push_status()
        return None

    # ------------------------------------------------------------------ clients
    def hello_msg(self):
        return json.dumps({
            "t": "hello", "bridge": BRIDGE_VERSION, "modes": lp.MODE_NAMES, "events": lp.EVENT_NAMES,
            "nav_states": NAV_STATE_NAMES, "tlm_bits": lp.TLM_BITS, "robot": self.hello,
            "cmd_vel_timeout_ms": lp.LINK_CMD_VEL_TIMEOUT_MS, "waypoints": self.waypoints,
            "range_geom": {"fwd_mm": SENSOR_FWD_MM, "radius_mm": SENSOR_RADIUS_MM},
        })

    async def ws_handler(self, ws):
        self.client_ids += 1
        who = "client%d@%s" % (self.client_ids, ws.peer[0] if ws.peer else "?")
        ws.send(self.hello_msg())
        ws.send(json.dumps(self.status()))
        ws.send(self.params_msg())
        if self.nav:
            ws.send(json.dumps(dict(t="nav", **self.nav)))
        if self.track:
            ws.send(json.dumps({"t": "track", "pts": list(self.track)}, separators=(",", ":")))
        pts = self.ranges_since(0, RANGE_ON_CONNECT)
        if pts:
            ws.send(json.dumps({"t": "ranges", "pts": pts}, separators=(",", ":")))
        for text in self.tools.hello_msgs():
            ws.send(json.dumps(text, separators=(",", ":")))
        ws.send(self.mapper.full_msg())
        ws.send(self.planner.msg())
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
                    if not isinstance(m, dict):
                        ws.send(json.dumps({"t": "error", "text": "bad message: expected a JSON object"}))
                        continue
                    if m.get("t") == "hi" and m.get("name"):
                        who = "%s@%s" % (str(m["name"])[:20], ws.peer[0] if ws.peer else "?")
                        continue
                    if m.get("t") == "sub":   # {"t": "sub", "tune": true} -> full-rate "tune" batches
                        if m.get("tune"):
                            self.tune_clients.add(ws)
                            if self.hz_base is not None:
                                self.hz_armed = True
                        else:
                            self.tune_clients.discard(ws)
                        continue
                    reply = self.run_command(m, who)
                    if reply:
                        ws.send(json.dumps(reply))
                        if reply.get("code") == "no_link":      # the page may have shown the new value already
                            ws.send(self.params_msg())
                except Exception as e:     # whatever a client sends, its socket (and the bridge) stays up
                    ws.send(json.dumps({"t": "error", "text": "bad message: %s: %s" % (type(e).__name__, e)}))
        finally:
            self.clients.discard(ws)
            self.tune_clients.discard(ws)
            # a driving client went away: stop immediately rather than waiting for the timeout
            if self.cmd_active:
                self.cmd_active = False
                self.send(lp.encode_cmd_vel(0.0, 0.0))
            self.say("%s disconnected (%d clients)" % (who, len(self.clients)))

    def run_command(self, m, who="?"):
        """handle_command for untrusted input (browsers, HTTP): a bug or odd value becomes an error reply."""
        try:
            return self.handle_command(m, who)
        except Exception as e:
            import traceback
            traceback.print_exc()
            try:
                self.say("command %r from %s failed: %s: %s" % (str(m.get("t"))[:30], who, type(e).__name__, e))
            except Exception:
                pass
            return {"t": "error", "text": "command failed: %s" % e}

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
            # JSON only: a text/plain or form POST from another site is a CORS "simple request" (no preflight)
            if not req.headers.get("content-type", "").lower().startswith("application/json"):
                return Response.json({"t": "error", "text": "POST /api/cmd needs Content-Type: application/json"}, 415)
            try:
                m = req.json()
            except (ValueError, RecursionError):
                return Response.json({"t": "error", "text": "bad json"}, 400)
        else:
            m = dict(req.query)
            if "points" in m:  # "x,y;x,y"
                m["points"] = [p.split(",") for p in m["points"].split(";") if "," in p]
        who = "http@%s" % (req.peer[0] if req.peer else "?")
        self.pollers[req.peer[0] if req.peer else "?"] = _now()
        reply = self.run_command(m, who) if isinstance(m, dict) else {"t": "error", "text": "expected a JSON object"}
        return Response.json(reply or {"t": "ok"})

    async def api_logs(self, req):
        name = req.path[len("/api/logs/"):] if req.path.startswith("/api/logs/") else ""
        if not name:
            return Response.json({"sessions": self.log.list(), "dir": self.log.dir})
        path = self.log.path(name)
        if not path:
            return Response.text("no such session", 404)
        if req.method == "DELETE":
            if self.log.f and name == self.log.name:
                return Response.text("session is active", 400)
            try:       # removing a big file takes a while on an SD card: not on the event loop
                await asyncio.get_running_loop().run_in_executor(None, os.remove, path)
            except OSError as e:
                return Response.text("could not delete: %s" % (e.strerror or e), 500)
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

    def api_map(self, req):
        """GET /api/map -> saved maps + stats; /api/map/current.png or /api/map/<name>.png -> image."""
        name = req.path[len("/api/map/"):] if req.path.startswith("/api/map/") else ""
        if not name:
            return Response.json({"maps": self.mapper.list(), "dir": self.mapper.dir, "name": self.mapper.name,
                                  "stats": self.mapper.stats(), "settings": self.mapper.settings})
        if name == "current.png":
            return Response(200, body=self.mapper.grid.png(2), ctype="image/png")
        p = self.mapper.path(name[:-4]) if name.endswith(".png") else None
        if not p or not os.path.exists(p[:-5] + ".png"):
            return Response.text("no such map", 404)
        return Response(200, ctype="image/png", file=p[:-5] + ".png")

    def api_status(self, req):
        return Response.json(self.status())

    # ------------------------------------------------------------------ run
    def build_http(self):
        """The HTTP + WebSocket server (not started). Origin / Host checks on /ws and /api/* unless
        allow_any_origin (--allow-any-origin)."""
        guard = None if self.allow_any_origin else \
            (lambda req: check_origin(req, self.allow_hosts, self.allow_origins))
        routes = [
            ("GET", "/api/state", self.api_state),
            ("*", "/api/cmd", self.api_cmd),
            ("GET", "/api/status", self.api_status),
            ("*", "/api/serial", self.api_serial),
            ("GET", "/api/logs", self.api_logs),
            ("*", "/api/logs/", self.api_logs),
            ("GET", "/api/map", self.api_map),
            ("GET", "/api/map/", self.api_map),
        ]
        return HttpServer(self.web_dir, routes, "/ws", self.ws_handler, guard=guard)

    async def run(self):
        http = self.build_http()
        await http.start(self.http_host, self.http_port)
        self.say("web UI on http://%s:%d/  (robot link: %s)" % (
            "localhost" if self.http_host in ("0.0.0.0", "") else self.http_host, self.http_port, self.port))
        await asyncio.gather(self.link_task(), self.tick_task())
