"""ROS stream (issue #96): full-rate LD19 scans + odometry for the ROS 2 adapter (ros2/sbr3_ros), opt-in per client.

A WebSocket client sends {"t": "ros", "action": "subscribe"} (and "unsubscribe" / "get"); the reply is a ros_hello.
Only subscribed clients get these messages, all droppable (a slow client skips them, never the robot's commands):

  ros_hello  on subscribe: frames, the lidar mount (base_link -> laser), the scan layout, the stream rates
  ros_scan   every LD19 turn (10 Hz): ranges binned on a fixed grid in the LASER frame at the scan's end time
             (deskewed: every packet's points go through the pose at that packet's time), base64 uint16 mm
  ros_odom   telemetry rows batched per bridge tick (25 Hz), decimated to <= ODOM_HZ rows/s
  ros_state  2 Hz: mode, armed, E-stops, battery, link, the scan-match correction, stream stats

Clock: every time field ("ts", "now") is the bridge's monotonic clock in seconds, the clock the lidar packet times
and the telemetry receive times are already in (lidar.py deskews with the same pairs). The adapter maps it onto ROS
time with one offset (min of ros_now - now over a window), so /scan, /odom and /tf stamps agree with each other.

Laser frame: base_link translated by the lidar.json mount (x_mm, y_mm, height_mm), NOT rotated: the mount yaw and the
LD19's clockwise count are applied here, so the static base_link -> laser transform always has yaw 0.
Ranges: d * cos(elevation) (horizontal), floor clip, mask, range / intensity window as lidar.py; 0 = no return.
Tilt: "tilt" = the largest |pitch - typical pitch| of the turn's packets; packets beyond "gate" (the wider of
pitch_limit_deg / pitch_max_deg, 6 deg by default, as lidar_obstacles.py) are left out and counted in "tilt_pk".
The adapter drops turns whose tilt exceeds its own max_tilt_deg.

Map correction: "corr" = [dx mm, dy mm, dth deg], map pose = corr o odometry pose (mapping.Mapper.corr), i.e. the
map <- odom transform of the bridge's own LD19 scan matcher (#78).
"""
import base64
import json
import math
import struct
import time

from . import link_protocol as lp
from .lidar import PITCH_ABS_MAX, POSE_STALE_S, interp
from .lidar_obstacles import pitch_ref

VERSION = 1
SCAN_BINS = 480               # 0.75 deg: the LD19 gives ~450 points per turn at 10 Hz
ODOM_HZ = 50.0
STATE_S = 0.5
MAX_SCAN_AGE_S = 0.25         # a turn older than this when it gets here (backlog) is not sent
WS_SLOW_BYTES = 128 * 1024    # a subscriber with more than this queued skips ROS messages
ODOM_COLS = ["ts", "x_mm", "y_mm", "heading_deg", "speed_mms", "turn_rate_dps",
             "pitch_deg", "pitch_rate_dps", "roll_deg", "yaw_rate_dps"]
_ODOM_ROUND = (3, 1, 1, 2, 1, 2, 2, 2, 2, 2)


def _js(d):
    return json.dumps(d, separators=(",", ":"))


def encode_ranges(rs):
    """[mm int, ...] -> base64 of little-endian uint16 (65535 max)."""
    return base64.b64encode(struct.pack("<%dH" % len(rs), *[min(65535, max(0, int(r))) for r in rs])).decode("ascii")


def decode_ranges(s):
    b = base64.b64decode(s)
    return list(struct.unpack("<%dH" % (len(b) // 2), b))


def bin_scan(scan, hist, S, mtab, bins=SCAN_BINS):
    """One LD19 turn -> (ranges mm [bins], stats dict) in the laser frame at scan.t1, or (None, reason).

    hist: lidar.PoseHistory items [(t, x, y, heading deg, pitch deg)], S: lidar settings, mtab: lidar mask table."""
    tco = S["time_offset_ms"] / 1000.0
    if not hist or hist[-1][0] < scan.t1 + tco - POSE_STALE_S:
        return None, "no_pose"
    ref = pitch_ref(hist, S)
    gate = max(S["pitch_limit_deg"], S["pitch_max_deg"])
    sgn = 1.0 if S["flip"] else -1.0                 # the LD19 counts clockwise; flipped over it counts CCW
    yaw, sx, sy = S["yaw_deg"], S["x_mm"], S["y_mm"]
    rmin, rmax, qmin = S["min_range_mm"], S["max_range_mm"], S["min_intensity"]
    clip, height, deskew = S["floor_clip"], S["height_mm"], S["deskew"]
    radians, cos, sin, tan, atan2, sqrt = math.radians, math.cos, math.sin, math.tan, math.atan2, math.sqrt
    _, x1, y1, h1, _p = interp(hist, 0, scan.t1 + tco)
    h1r = radians(h1)
    c1, s1 = cos(h1r), sin(h1r)
    o1x, o1y = x1 + c1 * sx - s1 * sy, y1 + s1 * sx + c1 * sy      # laser origin at t1, odometry frame
    k = bins / (2.0 * math.pi)
    out = [0] * bins
    tilt = 0.0
    tilt_pk = used = 0
    idx = 0
    px, py, ph, pp = x1, y1, h1, _p
    for pk in scan.packets:
        if deskew:
            idx, px, py, ph, pp = interp(hist, idx, pk.t + tco)
        dp = pp - ref
        if abs(dp) > tilt:
            tilt = abs(dp)
        if abs(dp) > gate or abs(pp) > PITCH_ABS_MAX:
            tilt_pk += 1
            continue
        hp = radians(ph)
        ch, sh = cos(hp), sin(hp)
        # this packet's laser origin, in the laser frame at t1
        dx, dy = px + ch * sx - sh * sy - o1x, py + sh * sx + ch * sy - o1y
        ox, oy = dx * c1 + dy * s1, -dx * s1 + dy * c1
        dh = hp - h1r
        dpr = radians(dp)
        for a, d, q in pk.pts:
            if d <= 0 or d < rmin or d > rmax or q < qmin:
                continue
            b = yaw + sgn * a
            if mtab[int((b % 360.0) * 2.0)]:
                continue
            rb = radians(b)
            if dpr:
                e = dpr * cos(rb)
                d = d * cos(e)
                ae = abs(e)
                if clip and ae > 1e-4 and d > 0.9 * height / tan(ae):
                    continue
            ang = dh + rb
            x, y = ox + d * cos(ang), oy + d * sin(ang)
            i = int((atan2(y, x) + math.pi) * k) % bins
            r = sqrt(x * x + y * y)
            cur = out[i]
            if cur == 0 or r < cur:
                out[i] = r
            used += 1
    return [int(round(r)) for r in out], {"tilt": round(tilt, 2), "tilt_pk": tilt_pk, "used": used,
                                          "pose": (x1, y1, h1)}


class RosStream:
    """Hooks (bridge.py): on_tlm(d, t) per telemetry frame, tick(now) per bridge tick, command(ws, m) for
    {"t": "ros"}, drop(ws) when a client leaves. Registers itself for LD19 turns."""

    def __init__(self, bridge, clock=time.monotonic):
        self.b = bridge
        self.clock = clock
        self.clients = set()
        self.odom_rows = []
        self.next_row_t = -10.0
        self.last_state = -10.0
        self.seq = 0
        self.n = {"scans": 0, "sent": 0, "no_pose": 0, "stale": 0, "off": 0, "skipped_slow": 0}
        self.cost_ms = 0.0            # bin_scan, running average
        self.cpu_s = 0.0
        self.bytes = 0                # sent to all subscribers
        self._rate_t0 = clock()
        self._rate_b0 = 0
        self._rate_c0 = 0.0
        self.kbps = 0.0               # per subscriber, last 5 s
        self.cpu_pct = 0.0
        lidar = getattr(bridge, "lidar", None)
        if lidar is not None:
            lidar.scan_listeners.append(self.on_scan)

    # ------------------------------------------------------------ clients
    def command(self, ws, m):
        act = m.get("action", "subscribe")
        if act == "subscribe":
            self.clients.add(ws)
            self.b.say("ROS stream: %s subscribed (%d)" % (ws.peer[0] if getattr(ws, "peer", None) else "?",
                                                           len(self.clients)))
        elif act == "unsubscribe":
            self.drop(ws)
            return {"t": "ros_hello", "subscribed": False}
        elif act != "get":
            return {"t": "error", "text": "unknown ros action %r" % act}
        return self.hello()

    def drop(self, ws):
        self.clients.discard(ws)

    def hello(self):
        lidar = getattr(self.b, "lidar", None)
        S = lidar.settings if lidar else None
        return {"t": "ros_hello", "v": VERSION, "subscribed": True, "now": round(self.clock(), 4),
                "frames": {"odom": "odom", "base": "base_link", "laser": "laser", "map": "map"},
                "lidar": None if S is None else {
                    "x_mm": S["x_mm"], "y_mm": S["y_mm"], "height_mm": S["height_mm"], "yaw_deg": 0.0,
                    "min_range_mm": S["min_range_mm"], "max_range_mm": S["max_range_mm"],
                    "gate_deg": max(S["pitch_limit_deg"], S["pitch_max_deg"]), "map_gate_deg": S["pitch_limit_deg"]},
                "scan": {"bins": SCAN_BINS, "angle_min": -math.pi, "angle_increment": 2 * math.pi / SCAN_BINS,
                         "enc": "b64u16mm", "hz": 10},
                "odom": {"cols": ODOM_COLS, "hz": ODOM_HZ},
                "modes": lp.MODE_NAMES, "cmd_vel_timeout_ms": lp.LINK_CMD_VEL_TIMEOUT_MS,
                "station_link": 1}          # #106: this bridge relays ros_map / ros_pose / ros_nav ... (ros_link.py)

    def _send(self, text):
        for c in list(self.clients):
            if c.closed:
                self.clients.discard(c)
                continue
            if c.buffered() > WS_SLOW_BYTES:
                self.n["skipped_slow"] += 1
                continue
            c.send(text)
            self.bytes += len(text)

    # ------------------------------------------------------------ inputs
    def on_tlm(self, d, t):
        if not self.clients or t < self.next_row_t:
            return
        # a row per 1 / ODOM_HZ slot (the first frame at or after it): 50 rows/s from any telemetry rate above that
        self.next_row_t = max(self.next_row_t + 1.0 / ODOM_HZ, t - 0.5 / ODOM_HZ)
        vals = [t] + [d.get(k, 0.0) for k in ODOM_COLS[1:]]
        self.odom_rows.append([round(v, r) for v, r in zip(vals, _ODOM_ROUND)])
        if len(self.odom_rows) > 100:               # no tick for 2 s: keep the newest
            del self.odom_rows[:-100]

    def on_scan(self, scan):
        if not self.clients:
            return
        lidar = self.b.lidar
        self.n["scans"] += 1
        now = self.clock()
        if not lidar.settings["enabled"]:
            self.n["off"] += 1
            return
        if now - scan.t1 > MAX_SCAN_AGE_S:
            self.n["stale"] += 1
            return
        t0 = time.perf_counter()
        rs, st = bin_scan(scan, lidar.hist.items(), lidar.settings, lidar._mask)
        dt = time.perf_counter() - t0
        self.cpu_s += dt
        self.cost_ms = dt * 1000.0 if not self.cost_ms else 0.9 * self.cost_ms + 0.1 * dt * 1000.0
        if rs is None:
            self.n[st] += 1
            return
        self.seq += 1
        self.n["sent"] += 1
        corr = self.b.mapper.corr
        self._send(_js({"t": "ros_scan", "seq": self.seq, "ts": round(scan.t1, 4), "now": round(now, 4),
                        "t0": round(scan.t0, 4), "tilt": st["tilt"], "tilt_pk": st["tilt_pk"], "n": st["used"],
                        "corr": [round(c, 2) for c in corr], "r": encode_ranges(rs)}))

    # ------------------------------------------------------------ periodic
    def tick(self, now):
        if not self.clients:
            self.odom_rows = []
            return
        t0 = time.perf_counter()
        if self.odom_rows:
            rows, self.odom_rows = self.odom_rows, []
            self._send(_js({"t": "ros_odom", "now": round(now, 4), "rows": rows}))
        if now - self.last_state >= STATE_S:
            self.last_state = now
            self._send(_js(self.state(now)))
        self.cpu_s += time.perf_counter() - t0
        dt = now - self._rate_t0
        if dt >= 5.0:
            nc = max(1, len(self.clients))
            self.kbps = (self.bytes - self._rate_b0) / dt / 1024.0 / nc
            self.cpu_pct = 100.0 * (self.cpu_s - self._rate_c0) / dt
            self._rate_t0, self._rate_b0, self._rate_c0 = now, self.bytes, self.cpu_s

    def state(self, now):
        b = self.b
        t = b.tlm or {}
        st = t.get("status", 0)
        lm = getattr(b.mapper, "lmatch", None)
        return {"t": "ros_state", "now": round(now, 4),
                "link": b.link_state, "alive": bool(t) and now - b.tlm_time < 1.0,
                "mode": lp.MODE_NAMES[t["mode"]] if t and 0 <= t.get("mode", -1) < lp.MODE_COUNT else None,
                "armed": bool(st & lp.TLM_ARMED), "upright": bool(st & lp.TLM_UPRIGHT),
                "obstacle": bool(st & lp.TLM_OBSTACLE), "low_batt": bool(st & lp.TLM_LOW_BATT),
                "estop": bool(b.estop), "robot_estop": bool(b.robot_estop),
                "battery_v": round(t.get("battery_v", 0.0), 2), "remote": bool(t.get("radio_ok", 0)),
                "nav_state": t.get("nav_state"), "corr": [round(c, 2) for c in b.mapper.corr],
                "match": getattr(lm, "state", None) if lm is not None else None,
                "stats": self.stats()}

    def stats(self):
        return {"subscribers": len(self.clients), "kbps": round(self.kbps, 1), "cpu_pct": round(self.cpu_pct, 2),
                "scan_ms": round(self.cost_ms, 2), **self.n}
