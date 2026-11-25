"""LD19 obstacles for the robot's own navigator (issue #77): one MSG_LIDAR_OBSTACLES per scan (~10 Hz).

The Teensy's navigator (src/navigator.h) stops / slows / steers off walls with these instead of the VL53L0X sweep.
Per scan (the same packets lidar.py maps, before its every_n / ray_step / mapping switches, which are map-only):
  1. gates: lidar enabled, a pose from telemetry that is at most LINK_LIDAR_MAX_AGE_MS old, the scan itself handled
     within MAX_SCAN_AGE_S of its last packet (no backlog), and EVERY packet inside the pitch gate: a dropped packet
     would leave its sectors looking open, so a scan with one is not sent at all. The gate is the wider of
     pitch_limit_deg / pitch_max_deg (lidar settings; the "project" limit, 6 deg by default) from the typical pitch,
     whatever tilt_mode the map uses: the map's 2.5 deg would drop every scan taken while the robot speeds up.
     At least MIN_POINTS usable points.
  2. per point the lidar.py filters: range window, intensity, mask, floor clip; range * cos(elevation) (horizontal).
  3. deskew: each packet's pose (telemetry interpolated) -> the point in the odometry frame -> the robot frame of the
     NEWEST telemetry pose; age_ms = how old that pose is now (the firmware dead-reckons that last bit). Steps 1-3 are
     lidar.Lidar.deskew (scan.turn), done once per turn for the map and the matcher too (#110).
  4. one point per 10 deg sector about the axle -> lp.encode_lidar_obstacles: the nearest point INSIDE the firmware's
     driving corridor (|y| <= its half width, nearest by the front / rear distance the firmware computes from it), else
     the nearest point (#99). Why: the firmware's front_ / rear_ are the minimum over the corridor points it gets, so
     with this rule they come out exactly as from the whole scan (a side-wall return nearer than an in-path post in the
     same sector can no longer hide the post). Side walls lose nothing they need: a sector keeps an in-corridor point
     over a nearer outside one only where the corridor point is itself close (a side point the firmware uses is
     < 300 mm ahead and > 16 deg off the axis; an in-corridor point in that sector is then < 321 mm ahead, a front
     distance < 282 mm, inside rng.obstacle_mm 300), so the robot is stopped by it anyway. Finer forward sectors
     would still hide a post behind a wall point in the same (narrower) sector, and would take sectors from the sides.
Nothing is sent while the lidar is silent / stopped, tilted or without a pose: 500 ms after the last frame the firmware
falls back to the sweep, or to its blind rules (EXPLORE pauses, GOTO <= nav.blind_speed). Nor before the robot profile is
bound (#97: lidar.json, the mount settings, is loaded then), see ready.

Link v10 (#113): firmware whose HELLO has LINK_CAP_LIDAR_SECTORS computes these sectors itself (src/lidar_sectors.cpp, a
port of this file + lidar.Lidar.deskew + ld19.Ld19Stream, checked against them by test/host/lidar_sectors_replay.py). It
then gets no frames from here (see teensy): only the robot profile's lidar settings, MSG_LIDAR_CONFIG, sent by tick() once
the profile is bound, at once when they change and every CONFIG_RESEND_S (a lost frame, a Teensy reboot). Older firmware
(no caps) gets the frames as before.
"""
import cmath
import math
import operator
import time
from itertools import compress, groupby, repeat

from . import link_protocol as lp
from .lidar import MIN_POINTS, POSE_STALE_S, TILT_REF_S

_mul, _sub, _add, _mod, _le = operator.mul, operator.sub, operator.add, operator.mod, operator.le
_IM = operator.attrgetter("imag")
_INF = float("inf")

MAX_SCAN_AGE_S = 0.25         # the scan's last packet is older than this when it gets here: not sent
CONFIG_RESEND_S = 2.0         # MSG_LIDAR_CONFIG again this often (#113)
# src/navigator.cpp: kAxisFwd, kSensorOut, kHalfWidth (front = x - 38.9 for x >= 26, rear = 13.1 - x)
AXIS_FWD_MM, SENSOR_OUT_MM, HALF_WIDTH_MM = 26.0, 12.9, 76.0
FRONT_OFF_MM, REAR_OFF_MM = AXIS_FWD_MM + SENSOR_OUT_MM, AXIS_FWD_MM - SENSOR_OUT_MM
OUTSIDE_KEY = 1e9             # sort key offset of points outside the corridor (in-corridor keys are < 13000)
NAV_DEFAULTS = {"nav.corridor_mm": 220.0, "nav.side_min_mm": 120.0}   # src/params.h


def corridor_half(params):
    """The corridor half width navigator.cpp updateObstacles uses, from the robot's params (defaults if not known)."""
    def get(k):
        try:
            v = float(params.get(k, NAV_DEFAULTS[k]))
        except (TypeError, ValueError, AttributeError):
            return NAV_DEFAULTS[k]
        return v if math.isfinite(v) else NAV_DEFAULTS[k]
    c, smin = get("nav.corridor_mm"), get("nav.side_min_mm")
    return min(0.5 * c, HALF_WIDTH_MM + 10.0) if smin > 0 else 0.5 * c


def pitch_ref(hist, settings):
    """Typical pitch, like Lidar.process_scan: median of the last 3 s (or the fixed pitch_ref_deg)."""
    if not settings["pitch_auto_ref"]:
        return settings["pitch_ref_deg"]
    cut = hist[-1][0] - TILT_REF_S
    ps = sorted(h[4] for h in hist[-300:] if h[0] >= cut)
    return ps[len(ps) // 2]


class LidarObstacles:
    def __init__(self, lidar, send, clock=time.monotonic, params=None, ready=None, teensy=None):
        """lidar: lidar.Lidar (settings, mask, pose history; this registers for its scans). send(frame) -> bool.
        params() -> {name: value} of the robot's params (the corridor, #99); ready() -> False while the robot profile
        (and with it lidar.json, the mount) is not bound yet (#97): nothing is sent then. None = always ready.
        teensy() -> True when the firmware computes the sectors itself (HELLO caps, link v10, #113): no frames then,
        tick() sends it the settings instead. None = never (older firmware)."""
        self.lidar = lidar
        self.send = send
        self.clock = clock
        self.params = params
        self.ready = ready
        self.teensy = teensy
        self.seq = 0
        self.n = {"sent": 0, "off": 0, "unbound": 0, "no_pose": 0, "stale": 0, "tilt": 0, "few": 0, "no_link": 0,
                  "teensy": 0}
        self.last_sent = None         # clock time of the last frame sent
        self.last_pts = None          # its sectors: [(x, y) or None] * 36, robot frame
        self.last_age_ms = None
        self.cfg_payload = None       # the last MSG_LIDAR_CONFIG payload sent (#113) ...
        self.cfg_t = None             # ... and when (None: send at the next tick)
        self.cfg_sent = 0
        lidar.scan_listeners.append(self.on_scan)

    def on_teensy(self):
        """The firmware computes the sectors itself (#113)."""
        try:
            return bool(self.teensy and self.teensy())
        except Exception:
            return False

    def config_reset(self):
        """A HELLO (a new connection or a Teensy reboot, #113): its settings are sent again at the next tick."""
        self.cfg_t = None

    def tick(self, now=None):
        """MSG_LIDAR_CONFIG to firmware that computes the sectors (#113): once the robot profile is bound, at once when the
        settings change, and every CONFIG_RESEND_S. True = one was sent."""
        if not self.on_teensy():
            return False
        if self.ready is not None and not self.ready():
            return False
        now = self.clock() if now is None else now
        p = lp.lidar_config_payload(self.lidar.settings)
        if p == self.cfg_payload and self.cfg_t is not None and now - self.cfg_t < CONFIG_RESEND_S:
            return False
        if not self.send(lp.encode_frame(lp.MSG_LIDAR_CONFIG, p)):
            return False
        self.cfg_payload, self.cfg_t = p, now
        self.cfg_sent += 1
        return True

    def on_scan(self, scan):
        if self.on_teensy():
            self.n["teensy"] += 1                       # the robot computes its own sectors (#113): nothing to do here
            return False
        frame = self.build(scan)
        if frame is None:
            return False
        if not self.send(frame):
            self.n["no_link"] += 1
            return False
        self.n["sent"] += 1
        self.seq = (self.seq + 1) & 0xFF
        self.last_sent = self.clock()
        return True

    def build(self, scan, now=None):
        """This scan's frame (bytes), or None (the counter in .n says why)."""
        L = self.lidar
        S = L.settings
        n = self.n
        now = self.clock() if now is None else now
        if not S["enabled"]:
            n["off"] += 1
            return None
        if self.ready is not None and not self.ready():
            n["unbound"] += 1                           # the mount settings of this robot are not loaded yet
            return None
        if now - scan.t1 > MAX_SCAN_AGE_S:
            n["stale"] += 1
            return None
        turn = getattr(scan, "turn", False)
        if turn is False:                               # (a scan that did not come through Lidar.on_scan)
            turn = scan.turn = L.deskew(scan)
        tco = S["time_offset_ms"] / 1000.0
        if turn is None or turn.newest[0] < scan.t1 + tco - POSE_STALE_S:
            n["no_pose"] += 1
            return None
        t_ref, x1, y1, h1 = turn.newest                 # newest pose: the frame of the points
        age_ms = (now - t_ref) * 1000.0
        if age_ms > lp.LINK_LIDAR_MAX_AGE_MS:
            n["no_pose"] += 1
            return None
        if turn.tilted:
            n["tilt"] += 1                              # one tilted packet: the whole scan is not sent
            return None
        z = turn.z
        if len(z) < MIN_POINTS:
            n["few"] += 1
            return None
        nsec = lp.LINK_LIDAR_SECTORS
        half = corridor_half(self.params() if self.params else {})
        # the deskewed points (odometry frame) in the robot frame of the newest pose, about the axle (#110: from the
        # turn the map uses too; per point only C-level map() calls)
        zr = list(map(_mul, map(_sub, z, repeat(complex(x1, y1))), repeat(cmath.rect(1.0, -math.radians(h1)))))
        dist = list(map(abs, zr))
        # sector of each point: floor(angle / 10 deg + 0.5) mod 36 (int() of a positive number is its floor)
        k_sec = nsec / (2.0 * math.pi)
        secs = list(map(_mod, map(int, map(_add, map(_mul, map(cmath.phase, zr), repeat(k_sec)), repeat(nsec + 0.5))),
                        repeat(nsec)))
        best_d = [_INF] * nsec
        best_i = [-1] * nsec
        i = 0
        get = dist.__getitem__
        for sec, grp in groupby(secs):                  # the points come round in order: a run per sector, mostly
            m = len(list(grp))
            j = min(range(i, i + m), key=get) if m > 1 else i
            if dist[j] < best_d[sec]:
                best_d[sec], best_i[sec] = dist[j], j
            i += m
        # in the corridor (|y| <= half): nearest by the firmware's front / rear distance, before any outside point
        ckey = [None] * nsec
        for j in compress(range(len(zr)), map(_le, map(abs, map(_IM, zr)), repeat(half))):
            x = zr[j].real
            key = x - FRONT_OFF_MM if x >= AXIS_FWD_MM else REAR_OFF_MM - x
            sec = secs[j]
            if ckey[sec] is None or key < ckey[sec]:
                ckey[sec], best_i[sec] = key, j
        best = [None if j < 0 else (0, zr[j].real, zr[j].imag) for j in best_i]
        pts = [None if b is None else (b[1], b[2]) for b in best]
        self.last_pts = pts
        self.last_age_ms = age_ms
        return lp.encode_lidar_obstacles(self.seq, age_ms, pts)

    def status(self, now=None):
        now = self.clock() if now is None else now
        return {"sent": self.n["sent"], "age_s": None if self.last_sent is None else round(now - self.last_sent, 2),
                "skipped": {k: v for k, v in self.n.items() if k != "sent"},
                "on_teensy": self.on_teensy(), "config_sent": self.cfg_sent,          # #113
                "config_age_s": None if self.cfg_t is None else round(now - self.cfg_t, 2)}
