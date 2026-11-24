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
     NEWEST telemetry pose; age_ms = how old that pose is now (the firmware dead-reckons that last bit).
  4. the nearest point per 10 deg sector about the axle -> lp.encode_lidar_obstacles.
Nothing is sent while the lidar is silent / stopped, tilted or without a pose: 500 ms after the last frame the firmware
falls back to the sweep, or to its blind rules (EXPLORE pauses, GOTO <= nav.blind_speed).
"""
import math
import time

from . import link_protocol as lp
from .lidar import MIN_POINTS, PITCH_ABS_MAX, POSE_STALE_S, TILT_REF_S, interp

MAX_SCAN_AGE_S = 0.25         # the scan's last packet is older than this when it gets here: not sent


def pitch_ref(hist, settings):
    """Typical pitch, like Lidar.process_scan: median of the last 3 s (or the fixed pitch_ref_deg)."""
    if not settings["pitch_auto_ref"]:
        return settings["pitch_ref_deg"]
    cut = hist[-1][0] - TILT_REF_S
    ps = sorted(h[4] for h in hist[-300:] if h[0] >= cut)
    return ps[len(ps) // 2]


class LidarObstacles:
    def __init__(self, lidar, send, clock=time.monotonic):
        """lidar: lidar.Lidar (settings, mask, pose history; this registers for its scans). send(frame) -> bool."""
        self.lidar = lidar
        self.send = send
        self.clock = clock
        self.seq = 0
        self.n = {"sent": 0, "off": 0, "no_pose": 0, "stale": 0, "tilt": 0, "few": 0, "no_link": 0}
        self.last_sent = None         # clock time of the last frame sent
        self.last_pts = None          # its sectors: [(x, y) or None] * 36, robot frame
        self.last_age_ms = None
        lidar.scan_listeners.append(self.on_scan)

    def on_scan(self, scan):
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
        if now - scan.t1 > MAX_SCAN_AGE_S:
            n["stale"] += 1
            return None
        hist = L.hist.items()
        tco = S["time_offset_ms"] / 1000.0
        if not hist or hist[-1][0] < scan.t1 + tco - POSE_STALE_S:
            n["no_pose"] += 1
            return None
        t_ref, x1, y1, h1, _p1 = hist[-1]               # newest pose: the frame of the points
        age_ms = (now - t_ref) * 1000.0
        if age_ms > lp.LINK_LIDAR_MAX_AGE_MS:
            n["no_pose"] += 1
            return None
        ref = pitch_ref(hist, S)
        # the "project" gate even in "skip" mode: a balancing robot leans this much to speed up / slow down, and the
        # map's 2.5 deg would drop a third of the scans while it drives (then the robot goes blind). Tilted beams
        # still see what is in front of the robot; floor hits are clipped below, ranges are made horizontal.
        limit = max(S["pitch_limit_deg"], S["pitch_max_deg"])
        sgn = 1.0 if S["flip"] else -1.0                # the LD19 counts clockwise; flipped over it counts CCW
        yaw, sx, sy = S["yaw_deg"], S["x_mm"], S["y_mm"]
        rmin, rmax, qmin = S["min_range_mm"], S["max_range_mm"], S["min_intensity"]
        clip, height, deskew = S["floor_clip"], S["height_mm"], S["deskew"]
        mtab = L._mask
        radians, cos, sin, tan, atan2 = math.radians, math.cos, math.sin, math.tan, math.atan2
        c1, s1 = cos(radians(h1)), sin(radians(h1))
        nsec = lp.LINK_LIDAR_SECTORS
        k_sec = nsec / (2.0 * math.pi)
        best = [None] * nsec                            # (r^2, x, y)
        used = 0
        idx = 0
        if not deskew:
            _, px, py, ph, pp = interp(hist, 0, scan.t1 + tco)
        for pk in scan.packets:
            if deskew:
                idx, px, py, ph, pp = interp(hist, idx, pk.t + tco)
            dp = pp - ref
            if abs(dp) > limit or abs(pp) > PITCH_ABS_MAX:
                n["tilt"] += 1                          # one tilted packet: the whole scan is not sent
                return None
            # the lidar origin of this packet, in the robot frame of the newest pose
            hp = radians(ph)
            ch, sh = cos(hp), sin(hp)
            dx, dy = px + ch * sx - sh * sy - x1, py + sh * sx + ch * sy - y1
            ox, oy = dx * c1 + dy * s1, -dx * s1 + dy * c1
            dh = radians(ph - h1)
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
                    d = d * cos(e)                      # horizontal range
                    ae = abs(e)
                    if clip and ae > 1e-4 and d > 0.9 * height / tan(ae):
                        continue
                ang = dh + rb
                x, y = ox + d * cos(ang), oy + d * sin(ang)
                used += 1
                i = int(math.floor(atan2(y, x) * k_sec + 0.5)) % nsec
                r2 = x * x + y * y
                cur = best[i]
                if cur is None or r2 < cur[0]:
                    best[i] = (r2, x, y)
        if used < MIN_POINTS:
            n["few"] += 1
            return None
        pts = [None if b is None else (b[1], b[2]) for b in best]
        self.last_pts = pts
        self.last_age_ms = age_ms
        return lp.encode_lidar_obstacles(self.seq, age_ms, pts)

    def status(self, now=None):
        now = self.clock() if now is None else now
        return {"sent": self.n["sent"], "age_s": None if self.last_sent is None else round(now - self.last_sent, 2),
                "skipped": {k: v for k, v in self.n.items() if k != "sent"}}
