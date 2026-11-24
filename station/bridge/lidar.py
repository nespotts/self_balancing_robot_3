"""LD19 lidar in the bridge (issue #76): turns each 360 degree scan (ld19.py) into occupancy-grid updates.

Per scan (about 450 points, 10 Hz):
  1. pose: the robot pose (odometry, telemetry) is interpolated at the time of every packet (~2.7 ms of the turn), so a
     robot that drives or turns during the 100 ms revolution does not smear the wall ("deskew"). Pose + pitch history
     comes from on_tlm(); the packet times come from the lidar's own timestamps (ld19.Ld19Stream).
  2. per point: range window, intensity, mask sectors (the cage posts, robot frame), pitch gate, floor clip.
  3. world points -> runs (stretches without an angular gap) -> Grid.integrate_scan: free space is the filled polygon
     origin -> points, hits are the point cells (+ lines between neighbours).
Pitch: it is a balancing robot, so the scan plane tilts. A beam at azimuth b (robot frame) points up / down by
pitch_dev * cos(b); pointing down it meets the floor at height / tan(e), and that would look like a wall. So: points of
packets with |pitch - typical pitch| > pitch_limit_deg are dropped ("skip"; "project" keeps them up to pitch_max_deg and
scales the range by cos(e)), and with floor_clip every point beyond 0.9 * height / tan|e| is dropped too.
The typical pitch is the median of the last 3 s, so the IMU mount offset (-3 deg when balanced) does not matter.

Settings (lidar.json in the map folder, like planner.json) hold the mount: fill them in after the mount is built (docs/ld19.md).
Frames: robot frame = x forward, y left (CCW +), origin at the axle centre, like the poses. The LD19 turns clockwise
(manual section 4); its 0 deg is yaw_deg CCW from robot forward.
"""
import collections
import json
import math
import os
import time

from .ld19 import POINT_RATE, Ld19Reader

PITCH_ABS_MAX = 30.0       # IMU pitch beyond this: lying down / carried, never map (same limit as mapping.py)
TILT_REF_S = 3.0
GAP_DEG = 3.0              # neighbours further apart than this (dropped / masked points between) start a new run
MIN_POINTS = 30            # fewer usable points than this: skip the scan
POSE_STALE_S = 0.5         # no telemetry for this long before the scan: pose unknown, skip
FLOWING_S = 1.5

DEFAULTS = {
    "enabled": True,
    # mount (issue #75): lidar axis relative to the axle centre, robot frame
    "x_mm": 0.0,                  # forward of the axle
    "y_mm": 0.0,                  # left of the axle
    "yaw_deg": 0.0,               # direction of the LD19's 0 deg (its front, the side away from the cable) from robot forward, CCW +
    "flip": False,                # mounted upside down: the angles then run counter-clockwise seen from above
    "height_mm": 200.0,           # scan plane above the floor (floor clip when the robot leans)
    # filters
    "min_range_mm": 120.0,        # closer returns are the robot's own cage
    "max_range_mm": 8000.0,       # farther returns are not mapped (LD19 reaches 12 m)
    "min_intensity": 15.0,        # ~200 for white within 6 m; weaker returns are noise
    "mask": [[87.0, 93.0], [267.0, 273.0]],   # [from, to] deg in the robot frame, CCW from forward: the two cage posts at the sides
    # pitch (balancing robot: the scan plane tilts)
    "pitch_limit_deg": 2.5,       # packets leaning more than this from the typical pitch: dropped
    "tilt_mode": "skip",          # "skip" | "project" (keep up to pitch_max_deg, range * cos(elevation))
    "pitch_max_deg": 6.0,
    "pitch_auto_ref": True,       # typical pitch = median of the last 3 s (False: pitch_ref_deg)
    "pitch_ref_deg": 0.0,
    "floor_clip": True,
    # timing / cost
    "deskew": True,               # per-packet pose (False: one pose at the end of the scan)
    "time_offset_ms": 0.0,        # + = the lidar's points are this much later than the telemetry clock says
    "every_n": 1,                 # map every n-th scan (Pi CPU)
    "ray_step": 1,                # use every n-th point (Pi CPU)
}
LIMITS = {"x_mm": (-500, 500), "y_mm": (-500, 500), "yaw_deg": (-360, 360), "height_mm": (20, 1000),
          "min_range_mm": (0, 2000), "max_range_mm": (200, 12000), "min_intensity": (0, 255),
          "pitch_limit_deg": (0.2, 30), "pitch_max_deg": (0.2, 30), "pitch_ref_deg": (-30, 30),
          "time_offset_ms": (-200, 200), "every_n": (1, 20), "ray_step": (1, 8)}
INTS = ("every_n", "ray_step")
CHOICES = {"tilt_mode": ("skip", "project")}
MAX_SECTORS = 8


def clean_settings(s, base):
    """Copy of base with the known keys of s applied (clamped, wrong types skipped)."""
    out = dict(base)
    for k, v in (s if isinstance(s, dict) else {}).items():
        if k not in DEFAULTS:
            continue
        d = DEFAULTS[k]
        if isinstance(d, bool):
            out[k] = bool(v)
        elif k == "mask":
            if isinstance(v, (list, tuple)):
                m = []
                for sec in v[:MAX_SECTORS]:
                    try:
                        a, b = float(sec[0]), float(sec[1])
                    except (TypeError, ValueError, IndexError, KeyError):
                        continue
                    if math.isfinite(a) and math.isfinite(b):
                        m.append([round(a % 360.0, 2), round(b % 360.0, 2)])
                out[k] = m
        elif k in CHOICES:
            if v in CHOICES[k]:
                out[k] = v
        else:
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(v):
                continue
            lo, hi = LIMITS.get(k, (-1e9, 1e9))
            v = max(lo, min(hi, v))
            out[k] = int(round(v)) if k in INTS else v
    return out


def mask_table(sectors):
    """720 flags (0.5 deg steps, robot frame, CCW from forward): 1 = masked. A sector [a, b] runs CCW from a to b
    (so [350, 10] is the 20 deg around forward); a == b masks nothing."""
    t = bytearray(720)
    for a, b in sectors:
        a, b = a % 360.0, b % 360.0
        if a == b:
            continue
        i0, i1 = int(math.floor(a * 2)), int(math.ceil(b * 2))
        if b > a:
            for i in range(i0, min(i1, 720)):
                t[i] = 1
        else:
            for i in range(i0, 720):
                t[i] = 1
            for i in range(0, min(i1, 720)):
                t[i] = 1
    return bytes(t)


class PoseHistory:
    """Recent telemetry poses [(t, x, y, heading deg, pitch deg)] for interpolation at lidar packet times."""

    def __init__(self, maxlen=600):
        self.h = collections.deque(maxlen=maxlen)

    def add(self, t, x, y, heading, pitch):
        if self.h and t <= self.h[-1][0]:
            return
        self.h.append((t, x, y, heading, pitch))

    def items(self):
        return list(self.h)

    def clear(self):
        self.h.clear()


def interp(hist, idx, t):
    """(next idx, x, y, heading, pitch) at time t; hist is time ordered, idx a hint that only moves forward."""
    n = len(hist)
    while idx + 1 < n and hist[idx + 1][0] <= t:
        idx += 1
    a = hist[idx]
    if t <= a[0] or idx + 1 >= n:
        return idx, a[1], a[2], a[3], a[4]
    b = hist[idx + 1]
    f = (t - a[0]) / (b[0] - a[0])
    dh = (b[3] - a[3] + 180.0) % 360.0 - 180.0
    return idx, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dh, a[4] + f * (b[4] - a[4])


class Lidar:
    def __init__(self, mapper, port, say=None, clock=time.monotonic):
        self.mapper = mapper
        self.say = say or (lambda text: None)
        self.clock = clock
        self.port = port or ""
        self.settings = dict(DEFAULTS)
        self.settings["mask"] = [list(s) for s in DEFAULTS["mask"]]
        self._load_settings()
        self._mask = mask_table(self.settings["mask"])
        self.hist = PoseHistory()
        self.reader = Ld19Reader(self.port, self.on_scan, say=self.say, clock=clock)
        self.n = {"scans": 0, "used": 0, "every_n": 0, "no_pose": 0, "few": 0, "points": 0,
                  "range": 0, "intensity": 0, "mask": 0, "tilt": 0, "floor": 0, "no_return": 0}
        self.cpu_s = 0.0
        self.cost_ms = 0.0            # last processed scan, ms
        self.cost_avg_ms = 0.0
        self.cpu_pct = 0.0
        self._cpu_t0 = clock()
        self._cpu_acc = 0.0
        self.last_used = None
        self.tilt_ref = None
        self.last_pts = []            # [[x, y], ...] of the last mapped scan, odometry frame (what the map view draws)
        self.pts_seq = 0
        self.pts_sent = 0
        self.last_send = 0.0
        self.pts_dirty_msg = True
        self.scan_listeners = []      # callables(scan), before the mapping: the navigator feed (lidar_obstacles.py, #77)
        mapper.lidar = self

    # ------------------------------------------------------------ settings
    def _settings_path(self):
        return os.path.join(self.mapper.dir, "lidar.json")

    def _load_settings(self):
        try:
            with open(self._settings_path(), encoding="utf-8") as f:
                self.settings = clean_settings(json.load(f), self.settings)
        except (OSError, ValueError):
            pass

    def apply_settings(self, s, save=True):
        self.settings = clean_settings(s, self.settings)
        self._mask = mask_table(self.settings["mask"])
        if save:
            try:
                os.makedirs(self.mapper.dir, exist_ok=True)
                with open(self._settings_path(), "w", encoding="utf-8") as f:
                    json.dump(self.settings, f, indent=1)
            except OSError:
                pass

    # ------------------------------------------------------------ inputs
    async def run(self):
        await self.reader.run()

    def on_tlm(self, d, t):
        self.hist.add(t, d["x_mm"], d["y_mm"], d["heading_deg"], d["pitch_deg"])

    def flowing(self, now=None):
        """True while scans are being mapped (the VL53L0X sweep is then not needed for the map)."""
        now = self.clock() if now is None else now
        return self.last_used is not None and now - self.last_used < FLOWING_S and self.settings["enabled"]

    def tick(self, now):
        self.reader.tick(now)
        dt = now - self._cpu_t0
        if dt >= 5.0:
            self.cpu_pct = 100.0 * self._cpu_acc / dt
            self._cpu_acc, self._cpu_t0 = 0.0, now

    def on_scan(self, scan):
        for f in self.scan_listeners:
            try:
                f(scan)
            except Exception as e:                  # a listener bug must not stop the mapping
                self.reader.error = "scan listener: %s: %s" % (type(e).__name__, e)
        t0 = time.perf_counter()
        try:
            self.process_scan(scan)
        finally:
            dt = time.perf_counter() - t0
            self._cpu_acc += dt

    # ------------------------------------------------------------ one scan
    def process_scan(self, scan):
        """Scan -> grid. Returns the number of points mapped (0 = skipped); counters in .n say why."""
        S = self.settings
        n = self.n
        n["scans"] += 1
        if not S["enabled"] or not self.mapper.settings["enabled"]:
            return 0
        if S["every_n"] > 1 and n["scans"] % S["every_n"]:
            n["every_n"] += 1
            return 0
        t0 = time.perf_counter()
        hist = self.hist.items()
        tco = S["time_offset_ms"] / 1000.0
        if not hist or hist[-1][0] < scan.t0 + tco - POSE_STALE_S:
            n["no_pose"] += 1
            return 0
        if S["pitch_auto_ref"]:
            cut = hist[-1][0] - TILT_REF_S
            ps = sorted(h[4] for h in hist[-300:] if h[0] >= cut)
            ref = ps[len(ps) // 2]
        else:
            ref = S["pitch_ref_deg"]
        self.tilt_ref = ref
        yaw = S["yaw_deg"]
        sgn = 1.0 if S["flip"] else -1.0            # the LD19 counts clockwise; flipped over it counts CCW
        sx, sy = S["x_mm"], S["y_mm"]
        rmin, rmax, qmin = S["min_range_mm"], S["max_range_mm"], S["min_intensity"]
        mtab = self._mask
        limit = S["pitch_limit_deg"]
        project = S["tilt_mode"] == "project"
        if project:
            limit = max(limit, S["pitch_max_deg"])
        clip = S["floor_clip"]
        height = S["height_mm"]
        step = S["ray_step"]
        deskew = S["deskew"]
        map_pose = self.mapper.map_pose
        radians, degrees, cos, sin, tan = math.radians, math.degrees, math.cos, math.sin, math.tan
        runs, run, pts_all = [], [], []
        prev_a = None
        idx = 0
        c_tot = c_range = c_int = c_mask = c_tilt = c_floor = c_zero = 0
        # pose at the end of the scan for a non-deskewed scan
        if not deskew:
            _, x1, y1, h1, p1 = interp(hist, 0, scan.t1 + tco)
        for pk in scan.packets:
            if deskew:
                idx, px, py, ph, pp = interp(hist, idx, pk.t + tco)
            else:
                px, py, ph, pp = x1, y1, h1, p1
            dp = pp - ref
            c_tot += 12
            if abs(dp) > limit or abs(pp) > PITCH_ABS_MAX:
                c_tilt += 12
                continue
            mx, my, mh = map_pose(px, py, ph)
            hr = radians(mh)
            ch, sh = cos(hr), sin(hr)
            ox, oy = mx + ch * sx - sh * sy, my + sh * sx + ch * sy
            dpr = radians(dp)
            k = 0
            for a, d, q in pk.pts:
                k += 1
                if step > 1 and k % step:
                    continue
                if d <= 0:
                    c_zero += 1
                    continue
                if d < rmin or d > rmax:
                    c_range += 1
                    continue
                if q < qmin:
                    c_int += 1
                    continue
                b = yaw + sgn * a
                if mtab[int((b % 360.0) * 2.0)]:
                    c_mask += 1
                    continue
                rb = radians(b)
                if dpr:
                    e = dpr * cos(rb)               # beam elevation at this azimuth (+ = down or up, symmetric)
                    if project:
                        d = d * cos(e)
                    ae = abs(e)
                    if clip and ae > 1e-4 and d > 0.9 * height / tan(ae):
                        c_floor += 1
                        continue
                gap = prev_a is None or (a - prev_a) % 360.0 > GAP_DEG
                prev_a = a
                ang = hr + rb
                p = (ox + d * cos(ang), oy + d * sin(ang))
                if gap and run:
                    runs.append(run)
                    run = []
                run.append(p)
                pts_all.append(p)
        if run:
            runs.append(run)
        n["points"] += c_tot
        n["no_return"] += c_zero
        n["range"] += c_range
        n["intensity"] += c_int
        n["mask"] += c_mask
        n["tilt"] += c_tilt
        n["floor"] += c_floor
        if len(pts_all) < MIN_POINTS:
            n["few"] += 1
            return 0
        if deskew:                                  # the fan of the free-space polygon starts at the mid-scan pose
            _, xm, ym, hm, _ = interp(hist, 0, 0.5 * (scan.t0 + scan.t1) + tco)
        else:
            xm, ym, hm = x1, y1, h1
        mx, my, mh = map_pose(xm, ym, hm)
        hr = radians(mh)
        ox, oy = mx + cos(hr) * sx - sin(hr) * sy, my + sin(hr) * sx + cos(hr) * sy
        lm = getattr(self.mapper, "lmatch", None)
        if lm is not None:          # scan-to-map matching (#78): may correct the pose; the turn then maps at the new one
            ox, oy, mx, my, runs, pts_all = lm.on_scan(ox, oy, mx, my, mh, runs, pts_all)
        if runs is not None:        # (None: the matcher keeps this turn out of the map, see lidarmatch._map_now)
            self.mapper.integrate_scan(ox, oy, runs, (mx, my))
        n["used"] += 1
        self.last_used = self.clock()
        self._keep_points(pts_all)
        self.cost_ms = (time.perf_counter() - t0) * 1000.0
        self.cost_avg_ms = self.cost_ms if not self.cost_avg_ms else 0.9 * self.cost_avg_ms + 0.1 * self.cost_ms
        return len(pts_all)

    def _keep_points(self, pts):
        """For the map view: the mapped points, back in the odometry frame (the robot / track are drawn there)."""
        dx, dy, dt = self.mapper.corr
        if dx or dy or dt:
            c, s = math.cos(math.radians(-dt)), math.sin(math.radians(-dt))
            pts = [(c * (x - dx) - s * (y - dy), s * (x - dx) + c * (y - dy)) for x, y in pts]
        self.last_pts = [[int(x), int(y)] for x, y in pts]
        self.pts_seq += 1

    # ------------------------------------------------------------ browsers
    def status(self, now=None):
        now = self.clock() if now is None else now
        st = self.reader.stats(now)
        st.update(enabled=self.settings["enabled"], flowing=self.flowing(now), used=self.n["used"],
                  cpu_pct=round(self.cpu_pct, 1), cost_ms=round(self.cost_avg_ms, 1),
                  tilt_ref=None if self.tilt_ref is None else round(self.tilt_ref, 2),
                  dropped={k: self.n[k] for k in ("range", "intensity", "mask", "tilt", "floor", "no_return")})
        return st

    def msg(self):
        return json.dumps({"t": "lidar", "settings": self.settings, "defaults": DEFAULTS, "status": self.status()},
                          separators=(",", ":"))

    def poll(self, now):
        """Messages to broadcast now: the live scan points, <= 3 Hz, only when a new scan was mapped."""
        if self.pts_seq == self.pts_sent or now - self.last_send < 0.33:
            return []
        self.last_send = now
        self.pts_sent = self.pts_seq
        flat = []
        for x, y in self.last_pts:
            flat.append(x)
            flat.append(y)
        return [json.dumps({"t": "lidar_scan", "seq": self.pts_seq, "pts": flat}, separators=(",", ":"))]

    def command(self, m, who="?"):
        """{"t": "lidar", "action": "set", "settings": {...}} | "get" | "defaults" -> reply or None."""
        act = m.get("action")
        if act == "set":
            self.apply_settings(m.get("settings") or {})
            self.say("%s: lidar settings changed" % who)
        elif act == "defaults":
            self.apply_settings(DEFAULTS)
            self.say("%s: lidar settings back to defaults" % who)
        elif act != "get":
            return {"t": "error", "text": "unknown lidar action %r" % act}
        return json.loads(self.msg())
