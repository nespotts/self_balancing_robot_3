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

Pi cost (#110): a turn is deskewed ONCE (Lidar.deskew -> Turn: the filtered points in the odometry frame) and shared by
the navigator's obstacle frame (lidar_obstacles.py), the scan matcher and the map. Points are complex numbers (x + iy):
a packet's 12 beam directions are one rotation of a cached table, its points one map() over the distances, so the
per-point work runs in C; only packets with a dropped point (no return, range, intensity, a mask sector, the floor clip)
take the per-point path. The map frame (the matcher's correction) is applied when the turn is matched, not when it
arrives: a correction made by the turn before (its callbacks may still be running) then counts (#110).
Pose times: telemetry is stamped with the robot's own clock (t_us, RobotClock), anchored to the Pi clock by the least
latency seen, like the LD19's packets. Its arrival time is no good: after a loop stall the frames come in a burst, and a
pose interpolated at a lidar packet's time was then up to ~300 ms old (seen on the robot 2026-11-25: smeared turns,
bad matches, false wheel slips).
"""
import asyncio
import cmath
import collections
import itertools
import json
import math
import operator
import os
import re
import time

from .ld19 import POINT_RATE, POINTS, TEENSY, Ld19Reader
from . import loopprof
from .lidarmatch import ODO_JUMP_MM
from .link_protocol import LINK_LIDAR_RAW_MAX

PITCH_ABS_MAX = 30.0       # IMU pitch beyond this: lying down / carried, never map (same limit as mapping.py)
TILT_REF_S = 3.0
GAP_DEG = 3.0              # neighbours further apart than this (dropped / masked points between) start a new run
MIN_POINTS = 30            # fewer usable points than this: skip the scan
POSE_STALE_S = 0.5         # no telemetry for this long before the scan: pose unknown, skip
FLOWING_S = 1.5
HEAD_JUMP_DEG = 30.0       # telemetry heading step this much beyond yaw rate x dt: a pose reset, not a turn (#99)
_FW_STAT_RE = re.compile(r"^lidar: .*usb drop (\d+), overflow (\d+)")
_FW_SECT_RE = re.compile(r"; sectors ([\d.]+)/s.*max (\d+) us/slice (\d+) us/turn")

DEFAULTS = {
    "enabled": True,
    # mount (issue #75): lidar axis relative to the axle centre, robot frame
    "x_mm": 0.0,                  # forward of the axle
    "y_mm": 0.0,                  # left of the axle
    "yaw_deg": 0.0,               # direction of the LD19's 0 deg (the arrow on its top; 90 deg CCW from the cable seen from above,
                                  # checked on the real unit 2026-11-24) from robot forward, CCW +. Mount #75 (cable toward the Pi): -90
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
MATCH_SLICE_S = 0.01       # a match step's time (Pi): then the loop gets a turn (#110; loop_bench --pi-budget scales it)
PART_POINTS = 100          # the bridge puts a turn into the grid in parts of this many points (#102; #110: ~10 ms each on the Pi)


def split_runs(runs, n=PART_POINTS):
    """Runs -> parts (lists of runs) of about n points. A run longer than that is cut with one shared point, so the free
    space fans of the two pieces meet edge to edge (the scanline fill is half-open: no cell twice)."""
    chunks = []
    for r in runs:
        if len(r) <= n:
            chunks.append(r)
        else:
            chunks += [r[j:j + n + 1] for j in range(0, len(r) - 1, n)]
    parts, cur, cnt = [], [], 0
    for c in chunks:
        if cur and cnt + len(c) > n:
            parts.append(cur)
            cur, cnt = [], 0
        cur.append(c)
        cnt += len(c)
    if cur:
        parts.append(cur)
    return parts


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


def mask_cum(tab):
    """Prefix sums of a mask table: cum[i] = masked entries before i (is a packet's span clear of every mask?)."""
    return list(itertools.accumulate(tab, initial=0))


class RobotClock:
    """Telemetry timestamps (#110): the robot's t_us (u32 microseconds) -> Pi clock seconds. The offset is the least
    (arrival - robot time) seen, allowed to creep up 500 ppm (clock rates differ), as ld19.Ld19Stream does for the
    LD19's own clock; both clocks then say when a thing was measured (+ the link's least latency, ~1-3 ms). A t_us that
    goes back (a Teensy reboot) starts it over; one that wraps (every 71.6 min) is unwrapped."""
    CREEP = 5e-4
    WRAP_S = (1 << 32) / 1e6

    def __init__(self):
        self.reset()

    def reset(self):
        self.off = None
        self.last_us = None
        self.wraps = 0
        self.last_now = None

    def map(self, t_us, now):
        if t_us is None:
            return now
        last = self.last_us
        if last is not None and t_us < last:
            if last - t_us > (1 << 31):
                self.wraps += 1                    # the u32 wrapped
            else:
                self.reset()                       # the robot rebooted: its clock starts over
        self.last_us = t_us
        rt = t_us / 1e6 + self.wraps * self.WRAP_S
        cand = now - rt
        off = self.off
        if off is None or cand < off:
            off = cand
        else:
            off = min(cand, off + self.CREEP * max(0.0, now - (self.last_now if self.last_now is not None else now)))
        self.off, self.last_now = off, now
        return rt + off


_RE = operator.attrgetter("real")
_IM = operator.attrgetter("imag")
_mul, _add = operator.mul, operator.add
_INF = float("inf")


def complex_to_xy(zs):
    """[complex] -> [(x, y)] (C speed)."""
    return list(zip(map(_RE, zs), map(_IM, zs)))


class Turn:
    """One LD19 revolution deskewed once (#110), shared by the obstacle frame, the matcher and the map.
    z: the points that pass the point filters, odometry frame, complex mm (horizontal range), scan order, from every
       packet inside the wider (obstacle) pitch gate; runs: [(i0, i1)] slices of z for the map (packets inside its own
       pitch gate, cut at angular gaps > GAP_DEG); tilted: a packet beyond the wide gate (no obstacle frame);
    counts: the map's filter counters for this turn; newest: (t, x, y, heading) of the newest telemetry pose;
    mid: the robot's odometry pose (x, y, heading) at mid-turn and origin (complex) the lidar axis then."""
    __slots__ = ("scan", "z", "runs", "tilted", "counts", "ref", "newest", "mid", "origin", "map_n")

    def __init__(self):
        self.z = []
        self.runs = []
        self.tilted = False
        self.counts = None
        self.map_n = 0


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


RAD = math.pi / 180.0
_P4 = operator.itemgetter(4)


def pitch_ref(hist, settings):
    """Typical pitch: the median of the last TILT_REF_S of the history (<= 300 entries), or the fixed pitch_ref_deg."""
    if not settings["pitch_auto_ref"]:
        return settings["pitch_ref_deg"]
    cut = hist[-1][0] - TILT_REF_S
    tail = hist[-300:]
    i = _seek(tail, cut)
    if tail[i][0] < cut:
        i += 1
    ps = sorted(map(_P4, tail[i:]))
    return ps[len(ps) // 2]


def _seek(hist, t):
    """Index of the last history entry at or before t (0 if none): where interp() starts (binary search, #110)."""
    lo, hi = 0, len(hist) - 1
    while lo < hi:
        mid = (lo + hi + 1) >> 1
        if hist[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return lo


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
        self._set_mask()
        self.hist = PoseHistory()
        self.rclock = RobotClock()    # telemetry stamped with the robot's clock (#110)
        self._rp = {}                 # beam step (deg) -> [unit rotation by k steps, k = 0 .. 11] (#110)
        self._last_raw = None         # (map-frame points, correction then) of the last turn: last_pts, built when needed
        self._last_cache = (None, [])
        self._matching = False        # a turn's match is running in steps (#110) ...
        self._match_q = collections.deque(maxlen=2)   # ... and the turns waiting for it (a backlog keeps the newest)
        # port "teensy": only the LD19 bytes the Teensy forwards (MSG_LIDAR_RAW, #100). A serial port (USB adapter,
        # UART, the sim's tcp://): that, and the Teensy's frames whenever the port has no valid packets (auto).
        self.reader = Ld19Reader("" if self.port.lower() == TEENSY else self.port, self.on_scan, say=self.say,
                                 clock=clock, teensy=True)
        self.n = {"scans": 0, "used": 0, "every_n": 0, "no_pose": 0, "few": 0, "points": 0,
                  "range": 0, "intensity": 0, "mask": 0, "tilt": 0, "floor": 0, "no_return": 0, "pose_jumps": 0}
        self.cpu_s = 0.0
        self.cost_ms = 0.0            # last processed scan, ms
        self.cost_avg_ms = 0.0
        self.cpu_pct = 0.0
        self._cpu_t0 = clock()
        self._cpu_acc = 0.0
        self.last_used = None
        self.tilt_ref = None
        self.pts_seq = 0              # last_pts: [[x, y], ...] of the last mapped scan, odometry frame (the map view)
        self.pts_sent = 0
        self.last_send = 0.0
        self.pts_dirty_msg = True
        self.scan_listeners = []      # callables(scan), before the mapping: the navigator feed (lidar_obstacles.py, #77)
        self.prof = loopprof.NULL     # the bridge's loop profile (#102)
        # the bridge sets this (#102): a turn's filter + match and its grid update run as separate event-loop callbacks
        # after the navigator feed, so the loop (heartbeat, the link's reads) gets a turn in between (~20-60 ms each on
        # the Pi). False (tests, tools): everything at once, as before.
        self.defer = False
        mapper.lidar = self

    # ------------------------------------------------------------ settings
    def _set_mask(self):
        self._mask = mask_table(self.settings["mask"])
        self._mcum = mask_cum(self._mask)
        self._masked = any(self._mask)

    def _settings_path(self):
        return os.path.join(self.mapper.dir, "lidar.json") if self.mapper.dir else None

    def _load_settings(self):
        try:
            with open(self._settings_path(), encoding="utf-8") as f:
                self.settings = clean_settings(json.load(f), self.settings)
        except (OSError, ValueError, TypeError):
            pass

    def reload_settings(self):
        """The mapper moved to another robot profile (#97): defaults, then that folder's lidar.json."""
        self.settings = dict(DEFAULTS)
        self.settings["mask"] = [list(s) for s in DEFAULTS["mask"]]
        self._load_settings()
        self._set_mask()

    def apply_settings(self, s, save=True):
        self.settings = clean_settings(s, self.settings)
        self._set_mask()
        if save and self.mapper.dir:
            try:
                os.makedirs(self.mapper.dir, exist_ok=True)
                with open(self._settings_path(), "w", encoding="utf-8") as f:
                    json.dump(self.settings, f, indent=1)
            except OSError:
                pass

    # ------------------------------------------------------------ inputs
    async def run(self):
        await self.reader.run()

    def on_raw(self, payload):
        """A MSG_LIDAR_RAW frame from the robot link (#100): the Teensy's Serial7 bytes into the same LD19 parser."""
        try:
            seq, flags, data = payload[0], payload[1], bytes(payload[2:])
        except IndexError:
            data = b""
        if not data or len(data) > LINK_LIDAR_RAW_MAX:
            self.reader.raw["bad"] += 1
            return
        self.reader.feed_teensy(seq, flags, data)

    def on_robot_log(self, text):
        """The firmware's "lidar: ... usb drop N, overflow M ..." line: its own counters for the Health tile."""
        m = _FW_STAT_RE.search(text)
        if m:
            self.reader.fw = {"usb_drop": int(m.group(1)), "overflow": int(m.group(2))}
            s = _FW_SECT_RE.search(text)      # link v10 (#113): the Teensy's own obstacle sectors, rate and cost
            if s:
                self.reader.fw.update(sectors_hz=float(s.group(1)), sectors_slice_us=int(s.group(2)),
                                      sectors_turn_us=int(s.group(3)))

    def on_tlm(self, d, t):
        """One telemetry frame; t its arrival time. With the robot's t_us the pose is stamped with when it was measured
        (RobotClock, #110), not with when the bridge got round to it."""
        t = self.rclock.map(d.get("t_us"), t)
        h = self.hist.h
        if h:
            # an odometry jump (pose reset, #99; its "pose reset" log line may come after the telemetry): never
            # interpolate a scan across it. Same test as lidarmatch (ODO_JUMP_MM), plus a heading step that the yaw
            # rate does not explain.
            t0, x0, y0, h0, _p0 = h[-1]
            dh = (d["heading_deg"] - h0 + 180.0) % 360.0 - 180.0
            if (math.hypot(d["x_mm"] - x0, d["y_mm"] - y0) > ODO_JUMP_MM
                    or abs(dh - d.get("yaw_rate_dps", 0.0) * max(0.0, t - t0)) > HEAD_JUMP_DEG):
                self.hist.clear()
                self.n["pose_jumps"] += 1
        self.hist.add(t, d["x_mm"], d["y_mm"], d["heading_deg"], d["pitch_deg"])
        lm = getattr(self.mapper, "lmatch", None)
        if lm is not None:
            lm.on_tlm(d, t)                         # the robot's mode, for the lidar stuck check (#79)

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
        """A whole turn (from the LD19 parser, inside the link's read callback): the rest in its own loop callback
        (#110: the read callback stays short)."""
        loop = self._loop()
        if loop is not None:
            loop.call_soon(self._on_turn, scan)
        else:
            self._on_turn(scan)

    def _on_turn(self, scan):
        """Deskewed once (scan.turn, #110), then the listeners (the navigator's obstacle frame) at once; the map and
        the matcher get it in their own loop callbacks."""
        t0 = time.perf_counter()
        try:
            with self.prof.part("lidar.deskew"):
                scan.turn = self.deskew(scan) if self.settings["enabled"] else None
        except Exception as e:                      # a bug here must not stop the reader
            scan.turn = None
            self.reader.error = "deskew: %s: %s" % (type(e).__name__, e)
        self._cpu_acc += time.perf_counter() - t0
        for f in self.scan_listeners:
            try:
                with self.prof.part("scan:" + type(getattr(f, "__self__", f)).__name__):
                    f(scan)
            except Exception as e:                  # a listener bug must not stop the mapping
                self.reader.error = "scan listener: %s: %s" % (type(e).__name__, e)
        loop = self._loop()
        if loop is not None:
            loop.call_soon(self._process_timed, scan)
        else:
            self._process_timed(scan)

    def _loop(self):
        if not self.defer:
            return None
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return None

    def _process_timed(self, scan):
        t0 = time.perf_counter()
        try:
            with self.prof.part("lidar.scan"):
                self.process_scan(scan)
        finally:
            dt = time.perf_counter() - t0
            self._cpu_acc += dt

    def _integrate(self, ox, oy, runs, robot_xy, version=None):
        """The turn into the grid; version: deferred (the map was reset / loaded meanwhile: dropped), then in parts."""
        if version is not None and version != self.mapper.version:
            return
        if version is not None:
            parts = split_runs(runs)
            if len(parts) > 1:
                g = self.mapper.grid
                pts = [p for r in runs for p in r]
                g._cover([p[0] for p in pts] + [ox], [p[1] for p in pts] + [oy])   # grown once, before the parts
                self._integrate_part(ox, oy, parts, 0, robot_xy, version, set(), g.cells)
                return
        t0 = time.perf_counter()
        with self.prof.part("lidar.integrate"):
            self.mapper.integrate_scan(ox, oy, runs, robot_xy)
        ms = (time.perf_counter() - t0) * 1000.0
        if version is not None:                     # deferred: not in the turn's time yet
            self._cpu_acc += ms / 1000.0
        self.cost_ms += ms
        self.cost_avg_ms += 0.1 * ms

    def _integrate_part(self, ox, oy, parts, k, robot_xy, version, seen, cells):
        if version != self.mapper.version:
            return
        if self.mapper.grid.cells is not cells:     # the grid grew meanwhile: the cell indices in seen moved
            seen = set()
        t0 = time.perf_counter()
        last = k == len(parts) - 1
        with self.prof.part("lidar.integrate"):
            self.mapper.integrate_scan(ox, oy, parts[k], robot_xy, seen=seen, last=last)
        ms = (time.perf_counter() - t0) * 1000.0
        self._cpu_acc += ms / 1000.0
        self.cost_ms += ms
        self.cost_avg_ms += 0.1 * ms
        if not last:
            loop = self._loop()
            args = (ox, oy, parts, k + 1, robot_xy, version, seen, self.mapper.grid.cells)
            if loop is None:
                self._integrate_part(*args)
            else:
                loop.call_soon(self._integrate_part, *args)

    # ------------------------------------------------------------ one scan
    def deskew(self, scan):
        """The turn's filtered points in the odometry frame, each packet at its own pose (Turn, see there; #110: once
        per turn for everybody). None without any telemetry pose."""
        hist = self.hist.items()
        if not hist:
            return None
        S = self.settings
        tco = S["time_offset_ms"] / 1000.0
        ref = pitch_ref(hist, S)
        self.tilt_ref = ref
        # two pitch gates: the navigator's (lidar_obstacles.py: the "project" limit whatever the map does, a balancing
        # robot leans that much to speed up) and the map's own (pitch_limit_deg in "skip" mode)
        lim_o = max(S["pitch_limit_deg"], S["pitch_max_deg"])
        lim_m = lim_o if S["tilt_mode"] == "project" else S["pitch_limit_deg"]
        yaw = S["yaw_deg"]
        sgn = 1.0 if S["flip"] else -1.0            # the LD19 counts clockwise; flipped over it counts CCW
        sx, sy = S["x_mm"], S["y_mm"]
        rmin, rmax, qmin = S["min_range_mm"], S["max_range_mm"], S["min_intensity"]
        rlo = rmin if rmin > 0 else 1e-9             # (0 = no return)
        mtab, mcum, masked = self._mask, self._mcum, self._masked
        clip, fk = S["floor_clip"], 0.9 * S["height_mm"]
        deskew = S["deskew"]
        rpc = self._rp
        radians, cos, sin, tan, ceil = math.radians, math.cos, math.sin, math.tan, math.ceil
        rect, repeat = cmath.rect, itertools.repeat
        turn = Turn()
        turn.scan = scan
        z, runs = turn.z, turn.runs
        append, extend = z.append, z.extend
        run0 = prev_a = None
        c_tot = c_zero = c_range = c_int = c_mask = c_tilt = c_floor = 0
        map_n = 0
        tilted = False
        if deskew:
            idx = _seek(hist, scan.packets[0].t + tco) if scan.packets else 0
        else:                                       # one pose for the whole turn: the one at its end
            _, px, py, ph, pp = interp(hist, _seek(hist, scan.t1 + tco), scan.t1 + tco)
        for pk in scan.packets:
            if deskew:
                idx, px, py, ph, pp = interp(hist, idx, pk.t + tco)
            c_tot += POINTS
            dp = pp - ref
            adp = dp if dp >= 0.0 else -dp
            if adp > lim_o or abs(pp) > PITCH_ABS_MAX:
                tilted = True
                c_tilt += POINTS
                continue
            map_ok = adp <= lim_m
            if not map_ok:                          # the navigator's points only: the map run ends here
                c_tilt += POINTS
                if run0 is not None:
                    if len(z) > run0:
                        runs.append((run0, len(z)))
                    run0 = None
            hr = ph * RAD
            ch, sh = cos(hr), sin(hr)
            o = complex(px + ch * sx - sh * sy, py + sh * sx + ch * sy)
            s, step = pk.start, pk.step
            db = sgn * step
            rp = rpc.get(db)
            if rp is None:
                if len(rpc) > 64:
                    rpc.clear()
                rp = rpc[db] = [rect(1.0, radians(db * k)) for k in range(POINTS)]
            b0 = yaw + sgn * s                      # the packet's first beam, robot frame, deg
            b1 = b0 + (POINTS - 1) * db
            dpr = dp * RAD
            # range * cos(elevation): horizontal (elevation = lean * cos(azimuth), here at the packet's middle beam)
            fh = cos(dpr * cos((0.5 * RAD) * (b0 + b1))) if dpr else 1.0
            us = list(map(_mul, repeat(rect(fh, hr + b0 * RAD), POINTS), rp))
            ds, qs = pk.dist, pk.qual
            lo, hi = (b0, b1) if b0 <= b1 else (b1, b0)
            mk = False
            if masked:                              # does the packet's span touch a masked sector?
                i0, i1 = int((lo % 360.0) * 2.0), int((hi % 360.0) * 2.0)
                mk = (mcum[i1 + 1] - mcum[i0] if i0 <= i1 else mcum[720] - mcum[i0] + mcum[i1 + 1]) > 0
            mxd = max(ds)
            lo_clip = _INF                          # no point of this packet closer than this can be a floor hit
            if clip and dpr:
                adpr = dpr if dpr > 0.0 else -dpr
                if adpr > 1e-4:
                    lo_clip = fk / tan(adpr)        # (the steepest beam of any packet)
                    if mxd > lo_clip:               # this packet's own steepest beam
                        cmax = 1.0 if ceil(lo / 180.0) * 180.0 <= hi else max(abs(cos(lo * RAD)), abs(cos(hi * RAD)))
                        lo_clip = fk / tan(adpr * cmax) if adpr * cmax > 1e-4 else _INF
            if not mk and mxd <= rmax and mxd <= lo_clip and min(ds) >= rlo and min(qs) >= qmin and step <= GAP_DEG:
                # every point passes (the common case): no per-point Python
                if map_ok:
                    if run0 is None or prev_a is None or (s - prev_a) % 360.0 > GAP_DEG:
                        if run0 is not None and len(z) > run0:
                            runs.append((run0, len(z)))
                        run0 = len(z)
                    prev_a = (s + (POINTS - 1) * step) % 360.0
                    map_n += POINTS
                extend(map(_add, repeat(o, POINTS), map(_mul, ds, us)))
                continue
            for k in range(POINTS):
                d = ds[k]
                if d <= 0:
                    if map_ok:
                        c_zero += 1
                    continue
                if d < rmin or d > rmax:
                    if map_ok:
                        c_range += 1
                    continue
                if qs[k] < qmin:
                    if map_ok:
                        c_int += 1
                    continue
                if mk and mtab[int(((b0 + k * db) % 360.0) * 2.0)]:
                    if map_ok:
                        c_mask += 1
                    continue
                if d > lo_clip:
                    e = dpr * cos((b0 + k * db) * RAD)    # beam elevation at this azimuth (+ = down or up, symmetric)
                    ae = abs(e)
                    if ae > 1e-4 and d * cos(e) > fk / tan(ae):
                        if map_ok:
                            c_floor += 1
                        continue
                if map_ok:
                    a = (s + k * step) % 360.0
                    if run0 is None or prev_a is None or (a - prev_a) % 360.0 > GAP_DEG:
                        if run0 is not None and len(z) > run0:
                            runs.append((run0, len(z)))
                        run0 = len(z)
                    prev_a = a
                    map_n += 1
                append(o + d * us[k])
        if run0 is not None and len(z) > run0:
            runs.append((run0, len(z)))
        if deskew:                                  # the fan of the free-space polygon starts at the mid-scan pose
            tm = 0.5 * (scan.t0 + scan.t1) + tco
            _, xm, ym, hm, _ = interp(hist, _seek(hist, tm), tm)
        else:
            xm, ym, hm = px, py, ph
        hr = radians(hm)
        turn.mid = (xm, ym, hm)
        turn.origin = complex(xm + cos(hr) * sx - sin(hr) * sy, ym + sin(hr) * sx + cos(hr) * sy)
        turn.newest = hist[-1][:4]
        turn.ref = ref
        turn.tilted = tilted
        turn.counts = (c_tot, c_zero, c_range, c_int, c_mask, c_tilt, c_floor)
        turn.map_n = map_n
        return turn

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
        turn = getattr(scan, "turn", False)
        if turn is False:                           # (a scan that did not come through on_scan)
            turn = scan.turn = self.deskew(scan)
        tco = S["time_offset_ms"] / 1000.0
        if turn is None or turn.newest[0] < scan.t0 + tco - POSE_STALE_S:
            n["no_pose"] += 1
            return 0
        self.tilt_ref = turn.ref
        c = turn.counts
        n["points"] += c[0]
        n["no_return"] += c[1]
        n["range"] += c[2]
        n["intensity"] += c[3]
        n["mask"] += c[4]
        n["tilt"] += c[5]
        n["floor"] += c[6]
        if turn.map_n < MIN_POINTS:
            n["few"] += 1
            return 0
        n["used"] += 1
        self.last_used = self.clock()
        ms = (time.perf_counter() - t0) * 1000.0
        loop = self._loop()
        if loop is not None:        # the field update, the match and the grid update each in its own loop callback (#102)
            loop.call_soon(self._prepare, turn, ms, self.mapper.version)
        else:
            self._match_turn(turn, ms)
        return turn.map_n

    def _map_frame(self, turn):
        """The turn in the map frame with the correction as it is NOW (#110: a correction the turn before made while
        this one waited counts): (ox, oy, mx, my, mh, runs [[(x, y)]], pts [(x, y)])."""
        cx, cy, ct = self.mapper.corr
        zs, o = turn.z, turn.origin
        if cx or cy or ct:
            rot, sh = cmath.rect(1.0, math.radians(ct)), complex(cx, cy)
            zs = list(map(_add, itertools.repeat(sh), map(_mul, zs, itertools.repeat(rot))))
            o = rot * o + sh
        step = self.settings["ray_step"]
        runs = [complex_to_xy(zs[i0:i1:step] if step > 1 else zs[i0:i1]) for i0, i1 in turn.runs]
        mx, my, mh = self.mapper.map_pose(*turn.mid)
        return o.real, o.imag, mx, my, mh, runs, list(itertools.chain.from_iterable(runs))

    def _match_turn(self, turn, ms, version=None):
        if version is not None and version != self.mapper.version:
            return
        if self._matching:                           # the turn before is still being matched (in steps): after it
            self._match_q.append((turn, ms, version))
            return
        t0 = time.perf_counter()
        with self.prof.part("lidar.scan"):
            args = self._map_frame(turn)
        dt = time.perf_counter() - t0
        if version is not None:
            self._cpu_acc += dt
        self._match_map(*args, ms + dt * 1000.0, version)

    def _prepare(self, turn, ms, version):
        """(deferred) the matcher's field first, then the match in the next loop round."""
        if version != self.mapper.version:
            return
        t0 = time.perf_counter()
        lm = getattr(self.mapper, "lmatch", None)
        if lm is not None and hasattr(lm, "prepare"):
            lm.prepare()
        dt = time.perf_counter() - t0
        self._cpu_acc += dt
        loop = self._loop()
        if loop is None:
            self._match_turn(turn, ms + dt * 1000.0, version)
        else:
            loop.call_soon(self._match_turn, turn, ms + dt * 1000.0, version)

    def _match_map(self, ox, oy, mx, my, mh, runs, pts_all, ms, version=None):
        """Second half of a turn: scan-to-map matching, then (maybe) into the grid. ms: the first half's cost.
        version: deferred (the map was reset / loaded meanwhile: dropped). Deferred, the match runs in steps of
        MATCH_SLICE_S, each its own loop callback (#110: a match is ~40 ms of Pi time, a lost one more)."""
        if version is not None and version != self.mapper.version:
            return
        lm = getattr(self.mapper, "lmatch", None)
        if lm is None:
            return self._match_done(ox, oy, mx, my, runs, pts_all, ms, 0.0, version)
        # scan-to-map matching (#78): may correct the pose; the turn then maps at the new one
        if version is None or self._loop() is None or not hasattr(lm, "on_scan_iter"):
            t0 = time.perf_counter()
            with self.prof.part("lidar.match"):
                ox, oy, mx, my, runs, pts_all = lm.on_scan(ox, oy, mx, my, mh, runs, pts_all)
            return self._match_done(ox, oy, mx, my, runs, pts_all, ms, time.perf_counter() - t0, version)
        self._matching = True
        self._match_step(lm.on_scan_iter(ox, oy, mx, my, mh, runs, pts_all), ms, 0.0, version)

    def _match_step(self, gen, ms, used, version):
        if version != self.mapper.version:           # the map was reset / loaded meanwhile
            gen.close()
            return self._match_next()
        t0 = time.perf_counter()
        end = t0 + MATCH_SLICE_S
        try:
            with self.prof.part("lidar.match"):
                while True:
                    next(gen)
                    if time.perf_counter() >= end:
                        break
        except StopIteration as e:
            used += time.perf_counter() - t0
            ox, oy, mx, my, runs, pts_all = e.value
            self._match_done(ox, oy, mx, my, runs, pts_all, ms, used, version)
            return self._match_next()
        except Exception as e:                       # a matcher bug must never stop the mapping
            self.reader.error = "matcher: %s: %s" % (type(e).__name__, e)
            return self._match_next()
        used += time.perf_counter() - t0
        loop = self._loop()
        if loop is None:
            return self._match_step(gen, ms, used, version)
        loop.call_soon(self._match_step, gen, ms, used, version)

    def _match_next(self):
        """The match of a turn is done: the next turn that waited for it (one at a time: the matcher's state)."""
        self._matching = False
        if self._match_q:
            args = self._match_q.popleft()
            loop = self._loop()
            if loop is None:
                self._match_turn(*args)
            else:
                loop.call_soon(self._match_turn, *args)

    def _match_done(self, ox, oy, mx, my, runs, pts_all, ms, dt, version):
        lm = getattr(self.mapper, "lmatch", None)
        if lm is not None and dt:
            lm.cost_ms = dt * 1000.0                 # (its own clock would count the waits between the steps)
            lm.cost_avg_ms = lm.cost_ms if not lm.cost_avg_ms else 0.9 * lm.cost_avg_ms + 0.1 * lm.cost_ms
        self._keep_points(pts_all)
        if version is not None:
            self._cpu_acc += dt
        self.cost_ms = ms + dt * 1000.0
        self.cost_avg_ms = self.cost_ms if not self.cost_avg_ms else 0.9 * self.cost_avg_ms + 0.1 * self.cost_ms
        if runs is not None:        # (None: the matcher keeps this turn out of the map, see lidarmatch._map_now)
            loop = self._loop()
            if loop is not None:
                loop.call_soon(self._integrate, ox, oy, runs, (mx, my), self.mapper.version)
            else:
                self._integrate(ox, oy, runs, (mx, my))

    def _keep_points(self, pts):
        """For the map view: the mapped points (back in the odometry frame, where the robot / track are drawn). Kept
        as they are with the correction of the moment; last_pts converts them when somebody looks (<= 3 Hz, #110)."""
        self._last_raw = (pts, tuple(self.mapper.corr))
        self.pts_seq += 1

    @property
    def last_pts(self):
        """[[x, y], ...] of the last mapped turn, odometry frame, whole mm."""
        seq, cached = self._last_cache
        if seq == self.pts_seq or self._last_raw is None:
            return cached
        pts, (dx, dy, dt) = self._last_raw
        if dx or dy or dt:
            c, s = math.cos(math.radians(-dt)), math.sin(math.radians(-dt))
            pts = [(c * (x - dx) - s * (y - dy), s * (x - dx) + c * (y - dy)) for x, y in pts]
        out = [[int(x), int(y)] for x, y in pts]
        self._last_cache = (self.pts_seq, out)
        return out

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
