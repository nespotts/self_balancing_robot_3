"""Mapping in the bridge (issue #12): RangeFrames -> occupancy grid -> browsers; save / load / reset.

Mapper.on_range(d) takes every decoded RangeFrame (bridge.range_listeners), Mapper.poll() returns
the throttled map updates (~2 Hz) to broadcast, Mapper.full_msg() the whole map for a new client,
Mapper.command(m) handles {"t": "map", "action": ...} from browsers.

Sensor model (see station/README.md "Map"):
- the beam starts at the sensor (servo axis 26 mm ahead of the axle, sensor 12.9 mm out from it);
- a valid reading r frees a wedge of +-cone_deg/2 out to r - (1 cell + 3 %) and marks an arc at r,
  +-hit_arc_deg/2 wide, occupied; readings beyond max_hit_mm only free space up to max_hit_mm;
- "no target" readings (valid = 0) are ignored unless no_target_mm > 0 (then a half-strength
  free wedge out to that distance: VL53L0X "no target" also happens on dark / shiny / angled surfaces);
- readings taken while |pitch - typical pitch| > pitch_limit_deg are dropped (the beam would hit
  the floor or go over obstacles); typical pitch = median of the last ~2 s, so the IMU mount offset does not matter.
- pose correction: "stop_scan" (match a scan taken standing still, #71) or the experimental "correct"
  (continuous): see scanmatch.py. map pose = corr o odometry pose; places, plans and the browser use it.

Event-loop cost (#102): a tile goes to the browsers only when its bytes changed since it was last sent (CRC of the raw
tile); encoding is limited to TILE_BUDGET_S per poll, the robot's nearest tiles first, the rest carried over (worked off
every TILE_AGAIN_S) so every change still arrives; the encoded tiles are cached for new clients. Autosave copies the
grid on the loop and writes JSON + PNG in a worker thread.
"""
import asyncio
import base64
import collections
import json
import math
import os
import re
import threading
import time
import zlib

from .occgrid import MAX_CELLS, Grid
from .places import HOME, Places
from .zones import Zones
from . import lidarmatch, loopprof, scanmatch

SENSOR_FWD_MM = 26.0
SENSOR_RADIUS_MM = 12.9
SEND_EVERY_S = 0.5
TILE_BUDGET_S = 0.015      # tile encoding per poll (#102; Pi Zero 2W: ~0.5 ms a tile); the rest waits for the next one
TILE_AGAIN_S = 0.1         # a carried-over backlog is worked off this often (new changes still every SEND_EVERY_S)
AUTOSAVE_S = 60.0
AUTOSAVE_NAME = "autosave"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
PITCH_ABS_MAX = 30.0       # beyond this the robot is lying down / being carried: never map

DEFAULTS = {
    "enabled": True,
    "pitch_limit_deg": 5.0,     # drop readings this far from the typical pitch
    "no_target_mm": 0.0,        # free space for "no target" readings (0 = ignore them)
    "max_hit_mm": 2000.0,       # valid readings beyond this only free space
    "min_mm": 40.0,             # closer readings are ignored (the robot itself, crosstalk)
    "cone_deg": 12.0,           # free-space wedge width (VL53L0X field of view is ~25 deg)
    "hit_arc_deg": 5.0,         # the target is marked on an arc this wide (where in the cone it is is unknown)
    "range_weight": True,       # near hits: narrow + strong; far hits: weak; near free space counts double
    "body_free_mm": 50.0,       # the floor within this of the robot's centre is free (0 = off; body half width 76)
    "correct": False,           # continuous scan-to-map pose correction (experimental, odometry drift)
    "stop_scan": False,         # stop-and-scan pose correction whenever the robot stands still (#71)
    "lidar_replaces_sweep": True,   # while LD19 scans are flowing (#76) the VL53L0X sweep does not draw into the map
    # LD19 scan-to-map matching (#78, lidarmatch.py): live pose correction from the dense turns
    "lidar_match": True,
    "match_every_n": 2,         # match every n-th mapped turn (Pi CPU); the others map with the current correction
    "match_points": 150,        # points per match after thinning to one per 50 mm
    "match_window_mm": 100.0,   # search window around the predicted pose (odometry + correction)
    "match_window_deg": 3.0,
    "match_min_score": 0.4,     # mean likelihood (0..1) of the points that land in mapped cells
    "match_min_inliers": 0.5,   # share of those within ~37 mm of a wall
    "match_min_curv": 0.05,     # score curvature below which a direction is degenerate (corridor: along it)
    "match_min_margin": 0.05,   # best pose must beat any pose clearly elsewhere by this share
    "match_gain": 0.5,          # share of a match's correction applied (smooths the match noise)
    "match_deadband_mm": 8.0,   # smaller changes are not applied (the matcher's own noise: they would walk the map off)
    "match_deadband_deg": 0.1,
    "match_step_mm": 40.0,      # at most this much per match (rate limit) ...
    "match_step_deg": 1.0,
    "match_jump_mm": 60.0,      # bigger corrections are jumps: applied only when 3 matches in a row agree
    "match_jump_deg": 2.0,
    "match_warmup": 10,         # turns mapped before matching starts (map only)
    "match_max_turn_dps": 90.0, # turning faster: the turn is neither matched nor mapped (timing errors turn the scan)
    "match_map_mm": 250.0,      # while matching, a turn goes into the map only after the robot moved this far ...
    "match_map_deg": 20.0,      # ... or turned this much ...
    "match_map_s": 5.0,         # ... or this long after the last one (the map is the reference: see lidarmatch.py)
    # wheel slip (#79): hold the position while the wheels spin, "lost" + a wider search instead of mapping a bad pose
    "match_slip": True,         # off: the #78 behaviour (a pose rejected for 10 s is mapped anyway)
    "match_wide_mm": 400.0,     # lost: the search widens up to this ...
    "match_wide_deg": 6.0,      # ... (the gyro heading does not slip: the heading window grows less)
    "match_stuck_s": 2.5,       # slipping in GOTO / EXPLORE, the lidar pose still this long: the robot is stuck (0 = off)
    # relocalisation (#105, reloc.py): lost > 5 s, a Teensy reboot / pose reset, a map loaded -> search the whole map
    "match_global": True,
}
LIMITS = {"pitch_limit_deg": (0.5, 45), "no_target_mm": (0, 4000), "max_hit_mm": (100, 8000),
          "min_mm": (0, 500), "cone_deg": (0, 30), "hit_arc_deg": (0, 30), "body_free_mm": (0, 150),
          "match_every_n": (1, 20), "match_points": (40, 450), "match_window_mm": (25, 300), "match_window_deg": (0.5, 10),
          "match_min_score": (0, 1), "match_min_inliers": (0, 1), "match_min_curv": (0, 1), "match_min_margin": (0, 1),
          "match_gain": (0.05, 1), "match_deadband_mm": (0, 100), "match_deadband_deg": (0, 3),"match_step_mm": (1, 300), "match_step_deg": (0.05, 10), "match_jump_mm": (5, 1000),
          "match_jump_deg": (0.2, 30), "match_warmup": (0, 1000), "match_map_mm": (0, 2000), "match_map_deg": (0, 180),
          "match_map_s": (0, 60), "match_max_turn_dps": (5, 1000), "match_wide_mm": (100, 400),
          "match_wide_deg": (1, 15), "match_stuck_s": (0, 30)}
INTS = ("match_every_n", "match_points", "match_warmup")


class Mapper:
    def __init__(self, map_dir="~/sbr3-maps", say=None, size_m=20.0, res_mm=25.0, max_cells=MAX_CELLS):
        self.dir = os.path.abspath(os.path.expanduser(map_dir)) if map_dir else None   # None: no robot profile yet (#97), nothing is written
        self.hold = False                # True: readings / scans are not mapped (a robot is being identified, #97)
        self.say = say or (lambda text: None)
        self.size_m, self.res_mm = size_m, res_mm
        self.max_cells = max_cells
        self.grid = Grid(size_m, res_mm, max_cells=max_cells)
        self.warned = None               # last grid warning that was announced
        self.settings = dict(DEFAULTS)
        self._load_settings()
        self.name = None                 # map last saved / loaded
        self.pitch_ref = None
        self.body_at = None              # where the body was last marked free (body_free_mm)
        self.pitches = collections.deque(maxlen=51)
        self.corr = [0.0, 0.0, 0.0]      # map pose = corr o odometry pose: [dx mm, dy mm, dtheta deg]
        self.matcher = scanmatch.Matcher(self)
        self.stopscan = scanmatch.StopScan(self)
        self.corr_listeners = []         # callables(): the LD19 matcher moved corr (planner: route in the odometry frame)
        self.lmatch = lidarmatch.LidarMatch(self)   # LD19 scan-to-map matching (#78), fed by lidar.py
        self.scan_sent = 0               # stopscan.seq last broadcast
        self.lm_sent, self.corr_sent, self.last_hdr = ("track", ""), None, 0.0   # last map header: matcher mode, corr, time
        self.lidar = None                # lidar.Lidar (#76), set by it
        self.n = {"readings": 0, "used": 0, "tilt": 0, "no_target": 0, "short": 0, "sweep_skipped": 0, "scans": 0}
        self.cpu_s = 0.0                 # time spent in on_range / poll since the last stats
        self.cpu_t0 = time.monotonic()
        self.cpu_pct = 0.0
        self.last_send = 0.0
        self.version = 0                 # bumps on reset / load (browsers drop their tiles)
        self.changed_since_save = False
        self.last_autosave = time.monotonic()
        self.full_pending = False
        self.extra_listeners = []        # callables(): the map changed (planner, #12 follow-up)
        self.places = Places()           # saved places (#53), map frame; stored as <map>.places.json
        self.places_pending = False      # broadcast the places on the next poll
        self.zones = Zones()             # virtual walls / no-go / fence / slow / mazes (#70), <map>.zones.json
        self.zones_pending = False
        self.zone_listeners = []         # callables(zones): the layers changed (the sim mirrors maze walls)
        self.prof = loopprof.NULL        # the bridge's loop profile (#102)
        self.tile_queue = set()          # absolute tiles changed but not sent yet (#102: the encoding budget carries them over)
        self.tile_sent = {}              # absolute tile -> (crc32 of its raw bytes, base64 png) as last broadcast
        self.tile_grid = None            # the Grid those belong to (load replaces it)
        self.tile_stats = {"sent": 0, "same": 0, "polls_cut": 0}
        self.last_tiles = 0.0
        self.saving = False              # an autosave is being written in a worker thread

    # ------------------------------------------------------------ settings
    def _settings_path(self):
        return os.path.join(self.dir, "settings.json") if self.dir else None

    def bind(self, map_dir, fresh=False):
        """Use this folder from now on (robot profile, #97): its settings.json is loaded over the defaults.
        fresh: a different robot - the map, places, zones and pose correction start empty (the caller saved the old ones)."""
        if fresh:
            self.reset()
            self.pitch_ref, self.body_at = None, None
            self.pitches.clear()
        self.dir = os.path.abspath(os.path.expanduser(map_dir)) if map_dir else None
        self.settings.clear()
        self.settings.update(DEFAULTS)
        self._load_settings()
        self.last_autosave = time.monotonic()
        self.full_pending = True
        self.last_send = 0.0

    def _load_settings(self):
        try:
            with open(self._settings_path(), encoding="utf-8") as f:
                self.apply_settings(json.load(f), save=False)
        except (OSError, ValueError, TypeError):
            pass

    def apply_settings(self, s, save=True):
        for k, v in (s if isinstance(s, dict) else {}).items():
            if k not in DEFAULTS:
                continue
            if isinstance(DEFAULTS[k], bool):
                self.settings[k] = bool(v)
            else:
                try:
                    v = float(v)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(v):
                    continue
                lo, hi = LIMITS.get(k, (-1e9, 1e9))
                v = max(lo, min(hi, v))
                self.settings[k] = int(round(v)) if k in INTS else v
        if save and self.dir:
            try:
                os.makedirs(self.dir, exist_ok=True)
                with open(self._settings_path(), "w", encoding="utf-8") as f:
                    json.dump(self.settings, f, indent=1)
            except OSError:
                pass

    # ------------------------------------------------------------ readings
    def map_pose(self, x, y, h_deg):
        """Odometry pose -> map frame (applies the pose correction)."""
        dx, dy, dt = self.corr
        if not (dx or dy or dt):
            return x, y, h_deg
        c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))
        return c * x - s * y + dx, s * x + c * y + dy, h_deg + dt

    def on_range(self, d):
        t0 = time.perf_counter()
        try:
            self._on_range(d)
        finally:
            self.cpu_s += time.perf_counter() - t0

    def _on_range(self, d):
        S = self.settings
        if not S["enabled"] or self.hold:
            return
        self.n["readings"] += 1
        if S["lidar_replaces_sweep"] and self.lidar is not None and self.lidar.flowing():
            self.n["sweep_skipped"] += 1           # the dense LD19 scan maps; the two sparse sensors would only fight it
            return
        p = d["pitch_deg"]
        if not math.isfinite(p) or abs(p) > PITCH_ABS_MAX:
            self.n["tilt"] += 1
            return
        self.pitches.append(p)                    # typical pitch = median of the last ~2 s
        if len(self.pitches) >= 5:
            self.pitch_ref = sorted(self.pitches)[len(self.pitches) // 2]
        else:
            self.pitch_ref = p
        if abs(p - self.pitch_ref) > S["pitch_limit_deg"]:
            self.n["tilt"] += 1
            return
        r = float(d["range_mm"])
        if d["valid"] and r < 8000:
            if r < S["min_mm"]:
                self.n["short"] += 1
                return
        elif S["no_target_mm"] <= 0:
            self.n["no_target"] += 1
            return
        if S["correct"]:
            self.matcher.add(d)           # buffers a sweep, matches it, then calls integrate()
        elif S["stop_scan"] or self.stopscan.busy():
            self.stopscan.add(d)          # held back while standing still, matched, then integrate()
        else:
            self.integrate(d)

    def integrate(self, d):
        """Put one (filtered) reading into the grid with the current pose correction."""
        S = self.settings
        g = self.grid
        x, y, hd = self.map_pose(d["x_mm"], d["y_mm"], d["heading_deg"])
        h = math.radians(hd)
        sx, sy = x + SENSOR_FWD_MM * math.cos(h), y + SENSOR_FWD_MM * math.sin(h)
        a = h + math.radians(d["sensor_angle_deg"])
        ca, sa = math.cos(a), math.sin(a)
        ox, oy = sx + SENSOR_RADIUS_MM * ca, sy + SENSOR_RADIUS_MM * sa
        half = math.radians(S["cone_deg"]) / 2
        r = float(d["range_mm"])
        if d["valid"] and r < 8000:
            if r <= S["max_hit_mm"]:
                # free space stops 1 cell + 3 % short of the target (range noise), so a noisy reading
                # does not erase the wall cell another reading just marked
                g.integrate_wedge(ox, oy, a, half, r - g.res - 0.03 * r, r,
                                  hit_half=math.radians(S["hit_arc_deg"]) / 2, rw=S["range_weight"])
            else:
                g.integrate_wedge(ox, oy, a, half, S["max_hit_mm"])
        else:
            g.integrate_wedge(ox, oy, a, half, S["no_target_mm"], weak=True)
            self.n["no_target"] += 1
        self._body_free(x, y)
        self.n["used"] += 1
        self.changed_since_save = True

    def _body_free(self, x, y):
        rb = self.settings["body_free_mm"]
        # the body's own spot, once per cell moved (standing still adds nothing): a far cone hit drawn into
        # a doorway goes once the robot has driven through it, whichever way it looked meanwhile
        if rb > 0 and (self.body_at is None or math.hypot(x - self.body_at[0], y - self.body_at[1]) >= self.grid.res):
            self.body_at = (x, y)
            self.grid.clear_disc(x, y, rb)

    def integrate_scan(self, ox, oy, runs, robot_xy, seen=None, last=True):
        """One LD19 revolution (#76), map frame: ox, oy the lidar axis, runs the point runs (lidar.py),
        robot_xy the axle centre (its own spot is free). A turn in parts (#102): seen carried from part to part
        (Grid.integrate_scan), last on the final one."""
        if not self.settings["enabled"] or self.hold:
            return
        self.grid.integrate_scan(ox, oy, runs, margin_mm=self.grid.res, seen=seen)
        self.changed_since_save = True
        if last:
            self._body_free(robot_xy[0], robot_xy[1])
            self.n["scans"] += 1
            self.n["used"] += 1

    # ------------------------------------------------------------ browser messages
    def _header(self, full):
        m = {"t": "map", "full": full, "v": self.version, "corr": [round(c, 2) for c in self.corr],
             "name": self.name, "stats": self.stats(), "settings": self.settings}
        if full:
            m.update(self.grid.info())
        return m

    def full_msg(self, store=False):
        """The whole map (a new client, or everyone after a reset / load). Tiles unchanged since they were last broadcast
        come from the cache; store: this goes to every client, so it also becomes what was last sent."""
        self._tile_sync()
        g = self.grid
        ox, oy = g.tile_origin()
        tiles = []
        for tx, ty in sorted(g.touched | g.dirty):
            t = (tx + ox, ty + oy)
            raw = g.tile_raw(tx, ty)
            crc = zlib.crc32(raw)
            c = self.tile_sent.get(t)
            if c is None or c[0] != crc:
                c = (crc, base64.b64encode(g.png_of_raw(raw)).decode("ascii"))
                if store:
                    self.tile_sent[t] = c
            tiles.append([t[0], t[1], c[1]])
        if store:
            self.tile_queue.clear()
        m = self._header(True)
        m["tiles"] = tiles
        return json.dumps(m, separators=(",", ":"))

    def _tile_sync(self):
        if self.grid is not self.tile_grid:          # a loaded map: nothing of the old one was sent
            self.tile_grid = self.grid
            self.tile_queue.clear()
            self.tile_sent.clear()

    def _queue_dirty(self):
        """The grid's dirty tiles -> the send queue (absolute indices: stable while the grid grows). True if any."""
        self._tile_sync()
        dirty = self.grid.take_dirty()
        if dirty:
            ox, oy = self.grid.tile_origin()
            self.tile_queue.update((tx + ox, ty + oy) for tx, ty in dirty)
        return bool(dirty)

    def _encode_tiles(self, now):
        """[[tx, ty, base64 png], ...] of queued tiles whose bytes changed, nearest the robot first, within
        TILE_BUDGET_S (at least one tile per call, so a backlog always shrinks)."""
        g = self.grid
        ox, oy = g.tile_origin()
        size = g.res * g.T
        rx, ry = self.body_at or (0.0, 0.0)
        rx, ry = math.floor(rx / size), math.floor(ry / size)
        order = sorted(self.tile_queue, key=lambda t: (t[0] - rx) ** 2 + (t[1] - ry) ** 2)
        clock = time.perf_counter
        end = clock() + TILE_BUDGET_S
        out = []
        q, sent, st = self.tile_queue, self.tile_sent, self.tile_stats
        for t in order:
            if out and clock() > end:
                st["polls_cut"] += 1
                break
            q.discard(t)
            tx, ty = t[0] - ox, t[1] - oy
            if not (0 <= tx < g.ntx and 0 <= ty < g.nty):
                continue
            raw = g.tile_raw(tx, ty)
            crc = zlib.crc32(raw)
            c = sent.get(t)
            if c is not None and c[0] == crc:
                st["same"] += 1                  # marked dirty, but the bytes are what the browsers already have
                continue
            b64 = base64.b64encode(g.png_of_raw(raw)).decode("ascii")
            sent[t] = (crc, b64)
            out.append([t[0], t[1], b64])
            st["sent"] += 1
        self.last_tiles = now
        return out

    def poll(self, now=None):
        """Messages to broadcast now (throttled to SEND_EVERY_S, a tile backlog every TILE_AGAIN_S); [] most of the time."""
        now = time.monotonic() if now is None else now
        if now - self.last_send < SEND_EVERY_S:
            if not self.tile_queue or now - self.last_tiles < TILE_AGAIN_S or self.full_pending:
                return []
            t0 = time.perf_counter()                     # the carried-over tiles only
            with self.prof.part("map.tiles"):
                tiles = self._encode_tiles(now)
            out = []
            if tiles:
                m = self._header(False)
                m["tiles"] = tiles
                out.append(json.dumps(m, separators=(",", ":")))
            self.cpu_s += time.perf_counter() - t0
            return out
        self.last_send = now
        t0 = time.perf_counter()
        out = []
        if self.grid.warning != self.warned:
            self.warned = self.grid.warning
            if self.warned:
                self.say(self.warned)
        if self.full_pending:
            self.full_pending = False
            self.grid.take_dirty()
            with self.prof.part("map.tiles"):
                out.append(self.full_msg(store=True))
            self.places_pending = True
            self.zones_pending = True
        else:
            dirty = self._queue_dirty()
            lm = self.lmatch
            key = (lm.mode, lm.state if lm.mode != "track" else "")
            # slipping / lost (#79) nothing is mapped, so no tiles would carry the new state and correction: header only
            hdr = key != self.lm_sent or (lm.mode != "track" and self.corr != self.corr_sent)
            tiles = []
            if self.tile_queue:
                with self.prof.part("map.tiles"):
                    tiles = self._encode_tiles(now)
            if tiles or (hdr and now - self.last_hdr >= 1.0):
                m = self._header(False)
                m["tiles"] = tiles
                out.append(json.dumps(m, separators=(",", ":")))
                self.lm_sent, self.corr_sent, self.last_hdr = key, list(self.corr), now
                if dirty:
                    for f in self.extra_listeners:
                        f()
        if self.places_pending:
            self.places_pending = False
            out.append(self.places_msg())
        if self.stopscan.seq != self.scan_sent:
            self.scan_sent = self.stopscan.seq
            out.append(self.scan_msg())
        if self.zones_pending:
            self.zones_pending = False
            out.append(self.zones_msg())
        if now - self.last_autosave >= AUTOSAVE_S:
            self.last_autosave = now
            if self.changed_since_save and self.n["used"] and self.dir and not self.saving:
                with self.prof.part("map.autosave"):
                    self.autosave()
        self.cpu_s += time.perf_counter() - t0
        dt = now - self.cpu_t0
        if dt >= 5.0:
            self.cpu_pct = 100.0 * self.cpu_s / dt
            self.cpu_s, self.cpu_t0 = 0.0, now
        return out

    def stats(self):
        g = self.grid
        return dict(self.n, cpu_pct=round(self.cpu_pct, 2), mem_mb=round(g.mem_mb(), 1),
                    size_m=[round(g.W * g.res / 1000.0, 1), round(g.H * g.res / 1000.0, 1)], warning=g.warning,
                    pitch_ref=None if self.pitch_ref is None else round(self.pitch_ref, 2),
                    match=self.matcher.last, lmatch=self.lmatch.stats())

    # ------------------------------------------------------------ commands
    def command(self, m, who="?", odom=None):
        """{"t": "map", "action": "reset" | "save" | "load" | "list" | "delete" | "set" | "trust_pose" | "scan" |
        "set_pose" | "relocalize", ...} -> reply. odom: the robot's odometry pose (x, y, heading) now (set_pose)."""
        act = m.get("action")
        if act == "reset":
            self.reset()
            self.say("%s: map reset" % who)
            return None
        if act == "save":
            name = str(m.get("name") or time.strftime("map-%Y%m%d-%H%M%S"))
            if not self.dir:
                return {"t": "error", "text": "map not saved: no robot identified yet (waiting for the robot)"}
            if not NAME_RE.match(name):
                return {"t": "error", "text": "map name: letters, digits, - and _ only"}
            self.save(name)
            self.say("%s: map saved as %s" % (who, name))
            return {"t": "map_saved", "name": name, "maps": self.list()}
        if act == "load":
            try:
                self.load(str(m.get("name")))
            except (OSError, ValueError) as e:
                return {"t": "error", "text": "map load failed: %s" % e}
            self.say("%s: map %s loaded" % (who, self.name))
            return None
        if act == "delete":
            p = self.path(str(m.get("name")))
            if p and os.path.exists(p):
                os.remove(p)
                png = p[:-5] + ".png"
                if os.path.exists(png):
                    os.remove(png)
                for pp in (self.places_path(str(m.get("name"))), self.zones_path(str(m.get("name")))):
                    if pp and os.path.exists(pp):
                        os.remove(pp)
            return {"t": "map_list", "maps": self.list()}
        if act == "list":
            return {"t": "map_list", "maps": self.list()}
        if act == "trust_pose":             # LD19 matcher lost (#79): take the pose as it is, map from here
            if not self.lmatch.trust_pose(who):
                return {"t": "error", "text": "the scan matcher is not lost"}
            self.full_pending = True
            return None
        if act == "set_pose":               # Set position (#105): the robot is at x, y (mm), heading th (deg) on the map
            try:
                x, y, th = float(m.get("x")), float(m.get("y")), float(m.get("th", m.get("heading_deg", 0.0)))
            except (TypeError, ValueError):
                return {"t": "error", "text": "set_pose: x, y (mm) and th (deg) needed"}
            if m.get("frame") == "odom":    # given in the odometry frame (what goto / places use): to the map frame
                x, y, th = self.map_pose(x, y, th)
            err = self.lmatch.set_pose(x, y, th, who, odom)
            if err:
                return {"t": "error", "text": err}
            self.full_pending = True        # browsers get the new correction now
            return None
        if act == "relocalize":             # "Find myself" (#105): search the whole map for the robot
            err = self.lmatch.relocalize(who)
            return {"t": "error", "text": err} if err else None
        if act == "scan":                   # "Scan & correct" (#71): match the next scan taken standing still
            if not self.n["used"]:
                return {"t": "error", "text": "no map yet to match against"}
            self.stopscan.ask(who)
            self.say("%s: scan & correct - hold still ~3 s" % who)
            return None
        if act == "set":
            self.apply_settings(m.get("settings") or {})
            if not self.settings["correct"]:
                self.matcher.flush()
            if not self.settings["stop_scan"]:
                self.stopscan.flush()
            self.full_pending = True        # browsers get the new settings
            return None
        return {"t": "error", "text": "unknown map action %r" % act}

    def reset(self):
        self.grid.reset()
        self.tile_queue.clear()
        self.tile_sent.clear()
        self.corr = [0.0, 0.0, 0.0]
        self.matcher.reset()
        self.stopscan.reset()
        self.lmatch.reset()
        self.scan_sent = -1
        self.name = None
        self.places = Places()          # places belong to a map; the saved map keeps its own
        self.zones = Zones()            # ... and so do the zones (#70)
        self._zones_notify()
        self.version += 1
        self.full_pending = True
        self.last_send = 0.0
        self.changed_since_save = False
        for k in self.n:
            self.n[k] = 0

    def scan_msg(self):
        """Stop-and-scan corrections (#71): the last one and a short history, newest first."""
        ss = self.stopscan
        return json.dumps({"t": "scan", "corr": [round(c, 2) for c in self.corr], "applied": ss.applied,
                           "last": ss.last, "history": list(ss.history)}, separators=(",", ":"))

    # ------------------------------------------------------------ places (#53)
    def default_home(self):
        """Home when none is saved: where the pose was reset (the odometry origin), in the map frame."""
        return tuple(self.places.origin) if self.places.origin else self.map_pose(0.0, 0.0, 0.0)[:2]

    def home_xy(self):
        return self.places.home(self.default_home())[:2]

    def place_xy(self, name):
        """Map-frame (x, y) of a saved place ('Home' always exists), or None."""
        if str(name or "").strip().lower() == HOME.lower():
            return self.home_xy()
        p = self.places.find(name)
        return (p["x"], p["y"]) if p else None

    def on_pose_reset(self):
        """The robot's odometry was reset here: the new origin is the default Home. With LD19 matching the correction
        first keeps the robot where the lidar last saw it (#79), so the map pose does not jump to the old origin."""
        self.lmatch.on_pose_reset()
        self.places.origin = [round(v, 1) for v in self.map_pose(0.0, 0.0, 0.0)[:2]]
        self.places_changed()

    def places_path(self, name):
        return os.path.join(self.dir, name + ".places.json") if self.dir and NAME_RE.match(name or "") else None

    def save_places(self, name=None):
        name = name or self.name or AUTOSAVE_NAME
        p = self.places_path(name)
        if not p:
            return
        if not self.places.items and not self.places.origin and not os.path.exists(p):
            return
        try:
            os.makedirs(self.dir, exist_ok=True)
            Places.write(p, self.places)
        except OSError:
            pass

    def places_changed(self):
        self.save_places()
        self.places_pending = True

    def places_msg(self):
        hx, hy = self.default_home()
        return json.dumps({"t": "places", "map": self.name, "items": self.places.listing((hx, hy))},
                          separators=(",", ":"))

    def places_command(self, m, who="?"):
        """{"t": "places", "action": "add", "name": "Couch", "x": mm, "y": mm}   (odometry frame, like goto;
        name "Home" moves Home) | "rename" (name, new) | "delete" (name) | "list" -> error reply or None;
        the new list reaches every browser with the next poll."""
        act = m.get("action")
        try:
            if act == "add":
                x, y = float(m.get("x")), float(m.get("y"))
                mx, my = self.map_pose(x, y, 0.0)[:2]
                name = self.places.add(m.get("name"), mx, my)
                self.say("%s: saved place %s" % (who, name))
            elif act == "rename":
                name = self.places.rename(m.get("name"), m.get("new"))
                self.say("%s: place renamed to %s" % (who, name))
            elif act == "delete":
                name = self.places.delete(m.get("name"))
                self.say("%s: place %s deleted" % (who, name))
            elif act == "list":
                self.places_pending = True
                return None
            else:
                return {"t": "error", "text": "unknown places action %r" % act}
        except (TypeError, ValueError) as e:
            return {"t": "error", "text": str(e)}
        self.places_changed()
        return None

    # ------------------------------------------------------------ zones (#70)
    def zones_path(self, name):
        return os.path.join(self.dir, name + ".zones.json") if self.dir and NAME_RE.match(name or "") else None

    def save_zones(self, name=None):
        name = name or self.name or AUTOSAVE_NAME
        p = self.zones_path(name)
        if not p or (not self.zones.items and not os.path.exists(p)):
            return
        try:
            os.makedirs(self.dir, exist_ok=True)
            Zones.write(p, self.zones)
        except OSError:
            pass

    def _zones_notify(self):
        self.zones_pending = True
        for f in self.zone_listeners:
            f(self.zones)

    def zones_changed(self):
        self.save_zones()
        self._zones_notify()
        for f in self.extra_listeners:       # the planner checks its route again
            f()

    def zones_msg(self):
        return json.dumps({"t": "zones", "map": self.name, "items": self.zones.items,
                           "undo": len(self.zones.history)}, separators=(",", ":"))

    def zones_command(self, m, who="?", robot_xy=None):
        """{"t": "zones", "action": ...}, points in the odometry frame (like places / goto), stored in the map frame:
          add     kind wall|nogo|fence|slow, pts [[x, y], ...], speed (slow, mm/s)   (a new fence replaces the old)
          update  id, pts and / or speed            delete  id          clear  [kind]        undo
          maze    x, y (centre), w, h (mm), cell (mm), seed (optional): a random maze, entrance facing the robot
          list    (everyone gets the layers again)
        robot_xy: the robot's odometry position (the maze entrance faces it).
        -> error reply or None; the layers reach every browser with the next poll."""
        act = m.get("action")
        Z = self.zones

        def to_map(pts):
            if not isinstance(pts, list):
                raise ValueError("pts: a list of [x, y]")
            return [list(self.map_pose(float(p[0]), float(p[1]), 0.0)[:2]) if isinstance(p, (list, tuple)) and len(p) >= 2
                    else None for p in pts]
        try:
            if act == "add":
                zid = Z.add(m.get("kind"), to_map(m.get("pts")), m.get("speed"))
                self.say("%s: added %s %s" % (who, m.get("kind"), zid))
            elif act == "update":
                Z.update(str(m.get("id")), to_map(m["pts"]) if m.get("pts") is not None else None, m.get("speed"))
            elif act == "delete":
                self.say("%s: deleted zone %s" % (who, Z.delete(str(m.get("id")))))
            elif act == "clear":
                Z.clear(m.get("kind") or None)
                self.say("%s: cleared %s" % (who, m.get("kind") or "all zones"))
            elif act == "undo":
                Z.undo()
            elif act == "maze":
                cx, cy = self.map_pose(float(m.get("x")), float(m.get("y")), 0.0)[:2]
                toward = self.map_pose(robot_xy[0], robot_xy[1], 0.0)[:2] if robot_xy else None
                kw = {k: m[k] for k in ("w", "h", "cell", "seed") if m.get(k) is not None}
                zid = Z.add_maze(cx, cy, toward=toward, **kw)
                it = Z.find(zid)
                self.say("%s: maze %s, %d x %d cells of %d mm" % (who, zid, it["cols"], it["rows"], it["cell"]))
            elif act == "list":
                self.zones_pending = True
                return None
            else:
                return {"t": "error", "text": "unknown zones action %r" % act}
        except (TypeError, ValueError, KeyError) as e:
            return {"t": "error", "text": "zones: %s" % e}
        self.zones_changed()
        return None

    # ------------------------------------------------------------ files
    def path(self, name):
        return os.path.join(self.dir, name + ".json") if self.dir and NAME_RE.match(name or "") else None

    def _save_meta(self):
        return {"corr": list(self.corr), "saved": time.strftime("%Y-%m-%dT%H:%M:%S"), "readings": self.n["used"]}

    @staticmethod
    def _write_map(p, grid, meta):
        """<name>.json + the <name>.png preview (not read back), each through a temp file (an autosave thread may be
        writing the same map at the same time)."""
        tmp = "%s.%d.tmp" % (p, threading.get_ident())
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(grid.to_json(meta))
        os.replace(tmp, p)
        png = p[:-5] + ".png"
        with open(png + ".tmp%d" % threading.get_ident(), "wb") as f:
            f.write(grid.png())
        os.replace(png + ".tmp%d" % threading.get_ident(), png)

    def save(self, name, quiet=False):
        if not self.dir:
            raise OSError("no robot profile yet")
        os.makedirs(self.dir, exist_ok=True)
        self._write_map(self.path(name), self.grid, self._save_meta())
        self.save_places(name)
        self.save_zones(name)
        if not quiet:
            self.name = name
            self.changed_since_save = False

    def autosave(self):
        """The periodic autosave (#102): the grid is copied here (on the loop, a few ms on the Pi), the JSON + PNG are
        built and written in a worker thread (zlib lets go of the GIL). Without a running event loop: right here."""
        if not self.dir or self.saving:
            return False
        d, p, snap, meta = self.dir, self.path(AUTOSAVE_NAME), self.grid.snapshot(), self._save_meta()
        # places / zones as text now (they are small); the files are written with the map
        side = []
        for path, obj, keep in ((self.places_path(AUTOSAVE_NAME), self.places, bool(self.places.items or self.places.origin)),
                                (self.zones_path(AUTOSAVE_NAME), self.zones, bool(self.zones.items))):
            if path:
                side.append((path, obj.to_json(), keep))

        def write():
            os.makedirs(d, exist_ok=True)
            self._write_map(p, snap, meta)
            for path, text, keep in side:
                if keep or os.path.exists(path):
                    try:
                        tmp = "%s.%d.tmp" % (path, threading.get_ident())
                        with open(tmp, "w", encoding="utf-8") as f:
                            f.write(text)
                        os.replace(tmp, path)
                    except OSError:
                        pass
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop is None:
            write()
            return True
        prof = self.prof

        def work():
            with prof.thread("map.autosave"):
                write()

        def done(fut):
            self.saving = False
            e = fut.exception()
            if e is not None:
                self.say("map autosave failed: %s: %s" % (type(e).__name__, e))
        self.saving = True
        loop.run_in_executor(None, work).add_done_callback(done)
        return True

    def load(self, name):
        p = self.path(name)
        if not p:
            raise ValueError("bad map name" if self.dir else "no robot identified yet")
        with open(p, encoding="utf-8") as f:
            g = Grid.from_json(f.read(), self.max_cells)
        self.grid = g
        self.corr = [float(c) for c in g.meta.get("corr", [0, 0, 0])][:3]
        self.places = Places.read(self.places_path(name))
        self.zones = Zones.read(self.zones_path(name))
        self._zones_notify()
        self.matcher.reset()
        self.stopscan.reset()
        self.lmatch.reset()
        self.lmatch.turns = 10 ** 6          # a loaded map is there to match against: no map-only warm-up
        if self.lidar is not None and self.settings.get("match_global", True):
            # where the robot is on it is not known (the saved correction is from another session's odometry): find it
            # (#105); nothing is mapped until it is found and confirmed
            self.lmatch.request_global("map %s loaded" % name, by="load")
        self.scan_sent = -1
        self.name = name
        self.version += 1
        self.full_pending = True
        self.last_send = 0.0
        self.changed_since_save = False

    def list(self):
        out = []
        if not self.dir:
            return out
        try:
            names = os.listdir(self.dir)
        except OSError:
            return out
        for n in names:
            if n.endswith(".json") and n not in ("settings.json", "planner.json", "lidar.json") and NAME_RE.match(n[:-5]):
                st = os.stat(os.path.join(self.dir, n))
                out.append({"name": n[:-5], "size": st.st_size, "mtime": int(st.st_mtime)})
        out.sort(key=lambda d: -d["mtime"])
        return out
