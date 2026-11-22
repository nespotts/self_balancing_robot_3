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
"""
import collections
import json
import math
import os
import re
import time

from .occgrid import MAX_CELLS, Grid
from .places import HOME, Places
from .zones import Zones
from . import scanmatch

SENSOR_FWD_MM = 26.0
SENSOR_RADIUS_MM = 12.9
SEND_EVERY_S = 0.5
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
}
LIMITS = {"pitch_limit_deg": (0.5, 45), "no_target_mm": (0, 4000), "max_hit_mm": (100, 8000),
          "min_mm": (0, 500), "cone_deg": (0, 30), "hit_arc_deg": (0, 30), "body_free_mm": (0, 150)}


class Mapper:
    def __init__(self, map_dir="~/sbr3-maps", say=None, size_m=20.0, res_mm=25.0, max_cells=MAX_CELLS):
        self.dir = os.path.abspath(os.path.expanduser(map_dir))
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
        self.scan_sent = 0               # stopscan.seq last broadcast
        self.n = {"readings": 0, "used": 0, "tilt": 0, "no_target": 0, "short": 0}
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

    # ------------------------------------------------------------ settings
    def _settings_path(self):
        return os.path.join(self.dir, "settings.json")

    def _load_settings(self):
        try:
            with open(self._settings_path(), encoding="utf-8") as f:
                self.apply_settings(json.load(f), save=False)
        except (OSError, ValueError):
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
                self.settings[k] = max(lo, min(hi, v))
        if save:
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
        if not S["enabled"]:
            return
        self.n["readings"] += 1
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
        rb = S["body_free_mm"]
        # the body's own spot, once per cell moved (standing still adds nothing): a far cone hit drawn into
        # a doorway goes once the robot has driven through it, whichever way it looked meanwhile
        if rb > 0 and (self.body_at is None or math.hypot(x - self.body_at[0], y - self.body_at[1]) >= self.grid.res):
            self.body_at = (x, y)
            self.grid.clear_disc(x, y, rb)
        self.n["used"] += 1
        self.changed_since_save = True

    # ------------------------------------------------------------ browser messages
    def _header(self, full):
        m = {"t": "map", "full": full, "v": self.version, "corr": [round(c, 2) for c in self.corr],
             "name": self.name, "stats": self.stats(), "settings": self.settings}
        if full:
            m.update(self.grid.info())
        return m

    def full_msg(self):
        tiles = sorted(self.grid.touched | self.grid.dirty)
        m = self._header(True)
        m["tiles"] = self.grid.tiles_msg(tiles)
        return json.dumps(m, separators=(",", ":"))

    def poll(self, now=None):
        """Messages to broadcast now (throttled to SEND_EVERY_S); [] most of the time."""
        now = time.monotonic() if now is None else now
        if now - self.last_send < SEND_EVERY_S:
            return []
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
            out.append(self.full_msg())
            self.places_pending = True
            self.zones_pending = True
        else:
            dirty = self.grid.take_dirty()
            if dirty:
                m = self._header(False)
                m["tiles"] = self.grid.tiles_msg(dirty)
                out.append(json.dumps(m, separators=(",", ":")))
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
            if self.changed_since_save and self.n["used"]:
                self.save(AUTOSAVE_NAME, quiet=True)
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
                    match=self.matcher.last)

    # ------------------------------------------------------------ commands
    def command(self, m, who="?"):
        """{"t": "map", "action": "reset" | "save" | "load" | "list" | "delete" | "set", ...} -> reply."""
        act = m.get("action")
        if act == "reset":
            self.reset()
            self.say("%s: map reset" % who)
            return None
        if act == "save":
            name = str(m.get("name") or time.strftime("map-%Y%m%d-%H%M%S"))
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
        self.corr = [0.0, 0.0, 0.0]
        self.matcher.reset()
        self.stopscan.reset()
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
        """The robot's odometry was reset here: the new origin is the default Home."""
        self.places.origin = [round(v, 1) for v in self.map_pose(0.0, 0.0, 0.0)[:2]]
        self.places_changed()

    def places_path(self, name):
        return os.path.join(self.dir, name + ".places.json") if NAME_RE.match(name or "") else None

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
        return os.path.join(self.dir, name + ".zones.json") if NAME_RE.match(name or "") else None

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
        return os.path.join(self.dir, name + ".json") if NAME_RE.match(name or "") else None

    def save(self, name, quiet=False):
        os.makedirs(self.dir, exist_ok=True)
        p = self.path(name)
        tmp = p + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(self.grid.to_json({"corr": self.corr, "saved": time.strftime("%Y-%m-%dT%H:%M:%S"),
                                       "readings": self.n["used"]}))
        os.replace(tmp, p)
        with open(p[:-5] + ".png", "wb") as f:        # preview to look at (not read back)
            f.write(self.grid.png())
        self.save_places(name)
        self.save_zones(name)
        if not quiet:
            self.name = name
            self.changed_since_save = False

    def load(self, name):
        p = self.path(name)
        if not p:
            raise ValueError("bad map name")
        with open(p, encoding="utf-8") as f:
            g = Grid.from_json(f.read(), self.max_cells)
        self.grid = g
        self.corr = [float(c) for c in g.meta.get("corr", [0, 0, 0])][:3]
        self.places = Places.read(self.places_path(name))
        self.zones = Zones.read(self.zones_path(name))
        self._zones_notify()
        self.matcher.reset()
        self.stopscan.reset()
        self.scan_sent = -1
        self.name = name
        self.version += 1
        self.full_pending = True
        self.last_send = 0.0
        self.changed_since_save = False

    def list(self):
        out = []
        try:
            names = os.listdir(self.dir)
        except OSError:
            return out
        for n in names:
            if n.endswith(".json") and n not in ("settings.json", "planner.json") and NAME_RE.match(n[:-5]):
                st = os.stat(os.path.join(self.dir, n))
                out.append({"name": n[:-5], "size": st.st_size, "mtime": int(st.st_mtime)})
        out.sort(key=lambda d: -d["mtime"])
        return out
