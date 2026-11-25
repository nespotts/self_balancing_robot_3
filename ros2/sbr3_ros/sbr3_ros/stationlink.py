"""Station link (issue #106): what the adapter tells the bridge about ROS, and the station's ROS commands. Pure Python
(no rclpy), unit tested on Windows. station_nav.py is the ROS side.

Adapter -> bridge (on the adapter's ROS WebSocket; the bridge relays them to the station UI):
  ros_map      the occupancy grid, on change, <= 0.5 Hz: w, h, res (m), origin [x, y, yaw] (m, rad), "rle" (see encode_grid)
  ros_pose     5 Hz: p = [x, y, yaw] robot in the ROS map (m, rad), T = map <- station_map [x, y, yaw] (or null)
  ros_plan     the Nav2 global plan, simplified, on change, <= 1 Hz: pts [[x, y], ...] (m, map frame)
  ros_nav      Nav2 status, on change + 1 Hz while busy (NavTracker.msg)
  ros_status   1 Hz: localization (loc_monitor state + fit), node health, drivable
  ros_explore  the explorer's status (#107), forwarded from /explore/status, "target" added in the station frame
  ros_reply    the answer to one ros_cmd: id, ok, text
Bridge -> adapter: {"t": "ros_cmd", "cmd": goal | place | patrol | cancel | initial_pose | explore | save_map, ...}.
"""
import base64
import hashlib
import math

from . import convert as cv

LINK_VERSION = 1

# ------------------------------------------------------------------ occupancy grid
UNKNOWN, FREE, OCC, MAYBE = 0, 1, 2, 3      # classes sent to the station (it colours them)


def grid_class(v, free_max=25, occ_min=65):
    """OccupancyGrid value (-1, 0..100) -> UNKNOWN / FREE / OCC / MAYBE."""
    if v < 0:
        return UNKNOWN
    if v >= occ_min:
        return OCC
    if v <= free_max:
        return FREE
    return MAYBE


def _classes(data, free_max=25, occ_min=65):
    try:
        import numpy as np
        a = np.asarray(data, dtype=np.int16)
        out = np.full(a.shape, MAYBE, dtype=np.uint8)
        out[a <= free_max] = FREE
        out[a >= occ_min] = OCC
        out[a < 0] = UNKNOWN
        return out
    except ImportError:                      # Windows test runs without numpy
        return bytes(grid_class(v, free_max, occ_min) for v in data)


def _runs(cls):
    """-> [(value, count)] of consecutive equal values."""
    try:
        import numpy as np
        if isinstance(cls, np.ndarray):
            n = len(cls)
            if not n:
                return []
            edges = np.flatnonzero(np.diff(cls)) + 1
            starts = np.concatenate(([0], edges))
            ends = np.concatenate((edges, [n]))
            return list(zip(cls[starts].tolist(), (ends - starts).tolist()))
    except ImportError:
        pass
    out = []
    prev, cnt = None, 0
    for v in cls:
        if v == prev:
            cnt += 1
        else:
            if prev is not None:
                out.append((prev, cnt))
            prev, cnt = v, 1
    if prev is not None:
        out.append((prev, cnt))
    return out


def encode_grid(data, free_max=25, occ_min=65):
    """OccupancyGrid data (row-major, row 0 = the map's bottom row at its origin) -> base64 run-length string:
    runs of [class byte, count as LEB128 varint]."""
    b = bytearray()
    for v, n in _runs(_classes(data, free_max, occ_min)):
        b.append(int(v))
        while True:
            byte = n & 0x7F
            n >>= 7
            if n:
                b.append(byte | 0x80)
            else:
                b.append(byte)
                break
    return base64.b64encode(bytes(b)).decode("ascii")


def decode_grid(s):
    """Inverse of encode_grid -> list of classes (the station's JS does the same)."""
    b = base64.b64decode(s)
    out = []
    i = 0
    while i < len(b):
        v = b[i]
        i += 1
        n = shift = 0
        while True:
            byte = b[i]
            i += 1
            n |= (byte & 0x7F) << shift
            shift += 7
            if not byte & 0x80:
                break
        out.extend([v] * n)
    return out


def grid_digest(w, h, res, origin, data):
    """Cheap change test for a grid (the map server republishes the same map)."""
    hsh = hashlib.blake2b(digest_size=12)
    hsh.update(("%d,%d,%.5f,%.4f,%.4f,%.4f" % (w, h, res, origin[0], origin[1], origin[2])).encode())
    try:
        import numpy as np
        hsh.update(np.asarray(data, dtype=np.int8).tobytes())
    except ImportError:
        hsh.update(bytes((v & 0xFF) for v in data))
    return hsh.hexdigest()


def map_msg(w, h, res, origin, data, seq, frame="map"):
    return {"t": "ros_map", "v": LINK_VERSION, "seq": seq, "frame": frame, "w": int(w), "h": int(h),
            "res": round(float(res), 5), "origin": [round(float(o), 4) for o in origin], "enc": "rle4",
            "rle": encode_grid(data)}


# ------------------------------------------------------------------ paths
def _seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L = dx * dx + dy * dy
    if L <= 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def simplify(pts, tol=0.03):
    """Douglas-Peucker (iterative): the fewest points within tol (m) of the path."""
    pts = [tuple(p[:2]) for p in pts]
    if len(pts) < 3:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        best, bi = -1.0, -1
        for k in range(i + 1, j):
            d = _seg_dist(pts[k], pts[i], pts[j])
            if d > best:
                best, bi = d, k
        if best > tol:
            keep[bi] = True
            stack += [(i, bi), (bi, j)]
    return [p for p, k in zip(pts, keep) if k]


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def plan_msg(pts, tol=0.03, max_pts=300):
    s = simplify(pts, tol)
    while len(s) > max_pts:
        tol *= 2.0
        s = simplify(pts, tol)
    return {"t": "ros_plan", "pts": [[round(x, 3), round(y, 3)] for x, y in s],
            "len_m": round(path_length(pts), 2)}


# ------------------------------------------------------------------ frames
def station_to_map(T, x_mm, y_mm, yaw=None):
    """station_map point (mm) -> ROS map (m) with T = map <- station_map (x m, y m, yaw rad)."""
    x, y, a = cv.pose_compose(T, (x_mm / 1000.0, y_mm / 1000.0, yaw or 0.0))
    return (x, y) if yaw is None else (x, y, a)


def map_to_station(T, x, y):
    """ROS map (m) -> station_map (mm)."""
    p = cv.pose_compose(cv.pose_inverse(T), (x, y, 0.0))
    return p[0] * 1000.0, p[1] * 1000.0


def explore_fill(st, T):
    """#107 explorer status -> ros_explore for the station: add the station-frame "target" (mm) from "target_map" (m)
    when the explorer left it out (and T is known)."""
    m = dict(st or {})
    m["t"] = "ros_explore"
    tm = m.get("target_map")
    if m.get("target") is None and tm and T is not None:
        try:
            x, y = map_to_station(T, float(tm[0]), float(tm[1]))
            m["target"] = [round(x), round(y)]
        except (TypeError, ValueError, IndexError):
            pass
    m.setdefault("target", None)
    return m


# ------------------------------------------------------------------ commands
def _f(v, lo=-1e6, hi=1e6):
    x = float(v)
    if not math.isfinite(x) or not lo <= x <= hi:
        raise ValueError("bad number %r" % (v,))
    return x


def parse_pose(m):
    """{"x", "y", "yaw"?, "frame": "map" (m, rad) | "station" (mm, yaw rad)} -> (frame, x, y, yaw or None)."""
    frame = m.get("frame") or "map"
    if frame not in ("map", "station"):
        raise ValueError("frame must be map or station")
    yaw = m.get("yaw")
    return frame, _f(m["x"]), _f(m["y"]), None if yaw is None else cv.wrap_pi(_f(yaw))


def parse_patrol(m):
    names = m.get("places")
    if isinstance(names, str):
        names = names.split(",")
    names = [" ".join(str(n).split()) for n in (names or []) if str(n).strip()][:50]
    if not names:
        raise ValueError("patrol needs at least one place")
    loops = int(_f(m.get("loops", 1), 0, 1000))
    dwell = _f(m.get("dwell", 3.0), 0, 3600)
    return names, loops, dwell


def clean_map_name(name):
    n = "".join(c if c.isalnum() or c in "_-" else "-" for c in str(name or "")).strip("-")[:40]
    return n or None


# ------------------------------------------------------------------ Nav2 status
TERMINAL = ("succeeded", "failed", "cancelled")
NAV2_RESULT = {4: "succeeded", 5: "cancelled", 6: "failed"}       # action_msgs GoalStatus
# Jazzy's NavigateToPose result carries the failing BT node's error code (FollowPath 1xx, ComputePathToPose 2xx)
NAV2_ERRORS = {100: "controller failed", 101: "no such controller", 102: "pose not available in time (TF)",
               103: "invalid path", 104: "stuck: controller patience exceeded", 105: "failed to make progress",
               106: "no valid move (blocked?)", 107: "controller timed out",
               200: "planner failed", 201: "no such planner", 202: "pose not available in time (TF, planner)",
               203: "the robot is outside the map", 204: "the goal is outside the map",
               205: "the robot is on an obstacle", 206: "the goal is on an obstacle / in a keep-out zone",
               207: "planner timed out", 208: "no path to the goal"}


def nav2_reason(code, msg=""):
    msg = (msg or "").strip()
    if msg:
        return msg
    if not code:
        return ""
    return NAV2_ERRORS.get(int(code), "Nav2 error %d" % code)


class NavTracker:
    """The one Nav2 task the station shows: a goal / a place / a patrol from the station, or one sent from elsewhere
    (RViz, go_to.ps1: "external"). msg() is the ros_nav message."""

    def __init__(self):
        self.state = "idle"        # idle | navigating | dwell | succeeded | failed | cancelled
        self.kind = None           # goal | place | patrol | external
        self.goal = None           # [x, y, yaw] map (m, rad)
        self.place = None
        self.reason = ""
        self.dist = self.eta = None
        self.recoveries = 0
        self.t0 = self.t_end = None
        self.patrol = None         # {"places", "i", "loop", "loops", "dwell", "missed", "dwell_end"}
        self.seq = 0

    def _bump(self):
        self.seq += 1

    def start(self, kind, goal, place=None, now=0.0, keep_patrol=False):
        self.state, self.kind, self.goal, self.place = "navigating", kind, list(goal) if goal else None, place
        self.reason, self.dist, self.eta, self.recoveries = "", None, None, 0
        self.t0, self.t_end = now, None
        if not keep_patrol:
            self.patrol = None
        self._bump()

    def feedback(self, dist, eta, recoveries=0):
        self.dist = None if dist is None else max(0.0, float(dist))
        self.eta = None if eta is None else max(0.0, float(eta))
        self.recoveries = int(recoveries or 0)

    def dwell(self, until):
        self.state = "dwell"
        if self.patrol is not None:
            self.patrol["dwell_end"] = until
        self.dist = self.eta = None
        self._bump()

    def finish(self, result, reason="", now=0.0):
        if result not in TERMINAL:
            result = "failed"
        self.state, self.reason, self.t_end = result, reason or "", now
        self.dist = self.eta = None
        self._bump()

    def busy(self):
        return self.state in ("navigating", "dwell")

    def msg(self, now=0.0):
        m = {"t": "ros_nav", "state": self.state, "kind": self.kind, "seq": self.seq,
             "goal": None if self.goal is None else [round(v, 3) for v in self.goal], "place": self.place,
             "reason": self.reason, "dist_m": None if self.dist is None else round(self.dist, 2),
             "eta_s": None if self.eta is None else round(self.eta),
             "recoveries": self.recoveries,
             "elapsed_s": round((self.t_end if self.t_end is not None else now) - self.t0, 1) if self.t0 is not None else None}
        if self.patrol is not None:
            p = self.patrol
            m["patrol"] = {"places": p["places"], "i": p["i"], "loop": p["loop"], "loops": p["loops"],
                           "dwell": p["dwell"], "missed": p["missed"],
                           "dwell_left": round(max(0.0, p["dwell_end"] - now), 1) if p.get("dwell_end") and
                           self.state == "dwell" else None}
        return m


def patrol_next(p):
    """Advance a patrol dict to the next stop: -> (i, loop) or None when it is over."""
    i, loop = p["i"] + 1, p["loop"]
    if i >= len(p["places"]):
        i, loop = 0, loop + 1
    if p["loops"] > 0 and loop >= p["loops"]:
        return None
    return i, loop
