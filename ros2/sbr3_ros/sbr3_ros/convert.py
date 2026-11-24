"""Bridge ROS stream (station/bridge/ros_stream.py) <-> ROS values. Pure Python: no rclpy, testable on Windows.

Units: the bridge speaks mm, deg, deg/s and its own monotonic clock (s); ROS wants m, rad, rad/s and ROS time.
"""
import base64
import math
import struct

INF = float("inf")


# ------------------------------------------------------------------ clock
class ClockSync:
    """Bridge monotonic seconds -> ROS seconds with one offset.

    Every stream message carries "now" (bridge clock at send). ros_now - now = offset + network latency, so the
    smallest value seen is the best offset; it may creep up slowly (clock rates differ) and is reset after a jump
    (bridge restart: its monotonic clock starts elsewhere)."""

    def __init__(self, creep=5e-4, jump_s=2.0):
        self.offset = None
        self.creep = creep
        self.jump_s = jump_s
        self._last_ros = None
        self.resets = 0

    def update(self, bridge_now, ros_now):
        cand = ros_now - bridge_now
        if self.offset is None or abs(cand - self.offset) > self.jump_s:
            if self.offset is not None:
                self.resets += 1
            self.offset = cand
        elif cand < self.offset:
            self.offset = cand
        else:
            dt = max(0.0, ros_now - (self._last_ros or ros_now))
            self.offset = min(cand, self.offset + self.creep * dt)
        self._last_ros = ros_now
        return self.offset

    def to_ros(self, ts):
        return None if self.offset is None else ts + self.offset


def split_stamp(t):
    """ROS seconds (float) -> (sec, nanosec)."""
    sec = int(math.floor(t))
    ns = int(round((t - sec) * 1e9))
    if ns >= 1000000000:
        sec, ns = sec + 1, ns - 1000000000
    return sec, ns


# ------------------------------------------------------------------ geometry
def quat_from_rpy(roll, pitch, yaw):
    """(x, y, z, w) from roll / pitch / yaw in radians (ZYX, as tf2)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_yaw(yaw):
    return quat_from_rpy(0.0, 0.0, yaw)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def corr_to_tf(corr):
    """Bridge map correction [dx mm, dy mm, dth deg] (map pose = corr o odometry pose) -> map <- odom (x m, y m, yaw rad)."""
    dx, dy, dth = corr
    return dx / 1000.0, dy / 1000.0, wrap_pi(math.radians(dth))


def pose_inverse(p):
    """(x, y, yaw) of b in a -> (x, y, yaw) of a in b."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return -(c * p[0] + s * p[1]), -(-s * p[0] + c * p[1]), wrap_pi(-p[2])


def pose_compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_pi(a[2] + b[2])


def station_frame_tf(corr):
    """odom <- station_map (x m, y m, yaw rad): the station's map frame (places, zones) as a child of odom, from the
    bridge's correction (map pose = corr o odometry pose). TF then gives the ROS map <- station_map."""
    return pose_inverse(corr_to_tf(corr or (0.0, 0.0, 0.0)))


# ------------------------------------------------------------------ station places / zones
def places_from_msg(m):
    """bridge {"t": "places", "map": ..., "items": [{"name", "x", "y" (station map mm), "home"?}]} ->
    [{"name", "x", "y" (m), "home"}]"""
    out = []
    for p in (m or {}).get("items") or []:
        try:
            out.append({"name": str(p["name"]), "x": float(p["x"]) / 1000.0, "y": float(p["y"]) / 1000.0,
                        "home": bool(p.get("home"))})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def find_place(places, name):
    """Case-insensitive, spaces collapsed; 'home' finds Home. -> place dict or None."""
    key = " ".join(str(name or "").split()).lower()
    return next((p for p in places if p["name"].lower() == key), None)


def route_yaws(points, start=None):
    """Heading for each goal: the direction it is approached from (previous point, or start), so the robot does not
    turn on the spot when it arrives. A point on top of the previous one keeps the previous heading."""
    yaws, prev, last = [], start, 0.0
    for p in points:
        if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) > 1e-3:
            last = math.atan2(p[1] - prev[1], p[0] - prev[0])
        yaws.append(last)
        prev = p
    return yaws


# ------------------------------------------------------------------ scans
def decode_ranges(s):
    b = base64.b64decode(s)
    return struct.unpack("<%dH" % (len(b) // 2), b)


def scan_ranges(msg, range_min_m=0.0, range_max_m=INF):
    """ros_scan "r" -> ranges in m; 0 (no return / filtered) and out-of-window bins -> +inf (REP 117)."""
    out = []
    for r in decode_ranges(msg["r"]):
        m = r / 1000.0
        out.append(m if r and range_min_m <= m <= range_max_m else INF)
    return out


def scan_layout(hello):
    """ros_hello -> LaserScan fields that never change (angle_min, angle_max, angle_increment, range_min, range_max m)."""
    sc = hello["scan"]
    li = hello.get("lidar") or {}
    n = int(sc["bins"])
    inc = float(sc["angle_increment"])
    amin = float(sc["angle_min"])
    return {"bins": n, "angle_min": amin, "angle_max": amin + inc * (n - 1), "angle_increment": inc,
            "range_min": li.get("min_range_mm", 120.0) / 1000.0, "range_max": li.get("max_range_mm", 8000.0) / 1000.0,
            "scan_time": 1.0 / float(sc.get("hz", 10))}


def keep_scan(msg, max_tilt_deg):
    """False for a turn taken while the robot leaned too far (the scan plane is not level)."""
    return msg.get("tilt", 0.0) <= max_tilt_deg and not msg.get("tilt_pk", 0)


def laser_mount(hello):
    """ros_hello -> base_link -> laser static transform (x, y, z m, yaw rad). The bridge already turned the ranges into
    the robot's orientation, so the yaw is 0."""
    li = hello.get("lidar") or {}
    return li.get("x_mm", 0.0) / 1000.0, li.get("y_mm", 0.0) / 1000.0, li.get("height_mm", 208.0) / 1000.0, 0.0


# ------------------------------------------------------------------ odometry
def odom_rows(msg, cols):
    """ros_odom -> [dict] in ROS units: t (bridge s), x, y (m), yaw (rad), v (m/s), w (rad/s), pitch, roll (rad),
    pitch_rate, yaw_rate (rad/s)."""
    out = []
    ix = {c: i for i, c in enumerate(cols)}
    for row in msg.get("rows", []):
        g = lambda k: row[ix[k]] if k in ix and ix[k] < len(row) else 0.0
        out.append({"t": g("ts"), "x": g("x_mm") / 1000.0, "y": g("y_mm") / 1000.0,
                    "yaw": wrap_pi(math.radians(g("heading_deg"))),
                    "v": g("speed_mms") / 1000.0, "w": math.radians(g("turn_rate_dps")),
                    "pitch": math.radians(g("pitch_deg")), "roll": math.radians(g("roll_deg")),
                    "pitch_rate": math.radians(g("pitch_rate_dps")), "yaw_rate": math.radians(g("yaw_rate_dps"))})
    return out


# ------------------------------------------------------------------ cmd_vel
DRIVE_MODES = ("STATION",)


def drivable(state, now_s=None, max_age_s=1.5):
    """(ok, why): the robot takes station cmd_vel only in STATION mode, armed, no E-stop, with live telemetry.
    The adapter never arms and never changes the mode: that stays on the phone / remote."""
    if not state:
        return False, "no robot state from the bridge yet"
    if now_s is not None and state.get("_rx") is not None and now_s - state["_rx"] > max_age_s:
        return False, "robot state is stale (bridge silent)"
    if state.get("link") != "connected" or not state.get("alive"):
        return False, "robot not connected / not responding"
    if state.get("estop") or state.get("robot_estop"):
        return False, "E-stop is latched"
    if not state.get("armed"):
        return False, "robot is disarmed (arm it on the phone / remote)"
    if state.get("mode") not in DRIVE_MODES:
        return False, "robot in %s; tap Manual (STATION) on the phone to let ROS drive" % state.get("mode")
    return True, ""


class CmdGate:
    """/cmd_vel -> station {"t": "cmd_vel", "speed": mm/s, "turn": deg/s}, at the timer rate.

    - deadman: no Twist for deadman_s -> one zero command, then silence (the bridge stops forwarding after 300 ms and
      the firmware after its own timeout);
    - caps: |v| <= max_speed m/s, |w| <= max_turn rad/s; slew limits max_accel m/s^2, max_turn_accel rad/s^2;
    - only while drivable(); otherwise the output is reset to 0 and nothing is sent."""

    def __init__(self, max_speed=0.25, max_turn=1.0, max_accel=0.4, max_turn_accel=2.0, deadman_s=0.5):
        self.max_speed, self.max_turn = max_speed, max_turn
        self.max_accel, self.max_turn_accel = max_accel, max_turn_accel
        self.deadman_s = deadman_s
        self.target = (0.0, 0.0)
        self.t_cmd = None
        self.out = (0.0, 0.0)
        self.t_step = None
        self.sending = False
        self.why = ""

    def on_twist(self, v, w, t):
        if not (math.isfinite(v) and math.isfinite(w)):
            v = w = 0.0
        self.target = (max(-self.max_speed, min(self.max_speed, v)), max(-self.max_turn, min(self.max_turn, w)))
        self.t_cmd = t

    def step(self, t, state):
        """-> station command dict to send now, or None."""
        dt = 0.05 if self.t_step is None else max(0.0, min(0.2, t - self.t_step))
        self.t_step = t
        fresh = self.t_cmd is not None and t - self.t_cmd <= self.deadman_s
        ok, why = drivable(state, t)
        self.why = "" if ok else why
        if not fresh or not ok:
            self.out = (0.0, 0.0)
            if self.sending:
                self.sending = False
                return {"t": "cmd_vel", "speed": 0, "turn": 0}     # one explicit stop
            return None
        v0, w0 = self.out
        tv, tw = self.target
        dv, dw = self.max_accel * dt, self.max_turn_accel * dt
        v = v0 + max(-dv, min(dv, tv - v0))
        w = w0 + max(-dw, min(dw, tw - w0))
        self.out = (v, w)
        self.sending = True
        return {"t": "cmd_vel", "speed": int(round(v * 1000.0)), "turn": round(math.degrees(w), 1)}
