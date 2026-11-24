"""Link protocol v6 (Teensy <-> Orange Pi). Mirror of include/protocol/link_protocol.h.

Frame: 0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 u16 LE
crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
All fields little-endian, packed. Keep in sync with the C header (tests check it).
Pure Python, 3.8+.
"""
import struct

LINK_PROTOCOL_VERSION = 6
LINK_SYNC1 = 0xA5
LINK_SYNC2 = 0x5A
LINK_MAX_PAYLOAD = 200
LINK_HEARTBEAT_TIMEOUT_MS = 500
LINK_CMD_VEL_TIMEOUT_MS = 300

# LinkMsgType
MSG_TELEMETRY = 0x01
MSG_RANGE = 0x02
MSG_LOG = 0x03
MSG_PARAM_INFO = 0x04
MSG_NAV_STATUS = 0x05
MSG_HELLO = 0x06
MSG_HEARTBEAT = 0x80
MSG_CMD_VEL = 0x81
MSG_SET_MODE = 0x82
MSG_SET_PARAM = 0x83
MSG_GET_PARAMS = 0x84
MSG_GOTO = 0x85
MSG_EVENT = 0x86
MSG_LIDAR_OBSTACLES = 0x87    # v5 (#77): LD19 obstacle sectors for the navigator

MSG_NAMES = {v: k for k, v in globals().items() if k.startswith("MSG_") and isinstance(v, int)}

# From radio_protocol.h (shared enums used on the link)
RADIO_PROTOCOL_VERSION = 2
MODE_IDLE, MODE_BALANCE, MODE_MANUAL, MODE_STATION, MODE_GOTO, MODE_EXPLORE, MODE_HOLD = range(7)
MODE_COUNT = 7
MODE_NAMES = ["IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"]

EVT_NONE, EVT_MARK_WAYPOINT, EVT_CLEAR_WAYPOINTS, EVT_GO_HOME, EVT_RESET_POSE, EVT_START_SCAN = range(6)
EVT_COUNT = 6
EVENT_NAMES = ["NONE", "MARK_WAYPOINT", "CLEAR_WAYPOINTS", "GO_HOME", "RESET_POSE", "START_SCAN"]

# Link-only EventFrame.event values (link_protocol.h LinkEvent, v4). Never from the remote.
EVT_LINK_ESTOP = 0x80          # station E-stop: IDLE + disarm, latched; no mode request can start the motors
EVT_LINK_ESTOP_RELEASE = 0x81  # clears the latch only (still IDLE + disarmed; then the normal arming rules)
EVT_LINK_ARM = 0x82            # explicit arm request: the next SET_MODE != IDLE within LINK_ARM_WINDOW_MS arms
EVT_LINK_STUCK = 0x83          # v6 (#79): the bridge's lidar saw the wheels turn but no body motion: navigator stuck
LINK_ARM_WINDOW_MS = 500

TLM_ARMED = 0x01
TLM_UPRIGHT = 0x02
TLM_FAILSAFE = 0x04
TLM_PI_LINK = 0x08
TLM_OBSTACLE = 0x10
TLM_LOW_BATT = 0x20
TLM_NAV_ACTIVE = 0x40
TLM_NAV_DONE = 0x80
TLM_BITS = {"armed": TLM_ARMED, "upright": TLM_UPRIGHT, "failsafe": TLM_FAILSAFE,
            "pi_link": TLM_PI_LINK, "obstacle": TLM_OBSTACLE, "low_batt": TLM_LOW_BATT,
            "nav_active": TLM_NAV_ACTIVE, "nav_done": TLM_NAV_DONE}

GOTO_MAX_POINTS = 16

# MSG_LIDAR_OBSTACLES (v5, #77): nearest LD19 return per 10 deg sector, robot frame (see LidarObstacleFrame)
LINK_LIDAR_SECTORS = 36
LINK_LIDAR_FRESH_MS = 500       # firmware: no frame for this long -> no lidar (sweep or blind)
LINK_LIDAR_MAX_AGE_MS = 300     # firmware drops frames whose pose is older (bridge: does not send them)
LIDAR_SECTOR_DEG = 360.0 / LINK_LIDAR_SECTORS

# TelemetryFrame.remote_flags (v3)
LINK_RF_ESTOP = 0x01      # E-stop latched (from the remote)
LINK_RF_ARM_WAIT = 0x02   # arming needs the remote's ARMED switch off -> on
LINK_RF_PRESENT = 0x04    # remote packets in the last second (radio_ok = last 250 ms)
LINK_RF_LINK_ESTOP = 0x08    # v4: station E-stop latched (until EVT_LINK_ESTOP_RELEASE)
LINK_RF_NAV_HANDOVER = 0x10  # v4: GOTO/EXPLORE/HOLD paused: remote lost; needs it back or a station SET_MODE
RF_BITS = {"estop": LINK_RF_ESTOP, "arm_wait": LINK_RF_ARM_WAIT, "present": LINK_RF_PRESENT,
           "link_estop": LINK_RF_LINK_ESTOP, "nav_handover": LINK_RF_NAV_HANDOVER}

# NavState (NavStatusFrame.state, TelemetryFrame.nav_state)
NAV_IDLE, NAV_TURNING, NAV_DRIVING, NAV_BLOCKED, NAV_ARRIVED, NAV_HOLDING = range(6)
NAV_STATE_COUNT = 6
NAV_STATE_NAMES = ["idle", "turning", "driving", "blocked", "arrived", "holding"]

# ---- struct layouts: (format, field names in C order) ------------------------------------
HELLO_FMT = "<BBH24s"
HELLO_FIELDS = ("link_version", "radio_version", "param_count", "firmware")

TELEMETRY_FMT = "<IBBBB23fIBBBB"
TELEMETRY_FIELDS = (
    "t_us", "mode", "status", "error_code", "radio_ok",
    "pitch_deg", "pitch_rate_dps", "roll_deg", "heading_deg", "yaw_rate_dps",
    "x_mm", "y_mm", "speed_mms", "turn_rate_dps", "cmd_speed_mms", "cmd_turn_dps",
    "wheel_left_mms", "wheel_right_mms", "balance_target_deg",
    "motor_left", "motor_right", "battery_v", "loop_hz",
    # v2: balance tuning data
    "bal_p", "bal_i", "bal_d", "steer_out", "vel_out_deg", "step",
    # v3: remote E-stop / arm-wait flags, nav state
    "remote_flags", "nav_state", "nav_index", "reserved2",
)
TELEMETRY_V1_SIZE = 80   # older frames are still decoded; the missing fields read as 0
TELEMETRY_V2_SIZE = 104

RANGE_FMT = "<I5fHBB"
RANGE_FIELDS = ("t_us", "x_mm", "y_mm", "heading_deg", "pitch_deg", "sensor_angle_deg",
                "range_mm", "sensor_id", "valid")

PARAM_INFO_FMT = "<H3f24s"
PARAM_INFO_FIELDS = ("id", "value", "min", "max", "name")

NAV_STATUS_FMT = "<BBBB4f"
NAV_STATUS_FIELDS = ("state", "waypoint_count", "nav_index", "reserved",
                     "target_x_mm", "target_y_mm", "distance_mm", "obstacle_mm")
NAV_STATUS_V2_SIZE = 16  # v2 frames have no obstacle_mm (reads as 0)

HEARTBEAT_FMT = "<IB"
HEARTBEAT_FIELDS = ("pi_ms", "clients")
CMD_VEL_FMT = "<2f"
CMD_VEL_FIELDS = ("speed_mms", "turn_dps")
SET_MODE_FMT = "<B"
SET_MODE_FIELDS = ("mode",)
SET_PARAM_FMT = "<Hf"
SET_PARAM_FIELDS = ("id", "value")
GOTO_HDR_FMT = "<BB"
GOTO_POINT_FMT = "<2f"
EVENT_FMT = "<B"
EVENT_FIELDS = ("event",)
LIDAR_OBST_FMT = "<BBH" + "h" * (2 * LINK_LIDAR_SECTORS)      # xy_mm[72] = x0, y0, x1, y1, ...
LIDAR_OBST_FIELDS = ("seq", "sectors", "age_ms", "xy_mm")

STRUCTS = {  # C struct name -> (format, fields); GotoFrame is variable length (see encode_goto)
    "HelloFrame": (HELLO_FMT, HELLO_FIELDS),
    "TelemetryFrame": (TELEMETRY_FMT, TELEMETRY_FIELDS),
    "RangeFrame": (RANGE_FMT, RANGE_FIELDS),
    "ParamInfoFrame": (PARAM_INFO_FMT, PARAM_INFO_FIELDS),
    "NavStatusFrame": (NAV_STATUS_FMT, NAV_STATUS_FIELDS),
    "HeartbeatFrame": (HEARTBEAT_FMT, HEARTBEAT_FIELDS),
    "CmdVelFrame": (CMD_VEL_FMT, CMD_VEL_FIELDS),
    "SetModeFrame": (SET_MODE_FMT, SET_MODE_FIELDS),
    "SetParamFrame": (SET_PARAM_FMT, SET_PARAM_FIELDS),
    "EventFrame": (EVENT_FMT, EVENT_FIELDS),
    "LidarObstacleFrame": (LIDAR_OBST_FMT, LIDAR_OBST_FIELDS),   # xy_mm is an array: encode/decode_lidar_obstacles
}

_S = {name: struct.Struct(fmt) for name, (fmt, _f) in STRUCTS.items()}
TELEMETRY_SIZE = _S["TelemetryFrame"].size      # 108
RANGE_SIZE = _S["RangeFrame"].size              # 28
PARAM_INFO_SIZE = _S["ParamInfoFrame"].size     # 38
HELLO_SIZE = _S["HelloFrame"].size              # 28
NAV_STATUS_SIZE = _S["NavStatusFrame"].size     # 20
LIDAR_OBST_SIZE = _S["LidarObstacleFrame"].size  # 148


# ---- CRC-16/CCITT-FALSE --------------------------------------------------------------------
def _make_table():
    t = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1)
        t.append(c & 0xFFFF)
    return t


_CRC_TABLE = _make_table()


def crc16(data, crc=0xFFFF):
    tbl = _CRC_TABLE
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ b) & 0xFF]
    return crc


# ---- framing -------------------------------------------------------------------------------
def encode_frame(msg_type, payload=b""):
    if len(payload) > LINK_MAX_PAYLOAD:
        raise ValueError("payload too long (%d > %d)" % (len(payload), LINK_MAX_PAYLOAD))
    body = bytes((msg_type & 0xFF, len(payload))) + bytes(payload)
    return bytes((LINK_SYNC1, LINK_SYNC2)) + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Incremental parser. feed(bytes) -> list of (type, payload). Resyncs on garbage/bad CRC."""

    def __init__(self):
        self.buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.bytes_dropped = 0

    def feed(self, data):
        buf = self.buf
        buf += data
        out = []
        sync = b"\xA5\x5A"
        while True:
            i = buf.find(sync)
            if i < 0:
                # keep a trailing 0xA5 (could be the first half of a sync)
                keep = 1 if buf and buf[-1] == LINK_SYNC1 else 0
                self.bytes_dropped += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if i:
                self.bytes_dropped += i
                del buf[:i]
            if len(buf) < 4:
                break
            n = buf[3]
            if n > LINK_MAX_PAYLOAD:
                self.crc_errors += 1
                self.bytes_dropped += 1
                del buf[:1]
                continue
            total = 4 + n + 2
            if len(buf) < total:
                break
            body = bytes(buf[2:4 + n])
            rx_crc = buf[4 + n] | (buf[5 + n] << 8)
            if crc16(body) != rx_crc:
                self.crc_errors += 1
                self.bytes_dropped += 1
                del buf[:1]
                continue
            out.append((body[0], body[2:]))
            self.frames_ok += 1
            del buf[:total]
        return out


# ---- decode (Teensy -> Pi) -----------------------------------------------------------------
def _cstr(b):
    return b.split(b"\0", 1)[0].decode("utf-8", "replace")


def _unpack(name, payload):
    s = _S[name]
    if len(payload) < s.size:
        raise ValueError("%s: payload %d < %d bytes" % (name, len(payload), s.size))
    return dict(zip(STRUCTS[name][1], s.unpack_from(payload)))


def decode_hello(p):
    d = _unpack("HelloFrame", p)
    d["firmware"] = _cstr(d["firmware"])
    return d


def _pad(p, sizes, full):
    """Older (shorter) frame of a known size -> zero-padded to the current size."""
    for n in sizes:
        if n <= len(p) < full:
            return bytes(p[:n]) + bytes(full - n)
    return p


def decode_telemetry(p):
    # older firmware: link v2 (104 bytes), v1 (80 bytes)
    return _unpack("TelemetryFrame", _pad(p, (TELEMETRY_V2_SIZE, TELEMETRY_V1_SIZE), TELEMETRY_SIZE))


def decode_range(p):
    return _unpack("RangeFrame", p)


def decode_param_info(p):
    d = _unpack("ParamInfoFrame", p)
    d["name"] = _cstr(d["name"])
    return d


def decode_nav_status(p):
    return _unpack("NavStatusFrame", _pad(p, (NAV_STATUS_V2_SIZE,), NAV_STATUS_SIZE))


def decode_log(p):
    return bytes(p).decode("utf-8", "replace")


def decode_heartbeat(p):
    return _unpack("HeartbeatFrame", p)


def decode_cmd_vel(p):
    return _unpack("CmdVelFrame", p)


def decode_set_mode(p):
    return _unpack("SetModeFrame", p)


def decode_set_param(p):
    return _unpack("SetParamFrame", p)


def decode_event(p):
    return _unpack("EventFrame", p)


def decode_goto(p):
    replace, count = struct.unpack_from(GOTO_HDR_FMT, p)
    count = min(count, GOTO_MAX_POINTS, (len(p) - 2) // 8)
    pts = [struct.unpack_from(GOTO_POINT_FMT, p, 2 + 8 * i) for i in range(count)]
    return {"replace": replace, "count": count, "points": pts}


def decode_lidar_obstacles(p):
    """-> {"seq", "sectors", "age_ms", "pts": [(x_mm, y_mm) per sector, (0, 0) = no return]}. A frame that is not
    exactly LIDAR_OBST_SIZE bytes (or has another sector count) raises ValueError, like the firmware drops it."""
    if len(p) != LIDAR_OBST_SIZE:
        raise ValueError("LidarObstacleFrame: payload %d != %d bytes" % (len(p), LIDAR_OBST_SIZE))
    v = _S["LidarObstacleFrame"].unpack_from(p)
    if v[1] != LINK_LIDAR_SECTORS:
        raise ValueError("LidarObstacleFrame: %d sectors, expected %d" % (v[1], LINK_LIDAR_SECTORS))
    xy = v[3:]
    return {"seq": v[0], "sectors": v[1], "age_ms": v[2],
            "pts": [(xy[2 * i], xy[2 * i + 1]) for i in range(LINK_LIDAR_SECTORS)]}


DECODERS = {
    MSG_TELEMETRY: decode_telemetry, MSG_RANGE: decode_range, MSG_LOG: decode_log,
    MSG_PARAM_INFO: decode_param_info, MSG_NAV_STATUS: decode_nav_status, MSG_HELLO: decode_hello,
    MSG_HEARTBEAT: decode_heartbeat, MSG_CMD_VEL: decode_cmd_vel, MSG_SET_MODE: decode_set_mode,
    MSG_SET_PARAM: decode_set_param, MSG_GET_PARAMS: lambda p: {}, MSG_GOTO: decode_goto,
    MSG_EVENT: decode_event, MSG_LIDAR_OBSTACLES: decode_lidar_obstacles,
}


def decode(msg_type, payload):
    """Decode a payload by type. Unknown types -> None."""
    f = DECODERS.get(msg_type)
    return f(payload) if f else None


# ---- encode --------------------------------------------------------------------------------
def _pack(name, d):
    fmt, fields = STRUCTS[name]
    vals = []
    for f in fields:
        v = d[f]
        if isinstance(v, str):
            v = v.encode("utf-8")
        vals.append(v)
    return _S[name].pack(*vals)


def encode_heartbeat(pi_ms, clients):
    return encode_frame(MSG_HEARTBEAT, struct.pack(HEARTBEAT_FMT, pi_ms & 0xFFFFFFFF, min(clients, 255)))


def encode_cmd_vel(speed_mms, turn_dps):
    return encode_frame(MSG_CMD_VEL, struct.pack(CMD_VEL_FMT, speed_mms, turn_dps))


def encode_set_mode(mode):
    return encode_frame(MSG_SET_MODE, struct.pack(SET_MODE_FMT, mode))


def encode_set_param(pid, value):
    return encode_frame(MSG_SET_PARAM, struct.pack(SET_PARAM_FMT, pid, value))


def encode_get_params():
    return encode_frame(MSG_GET_PARAMS)


def encode_goto(points, replace=True):
    """points: list of (x_mm, y_mm), at most 16 per frame (see goto_frames for longer lists)."""
    if len(points) > GOTO_MAX_POINTS:
        raise ValueError("max %d points per GotoFrame" % GOTO_MAX_POINTS)
    p = struct.pack(GOTO_HDR_FMT, 1 if replace else 0, len(points))
    p += b"".join(struct.pack(GOTO_POINT_FMT, x, y) for x, y in points)
    return encode_frame(MSG_GOTO, p)


def goto_frames(points, replace=True):
    """Split any number of points into GotoFrames (first one replaces, the rest append)."""
    frames = []
    for i in range(0, max(len(points), 1), GOTO_MAX_POINTS):
        frames.append(encode_goto(points[i:i + GOTO_MAX_POINTS], replace and i == 0))
    return frames


def encode_event(event):
    return encode_frame(MSG_EVENT, struct.pack(EVENT_FMT, event))


def _i16(v):
    v = int(round(v))
    return -32768 if v < -32768 else (32767 if v > 32767 else v)


def encode_lidar_obstacles(seq, age_ms, pts):
    """pts: LINK_LIDAR_SECTORS (x_mm, y_mm) in the robot frame, None = no return in that sector (sent as 0, 0).
    A point that would round to (0, 0) is sent as (1, 0): it is a return (at the axle), not "none"."""
    if len(pts) != LINK_LIDAR_SECTORS:
        raise ValueError("need %d sectors, got %d" % (LINK_LIDAR_SECTORS, len(pts)))
    xy = []
    for p in pts:
        if p is None:
            xy += (0, 0)
            continue
        x, y = _i16(p[0]), _i16(p[1])
        xy += (x, y) if (x or y) else (1, 0)
    age = max(0, min(0xFFFF, int(round(age_ms))))
    return encode_frame(MSG_LIDAR_OBSTACLES, _S["LidarObstacleFrame"].pack(seq & 0xFF, LINK_LIDAR_SECTORS, age, *xy))


# Teensy -> Pi encoders (used by the simulator and tests)
def encode_hello(link_version=LINK_PROTOCOL_VERSION, radio_version=RADIO_PROTOCOL_VERSION,
                 param_count=0, firmware=""):
    return encode_frame(MSG_HELLO, _pack("HelloFrame", locals()))


def encode_telemetry(d):
    d = dict(d)
    for k in ("remote_flags", "nav_state", "nav_index", "reserved2"):   # v3 fields
        d.setdefault(k, 0)
    return encode_frame(MSG_TELEMETRY, _pack("TelemetryFrame", d))


def encode_range(d):
    return encode_frame(MSG_RANGE, _pack("RangeFrame", d))


def encode_param_info(pid, value, vmin, vmax, name):
    return encode_frame(MSG_PARAM_INFO, _pack("ParamInfoFrame", {
        "id": pid, "value": value, "min": vmin, "max": vmax, "name": name}))


def encode_nav_status(d):
    d = dict(d)
    d.setdefault("reserved", 0)
    d.setdefault("obstacle_mm", 0.0)
    return encode_frame(MSG_NAV_STATUS, _pack("NavStatusFrame", d))


def encode_log(text):
    return encode_frame(MSG_LOG, text.encode("utf-8")[:LINK_MAX_PAYLOAD])
