"""Link protocol v10 (Teensy <-> Orange Pi). Mirror of include/protocol/link_protocol.h.

Frame: 0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 u16 LE
crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
All fields little-endian, packed. Keep in sync with the C header (tests check it).
Pure Python, 3.8+.
"""
import binascii
import struct

LINK_PROTOCOL_VERSION = 10
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
MSG_LIDAR_RAW = 0x07          # v7 (#100): raw LD19 bytes from the Teensy's Serial7
MSG_CONTACT = 0x08            # v8 (#104): the navigator felt a contact below the lidar's scan plane
MSG_HEARTBEAT = 0x80
MSG_CMD_VEL = 0x81
MSG_SET_MODE = 0x82
MSG_SET_PARAM = 0x83
MSG_GET_PARAMS = 0x84
MSG_GOTO = 0x85
MSG_EVENT = 0x86
MSG_LIDAR_OBSTACLES = 0x87    # v5 (#77): LD19 obstacle sectors for the navigator
MSG_LIDAR_CONFIG = 0x88       # v10 (#113): the lidar mount settings for the Teensy's own obstacle sectors

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

# HelloFrame.caps (v10, #113)
LINK_CAP_LIDAR_SECTORS = 0x01   # the firmware computes the LD19 sectors: send it MSG_LIDAR_CONFIG, not obstacle frames
# MSG_LIDAR_CONFIG (v10, #113): LidarConfigFrame.flags
LINK_LCFG_ENABLED = 0x01
LINK_LCFG_FLIP = 0x02
LINK_LCFG_PITCH_AUTO = 0x04
LINK_LCFG_FLOOR_CLIP = 0x08
LINK_LCFG_DESKEW = 0x10
LINK_LIDAR_MASK_MAX = 8

# MSG_LIDAR_RAW (v7, #100): seq u8, flags u8, then 1 .. LINK_LIDAR_RAW_MAX raw LD19 bytes (see LidarRawFrame)
LINK_LIDAR_RAW_MAX = 180
LINK_LIDAR_RAW_OVERFLOW = 0x01  # the Teensy's Serial7 buffer filled / overran since the previous frame: bytes missing

# MSG_CONTACT (v8, #104): ContactFrame.kind / flags
LINK_CONTACT_BUMP = 1           # wheels slowed far more than the command explains + a pitch-rate spike
LINK_CONTACT_STUCK = 2          # pushing (command, lean, effort) but the speed held ~0
LINK_CONTACT_LIDAR = 0x01       # flags: the bridge's lidar stuck (EVT_LINK_STUCK) agreed
LINK_CONTACT_FELL = 0x02        # flags: the robot fell over right after
LINK_CONTACT_REPORT = 0x04      # flags: STATION, reported only (the motion was not changed)
CONTACT_KIND_NAMES = {LINK_CONTACT_BUMP: "bump", LINK_CONTACT_STUCK: "stuck"}

# HelloFrame.reset_cause (v9, #109): why the Teensy last reset (src/reset_cause.h)
LINK_RESET_UNKNOWN = 0
LINK_RESET_POWER_ON = 1      # cold start: power was off
LINK_RESET_POWER_DIP = 2     # power-on reset with the SNVS domain kept, off < LINK_RESET_DIP_S: brownout likely
LINK_RESET_WATCHDOG = 3      # the hardware watchdog fired: the main loop hung / starved the link
LINK_RESET_LOCKUP = 4        # CPU lockup or a software reset the firmware did not ask for
LINK_RESET_FAULT = 5         # crash: the core's fault handler rebooted (CrashReport logged)
LINK_RESET_REQUESTED = 6     # the firmware asked for it (reboot button)
LINK_RESET_FLASHED = 7       # another firmware build than the previous run
LINK_RESET_TEMP = 8          # chip over-temperature
LINK_RESET_OTHER = 9         # WDOG1/2, JTAG, security monitor
LINK_RESET_DIP_S = 3
RESET_CAUSE_NAMES = ["unknown", "power-on", "power dip (brownout?)", "watchdog", "lockup / software reset",
                     "crash (fault handler)", "requested", "flashed", "over-temperature", "other"]
RESET_UNEXPECTED = {LINK_RESET_POWER_DIP, LINK_RESET_WATCHDOG, LINK_RESET_LOCKUP, LINK_RESET_FAULT,
                    LINK_RESET_TEMP, LINK_RESET_OTHER}
LINK_RESETF_CRASH = 0x01     # a CrashReport was saved before this boot
LINK_RESETF_SNVS = 0x02      # the SNVS domain kept its contents: prev_uptime_s / off_s known
LINK_RESETF_WDT = 0x04       # the hardware watchdog runs this boot

# TelemetryFrame.fault_flags (v9, #109; was reserved2)
LINK_FAULT_ENC_L = 0x01      # left encoder fault latched (error_code 3)
LINK_FAULT_ENC_R = 0x02      # right encoder fault latched (error_code 4)
LINK_FAULT_ENC_L_ERR = 0x04  # left encoder I2C errors / rejected jumps / recoveries in the last second
LINK_FAULT_ENC_R_ERR = 0x08
LINK_FAULT_WDT_OFF = 0x10    # the hardware watchdog is not running

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
HELLO_FMT = "<BBH24sBBHIIII"
HELLO_FIELDS = ("link_version", "radio_version", "param_count", "firmware",
                # v9 (#109): why the Teensy last reset
                "reset_cause", "reset_flags", "srsr", "prev_uptime_s", "off_s", "uptime_ms",
                "caps")                  # v10 (#113): LINK_CAP_* bits
HELLO_V8_SIZE = 28       # v8 and older: no reset cause (decoded as 0 = unknown)
HELLO_V9_SIZE = 44       # v9: no caps (decoded as 0)

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
    "remote_flags", "nav_state", "nav_index", "fault_flags",   # v9: fault_flags (was reserved2)
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
LIDAR_RAW_FMT = "<BB%dB" % LINK_LIDAR_RAW_MAX                  # header + the most data bytes; len = 2 + n
LIDAR_RAW_FIELDS = ("seq", "flags", "data")
LIDAR_CONFIG_FMT = "<BBH10f" + "H" * (2 * LINK_LIDAR_MASK_MAX)    # mask_cdeg[16] = from0, to0, from1, to1, ...
LIDAR_CONFIG_FIELDS = ("flags", "mask_n", "reserved", "x_mm", "y_mm", "yaw_deg", "height_mm", "min_range_mm",
                       "max_range_mm", "min_intensity", "pitch_gate_deg", "pitch_ref_deg", "time_offset_ms", "mask_cdeg")
CONTACT_FMT = "<BBBB4f"
CONTACT_FIELDS = ("kind", "seq", "mode", "flags", "x_mm", "y_mm", "heading_deg", "value")

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
    "LidarRawFrame": (LIDAR_RAW_FMT, LIDAR_RAW_FIELDS),          # variable length: encode/decode_lidar_raw
    "ContactFrame": (CONTACT_FMT, CONTACT_FIELDS),               # v8 (#104)
    "LidarConfigFrame": (LIDAR_CONFIG_FMT, LIDAR_CONFIG_FIELDS),  # v10 (#113); mask_cdeg is an array: encode_lidar_config
}

_S = {name: struct.Struct(fmt) for name, (fmt, _f) in STRUCTS.items()}
TELEMETRY_SIZE = _S["TelemetryFrame"].size      # 108
RANGE_SIZE = _S["RangeFrame"].size              # 28
PARAM_INFO_SIZE = _S["ParamInfoFrame"].size     # 38
HELLO_SIZE = _S["HelloFrame"].size              # 48 (v10)
NAV_STATUS_SIZE = _S["NavStatusFrame"].size     # 20
LIDAR_OBST_SIZE = _S["LidarObstacleFrame"].size  # 148
CONTACT_SIZE = _S["ContactFrame"].size          # 20
LIDAR_CONFIG_SIZE = _S["LidarConfigFrame"].size  # 76


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


def crc16_py(data, crc=0xFFFF):
    tbl = _CRC_TABLE
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ b) & 0xFF]
    return crc


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE. binascii.crc_hqx is the same CRC (poly 0x1021, not reflected, given start value) in C: with
    ~100 MSG_LIDAR_RAW frames a second (#100) the Python loop above would cost the Pi a few % of a core."""
    return binascii.crc_hqx(data, crc)


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
        # (#110) walks the buffer with an index and cuts it once at the end: a `del buf[:n]` per frame moves the whole
        # backlog every time (after a stall: a few hundred frames x kilobytes)
        buf = self.buf
        buf += data
        out = []
        sync = b"\xA5\x5A"
        pos, end = 0, len(buf)
        while True:
            i = buf.find(sync, pos)
            if i < 0:
                # keep a trailing 0xA5 (could be the first half of a sync)
                keep = 1 if end > pos and buf[end - 1] == LINK_SYNC1 else 0
                self.bytes_dropped += end - pos - keep
                pos = end - keep
                break
            if i > pos:
                self.bytes_dropped += i - pos
                pos = i
            if end - pos < 4:
                break
            n = buf[pos + 3]
            if n > LINK_MAX_PAYLOAD:
                self.crc_errors += 1
                self.bytes_dropped += 1
                pos += 1
                continue
            total = 4 + n + 2
            if end - pos < total:
                break
            body = bytes(buf[pos + 2:pos + 4 + n])
            rx_crc = buf[pos + 4 + n] | (buf[pos + 5 + n] << 8)
            if crc16(body) != rx_crc:
                self.crc_errors += 1
                self.bytes_dropped += 1
                pos += 1
                continue
            out.append((body[0], body[2:]))
            self.frames_ok += 1
            pos += total
        if pos:
            del buf[:pos]
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
    d = _unpack("HelloFrame", _pad(p, (HELLO_V9_SIZE, HELLO_V8_SIZE), HELLO_SIZE))   # v9: caps 0; v8: no reset fields
    if len(p) < HELLO_V9_SIZE:
        d["off_s"] = 0xFFFFFFFF
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


def decode_lidar_raw(p):
    """MSG_LIDAR_RAW -> {"seq", "flags", "data": bytes}. ValueError without data or with more than LINK_LIDAR_RAW_MAX."""
    if not 3 <= len(p) <= 2 + LINK_LIDAR_RAW_MAX:
        raise ValueError("LidarRawFrame: payload %d bytes, expected 3 .. %d" % (len(p), 2 + LINK_LIDAR_RAW_MAX))
    return {"seq": p[0], "flags": p[1], "data": bytes(p[2:])}


def decode_lidar_config(p):
    """MSG_LIDAR_CONFIG (v10, #113) -> the LidarConfigFrame fields, "mask": [[from, to] deg] (mask_n of them)."""
    if len(p) != LIDAR_CONFIG_SIZE:
        raise ValueError("LidarConfigFrame: payload %d != %d bytes" % (len(p), LIDAR_CONFIG_SIZE))
    v = _S["LidarConfigFrame"].unpack_from(p)
    d = dict(zip(LIDAR_CONFIG_FIELDS[:-1], v[:13]))
    m = v[13:]
    d["mask"] = [[m[2 * i] / 100.0, m[2 * i + 1] / 100.0] for i in range(min(d["mask_n"], LINK_LIDAR_MASK_MAX))]
    return d


def lidar_config_payload(s):
    """lidar.json settings (station/bridge/lidar.py DEFAULTS keys) -> the LidarConfigFrame payload (#113)."""
    flags = ((LINK_LCFG_ENABLED if s["enabled"] else 0) | (LINK_LCFG_FLIP if s["flip"] else 0)
             | (LINK_LCFG_PITCH_AUTO if s["pitch_auto_ref"] else 0) | (LINK_LCFG_FLOOR_CLIP if s["floor_clip"] else 0)
             | (LINK_LCFG_DESKEW if s["deskew"] else 0))
    mask = list(s.get("mask") or [])[:LINK_LIDAR_MASK_MAX]
    cd = []
    for a, b in mask:
        cd += (int(round((float(a) % 360.0) * 100)) % 36000, int(round((float(b) % 360.0) * 100)) % 36000)
    cd += [0] * (2 * LINK_LIDAR_MASK_MAX - len(cd))
    gate = max(float(s["pitch_limit_deg"]), float(s["pitch_max_deg"]))   # the navigator's gate (lidar.Lidar.deskew)
    return _S["LidarConfigFrame"].pack(
        flags, len(mask), 0, float(s["x_mm"]), float(s["y_mm"]), float(s["yaw_deg"]), float(s["height_mm"]),
        float(s["min_range_mm"]), float(s["max_range_mm"]), float(s["min_intensity"]), gate,
        float(s["pitch_ref_deg"]), float(s["time_offset_ms"]), *cd)


def encode_lidar_config(s):
    """One MSG_LIDAR_CONFIG frame from the lidar settings (#113)."""
    return encode_frame(MSG_LIDAR_CONFIG, lidar_config_payload(s))


def decode_contact(p):
    """MSG_CONTACT (v8, #104) -> {"kind", "seq", "mode", "flags", "x_mm", "y_mm", "heading_deg", "value"} + "name"
    (bump / stuck). Pose frame, like the telemetry."""
    d = _unpack("ContactFrame", p)
    d["name"] = CONTACT_KIND_NAMES.get(d["kind"], "contact")
    return d


DECODERS = {
    MSG_TELEMETRY: decode_telemetry, MSG_RANGE: decode_range, MSG_LOG: decode_log,
    MSG_PARAM_INFO: decode_param_info, MSG_NAV_STATUS: decode_nav_status, MSG_HELLO: decode_hello,
    MSG_HEARTBEAT: decode_heartbeat, MSG_CMD_VEL: decode_cmd_vel, MSG_SET_MODE: decode_set_mode,
    MSG_SET_PARAM: decode_set_param, MSG_GET_PARAMS: lambda p: {}, MSG_GOTO: decode_goto,
    MSG_EVENT: decode_event, MSG_LIDAR_OBSTACLES: decode_lidar_obstacles, MSG_LIDAR_RAW: decode_lidar_raw,
    MSG_CONTACT: decode_contact, MSG_LIDAR_CONFIG: decode_lidar_config,
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
                 param_count=0, firmware="", reset_cause=0, reset_flags=0, srsr=0, prev_uptime_s=0,
                 off_s=0xFFFFFFFF, uptime_ms=0, caps=0):
    return encode_frame(MSG_HELLO, _pack("HelloFrame", locals()))


def encode_telemetry(d):
    d = dict(d)
    for k in ("remote_flags", "nav_state", "nav_index", "fault_flags"):   # v3 fields (+ v9 fault_flags)
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


def encode_contact(d):
    """MSG_CONTACT (v8, #104): kind, seq, mode, flags, x_mm, y_mm, heading_deg, value."""
    d = dict(d)
    for k in ("seq", "mode", "flags"):
        d.setdefault(k, 0)
    d.setdefault("value", 0.0)
    d["seq"] &= 0xFF
    return encode_frame(MSG_CONTACT, _pack("ContactFrame", d))


def encode_lidar_raw(seq, data, flags=0):
    """One MSG_LIDAR_RAW frame: 1 .. LINK_LIDAR_RAW_MAX raw LD19 bytes."""
    if not 1 <= len(data) <= LINK_LIDAR_RAW_MAX:
        raise ValueError("MSG_LIDAR_RAW carries 1 .. %d bytes, got %d" % (LINK_LIDAR_RAW_MAX, len(data)))
    return encode_frame(MSG_LIDAR_RAW, bytes((seq & 0xFF, flags & 0xFF)) + bytes(data))


class LidarRawPacker:
    """The firmware's chunking (src/lidar_fwd.cpp) for the simulator and tests: bytes in, MSG_LIDAR_RAW frames out.
    A chunk goes when LINK_LIDAR_RAW_MAX bytes are waiting, or flush_s after the first of fewer arrived."""

    def __init__(self, flush_s=0.012):
        self.buf = bytearray()
        self.seq = 0
        self.first = None
        self.flush_s = flush_s
        self.frames = 0

    def feed(self, data, now):
        if data:
            if not self.buf:
                self.first = now
            self.buf += data
        out = []
        while self.buf and (len(self.buf) >= LINK_LIDAR_RAW_MAX or now - self.first >= self.flush_s):
            chunk = bytes(self.buf[:LINK_LIDAR_RAW_MAX])
            del self.buf[:LINK_LIDAR_RAW_MAX]
            out.append(encode_lidar_raw(self.seq, chunk))
            self.seq = (self.seq + 1) & 0xFF
            self.frames += 1
            self.first = now
        return out
