// Link protocol (version below): Teensy 4.1 <-> Orange Pi over the Teensy's USB serial (the same cable the Pi
// uses to flash the Teensy). The Pi bridge (station/) mirrors these definitions in Python;
// keep station/bridge/link_protocol.py in sync with this file.
//
// Frame:  0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 u16 (little-endian)
//   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
// All multi-byte fields are little-endian; structs are packed. Floats are IEEE-754 32-bit.
//
// Units: mm, mm/s, degrees, deg/s, seconds as micros (uint32, wraps). Pose frame: origin where the
// pose was last reset, +X = forward at reset, heading + = counter-clockwise (same as radio_protocol.h).
#pragma once
#include <stdint.h>

// v2: TelemetryFrame gained bal_p .. step at the end (balance tuning data).
// v3: TelemetryFrame gained remote_flags, nav_state, nav_index, reserved2 (108 bytes);
//     NavStatusFrame gained obstacle_mm (20 bytes). Older frames are shorter: receivers pad with 0.
// v4: no layout change, new rules (safety review #38 / #41): link-only events EVT_LINK_ESTOP /
//     EVT_LINK_ESTOP_RELEASE (latching station E-stop) and EVT_LINK_ARM (SET_MODE alone no longer
//     arms), remote_flags bits LINK_RF_LINK_ESTOP / LINK_RF_NAV_HANDOVER, HeartbeatFrame.clients
//     counts (0 = nobody watching: the Pi does not supervise). A v3 bridge can't arm a v4 robot
//     without the remote (fail-safe); a v4 bridge works with v3 firmware (the events are ignored).
// v5: MSG_LIDAR_OBSTACLES (Pi -> Teensy, issue #77): LD19 obstacles for the navigator. v4 firmware ignores it
//     (unknown type) and keeps using the range sweep; with a v4 bridge the v5 firmware uses the sweep as before.
// v6: link-only event EVT_LINK_STUCK (issue #79): the bridge's LD19 scan matcher saw the wheels turn but the robot
//     not move (wheel slip, a low obstacle). v5 firmware ignores it (only its own odometry stuck check); a v5 bridge
//     never sends it. No layout change.
// v7: MSG_LIDAR_RAW (Teensy -> Pi, issue #100): the LD19 is wired to the Teensy's Serial7 (the Pi's H618 UART can't make
//     230400 baud) and its raw bytes are forwarded in chunks. A v6 bridge ignores the message (unknown type: no lidar,
//     as before); v6 firmware never sends it (the bridge then reads the LD19 from a serial port, if it has one).
// v8: MSG_CONTACT (Teensy -> Pi, issue #104): the navigator felt a contact the lidar can't see (a bump, or pushing but
//     not moving) and tells where. A v7 bridge ignores it (unknown type: no bump marks, the "nav: contact" log line still
//     shows); v7 firmware never sends it. No other change.
// v9 (#109, safety): HelloFrame gained the reset cause (reset_cause .. uptime_ms, 44 bytes; a v8 bridge reads the first
//     28 bytes as before, a v9 bridge pads a v8 HELLO with 0 = unknown). TelemetryFrame.reserved2 is now fault_flags
//     (LINK_FAULT_* bits, same layout: a v8 bridge ignores it). New rules: a latched encoder fault refuses arming and the
//     GOTO / EXPLORE / HOLD / STATION modes (SET_MODE logs "ignored: encoder fault").
// v10 (#113): the Teensy computes the LD19 obstacle sectors itself. HelloFrame gained caps (48 bytes; a v9 bridge reads
//     the first 44 as before, a v10 bridge pads a v9 HELLO with caps 0). With LINK_CAP_LIDAR_SECTORS the bridge sends the
//     robot profile's lidar mount settings (MSG_LIDAR_CONFIG, Pi -> Teensy) instead of MSG_LIDAR_OBSTACLES frames. A v9
//     bridge never sends MSG_LIDAR_CONFIG: the v10 firmware then takes its MSG_LIDAR_OBSTACLES as before. v9 firmware has
//     no caps: a v10 bridge keeps sending it MSG_LIDAR_OBSTACLES.
#define LINK_PROTOCOL_VERSION 10
#define LINK_SYNC1 0xA5
#define LINK_SYNC2 0x5A
#define LINK_MAX_PAYLOAD 200
// Teensy: no MSG_HEARTBEAT from the Pi for this long -> Pi link lost; MODE_STATION commands stop.
#define LINK_HEARTBEAT_TIMEOUT_MS 500
// Teensy: MSG_CMD_VEL older than this is treated as zero.
#define LINK_CMD_VEL_TIMEOUT_MS 300

enum LinkMsgType : uint8_t {
  // Teensy -> Pi
  MSG_TELEMETRY = 0x01,   // TelemetryFrame, param tlm.hz (default 100; up to one per balance step)
  MSG_RANGE = 0x02,       // RangeFrame, one per range reading, tagged with the pose at that instant
  MSG_LOG = 0x03,         // UTF-8 text, no terminator (len = text length)
  MSG_PARAM_INFO = 0x04,  // ParamInfoFrame, sent for every param in reply to MSG_GET_PARAMS
  MSG_NAV_STATUS = 0x05,  // NavStatusFrame, at once on a change (state, index, count) + 2 Hz in GOTO/EXPLORE/HOLD
  MSG_HELLO = 0x06,       // HelloFrame, at boot and in reply to MSG_GET_PARAMS
  MSG_LIDAR_RAW = 0x07,   // LidarRawFrame (v7): raw LD19 bytes from Serial7, ~98 frames/s while the lidar runs
  MSG_CONTACT = 0x08,     // ContactFrame (v8): one per contact the navigator detected (bump / stuck, #104)
  // Pi -> Teensy
  MSG_HEARTBEAT = 0x80,   // HeartbeatFrame, 10 Hz
  MSG_CMD_VEL = 0x81,     // CmdVelFrame, used in MODE_STATION (and by Pi-side autonomy)
  MSG_SET_MODE = 0x82,    // SetModeFrame. No remote present: IDLE disarms; a mode != IDLE arms only
                          // right after EVT_LINK_ARM (else it just changes an armed robot's mode).
                          // Ignored while the link E-stop is latched.
  MSG_SET_PARAM = 0x83,   // SetParamFrame. Echoed with a ParamInfoFrame holding the value actually
                          // applied (clamped to min..max). cfg.* params are actions, not settings:
                          // cfg.save = 1 -> write EEPROM (waits until the motors are off),
                          // cfg.defaults = 1 -> all defaults (not saved, every ParamInfo re-sent;
                          // also waits until the motors are off); both read back 0.
                          // mot.test = 1..4 starts the wheels-up motor test.
  MSG_GET_PARAMS = 0x84,  // empty payload; reply: HelloFrame, then one ParamInfoFrame per param
  MSG_GOTO = 0x85,        // GotoFrame: add/replace waypoints. GOTO and EXPLORE follow the list;
                          // a new list while BLOCKED resumes.
  MSG_EVENT = 0x86,       // EventFrame: same events as RadioEvent, plus the link-only LinkEvent values
  MSG_LIDAR_OBSTACLES = 0x87,  // LidarObstacleFrame (v5): one per LD19 scan that passed the bridge's checks, ~10 Hz
  MSG_LIDAR_CONFIG = 0x88,     // LidarConfigFrame (v10): the lidar mount settings for the Teensy's own sectors (#113)
};

// Link-only EventFrame.event values (never sent by the remote; RadioEvent values stay < EVT_COUNT).
enum LinkEvent : uint8_t {
  // Station E-stop: IDLE + disarm now, latched. While latched no mode request from any source
  // (remote selector, SET_MODE, GO_HOME) can start the motors.
  EVT_LINK_ESTOP = 0x80,
  // Clears the latch only; the robot stays IDLE and disarmed. Then the normal arming rules:
  // remote present -> its ARMED switch off -> on + a mode selection; no remote -> a SET_MODE != IDLE.
  EVT_LINK_ESTOP_RELEASE = 0x81,
  // Explicit arm request (no remote present): the next SET_MODE != IDLE within LINK_ARM_WINDOW_MS
  // arms. Without it a SET_MODE never arms (it only changes the mode of an armed robot), so a stray
  // or late mode request (another client, the app stopping in the background) can't start the motors.
  // Ignored while an E-stop (remote or link) is latched. The bridge sends it for {"arm": true} only.
  EVT_LINK_ARM = 0x82,
  // v6 (#79): lidar stuck. The bridge's scan matcher (station/bridge/lidarmatch.py) saw the wheels turn for
  // match_stuck_s while the matched pose stayed put (odometry progress is phantom). GOTO / EXPLORE, driving: the
  // navigator's next stuck check takes it (BLOCKED, "nav: stuck ...", back off) if it comes within 500 ms; else dropped.
  EVT_LINK_STUCK = 0x83,
};
#define LINK_ARM_WINDOW_MS 500

struct __attribute__((packed)) HelloFrame {
  uint8_t link_version;   // LINK_PROTOCOL_VERSION
  uint8_t radio_version;  // RADIO_PROTOCOL_VERSION
  uint16_t param_count;
  char firmware[24];      // build id, e.g. "v4 2026-11-20 abc1234", zero padded
  // v9 (#109): why the Teensy last reset, read once at boot (src/reset_cause.h); the same in every HELLO of one boot
  uint8_t reset_cause;    // LinkResetCause
  uint8_t reset_flags;    // LINK_RESETF_* bits
  uint16_t srsr;          // SRC_SRSR at boot (raw reset-source bits, cleared after reading)
  uint32_t prev_uptime_s; // how long the previous run lasted, s (0 = unknown)
  uint32_t off_s;         // s from the previous run's last second to this boot (0xFFFFFFFF = unknown)
  uint32_t uptime_ms;     // millis() when this HELLO was sent: a smaller value than before = the Teensy rebooted
  uint32_t caps;          // v10 (#113): LINK_CAP_* bits, what this firmware can do
};  // 48 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(HelloFrame) == 48, "HelloFrame size changed - bump LINK_PROTOCOL_VERSION");
#endif

// HelloFrame.caps (v10)
#define LINK_CAP_LIDAR_SECTORS 0x01  // computes the LD19 obstacle sectors itself: wants MSG_LIDAR_CONFIG, not the frames

// HelloFrame.reset_cause (v9). The i.MX RT1062 has no brownout flag: a power dip shows as a power-on reset; it is told
// from a real power cycle by the SNVS domain (its RTC and 4 general-purpose registers) surviving the dip.
enum LinkResetCause : uint8_t {
  LINK_RESET_UNKNOWN = 0,
  LINK_RESET_POWER_ON = 1,    // cold start: power was off (SNVS lost, or off for >= LINK_RESET_DIP_S)
  LINK_RESET_POWER_DIP = 2,   // power-on / reset-pin reset with SNVS kept and off < LINK_RESET_DIP_S: brownout likely
  LINK_RESET_WATCHDOG = 3,    // the hardware watchdog (RTWDOG) fired: the main loop hung or starved the link
  LINK_RESET_LOCKUP = 4,      // CPU lockup or a software reset the firmware did not ask for
  LINK_RESET_FAULT = 5,       // the Teensy core's fault handler rebooted after a crash (CrashReport saved)
  LINK_RESET_REQUESTED = 6,   // the firmware asked for it (reboot button)
  LINK_RESET_FLASHED = 7,     // a different firmware build than the previous run (flashed)
  LINK_RESET_TEMP = 8,        // chip over-temperature
  LINK_RESET_OTHER = 9,       // WDOG1/2, JTAG, security monitor
  LINK_RESET_CAUSE_COUNT
};
#define LINK_RESET_DIP_S 3
// HelloFrame.reset_flags (v9)
#define LINK_RESETF_CRASH 0x01    // a CrashReport (fault address, CFSR) was saved before this boot: logged at boot
#define LINK_RESETF_SNVS 0x02     // the SNVS domain kept its contents: prev_uptime_s / off_s are known
#define LINK_RESETF_WDT 0x04      // the hardware watchdog runs this boot

// Everything below is a snapshot of ONE balance step (t_us = that step's IMU sample time).
struct __attribute__((packed)) TelemetryFrame {
  uint32_t t_us;
  uint8_t mode;             // RobotMode
  uint8_t status;           // TLM_* bits from radio_protocol.h
  uint8_t error_code;
  uint8_t radio_ok;         // 1 = remote packets arriving
  float pitch_deg;          // + = leaning forward
  float pitch_rate_dps;
  float roll_deg;
  float heading_deg;        // continuous (does not wrap), + = CCW
  float yaw_rate_dps;
  float x_mm, y_mm;
  float speed_mms;          // measured forward speed
  float turn_rate_dps;      // measured
  float cmd_speed_mms;      // what the controller is trying to do
  float cmd_turn_dps;
  float wheel_left_mms, wheel_right_mms;
  float balance_target_deg; // lean angle the outer loop asked for
  float motor_left, motor_right;  // -1..1 normalised output
  float battery_v;
  float loop_hz;
  // v2 additions: balance PID contributions in motor counts (-4095..4095), out = p + i + d
  float bal_p, bal_i, bal_d;
  float steer_out;          // heading PID output, counts (left -= , right +=)
  float vel_out_deg;        // velocity loop output (lean added to the balance target)
  uint32_t step;            // balance step counter (gaps = steps not sent)
  // v3 additions
  uint8_t remote_flags;     // LINK_RF_* bits (the radio TLM_* status byte is full)
  uint8_t nav_state;        // NavState
  uint8_t nav_index;        // waypoint being driven to (= NavStatusFrame.nav_index)
  uint8_t fault_flags;      // v9 (was reserved2): LINK_FAULT_* bits
};  // 108 bytes

// TelemetryFrame.fault_flags (v9, #109). error_code shows only the first fault; these show every encoder fault.
#define LINK_FAULT_ENC_L 0x01      // left encoder fault latched (error_code 3): clears after 2 s of clean reads
#define LINK_FAULT_ENC_R 0x02      // right encoder fault latched (error_code 4)
#define LINK_FAULT_ENC_L_ERR 0x04  // left encoder I2C errors / rejected jumps / bus recoveries in the last second
#define LINK_FAULT_ENC_R_ERR 0x08  // right encoder, the same
#define LINK_FAULT_WDT_OFF 0x10    // the hardware watchdog is not running (failed to start)

// TelemetryFrame.remote_flags
#define LINK_RF_ESTOP 0x01     // E-stop latched (from the remote); clears with a packet without it
#define LINK_RF_ARM_WAIT 0x02  // arming needs the remote's ARMED switch off -> on (after boot / E-stop)
#define LINK_RF_PRESENT 0x04   // remote packets in the last RADIO_LOST_MS (radio_ok = last 250 ms)
#define LINK_RF_LINK_ESTOP 0x08  // station E-stop latched (EVT_LINK_ESTOP) until EVT_LINK_ESTOP_RELEASE
#define LINK_RF_NAV_HANDOVER 0x10  // GOTO/EXPLORE/HOLD paused: the remote was lost, needs the remote back
                                   // or a fresh SET_MODE from the station (issue #38)

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(TelemetryFrame) == 108, "TelemetryFrame size changed - bump LINK_PROTOCOL_VERSION");
#endif

struct __attribute__((packed)) RangeFrame {
  uint32_t t_us;
  float x_mm, y_mm, heading_deg;  // robot pose when the reading was taken
  float pitch_deg;                // so the receiver can drop readings taken while tilted
  float sensor_angle_deg;         // sensor direction relative to robot forward, + = CCW
  uint16_t range_mm;
  uint8_t sensor_id;
  uint8_t valid;                  // 0 = no target / out of range / sensor error
};

struct __attribute__((packed)) ParamInfoFrame {
  uint16_t id;
  float value, min, max;
  char name[24];   // e.g. "bal.kp", zero padded; the station builds its tuning sliders from these
};

// NavStatusFrame.state / TelemetryFrame.nav_state
enum NavState : uint8_t {
  NAV_IDLE = 0,      // not navigating (other mode, or GOTO/EXPLORE with an empty list)
  NAV_TURNING = 1,   // turning in place toward the target (or to nav.end_heading_deg at the end)
  NAV_DRIVING = 2,   // driving to the target
  NAV_BLOCKED = 3,   // stopped: obstacle closer than rng.obstacle_mm in the direction of travel
                     // (GOTO: resumes when clear; EXPLORE: waits for a new MSG_GOTO list)
  NAV_ARRIVED = 4,   // stopped at the last waypoint (TLM_NAV_DONE)
  NAV_HOLDING = 5,   // MODE_HOLD: holding the position it had when HOLD started
  NAV_STATE_COUNT
};

struct __attribute__((packed)) NavStatusFrame {
  uint8_t state;          // NavState
  uint8_t waypoint_count;
  uint8_t nav_index;      // waypoint being driven to (= count once the list is done)
  uint8_t reserved;
  float target_x_mm, target_y_mm;  // current target (HOLD: the hold point)
  float distance_mm;      // to the current target
  float obstacle_mm;      // v3: nearest obstacle in the direction of travel, as a sensor range (0 = none)
};  // 20 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(NavStatusFrame) == 20, "NavStatusFrame size changed - bump LINK_PROTOCOL_VERSION");
#endif

struct __attribute__((packed)) HeartbeatFrame {
  uint32_t pi_ms;
  uint8_t clients;        // ground-station clients connected to the bridge (browsers / app / pollers,
                          // not the planner). 0 = nobody watching: the Pi does not supervise.
};

struct __attribute__((packed)) CmdVelFrame {
  float speed_mms;        // + = forward
  float turn_dps;         // + = CCW
};

struct __attribute__((packed)) SetModeFrame {
  uint8_t mode;           // RobotMode
};

struct __attribute__((packed)) SetParamFrame {
  uint16_t id;
  float value;
};

struct __attribute__((packed)) GotoFrame {
  uint8_t replace;        // 1 = clear the list first, 0 = append
  uint8_t count;          // number of points that follow (max 16)
  struct __attribute__((packed)) {
    float x_mm, y_mm;
  } points[16];           // only `count` entries are sent (len = 2 + count * 8)
};

struct __attribute__((packed)) EventFrame {
  uint8_t event;          // RadioEvent
};

// MSG_LIDAR_OBSTACLES (v5, issue #77): one LD19 return in each of LINK_LIDAR_SECTORS sectors of one scan (the nearest
// inside the navigator's driving corridor, else the nearest: #99; the layout is unchanged).
// Robot frame: x forward, y left, mm from the axle centre. Sector i holds azimuths (i - 0.5) * 10 .. (i + 0.5) * 10 deg
// CCW from forward (about the axle); x = y = 0: no return in that sector (open, out of range, or masked).
// The bridge (station/bridge/lidar_obstacles.py) sends only whole scans that passed its pitch gate, with its range /
// intensity / mask / floor filters applied, deskewed and moved into the robot frame of its newest telemetry pose;
// age_ms = how old that pose is at sending. Firmware (navigator.h): frames older than LINK_LIDAR_MAX_AGE_MS are
// dropped; no frame for LINK_LIDAR_FRESH_MS = no lidar (range sweep, or blind).
#define LINK_LIDAR_SECTORS 36
#define LINK_LIDAR_FRESH_MS 500
#define LINK_LIDAR_MAX_AGE_MS 300
struct __attribute__((packed)) LidarObstacleFrame {
  uint8_t seq;            // + 1 per frame (wraps)
  uint8_t sectors;        // LINK_LIDAR_SECTORS
  uint16_t age_ms;
  int16_t xy_mm[72];      // x0, y0, x1, y1, ... (2 x LINK_LIDAR_SECTORS)
};  // 148 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(LidarObstacleFrame) == 148, "LidarObstacleFrame size changed - bump LINK_PROTOCOL_VERSION");
static_assert(4 + 4 * LINK_LIDAR_SECTORS == sizeof(LidarObstacleFrame), "LidarObstacleFrame: 2 x int16 per sector");
#endif

// MSG_LIDAR_CONFIG (v10, issue #113, Pi -> Teensy): the robot profile's LD19 settings (the bridge's lidar.json, the source
// of truth) for the firmware's own obstacle sectors (src/lidar_sectors.h), which then replace MSG_LIDAR_OBSTACLES. The
// bridge sends it to firmware whose HELLO has LINK_CAP_LIDAR_SECTORS once the robot profile is bound, again whenever the
// settings change, and every 2 s (a lost frame, a Teensy reboot). Until the first one the firmware computes nothing (the
// navigator then has no lidar: blind rules). Same meaning as lidar.json (station/bridge/lidar.py DEFAULTS):
//   flags LINK_LCFG_*; x / y / yaw / height: the mount; range window, intensity floor; pitch_gate_deg: packets leaning
//   more than this from the typical pitch (median of 3 s, or pitch_ref_deg without LINK_LCFG_PITCH_AUTO) or > 30 deg
//   absolute make the whole scan "tilted" (no frame) = max(pitch_limit_deg, pitch_max_deg); time_offset_ms: + = the lidar's
//   points are this much later than its clock says; mask: mask_n sectors [from, to] in 0.01 deg, robot frame CCW from
//   forward, a sector runs CCW from "from" to "to".
#define LINK_LCFG_ENABLED 0x01
#define LINK_LCFG_FLIP 0x02         // mounted upside down: the angles run counter-clockwise seen from above
#define LINK_LCFG_PITCH_AUTO 0x04   // typical pitch = median of the last 3 s (else pitch_ref_deg)
#define LINK_LCFG_FLOOR_CLIP 0x08   // drop points beyond 0.9 x height / tan(beam elevation)
#define LINK_LCFG_DESKEW 0x10       // a pose per packet (else the pose at the end of the scan)
#define LINK_LIDAR_MASK_MAX 8
struct __attribute__((packed)) LidarConfigFrame {
  uint8_t flags;          // LINK_LCFG_* bits
  uint8_t mask_n;         // 0 .. LINK_LIDAR_MASK_MAX
  uint16_t reserved;
  float x_mm, y_mm, yaw_deg, height_mm;
  float min_range_mm, max_range_mm, min_intensity;
  float pitch_gate_deg, pitch_ref_deg, time_offset_ms;
  uint16_t mask_cdeg[16]; // from0, to0, from1, to1, ... (2 x LINK_LIDAR_MASK_MAX)
};  // 76 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(LidarConfigFrame) == 76, "LidarConfigFrame size changed - bump LINK_PROTOCOL_VERSION");
#endif

// MSG_LIDAR_RAW (v7, issue #100): the LD19's own bytes (230400 8N1 on the Teensy's Serial7, D28 = RX7), unparsed, in the
// order they arrived. Chunks do not follow the LD19's 47-byte packets: the bridge (station/bridge/ld19.py) joins them and
// finds the packets (0x54 0x2C, CRC8) as it does on a serial port. The packets' own millisecond timestamps time the points,
// so the link's delivery jitter does not matter.
//   len = 2 + n, n = 1 .. LINK_LIDAR_RAW_MAX raw bytes (a frame is sent when 180 bytes are waiting, or 12 ms after the
//   first of fewer arrived).
//   seq: + 1 per chunk taken from Serial7, also for a chunk dropped because the USB link had no room: a gap = lost bytes.
//   flags LINK_LIDAR_RAW_OVERFLOW: Serial7's receive buffer (2 KB, ~120 ms of data) filled up or the UART overran since the
//   previous frame: bytes before this chunk are missing.
// The firmware sends it only while Serial7 runs (rng.enable 0 at boot: D28 / D29 are also the range sweep's sensor 1 pins).
#define LINK_LIDAR_RAW_MAX 180
#define LINK_LIDAR_RAW_OVERFLOW 0x01
struct __attribute__((packed)) LidarRawFrame {
  uint8_t seq;
  uint8_t flags;          // LINK_LIDAR_RAW_* bits
  uint8_t data[180];      // LINK_LIDAR_RAW_MAX; only n bytes are sent
};

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(2 + LINK_LIDAR_RAW_MAX <= LINK_MAX_PAYLOAD, "LidarRawFrame must fit one link frame");
#endif

// MSG_CONTACT (v8, issue #104): something below the LD19's scan plane (~208 mm) stopped the robot. The navigator
// (src/contact.h) saw either
//   LINK_CONTACT_BUMP   the wheels slowing far more than the command explains, with a pitch-rate spike, while driving;
//   LINK_CONTACT_STUCK  the speed held ~0 while the command, the velocity loop's lean and the motor effort pushed on.
// x, y: the contact point in the pose frame (on the body's front edge in the direction of travel), heading_deg: the
// direction of travel (heading + 180 when it was reversing). value: peak pitch rate (deg/s) for a bump, how long it was
// stalled (ms) for a stuck. flags LINK_CONTACT_*: what else confirmed it. The bridge marks a small "invisible obstacle"
// disc there that its planner treats as a wall (station/bridge/zones.py, kind "bump").
#define LINK_CONTACT_BUMP 1
#define LINK_CONTACT_STUCK 2
#define LINK_CONTACT_LIDAR 0x01     // flags: the bridge's lidar stuck (EVT_LINK_STUCK) agreed
#define LINK_CONTACT_FELL 0x02      // flags: the robot fell over right after (the fall finished the confirmation)
#define LINK_CONTACT_REPORT 0x04    // flags: STATION (someone drives): reported only, the motion was not changed
struct __attribute__((packed)) ContactFrame {
  uint8_t kind;           // LINK_CONTACT_BUMP / LINK_CONTACT_STUCK
  uint8_t seq;            // + 1 per contact (wraps)
  uint8_t mode;           // RobotMode it happened in
  uint8_t flags;          // LINK_CONTACT_LIDAR / _FELL / _REPORT
  float x_mm, y_mm;       // contact point, pose frame
  float heading_deg;      // direction of travel
  float value;            // bump: peak pitch rate deg/s; stuck: stalled ms
};  // 20 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(ContactFrame) == 20, "ContactFrame size changed - bump LINK_PROTOCOL_VERSION");
#endif
