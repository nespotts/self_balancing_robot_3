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
#define LINK_PROTOCOL_VERSION 6
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
};

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
  uint8_t reserved2;
};  // 108 bytes

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

// MSG_LIDAR_OBSTACLES (v5, issue #77): the nearest LD19 return in each of LINK_LIDAR_SECTORS sectors of one scan.
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
