// Link protocol v1: Teensy 4.1 <-> Orange Pi over the Teensy's USB serial (the same cable the Pi
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
#define LINK_PROTOCOL_VERSION 3
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
  MSG_SET_MODE = 0x82,    // SetModeFrame. No remote present: a mode != IDLE also ARMS, IDLE disarms.
  MSG_SET_PARAM = 0x83,   // SetParamFrame. Echoed with a ParamInfoFrame holding the value actually
                          // applied (clamped to min..max). cfg.* params are actions, not settings:
                          // cfg.save = 1 -> write EEPROM (waits until the motors are off),
                          // cfg.defaults = 1 -> all defaults (not saved, every ParamInfo re-sent);
                          // both read back 0. mot.test = 1..4 starts the wheels-up motor test.
  MSG_GET_PARAMS = 0x84,  // empty payload; reply: HelloFrame, then one ParamInfoFrame per param
  MSG_GOTO = 0x85,        // GotoFrame: add/replace waypoints. GOTO and EXPLORE follow the list;
                          // a new list while BLOCKED resumes.
  MSG_EVENT = 0x86,       // EventFrame: same events as RadioEvent
};

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
  uint8_t clients;        // ground-station clients connected to the bridge
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
