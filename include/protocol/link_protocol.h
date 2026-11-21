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
#define LINK_PROTOCOL_VERSION 2
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
  MSG_NAV_STATUS = 0x05,  // NavStatusFrame, on change and at 2 Hz while navigating
  MSG_HELLO = 0x06,       // HelloFrame, at boot and in reply to MSG_GET_PARAMS
  // Pi -> Teensy
  MSG_HEARTBEAT = 0x80,   // HeartbeatFrame, 10 Hz
  MSG_CMD_VEL = 0x81,     // CmdVelFrame, used in MODE_STATION (and by Pi-side autonomy)
  MSG_SET_MODE = 0x82,    // SetModeFrame
  MSG_SET_PARAM = 0x83,   // SetParamFrame
  MSG_GET_PARAMS = 0x84,  // empty payload
  MSG_GOTO = 0x85,        // GotoFrame: add/replace waypoints
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
};  // 104 bytes

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(TelemetryFrame) == 104, "TelemetryFrame size changed - bump LINK_PROTOCOL_VERSION");
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

struct __attribute__((packed)) NavStatusFrame {
  uint8_t state;          // robot-defined (idle, turning, driving, blocked, arrived, ...)
  uint8_t waypoint_count;
  uint8_t nav_index;
  uint8_t reserved;
  float target_x_mm, target_y_mm;
  float distance_mm;      // to the current target
};

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
