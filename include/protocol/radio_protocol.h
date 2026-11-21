// Radio protocol v2: remote (Arduino Nano, AVR) <-> robot (Teensy 4.1, ARM) over nRF24L01.
//
// KEEP THIS FILE IDENTICAL in both repos:
//   self_balancing_robot_3/include/protocol/radio_protocol.h
//   Self_Balancing_Remote/include/radio_protocol.h
// Bump RADIO_PROTOCOL_VERSION on any change to a struct or enum value.
//
// Both MCUs are little-endian. Every struct is packed so the size is the same on 8-bit and 32-bit.
//
// Flow: the remote writes a ControlPacket to the robot RADIO_CONTROL_HZ times a second.
// The robot answers each one with a TelemetryPacket as the nRF24 ACK PAYLOAD
// (enableDynamicPayloads + enableAckPayload on both sides), so the remote never has to
// switch to listening and wait. The telemetry in an ack is the one the robot loaded after
// the previous packet, so it is one control period old. That's fine.
#pragma once
#include <stdint.h>

#define RADIO_PROTOCOL_VERSION 2
#define RADIO_CHANNEL 76
#define RADIO_DATA_RATE RF24_1MBPS
// 5-byte pipe address the robot listens on (the remote writes to it).
#define RADIO_ROBOT_ADDR "BBOT2"

#define RADIO_CONTROL_HZ 50
// Robot: no valid ControlPacket for this long -> ramp commanded speed/turn to zero, keep balancing.
#define RADIO_FAILSAFE_MS 250
// Robot/remote: no packet (or no ack) for this long -> report the link as lost.
#define RADIO_LOST_MS 1000

// Operating modes. The remote asks for one (requested_mode); the robot decides and reports
// the mode it is actually in (TelemetryPacket.mode). The remote LCD shows names from its own table.
enum RobotMode : uint8_t {
  MODE_IDLE = 0,      // motors off (safe to pick up)
  MODE_BALANCE = 1,   // balance in place, ignore drive commands
  MODE_MANUAL = 2,    // drive with the remote sticks
  MODE_STATION = 3,   // driven by the ground station (PC / phone / Kindle) through the Orange Pi
  MODE_GOTO = 4,      // drive through the waypoint list, stop for obstacles
  MODE_EXPLORE = 5,   // wander, avoid obstacles, build the map
  MODE_HOLD = 6,      // hold the current position (push it and it comes back)
  MODE_COUNT
};

// One-shot commands. The remote sends the event with a new event_seq and keeps repeating the
// same (event, event_seq) for ~200 ms so a dropped packet can't lose it. The robot acts once per
// new event_seq.
enum RadioEvent : uint8_t {
  EVT_NONE = 0,
  EVT_MARK_WAYPOINT = 1,   // append current pose to the waypoint list
  EVT_CLEAR_WAYPOINTS = 2,
  EVT_GO_HOME = 3,         // drive back to the first waypoint / origin
  EVT_RESET_POSE = 4,      // current position becomes (0,0), heading 0
  EVT_START_SCAN = 5,      // stop and do a full range sweep
  EVT_COUNT
};

// ControlPacket.flags
#define CTRL_FLAG_ARMED 0x01   // motors may run. Cleared = robot goes to MODE_IDLE behaviour.
#define CTRL_FLAG_ESTOP 0x02   // emergency stop: motors off immediately, overrides everything.

// ControlPacket.switches: 2 bits each, 0/1/2 (raw switch positions, for display/debug only;
// the robot must NOT give switch positions hidden meanings - modes and events carry intent).
#define CTRL_SW_A(s) ((s) & 0x03)
#define CTRL_SW_B(s) (((s) >> 2) & 0x03)
#define CTRL_SW_C(s) (((s) >> 4) & 0x03)
#define CTRL_SW_D(s) (((s) >> 6) & 0x03)

struct __attribute__((packed)) ControlPacket {
  uint8_t version;         // RADIO_PROTOCOL_VERSION
  uint8_t seq;             // increments every packet
  uint8_t requested_mode;  // RobotMode
  uint8_t flags;           // CTRL_FLAG_*
  int16_t drive;           // -1000..1000, + = forward (trim + deadband already applied by the remote)
  int16_t turn;            // -1000..1000, + = turn left (counter-clockwise seen from above)
  int16_t aux_x;           // other stick, -1000..1000
  int16_t aux_y;           // other stick, -1000..1000
  uint8_t knob_left;       // 0..255
  uint8_t knob_right;      // 0..255
  uint8_t speed_limit;     // 0..100 percent of the robot's max speed
  uint8_t switches;        // CTRL_SW_*
  uint8_t event;           // RadioEvent
  uint8_t event_seq;       // new value = new event
};  // 18 bytes

// TelemetryPacket.status
#define TLM_ARMED 0x01
#define TLM_UPRIGHT 0x02      // balancing (not fallen over)
#define TLM_FAILSAFE 0x04     // radio failsafe active
#define TLM_PI_LINK 0x08      // Orange Pi heartbeat is live
#define TLM_OBSTACLE 0x10     // obstacle blocking the current motion
#define TLM_LOW_BATT 0x20
#define TLM_NAV_ACTIVE 0x40   // following waypoints / exploring
#define TLM_NAV_DONE 0x80     // reached the last waypoint

struct __attribute__((packed)) TelemetryPacket {
  uint8_t version;          // RADIO_PROTOCOL_VERSION
  uint8_t ack_seq;          // seq of the last ControlPacket the robot received
  uint8_t mode;             // RobotMode the robot is actually in
  uint8_t status;           // TLM_*
  uint16_t battery_mv;      // pack voltage
  uint16_t min_cell_mv;     // lowest cell
  int16_t pitch_cdeg;       // pitch, 0.01 deg
  int16_t heading_ddeg;     // heading -1800..1800, 0.1 deg, 0 = +X, + = counter-clockwise
  int16_t x_cm;             // position from the pose origin
  int16_t y_cm;
  int16_t speed_mms;        // forward speed
  uint16_t range_front_mm;  // latest range ahead (0 = no reading)
  uint16_t range_rear_mm;
  uint8_t waypoint_count;
  uint8_t nav_index;        // waypoint currently being driven to
  uint8_t error_code;       // 0 = ok; robot-defined codes (IMU fail, encoder fail, ...)
  uint8_t loop_khz_x10;     // main loop rate / 100 Hz, for health display
};  // 26 bytes (nRF24 max payload is 32)

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(ControlPacket) == 18, "ControlPacket size changed - bump RADIO_PROTOCOL_VERSION");
static_assert(sizeof(TelemetryPacket) == 26, "TelemetryPacket size changed - bump RADIO_PROTOCOL_VERSION");
#endif
