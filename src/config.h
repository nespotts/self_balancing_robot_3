// Pins and fixed constants. Tunable values live in params.h (settable over the Pi link).
#pragma once
#include <stdint.h>

#define FIRMWARE_ID "v4 " __DATE__ " " __TIME__  // 23 chars, fits HelloFrame.firmware[24]

namespace pins {
constexpr uint8_t IMU_CS = 10, IMU_INT = 30, IMU_RST = 33;      // BNO08x on SPI
constexpr uint8_t ENC_L_DIR = 38, ENC_R_DIR = 39;               // AS5600: left on Wire, right on Wire1
constexpr uint8_t MOT_L_EN = 4, MOT_L_D1 = 5, MOT_L_D2 = 6;     // H-bridge
constexpr uint8_t MOT_R_EN = 7, MOT_R_D1 = 8, MOT_R_D2 = 9;
constexpr uint8_t RADIO_CE = 32, RADIO_CSN = 36;                // nRF24 on SPI
constexpr uint8_t BATT_TAP1 = 14, BATT_TAP2 = 40, BATT_TAP3 = 41;
constexpr uint8_t REBOOT_BTN = 31;
// Lidar sweep (issue #7): servo 2, XSHUT 34/28, INT 35/29 - set in Lidar/lidar_scan.cpp
}  // namespace pins

// IMU: gyro-integrated rotation vector (no magnetometer), one report = quaternion + gyro.
// Report rate = balance loop rate = param ctl.rate_hz (default 400 Hz, same as v3).
constexpr uint32_t IMU_SPI_HZ = 1000000;
constexpr uint32_t IMU_STALE_US = 25000;      // no sample this long -> motors off

// Encoders / odometry
constexpr uint32_t I2C_CLOCK_HZ = 400000;     // AS5600 + VL53L0X both support 400 kHz
constexpr uint32_t ENCODER_PERIOD_US = 4000;  // read encoders after a balance step once 4 ms passed (200 Hz at 400 Hz)
constexpr uint8_t ENCODER_WINDOW = 16;        // speed = position change over 16 reads (80 ms)
constexpr float WHEEL_DIAMETER_MM = 66.0f;
constexpr float TRACK_MM = 123.0f;            // wheel separation

// Control
constexpr float VEL_LOOP_DT = 0.01f;          // velocity loop 100 Hz
constexpr float MOTOR_FULL = 4095.0f;         // 12-bit PWM

// Link / telemetry
constexpr uint32_t TELEMETRY_PERIOD_US = 10000;  // 100 Hz TelemetryFrame
constexpr uint8_t WAYPOINT_MAX = 32;

// TelemetryPacket.error_code / TelemetryFrame.error_code (first active one, in this order)
enum ErrorCode : uint8_t {
  ERR_OK = 0,
  ERR_IMU_INIT = 1,      // BNO08x did not answer at boot (retried every 5 s while motors off)
  ERR_IMU_STALE = 2,     // no IMU sample for IMU_STALE_US -> motors off
  ERR_ENC_LEFT = 3,      // left AS5600 not answering -> velocity loop off
  ERR_ENC_RIGHT = 4,     // right AS5600 not answering -> velocity loop off
  ERR_RADIO_CHIP = 5,    // nRF24 not found / lost its config
  ERR_RADIO_VERSION = 6, // packets with the wrong RADIO_PROTOCOL_VERSION in the last second
};
