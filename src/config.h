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
// Range sweep (issue #7): servo + 2x VL53L0X on Wire (shared with the left AS5600).
// Sensor 0 (0x30) faces forward with the head centred, sensor 1 (0x31) faces back.
constexpr uint8_t RNG_SERVO = 2;
constexpr uint8_t RNG_XSHUT0 = 34, RNG_INT0 = 35;
constexpr uint8_t RNG_XSHUT1 = 28, RNG_INT1 = 29;
// LD19 lidar (issue #100) on the PCB's "RPLidar C1" header: 5V, D28 = RX7 <- LD19 Tx, D29 = TX7 (unused, kept an input),
// GND. The same two pins as the sweep's sensor 1 above: Serial7 runs only while the sweep is off (rng.enable 0 at boot).
constexpr uint8_t LIDAR_RX = 28, LIDAR_TX = 29;
}  // namespace pins

// LD19 on Serial7: 230400 8N1, ~17.6 kB/s. Receive buffer = 64 (core) + this: ~120 ms of data, so blocking work in the
// loop (IMU / radio re-init, motors off) loses nothing. (cfg.save's flash write turns interrupts off for a while: the
// UART can overrun then; that is flagged, LINK_LIDAR_RAW_OVERFLOW.)
constexpr uint32_t LIDAR_BAUD = 230400;
constexpr uint32_t LIDAR_RX_EXTRA = 2048;

// IMU: gyro-integrated rotation vector (no magnetometer), one report = quaternion + gyro.
// Report rate = balance loop rate = param ctl.rate_hz (default 400 Hz, same as v3).
constexpr uint32_t IMU_SPI_HZ = 1000000;
constexpr uint32_t IMU_STALE_US = 25000;      // no sample this long -> motors off

// Encoders / odometry
constexpr uint32_t I2C_CLOCK_HZ = 400000;     // AS5600 + VL53L0X both support 400 kHz
constexpr uint32_t ENCODER_PERIOD_US = 4000;  // read encoders after a balance step once 4 ms passed (200 Hz at 400 Hz)
constexpr uint8_t ENCODER_WINDOW = 16;        // speed = position change over 16 reads (80 ms)
// wheel diameter / track width / left-right ratio are params now: odo.wheel_d_mm, odo.track_mm, odo.lr_ratio (issue #14)


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
