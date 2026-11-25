// Pins and fixed constants. Tunable values live in params.h (settable over the Pi link).
#pragma once
#include <stdint.h>

#define FIRMWARE_ID "v4 " __DATE__ " " __TIME__  // 23 chars, fits HelloFrame.firmware[24]

// ---- compile-time switches for bisecting the v9 crash on the robot (issue #109 follow-up) ----
// Defaults = the fix. platformio.ini has one env per variant (-D flags); the boot log prints them ("build: ...").
#ifndef SBR_WDOG_ENABLE
#define SBR_WDOG_ENABLE 1      // RTWDOG hardware watchdog (src/watchdog.cpp). 0 = never touched
#endif
#ifndef SBR_WDT_FEED_EVERY_PASS
#define SBR_WDT_FEED_EVERY_PASS 0  // 1 = v9: wdt_can_block() refreshed the RTWDOG on every background pass (~100 kHz)
#endif
#ifndef SBR_SNVS_MARKER
#define SBR_SNVS_MARKER 0      // 1 = v9: SNVS_LPGPR2/3 rewritten every second (each SNVS access stalls the CPU ~90 us)
#endif
#ifndef SBR_I2C_BOUNDED
#define SBR_I2C_BOUNDED 1      // 1 = bounded LPI2C reads + bit-bang recovery; 0 = plain Wire reads (v8 style), no recovery
#endif
#ifndef SBR_EARLY_HOOK
#define SBR_EARLY_HOOK 1       // startup_early_hook: motor pins low a few us after reset (src/motors.cpp)
#endif
#ifndef SBR_CRASH_CAPTURE
#define SBR_CRASH_CAPTURE 1    // own fault handlers + last-phase record in uncached RAM (src/crash_capture.h)
#endif

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
// I2C clocks (compile time, issue #109). Supported: 100000, 200000, 400000 (src/i2c_bounded.cpp). The left bus (Wire,
// D18 / D19, shared with the VL53L0X sweep) has dropped out twice with the motors running: 100 kHz gives it the LPI2C's
// widest glitch filter (~0.4 us) and 4x the rise-time margin. One AS5600 angle read (5 bytes on the wire) takes
// ~0.12 ms at 400 kHz, ~0.25 ms at 200 kHz, ~0.5 ms at 100 kHz; the "enc:" log line reports the measured mean / max.
constexpr uint32_t I2C_LEFT_HZ = 100000;      // Wire  (left AS5600 + range sweep)
constexpr uint32_t I2C_RIGHT_HZ = 400000;     // Wire1 (right AS5600): never failed
constexpr uint32_t I2C_CLOCK_HZ = I2C_LEFT_HZ;  // Wire users other than the encoders (range sweep)
static_assert(I2C_LEFT_HZ == 100000 || I2C_LEFT_HZ == 200000 || I2C_LEFT_HZ == 400000, "I2C_LEFT_HZ: 100k / 200k / 400k");
static_assert(I2C_RIGHT_HZ == 100000 || I2C_RIGHT_HZ == 200000 || I2C_RIGHT_HZ == 400000, "I2C_RIGHT_HZ: 100k / 200k / 400k");
constexpr uint32_t ENCODER_PERIOD_US = 4000;  // read encoders after a balance step once 4 ms passed (200 Hz at 400 Hz)
constexpr uint8_t ENCODER_WINDOW = 16;        // speed = position change over 16 reads (80 ms)
// Encoder health (src/wheel_encoder.h, issue #109)
constexpr uint8_t ENC_FAULT_BAD = 10;         // this many bad reads in a row (errors / rejected jumps) -> fault latched
constexpr uint32_t ENC_STALE_US = 50000;      // no good read for this long -> fault latched (and a resync on the next)
constexpr uint32_t ENC_CLEAR_MS = 2000;       // a latched fault clears after this long with only good reads
constexpr uint8_t ENC_RECOVER_AFTER = 3;      // NACKs in a row before a bus recovery (busy / timeout / pin low: at once)
constexpr uint32_t ENC_BACKOFF_MIN_MS = 20;   // after a recovery: next read attempt this much later, doubling ...
constexpr uint32_t ENC_BACKOFF_MAX_MS = 1000; // ... up to this, while the reads keep failing
constexpr float ENC_MAX_MMS = 3000.0f;        // a read implying a faster wheel is a corrupted read (TT top speed ~2500)
constexpr float ENC_JUMP_MMS = 1500.0f;       // ... or a speed change this big from one read to the next (375 m/s^2)
constexpr uint8_t ENC_RESYNC_REJECTS = 3;     // rejected in a row: take the new angle as the reference (no distance)
constexpr uint8_t AS5600_ADDR = 0x36, AS5600_REG_ANGLE = 0x0E;
// I2C pins (Teensy 4.1 Wire / Wire1 defaults), for the bus recovery's bit-banged clocks
constexpr uint8_t I2C_LEFT_SDA = 18, I2C_LEFT_SCL = 19, I2C_RIGHT_SDA = 17, I2C_RIGHT_SCL = 16;

// Hardware watchdog (src/watchdog.h, issue #109): RTWDOG, fed once per loop pass while the loop is healthy.
constexpr uint32_t WDT_RUN_MS = 100;          // timeout while the motors run (balance / motor test)
constexpr uint32_t WDT_IDLE_MS = 2000;        // timeout with the motors off (EEPROM saves, sensor re-inits block)
constexpr uint32_t WDT_BG_MAX_US = 50000;     // no background pass (link poll, radio, failsafes) this long = unhealthy
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
  ERR_ENC_LEFT = 3,      // left AS5600 fault latched -> speed from the right wheel + gyro; no arming / drive modes
  ERR_ENC_RIGHT = 4,     // right AS5600 fault latched -> speed from the left wheel + gyro; no arming / drive modes
  ERR_RADIO_CHIP = 5,    // nRF24 not found / lost its config
  ERR_RADIO_VERSION = 6, // packets with the wrong RADIO_PROTOCOL_VERSION in the last second
};
