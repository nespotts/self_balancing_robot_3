// Result of one bounded I2C transaction (src/i2c_bounded.h). Pure: also used by the host tests.
#pragma once
#include <stdint.h>

enum I2cStatus : uint8_t {
  I2C_OK = 0,
  I2C_NACK,       // no ACK: device missing / cable off / address glitch
  I2C_BUSY,       // the bus was busy before we started (a slave holds SDA low: stuck bus)
  I2C_ARB_LOST,   // arbitration lost (SDA did not follow us: noise or a stuck slave)
  I2C_FIFO_ERR,   // the LPI2C command FIFO went wrong
  I2C_PIN_LOW,    // SCL / SDA held low too long (pin low timeout)
  I2C_TIMEOUT,    // not done within the time budget: aborted
  I2C_STATUS_COUNT
};

inline const char* i2c_status_name(uint8_t s) {
  static const char* const k[] = {"ok", "nack", "busy", "arb", "fifo", "pinlow", "timeout"};
  return s < I2C_STATUS_COUNT ? k[s] : "?";
}

// A failure that says the bus itself is wedged (not just one bad transfer): recover at once.
inline bool i2c_status_stuck(uint8_t s) { return s == I2C_BUSY || s == I2C_PIN_LOW || s == I2C_TIMEOUT || s == I2C_ARB_LOST; }
