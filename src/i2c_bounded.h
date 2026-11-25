// Bounded I2C master transfers on a Teensy 4 LPI2C port, for the AS5600 encoder reads (issue #109).
//
// The Teensy Wire library can wait up to ~16 ms for a busy bus and 50 ms per transfer phase, so one glitched read could
// stall the 400 Hz balance loop for >100 ms (and trip the hardware watchdog). Here the whole register read is queued in
// the LPI2C command FIFO and polled against a hard time budget (2.5x the transfer's wire time + 50 us); on any error or
// timeout the FIFOs are flushed and the call returns. Nothing is ever left in flight: the range sweep can use Wire on
// the same bus right after.
//
// recover(): the standard stuck-bus cure - SDA released, 9 SCL clocks (a slave holding SDA low mid-byte finishes its
// byte), a STOP, then the LPI2C is reset and re-initialised (Wire.begin() + the clock). ~0.15 ms, bounded.
#pragma once
#include <Wire.h>
#include <stdint.h>

#include "i2c_status.h"

class BoundedI2c {
 public:
  BoundedI2c(TwoWire& wire, uintptr_t port, uint8_t sda, uint8_t scl, uint32_t hz)
      : wire_(wire), port_(port), sda_(sda), scl_(scl), hz_(hz) {}
  // Wire.begin() + the clock (100 / 200 / 400 kHz). Also what recover() ends with.
  void begin();
  void applyClock();
  // Write `reg`, repeated START, read n (1..4) bytes, STOP.
  I2cStatus readReg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);
  bool recover();          // true = both lines high afterwards
  uint32_t lastUs() const { return last_us_; }
  uint32_t hz() const { return hz_; }

 private:
  TwoWire& wire_;
  uintptr_t port_;
  uint8_t sda_, scl_;
  uint32_t hz_;
  uint32_t last_us_ = 0;
};
