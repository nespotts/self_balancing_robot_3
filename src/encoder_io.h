// The robot's EncoderIo (src/wheel_encoder.h): an AS5600 angle read over a bounded LPI2C transfer (src/i2c_bounded.h).
#pragma once
#include <Arduino.h>

#include "config.h"
#include "i2c_bounded.h"
#include "wheel_encoder.h"

class As5600Io : public EncoderIo {
 public:
  As5600Io(TwoWire& wire, uintptr_t port, uint8_t sda, uint8_t scl, uint32_t hz, uint8_t dir_pin, bool clockwise)
      : bus_(wire, port, sda, scl, hz), dir_pin_(dir_pin), clockwise_(clockwise) {}
  // DIR pin (the AS5600 counts clockwise with DIR low), then the bus. The bus must be up before the first read.
  void begin() {
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, clockwise_ ? LOW : HIGH);
    bus_.begin();
  }
  I2cStatus readAngle(uint16_t& raw) override {
    uint8_t b[2] = {0, 0};
    const I2cStatus s = bus_.readReg(AS5600_ADDR, AS5600_REG_ANGLE, b, 2);
    raw = (uint16_t)(((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
    return s;
  }
  bool recover() override { return bus_.recover(); }
  uint32_t lastReadUs() const override { return bus_.lastUs(); }
  BoundedI2c& bus() { return bus_; }

 private:
  BoundedI2c bus_;
  uint8_t dir_pin_;
  bool clockwise_;
};
