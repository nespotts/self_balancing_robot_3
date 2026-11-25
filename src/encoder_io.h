// The robot's EncoderIo (src/wheel_encoder.h): an AS5600 angle read.
//   SBR_I2C_BOUNDED 1: one bounded LPI2C transfer (src/i2c_bounded.h) + bit-banged bus recovery (v9)
//   SBR_I2C_BOUNDED 0: plain Wire calls (v8 style, Wire's own 16 ms / 50 ms timeouts), no bus recovery
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "crash_capture.h"
#include "i2c_bounded.h"
#include "wheel_encoder.h"

class As5600Io : public EncoderIo {
 public:
  As5600Io(TwoWire& wire, uintptr_t port, uint8_t sda, uint8_t scl, uint32_t hz, uint8_t dir_pin, bool clockwise,
           uint8_t phase)
      : bus_(wire, port, sda, scl, hz), wire_(wire), hz_(hz), dir_pin_(dir_pin), clockwise_(clockwise), phase_(phase) {}
  // DIR pin (the AS5600 counts clockwise with DIR low), then the bus. The bus must be up before the first read.
  void begin() {
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, clockwise_ ? LOW : HIGH);
#if SBR_I2C_BOUNDED
    bus_.begin();
#else
    wire_.begin();
    wire_.setClock(hz_ >= 400000 ? 400000 : 100000);  // Wire: 100 k / 400 k / 1 M only
#endif
  }
  I2cStatus readAngle(uint16_t& raw) override {
    crash_phase(phase_);
#if SBR_I2C_BOUNDED
    uint8_t b[2] = {0, 0};
    const I2cStatus s = bus_.readReg(AS5600_ADDR, AS5600_REG_ANGLE, b, 2);
    raw = (uint16_t)(((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
    return s;
#else
    const uint32_t t0 = micros();
    wire_.beginTransmission(AS5600_ADDR);
    wire_.write(AS5600_REG_ANGLE);
    const uint8_t e = wire_.endTransmission(false);
    I2cStatus s = I2C_OK;
    if (e == 2 || e == 3) s = I2C_NACK;
    else if (e != 0) s = I2C_TIMEOUT;  // 4 = bus busy / timeout, 5 = FIFO error
    else if (wire_.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) s = I2C_NACK;
    if (s == I2C_OK) {
      const uint8_t hi = (uint8_t)wire_.read();
      const uint8_t lo = (uint8_t)wire_.read();
      raw = (uint16_t)(((uint16_t)hi << 8) | lo) & 0x0FFF;
    }
    last_us_ = micros() - t0;
    return s;
#endif
  }
  bool recover() override {
#if SBR_I2C_BOUNDED
    crash_phase(PH_ENC_REC);
    return bus_.recover();
#else
    return false;  // v8 had none: the wheel encoder still backs off (ENC_BACKOFF_*)
#endif
  }
#if SBR_I2C_BOUNDED
  uint32_t lastReadUs() const override { return bus_.lastUs(); }
#else
  uint32_t lastReadUs() const override { return last_us_; }
#endif
  BoundedI2c& bus() { return bus_; }

 private:
  BoundedI2c bus_;
  TwoWire& wire_;
  uint32_t hz_;
  uint8_t dir_pin_;
  bool clockwise_;
  uint8_t phase_;
  uint32_t last_us_ = 0;
};
