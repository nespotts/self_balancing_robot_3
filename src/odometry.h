// Wheel odometry from two AS5600 magnetic encoders (one I2C read per wheel per update).
// Pose uses the IMU heading for direction and the wheels for distance.
#pragma once
#include <AS5600.h>
#include <Wire.h>
#include <stdint.h>

#include "config.h"

class WheelEncoder {
 public:
  WheelEncoder(TwoWire* wire, uint8_t dir_pin, bool clockwise)
      : as_(wire), dir_pin_(dir_pin), clockwise_(clockwise) {}
  void begin();
  // One I2C read (~0.15 ms at 400 kHz). Returns false on an I2C error.
  bool read(uint32_t t_us);
  int32_t counts() const { return counts_; }
  float speedMms() const { return speed_mms_; }  // over the last ENCODER_WINDOW reads
  bool ok() const { return have_ && bad_reads_ < 10; }
  uint32_t errors() const { return errors_; }

 private:
  AS5600 as_;
  uint8_t dir_pin_;
  bool clockwise_;
  bool have_ = false;
  uint16_t last_raw_ = 0;
  int32_t counts_ = 0;
  float speed_mms_ = 0;
  uint32_t bad_reads_ = 0, errors_ = 0;
  // ring of (counts, time) for the windowed speed
  int32_t ring_c_[ENCODER_WINDOW] = {};
  uint32_t ring_t_[ENCODER_WINDOW] = {};
  uint8_t ring_head_ = 0, ring_n_ = 0;
};

class Odometry {
 public:
  void begin();
  // Read both encoders and integrate the pose. heading_deg: IMU heading, + = CCW.
  void update(uint32_t t_us, float heading_deg);
  void resetPose();

  float x_mm = 0, y_mm = 0;
  float speed_mms = 0;       // forward, (left + right) / 2
  float turn_rate_dps = 0;   // from the wheels, + = CCW
  float left_mms = 0, right_mms = 0;
  bool leftOk() const { return left_.ok(); }
  bool rightOk() const { return right_.ok(); }
  bool ok() const { return left_.ok() && right_.ok(); }
  int32_t leftCounts() const { return left_.counts(); }  // 4096 per wheel rev
  int32_t rightCounts() const { return right_.counts(); }
  uint32_t lastUpdateUs() const { return last_us_; }

 private:
  WheelEncoder left_{&Wire, pins::ENC_L_DIR, false};
  WheelEncoder right_{&Wire1, pins::ENC_R_DIR, true};
  int32_t last_l_ = 0, last_r_ = 0;
  float last_heading_ = 0;
  bool have_ = false;
  uint32_t last_us_ = 0;
};
