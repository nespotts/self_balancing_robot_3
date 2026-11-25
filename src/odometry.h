// Wheel odometry from two AS5600 magnetic encoders (one bounded I2C read per wheel per update, src/wheel_encoder.h).
// Pose uses the IMU heading for direction and the wheels for distance. Pure logic: the I2C reads come through the
// EncoderIo objects given to begin() (src/encoder_io.h on the robot, mocks in the host tests).
//
// One wheel's fault latched (issue #109): its travel and speed come from the other wheel and the gyro
// (left = right - yaw rate x track), so the balance loop keeps its velocity loop and the robot holds its place.
// Both faulted: no speed (speedOk() false: the velocity loop is off, balance + heading only), the pose stops.
#pragma once
#include <stdint.h>

#include "config.h"
#include "params.h"
#include "wheel_encoder.h"

class Odometry {
 public:
  void begin(EncoderIo* left, EncoderIo* right);
  // Read both encoders and integrate the pose. heading_deg / yaw_rate_dps: IMU, + = CCW.
  void update(uint32_t t_us, float heading_deg, float yaw_rate_dps);
  void resetPose();

  float x_mm = 0, y_mm = 0;
  float speed_mms = 0;       // forward, (left + right) / 2
  float turn_rate_dps = 0;   // from the wheels, + = CCW (a faulted wheel: the gyro's)
  float left_mms = 0, right_mms = 0;
  bool leftOk() const { return !left_.faulted(); }    // latched fault (error_code 3)
  bool rightOk() const { return !right_.faulted(); }  // (error_code 4)
  bool ok() const { return leftOk() && rightOk(); }
  // A speed the velocity loop can use: both wheels, or one wheel + the gyro.
  bool speedOk() const { return speed_ok_; }
  bool estimated() const { return speed_ok_ && !ok(); }  // one wheel is derived from the other + the gyro
  uint8_t faultFlags(uint32_t t_us) const;             // LINK_FAULT_ENC_* bits
  bool leftBusSuspect(uint32_t t_us) const { return left_.busSuspect(t_us); }  // Wire: also the range sweep's bus
  int32_t leftCounts() const { return left_.counts(); }  // 4096 per wheel rev
  int32_t rightCounts() const { return right_.counts(); }
  uint32_t lastUpdateUs() const { return last_us_; }
  WheelEncoder& wheel(bool right) { return right ? right_ : left_; }
  const WheelEncoder& wheel(bool right) const { return right ? right_ : left_; }

 private:
  WheelEncoder left_, right_;
  int32_t last_l_ = 0, last_r_ = 0;
  float last_heading_ = 0;
  bool have_ = false, speed_ok_ = false;
  uint32_t last_us_ = 0;
};
