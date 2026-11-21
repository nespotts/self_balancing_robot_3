// BNO08x over SPI with our own non-blocking SH-2 HAL (the Adafruit HAL waits up to 500 ms for
// the INT pin and resets the chip on timeout). One gyro-integrated rotation vector report gives
// orientation + angular velocity from the same instant; no magnetometer is used.
#pragma once
#include <stdint.h>

struct ImuSample {
  uint32_t t_us = 0;
  float pitch_deg = 0, roll_deg = 0, yaw_deg = 0;  // yaw wrapped to (-180, 180], + = CCW
  float pitch_rate_dps = 0, yaw_rate_dps = 0;      // Euler rates computed from the gyro
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;        // raw body rates
};

class Imu {
 public:
  bool begin();
  // If the INT pin says data is waiting, read one SHTP packet (~0.2 ms). Returns true when it
  // produced a new orientation sample. Returns immediately otherwise.
  bool poll();
  bool dataReady() const;
  // Re-enable reports after a sensor reset / stall and retry a failed init. Some of this
  // blocks for a few ms, so it only acts when motors_off.
  void service(uint32_t now_ms, bool motors_off);

  const ImuSample& sample() const { return s_; }
  float heading() const { return heading_ - heading_offset_; }  // continuous, deg, + = CCW
  void resetHeading() { heading_offset_ = heading_; }
  bool initialised() const { return ok_; }
  bool healthy(uint32_t now_us) const;
  uint32_t samples() const { return samples_; }
  float nominalDt() const { return interval_us_ ? interval_us_ * 1e-6f : 0.0025f; }

  // called from the SH-2 callbacks
  void onGyroRv(float qr, float qi, float qj, float qk, float wx, float wy, float wz);
  void onReset() { need_reports_ = true; }

 private:
  bool enableReports();
  uint32_t wantedIntervalUs() const;
  uint32_t interval_us_ = 0;
  ImuSample s_;
  bool ok_ = false, fresh_ = false, need_reports_ = false, have_yaw_ = false;
  float heading_ = 0, heading_offset_ = 0, last_yaw_ = 0;
  uint32_t samples_ = 0, last_retry_ms_ = 0;
};
