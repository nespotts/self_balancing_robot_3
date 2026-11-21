#include "odometry.h"

#include <Arduino.h>
#include <math.h>

static constexpr float kMmPerCount = (float)M_PI * WHEEL_DIAMETER_MM / 4096.0f;

void WheelEncoder::begin() {
  as_.begin(dir_pin_);
  as_.setDirection(clockwise_ ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE);
}

bool WheelEncoder::read(uint32_t t_us) {
  uint16_t raw = as_.readAngle();
  if (as_.lastError() != AS5600_OK) {
    bad_reads_++;
    errors_++;
    return false;
  }
  bad_reads_ = 0;
  if (!have_) {
    last_raw_ = raw;
    have_ = true;
  }
  // unwrap: assumes < half a revolution between reads (fine below ~7000 rpm at 200 Hz)
  int16_t d = (int16_t)(raw - last_raw_);
  if (d > 2047) d -= 4096;
  if (d < -2048) d += 4096;
  last_raw_ = raw;
  counts_ += d;

  // windowed speed: change over the oldest sample still in the ring
  ring_c_[ring_head_] = counts_;
  ring_t_[ring_head_] = t_us;
  ring_head_ = (ring_head_ + 1) % ENCODER_WINDOW;
  if (ring_n_ < ENCODER_WINDOW) ring_n_++;
  uint8_t oldest = (ring_head_ + ENCODER_WINDOW - ring_n_) % ENCODER_WINDOW;
  uint32_t dt = t_us - ring_t_[oldest];
  speed_mms_ = dt > 0 ? (float)(counts_ - ring_c_[oldest]) * kMmPerCount * 1e6f / (float)dt : 0.0f;
  return true;
}

void Odometry::begin() {
  left_.begin();
  right_.begin();
}

void Odometry::update(uint32_t t_us, float heading_deg) {
  left_.read(t_us);
  right_.read(t_us);
  last_us_ = t_us;

  int32_t l = left_.counts(), r = right_.counts();
  if (!have_) {
    last_l_ = l;
    last_r_ = r;
    last_heading_ = heading_deg;
    have_ = true;
  }
  float dl = (float)(l - last_l_) * kMmPerCount;
  float dr = (float)(r - last_r_) * kMmPerCount;
  last_l_ = l;
  last_r_ = r;

  float h_mid = 0.5f * (heading_deg + last_heading_) * DEG_TO_RAD;
  last_heading_ = heading_deg;
  float d = 0.5f * (dl + dr);
  x_mm += d * cosf(h_mid);
  y_mm += d * sinf(h_mid);

  left_mms = left_.speedMms();
  right_mms = right_.speedMms();
  speed_mms = 0.5f * (left_mms + right_mms);
  turn_rate_dps = (right_mms - left_mms) / TRACK_MM * RAD_TO_DEG;
}

void Odometry::resetPose() {
  x_mm = 0;
  y_mm = 0;
}
