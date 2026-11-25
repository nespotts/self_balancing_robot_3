#include "odometry.h"

#include <math.h>

#include "protocol/link_protocol.h"

static constexpr float kDegToRad = 0.017453292519943295f;
static constexpr float kRadToDeg = 57.29577951308232f;

void Odometry::begin(EncoderIo* left, EncoderIo* right) {
  left_.attach(left);
  right_.attach(right);
}

void Odometry::update(uint32_t t_us, float heading_deg, float yaw_rate_dps) {
  const float mm_l = odo_mm_per_count(false), mm_r = odo_mm_per_count(true);  // params: live, issue #14
  left_.update(t_us, mm_l);
  right_.update(t_us, mm_r);
  last_us_ = t_us;

  const int32_t l = left_.counts(), r = right_.counts();
  if (!have_) {
    last_l_ = l;
    last_r_ = r;
    last_heading_ = heading_deg;
    have_ = true;
  }
  const float dl_w = (float)(l - last_l_) * mm_l;
  const float dr_w = (float)(r - last_r_) * mm_r;
  last_l_ = l;  // always: a wheel coming back from a fault must not add what it counted meanwhile
  last_r_ = r;
  const float dh = (heading_deg - last_heading_) * kDegToRad;
  const float h_mid = 0.5f * (heading_deg + last_heading_) * kDegToRad;
  last_heading_ = heading_deg;
  const float track = P(P_ODO_TRACK);
  const float yaw = yaw_rate_dps * kDegToRad;

  const bool lok = leftOk(), rok = rightOk();
  float dl = 0, dr = 0, vl = 0, vr = 0;
  if (lok && rok) {
    dl = dl_w;
    dr = dr_w;
    vl = left_.speedMms();
    vr = right_.speedMms();
    speed_ok_ = left_.speedValid() && right_.speedValid();
  } else if (rok) {  // left faulted: right - left = yaw x track
    dr = dr_w;
    dl = dr - dh * track;
    vr = right_.speedMms();
    vl = vr - yaw * track;
    speed_ok_ = right_.speedValid();
  } else if (lok) {
    dl = dl_w;
    dr = dl + dh * track;
    vl = left_.speedMms();
    vr = vl + yaw * track;
    speed_ok_ = left_.speedValid();
  } else {
    speed_ok_ = false;  // both faulted: no distance, no speed
  }
  const float d = 0.5f * (dl + dr);
  x_mm += d * cosf(h_mid);
  y_mm += d * sinf(h_mid);

  left_mms = vl;
  right_mms = vr;
  speed_mms = 0.5f * (vl + vr);
  turn_rate_dps = (lok && rok) ? (vr - vl) / track * kRadToDeg : yaw_rate_dps;
}

uint8_t Odometry::faultFlags(uint32_t t_us) const {
  uint8_t f = 0;
  if (!leftOk()) f |= LINK_FAULT_ENC_L;
  if (!rightOk()) f |= LINK_FAULT_ENC_R;
  if (left_.troubled(t_us)) f |= LINK_FAULT_ENC_L_ERR;
  if (right_.troubled(t_us)) f |= LINK_FAULT_ENC_R_ERR;
  return f;
}

void Odometry::resetPose() {
  x_mm = 0;
  y_mm = 0;
}
