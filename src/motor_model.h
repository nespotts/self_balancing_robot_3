// Motor maths shared by the drive layer (motors.cpp), the motor test (motor_test.cpp) and the host
// tests: no Arduino calls (issue #26).
//
//   motor_curve       mot.model 0: v3 dead-zone linearisation curve (duty in, duty out)
//   motor_dz_volts    mot.model 1: u -> volts with the dead zone added
//   WheelSpeedEst     fast wheel speed from encoder counts (20 ms window, ~10 ms lag)
//   WheelSpeedLoop    closed-loop wheel speed (mot.speed_loop): feed-forward from the volts model
//                     + PI on the speed error, dead-zone inverse on the result, anti-windup
#pragma once
#include <math.h>
#include <stdint.h>

// |in| 0..4095 -> PWM 0..4095. Below knee_in: knee_out * (in/knee_in)^(1/exp), above: linear to 4095.
inline float motor_curve(float in, float exp, float knee_in, float knee_out) {
  in = fabsf(in);
  if (in <= 0.0f) return 0.0f;
  if (in > 4095.0f) in = 4095.0f;
  float out;
  if (in <= knee_in) {
    out = knee_out * powf(in / knee_in, 1.0f / exp);
  } else {
    const float slope = (4095.0f - knee_out) / (4095.0f - knee_in);
    out = knee_out + slope * (in - knee_in);
  }
  return out > 4095.0f ? 4095.0f : out;
}

// |u| 0..1 -> target motor volts (model 1): dz + (v_ref - dz) * |u|, straight taper 0 -> dz inside |u| < band.
inline float motor_dz_volts(float u, float dz, float v_ref, float band) {
  u = fabsf(u);
  if (u <= 0.0f) return 0.0f;
  if (u > 1.0f) u = 1.0f;
  if (dz > v_ref) dz = v_ref;
  if (u < band) return u / band * (dz + (v_ref - dz) * band);  // taper: no full dead-zone kick on noise
  return dz + (v_ref - dz) * u;
}

// Dead-zone inverse in volts: "volts above the dead zone" (signed) -> motor volts (signed).
// Inside |x| < band: straight taper 0 -> dz + band (no full dead-zone kick on noise around 0).
inline float motor_dz_inverse(float x, float dz_fwd, float dz_rev, float band) {
  const float dz = x >= 0.0f ? dz_fwd : dz_rev;
  const float a = fabsf(x);
  float v;
  if (band > 0.0f && a < band) v = a / band * (dz + band);
  else v = dz + a;
  return x >= 0.0f ? v : -v;
}

// Speed from the encoder counts over the last kN reads (200 Hz reads -> 20 ms window).
class WheelSpeedEst {
 public:
  static constexpr int kN = 5;  // samples in the ring: speed over kN-1 intervals
  void reset() { n_ = 0; head_ = 0; speed_ = 0; }
  // one encoder read; mm_per_count = pi * wheel diameter / 4096
  void sample(int32_t counts, uint32_t t_us, float mm_per_count) {
    c_[head_] = counts;
    t_[head_] = t_us;
    head_ = (head_ + 1) % kN;
    if (n_ < kN) n_++;
    const int old = (head_ + kN - n_) % kN;
    const uint32_t dt = t_us - t_[old];
    speed_ = (n_ > 1 && dt > 0) ? (float)(counts - c_[old]) * mm_per_count * 1e6f / (float)dt : 0.0f;
    last_us_ = t_us;
  }
  float speed() const { return speed_; }
  bool valid() const { return n_ >= 2; }
  uint32_t lastUs() const { return last_us_; }

 private:
  int32_t c_[kN] = {};
  uint32_t t_[kN] = {};
  int head_ = 0, n_ = 0;
  float speed_ = 0;
  uint32_t last_us_ = 0;
};

struct SpeedLoopCfg {
  float kp = 0.006f;        // V per mm/s of error
  float ki = 0.08f;         // V per (mm/s * s)
  float i_max = 3.0f;       // |integrator| limit, V
  float k_mms_per_v = 200;  // plant gain above the dead zone (motor_fit.py "k")
  float dz_fwd = 6.0f, dz_rev = 6.0f;  // dead zone, V (motor_fit.py proposal, mot.dz_*)
  float band = 0.3f;        // dead-zone inverse taper, V
  float v_max = 12.0f;      // output limit, V (pack voltage or the duty cap)
};

// Wheel speed PI with feed-forward:
//   x = target / k + kp * e + I      (volts above the dead zone)
//   V = dz_inverse(x), |V| <= v_max
//   I += ki * e * dt, |I| <= i_max, frozen while V is at the limit and e pushes further (anti-windup)
// target 0 with the wheel (nearly) stopped: output 0 and I bleeds off (no creep / hum at rest).
class WheelSpeedLoop {
 public:
  void reset() { i_ = 0; ff_ = p_ = out_ = 0; }
  float step(float target_mms, float speed_mms, float dt, const SpeedLoopCfg& c) {
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;
    const float e = target_mms - speed_mms;
    if (target_mms == 0.0f && fabsf(speed_mms) < 8.0f) {
      i_ *= 0.9f;
      ff_ = p_ = out_ = 0;
      return 0.0f;
    }
    const float k = c.k_mms_per_v > 1.0f ? c.k_mms_per_v : 1.0f;
    ff_ = target_mms / k;
    p_ = c.kp * e;
    float v = motor_dz_inverse(ff_ + p_ + i_, c.dz_fwd, c.dz_rev, c.band);
    const bool sat = fabsf(v) >= c.v_max;
    if (sat) v = v > 0 ? c.v_max : -c.v_max;
    if (!sat || (e > 0) != (v > 0)) {
      i_ += c.ki * e * dt;
      if (i_ > c.i_max) i_ = c.i_max;
      if (i_ < -c.i_max) i_ = -c.i_max;
    }
    out_ = v;
    return v;
  }
  float out() const { return out_; }
  float ff() const { return ff_; }
  float p() const { return p_; }
  float i() const { return i_; }

 private:
  float i_ = 0, ff_ = 0, p_ = 0, out_ = 0;
};
