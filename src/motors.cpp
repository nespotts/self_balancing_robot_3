#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "params.h"

static constexpr float kVbattMin = 6.0f, kVbattMax = 17.0f;  // outside: reading not trusted
static constexpr float kVbattTauS = 1.5f;
static constexpr float kVscaleMax = 1.3f, kVscaleMin = 0.5f;

static float g_vbatt = 0;  // filtered, 0 = none yet / invalid
static uint32_t g_vbatt_ms = 0;

float motor_curve(float in, float exp, float knee_in, float knee_out) {
  in = fabsf(in);
  if (in <= 0.0f) return 0.0f;
  if (in > MOTOR_FULL) in = MOTOR_FULL;
  float out;
  if (in <= knee_in) {
    out = knee_out * powf(in / knee_in, 1.0f / exp);
  } else {
    float slope = (MOTOR_FULL - knee_out) / (MOTOR_FULL - knee_in);
    out = knee_out + slope * (in - knee_in);
  }
  if (out > MOTOR_FULL) out = MOTOR_FULL;
  return out;
}

float motor_dz_volts(float u, float dz, float v_ref, float band) {
  u = fabsf(u);
  if (u <= 0.0f) return 0.0f;
  if (u > 1.0f) u = 1.0f;
  if (dz > v_ref) dz = v_ref;
  if (u < band) return u / band * (dz + (v_ref - dz) * band);  // taper: no full dead-zone kick on noise
  return dz + (v_ref - dz) * u;
}

void motors_set_vbatt(float v, uint32_t now_ms) {
  if (isnan(v) || v < kVbattMin || v > kVbattMax) {
    g_vbatt = 0;  // bad / unplugged balance lead: no compensation (scale 1)
    return;
  }
  if (g_vbatt <= 0.0f) {
    g_vbatt = v;
  } else {
    const float dt = (float)(now_ms - g_vbatt_ms) * 1e-3f;
    if (dt <= 0.0f) return;
    g_vbatt += (v - g_vbatt) * (dt > kVbattTauS ? 1.0f : dt / kVbattTauS);
  }
  g_vbatt_ms = now_ms;
}

float motors_vbatt() { return g_vbatt; }

float motors_vscale() {
  if (P(P_MOT_VCOMP) < 0.5f || g_vbatt <= 0.0f) return 1.0f;
  const float s = P(P_MOT_V_REF) / g_vbatt;
  return s > kVscaleMax ? kVscaleMax : (s < kVscaleMin ? kVscaleMin : s);
}

void Motor::begin() {
  pinMode(en_, OUTPUT);
  pinMode(d1_, OUTPUT);
  pinMode(d2_, OUTPUT);
  digitalWriteFast(en_, LOW);
  digitalWriteFast(d1_, LOW);
  digitalWriteFast(d2_, LOW);
  analogWriteFrequency(en_, 36621.09f);
  analogWrite(en_, 0);
}

void Motor::drive(float pwm) {
  if (pwm > 0) {
    digitalWriteFast(d1_, HIGH);
    digitalWriteFast(d2_, LOW);
  } else if (pwm < 0) {
    digitalWriteFast(d1_, LOW);
    digitalWriteFast(d2_, HIGH);
  } else {
    digitalWriteFast(d1_, LOW);
    digitalWriteFast(d2_, LOW);
  }
  float a = fabsf(pwm);
  if (a > MOTOR_FULL) a = MOTOR_FULL;
  analogWrite(en_, (int)(a + 0.5f));
}

void Motor::set(float cmd) {
  if (isnan(cmd)) cmd = 0;
  cmd_ = cmd;
  float pwm;
  if (P(P_MOT_MODEL) >= 0.5f) {
    const float v_ref = P(P_MOT_V_REF);
    const float dz = P((ParamId)(cmd >= 0 ? dz_fwd_ : dz_rev_));
    pwm = motor_dz_volts(cmd / MOTOR_FULL, dz, v_ref, P(P_MOT_DZ_BAND)) / v_ref * MOTOR_FULL;
  } else {
    pwm = motor_curve(cmd, P(P_MOT_CURVE_EXP), P(P_MOT_KNEE_IN), P(P_MOT_KNEE_OUT));
  }
  pwm *= motors_vscale();
  drive(cmd > 0 ? pwm : (cmd < 0 ? -pwm : 0.0f));
}

void Motor::setRaw(float duty) {
  if (isnan(duty)) duty = 0;
  cmd_ = duty;
  drive(duty);
}

static Motor g_left(pins::MOT_L_EN, pins::MOT_L_D1, pins::MOT_L_D2, P_MOT_DZ_L_FWD, P_MOT_DZ_L_REV);
static Motor g_right(pins::MOT_R_EN, pins::MOT_R_D1, pins::MOT_R_D2, P_MOT_DZ_R_FWD, P_MOT_DZ_R_REV);

void motors_begin() {
  analogWriteResolution(12);
  g_left.begin();
  g_right.begin();
}

void motors_set(float left, float right) {
  g_left.set(left);
  g_right.set(right);
}

void motors_set_raw(float left_duty, float right_duty) {
  g_left.setRaw(left_duty);
  g_right.setRaw(right_duty);
}

void motors_stop() { motors_set(0, 0); }
float motors_left() { return g_left.command(); }
float motors_right() { return g_right.command(); }
