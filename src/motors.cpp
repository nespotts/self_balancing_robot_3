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

// drive config in use (applied only with both motors off, see motors.h)
static uint8_t g_mode = 0;
static float g_hz = 0;
static bool g_cfg_changed = false;

void Motor::begin() {
  for (uint8_t i = 0; i < 3; i++) {
    digitalWriteFast(pin_[i], LOW);  // level first, then GPIO output: no glitch
    pinMode(pin_[i], OUTPUT);
    is_pwm_[i] = false;
  }
  off_ = true;
  cmd_ = 0;
}

// GPIO level. A pin that was PWM gets its duty cleared first (it loads at the next PWM period,
// while the pin is already GPIO), so its next pwm() starts from 0, never from a stale duty.
// pinMode(OUTPUT) only switches the mux (the GPIO data bit was written just before): the level
// applies at once, not at the end of the PWM period.
void Motor::level(uint8_t i, bool high) {
  const uint8_t p = pin_[i];
  if (is_pwm_[i]) {
    analogWrite(p, 0);
    digitalWriteFast(p, high ? HIGH : LOW);
    pinMode(p, OUTPUT);
    is_pwm_[i] = false;
  } else {
    digitalWriteFast(p, high ? HIGH : LOW);
  }
}

void Motor::pwm(uint8_t i, int duty) {
  analogWrite(pin_[i], duty);  // also switches the mux to PWM
  is_pwm_[i] = true;
}

void Motor::off() {
  level(EN, false);  // EN first: the bridge lets go at once
  level(D1, false);
  level(D2, false);
  off_ = true;
  cmd_ = 0;
}

void Motor::drive(float pwm_signed) {
  float a = fabsf(pwm_signed);
  if (a > MOTOR_FULL) a = MOTOR_FULL;
  const int duty = (int)(a + 0.5f);
  off_ = false;
  // The IN pin that goes low is always set first, so both INs are never high together.
  if (g_mode == 1) {
    // slow decay / brake: EN high, PWM on the direction's IN pin, the other IN low
    if (pwm_signed > 0) {
      level(D2, false);
      level(EN, true);
      pwm(D1, duty);
    } else if (pwm_signed < 0) {
      level(D1, false);
      level(EN, true);
      pwm(D2, duty);
    } else {
      level(D1, false);
      level(D2, false);
      level(EN, true);  // both INs low, EN high = brake (hold); disarmed / IDLE uses off() instead
    }
    return;
  }
  // mode 0 (v3): fast decay, PWM on EN, INs static
  if (pwm_signed > 0) {
    level(D2, false);
    level(D1, true);
  } else if (pwm_signed < 0) {
    level(D1, false);
    level(D2, true);
  } else {
    level(D1, false);
    level(D2, false);
  }
  pwm(EN, duty);
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

static uint8_t wantMode() { return P(P_MOT_DRIVE_MODE) >= 0.5f ? 1 : 0; }
// mot.pwm_hz, or its default while the params are not loaded yet (motors_begin runs first: all 0)
static float wantHz() {
  const ParamDef& d = params_def(P_MOT_PWM_HZ);
  const float hz = P(P_MOT_PWM_HZ);
  return (hz >= d.min && hz <= d.max) ? hz : d.def;
}

// Only with both motors off (EN low, INs GPIO low): nothing is driven while the timers change.
static void applyConfig() {
  const float hz = wantHz();
  if (hz != g_hz) {
    // every motor pin's FlexPWM submodule (EN and both INs, see motors.h); pins sharing one are
    // simply set twice to the same value
    const Motor* const ms[2] = {&g_left, &g_right};
    for (const Motor* m : ms)
      for (uint8_t i = 0; i < 3; i++) analogWriteFrequency(m->pins()[i], hz);
    g_hz = hz;
  }
  g_mode = wantMode();
}

void motors_begin() {
  analogWriteResolution(12);
  g_left.begin();
  g_right.begin();
  applyConfig();
}

void motors_service() {
  if (!g_left.isOff() || !g_right.isOff()) return;
  if (wantHz() == g_hz && wantMode() == g_mode) return;
  applyConfig();
  g_cfg_changed = true;
}

bool motors_take_config_changed() {
  const bool c = g_cfg_changed;
  g_cfg_changed = false;
  return c;
}

uint8_t motors_drive_mode() { return g_mode; }
float motors_pwm_hz() { return g_hz; }

void motors_set(float left, float right) {
  g_left.set(left);
  g_right.set(right);
}

void motors_set_raw(float left_duty, float right_duty) {
  g_left.setRaw(left_duty);
  g_right.setRaw(right_duty);
}

void motors_stop() {
  g_left.off();
  g_right.off();
  motors_service();
}
float motors_left() { return g_left.command(); }
float motors_right() { return g_right.command(); }
