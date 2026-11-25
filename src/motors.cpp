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
static bool g_test = false;  // the motor test holds the motors (motors_test_begin .. _end)

static constexpr uint32_t kFeedbackStaleUs = 30000;

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
  loop_.reset();
  loop_run_ = false;
  sl_target_ = 0;
  kick_.reset();
}

void Motor::drive(float pwm_signed) {
  // Wiring direction (mot.left_dir / mot.right_dir): flipped here, below every caller (balance loop,
  // speed loop, motor test), so a + command always means "wheel forward" whatever the wires do.
  if (P((ParamId)dir_) < 0.0f) pwm_signed = -pwm_signed;
  float a = fabsf(pwm_signed);
  if (a > MOTOR_FULL) a = MOTOR_FULL;
  const int duty = (int)(a + 0.5f);
  off_ = false;
  // The IN pin that goes low is always set first, so both INs are never high together.
  if (g_mode == 1) {
    // slow decay / brake (motor test only): EN high, PWM on the direction's IN pin, the other IN low
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
      level(EN, true);  // both INs low, EN high = brake (hold); the test sends 0 as off() instead
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

void Motor::feedback(int32_t counts, uint32_t t_us, float mm_per_count) { est_.sample(counts, t_us, mm_per_count); }

bool Motor::feedbackFresh(uint32_t now_us) const {
  return est_.valid() && (int32_t)(now_us - est_.lastUs()) <= (int32_t)kFeedbackStaleUs;
}

// mot.speed_loop 1: cmd -> target wheel speed -> WheelSpeedLoop -> signed duty. false = not active
// (off, or no fresh encoder feedback): the caller uses mot.model instead.
bool Motor::speedLoop(float cmd, float& duty) {
  const uint32_t now = micros();
  if (P(P_MOT_SPEED_LOOP) < 0.5f || !feedbackFresh(now)) {
    if (loop_run_) loop_.reset();
    loop_run_ = false;
    sl_target_ = 0;
    return false;
  }
  SpeedLoopCfg c;
  c.kp = P(P_MOT_SL_KP);
  c.ki = P(P_MOT_SL_KI);
  c.i_max = P(P_MOT_SL_IMAX);
  c.band = P(P_MOT_SL_BAND);
  c.k_mms_per_v = P((ParamId)k_);
  c.dz_fwd = P((ParamId)dz_fwd_);
  c.dz_rev = P((ParamId)dz_rev_);
  const float vb = g_vbatt > 0.0f ? g_vbatt : P(P_MOT_V_REF);
  c.v_max = vb;
  const float dz = 0.5f * (c.dz_fwd + c.dz_rev);
  const float span = P(P_MOT_V_REF) - dz;
  sl_target_ = cmd / MOTOR_FULL * c.k_mms_per_v * (span > 0.5f ? span : 0.5f);  // u = 1 -> model-1 top speed
  const float dt = loop_run_ ? (float)(now - loop_us_) * 1e-6f : 0.0025f;
  loop_us_ = now;
  loop_run_ = true;
  duty = loop_.step(sl_target_, est_.speed(), dt, c) / vb * MOTOR_FULL;
  return true;
}

void Motor::set(float cmd) {
  if (isnan(cmd)) cmd = 0;
  cmd_ = cmd;
  float duty;
  if (speedLoop(cmd, duty)) {
    kick_.reset();
  } else {
    float pwm;
    // Brake mode never with the v3 curve (issue #40): mode 1 is only entered with mot.model 1, and a live
    // change to model 0 keeps the volts model until the next stop (then mode 0 is used).
    if (P(P_MOT_MODEL) >= 0.5f || g_mode == 1) {
      const float v_ref = P(P_MOT_V_REF);
      const float dz = P((ParamId)(cmd >= 0 ? dz_fwd_ : dz_rev_));
      const float u = cmd / MOTOR_FULL;
      float v = motor_dz_volts(u, dz, v_ref, P(P_MOT_DZ_BAND));
      // stiction kick (issue #69): breakaway volts for a stopped wheel; volts, so vcomp below applies too
      StictionCfg kc;
      kc.bk_fwd = P((ParamId)bk_fwd_);
      kc.bk_rev = P((ParamId)bk_rev_);
      kc.gain = P(P_MOT_BK_GAIN);
      kc.kick_ms = P(P_MOT_BK_MS);
      kc.band = P(P_MOT_BK_BAND);
      kc.move_mms = P(P_MOT_BK_MMS);
      const uint32_t now = micros();
      v = fabsf(kick_.apply(u, cmd >= 0 ? v : -v, est_.speed(), feedbackFresh(now), now, kc));
      if (v > v_ref) v = v_ref;
      pwm = v / v_ref * MOTOR_FULL;
    } else {
      kick_.reset();
      pwm = motor_curve(cmd, P(P_MOT_CURVE_EXP), P(P_MOT_KNEE_IN), P(P_MOT_KNEE_OUT));
    }
    pwm *= motors_vscale();
    duty = cmd > 0 ? pwm : (cmd < 0 ? -pwm : 0.0f);
  }
  drive(motor_brake_cap(duty, g_mode, P(P_MOT_BRAKE_DUTY_MAX)));
}

void Motor::setRaw(float duty) {
  if (isnan(duty)) duty = 0;
  cmd_ = duty;
  drive(duty);
}

static Motor g_left(pins::MOT_L_EN, pins::MOT_L_D1, pins::MOT_L_D2, P_MOT_DZ_L_FWD, P_MOT_DZ_L_REV, P_MOT_K_L,
                    P_MOT_LEFT_DIR, P_MOT_BK_L_FWD, P_MOT_BK_L_REV);
static Motor g_right(pins::MOT_R_EN, pins::MOT_R_D1, pins::MOT_R_D2, P_MOT_DZ_R_FWD, P_MOT_DZ_R_REV, P_MOT_K_R,
                     P_MOT_RIGHT_DIR, P_MOT_BK_R_FWD, P_MOT_BK_R_REV);

// mot.pwm_hz, or its default while the params are not loaded yet (motors_begin runs first: all 0)
static float validHz(float hz) {
  const ParamDef& d = params_def(P_MOT_PWM_HZ);
  return (hz >= d.min && hz <= d.max) ? hz : d.def;
}
static float wantHz() { return validHz(P(P_MOT_PWM_HZ)); }
static uint8_t wantMode();

// Only with both motors off (EN low, INs GPIO low): nothing is driven while the timers change.
static void applyConfig(float hz, uint8_t mode) {
  if (hz != g_hz) {
    // every motor pin's FlexPWM submodule (EN and both INs, see motors.h); pins sharing one are
    // simply set twice to the same value
    const Motor* const ms[2] = {&g_left, &g_right};
    for (const Motor* m : ms)
      for (uint8_t i = 0; i < 3; i++) analogWriteFrequency(m->pins()[i], hz);
    g_hz = hz;
    g_cfg_changed = true;
  }
  if (mode != g_mode) {
    g_mode = mode;
    g_cfg_changed = true;
  }
}

// ---- motor pins low as early and as hard as possible (issue #109) ----
// One pin straight to GPIO low output with the core's constant register macros (CORE_PINn_*): plain stores to fixed
// addresses, no RAM tables (digital_pin_to_info_PGM lives in DTCM, not yet copied at reset) and no call into ITCM code,
// so it also works in startup_early_hook (before RAM / ITCM are initialised) and in the watchdog interrupt.
#define PIN_LOW_RAW(n)                                  \
  do {                                                  \
    CORE_PIN##n##_PORTCLEAR = CORE_PIN##n##_BITMASK;    \
    CORE_PIN##n##_DDRREG |= CORE_PIN##n##_BITMASK;      \
    CORE_PIN##n##_PADCONFIG = IOMUXC_PAD_DSE(7);        \
    CORE_PIN##n##_CONFIG = 5 | 0x10; /* GPIO (off a PWM timer too) */ \
  } while (0)
static_assert(pins::MOT_L_EN == 4 && pins::MOT_L_D1 == 5 && pins::MOT_L_D2 == 6 && pins::MOT_R_EN == 7 &&
                  pins::MOT_R_D1 == 8 && pins::MOT_R_D2 == 9,
              "MOTOR_PINS_LOW() names the motor pins D4..D9 literally: update it with config.h");
// EN pins first: the bridge lets go at once
#define MOTOR_PINS_LOW() \
  do {                   \
    PIN_LOW_RAW(4);      \
    PIN_LOW_RAW(7);      \
    PIN_LOW_RAW(5);      \
    PIN_LOW_RAW(6);      \
    PIN_LOW_RAW(8);      \
    PIN_LOW_RAW(9);      \
  } while (0)

// Teensy core hook (cores/teensy4/startup.c): runs a few instructions after reset, ~300 ms before setup() (the core
// waits for USB). Without it the L298N inputs float (or the pad keepers hold their last level) all that time.
// The core selects the fast GPIOs (GPR26..29) a little later; done here first so the levels set now stay.
#if SBR_EARLY_HOOK
extern "C" FLASHMEM void startup_early_hook(void) {
  IOMUXC_GPR_GPR26 = 0xFFFFFFFF;
  IOMUXC_GPR_GPR27 = 0xFFFFFFFF;
  IOMUXC_GPR_GPR28 = 0xFFFFFFFF;
  IOMUXC_GPR_GPR29 = 0xFFFFFFFF;
  MOTOR_PINS_LOW();
}
#endif

void motors_emergency_off() { MOTOR_PINS_LOW(); }

void motors_begin() {
  MOTOR_PINS_LOW();  // again (the hook should have), before anything else
  analogWriteResolution(12);
  g_left.begin();
  g_right.begin();
  applyConfig(wantHz(), wantMode());
  g_cfg_changed = false;
}

// mot.drive_mode 1 only at mot.pwm_hz <= mot.brake_max_hz (brake decay heats the L298N more at high
// frequency: issues #23 / #26) and with mot.model 1 (issue #40); else mode 0 (motor_drive_refusal()).
static uint8_t g_refused = MOTOR_DRIVE_OK;
static uint8_t wantMode() {
  const bool want1 = P(P_MOT_DRIVE_MODE) >= 0.5f;
  const uint8_t r = motor_drive_refusal(want1, wantHz(), P(P_MOT_BRAKE_MAX_HZ), P(P_MOT_MODEL));
  if (r != MOTOR_DRIVE_OK && r != g_refused) g_cfg_changed = true;  // log the refusal once
  g_refused = r;
  return want1 && r == MOTOR_DRIVE_OK ? 1 : 0;
}

void motors_service() {
  if (g_test || !g_left.isOff() || !g_right.isOff()) return;
  const uint8_t mode = wantMode();
  if (wantHz() == g_hz && g_mode == mode) return;
  applyConfig(wantHz(), mode);
}

uint8_t motors_drive_mode_refused() { return g_refused; }

bool motors_test_begin(uint8_t drive_mode, float pwm_hz) {
  g_left.off();
  g_right.off();
  pwm_hz = validHz(pwm_hz);
  const uint8_t mode = drive_mode ? 1 : 0;
  if (mode == 1 && pwm_hz > P(P_MOT_BRAKE_MAX_HZ)) return false;  // second guard (motor_test.cpp checks first)
  applyConfig(pwm_hz, mode);
  g_test = true;
  return true;
}

void motors_test_end() {
  g_left.off();
  g_right.off();
  g_test = false;
  applyConfig(wantHz(), wantMode());
}

void motors_test_drive(float left_duty, float right_duty, uint8_t wheels) {
  Motor* const m[2] = {&g_left, &g_right};
  const float d[2] = {left_duty, right_duty};
  for (int i = 0; i < 2; i++) {
    const bool selected = wheels == 0 || wheels == (uint8_t)(i + 1);
    if (!g_test || !selected || isnan(d[i]) || d[i] == 0.0f) m[i]->off();
    else m[i]->setRaw(d[i]);
  }
}

bool motors_take_config_changed() {
  const bool c = g_cfg_changed;
  g_cfg_changed = false;
  return c;
}

uint8_t motors_drive_mode() { return g_mode; }
float motors_pwm_hz() { return g_hz; }

void motors_set(float left, float right) {
  if (g_test) return;  // the motor test holds the motors (and may be in drive mode 1)
  g_left.set(left);
  g_right.set(right);
}

void motors_speed_feedback(int32_t left_counts, int32_t right_counts, uint32_t t_us) {
  g_left.feedback(left_counts, t_us, odo_mm_per_count(false));
  g_right.feedback(right_counts, t_us, odo_mm_per_count(true));
}

void motors_stop() {
  g_left.off();
  g_right.off();
  motors_service();
}
float motors_left() { return g_left.command(); }
float motors_right() { return g_right.command(); }