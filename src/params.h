// Named runtime parameters. Every gain/limit is here; the Pi reads them as ParamInfoFrames,
// sets them live with MSG_SET_PARAM, and "cfg.save" = 1 writes them to EEPROM.
// Ids are the row order below. Once released, append new params at the end to keep ids stable.
// Gains use br3ttb PID units: ki per second, kd in seconds (D = kd * rate in units/s).
#pragma once
#include <stdint.h>

//  id               name                  default    min      max
#define PARAM_TABLE(X)                                                   \
  X(BAL_KP,          "bal.kp",             250.0f,    0.0f,    2000.0f)  \
  X(BAL_KI,          "bal.ki",             2010.0f,   0.0f,    20000.0f) \
  X(BAL_KD,          "bal.kd",             4.63f,     0.0f,    50.0f)    \
  X(BAL_OFFSET,      "bal.offset_deg",     3.15f,     -15.0f,  15.0f)    \
  X(BAL_CUTOFF,      "bal.cutoff_deg",     45.0f,     5.0f,    80.0f)    \
  X(BAL_REENGAGE,    "bal.reengage_deg",   15.0f,     1.0f,    45.0f)    \
  X(BAL_KNOB_TRIM,   "bal.knob_trim_deg",  0.0f,      0.0f,    15.0f)    \
  X(BAL_D_SRC,       "bal.d_src",          0.0f,      0.0f,    1.0f)     \
  X(STR_KP,          "str.kp",             20.0f,     0.0f,    500.0f)   \
  X(STR_KI,          "str.ki",             5.9f,      0.0f,    500.0f)   \
  X(STR_KD,          "str.kd",             0.075f,    0.0f,    10.0f)    \
  X(STR_MAX_ERR,     "str.max_err_deg",    45.0f,     1.0f,    180.0f)   \
  X(VEL_KP,          "vel.kp",             0.0060f,   0.0f,    1.0f)     \
  X(VEL_KI,          "vel.ki",             0.00869f,  0.0f,    1.0f)     \
  X(VEL_KD,          "vel.kd",             0.00007f,  0.0f,    0.1f)     \
  X(VEL_LIMIT,       "vel.limit_deg",      20.0f,     0.0f,    30.0f)    \
  X(DRV_MAX_SPEED,   "drv.max_speed",      450.0f,    0.0f,    1500.0f)  \
  X(DRV_MAX_TURN,    "drv.max_turn",       180.0f,    0.0f,    720.0f)   \
  X(DRV_ACCEL,       "drv.accel",          1500.0f,   50.0f,   10000.0f) \
  X(DRV_TURN_ACCEL,  "drv.turn_accel",     1000.0f,   50.0f,   20000.0f) \
  X(DRV_TURN_DB,     "drv.turn_deadband",  2.0f,      0.0f,    30.0f)    \
  X(MOT_LEFT_SCALE,  "mot.left_scale",     0.95f,     0.5f,    1.5f)     \
  X(MOT_RIGHT_SCALE, "mot.right_scale",    1.0f,      0.5f,    1.5f)     \
  X(MOT_MAX,         "mot.max",            4095.0f,   0.0f,    4095.0f)  \
  X(MOT_CURVE_EXP,   "mot.curve_exp",      3.92f,     1.0f,    10.0f)    \
  X(MOT_KNEE_IN,     "mot.knee_in",        1250.0f,   1.0f,    4000.0f)  \
  X(MOT_KNEE_OUT,    "mot.knee_out",       2750.0f,   0.0f,    4095.0f)  \
  X(CTL_RATE_HZ,     "ctl.rate_hz",        400.0f,    100.0f,  1000.0f)  \
  X(CTL_LOOPS,       "ctl.loops",          3.0f,      0.0f,    3.0f)     \
  X(CTL_SETTLE_MS,   "ctl.settle_ms",      500.0f,    0.0f,    5000.0f)  \
  X(BATT_LOW_CELL,   "batt.low_cell",      3.4f,      2.5f,    4.2f)     \
  X(BATT_ADC_PER_V,  "batt.adc_per_v",     73.3182f,  10.0f,   200.0f)   \
  X(TLM_HZ,          "tlm.hz",             100.0f,    1.0f,    1000.0f)  \
  X(RNG_ENABLE,      "rng.enable",         0.0f,      0.0f,    1.0f)     \
  X(CFG_SAVE,        "cfg.save",           0.0f,      0.0f,    1.0f)     \
  X(CFG_DEFAULTS,    "cfg.defaults",       0.0f,      0.0f,    1.0f)     \
  X(RNG_SERVO_TRIM,  "rng.servo_trim_deg", 0.0f,      -45.0f,  45.0f)    \
  X(RNG_SWEEP_DEG,   "rng.sweep_deg",      170.0f,    0.0f,    180.0f)   \
  X(RNG_STEPS,       "rng.steps",          16.0f,     1.0f,    64.0f)    \
  X(RNG_SETTLE_MS,   "rng.settle_ms",      40.0f,     0.0f,    500.0f)   \
  X(RNG_BUDGET_MS,   "rng.timing_budget_ms", 33.0f,   20.0f,   200.0f)   \
  X(RNG_LONG_RANGE,  "rng.long_range",     1.0f,      0.0f,    1.0f)     \
  X(RNG_MAX_MM,      "rng.max_mm",         2000.0f,   50.0f,   8190.0f)  \
  X(RNG_SERVO_HOLD,  "rng.servo_hold_deg", -1.0f,     -1.0f,   180.0f)   \
  X(RNG_OBSTACLE_MM, "rng.obstacle_mm",    300.0f,    0.0f,    2000.0f)  \
  X(RNG_SERVO_DIR,   "rng.servo_dir",      1.0f,      -1.0f,   1.0f)     \
  X(RNG_SERVO_MIN,   "rng.servo_min_deg",  5.0f,      0.0f,    90.0f)    \
  X(RNG_SERVO_MAX,   "rng.servo_max_deg",  175.0f,    90.0f,   180.0f)   \
  X(RNG_SERVO_SCALE, "rng.servo_deg_per_cmd", 1.0f,   0.5f,    1.5f)  \
  /* motor test + correction (issue #16, see motors.h / motor_test.h) */ \
  X(MOT_TEST,        "mot.test",           0.0f,      0.0f,    7.0f)     \
  X(MOT_TEST_MAX,    "mot.test_max",       4095.0f,   500.0f,  4095.0f)  \
  X(MOT_MODEL,       "mot.model",          0.0f,      0.0f,    1.0f)     \
  X(MOT_VCOMP,       "mot.vcomp",          0.0f,      0.0f,    1.0f)     \
  X(MOT_V_REF,       "mot.v_ref",          12.0f,     6.0f,    17.0f)    \
  X(MOT_DZ_L_FWD,    "mot.dz_l_fwd",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_L_REV,    "mot.dz_l_rev",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_R_FWD,    "mot.dz_r_fwd",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_R_REV,    "mot.dz_r_rev",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_BAND,     "mot.dz_band",        0.01f,     0.0f,    0.2f)     \
  /* range sweep idle off (issue #20, see range_sweep.h) */             \
  X(RNG_IDLE_OFF,    "rng.idle_off",       1.0f,      0.0f,    1.0f)     \
  /* L298N drive experiments (issue #22, see motors.h); applied while the motors are off */ \
  X(MOT_PWM_HZ,      "mot.pwm_hz",         36621.09f, 100.0f,  50000.0f) \
  /* drive_mode 1 (PWM on IN, brake decay): only at mot.pwm_hz <= mot.brake_max_hz, else mode 0   */ \
  /* (issue #23: the smoke was the binding left motor at 36.6 kHz; #26)                           */ \
  X(MOT_DRIVE_MODE,  "mot.drive_mode",     0.0f,      0.0f,    1.0f)     \
  /* motor profile (issue #26, see motor_test.h / motor_model.h) */      \
  X(MOT_TEST_WHEELS, "mot.test_wheels",    0.0f,      0.0f,    2.0f)     \
  X(MOT_BRAKE_MAX_HZ, "mot.brake_max_hz",  2000.0f,   100.0f,  10000.0f) \
  X(MOT_BRAKE_DUTY_MAX, "mot.brake_duty_max", 3000.0f, 500.0f, 4095.0f)  \
  X(MOT_SPEED_LOOP,  "mot.speed_loop",     0.0f,      0.0f,    1.0f)     \
  X(MOT_SL_KP,       "mot.sl_kp",          0.006f,    0.0f,    0.1f)     \
  X(MOT_SL_KI,       "mot.sl_ki",          0.08f,     0.0f,    2.0f)     \
  X(MOT_SL_IMAX,     "mot.sl_imax",        3.0f,      0.0f,    12.0f)    \
  X(MOT_SL_BAND,     "mot.sl_band",        0.3f,      0.0f,    3.0f)     \
  X(MOT_K_L,         "mot.k_l",            200.0f,    10.0f,   2000.0f)  \
  X(MOT_K_R,         "mot.k_r",            200.0f,    10.0f,   2000.0f) \
  /* navigation: GOTO / EXPLORE / HOLD (issue #8, see navigator.h) */  \
  X(NAV_MAX_SPEED,   "nav.max_speed",      250.0f,    0.0f,    1500.0f)  \
  X(NAV_MAX_TURN,    "nav.max_turn",       90.0f,     0.0f,    720.0f)   \
  X(NAV_SPEED_KP,    "nav.speed_kp",       0.6f,      0.0f,    10.0f)    \
  X(NAV_MIN_SPEED,   "nav.min_speed",      40.0f,     0.0f,    300.0f)   \
  X(NAV_TURN_KP,     "nav.turn_kp",        2.0f,      0.0f,    20.0f)    \
  X(NAV_ALIGN_DEG,   "nav.align_deg",      10.0f,     1.0f,    60.0f)    \
  X(NAV_ARRIVE_MM,   "nav.arrive_mm",      50.0f,     5.0f,    500.0f)   \
  X(NAV_STOP_SPEED,  "nav.stop_speed",     50.0f,     1.0f,    300.0f)   \
  X(NAV_STOP_TURN,   "nav.stop_turn_dps",  20.0f,     1.0f,    180.0f)   \
  X(NAV_DWELL_MS,    "nav.dwell_ms",       500.0f,    0.0f,    10000.0f) \
  X(NAV_END_HEADING, "nav.end_heading_deg", -1.0f,    -1.0f,   360.0f)   \
  X(NAV_HEADING_TOL, "nav.heading_tol_deg", 3.0f,     0.5f,    45.0f)    \
  X(NAV_SLOW_MM,     "nav.slow_mm",        800.0f,    0.0f,    3000.0f)  \
  X(NAV_CLEAR_MM,    "nav.clear_mm",       100.0f,    0.0f,    1000.0f)  \
  X(NAV_CORRIDOR_MM, "nav.corridor_mm",    220.0f,    50.0f,   1000.0f)  \
  X(NAV_OBST_AGE_MS, "nav.obst_age_ms",    1500.0f,   100.0f,  10000.0f) \
  X(NAV_FLOOR_MM,    "nav.floor_mm",       50.0f,     0.0f,    200.0f)   \
  X(NAV_BACKOFF_MM,  "nav.backoff_mm",     0.0f,      0.0f,    500.0f)   \
  X(NAV_HOLD_KP,     "nav.hold_kp",        2.0f,      0.0f,    20.0f)    \
  X(NAV_HOLD_MAX,    "nav.hold_max_speed", 150.0f,    0.0f,    1000.0f)  \
  X(NAV_HOLD_DB_MM,  "nav.hold_db_mm",     15.0f,     0.0f,    200.0f)   \
  /* safety review #38: GOTO without range readings (rng.enable 0): speed cap, 0 = refuse */ \
  X(NAV_BLIND_SPEED, "nav.blind_speed",    150.0f,    0.0f,    1500.0f)  \
  /* motor wiring: -1 = that motor's wires are swapped; fixed in software at the lowest level */ \
  X(MOT_LEFT_DIR,    "mot.left_dir",       1.0f,      -1.0f,   1.0f)     \
  X(MOT_RIGHT_DIR,   "mot.right_dir",      1.0f,      -1.0f,   1.0f)

enum ParamId : uint16_t {
#define PARAM_ENUM(id, name, def, mn, mx) P_##id,
  PARAM_TABLE(PARAM_ENUM)
#undef PARAM_ENUM
  P_COUNT
};

struct ParamDef {
  const char* name;
  float def, min, max;
};

extern float g_param[P_COUNT];
inline float P(ParamId id) { return g_param[id]; }

const ParamDef& params_def(uint16_t id);
// Clamp to [min, max] and apply. Returns false for a bad id or NaN.
// cfg.save / cfg.defaults are actions: they set the flags below and read back as 0.
bool params_set(uint16_t id, float value);
void params_reset_defaults();
// Returns true if valid values were loaded from EEPROM, false if defaults were used.
bool params_load();
// Writes EEPROM. Blocks for a few ms (flash) - only call with the motors off.
bool params_save();

extern bool g_param_save_requested;
extern bool g_param_defaults_requested;
