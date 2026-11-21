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
  X(MOT_TEST,        "mot.test",           0.0f,      0.0f,    4.0f)     \
  X(MOT_TEST_MAX,    "mot.test_max",       4095.0f,   500.0f,  4095.0f)  \
  X(MOT_MODEL,       "mot.model",          0.0f,      0.0f,    1.0f)     \
  X(MOT_VCOMP,       "mot.vcomp",          0.0f,      0.0f,    1.0f)     \
  X(MOT_V_REF,       "mot.v_ref",          12.0f,     6.0f,    17.0f)    \
  X(MOT_DZ_L_FWD,    "mot.dz_l_fwd",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_L_REV,    "mot.dz_l_rev",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_R_FWD,    "mot.dz_r_fwd",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_R_REV,    "mot.dz_r_rev",       2.0f,      0.0f,    8.0f)     \
  X(MOT_DZ_BAND,     "mot.dz_band",        0.01f,     0.0f,    0.2f)

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
