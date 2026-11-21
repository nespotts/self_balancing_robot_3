// H-bridge drive (EN = PWM, DIR1/DIR2 = direction), 12-bit PWM at 36.6 kHz, with a selectable
// correction (mot.model):
//   0  v3 dead-zone linearisation curve (mot.curve_exp / mot.knee_in / mot.knee_out), default
//   1  volts model: u = cmd/4095 -> V = dz + (v_ref - dz) * |u|, dz per wheel and direction
//      (mot.dz_l_fwd ...), straight taper 0 -> dz inside |u| < mot.dz_band; duty = V / v_ref
// mot.vcomp 1: duty *= v_ref / v_batt (filtered pack voltage, capped at 1.3x; 1 if the reading is bad),
// so the same u gives the same motor voltage at any charge. Works with both models.
#pragma once
#include <stdint.h>

// |in| 0..4095 -> PWM 0..4095. Below knee_in: knee_out * (in/knee_in)^(1/exp), above: linear to 4095.
float motor_curve(float in, float exp, float knee_in, float knee_out);
// |u| 0..1 -> target motor volts (model 1).
float motor_dz_volts(float u_abs, float dz_v, float v_ref, float band);

class Motor {
 public:
  Motor(uint8_t en, uint8_t d1, uint8_t d2, uint16_t dz_fwd, uint16_t dz_rev)
      : en_(en), d1_(d1), d2_(d2), dz_fwd_(dz_fwd), dz_rev_(dz_rev) {}
  void begin();
  void set(float cmd);       // -4095..4095, + = forward, through the correction
  void setRaw(float duty);   // -4095..4095 raw duty, no correction (motor test)
  float command() const { return cmd_; }

 private:
  void drive(float signed_pwm);
  uint8_t en_, d1_, d2_;
  uint16_t dz_fwd_, dz_rev_;  // ParamId of this wheel's dead zones (V)
  float cmd_ = 0;
};

void motors_begin();
void motors_set(float left, float right);
void motors_set_raw(float left_duty, float right_duty);
void motors_stop();
float motors_left();   // last command, -4095..4095
float motors_right();
// Feed the pack voltage (call often; low-passed with a 1.5 s time constant). Valid 6..17 V.
void motors_set_vbatt(float v, uint32_t now_ms);
float motors_vbatt();   // filtered pack voltage, 0 = no valid reading
float motors_vscale();  // duty scale applied now (1 unless mot.vcomp = 1)
