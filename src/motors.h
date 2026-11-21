// L298N H-bridge drive, 12-bit PWM at mot.pwm_hz (default 36.6 kHz), with a selectable
// correction (mot.model):
//   0  v3 dead-zone linearisation curve (mot.curve_exp / mot.knee_in / mot.knee_out), default
//   1  volts model: u = cmd/4095 -> V = dz + (v_ref - dz) * |u|, dz per wheel and direction
//      (mot.dz_l_fwd ...), straight taper 0 -> dz inside |u| < mot.dz_band; duty = V / v_ref
// mot.vcomp 1: duty *= v_ref / v_batt (filtered pack voltage, capped at 1.3x; 1 if the reading is bad),
// so the same u gives the same motor voltage at any charge. Works with both models.
//
// Drive mode (mot.drive_mode, issue #22), for a command with duty d:
//   0  PWM on EN, IN1/IN2 static for the direction: off-time = both bridge sides off = coast (fast decay)
//   1  EN high, PWM on the IN pin of the direction, the other IN low: off-time = both INs low =
//      motor shorted through the low side = brake (slow decay). Command 0 = brake (hold).
// Either way the motor sees ~ d * V_pack. motors_stop() (IDLE, disarm, E-stop, faults) is always
// free-wheeling: EN low, both INs low, in both modes.
// Direction changes: the IN pin going low is switched to GPIO low at once, before the other one
// starts its PWM, so both INs are never high together.
//
// mot.pwm_hz and mot.drive_mode are applied only while both motors are off (motors_stop() /
// motors_service()), never in the middle of a drive; motors_drive_mode() / motors_pwm_hz() are the
// values in use. Timers (Teensy 4.1 FlexPWM, one frequency per submodule):
//   pin 4 FlexPWM2.0 A, 5 FlexPWM2.1 A, 6 FlexPWM2.2 A, 9 FlexPWM2.2 B, 7 FlexPWM1.3 B, 8 FlexPWM1.3 A
// Also on those submodules: pins 33 (IMU_RST, GPIO) and 25 (unused) - never analogWrite them.
// The range servo (pin 2, 50 Hz) is on FlexPWM4.2, not affected.
#pragma once
#include <stdint.h>

// |in| 0..4095 -> PWM 0..4095. Below knee_in: knee_out * (in/knee_in)^(1/exp), above: linear to 4095.
float motor_curve(float in, float exp, float knee_in, float knee_out);
// |u| 0..1 -> target motor volts (model 1).
float motor_dz_volts(float u_abs, float dz_v, float v_ref, float band);

class Motor {
 public:
  Motor(uint8_t en, uint8_t d1, uint8_t d2, uint16_t dz_fwd, uint16_t dz_rev)
      : pin_{en, d1, d2}, dz_fwd_(dz_fwd), dz_rev_(dz_rev) {}
  void begin();
  void set(float cmd);       // -4095..4095, + = forward, through the correction
  void setRaw(float duty);   // -4095..4095 raw duty, no correction (motor test)
  void off();                // free-wheel: EN low, both INs low (any drive mode)
  bool isOff() const { return off_; }
  float command() const { return cmd_; }
  const uint8_t* pins() const { return pin_; }

 private:
  enum : uint8_t { EN = 0, D1 = 1, D2 = 2 };
  void drive(float signed_pwm);
  void level(uint8_t i, bool high);  // pin i as GPIO at this level (leaves PWM cleanly)
  void pwm(uint8_t i, int duty);     // pin i as PWM with this duty
  uint8_t pin_[3];
  bool is_pwm_[3] = {false, false, false};
  uint16_t dz_fwd_, dz_rev_;  // ParamId of this wheel's dead zones (V)
  float cmd_ = 0;
  bool off_ = true;
};

void motors_begin();
void motors_set(float left, float right);
void motors_set_raw(float left_duty, float right_duty);
void motors_stop();     // both free-wheeling (EN low), then applies a pending pwm_hz / drive_mode
void motors_service();  // applies a pending pwm_hz / drive_mode if both motors are off (call often)
// True once after mot.pwm_hz / mot.drive_mode were applied (for a log line).
bool motors_take_config_changed();
uint8_t motors_drive_mode();  // in use: 0 EN PWM (coast), 1 IN PWM (brake)
float motors_pwm_hz();        // in use
float motors_left();   // last command, -4095..4095
float motors_right();
// Feed the pack voltage (call often; low-passed with a 1.5 s time constant). Valid 6..17 V.
void motors_set_vbatt(float v, uint32_t now_ms);
float motors_vbatt();   // filtered pack voltage, 0 = no valid reading
float motors_vscale();  // duty scale applied now (1 unless mot.vcomp = 1)
