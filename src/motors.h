// L298N H-bridge drive, 12-bit PWM at mot.pwm_hz (default 36.6 kHz), with a selectable
// correction (mot.model, maths in motor_model.h):
//   0  v3 dead-zone linearisation curve (mot.curve_exp / mot.knee_in / mot.knee_out), default
//   1  volts model: u = cmd/4095 -> V = dz + (v_ref - dz) * |u|, dz per wheel and direction
//      (mot.dz_l_fwd ...), straight taper 0 -> dz inside |u| < mot.dz_band; duty = V / v_ref
//      + stiction kick (issue #69, StictionKick): a stopped wheel gets its breakaway volts (mot.bk_*,
//      scaled by mot.bk_gain, 0 = off) for <= mot.bk_ms or until it turns, then fades back to the model
// mot.vcomp 1: duty *= v_ref / v_batt (filtered pack voltage, capped at 1.3x; 1 if the reading is bad),
// so the same u gives the same motor voltage at any charge. Works with both models.
// mot.speed_loop 1 (issue #26, experimental, default 0): the command is a wheel speed instead,
//   target = cmd/4095 * k * (v_ref - dz) mm/s, closed loop on the encoder speed (WheelSpeedLoop:
//   feed-forward from mot.k_l/k_r + mot.dz_*, PI mot.sl_kp / sl_ki, anti-windup mot.sl_imax).
//   Needs motors_speed_feedback() at the encoder rate; without fresh feedback (> 30 ms) it falls
//   back to mot.model for that command.
//
// Drive mode, for a command with duty d:
//   0  PWM on EN, IN1/IN2 static for the direction: off-time = both bridge sides off = coast (fast decay)
//   1  EN high, PWM on the IN pin of the direction, the other IN low: off-time = both INs low =
//      motor shorted through the low side = brake (slow decay). Command 0 = brake (hold).
// Mode 1 only at mot.pwm_hz <= mot.brake_max_hz (default 2000 Hz) and with mot.model 1; else
// mot.drive_mode 1 is refused (mode 0 is used, logged once): brake decay at high PWM frequency heats the
// L298N more (issues #23 / #26), and brake + the v3 curve is far too hot near 0 (issue #40). Balancing in
// mode 1: duty capped at mot.brake_duty_max (default 3000). Default mot.drive_mode 0.
// Either way the motor sees ~ d * V_pack. motors_stop() (IDLE, disarm, E-stop, faults) is always
// free-wheeling: EN low, both INs low, in both modes.
// Direction changes: the IN pin going low is switched to GPIO low at once, before the other one
// starts its PWM, so both INs are never high together.
//
// mot.pwm_hz is applied only while both motors are off (motors_stop() / motors_service()), never in
// the middle of a drive; motors_drive_mode() / motors_pwm_hz() are the values in use. Timers
// (Teensy 4.1 FlexPWM, one frequency per submodule):
//   pin 4 FlexPWM2.0 A, 5 FlexPWM2.1 A, 6 FlexPWM2.2 A, 9 FlexPWM2.2 B, 7 FlexPWM1.3 B, 8 FlexPWM1.3 A
// Also on those submodules: pins 33 (IMU_RST, GPIO) and 25 (unused) - never analogWrite them.
// The range servo (pin 2, 50 Hz) is on FlexPWM4.2, not affected.
#pragma once
#include <stdint.h>

#include "motor_model.h"

class Motor {
 public:
  Motor(uint8_t en, uint8_t d1, uint8_t d2, uint16_t dz_fwd, uint16_t dz_rev, uint16_t k, uint16_t dir,
        uint16_t bk_fwd, uint16_t bk_rev)
      : pin_{en, d1, d2}, dz_fwd_(dz_fwd), dz_rev_(dz_rev), k_(k), dir_(dir), bk_fwd_(bk_fwd), bk_rev_(bk_rev) {}
  void begin();
  void set(float cmd);       // -4095..4095, + = forward, through the correction (or the speed loop)
  void setRaw(float duty);   // -4095..4095 raw duty, no correction (motor test)
  void off();                // free-wheel: EN low, both INs low (any drive mode)
  bool isOff() const { return off_; }
  float command() const { return cmd_; }
  const uint8_t* pins() const { return pin_; }
  void feedback(int32_t counts, uint32_t t_us);  // encoder read (speed loop)
  float loopTarget() const { return sl_target_; }
  uint8_t kickState() const { return kick_.state(); }  // StictionKick IDLE / KICK / FADE

 private:
  enum : uint8_t { EN = 0, D1 = 1, D2 = 2 };
  void drive(float signed_pwm);
  void level(uint8_t i, bool high);  // pin i as GPIO at this level (leaves PWM cleanly)
  void pwm(uint8_t i, int duty);     // pin i as PWM with this duty
  bool speedLoop(float cmd, float& duty);
  bool feedbackFresh(uint32_t now_us) const;
  uint8_t pin_[3];
  bool is_pwm_[3] = {false, false, false};
  uint16_t dz_fwd_, dz_rev_, k_;  // ParamIds of this wheel's dead zones (V) and speed gain (mm/s/V)
  uint16_t dir_;                  // ParamId of this wheel's wiring direction (+1 normal, -1 swapped wires)
  uint16_t bk_fwd_, bk_rev_;      // ParamIds of this wheel's breakaway volts (stiction kick)
  float cmd_ = 0;
  bool off_ = true;
  WheelSpeedEst est_;
  WheelSpeedLoop loop_;
  StictionKick kick_;
  uint32_t loop_us_ = 0;
  bool loop_run_ = false;
  float sl_target_ = 0;
};

void motors_begin();
void motors_set(float left, float right);
void motors_set_raw(float left_duty, float right_duty);
// Motor test: wheels 0 both, 1 left only, 2 right only; an unselected wheel, or duty 0, is off()
// (EN low, INs low = free) - never driven, never braked.
void motors_test_drive(float left_duty, float right_duty, uint8_t wheels);
// Motor test start / end: both motors off, then the test's drive config is applied (mode 1 only up
// to mot.brake_max_hz: returns false = refused, nothing applied). While the test holds the motors
// motors_set() is ignored and motors_service() changes nothing. motors_test_end(): both off, back
// to the params' config.
bool motors_test_begin(uint8_t drive_mode, float pwm_hz);
void motors_test_end();
void motors_stop();     // both free-wheeling (EN low), then applies a pending pwm_hz / drive_mode
void motors_service();  // applies a pending pwm_hz / drive_mode if both motors are off (call often)
// mot.drive_mode 1 asked for but refused (mode 0 used): MotorDriveRefusal (motor_model.h), 0 = not refused
uint8_t motors_drive_mode_refused();
// True once after mot.pwm_hz / the drive mode were applied (for a log line).
bool motors_take_config_changed();
uint8_t motors_drive_mode();  // in use: 0 EN PWM (coast), 1 IN PWM (brake)
float motors_pwm_hz();        // in use
float motors_left();   // last command, -4095..4095
float motors_right();
// Encoder reads for mot.speed_loop (call after every odometry update).
void motors_speed_feedback(int32_t left_counts, int32_t right_counts, uint32_t t_us);
// Feed the pack voltage (call often; low-passed with a 1.5 s time constant). Valid 6..17 V.
void motors_set_vbatt(float v, uint32_t now_ms);
float motors_vbatt();   // filtered pack voltage, 0 = no valid reading
float motors_vscale();  // duty scale applied now (1 unless mot.vcomp = 1)
