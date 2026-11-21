// H-bridge drive (EN = PWM, DIR1/DIR2 = direction), 12-bit PWM at 36.6 kHz, with the v3
// dead-zone linearisation curve (mot.curve_exp / mot.knee_in / mot.knee_out).
#pragma once
#include <stdint.h>

// |in| 0..4095 -> PWM 0..4095. Below knee_in: knee_out * (in/knee_in)^(1/exp), above: linear to 4095.
float motor_curve(float in, float exp, float knee_in, float knee_out);

class Motor {
 public:
  Motor(uint8_t en, uint8_t d1, uint8_t d2) : en_(en), d1_(d1), d2_(d2) {}
  void begin();
  void set(float cmd);  // -4095..4095, + = forward
  float command() const { return cmd_; }

 private:
  uint8_t en_, d1_, d2_;
  float cmd_ = 0;
};

void motors_begin();
void motors_set(float left, float right);
void motors_stop();
float motors_left();   // last command, -4095..4095
float motors_right();
