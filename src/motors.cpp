#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "params.h"

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

void Motor::set(float cmd) {
  if (isnan(cmd)) cmd = 0;
  cmd_ = cmd;
  if (cmd > 0) {
    digitalWriteFast(d1_, HIGH);
    digitalWriteFast(d2_, LOW);
  } else if (cmd < 0) {
    digitalWriteFast(d1_, LOW);
    digitalWriteFast(d2_, HIGH);
  } else {
    digitalWriteFast(d1_, LOW);
    digitalWriteFast(d2_, LOW);
  }
  float pwm = motor_curve(cmd, P(P_MOT_CURVE_EXP), P(P_MOT_KNEE_IN), P(P_MOT_KNEE_OUT));
  analogWrite(en_, (int)(pwm + 0.5f));
}

static Motor g_left(pins::MOT_L_EN, pins::MOT_L_D1, pins::MOT_L_D2);
static Motor g_right(pins::MOT_R_EN, pins::MOT_R_D1, pins::MOT_R_D2);

void motors_begin() {
  analogWriteResolution(12);
  g_left.begin();
  g_right.begin();
}

void motors_set(float left, float right) {
  g_left.set(left);
  g_right.set(right);
}

void motors_stop() { motors_set(0, 0); }
float motors_left() { return g_left.command(); }
float motors_right() { return g_right.command(); }
