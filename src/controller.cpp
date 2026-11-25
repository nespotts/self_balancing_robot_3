#include "controller.h"

#include <math.h>

#include "config.h"
#include "params.h"

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static float slew(float cur, float target, float max_step) {
  if (target > cur + max_step) return cur + max_step;
  if (target < cur - max_step) return cur - max_step;
  return target;
}

void Controller::reset(const CtlIn& in) {
  bal_i_ = str_i_ = vel_i_ = vel_out_ = vel_acc_dt_ = 0;
  speed_sp_ = turn_sp_ = 0;
  heading_sp_ = in.heading;
  last_speed_ = in.speed_mms;
  last_pitch_ = in.pitch;
  spd_abs_ = free_i_ = last_sp_ = 0;
  clearBlock();
}

void Controller::contactReset() {
  speed_sp_ = last_sp_ = 0;
  vel_i_ = free_i_;  // (vel_out_ follows at the next velocity-loop step, <= 10 ms)
  clearBlock();
}

void Controller::disengage() {
  upright_ = false;
  bal_i_ = str_i_ = vel_i_ = vel_out_ = vel_acc_dt_ = 0;
  speed_sp_ = turn_sp_ = 0;
  spd_abs_ = free_i_ = last_sp_ = 0;
  clearBlock();
  snap_heading_ = true;  // last_pitch_ / last_speed_ / heading_sp_ are re-taken by reset() on re-engage
  out_.upright = false;
  out_.active = false;
  out_.left = out_.right = 0;
}

CtlOut Controller::step(const CtlIn& in) {
  CtlOut o;
  const float offset = P(P_BAL_OFFSET) + in.trim_deg;  // balance point is pitch = -offset
  const float cutoff = P(P_BAL_CUTOFF);
  const float mot_max = P(P_MOT_MAX);
  const bool too_far = fabsf(in.pitch) > cutoff || fabsf(in.roll) > cutoff;

  if (snap_heading_) {
    heading_sp_ = in.heading;
    str_i_ = 0;
    snap_heading_ = false;
  }

  // --- fallen / re-engage
  if (!in.enable || (upright_ && too_far)) {
    upright_ = false;
    reset(in);
  }
  if (!upright_) {
    if (in.enable && !too_far && fabsf(in.pitch + offset) < P(P_BAL_REENGAGE)) {
      upright_ = true;
      start_ms_ = in.now_ms;
      reset(in);
    } else {
      o.balance_target = -offset;
      o.heading_sp = heading_sp_ = in.heading;
      out_ = o;
      return o;  // motors off
    }
  }

  const bool settled = (in.now_ms - start_ms_) >= (uint32_t)P(P_CTL_SETTLE_MS);
  const int loops = (int)lroundf(P(P_CTL_LOOPS));
  const float dt = in.dt;

  // --- ramped setpoints (also does the failsafe / mode-change ramp to zero)
  speed_sp_ = slew(speed_sp_, settled ? in.target_speed : 0.0f, P(P_DRV_ACCEL) * dt);
  turn_sp_ = slew(turn_sp_, settled ? in.target_turn : 0.0f, P(P_DRV_TURN_ACCEL) * dt);

  // --- velocity loop, 100 Hz: speed error -> lean angle
  if ((loops & 2) && settled && in.speed_ok) {
    vel_acc_dt_ += dt;
    if (vel_acc_dt_ >= VEL_LOOP_DT) {
      const float vdt = vel_acc_dt_;
      vel_acc_dt_ = 0;
      const float lim = P(P_VEL_LIMIT);
      const float e = speed_sp_ - in.speed_mms;
      // anti-windup (#111): blocked = commanded but the wheels not turning (header)
      const float c = fabsf(speed_sp_), d = speed_sp_ >= 0 ? 1.0f : -1.0f;
      spd_abs_ += (fabsf(in.speed_mms) - spd_abs_) * fminf(1.0f, vdt / kBlkTauS);
      const float sp_acc = (speed_sp_ - last_sp_) / vdt;
      last_sp_ = speed_sp_;
      // free value: not commanded, or following a steady command (not speeding up / slowing down: that lean is not
      // the robot's trim)
      if (c < kBlkMinCmd || (fabsf(spd_abs_ - c) < kFreeErr * c && fabsf(sp_acc) < kFreeAcc)) free_i_ = vel_i_;
      if (c < kBlkMinCmd || spd_abs_ >= kBlkStillMms || (blk_ms_ && d != blk_d_)) {
        clearBlock();
      } else if (!blk_ms_) {
        blk_ms_ = in.now_ms ? in.now_ms : 1;
        blk_d_ = d;
      }
      vel_i_ = clampf(vel_i_ + P(P_VEL_KI) * e * vdt, lim);
      if (blk_ms_ && in.now_ms - blk_ms_ >= kBlkMs) {
        const float top = free_i_ + blk_d_ * kBlkLeanDeg;
        if (blk_d_ * (vel_i_ - top) >= 0) {
          vel_i_ = top;
          held_ = true;
        } else {
          held_ = false;
        }
      }
      const float accel = (in.speed_mms - last_speed_) / vdt;  // D on measurement
      last_speed_ = in.speed_mms;
      vel_out_ = clampf(P(P_VEL_KP) * e + vel_i_ - P(P_VEL_KD) * accel, lim);
    }
  } else {
    vel_out_ = vel_i_ = vel_acc_dt_ = 0;
    last_speed_ = in.speed_mms;
    spd_abs_ = free_i_ = last_sp_ = 0;
    clearBlock();
  }

  // --- balance loop: lean error -> power.
  // v3 was br3ttb with input = -pitch, setpoint = offset - vel_out, so
  // error = pitch + offset - vel_out and D = -kd * d(input)/dt = +kd * d(pitch)/dt.
  const float target = vel_out_ - offset;
  const float e = in.pitch - target;
  const float rate = P(P_BAL_D_SRC) >= 0.5f ? in.pitch_rate : (in.pitch - last_pitch_) / dt;
  last_pitch_ = in.pitch;
  bal_i_ = clampf(bal_i_ + P(P_BAL_KI) * e * dt, mot_max);
  o.bal_p = P(P_BAL_KP) * e;
  o.bal_i = bal_i_;
  o.bal_d = P(P_BAL_KD) * rate;
  const float bal = clampf(o.bal_p + o.bal_i + o.bal_d, mot_max);

  // --- heading loop: integrate the turn rate into a heading target
  float steer = 0;
  if ((loops & 1) && settled) {
    heading_sp_ += turn_sp_ * dt;
    const float max_err = P(P_STR_MAX_ERR);
    if (heading_sp_ > in.heading + max_err) heading_sp_ = in.heading + max_err;
    if (heading_sp_ < in.heading - max_err) heading_sp_ = in.heading - max_err;
    const float eh = heading_sp_ - in.heading;
    str_i_ = clampf(str_i_ + P(P_STR_KI) * eh * dt, mot_max);
    steer = clampf(P(P_STR_KP) * eh + str_i_ - P(P_STR_KD) * in.yaw_rate, mot_max);
  } else {
    heading_sp_ = in.heading;
    str_i_ = 0;
  }

  // + steer = right wheel faster = turn CCW
  o.left = clampf((bal - steer) * P(P_MOT_LEFT_SCALE), mot_max);
  o.right = clampf((bal + steer) * P(P_MOT_RIGHT_SCALE), mot_max);
  o.balance_target = target;
  o.speed_sp = speed_sp_;
  o.turn_sp = turn_sp_;
  o.heading_sp = heading_sp_;
  o.steer = steer;
  o.vel_out = vel_out_;
  o.held = held_;
  o.upright = true;
  o.active = true;
  out_ = o;
  return o;
}
