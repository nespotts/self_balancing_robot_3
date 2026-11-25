// Balance cascade, run once per IMU sample (ctl.rate_hz, default 400 Hz) with the real dt:
//   velocity PID (100 Hz) -> lean target -> balance PID -> power
//   heading PID (D from gyro yaw rate) -> differential power
// Same maths and units as the v3 br3ttb PIDs (P on error, D on measurement, I clamped to the
// output limits, ki per second, kd seconds). Balance D source = bal.d_src:
//   0 = d(pitch)/dt from consecutive samples (v3), 1 = gyro pitch rate.
// Velocity-loop anti-windup (#111): pushing against something the wheels can't pass, the integrator would lean the
// robot harder and harder until it falls. Blocked = |ramped command| >= kBlkMinCmd and the mean |speed| (~0.2 s)
// below kBlkStillMms; after kBlkMs of that the integrator leans at most kBlkLeanDeg further toward the command than its
// last free value (the P term still pushes: vel.kp x command), CtlOut.held = at that limit. Free value = the integrator
// while the robot follows a steady command (mean |speed| within kFreeErr of it, the ramped command not changing) or is
// not commanded to move: the robot's own balance trim (the real one sat at -2..-4 deg), not a lean to speed up. contactReset() (the navigator found a contact): the ramped command to
// 0 and the integrator back to its free value, so the robot stands up instead of leaning on the obstacle (and does not
// rock back into it when it backs off). Normal driving and balancing never meet the limit (same sim runs, #111).
// Pure logic (no Arduino calls) so it can be unit-tested on a host.
#pragma once
#include <stdint.h>

struct CtlIn {
  float dt = 0.0025f;          // s, real time since the previous sample
  uint32_t now_ms = 0;
  float pitch = 0, pitch_rate = 0, roll = 0;  // deg, deg/s; pitch + = leaning forward
  float heading = 0, yaw_rate = 0;            // deg (continuous), deg/s, + = CCW
  float speed_mms = 0;                        // measured forward speed
  bool speed_ok = true;                       // encoders healthy (else the velocity loop is off)
  float target_speed = 0, target_turn = 0;    // mm/s, deg/s from the mode manager
  float trim_deg = 0;                         // extra balance offset (knob trim)
  bool enable = false;                        // mode allows motors
};

struct CtlOut {
  float left = 0, right = 0;   // -mot.max..mot.max, scaled per side
  float balance_target = 0;    // pitch the balance loop is holding, deg
  float speed_sp = 0, turn_sp = 0;  // ramped setpoints actually used
  float heading_sp = 0;
  float bal_p = 0, bal_i = 0, bal_d = 0;  // balance PID terms, counts (before clamping the sum)
  float steer = 0;             // heading PID output, counts
  float vel_out = 0;           // velocity loop output, deg of lean
  bool held = false;           // anti-windup: blocked, the integrator held at its limit (#111)
  bool upright = false;       // balancing (not fallen / not waiting to re-engage)
  bool active = false;         // motors should be driven with left/right
};

class Controller {
 public:
  CtlOut step(const CtlIn& in);
  // Mode changed: forget the heading target (no turning back to an old heading).
  void onModeChange() { snap_heading_ = true; }
  // Heading frame shifted by delta (pose reset): move the target with it.
  void shiftHeading(float delta) { heading_sp_ += delta; }
  // Motors were stopped outside step() (IMU dropout / stale, disarm): forget being upright, the
  // integrators, ramps and D history, so the next step must re-engage inside bal.reengage_deg and
  // goes through ctl.settle_ms again (issue #39).
  void disengage();
  // A contact (#111, the navigator): ramped command to 0, velocity integrator back to its free value.
  void contactReset();
  const CtlOut& last() const { return out_; }

  // anti-windup (#111, compile-time)
  static constexpr float kBlkMinCmd = 30.0f;    // mm/s
  static constexpr float kBlkStillMms = 15.0f;  // mean |speed| below this: the wheels are not turning
  static constexpr float kBlkTauS = 0.2f;       // ... mean over about this long
  static constexpr uint32_t kBlkMs = 300;       // blocked this long: the limit applies (contacts trip at >= 0.4 s)
  static constexpr float kBlkLeanDeg = 1.0f;    // the integrator's lean beyond its free value, toward the command
  static constexpr float kFreeErr = 0.25f;      // free value: mean |speed| within this share of the command ...
  static constexpr float kFreeAcc = 50.0f;      // ... and the ramped command changing slower than this (mm/s^2)

 private:
  void reset(const CtlIn& in);
  void clearBlock() { blk_ms_ = 0; held_ = false; }
  bool upright_ = false;
  uint32_t start_ms_ = 0;
  float bal_i_ = 0, str_i_ = 0, vel_i_ = 0, vel_out_ = 0, vel_acc_dt_ = 0, last_speed_ = 0;
  float speed_sp_ = 0, turn_sp_ = 0, heading_sp_ = 0, last_pitch_ = 0;
  bool snap_heading_ = true;
  // anti-windup (#111)
  float spd_abs_ = 0;     // mean |speed| (kBlkTauS)
  float free_i_ = 0;      // the integrator while following a steady command / not commanded
  float last_sp_ = 0;     // speed_sp_ at the last velocity-loop step
  uint32_t blk_ms_ = 0;   // blocked since (0 = not)
  float blk_d_ = 1;       // ... the command's direction
  bool held_ = false;
  CtlOut out_;
};
