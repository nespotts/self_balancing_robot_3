// Balance cascade, run once per IMU sample (ctl.rate_hz, default 400 Hz) with the real dt:
//   velocity PID (100 Hz) -> lean target -> balance PID -> power
//   heading PID (D from gyro yaw rate) -> differential power
// Same maths and units as the v3 br3ttb PIDs (P on error, D on measurement, I clamped to the
// output limits, ki per second, kd seconds). Balance D source = bal.d_src:
//   0 = d(pitch)/dt from consecutive samples (v3), 1 = gyro pitch rate.
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
  const CtlOut& last() const { return out_; }

 private:
  void reset(const CtlIn& in);
  bool upright_ = false;
  uint32_t start_ms_ = 0;
  float bal_i_ = 0, str_i_ = 0, vel_i_ = 0, vel_out_ = 0, vel_acc_dt_ = 0, last_speed_ = 0;
  float speed_sp_ = 0, turn_sp_ = 0, heading_sp_ = 0, last_pitch_ = 0;
  bool snap_heading_ = true;
  CtlOut out_;
};
