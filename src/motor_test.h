// Wheels-up motor characterisation (issues #16, #26). Pure logic: no Arduino calls, time is passed
// in, so it also builds on a PC (test/host/). main.cpp feeds it once per background pass and drives
// the motors with raw duty while driving() is true (no correction curve, no battery compensation):
// motors_test_begin(driveMode(), pwmHz()) when it starts, motors_test_drive() while it runs,
// motors_test_end() when it stops.
//
//   mot.test 1 = breakaway / stop thresholds (slow ramps)   2 = steady speed vs duty (16 steps)
//            3 = reversal lag / backlash                   4 = 1 + 2 + 3
//            5 = low-speed tracking, open loop: model 0 (v3 curve), then model 1 (volts model)
//            6 = low-speed tracking, closed loop (WheelSpeedLoop, motor_model.h)
//            7 = 5 + 6
//   mot.test_wheels 0 both, 1 left only, 2 right only: the other wheel stays off (EN low, INs low).
// Bench supply without the balance lead (pack reading < 6 V): set mot.v_ref to the supply voltage.
//
// Tracking profile (tests 5-7, each controller, per wheel): r+ 0 -> 300 -> 0 mm/s (3 s ramps, 1 s
// hold), r- the same backwards, s0 50 mm/s sine around 0 (1 Hz, 4 s), s150 150 +- 50 mm/s sine
// (1 Hz, 4 s), st steps 150 -> 250 -> 150 mm/s (t63 response). Models from the params: mot.k_l/k_r
// (mm/s per V above the dead zone), mot.dz_*, mot.v_ref, mot.dz_band, v3 curve; loop mot.sl_*.
//
// Runs only when: requested, remote ARMED, mode IDLE (balance controller off), remote link live,
// IMU + encoders ok, |pitch| and |roll| < 20 deg, still, for
// 1 s. Any of those failing, the request going back to 0, or a timeout -> motors 0 at once,
// "mt,abort,<reason>". Anomaly aborts (binding / heat / supply):
//   stall       a driven wheel below 8 mm/s for 1 s while its volts are above its start threshold
//               (this run's test 1 start + 1 V, else max(8.5 V, mot.dz + 2 V))
//   speed_drop  at a fixed duty the speed falls > 30 % below the first measurement of that step
//               (test 2: within each 1.2 s level; test 3: each spin vs the first one that way)
//   supply_sag  pack voltage > 1.5 V below its value at the start, for 0.2 s (skipped when the run
//               started without a valid reading: then volts use mot.v_ref, flagged vbsrc=v_ref)
//   drive_time  one stretch of continuous drive > 60 s (drive mode 1: 25 s)
// Drive mode 1 (brake decay) guards: refused above mot.brake_max_hz; duty capped at
// mot.brake_duty_max; 60 s cool-down (motors off) between test parts, 10 s between directions,
// 5 s between tracking segments.
// Results are MSG_LOG lines "mt,<kind>,..." (format: docs/firmware.md, parser: tools/motor_fit.py).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "motor_model.h"

struct MtIn {
  uint32_t now_us = 0;
  uint8_t request = 0;  // mot.test
  uint8_t wheels = 0;   // mot.test_wheels: 0 both, 1 left, 2 right
  bool armed = false, estop = false, idle = false, radio_live = false, imu_ok = false, enc_ok = false;
  float pitch = 0, roll = 0, pitch_rate = 0;  // deg, deg/s
  int32_t enc_l = 0, enc_r = 0;               // encoder counts (4096 / wheel rev)
  uint32_t enc_us = 0;                        // time of the last encoder read
  float speed_l = 0, speed_r = 0;             // mm/s (80 ms window)
  float vbatt = 0;                            // pack voltage (filtered), V
  float max_duty = 4095;                      // mot.test_max
  float pwm_hz = 36621.09f;                   // drive config the test will use (mot.pwm_hz,
  uint8_t drive_mode = 0;                     //   mot.drive_mode), recorded as hz=<Hz>,drive=<0|1>
  float brake_max_hz = 2000;                  // mot.brake_max_hz
  float brake_duty_max = 3000;                // mot.brake_duty_max
  // models for the tracking tests (params)
  float curve_exp = 3.92f, knee_in = 1250, knee_out = 2750;
  float v_ref = 12, dz_band = 0.01f;
  float dz[2][2] = {{2, 2}, {2, 2}};  // [wheel][0 fwd, 1 rev], V
  float k[2] = {200, 200};            // mm/s per V above the dead zone
  SpeedLoopCfg sl;                    // kp, ki, i_max, band (dz / k / v_max filled per wheel)
};

class MotorTest {
 public:
  static constexpr float kMmPerCount = 3.14159265f * 66.0f / 4096.0f;  // config.h WHEEL_DIAMETER_MM

  void step(const MtIn& raw);
  bool driving() const { return state_ == S_RUN; }  // the test owns the motors
  bool pending() const { return state_ != S_IDLE; }
  float left() const { return duty_[0]; }           // signed raw duty, -4095..4095
  float right() const { return duty_[1]; }
  uint8_t wheels() const { return wheels_; }        // selection of the running test
  uint8_t driveMode() const { return (uint8_t)drive_; }
  float pwmHz() const { return hz_; }
  // main.cpp could not apply the drive config: stop with this reason
  void abort(const char* why, uint32_t now_us);
  // True once when the test ended (finished / aborted / cancelled): set mot.test back to 0.
  bool takeDone() {
    bool d = done_;
    done_ = false;
    return d;
  }
  // Result lines, oldest first. peek, send, then pop (so a full USB buffer loses nothing).
  const char* peekLine() const { return q_n_ ? q_[q_head_] : nullptr; }
  void popLine();
  uint32_t droppedLines() const { return q_drop_; }

 private:
  enum State : uint8_t { S_IDLE, S_WAIT, S_RUN };
  static constexpr int kQ = 48, kLine = 176, kTrace = 260;

  void emit(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  const char* startBlocker(const MtIn& in) const;
  const char* abortReason(const MtIn& in) const;
  const char* anomaly(const MtIn& in);
  void stopAll(bool done);
  void hold(uint32_t ms, const MtIn& in);  // motors 0 for ms, then the current part continues
  void startTest(const MtIn& in);
  void nextPart(const MtIn& in);
  void stepPart(const MtIn& in);
  void endPart(const MtIn& in);
  void runA(const MtIn& in, bool new_enc);
  void runB(const MtIn& in, bool new_enc);
  void runC(const MtIn& in, bool new_enc);
  void runL(const MtIn& in, bool new_enc);
  void finishA(const MtIn& in);
  void finishB(const MtIn& in);
  void finishC(const MtIn& in);
  void finishSeg(const MtIn& in);
  float volts(float duty, const MtIn& in) const { return duty / 4095.0f * in.vbatt; }
  bool sel(int w) const { return wheels_ == 0 || wheels_ == w + 1; }
  bool brake() const { return drive_ == 1; }
  float stallVolts(int w, int d, const MtIn& in) const;
  float lDuty(int w, float target, float dt, const MtIn& in);

  State state_ = S_IDLE;
  uint8_t test_ = 0;       // running request
  uint8_t wheels_ = 0;
  uint8_t parts_[3] = {};  // test letters to run ('a', 'b', 'c', 'o', 'p', 'q'), 0 = end
  uint8_t part_i_ = 0, part_ = 0;
  bool done_ = false, need_begin_ = false, holding_ = false;
  float duty_[2] = {0, 0};
  float max_duty_ = 4095;
  float hz_ = 0;      // drive config recorded at the start of the run
  unsigned drive_ = 0;
  uint32_t t_req_us_ = 0, t_ok_us_ = 0, t_start_us_ = 0, deadline_us_ = 0, t_phase_us_ = 0, hold_until_us_ = 0;
  bool ok_timing_ = false;
  const char* last_block_ = nullptr;
  uint32_t last_block_log_us_ = 0;
  uint32_t last_enc_us_ = 0;
  bool have_enc_ = false;
  float vb_min_ = 0, vb_max_ = 0, vb_begin_ = 0;
  float raw_vb_ = 0;          // pack reading as given
  bool vb_valid_ = false;     // reading inside 6..17 V now
  bool vb_assumed_ = false;   // run started without a valid reading: volts = mot.v_ref

  // anomaly watch
  uint32_t stall_us_[2] = {};
  bool stall_on_[2] = {};
  uint32_t sag_us_ = 0;
  bool sag_on_ = false;
  uint32_t drive_us_ = 0;  // start of the current stretch of continuous drive
  bool drive_on_ = false;
  bool drop_ref_ok_[2] = {};
  float drop_ref_[2] = {};
  uint32_t drop_us_[2] = {};
  bool drop_on_[2] = {};
  const char* anomaly_ = nullptr;
  char anomaly_buf_[80] = "";

  // test a: per direction, per wheel ramp up -> start, ramp down -> stop
  int8_t dir_ = 1;
  uint8_t a_st_[2] = {};      // 0 up, 1 down, 2 done
  float a_duty_[2] = {};
  float a_ad_next_[2] = {};   // next ramp-down duty to report the speed at (mt,ad)
  uint32_t a_tick_us_ = 0;
  float a_start_[2][2] = {{-1, -1}, {-1, -1}};  // [wheel][dir 0 fwd 1 rev], duty, -1 = none
  float a_stop_[2][2] = {{-1, -1}, {-1, -1}};
  float a_start_v_[2][2] = {{-1, -1}, {-1, -1}};
  float a_stop_v_[2][2] = {{-1, -1}, {-1, -1}};
  bool a_valid_ = false;  // thresholds from a finished test a (used by test c)
  bool a_run_ = false;    // ... and from this run (stall thresholds)

  // test b: levels 1..16, hold 1.2 s, mean speed over the last 0.5 s
  uint8_t b_level_ = 0;
  bool b_have0_[2] = {};
  int32_t b_c0_[2] = {};
  uint32_t b_t0_[2] = {};
  float b_prev_[2] = {};
  float b_top_[2][2] = {};
  float b_move_v_[2][2] = {{-1, -1}, {-1, -1}};
  uint16_t b_n_[2] = {};
  uint16_t b_tr_ms_[2][kTrace];
  float b_tr_v_[2][kTrace];

  // test c: alternate direction at a fixed duty, time / angle until the wheel really reverses
  uint8_t c_rep_ = 0;
  bool c_spin_ = true;
  float c_duty_ = 0;
  uint32_t c_t0_us_ = 0;
  int32_t c_c0_[2] = {}, c_ext_[2] = {};
  uint32_t c_ext_us_[2] = {};
  bool c_det_[2] = {};
  float c_sum_stop_[2] = {}, c_sum_rev_[2] = {}, c_sum_coast_[2] = {};
  uint8_t c_cnt_[2] = {};
  float c_ref_[2][2] = {};  // spin speed of the first spin each way, mm/s

  // tests 5-7: tracking ('o' open loop model 0, 'p' open loop model 1, 'q' closed loop)
  struct LStat {
    uint32_t n, ndv;
    float se, se2, emax, dv2;
    float start_tgt, stop_tgt, t63u, t63d;
    uint16_t flips;
    int8_t last_sign;
    bool moved, stopped;
  };
  uint8_t l_seg_ = 0;
  uint32_t l_t0_us_ = 0, l_last_us_ = 0, l_tr_us_ = 0;
  bool l_have_last_ = false;
  float l_v_[2] = {}, l_prev_v_[2] = {};
  WheelSpeedEst est_[2];
  WheelSpeedLoop loop_[2];
  LStat ls_[2] = {};
  float l_sum_[2][5] = {};   // rms error per segment, for the part summary
  float l_t63_[2][2] = {};   // st: up / down

  char q_[kQ][kLine];
  uint8_t q_head_ = 0, q_n_ = 0;
  uint32_t q_drop_ = 0;
};
