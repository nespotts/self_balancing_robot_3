// Wheels-up motor characterisation (issue #16). Pure logic: no Arduino calls, time is passed in,
// so it also builds on a PC (test/host/). main.cpp feeds it once per background pass and drives
// the motors with raw duty while driving() is true (no correction curve, no battery compensation).
//
//   mot.test 1 = breakaway / stop thresholds (slow ramps)   2 = steady speed vs duty (16 steps)
//            3 = reversal lag / backlash                   4 = all three
//
// Runs only when: requested, remote ARMED, mode IDLE (balance controller off), remote link live,
// IMU + encoders ok, |pitch| and |roll| < 20 deg, still, for 1 s. Any of those failing, the
// request going back to 0, or a timeout -> motors 0 at once, "mt,abort,<reason>".
// Results are MSG_LOG lines "mt,<kind>,..." (format: docs/firmware.md, parser: tools/motor_fit.py).
#pragma once
#include <stddef.h>
#include <stdint.h>

struct MtIn {
  uint32_t now_us = 0;
  uint8_t request = 0;  // mot.test
  bool armed = false, estop = false, idle = false, radio_live = false, imu_ok = false, enc_ok = false;
  float pitch = 0, roll = 0, pitch_rate = 0;  // deg, deg/s
  int32_t enc_l = 0, enc_r = 0;               // encoder counts (4096 / wheel rev)
  uint32_t enc_us = 0;                        // time of the last encoder read
  float speed_l = 0, speed_r = 0;             // mm/s (80 ms window)
  float vbatt = 0;                            // pack voltage (filtered), V
  float max_duty = 4095;                      // mot.test_max
};

class MotorTest {
 public:
  static constexpr float kMmPerCount = 3.14159265f * 66.0f / 4096.0f;  // config.h WHEEL_DIAMETER_MM

  void step(const MtIn& in);
  bool driving() const { return state_ == S_RUN; }  // the test owns the motors
  bool pending() const { return state_ != S_IDLE; }
  float left() const { return duty_[0]; }           // signed raw duty, -4095..4095
  float right() const { return duty_[1]; }
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
  static constexpr int kQ = 32, kLine = 176, kTrace = 260;

  void emit(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  const char* startBlocker(const MtIn& in) const;
  const char* abortReason(const MtIn& in) const;
  void stopAll(bool done);
  void hold(uint32_t ms, const MtIn& in);  // motors 0 for ms, then the current part continues
  void startTest(const MtIn& in);
  void nextPart(const MtIn& in);
  void stepPart(const MtIn& in);
  void endPart(const MtIn& in);
  void runA(const MtIn& in, bool new_enc);
  void runB(const MtIn& in, bool new_enc);
  void runC(const MtIn& in, bool new_enc);
  void finishA(const MtIn& in);
  void finishB(const MtIn& in);
  void finishC(const MtIn& in);
  float volts(float duty, const MtIn& in) const { return duty / 4095.0f * in.vbatt; }

  State state_ = S_IDLE;
  uint8_t test_ = 0;       // running request
  uint8_t parts_[3] = {};  // test letters to run ('a', 'b', 'c'), 0 = end
  uint8_t part_i_ = 0, part_ = 0;
  bool done_ = false, need_begin_ = false, holding_ = false;
  float duty_[2] = {0, 0};
  float max_duty_ = 4095;
  uint32_t t_req_us_ = 0, t_ok_us_ = 0, t_start_us_ = 0, deadline_us_ = 0, t_phase_us_ = 0, hold_until_us_ = 0;
  bool ok_timing_ = false;
  const char* last_block_ = nullptr;
  uint32_t last_block_log_us_ = 0;
  uint32_t last_enc_us_ = 0;
  bool have_enc_ = false;
  float vb_min_ = 0, vb_max_ = 0;

  // test a: per direction, per wheel ramp up -> start, ramp down -> stop
  int8_t dir_ = 1;
  uint8_t a_st_[2] = {};      // 0 up, 1 down, 2 done
  float a_duty_[2] = {};
  uint32_t a_tick_us_ = 0;
  float a_start_[2][2] = {{-1, -1}, {-1, -1}};  // [wheel][dir 0 fwd 1 rev], duty, -1 = none
  float a_stop_[2][2] = {{-1, -1}, {-1, -1}};
  float a_start_v_[2][2] = {{-1, -1}, {-1, -1}};
  float a_stop_v_[2][2] = {{-1, -1}, {-1, -1}};
  bool a_valid_ = false;  // thresholds from a finished test a (used by test c)

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
  uint32_t c_det_us_[2] = {};
  float c_sum_stop_[2] = {}, c_sum_rev_[2] = {}, c_sum_coast_[2] = {};
  uint8_t c_cnt_[2] = {};

  char q_[kQ][kLine];
  uint8_t q_head_ = 0, q_n_ = 0;
  uint32_t q_drop_ = 0;
};
