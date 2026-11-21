#include "motor_test.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {
// start / safety windows
constexpr uint32_t kStartHoldUs = 1000000;     // conditions ok this long before the motors move
constexpr uint32_t kWaitTimeoutUs = 60000000;  // request not started within 60 s -> dropped
constexpr uint32_t kBlockLogUs = 5000000;      // repeat the "waiting" reason every 5 s
constexpr uint32_t kEncStaleUs = 50000;        // no encoder read this long -> abort
constexpr float kTiltDeg = 20.0f;              // |pitch|, |roll| window (start and run)
constexpr float kStillRateDps = 10.0f;         // to start: body still
constexpr float kAbortRateDps = 150.0f;        // while running: body knocked / falling
constexpr float kMoveMms = 20.0f;              // wheel "moving" (~6 rpm)
constexpr float kStopMms = 8.0f;               // wheel "stopped"
// test a
constexpr float kAStep = 10.0f;                // duty counts per tick
constexpr uint32_t kATickUs = 50000;
// test b
constexpr uint8_t kBLevels = 16;
constexpr uint32_t kBHoldUs = 1200000, kBMeasUs = 500000;
constexpr float kBHalfWindowMs = 40.0f;        // 80 ms speed window -> ~40 ms lag in the trace
// test c
constexpr uint8_t kCReps = 6;                  // alternating, 3 each way
constexpr uint32_t kCSpinUs = 1000000, kCTimeoutUs = 2000000;
constexpr int32_t kCRevCounts = 23;            // 2 deg of wheel back from the turning point
constexpr float kCMargin = 300.0f;             // run duty = highest breakaway + this
constexpr float kCDefaultDuty = 2000.0f;
// per-test time limits (whole test aborts past the sum)
constexpr uint32_t kALimitUs = 150000000, kBLimitUs = 70000000, kCLimitUs = 60000000;

const char* const kWheel[2] = {"left", "right"};
inline const char* dirName(int8_t d) { return d > 0 ? "fwd" : "rev"; }
inline int di(int8_t d) { return d > 0 ? 0 : 1; }
inline bool after(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }
}  // namespace

void MotorTest::emit(const char* fmt, ...) {
  if (q_n_ >= kQ) {
    q_drop_++;
    return;
  }
  char* s = q_[(q_head_ + q_n_) % kQ];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s, kLine, fmt, ap);
  va_end(ap);
  q_n_++;
}

void MotorTest::popLine() {
  if (!q_n_) return;
  q_head_ = (q_head_ + 1) % kQ;
  q_n_--;
}

const char* MotorTest::startBlocker(const MtIn& in) const {
  if (in.estop) return "estop";
  if (!in.armed) return "arm_the_remote_in_IDLE";
  if (!in.idle) return "mode_not_IDLE";
  if (!in.radio_live) return "no_remote";
  if (!in.imu_ok) return "imu";
  if (!in.enc_ok) return "encoders";
  if (fabsf(in.pitch) > kTiltDeg || fabsf(in.roll) > kTiltDeg) return "not_upright";
  if (fabsf(in.pitch_rate) > kStillRateDps) return "not_still";
  if (fabsf(in.speed_l) > kStopMms || fabsf(in.speed_r) > kStopMms) return "wheels_turning";
  return nullptr;
}

const char* MotorTest::abortReason(const MtIn& in) const {
  if (in.request != test_) return "cancelled";
  if (in.estop) return "estop";
  if (!in.armed) return "disarmed";
  if (!in.idle) return "mode_change";
  if (!in.radio_live) return "remote_lost";
  if (!in.imu_ok) return "imu";
  if (!in.enc_ok) return "encoder";
  if ((int32_t)(in.now_us - in.enc_us) > (int32_t)kEncStaleUs) return "encoder_stale";
  if (fabsf(in.pitch) > kTiltDeg || fabsf(in.roll) > kTiltDeg) return "tilt";
  if (fabsf(in.pitch_rate) > kAbortRateDps) return "pitch_rate";
  if (after(in.now_us, deadline_us_)) return "timeout";
  return nullptr;
}

void MotorTest::stopAll(bool done) {
  duty_[0] = duty_[1] = 0;
  state_ = S_IDLE;
  holding_ = false;
  test_ = 0;
  done_ = done;
}

void MotorTest::hold(uint32_t ms, const MtIn& in) {
  duty_[0] = duty_[1] = 0;
  holding_ = true;
  hold_until_us_ = in.now_us + ms * 1000u;
}

void MotorTest::step(const MtIn& in) {
  const bool new_enc = !have_enc_ || in.enc_us != last_enc_us_;
  last_enc_us_ = in.enc_us;
  have_enc_ = true;

  switch (state_) {
    case S_IDLE:
      if (in.request) {
        state_ = S_WAIT;
        test_ = in.request;
        t_req_us_ = in.now_us;
        ok_timing_ = false;
        last_block_ = nullptr;
        emit("mt,request,%u", test_);
      }
      return;

    case S_WAIT: {
      if (in.request == 0) {
        emit("mt,cancel,%u", test_);
        stopAll(false);
        return;
      }
      if (in.request != test_) {
        test_ = in.request;
        emit("mt,request,%u", test_);
      }
      const char* b = startBlocker(in);
      if (b) {
        ok_timing_ = false;
        if (b != last_block_ || in.now_us - last_block_log_us_ >= kBlockLogUs) {
          last_block_ = b;
          last_block_log_us_ = in.now_us;
          emit("mt,wait,%s", b);
        }
        if (in.now_us - t_req_us_ >= kWaitTimeoutUs) {
          emit("mt,abort,not_started_in_60s,%s", b);
          stopAll(true);
        }
        return;
      }
      if (!ok_timing_) {
        ok_timing_ = true;
        t_ok_us_ = in.now_us;
      }
      if (in.now_us - t_ok_us_ >= kStartHoldUs) startTest(in);
      return;
    }

    case S_RUN:
      break;
  }

  const char* r = abortReason(in);
  if (r) {
    emit("mt,abort,%s,part=%c,t=%.1f,vb=%.2f", r, part_ ? (char)part_ : '-',
         (double)(in.now_us - t_start_us_) * 1e-6, (double)in.vbatt);
    stopAll(true);
    return;
  }
  if (in.vbatt < vb_min_) vb_min_ = in.vbatt;
  if (in.vbatt > vb_max_) vb_max_ = in.vbatt;
  if (holding_) {
    if (!after(in.now_us, hold_until_us_)) return;
    holding_ = false;
  }
  switch (part_) {
    case 'a': runA(in, new_enc); break;
    case 'b': runB(in, new_enc); break;
    case 'c': runC(in, new_enc); break;
    default: nextPart(in); break;
  }
}

void MotorTest::startTest(const MtIn& in) {
  memset(parts_, 0, sizeof(parts_));
  uint32_t limit = 0;
  switch (test_) {
    case 1: parts_[0] = 'a'; limit = kALimitUs; break;
    case 2: parts_[0] = 'b'; limit = kBLimitUs; break;
    case 3: parts_[0] = 'c'; limit = kCLimitUs; break;
    default:
      parts_[0] = 'a';
      parts_[1] = 'b';
      parts_[2] = 'c';
      limit = kALimitUs + kBLimitUs + kCLimitUs;
      break;
  }
  state_ = S_RUN;
  part_i_ = 0;
  part_ = 0;
  holding_ = false;
  t_start_us_ = in.now_us;
  deadline_us_ = in.now_us + limit;
  vb_min_ = vb_max_ = in.vbatt;
  max_duty_ = in.max_duty > 4095.0f ? 4095.0f : (in.max_duty < 100.0f ? 100.0f : in.max_duty);
  emit("mt,begin,%u,max=%.0f,vb=%.2f,pitch=%.1f", test_, (double)max_duty_, (double)in.vbatt,
       (double)in.pitch);
  nextPart(in);
}

void MotorTest::nextPart(const MtIn& in) {
  const uint8_t p = part_i_ < sizeof(parts_) ? parts_[part_i_] : 0;
  part_i_++;
  if (!p) {
    emit("mt,end,%u,ok,%.1fs,vb=%.2f..%.2f", test_, (double)(in.now_us - t_start_us_) * 1e-6,
         (double)vb_min_, (double)vb_max_);
    stopAll(true);
    return;
  }
  part_ = p;
  dir_ = 1;
  need_begin_ = true;
  if (p == 'a') {
    for (int w = 0; w < 2; w++)
      for (int d = 0; d < 2; d++) a_start_[w][d] = a_stop_[w][d] = a_start_v_[w][d] = a_stop_v_[w][d] = -1;
  } else if (p == 'b') {
    b_level_ = 1;
    b_prev_[0] = b_prev_[1] = 0;
    for (int w = 0; w < 2; w++)
      for (int d = 0; d < 2; d++) {
        b_top_[w][d] = 0;
        b_move_v_[w][d] = -1;
      }
  } else {
    float hi = -1;
    if (a_valid_)
      for (int w = 0; w < 2; w++)
        for (int d = 0; d < 2; d++)
          if (a_start_[w][d] > hi) hi = a_start_[w][d];
    c_duty_ = hi > 0 ? hi + kCMargin : kCDefaultDuty;
    if (c_duty_ > max_duty_) c_duty_ = max_duty_;
    c_rep_ = 0;
    c_spin_ = true;
    for (int w = 0; w < 2; w++) c_sum_stop_[w] = c_sum_rev_[w] = c_sum_coast_[w] = c_cnt_[w] = 0;
  }
  stepPart(in);
}

void MotorTest::stepPart(const MtIn& in) {
  switch (part_) {
    case 'a': runA(in, false); break;
    case 'b': runB(in, false); break;
    case 'c': runC(in, false); break;
    default: break;
  }
}

void MotorTest::endPart(const MtIn& in) {
  duty_[0] = duty_[1] = 0;
  part_ = 0;
  holding_ = true;
  hold_until_us_ = in.now_us + 1000000u;
}

// ------------------------------------------------------------------ test a: thresholds

void MotorTest::runA(const MtIn& in, bool new_enc) {
  if (need_begin_) {
    need_begin_ = false;
    for (int w = 0; w < 2; w++) {
      a_st_[w] = 0;
      a_duty_[w] = 0;
    }
    a_tick_us_ = in.now_us;
  }
  if (in.now_us - a_tick_us_ >= kATickUs) {
    a_tick_us_ = in.now_us;
    for (int w = 0; w < 2; w++) {
      if (a_st_[w] == 0) a_duty_[w] = fminf(a_duty_[w] + kAStep, max_duty_);
      else if (a_st_[w] == 1) a_duty_[w] = fmaxf(a_duty_[w] - kAStep, 0.0f);
    }
  }
  const int d = di(dir_);
  if (new_enc) {
    for (int w = 0; w < 2; w++) {
      const float sp = w ? in.speed_r : in.speed_l;
      if (a_st_[w] == 0) {
        if (fabsf(sp) > kMoveMms) {
          a_start_[w][d] = a_duty_[w];
          a_start_v_[w][d] = volts(a_duty_[w], in);
          emit("mt,a,%s,%s,start,%.0f,%.3f,%.2f,%.0f", kWheel[w], dirName(dir_), (double)a_duty_[w],
               (double)a_start_v_[w][d], (double)in.vbatt, (double)sp);
          a_st_[w] = 1;
        } else if (a_duty_[w] >= max_duty_) {
          emit("mt,a,%s,%s,start,-1,-1,%.2f,%.0f", kWheel[w], dirName(dir_), (double)in.vbatt, (double)sp);
          emit("mt,a,%s,%s,stop,-1,-1,%.2f,%.0f", kWheel[w], dirName(dir_), (double)in.vbatt, (double)sp);
          a_st_[w] = 2;
        }
      } else if (a_st_[w] == 1) {
        if (fabsf(sp) < kStopMms || a_duty_[w] <= 0.0f) {
          a_stop_[w][d] = a_duty_[w];
          a_stop_v_[w][d] = volts(a_duty_[w], in);
          emit("mt,a,%s,%s,stop,%.0f,%.3f,%.2f,%.0f", kWheel[w], dirName(dir_), (double)a_duty_[w],
               (double)a_stop_v_[w][d], (double)in.vbatt, (double)sp);
          a_st_[w] = 2;
        }
      }
    }
  }
  for (int w = 0; w < 2; w++) duty_[w] = a_st_[w] < 2 ? dir_ * a_duty_[w] : 0.0f;
  if (a_st_[0] == 2 && a_st_[1] == 2) {
    if (dir_ > 0) {
      dir_ = -1;
      need_begin_ = true;
      hold(1000, in);
    } else {
      finishA(in);
    }
  }
}

void MotorTest::finishA(const MtIn& in) {
  a_valid_ = true;
  emit("mt,sum,a,vb=%.2f,l_fwd=%.0f/%.0f,l_rev=%.0f/%.0f,r_fwd=%.0f/%.0f,r_rev=%.0f/%.0f (start/stop duty)",
       (double)in.vbatt, (double)a_start_[0][0], (double)a_stop_[0][0], (double)a_start_[0][1],
       (double)a_stop_[0][1], (double)a_start_[1][0], (double)a_stop_[1][0], (double)a_start_[1][1],
       (double)a_stop_[1][1]);
  emit("mt,sum,a,volts,l_fwd=%.2f/%.2f,l_rev=%.2f/%.2f,r_fwd=%.2f/%.2f,r_rev=%.2f/%.2f",
       (double)a_start_v_[0][0], (double)a_stop_v_[0][0], (double)a_start_v_[0][1], (double)a_stop_v_[0][1],
       (double)a_start_v_[1][0], (double)a_stop_v_[1][0], (double)a_start_v_[1][1], (double)a_stop_v_[1][1]);
  endPart(in);
}

// ------------------------------------------------------------------ test b: speed curve

void MotorTest::runB(const MtIn& in, bool new_enc) {
  if (need_begin_) {
    need_begin_ = false;
    t_phase_us_ = in.now_us;
    for (int w = 0; w < 2; w++) {
      b_have0_[w] = false;
      b_n_[w] = 0;
    }
  }
  const float duty = max_duty_ * (float)b_level_ / (float)kBLevels;
  duty_[0] = duty_[1] = dir_ * duty;
  const uint32_t el = in.now_us - t_phase_us_;
  if (!new_enc) return;
  const int32_t enc[2] = {in.enc_l, in.enc_r};
  const float sp[2] = {in.speed_l, in.speed_r};
  for (int w = 0; w < 2; w++) {
    if (b_n_[w] < kTrace) {
      b_tr_ms_[w][b_n_[w]] = (uint16_t)(el / 1000u);
      b_tr_v_[w][b_n_[w]] = sp[w];
      b_n_[w]++;
    }
    if (!b_have0_[w] && el >= kBHoldUs - kBMeasUs) {
      b_have0_[w] = true;
      b_c0_[w] = enc[w];
      b_t0_[w] = in.enc_us;
    }
  }
  if (el < kBHoldUs) return;

  const int d = di(dir_);
  for (int w = 0; w < 2; w++) {
    const uint32_t dt = in.enc_us - b_t0_[w];
    const float v = (b_have0_[w] && dt > 0) ? (float)(enc[w] - b_c0_[w]) * kMmPerCount * 1e6f / (float)dt : 0.0f;
    float t63 = -1;
    const float dv = v - b_prev_[w];
    if (fabsf(dv) > 30.0f) {
      const float target = b_prev_[w] + 0.632f * dv;
      for (uint16_t i = 0; i < b_n_[w]; i++) {
        if ((b_tr_v_[w][i] - target) * (dv > 0 ? 1.0f : -1.0f) >= 0.0f) {
          t63 = fmaxf(0.0f, (float)b_tr_ms_[w][i] - kBHalfWindowMs);
          break;
        }
      }
    }
    emit("mt,b,%s,%s,%.0f,%.3f,%.2f,%.1f,%.0f", kWheel[w], dirName(dir_), (double)duty,
         (double)volts(duty, in), (double)in.vbatt, (double)v, (double)t63);
    if (fabsf(v) > kMoveMms && b_move_v_[w][d] < 0) b_move_v_[w][d] = volts(duty, in);
    b_top_[w][d] = v;
    b_prev_[w] = v;
  }
  need_begin_ = true;
  if (++b_level_ > kBLevels) {
    if (dir_ > 0) {
      dir_ = -1;
      b_level_ = 1;
      b_prev_[0] = b_prev_[1] = 0;
      hold(1500, in);  // coast down from full speed
    } else {
      finishB(in);
    }
  }
}

void MotorTest::finishB(const MtIn& in) {
  emit("mt,sum,b,vb=%.2f,top_mms l_fwd=%.0f l_rev=%.0f r_fwd=%.0f r_rev=%.0f,moves_at_V l_fwd=%.2f l_rev=%.2f "
       "r_fwd=%.2f r_rev=%.2f",
       (double)in.vbatt, (double)b_top_[0][0], (double)b_top_[0][1], (double)b_top_[1][0], (double)b_top_[1][1],
       (double)b_move_v_[0][0], (double)b_move_v_[0][1], (double)b_move_v_[1][0], (double)b_move_v_[1][1]);
  endPart(in);
}

// ------------------------------------------------------------------ test c: reversal

void MotorTest::runC(const MtIn& in, bool new_enc) {
  const int32_t enc[2] = {in.enc_l, in.enc_r};
  if (need_begin_) {
    need_begin_ = false;
    t_phase_us_ = in.now_us;
    c_spin_ = true;
  }
  duty_[0] = duty_[1] = dir_ * c_duty_;
  if (c_spin_) {
    if (in.now_us - t_phase_us_ >= kCSpinUs && new_enc) {
      dir_ = -dir_;  // reverse now
      duty_[0] = duty_[1] = dir_ * c_duty_;
      c_spin_ = false;
      c_t0_us_ = in.now_us;
      for (int w = 0; w < 2; w++) {
        c_c0_[w] = c_ext_[w] = enc[w];
        c_ext_us_[w] = in.enc_us;
        c_det_[w] = false;
      }
    }
    return;
  }
  const int8_t old = (int8_t)-dir_;
  if (new_enc) {
    for (int w = 0; w < 2; w++) {
      if (c_det_[w]) continue;
      if (old * (enc[w] - c_ext_[w]) > 0) {
        c_ext_[w] = enc[w];
        c_ext_us_[w] = in.enc_us;
      }
      if (old * (c_ext_[w] - enc[w]) >= kCRevCounts) {
        c_det_[w] = true;
        const float t_stop = fmaxf(0.0f, (float)(int32_t)(c_ext_us_[w] - c_t0_us_) * 1e-3f);
        const float t_rev = (float)(int32_t)(in.enc_us - c_t0_us_) * 1e-3f;
        const float coast = (float)(old * (c_ext_[w] - c_c0_[w])) * 360.0f / 4096.0f;
        emit("mt,c,%s,%s,%u,%.0f,%.3f,%.2f,%.0f,%.0f,%.1f", kWheel[w], dirName(dir_), c_rep_, (double)c_duty_,
             (double)volts(c_duty_, in), (double)in.vbatt, (double)t_stop, (double)t_rev, (double)coast);
        c_sum_stop_[w] += t_stop;
        c_sum_rev_[w] += t_rev;
        c_sum_coast_[w] += coast;
        c_cnt_[w]++;
      }
    }
  }
  const bool timeout = in.now_us - c_t0_us_ >= kCTimeoutUs;
  if ((c_det_[0] && c_det_[1]) || timeout) {
    for (int w = 0; w < 2; w++)
      if (!c_det_[w])
        emit("mt,c,%s,%s,%u,%.0f,%.3f,%.2f,-1,-1,-1", kWheel[w], dirName(dir_), c_rep_, (double)c_duty_,
             (double)volts(c_duty_, in), (double)in.vbatt);
    if (++c_rep_ >= kCReps) {
      finishC(in);
    } else {
      c_spin_ = true;
      t_phase_us_ = in.now_us;
    }
  }
}

void MotorTest::finishC(const MtIn& in) {
  float m[2][3];
  for (int w = 0; w < 2; w++) {
    const float n = c_cnt_[w] ? (float)c_cnt_[w] : 1.0f;
    m[w][0] = c_cnt_[w] ? c_sum_stop_[w] / n : -1;
    m[w][1] = c_cnt_[w] ? c_sum_rev_[w] / n : -1;
    m[w][2] = c_cnt_[w] ? c_sum_coast_[w] / n : -1;
  }
  emit("mt,sum,c,duty=%.0f,V=%.2f,mean left stop=%.0fms rev=%.0fms coast=%.1fdeg n=%u,right stop=%.0fms "
       "rev=%.0fms coast=%.1fdeg n=%u",
       (double)c_duty_, (double)volts(c_duty_, in), (double)m[0][0], (double)m[0][1], (double)m[0][2], c_cnt_[0],
       (double)m[1][0], (double)m[1][1], (double)m[1][2], c_cnt_[1]);
  endPart(in);
}
