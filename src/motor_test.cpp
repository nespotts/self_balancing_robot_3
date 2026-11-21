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
constexpr float kVbMin = 6.0f, kVbMax = 17.0f; // pack voltage reading trusted inside
// anomaly aborts (issue #26)
constexpr uint32_t kStallUs = 1000000;         // stalled this long above the start threshold
constexpr float kStallMarginV = 1.0f;          // over this run's test-1 start volts
constexpr float kStallFallbackV = 8.5f;        // without test 1 in this run: max(this, dz + 2 V)
constexpr float kStallDzMarginV = 2.0f;
constexpr float kDropFrac = 0.7f;              // speed < 70 % of the step's first measurement
constexpr float kDropMinMms = 100.0f;          // ... only checked above this reference speed
constexpr uint32_t kDropUs = 150000;           // ... for this long
constexpr float kSagV = 1.5f;                  // pack below its start value by this much
constexpr uint32_t kSagUs = 200000;
constexpr uint32_t kDriveMaxUs = 60000000;     // one stretch of continuous drive, mode 0
constexpr uint32_t kBrakeDriveMaxUs = 25000000;  // ... mode 1
// drive mode 1 cool-downs (motors off)
constexpr uint32_t kBrakeCoolPartMs = 60000, kBrakeCoolDirMs = 10000, kBrakeCoolSegMs = 5000;
// test a
constexpr float kAStep = 10.0f;                // duty counts per tick
constexpr uint32_t kATickUs = 50000;
constexpr float kAdStep = 60.0f;               // ramp-down speed sample every 60 duty (~0.18 V at 12 V)
// test b
constexpr uint8_t kBLevels = 16;
constexpr uint32_t kBHoldUs = 1200000, kBMeasUs = 500000, kBDropRefUs = 800000;  // ref after the step settles
constexpr float kBHalfWindowMs = 40.0f;        // 80 ms speed window -> ~40 ms lag in the trace
// test c
constexpr uint8_t kCReps = 6;                  // alternating, 3 each way
constexpr uint32_t kCSpinUs = 1000000, kCTimeoutUs = 2000000;
constexpr int32_t kCRevCounts = 23;            // 2 deg of wheel back from the turning point
constexpr float kCMargin = 300.0f;             // run duty = highest breakaway + this
constexpr float kCDefaultDuty = 2000.0f;
// tests 5-7: tracking segments
constexpr uint8_t kLSegs = 5;
const char* const kSegName[kLSegs] = {"r+", "r-", "s0", "s150", "st"};
constexpr float kSegMs[kLSegs] = {7500, 7500, 4000, 5000, 4000};
constexpr uint32_t kLTraceUs = 50000;          // trace line every 50 ms per wheel
constexpr float kChatterV = 0.2f;              // output sign flips counted beyond +-this
// per-test time limits (whole test aborts past the sum; mode-1 cool-downs extend it)
constexpr uint32_t kALimitUs = 150000000, kBLimitUs = 70000000, kCLimitUs = 60000000, kLLimitUs = 50000000;

const char* const kWheel[2] = {"left", "right"};
inline const char* dirName(int8_t d) { return d > 0 ? "fwd" : "rev"; }
inline int di(int8_t d) { return d > 0 ? 0 : 1; }
inline bool after(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }
inline const char* ctlName(uint8_t p) { return p == 'o' ? "ol0" : (p == 'p' ? "ol1" : "cl"); }
inline float sgn(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

// target speed (mm/s) of segment s at t ms; scored = counts for the tracking error
float segTarget(uint8_t s, float t, bool& scored) {
  scored = true;
  const float w = 2.0f * 3.14159265f / 1000.0f;  // 1 Hz
  switch (s) {
    case 0:
    case 1: {
      float v = t < 3000 ? 0.1f * t : (t < 4000 ? 300.0f : (t < 7000 ? 300.0f - 0.1f * (t - 4000) : 0.0f));
      return s == 0 ? v : -v;
    }
    case 2:
      return 50.0f * sinf(w * t);
    case 3:
      if (t < 1000) {
        scored = false;
        return 0.15f * t;
      }
      return 150.0f + 50.0f * sinf(w * (t - 1000));
    default:
      if (t < 1000) {
        scored = false;
        return 0.15f * t;
      }
      return (t >= 2000 && t < 3000) ? 250.0f : 150.0f;
  }
}
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

float MotorTest::stallVolts(int w, int d, const MtIn& in) const {
  if (a_run_ && a_start_v_[w][d] > 0.0f) return a_start_v_[w][d] + kStallMarginV;
  const float v = in.dz[w][d] + kStallDzMarginV;
  return v > kStallFallbackV ? v : kStallFallbackV;
}

// Binding / heat / supply checks while running (issue #26). Returns the abort reason or nullptr.
const char* MotorTest::anomaly(const MtIn& in) {
  if (anomaly_) return anomaly_;  // set by a part (test c spin speed)
  const uint32_t now = in.now_us;
  // supply sag (only with a valid reading at the start; a reading lost since then counts as a sag)
  if (!vb_assumed_ && raw_vb_ < vb_begin_ - kSagV) {
    if (!sag_on_) {
      sag_on_ = true;
      sag_us_ = now;
    } else if (now - sag_us_ >= kSagUs) {
      snprintf(anomaly_buf_, sizeof(anomaly_buf_), "supply_sag,vb=%.2f,begin=%.2f", (double)raw_vb_,
               (double)vb_begin_);
      return anomaly_buf_;
    }
  } else {
    sag_on_ = false;
  }
  // continuous drive
  if (duty_[0] != 0.0f || duty_[1] != 0.0f) {
    if (!drive_on_) {
      drive_on_ = true;
      drive_us_ = now;
    } else if (now - drive_us_ > (brake() ? kBrakeDriveMaxUs : kDriveMaxUs)) {
      snprintf(anomaly_buf_, sizeof(anomaly_buf_), "drive_time,%.0fs", (double)(now - drive_us_) * 1e-6);
      return anomaly_buf_;
    }
  } else {
    drive_on_ = false;
  }
  const float sp[2] = {in.speed_l, in.speed_r};
  for (int w = 0; w < 2; w++) {
    if (!sel(w)) continue;
    // stall: volts above the start threshold, wheel not turning
    const int d = duty_[w] >= 0.0f ? 0 : 1;
    const float v = fabsf(duty_[w]) / 4095.0f * in.vbatt;
    const float thr = stallVolts(w, d, in);
    if (duty_[w] != 0.0f && v > thr && fabsf(sp[w]) < kStopMms) {
      if (!stall_on_[w]) {
        stall_on_[w] = true;
        stall_us_[w] = now;
      } else if (now - stall_us_[w] >= kStallUs) {
        snprintf(anomaly_buf_, sizeof(anomaly_buf_), "stall,%s,V=%.2f,thr=%.2f", kWheel[w], (double)v, (double)thr);
        return anomaly_buf_;
      }
    } else {
      stall_on_[w] = false;
    }
    // speed drop at a fixed duty (reference set by the part)
    if (drop_ref_ok_[w] && fabsf(sp[w]) < kDropFrac * drop_ref_[w]) {
      if (!drop_on_[w]) {
        drop_on_[w] = true;
        drop_us_[w] = now;
      } else if (now - drop_us_[w] >= kDropUs) {
        snprintf(anomaly_buf_, sizeof(anomaly_buf_), "speed_drop,%s,ref=%.0f,now=%.0f,duty=%.0f", kWheel[w],
                 (double)drop_ref_[w], (double)fabsf(sp[w]), (double)fabsf(duty_[w]));
        return anomaly_buf_;
      }
    } else {
      drop_on_[w] = false;
    }
  }
  return nullptr;
}

void MotorTest::stopAll(bool done) {
  duty_[0] = duty_[1] = 0;
  state_ = S_IDLE;
  holding_ = false;
  test_ = 0;
  done_ = done;
  anomaly_ = nullptr;
  for (int w = 0; w < 2; w++) {
    stall_on_[w] = drop_on_[w] = drop_ref_ok_[w] = false;
    loop_[w].reset();
  }
  sag_on_ = drive_on_ = false;
}

void MotorTest::abort(const char* why, uint32_t now_us) {
  if (state_ == S_IDLE) return;
  emit("mt,abort,%s,part=%c,t=%.1f", why, part_ ? (char)part_ : '-', (double)(now_us - t_start_us_) * 1e-6);
  stopAll(true);
}

void MotorTest::hold(uint32_t ms, const MtIn& in) {
  duty_[0] = duty_[1] = 0;
  drop_ref_ok_[0] = drop_ref_ok_[1] = false;
  holding_ = true;
  hold_until_us_ = in.now_us + ms * 1000u;
  if (brake()) deadline_us_ += ms * 1000u;  // cool-downs don't eat the test's time limit
}

// No valid pack reading (bench supply without the balance lead): volts use mot.v_ref instead
// (set it to the supply voltage); flagged in the begin line, no sag check.
void MotorTest::step(const MtIn& raw) {
  MtIn in = raw;
  raw_vb_ = raw.vbatt;
  vb_valid_ = raw.vbatt >= kVbMin && raw.vbatt <= kVbMax;
  if (!vb_valid_) in.vbatt = raw.v_ref;
  const bool new_enc = !have_enc_ || in.enc_us != last_enc_us_;
  last_enc_us_ = in.enc_us;
  have_enc_ = true;
  if (new_enc) {
    est_[0].sample(in.enc_l, in.enc_us, kMmPerCount);
    est_[1].sample(in.enc_r, in.enc_us, kMmPerCount);
  }

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
  if (!r) r = anomaly(in);
  if (r) {
    emit("mt,abort,%s,part=%c,t=%.1f,vb=%.2f,hz=%.0f,drive=%u", r, part_ ? (char)part_ : '-',
         (double)(in.now_us - t_start_us_) * 1e-6, (double)in.vbatt, (double)hz_, drive_);
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
    case 'o':
    case 'p':
    case 'q': runL(in, new_enc); break;
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
    case 5:
      parts_[0] = 'o';
      parts_[1] = 'p';
      limit = 2 * kLLimitUs;
      break;
    case 6: parts_[0] = 'q'; limit = kLLimitUs; break;
    case 7:
      parts_[0] = 'o';
      parts_[1] = 'p';
      parts_[2] = 'q';
      limit = 3 * kLLimitUs;
      break;
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
  anomaly_ = nullptr;
  a_run_ = false;
  t_start_us_ = in.now_us;
  deadline_us_ = in.now_us + limit;
  vb_min_ = vb_max_ = vb_begin_ = in.vbatt;
  vb_assumed_ = !vb_valid_;
  wheels_ = in.wheels <= 2 ? in.wheels : 0;
  hz_ = in.pwm_hz;
  drive_ = in.drive_mode ? 1 : 0;
  max_duty_ = in.max_duty > 4095.0f ? 4095.0f : (in.max_duty < 100.0f ? 100.0f : in.max_duty);
  if (brake() && max_duty_ > in.brake_duty_max) max_duty_ = in.brake_duty_max;
  emit("mt,begin,%u,max=%.0f,vb=%.2f,pitch=%.1f,hz=%.0f,drive=%u,wheels=%u,vbsrc=%s", test_, (double)max_duty_,
       (double)in.vbatt, (double)in.pitch, (double)hz_, drive_, wheels_, vb_assumed_ ? "v_ref" : "pack");
  if (vb_assumed_)
    emit("mt,warn,no_supply_reading (%.2f V): volts use mot.v_ref %.2f V, no sag check", (double)raw_vb_,
         (double)in.v_ref);
  if (brake() && hz_ > in.brake_max_hz + 0.5f) {
    emit("mt,abort,brake_hz_above_max,hz=%.0f,max=%.0f,drive=1", (double)hz_, (double)in.brake_max_hz);
    stopAll(true);
    return;
  }
  nextPart(in);
}

void MotorTest::nextPart(const MtIn& in) {
  const uint8_t p = part_i_ < sizeof(parts_) ? parts_[part_i_] : 0;
  part_i_++;
  if (!p) {
    emit("mt,end,%u,ok,%.1fs,vb=%.2f..%.2f,hz=%.0f,drive=%u,wheels=%u", test_,
         (double)(in.now_us - t_start_us_) * 1e-6, (double)vb_min_, (double)vb_max_, (double)hz_, drive_, wheels_);
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
  } else if (p == 'c') {
    float hi = -1;
    if (a_valid_)
      for (int w = 0; w < 2; w++)
        for (int d = 0; d < 2; d++)
          if (sel(w) && a_start_[w][d] > hi) hi = a_start_[w][d];
    c_duty_ = hi > 0 ? hi + kCMargin : kCDefaultDuty;
    if (c_duty_ > max_duty_) c_duty_ = max_duty_;
    c_rep_ = 0;
    c_spin_ = true;
    for (int w = 0; w < 2; w++) {
      c_sum_stop_[w] = c_sum_rev_[w] = c_sum_coast_[w] = 0;
      c_cnt_[w] = 0;
      c_ref_[w][0] = c_ref_[w][1] = 0;
    }
  } else {
    l_seg_ = 0;
    for (int w = 0; w < 2; w++) {
      for (int s = 0; s < kLSegs; s++) l_sum_[w][s] = -1;
      l_t63_[w][0] = l_t63_[w][1] = -1;
    }
  }
  stepPart(in);
}

void MotorTest::stepPart(const MtIn& in) {
  switch (part_) {
    case 'a': runA(in, false); break;
    case 'b': runB(in, false); break;
    case 'c': runC(in, false); break;
    case 'o':
    case 'p':
    case 'q': runL(in, false); break;
    default: break;
  }
}

void MotorTest::endPart(const MtIn& in) {
  part_ = 0;
  const bool more = part_i_ < sizeof(parts_) && parts_[part_i_];
  if (brake() && more) {
    emit("mt,cool,%us,motors off before the next part (drive mode 1)", (unsigned)(kBrakeCoolPartMs / 1000));
    hold(kBrakeCoolPartMs, in);
  } else {
    hold(1000, in);
  }
}

// ------------------------------------------------------------------ test a: thresholds

void MotorTest::runA(const MtIn& in, bool new_enc) {
  if (need_begin_) {
    need_begin_ = false;
    for (int w = 0; w < 2; w++) {
      a_st_[w] = sel(w) ? 0 : 2;
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
          a_ad_next_[w] = a_duty_[w] - kAdStep;
        } else if (a_duty_[w] >= max_duty_) {
          emit("mt,a,%s,%s,start,-1,-1,%.2f,%.0f", kWheel[w], dirName(dir_), (double)in.vbatt, (double)sp);
          emit("mt,a,%s,%s,stop,-1,-1,%.2f,%.0f", kWheel[w], dirName(dir_), (double)in.vbatt, (double)sp);
          a_st_[w] = 2;
        }
      } else if (a_st_[w] == 1) {
        if (a_duty_[w] <= a_ad_next_[w] && fabsf(sp) >= kStopMms) {  // speed while turning, ramping down
          a_ad_next_[w] -= kAdStep;
          emit("mt,ad,%s,%s,%.0f,%.3f,%.2f,%.0f", kWheel[w], dirName(dir_), (double)a_duty_[w],
               (double)volts(a_duty_[w], in), (double)in.vbatt, (double)sp);
        }
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
      hold(brake() ? kBrakeCoolDirMs : 1000, in);
    } else {
      finishA(in);
    }
  }
}

void MotorTest::finishA(const MtIn& in) {
  a_valid_ = true;
  a_run_ = true;
  emit("mt,sum,a,vb=%.2f,l_fwd=%.0f/%.0f,l_rev=%.0f/%.0f,r_fwd=%.0f/%.0f,r_rev=%.0f/%.0f (start/stop duty),hz=%.0f,drive=%u",
       (double)in.vbatt, (double)a_start_[0][0], (double)a_stop_[0][0], (double)a_start_[0][1],
       (double)a_stop_[0][1], (double)a_start_[1][0], (double)a_stop_[1][0], (double)a_start_[1][1],
       (double)a_stop_[1][1], (double)hz_, drive_);
  emit("mt,sum,a,volts,l_fwd=%.2f/%.2f,l_rev=%.2f/%.2f,r_fwd=%.2f/%.2f,r_rev=%.2f/%.2f,hz=%.0f,drive=%u",
       (double)a_start_v_[0][0], (double)a_stop_v_[0][0], (double)a_start_v_[0][1], (double)a_stop_v_[0][1],
       (double)a_start_v_[1][0], (double)a_stop_v_[1][0], (double)a_start_v_[1][1], (double)a_stop_v_[1][1],
       (double)hz_, drive_);
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
      drop_ref_ok_[w] = false;
    }
  }
  const float duty = max_duty_ * (float)b_level_ / (float)kBLevels;
  for (int w = 0; w < 2; w++) duty_[w] = sel(w) ? dir_ * duty : 0.0f;
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
    // speed-drop reference: first settled speed of this level, only when this duty really drives the
    // wheel (above this run's test-1 start duty, else 25 % of max). Below that the wheel is just coasting
    // down from the previous step, and "slowing" is not binding (false aborts on 2026-11-21: right wheel
    // at duty 256 with ref 596 mm/s, left at duty 938 with ref 701 mm/s).
    const int dI = di(dir_);
    const float drive_min = a_start_[w][dI] > 0 ? 1.05f * a_start_[w][dI] : 0.25f * max_duty_;
    if (sel(w) && !drop_ref_ok_[w] && el >= kBDropRefUs && fabsf(duty_[w]) >= drive_min &&
        fabsf(sp[w]) > kDropMinMms) {
      drop_ref_ok_[w] = true;
      drop_ref_[w] = fabsf(sp[w]);
      drop_on_[w] = false;
    }
  }
  if (el < kBHoldUs) return;

  const int d = di(dir_);
  for (int w = 0; w < 2; w++) {
    if (!sel(w)) continue;
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
      hold(brake() ? kBrakeCoolDirMs : 1500, in);  // coast down from full speed
    } else {
      finishB(in);
    }
  }
}

void MotorTest::finishB(const MtIn& in) {
  emit("mt,sum,b,vb=%.2f,top_mms l_fwd=%.0f l_rev=%.0f r_fwd=%.0f r_rev=%.0f,moves_at_V l_fwd=%.2f l_rev=%.2f "
       "r_fwd=%.2f r_rev=%.2f,hz=%.0f,drive=%u",
       (double)in.vbatt, (double)b_top_[0][0], (double)b_top_[0][1], (double)b_top_[1][0], (double)b_top_[1][1],
       (double)b_move_v_[0][0], (double)b_move_v_[0][1], (double)b_move_v_[1][0], (double)b_move_v_[1][1],
       (double)hz_, drive_);
  endPart(in);
}

// ------------------------------------------------------------------ test c: reversal

void MotorTest::runC(const MtIn& in, bool new_enc) {
  const int32_t enc[2] = {in.enc_l, in.enc_r};
  const float sp[2] = {in.speed_l, in.speed_r};
  if (need_begin_) {
    need_begin_ = false;
    t_phase_us_ = in.now_us;
    c_spin_ = true;
  }
  for (int w = 0; w < 2; w++) duty_[w] = sel(w) ? dir_ * c_duty_ : 0.0f;
  if (c_spin_) {
    if (in.now_us - t_phase_us_ >= kCSpinUs && new_enc) {
      // spin speed at this fixed duty vs the first spin the same way (binding / heat)
      const int d = di(dir_);
      for (int w = 0; w < 2; w++) {
        if (!sel(w)) continue;
        const float s = fabsf(sp[w]);
        if (c_ref_[w][d] <= 0.0f) {
          if (s > kDropMinMms) c_ref_[w][d] = s;
        } else if (s < kDropFrac * c_ref_[w][d]) {
          snprintf(anomaly_buf_, sizeof(anomaly_buf_), "speed_drop,%s,c,%s,ref=%.0f,now=%.0f,duty=%.0f", kWheel[w],
                   dirName(dir_), (double)c_ref_[w][d], (double)s, (double)c_duty_);
          anomaly_ = anomaly_buf_;
        }
      }
      dir_ = -dir_;  // reverse now
      for (int w = 0; w < 2; w++) duty_[w] = sel(w) ? dir_ * c_duty_ : 0.0f;
      c_spin_ = false;
      c_t0_us_ = in.now_us;
      for (int w = 0; w < 2; w++) {
        c_c0_[w] = c_ext_[w] = enc[w];
        c_ext_us_[w] = in.enc_us;
        c_det_[w] = !sel(w);
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
       "rev=%.0fms coast=%.1fdeg n=%u,hz=%.0f,drive=%u",
       (double)c_duty_, (double)volts(c_duty_, in), (double)m[0][0], (double)m[0][1], (double)m[0][2], c_cnt_[0],
       (double)m[1][0], (double)m[1][1], (double)m[1][2], c_cnt_[1], (double)hz_, drive_);
  endPart(in);
}

// ------------------------------------------------------------------ tests 5-7: low-speed tracking

// Signed raw duty for a target wheel speed with the controller of this part.
float MotorTest::lDuty(int w, float target, float dt, const MtIn& in) {
  const int d = target >= 0.0f ? 0 : 1;
  const float dz_avg = 0.5f * (in.dz[w][0] + in.dz[w][1]);
  const float k = in.k[w] > 1.0f ? in.k[w] : 1.0f;
  float duty = 0;
  if (part_ == 'o') {  // model 0: u = target / top speed at full duty, duty = v3 curve(u)
    const float span = in.vbatt - dz_avg;
    const float u = target / (k * (span > 0.5f ? span : 0.5f));
    duty = sgn(u) * motor_curve(fabsf(u) * 4095.0f, in.curve_exp, in.knee_in, in.knee_out);
  } else if (part_ == 'p') {  // model 1 + vcomp: u = target / top speed at v_ref, V = dz + (v_ref - dz) u
    const float span = in.v_ref - dz_avg;
    const float u = target / (k * (span > 0.5f ? span : 0.5f));
    const float v = sgn(u) * motor_dz_volts(fabsf(u), in.dz[w][d], in.v_ref, in.dz_band);
    duty = v / in.vbatt * 4095.0f;
  } else {  // closed loop
    SpeedLoopCfg c = in.sl;
    c.k_mms_per_v = k;
    c.dz_fwd = in.dz[w][0];
    c.dz_rev = in.dz[w][1];
    c.v_max = max_duty_ / 4095.0f * in.vbatt;
    duty = loop_[w].step(target, est_[w].speed(), dt, c) / in.vbatt * 4095.0f;
  }
  if (duty > max_duty_) duty = max_duty_;
  if (duty < -max_duty_) duty = -max_duty_;
  return duty;
}

void MotorTest::runL(const MtIn& in, bool new_enc) {
  if (need_begin_) {  // segment start
    need_begin_ = false;
    l_t0_us_ = in.now_us;
    l_tr_us_ = in.now_us;
    l_have_last_ = false;
    for (int w = 0; w < 2; w++) {
      memset(&ls_[w], 0, sizeof(ls_[w]));
      ls_[w].start_tgt = ls_[w].stop_tgt = ls_[w].t63u = ls_[w].t63d = -1;
      l_v_[w] = l_prev_v_[w] = 0;
      loop_[w].reset();
      duty_[w] = 0;
      if (l_seg_ == 0 && sel(w))
        emit("mt,lcfg,%s,%s,k=%.0f,dz=%.2f/%.2f,vref=%.1f,vb=%.2f,kp=%.4f,ki=%.3f,imax=%.1f,band=%.2f,hz=%.0f,drive=%u",
             ctlName(part_), kWheel[w], (double)in.k[w], (double)in.dz[w][0], (double)in.dz[w][1], (double)in.v_ref,
             (double)in.vbatt, (double)in.sl.kp, (double)in.sl.ki, (double)in.sl.i_max, (double)in.sl.band,
             (double)hz_, drive_);
    }
  }
  const float t = (float)(in.now_us - l_t0_us_) * 1e-3f;
  if (t >= kSegMs[l_seg_]) {
    finishSeg(in);
    return;
  }
  if (!new_enc) return;  // controller at the encoder rate (200 Hz)
  const bool first = !l_have_last_;
  const float dt = first ? 0.005f : (float)(in.enc_us - l_last_us_) * 1e-6f;
  l_last_us_ = in.enc_us;
  l_have_last_ = true;
  bool scored;
  const float tgt = segTarget(l_seg_, t, scored);
  const bool trace = in.now_us - l_tr_us_ >= kLTraceUs;
  if (trace) l_tr_us_ = in.now_us;
  for (int w = 0; w < 2; w++) {
    if (!sel(w)) {
      duty_[w] = 0;
      continue;
    }
    const float sp = est_[w].speed();
    duty_[w] = lDuty(w, tgt, dt, in);
    const float v = volts(duty_[w], in);
    LStat& s = ls_[w];
    if (scored) {
      const float e = tgt - sp;
      s.n++;
      s.se += e;
      s.se2 += e * e;
      if (fabsf(e) > s.emax) s.emax = fabsf(e);
    }
    if (fabsf(v) > kChatterV) {
      const int8_t sg = v > 0 ? 1 : -1;
      if (s.last_sign && sg != s.last_sign) s.flips++;
      s.last_sign = sg;
    }
    if (!first) {
      s.dv2 += (v - l_prev_v_[w]) * (v - l_prev_v_[w]);
      s.ndv++;
    }
    l_prev_v_[w] = v;
    l_v_[w] = v;
    if (l_seg_ <= 1) {  // ramps: target speed when the wheel starts / stops
      const float a = fabsf(sp), at = fabsf(tgt);
      if (!s.moved && a > kMoveMms) {
        s.moved = true;
        s.start_tgt = at;
      }
      if (s.moved && !s.stopped && t >= 4000 && a < kStopMms) {
        s.stopped = true;
        s.stop_tgt = at;
      }
    } else if (l_seg_ == 4) {  // steps 150 -> 250 -> 150: 63 % response time
      if (t >= 2000 && t < 3000 && s.t63u < 0 && sp >= 150.0f + 63.2f) s.t63u = t - 2000;
      if (t >= 3000 && s.t63d < 0 && sp <= 250.0f - 63.2f) s.t63d = t - 3000;
    }
    if (trace)
      emit("mt,tr,%s,%c,%s,%.0f,%.0f,%.0f,%.2f", ctlName(part_), kWheel[w][0], kSegName[l_seg_], (double)t,
           (double)tgt, (double)sp, (double)v);
  }
}

void MotorTest::finishSeg(const MtIn& in) {
  const float dur_s = kSegMs[l_seg_] * 1e-3f;
  for (int w = 0; w < 2; w++) {
    if (!sel(w)) continue;
    const LStat& s = ls_[w];
    const float n = s.n ? (float)s.n : 1.0f;
    const float rms = sqrtf(s.se2 / n);
    l_sum_[w][l_seg_] = rms;
    if (l_seg_ == 4) {
      l_t63_[w][0] = s.t63u;
      l_t63_[w][1] = s.t63d;
    }
    emit("mt,l,%s,%s,%s,rms=%.1f,mean=%.1f,max=%.0f,start=%.0f,stop=%.0f,t63u=%.0f,t63d=%.0f,flips=%.1f/s,"
         "dvrms=%.3f,vb=%.2f,hz=%.0f,drive=%u",
         ctlName(part_), kWheel[w], kSegName[l_seg_], (double)rms, (double)(s.se / n), (double)s.emax,
         (double)s.start_tgt, (double)s.stop_tgt, (double)s.t63u, (double)s.t63d, (double)(s.flips / dur_s),
         (double)(s.ndv ? sqrtf(s.dv2 / (float)s.ndv) : 0.0f), (double)in.vbatt, (double)hz_, drive_);
  }
  duty_[0] = duty_[1] = 0;
  if (++l_seg_ < kLSegs) {
    need_begin_ = true;
    hold(brake() ? kBrakeCoolSegMs : 1000, in);
    return;
  }
  for (int w = 0; w < 2; w++) {
    if (!sel(w)) continue;
    emit("mt,sum,l,%s,%s,rms r+=%.0f r-=%.0f s0=%.0f s150=%.0f st=%.0f,t63 up=%.0f down=%.0f,hz=%.0f,drive=%u",
         ctlName(part_), kWheel[w], (double)l_sum_[w][0], (double)l_sum_[w][1], (double)l_sum_[w][2],
         (double)l_sum_[w][3], (double)l_sum_[w][4], (double)l_t63_[w][0], (double)l_t63_[w][1], (double)hz_, drive_);
  }
  endPart(in);
}
