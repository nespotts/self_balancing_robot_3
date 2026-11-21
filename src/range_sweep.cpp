#include "range_sweep.h"

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "params.h"

// VL53L0X registers used directly (single-shot ranging, as the Pololu library does it)
static constexpr uint8_t REG_SYSRANGE_START = 0x00;
static constexpr uint8_t REG_INTERRUPT_CLEAR = 0x0B;
static constexpr uint8_t REG_INTERRUPT_STATUS = 0x13;  // then 0x14 range status ... 0x1E/0x1F range
static constexpr uint8_t DEVICE_STATUS_VALID = 11;

static constexpr uint8_t kAddr[2] = {0x30, 0x31};
static constexpr uint8_t kXshut[2] = {pins::RNG_XSHUT0, pins::RNG_XSHUT1};
static constexpr uint8_t kInt[2] = {pins::RNG_INT0, pins::RNG_INT1};

// Servo pulse: same range as PWMServo's defaults (v3 used PWMServo), 50 Hz, 12-bit duty.
static constexpr float SERVO_MIN_US = 544.0f, SERVO_MAX_US = 2400.0f, SERVO_PERIOD_US = 20000.0f;

static constexpr uint32_t STALE_MS = 2000;   // front/rear reading older than this -> 0

static Adafruit_VL53L0X g_dev[2];

static float wrap180(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  if (a <= -180.0f) a += 360.0f;
  return a;
}

static bool wr(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int rd(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(addr, n) != n) return -1;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return n;
}

static int servoDir() { return P(P_RNG_SERVO_DIR) < 0.0f ? -1 : 1; }
static int stepCount() {
  int n = (int)lroundf(P(P_RNG_STEPS));
  return n < 1 ? 1 : (n > 64 ? 64 : n);
}
// spacing between sweep positions, deg
static float stepDeg() {
  const float w = P(P_RNG_SWEEP_DEG);
  const int n = stepCount();
  if (w >= 179.5f) return w / n;  // full circle with two sensors: no repeated end point
  return n > 1 ? w / (n - 1) : 180.0f;
}

// ------------------------------------------------------------------ setup

bool RangeSweep::begin() {
  analogWriteFrequency(pins::RNG_SERVO, 50);
  pinMode(pins::RNG_SERVO, OUTPUT);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(kXshut[i], OUTPUT);
    digitalWrite(kXshut[i], LOW);
    pinMode(kInt[i], INPUT_PULLUP);
  }
  delay(10);
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(kXshut[i], HIGH);
    delay(10);
    // DEFAULT config: the library prints to Serial (= the Pi link) for LONG_RANGE. Ours: configure().
    ok_[i] = g_dev[i].begin(kAddr[i], false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    if (ok_[i]) {
      // stop variable (needed before every single-shot start)
      uint8_t sv = 0;
      ok_[i] = wr(kAddr[i], 0x80, 0x01) && wr(kAddr[i], 0xFF, 0x01) && wr(kAddr[i], 0x00, 0x00) &&
               rd(kAddr[i], 0x91, &sv, 1) == 1 && wr(kAddr[i], 0x00, 0x01) && wr(kAddr[i], 0xFF, 0x00) &&
               wr(kAddr[i], 0x80, 0x00);
      stop_var_[i] = sv;
    }
    if (ok_[i]) configure(i);
    else digitalWrite(kXshut[i], LOW);  // keep a dead sensor off the bus (it would sit at 0x29)
  }
  cfg_budget_ = P(P_RNG_BUDGET_MS);
  cfg_long_ = P(P_RNG_LONG_RANGE);
  Wire.setClock(I2C_CLOCK_HZ);  // the library re-begins Wire (100 kHz)
  started_ = true;
  state_ = MOVE;
  stat_ms_ = millis();
  return ok_[0] || ok_[1];
}

void RangeSweep::configure(uint8_t i) {
  Adafruit_VL53L0X& s = g_dev[i];
  const bool lr = P(P_RNG_LONG_RANGE) >= 0.5f;
  // long range (ST/Adafruit "LONG_RANGE"): lower return-signal limit, longer VCSEL pulses
  s.setLimitCheckEnable(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  s.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                       (FixPoint1616_t)((lr ? 0.1f : 0.25f) * 65536));
  s.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, lr ? 18 : 14);
  s.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, lr ? 14 : 10);
  s.setMeasurementTimingBudgetMicroSeconds((uint32_t)(P(P_RNG_BUDGET_MS) * 1000.0f));
  wr(kAddr[i], REG_INTERRUPT_CLEAR, 0x01);
}

// ------------------------------------------------------------------ servo / positions

float RangeSweep::positionAngle(int k) const {
  const float w = P(P_RNG_SWEEP_DEG);
  const int n = stepCount();
  if (n == 1 || w <= 0.0f) return 0.0f;
  return -0.5f * w + k * stepDeg();
}

void RangeSweep::commandServo(float head_deg, uint32_t now_us) {
  const float trim = P(P_RNG_SERVO_TRIM);
  const int dir = servoDir();
  const float scale = P(P_RNG_SERVO_SCALE);  // real head degrees per commanded degree
  const float lo = P(P_RNG_SERVO_MIN), hi = P(P_RNG_SERVO_MAX);
  float cmd = 90.0f + trim + dir * head_deg / scale;
  if (cmd < lo) cmd = lo;  // stay off the servo's end stops
  if (cmd > hi) cmd = hi;
  cur_head_deg_ = dir * scale * (cmd - 90.0f - trim);  // where the head really points if it clipped

  const float moved = fabsf(cmd - last_cmd_);  // last_cmd_ = -1e9 after start / off
  const float step = P(P_RNG_SERVO_HOLD) >= 0.0f ? 11.25f : stepDeg();
  if (moved < 0.01f) {
    settle_us_ = 0;
  } else {
    float f = moved / (step > 0.5f ? step : 0.5f);
    if (f < 1.0f) f = 1.0f;
    float us = P(P_RNG_SETTLE_MS) * 1000.0f * f;
    settle_us_ = (uint32_t)(us > 1.5e6f ? 1.5e6f : us);  // first move: unknown start, capped
    const float pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * cmd / 180.0f;
    analogWrite(pins::RNG_SERVO, (int)lroundf(pulse / SERVO_PERIOD_US * 4096.0f));
  }
  last_cmd_ = cmd;
  t_state_us_ = now_us;
}

void RangeSweep::nextPosition() {
  const int n = stepCount();
  if (k_ >= n) k_ = n - 1;
  if (n == 1) {
    k_ = 0;
    return;
  }
  if (k_ >= n - 1) dir_ = -1;
  else if (k_ <= 0) dir_ = 1;
  k_ += dir_;
}

// ------------------------------------------------------------------ ops

void RangeSweep::queue(uint8_t kind, uint8_t sensor, uint8_t reg, uint8_t val) {
  if (op_n_ >= sizeof(ops_) / sizeof(ops_[0])) return;
  ops_[(op_head_ + op_n_) % (sizeof(ops_) / sizeof(ops_[0]))] = Op{kind, sensor, reg, val};
  op_n_++;
}

bool RangeSweep::runOp(const Op& op, uint32_t now_us) {
  const uint8_t i = op.sensor, a = kAddr[i];
  if (op.kind == 0) {
    if (!wr(a, op.reg, op.val)) n_i2c_err_++;
    return true;
  }
  if (op.kind == 2) {  // poll the interrupt status (INT pin did not fire)
    uint8_t b = 0;
    if (rd(a, REG_INTERRUPT_STATUS, &b, 1) != 1) {
      n_i2c_err_++;
    } else if ((b & 0x07) != 0 && pending_[i] && !ready_[i]) {
      ready_[i] = true;
      int_ok_[i] = false;
      t_ready_us_[i] = now_us;
      pose_at_ready_[i] = pose_;
      queue(1, i);
    }
    return true;
  }
  // kind 1: read the result (0x13 status .. 0x1F range), then clear the interrupt
  uint8_t b[13];
  if (rd(a, REG_INTERRUPT_STATUS, b, sizeof(b)) != (int)sizeof(b)) {
    n_i2c_err_++;
    finishReading(i, 0, false, t_ready_us_[i]);
    queue(0, i, REG_INTERRUPT_CLEAR, 0x01);
    return true;
  }
  if ((b[0] & 0x07) == 0) {  // INT pin said ready but the chip has no new sample: poll from now on
    int_ok_[i] = false;
    ready_[i] = false;
    return true;
  }
  const uint8_t status = (b[1] >> 3) & 0x0F;
  const uint16_t mm = (uint16_t)(b[11] << 8 | b[12]);
  queue(0, i, REG_INTERRUPT_CLEAR, 0x01);
  const bool valid = status == DEVICE_STATUS_VALID && mm > 0 && mm <= P(P_RNG_MAX_MM);
  finishReading(i, mm, valid, t_ready_us_[i]);
  return true;
}

void RangeSweep::finishReading(uint8_t i, uint16_t mm, bool valid, uint32_t t_us) {
  pending_[i] = false;
  n_read_++;
  if (valid) n_valid_++;
  const RangePose& p = pose_at_ready_[i];
  RangeFrame f = {};
  f.t_us = t_us;
  f.x_mm = p.x_mm;
  f.y_mm = p.y_mm;
  f.heading_deg = p.heading_deg;
  f.pitch_deg = p.pitch_deg;
  f.sensor_angle_deg = wrap180(cur_head_deg_ + (i == 1 ? 180.0f : 0.0f));
  f.range_mm = mm;
  f.sensor_id = i;
  f.valid = valid ? 1 : 0;
  pushFrame(f);

  // front / rear: latest reading within half a sweep step of straight ahead / behind
  const float win = P(P_RNG_SERVO_HOLD) >= 0.0f ? 5.0f : fmaxf(0.5f * stepDeg() + 0.5f, 3.0f);
  const float a = fabsf(f.sensor_angle_deg);
  const uint32_t now_ms = millis();
  if (a <= win) {
    front_mm_ = valid ? mm : 0;
    front_ms_ = now_ms;
  }
  if (180.0f - a <= win) {
    rear_mm_ = valid ? mm : 0;
    rear_ms_ = now_ms;
  }
}

void RangeSweep::pushFrame(const RangeFrame& f) {
  const uint8_t cap = sizeof(out_) / sizeof(out_[0]);
  if (out_n_ == cap) {  // full: drop the oldest
    out_head_ = (out_head_ + 1) % cap;
    out_n_--;
  }
  out_[(out_head_ + out_n_) % cap] = f;
  out_n_++;
}

bool RangeSweep::peekFrame(RangeFrame& f) const {
  if (out_n_ == 0) return false;
  f = out_[out_head_];
  return true;
}

void RangeSweep::popFrame() {
  if (out_n_ == 0) return;
  out_head_ = (out_head_ + 1) % (sizeof(out_) / sizeof(out_[0]));
  out_n_--;
}

uint16_t RangeSweep::frontMm(uint32_t now_ms) const {
  return (front_ms_ != 0 && now_ms - front_ms_ < STALE_MS) ? front_mm_ : 0;
}
uint16_t RangeSweep::rearMm(uint32_t now_ms) const {
  return (rear_ms_ != 0 && now_ms - rear_ms_ < STALE_MS) ? rear_mm_ : 0;
}

// ------------------------------------------------------------------ fast path

void RangeSweep::service(uint32_t now_us, uint32_t budget_us, const RangePose& pose) {
  if (state_ == OFF) return;
  const uint32_t t0 = micros();
  pose_ = pose;
  const bool any = ok_[0] || ok_[1];
  const uint8_t cap = sizeof(ops_) / sizeof(ops_[0]);

  for (int pass = 0; pass < 2; pass++) {
    switch (state_) {
      case MOVE: {
        const float hold = P(P_RNG_SERVO_HOLD);
        // hold: servo command hold + trim (head = dir * scale * (hold - 90))
        const float head = hold >= 0.0f ? servoDir() * P(P_RNG_SERVO_SCALE) * (hold - 90.0f)
                                        : (any ? positionAngle(k_) : 0.0f);
        commandServo(head, now_us);
        state_ = SETTLE;
        break;
      }
      case SETTLE:
        if (any && op_n_ == 0 && now_us - t_state_us_ >= settle_us_) {
          for (uint8_t i = 0; i < 2; i++) {
            if (!ok_[i]) continue;
            queue(0, i, 0x80, 0x01);
            queue(0, i, 0xFF, 0x01);
            queue(0, i, 0x00, 0x00);
            queue(0, i, 0x91, stop_var_[i]);
            queue(0, i, 0x00, 0x01);
            queue(0, i, 0xFF, 0x00);
            queue(0, i, 0x80, 0x00);
            queue(0, i, REG_SYSRANGE_START, 0x01);
            pending_[i] = true;
            ready_[i] = false;
          }
          state_ = START;
        }
        break;
      case START:
        if (op_n_ == 0) {
          t_start_us_ = now_us;
          last_poll_us_ = now_us;
          state_ = WAIT;
        }
        break;
      case WAIT: {
        const uint32_t budget = (uint32_t)(P(P_RNG_BUDGET_MS) * 1000.0f);
        const uint32_t waited = now_us - t_start_us_;
        bool all_done = true;
        for (uint8_t i = 0; i < 2; i++) {
          if (!pending_[i]) continue;
          all_done = false;
          if (ready_[i]) continue;  // read queued
          if (int_ok_[i] && digitalReadFast(kInt[i]) == LOW) {
            ready_[i] = true;
            t_ready_us_[i] = now_us;
            pose_at_ready_[i] = pose_;
            queue(1, i);
          } else if (waited > 2 * budget + 60000) {  // no answer: report it and move on
            n_timeout_++;
            pose_at_ready_[i] = pose_;
            finishReading(i, 0, false, now_us);
            queue(0, i, REG_INTERRUPT_CLEAR, 0x01);
          } else if ((!int_ok_[i] ? waited > budget : waited > budget + 15000) &&
                     now_us - last_poll_us_ >= 4000 && op_n_ + 1 < cap) {
            last_poll_us_ = now_us;
            queue(2, i);
          }
        }
        if (all_done) {
          nextPosition();
          state_ = MOVE;
        }
        break;
      }
      default:
        break;
    }
    // I2C work within the budget (one op may run past it: <= ~0.4 ms)
    while (op_n_ > 0 && micros() - t0 < budget_us) {
      Op op = ops_[op_head_];
      op_head_ = (op_head_ + 1) % cap;
      op_n_--;
      runOp(op, now_us);
    }
  }
  const uint32_t dt = micros() - t0;
  if (dt > max_service_us_) max_service_us_ = dt;
}

// ------------------------------------------------------------------ slow path

const char* RangeSweep::serviceSlow(uint32_t now_ms, bool motors_off) {
  const bool want = P(P_RNG_ENABLE) >= 0.5f;
  if (want && !started_) {
    if (!motors_off) return nullptr;  // begin() blocks ~0.2 s
    begin();
    snprintf(status_, sizeof(status_), "rng: started, sensor0 %s, sensor1 %s", ok_[0] ? "ok" : "FAIL",
             ok_[1] ? "ok" : "FAIL");
    return status_;
  }
  if (!started_) return nullptr;
  if (!want && state_ != OFF) {
    state_ = OFF;
    op_n_ = 0;
    pending_[0] = pending_[1] = false;
    analogWrite(pins::RNG_SERVO, 0);  // no pulses: servo goes limp
    last_cmd_ = -1e9f;
    return "rng: off";
  }
  if (want && state_ == OFF) {
    for (uint8_t i = 0; i < 2; i++)
      if (ok_[i]) queue(0, i, REG_INTERRUPT_CLEAR, 0x01);
    state_ = MOVE;
    stat_ms_ = now_ms;
    stat_reads_ = n_read_;
    return "rng: on";
  }
  if (state_ == OFF) return nullptr;

  // timing budget / long range changed: reconfigure between measurements, motors off only
  if ((P(P_RNG_BUDGET_MS) != cfg_budget_ || P(P_RNG_LONG_RANGE) != cfg_long_) && motors_off &&
      (state_ == SETTLE || state_ == MOVE) && op_n_ == 0) {
    for (uint8_t i = 0; i < 2; i++)
      if (ok_[i]) configure(i);
    cfg_budget_ = P(P_RNG_BUDGET_MS);
    cfg_long_ = P(P_RNG_LONG_RANGE);
    snprintf(status_, sizeof(status_), "rng: budget %.0f ms, long range %s", cfg_budget_,
             cfg_long_ >= 0.5f ? "on" : "off");
    return status_;
  }

  // status line 5 s after start, then every 30 s
  const uint32_t period = stat_first_ ? 5000 : 30000;
  if (now_ms - stat_ms_ >= period) {
    const float s = (now_ms - stat_ms_) * 0.001f;
    snprintf(status_, sizeof(status_),
             "rng: %.1f readings/s, valid %lu/%lu, timeouts %lu, i2c err %lu, max slice %lu us, int %s/%s, "
             "front %u rear %u",
             (n_read_ - stat_reads_) / s, (unsigned long)n_valid_, (unsigned long)n_read_,
             (unsigned long)n_timeout_, (unsigned long)n_i2c_err_, (unsigned long)max_service_us_,
             ok_[0] ? (int_ok_[0] ? "pin" : "poll") : "-", ok_[1] ? (int_ok_[1] ? "pin" : "poll") : "-",
             frontMm(now_ms), rearMm(now_ms));
    stat_ms_ = now_ms;
    stat_reads_ = n_read_;
    stat_first_ = false;
    max_service_us_ = 0;
    return status_;
  }
  return nullptr;
}
