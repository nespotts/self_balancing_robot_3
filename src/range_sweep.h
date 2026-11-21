// Range sweep (issue #7): two VL53L0X back to back on a servo head, swept to build a 360 deg scan.
//
// Never blocks the balance loop: service() runs right after each balance step and does at most a
// time budget of short I2C transactions (Wire is shared with the left AS5600 encoder).
// Ranging is single-shot, started with raw register writes and finished when the sensor's INT pin
// goes low (fallback: polling its interrupt status register).
//
// Angles: head angle a (deg, + = CCW, 0 = robot forward) is where sensor 0 points; sensor 1 points
// at a + 180. Servo command = 90 + rng.servo_dir * a + rng.servo_trim_deg (0..180, 90 = centre).
// See docs/firmware.md "Range sweep".
#pragma once
#include <stdint.h>

#include "protocol/link_protocol.h"

struct RangePose {
  float x_mm = 0, y_mm = 0, heading_deg = 0, pitch_deg = 0;
};

class RangeSweep {
 public:
  // Sensors + servo (blocking, ~0.2 s). Returns true if at least one sensor answered.
  bool begin();
  bool started() const { return started_; }
  bool sensorOk(uint8_t i) const { return i < 2 && ok_[i]; }

  // Right after a balance step: bounded slice of work (I2C, servo). budget_us = time it may use.
  void service(uint32_t now_us, uint32_t budget_us, const RangePose& pose);
  // Background, may block a few ms when motors_off: late enable, sensor reconfig, status log.
  // Returns a status text to log (or nullptr).
  const char* serviceSlow(uint32_t now_ms, bool motors_off);

  // Frames for the Pi link, oldest first.
  bool peekFrame(RangeFrame& f) const;
  void popFrame();

  // Latest reading nearest straight ahead / behind (0 = none in the last 2 s or no target).
  uint16_t frontMm(uint32_t now_ms) const;
  uint16_t rearMm(uint32_t now_ms) const;

 private:
  enum State : uint8_t { OFF, MOVE, SETTLE, START, WAIT, READ };
  struct Op {
    uint8_t kind;  // 0 write reg, 1 read result, 2 poll status
    uint8_t sensor, reg, val;
  };

  void configure(uint8_t i);  // timing budget + long range (blocking)
  void commandServo(float head_deg, uint32_t now_us);
  float positionAngle(int k) const;
  void nextPosition();
  void queue(uint8_t kind, uint8_t sensor, uint8_t reg = 0, uint8_t val = 0);
  bool runOp(const Op& op, uint32_t now_us);
  void finishReading(uint8_t i, uint16_t mm, bool valid, uint32_t t_us);
  void pushFrame(const RangeFrame& f);

  bool started_ = false, ok_[2] = {false, false};
  uint8_t stop_var_[2] = {0, 0};
  State state_ = OFF;
  uint32_t t_state_us_ = 0, settle_us_ = 0, t_start_us_ = 0, last_poll_us_ = 0;
  // sweep position
  int k_ = 0, dir_ = 1;
  float head_deg_ = 0, cmd_head_deg_ = 1e9f, cmd_trim_ = 0, cmd_dir_ = 0;
  float cur_head_deg_ = 0;   // head angle the current measurement is taken at
  bool pending_[2] = {false, false}, ready_[2] = {false, false};
  bool int_ok_[2] = {true, true};
  RangePose pose_at_ready_[2];
  RangePose pose_;
  uint32_t t_ready_us_[2] = {0, 0};
  // op queue
  Op ops_[24];
  uint8_t op_head_ = 0, op_n_ = 0;
  // out frames
  RangeFrame out_[24];
  uint8_t out_head_ = 0, out_n_ = 0;
  // front/back
  uint16_t front_mm_ = 0, rear_mm_ = 0;
  uint32_t front_ms_ = 0, rear_ms_ = 0;
  // applied sensor config
  float cfg_budget_ = -1, cfg_long_ = -1;
  // stats
  uint32_t n_read_ = 0, n_valid_ = 0, n_timeout_ = 0, n_i2c_err_ = 0, max_service_us_ = 0;
  uint32_t stat_ms_ = 0, stat_reads_ = 0;
  bool stat_first_ = true;
  char status_[160];
};
