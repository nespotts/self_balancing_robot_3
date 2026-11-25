// One AS5600 wheel encoder: reads through an EncoderIo (bounded I2C on the robot, a mock in the host tests), unwraps
// the 12-bit angle into counts, filters corrupted reads, recovers a stuck bus, and latches a fault (issue #109).
// Pure logic: time is passed in.
//
//   read error            -> counted; ENC_RECOVER_AFTER NACKs in a row, or one busy / timeout / pin-low / arbitration
//                            loss: bus recovery (9 SCL clocks + STOP + I2C re-init), then the next read waits
//                            ENC_BACKOFF_MIN_MS, doubling to ENC_BACKOFF_MAX_MS while it keeps failing
//   implausible angle     -> rejected (not integrated): implies > ENC_MAX_MMS, or a speed change > ENC_JUMP_MMS from
//                            the previous read; ENC_RESYNC_REJECTS in a row: the new angle becomes the reference
//   ENC_FAULT_BAD bad reads in a row, or no good read for ENC_STALE_US  -> fault latched
//   fault latched         -> clears after ENC_CLEAR_MS with nothing but good reads
#pragma once
#include <stdint.h>

#include "config.h"
#include "i2c_status.h"

struct EncoderIo {
  // One bounded read of the 12-bit angle (0..4095). Never blocks for long (see src/i2c_bounded.h).
  virtual I2cStatus readAngle(uint16_t& raw) = 0;
  // Bus recovery: release SDA, 9 SCL clocks, STOP, re-init the I2C peripheral. Bounded (~0.15 ms).
  virtual bool recover() = 0;
  // Duration of the last readAngle(), us (0 = not measured).
  virtual uint32_t lastReadUs() const { return 0; }
  virtual ~EncoderIo() {}
};

struct EncoderStats {
  uint32_t reads = 0;       // read attempts
  uint32_t errors = 0;      // I2C errors
  uint32_t rejects = 0;     // implausible angles dropped
  uint32_t recoveries = 0;  // bus recoveries
  uint32_t resyncs = 0;     // reference re-taken (after a gap or rejected reads): that motion is lost
  uint32_t faults = 0;      // times the fault latched
  uint32_t read_us_sum = 0, read_us_max = 0, read_n = 0;  // read timing (reset by takeTiming())
};

class WheelEncoder {
 public:
  void attach(EncoderIo* io) { io_ = io; }
  // One update: a read (skipped while a recovery backoff runs). True = a new accepted reading.
  bool update(uint32_t t_us, float mm_per_count);
  int32_t counts() const { return counts_; }
  // Windowed speed over the last ENCODER_WINDOW accepted reads; held through single bad reads.
  float speedMms() const { return speed_mms_; }
  bool speedValid() const { return ring_n_ >= 2; }
  bool faulted() const { return faulted_; }
  // I2C errors / rejects / recoveries within the last second (the shared left bus backs off, the fault flags show it).
  bool troubled(uint32_t t_us) const { return have_bad_ && t_us - last_bad_us_ < 1000000u; }
  // A recovery backoff is running or the bus failed recently: other users of the same bus should keep off it.
  bool busSuspect(uint32_t t_us) const { return backoff_ || (have_io_err_ && t_us - last_io_err_us_ < 1000000u); }
  uint8_t lastError() const { return last_err_; }
  const EncoderStats& stats() const { return st_; }
  void takeTiming(uint32_t& mean_us, uint32_t& max_us);
  // True once each time the latched fault sets or clears.
  bool takeFaultChanged() {
    const bool c = fault_changed_;
    fault_changed_ = false;
    return c;
  }

 private:
  void bad(uint32_t t_us);
  void resync(uint16_t raw, uint32_t t_us);
  void pushRing(uint32_t t_us);
  void updateFault(uint32_t t_us);

  EncoderIo* io_ = nullptr;
  EncoderStats st_;
  bool started_ = false;   // first update() seen (staleness counts from it)
  bool have_ = false;      // a reference angle exists
  uint16_t last_raw_ = 0;
  int32_t counts_ = 0;
  uint32_t good_us_ = 0;   // last accepted (or resync) read
  float prev_v_ = 0;       // speed implied by the last accepted read (jump check)
  bool prev_v_ok_ = false;
  float speed_mms_ = 0;
  uint8_t bad_run_ = 0, nack_run_ = 0, rej_run_ = 0;
  bool backoff_ = false, after_recover_ = false;
  uint32_t backoff_ms_ = 0, next_try_us_ = 0;
  bool have_bad_ = false, have_io_err_ = false;
  uint32_t last_bad_us_ = 0, last_io_err_us_ = 0;
  uint8_t last_err_ = I2C_OK;
  bool faulted_ = false, fault_changed_ = false;
  int32_t ring_c_[ENCODER_WINDOW] = {};
  uint32_t ring_t_[ENCODER_WINDOW] = {};
  uint8_t ring_head_ = 0, ring_n_ = 0;
};
