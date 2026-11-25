#include "wheel_encoder.h"

#include <math.h>

void WheelEncoder::bad(uint32_t t_us) {
  if (bad_run_ < 255) bad_run_++;
  have_bad_ = true;
  last_bad_us_ = t_us;
}

void WheelEncoder::resync(uint16_t raw, uint32_t t_us) {
  if (have_) st_.resyncs++;
  have_ = true;
  last_raw_ = raw;
  good_us_ = t_us;
  prev_v_ok_ = false;
  rej_run_ = 0;
  ring_n_ = 0;
  ring_head_ = 0;
  speed_mms_ = 0;
  pushRing(t_us);
}

void WheelEncoder::pushRing(uint32_t t_us) {
  ring_c_[ring_head_] = counts_;
  ring_t_[ring_head_] = t_us;
  ring_head_ = (ring_head_ + 1) % ENCODER_WINDOW;
  if (ring_n_ < ENCODER_WINDOW) ring_n_++;
}

void WheelEncoder::updateFault(uint32_t t_us) {
  const bool stale = t_us - good_us_ > ENC_STALE_US;
  if (stale) {  // nothing read (failing, or waiting out a recovery backoff): not clean either
    have_bad_ = true;
    last_bad_us_ = t_us;
  }
  if (bad_run_ >= ENC_FAULT_BAD || stale) {
    if (!faulted_) {
      faulted_ = true;
      fault_changed_ = true;
      st_.faults++;
    }
  } else if (faulted_ && t_us - last_bad_us_ >= ENC_CLEAR_MS * 1000u) {
    faulted_ = false;
    fault_changed_ = true;
  }
}

bool WheelEncoder::update(uint32_t t_us, float mm_per_count) {
  if (!started_) {
    started_ = true;
    good_us_ = t_us;
  }
  if (!io_ || (backoff_ && (int32_t)(t_us - next_try_us_) < 0)) {  // recovery backoff: leave the bus alone
    if (backoff_) {
      have_bad_ = true;
      last_bad_us_ = t_us;
    }
    updateFault(t_us);
    return false;
  }
  uint16_t raw = 0;
  const I2cStatus s = io_->readAngle(raw);
  st_.reads++;
  const uint32_t us = io_->lastReadUs();
  if (us) {
    st_.read_us_sum += us;
    st_.read_n++;
    if (us > st_.read_us_max) st_.read_us_max = us;
  }

  if (s != I2C_OK) {
    st_.errors++;
    last_err_ = s;
    have_io_err_ = true;
    last_io_err_us_ = t_us;
    bad(t_us);
    if (s == I2C_NACK && nack_run_ < 255) nack_run_++;
    // a stuck bus never fixes itself (issue #109: minutes until a power cycle): recover, then back off
    if (after_recover_ || i2c_status_stuck(s) || nack_run_ >= ENC_RECOVER_AFTER) {
      io_->recover();
      st_.recoveries++;
      backoff_ms_ = backoff_ms_ ? backoff_ms_ * 2 : ENC_BACKOFF_MIN_MS;
      if (backoff_ms_ > ENC_BACKOFF_MAX_MS) backoff_ms_ = ENC_BACKOFF_MAX_MS;
      backoff_ = true;
      after_recover_ = true;
      next_try_us_ = t_us + backoff_ms_ * 1000u;
      nack_run_ = 0;
    }
    updateFault(t_us);
    return false;
  }
  nack_run_ = 0;
  backoff_ = after_recover_ = false;
  backoff_ms_ = 0;
  raw &= 0x0FFF;

  if (!have_ || t_us - good_us_ > ENC_STALE_US) {  // first read, or after a gap: unwrapping across it is a guess
    resync(raw, t_us);
    updateFault(t_us);
    return false;
  }
  // unwrap: < half a revolution between reads (true below ~7000 rpm at 200 Hz)
  int32_t d = (int32_t)raw - (int32_t)last_raw_;
  if (d > 2047) d -= 4096;
  if (d < -2048) d += 4096;
  const uint32_t dt_us = t_us - good_us_;
  const float v = dt_us ? (float)d * mm_per_count * 1e6f / (float)dt_us : (d ? 1e9f : 0.0f);
  // plausibility (issue #109): a glitched transfer gives a random angle; a huge delta must not reach the odometry
  if (fabsf(v) > ENC_MAX_MMS || (prev_v_ok_ && fabsf(v - prev_v_) > ENC_JUMP_MMS)) {
    st_.rejects++;
    bad(t_us);
    if (++rej_run_ >= ENC_RESYNC_REJECTS) resync(raw, t_us);  // it really is there now: carry on from it
    updateFault(t_us);
    return false;
  }
  rej_run_ = 0;
  bad_run_ = 0;
  counts_ += d;
  last_raw_ = raw;
  good_us_ = t_us;
  prev_v_ = v;
  prev_v_ok_ = true;
  pushRing(t_us);
  // windowed speed: change over the oldest sample still in the ring
  const uint8_t oldest = (ring_head_ + ENCODER_WINDOW - ring_n_) % ENCODER_WINDOW;
  const uint32_t wdt = t_us - ring_t_[oldest];
  speed_mms_ = wdt > 0 ? (float)(counts_ - ring_c_[oldest]) * mm_per_count * 1e6f / (float)wdt : 0.0f;
  updateFault(t_us);
  return true;
}

void WheelEncoder::takeTiming(uint32_t& mean_us, uint32_t& max_us) {
  mean_us = st_.read_n ? st_.read_us_sum / st_.read_n : 0;
  max_us = st_.read_us_max;
  st_.read_us_sum = st_.read_us_max = st_.read_n = 0;
}
