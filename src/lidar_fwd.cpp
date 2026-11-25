#include "lidar_fwd.h"

#include <stdio.h>

#include "link_codec.h"

void LidarForwarder::begin(int rx_capacity, uint32_t now_ms, uint32_t cycles_per_us) {
  capacity_ = rx_capacity;
  // the UART buffer this full while the USB link has no room: read the oldest chunk and drop it (two chunks of margin,
  // ~20 ms at the LD19's 17.6 kB/s, before the receive interrupt itself has to drop bytes)
  high_water_ = rx_capacity - 1 - 2 * LINK_LIDAR_RAW_MAX;
  if (high_water_ < LINK_LIDAR_RAW_MAX) high_water_ = LINK_LIDAR_RAW_MAX;
  base_high_water_ = high_water_;
  setMaxHold(max_hold_);
  tap_hole_ = false;
  cycles_per_us_ = cycles_per_us ? cycles_per_us : 1;
  on_ = true;
  waiting_ = overflow_ = flowing_ = said_first_ = have_last_ = false;
  left_ = 0;
  stat_ms_ = last_byte_ms_ = now_ms;
  stat_first_ = true;
}

void LidarForwarder::setMaxHold(int bytes) {
  max_hold_ = bytes > 0 ? bytes : 0;
  high_water_ = base_high_water_;
  if (max_hold_ && max_hold_ < high_water_) high_water_ = max_hold_ < LINK_LIDAR_RAW_MAX ? LINK_LIDAR_RAW_MAX : max_hold_;
}

bool LidarForwarder::service(uint32_t now_us, uint32_t now_ms, LidarFwdIo& io) {
  if (!on_) return false;
  if (io.takeOverrun()) {
    if (!overflow_) overflows_++;
    overflow_ = tap_hole_ = true;
  }
  const int avail = io.available();
  if (avail > left_) last_byte_ms_ = now_ms;  // new bytes since the last pass
  left_ = avail;
  if (avail <= 0) {
    left_ = 0;
    waiting_ = false;
    return false;
  }
  if (avail >= capacity_ - 1) {  // full: the receive interrupt drops every new byte now
    if (!overflow_) overflows_++;
    overflow_ = tap_hole_ = true;
  }
  if (!waiting_) {
    waiting_ = true;
    first_us_ = now_us;
  }
  if (avail < LINK_LIDAR_RAW_MAX && (uint32_t)(now_us - first_us_) < kFlushUs) return false;
  if (have_last_ && (uint32_t)(now_us - last_us_) < kMinGapUs) return false;
  const int n = avail < LINK_LIDAR_RAW_MAX ? avail : LINK_LIDAR_RAW_MAX;
  const bool room = io.room() >= n + 2 + (int)LINK_FRAME_OVERHEAD + kReserve;
  if (!room && avail < high_water_) return false;  // keep the bytes in the UART buffer until the link has room

  uint8_t buf[2 + LINK_LIDAR_RAW_MAX];
  buf[0] = seq_;
  buf[1] = overflow_ ? LINK_LIDAR_RAW_OVERFLOW : 0;
  const int got = io.read(buf + 2, n);
  if (got <= 0) return false;
  io.tap(buf + 2, got, tap_hole_);  // the Teensy's own parser (#113) gets every chunk, sent or not
  tap_hole_ = false;
  seq_++;  // also for a dropped chunk: the bridge sees the gap
  have_last_ = true;
  last_us_ = now_us;
  bytes_ += (uint32_t)got;
  if (got > max_chunk_) max_chunk_ = got;
  left_ = avail - got;
  waiting_ = left_ > 0;
  first_us_ = now_us;
  if (room && io.send(buf, (uint8_t)(2 + got))) {
    frames_++;
    overflow_ = false;  // reported (a dropped chunk carries it over to the next one)
    return true;
  }
  drops_++;
  return false;
}

const char* LidarForwarder::status(uint32_t now_ms, StatusExtra extra) {
  if (!on_) return nullptr;
  if (!said_first_) {
    if (bytes_ == 0) return nullptr;
    said_first_ = flowing_ = true;
    stat_ms_ = now_ms;
    stat_bytes_ = bytes_;
    stat_frames_ = frames_;
    stat_first_ = true;
    return "lidar: LD19 bytes on Serial7 (D28), forwarding them as MSG_LIDAR_RAW";
  }
  // signed: service() may have stamped last_byte_ms_ with a millis() a little newer than this now_ms (the background
  // loop's), and an unsigned difference would then wrap to "silent for 49 days" (false alarm every ~3 s, seen 2026-11-24)
  const int32_t quiet_ms = (int32_t)(now_ms - last_byte_ms_);
  if (flowing_ && quiet_ms >= (int32_t)kSilentMs) {
    flowing_ = false;
    snprintf(text_, sizeof(text_), "lidar: no bytes on Serial7 for %lu s (LD19 power, Tx wire on D28?); usb drop %lu, overflow %lu",
             (unsigned long)(kSilentMs / 1000), (unsigned long)drops_, (unsigned long)overflows_);
    return text_;
  }
  if (!flowing_) {
    if (quiet_ms >= (int32_t)kSilentMs) return nullptr;
    flowing_ = true;
    stat_ms_ = now_ms;
    stat_bytes_ = bytes_;
    stat_frames_ = frames_;
    stat_first_ = true;
    return "lidar: LD19 bytes on Serial7 again";
  }
  const uint32_t period = stat_first_ ? kStatFirstMs : kStatMs;
  if (now_ms - stat_ms_ < period) return nullptr;
  const float s = (now_ms - stat_ms_) * 0.001f;
  int k = snprintf(text_, sizeof(text_),
                   "lidar: %.1f kB/s, %.0f frames/s, usb drop %lu, overflow %lu, max %d B / %.1f us per pass",
                   (double)((bytes_ - stat_bytes_) / s / 1000.0f), (double)((frames_ - stat_frames_) / s),
                   (unsigned long)drops_, (unsigned long)overflows_, max_chunk_,
                   (double)((float)max_cycles_ / (float)cycles_per_us_));
  if (extra && k > 0 && k + 3 < (int)sizeof(text_)) {  // #113: "; sectors ..." (the Teensy's own obstacle sectors)
    text_[k++] = ';';
    text_[k++] = ' ';
    text_[k] = 0;
    extra(text_ + k, (int)sizeof(text_) - k, now_ms);
  }
  stat_ms_ = now_ms;
  stat_bytes_ = bytes_;
  stat_frames_ = frames_;
  stat_first_ = false;
  max_cycles_ = 0;
  max_chunk_ = 0;
  return text_;
}
