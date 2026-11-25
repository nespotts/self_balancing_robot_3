// LD19 lidar on the Teensy's Serial7 -> MSG_LIDAR_RAW frames to the Pi bridge (issue #100). The Pi's H618 UART can't make
// 230400 baud; the Teensy's LPUART can (24 MHz / (13 x 8) = 230769, +0.16 %). See docs/ld19.md.
//
// Pure C++ (no Arduino): the UART and the USB link are reached through LidarFwdIo, so test/host/lidar_fwd_sim.cpp runs this
// exact code against a fake UART / USB. main.cpp wires it to Serial7 / PiLink and calls service() from the background loop.
//
// One service() call does at most one frame: read <= LINK_LIDAR_RAW_MAX (180) bytes out of the UART buffer, CRC + copy them
// into the USB buffer. Never waits:
//  - fewer than 180 bytes waiting: nothing, until the first of them has waited kFlushUs (then a short frame);
//  - kMinGapUs since the last frame: nothing (<= 500 frames/s; the LD19 needs ~98/s);
//  - the USB buffer lacks room for the frame + kReserve (kept for telemetry / logs): the bytes stay in the UART buffer;
//    once that is nearly full (kHighWater) the chunk is read and dropped (counted, seq still moves on: the bridge sees a gap).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "protocol/link_protocol.h"

struct LidarFwdIo {
  virtual int available() = 0;                                 // bytes waiting in the UART receive buffer
  virtual int read(uint8_t* dst, int n) = 0;                   // copy up to n of them, returns how many
  virtual int room() = 0;                                      // free bytes in the USB transmit buffer now
  virtual bool send(const uint8_t* payload, uint8_t len) = 0;  // one MSG_LIDAR_RAW frame (false = not sent)
  virtual bool takeOverrun() { return false; }                 // UART hardware overrun since the last call (and clear it)
  // every chunk read, sent or dropped (#113: the Teensy's own LD19 parser); hole = bytes before it were lost
  virtual void tap(const uint8_t* data, int n, bool hole) { (void)data; (void)n; (void)hole; }
  virtual ~LidarFwdIo() {}
};

class LidarForwarder {
 public:
  static constexpr uint32_t kFlushUs = 12000;   // a partial chunk goes once its first byte waited this long
  static constexpr uint32_t kMinGapUs = 2000;   // at most one frame per 2 ms
  static constexpr int kReserve = 256;          // USB room left for the other frames (telemetry is 114 bytes)
  static constexpr uint32_t kSilentMs = 2000;   // no byte this long: "no bytes" log line
  static constexpr uint32_t kStatFirstMs = 5000, kStatMs = 30000;

  // rx_capacity: the UART ring buffer's size (64 + the memory added with addMemoryForRead); it holds rx_capacity - 1.
  // cycles_per_us: for noteCycles() (600 on the Teensy 4.1).
  void begin(int rx_capacity, uint32_t now_ms, uint32_t cycles_per_us = 600);
  void end() { on_ = false; }
  // #113: while the Teensy computes the obstacle sectors from these bytes, they may wait in the UART buffer for USB room
  // only this long (bytes): then they are read (parsed) and dropped for the Pi, so a stalled Pi link never delays the
  // navigator's lidar by more than ~20 ms. 0 = the default (the buffer nearly full, ~100 ms).
  void setMaxHold(int bytes);
  bool on() const { return on_; }

  // One background pass. Returns true if a frame went out.
  bool service(uint32_t now_us, uint32_t now_ms, LidarFwdIo& io);

  // A status line for the robot log (or nullptr): first bytes, every 30 s while bytes flow (5 s after they start), and
  // when they stop. "lidar: ... usb drop N" is read by the bridge (station/bridge/lidar.py) for the Health tile.
  // extra (#113): appends "; <its text>" to the periodic rates line (the sectors' own cost and counters).
  typedef int (*StatusExtra)(char* out, int n, uint32_t now_ms);
  const char* status(uint32_t now_ms, StatusExtra extra = nullptr);

  // CPU time of one service() call as measured by the caller (cycles), for the status line's "max ... us".
  void noteCycles(uint32_t cycles) {
    if (cycles > max_cycles_) max_cycles_ = cycles;
  }

  uint8_t seq() const { return seq_; }
  uint32_t bytes() const { return bytes_; }
  uint32_t frames() const { return frames_; }
  uint32_t drops() const { return drops_; }          // chunks read but not sent (USB link full)
  uint32_t overflows() const { return overflows_; }  // UART buffer full / overrun events
  int maxChunk() const { return max_chunk_; }

 private:
  int high_water_ = 0, base_high_water_ = 0, max_hold_ = 0, capacity_ = 0, left_ = 0;
  bool tap_hole_ = false;
  uint32_t cycles_per_us_ = 600;
  bool on_ = false, waiting_ = false, overflow_ = false, flowing_ = false, said_first_ = false, have_last_ = false;
  uint8_t seq_ = 0;
  uint32_t first_us_ = 0, last_us_ = 0;
  uint32_t last_byte_ms_ = 0, stat_ms_ = 0, stat_bytes_ = 0, stat_frames_ = 0;
  bool stat_first_ = true;
  uint32_t bytes_ = 0, frames_ = 0, drops_ = 0, overflows_ = 0, max_cycles_ = 0;
  int max_chunk_ = 0;
  char text_[LINK_MAX_PAYLOAD + 1];
};
