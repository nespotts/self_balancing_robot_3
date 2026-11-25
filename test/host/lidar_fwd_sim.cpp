// Host test for the LD19 forwarder (issue #100, src/lidar_fwd.cpp): real LD19 packets (CRC8) arriving at 230400 baud into a
// model of the Teensy's Serial7 (64 + 2048 byte ring, 4-byte FIFO, overrun while interrupts are off), the main loop's
// passes around a 400 Hz balance step (with the step-phase window of main.cpp's lidarService), a USB link that can stall,
// and the frames decoded again with the firmware's own LinkParser. Checks the bytes come out identical, the seq / overflow
// rules, drops instead of blocking, the status lines, and times one service() call. Build + run (any C++17 compiler):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/lidar_fwd_sim.cpp src/lidar_fwd.cpp src/link_codec.cpp
//     -o /tmp/lidar_fwd && /tmp/lidar_fwd
//   MSVC: cl /std:c++17 /O2 /EHsc /Zp1 /FI <a header with #define __attribute__(x)> /I src /I include ...
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <deque>
#include <vector>

#include "lidar_fwd.h"
#include "link_codec.h"

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

// ------------------------------------------------------------------ LD19 packets (manual V2.3, section 3)

static uint8_t g_crc_table[256];
static void makeCrcTable() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    g_crc_table[i] = c;
  }
}
static uint8_t crc8(const uint8_t* d, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c = g_crc_table[c ^ d[i]];
  return c;
}
// the manual's example packet (section 3.3)
static const uint8_t kExample[47] = {0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00, 0xE5, 0xD5,
                                     0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2, 0xC7, 0x00,
                                     0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5, 0xBE, 0x82, 0x3A, 0x1A, 0x50};

static void ld19Packet(uint32_t k, uint8_t* p) {  // packet k of a 10 Hz turn (37.5 packets, 0.8 deg per point)
  p[0] = 0x54;
  p[1] = 0x2C;
  p[2] = 3600 & 0xFF;
  p[3] = 3600 >> 8;
  const uint32_t a0 = (k * 12u * 80u) % 36000u, a1 = (a0 + 11u * 80u) % 36000u;
  p[4] = a0 & 0xFF;
  p[5] = (uint8_t)(a0 >> 8);
  for (int i = 0; i < 12; i++) {
    const uint16_t d = (uint16_t)(1000 + (k * 7 + i * 13) % 3000);
    p[6 + 3 * i] = d & 0xFF;
    p[7 + 3 * i] = d >> 8;
    p[8 + 3 * i] = (uint8_t)(100 + i);
  }
  p[42] = a1 & 0xFF;
  p[43] = (uint8_t)(a1 >> 8);
  const uint32_t ts = (k * 8 / 3) % 30000;  // ms
  p[44] = ts & 0xFF;
  p[45] = (uint8_t)(ts >> 8);
  p[46] = crc8(p, 46);
}

// count CRC-valid packets in a byte stream (resyncing like ld19.Ld19Stream)
static int countPackets(const std::vector<uint8_t>& s) {
  int n = 0;
  size_t i = 0;
  while (i + 47 <= s.size()) {
    if (s[i] == 0x54 && s[i + 1] == 0x2C && crc8(&s[i], 46) == s[i + 46]) {
      n++;
      i += 47;
    } else {
      i++;
    }
  }
  return n;
}

// ------------------------------------------------------------------ Teensy model

struct Sim : LidarFwdIo {
  // Serial7: ring (capacity - 1 bytes usable, like HardwareSerialIMXRT), FIFO of 4 while interrupts are off
  int capacity = 64 + 2048;
  std::deque<uint8_t> ring;
  int fifo = 0;
  std::deque<uint8_t> fifo_bytes;
  bool irq_off = false, or_flag = false;
  uint32_t lost_isr = 0, lost_or = 0;
  // the LD19
  std::vector<uint8_t> sent_stream;      // every byte the lidar put on the wire
  std::vector<uint32_t> byte_t;          // ... and when
  // USB
  int usb_cap = 4 * 2048, usb_used = 0;
  bool usb_blocked = false;
  LinkParser parser;
  std::vector<uint8_t> rx_data;          // reassembled from MSG_LIDAR_RAW frames
  std::vector<int> rx_seq, rx_flags, rx_len;
  std::vector<uint32_t> rx_t;
  uint32_t now = 0;
  uint32_t max_latency_us = 0;
  size_t latency_idx = 0;
  int other_frames = 0;

  void uartByte(uint8_t b) {
    if (irq_off) {
      if (fifo_bytes.size() < 4) fifo_bytes.push_back(b);
      else {
        or_flag = true;
        lost_or++;
      }
      return;
    }
    if ((int)ring.size() < capacity - 1) ring.push_back(b);
    else lost_isr++;
  }
  void irqOn() {
    irq_off = false;
    while (!fifo_bytes.empty()) {
      uartByte(fifo_bytes.front());
      fifo_bytes.pop_front();
    }
  }
  int available() override { return (int)ring.size(); }
  int read(uint8_t* dst, int n) override {
    int k = 0;
    while (k < n && !ring.empty()) {
      dst[k++] = ring.front();
      ring.pop_front();
    }
    return k;
  }
  int room() override { return usb_cap - usb_used; }
  bool rawSend(uint8_t type, const uint8_t* p, uint8_t len) {
    uint8_t buf[LINK_MAX_PAYLOAD + LINK_FRAME_OVERHEAD];
    const size_t n = link_encode(type, p, len, buf);
    if (n == 0 || (int)n > room()) return false;
    usb_used += (int)n;
    for (size_t i = 0; i < n; i++) {
      if (!parser.feed(buf[i])) continue;
      if (parser.type() == MSG_LIDAR_RAW) {
        rx_seq.push_back(parser.payload()[0]);
        rx_flags.push_back(parser.payload()[1]);
        rx_len.push_back(parser.len() - 2);
        rx_t.push_back(now);
        rx_data.insert(rx_data.end(), parser.payload() + 2, parser.payload() + parser.len());
      } else {
        other_frames++;
      }
    }
    return true;
  }
  bool send(const uint8_t* p, uint8_t len) override { return rawSend(MSG_LIDAR_RAW, p, len); }
  bool takeOverrun() override {
    const bool o = or_flag;
    or_flag = false;
    return o;
  }
  void usbTick() {  // the Pi reads ~1 MB/s when it reads at all
    if (!usb_blocked) usb_used = usb_used > 1 ? usb_used - 1 : 0;
  }
};

struct RunOpts {
  uint32_t seconds = 10;
  uint32_t usb_block_from_ms = 0, usb_block_ms = 0;   // the Pi stops reading for a while
  uint32_t stall_from_ms = 0, stall_ms = 0;           // the main loop blocked (no background passes)
  uint32_t irq_off_from_ms = 0, irq_off_ms = 0;       // interrupts off (flash write)
  bool stop_lidar_at_end = false;
};

struct Result {
  Sim sim;
  LidarForwarder fwd;
  std::vector<const char*> lines;
  std::vector<std::vector<char>> line_text;
  int max_bytes_per_pass = 0;
  int bad_window_calls = 0;   // service() called outside the step-phase window (must stay 0)
};

// Teensy main loop: a 400 Hz balance step that keeps the CPU for 300 us, background passes every 3 us otherwise.
static void run(Result& r, const RunOpts& o) {
  Sim& s = r.sim;
  LidarForwarder& f = r.fwd;
  f.begin(s.capacity, 0);
  const uint32_t end_us = o.seconds * 1000000u;
  const double byte_us = 10.0 * 1e6 / 230400.0;        // 43.4 us per byte (8N1)
  const double pkt_us = 1e6 / 375.0;                   // 4500 points/s / 12
  uint32_t k = 0;
  int pkt_pos = 47;
  uint8_t pkt[47];
  double next_byte = 0.0;
  uint32_t last_step_us = 0, busy_until = 0, next_pass = 0;
  const uint32_t period = 2500, window = 1500;
  for (uint32_t t = 0; t < end_us; t++) {
    s.now = t;
    const uint32_t ms = t / 1000;
    // the lidar: 47-byte packets every 2.667 ms, bytes 43.4 us apart
    const bool lidar_on = !(o.stop_lidar_at_end && t > end_us - 3000000u);
    if (lidar_on && t >= next_byte) {
      if (pkt_pos >= 47) {
        if (t >= k * pkt_us) {
          if (k == 0) memcpy(pkt, kExample, 47);  // the manual's own packet first
          else ld19Packet(k, pkt);
          k++;
          pkt_pos = 0;
        }
      }
      if (pkt_pos < 47) {
        s.uartByte(pkt[pkt_pos]);
        s.sent_stream.push_back(pkt[pkt_pos]);
        s.byte_t.push_back(t);
        pkt_pos++;
        next_byte += byte_us;
      } else {
        next_byte = t + 1;
      }
    }
    const bool irq_off = o.irq_off_ms && ms >= o.irq_off_from_ms && ms < o.irq_off_from_ms + o.irq_off_ms;
    if (irq_off) s.irq_off = true;
    else if (s.irq_off) s.irqOn();
    s.usb_blocked = o.usb_block_ms && ms >= o.usb_block_from_ms && ms < o.usb_block_from_ms + o.usb_block_ms;
    s.usbTick();
    // telemetry: 114 bytes every 10 ms, sent before the lidar in sendOutput (not checked here, it just takes room)
    if (t % 10000 == 5) {
      uint8_t tl[108] = {};
      s.rawSend(MSG_TELEMETRY, tl, sizeof(tl));
    }
    if (t % period == 0) {  // IMU sample: the balance step
      last_step_us = t;
      busy_until = t + 300;
    }
    const bool stalled = o.stall_ms && ms >= o.stall_from_ms && ms < o.stall_from_ms + o.stall_ms;
    if (t % 1000 == 999 && !stalled) {  // the background's slow section: status lines
      if (const char* line = f.status(ms)) {
        r.line_text.emplace_back(line, line + strlen(line) + 1);
        printf("    [%6.2f s] %s\n", t / 1e6, line);
      }
    }
    if (t < busy_until || stalled || t < next_pass) continue;
    next_pass = t + 3;
    // main.cpp lidarService(): only in the first 60 % of the period after a step
    const uint32_t since = t - last_step_us;
    if (since >= window && since <= 25000) continue;
    if (since >= window) r.bad_window_calls++;
    const int before = s.available();
    const size_t rx0 = s.rx_data.size();
    f.service(t, ms, s);
    const int took = before - s.available();
    if (took > r.max_bytes_per_pass) r.max_bytes_per_pass = took;
    // latency: from a byte's arrival on the wire to its frame going out
    for (size_t i = rx0; i < s.rx_data.size() && s.latency_idx < s.byte_t.size(); i++) {
      const uint32_t lat = t - s.byte_t[s.latency_idx++];
      if (lat > s.max_latency_us) s.max_latency_us = lat;
    }
  }
}

static int seqGaps(const std::vector<int>& seq) {
  int gaps = 0;
  for (size_t i = 1; i < seq.size(); i++) gaps += (uint8_t)(seq[i] - seq[i - 1] - 1);
  return gaps;
}

static bool anyLine(const Result& r, const char* needle) {
  for (const auto& l : r.line_text)
    if (strstr(l.data(), needle)) return true;
  return false;
}

int main() {
  makeCrcTable();
  CHECK(crc8(kExample, 46) == kExample[46]);  // the manual's CRC table / example
  {  // the link's table CRC-16 = CRC-16/CCITT-FALSE (check value) = the old bitwise loop
    CHECK(link_crc16((const uint8_t*)"123456789", 9) == 0x29B1);
    CHECK(link_crc16(nullptr, 0) == 0xFFFF);
    uint8_t d[300];
    uint32_t x = 12345;
    for (int i = 0; i < 300; i++) d[i] = (uint8_t)((x = x * 1103515245u + 12345u) >> 16);
    for (int len = 0; len <= 300; len += 37) {
      uint16_t c = 0xFFFF;
      for (int i = 0; i < len; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
      }
      CHECK(link_crc16(d, (size_t)len) == c);
    }
  }

  {  // 1. normal running: every byte arrives, in order, in full frames, ~98 frames/s
    printf("normal, 10 s:\n");
    Result r;
    RunOpts o;
    run(r, o);
    Sim& s = r.sim;
    const size_t n = s.rx_data.size();
    CHECK(n + 400 > s.sent_stream.size());  // all but the last few ms
    CHECK(memcmp(s.rx_data.data(), s.sent_stream.data(), n) == 0);
    CHECK(seqGaps(s.rx_seq) == 0);
    CHECK(r.fwd.drops() == 0 && r.fwd.overflows() == 0);
    int flags = 0, full = 0, maxlen = 0;
    for (size_t i = 0; i < s.rx_len.size(); i++) {
      flags |= s.rx_flags[i];
      full += s.rx_len[i] == LINK_LIDAR_RAW_MAX;
      if (s.rx_len[i] > maxlen) maxlen = s.rx_len[i];
    }
    CHECK(flags == 0);
    CHECK(maxlen == LINK_LIDAR_RAW_MAX);
    CHECK(full * 10 > (int)s.rx_len.size() * 9);  // > 90 % full 180-byte chunks
    const double fps = s.rx_len.size() / 10.0, kbs = n / 10.0 / 1000.0;
    printf("  %.1f kB/s, %.0f frames/s, max %d bytes per pass, latency max %.1f ms, %d LD19 packets with a good CRC\n",
           kbs, fps, r.max_bytes_per_pass, s.max_latency_us / 1000.0, countPackets(s.rx_data));
    CHECK(kbs > 17.0 && kbs < 18.0);
    CHECK(fps > 90 && fps < 105);
    CHECK(r.max_bytes_per_pass <= LINK_LIDAR_RAW_MAX);
    CHECK(s.max_latency_us < 20000);
    CHECK(countPackets(s.rx_data) >= 3740);  // 375/s
    CHECK(memcmp(s.rx_data.data(), kExample, 47) == 0);
    CHECK(r.bad_window_calls == 0);
    CHECK(s.other_frames > 900);  // the telemetry got through too
    CHECK(anyLine(r, "LD19 bytes on Serial7"));
    CHECK(anyLine(r, "usb drop 0, overflow 0"));
  }
  {  // 2. the Pi stops reading for 60 ms: the UART buffer holds it, nothing lost
    printf("USB stalled 60 ms:\n");
    Result r;
    RunOpts o;
    o.seconds = 3;
    o.usb_block_from_ms = 1000;
    o.usb_block_ms = 60;
    run(r, o);
    Sim& s = r.sim;
    CHECK(memcmp(s.rx_data.data(), s.sent_stream.data(), s.rx_data.size()) == 0);
    CHECK(r.fwd.drops() == 0);
    CHECK(seqGaps(s.rx_seq) == 0);
    printf("  drops %lu, latency max %.1f ms\n", (unsigned long)r.fwd.drops(), s.max_latency_us / 1000.0);
  }
  {  // 3. the Pi stops reading for 1 s: chunks are dropped (never blocks), seq gaps = drops, packets parse again after
    printf("USB stalled 1 s:\n");
    Result r;
    RunOpts o;
    o.seconds = 4;
    o.usb_block_from_ms = 1000;
    o.usb_block_ms = 1000;
    run(r, o);
    Sim& s = r.sim;
    printf("  drops %lu, seq gaps %d, frames %zu, packets %d\n", (unsigned long)r.fwd.drops(), seqGaps(s.rx_seq),
           s.rx_len.size(), countPackets(s.rx_data));
    CHECK(r.fwd.drops() > 50);
    CHECK(seqGaps(s.rx_seq) == (int)r.fwd.drops());
    CHECK(s.lost_isr == 0);  // dropped by the forwarder, before the ring overflowed
    CHECK(r.max_bytes_per_pass <= LINK_LIDAR_RAW_MAX);
    CHECK(countPackets(s.rx_data) > 375 * 2);
  }
  {  // 4. the loop blocked for 200 ms (ring overflows): the next frame carries the overflow flag
    printf("loop stalled 200 ms:\n");
    Result r;
    RunOpts o;
    o.seconds = 2;
    o.stall_from_ms = 500;
    o.stall_ms = 200;
    run(r, o);
    Sim& s = r.sim;
    int flagged = 0;
    for (int fl : s.rx_flags) flagged += (fl & LINK_LIDAR_RAW_OVERFLOW) != 0;
    printf("  ring lost %lu bytes, overflow events %lu, frames flagged %d\n", (unsigned long)s.lost_isr,
           (unsigned long)r.fwd.overflows(), flagged);
    CHECK(s.lost_isr > 0);
    CHECK(r.fwd.overflows() == 1);
    CHECK(flagged == 1);
    CHECK(seqGaps(s.rx_seq) == 0);
  }
  {  // 5. interrupts off 5 ms (flash write): UART overrun -> flagged
    printf("interrupts off 5 ms:\n");
    Result r;
    RunOpts o;
    o.seconds = 2;
    o.irq_off_from_ms = 700;
    o.irq_off_ms = 5;
    run(r, o);
    Sim& s = r.sim;
    int flagged = 0;
    for (int fl : s.rx_flags) flagged += (fl & LINK_LIDAR_RAW_OVERFLOW) != 0;
    printf("  overrun lost %lu bytes, frames flagged %d\n", (unsigned long)s.lost_or, flagged);
    CHECK(s.lost_or > 0);
    CHECK(flagged == 1);
  }
  {  // 6. the lidar stops: a "no bytes" line, nothing sent, no spin
    printf("lidar unplugged after 3 s:\n");
    Result r;
    RunOpts o;
    o.seconds = 6;
    o.stop_lidar_at_end = true;
    run(r, o);
    CHECK(anyLine(r, "no bytes on Serial7"));
    CHECK(r.sim.rx_data.size() == r.sim.sent_stream.size());  // the last partial chunk was flushed
  }
  {  // 7. no lidar at all: no frame, no line
    Sim s;
    LidarForwarder f;
    f.begin(s.capacity, 0);
    for (uint32_t t = 0; t < 100000; t += 3) f.service(t, t / 1000, s);
    CHECK(s.rx_len.empty());
    CHECK(f.status(100) == nullptr);
  }
  {  // cost of one full-chunk service() call on this PC (the Teensy's Cortex-M7 at 600 MHz: see docs/ld19.md)
    Sim s;
    LidarForwarder f;
    f.begin(1 << 30, 0);
    const int iters = 200000;
    uint8_t chunk[LINK_LIDAR_RAW_MAX];
    for (int i = 0; i < LINK_LIDAR_RAW_MAX; i++) chunk[i] = (uint8_t)i;
    double total_ns = 0;
    uint32_t t = 0;
    for (int i = 0; i < iters; i++) {
      s.ring.insert(s.ring.end(), chunk, chunk + LINK_LIDAR_RAW_MAX);
      s.usb_used = 0;
      s.parser = LinkParser();
      t += 3000;
      const auto a = std::chrono::high_resolution_clock::now();
      f.service(t, t / 1000, s);
      const auto b = std::chrono::high_resolution_clock::now();
      total_ns += std::chrono::duration<double, std::nano>(b - a).count();
    }
    printf("one full chunk (read 180 + CRC + frame, this PC, incl. the fake UART / USB): %.0f ns\n", total_ns / iters);
    uint8_t buf[LINK_MAX_PAYLOAD + LINK_FRAME_OVERHEAD];
    const auto a = std::chrono::high_resolution_clock::now();
    size_t sink = 0;
    for (int i = 0; i < iters; i++) {
      chunk[0] = (uint8_t)i;
      sink += link_encode(MSG_LIDAR_RAW, chunk, 2 + 178, buf) + buf[100];
    }
    const auto b = std::chrono::high_resolution_clock::now();
    printf("link_encode of a 182-byte payload alone (table CRC-16): %.0f ns (%zu)\n",
           std::chrono::duration<double, std::nano>(b - a).count() / iters, sink % 7);
  }

  printf(g_fail ? "%d FAILED\n" : "all lidar forwarder checks passed\n", g_fail);
  return g_fail ? 1 : 0;
}
