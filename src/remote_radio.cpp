#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "remote_radio.h"

static RF24 g_rf(pins::RADIO_CE, pins::RADIO_CSN);
static const uint8_t kPipe = 1;
static constexpr uint32_t kStatsMs = 10000, kGapMs = 100;

bool RemoteRadio::begin() {
  ok_ = g_rf.begin();
  if (!ok_) return false;
  g_rf.setChannel(RADIO_CHANNEL);
  g_rf.setDataRate(RADIO_DATA_RATE);
  g_rf.setPALevel(RF24_PA_HIGH);
  g_rf.enableDynamicPayloads();
  g_rf.enableAckPayload();
  g_rf.openReadingPipe(kPipe, (const uint8_t*)RADIO_ROBOT_ADDR);
  g_rf.startListening();  // flushes TX when ack payloads are on, so load the ack after this
  last_ack_.version = RADIO_PROTOCOL_VERSION;
  g_rf.writeAckPayload(kPipe, &last_ack_, sizeof(last_ack_));
  return true;
}

bool RemoteRadio::poll(ControlPacket& out, uint32_t now_ms) {
  if (!ok_) return false;
  const uint32_t t0 = micros();
  bool got = false;
  uint8_t pipe, n_read = 0;
  for (int guard = 0; guard < 4 && g_rf.available(&pipe); guard++) {
    uint8_t len = g_rf.getDynamicPayloadSize();  // 0 = corrupt (RF24 flushes RX)
    n_read++;
    if (len == 0) {
      bad_++;
      break;
    }
    uint8_t buf[32];
    g_rf.read(buf, len);
    if (len == sizeof(ControlPacket) && buf[0] == RADIO_PROTOCOL_VERSION) {
      memcpy(&out, buf, sizeof(ControlPacket));
      got = true;
      packets_++;
      if (last_pkt_ms_) {
        const uint32_t gap = now_ms - last_pkt_ms_;
        if (gap > kGapMs) gaps_++;
        if (gap > max_gap_ms_) max_gap_ms_ = gap;
      }
      last_pkt_ms_ = now_ms ? now_ms : 1;
    } else {
      bad_++;
      if (len >= 1 && buf[0] != RADIO_PROTOCOL_VERSION) bad_version_ms_ = now_ms ? now_ms : 1;
    }
  }
  if (n_read > 1) multi_++;  // more than one waiting: the background loop was late (> 20 ms)
  // Something was read but nothing valid: the caller won't call setAck, so reload the last ack here.
  if (n_read && !got) loadAck(last_ack_);
  const uint32_t dt = micros() - t0;
  if (dt > poll_max_us_) poll_max_us_ = dt;
  return got;
}

void RemoteRadio::loadAck(const TelemetryPacket& t) {
  // Normally the packet just read took the loaded ack and the TX FIFO is empty. If not (the ack
  // wasn't taken, or a re-init), drop the stale ones so the next ack carries this telemetry.
  if (g_rf.isFifo(true) != RF24_FIFO_EMPTY) g_rf.flush_tx();
  if (!g_rf.writeAckPayload(kPipe, &t, sizeof(t))) {
    ack_fail_++;
    g_rf.flush_tx();
    g_rf.writeAckPayload(kPipe, &t, sizeof(t));
  }
}

void RemoteRadio::setAck(const TelemetryPacket& t) {
  if (!ok_) return;
  last_ack_ = t;
  const uint32_t t0 = micros();
  loadAck(t);
  const uint32_t dt = micros() - t0;
  if (dt > poll_max_us_) poll_max_us_ = dt;
}

void RemoteRadio::service(uint32_t now_ms, bool motors_off) {
  if (now_ms - check_ms_ < 1000) return;
  check_ms_ = now_ms;
  // A brown-out resets the nRF24 to defaults (channel 2), which isChipConnected() can't see.
  if (ok_ && (!g_rf.isChipConnected() || g_rf.getChannel() != RADIO_CHANNEL)) ok_ = false;
  if (!ok_ && motors_off) begin();
}

const char* RemoteRadio::stats(uint32_t now_ms) {
  if (!stat_ms_) {
    stat_ms_ = now_ms ? now_ms : 1;
    stat_pkts_ = packets_;
    stat_bad_ = bad_;
    return nullptr;
  }
  const uint32_t span = now_ms - stat_ms_;
  if (span < kStatsMs) return nullptr;
  const uint32_t n = packets_ - stat_pkts_, bad = bad_ - stat_bad_;
  const bool active = n > 0;
  const char* ret = nullptr;
  if (active || stat_active_) {  // every 10 s while packets come, plus one line when they stop
    snprintf(line_, sizeof(line_),
             "radio: %.1f pkt/s, gaps>100ms %lu (max %lu ms), bad %lu, multi %lu, ack fail %lu, poll max %lu us%s",
             (double)(n * 1000.0f / (float)span), (unsigned long)gaps_, (unsigned long)max_gap_ms_,
             (unsigned long)bad, (unsigned long)multi_, (unsigned long)ack_fail_, (unsigned long)poll_max_us_,
             active ? "" : " (no packets: remote off?)");
    ret = line_;
  }
  stat_active_ = active;
  stat_ms_ = now_ms ? now_ms : 1;
  stat_pkts_ = packets_;
  stat_bad_ = bad_;
  gaps_ = max_gap_ms_ = multi_ = ack_fail_ = poll_max_us_ = 0;
  if (!active) last_pkt_ms_ = 0;  // a reconnect is not counted as a gap
  return ret;
}
