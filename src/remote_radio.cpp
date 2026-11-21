#include <RF24.h>
#include <SPI.h>
#include <string.h>

#include "config.h"
#include "remote_radio.h"

static RF24 g_rf(pins::RADIO_CE, pins::RADIO_CSN);
static const uint8_t kPipe = 1;

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
  bool got = false;
  uint8_t pipe;
  for (int guard = 0; guard < 4 && g_rf.available(&pipe); guard++) {
    uint8_t len = g_rf.getDynamicPayloadSize();  // 0 = corrupt (RF24 flushes RX)
    if (len == 0) break;
    uint8_t buf[32];
    g_rf.read(buf, len);
    if (len == sizeof(ControlPacket) && buf[0] == RADIO_PROTOCOL_VERSION) {
      memcpy(&out, buf, sizeof(ControlPacket));
      got = true;
      packets_++;
    } else {
      bad_++;
      if (len >= 1 && buf[0] != RADIO_PROTOCOL_VERSION) bad_version_ms_ = now_ms ? now_ms : 1;
    }
  }
  return got;
}

void RemoteRadio::setAck(const TelemetryPacket& t) {
  if (!ok_) return;
  last_ack_ = t;
  // Each received packet uses up one ack payload, so the TX FIFO normally holds <= 1.
  // If it is full anyway, drop the stale ones.
  if (!g_rf.writeAckPayload(kPipe, &t, sizeof(t))) {
    g_rf.flush_tx();
    g_rf.writeAckPayload(kPipe, &t, sizeof(t));
  }
}

void RemoteRadio::service(uint32_t now_ms, bool motors_off) {
  if (now_ms - check_ms_ < 1000) return;
  check_ms_ = now_ms;
  // A brown-out resets the nRF24 to defaults (channel 2), which isChipConnected() can't see.
  if (ok_ && (!g_rf.isChipConnected() || g_rf.getChannel() != RADIO_CHANNEL)) ok_ = false;
  if (!ok_ && motors_off) begin();
}
