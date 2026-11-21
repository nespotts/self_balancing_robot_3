// nRF24 link to the remote, radio protocol v2 (include/protocol/radio_protocol.h).
// The robot only listens; telemetry rides back as the ACK payload of each ControlPacket.
//
// Ack payloads: every packet read uses up the loaded ack, so one is loaded again after every poll that
// read anything (valid or not) - the remote never gets an empty ack because of us. A stale ack still in
// the TX FIFO is flushed first, so the next ack always carries the newest telemetry.
// Never blocks: poll() / setAck() are a few short SPI transactions (~10 us each, measured: stats line);
// only begin() (re-init after a brown-out) waits, and service() calls it only with the motors off.
#pragma once
#include <stdint.h>

#include "protocol/radio_protocol.h"

class RemoteRadio {
 public:
  bool begin();
  // Drain the RX FIFO. Returns true with the newest valid v2 ControlPacket.
  bool poll(ControlPacket& out, uint32_t now_ms);
  // Queue the telemetry that goes out with the ACK of the next packet.
  void setAck(const TelemetryPacket& t);
  // Once a second: check the chip still has our config; re-init (blocks ~10 ms) only if motors_off.
  void service(uint32_t now_ms, bool motors_off);
  // Every 10 s while packets arrive (and once when they stop): a stats line for MSG_LOG, else nullptr.
  //   radio: 50.0 pkt/s, gaps>100ms 0 (max 32 ms), bad 0, multi 0, ack fail 0, poll max 61 us
  const char* stats(uint32_t now_ms);

  bool chipOk() const { return ok_; }
  bool versionMismatch(uint32_t now_ms) const { return bad_version_ms_ && now_ms - bad_version_ms_ < 1000; }
  uint32_t packets() const { return packets_; }

 private:
  void loadAck(const TelemetryPacket& t);
  bool ok_ = false;
  uint32_t packets_ = 0, bad_ = 0, bad_version_ms_ = 0, check_ms_ = 0;
  TelemetryPacket last_ack_ = {};
  // stats (since the last stats line)
  uint32_t last_pkt_ms_ = 0, max_gap_ms_ = 0, gaps_ = 0, multi_ = 0, ack_fail_ = 0, poll_max_us_ = 0;
  uint32_t stat_ms_ = 0, stat_pkts_ = 0, stat_bad_ = 0;
  bool stat_active_ = false;
  char line_[120];
};
