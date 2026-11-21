// nRF24 link to the remote, radio protocol v2 (include/protocol/radio_protocol.h).
// The robot only listens; telemetry rides back as the ACK payload of each ControlPacket.
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

  bool chipOk() const { return ok_; }
  bool versionMismatch(uint32_t now_ms) const { return bad_version_ms_ && now_ms - bad_version_ms_ < 1000; }
  uint32_t packets() const { return packets_; }

 private:
  bool ok_ = false;
  uint32_t packets_ = 0, bad_ = 0, bad_version_ms_ = 0, check_ms_ = 0;
  TelemetryPacket last_ack_ = {};
};
