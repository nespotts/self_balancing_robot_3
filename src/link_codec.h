// Frame encode/decode for include/protocol/link_protocol.h. Pure C++, no Arduino.
//   0xA5 0x5A | type | len | payload[len] | crc16 LE   (CRC-16/CCITT-FALSE over type, len, payload)
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "protocol/link_protocol.h"

constexpr size_t LINK_FRAME_OVERHEAD = 6;

uint16_t link_crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// Writes a whole frame into out (needs len + 6 bytes). Returns the frame size, 0 if len too big.
size_t link_encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out);

class LinkParser {
 public:
  // Feed one byte. Returns true when a complete frame with a good CRC is ready in
  // type()/len()/payload(); valid until the next feed().
  bool feed(uint8_t b);
  uint8_t type() const { return type_; }
  uint8_t len() const { return len_; }
  const uint8_t* payload() const { return buf_; }
  uint32_t crcErrors() const { return crc_errors_; }
  uint32_t frames() const { return frames_; }

 private:
  enum State : uint8_t { SYNC1, SYNC2, TYPE, LEN, PAYLOAD, CRC_LO, CRC_HI };
  State state_ = SYNC1;
  uint8_t type_ = 0, len_ = 0, pos_ = 0;
  uint16_t crc_rx_ = 0;
  uint8_t buf_[LINK_MAX_PAYLOAD];
  uint32_t crc_errors_ = 0, frames_ = 0;
};
