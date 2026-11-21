// Teensy <-> Orange Pi framed link over USB serial (include/protocol/link_protocol.h).
// Never blocks: a frame is only written if the USB buffer has room, otherwise it is dropped.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "link_codec.h"

typedef void (*LinkFrameHandler)(uint8_t type, const uint8_t* payload, uint8_t len);

class PiLink {
 public:
  // Parse up to max_bytes of input, calling handler for every good frame.
  void poll(LinkFrameHandler handler, size_t max_bytes = 512);
  // Returns false (frame dropped) if it doesn't fit in the USB buffer right now.
  bool send(uint8_t type, const void* payload, uint8_t len);
  bool log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  // True once each time the host opens the port (DTR rises).
  bool hostConnected();
  uint32_t dropped() const { return dropped_; }
  const LinkParser& parser() const { return parser_; }

 private:
  LinkParser parser_;
  uint32_t dropped_ = 0;
  bool dtr_ = false;
};
