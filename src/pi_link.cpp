#include "pi_link.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

void PiLink::poll(LinkFrameHandler handler, size_t max_bytes) {
  int n = Serial.available();
  if (n <= 0) return;
  if ((size_t)n > max_bytes) n = (int)max_bytes;
  while (n-- > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (parser_.feed((uint8_t)c)) handler(parser_.type(), parser_.payload(), parser_.len());
  }
}

bool PiLink::send(uint8_t type, const void* payload, uint8_t len) {
  uint8_t buf[LINK_MAX_PAYLOAD + LINK_FRAME_OVERHEAD];
  size_t n = link_encode(type, payload, len, buf);
  if (n == 0) return false;
  // availableForWrite() counts whole free USB buffers, so a write that fits never waits.
  // (If the host isn't reading, the buffers stay busy and this reports 0.)
  if (Serial.availableForWrite() < (int)n) {
    dropped_++;
    return false;
  }
  Serial.write(buf, n);
  return true;
}

bool PiLink::log(const char* fmt, ...) {
  char text[LINK_MAX_PAYLOAD + 1];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  if (n < 0) return false;
  if (n > LINK_MAX_PAYLOAD) n = LINK_MAX_PAYLOAD;
  return send(MSG_LOG, text, (uint8_t)n);
}

bool PiLink::hostConnected() {
  bool d = Serial.dtr();
  bool rose = d && !dtr_;
  dtr_ = d;
  return rose;
}
