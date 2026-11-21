#include "link_codec.h"

#include <string.h>

uint16_t link_crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

size_t link_encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out) {
  if (len > LINK_MAX_PAYLOAD) return 0;
  out[0] = LINK_SYNC1;
  out[1] = LINK_SYNC2;
  out[2] = type;
  out[3] = len;
  if (len) memcpy(out + 4, payload, len);
  uint16_t crc = link_crc16(out + 2, (size_t)len + 2);
  out[4 + len] = (uint8_t)(crc & 0xFF);
  out[5 + len] = (uint8_t)(crc >> 8);
  return (size_t)len + LINK_FRAME_OVERHEAD;
}

bool LinkParser::feed(uint8_t b) {
  switch (state_) {
    case SYNC1:
      if (b == LINK_SYNC1) state_ = SYNC2;
      break;
    case SYNC2:
      if (b == LINK_SYNC2) state_ = TYPE;
      else if (b != LINK_SYNC1) state_ = SYNC1;
      break;
    case TYPE:
      type_ = b;
      state_ = LEN;
      break;
    case LEN:
      len_ = b;
      pos_ = 0;
      if (len_ > LINK_MAX_PAYLOAD) state_ = SYNC1;
      else state_ = len_ ? PAYLOAD : CRC_LO;
      break;
    case PAYLOAD:
      buf_[pos_++] = b;
      if (pos_ >= len_) state_ = CRC_LO;
      break;
    case CRC_LO:
      crc_rx_ = b;
      state_ = CRC_HI;
      break;
    case CRC_HI: {
      crc_rx_ |= (uint16_t)b << 8;
      state_ = SYNC1;
      uint8_t hdr[2] = {type_, len_};
      uint16_t crc = link_crc16(hdr, 2);
      crc = link_crc16(buf_, len_, crc);
      if (crc == crc_rx_) {
        frames_++;
        return true;
      }
      crc_errors_++;
      break;
    }
  }
  return false;
}
