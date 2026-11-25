#include "i2c_bounded.h"

#include <Arduino.h>

void BoundedI2c::applyClock() {
  IMXRT_LPI2C_t* p = (IMXRT_LPI2C_t*)port_;
  if (hz_ == 200000) {
    // Wire knows 100 k / 400 k / 1 M: the 400 kHz timing with the functional clock halved (prescaler 2) = 200 kHz,
    // with the glitch filters, bus-idle and pin-low timeouts twice as long in time.
    wire_.setClock(400000);
    p->MCR = 0;
    p->MCFGR1 = LPI2C_MCFGR1_PRESCALE(1);
    p->MCR = LPI2C_MCR_MEN;
  } else {
    wire_.setClock(hz_);
  }
}

void BoundedI2c::begin() {
  wire_.begin();  // resets the LPI2C master (MCR RST) and muxes SDA / SCL back to I2C
  applyClock();
}

I2cStatus BoundedI2c::readReg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  IMXRT_LPI2C_t* p = (IMXRT_LPI2C_t*)port_;
  const uint32_t c0 = ARM_DWT_CYCCNT;
  if (n < 1) n = 1;
  if (n > 4) n = 4;
  // wire time: address + reg + address + n data bytes, 9 bits each, + START / repeated START / STOP
  const uint32_t wire_us = ((3u + n) * 9u + 3u) * 1000000u / hz_;
  const uint32_t budget = (wire_us * 5u / 2u + 50u) * (F_CPU_ACTUAL / 1000000u);
  I2cStatus st = I2C_OK;
  uint8_t got = 0;
  bool stop_queued = false;

  uint32_t s = p->MSR;
  if ((s & LPI2C_MSR_BBF) && !(s & LPI2C_MSR_MBF)) {
    st = I2C_BUSY;  // someone else holds the bus: a slave keeping SDA low after a glitched transfer
    goto done;
  }
  if (p->MFSR & 0x00070007) p->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;  // leftovers from an aborted transfer
  p->MSR = 0x00007F00;                                                // clear the w1c flags
  // 4 commands = the TX FIFO's depth; the STOP goes in as soon as the first one leaves
  p->MTDR = LPI2C_MTDR_CMD_START | ((uint32_t)addr << 1);
  p->MTDR = LPI2C_MTDR_CMD_TRANSMIT | reg;
  p->MTDR = LPI2C_MTDR_CMD_START | ((uint32_t)addr << 1) | 1u;
  p->MTDR = LPI2C_MTDR_CMD_RECEIVE | (uint32_t)(n - 1);
  while (true) {
    if (!stop_queued && (p->MFSR & 0x07) < 4) {
      p->MTDR = LPI2C_MTDR_CMD_STOP;
      stop_queued = true;
    }
    while (got < n && ((p->MFSR >> 16) & 0x07)) {
      const uint32_t d = p->MRDR;
      if (!(d & LPI2C_MRDR_RXEMPTY)) buf[got++] = (uint8_t)d;
    }
    s = p->MSR;
    if (s & LPI2C_MSR_NDF) { st = I2C_NACK; break; }
    if (s & LPI2C_MSR_ALF) { st = I2C_ARB_LOST; break; }
    if (s & LPI2C_MSR_FEF) { st = I2C_FIFO_ERR; break; }
    if (s & LPI2C_MSR_PLTF) { st = I2C_PIN_LOW; break; }
    if (got >= n && stop_queued && (s & LPI2C_MSR_SDF)) break;  // done
    if (ARM_DWT_CYCCNT - c0 > budget) { st = I2C_TIMEOUT; break; }
  }
  if (st != I2C_OK) {
    p->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;  // drop what is still queued
    // like Wire: a STOP after a NACK / timeout / pin-low (not after a FIFO error or a lost arbitration)
    if (st == I2C_NACK || st == I2C_TIMEOUT || st == I2C_PIN_LOW) p->MTDR = LPI2C_MTDR_CMD_STOP;
  }
done:
  last_us_ = (ARM_DWT_CYCCNT - c0) / (F_CPU_ACTUAL / 1000000u);
  return st;
}

bool BoundedI2c::recover() {
  IMXRT_LPI2C_t* p = (IMXRT_LPI2C_t*)port_;
  p->MCR = 0;  // master off while the pins are GPIO
  // open drain: HIGH = released (the pull-ups lift the line), LOW = driven low
  digitalWriteFast(sda_, HIGH);
  pinMode(sda_, OUTPUT_OPENDRAIN);
  digitalWriteFast(scl_, HIGH);
  pinMode(scl_, OUTPUT_OPENDRAIN);
  const uint32_t half = hz_ >= 400000 ? 2 : 5;  // us per half clock (<= 100 kHz)
  delayMicroseconds(half);
  for (int i = 0; i < 9; i++) {  // a slave mid-byte finishes it; with no ACK from us it lets SDA go
    digitalWriteFast(scl_, LOW);
    delayMicroseconds(half);
    digitalWriteFast(scl_, HIGH);
    for (int w = 0; w < 20 && !digitalReadFast(scl_); w++) delayMicroseconds(1);  // clock stretching, bounded
    delayMicroseconds(half);
  }
  // STOP: SDA low -> high while SCL is high
  digitalWriteFast(scl_, LOW);
  delayMicroseconds(half);
  digitalWriteFast(sda_, LOW);
  delayMicroseconds(half);
  digitalWriteFast(scl_, HIGH);
  delayMicroseconds(half);
  digitalWriteFast(sda_, HIGH);
  delayMicroseconds(half);
  const bool ok = digitalReadFast(sda_) && digitalReadFast(scl_);
  begin();  // LPI2C reset + pins back to I2C + clock
  return ok;
}
