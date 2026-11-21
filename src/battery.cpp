#include "battery.h"

#include <Arduino.h>

#include "config.h"
#include "params.h"

static const uint8_t kPins[3] = {pins::BATT_TAP1, pins::BATT_TAP2, pins::BATT_TAP3};

void Battery::begin() {
  for (uint8_t p : kPins) pinMode(p, INPUT);
}

void Battery::update(uint32_t now_ms) {
  if (now_ms - last_ms_ < 100) return;
  last_ms_ = now_ms;
  const float k = P(P_BATT_ADC_PER_V);
  for (int i = 0; i < 3; i++) {
    float v = analogRead(kPins[i]) / k;  // tap voltage (10-bit ADC)
    tap_[i] = have_ ? tap_[i] + 0.2f * (v - tap_[i]) : v;
  }
  have_ = true;
  cell_v_[0] = tap_[0];
  cell_v_[1] = tap_[1] - tap_[0];
  cell_v_[2] = tap_[2] - tap_[1];
  pack_v_ = tap_[2];
  min_cell_v_ = cell_v_[0];
  for (int i = 1; i < 3; i++)
    if (cell_v_[i] < min_cell_v_) min_cell_v_ = cell_v_[i];
  // hysteresis so the flag doesn't flicker under motor load
  const float lowv = P(P_BATT_LOW_CELL);
  if (!present()) low_ = false;
  else if (min_cell_v_ < lowv) low_ = true;
  else if (min_cell_v_ > lowv + 0.1f) low_ = false;
}
