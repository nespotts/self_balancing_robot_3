#include "params.h"

#include <EEPROM.h>
#include <math.h>
#include <string.h>

#include "link_codec.h"

static const ParamDef kDefs[P_COUNT] = {
#define PARAM_DEF(id, name, def, mn, mx) {name, def, mn, mx},
    PARAM_TABLE(PARAM_DEF)
#undef PARAM_DEF
};

float g_param[P_COUNT];
bool g_param_save_requested = false;
bool g_param_defaults_requested = false;

// EEPROM layout: header, then P_COUNT floats.
// layout_crc covers the first `count` param names in order, so renaming/reordering params
// invalidates the saved block (defaults are used) instead of loading values into wrong slots.
// Params appended at the end keep the older ones' saved values (the new ones start at default).
static const uint32_t kMagic = 0x34524253;  // "SBR4"
static const int kAddr = 0;

struct __attribute__((packed)) EepromHeader {
  uint32_t magic;
  uint16_t layout_crc;
  uint16_t count;
  uint16_t values_crc;
};

static uint16_t layoutCrc(uint16_t n = P_COUNT) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < n && i < P_COUNT; i++) {
    crc = link_crc16((const uint8_t*)kDefs[i].name, strlen(kDefs[i].name) + 1, crc);
  }
  return crc;
}

const ParamDef& params_def(uint16_t id) { return kDefs[id < P_COUNT ? id : 0]; }

bool params_set(uint16_t id, float value) {
  if (id >= P_COUNT || isnan(value) || isinf(value)) return false;
  if (id == P_CFG_SAVE) {
    if (value >= 0.5f) g_param_save_requested = true;
    return true;
  }
  if (id == P_CFG_DEFAULTS) {
    if (value >= 0.5f) g_param_defaults_requested = true;
    return true;
  }
  const ParamDef& d = kDefs[id];
  if (value < d.min) value = d.min;
  if (value > d.max) value = d.max;
  g_param[id] = value;
  return true;
}

void params_reset_defaults() {
  for (uint16_t i = 0; i < P_COUNT; i++) g_param[i] = kDefs[i].def;
}

bool params_load() {
  params_reset_defaults();
  EepromHeader h;
  EEPROM.get(kAddr, h);
  // h.count < P_COUNT: saved by older firmware with fewer params (appended since) - keep those
  if (h.magic != kMagic || h.count == 0 || h.count > P_COUNT || h.layout_crc != layoutCrc(h.count))
    return false;
  float v[P_COUNT];
  for (uint16_t i = 0; i < h.count; i++) EEPROM.get(kAddr + (int)sizeof(h) + 4 * i, v[i]);
  if (link_crc16((const uint8_t*)v, h.count * sizeof(float)) != h.values_crc) return false;
  for (uint16_t i = 0; i < h.count; i++) {
    if (i == P_CFG_SAVE || i == P_CFG_DEFAULTS) continue;
    params_set(i, v[i]);  // clamps to the current limits
  }
  return true;
}

bool params_save() {
  float v[P_COUNT];
  memcpy(v, g_param, sizeof(v));
  v[P_CFG_SAVE] = 0;
  v[P_CFG_DEFAULTS] = 0;
  EepromHeader h{kMagic, layoutCrc(), P_COUNT, link_crc16((const uint8_t*)v, sizeof(v))};
  EEPROM.put(kAddr, h);
  for (uint16_t i = 0; i < P_COUNT; i++) EEPROM.put(kAddr + (int)sizeof(h) + 4 * i, v[i]);
  // verify
  EepromHeader r;
  EEPROM.get(kAddr, r);
  return memcmp(&r, &h, sizeof(h)) == 0;
}
