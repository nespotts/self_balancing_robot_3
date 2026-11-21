// 3S pack monitor: three balance-tap dividers (pins 14/40/41), read at 10 Hz and filtered.
#pragma once
#include <stdint.h>

class Battery {
 public:
  void begin();
  void update(uint32_t now_ms);
  float packV() const { return pack_v_; }
  float cellV(int i) const { return cell_v_[i]; }
  float minCellV() const { return min_cell_v_; }
  bool present() const { return pack_v_ > 3.0f; }  // below this we're on USB power only
  bool low() const { return low_; }

 private:
  float tap_[3] = {0, 0, 0};
  float cell_v_[3] = {0, 0, 0};
  float pack_v_ = 0, min_cell_v_ = 0;
  bool low_ = false, have_ = false;
  uint32_t last_ms_ = 0;
};
