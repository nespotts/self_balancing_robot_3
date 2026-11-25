#include "contact.h"

#include <math.h>

#include "protocol/link_protocol.h"

void ContactDetector::reset() {
  bump_ms_ = cand_ms_ = 0;
  bump_peak_ = 0;
  hold_ms_ = 0;
  run_ = moved_ = false;
  // (the signal history and was_upright_ stay: they describe the robot, not a detection)
}

uint8_t ContactDetector::fire(uint8_t kind, float d, float value, uint8_t flags) {
  dir_ = d;
  value_ = value;
  flags_ = flags;
  reset();
  return kind;
}

uint8_t ContactDetector::update(const ContactIn& in) {
  const uint32_t now = in.now_ms;
  uint32_t dt = have_ ? now - last_ms_ : 0;
  if (dt > 50) dt = 50;
  last_ms_ = now;
  // ---- signal history (always, also while not watching: a bump needs the speed before it)
  if (!have_) {
    have_ = true;
    bin_ms_ = now;
    cur_max_ = cur_min_ = in.pitch_rate;
  }
  if (in.pitch_rate > cur_max_) cur_max_ = in.pitch_rate;
  if (in.pitch_rate < cur_min_) cur_min_ = in.pitch_rate;
  if (now - bin_ms_ >= kBinMs) {
    bs_[bhead_] = in.speed;
    bc_[bhead_] = in.cmd;
    bmax_[bhead_] = cur_max_;
    bmin_[bhead_] = cur_min_;
    bhead_ = (bhead_ + 1) % kBins;
    if (bn_ < kBins) bn_++;
    bin_ms_ = now;
    cur_max_ = cur_min_ = in.pitch_rate;
  }
  const bool fell = was_upright_ && !in.upright;
  was_upright_ = in.upright;
  const float d = in.cmd >= 0 ? 1.0f : -1.0f;
  const float c = fabsf(in.cmd);
  const float v = d * in.speed;
  const uint32_t still = moved_ ? now - new_ms_ : 0;  // ms without new ground

  // ---- a fall right after a candidate confirms it (a hard bump throws it over before the hold time is up)
  if (fell) {
    if (cand_ms_ && now - cand_ms_ <= kFallMs) return fire(LINK_CONTACT_BUMP, cand_d_, bump_peak_, LINK_CONTACT_FELL);
    if (still >= kStuckMs / 2) return fire(LINK_CONTACT_STUCK, run_d_, (float)still, LINK_CONTACT_FELL);
  }
  if (cand_ms_ && now - cand_ms_ > kFallMs) cand_ms_ = 0;
  if (!in.watch || !in.upright) {
    bump_ms_ = hold_ms_ = 0;
    moved_ = false;
    run_ = false;
    return 0;
  }

  // ---- bump
  if (!bump_ms_ && c >= kBumpMinCmd) {
    float v_hi = v, c_hi = c, r_hi = d > 0 ? cur_max_ : -cur_min_;
    for (uint32_t k = 1; k <= bn_; k++) {
      const uint32_t i = (bhead_ + kBins - k) % kBins;
      const float s = d * bs_[i], cc = d * bc_[i], r = d > 0 ? bmax_[i] : -bmin_[i];
      if (s > v_hi) v_hi = s;
      if (cc > c_hi) c_hi = cc;
      if (r > r_hi) r_hi = r;
    }
    const float drop = fmaxf(kBumpDropMm, kBumpDropFrac * v_hi);
    if (c >= kBumpSteady * c_hi && v_hi >= kBumpNear * c && v < kBumpNear * c && v_hi - v >= drop &&
        r_hi >= kBumpRate) {
      bump_ms_ = cand_ms_ = now ? now : 1;
      bump_peak_ = r_hi;
      cand_d_ = d;
      hold_ms_ = 0;
    }
  }
  if (bump_ms_) {
    const float r = d > 0 ? cur_max_ : -cur_min_;
    if (r > bump_peak_) bump_peak_ = r;
    if (v >= kBumpBack * c || c < 0.5f * kBumpMinCmd) {
      bump_ms_ = cand_ms_ = 0;  // rolled over it (a door strip), or the command went: not a contact
    } else if (now - bump_ms_ > kBumpWindowMs) {
      bump_ms_ = 0;             // not confirmed (a fall soon after still counts: cand_ms_)
    } else {
      if (v < kBumpHold * c) hold_ms_ += dt;
      if (hold_ms_ >= kBumpHoldMs) return fire(LINK_CONTACT_BUMP, d, bump_peak_, 0);
    }
  }

  // ---- pushing but not moving: no new ground for kStuckMs since it got going
  if (in.cruise && c >= kStuckMinCmd && (!run_ || d == run_d_)) {
    if (!run_) {
      run_ = true;
      run_d_ = d;
      run_ms_ = now;
      moved_ = false;
      pos_ = high_ = 0;
    }
    pos_ += v * (float)dt * 0.001f;  // along the commanded direction, mm (encoder speed integrated)
    // it got going toward it (not in the start: a balancing robot leans in and rolls back first)
    if (!moved_ && now - run_ms_ >= kStuckStartMs && v >= fminf(kStuckMovedFrac * c, kStuckMovedMm)) {
      moved_ = true;
      high_ = pos_;
      new_ms_ = now;
    } else if (moved_ && pos_ > high_ + kStuckNewMm) {
      high_ = pos_;  // new ground
      new_ms_ = now;
    }
  } else {
    run_ = moved_ = false;
  }
  if (moved_ && d * in.vel_out >= kStuckLeanDeg) {
    const uint32_t s = now - new_ms_;
    if (s >= kStuckMs) return fire(LINK_CONTACT_STUCK, d, (float)s, 0);
    if (in.lidar_stuck && s >= kStuckMs / 2) return fire(LINK_CONTACT_STUCK, d, (float)s, LINK_CONTACT_LIDAR);
  }
  return 0;
}
