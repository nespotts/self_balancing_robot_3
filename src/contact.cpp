#include "contact.h"

#include <math.h>

#include "protocol/link_protocol.h"

void ContactDetector::reset() {
  bump_ms_ = cand_ms_ = 0;
  bump_peak_ = 0;
  hold_ms_ = 0;
  run_ = moved_ = false;
  s_run_ = s_win_ = d_win_ = false;
  // (the signal history and was_upright_ stay: they describe the robot, not a detection)
}

uint8_t ContactDetector::fire(uint8_t kind, float d, float value, uint8_t flags, bool stall) {
  dir_ = d;
  value_ = value;
  flags_ = flags;
  stall_ = stall;
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
  const uint32_t still = moved_ ? now - new_ms_ : 0;    // ms without new ground
  const uint32_t s_still = s_win_ ? now - s_a_ms_ : 0;  // stall: ms in the band

  // ---- a fall right after a candidate confirms it (a hard bump throws it over before the hold time is up)
  if (fell) {
    if (cand_ms_ && now - cand_ms_ <= kFallMs) return fire(LINK_CONTACT_BUMP, cand_d_, bump_peak_, LINK_CONTACT_FELL);
    if (still >= kStuckMs / 2) return fire(LINK_CONTACT_STUCK, run_d_, (float)still, LINK_CONTACT_FELL);
    if (s_still >= kStallMs / 2) return fire(LINK_CONTACT_STUCK, s_d_, (float)s_still, LINK_CONTACT_FELL, true);
  }
  if (cand_ms_ && now - cand_ms_ > kFallMs) cand_ms_ = 0;
  if (!in.watch || !in.upright) {
    bump_ms_ = hold_ms_ = 0;
    moved_ = false;
    run_ = false;
    s_run_ = s_win_ = d_win_ = false;
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

  // ---- stall (#111): commanded (>= kStallMinCmd, either way) but the wheels (nearly) still while the lean builds
  if (c >= kStallMinCmd && (!s_run_ || d == s_d_)) {
    if (!s_run_) {
      s_run_ = true;
      s_d_ = d;
      s_on_ms_ = now;
      s_pos_ = 0;
      s_win_ = d_win_ = false;
    }
    s_pos_ += v * (float)dt * 0.001f;
    s_abs_ += (fabsf(v) - s_abs_) * fminf(1.0f, (float)dt / kStallStillTauMs);  // mean |speed|, last ~0.2 s
    s_rate_ += (fabsf(in.pitch_rate) - s_rate_) * fminf(1.0f, (float)dt / kStallStillTauMs);  // mean |pitch rate|
    // reversing (backing off, after a bounce or from standing): a balancing robot rocks for 0.5-1 s before the motion
    // shows - the rear rules count only after kStallRevSettleMs, with the rocking died down (kStallRevRateDps), with a
    // longer window, a stiller one and a bigger build-up (2026-11-25 18:36: ~20 false rear contacts in back-offs)
    const bool rev = d < 0;
    const bool rev_ok = !rev || (now - s_on_ms_ >= kStallRevSettleMs && s_rate_ <= kStallRevRateDps);
    // dead still (#111, any command >= kStallMinCmd: slowing for the last point, 40 mm/s, ramping up too): the wheels
    // not turning at all (mean |speed| <= kStallDeadMms, travel within +-kStallDeadMm) for kStallDeadMs while the lean
    // builds. A free balancing robot in its motor dead band creeps or rocks (the sim's: <= ~0.1 s that still)
    if (now - s_on_ms_ >= kStallStartMs) {
      d_sum_ += fabsf(v) * (float)dt * 0.001f;
      const uint32_t age = now - d_a_ms_;
      if (!d_win_ || fabsf(s_pos_ - d_a_pos_) > kStallDeadMm ||
          d_sum_ * 1000.0f > kStallDeadMeanMms * (float)(age > kStallDeadMs ? age : kStallDeadMs)) {
        // (starts once it is still; a twitch on the way is fine: the travel and the mean speed decide)
        d_win_ = s_abs_ <= kStallDeadMms;
        d_a_pos_ = s_pos_;
        d_a_ms_ = now;
        d_a_lean_ = in.vel_out;
        d_sum_ = 0;
      } else if (rev_ok && now - d_a_ms_ >= kStallDeadMs &&
                 (d * (in.vel_out - d_a_lean_) >= (rev ? kStallRevBuildDeg : kStallDeadBuildDeg) || in.held)) {
        return fire(LINK_CONTACT_STUCK, d, (float)(now - d_a_ms_), in.lidar_stuck ? LINK_CONTACT_LIDAR : 0, true);
      }
    }
    // (driving: not while slowing for a stop - as STUCK - and not this slowly: a balancing robot can stand in its motor
    // dead band there; the dead-still rule above only)
    if (in.ramp || now - s_on_ms_ < kStallStartMs || (in.drive && (c < kStallDriveMinCmd || !in.cruise))) {
      s_win_ = false;  // leaning in / ramping up: a balancing robot lags there
    } else {
      if (!s_win_ || s_pos_ > s_a_pos_ + kStallNewMm || s_pos_ < s_a_pos_ - kStallBackMm) {
        s_win_ = true;  // (it moved: the window starts again here)
        s_a_pos_ = s_pos_;
        s_a_ms_ = now;
        s_a_lean_ = in.vel_out;
        s_sum_ = 0;
      }
      s_sum_ += fabsf(v) * (float)dt * 0.001f;  // |travel| in the window
      const uint32_t s = now - s_a_ms_;
      // (driving slowly: a balancing robot can stand in its motor dead band a while)
      const uint32_t win = rev ? kStallRevMs : (in.drive && c < kStuckMinCmd ? kStallDriveMs : kStallMs);
      // (still now; rocking against it before is fine, up to twice that on average - reversing: no rocking)
      if (rev_ok && s >= win && s_abs_ <= kStallStillMms &&
          s_sum_ * 1000.0f <= (rev ? kStallRevMean : 2.0f) * kStallStillMms * (float)s &&
          (d * (in.vel_out - s_a_lean_) >= (rev ? kStallRevBuildDeg : kStallBuildDeg) || in.held))
        return fire(LINK_CONTACT_STUCK, d, (float)s, in.lidar_stuck ? LINK_CONTACT_LIDAR : 0, true);
    }
  } else {
    s_run_ = s_win_ = d_win_ = false;
  }
  return 0;
}
