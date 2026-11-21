#include "navigator.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "params.h"

// Range head geometry (docs/firmware.md "Range sweep"): servo axis 26 mm ahead of the axle, each sensor
// 12.9 mm out from it, 189 mm above the axle.
static constexpr float kAxisFwd = 26.0f, kSensorOut = 12.9f, kSensorHeight = 189.0f;
static constexpr float kDeg = 0.017453292519943f;
// Footprint 152 x 90 mm about the axle (#62). Turning in place, the corners sweep a circle of r 88.
static constexpr float kHalfWidth = 76.0f;
static constexpr float kSwingR = 105.0f;                  // 88 + margin: inside it, a turn in place scrapes
static constexpr float kSideBack = 45.0f, kSideAhead = 300.0f;   // side window: body rear .. 300 ahead
static constexpr float kBlockedTurnDeg = 30.0f;           // GOTO blocked by a wall alongside: turn away <= this
static constexpr float kSideInner = 40.0f;                // |lateral| below this = straight ahead / behind
static constexpr float kStuckDeg = 15.0f;                 // turning: progress = this much heading change

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static float wrap180(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  if (a <= -180.0f) a += 360.0f;
  return a;
}

// ------------------------------------------------------------------ list

void Navigator::clearList() { n_ = 0; }

bool Navigator::add(float x, float y) {
  if (n_ >= WAYPOINT_MAX || isnan(x) || isnan(y) || isinf(x) || isinf(y)) return false;
  wx_[n_] = x;
  wy_[n_] = y;
  n_++;
  return true;
}

void Navigator::listChanged(bool replaced) {
  if (replaced || idx_ > n_) {
    idx_ = 0;
    phase_ = SEEK;
    driving_ = false;
  } else if (phase_ == DONE && idx_ < n_) {
    phase_ = SEEK;
    driving_ = false;
  }
  if (n_ == 0) idx_ = 0;
  blocked_ = latched_ = backing_ = false;  // a new / changed list always resumes
  blk_fwd_ = blk_rev_ = false;
  stop_since_ = 0;
  stuck_ = away_ = away_done_ = false;
  stuck_ms_ = 0;
}

// ------------------------------------------------------------------ inputs

void Navigator::addRange(const RangeFrame& f, uint32_t now_ms) {
  sensed_ms_ = now_ms ? now_ms : 1;  // the sweep is running (a "no target" reading counts too)
  if (!f.valid || f.range_mm == 0) return;
  const float a = f.sensor_angle_deg;
  const float r = (float)f.range_mm + kSensorOut;
  // lean forward tips the front sensor down (and the rear one up): ignore hits on the floor
  const float lean = f.pitch_deg + P(P_BAL_OFFSET);
  const float tilt = lean * cosf(a * kDeg);
  if (tilt > 0.5f && kSensorHeight - r * sinf(tilt * kDeg) < P(P_NAV_FLOOR_MM)) return;
  const float h = f.heading_deg * kDeg;
  const float ax = f.x_mm + kAxisFwd * cosf(h), ay = f.y_mm + kAxisFwd * sinf(h);
  const float d = h + a * kDeg;
  px_[pt_head_] = ax + r * cosf(d);
  py_[pt_head_] = ay + r * sinf(d);
  pt_ms_[pt_head_] = now_ms;
  pt_head_ = (pt_head_ + 1) % kPts;
  if (pt_n_ < kPts) pt_n_++;
  last_range_ms_ = now_ms ? now_ms : 1;
}

void Navigator::onModeChange(const NavIn& in) {
  mode_ = in.mode;
  now_ms_ = in.now_ms;
  speed_ = turn_ = 0;
  blocked_ = latched_ = backing_ = false;
  blk_fwd_ = blk_rev_ = false;
  driving_ = false;
  stop_since_ = 0;
  stuck_ = away_ = away_done_ = false;
  stuck_ms_ = 0;
  if (in.mode == MODE_GOTO || in.mode == MODE_EXPLORE) {
    if (idx_ >= n_) idx_ = 0;  // list done (or empty): start over
    phase_ = n_ ? SEEK : DONE;
  }
  if (in.mode == MODE_HOLD) {
    hold_x_ = in.x;
    hold_y_ = in.y;
  }
  if (navMode(in.mode)) {
    warned_no_range_ = blind(in.now_ms);
    if (warned_no_range_) logBlind(in.mode);
    if (in.mode != MODE_HOLD) log("nav: %u waypoints, going to %u", (unsigned)n_, (unsigned)idx_ + 1);
  }
}

// No RangeFrame for kBlindMs (rng.enable 0, sensors dead): no obstacle stop is possible (safety review #38).
bool Navigator::blind(uint32_t now_ms) const { return !sensed_ms_ || now_ms - sensed_ms_ > kBlindMs; }

void Navigator::logBlind(uint8_t mode) {
  if (mode == MODE_EXPLORE || P(P_NAV_BLIND_SPEED) <= 0)
    log("nav: no range readings - obstacle stop is off: %s paused", mode == MODE_EXPLORE ? "EXPLORE" : "GOTO");
  else if (mode == MODE_GOTO)
    log("nav: no range readings - obstacle stop is off: GOTO max %.0f mm/s", (double)P(P_NAV_BLIND_SPEED));
  else
    log("nav: no range readings - obstacle stop is off");
}

void Navigator::onPoseReset() {
  n_ = 0;
  pt_n_ = 0;
  hold_x_ = hold_y_ = 0;
  listChanged(true);
}

// ------------------------------------------------------------------ obstacles

void Navigator::updateObstacles(const NavIn& in) {
  const float c = cosf(in.heading * kDeg), s = sinf(in.heading * kDeg);
  // With the side check on (#62), points beside the body (> its half width + 10) steer / slow it down
  // instead of stopping it: a wall alongside or a door jamb is not in the way.
  const float half = P(P_NAV_SIDE_MIN_MM) > 0 ? fminf(0.5f * P(P_NAV_CORRIDOR_MM), kHalfWidth + 10.0f)
                                              : 0.5f * P(P_NAV_CORRIDOR_MM);
  const uint32_t age = (uint32_t)P(P_NAV_OBST_AGE_MS);
  float front = 0, rear = 0, gl = 0, gr = 0, near = 0, near_xr = 0;
  for (uint8_t i = 0; i < pt_n_; i++) {
    if (in.now_ms - pt_ms_[i] > age) continue;
    const float dx = px_[i] - in.x, dy = py_[i] - in.y;
    const float xr = dx * c + dy * s, yr = -dx * s + dy * c;
    if (xr > -kSideBack && xr < kSideAhead && fabsf(yr) >= kSideInner) {  // beside the body: side gap
      const float g = fmaxf(1.0f, fabsf(yr) - kHalfWidth);
      float& gs = yr > 0 ? gl : gr;
      if (gs == 0 || g < gs) gs = g;
    }
    if (fabsf(xr) > 30.0f) {  // ahead / behind, inside the corners' swing circle
      const float r = hypotf(xr, yr);
      if (r < kSwingR && (near == 0 || r < near)) {
        near = r;
        near_xr = xr;
      }
    }
    if (fabsf(yr) > half) continue;
    if (xr >= kAxisFwd) {  // ahead of the head: what sensor 0 would read straight ahead
      const float d = fmaxf(1.0f, xr - kAxisFwd - kSensorOut);
      if (front == 0 || d < front) front = d;
    } else {               // behind: what sensor 1 would read straight back
      const float d = fmaxf(1.0f, kAxisFwd - kSensorOut - xr);
      if (rear == 0 || d < rear) rear = d;
    }
  }
  front_ = front;
  rear_ = rear;
  gap_l_ = gl;
  gap_r_ = gr;
  near_mm_ = near;
  near_xr_ = near_xr;
}

// Side walls (#62): turn (deg/s, + = CCW = away from a right wall) away from walls closer than
// nav.side_min_mm to the body side; *inside = how far the nearer one is inside it (0 = none).
float Navigator::sideTurn(float* inside) const {
  const float smin = P(P_NAV_SIDE_MIN_MM);
  const float pl = smin > 0 && gap_l_ > 0 && gap_l_ < smin ? smin - gap_l_ : 0.0f;
  const float pr = smin > 0 && gap_r_ > 0 && gap_r_ < smin ? smin - gap_r_ : 0.0f;
  if (inside) *inside = fmaxf(pl, pr);
  return P(P_NAV_SIDE_KP) * (pr - pl);
}

// Driving: steer away from side walls and slow down (linear to nav.min_speed at gap 0).
void Navigator::sideAvoid(float vmax, float tmax) {
  float inside = 0;
  const float t = sideTurn(&inside);
  if (inside <= 0) return;
  turn_ = clampf(turn_ + t, tmax);
  const float smin = P(P_NAV_SIDE_MIN_MM);
  const float lim = fmaxf(P(P_NAV_MIN_SPEED), vmax * (smin - inside) / smin);
  if (speed_ > lim) speed_ = lim;
}

// Stuck (#62): commanded to move (trying) but no progress for nav.stuck_ms.
bool Navigator::stuckCheck(const NavIn& in, bool turning, bool trying) {
  if (!trying || P(P_NAV_STUCK_MS) <= 0) return false;
  stuck_seen_ = true;
  const bool moved = turning ? fabsf(in.heading - stuck_h_) >= kStuckDeg
                             : hypotf(in.x - stuck_x_, in.y - stuck_y_) >= P(P_NAV_STUCK_MM);
  if (!stuck_ms_ || turning != stuck_turn_ || moved) {
    stuck_ms_ = in.now_ms ? in.now_ms : 1;
    stuck_x_ = in.x;
    stuck_y_ = in.y;
    stuck_h_ = in.heading;
    stuck_turn_ = turning;
    return false;
  }
  return in.now_ms - stuck_ms_ >= (uint32_t)P(P_NAV_STUCK_MS);
}

void Navigator::startStuck(const NavIn& in, bool turning) {
  stuck_ = blocked_ = true;
  blk_h_ = in.heading;
  driving_ = true;  // so the BLOCKED branch runs (backs off) even if it was turning in place
  stuck_ms_ = 0;
  if (in.mode == MODE_EXPLORE) latched_ = true;
  log("nav: stuck, no progress %s for %.1f s (waypoint %u/%u)", turning ? "turning" : "driving",
      (double)(P(P_NAV_STUCK_MS) / 1000.0f), (unsigned)idx_ + 1, (unsigned)n_);
  back_mm_ = P(P_NAV_STUCK_BACK);
  backing_ = back_mm_ > 0;
  back_x_ = in.x;
  back_y_ = in.y;
  back_ms_ = in.now_ms;
  speed_ = turn_ = 0;
  state_ = NAV_BLOCKED;
  obst_ = front_;
}

float Navigator::limitSpeed(float v, float vmax, bool apply) {
  obst_flag_ = false;
  if (fabsf(v) < 1e-3f) {
    obst_ = front_;
    return v;
  }
  const bool fwd = v > 0;
  const float d = fwd ? front_ : rear_;
  bool& blk = fwd ? blk_fwd_ : blk_rev_;
  uint32_t& close_ms = fwd ? close_fwd_ms_ : close_rev_ms_;
  float& close_mm = fwd ? close_fwd_mm_ : close_rev_mm_;
  const float stop = P(P_RNG_OBSTACLE_MM);
  if (stop <= 0) {
    blk = false;
    obst_ = d;
    return v;
  }
  // Blocked until the way is clear: nothing closer than stop + clear (+ backoff) seen in the corridor
  // for nav.obst_age_ms (a sweep can miss a corner for a pass or two; no data is not "clear").
  const float clear = stop + P(P_NAV_CLEAR_MM) + P(P_NAV_BACKOFF_MM);
  if (d > 0 && d < clear) {
    close_ms = now_ms_ ? now_ms_ : 1;
    close_mm = d;
  }
  if (d > 0 && d < stop) blk = true;
  else if (blk && (d <= 0 || d > clear) && now_ms_ - close_ms >= (uint32_t)P(P_NAV_OBST_AGE_MS)) blk = false;
  obst_ = d > 0 ? d : (blk ? close_mm : 0.0f);
  if (blk) {
    obst_flag_ = true;
    return apply ? 0.0f : v;
  }
  if (d <= 0) return v;
  const float slow = P(P_NAV_SLOW_MM);
  if (apply && slow > stop && d < slow) {
    const float lim = fmaxf(P(P_NAV_MIN_SPEED), vmax * (d - stop) / (slow - stop));
    if (fabsf(v) > lim) v = fwd ? lim : -lim;
  }
  return v;
}

// ------------------------------------------------------------------ update

void Navigator::setTarget(const NavIn& in, float x, float y) {
  tx_ = x;
  ty_ = y;
  dist_ = hypotf(x - in.x, y - in.y);
}

void Navigator::update(const NavIn& in) {
  mode_ = in.mode;
  now_ms_ = in.now_ms;
  speed_ = turn_ = 0;
  obst_flag_ = false;
  stuck_seen_ = false;
  updateObstacles(in);
  if (in.mode == MODE_GOTO || in.mode == MODE_EXPLORE) {
    updateGoto(in);
  } else if (in.mode == MODE_HOLD) {
    updateHold(in);
  } else {
    state_ = NAV_IDLE;
    obst_ = front_;
    if (n_) setTarget(in, wx_[idx_ < n_ ? idx_ : n_ - 1], wy_[idx_ < n_ ? idx_ : n_ - 1]);
    else dist_ = 0;
  }
  if (!stuck_seen_) stuck_ms_ = 0;  // not trying to move this step: the no-progress window starts over
}

void Navigator::startBlocked(const NavIn& in) {
  blocked_ = true;
  blk_h_ = in.heading;
  if (in.mode == MODE_EXPLORE) latched_ = true;
  log("nav: blocked, obstacle at %.0f mm (waypoint %u/%u)", (double)obst_, (unsigned)idx_ + 1, (unsigned)n_);
  back_mm_ = P(P_NAV_BACKOFF_MM);
  if (back_mm_ > 0) {
    backing_ = true;
    back_x_ = in.x;
    back_y_ = in.y;
    back_ms_ = in.now_ms;
  }
}

void Navigator::updateGoto(const NavIn& in) {
  if (n_ == 0) {
    state_ = NAV_IDLE;
    tx_ = ty_ = dist_ = 0;
    obst_ = front_;
    return;
  }
  if (phase_ == DONE && idx_ < n_) phase_ = SEEK;
  const uint8_t ti = idx_ < n_ ? idx_ : n_ - 1;
  setTarget(in, wx_[ti], wy_[ti]);
  // No range readings: EXPLORE (wanders by itself) pauses; GOTO is capped at nav.blind_speed (0 = pauses).
  const bool no_range = blind(in.now_ms);
  if (no_range != warned_no_range_) {
    warned_no_range_ = no_range;
    if (no_range) logBlind(in.mode);
    else log("nav: range readings back - obstacle stop on");
  }
  const float blind_v = P(P_NAV_BLIND_SPEED);
  const bool blind_stop = no_range && (in.mode == MODE_EXPLORE || blind_v <= 0);
  const bool run = in.upright && in.live && !blind_stop;  // else paused: stand still, keep the state
  if (phase_ == DONE) {  // arrived: hold the last point (a balancing robot rolls on after stopping)
    state_ = NAV_ARRIVED;
    obst_ = front_;
    if (run) speed_ = holdSpeed(in, tx_, ty_);
    return;
  }
  const float vmax = fminf(fminf(P(P_NAV_MAX_SPEED), in.max_speed), no_range ? blind_v : 1e9f);
  const float tmax = fminf(P(P_NAV_MAX_TURN), in.max_turn);
  const float arrive = P(P_NAV_ARRIVE_MM);
  const bool stopped = fabsf(in.speed) < P(P_NAV_STOP_SPEED) && fabsf(in.yaw_rate) < P(P_NAV_STOP_TURN);
  if (!run) {
    stop_since_ = 0;
    obst_ = front_;
    return;
  }

  if (phase_ == END_TURN) {
    const float e = wrap180(P(P_NAV_END_HEADING) - in.heading);
    turn_ = clampf(P(P_NAV_TURN_KP) * e, tmax);
    speed_ = holdSpeed(in, tx_, ty_);
    state_ = NAV_TURNING;
    obst_ = front_;
    if (fabsf(e) < P(P_NAV_HEADING_TOL) && fabsf(in.yaw_rate) < P(P_NAV_STOP_TURN)) {
      turn_ = 0;
      phase_ = DONE;
      idx_ = n_;
      state_ = NAV_ARRIVED;
      log("nav: arrived, heading %.1f", (double)wrap180(in.heading));
    }
    return;
  }

  if (phase_ == STOPPING) {
    state_ = NAV_DRIVING;
    obst_ = front_;
    // hold the point (forward / back, no steering) until it has really stopped
    speed_ = holdSpeed(in, tx_, ty_);
    obst_ = front_;
    if (dist_ > 2.0f * arrive) {  // overshot / pushed away: seek again
      phase_ = SEEK;
      driving_ = false;
    } else if (!stopped) {
      stop_since_ = 0;
    } else {
      if (!stop_since_) stop_since_ = in.now_ms ? in.now_ms : 1;
      if (in.now_ms - stop_since_ >= (uint32_t)P(P_NAV_DWELL_MS)) {
        log("nav: reached %u/%u, %.0f mm off", (unsigned)idx_ + 1, (unsigned)n_, (double)dist_);
        stop_since_ = 0;
        if (idx_ + 1 < n_) {
          idx_++;
          phase_ = SEEK;
          driving_ = false;
          setTarget(in, wx_[idx_], wy_[idx_]);
          state_ = NAV_TURNING;
        } else if (P(P_NAV_END_HEADING) >= 0) {
          phase_ = END_TURN;
          state_ = NAV_TURNING;
        } else {
          phase_ = DONE;
          idx_ = n_;
          state_ = NAV_ARRIVED;
          log("nav: arrived");
        }
      }
    }
    return;
  }

  // SEEK
  if (dist_ < arrive) {
    phase_ = STOPPING;
    stop_since_ = 0;
    blocked_ = latched_ = backing_ = false;
    state_ = NAV_DRIVING;
    obst_ = front_;
    return;
  }
  const float bearing = atan2f(ty_ - in.y, tx_ - in.x) / kDeg;
  const float e = wrap180(bearing - in.heading);
  const bool far = dist_ > 2.0f * arrive;  // close in, the bearing is noisy: don't steer
  const float align = P(P_NAV_ALIGN_DEG);
  if (!blocked_) {
    if (!driving_ && fabsf(e) < align) {
      driving_ = true;
      away_ = away_done_ = false;
    } else if (driving_ && far && fabsf(e) > 3.0f * align) {
      driving_ = false;
    }
  }
  if (!driving_) {
    state_ = NAV_TURNING;
    obst_ = front_;
    // something inside the corners' swing circle ahead / behind: first step straight away from it (#62)
    if (!away_done_ && near_mm_ > 0 && P(P_NAV_STUCK_BACK) > 0) {
      if (!away_) {
        away_ = true;
        away_x_ = in.x;
        away_y_ = in.y;
        away_ms_ = in.now_ms;
        log("nav: too close to turn (%.0f mm), stepping %s", (double)near_mm_, near_xr_ > 0 ? "back" : "forward");
      }
      const float va = limitSpeed(near_xr_ > 0 ? -fmaxf(P(P_NAV_MIN_SPEED), 30.0f) : fmaxf(P(P_NAV_MIN_SPEED), 30.0f),
                                  vmax, true);
      if (!obst_flag_ && hypotf(in.x - away_x_, in.y - away_y_) < P(P_NAV_STUCK_BACK) &&
          in.now_ms - away_ms_ < 4000) {
        speed_ = va;
        obst_ = front_;
        return;
      }
      away_done_ = true;  // far enough (or the way out is blocked too): turn anyway
    }
    away_ = false;
    obst_flag_ = false;
    turn_ = clampf(P(P_NAV_TURN_KP) * e, tmax);
    // target behind and a wall close at one side: turn the nose away from that wall (#62)
    const float smin = P(P_NAV_SIDE_MIN_MM);
    if (smin > 0 && fabsf(e) > 120.0f) {
      const bool l = gap_l_ > 0 && gap_l_ < smin, r = gap_r_ > 0 && gap_r_ < smin;
      if (l && (!r || gap_l_ < gap_r_)) turn_ = -fabsf(turn_);
      else if (r) turn_ = fabsf(turn_);
    }
    if (stuckCheck(in, true, fabsf(turn_) >= P(P_NAV_STOP_TURN))) startStuck(in, true);
    return;
  }
  float v = P(P_NAV_SPEED_KP) * dist_ * fmaxf(0.0f, cosf(e * kDeg));
  v = fminf(fmaxf(v, P(P_NAV_MIN_SPEED)), vmax);
  float vl = limitSpeed(v, vmax, true);
  if (latched_ || stuck_) obst_flag_ = true;  // EXPLORE: stays blocked until a new list; stuck: backing off
  if (obst_flag_) {
    if (!blocked_) startBlocked(in);
    const float obst = obst_;
    vl = 0;
    if (backing_) {
      const float back = hypotf(in.x - back_x_, in.y - back_y_);
      // stuck: twice as fast (a balancing robot pressed against a wall barely follows the slowest speed)
      float vb = -fmaxf(P(P_NAV_MIN_SPEED), 30.0f) * (stuck_ ? 2.0f : 1.0f);
      vb = limitSpeed(vb, vmax, true);
      if (obst_flag_ || back >= back_mm_ || in.now_ms - back_ms_ > 5000) {
        backing_ = false;
        vb = 0;
      }
      vl = vb;
      obst_ = obst;  // report the obstacle ahead
      obst_flag_ = true;
    }
    if (!backing_ && in.mode != MODE_EXPLORE) stuck_ = false;  // GOTO: backed off, try again
    speed_ = vl;
    turn_ = 0;
    // GOTO, blocked with a wall alongside (heading into it): turn away from it a little, in place (#62)
    if (!backing_ && !latched_ && !stuck_ && fabsf(in.heading - blk_h_) < kBlockedTurnDeg)
      turn_ = clampf(sideTurn(nullptr), tmax);
    state_ = NAV_BLOCKED;
    return;
  }
  if (blocked_) log("nav: clear, going on");
  blocked_ = backing_ = false;
  speed_ = vl;
  turn_ = far ? clampf(P(P_NAV_TURN_KP) * e, tmax) : 0.0f;
  sideAvoid(vmax, tmax);
  state_ = NAV_DRIVING;
  // (not in the last 3 x arrive_mm: slow there, and a balancing robot rocks to and fro before it moves)
  if (stuckCheck(in, false, dist_ > 3.0f * arrive && speed_ >= fmaxf(20.0f, 0.5f * P(P_NAV_MIN_SPEED))))
    startStuck(in, false);
}

float Navigator::holdSpeed(const NavIn& in, float x, float y) {
  const float h = in.heading * kDeg;
  const float f = (x - in.x) * cosf(h) + (y - in.y) * sinf(h);  // forward error
  if (fabsf(f) <= P(P_NAV_HOLD_DB_MM)) return 0.0f;
  const float vmax = fminf(P(P_NAV_HOLD_MAX), in.max_speed);
  return limitSpeed(clampf(P(P_NAV_HOLD_KP) * f, vmax), vmax, true);
}

void Navigator::updateHold(const NavIn& in) {
  setTarget(in, hold_x_, hold_y_);
  state_ = NAV_HOLDING;
  obst_ = front_;
  if (!in.upright || !in.live) return;
  speed_ = holdSpeed(in, hold_x_, hold_y_);
  if (obst_flag_) state_ = NAV_BLOCKED;
}

// ------------------------------------------------------------------ status / log

bool Navigator::statusDue(uint32_t now_ms) const {
  if (state_ != sent_state_ || idx_ != sent_idx_ || n_ != sent_n_) return true;
  return navMode(mode_) && now_ms - sent_ms_ >= 500;
}

void Navigator::statusSent(uint32_t now_ms) {
  sent_state_ = state_;
  sent_idx_ = idx_;
  sent_n_ = n_;
  sent_ms_ = now_ms;
}

void Navigator::log(const char* fmt, ...) {
  if (log_n_ >= 4) return;  // full: drop
  char* s = log_[(log_head_ + log_n_) % 4];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s, sizeof(log_[0]), fmt, ap);
  va_end(ap);
  log_n_++;
}

void Navigator::popLog() {
  if (!log_n_) return;
  log_head_ = (log_head_ + 1) % 4;
  log_n_--;
}
