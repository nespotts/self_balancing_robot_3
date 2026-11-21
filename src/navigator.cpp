#include "navigator.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "params.h"

// Range head geometry (docs/firmware.md "Range sweep"): servo axis 26 mm ahead of the axle, each sensor
// 12.9 mm out from it, 189 mm above the axle.
static constexpr float kAxisFwd = 26.0f, kSensorOut = 12.9f, kSensorHeight = 189.0f;
static constexpr float kDeg = 0.017453292519943f;

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
}

// ------------------------------------------------------------------ inputs

void Navigator::addRange(const RangeFrame& f, uint32_t now_ms) {
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
  if (in.mode == MODE_GOTO || in.mode == MODE_EXPLORE) {
    if (idx_ >= n_) idx_ = 0;  // list done (or empty): start over
    phase_ = n_ ? SEEK : DONE;
  }
  if (in.mode == MODE_HOLD) {
    hold_x_ = in.x;
    hold_y_ = in.y;
  }
  if (navMode(in.mode)) {
    if (!last_range_ms_ || in.now_ms - last_range_ms_ > 3000) log("nav: no range readings - obstacle stop is off");
    if (in.mode != MODE_HOLD) log("nav: %u waypoints, going to %u", (unsigned)n_, (unsigned)idx_ + 1);
  }
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
  const float half = 0.5f * P(P_NAV_CORRIDOR_MM);
  const uint32_t age = (uint32_t)P(P_NAV_OBST_AGE_MS);
  float front = 0, rear = 0;
  for (uint8_t i = 0; i < pt_n_; i++) {
    if (in.now_ms - pt_ms_[i] > age) continue;
    const float dx = px_[i] - in.x, dy = py_[i] - in.y;
    const float xr = dx * c + dy * s, yr = -dx * s + dy * c;
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
}

void Navigator::startBlocked(const NavIn& in) {
  blocked_ = true;
  if (in.mode == MODE_EXPLORE) latched_ = true;
  log("nav: blocked, obstacle at %.0f mm (waypoint %u/%u)", (double)obst_, (unsigned)idx_ + 1, (unsigned)n_);
  if (P(P_NAV_BACKOFF_MM) > 0) {
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
  const bool run = in.upright && in.live;  // else paused: stand still, keep the state
  if (phase_ == DONE) {  // arrived: hold the last point (a balancing robot rolls on after stopping)
    state_ = NAV_ARRIVED;
    obst_ = front_;
    if (run) speed_ = holdSpeed(in, tx_, ty_);
    return;
  }
  const float vmax = fminf(P(P_NAV_MAX_SPEED), in.max_speed);
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
    if (!driving_ && fabsf(e) < align) driving_ = true;
    else if (driving_ && far && fabsf(e) > 3.0f * align) driving_ = false;
  }
  if (!driving_) {
    turn_ = clampf(P(P_NAV_TURN_KP) * e, tmax);
    state_ = NAV_TURNING;
    obst_ = front_;
    return;
  }
  float v = P(P_NAV_SPEED_KP) * dist_ * fmaxf(0.0f, cosf(e * kDeg));
  v = fminf(fmaxf(v, P(P_NAV_MIN_SPEED)), vmax);
  float vl = limitSpeed(v, vmax, true);
  if (latched_) obst_flag_ = true;  // EXPLORE: stays blocked until a new list
  if (obst_flag_) {
    if (!blocked_) startBlocked(in);
    const float obst = obst_;
    vl = 0;
    if (backing_) {
      const float back = hypotf(in.x - back_x_, in.y - back_y_);
      float vb = -fmaxf(P(P_NAV_MIN_SPEED), 30.0f);
      vb = limitSpeed(vb, vmax, true);
      if (obst_flag_ || back >= P(P_NAV_BACKOFF_MM) || in.now_ms - back_ms_ > 5000) {
        backing_ = false;
        vb = 0;
      }
      vl = vb;
      obst_ = obst;  // report the obstacle ahead
      obst_flag_ = true;
    }
    speed_ = vl;
    turn_ = 0;
    state_ = NAV_BLOCKED;
    return;
  }
  if (blocked_) log("nav: clear, going on");
  blocked_ = backing_ = false;
  speed_ = vl;
  turn_ = far ? clampf(P(P_NAV_TURN_KP) * e, tmax) : 0.0f;
  state_ = NAV_DRIVING;
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
