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
// LD19 dead zone (#99): the bridge drops returns closer than its min_range_mm (120 mm default, 60 on the robot) to
// the lidar; + 60 mm for the mount offset (x_mm -12), the lean (170 mm above the axle) and a scan's travel. Lidar
// points closer than this to the axle are kept as ghosts until the robot is this far from them again.
static constexpr float kLidarDeadR = 180.0f;
static constexpr float kDeadFront = kLidarDeadR - kAxisFwd - kSensorOut;  // the same, as a front distance (141)
static constexpr float kGhostMerge = 40.0f;               // a new point this close to a ghost stands for it
// Pass-through waypoints (#103)
static constexpr float kSharpDeg = 75.0f;   // a sharper corner: slow down as for a stop and turn in place there
static constexpr float kLookS = 1.0f;       // pursuit lookahead / pass radius: this x the speed (>= nav.arrive_mm)
static constexpr float kCutMm = 30.0f;      // corner speed: the pursuit cuts a corner by about this much
static constexpr float kLookFar = 0.5f;     // pursuit lookahead up to this x the distance to the waypoint
static constexpr uint8_t kCapAhead = 8;     // corners ahead that limit the speed
static constexpr uint8_t kPursueAhead = 3;  // waypoints ahead the pursuit point may be on

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
    // driving on (no blocked / stuck): a new route while under way (the planner's resends) does not stop it to
    // turn in place first (#103); a large bearing error still does
    const bool on = phase_ == SEEK && driving_ && !blocked_ && !stuck_;
    idx_ = 0;
    phase_ = SEEK;
    driving_ = on;
  } else if (phase_ == DONE && idx_ < n_) {
    phase_ = SEEK;
    driving_ = false;
  }
  if (n_ == 0) idx_ = 0;
  leg_set_ = q_set_ = false;               // the leg to the (new) waypoint starts at the pose
  blocked_ = latched_ = backing_ = false;  // a new / changed list always resumes
  if (fresh(now_ms_)) blk_fwd_ = blk_rev_ = false;  // ... but without fresh data a latch stays (#99)
  dead_back_ = false;
  stop_since_ = 0;
  stuck_ = away_ = away_done_ = false;
  stuck_ms_ = 0;
  contact_ = false;  // (#104) the planner's new route goes round the mark
  gave_up_ = false;  // (#111)
  stop_n_ = 0;
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

// LD19 obstacles (#77): robot-frame points of the pose age_ms ago (dead-reckoned back from the pose now with its
// speed and yaw rate) -> pose frame. They replace the previous frame's points.
bool Navigator::addLidar(const LidarObstacleFrame& f, const NavIn& in) {
  if (f.sectors != LINK_LIDAR_SECTORS || f.age_ms > LINK_LIDAR_MAX_AGE_MS) return false;
  const float age = (float)f.age_ms * 0.001f;
  const float hm = (in.heading - 0.5f * in.yaw_rate * age) * kDeg;  // mean heading over that time
  const float ox = in.x - in.speed * age * cosf(hm), oy = in.y - in.speed * age * sinf(hm);
  const float h = (in.heading - in.yaw_rate * age) * kDeg;
  const float c = cosf(h), s = sinf(h);
  uint8_t n = 0;
  for (uint8_t i = 0; i < LINK_LIDAR_SECTORS; i++) {
    const float x = f.xy_mm[2 * i], y = f.xy_mm[2 * i + 1];
    if (x == 0 && y == 0) continue;  // no return in this sector
    lx_[n] = ox + x * c - y * s;
    ly_[n] = oy + x * s + y * c;
    n++;
  }
  lidar_n_ = n;
  lidar_ms_ = in.now_ms ? in.now_ms : 1;
  // ghosts (#99): this frame's points within kLidarDeadR of the axle, plus the older ghosts still that close that no
  // new point stands for (they may be inside the lidar's min range now); the kGhosts nearest are kept
  float cx[kGhosts + LINK_LIDAR_SECTORS], cy[kGhosts + LINK_LIDAR_SECTORS], cr[kGhosts + LINK_LIDAR_SECTORS];
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) {
    const float r = hypotf(lx_[i] - in.x, ly_[i] - in.y);
    if (r >= kLidarDeadR) continue;
    cx[m] = lx_[i];
    cy[m] = ly_[i];
    cr[m] = r;
    m++;
  }
  const uint8_t m_new = m;
  for (uint8_t j = 0; j < ghost_n_; j++) {
    const float r = hypotf(gx_[j] - in.x, gy_[j] - in.y);
    if (r >= kLidarDeadR) continue;
    bool seen = false;
    for (uint8_t i = 0; i < m_new && !seen; i++) seen = hypotf(cx[i] - gx_[j], cy[i] - gy_[j]) < kGhostMerge;
    if (seen) continue;
    cx[m] = gx_[j];
    cy[m] = gy_[j];
    cr[m] = r;
    m++;
  }
  ghost_n_ = 0;
  while (ghost_n_ < kGhosts && m) {
    uint8_t b = 0;
    for (uint8_t i = 1; i < m; i++)
      if (cr[i] < cr[b]) b = i;
    gx_[ghost_n_] = cx[b];
    gy_[ghost_n_] = cy[b];
    ghost_n_++;
    m--;
    cx[b] = cx[m];
    cy[b] = cy[m];
    cr[b] = cr[m];
  }
  return true;
}

void Navigator::onModeChange(const NavIn& in) {
  mode_ = in.mode;
  now_ms_ = in.now_ms;
  speed_ = turn_ = 0;
  blocked_ = latched_ = backing_ = false;
  if (fresh(in.now_ms)) blk_fwd_ = blk_rev_ = false;  // without fresh data a latch stays (#99)
  dead_back_ = false;
  driving_ = false;
  leg_set_ = q_set_ = false;
  last_v_ = look_far_ = 0;
  stop_since_ = 0;
  stuck_ = away_ = away_done_ = false;
  stuck_ms_ = 0;
  lidar_stuck_ms_ = 0;
  contact_ = contact_watch_ = launch_ = contact_ramp_ = gave_up_ = false;
  stop_n_ = 0;
  contact_src_ = SRC_NONE;
  cdet_.reset();
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

// No fresh lidar frame and no RangeFrame for kBlindMs (rng.enable 0, sensors dead): no obstacle stop is
// possible (safety review #38).
bool Navigator::blind(uint32_t now_ms) const {
  if (lidarFresh(now_ms)) return false;
  return !sensed_ms_ || now_ms - sensed_ms_ > kBlindMs;
}

// Fresh obstacle data (#99): only this can release a BLOCKED latch (never blind, stale or "no data").
// (signed: listChanged() passes the last update's time, and a frame may have come in a few ms after it)
bool Navigator::fresh(uint32_t now_ms) const {
  const auto recent = [now_ms](uint32_t t, uint32_t lim) { return t && (int32_t)(now_ms - t) <= (int32_t)lim; };
  return recent(lidar_ms_, LINK_LIDAR_FRESH_MS) || recent(sensed_ms_, kSweepFreshMs);
}

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
  lidar_n_ = 0;
  lidar_ms_ = 0;  // its points were in the old frame: no lidar until the next frame
  // ghosts (#99): the same spots around the robot, in the new frame (the pose is (0,0,0) now)
  const float c = cosf(pose_h_ * kDeg), s = sinf(pose_h_ * kDeg);
  for (uint8_t i = 0; i < ghost_n_; i++) {
    const float dx = gx_[i] - pose_x_, dy = gy_[i] - pose_y_;
    gx_[i] = dx * c + dy * s;
    gy_[i] = -dx * s + dy * c;
  }
  pose_x_ = pose_y_ = pose_h_ = 0;
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
  const bool lidar = lidarFresh(in.now_ms);
  // ghosts the robot has moved away from (the lidar would see them again if they were there) are dropped (#99)
  uint8_t gn = 0;
  for (uint8_t i = 0; i < ghost_n_; i++) {
    if (hypotf(gx_[i] - in.x, gy_[i] - in.y) >= kLidarDeadR) continue;
    gx_[gn] = gx_[i];
    gy_[gn] = gy_[i];
    gn++;
  }
  ghost_n_ = gn;
  float front = 0, rear = 0, gl = 0, gr = 0, near = 0, near_xr = 0;
  auto take = [&](float px, float py) {
    const float dx = px - in.x, dy = py - in.y;
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
    if (fabsf(yr) > half) return;
    if (xr >= kAxisFwd) {  // ahead of the head: what sensor 0 would read straight ahead
      const float d = fmaxf(1.0f, xr - kAxisFwd - kSensorOut);
      if (front == 0 || d < front) front = d;
    } else {               // behind: what sensor 1 would read straight back
      const float d = fmaxf(1.0f, kAxisFwd - kSensorOut - xr);
      if (rear == 0 || d < rear) rear = d;
    }
  };
  // fresh lidar and the sweep's recent points together (#99: the union), and the ghosts
  if (lidar)
    for (uint8_t i = 0; i < lidar_n_; i++) take(lx_[i], ly_[i]);
  for (uint8_t i = 0; i < pt_n_; i++)
    if (in.now_ms - pt_ms_[i] <= age) take(px_[i], py_[i]);
  for (uint8_t i = 0; i < ghost_n_; i++) take(gx_[i], gy_[i]);
  front_ = front;
  rear_ = rear;
  gap_l_ = gl;
  gap_r_ = gr;
  near_mm_ = near;
  near_xr_ = near_xr;
  // release evidence for limitSpeed (#99): since when fresh data shows nothing inside the clear distance
  const bool ok = fresh(in.now_ms);
  const float clear = P(P_RNG_OBSTACLE_MM) + P(P_NAV_CLEAR_MM) + P(P_NAV_BACKOFF_MM);
  const uint32_t t = in.now_ms ? in.now_ms : 1;
  if (!ok || (front > 0 && front < clear)) clr_fwd_ms_ = 0;
  else if (!clr_fwd_ms_) clr_fwd_ms_ = t;
  if (!ok || (rear > 0 && rear < clear)) clr_rev_ms_ = 0;
  else if (!clr_rev_ms_) clr_rev_ms_ = t;
  pose_x_ = in.x;
  pose_y_ = in.y;
  pose_h_ = in.heading;
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
  if (!turning && lidar_stuck_ms_) {  // the lidar saw no motion while the wheels turn (#79)
    lidar_stuck_ms_ = 0;
    stuck_lidar_ = true;
    return true;
  }
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
  if (stuck_lidar_)
    log("nav: stuck, wheels turn but the lidar sees no motion (waypoint %u/%u)", (unsigned)idx_ + 1, (unsigned)n_);
  else
    log("nav: stuck, no progress %s for %.1f s (waypoint %u/%u)", turning ? "turning" : "driving",
        (double)(P(P_NAV_STUCK_MS) / 1000.0f), (unsigned)idx_ + 1, (unsigned)n_);
  // #111: driving, the wheels not getting anywhere: stand up (contactReset) before backing off, or a robot leaning on
  // what stopped it can't back away. No contact report: a balancing robot crawling at 40 mm/s - a squeeze through a
  // doorway, a slow zone - can stand in its motor dead band this long with nothing in front of it, and a mark there
  // shuts the doorway. Again and again at one spot: the planner's same-spot rule marks it
  if (!turning && !stuck_lidar_) {
    unwind_ = true;
    noteStop(in);
  }
  stuck_lidar_ = false;
  back_mm_ = P(P_NAV_STUCK_BACK);
  backing_ = back_mm_ > 0;
  back_x_ = in.x;
  back_y_ = in.y;
  back_ms_ = in.now_ms;
  speed_ = turn_ = 0;
  state_ = NAV_BLOCKED;
  obst_ = front_;
}

// Contacts (#104): obstacles under the lidar's scan plane, found by touch (src/contact.h). Watching: GOTO / EXPLORE
// when the last step drove forward, STATION while it commands motion (reported only). nav.stuck_ms 0 = off.
void Navigator::contactCheck(const NavIn& in) {
  const bool station = in.mode == MODE_STATION;
  ContactIn c;
  c.now_ms = in.now_ms;
  c.upright = in.upright;
  c.cmd = in.cmd_speed;
  c.speed = in.speed;
  c.pitch_rate = in.pitch_rate;
  c.vel_out = in.vel_out;
  c.lidar_stuck = lidar_stuck_ms_ != 0;
  c.cruise = station || contact_cruise_;
  c.ramp = !station && contact_ramp_;
  c.drive = station || contact_src_ == SRC_DRIVE;
  c.held = in.held;
  const bool nav = (in.mode == MODE_GOTO || in.mode == MODE_EXPLORE) && contact_watch_;
  c.watch = P(P_NAV_STUCK_MS) > 0 && (nav || (station && in.live && in.now_ms - report_ms_ >= kReportGapMs));
  contact_watch_ = false;  // (the driving / backing / stepping branches set it again)
  const uint8_t kind = cdet_.update(c);
  if (!kind) return;
  if (cdet_.flags() & LINK_CONTACT_LIDAR) lidar_stuck_ms_ = 0;  // used up
  if (station) {
    report_ms_ = in.now_ms ? in.now_ms : 1;
    reportContact(in, kind, cdet_.flags() | LINK_CONTACT_REPORT, false, cdet_.dir(), cdet_.value());
  } else if (in.mode == MODE_GOTO || in.mode == MODE_EXPLORE) {
    startContact(in, kind);
  }
}

void Navigator::reportContact(const NavIn& in, uint8_t kind, uint8_t flags, bool hold, float dir, float value) {
  const float h = dir > 0 ? in.heading : in.heading + 180.0f;
  ContactFrame& f = contact_frame_;
  f.kind = kind;
  f.seq = ++contact_seq_;
  f.mode = in.mode;
  f.flags = flags;
  f.x_mm = in.x + kContactAhead * cosf(h * kDeg);
  f.y_mm = in.y + kContactAhead * sinf(h * kDeg);
  f.heading_deg = h;
  f.value = value;
  contact_due_ = true;
  char why[28];
  if (kind == LINK_CONTACT_BUMP) snprintf(why, sizeof(why), "%.0f dps", (double)f.value);
  else snprintf(why, sizeof(why), "%.0f ms", (double)f.value);
  log("nav: contact %s at %.0f,%.0f (%s%s%s%s, %s)", kind == LINK_CONTACT_BUMP ? "bump" : "stuck", (double)f.x_mm,
      (double)f.y_mm, why, dir < 0 ? ", rear" : "", flags & LINK_CONTACT_LIDAR ? ", lidar" : "",
      flags & LINK_CONTACT_FELL ? ", fell" : "",
      flags & LINK_CONTACT_REPORT ? "reported only" : (hold ? "holding" : "backing off"));
}

void Navigator::startContact(const NavIn& in, uint8_t kind) {
  // found while backing off / stepping away (#111): that way is blocked too - stop and hold here, don't push on
  const bool hold = contact_src_ != SRC_DRIVE;
  reportContact(in, kind, cdet_.flags(), hold, cdet_.dir(), cdet_.value());
  contact_ = stuck_ = blocked_ = latched_ = true;  // GOTO: latched_ until a new list or kContactWaitMs
  contact_ms_ = in.now_ms;
  blk_h_ = in.heading;
  driving_ = true;  // the BLOCKED branch backs off / holds
  stuck_ms_ = 0;
  if (contact_src_ == SRC_AWAY) away_done_ = true;
  away_ = false;
  dead_back_ = hold;  // (held: no dead-zone back-off into it either)
  back_mm_ = hold ? 0.0f : P(P_NAV_STUCK_BACK);  // (driving forward: back off backwards, the rear corridor checked)
  backing_ = back_mm_ > 0;
  back_x_ = contact_hx_ = in.x;
  back_y_ = contact_hy_ = in.y;
  back_ms_ = in.now_ms;
  unwind_ = true;  // (#111) the controller: command 0, the velocity loop's lean back to its free value
  if (!hold) noteStop(in);
  speed_ = turn_ = 0;
  state_ = NAV_BLOCKED;
  obst_ = front_;
}

// #111: GOTO stopped driving toward the same waypoint at the same spot again (a contact or a stuck while driving):
// after kGiveUpStops it gives up on it - BLOCKED until a new list (18:36: 13 tries in 3 min at a goal next to a low
// box, the planner idle). A new list, a mode change or getting past the spot starts the count again.
void Navigator::noteStop(const NavIn& in) {
  if (stop_n_ && idx_ == stop_idx_ && hypotf(in.x - stop_x_, in.y - stop_y_) < kSameSpotMm) {
    stop_n_++;
  } else {
    stop_n_ = 1;
    stop_x_ = in.x;
    stop_y_ = in.y;
    stop_idx_ = idx_;
  }
  if (stop_n_ >= kGiveUpStops && in.mode == MODE_GOTO && !gave_up_) {
    gave_up_ = latched_ = true;
    log("nav: gave up on waypoint %u/%u: stopped %u times at %.0f,%.0f - waiting for a new list", (unsigned)idx_ + 1,
        (unsigned)n_, (unsigned)stop_n_, (double)stop_x_, (double)stop_y_);
  }
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
  const uint32_t clr_ms = fwd ? clr_fwd_ms_ : clr_rev_ms_;
  float& close_mm = fwd ? close_fwd_mm_ : close_rev_mm_;
  const float stop = P(P_RNG_OBSTACLE_MM);
  if (stop <= 0) {
    blk = false;
    obst_ = d;
    return v;
  }
  // Blocked until the way is clear: fresh data (updateObstacles) has shown nothing closer than stop + clear
  // (+ backoff) in the corridor for nav.obst_age_ms (a sweep can miss a corner for a pass or two). No data,
  // stale data or blind is never "clear" (#99); an obstacle in the lidar's dead zone stays as a ghost.
  const float clear = stop + P(P_NAV_CLEAR_MM) + P(P_NAV_BACKOFF_MM);
  if (d > 0 && d < clear) close_mm = d;
  if (d > 0 && d < stop) blk = true;
  else if (blk && clr_ms && now_ms_ - clr_ms >= (uint32_t)P(P_NAV_OBST_AGE_MS)) blk = false;
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
  const bool lidar = lidarFresh(in.now_ms);
  if (lidar != lidar_was_) {
    lidar_was_ = lidar;
    if (navMode(in.mode)) log(lidar ? "nav: lidar obstacles on" : "nav: lidar obstacles lost (no scan for 0.5 s)");
  }
  updateObstacles(in);
  contactCheck(in);  // #104: may start BLOCKED (contact) before the mode's logic runs
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
  if (lidar_stuck_ms_ && in.now_ms - lidar_stuck_ms_ > kLidarStuckMs) lidar_stuck_ms_ = 0;  // not driving: dropped
  last_v_ = speed_;
  last_ms_ = in.now_ms;
}

// ------------------------------------------------------------------ pass-through waypoints (#103)

void Navigator::legStart(uint8_t i, float* x, float* y) const {
  *x = i == idx_ ? sx_ : wx_[i - 1];
  *y = i == idx_ ? sy_ : wy_[i - 1];
}

float Navigator::cornerDeg(uint8_t i, float heading) const {
  float ax, ay;
  legStart(i, &ax, &ay);
  const float ix = wx_[i] - ax, iy = wy_[i] - ay, ox = wx_[i + 1] - wx_[i], oy = wy_[i + 1] - wy_[i];
  if (ox * ox + oy * oy < 1.0f) return 0.0f;  // the same point twice
  const float a_in = ix * ix + iy * iy < 1.0f ? heading : atan2f(iy, ix) / kDeg;
  return fabsf(wrap180(atan2f(oy, ox) / kDeg - a_in));
}

// Speed cap at the current waypoint: each corner ahead at most vc, reached with at most nav.speed_kp x the leg to
// spare; 0 at the last point (it stops there). vc: the pursuit (lookahead kLookS x speed) cuts a corner of th by
// about kLookS x v x tan(th / 4), so v <= kCutMm / tan(th / 4) / kLookS (>= nav.min_speed); 0 sharper than kSharpDeg.
// stops_only (#104): only the stops count (the last point, sharp corners), pass-through corners not.
float Navigator::cornerCap(float heading, float vmax, bool stops_only) const {
  const int top = (int)n_ - 2 < (int)idx_ + kCapAhead ? (int)n_ - 2 : (int)idx_ + kCapAhead;
  const float kp = P(P_NAV_SPEED_KP), vmin = fminf(P(P_NAV_MIN_SPEED), vmax);
  float cap = top == (int)n_ - 2 ? 0.0f : vmax;
  for (int i = top; i >= (int)idx_; i--) {
    const float th = cornerDeg((uint8_t)i, heading);
    const float tq = tanf(0.25f * th * kDeg);
    const float vc = th > kSharpDeg ? 0.0f
                     : stops_only ? vmax : fmaxf(vmin, tq * vmax * kLookS > kCutMm ? kCutMm / (tq * kLookS) : vmax);
    cap = fminf(vc, cap + kp * hypotf(wx_[i + 1] - wx_[i], wy_[i + 1] - wy_[i]));
  }
  return cap;
}

// Segment (x0,y0) -> (x1,y1): where a circle of radius L about (px,py) leaves it (false: it doesn't).
static bool circleExit(float x0, float y0, float x1, float y1, float px, float py, float L, float* ax, float* ay) {
  const float dx = x1 - x0, dy = y1 - y0, a = dx * dx + dy * dy;
  if (a < 1.0f) return false;
  const float fx = x0 - px, fy = y0 - py;
  const float b = fx * dx + fy * dy, c = fx * fx + fy * fy - L * L;
  const float disc = b * b - a * c;
  if (disc < 0) return false;
  const float t = (-b + sqrtf(disc)) / a;  // the far crossing
  if (t < 0 || t > 1) return false;
  *ax = x0 + t * dx;
  *ay = y0 + t * dy;
  return true;
}

static float segDist(float x0, float y0, float x1, float y1, float px, float py) {
  const float dx = x1 - x0, dy = y1 - y0, a = dx * dx + dy * dy;
  const float t = a > 1.0f ? fminf(1.0f, fmaxf(0.0f, ((px - x0) * dx + (py - y0) * dy) / a)) : 1.0f;
  return hypotf(x0 + t * dx - px, y0 + t * dy - py);
}

// Pure pursuit: the point L from the robot on the route ((the leg before, q_set_) leg start -> waypoints idx_ ..
// last), furthest along. false: the route is farther than L away (aim at the waypoint).
bool Navigator::pursuit(const NavIn& in, float L, uint8_t last, float* ax, float* ay) const {
  if (hypotf(wx_[last] - in.x, wy_[last] - in.y) <= L) {
    *ax = wx_[last];
    *ay = wy_[last];
    return true;
  }
  for (int k = last; k >= (int)idx_; k--) {
    float x0, y0;
    legStart((uint8_t)k, &x0, &y0);
    if (circleExit(x0, y0, wx_[k], wy_[k], in.x, in.y, L, ax, ay)) return true;
  }
  return q_set_ && circleExit(qx_, qy_, sx_, sy_, in.x, in.y, L, ax, ay);
}

float Navigator::routeOff(const NavIn& in) const {
  const float d = segDist(sx_, sy_, tx_, ty_, in.x, in.y);
  return q_set_ ? fminf(d, segDist(qx_, qy_, sx_, sy_, in.x, in.y)) : d;
}

void Navigator::startBlocked(const NavIn& in) {
  blocked_ = true;
  blk_h_ = in.heading;
  dead_back_ = false;
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
        if (idx_ + 1 < n_) {  // (points added while it stopped at the last one)
          q_set_ = false;
          sx_ = tx_;
          sy_ = ty_;
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
  if (!leg_set_) {
    sx_ = in.x;
    sy_ = in.y;
    leg_set_ = true;
  }
  // pass-through (#103): an intermediate waypoint is done once closer than L (sharp corner: nav.arrive_mm) or
  // passed (beyond its end along its leg); a sharp corner then turns in place
  const float L = fmaxf(arrive, kLookS * fabsf(last_v_));
  while (idx_ + 1 < n_) {
    const bool sharp = cornerDeg(idx_, in.heading) > kSharpDeg;
    const bool passed = (in.x - tx_) * (tx_ - sx_) + (in.y - ty_) * (ty_ - sy_) >= 0;
    if (!passed && dist_ >= (sharp ? arrive : L)) break;
    log("nav: passed %u/%u, %.0f mm off", (unsigned)idx_ + 1, (unsigned)n_, (double)dist_);
    qx_ = sx_;
    qy_ = sy_;
    q_set_ = !sharp;  // (a sharp corner: the new leg only)
    sx_ = tx_;
    sy_ = ty_;
    idx_++;
    setTarget(in, wx_[idx_], wy_[idx_]);
    if (sharp && !blocked_) driving_ = false;
  }
  if (idx_ + 1 >= n_ && dist_ < arrive) {  // the last point: stop there
    phase_ = STOPPING;
    stop_since_ = 0;
    blocked_ = latched_ = backing_ = false;
    state_ = NAV_DRIVING;
    obst_ = front_;
    return;
  }
  // steer on the pursuit point (the route up to the next sharp corner / the last point), else the waypoint
  uint8_t last = idx_;
  while (last + 1 < n_ && last < idx_ + kPursueAhead && cornerDeg(last, in.heading) <= kSharpDeg) last++;
  // (lookahead >= 3 x the distance to the route: the point is always on it, <= 20 deg off it; and up to half the
  // way to the waypoint, growing no faster than the robot drives: on a long leg it aims far ahead, as at the
  // waypoint before #103, a turn command does not wind up while the wheels slip)
  const float dt = last_ms_ && in.now_ms - last_ms_ < 100 ? (in.now_ms - last_ms_) * 0.001f : 0.1f;
  look_far_ = fminf(kLookFar * dist_, look_far_ + fabsf(last_v_) * dt);
  float ax = tx_, ay = ty_;
  pursuit(in, fmaxf(fmaxf(L, look_far_), 3.0f * routeOff(in)), last, &ax, &ay);
  const float bearing = atan2f(ay - in.y, ax - in.x) / kDeg;
  const float e = wrap180(bearing - in.heading);
  // close in to a stop / a sharp corner, the bearing is noisy: don't steer
  const bool far = last > idx_ || dist_ > 2.0f * arrive;
  const float align = P(P_NAV_ALIGN_DEG);
  if (!blocked_) {
    if (!driving_ && fabsf(e) < align) {
      driving_ = true;
      away_ = away_done_ = false;
    } else if (driving_ && far && fabsf(e) > fmaxf(3.0f * align, kSharpDeg)) {
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
        contactWatch(SRC_AWAY, va != 0, false, false);  // #111: blocked that way too?
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
  // corners ahead slow it down (#103); the speed rises at most nav.speed_kp x vmax per s (a balancing robot
  // leans to speed up: no steps, e.g. after a corner)
  const float kp = P(P_NAV_SPEED_KP);
  // (a corner to pass: its cap at the pass radius L; a stop / sharp corner: at the point)
  const float to_go = last > idx_ ? fmaxf(0.0f, dist_ - L) : dist_;
  float v = fminf(cornerCap(in.heading, vmax) + kp * to_go, vmax) * fmaxf(0.0f, cosf(e * kDeg));
  v = fminf(fmaxf(v, P(P_NAV_MIN_SPEED)), vmax);
  const float v_rise = fmaxf(last_v_, P(P_NAV_MIN_SPEED)) + kp * vmax * dt;
  const bool rising = v > v_rise;  // (#111: the stall rule waits while the rise limit holds the command)
  // #104: ramping up from a stop (a start, after turning in place, after BLOCKED) - the rise limit holds the
  // command low for a while (~1.4 s to 250 mm/s) and a balancing robot still rocks from the stop
  if (rising) {
    if (last_v_ <= P(P_NAV_MIN_SPEED) + 0.5f) launch_ = true;
  } else {
    launch_ = false;
  }
  v = fminf(v, v_rise);
  // #104: cruising = not slowing down for a stop (the last point, a sharp corner that turns in place) and not
  // ramping up from one: the stuck contact rule counts then. Pass-through corner slow-downs (#103) and the ramp
  // back up after them count as cruising (their caps are >= ~88 mm/s, the robot keeps rolling through them).
  const bool cruise = cornerCap(in.heading, vmax, true) + kp * to_go >= vmax && !launch_;
  float vl = limitSpeed(v, vmax, true);
  // contact (#104): GOTO waits kContactWaitMs for a new list (the planner's route round the mark), then tries again
  if (contact_ && in.mode == MODE_GOTO && !backing_ && !gave_up_ && in.now_ms - contact_ms_ >= kContactWaitMs)
    latched_ = contact_ = false;
  if (gave_up_) latched_ = true;  // (#111) waits for a new list
  // (#111) got well past the spot it kept stopping at: count again
  if (stop_n_ && !blocked_ && hypotf(in.x - stop_x_, in.y - stop_y_) > 2.0f * kSameSpotMm) stop_n_ = 0;
  if (latched_ || stuck_) obst_flag_ = true;  // EXPLORE: stays blocked until a new list; stuck: backing off
  if (obst_flag_) {
    if (!blocked_) startBlocked(in);
    // closer than the lidar can see (its min range, #99): back off once so it shows again (or is seen to be gone)
    if (!backing_ && !dead_back_ && front_ > 0 && front_ < kDeadFront && P(P_NAV_STUCK_BACK) > 0 &&
        P(P_RNG_OBSTACLE_MM) > 0 && (lidarFresh(in.now_ms) || ghost_n_)) {
      dead_back_ = backing_ = true;
      back_mm_ = P(P_NAV_STUCK_BACK);
      back_x_ = in.x;
      back_y_ = in.y;
      back_ms_ = in.now_ms;
      blk_h_ = in.heading;
      log("nav: obstacle %.0f mm ahead, too close for the lidar: backing off", (double)front_);
    }
    const float obst = obst_;
    vl = 0;
    if (backing_) {
      // backed off = travel backwards along the heading it blocked at (a robot still rolling on, or wheels still
      // spinning forward after a lidar stuck, #79, is not backing off)
      const float back = -((in.x - back_x_) * cosf(blk_h_ * kDeg) + (in.y - back_y_) * sinf(blk_h_ * kDeg));
      // stuck: twice as fast (a balancing robot pressed against a wall barely follows the slowest speed)
      float vb = -fmaxf(P(P_NAV_MIN_SPEED), 30.0f) * (stuck_ ? 2.0f : 1.0f);
      vb = limitSpeed(vb, vmax, true);
      if (obst_flag_ || back >= back_mm_ || in.now_ms - back_ms_ > 5000) {
        if (back >= back_mm_) dead_back_ = false;  // backed off all the way: again if it is still that close (#99)
        backing_ = false;
        vb = 0;
        contact_hx_ = in.x;  // (#104) where it waits
        contact_hy_ = in.y;
      }
      vl = vb;
      obst_ = obst;  // report the obstacle ahead
      obst_flag_ = true;
      contactWatch(SRC_BACK, vb != 0, false, false);  // #111: a blocked back-off is a contact too (rear)
    } else if (contact_) {
      // contact (#104), backed off: hold that spot while it waits (with the speed only at 0 the velocity loop's wind-up
      // from pushing against it rolls a balancing robot straight back into it)
      vl = holdSpeed(in, contact_hx_, contact_hy_);
      obst_ = obst;
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
  contactWatch(SRC_DRIVE, speed_ != 0, cruise, rising);  // #104: driving - watch for contacts from the next step
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
