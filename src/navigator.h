// Navigation (issue #8): waypoint following for MODE_GOTO / MODE_EXPLORE, position hold for MODE_HOLD,
// and the obstacle check from the range sweep. Pure logic (time and pose are passed in), so it runs on a
// host test (test/host/nav_sim.cpp) and station/sim/sim.py mirrors it line by line.
//
// GOTO / EXPLORE, per waypoint (list from EVT_MARK_WAYPOINT or MSG_GOTO, pose frame):
//   TURNING  turn in place (nav.turn_kp x bearing error) until |error| < nav.align_deg
//   DRIVING  speed = nav.speed_kp x distance (>= nav.min_speed, <= nav.max_speed), steer on the bearing;
//            back to TURNING if |error| > 3 x nav.align_deg
//            within nav.arrive_mm: speed 0; stopped (|speed| < nav.stop_speed, |yaw| < nav.stop_turn_dps)
//            for nav.dwell_ms -> next waypoint (pushed out past 2 x arrive_mm -> seek again)
//   last point: nav.end_heading_deg >= 0 -> TURNING to it (nav.heading_tol_deg), then ARRIVED
//   BLOCKED  obstacle < rng.obstacle_mm in the direction of travel: stop (and back off nav.backoff_mm).
//            GOTO resumes when it is > rng.obstacle_mm + nav.clear_mm; EXPLORE waits for a new list.
//            A new / changed list always resumes.
// HOLD: hold point = pose when HOLD starts; speed = nav.hold_kp x forward error (deadband nav.hold_db_mm,
//   max nav.hold_max_speed); heading is held by the controller (turn 0).
// Obstacles: valid range readings become world points (kept nav.obst_age_ms); points inside a corridor
//   nav.corridor_mm wide ahead / behind give front / rear distances, as a sensor range. Readings that hit
//   lower than nav.floor_mm (sensor tipped down by the lean) are ignored. Slow down linearly from
//   nav.slow_mm to rng.obstacle_mm. STATION commands get the same limit (main.cpp); MANUAL only the flag.
// No range readings for 3 s (rng.enable 0, sensor fault): no obstacle stop is possible, so EXPLORE pauses
//   (stands still, keeps its state) and GOTO drives at most nav.blind_speed (0 = GOTO pauses too).
// Paused too while !upright or !live (nobody supervises, see ModeManager::supervised).
#pragma once
#include <stdint.h>

#include "config.h"
#include "protocol/link_protocol.h"
#include "protocol/radio_protocol.h"

struct NavIn {
  uint32_t now_ms = 0;
  uint8_t mode = 0;                   // RobotMode in effect
  bool upright = false;               // controller balancing (else: pause, output 0)
  bool live = true;                   // a supervising link is live (remote, or the Pi with no remote)
  float x = 0, y = 0, heading = 0;    // pose: mm, deg (continuous, + = CCW)
  float speed = 0, yaw_rate = 0;      // measured mm/s, deg/s
  float max_speed = 450;              // drv.max_speed x remote speed limit
  float max_turn = 180;               // drv.max_turn
};

class Navigator {
 public:
  // ---- waypoint list (pose frame)
  void clearList();
  bool add(float x, float y);         // false if full
  void listChanged(bool replaced);    // after add()/clear: restart (replaced) / continue, un-block
  uint8_t count() const { return n_; }
  uint8_t index() const { return idx_; }
  float wpX(uint8_t i) const { return wx_[i < WAYPOINT_MAX ? i : 0]; }
  float wpY(uint8_t i) const { return wy_[i < WAYPOINT_MAX ? i : 0]; }

  // ---- inputs
  void addRange(const RangeFrame& f, uint32_t now_ms);
  void onModeChange(const NavIn& in);
  void onPoseReset();                 // pose is now (0,0,0): list, obstacle points and hold point dropped
  void update(const NavIn& in);       // every control step (or at >= 50 Hz)
  // Obstacle limit for a speed command not made here (STATION; MANUAL with apply = false: flag only).
  float limitSpeed(float v, float vmax, bool apply);

  // ---- outputs
  float speed() const { return speed_; }
  float turn() const { return turn_; }
  uint8_t state() const { return state_; }
  float targetX() const { return tx_; }
  float targetY() const { return ty_; }
  float distance() const { return dist_; }
  float frontMm() const { return front_; }   // 0 = nothing in the corridor
  float rearMm() const { return rear_; }
  float obstacleMm() const { return obst_; } // in the direction of travel (NavStatusFrame.obstacle_mm)
  bool obstacle() const { return obst_flag_; }  // TLM_OBSTACLE: blocking the current motion
  bool active() const { return state_ == NAV_TURNING || state_ == NAV_DRIVING || state_ == NAV_BLOCKED; }
  bool done() const { return state_ == NAV_ARRIVED; }

  // NavStatusFrame due: at once after a change of state / index / count, else 2 Hz in nav modes.
  bool statusDue(uint32_t now_ms) const;
  void statusSent(uint32_t now_ms);
  // One queued log line ("nav: ..."), or nullptr.
  const char* peekLog() const { return log_n_ ? log_[log_head_] : nullptr; }
  void popLog();

  static bool navMode(uint8_t m) { return m == MODE_GOTO || m == MODE_EXPLORE || m == MODE_HOLD; }

 private:
  enum Phase : uint8_t { SEEK, STOPPING, END_TURN, DONE };
  void updateObstacles(const NavIn& in);
  void updateGoto(const NavIn& in);
  void updateHold(const NavIn& in);
  float holdSpeed(const NavIn& in, float x, float y);  // nav.hold_* position loop on the forward error
  void startBlocked(const NavIn& in);
  bool blind(uint32_t now_ms) const;  // no RangeFrame for kBlindMs
  void logBlind(uint8_t mode);
  void setTarget(const NavIn& in, float x, float y);
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

  float wx_[WAYPOINT_MAX] = {}, wy_[WAYPOINT_MAX] = {};
  uint8_t n_ = 0, idx_ = 0;
  uint8_t mode_ = 0;
  Phase phase_ = SEEK;
  bool driving_ = false;              // SEEK: driving (else turning in place)
  bool blocked_ = false, latched_ = false;
  bool backing_ = false;
  float back_x_ = 0, back_y_ = 0;
  uint32_t back_ms_ = 0, stop_since_ = 0;
  float hold_x_ = 0, hold_y_ = 0;
  uint8_t state_ = NAV_IDLE;
  float speed_ = 0, turn_ = 0, tx_ = 0, ty_ = 0, dist_ = 0;
  float front_ = 0, rear_ = 0, obst_ = 0;
  bool obst_flag_ = false;
  bool blk_fwd_ = false, blk_rev_ = false;  // limitSpeed hysteresis per direction
  uint32_t close_fwd_ms_ = 0, close_rev_ms_ = 0;  // last time something was inside the clear distance
  float close_fwd_mm_ = 0, close_rev_mm_ = 0;
  uint32_t now_ms_ = 0;
  bool warned_no_range_ = false;
  // obstacle points (world = pose frame)
  static constexpr uint8_t kPts = 48;
  float px_[kPts] = {}, py_[kPts] = {};
  uint32_t pt_ms_[kPts] = {};
  uint8_t pt_head_ = 0, pt_n_ = 0;
  uint32_t last_range_ms_ = 0;
  uint32_t sensed_ms_ = 0;            // last RangeFrame of any kind (valid or not)
  static constexpr uint32_t kBlindMs = 3000;
  // status
  uint8_t sent_state_ = 0xFF, sent_idx_ = 0xFF, sent_n_ = 0xFF;
  uint32_t sent_ms_ = 0;
  // log queue
  char log_[4][72] = {};
  uint8_t log_head_ = 0, log_n_ = 0;
};
