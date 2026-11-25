// Navigation (issue #8): waypoint following for MODE_GOTO / MODE_EXPLORE, position hold for MODE_HOLD,
// and the obstacle check from the range sweep or the LD19 lidar. Pure logic (time and pose are passed in), so
// it runs on a host test (test/host/nav_sim.cpp) and station/sim/sim.py mirrors it line by line.
//
// GOTO / EXPLORE, per waypoint (list from EVT_MARK_WAYPOINT or MSG_GOTO, pose frame):
//   TURNING  turn in place (nav.turn_kp x bearing error) until |error| < nav.align_deg
//   DRIVING  steer (nav.turn_kp x bearing error) on a pure-pursuit point: the point of the route (the leg before
//            after a pass, leg start -> waypoints ahead, up to the next sharp corner or the last point) L from the
//            robot, furthest along; L = max(nav.arrive_mm, 1 s x speed, 3 x the distance to the route, up to half
//            the distance to the waypoint (growing no faster than it drives)) (none: the waypoint itself). Back to TURNING if |error| > max(3 x nav.align_deg, 75 deg).
//            Speed = (corner cap + nav.speed_kp x distance to go) x cos(error), >= nav.min_speed,
//            <= nav.max_speed, rising at most nav.speed_kp x max speed per s (no steps).
//   Intermediate waypoints are passed, not stopped at (#103): next one when closer than L (sharp corner:
//            nav.arrive_mm) or past the waypoint along its leg. Corner cap: the speed at which the pursuit cuts
//            the corner by ~30 mm (30 / tan(turn / 4) mm/s: 228 at 30 deg, 112 at 60, 88 at 75, >= nav.min_speed),
//            each corner ahead reached with at most nav.speed_kp x the distance to spare; distance to go = to
//            the pass radius L. Sharper than 75 deg: cap 0 (slow down as for a stop), turn in place there.
//   Last point (the only stop; the planner's stop-and-scan points are the last of their list): within
//            nav.arrive_mm speed 0; stopped (|speed| < nav.stop_speed, |yaw| < nav.stop_turn_dps) for
//            nav.dwell_ms -> ARRIVED (pushed out past 2 x arrive_mm -> seek again); within 2 x arrive_mm no
//            steering. nav.end_heading_deg >= 0 -> TURNING to it (nav.heading_tol_deg) first.
//   BLOCKED  obstacle < rng.obstacle_mm in the direction of travel: stop (and back off nav.backoff_mm).
//            GOTO resumes after nav.obst_age_ms of FRESH data (a lidar frame < 0.5 s old, or sweep readings
//            < 0.5 s old) with nothing closer than rng.obstacle_mm + nav.clear_mm (+ nav.backoff_mm) in the
//            corridor (#99): no data, stale data or blind is never "clear". EXPLORE waits for a new list.
//            A new / changed list always resumes (the per-direction latch stays set without fresh data).
//            Obstacle closer than kDeadFront (141 mm, the lidar's min range): back off nav.stuck_backoff_mm
//            once, so it shows again (or is seen to be gone).
// HOLD: hold point = pose when HOLD starts; speed = nav.hold_kp x forward error (deadband nav.hold_db_mm,
//   max nav.hold_max_speed); heading is held by the controller (turn 0).
// Obstacles: valid range readings become world points (kept nav.obst_age_ms); points inside a corridor
//   nav.corridor_mm wide ahead / behind give front / rear distances, as a sensor range. Readings that hit
//   lower than nav.floor_mm (sensor tipped down by the lean) are ignored. Slow down linearly from
//   nav.slow_mm to rng.obstacle_mm. STATION commands get the same limit (main.cpp); MANUAL only the flag.
// LD19 lidar (#77, MSG_LIDAR_OBSTACLES from the bridge, ~10 Hz): one return per 10 deg sector of the newest
//   scan (the nearest one inside the corridor, else the nearest, #99), robot frame, placed in the pose frame on
//   arrival (dead-reckoned back by the frame's age_ms). While a frame came in the last LINK_LIDAR_FRESH_MS
//   (500 ms) these 36 points count TOGETHER with the sweep's recent points (the union, #99); corridor, side
//   walls, slow-down, BLOCKED, stuck all work on them the same way (same params, front / rear still "as a sensor
//   range" = x - 38.9 / 13.1 - x). The bridge already dropped tilted scans and floor hits (nav.floor_mm is not
//   used). Frames older than LINK_LIDAR_MAX_AGE_MS are dropped. Lidar stale: the sweep's points only (if it
//   runs), else blind (below).
//   Dead zone (#99): returns closer than the bridge's min_range_mm (120 mm default, 60 on the robot) never
//   arrive. Lidar points within kLidarDeadR (180 mm) of the axle are kept as "ghosts" until the robot is
//   kLidarDeadR away from them (then the lidar would see them again): an obstacle that gets that close does not
//   vanish. Ghosts count like lidar points (corridor, side walls, swing circle), also while the lidar is stale.
// Side walls (#62): recent points beside the body (gap body side -> wall < nav.side_min_mm, 0 = off) steer
//   away (nav.side_kp deg/s per mm inside it) and slow down (linear to nav.min_speed at gap 0). Turning in
//   place: a wall close at one side and the target behind -> turn the nose away from it; something inside the
//   corners' swing circle ahead / behind -> first step straight away from it (<= nav.stuck_backoff_mm).
// Stuck (#62): driving / turning but < nav.stuck_mm (or 15 deg) of progress for nav.stuck_ms -> BLOCKED,
//   log "nav: stuck ...", back off nav.stuck_backoff_mm. GOTO then goes on; EXPLORE waits for a new list.
//   Driving (not the lidar's): contactReset first (#111: stand up, or it can't back away from what it leans on). No
//   contact report: crawling at 40 mm/s the robot can stand in its motor dead band this long with nothing in front of
//   it. Again and again at one spot: the planner's same-spot rule marks it.
//   Lidar stuck (#79, EVT_LINK_STUCK): spinning wheels count as odometry progress, so the bridge's LD19 matcher
//   reports "wheels turn, robot still"; the next driving stuck check (within 500 ms, while trying to drive) takes it
//   the same way. Turning in place needs no help: the heading is the gyro's.
// Contacts (#104, src/contact.h): obstacles below the lidar's scan plane are found by touch. Driving in GOTO /
//   EXPLORE, backing off (BLOCKED, stuck, contact) or stepping away from a wall before a turn (#111), a BUMP (speed falls
//   far below the command with a pitch-rate spike), STUCK (no progress for 0.4 s while the command and the velocity
//   loop's lean push on) or STALL (#111: from 30 mm/s either way, the wheels still for 0.4 s while the lean builds) ->
//   BLOCKED, log "nav: contact bump|stuck at x,y (..., rear when reversing, backing off|holding)", MSG_CONTACT to the
//   bridge (contact point on the body's edge in the direction of travel, pose frame: it marks the spot and its planner
//   replans round it), and the controller's contactReset() (takeUnwind(): command 0, velocity integrator back to its
//   free value - it stands up instead of leaning on it). Found while driving: back off nav.stuck_backoff_mm (rear
//   corridor checked, as stuck). Found while backing off / stepping away (that way is blocked too): no back-off, hold
//   the spot (and no dead-zone back-off). GOTO waits kContactWaitMs for that new list, then tries again; EXPLORE waits
//   for a new list. STATION: reported only (someone drives). HOLD: never. nav.stuck_ms 0 turns these checks off too.
//   STUCK counts only while cruising: not slowing down for a stop (the last point, a sharp corner) and not ramping up
//   from one (the speed rise limit after a start / turn in place / BLOCKED); slowing for a pass-through corner (#103)
//   and the ramp back up after it are cruising. STALL waits while the command is held by its rise limit (any ramp).
// No fresh lidar frame and no range readings for 3 s (rng.enable 0, sensor fault): no obstacle stop is possible,
//   so EXPLORE pauses (stands still, keeps its state) and GOTO drives at most nav.blind_speed (0 = GOTO pauses
//   too). Without the sweep that is 500 ms after the last lidar frame (lidar silent / stopped, bridge gone).
// Paused too while !upright or !live (nobody supervises, see ModeManager::supervised).
#pragma once
#include <stdint.h>

#include "config.h"
#include "contact.h"
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
  // contact detection (#104): the balance loop's last step
  float pitch_rate = 0;               // deg/s, + = pitching forward
  float cmd_speed = 0;                // the controller's ramped speed setpoint, mm/s
  float vel_out = 0;                  // velocity loop lean, deg
  bool held = false;                  // the velocity loop held at its anti-windup limit (CtlOut.held, #111)
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
  // LD19 obstacles (#77): in = the pose now. false = dropped (wrong sector count, older than LINK_LIDAR_MAX_AGE_MS).
  bool addLidar(const LidarObstacleFrame& f, const NavIn& in);
  bool lidarFresh(uint32_t now_ms) const { return lidar_ms_ && now_ms - lidar_ms_ <= LINK_LIDAR_FRESH_MS; }
  void onModeChange(const NavIn& in);
  void onPoseReset();                 // pose is now (0,0,0): list, obstacle points and hold point dropped
                                      // (ghosts are moved into the new frame)
  void onLidarStuck(uint32_t now_ms) { lidar_stuck_ms_ = now_ms ? now_ms : 1; }  // EVT_LINK_STUCK (#79)
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
  float sideLeftMm() const { return gap_l_; }   // gap body side -> nearest point beside it, 0 = none
  float sideRightMm() const { return gap_r_; }
  bool stuck() const { return stuck_; }
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
  // Contact report (#104): MSG_CONTACT due (the newest one, if two came before it went out).
  bool contactDue() const { return contact_due_; }
  const ContactFrame& contactFrame() const { return contact_frame_; }
  void contactSent() { contact_due_ = false; }
  bool contact() const { return contact_; }   // BLOCKED after a contact (backing off / waiting)
  bool gaveUp() const { return gave_up_; }    // GOTO gave up on its waypoint (#111): BLOCKED until a new list
  // A contact it acts on (GOTO / EXPLORE, #111): the controller's contactReset() is due (once).
  bool takeUnwind() {
    const bool u = unwind_;
    unwind_ = false;
    return u;
  }

  static bool navMode(uint8_t m) { return m == MODE_GOTO || m == MODE_EXPLORE || m == MODE_HOLD; }

 private:
  enum Phase : uint8_t { SEEK, STOPPING, END_TURN, DONE };
  void updateObstacles(const NavIn& in);
  void updateGoto(const NavIn& in);
  void updateHold(const NavIn& in);
  float holdSpeed(const NavIn& in, float x, float y);  // nav.hold_* position loop on the forward error
  void startBlocked(const NavIn& in);
  void startStuck(const NavIn& in, bool turning);
  bool stuckCheck(const NavIn& in, bool turning, bool trying);  // true: no progress for nav.stuck_ms
  float sideTurn(float* inside) const;                           // nav.side_*: turn away from side walls
  void sideAvoid(float vmax, float tmax);                        // driving: steer away, slow down
  bool blind(uint32_t now_ms) const;  // no fresh lidar frame and no RangeFrame for kBlindMs
  bool fresh(uint32_t now_ms) const;  // fresh obstacle data: a lidar frame or a RangeFrame within 500 ms (#99)
  void logBlind(uint8_t mode);
  void setTarget(const NavIn& in, float x, float y);
  // pass-through waypoints (#103)
  void legStart(uint8_t i, float* x, float* y) const;  // where the leg to waypoint i starts
  float cornerDeg(uint8_t i, float heading) const;     // turn at waypoint i (i + 1 < n_), 0..180 deg
  float cornerCap(float heading, float vmax, bool stops_only = false) const;  // speed cap at the current waypoint
  bool pursuit(const NavIn& in, float L, uint8_t last, float* ax, float* ay) const;
  float routeOff(const NavIn& in) const;               // distance to this leg (and the one before, q_set_)
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void contactCheck(const NavIn& in);                       // #104: the detector, then the reaction
  void startContact(const NavIn& in, uint8_t kind);
  void noteStop(const NavIn& in);  // #111: the same spot again? (GOTO gives up after kGiveUpStops)
  void reportContact(const NavIn& in, uint8_t kind, uint8_t flags, bool hold, float dir, float value);
  void contactWatch(uint8_t src, bool on, bool cruise, bool ramp) {  // this step's motion, watched from the next step
    contact_watch_ = on;
    contact_src_ = on ? src : (uint8_t)SRC_NONE;
    contact_cruise_ = cruise;
    contact_ramp_ = ramp;
  }

  // contacts (#104)
  ContactDetector cdet_;
  enum Src : uint8_t { SRC_NONE, SRC_DRIVE, SRC_BACK, SRC_AWAY };
  bool contact_watch_ = false;        // the last step moved (GOTO / EXPLORE): watch for contacts
  uint8_t contact_src_ = SRC_NONE;    // ... driving, backing off, or stepping away from a wall before a turn (#111)
  bool contact_cruise_ = false;       // ... not slowing down for a stop (pass-through corners #103 count as cruising)
  bool contact_ramp_ = false;         // ... the command held by its rise limit (#111: the stall rule waits)
  bool launch_ = false;               // ramping up from a stop (the speed rise limit, #103): not cruising
  bool unwind_ = false;               // a contact it acts on: the controller's contactReset() due (#111)
  bool contact_ = false;              // BLOCKED by a contact; GOTO: waiting for a new list
  uint32_t contact_ms_ = 0;
  float contact_hx_ = 0, contact_hy_ = 0;  // where it backed off to (held while it waits)
  bool contact_due_ = false;
  ContactFrame contact_frame_ = {};
  uint8_t contact_seq_ = 0;
  uint32_t report_ms_ = 0;            // STATION: last report (one per kReportGapMs)
  static constexpr uint32_t kContactWaitMs = 4000, kReportGapMs = 3000;
  // #111: GOTO stopped at the same spot driving to the same waypoint this often (contacts / stucks while driving):
  // it gives up on it (BLOCKED until a new list)
  static constexpr uint8_t kGiveUpStops = 3;
  static constexpr float kSameSpotMm = 150.0f;
  uint8_t stop_n_ = 0, stop_idx_ = 0;
  float stop_x_ = 0, stop_y_ = 0;
  bool gave_up_ = false;
  static constexpr float kContactAhead = 50.0f;   // contact point: the body's front edge (45 mm) + 5, from the axle

  float wx_[WAYPOINT_MAX] = {}, wy_[WAYPOINT_MAX] = {};
  uint8_t n_ = 0, idx_ = 0;
  uint8_t mode_ = 0;
  Phase phase_ = SEEK;
  bool driving_ = false;              // SEEK: driving (else turning in place)
  float sx_ = 0, sy_ = 0;             // start of the leg to the current waypoint (#103)
  bool leg_set_ = false;              // false: the next update starts it at the pose
  float qx_ = 0, qy_ = 0;             // start of the leg before it, after a pass-through (pursuit, no jump)
  bool q_set_ = false;
  float last_v_ = 0;                  // speed command of the last update (rise limit)
  float look_far_ = 0;                // pursuit lookahead, long-leg part (grows as the robot drives)
  uint32_t last_ms_ = 0;
  bool blocked_ = false, latched_ = false;
  bool backing_ = false;
  float back_x_ = 0, back_y_ = 0;
  uint32_t back_ms_ = 0, stop_since_ = 0;
  float hold_x_ = 0, hold_y_ = 0;
  uint8_t state_ = NAV_IDLE;
  float speed_ = 0, turn_ = 0, tx_ = 0, ty_ = 0, dist_ = 0;
  float front_ = 0, rear_ = 0, obst_ = 0;
  float gap_l_ = 0, gap_r_ = 0;       // side gaps (0 = nothing beside)
  float near_mm_ = 0, near_xr_ = 0;   // closest point ahead / behind inside the turning swing circle
  float blk_h_ = 0;                   // heading when it blocked
  float back_mm_ = 0;                 // backing off this far (nav.backoff_mm or nav.stuck_backoff_mm)
  bool stuck_ = false, stuck_turn_ = false, stuck_seen_ = false;
  uint32_t stuck_ms_ = 0;             // start of the current no-progress window (0 = none)
  float stuck_x_ = 0, stuck_y_ = 0, stuck_h_ = 0;
  uint32_t lidar_stuck_ms_ = 0;       // EVT_LINK_STUCK arrival, waiting for a driving stuck check (0 = none), #79
  bool stuck_lidar_ = false;          // the stuck being started came from it (log line)
  static constexpr uint32_t kLidarStuckMs = 500;
  bool away_ = false, away_done_ = false;  // turning in place: stepping away from a close wall first
  float away_x_ = 0, away_y_ = 0;
  uint32_t away_ms_ = 0;
  bool obst_flag_ = false;
  bool blk_fwd_ = false, blk_rev_ = false;  // limitSpeed hysteresis per direction
  uint32_t clr_fwd_ms_ = 0, clr_rev_ms_ = 0;  // since when fresh data shows that direction clear (0 = not), #99
  float close_fwd_mm_ = 0, close_rev_mm_ = 0;  // last distance inside the clear distance (obstacle_mm)
  bool dead_back_ = false;            // backed off once from an obstacle inside kDeadFront (#99)
  float pose_x_ = 0, pose_y_ = 0, pose_h_ = 0;  // pose at the last update (ghosts across a pose reset)
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
  // LD19 points of the newest MSG_LIDAR_OBSTACLES frame (world = pose frame), #77
  float lx_[LINK_LIDAR_SECTORS] = {}, ly_[LINK_LIDAR_SECTORS] = {};
  uint8_t lidar_n_ = 0;
  uint32_t lidar_ms_ = 0;             // arrival of that frame (0 = none)
  bool lidar_was_ = false;            // lidarFresh() at the last update (for the log line)
  static constexpr uint32_t kSweepFreshMs = 500;
  // ghosts (#99): lidar points close enough to vanish into its min range (world = pose frame)
  static constexpr uint8_t kGhosts = 8;
  float gx_[kGhosts] = {}, gy_[kGhosts] = {};
  uint8_t ghost_n_ = 0;
  // status
  uint8_t sent_state_ = 0xFF, sent_idx_ = 0xFF, sent_n_ = 0xFF;
  uint32_t sent_ms_ = 0;
  // log queue
  char log_[4][72] = {};
  uint8_t log_head_ = 0, log_n_ = 0;
};
