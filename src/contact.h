// Contact detection (issue #104): obstacles below the LD19's scan plane (~208 mm: shoes, toys, a box, a step) are
// invisible to the lidar, so the robot finds them by touch. Pure logic (time is passed in): the navigator feeds it every
// control step; test/host/nav_sim.cpp tests it and station/sim/contact.py is a line-by-line port.
//
// Both rules count only while it is watching (GOTO / EXPLORE driving forward, or STATION commanding motion; never in
// HOLD, BALANCE, MANUAL, or turning in place). d = the sign of the controller's ramped speed command, c = |command|.
//   BUMP   candidate: c >= kBumpMinCmd and steady (>= kBumpSteady x its peak in the last kBins x kBinMs), the speed
//          was >= kBumpNear x c in that time and fell by >= max(kBumpDropMm, kBumpDropFrac x that) to below
//          kBumpNear x c, with a pitch-rate peak >= kBumpRate in the direction of travel in the same time (a wheel that
//          stops throws the body on: ~0.4-0.5 deg/s per mm/s; normal balancing is ~20 deg/s RMS).
//          confirmed: the speed stays below kBumpHold x c for kBumpHoldMs within kBumpWindowMs (stopped, or bounced
//          back). Dropped when it comes back (>= kBumpBack x c: a door strip it rolled over) or the command goes.
//   STUCK  pushing but not moving: c >= kStuckMinCmd while cruising (not slowing down for the waypoint: there the
//          velocity loop lags and a balancing robot can stand still a moment with the command still up), and kStuckStartMs
//          after the command came it got going toward it (>= min(kStuckMovedFrac x c, kStuckMovedMm): a balancing robot
//          leans in and rolls back first when it starts); then no new ground (kStuckNewMm past the furthest point so far,
//          in d) for kStuckMs while the velocity loop's lean pushes on (>= kStuckLeanDeg in d): wheels stalled against
//          it, or rocking against it. A lidar stuck event waiting (EVT_LINK_STUCK, #79) confirms it at half the time
//          (LINK_CONTACT_LIDAR).
//   A fall right after (upright lost within kFallMs of a bump candidate, or half the stuck time without new ground)
//   confirms it too (LINK_CONTACT_FELL): a hard bump throws a balancing robot over before the hold time is up.
// Speed = the encoders' (80 ms window), lean = vel_out (deg).
#pragma once
#include <stdint.h>

struct ContactIn {
  uint32_t now_ms = 0;
  bool watch = false;        // driving under nav control (or STATION moving): detections count
  bool upright = false;      // balancing
  float cmd = 0;             // the controller's ramped speed setpoint, mm/s
  float speed = 0;           // measured forward speed, mm/s (encoders, 80 ms window)
  float pitch_rate = 0;      // deg/s, + = pitching forward
  float vel_out = 0;         // velocity loop output (lean asked for), deg, + = forward
  bool lidar_stuck = false;  // an EVT_LINK_STUCK is waiting (#79)
  bool cruise = true;        // the command is not being cut back for the waypoint ahead (STUCK only counts then)
};

class ContactDetector {
 public:
  // thresholds (compile-time, no params: docs/range-nav.md "Contacts")
  static constexpr uint32_t kBinMs = 10, kBins = 12;  // bump history: 12 x 10 ms
  static constexpr float kBumpMinCmd = 100.0f;        // mm/s
  static constexpr float kBumpSteady = 0.8f;
  static constexpr float kBumpNear = 0.6f;            // was >= 0.6 c, is < 0.6 c
  static constexpr float kBumpDropMm = 70.0f, kBumpDropFrac = 0.4f;
  static constexpr float kBumpRate = 80.0f;           // deg/s (balancing ~20 RMS, peaks ~65; hard bumps 100-126)
  static constexpr float kBumpHold = 0.5f, kBumpBack = 0.7f;
  static constexpr uint32_t kBumpHoldMs = 80, kBumpWindowMs = 300;
  static constexpr float kStuckMinCmd = 100.0f;       // mm/s
  static constexpr float kStuckMovedFrac = 0.5f, kStuckMovedMm = 50.0f;
  static constexpr float kStuckNewMm = 10.0f;
  static constexpr uint32_t kStuckMs = 400, kStuckStartMs = 500;
  static constexpr float kStuckLeanDeg = 0.6f;        // (pushing: 1-2 deg in the sim; a slow-down: < 0.45)
  static constexpr uint32_t kFallMs = 600;

  void reset();
  // Every control step. Returns LINK_CONTACT_BUMP / LINK_CONTACT_STUCK when a contact is detected (then it starts over),
  // else 0. dir() / value() / flags() describe that detection.
  uint8_t update(const ContactIn& in);
  float dir() const { return dir_; }          // +1 forward, -1 reversing
  float value() const { return value_; }      // bump: peak pitch rate deg/s; stuck: ms without new ground
  uint8_t flags() const { return flags_; }    // LINK_CONTACT_LIDAR / LINK_CONTACT_FELL
  bool bumpPending() const { return bump_ms_ != 0; }

 private:
  uint8_t fire(uint8_t kind, float d, float value, uint8_t flags);
  // 10 ms bins (newest at bhead_ - 1): speed and command at the bin's end, pitch-rate max / min inside it
  float bs_[kBins] = {}, bc_[kBins] = {}, bmax_[kBins] = {}, bmin_[kBins] = {};
  uint8_t bn_ = 0, bhead_ = 0;
  uint32_t bin_ms_ = 0;
  float cur_max_ = 0, cur_min_ = 0;
  bool have_ = false;
  uint32_t last_ms_ = 0;
  bool was_upright_ = false;
  // bump
  uint32_t bump_ms_ = 0;           // candidate since (0 = none)
  uint32_t cand_ms_ = 0;           // last candidate not disproved (fall check)
  float bump_peak_ = 0, cand_d_ = 1;
  uint32_t hold_ms_ = 0;
  // stuck
  bool run_ = false;               // the command is >= kStuckMinCmd (direction run_d_)
  float run_d_ = 1;
  bool moved_ = false;             // ... and it got going (>= kStuckMovedFrac x c) since
  float pos_ = 0, high_ = 0;       // integrated speed along run_d_, the furthest point (mm)
  uint32_t new_ms_ = 0;            // when it last got kStuckNewMm past high_
  uint32_t run_ms_ = 0;            // when the command came (run_)
  // last detection
  float dir_ = 1, value_ = 0;
  uint8_t flags_ = 0;
};
