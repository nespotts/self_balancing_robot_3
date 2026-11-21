// Who is in charge: arming, E-stop, mode requests from the remote and the Pi, failsafes, and
// the commanded speed/turn for the current mode. Pure logic (time is passed in).
//
//   ESTOP flag (latched until a packet without it) -> IDLE, and arming needs a fresh ARMED edge
//   not armed / IMU unhealthy                      -> IDLE (motors off)
//   MANUAL:  remote sticks, zero if no packet for RADIO_FAILSAFE_MS
//   STATION: MSG_CMD_VEL, zero if heartbeat lost or cmd older than LINK_CMD_VEL_TIMEOUT_MS
//   BALANCE / GOTO / EXPLORE / HOLD: zero motion (navigation is issue #8)
//   any change of the effective mode -> commanded motion zeroed, stored cmd_vel dropped
//   mode requests act on change only; after boot the remote's current selection is just learned,
//   so the robot stays IDLE until that selection changes (or the Pi sends SET_MODE)
#pragma once
#include <stdint.h>

#include "protocol/link_protocol.h"
#include "protocol/radio_protocol.h"

struct ModeLimits {
  float max_speed = 450, max_turn = 180, turn_deadband = 2;
};

class ModeManager {
 public:
  // Remote: a ControlPacket that already passed the version/size check.
  void onRadioPacket(const ControlPacket& p, uint32_t now_ms);
  void onPiHeartbeat(uint32_t now_ms);
  void onPiSetMode(uint8_t mode, uint32_t now_ms);
  void onPiCmdVel(float speed_mms, float turn_dps, uint32_t now_ms);
  void update(uint32_t now_ms, bool imu_ok, const ModeLimits& lim);

  RobotMode mode() const { return mode_; }
  RobotMode requested() const { return requested_; }
  bool motorsAllowed() const { return mode_ != MODE_IDLE; }
  bool armed() const { return armed_; }
  bool estop() const { return estop_; }
  bool radioLive(uint32_t now_ms) const { return have_radio_ && now_ms - radio_ms_ < RADIO_FAILSAFE_MS; }
  bool radioPresent(uint32_t now_ms) const { return have_radio_ && now_ms - radio_ms_ < RADIO_LOST_MS; }
  bool piLive(uint32_t now_ms) const { return have_pi_ && now_ms - pi_ms_ < LINK_HEARTBEAT_TIMEOUT_MS; }
  float targetSpeed() const { return speed_; }
  float targetTurn() const { return turn_; }
  const ControlPacket& lastPacket() const { return pkt_; }

  bool takeModeChanged() {
    bool c = mode_changed_;
    mode_changed_ = false;
    return c;
  }
  // Deduped one-shot events from the remote (RadioEvent values).
  bool takeEvent(uint8_t& ev);

 private:
  void pushEvent(uint8_t ev);
  ControlPacket pkt_ = {};
  bool have_radio_ = false, have_pi_ = false;
  uint32_t radio_ms_ = 0, pi_ms_ = 0;
  bool armed_ = false, estop_ = false, need_arm_edge_ = true;
  uint8_t last_req_ = MODE_IDLE, last_event_seq_ = 0;
  RobotMode requested_ = MODE_IDLE, mode_ = MODE_IDLE;
  bool mode_changed_ = false;
  bool cmd_valid_ = false;
  float cmd_speed_ = 0, cmd_turn_ = 0;
  uint32_t cmd_ms_ = 0;
  float speed_ = 0, turn_ = 0;
  uint8_t ev_q_[4] = {};
  uint8_t ev_head_ = 0, ev_n_ = 0;
};
