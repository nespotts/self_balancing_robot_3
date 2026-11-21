// Who is in charge: arming, E-stop, mode requests from the remote and the Pi, failsafes, and
// the commanded speed/turn for the current mode. Pure logic (time is passed in).
//
//   ESTOP flag (latched until a packet without it) -> IDLE, and arming needs a fresh ARMED edge
//   link E-stop (EVT_LINK_ESTOP, issue #41)       -> IDLE + disarmed, latched until EVT_LINK_ESTOP_RELEASE;
//                                                    while latched every mode request is ignored; after the
//                                                    release the normal arming rules below apply again
//   not armed / IMU unhealthy                      -> IDLE (motors off)
//   no remote present: arming needs an explicit arm request (EVT_LINK_ARM) followed by SET_MODE != IDLE
//            within LINK_ARM_WINDOW_MS; a plain SET_MODE only changes the mode of an armed robot;
//            SET_MODE IDLE disarms
//   MANUAL:  remote sticks, zero if no packet for RADIO_FAILSAFE_MS
//   STATION: MSG_CMD_VEL, zero if heartbeat lost or cmd older than LINK_CMD_VEL_TIMEOUT_MS
//   GOTO / EXPLORE / HOLD: the navigator's command (setAutoCmd), zero if it is older than 100 ms or
//            nobody supervises (see supervised())
//   BALANCE: zero motion
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

// Result of a Pi SET_MODE (for a log line in main.cpp).
enum SetModeResult : uint8_t {
  SETMODE_OK = 0,
  SETMODE_BAD,          // mode out of range
  SETMODE_ESTOP,        // ignored: link E-stop latched
  SETMODE_NOT_ARMED,    // ignored: no remote, not armed, no fresh arm request
};

class ModeManager {
 public:
  // Remote: a ControlPacket that already passed the version/size check.
  void onRadioPacket(const ControlPacket& p, uint32_t now_ms);
  // clients = HeartbeatFrame.clients: browsers / app connected to the bridge (0 = nobody watching).
  void onPiHeartbeat(uint32_t now_ms, uint8_t clients);
  SetModeResult onPiSetMode(uint8_t mode, uint32_t now_ms);
  void onPiCmdVel(float speed_mms, float turn_dps, uint32_t now_ms);
  // Link-only events (link_protocol.h LinkEvent).
  void onLinkEstop();
  void onLinkEstopRelease();
  void onLinkArm(uint32_t now_ms);
  // Navigator output for GOTO / EXPLORE / HOLD (src/navigator.h), set every control step.
  void setAutoCmd(float speed_mms, float turn_dps, uint32_t now_ms);
  // Mode request from an event (EVT_GO_HOME): like a remote selection change, never arms.
  void requestMode(uint8_t mode);
  void update(uint32_t now_ms, bool imu_ok, const ModeLimits& lim);

  RobotMode mode() const { return mode_; }
  RobotMode requested() const { return requested_; }
  bool motorsAllowed() const { return mode_ != MODE_IDLE; }
  bool armed() const { return armed_; }
  bool estop() const { return estop_; }
  bool linkEstop() const { return estop_link_; }
  bool armWait() const { return need_arm_edge_; }  // the remote's ARMED switch must go off -> on
  // GOTO / EXPLORE / HOLD paused after the remote was lost (issue #38): needs the remote back, or a
  // fresh SET_MODE from the station (explicit handover).
  bool handoverWait() const { return handover_ && (mode_ == MODE_GOTO || mode_ == MODE_EXPLORE || mode_ == MODE_HOLD); }
  // Someone can stop it (autonomous modes need this):
  //   the remote live (packets < RADIO_FAILSAFE_MS), or
  //   no remote present, the Pi heartbeat live AND a station client connected (clients > 0), and no
  //   pending handover (a nav mode that the remote supervised never silently passes to the Pi).
  bool supervised(uint32_t now_ms) const {
    return radioLive(now_ms) || (!radioPresent(now_ms) && piWatching(now_ms) && !handover_);
  }
  // Remote speed limit (0..1) while the remote is present, else 1.
  float speedScale(uint32_t now_ms) const;
  bool radioLive(uint32_t now_ms) const { return have_radio_ && now_ms - radio_ms_ < RADIO_FAILSAFE_MS; }
  bool radioPresent(uint32_t now_ms) const { return have_radio_ && now_ms - radio_ms_ < RADIO_LOST_MS; }
  bool piLive(uint32_t now_ms) const { return have_pi_ && now_ms - pi_ms_ < LINK_HEARTBEAT_TIMEOUT_MS; }
  bool piWatching(uint32_t now_ms) const { return piLive(now_ms) && pi_clients_ > 0; }
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
  uint8_t pi_clients_ = 0;
  bool armed_ = false, estop_ = false, need_arm_edge_ = true;
  bool estop_link_ = false;
  bool arm_req_ = false;
  uint32_t arm_req_ms_ = 0;
  bool remote_sup_ = false, handover_ = false;  // armed: the remote was live / it was lost since
  uint8_t last_req_ = MODE_IDLE, last_event_seq_ = 0;
  RobotMode requested_ = MODE_IDLE, mode_ = MODE_IDLE;
  bool mode_changed_ = false;
  bool cmd_valid_ = false;
  float cmd_speed_ = 0, cmd_turn_ = 0;
  uint32_t cmd_ms_ = 0;
  bool auto_valid_ = false;
  float auto_speed_ = 0, auto_turn_ = 0;
  uint32_t auto_ms_ = 0;
  float speed_ = 0, turn_ = 0;
  uint8_t ev_q_[4] = {};
  uint8_t ev_head_ = 0, ev_n_ = 0;
};
