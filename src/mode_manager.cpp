#include "mode_manager.h"

#include <math.h>

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static bool navMode(RobotMode m) { return m == MODE_GOTO || m == MODE_EXPLORE || m == MODE_HOLD; }

void ModeManager::onRadioPacket(const ControlPacket& p, uint32_t now_ms) {
  // first packet, or first after the link was lost: the remote's current state counts as new
  const bool fresh_link = !radioPresent(now_ms);
  const bool first_link = !have_radio_;
  have_radio_ = true;
  radio_ms_ = now_ms;
  pkt_ = p;

  // E-stop latches until a packet without it; afterwards arming needs ARMED to go 0 -> 1.
  // The link E-stop (station) keeps it disarmed too; its arm edge only counts after the release.
  if (p.flags & CTRL_FLAG_ESTOP) {
    estop_ = true;
    armed_ = false;
    need_arm_edge_ = true;
  } else {
    estop_ = false;
    const bool a = (p.flags & CTRL_FLAG_ARMED) != 0;
    if (!a && !estop_link_) need_arm_edge_ = false;
    armed_ = a && !need_arm_edge_ && !estop_link_;
  }

  // Mode requests act only on change, so a Pi SET_MODE is not overwritten by the remote's
  // unchanged selection - not 50 times a second, and not when the radio reconnects after a
  // dropout (that dropped a balancing robot to IDLE: it fell over).
  // The first packet after boot only teaches last_req_: the robot stays IDLE until the remote's
  // selection changes (or the Pi sends SET_MODE), so arming after a reboot never starts the
  // motors in whatever mode the remote still shows (issue #21; the remote shows robot>requested).
  // While the link E-stop is latched a selection change is ignored (learned only).
  if (!first_link && !estop_link_ && p.requested_mode != last_req_ && p.requested_mode < MODE_COUNT) {
    requested_ = (RobotMode)p.requested_mode;
  }
  last_req_ = p.requested_mode;

  // Events: act once per new event_seq. On (re)connect just learn the seq (don't replay).
  if (fresh_link) {
    last_event_seq_ = p.event_seq;
  } else if (p.event_seq != last_event_seq_) {
    last_event_seq_ = p.event_seq;
    if (p.event != EVT_NONE && p.event < EVT_COUNT) pushEvent(p.event);
  }
}

void ModeManager::onPiHeartbeat(uint32_t now_ms, uint8_t clients) {
  have_pi_ = true;
  pi_ms_ = now_ms;
  pi_clients_ = clients;
}

SetModeResult ModeManager::onPiSetMode(uint8_t m, uint32_t now_ms) {
  if (m >= MODE_COUNT) return SETMODE_BAD;
  if (estop_link_) return SETMODE_ESTOP;  // latched: no request is stored either
  const bool arm_req = arm_req_ && now_ms - arm_req_ms_ <= LINK_ARM_WINDOW_MS;
  arm_req_ = false;  // one-shot
  if (!radioPresent(now_ms)) {
    // With no remote around the Pi is the arm switch: IDLE disarms; arming needs the explicit
    // arm request just before (EVT_LINK_ARM), a plain SET_MODE only changes an armed robot's mode.
    if (m == MODE_IDLE) {
      armed_ = false;
    } else if (!armed_) {
      if (!arm_req || estop_) return SETMODE_NOT_ARMED;
      armed_ = true;
      need_arm_edge_ = false;
    }
  }
  // With the remote present its ARMED/ESTOP flags rule; the request is stored.
  requested_ = (RobotMode)m;
  // A station mode request is the explicit handover of a nav mode the remote supervised (issue #38).
  handover_ = remote_sup_ = false;
  return SETMODE_OK;
}

void ModeManager::onLinkEstop() {
  estop_link_ = true;
  armed_ = false;
  need_arm_edge_ = true;
  requested_ = MODE_IDLE;
  arm_req_ = false;
}

void ModeManager::onLinkEstopRelease() {
  if (!estop_link_) return;
  estop_link_ = false;
  // still disarmed + IDLE: remote present -> ARMED off -> on and a selection change;
  // no remote -> an arm request + SET_MODE (the next one after this release)
  armed_ = false;
  need_arm_edge_ = true;
  requested_ = MODE_IDLE;
  arm_req_ = false;
}

void ModeManager::onLinkArm(uint32_t now_ms) {
  if (estop_link_ || estop_) return;
  arm_req_ = true;
  arm_req_ms_ = now_ms;
}

void ModeManager::onPiCmdVel(float speed_mms, float turn_dps, uint32_t now_ms) {
  if (isnan(speed_mms) || isnan(turn_dps) || isinf(speed_mms) || isinf(turn_dps)) return;
  cmd_speed_ = speed_mms;
  cmd_turn_ = turn_dps;
  cmd_ms_ = now_ms;
  cmd_valid_ = true;
}

void ModeManager::setAutoCmd(float speed_mms, float turn_dps, uint32_t now_ms) {
  if (isnan(speed_mms) || isnan(turn_dps) || isinf(speed_mms) || isinf(turn_dps)) {
    auto_valid_ = false;
    return;
  }
  auto_speed_ = speed_mms;
  auto_turn_ = turn_dps;
  auto_ms_ = now_ms;
  auto_valid_ = true;
}

void ModeManager::requestMode(uint8_t m) {
  if (m < MODE_COUNT && !estop_link_) requested_ = (RobotMode)m;
}

float ModeManager::speedScale(uint32_t now_ms) const {
  if (!radioPresent(now_ms)) return 1.0f;
  return (pkt_.speed_limit > 100 ? 100 : pkt_.speed_limit) / 100.0f;
}

void ModeManager::update(uint32_t now_ms, bool imu_ok, const ModeLimits& lim) {
  const RobotMode eff = (estop_ || estop_link_ || !armed_ || !imu_ok) ? MODE_IDLE : requested_;
  if (eff != mode_) {
    mode_ = eff;
    mode_changed_ = true;
    cmd_valid_ = false;  // a mode change never inherits an old command
    auto_valid_ = false;
  }
  if (arm_req_ && now_ms - arm_req_ms_ > LINK_ARM_WINDOW_MS) arm_req_ = false;

  // Supervision handover (issue #38): once the remote was live while armed, losing it (no packet for
  // RADIO_LOST_MS) pauses GOTO / EXPLORE / HOLD until the remote is back or the station sends a
  // fresh SET_MODE. The Pi heartbeat alone never takes over silently. Disarming forgets it.
  if (!armed_) {
    remote_sup_ = handover_ = false;
  } else if (radioLive(now_ms)) {
    remote_sup_ = true;
    handover_ = false;
  } else if (remote_sup_ && !radioPresent(now_ms)) {
    handover_ = true;
  }

  float s = 0, t = 0;
  if (mode_ == MODE_MANUAL && radioLive(now_ms)) {
    const float drive = clampf(pkt_.drive, 1000.0f) / 1000.0f;
    const float turn = clampf(pkt_.turn, 1000.0f) / 1000.0f;
    const float limit = (pkt_.speed_limit > 100 ? 100 : pkt_.speed_limit) / 100.0f;
    s = drive * lim.max_speed * limit;
    t = turn * lim.max_turn;
  } else if (mode_ == MODE_STATION && piLive(now_ms) && cmd_valid_ &&
             now_ms - cmd_ms_ <= LINK_CMD_VEL_TIMEOUT_MS) {
    s = clampf(cmd_speed_, lim.max_speed);
    t = clampf(cmd_turn_, lim.max_turn);
  } else if (navMode(mode_) && auto_valid_ && now_ms - auto_ms_ <= 100 && supervised(now_ms)) {
    s = clampf(auto_speed_, lim.max_speed * speedScale(now_ms));
    t = clampf(auto_turn_, lim.max_turn);
  }
  if (fabsf(t) < lim.turn_deadband) t = 0;
  speed_ = s;
  turn_ = t;
}

void ModeManager::pushEvent(uint8_t ev) {
  if (ev_n_ >= sizeof(ev_q_)) return;  // full: drop
  ev_q_[(ev_head_ + ev_n_) % sizeof(ev_q_)] = ev;
  ev_n_++;
}

bool ModeManager::takeEvent(uint8_t& ev) {
  if (!ev_n_) return false;
  ev = ev_q_[ev_head_];
  ev_head_ = (ev_head_ + 1) % sizeof(ev_q_);
  ev_n_--;
  return true;
}
