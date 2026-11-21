#include "mode_manager.h"

#include <math.h>

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

void ModeManager::onRadioPacket(const ControlPacket& p, uint32_t now_ms) {
  // first packet, or first after the link was lost: the remote's current state counts as new
  const bool fresh_link = !radioPresent(now_ms);
  const bool first_link = !have_radio_;
  have_radio_ = true;
  radio_ms_ = now_ms;
  pkt_ = p;

  // E-stop latches until a packet without it; afterwards arming needs ARMED to go 0 -> 1.
  if (p.flags & CTRL_FLAG_ESTOP) {
    estop_ = true;
    armed_ = false;
    need_arm_edge_ = true;
  } else {
    estop_ = false;
    const bool a = (p.flags & CTRL_FLAG_ARMED) != 0;
    if (!a) need_arm_edge_ = false;
    armed_ = a && !need_arm_edge_;
  }

  // Mode requests act on change (or on the very first packet after boot), so a Pi SET_MODE is
  // not overwritten by the remote's unchanged selection - not 50 times a second, and not when
  // the radio reconnects after a dropout (that dropped a balancing robot to IDLE: it fell over).
  if ((first_link || p.requested_mode != last_req_) && p.requested_mode < MODE_COUNT) {
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

void ModeManager::onPiHeartbeat(uint32_t now_ms) {
  have_pi_ = true;
  pi_ms_ = now_ms;
}

void ModeManager::onPiSetMode(uint8_t m, uint32_t now_ms) {
  if (m >= MODE_COUNT) return;
  requested_ = (RobotMode)m;
  // With no remote around, the Pi's mode request is also the arm/disarm switch.
  // With the remote present, the remote's ARMED/ESTOP flags rule.
  if (!radioPresent(now_ms) && !estop_) {
    armed_ = (m != MODE_IDLE);
    if (armed_) need_arm_edge_ = false;
  }
}

void ModeManager::onPiCmdVel(float speed_mms, float turn_dps, uint32_t now_ms) {
  if (isnan(speed_mms) || isnan(turn_dps) || isinf(speed_mms) || isinf(turn_dps)) return;
  cmd_speed_ = speed_mms;
  cmd_turn_ = turn_dps;
  cmd_ms_ = now_ms;
  cmd_valid_ = true;
}

void ModeManager::update(uint32_t now_ms, bool imu_ok, const ModeLimits& lim) {
  const RobotMode eff = (estop_ || !armed_ || !imu_ok) ? MODE_IDLE : requested_;
  if (eff != mode_) {
    mode_ = eff;
    mode_changed_ = true;
    cmd_valid_ = false;  // a mode change never inherits an old command
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
