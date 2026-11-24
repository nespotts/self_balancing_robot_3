// Self-balancing robot v4 firmware (Teensy 4.1). See docs/firmware.md.
//
// loop():  fresh IMU sample?  -> controlStep()  (balance, then encoders if due)
//          otherwise          -> background()   (radio, Pi link, telemetry, battery, ...)
// Every background task is short and checks imu.dataReady() so a new sample is never left waiting.
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "battery.h"
#include "config.h"
#include "controller.h"
#include "imu.h"
#include "mode_manager.h"
#include "motor_test.h"
#include "motors.h"
#include "navigator.h"
#include "odometry.h"
#include "params.h"
#include "pi_link.h"
#include "protocol/link_protocol.h"
#include "protocol/radio_protocol.h"
#include "range_sweep.h"
#include "reboot_button.h"
#include "remote_radio.h"

static Imu imu;
static RangeSweep sweep;
static Odometry odom;
static Controller ctl;
static ModeManager modes;
static RemoteRadio radio;
static PiLink link;
static Battery batt;
static RebootButton button;
static MotorTest mtest;  // wheels-up motor test (mot.test), owns the motors while driving()
static Navigator nav;    // waypoints, GOTO / EXPLORE / HOLD, obstacle check (issue #8)

// control step state
static bool have_step = false, motors_active = false;
static uint32_t last_step_us = 0, enc_last_us = 0, step_count = 0;
static float trim_deg = 0;

// rates / telemetry
static float loop_hz = 0;
static uint32_t rate_t0_ms = 0, rate_steps0 = 0;
static TelemetryFrame tlm;            // snapshot of the last balance step
static bool tlm_fresh = false;
static uint32_t last_tlm_us = 0;
static bool hello_pending = true;
static uint16_t param_stream = P_COUNT;  // next ParamInfo to send (P_COUNT = idle)
static bool save_wait_logged = false, defaults_wait_logged = false;

// ------------------------------------------------------------------ helpers

static void logImuStatus() {
  char s[2 * LINK_MAX_PAYLOAD];
  int n = imu.debugString(s, sizeof(s));
  link.log("%.180s", s);
  if (n > 180) link.log("  ..%s", s + 180);
}

static float wrap180(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  if (a <= -180.0f) a += 360.0f;
  return a;
}

static int16_t clamp16(float v) {
  if (isnan(v)) return 0;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static uint16_t clampU16(float v) {
  if (isnan(v) || v < 0) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

static ModeLimits limits() {
  ModeLimits l;
  l.max_speed = P(P_DRV_MAX_SPEED);
  l.max_turn = P(P_DRV_MAX_TURN);
  l.turn_deadband = P(P_DRV_TURN_DB);
  return l;
}

static NavIn navIn(uint32_t now_ms) {
  NavIn n;
  n.now_ms = now_ms;
  n.mode = modes.mode();
  n.upright = ctl.last().upright;
  n.live = modes.supervised(now_ms);
  n.x = odom.x_mm;
  n.y = odom.y_mm;
  n.heading = imu.heading();
  n.speed = odom.speed_mms;
  n.yaw_rate = imu.sample().yaw_rate_dps;
  n.max_speed = P(P_DRV_MAX_SPEED) * modes.speedScale(now_ms);
  n.max_turn = P(P_DRV_MAX_TURN);
  return n;
}

static RangePose rangePose() {
  RangePose p;
  p.x_mm = odom.x_mm;
  p.y_mm = odom.y_mm;
  p.heading_deg = imu.heading();
  p.pitch_deg = imu.sample().pitch_deg;
  return p;
}

static uint8_t statusBits(uint32_t now_ms) {
  uint8_t st = 0;
  if (nav.obstacle()) st |= TLM_OBSTACLE;  // blocking the current motion (navigator.h)
  if (modes.mode() == MODE_GOTO || modes.mode() == MODE_EXPLORE) {
    if (nav.active()) st |= TLM_NAV_ACTIVE;
    if (nav.done()) st |= TLM_NAV_DONE;
  }
  if (modes.armed()) st |= TLM_ARMED;
  if (ctl.last().upright) st |= TLM_UPRIGHT;
  if (!modes.radioLive(now_ms)) st |= TLM_FAILSAFE;
  if (modes.piLive(now_ms)) st |= TLM_PI_LINK;
  if (batt.low()) st |= TLM_LOW_BATT;
  return st;
}

static uint8_t errorCode(uint32_t now_ms) {
  if (!imu.initialised()) return ERR_IMU_INIT;
  if (!imu.healthy(micros())) return ERR_IMU_STALE;
  if (!odom.leftOk()) return ERR_ENC_LEFT;
  if (!odom.rightOk()) return ERR_ENC_RIGHT;
  if (!radio.chipOk()) return ERR_RADIO_CHIP;
  if (radio.versionMismatch(now_ms)) return ERR_RADIO_VERSION;
  return ERR_OK;
}

static void buildTelemetry(uint32_t t_us) {
  const uint32_t now_ms = millis();
  const ImuSample& s = imu.sample();
  const CtlOut& o = ctl.last();
  TelemetryFrame& f = tlm;
  f.t_us = t_us;
  f.mode = modes.mode();
  f.status = statusBits(now_ms);
  f.error_code = errorCode(now_ms);
  f.radio_ok = modes.radioLive(now_ms) ? 1 : 0;
  f.pitch_deg = s.pitch_deg;
  f.pitch_rate_dps = s.pitch_rate_dps;
  f.roll_deg = s.roll_deg;
  f.heading_deg = imu.heading();
  f.yaw_rate_dps = s.yaw_rate_dps;
  f.x_mm = odom.x_mm;
  f.y_mm = odom.y_mm;
  f.speed_mms = odom.speed_mms;
  f.turn_rate_dps = odom.turn_rate_dps;
  f.cmd_speed_mms = o.speed_sp;
  f.cmd_turn_dps = o.turn_sp;
  f.wheel_left_mms = odom.left_mms;
  f.wheel_right_mms = odom.right_mms;
  f.balance_target_deg = o.balance_target;
  f.motor_left = motors_left() / MOTOR_FULL;
  f.motor_right = motors_right() / MOTOR_FULL;
  f.battery_v = batt.packV();
  f.loop_hz = loop_hz;
  f.bal_p = o.bal_p;
  f.bal_i = o.bal_i;
  f.bal_d = o.bal_d;
  f.steer_out = o.steer;
  f.vel_out_deg = o.vel_out;
  f.step = step_count;
  f.remote_flags = (modes.estop() ? LINK_RF_ESTOP : 0) | (modes.armWait() ? LINK_RF_ARM_WAIT : 0) |
                   (modes.radioPresent(now_ms) ? LINK_RF_PRESENT : 0) |
                   (modes.linkEstop() ? LINK_RF_LINK_ESTOP : 0) | (modes.handoverWait() ? LINK_RF_NAV_HANDOVER : 0);
  f.nav_state = nav.state();
  f.nav_index = nav.index();
  tlm_fresh = true;
}

static TelemetryPacket buildRadioTelemetry(uint8_t ack_seq) {
  const uint32_t now_ms = millis();
  TelemetryPacket t = {};
  t.version = RADIO_PROTOCOL_VERSION;
  t.ack_seq = ack_seq;
  t.mode = modes.mode();
  t.status = statusBits(now_ms);
  t.battery_mv = clampU16(batt.packV() * 1000.0f);
  t.min_cell_mv = clampU16(batt.minCellV() * 1000.0f);
  t.pitch_cdeg = clamp16(imu.sample().pitch_deg * 100.0f);
  t.heading_ddeg = clamp16(wrap180(imu.heading()) * 10.0f);
  t.x_cm = clamp16(odom.x_mm / 10.0f);
  t.y_cm = clamp16(odom.y_mm / 10.0f);
  t.speed_mms = clamp16(odom.speed_mms);
  if (nav.lidarFresh(now_ms)) {  // LD19 (#77): the navigator's corridor distances (0 = nothing in it)
    t.range_front_mm = clampU16(nav.frontMm());
    t.range_rear_mm = clampU16(nav.rearMm());
  } else {
    t.range_front_mm = sweep.frontMm(now_ms);
    t.range_rear_mm = sweep.rearMm(now_ms);
  }
  t.waypoint_count = nav.count();
  t.nav_index = nav.index();
  t.error_code = errorCode(now_ms);
  t.loop_khz_x10 = (uint8_t)fminf(255.0f, loop_hz / 100.0f + 0.5f);  // balance loop rate
  return t;
}

static bool sendParam(uint16_t id) {
  const ParamDef& d = params_def(id);
  ParamInfoFrame f = {};
  f.id = id;
  f.value = g_param[id];
  f.min = d.min;
  f.max = d.max;
  strncpy(f.name, d.name, sizeof(f.name) - 1);
  return link.send(MSG_PARAM_INFO, &f, sizeof(f));
}

static bool sendHello() {
  HelloFrame h = {};
  h.link_version = LINK_PROTOCOL_VERSION;
  h.radio_version = RADIO_PROTOCOL_VERSION;
  h.param_count = P_COUNT;
  snprintf(h.firmware, sizeof(h.firmware), "%s", FIRMWARE_ID);
  return link.send(MSG_HELLO, &h, sizeof(h));
}

static bool sendNavStatus() {
  NavStatusFrame n = {};
  n.state = nav.state();
  n.waypoint_count = nav.count();
  n.nav_index = nav.index();
  n.target_x_mm = nav.targetX();
  n.target_y_mm = nav.targetY();
  n.distance_mm = nav.distance();
  n.obstacle_mm = nav.obstacleMm();
  return link.send(MSG_NAV_STATUS, &n, sizeof(n));
}

static void handleEvent(uint8_t ev) {
  switch (ev) {
    case EVT_MARK_WAYPOINT:
      if (nav.add(odom.x_mm, odom.y_mm)) nav.listChanged(false);
      else link.log("waypoints: list full (%u)", (unsigned)WAYPOINT_MAX);
      break;
    case EVT_CLEAR_WAYPOINTS:
      nav.clearList();
      nav.listChanged(true);
      break;
    case EVT_RESET_POSE: {
      const float h_old = imu.heading();
      odom.resetPose();
      imu.resetHeading();
      ctl.shiftHeading(-h_old);  // keep the heading target where it physically was
      nav.onPoseReset();         // waypoints / obstacle points / hold point were in the old frame
      link.log("pose reset (waypoints cleared)");
      break;
    }
    case EVT_GO_HOME: {  // list = [first waypoint, or the origin], then GOTO (an event never arms)
      const float hx = nav.count() ? nav.wpX(0) : 0.0f, hy = nav.count() ? nav.wpY(0) : 0.0f;
      nav.clearList();
      nav.add(hx, hy);
      nav.listChanged(true);
      modes.requestMode(MODE_GOTO);
      link.log("GO_HOME: to %.0f, %.0f", (double)hx, (double)hy);
      break;
    }
    case EVT_START_SCAN:
      link.log(P(P_RNG_ENABLE) >= 0.5f ? "START_SCAN: the sweep runs all the time while rng.enable = 1"
                                       : "START_SCAN: rng.enable is 0");
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ Pi link input

static void onLinkFrame(uint8_t type, const uint8_t* p, uint8_t len) {
  const uint32_t now_ms = millis();
  switch (type) {
    case MSG_HEARTBEAT:
      if (len >= sizeof(HeartbeatFrame)) {
        HeartbeatFrame hb;
        memcpy(&hb, p, sizeof(hb));
        modes.onPiHeartbeat(now_ms, hb.clients);  // clients 0 = nobody watching: not supervision (#38)
      }
      break;
    case MSG_CMD_VEL:
      if (len >= sizeof(CmdVelFrame)) {
        CmdVelFrame c;
        memcpy(&c, p, sizeof(c));
        modes.onPiCmdVel(c.speed_mms, c.turn_dps, now_ms);
      }
      break;
    case MSG_SET_MODE:
      if (len >= sizeof(SetModeFrame)) {
        const SetModeResult r = modes.onPiSetMode(p[0], now_ms);
        if (r == SETMODE_ESTOP) link.log("SET_MODE %u ignored: station E-stop latched (release it first)", (unsigned)p[0]);
        else if (r == SETMODE_NOT_ARMED) link.log("SET_MODE %u ignored: not armed (arm first)", (unsigned)p[0]);
      }
      break;
    case MSG_SET_PARAM:
      if (len >= sizeof(SetParamFrame)) {
        SetParamFrame s;
        memcpy(&s, p, sizeof(s));
        if (params_set(s.id, s.value)) sendParam(s.id);  // echo the value actually applied
      }
      break;
    case MSG_GET_PARAMS:
      hello_pending = true;
      param_stream = 0;
      break;
    case MSG_GOTO:
      if (len >= 2) {
        const uint8_t count = p[1];
        if (count <= 16 && len == 2 + count * 8) {
          GotoFrame g = {};
          memcpy(&g, p, len);
          if (g.replace) nav.clearList();
          uint8_t added = 0;
          for (uint8_t i = 0; i < count; i++) added += nav.add(g.points[i].x_mm, g.points[i].y_mm) ? 1 : 0;
          if (added < count)
            link.log("waypoints: list full (%u), %u dropped", (unsigned)WAYPOINT_MAX, (unsigned)(count - added));
          nav.listChanged(g.replace != 0);  // also resumes from BLOCKED
        }
      }
      break;
    case MSG_LIDAR_OBSTACLES:  // LD19 obstacle sectors from the bridge (#77): replace the sweep while fresh
      if (len == sizeof(LidarObstacleFrame)) {
        LidarObstacleFrame f;
        memcpy(&f, p, sizeof(f));
        nav.addLidar(f, navIn(now_ms));
      }
      break;
    case MSG_EVENT:
      if (len < sizeof(EventFrame)) break;
      if (p[0] == EVT_LINK_ESTOP) {  // latching station E-stop (issue #41)
        const bool was = modes.linkEstop();
        modes.onLinkEstop();
        modes.update(now_ms, imu.healthy(micros()), limits());
        motors_stop();
        motors_active = false;
        ctl.disengage();
        if (!was) link.log("E-STOP (station): latched, motors off");
      } else if (p[0] == EVT_LINK_ESTOP_RELEASE) {
        if (modes.linkEstop()) {
          modes.onLinkEstopRelease();
          link.log(modes.radioPresent(now_ms) ? "E-stop released: re-arm with the remote (ARMED off -> on)"
                                              : "E-stop released: arm from the station to continue");
        }
      } else if (p[0] == EVT_LINK_ARM) {
        modes.onLinkArm(now_ms);
      } else if (p[0] == EVT_LINK_STUCK) {  // lidar: wheels turn, robot still (#79)
        nav.onLidarStuck(now_ms);
      } else {
        handleEvent(p[0]);
      }
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ control step

static void controlStep() {
  const ImuSample& s = imu.sample();
  const uint32_t now_ms = millis();
  // A gap longer than IMU_STALE_US (IMU stall / self-reset, a blocking call): the motors were (or should
  // have been) stopped, so balancing restarts through the re-engage window, never with stale I / D (#39).
  if (have_step && s.t_us - last_step_us > IMU_STALE_US) ctl.disengage();
  float dt = have_step ? (float)(s.t_us - last_step_us) * 1e-6f : imu.nominalDt();
  if (dt <= 0.0f || dt > 0.05f) dt = imu.nominalDt();
  have_step = true;
  last_step_us = s.t_us;

  modes.update(now_ms, true, limits());
  if (modes.takeModeChanged()) {
    ctl.onModeChange();
    nav.onModeChange(navIn(now_ms));
  }
  // navigation: its command is used by modes.update() from the next step on (2.5 ms later)
  nav.update(navIn(now_ms));
  modes.setAutoCmd(nav.speed(), nav.turn(), now_ms);
  float target_speed = modes.targetSpeed();
  if (!Navigator::navMode(modes.mode())) {
    // STATION commands get the obstacle stop / slow-down too; other modes only the TLM_OBSTACLE flag
    if (modes.mode() == MODE_STATION && target_speed != 0.0f) {
      target_speed = nav.limitSpeed(target_speed, P(P_DRV_MAX_SPEED), true);
    } else {
      const float moving = fabsf(odom.speed_mms) > 50.0f ? odom.speed_mms : 0.0f;
      nav.limitSpeed(target_speed != 0.0f ? target_speed : moving, P(P_DRV_MAX_SPEED), false);
    }
  }

  CtlIn in;
  in.dt = dt;
  in.now_ms = now_ms;
  in.pitch = s.pitch_deg;
  in.pitch_rate = s.pitch_rate_dps;
  in.roll = s.roll_deg;
  in.heading = imu.heading();
  in.yaw_rate = s.yaw_rate_dps;
  in.speed_mms = odom.speed_mms;
  in.speed_ok = odom.ok();
  in.target_speed = target_speed;
  in.target_turn = modes.targetTurn();
  in.trim_deg = trim_deg;
  in.enable = modes.motorsAllowed();
  const CtlOut& o = ctl.step(in);
  if (o.active) motors_set(o.left, o.right);
  else if (!mtest.driving()) motors_stop();
  motors_active = o.active;
  step_count++;

  // encoders right after the step, when there's the most time before the next sample
  const uint32_t t = micros();
  if (t - enc_last_us >= ENCODER_PERIOD_US) {
    enc_last_us = t;
    odom.update(t, imu.heading());
    if (odom.ok()) motors_speed_feedback(odom.leftCounts(), odom.rightCounts(), t);
  }
  // range sweep: a bounded slice of I2C work (a quarter of the step period), long before the next sample
  if (sweep.started())
    sweep.service(micros(), (uint32_t)(250000.0f / P(P_CTL_RATE_HZ)), rangePose(), modes.motorsAllowed());
  for (RangeFrame rf; sweep.takeNavReading(rf);) nav.addRange(rf, now_ms);
  buildTelemetry(s.t_us);
}

// ------------------------------------------------------------------ background

static void sendOutput(uint32_t now_us) {
  if (hello_pending && sendHello()) hello_pending = false;
  while (!hello_pending && param_stream < P_COUNT && sendParam(param_stream)) param_stream++;

  const float hz = P(P_TLM_HZ);
  const uint32_t period = (uint32_t)(1e6f / (hz < 1.0f ? 1.0f : hz));
  if (tlm_fresh && now_us - last_tlm_us >= period) {
    if (link.send(MSG_TELEMETRY, &tlm, sizeof(tlm))) {
      last_tlm_us = now_us;
      tlm_fresh = false;
    }
  }
  RangeFrame rf;
  while (sweep.peekFrame(rf) && link.send(MSG_RANGE, &rf, sizeof(rf))) sweep.popFrame();

  const uint32_t now_ms = millis();
  if (nav.statusDue(now_ms) && sendNavStatus()) nav.statusSent(now_ms);
  for (const char* l; (l = nav.peekLog()) != nullptr && link.log("%s", l);) nav.popLog();
  if (const char* r = radio.stats(now_ms)) link.log("%s", r);
}

// Motor test (issues #16, #26): own safety interlocks inside MotorTest; here only the wiring.
static void motorTestPoll(uint32_t now_us, uint32_t now_ms) {
  static bool drove = false, echo = false, first = true;
  static float last_l = 0, last_r = 0;
  const ImuSample& s = imu.sample();
  MtIn in;
  in.now_us = now_us;
  in.request = (uint8_t)lroundf(P(P_MOT_TEST));
  in.armed = modes.armed();
  in.estop = modes.estop() || modes.linkEstop();
  in.idle = modes.mode() == MODE_IDLE;
  in.radio_live = modes.radioLive(now_ms);
  in.imu_ok = imu.healthy(now_us);
  in.enc_ok = odom.ok();
  in.pitch = s.pitch_deg;
  in.roll = s.roll_deg;
  in.pitch_rate = s.pitch_rate_dps;
  in.enc_l = odom.leftCounts();
  in.enc_r = odom.rightCounts();
  in.enc_us = odom.lastUpdateUs();
  in.speed_l = odom.left_mms;
  in.speed_r = odom.right_mms;
  in.vbatt = batt.packV();
  in.max_duty = P(P_MOT_TEST_MAX);
  in.wheels = (uint8_t)lroundf(P(P_MOT_TEST_WHEELS));
  // drive config the test will use: applied by motors_test_begin() when it starts
  in.pwm_hz = P(P_MOT_PWM_HZ);
  in.drive_mode = P(P_MOT_DRIVE_MODE) >= 0.5f ? 1 : 0;
  in.brake_max_hz = P(P_MOT_BRAKE_MAX_HZ);
  in.brake_duty_max = P(P_MOT_BRAKE_DUTY_MAX);
  in.curve_exp = P(P_MOT_CURVE_EXP);
  in.knee_in = P(P_MOT_KNEE_IN);
  in.knee_out = P(P_MOT_KNEE_OUT);
  in.v_ref = P(P_MOT_V_REF);
  in.dz_band = P(P_MOT_DZ_BAND);
  in.dz[0][0] = P(P_MOT_DZ_L_FWD);
  in.dz[0][1] = P(P_MOT_DZ_L_REV);
  in.dz[1][0] = P(P_MOT_DZ_R_FWD);
  in.dz[1][1] = P(P_MOT_DZ_R_REV);
  in.k[0] = P(P_MOT_K_L);
  in.k[1] = P(P_MOT_K_R);
  in.sl.kp = P(P_MOT_SL_KP);
  in.sl.ki = P(P_MOT_SL_KI);
  in.sl.i_max = P(P_MOT_SL_IMAX);
  in.sl.band = P(P_MOT_SL_BAND);
  mtest.step(in);
  if (mtest.driving() && !drove) {
    // test start: both motors off, then its drive config (refused: mode 1 above mot.brake_max_hz)
    if (motors_test_begin(mtest.driveMode(), mtest.pwmHz())) drove = true;
    else mtest.abort("drive_config_refused", now_us);
    first = true;
  }
  if (mtest.driving()) {
    if (first || mtest.left() != last_l || mtest.right() != last_r)
      motors_test_drive(mtest.left(), mtest.right(), mtest.wheels());
    last_l = mtest.left();
    last_r = mtest.right();
    first = false;
  } else if (drove) {
    drove = false;
    motors_test_end();  // both off, back to the params' drive config
  }
  if (mtest.takeDone()) {
    g_param[P_MOT_TEST] = 0;
    echo = true;
  }
  if (echo && sendParam(P_MOT_TEST)) echo = false;
  for (const char* l; (l = mtest.peekLine()) != nullptr && link.log("%s", l);) mtest.popLine();
}

static void background() {
  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  // Safety: no balance step for IMU_STALE_US -> the motors must not keep their last command.
  if (motors_active && now_us - last_step_us > IMU_STALE_US) {
    motors_stop();
    motors_active = false;
    ctl.disengage();  // re-engage check + settle when samples come back (issue #39)
  }

  ControlPacket pkt;
  if (radio.poll(pkt, now_ms)) {
    modes.onRadioPacket(pkt, now_ms);
    trim_deg = ((float)pkt.knob_left - 127.5f) / 127.5f * P(P_BAL_KNOB_TRIM);
    radio.setAck(buildRadioTelemetry(pkt.seq));
  }
  modes.update(now_ms, imu.healthy(now_us), limits());
  if (!modes.motorsAllowed() && motors_active) {  // E-stop / disarm / IMU fault: don't wait for the next step
    motors_stop();
    motors_active = false;
    ctl.disengage();
  }
  uint8_t ev;
  while (modes.takeEvent(ev)) handleEvent(ev);
  motorTestPoll(now_us, now_ms);
  // mot.pwm_hz / mot.drive_mode: applied only while both motors are off (issue #22); mode 1 only up to mot.brake_max_hz
  motors_service();
  static bool cfg_log = false;
  if (motors_take_config_changed()) cfg_log = true;
  static const char* const kRefused[] = {"", "; drive_mode 1 refused above mot.brake_max_hz",
                                         "; drive_mode 1 refused: needs mot.model 1"};
  const uint8_t refused = motors_drive_mode_refused();
  if (cfg_log && link.log("mot: drive_mode %u (%s%s), pwm %.0f Hz", (unsigned)motors_drive_mode(),
                          motors_drive_mode() ? "IN PWM, brake decay" : "EN PWM, coast decay",
                          kRefused[refused < 3 ? refused : 0], (double)motors_pwm_hz()))
    cfg_log = false;

  if (imu.dataReady()) return;
  link.poll(onLinkFrame, 256);
  if (imu.dataReady()) return;

  if (link.hostConnected()) hello_pending = true;
  sendOutput(now_us);

  batt.update(now_ms);
  motors_set_vbatt(batt.packV(), now_ms);
  button.update();

  // no IMU steps (IMU fault): keep odometry and telemetry going at a low rate
  if (now_us - last_step_us > 20000) {
    if (now_us - enc_last_us >= 10000) {
      enc_last_us = now_us;
      odom.update(now_us, imu.heading());
      if (odom.ok()) motors_speed_feedback(odom.leftCounts(), odom.rightCounts(), now_us);
    }
    if (now_us - last_tlm_us >= 100000 && !tlm_fresh) buildTelemetry(now_us);
    static uint32_t sweep_us = 0;
    if (sweep.started() && now_us - sweep_us >= 2500) {
      sweep_us = now_us;
      sweep.service(now_us, 600, rangePose(), modes.motorsAllowed());
      for (RangeFrame rf; sweep.takeNavReading(rf);) nav.addRange(rf, now_ms);
    }
  }

  // IMU status line: 3 s after boot, whenever its health changes, and every 60 s
  static uint32_t imu_log_ms = 0;
  static int8_t imu_was = -1;
  const int8_t imu_now = imu.healthy(now_us) ? 1 : 0;
  if ((imu_now != imu_was && now_ms > 3000) || now_ms - imu_log_ms >= 60000) {
    imu_was = imu_now;
    imu_log_ms = now_ms;
    logImuStatus();
  }

  if (now_ms - rate_t0_ms >= 500) {
    loop_hz = (float)(step_count - rate_steps0) * 1000.0f / (float)(now_ms - rate_t0_ms);
    rate_t0_ms = now_ms;
    rate_steps0 = step_count;
  }

  // things that may block for a few ms: only with the motors off
  if (imu.dataReady()) return;
  const bool motors_off = !motors_active && !mtest.driving();
  imu.service(now_ms, motors_off);
  radio.service(now_ms, motors_off);
  if (const char* s = sweep.serviceSlow(now_ms, motors_off)) link.log("%s", s);
  if (g_param_save_requested) {
    if (motors_off) {
      g_param_save_requested = false;
      save_wait_logged = false;
      link.log(params_save() ? "params: saved" : "params: SAVE FAILED");
    } else if (!save_wait_logged) {
      save_wait_logged = link.log("params: save waits until the motors are off");
    }
  }
  // cfg.defaults swaps every gain / limit at once: only with the motors off, like cfg.save (issue #42).
  // After a pending save, so a save asked for earlier never writes the defaults.
  if (g_param_defaults_requested) {
    if (motors_off) {
      g_param_defaults_requested = false;
      defaults_wait_logged = false;
      params_reset_defaults();
      param_stream = 0;
      link.log("params: defaults restored (not saved)");
    } else if (!defaults_wait_logged) {
      defaults_wait_logged = link.log("params: defaults wait until the motors are off");
    }
  }
}

// ------------------------------------------------------------------ setup / loop

void setup() {
  motors_begin();  // H-bridge pins driven low first
  // Deselect every SPI chip before anything talks on the bus (a floating IMU CS breaks the nRF24 init).
  pinMode(pins::IMU_CS, OUTPUT);
  digitalWriteFast(pins::IMU_CS, HIGH);
  pinMode(pins::RADIO_CSN, OUTPUT);
  digitalWriteFast(pins::RADIO_CSN, HIGH);
  Serial.begin(115200);
  const bool loaded = params_load();
  SPI.begin();
  Wire.begin();
  Wire1.begin();
  batt.begin();
  button.begin();
  // VL53L0X + servo: only with rng.enable = 1 (or later, when it is set with the motors off).
  // Their init re-begins Wire, so the clocks are set after it.
  if (P(P_RNG_ENABLE) >= 0.5f) sweep.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire1.setClock(I2C_CLOCK_HZ);
  odom.begin();
  while (millis() < 150) {}  // nRF24 needs ~100 ms after power-up before it takes its config
  radio.begin();
  imu.begin();
  link.log("boot %s, params %s, imu %s, radio %s", FIRMWARE_ID, loaded ? "from EEPROM" : "DEFAULTS",
           imu.initialised() ? "ok" : "FAIL", radio.chipOk() ? "ok" : "FAIL");
  if (sweep.started())
    link.log("rng: sensor0 %s, sensor1 %s", sweep.sensorOk(0) ? "ok" : "FAIL", sweep.sensorOk(1) ? "ok" : "FAIL");
  else
    link.log("rng: off (rng.enable 0)");
  rate_t0_ms = millis();
}

void loop() {
  if (imu.poll()) {
    controlStep();
    return;
  }
  background();
}
