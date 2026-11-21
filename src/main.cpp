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

#include "Lidar/sweep_api.h"
#include "battery.h"
#include "config.h"
#include "controller.h"
#include "imu.h"
#include "mode_manager.h"
#include "motors.h"
#include "odometry.h"
#include "params.h"
#include "pi_link.h"
#include "protocol/link_protocol.h"
#include "protocol/radio_protocol.h"
#include "reboot_button.h"
#include "remote_radio.h"

static Imu imu;
static Odometry odom;
static Controller ctl;
static ModeManager modes;
static RemoteRadio radio;
static PiLink link;
static Battery batt;
static RebootButton button;

// waypoints (navigation itself is issue #8)
static float wp_x[WAYPOINT_MAX], wp_y[WAYPOINT_MAX];
static uint8_t wp_count = 0;

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
static bool save_wait_logged = false;

// ------------------------------------------------------------------ helpers

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

static uint8_t statusBits(uint32_t now_ms) {
  uint8_t st = 0;
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
  t.range_front_mm = 0;  // lidar sweep: issue #7
  t.range_rear_mm = 0;
  t.waypoint_count = wp_count;
  t.nav_index = 0;
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

static void sendNavStatus() {
  NavStatusFrame n = {};
  n.state = 0;  // idle (navigation: issue #8)
  n.waypoint_count = wp_count;
  link.send(MSG_NAV_STATUS, &n, sizeof(n));
}

static void addWaypoint(float x, float y) {
  if (wp_count < WAYPOINT_MAX) {
    wp_x[wp_count] = x;
    wp_y[wp_count] = y;
    wp_count++;
  }
}

static void handleEvent(uint8_t ev) {
  switch (ev) {
    case EVT_MARK_WAYPOINT:
      addWaypoint(odom.x_mm, odom.y_mm);
      sendNavStatus();
      break;
    case EVT_CLEAR_WAYPOINTS:
      wp_count = 0;
      sendNavStatus();
      break;
    case EVT_RESET_POSE: {
      const float h_old = imu.heading();
      odom.resetPose();
      imu.resetHeading();
      ctl.shiftHeading(-h_old);  // keep the heading target where it physically was
      link.log("pose reset");
      break;
    }
    case EVT_GO_HOME:
      link.log("GO_HOME: navigation not implemented yet (#8)");
      break;
    case EVT_START_SCAN:
      link.log("START_SCAN: sweep not implemented yet (#7)");
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
      if (len >= sizeof(HeartbeatFrame)) modes.onPiHeartbeat(now_ms);
      break;
    case MSG_CMD_VEL:
      if (len >= sizeof(CmdVelFrame)) {
        CmdVelFrame c;
        memcpy(&c, p, sizeof(c));
        modes.onPiCmdVel(c.speed_mms, c.turn_dps, now_ms);
      }
      break;
    case MSG_SET_MODE:
      if (len >= sizeof(SetModeFrame)) modes.onPiSetMode(p[0], now_ms);
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
          if (g.replace) wp_count = 0;
          for (uint8_t i = 0; i < count; i++) addWaypoint(g.points[i].x_mm, g.points[i].y_mm);
          sendNavStatus();
        }
      }
      break;
    case MSG_EVENT:
      if (len >= sizeof(EventFrame)) handleEvent(p[0]);
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ control step

static void controlStep() {
  const ImuSample& s = imu.sample();
  const uint32_t now_ms = millis();
  float dt = have_step ? (float)(s.t_us - last_step_us) * 1e-6f : imu.nominalDt();
  if (dt <= 0.0f || dt > 0.05f) dt = imu.nominalDt();
  have_step = true;
  last_step_us = s.t_us;

  modes.update(now_ms, true, limits());
  if (modes.takeModeChanged()) ctl.onModeChange();

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
  in.target_speed = modes.targetSpeed();
  in.target_turn = modes.targetTurn();
  in.trim_deg = trim_deg;
  in.enable = modes.motorsAllowed();
  const CtlOut& o = ctl.step(in);
  if (o.active) motors_set(o.left, o.right);
  else motors_stop();
  motors_active = o.active;
  step_count++;

  // encoders right after the step, when there's the most time before the next sample
  const uint32_t t = micros();
  if (t - enc_last_us >= ENCODER_PERIOD_US) {
    enc_last_us = t;
    odom.update(t, imu.heading());
  }
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
}

static void background() {
  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  // Safety: no balance step for IMU_STALE_US -> the motors must not keep their last command.
  if (motors_active && now_us - last_step_us > IMU_STALE_US) {
    motors_stop();
    motors_active = false;
  }

  ControlPacket pkt;
  if (radio.poll(pkt, now_ms)) {
    modes.onRadioPacket(pkt, now_ms);
    trim_deg = ((float)pkt.knob_left - 127.5f) / 127.5f * P(P_BAL_KNOB_TRIM);
    radio.setAck(buildRadioTelemetry(pkt.seq));
  }
  modes.update(now_ms, imu.healthy(now_us), limits());
  if (!modes.motorsAllowed() && motors_active) {  // E-stop / disarm: don't wait for the next step
    motors_stop();
    motors_active = false;
  }
  uint8_t ev;
  while (modes.takeEvent(ev)) handleEvent(ev);

  if (imu.dataReady()) return;
  link.poll(onLinkFrame, 256);
  if (imu.dataReady()) return;

  if (link.hostConnected()) hello_pending = true;
  sendOutput(now_us);

  batt.update(now_ms);
  button.update();

  // no IMU steps (IMU fault): keep odometry and telemetry going at a low rate
  if (now_us - last_step_us > 20000) {
    if (now_us - enc_last_us >= 10000) {
      enc_last_us = now_us;
      odom.update(now_us, imu.heading());
    }
    if (now_us - last_tlm_us >= 100000 && !tlm_fresh) buildTelemetry(now_us);
  }

  if (now_ms - rate_t0_ms >= 500) {
    loop_hz = (float)(step_count - rate_steps0) * 1000.0f / (float)(now_ms - rate_t0_ms);
    rate_t0_ms = now_ms;
    rate_steps0 = step_count;
  }

  // things that may block for a few ms: only with the motors off
  if (imu.dataReady()) return;
  const bool motors_off = !motors_active;
  imu.service(now_ms, motors_off);
  radio.service(now_ms, motors_off);
  if (g_param_defaults_requested) {
    g_param_defaults_requested = false;
    params_reset_defaults();
    param_stream = 0;
    link.log("params: defaults restored (not saved)");
  }
  if (g_param_save_requested) {
    if (motors_off) {
      g_param_save_requested = false;
      save_wait_logged = false;
      link.log(params_save() ? "params: saved" : "params: SAVE FAILED");
    } else if (!save_wait_logged) {
      save_wait_logged = link.log("params: save waits until the motors are off");
    }
  }
}

// ------------------------------------------------------------------ setup / loop

void setup() {
  motors_begin();  // H-bridge pins driven low first
  Serial.begin(115200);
  const bool loaded = params_load();
  SPI.begin();
  Wire.begin();
  Wire1.begin();
  batt.begin();
  button.begin();
  lidar_sweep_setup();  // VL53L0X init may re-begin Wire, so set the clocks after it
  Wire.setClock(I2C_CLOCK_HZ);
  Wire1.setClock(I2C_CLOCK_HZ);
  odom.begin();
  radio.begin();
  imu.begin();
  link.log("boot %s, params %s, imu %s, radio %s", FIRMWARE_ID, loaded ? "from EEPROM" : "DEFAULTS",
           imu.initialised() ? "ok" : "FAIL", radio.chipOk() ? "ok" : "FAIL");
  rate_t0_ms = millis();
}

void loop() {
  if (imu.poll()) {
    controlStep();
    return;
  }
  background();
}
