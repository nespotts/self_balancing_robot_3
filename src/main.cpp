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
#include "crash_capture.h"
#include "encoder_io.h"
#include "imu.h"
#include "lidar_fwd.h"
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
#include "reset_cause.h"
#include "watchdog.h"

static Imu imu;
static RangeSweep sweep;
// AS5600s on bounded I2C reads (issue #109): left on Wire (LPI2C1, D18 / D19), right on Wire1 (LPI2C3, D17 / D16)
static As5600Io enc_left(Wire, IMXRT_LPI2C1_ADDRESS, I2C_LEFT_SDA, I2C_LEFT_SCL, I2C_LEFT_HZ, pins::ENC_L_DIR, false,
                         PH_ENC_L);
static As5600Io enc_right(Wire1, IMXRT_LPI2C3_ADDRESS, I2C_RIGHT_SDA, I2C_RIGHT_SCL, I2C_RIGHT_HZ, pins::ENC_R_DIR, true,
                          PH_ENC_R);
static Odometry odom;
static Controller ctl;
static ModeManager modes;
static RemoteRadio radio;
static PiLink link;
static Battery batt;
static RebootButton button;
static MotorTest mtest;  // wheels-up motor test (mot.test), owns the motors while driving()
static Navigator nav;    // waypoints, GOTO / EXPLORE / HOLD, obstacle check (issue #8)
static LidarForwarder lidar;  // LD19 on Serial7 -> MSG_LIDAR_RAW (issue #100)
static uint8_t lidar_rx_mem[LIDAR_RX_EXTRA];

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
static bool reset_log_pending = true;    // the reset-cause lines: at boot and whenever the host (re)connects
static uint32_t bg_poll_us = 0;          // last background pass that polled the Pi link (watchdog health)

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
  // contact detection (#104): the balance loop's last step
  const CtlOut& o = ctl.last();
  n.pitch_rate = imu.sample().pitch_rate_dps;
  n.cmd_speed = o.speed_sp;
  n.vel_out = o.vel_out;
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
  f.fault_flags = odom.faultFlags(micros()) | (wdt_running() ? 0 : LINK_FAULT_WDT_OFF);
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
  const ResetInfo& r = reset_info();  // v9 (#109)
  h.reset_cause = r.cause;
  h.reset_flags = r.flags;
  h.srsr = (uint16_t)r.srsr;
  h.prev_uptime_s = r.prev_uptime_s;
  h.off_s = r.off_s;
  h.uptime_ms = millis();
  return link.send(MSG_HELLO, &h, sizeof(h));
}

// "reset: ..." (+ the CrashReport summary): at boot and on every host connect, so the bridge always gets it.
static bool sendResetLog() {
  const ResetInfo& r = reset_info();
  char off[24] = "?";
  if (r.flags & LINK_RESETF_SNVS) snprintf(off, sizeof(off), "%lu s", (unsigned long)r.off_s);
  if (!link.log("reset: %s (SRSR 0x%03lx), previous run %lu s, off %s, watchdog %s", reset_cause_name(r.cause),
                (unsigned long)r.srsr, (unsigned long)r.prev_uptime_s, (r.flags & LINK_RESETF_SNVS) ? off : "unknown",
                wdt_running() ? "on" : "OFF"))
    return false;
  if (r.crash[0]) {
    link.log("reset: crash report: %.150s", r.crash);
    if (strlen(r.crash) > 150) link.log("reset: crash report (2): %.150s", r.crash + 150);
  }
  if (r.last[0]) link.log("reset: %s", r.last);  // the previous run's last phases (crash record, crash_capture.h)
  if (r.snvs[0]) link.log("reset: %s", r.snvs);
  link.log("build: wdog %d feed-every-pass %d snvs %d i2c %s early-hook %d capture %d", SBR_WDOG_ENABLE,
           SBR_WDT_FEED_EVERY_PASS, SBR_SNVS_MARKER, SBR_I2C_BOUNDED ? "bounded" : "wire", SBR_EARLY_HOOK,
           SBR_CRASH_CAPTURE);
  return true;
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

// ------------------------------------------------------------------ LD19 lidar on Serial7 (issue #100)

struct Serial7LidarIo : LidarFwdIo {
  int available() override { return Serial7.available(); }
  int read(uint8_t* dst, int n) override {
    int k = 0;
    while (k < n) {
      const int c = Serial7.read();
      if (c < 0) break;
      dst[k++] = (uint8_t)c;
    }
    return k;
  }
  int room() override { return Serial.availableForWrite(); }
  bool send(const uint8_t* payload, uint8_t len) override { return link.send(MSG_LIDAR_RAW, payload, len); }
  bool takeOverrun() override {  // LPUART7 receiver overrun (its 4-byte FIFO overflowed: interrupts were off)
    const uint32_t s = LPUART7_STAT;
    if (!(s & LPUART_STAT_OR)) return false;
    // write 1 to OR only: the other write-1-to-clear flags (IDLE is the receive interrupt's) are written as 0
    LPUART7_STAT = (s & ~(LPUART_STAT_LBKDIF | LPUART_STAT_RXEDGIF | LPUART_STAT_IDLE | LPUART_STAT_NF |
                          LPUART_STAT_FE | LPUART_STAT_PF | LPUART_STAT_MA1F | LPUART_STAT_MA2F)) |
                   LPUART_STAT_OR;
    return true;
  }
};
static Serial7LidarIo lidar_io;

static void lidarBegin() {
  Serial7.addMemoryForRead(lidar_rx_mem, sizeof(lidar_rx_mem));
  Serial7.begin(LIDAR_BAUD);      // 8N1
  pinMode(pins::LIDAR_TX, INPUT);  // TX7 is not used (the LD19 takes no commands): D29 stays high impedance
  lidar.begin(64 + (int)sizeof(lidar_rx_mem), millis());  // 64 = SERIAL7_RX_BUFFER_SIZE (core)
}

static void lidarEnd() {
  Serial7.end();
  lidar.end();
}

// Background, after the telemetry: one chunk at most, and only in the first 60 % of the balance period after a step, so
// the ~20 us of work can never be running when the next IMU sample comes (no added jitter in the balance step). With no
// IMU steps (fault) any time.
static void lidarService(uint32_t now_us, uint32_t now_ms) {
  if (!lidar.on()) return;
  const float rate = P(P_CTL_RATE_HZ);
  const uint32_t window_us = (uint32_t)(600000.0f / (rate < 50.0f ? 50.0f : rate));
  const uint32_t since = now_us - last_step_us;
  if (since >= window_us && since <= IMU_STALE_US) return;
  const uint32_t c0 = ARM_DWT_CYCCNT;
  lidar.service(now_us, now_ms, lidar_io);
  lidar.noteCycles(ARM_DWT_CYCCNT - c0);
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

// ------------------------------------------------------------------ encoder faults (issue #109)

static const char* encFaultWhich() {
  return odom.leftOk() ? (odom.rightOk() ? "no" : "right") : (odom.rightOk() ? "left" : "left + right");
}

// Fault set / cleared, what the fault stopped or refused, and the counters (every 10 s while anything happens, else
// every 60 s). The bridge's Health page reads the "enc:" line (station/bridge/health.py ENC_RE).
static void encoderLog(uint32_t now_us, uint32_t now_ms) {
  static bool pend[2] = {false, false};
  static uint8_t note_pend = 0, note_mode = 0;
  static uint32_t stats_ms = 0;
  static uint32_t last_sum = 0;
  for (int i = 0; i < 2; i++) {
    WheelEncoder& w = odom.wheel(i == 1);
    if (w.takeFaultChanged()) pend[i] = true;
    if (!pend[i]) continue;
    const char side = i ? 'R' : 'L';
    bool sent;
    if (w.faulted()) {
      const EncoderStats& s = w.stats();
      sent = link.log("enc %c: FAULT (last %s; %lu errors, %lu rejected, %lu bus recoveries): %s; drive modes stopped, "
                      "no arming", side, i2c_status_name(w.lastError()), (unsigned long)s.errors, (unsigned long)s.rejects,
                      (unsigned long)s.recoveries,
                      odom.speedOk() ? "speed from the other wheel + gyro, balancing in place"
                                     : "no wheel speed: velocity loop off (balance + heading only)");
    } else {
      sent = link.log("enc %c: ok again (%lu s of clean reads)", side, (unsigned long)(ENC_CLEAR_MS / 1000));
    }
    if (sent) pend[i] = false;
  }
  static const char* const kModes[] = {"IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"};
  if (!note_pend) modes.takeEncNotice(note_pend, note_mode);
  if (note_pend) {
    const char* m = note_mode < MODE_COUNT ? kModes[note_mode] : "?";
    bool sent = true;
    if (note_pend == ENC_NOTE_STOPPED)
      sent = link.log("enc fault (%s): %s stopped -> BALANCE, balancing in place", encFaultWhich(), m);
    else if (note_pend == ENC_NOTE_ARM_REFUSED)
      sent = link.log("enc fault (%s): arming refused (ARMED off -> on again once it clears)", encFaultWhich());
    else if (note_pend == ENC_NOTE_MODE_REFUSED)
      sent = link.log("enc fault (%s): %s refused", encFaultWhich(), m);
    if (sent) note_pend = 0;
  }
  const EncoderStats& a = odom.wheel(false).stats();
  const EncoderStats& b = odom.wheel(true).stats();
  const uint32_t sum = a.errors + a.rejects + a.recoveries + b.errors + b.rejects + b.recoveries;
  const uint32_t every = (sum != last_sum || !odom.ok()) ? 10000u : 60000u;
  if (now_ms - stats_ms >= every && now_ms > 5000) {
    uint32_t ma, xa, mb, xb;
    odom.wheel(false).takeTiming(ma, xa);
    odom.wheel(true).takeTiming(mb, xb);
    if (link.log("enc: L %s err %lu rej %lu rec %lu resync %lu read %lu/%lu us last %s | R %s err %lu rej %lu rec %lu "
                 "resync %lu read %lu/%lu us last %s",
                 odom.leftOk() ? "ok" : "FAULT", (unsigned long)a.errors, (unsigned long)a.rejects,
                 (unsigned long)a.recoveries, (unsigned long)a.resyncs, (unsigned long)ma, (unsigned long)xa,
                 i2c_status_name(odom.wheel(false).lastError()), odom.rightOk() ? "ok" : "FAULT",
                 (unsigned long)b.errors, (unsigned long)b.rejects, (unsigned long)b.recoveries,
                 (unsigned long)b.resyncs, (unsigned long)mb, (unsigned long)xb,
                 i2c_status_name(odom.wheel(true).lastError()))) {
      stats_ms = now_ms;
      last_sum = sum;
    }
  }
  (void)now_us;
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
        else if (r == SETMODE_ENC_FAULT)
          link.log("SET_MODE %u ignored: %s encoder fault: no arming, no GOTO / EXPLORE / HOLD / STATION until it reads "
                   "clean for %lu s", (unsigned)p[0], encFaultWhich(), (unsigned long)(ENC_CLEAR_MS / 1000));
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

  modes.setEncoderFault(!odom.ok());  // #109: no arming / drive modes on a faulted encoder
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
  in.speed_ok = odom.speedOk();  // one wheel faulted: the other wheel + gyro (odometry.h)
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
    odom.update(t, imu.heading(), s.yaw_rate_dps);  // phases enc-L / enc-R / i2c-recover inside (encoder_io.h)
    crash_phase(PH_STEP);
    if (odom.ok()) motors_speed_feedback(odom.leftCounts(), odom.rightCounts(), t);
  }
  // range sweep: a bounded slice of I2C work (a quarter of the step period), long before the next sample. Not while
  // the left encoder's bus (also the sweep's) is failing / recovering: Wire's own waits could stall the loop (#109).
  if (sweep.started() && !odom.leftBusSuspect(micros())) {
    crash_phase(PH_SWEEP);
    sweep.service(micros(), (uint32_t)(250000.0f / P(P_CTL_RATE_HZ)), rangePose(), modes.motorsAllowed());
  }
  crash_phase(PH_STEP);
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
  if (nav.contactDue() && link.send(MSG_CONTACT, &nav.contactFrame(), sizeof(ContactFrame))) nav.contactSent();  // #104
  for (const char* l; (l = nav.peekLog()) != nullptr && link.log("%s", l);) nav.popLog();
  if (const char* r = radio.stats(now_ms)) link.log("%s", r);
  // LD19 bytes last: the telemetry above goes first, and the forwarder leaves it USB room (LidarForwarder::kReserve)
  if (!imu.dataReady()) {
    crash_phase(PH_LIDAR);
    lidarService(micros(), now_ms);
  }
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

  crash_phase(PH_BG);
  ControlPacket pkt;
  if (radio.poll(pkt, now_ms)) {
    modes.onRadioPacket(pkt, now_ms);
    trim_deg = ((float)pkt.knob_left - 127.5f) / 127.5f * P(P_BAL_KNOB_TRIM);
    radio.setAck(buildRadioTelemetry(pkt.seq));
  }
  modes.setEncoderFault(!odom.ok());
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
  bg_poll_us = micros();  // the watchdog's "comms healthy" (watchdog.h)
  crash_phase(PH_LINK);
  link.poll(onLinkFrame, 256);
  if (imu.dataReady()) return;

  crash_phase(PH_LINK);
  if (link.hostConnected()) hello_pending = reset_log_pending = true;
  sendOutput(now_us);
  crash_phase(PH_LINK);
  if (reset_log_pending && !hello_pending && sendResetLog()) reset_log_pending = false;
  crash_phase(PH_ENC_LOG);
  encoderLog(now_us, now_ms);
  reset_cause_tick(now_ms);  // SBR_SNVS_MARKER only: the v9 SNVS marker, 1 Hz (phase "snvs")

  crash_phase(PH_BATT);
  batt.update(now_ms);
  motors_set_vbatt(batt.packV(), now_ms);
  button.update();

  // no IMU steps (IMU fault): keep odometry and telemetry going at a low rate
  if (now_us - last_step_us > 20000) {
    if (now_us - enc_last_us >= 10000) {
      enc_last_us = now_us;
      odom.update(now_us, imu.heading(), imu.sample().yaw_rate_dps);
      crash_phase(PH_BG);
      if (odom.ok()) motors_speed_feedback(odom.leftCounts(), odom.rightCounts(), now_us);
    }
    if (now_us - last_tlm_us >= 100000 && !tlm_fresh) buildTelemetry(now_us);
    static uint32_t sweep_us = 0;
    if (sweep.started() && now_us - sweep_us >= 2500 && !odom.leftBusSuspect(now_us)) {
      crash_phase(PH_SWEEP);
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

  // things that may block for a few ms: only with the motors off, and the watchdog on its long (idle) timeout
  if (imu.dataReady()) return;
  crash_phase(PH_WDT);
  const bool motors_off = !motors_active && !mtest.driving() && wdt_can_block();
  crash_phase(PH_IMU_SVC);
  imu.service(now_ms, motors_off);
  radio.service(now_ms, motors_off);
  crash_phase(PH_SLOW);
  // rng.enable 1 while the lidar runs: the sweep is about to start (motors off) and drive D28 / D29 (its sensor 1's
  // XSHUT / INT): Serial7 lets go of them first. The lidar comes back after a reboot with rng.enable 0.
  if (lidar.on() && P(P_RNG_ENABLE) >= 0.5f && motors_off && !sweep.started()) {
    lidarEnd();
    link.log("lidar: Serial7 off, rng.enable 1 takes D28 / D29 (sweep sensor 1); reboot with rng.enable 0 for the LD19");
  }
  if (const char* s = sweep.serviceSlow(now_ms, motors_off)) link.log("%s", s);
  if (const char* s = lidar.status(now_ms)) link.log("%s", s);
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
  // H-bridge pins driven low first (they already are: startup_early_hook in motors.cpp, a few us after reset)
  motors_begin();
  reset_cause_capture(FIRMWARE_ID);  // SRC_SRSR + SNVS marker + CrashReport, before anything can reset again (#109)
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
  // the encoders' buses: Wire.begin() again + each bus's clock (I2C_LEFT_HZ / I2C_RIGHT_HZ, config.h)
  enc_left.begin();
  enc_right.begin();
  odom.begin(&enc_left, &enc_right);
  while (millis() < 150) {}  // nRF24 needs ~100 ms after power-up before it takes its config
  radio.begin();
  imu.begin();
  link.log("boot %s, params %s, imu %s, radio %s", FIRMWARE_ID, loaded ? "from EEPROM" : "DEFAULTS",
           imu.initialised() ? "ok" : "FAIL", radio.chipOk() ? "ok" : "FAIL");
  if (sweep.started())
    link.log("rng: sensor0 %s, sensor1 %s", sweep.sensorOk(0) ? "ok" : "FAIL", sweep.sensorOk(1) ? "ok" : "FAIL");
  else
    link.log("rng: off (rng.enable 0)");
  // LD19 (issue #100): Serial7 on the "RPLidar C1" header, only while the sweep doesn't own D28 / D29
  if (!sweep.started()) {
    lidarBegin();
    link.log("lidar: Serial7 %lu 8N1 on D28 (RX7), forwarding LD19 bytes as MSG_LIDAR_RAW", (unsigned long)LIDAR_BAUD);
  } else {
    link.log("lidar: Serial7 off (rng.enable 1: D28 / D29 are the sweep's sensor 1 pins)");
  }
  // hardware watchdog last: setup() itself may block (IMU / radio init) with the motors off
  const bool wdt = wdt_begin();
  reset_cause_set_wdt(wdt);
  if (wdt)
    link.log("watchdog: RTWDOG on, %lu ms while the motors may run, %lu ms idle", (unsigned long)WDT_RUN_MS,
             (unsigned long)WDT_IDLE_MS);
  else if (SBR_WDOG_ENABLE)
    link.log("watchdog: FAILED to start (RTWDOG locked): no hang protection this boot");
  else
    link.log("watchdog: off in this build (SBR_WDOG_ENABLE 0)");
  bg_poll_us = micros();  // (the "reset: ..." line follows the first HELLO: reset_log_pending)
  rate_t0_ms = millis();
}

void loop() {
  crash_tick(millis());  // uptime in the crash record (one uncached store)
  crash_phase(PH_WDT);
  // Hardware watchdog (#109): fed here once per pass while background() runs and, with the motors possibly driven, the
  // balance steps come. Short timeout whenever the motors may run (armed in a mode, or the motor test).
  WdtIn w;
  w.now_us = micros();
  w.bg_us = bg_poll_us;
  w.step_us = last_step_us;
  w.motors_active = motors_active || mtest.driving() || modes.motorsAllowed();
  if (w.motors_active && !have_step) w.step_us = w.now_us;  // armed before the first step: background stops it
  wdt_service(w);
  crash_phase(PH_IMU_POLL);
  if (imu.poll()) {
    crash_phase(PH_STEP);
    controlStep();
    crash_phase(PH_LOOP);
    return;
  }
  background();
  crash_phase(PH_LOOP);  // back in the core (yield: serialEvent, EventResponder)
}
