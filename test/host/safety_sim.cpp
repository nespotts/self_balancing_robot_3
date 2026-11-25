// Host test for the safety rules of the 2026-11 review (issues #38 #39 #40 #41 #42): ModeManager supervision /
// handover / link E-stop / explicit arming, Controller::disengage after an IMU gap, the brake drive mode
// interlock (motor_model.h), and the Navigator's no-range rule. Build + run (any g++, e.g. on the Pi):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/safety_sim.cpp src/mode_manager.cpp
//     src/controller.cpp src/navigator.cpp src/contact.cpp -o /tmp/safety && /tmp/safety
// (cfg.defaults waiting for motors off is main.cpp wiring; the simulator test covers it: station/sim/tests.)
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "mode_manager.h"
#include "motor_model.h"
#include "navigator.h"
#include "params.h"

float g_param[P_COUNT];
static const float kDefaults[P_COUNT] = {
#define PARAM_DEF(id, name, def, mn, mx) def,
    PARAM_TABLE(PARAM_DEF)
#undef PARAM_DEF
};
static void reset() { memcpy(g_param, kDefaults, sizeof(g_param)); }

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

static ControlPacket pkt(uint8_t mode, bool armed, uint8_t seq, bool estop = false) {
  ControlPacket p = {};
  p.version = RADIO_PROTOCOL_VERSION;
  p.seq = seq;
  p.requested_mode = mode;
  p.flags = (armed ? CTRL_FLAG_ARMED : 0) | (estop ? CTRL_FLAG_ESTOP : 0);
  p.speed_limit = 100;
  return p;
}

// A clock with a remote and a Pi bridge sending at their real rates.
struct Rig {
  ModeManager m;
  ModeLimits lim;
  uint32_t ms = 1000;
  uint8_t seq = 0, sel = MODE_IDLE, clients = 1;
  bool remote = true, remote_armed = true, pi = true;
  void run(uint32_t dur_ms, float nav_speed = 100.0f) {
    for (uint32_t end = ms + dur_ms; ms < end;) {
      ms += 5;
      if (remote && ms % 20 == 0) m.onRadioPacket(pkt(sel, remote_armed, ++seq), ms);
      if (pi && ms % 100 == 0) m.onPiHeartbeat(ms, clients);
      m.setAutoCmd(nav_speed, 0, ms);
      m.update(ms, true, lim);
    }
  }
  void boot() {  // remote: ARMED off -> on after boot (the arm edge)
    remote_armed = false;
    run(100);
    remote_armed = true;
    run(100);
  }
};

// ---------------------------------------------------------------- #38 supervision
static void testRemoteLostNoResume() {
  printf("#38: GOTO under the remote, remote lost -> stays paused with the bridge up (clients 1)\n");
  Rig r;
  r.boot();
  r.sel = MODE_BALANCE;  // selection change -> BALANCE
  r.run(100);
  r.sel = MODE_GOTO;
  r.run(200);
  CHECK(r.m.mode() == MODE_GOTO && r.m.supervised(r.ms) && r.m.targetSpeed() > 0);
  r.remote = false;  // remote battery dies
  r.run(3000);
  CHECK(r.m.mode() == MODE_GOTO);          // still armed, balancing
  CHECK(!r.m.supervised(r.ms));
  CHECK(r.m.targetSpeed() == 0);           // does NOT resume on the Pi heartbeat
  CHECK(r.m.handoverWait());
  // explicit handover: the station sends GOTO
  CHECK(r.m.onPiSetMode(MODE_GOTO, r.ms) == SETMODE_OK);
  r.run(100);
  CHECK(r.m.supervised(r.ms) && r.m.targetSpeed() > 0 && !r.m.handoverWait());
}

static void testRemoteBack() {
  printf("#38: remote lost then back -> resumes under the remote\n");
  Rig r;
  r.boot();
  r.sel = MODE_GOTO;
  r.run(200);
  r.remote = false;
  r.run(2000);
  CHECK(r.m.targetSpeed() == 0 && r.m.handoverWait());
  r.remote = true;
  r.run(200);
  CHECK(r.m.mode() == MODE_GOTO && r.m.supervised(r.ms) && r.m.targetSpeed() > 0 && !r.m.handoverWait());
}

static void testPiNeedsClients() {
  printf("#38: no remote: Pi supervises only with clients > 0\n");
  Rig r;
  r.remote = false;
  r.run(200);
  r.m.onLinkArm(r.ms);
  CHECK(r.m.onPiSetMode(MODE_GOTO, r.ms) == SETMODE_OK);
  r.run(300);
  CHECK(r.m.mode() == MODE_GOTO && r.m.supervised(r.ms) && r.m.targetSpeed() > 0);
  r.clients = 0;  // browser closed: heartbeats keep coming
  r.run(300);
  CHECK(r.m.mode() == MODE_GOTO && !r.m.supervised(r.ms) && r.m.targetSpeed() == 0);
  r.clients = 2;
  r.run(300);
  CHECK(r.m.supervised(r.ms) && r.m.targetSpeed() > 0);
  r.pi = false;  // bridge gone
  r.run(700);
  CHECK(!r.m.supervised(r.ms) && r.m.targetSpeed() == 0);
}

// ---------------------------------------------------------------- explicit arming (no remote)
static void testArmIntent() {
  printf("arming: no remote, a plain SET_MODE never arms; arm request + SET_MODE does\n");
  Rig r;
  r.remote = false;
  r.run(200);
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_NOT_ARMED);  // e.g. the app stopping in the background
  r.run(100);
  CHECK(!r.m.armed() && r.m.mode() == MODE_IDLE);
  r.m.onLinkArm(r.ms);
  r.run(LINK_ARM_WINDOW_MS + 50);  // too late
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_NOT_ARMED);
  r.m.onLinkArm(r.ms);
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_OK);
  r.run(50);
  CHECK(r.m.armed() && r.m.mode() == MODE_BALANCE);
  CHECK(r.m.onPiSetMode(MODE_STATION, r.ms) == SETMODE_OK);  // armed: plain mode changes work
  r.run(50);
  CHECK(r.m.mode() == MODE_STATION);
  CHECK(r.m.onPiSetMode(MODE_IDLE, r.ms) == SETMODE_OK);     // IDLE disarms
  r.run(50);
  CHECK(!r.m.armed() && r.m.mode() == MODE_IDLE);
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_NOT_ARMED);  // the arm request was one-shot
}

// ---------------------------------------------------------------- #41 link E-stop
static void testLinkEstopRemote() {
  printf("#41: link E-stop with the remote present: selector / SET_MODE can't restart; release + arm edge can\n");
  Rig r;
  r.boot();
  r.sel = MODE_BALANCE;
  r.run(100);
  CHECK(r.m.mode() == MODE_BALANCE);
  r.m.onLinkEstop();
  r.m.update(r.ms, true, r.lim);
  CHECK(r.m.mode() == MODE_IDLE && !r.m.armed() && r.m.linkEstop());
  r.sel = MODE_MANUAL;  // remote operator moves the selector
  r.run(200);
  CHECK(r.m.mode() == MODE_IDLE);
  CHECK(r.m.onPiSetMode(MODE_GOTO, r.ms) == SETMODE_ESTOP);
  r.m.requestMode(MODE_GOTO);  // GO_HOME
  r.remote_armed = false;       // toggling ARMED during the latch does not count
  r.run(100);
  r.remote_armed = true;
  r.run(100);
  CHECK(r.m.mode() == MODE_IDLE && !r.m.armed());
  r.m.onLinkEstopRelease();
  r.run(200);
  CHECK(!r.m.linkEstop() && r.m.mode() == MODE_IDLE && !r.m.armed() && r.m.armWait());  // needs a fresh edge
  r.remote_armed = false;
  r.run(100);
  r.remote_armed = true;
  r.run(100);
  CHECK(r.m.armed() && r.m.mode() == MODE_IDLE);  // armed, but requests were dropped while latched
  r.sel = MODE_BALANCE;                            // a selection change after the release
  r.run(100);
  CHECK(r.m.mode() == MODE_BALANCE);
}

static void testLinkEstopNoRemote() {
  printf("#41: link E-stop without a remote: arm + SET_MODE refused until released\n");
  Rig r;
  r.remote = false;
  r.run(200);
  r.m.onLinkArm(r.ms);
  r.m.onPiSetMode(MODE_BALANCE, r.ms);
  r.run(50);
  CHECK(r.m.mode() == MODE_BALANCE);
  r.m.onLinkEstop();
  r.run(50);
  CHECK(r.m.mode() == MODE_IDLE && !r.m.armed());
  r.m.onLinkArm(r.ms);  // ignored while latched
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_ESTOP);
  r.run(100);
  CHECK(r.m.mode() == MODE_IDLE);
  r.m.onLinkEstopRelease();
  r.run(50);
  CHECK(r.m.mode() == MODE_IDLE);
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_NOT_ARMED);  // release alone never arms
  r.m.onLinkArm(r.ms);
  CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_OK);
  r.run(50);
  CHECK(r.m.mode() == MODE_BALANCE);
}

// ---------------------------------------------------------------- #39 controller after an IMU gap
static CtlIn ctlIn(uint32_t ms, float pitch) {
  CtlIn in;
  in.now_ms = ms;
  in.pitch = pitch;
  in.enable = true;
  return in;
}

static void testDisengage() {
  printf("#39: IMU gap with the robot tipped to 20 / 30 deg -> no output until back inside reengage\n");
  reset();
  Controller c;
  const float level = -P(P_BAL_OFFSET);
  uint32_t ms = 0;
  for (int i = 0; i < 800; i++) c.step(ctlIn(ms += 2, level + 0.2f * sinf(i * 0.05f)));
  CHECK(c.last().upright && c.last().active);
  // 100 ms gap: the robot tips 20 deg past the balance point (below the 45 deg cutoff)
  c.disengage();  // what main.cpp does when it stops the motors for staleness
  CHECK(!c.last().upright && !c.last().active);
  CtlOut o = c.step(ctlIn(ms += 100, level + 20.0f));
  CHECK(!o.active && o.left == 0 && o.right == 0);  // no full-power D kick
  o = c.step(ctlIn(ms += 2, level + 30.0f));         // someone holding it at 30 deg
  CHECK(!o.active);
  o = c.step(ctlIn(ms += 2, level + 5.0f));          // back inside bal.reengage_deg: re-engages
  CHECK(o.active && fabsf(o.bal_d) < 1e-3f && fabsf(o.bal_i) < 50.0f);  // fresh D history and I
  CHECK(o.speed_sp == 0);
  // without disengage (the old behaviour) the first step after the gap would have run with stale state:
  Controller old;
  ms = 0;
  for (int i = 0; i < 800; i++) old.step(ctlIn(ms += 2, level));
  o = old.step(ctlIn(ms += 100, level + 20.0f));
  CHECK(o.active && fabsf(o.right) >= P(P_MOT_MAX) - 1);  // documents the bug that disengage() avoids
}

// ---------------------------------------------------------------- #40 brake mode interlock
static void testBrakeInterlock() {
  printf("#40: drive_mode 1 needs pwm_hz <= brake_max_hz AND mot.model 1; brake duty cap\n");
  CHECK(motor_drive_refusal(false, 36621, 2000, 0) == MOTOR_DRIVE_OK);
  CHECK(motor_drive_refusal(true, 36621, 2000, 1) == MOTOR_DRIVE_REFUSED_HZ);
  CHECK(motor_drive_refusal(true, 1000, 2000, 0) == MOTOR_DRIVE_REFUSED_MODEL);
  CHECK(motor_drive_refusal(true, 1000, 2000, 1) == MOTOR_DRIVE_OK);
  CHECK(motor_brake_cap(4095, 1, 3000) == 3000);
  CHECK(motor_brake_cap(-4095, 1, 3000) == -3000);
  CHECK(motor_brake_cap(1200, 1, 3000) == 1200);
  CHECK(motor_brake_cap(4095, 0, 3000) == 4095);
}

// ---------------------------------------------------------------- #38 navigator without range readings
static NavIn navIn(uint8_t mode, uint32_t ms) {
  NavIn n;
  n.now_ms = ms;
  n.mode = mode;
  n.upright = true;
  n.live = true;
  return n;
}

static void feedRange(Navigator& nav, uint32_t ms) {
  RangeFrame f = {};
  f.valid = 0;  // nothing in range: still proves the sweep runs
  f.range_mm = 8190;
  nav.addRange(f, ms);
}

static float driveSpeed(uint8_t mode, bool ranges) {
  Navigator nav;
  nav.add(3000, 0);
  nav.listChanged(true);
  uint32_t ms = 10000;
  if (ranges) feedRange(nav, ms);
  nav.onModeChange(navIn(mode, ms));
  float vmax = 0;
  for (int i = 0; i < 400; i++) {
    ms += 5;
    if (ranges && i % 16 == 0) feedRange(nav, ms);
    nav.update(navIn(mode, ms));
    vmax = fmaxf(vmax, nav.speed());
  }
  while (nav.peekLog()) nav.popLog();
  return vmax;
}

static void testNoRange() {
  printf("#38: rng.enable 0 -> EXPLORE paused, GOTO capped at nav.blind_speed (0 = paused)\n");
  reset();
  CHECK(driveSpeed(MODE_EXPLORE, true) > 200);
  CHECK(driveSpeed(MODE_EXPLORE, false) == 0);
  CHECK(driveSpeed(MODE_GOTO, true) > 200);
  const float blind = driveSpeed(MODE_GOTO, false);
  CHECK(blind > 0 && blind <= P(P_NAV_BLIND_SPEED) + 0.01f);
  g_param[P_NAV_BLIND_SPEED] = 0;
  CHECK(driveSpeed(MODE_GOTO, false) == 0);
}

int main() {
  reset();
  testRemoteLostNoResume();
  testRemoteBack();
  testPiNeedsClients();
  testArmIntent();
  testLinkEstopRemote();
  testLinkEstopNoRemote();
  testDisengage();
  testBrakeInterlock();
  testNoRange();
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
