// Host test for issue #109 (runaway motor after a fall; left encoder dropped out for minutes):
//   WheelEncoder through a mock I2C: corrupted reads rejected, stuck bus recovered with backoff (no reads meanwhile),
//   fault latch + clear after ENC_CLEAR_MS of clean reads; Odometry speed from the good wheel + gyro;
//   ModeManager: no arming and no GOTO / EXPLORE / HOLD / STATION on a faulted encoder, an armed robot stays armed and
//   balances in place; Controller keeps its velocity loop on the estimate; watchdog feed rule; reset-cause rules.
// Build + run (any g++, or MSVC /Zp1 with __attribute__ blanked):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/encoder_fault_sim.cpp src/wheel_encoder.cpp
//     src/odometry.cpp src/mode_manager.cpp src/controller.cpp -o /tmp/encfault && /tmp/encfault
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "mode_manager.h"
#include "odometry.h"
#include "params.h"
#include "reset_cause.h"
#include "watchdog.h"
#include "wheel_encoder.h"

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

// A wheel turning at `mms` (true angle integrated), with scripted failures. Every read "costs" 0.15 ms (lastReadUs).
struct MockIo : EncoderIo {
  double angle = 1000;          // counts, unwrapped
  I2cStatus fail = I2C_OK;      // returned instead of a reading while != OK
  int fail_reads = 0;           // ... for this many reads (-1 = until changed)
  int corrupt_next = 0;         // the next n reads return a random angle
  bool stuck = false;           // bus stuck: every read BUSY until recover() unsticks it
  bool recover_fixes = true;
  int reads = 0, recovers = 0;
  uint32_t seed = 12345;
  I2cStatus readAngle(uint16_t& raw) override {
    reads++;
    if (stuck) return I2C_BUSY;
    if (fail != I2C_OK && fail_reads != 0) {
      if (fail_reads > 0) fail_reads--;
      return fail;
    }
    if (corrupt_next > 0) {
      corrupt_next--;
      seed = seed * 1103515245u + 12345u;
      raw = (uint16_t)((uint32_t)(angle + 1500 + (seed >> 16) % 1000) & 0x0FFF);  // a far-off angle
      return I2C_OK;
    }
    raw = (uint16_t)(((int64_t)floor(angle)) & 0x0FFF);
    return I2C_OK;
  }
  bool recover() override {
    recovers++;
    if (recover_fixes) stuck = false;
    return !stuck;
  }
  uint32_t lastReadUs() const override { return 150; }
};

static const float kMmPerCount = 3.14159265f * 66.0f / 4096.0f;

// Run one encoder for dur_ms at 200 Hz with the wheel at `mms`.
static void spin(WheelEncoder& e, MockIo& io, uint32_t& t_us, uint32_t dur_ms, float mms) {
  for (uint32_t end = t_us + dur_ms * 1000u; t_us < end;) {
    t_us += 5000;
    io.angle += mms * 0.005 / kMmPerCount;
    e.update(t_us, kMmPerCount);
  }
}

// ---------------------------------------------------------------- plausibility
static void testRejectCorrupted() {
  printf("#109: a corrupted angle (huge delta) is rejected, counted, never integrated\n");
  MockIo io;
  WheelEncoder e;
  e.attach(&io);
  uint32_t t = 1000000;
  spin(e, io, t, 500, 200.0f);
  CHECK(!e.faulted() && e.speedValid() && fabsf(e.speedMms() - 200.0f) < 15.0f);
  const int32_t c0 = e.counts();
  io.corrupt_next = 1;
  spin(e, io, t, 5, 200.0f);      // the glitched read
  CHECK(e.stats().rejects == 1);
  spin(e, io, t, 300, 200.0f);
  const double expect = 300.0 * 0.2 / kMmPerCount + 200.0 * 0.005 / kMmPerCount;  // 305 ms of travel at 200 mm/s
  CHECK(fabs((e.counts() - c0) - expect) < 5);       // the jump did not get into the counts
  CHECK(fabsf(e.speedMms() - 200.0f) < 15.0f);        // nor into the speed (the 600 mm/s of the incident)
  CHECK(!e.faulted());
  // the wheel really jumped (e.g. after a recovery gap with < ENC_STALE_US): 3 rejects, then it is the new reference
  io.angle += 1500;
  spin(e, io, t, 5 * ENC_RESYNC_REJECTS + 5, 200.0f);
  CHECK(e.stats().resyncs >= 1);
  spin(e, io, t, 300, 200.0f);
  CHECK(fabsf(e.speedMms() - 200.0f) < 15.0f && !e.faulted());
}

// ---------------------------------------------------------------- stuck bus: recovery + backoff, never hammering it
static void testStuckBusRecovery() {
  printf("#109: stuck bus -> recovery at once, backoff doubles to %lu ms, no reads while backing off\n",
         (unsigned long)ENC_BACKOFF_MAX_MS);
  MockIo io;
  WheelEncoder e;
  e.attach(&io);
  uint32_t t = 1000000;
  spin(e, io, t, 300, 0.0f);
  io.stuck = true;
  io.recover_fixes = false;   // a slave that keeps SDA low
  const int r0 = io.reads;
  spin(e, io, t, 10000, 0.0f);  // 10 s: 2000 update() calls
  CHECK(e.faulted());
  const int reads = io.reads - r0;
  // 20, 40, 80, ... 1000 ms backoff: ~16 attempts in 10 s, not 2000
  CHECK(reads >= 10 && reads <= 25);
  CHECK(io.recovers == reads);  // every failed attempt after a recovery recovers again
  // the bus comes back by itself (cable reseated): the next attempt reads, the fault clears after ENC_CLEAR_MS
  io.stuck = false;
  spin(e, io, t, 1100, 0.0f);   // <= one max backoff to the next attempt
  CHECK(e.faulted());           // reading again, but not clean long enough yet
  spin(e, io, t, ENC_CLEAR_MS + 50, 0.0f);
  CHECK(!e.faulted());
  CHECK(e.stats().faults == 1);
  // recover() fixes it at once (the usual case: a slave mid-byte)
  MockIo io2;
  WheelEncoder e2;
  e2.attach(&io2);
  t = 1000000;
  spin(e2, io2, t, 300, 0.0f);
  io2.stuck = true;
  spin(e2, io2, t, 100, 0.0f);
  CHECK(io2.recovers == 1 && !io2.stuck);
  CHECK(!e2.faulted());         // one bad read + a 20 ms pause: below ENC_FAULT_BAD / ENC_STALE_US
}

static void testNackRecoveryAndFault() {
  printf("#109: NACKs: recovery after %u in a row; %u bad reads -> fault; clears only after %lu ms clean\n",
         (unsigned)ENC_RECOVER_AFTER, (unsigned)ENC_FAULT_BAD, (unsigned long)ENC_CLEAR_MS);
  MockIo io;
  WheelEncoder e;
  e.attach(&io);
  uint32_t t = 1000000;
  spin(e, io, t, 300, 100.0f);
  io.fail = I2C_NACK;
  io.fail_reads = 2;
  spin(e, io, t, 50, 100.0f);
  CHECK(io.recovers == 0 && !e.faulted() && e.stats().errors == 2);  // two NACKs: no recovery, no fault
  CHECK(e.speedValid() && fabsf(e.speedMms() - 100.0f) < 15.0f);      // held through them: no velocity-loop reset
  io.fail_reads = -1;                                                  // the cable is off
  spin(e, io, t, 200, 100.0f);
  CHECK(io.recovers >= 1 && e.faulted());
  io.fail = I2C_OK;
  spin(e, io, t, 1500, 100.0f);
  CHECK(e.faulted());
  io.fail = I2C_NACK;   // one glitch during the clean period restarts it
  io.fail_reads = 1;
  spin(e, io, t, 1500, 100.0f);
  CHECK(e.faulted());
  spin(e, io, t, ENC_CLEAR_MS, 100.0f);
  CHECK(!e.faulted());
}

// ---------------------------------------------------------------- odometry: the good wheel + gyro
static void testOdometryFallback() {
  printf("#109: left faulted -> speed from the right wheel + gyro; both faulted -> no speed\n");
  reset();
  MockIo l, r;
  Odometry o;
  o.begin(&l, &r);
  uint32_t t = 1000000;
  const float track = P(P_ODO_TRACK);
  // driving at 200 mm/s while turning at 30 deg/s CCW: right faster than left
  const float yaw = 30.0f, v = 200.0f;
  const float vl = v - yaw * 0.0174533f * track / 2, vr = v + yaw * 0.0174533f * track / 2;
  float heading = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t end = t + ms * 1000u; t < end;) {
      t += 5000;
      l.angle += vl * 0.005 / kMmPerCount;
      r.angle += vr * 0.005 / kMmPerCount;
      heading += yaw * 0.005f;
      o.update(t, heading, yaw);
    }
  };
  run(500);
  CHECK(o.ok() && o.speedOk() && fabsf(o.speed_mms - v) < 15.0f);
  l.fail = I2C_NACK;
  l.fail_reads = -1;
  run(300);
  CHECK(!o.leftOk() && o.rightOk() && o.speedOk() && o.estimated());
  CHECK(fabsf(o.speed_mms - v) < 15.0f);        // not frozen (the incident's 600 mm/s), not halved
  CHECK(fabsf(o.left_mms - vl) < 20.0f);
  CHECK(o.faultFlags(t) & LINK_FAULT_ENC_L);
  const float x0 = o.x_mm, y0 = o.y_mm;
  run(1000);
  const float d = sqrtf((o.x_mm - x0) * (o.x_mm - x0) + (o.y_mm - y0) * (o.y_mm - y0));
  CHECK(fabsf(d - 0.95f * v) < 30.0f);           // ~200 mm of arc (chord a bit shorter)
  r.fail = I2C_NACK;
  r.fail_reads = -1;
  run(300);
  CHECK(!o.leftOk() && !o.rightOk() && !o.speedOk() && o.speed_mms == 0.0f);
}

// ---------------------------------------------------------------- mode manager
static ControlPacket pkt(uint8_t mode, bool armed, uint8_t seq) {
  ControlPacket p = {};
  p.version = RADIO_PROTOCOL_VERSION;
  p.seq = seq;
  p.requested_mode = mode;
  p.flags = armed ? CTRL_FLAG_ARMED : 0;
  p.speed_limit = 100;
  return p;
}

struct Rig {
  ModeManager m;
  ModeLimits lim;
  uint32_t ms = 1000;
  uint8_t seq = 0, sel = MODE_IDLE, clients = 1;
  bool remote = true, remote_armed = true, enc_fault = false;
  void run(uint32_t dur_ms, float nav_speed = 100.0f) {
    for (uint32_t end = ms + dur_ms; ms < end;) {
      ms += 5;
      if (remote && ms % 20 == 0) m.onRadioPacket(pkt(sel, remote_armed, ++seq), ms);
      if (ms % 100 == 0) m.onPiHeartbeat(ms, clients);
      if (ms % 50 == 0) m.onPiCmdVel(150.0f, 0.0f, ms);
      m.setEncoderFault(enc_fault);
      m.setAutoCmd(nav_speed, 0, ms);
      m.update(ms, true, lim);
    }
  }
  int notes(uint8_t want) {
    int n = 0;
    for (uint8_t note, mode; m.takeEncNotice(note, mode);) n += note == want;
    return n;
  }
};

static void testNoArmOnFault() {
  printf("#109: encoder fault -> arming refused (remote switch and station two-tap)\n");
  Rig r;
  r.enc_fault = true;
  r.remote_armed = false;
  r.run(100);
  r.remote_armed = true;  // ARMED off -> on while faulted
  r.run(100);
  CHECK(!r.m.armed() && r.notes(ENC_NOTE_ARM_REFUSED) == 1);
  r.enc_fault = false;    // clears: still not armed (needs a fresh edge)
  r.run(200);
  CHECK(!r.m.armed() && r.m.armWait());
  r.remote_armed = false;
  r.run(100);
  r.remote_armed = true;
  r.run(100);
  CHECK(r.m.armed());
  // no remote: arm request + SET_MODE refused
  Rig s;
  s.remote = false;
  s.enc_fault = true;
  s.run(200);
  s.m.onLinkArm(s.ms);
  CHECK(s.m.onPiSetMode(MODE_BALANCE, s.ms) == SETMODE_ENC_FAULT);
  s.run(100);
  CHECK(!s.m.armed() && s.m.mode() == MODE_IDLE);
  s.enc_fault = false;
  s.run(100);
  s.m.onLinkArm(s.ms);
  CHECK(s.m.onPiSetMode(MODE_BALANCE, s.ms) == SETMODE_OK);
}

static void testAutonomyStops() {
  printf("#109: encoder fault while driving GOTO / EXPLORE / STATION / HOLD -> BALANCE in place, stays armed\n");
  const uint8_t modes[] = {MODE_GOTO, MODE_EXPLORE, MODE_HOLD, MODE_STATION};
  for (uint8_t md : modes) {
    Rig r;
    r.remote = false;
    r.run(200);
    r.m.onLinkArm(r.ms);
    CHECK(r.m.onPiSetMode(MODE_BALANCE, r.ms) == SETMODE_OK);
    r.run(50);
    CHECK(r.m.onPiSetMode(md, r.ms) == SETMODE_OK);
    r.run(200);
    CHECK(r.m.mode() == md);
    if (md != MODE_HOLD) CHECK(r.m.targetSpeed() > 0);
    r.enc_fault = true;
    r.run(20);
    CHECK(r.m.armed());                                   // never dropped
    CHECK(r.m.mode() == MODE_BALANCE);
    CHECK(r.m.targetSpeed() == 0 && r.m.targetTurn() == 0);
    CHECK(r.notes(ENC_NOTE_STOPPED) == 1);
    // the station / planner asks again: refused
    CHECK(r.m.onPiSetMode(md, r.ms) == SETMODE_ENC_FAULT);
    r.m.requestMode(MODE_GOTO);                           // GO_HOME
    r.run(500);
    CHECK(r.m.mode() == MODE_BALANCE && r.m.targetSpeed() == 0);
    CHECK(r.notes(ENC_NOTE_MODE_REFUSED) == 1);
    // cleared: nothing resumes by itself, a fresh request works
    r.enc_fault = false;
    r.run(500);
    CHECK(r.m.mode() == MODE_BALANCE);
    CHECK(r.m.onPiSetMode(md, r.ms) == SETMODE_OK);
    r.run(100);
    CHECK(r.m.mode() == md);
  }
  // the RC remote's selector: GOTO refused, MANUAL (a person driving) allowed
  Rig q;
  q.remote_armed = false;
  q.run(100);
  q.remote_armed = true;
  q.run(100);
  q.sel = MODE_BALANCE;
  q.run(100);
  q.enc_fault = true;
  q.sel = MODE_GOTO;
  q.run(200);
  CHECK(q.m.mode() == MODE_BALANCE);
  q.sel = MODE_MANUAL;
  q.run(100);
  CHECK(q.m.mode() == MODE_MANUAL);
}

// ---------------------------------------------------------------- controller on the estimate
static void testBalanceHoldsInPlace() {
  printf("#109: BALANCE on the one-wheel estimate: the velocity loop stays on and pushes back against drift\n");
  reset();
  Controller c;
  const float level = -P(P_BAL_OFFSET);
  uint32_t ms = 0;
  CtlIn in;
  in.enable = true;
  in.dt = 0.0025f;
  in.pitch = level;
  in.speed_ok = true;
  for (int i = 0; i < 2000; i++) {  // settle
    in.now_ms = ms += 2;
    c.step(in);
  }
  // drifting forward at 150 mm/s with zero command: the velocity loop leans it back (target behind the offset)
  in.speed_mms = 150.0f;
  for (int i = 0; i < 400; i++) {
    in.now_ms = ms += 2;
    c.step(in);
  }
  CHECK(c.last().active && c.last().speed_sp == 0.0f);
  CHECK(fabsf(c.last().vel_out) > 0.1f);
  CHECK(c.last().balance_target < level - 0.1f || c.last().balance_target > level + 0.1f);
  // no estimate at all (both wheels faulted): balance only
  in.speed_ok = false;
  in.now_ms = ms += 2;
  c.step(in);
  CHECK(c.last().active && c.last().vel_out == 0.0f);
}

// ---------------------------------------------------------------- watchdog feed rule
static void testWatchdogFeed() {
  printf("#109: watchdog: fed only with a recent background pass, and balance steps while the motors may run\n");
  WdtIn w;
  w.now_us = 10000000;
  w.bg_us = w.now_us - 1000;
  w.step_us = w.now_us - 2500;
  w.motors_active = true;
  CHECK(wdt_healthy(w));
  w.bg_us = w.now_us - WDT_BG_MAX_US - 1;       // background starved (link / radio / STOP can't get through)
  CHECK(!wdt_healthy(w));
  w.bg_us = w.now_us - 1000;
  w.step_us = w.now_us - IMU_STALE_US - 20000;  // driving without balance steps
  CHECK(!wdt_healthy(w));
  w.motors_active = false;                      // ... fine with the motors off (IMU fault: IDLE)
  CHECK(wdt_healthy(w));
  CHECK(wdt_timeout_for(true) == WDT_RUN_MS && wdt_timeout_for(false) == WDT_IDLE_MS);
  CHECK(WDT_RUN_MS >= 50 && WDT_RUN_MS <= 100);
  // wrap-around of micros()
  w.now_us = 1000;
  w.bg_us = 0xFFFFFFFFu - 2000;
  w.step_us = 0xFFFFFFFFu - 1000;
  w.motors_active = true;
  CHECK(wdt_healthy(w));
}

// ---------------------------------------------------------------- reset cause
static void testResetCause() {
  printf("#109: reset cause from SRC_SRSR + the SNVS marker\n");
  ResetFacts f;
  f.srsr = RST_SRSR_POR;
  CHECK(reset_classify(f) == LINK_RESET_POWER_ON);         // cold: SNVS lost
  f.snvs_ok = f.same_build = true;
  f.off_s = 1;
  CHECK(reset_classify(f) == LINK_RESET_POWER_DIP);        // the incident's suspect: a 5 V brownout
  f.off_s = 30;
  CHECK(reset_classify(f) == LINK_RESET_POWER_ON);
  f.same_build = false;
  CHECK(reset_classify(f) == LINK_RESET_FLASHED);
  f = ResetFacts();
  f.srsr = RST_SRSR_POR | RST_SRSR_WDOG3;
  CHECK(reset_classify(f) == LINK_RESET_WATCHDOG);
  f.srsr = RST_SRSR_LOCKUP_SW;
  CHECK(reset_classify(f) == LINK_RESET_LOCKUP);
  f.gpr5 = RST_GPR5_FAULT;
  CHECK(reset_classify(f) == LINK_RESET_FAULT);
  f.gpr5 = 0;
  f.snvs_ok = f.same_build = f.planned = true;
  CHECK(reset_classify(f) == LINK_RESET_REQUESTED);
  f.srsr = RST_SRSR_TEMP;
  CHECK(reset_classify(f) == LINK_RESET_TEMP);
  // crash record flags (src/crash_capture.h): the RTWDOG interrupt / our fault handler ran, whatever SRSR shows
  f = ResetFacts();
  f.srsr = RST_SRSR_LOCKUP_SW;
  f.snvs_ok = f.same_build = true;
  CHECK(reset_classify(f) == LINK_RESET_LOCKUP);
  f.wdog_isr = true;
  CHECK(reset_classify(f) == LINK_RESET_WATCHDOG);
  f.fault = true;
  CHECK(reset_classify(f) == LINK_RESET_FAULT);
  f.snvs_ok = false;  // no record: the flags mean nothing
  CHECK(reset_classify(f) == LINK_RESET_LOCKUP);
  CHECK(reset_unexpected(LINK_RESET_WATCHDOG) && reset_unexpected(LINK_RESET_POWER_DIP));
  CHECK(!reset_unexpected(LINK_RESET_POWER_ON) && !reset_unexpected(LINK_RESET_FLASHED) &&
        !reset_unexpected(LINK_RESET_REQUESTED));
  for (uint8_t c = 0; c < LINK_RESET_CAUSE_COUNT; c++) CHECK(strcmp(reset_cause_name(c), "?") != 0);
}

int main() {
  reset();
  testRejectCorrupted();
  testStuckBusRecovery();
  testNackRecoveryAndFault();
  testOdometryFallback();
  testNoArmOnFault();
  testAutonomyStops();
  testBalanceHoldsInPlace();
  testWatchdogFeed();
  testResetCause();
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
