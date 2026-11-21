// Host test for src/motor_test.cpp: runs the state machine against simulated TT gear motors
// (static + kinetic dead zone, first-order speed, gearbox backlash, encoder noise) and checks
// the safety interlocks. Build + run (any g++, e.g. on the Pi):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src test/host/motor_test_sim.cpp src/motor_test.cpp -o /tmp/mt && /tmp/mt
//   /tmp/mt --log > mt.log && python3 tools/motor_fit.py mt.log      (full test 4 log -> fit)
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_test.h"

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

struct Wheel {
  // plant (volts)
  float v_start, v_kin, k_mms_per_v, tau_s, backlash_counts;
  float w = 0;         // motor-side speed, mm/s at the wheel
  double pm = 0;       // motor-side position, counts
  double po = 0;       // output (encoder) position, counts
  void step(float vm, float dt) {
    const float mag = fabsf(vm);
    float target = 0;
    if (fabsf(w) < 1.0f && mag < v_start) target = 0;  // stiction
    else if (mag > v_kin) target = (vm > 0 ? 1 : -1) * k_mms_per_v * (mag - v_kin);
    w += (target - w) * (dt / tau_s);
    if (fabsf(w) < 1.0f && mag < v_start) w = 0;
    pm += w * dt / MotorTest::kMmPerCount;
    const double g = backlash_counts / 2;
    if (pm - po > g) po = pm - g;
    if (pm - po < -g) po = pm + g;
  }
};

struct Sim {
  Wheel wl{1.9f, 1.5f, 95.0f, 0.06f, 40}, wr{2.1f, 1.6f, 90.0f, 0.07f, 30};
  MtIn in;
  MotorTest mt;
  uint32_t t_us = 1000;
  int32_t ring[2][16] = {};
  uint32_t ring_t[16] = {};
  int ring_i = 0, ring_n = 0;
  float vb = 11.8f;
  bool print = false;
  int lines = 0;
  char last[200] = "";
  bool saw_abort = false, saw_end = false;

  Sim() {
    in.request = 0;
    in.armed = true;
    in.idle = true;
    in.radio_live = true;
    in.imu_ok = true;
    in.enc_ok = true;
    in.pitch = -3.0f;
    in.vbatt = vb;
  }
  void drain() {
    for (const char* l; (l = mt.peekLine()) != nullptr; mt.popLine()) {
      if (print) printf("[\"log\",%.3f,\"%s\"]\n", t_us * 1e-6, l);
      snprintf(last, sizeof(last), "%s", l);
      if (!strncmp(l, "mt,abort", 8)) saw_abort = true;
      if (!strncmp(l, "mt,end", 6)) saw_end = true;
      lines++;
    }
  }
  // advance ms milliseconds at 1 kHz; encoders at 200 Hz
  void run(uint32_t ms, bool enc_running = true) {
    for (uint32_t i = 0; i < ms; i++) {
      t_us += 1000;
      const float dl = mt.driving() ? mt.left() : 0.0f, dr = mt.driving() ? mt.right() : 0.0f;
      // battery sags a little under load
      in.vbatt = vb - 0.0002f * (fabsf(dl) + fabsf(dr));
      wl.step(dl / 4095.0f * in.vbatt, 1e-3f);
      wr.step(dr / 4095.0f * in.vbatt, 1e-3f);
      if (enc_running && t_us % 5000 == 0) {
        const int32_t cl = (int32_t)floor(wl.po) + (rand() % 3) - 1;
        const int32_t cr = (int32_t)floor(wr.po) + (rand() % 3) - 1;
        ring[0][ring_i] = cl;
        ring[1][ring_i] = cr;
        ring_t[ring_i] = t_us;
        ring_i = (ring_i + 1) % 16;
        if (ring_n < 16) ring_n++;
        const int old = (ring_i + 16 - ring_n) % 16;
        const float dt = (t_us - ring_t[old]) * 1e-6f;
        in.speed_l = dt > 0 ? (cl - ring[0][old]) * MotorTest::kMmPerCount / dt : 0;
        in.speed_r = dt > 0 ? (cr - ring[1][old]) * MotorTest::kMmPerCount / dt : 0;
        in.enc_l = cl;
        in.enc_r = cr;
        in.enc_us = t_us;
      }
      in.now_us = t_us;
      mt.step(in);
      if (mt.takeDone()) in.request = 0;  // what main.cpp does
      // invariant: never any duty unless driving
      if (!mt.driving()) CHECK(mt.left() == 0 && mt.right() == 0);
      CHECK(fabsf(mt.left()) <= 4095 && fabsf(mt.right()) <= 4095);
      drain();
    }
  }
};

static void testFull(bool print) {
  Sim s;
  s.print = print;
  s.in.request = 4;
  s.run(500);
  CHECK(!s.mt.driving());  // 1 s still-hold before the motors move
  s.run(700);
  CHECK(s.mt.driving());
  s.run(400000);
  CHECK(s.saw_end && !s.saw_abort);
  CHECK(!s.mt.driving() && !s.mt.pending());
  CHECK(s.in.request == 0);
  if (!print) printf("full test: %d lines, last: %s\n", s.lines, s.last);
}

static void testAbort(const char* name, void (*poke)(Sim&), const char* expect) {
  Sim s;
  s.in.request = 4;
  s.run(3000);
  CHECK(s.mt.driving());
  poke(s);
  s.run(1);
  CHECK(!s.mt.driving());
  CHECK(s.mt.left() == 0 && s.mt.right() == 0);
  CHECK(s.saw_abort && strstr(s.last, expect) != nullptr);
  CHECK(s.in.request == 0);
  printf("abort %-14s -> %s\n", name, s.last);
}

int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "--log")) {
    testFull(true);
    return g_fail ? 1 : 0;
  }
  testFull(false);
  testAbort("disarm", [](Sim& s) { s.in.armed = false; }, "disarmed");
  testAbort("estop", [](Sim& s) { s.in.estop = true; }, "estop");
  testAbort("mode", [](Sim& s) { s.in.idle = false; }, "mode_change");
  testAbort("remote", [](Sim& s) { s.in.radio_live = false; }, "remote_lost");
  testAbort("imu", [](Sim& s) { s.in.imu_ok = false; }, "imu");
  testAbort("encoder", [](Sim& s) { s.in.enc_ok = false; }, "encoder");
  testAbort("tilt", [](Sim& s) { s.in.pitch = 25; }, "tilt");
  testAbort("rate", [](Sim& s) { s.in.pitch_rate = 200; }, "pitch_rate");
  testAbort("cancel", [](Sim& s) { s.in.request = 0; }, "cancelled");
  testAbort("other test", [](Sim& s) { s.in.request = 2; }, "cancelled");

  {  // encoder reads stop -> abort within ~50 ms
    Sim s;
    s.in.request = 1;
    s.run(3000);
    CHECK(s.mt.driving());
    s.run(60, false);
    CHECK(!s.mt.driving() && strstr(s.last, "encoder_stale"));
    printf("abort %-14s -> %s\n", "enc stale", s.last);
  }
  {  // not armed: waits, never drives, gives up after 60 s
    Sim s;
    s.in.armed = false;
    s.in.request = 1;
    s.run(30000);
    CHECK(!s.mt.driving() && s.mt.pending());
    s.in.idle = false;  // armed in BALANCE: still waits
    s.in.armed = true;
    s.run(20000);
    CHECK(!s.mt.driving());
    s.run(11000);
    CHECK(!s.mt.pending() && s.in.request == 0 && strstr(s.last, "not_started"));
    printf("wait timeout       -> %s\n", s.last);
  }
  {  // moving body / tilted: never starts
    Sim s;
    s.in.request = 2;
    s.in.pitch_rate = 30;
    s.run(5000);
    CHECK(!s.mt.driving());
    s.in.pitch_rate = 0;
    s.in.pitch = 30;
    s.run(5000);
    CHECK(!s.mt.driving());
    s.in.pitch = -2;
    s.run(1200);
    CHECK(s.mt.driving());
    printf("start interlocks   ok\n");
  }
  {  // cancel while waiting
    Sim s;
    s.in.armed = false;
    s.in.request = 3;
    s.run(2000);
    s.in.request = 0;
    s.run(10);
    CHECK(!s.mt.pending() && strstr(s.last, "mt,cancel"));
  }
  {  // per-test time limit: a wheel that never moves still ends (ramp to max) - no hang
    Sim s;
    s.wl.v_start = 50;  // stuck left wheel
    s.wl.v_kin = 50;
    s.in.request = 4;
    s.run(400000);
    CHECK(s.saw_end || s.saw_abort);
    CHECK(!s.mt.pending());
    printf("stuck wheel        -> %s\n", s.last);
  }
  printf(g_fail ? "%d FAILED\n" : "all ok\n", g_fail);
  return g_fail ? 1 : 0;
}
