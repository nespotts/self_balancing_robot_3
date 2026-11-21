// Host test for the stiction kick (issue #69, StictionKick in src/motor_model.h): kick / fallback /
// hysteresis logic, then a stopped TT wheel (breakaway 2.5 V, kinetic 0.8 V) under a slowly rising command.
// Build + run (any g++, e.g. on the Pi):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src test/host/stiction_sim.cpp -o /tmp/st && /tmp/st
#include <math.h>
#include <stdio.h>

#include "motor_model.h"

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

static StictionCfg cfg() {
  StictionCfg c;
  c.bk_fwd = 2.5f;
  c.bk_rev = 2.1f;
  c.gain = 1.0f;
  c.kick_ms = 80;
  c.band = 0.03f;
  c.move_mms = 15;
  return c;
}

static const uint32_t kDt = 2500;  // 400 Hz

// kinetic model volts for u (mot.model 1, dz 0.8 V, v_ref 12)
static float vk(float u) { return u >= 0 ? motor_dz_volts(u, 0.8f, 12, 0.01f) : -motor_dz_volts(u, 0.8f, 12, 0.01f); }

static void testKickAndTimeout() {
  StictionKick k;
  const StictionCfg c = cfg();
  uint32_t t = 1000;
  // stopped, command 0 for a while: never a kick
  for (int i = 0; i < 40; i++, t += kDt) CHECK(k.apply(0, 0, 0, true, t, c) == 0);
  // inside the band: no kick, just the model
  for (int i = 0; i < 40; i++, t += kDt) CHECK(fabsf(k.apply(0.02f, vk(0.02f), 0, true, t, c) - vk(0.02f)) < 1e-6f);
  CHECK(k.state() == StictionKick::IDLE);
  // beyond the band, wheel stopped: kick at the breakaway volts at once (stopped > kStopUs already)
  float v = k.apply(0.05f, vk(0.05f), 0, true, t, c);
  CHECK(k.state() == StictionKick::KICK && fabsf(v - 2.5f) < 1e-6f);
  // the wheel stays stuck: kick lasts kick_ms, then fades to the model over kFadeUs
  t += kDt;
  int kick_steps = 1;
  while (k.apply(0.05f, vk(0.05f), 0, true, t, c) > 2.49f && kick_steps < 200) kick_steps++, t += kDt;
  CHECK(kick_steps >= 31 && kick_steps <= 34);  // 80 ms / 2.5 ms
  CHECK(k.state() == StictionKick::FADE);
  float last = 2.5f;
  bool mono = true;
  int fade_steps = 0;
  while (k.state() == StictionKick::FADE && fade_steps < 100) {
    t += kDt;
    const float o = k.apply(0.05f, vk(0.05f), 0, true, t, c);
    if (k.state() != StictionKick::FADE) break;  // fade over (a still-stuck wheel with a sustained command re-kicks)
    if (o > last + 1e-6f) mono = false;
    last = o;
    fade_steps++;
  }
  CHECK(mono && fade_steps >= 14 && fade_steps <= 18);  // 40 ms fade
  CHECK(last >= vk(0.05f) - 1e-6f);
}

static void testMotionEndsKick() {
  StictionKick k;
  const StictionCfg c = cfg();
  uint32_t t = 1000;
  for (int i = 0; i < 20; i++, t += kDt) k.apply(0, 0, 0, true, t, c);
  // reverse: uses bk_rev, negative
  float v = k.apply(-0.1f, vk(-0.1f), 0, true, t, c);
  CHECK(k.state() == StictionKick::KICK && fabsf(v + 2.1f) < 1e-6f);
  t += kDt;
  k.apply(-0.1f, vk(-0.1f), -5, true, t, c);  // creeping, still below bk_mms
  CHECK(k.state() == StictionKick::KICK);
  t += kDt;
  v = k.apply(-0.1f, vk(-0.1f), -40, true, t, c);  // broke loose
  CHECK(k.state() == StictionKick::FADE && v < 0 && fabsf(v) <= 2.1f && fabsf(v) >= fabsf(vk(-0.1f)));
  // big command: the model is above the floor, unchanged
  t += kDt;
  CHECK(fabsf(k.apply(-0.5f, vk(-0.5f), -40, true, t, c) - vk(-0.5f)) < 1e-6f);
}

static void testBandAndReverseDropFloor() {
  StictionKick k;
  const StictionCfg c = cfg();
  uint32_t t = 1000;
  for (int i = 0; i < 20; i++, t += kDt) k.apply(0, 0, 0, true, t, c);
  k.apply(0.1f, vk(0.1f), 0, true, t, c);
  CHECK(k.state() == StictionKick::KICK);
  t += kDt;
  // command back inside the band: floor gone at once
  CHECK(fabsf(k.apply(0.01f, vk(0.01f), 0, true, t, c) - vk(0.01f)) < 1e-6f);
  CHECK(k.state() == StictionKick::IDLE);
}

static void testHysteresis() {
  StictionKick k;
  const StictionCfg c = cfg();
  uint32_t t = 1000;
  for (int i = 0; i < 20; i++, t += kDt) k.apply(0, 0, 0, true, t, c);
  k.apply(0.1f, vk(0.1f), 0, true, t, c);  // kick 1
  t += kDt;
  k.apply(0.1f, vk(0.1f), 50, true, t, c);  // moving
  // the wheel stops again 10 ms later, the command flips sign: stopped < kStopUs -> no kick
  for (int i = 0; i < 4; i++, t += kDt) k.apply(-0.1f, vk(-0.1f), 0, true, t, c);
  CHECK(k.state() == StictionKick::IDLE);
  // after kStopUs but inside kLockUs and not sustained (command just flipped): still no kick
  for (int i = 0; i < 6; i++, t += kDt) CHECK(fabsf(k.apply(-0.1f, vk(-0.1f), 0, true, t, c) - vk(-0.1f)) < 1e-6f);
  // sustained (kSustainUs in one direction): kick even inside kLockUs
  int n = 0;
  while (k.state() != StictionKick::KICK && n < 100) {
    t += kDt;
    k.apply(-0.1f, vk(-0.1f), 0, true, t, c);
    n++;
  }
  CHECK(k.state() == StictionKick::KICK);
  CHECK(n >= 12 && n <= 16);  // ~60 ms after the flip, minus the 25 ms already spent
  // noise around 0 (command flipping every step beyond the band, wheel still): at most one kick per kLockUs
  StictionKick q;
  t = 5000;
  for (int i = 0; i < 20; i++, t += kDt) q.apply(0, 0, 0, true, t, c);
  int kicks = 0;
  uint8_t prev = StictionKick::IDLE;
  for (int i = 0; i < 400; i++, t += kDt) {  // 1 s
    const float u = (i & 1) ? 0.05f : -0.05f;
    q.apply(u, vk(u), 0, true, t, c);
    if (q.state() == StictionKick::KICK && prev != StictionKick::KICK) kicks++;
    prev = q.state();
  }
  CHECK(kicks <= 7);  // 1 s / 150 ms
  printf("noise flips: %d kicks / s\n", kicks);
}

static void testOffAndStale() {
  StictionKick k;
  StictionCfg c = cfg();
  uint32_t t = 1000;
  c.gain = 0;
  for (int i = 0; i < 40; i++, t += kDt) CHECK(fabsf(k.apply(0.1f, vk(0.1f), 0, true, t, c) - vk(0.1f)) < 1e-6f);
  c.gain = 1;
  for (int i = 0; i < 40; i++, t += kDt) CHECK(fabsf(k.apply(0.1f, vk(0.1f), 0, false, t, c) - vk(0.1f)) < 1e-6f);
  // gain scales the kick
  c.gain = 1.2f;
  StictionKick g;
  for (int i = 0; i < 20; i++, t += kDt) g.apply(0, 0, 0, true, t, c);
  CHECK(fabsf(g.apply(0.1f, vk(0.1f), 0, true, t, c) - 3.0f) < 1e-5f);
  // micros() wrap
  StictionKick w;
  t = 0xFFFFFFFFu - 30000u;
  c.gain = 1;
  for (int i = 0; i < 20; i++, t += kDt) w.apply(0, 0, 0, true, t, c);
  CHECK(fabsf(w.apply(0.1f, vk(0.1f), 0, true, t, c) - 2.5f) < 1e-6f);
}

// A stopped wheel (breakaway 2.5 V, kinetic 0.8 V) and a command ramping up 0 -> 0.3 in 0.3 s: time to motion.
static float timeToMove(bool kick) {
  StictionKick k;
  StictionCfg c = cfg();
  c.gain = kick ? 1.0f : 0.0f;
  float w = 0, pos = 0, pos_hist[8] = {};
  uint32_t t = 1000;
  for (int i = 0; i < 400; i++, t += kDt) {
    const float u = i < 20 ? 0 : fminf(0.3f, (i - 20) * kDt * 1e-6f);
    const float speed = (pos - pos_hist[i % 8]) / (8 * kDt * 1e-6f);  // 20 ms window
    pos_hist[i % 8] = pos;
    const float v = k.apply(u, vk(u), speed, true, t, c);
    const float a = fabsf(v);
    if (fabsf(w) < 1 && a < 2.5f) w = 0;
    else w += ((a > 0.8f ? (v > 0 ? 1 : -1) * 124 * (a - 0.8f) : 0) - w) * (kDt * 1e-6f / 0.045f);
    pos += w * kDt * 1e-6f;
    if (w > 5) return (i - 20) * kDt * 1e-3f;
  }
  return 1e9f;
}

int main() {
  testKickAndTimeout();
  testMotionEndsKick();
  testBandAndReverseDropFloor();
  testHysteresis();
  testOffAndStale();
  const float off = timeToMove(false), on = timeToMove(true);
  printf("stopped wheel, ramping command: moves after %.1f ms (kick off) vs %.1f ms (kick on)\n", off, on);
  CHECK(on < 60 && off > 120);
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
