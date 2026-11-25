// Host test for src/contact.cpp (#111): the real robot's session of 2026-11-25 (v8 firmware) replayed through the C++
// contact detector - the detector's inputs rebuilt from its 100 Hz telemetry (station/tools/contact_replay.py extract;
// station/sim/tests/test_contact.py replays the same file through the Python port).
//   - the 3 pushes v8 missed (backing off at -40 / -80 mm/s into something, the wheels still, the lean growing) are found;
//   - nothing anywhere else: starts, stops, turns, slow corners, the balancing wiggle.
// Build + run from the repo root:
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/contact_replay.cpp src/contact.cpp -o /tmp/cr && /tmp/cr
//   MSVC: cl /std:c++17 /O2 /EHsc /Zp1 /FI <a header with #define __attribute__(x)> /I src /I include (same files)
// Optional argument: the CSV (default station/sim/tests/data/contact_replay_20261125.csv).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "contact.h"
#include "protocol/link_protocol.h"

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

// Log time (s). Where the robot really was blocked while commanded to move (by hand from the log, issue #111):
struct Span {
  double t0, t1;
  bool must;  // a push v8 missed: it must be found
  const char* what;
};
static const Span kTrue[] = {
    {358.20, 361.10, true, "backing off at -80 after the 358.0 contact: wheels still 2.8 s"},
    {386.55, 389.30, true, "the dead-zone back-off at -40 into something behind (the BLOCKED loop starts)"},
    {389.30, 401.60, false, "... the rest of that BLOCKED loop (v8 kept backing off into it)"},
    {413.40, 417.40, false, "the stuck back-off at -80 there: 4 s without moving"},
    {436.90, 439.70, true, "backing off at -40: crept back, then still for 1 s"},
    {546.60, 549.20, false, "v8 'stuck, no progress driving for 2.5 s' (wheels still 2 s): its stuck check reports it"},
};
static const int kSpans = sizeof(kTrue) / sizeof(kTrue[0]);

int main(int argc, char** argv) {
  const char* path = argc > 1 ? argv[1] : "station/sim/tests/data/contact_replay_20261125.csv";
  FILE* f = fopen(path, "r");
  if (!f) {
    printf("FAIL: can't open %s (run from the repo root)\n", path);
    return 1;
  }
  char line[256];
  if (!fgets(line, sizeof(line), f)) return 1;  // header
  ContactDetector det;
  int rows = 0, found[kSpans] = {}, false_pos = 0;
  while (fgets(line, sizeof(line), f)) {
    double t;
    unsigned long now;
    int watch, upright, cruise, ramp, drive;
    float cmd, speed, rate, vel;
    if (sscanf(line, "%lf,%lu,%d,%d,%f,%f,%f,%f,%d,%d,%d", &t, &now, &watch, &upright, &cmd, &speed, &rate, &vel, &cruise,
               &ramp, &drive) != 11)
      continue;
    rows++;
    ContactIn in;
    in.now_ms = (uint32_t)now;
    in.watch = watch != 0;
    in.upright = upright != 0;
    in.cmd = cmd;
    in.speed = speed;
    in.pitch_rate = rate;
    in.vel_out = vel;
    in.cruise = cruise != 0;
    in.ramp = ramp != 0;
    in.drive = drive != 0;
    const uint8_t k = det.update(in);
    if (!k) continue;
    int s = -1;
    for (int i = 0; i < kSpans; i++)
      if (t >= kTrue[i].t0 && t <= kTrue[i].t1) s = i;
    printf("  %8.2f s  %s%s %s %4.0f ms  %s\n", t, k == LINK_CONTACT_BUMP ? "bump " : "stuck", det.stall() ? " (stall)" : "",
           det.dir() < 0 ? "rear " : "front", det.value(), s >= 0 ? kTrue[s].what : "<- FALSE POSITIVE");
    if (s >= 0) found[s]++;
    else false_pos++;
  }
  fclose(f);
  printf("%d rows replayed\n", rows);
  CHECK(rows > 10000);
  for (int i = 0; i < kSpans; i++)
    if (kTrue[i].must) CHECK(found[i] > 0);
  CHECK(false_pos == 0);
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
