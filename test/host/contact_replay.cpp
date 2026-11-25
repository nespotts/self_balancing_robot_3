// Host test for src/contact.cpp (#111): the real robot's session of 2026-11-25 (v8 firmware) replayed through the C++
// contact detector - the detector's inputs rebuilt from its 100 Hz telemetry (station/tools/contact_replay.py extract;
// station/sim/tests/test_contact.py replays the same file through the Python port).
//   - the 3 pushes v8 missed (backing off at -40 / -80 mm/s into something, the wheels still, the lean growing) are found;
//   - nothing anywhere else: starts, stops, turns, slow corners, the balancing wiggle.
// Build + run from the repo root:
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/contact_replay.cpp src/contact.cpp -o /tmp/cr && /tmp/cr
//   MSVC: cl /std:c++17 /O2 /EHsc /Zp1 /FI <a header with #define __attribute__(x)> /I src /I include (same files)
// Optional argument: the folder of the CSVs (default station/sim/tests/data). A second session (18:36, push-detect on
// the robot): a slow push into a low box just before the last point, which it missed - found within 1.5 s now.
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
  bool must;     // a push it missed: it must be found ...
  double by;     // ... by this log time (0: anywhere in the span)
  const char* what;
};
// 14:46 session (v8): 3 pushes v8 missed while backing off
static const Span kTrueA[] = {
    {358.20, 361.10, true, 0, "backing off at -80 after the 358.0 contact: wheels still 2.8 s"},
    {386.55, 389.30, true, 0, "the dead-zone back-off at -40 into something behind (the BLOCKED loop starts)"},
    {389.30, 401.60, false, 0, "... the rest of that BLOCKED loop (v8 kept backing off into it)"},
    {413.40, 417.40, false, 0, "the stuck back-off at -80 there: 4 s without moving"},
    {436.90, 439.70, true, 0, "backing off at -40: crept back, then still for 1 s"},
    {546.60, 549.20, false, 0, "v8 'stuck, no progress driving for 2.5 s' (wheels still 2 s)"},
};
// 18:36 session (push-detect flashed): GOTO into a low box 135 mm before the last point at 40 mm/s, missed (the
// stall rule skipped slow driving / slowing for a stop, the stuck check is off within 3 x arrive_mm): the wheels at 0
// from 132.9 s for 9 s while it leaned in, then spinning / rocking until the slip moved its odometry onto the goal
// ... and in the rest of it (370-700 s, 940-980 s) real forward contacts in a pocket between two low things (it drove in
// at 100-250 mm/s and stopped dead) - and ~20 back-offs that rocked after the bounce, which the robot called "rear,
// holding": none of those may be a rear contact (no_rear)
static const Span kTrueB[] = {
    {132.80, 150.30, true, 134.40, "40 mm/s into a low box before the last point: wheels still from 132.9 s"},
    {376.00, 420.00, false, 0, "the pocket at (0.8, -1.3): drove in at 120-150 mm/s, stopped dead (front)"},
    {950.00, 965.00, false, 0, "drove in at 250 mm/s, stopped dead (front)"},
};

static int replay(const char* path, const Span* spans, int n, bool no_rear = false) {
  FILE* f = fopen(path, "r");
  if (!f) {
    printf("FAIL: can't open %s (run from the repo root)\n", path);
    return 1;
  }
  printf("%s\n", path);
  char line[256];
  if (!fgets(line, sizeof(line), f)) return 1;  // header
  ContactDetector det;
  int rows = 0, found[8] = {}, false_pos = 0;
  double first[8] = {};
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
    for (int i = 0; i < n; i++)
      if (t >= spans[i].t0 && t <= spans[i].t1) s = i;
    printf("  %8.2f s  %s%s %s %4.0f ms  %s\n", t, k == LINK_CONTACT_BUMP ? "bump " : "stuck", det.stall() ? " (stall)" : "",
           det.dir() < 0 ? "rear " : "front", det.value(), s >= 0 ? spans[s].what : "<- FALSE POSITIVE");
    if (no_rear && det.dir() < 0) {
      printf("    ^ a rear contact in a back-off that rocked: FALSE\n");
      false_pos++;
    } else if (s >= 0) {
      if (!found[s]++) first[s] = t;
    } else {
      false_pos++;
    }
  }
  fclose(f);
  printf("  %d rows replayed\n", rows);
  CHECK(rows > 10000);  // (the 18:36 file: 125-160 s, 365-700 s, 940-980 s of the session)
  for (int i = 0; i < n; i++) {
    if (spans[i].must) CHECK(found[i] > 0);
    if (spans[i].must && spans[i].by > 0) CHECK(found[i] > 0 && first[i] <= spans[i].by);
  }
  CHECK(false_pos == 0);
  return 0;
}

int main(int argc, char** argv) {
  const char* dir = argc > 1 ? argv[1] : "station/sim/tests/data";
  char a[512], b[512];
  snprintf(a, sizeof(a), "%s/contact_replay_20261125.csv", dir);
  snprintf(b, sizeof(b), "%s/contact_replay_20261125b.csv", dir);
  replay(a, kTrueA, sizeof(kTrueA) / sizeof(kTrueA[0]));
  replay(b, kTrueB, sizeof(kTrueB) / sizeof(kTrueB[0]), true);
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
