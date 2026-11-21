// Host test for src/navigator.cpp: a kinematic robot (first-order speed and turn response, like the balance
// controller's ramps) driven by the Navigator, with a simulated range sweep against walls.
// Build + run (any g++, e.g. on the Pi):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/nav_sim.cpp src/navigator.cpp -o /tmp/nav && /tmp/nav
// The Python simulator (station/sim/sim.py, Navigator) is a port of the same code with a balancing model.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "navigator.h"
#include "params.h"

float g_param[P_COUNT];
static const float kDefaults[P_COUNT] = {
#define PARAM_DEF(id, name, def, mn, mx) def,
    PARAM_TABLE(PARAM_DEF)
#undef PARAM_DEF
};

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

struct Seg {
  float x1, y1, x2, y2;
};

struct World {
  Seg segs[8];
  int n = 0;
  void add(float x1, float y1, float x2, float y2) { segs[n++] = {x1, y1, x2, y2}; }
  float ray(float ox, float oy, float a) const {  // distance to the first wall, or -1
    const float dx = cosf(a), dy = sinf(a);
    float best = -1;
    for (int i = 0; i < n; i++) {
      const Seg& s = segs[i];
      const float ex = s.x2 - s.x1, ey = s.y2 - s.y1;
      const float den = dx * ey - dy * ex;
      if (fabsf(den) < 1e-9f) continue;
      const float t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den;
      const float u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
      if (t > 0 && u >= 0 && u <= 1 && (best < 0 || t < best)) best = t;
    }
    return best;
  }
};

struct Bot {
  float x = 0, y = 0, h = 0, v = 0, w = 0;
  uint8_t mode = MODE_GOTO;
  uint32_t ms = 0;
  int k = 0, dir = 1;
  Navigator nav;
  World world;
  bool live = true;

  NavIn in() const {
    NavIn n;
    n.now_ms = ms;
    n.mode = mode;
    n.upright = true;
    n.live = live;
    n.x = x;
    n.y = y;
    n.heading = h;
    n.speed = v;
    n.yaw_rate = w;
    return n;
  }
  void sweep() {  // two sensors, 16 positions over 170 deg, one reading pair per 80 ms
    const float head = -85.0f + k * (170.0f / 15.0f);
    k += dir;
    if (k >= 15 || k <= 0) dir = -dir;
    for (int s = 0; s < 2; s++) {
      const float a = head + (s ? 180.0f : 0.0f);
      const float hr = h * 0.0174533f;
      const float ax = x + 26 * cosf(hr), ay = y + 26 * sinf(hr);
      const float d = world.ray(ax, ay, hr + a * 0.0174533f);
      RangeFrame f = {};
      f.x_mm = x;
      f.y_mm = y;
      f.heading_deg = h;
      f.pitch_deg = -P(P_BAL_OFFSET);  // upright
      f.sensor_angle_deg = fmodf(a + 540.0f, 360.0f) - 180.0f;
      f.sensor_id = s;
      f.valid = d > 0 && d - 12.9f <= 2000 ? 1 : 0;
      f.range_mm = f.valid ? (uint16_t)(d - 12.9f) : 8190;
      nav.addRange(f, ms);
    }
  }
  void run(float seconds) {
    const uint32_t end = ms + (uint32_t)(seconds * 1000);
    while (ms < end) {
      ms += 5;
      if (ms % 80 == 0) sweep();
      nav.update(in());
      // ModeManager clamp + controller ramps (drv.accel 1500, turn 1000) + lag of the balancing loops
      float vt = nav.speed(), wt = nav.turn();
      if (fabsf(wt) < P(P_DRV_TURN_DB)) wt = 0;
      const float dt = 0.005f;
      v += fmaxf(-1500 * dt, fminf(1500 * dt, vt - v)) * 0.5f;
      w += fmaxf(-1000 * dt, fminf(1000 * dt, wt - w)) * 0.5f;
      h += w * dt;
      x += v * cosf(h * 0.0174533f) * dt;
      y += v * sinf(h * 0.0174533f) * dt;
      while (const char* l = nav.peekLog()) {
        printf("  %6.2f s  %s\n", ms / 1000.0, l);
        nav.popLog();
      }
    }
  }
};

static void reset() { memcpy(g_param, kDefaults, sizeof(g_param)); }

static void testThreeWaypoints() {
  printf("goto: 3 waypoints\n");
  reset();
  Bot b;
  b.nav.add(800, 0);
  b.nav.add(800, 500);
  b.nav.add(0, 500);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(60);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(b.nav.index() == 3);
  CHECK(hypotf(b.x - 0, b.y - 500) < 2 * P(P_NAV_ARRIVE_MM));
  CHECK(fabsf(b.v) < 5);
  printf("  end %.0f, %.0f  heading %.1f\n", b.x, b.y, b.h);
}

static void testEndHeading() {
  printf("goto: end heading 90\n");
  reset();
  g_param[P_NAV_END_HEADING] = 90;
  Bot b;
  b.nav.add(500, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(30);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(fabsf(b.h - 90) < 5);
}

static void testObstacle(uint8_t mode) {
  printf("%s: wall at x = 1000\n", mode == MODE_GOTO ? "goto" : "explore");
  reset();
  Bot b;
  b.mode = mode;
  b.world.add(1000, -500, 1000, 500);
  b.nav.add(2000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(20);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.nav.obstacle());
  CHECK(fabsf(b.v) < 5);
  const float gap = 1000 - b.x - 26 - 12.9f;  // what the front sensor reads
  printf("  stopped at x %.0f, sensor gap %.0f mm, obstacle_mm %.0f\n", b.x, gap, b.nav.obstacleMm());
  CHECK(gap > P(P_RNG_OBSTACLE_MM) - 60 && gap < P(P_RNG_OBSTACLE_MM) + 10);
  b.world.n = 0;  // wall gone
  b.run(3);
  if (mode == MODE_GOTO) {
    CHECK(b.nav.state() == NAV_DRIVING);
  } else {
    CHECK(b.nav.state() == NAV_BLOCKED);  // waits for a new list
    b.nav.clearList();
    b.nav.add(b.x, 400);
    b.nav.listChanged(true);
    b.run(20);
    CHECK(b.nav.state() == NAV_ARRIVED);
  }
}

static void testUnsupervised() {
  printf("goto: no supervisor -> no motion\n");
  reset();
  Bot b;
  b.live = false;
  b.nav.add(800, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(3);
  CHECK(fabsf(b.x) < 1 && b.nav.speed() == 0 && b.nav.turn() == 0);
}

static void testHold() {
  printf("hold: pushed 200 mm\n");
  reset();
  Bot b;
  b.mode = MODE_HOLD;
  b.nav.onModeChange(b.in());
  b.x = 200;
  b.run(5);
  CHECK(b.nav.state() == NAV_HOLDING);
  CHECK(fabsf(b.x) < P(P_NAV_HOLD_DB_MM) + 5);
  printf("  back to x %.1f\n", b.x);
}

static void testStatusDue() {
  printf("status cadence\n");
  reset();
  Bot b;
  CHECK(b.nav.statusDue(0));  // first one
  b.nav.statusSent(0);
  b.mode = MODE_BALANCE;
  b.nav.update(b.in());
  CHECK(!b.nav.statusDue(1000));  // not navigating, nothing changed
  b.nav.add(100, 0);
  CHECK(b.nav.statusDue(1001));   // count changed
  b.nav.statusSent(1001);
  b.mode = MODE_GOTO;
  b.nav.onModeChange(b.in());
  b.ms = 1002;
  b.nav.update(b.in());
  CHECK(b.nav.statusDue(1002));   // state idle -> driving / turning
  b.nav.statusSent(1002);
  CHECK(!b.nav.statusDue(1400));
  CHECK(b.nav.statusDue(1502));   // 2 Hz
}

int main() {
  testThreeWaypoints();
  testEndHeading();
  testObstacle(MODE_GOTO);
  testObstacle(MODE_EXPLORE);
  testUnsupervised();
  testHold();
  testStatusDue();
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
