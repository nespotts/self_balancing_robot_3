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
  bool jam = false;     // wheels blocked: the pose does not change (pushing against a wall)
  float min_wall_y = 1e9;  // closest |y - wall_y| seen (side wall tests)
  float wall_y = 0;
  bool sweep_on = true;    // VL53L0X sweep readings every 80 ms
  bool lidar_on = false;   // LD19 frames (MSG_LIDAR_OBSTACLES, #77) every 100 ms
  bool lworld_own = false; // the lidar sees lworld instead of world
  World lworld;
  uint8_t lseq = 0;
  uint32_t lidar_frames = 0;

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
  // what the bridge sends (lidar_obstacles.py): lidar at the axle, 1 deg rays, nearest return 120..8000 mm per
  // 10 deg sector, robot frame, pose of now (age 0)
  void lidar() {
    const World& wd = lworld_own ? lworld : world;
    LidarObstacleFrame f = {};
    f.seq = lseq++;
    f.sectors = LINK_LIDAR_SECTORS;
    const float hr = h * 0.0174533f;
    for (int s = 0; s < LINK_LIDAR_SECTORS; s++) {
      float best = -1, bx = 0, by = 0;
      for (int k = 0; k < 10; k++) {
        const float a = (s * 10.0f - 4.5f + k) * 0.0174533f;
        const float d = wd.ray(x, y, hr + a);
        if (d < 120 || d > 8000 || (best > 0 && d >= best)) continue;
        best = d;
        bx = d * cosf(a);
        by = d * sinf(a);
      }
      if (best > 0) {
        f.xy_mm[2 * s] = (int16_t)lroundf(bx);
        f.xy_mm[2 * s + 1] = (int16_t)lroundf(by);
      }
    }
    if (nav.addLidar(f, in())) lidar_frames++;
  }
  void run(float seconds) {
    const uint32_t end = ms + (uint32_t)(seconds * 1000);
    while (ms < end) {
      ms += 5;
      if (sweep_on && ms % 80 == 0) sweep();
      if (lidar_on && ms % 100 == 0) lidar();
      nav.update(in());
      // ModeManager clamp + controller ramps (drv.accel 1500, turn 1000) + lag of the balancing loops
      float vt = nav.speed(), wt = nav.turn();
      if (fabsf(wt) < P(P_DRV_TURN_DB)) wt = 0;
      const float dt = 0.005f;
      v += fmaxf(-1500 * dt, fminf(1500 * dt, vt - v)) * 0.5f;
      w += fmaxf(-1000 * dt, fminf(1000 * dt, wt - w)) * 0.5f;
      h += w * dt;
      if (!jam) {
        x += v * cosf(h * 0.0174533f) * dt;
        y += v * sinf(h * 0.0174533f) * dt;
      }
      if (fabsf(y - wall_y) < min_wall_y) min_wall_y = fabsf(y - wall_y);
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

// issue #62: driving parallel to a wall 105 mm from the centre line (29 mm from the body side)
static void testSideWall(bool side_on) {
  printf("goto: wall alongside, side check %s\n", side_on ? "on" : "off");
  reset();
  if (!side_on) g_param[P_NAV_SIDE_MIN_MM] = 0;
  Bot b;
  b.world.add(-1000, -105, 4000, -105);
  b.wall_y = -105;
  b.nav.add(2500, 60);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(25);
  printf("  at %.0f, %.0f, closest to the wall %.0f mm (centre)\n", b.x, b.y, b.min_wall_y);
  if (side_on) {
    CHECK(b.nav.state() == NAV_ARRIVED);
    CHECK(b.min_wall_y > 76 + 15);  // body side never closer than 15 mm
  } else {
    CHECK(b.nav.state() == NAV_BLOCKED);  // the old corridor: the wall alongside is "in the way"
    CHECK(b.x < 500);
  }
}

static void testStuck() {
  printf("goto: wheels jammed -> stuck, back off\n");
  reset();
  Bot b;
  b.nav.add(3000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(2);
  b.jam = true;
  const float x0 = b.x;
  b.run(P(P_NAV_STUCK_MS) / 1000.0f + 0.5f);
  CHECK(b.nav.stuck());
  CHECK(b.nav.state() == NAV_BLOCKED);
  b.jam = false;
  b.run(0.4f);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.v < 0);  // backing off
  for (int i = 0; i < 100 && b.nav.state() == NAV_BLOCKED; i++) b.run(0.05f);
  printf("  backed off %.0f mm\n", x0 - b.x);
  CHECK(x0 - b.x > 80 && x0 - b.x < 140);
  CHECK(!b.nav.stuck());  // GOTO goes on
  b.run(20);
  CHECK(b.nav.state() == NAV_ARRIVED);
}

// ---- LD19 lidar sectors (#77): no sweep at all (rng.enable 0), obstacles only from MSG_LIDAR_OBSTACLES

static void lidarOnly(Bot& b) {
  b.sweep_on = false;
  b.lidar_on = true;
}

static void testLidarObstacle(uint8_t mode) {
  printf("%s, lidar only: wall at x = 1000\n", mode == MODE_GOTO ? "goto" : "explore");
  reset();
  Bot b;
  lidarOnly(b);
  b.mode = mode;
  b.world.add(1000, -500, 1000, 500);
  b.run(0.3);  // frames flowing before the mode starts
  CHECK(b.nav.lidarFresh(b.ms));
  b.nav.add(2000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  float vmax_slow = 0;  // fastest command while the wall is inside nav.slow_mm
  float worst = 0;      // speed over the slow-down limit
  for (int i = 0; i < 4000; i++) {
    b.run(0.005f);
    const float d = b.nav.frontMm();
    if (d > P(P_RNG_OBSTACLE_MM) && d < P(P_NAV_SLOW_MM)) {
      const float lim = fmaxf(P(P_NAV_MIN_SPEED), P(P_NAV_MAX_SPEED) * (d - P(P_RNG_OBSTACLE_MM)) /
                                                      (P(P_NAV_SLOW_MM) - P(P_RNG_OBSTACLE_MM)));
      vmax_slow = fmaxf(vmax_slow, b.nav.speed());
      worst = fmaxf(worst, b.nav.speed() - lim);
    }
  }
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.nav.obstacle());
  CHECK(fabsf(b.v) < 5);
  CHECK(b.x > 400);              // it drove: never paused blind
  CHECK(vmax_slow > 0);          // and it did slow down on the way in
  CHECK(worst < 1);
  const float gap = 1000 - b.x - 26 - 12.9f;
  printf("  stopped at x %.0f, gap %.0f mm, obstacle_mm %.0f, %u frames\n", b.x, gap, b.nav.obstacleMm(),
         (unsigned)b.lidar_frames);
  CHECK(gap > P(P_RNG_OBSTACLE_MM) - 60 && gap < P(P_RNG_OBSTACLE_MM) + 10);
  b.world.n = 0;  // wall gone
  b.run(3);
  if (mode == MODE_GOTO) {
    CHECK(b.nav.state() == NAV_DRIVING);
  } else {
    CHECK(b.nav.state() == NAV_BLOCKED);  // waits for a new list
  }
}

static void testLidarSideWall() {
  printf("goto, lidar only: wall alongside\n");
  reset();
  Bot b;
  lidarOnly(b);
  b.world.add(-1000, -105, 4000, -105);
  b.wall_y = -105;
  b.run(0.3);
  b.nav.add(2500, 60);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(25);
  printf("  at %.0f, %.0f, closest to the wall %.0f mm (centre)\n", b.x, b.y, b.min_wall_y);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(b.min_wall_y > 76 + 15);
}

static void testLidarStale() {
  printf("lidar only: frames stop -> blind (GOTO capped, EXPLORE paused), back -> normal\n");
  reset();
  Bot b;
  lidarOnly(b);
  b.run(0.3);
  b.nav.add(5000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(4);
  CHECK(b.nav.state() == NAV_DRIVING);
  CHECK(b.v > P(P_NAV_BLIND_SPEED) + 50);  // full nav.max_speed with the lidar
  b.lidar_on = false;
  b.run(0.4);
  CHECK(b.nav.lidarFresh(b.ms));           // < LINK_LIDAR_FRESH_MS: still fresh
  b.run(0.3);
  CHECK(!b.nav.lidarFresh(b.ms));
  b.run(2);
  CHECK(b.nav.state() == NAV_DRIVING);
  CHECK(b.nav.speed() <= P(P_NAV_BLIND_SPEED) + 0.01f);  // GOTO blind: nav.blind_speed
  b.mode = MODE_EXPLORE;
  b.nav.onModeChange(b.in());
  b.run(1);
  CHECK(b.nav.speed() == 0 && b.nav.turn() == 0);       // EXPLORE blind: paused
  b.lidar_on = true;
  b.run(1.5);
  CHECK(b.nav.speed() > P(P_NAV_BLIND_SPEED));          // lidar back: full speed again
  // a frame older than LINK_LIDAR_MAX_AGE_MS or with another sector count is dropped
  LidarObstacleFrame f = {};
  f.sectors = LINK_LIDAR_SECTORS;
  f.age_ms = LINK_LIDAR_MAX_AGE_MS + 1;
  CHECK(!b.nav.addLidar(f, b.in()));
  f.age_ms = 0;
  f.sectors = 16;
  CHECK(!b.nav.addLidar(f, b.in()));
  // pose reset: the lidar points were in the old frame -> not fresh until the next frame
  b.nav.onPoseReset();
  CHECK(!b.nav.lidarFresh(b.ms));
}

static void testLidarAge() {
  printf("lidar: age_ms dead-reckons the pose back\n");
  reset();
  Navigator nav;
  NavIn in;
  in.now_ms = 1000;
  in.mode = MODE_GOTO;
  in.upright = true;
  in.speed = 200;  // mm/s forward, heading 0, at x = 0
  LidarObstacleFrame f = {};
  f.sectors = LINK_LIDAR_SECTORS;
  f.age_ms = 100;  // the scan's robot frame was 20 mm behind
  f.xy_mm[0] = 400;  // sector 0: 400 mm ahead of the axle back then
  CHECK(nav.addLidar(f, in));
  nav.update(in);  // front = x - 26 - 12.9 with x = 400 - 20
  printf("  front %.1f mm\n", nav.frontMm());
  CHECK(fabsf(nav.frontMm() - (380 - 38.9f)) < 0.5f);
}

static void testLidarWins() {
  printf("lidar fresh wins over the sweep (sweep sees a wall the lidar does not)\n");
  reset();
  Bot b;
  b.lidar_on = true;  // sweep on too
  b.lworld_own = true;
  b.world.add(800, -500, 800, 500);  // only the sweep sees this
  b.run(0.5);
  b.nav.add(1500, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(15);
  CHECK(b.nav.state() == NAV_ARRIVED);
  printf("  at %.0f (the sweep's wall at 800)\n", b.x);
  b.mode = MODE_BALANCE;  // stand still
  b.nav.onModeChange(b.in());
  b.run(0.2);
  CHECK(b.nav.rearMm() == 0);  // lidar: nothing behind
  b.lidar_on = false;          // lidar gone: the sweep's points count again
  b.run(0.6);
  CHECK(!b.nav.lidarFresh(b.ms));
  printf("  lidar stale: rear %.0f mm (sweep)\n", b.nav.rearMm());
  CHECK(b.nav.rearMm() > 600 && b.nav.rearMm() < 800);  // the sweep's wall at 800, behind
}

static void testLidarStuck() {
  printf("goto, lidar only: wheels jammed -> stuck, back off\n");
  reset();
  Bot b;
  lidarOnly(b);
  b.run(0.3);
  b.nav.add(3000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(2);
  b.jam = true;
  b.run(P(P_NAV_STUCK_MS) / 1000.0f + 0.5f);
  CHECK(b.nav.stuck());
  CHECK(b.nav.state() == NAV_BLOCKED);
  b.jam = false;
  b.run(0.4f);
  CHECK(b.v < 0);  // backing off
  b.run(20);
  CHECK(b.nav.state() == NAV_ARRIVED);
}

// ---- lidar stuck (#79, EVT_LINK_STUCK): spinning wheels look like progress to the odometry; the bridge says "stuck"
static void testLidarStuckEvent() {
  printf("goto: wheels spin (odometry moves), bridge sends EVT_LINK_STUCK -> stuck, back off\n");
  reset();
  Bot b;
  b.nav.add(3000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(2);
  CHECK(b.nav.state() == NAV_DRIVING);
  b.nav.onLidarStuck(b.ms);
  b.run(0.05f);
  CHECK(b.nav.stuck());
  CHECK(b.nav.state() == NAV_BLOCKED);
  b.run(0.4f);
  CHECK(b.v < 0);  // backing off
  b.run(20);
  CHECK(b.nav.state() == NAV_ARRIVED);  // GOTO goes on

  printf("  not driving (turning in place / arrived): the event is dropped after 500 ms\n");
  reset();
  Bot c;
  c.nav.add(0, 2000);  // 90 deg to the left: turns in place first
  c.nav.listChanged(true);
  c.nav.onModeChange(c.in());
  c.run(0.1f);
  CHECK(c.nav.state() == NAV_TURNING);
  c.nav.onLidarStuck(c.ms);
  c.run(0.3f);
  CHECK(!c.nav.stuck());
  c.run(0.5f);               // driving now, but the event is too old
  CHECK(!c.nav.stuck());
  c.run(15);
  CHECK(c.nav.state() == NAV_ARRIVED);

  printf("  nav.stuck_ms 0 (stuck checks off): ignored\n");
  reset();
  g_param[P_NAV_STUCK_MS] = 0;
  Bot d;
  d.nav.add(3000, 0);
  d.nav.listChanged(true);
  d.nav.onModeChange(d.in());
  d.run(2);
  d.nav.onLidarStuck(d.ms);
  d.run(0.1f);
  CHECK(!d.nav.stuck());
  CHECK(d.nav.state() == NAV_DRIVING);
}

int main() {  testThreeWaypoints();
  testEndHeading();
  testObstacle(MODE_GOTO);
  testObstacle(MODE_EXPLORE);
  testUnsupervised();
  testHold();
  testStatusDue();
  testSideWall(false);
  testSideWall(true);
  testStuck();
  testLidarObstacle(MODE_GOTO);
  testLidarObstacle(MODE_EXPLORE);
  testLidarSideWall();
  testLidarStale();
  testLidarAge();
  testLidarWins();
  testLidarStuck();
  testLidarStuckEvent();
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
