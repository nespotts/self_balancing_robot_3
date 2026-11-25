// Host test for src/navigator.cpp: a kinematic robot (first-order speed and turn response, like the balance
// controller's ramps) driven by the Navigator, with a simulated range sweep against walls.
// Build + run (any g++, e.g. on the Pi):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/nav_sim.cpp src/navigator.cpp src/contact.cpp -o /tmp/nav && /tmp/nav
//   MSVC: cl /std:c++17 /O2 /EHsc /Zp1 /FI <a header with #define __attribute__(x)> /I src /I include ... (same files)
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
  void post(float cx, float cy, float w) {  // a square post, w mm wide (4 segments)
    const float a = 0.5f * w;
    add(cx - a, cy - a, cx + a, cy - a);
    add(cx + a, cy - a, cx + a, cy + a);
    add(cx + a, cy + a, cx - a, cy + a);
    add(cx - a, cy + a, cx - a, cy - a);
  }
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
  // pass-through stats (#103)
  uint8_t last_idx = 0;
  float pass_v[WAYPOINT_MAX] = {};  // measured speed when the index moved on from waypoint i
  uint32_t pass_ms[WAYPOINT_MAX] = {};
  float max_turn_cmd = 0;           // |turn| commanded
  float max_rise = 0, max_drop = 0; // speed command steps per 5 ms (driving, not blocked)
  float prev_cmd = 0;
  uint32_t spin_ms = 0;             // turning in place: TURNING, |w| > 20 deg/s, |v| < 60 mm/s
  float path_len = 0;
  // contact detection (#104), dyn = true: the balance loop's signals. The controller's ramped command (vt_ramp), the
  // encoders' speed (40 ms lag, 0 while stalled), the velocity loop's lean (vel.kp / ki), the pitch rate (noise_dps of
  // pseudo-random noise, plus 0.45 deg/s per mm/s for 30 ms when the wheels stop at low_x: what a sudden stop throws
  // the body by). A low obstacle at x = low_x (the body's front, 45 mm ahead of the axle, stops there).
  bool dyn = false;
  bool upright = true;
  float low_x = 1e9f;
  float low_y0 = -1e9f, low_y1 = 1e9f;  // ... from y low_y0 to low_y1
  float vt_ramp = 0, speed_meas = 0, vel_i = 0, vel_out = 0, pitch_rate = 0, noise_dps = 0;
  uint32_t spike_until = 0, impact_ms = 0;
  float spike = 0;
  bool stalled = false;
  float station_v = 0;  // MODE_STATION: the command (cmd_vel)
  uint32_t rnd = 12345;

  NavIn in() const {
    NavIn n;
    n.now_ms = ms;
    n.mode = mode;
    n.upright = upright;
    n.live = live;
    n.x = x;
    n.y = y;
    n.heading = h;
    n.speed = dyn ? speed_meas : v;
    n.yaw_rate = w;
    if (dyn) {
      n.pitch_rate = pitch_rate;
      n.cmd_speed = vt_ramp;
      n.vel_out = vel_out;
    }
    return n;
  }
  float noise() {  // -1..1
    rnd = rnd * 1103515245u + 12345u;
    return ((rnd >> 8) & 0xFFFF) / 32768.0f - 1.0f;
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
  // what the bridge sends (lidar_obstacles.py): lidar at the axle, 1 deg rays, returns 120..8000 mm, per 10 deg
  // sector the nearest one inside the corridor (|y| <= 86, by the firmware's front / rear distance) else the
  // nearest (#99), robot frame, pose of now (age 0)
  void lidar() {
    const World& wd = lworld_own ? lworld : world;
    LidarObstacleFrame f = {};
    f.seq = lseq++;
    f.sectors = LINK_LIDAR_SECTORS;
    const float hr = h * 0.0174533f;
    for (int s = 0; s < LINK_LIDAR_SECTORS; s++) {
      float best = 0, bx = 0, by = 0;
      bool have = false, best_in = false;
      for (int k = 0; k < 10; k++) {
        const float a = (s * 10.0f - 4.5f + k) * 0.0174533f;
        const float d = wd.ray(x, y, hr + a);
        if (d < 120 || d > 8000) continue;
        const float px = d * cosf(a), py = d * sinf(a);
        const bool in = fabsf(py) <= 86.0f;
        const float key = in ? (px >= 26 ? px - 38.9f : 13.1f - px) : d;
        if (have && (best_in > in || (best_in == in && key >= best))) continue;
        have = true;
        best = key;
        best_in = in;
        bx = px;
        by = py;
      }
      if (have) {
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
      float vt = mode == MODE_STATION ? nav.limitSpeed(station_v, 450, true) : nav.speed(), wt = nav.turn();
      if (fabsf(wt) < P(P_DRV_TURN_DB)) wt = 0;
      const float dt = 0.005f;
      v += fmaxf(-1500 * dt, fminf(1500 * dt, vt - v)) * 0.5f;
      w += fmaxf(-1000 * dt, fminf(1000 * dt, wt - w)) * 0.5f;
      h += w * dt;
      float moved = 0;
      if (!jam) {
        const float nx = x + v * cosf(h * 0.0174533f) * dt;
        if (dyn && v >= 0 && nx + 45.0f >= low_x && y >= low_y0 && y <= low_y1) {  // the body's front at it: the wheels stop
          if (!stalled && v > 0) {
            spike = 0.45f * v;
            spike_until = ms + 30;
            if (!impact_ms) impact_ms = ms;  // (the first one)
          }
          stalled = true;
          v = 0;
        } else {
          stalled = false;
          moved = v;
          x = nx;
          y += v * sinf(h * 0.0174533f) * dt;
          path_len += fabsf(v) * dt;
        }
      }
      if (dyn) {
        vt_ramp += fmaxf(-1500 * dt, fminf(1500 * dt, vt - vt_ramp));
        speed_meas += (moved - speed_meas) * dt / 0.04f;
        const float e = vt_ramp - speed_meas;
        vel_i = fmaxf(-20, fminf(20, vel_i + P(P_VEL_KI) * e * dt));
        vel_out = fmaxf(-20, fminf(20, P(P_VEL_KP) * e + vel_i));
        pitch_rate = noise_dps * noise() + (ms < spike_until ? spike : 0.0f);
      }
      if (nav.index() != last_idx && nav.index() > last_idx && last_idx < WAYPOINT_MAX) {
        pass_v[last_idx] = v;
        pass_ms[last_idx] = ms;
      }
      last_idx = nav.index();
      max_turn_cmd = fmaxf(max_turn_cmd, fabsf(nav.turn()));
      if (nav.state() == NAV_DRIVING && prev_cmd > 0 && nav.speed() > 0 && nav.index() + 1 < nav.count()) {
        max_rise = fmaxf(max_rise, nav.speed() - prev_cmd);
        max_drop = fmaxf(max_drop, prev_cmd - nav.speed());
      }
      prev_cmd = nav.speed();
      if (nav.state() == NAV_TURNING && fabsf(w) > 20 && fabsf(v) < 60) spin_ms += 5;
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

static void testLidarUnion() {
  printf("lidar + sweep both fresh: the union (#99), the sweep sees a wall the lidar does not, then the reverse\n");
  reset();
  Bot b;
  b.lidar_on = true;  // sweep on too
  b.lworld_own = true;
  b.world.add(800, -500, 800, 500);  // only the sweep sees this
  b.run(0.5);
  CHECK(b.nav.lidarFresh(b.ms));
  b.nav.add(1500, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(15);
  CHECK(b.nav.state() == NAV_BLOCKED);  // the sweep's wall counts although the lidar is fresh
  const float gap = 800 - b.x - 26 - 12.9f;
  printf("  stopped at x %.0f for the sweep's wall, gap %.0f mm\n", b.x, gap);
  CHECK(gap > P(P_RNG_OBSTACLE_MM) - 60 && gap < P(P_RNG_OBSTACLE_MM) + 10);
  b.world.n = 0;                       // the reverse: only the lidar sees it
  b.lworld.add(800, -500, 800, 500);
  b.run(5);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.x < 800 - 26 - 12.9f - P(P_RNG_OBSTACLE_MM) + 10);
  b.lworld.n = 0;                      // gone for both
  b.run(15);
  CHECK(b.nav.state() == NAV_ARRIVED);
  b.mode = MODE_BALANCE;               // stand still: a wall behind that only the lidar sees
  b.nav.onModeChange(b.in());
  b.lworld.add(1000, -500, 1000, 500);
  b.run(0.3);
  printf("  lidar fresh: rear %.0f mm (the lidar's wall at 1000)\n", b.nav.rearMm());
  CHECK(b.nav.rearMm() > 400 && b.nav.rearMm() < 600);
  b.lidar_on = false;                  // lidar gone: its points no longer count, the sweep sees nothing
  b.run(0.6);
  CHECK(!b.nav.lidarFresh(b.ms));
  CHECK(b.nav.rearMm() == 0);
}

// #99 (a): a narrow post comes into the lidar's dead zone (< 120 mm) while the robot is BLOCKED by it. It must stay
// blocked (the old latch let go after nav.obst_age_ms of "no return"), and with nav.stuck_backoff_mm it backs off so
// the post shows again; GOTO goes on once it is really gone.
static void approachPost(Bot& b, float& px, float stop_at) {
  while (px - 10 - b.x > stop_at) {  // a leg stepping closer: 5 mm per 50 ms
    px -= 5;
    b.world.n = 0;
    b.world.post(px, 0, 20);
    b.run(0.05f);
  }
}

static void testLidarDeadZone() {
  printf("goto, lidar only: a narrow post comes into the lidar's dead zone while blocked (#99)\n");
  for (int backoff = 0; backoff < 2; backoff++) {
    printf("  nav.stuck_backoff_mm %s\n", backoff ? "100 (default)" : "0");
    reset();
    if (!backoff) g_param[P_NAV_STUCK_BACK] = 0;
    Bot b;
    lidarOnly(b);
    float px = 900;
    b.world.post(px, 0, 20);
    b.run(0.3);
    b.nav.add(2500, 0);
    b.nav.listChanged(true);
    b.nav.onModeChange(b.in());
    b.run(15);
    CHECK(b.nav.state() == NAV_BLOCKED);
    const float x0 = b.x;
    approachPost(b, px, 100);  // the post's face 100 mm from the axle: no return (lidar min range 120)
    printf("    post face %.0f mm from the axle, front %.0f mm\n", px - 10 - b.x, b.nav.frontMm());
    if (!backoff) {
      CHECK(b.nav.frontMm() > 0 && b.nav.frontMm() < 141);  // the ghost, not "nothing"
      b.run(5);  // > nav.obst_age_ms of fresh frames with no return from it
      CHECK(b.nav.state() == NAV_BLOCKED);
      CHECK(b.x < x0 + 2);
      CHECK(b.nav.speed() <= 0);
      b.nav.listChanged(true);  // the planner sends the route again: still blocked
      b.run(3);
      CHECK(b.nav.state() == NAV_BLOCKED);
      CHECK(b.x < x0 + 2);
      CHECK(px - 10 - b.x > 95);  // never closer than the post put itself
    } else {
      float xmin = b.x;
      for (int i = 0; i < 100; i++) {
        b.run(0.05f);
        xmin = fminf(xmin, b.x);
      }
      printf("    backed off %.0f mm, post face now %.0f mm, front %.0f mm\n", x0 - xmin, px - 10 - b.x,
             b.nav.frontMm());
      CHECK(px - 10 - xmin > 180);  // backed off far enough for the lidar to see it again
      CHECK(b.nav.state() == NAV_BLOCKED);
      CHECK(b.nav.frontMm() > 141 && b.nav.frontMm() < P(P_RNG_OBSTACLE_MM));  // seen again, still in the way
      b.run(5);
      CHECK(b.nav.state() == NAV_BLOCKED);
      CHECK(px - 10 - b.x > 140);
      b.world.n = 0;  // gone
      b.run(20);
      CHECK(b.nav.state() == NAV_ARRIVED);
    }
  }
}

// #99 (b): the lidar stops while BLOCKED (no sweep). The old latch let go after nav.obst_age_ms and GOTO drove at
// nav.blind_speed into the obstacle. Now: never released without fresh data (also not by a new list); STATION can
// still back away; lidar back + way clear -> goes on.
static void testLidarLostBlocked() {
  printf("goto, lidar only: lidar lost while blocked (#99)\n");
  reset();
  Bot b;
  lidarOnly(b);
  b.world.add(1000, -500, 1000, 500);
  b.run(0.3);
  b.nav.add(2000, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  b.run(15);
  CHECK(b.nav.state() == NAV_BLOCKED);
  const float x0 = b.x;
  b.lidar_on = false;
  b.run(5);
  CHECK(!b.nav.lidarFresh(b.ms));
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.x < x0 + 2);
  CHECK(b.nav.obstacle());
  b.nav.clearList();  // the planner replans (same route)
  b.nav.add(2000, 0);
  b.nav.listChanged(true);
  b.run(3);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.x < x0 + 2);
  printf("  blind 8 s: stayed at x %.0f (blocked at %.0f)\n", b.x, x0);
  b.mode = MODE_STATION;  // the station drives: forward still blocked, backwards allowed
  b.nav.onModeChange(b.in());
  b.run(0.1f);
  CHECK(b.nav.limitSpeed(200, 450, true) == 0);
  CHECK(b.nav.limitSpeed(-100, 450, true) == -100);
  b.mode = MODE_GOTO;
  b.nav.onModeChange(b.in());
  b.lidar_on = true;  // back, the wall still there: still blocked
  b.run(3);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.x < x0 + 2);
  b.world.n = 0;  // gone: goes on
  b.run(3);
  CHECK(b.nav.state() == NAV_DRIVING);
}

// #99: ghosts keep their place around the robot across a pose reset
static void testGhostPoseReset() {
  printf("lidar: a ghost survives a pose reset in the new frame\n");
  reset();
  Bot b;
  lidarOnly(b);
  b.x = 500;
  b.y = 300;
  b.h = 90;
  b.world.post(500, 300 + 150 + 10, 20);  // 150 mm ahead (heading 90)
  b.run(0.3);
  b.world.n = 0;  // now "no return" (as if inside the min range)
  b.lidar_on = false;
  b.nav.update(b.in());
  const float f0 = b.nav.frontMm();
  CHECK(f0 > 100 && f0 < 141);
  b.nav.onPoseReset();
  b.x = b.y = 0;
  b.h = 0;
  b.nav.update(b.in());
  printf("  front %.1f mm before, %.1f after the reset\n", f0, b.nav.frontMm());
  CHECK(fabsf(b.nav.frontMm() - f0) < 1);
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

// ---- pass-through waypoints (#103)

// the robot's live values (2026-11-24)
static void liveParams() {
  g_param[P_NAV_ARRIVE_MM] = 120;
  g_param[P_NAV_DWELL_MS] = 100;
  g_param[P_NAV_STOP_SPEED] = 90;
  g_param[P_NAV_STOP_TURN] = 45;
  g_param[P_NAV_ALIGN_DEG] = 20;
  g_param[P_NAV_MAX_SPEED] = 250;
  g_param[P_NAV_MIN_SPEED] = 40;
  g_param[P_RNG_OBSTACLE_MM] = 160;
  g_param[P_NAV_SLOW_MM] = 450;
}

// 5 points from (0,0) heading 0: corners 30, 45, 60 deg, then 110 deg (sharp), legs 1000 / 800 / 800 / 800 / 800
static const float kRoute[5][2] = {{1000, 0}, {1692.8f, 400}, {1899.9f, 1172.7f}, {1334.2f, 1738.4f}, {996.1f, 1013.4f}};

static void addRoute(Bot& b) {
  for (auto& p : kRoute) b.nav.add(p[0], p[1]);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
}

static void testPassThrough(bool live) {
  printf("goto: 5-point route, corners 30 / 45 / 60 / 110 deg, %s params (#103)\n", live ? "live" : "default");
  reset();
  if (live) liveParams();
  Bot b;
  addRoute(b);
  float min_v = 1e9, max_off = 0;  // slowest between passing 1 and passing 3; farthest from the route
  uint32_t spin_early = 0, t_done = 0;
  for (int i = 0; i < 12000 && !t_done; i++) {
    const uint32_t spin0 = b.spin_ms;
    b.run(0.005f);
    const uint8_t k = b.nav.index();
    if (k >= 1 && k <= 2) min_v = fminf(min_v, b.v);
    if (k < 3) spin_early += b.spin_ms - spin0;
    if (k >= 1 && k <= 3) {  // distance from the route start .. 4 (corner cutting at 1, 2, 3)
      float best = 1e9;
      for (int s = 0; s < 4; s++) {
        const float ax = s ? kRoute[s - 1][0] : 0, ay = s ? kRoute[s - 1][1] : 0;
        const float dx = kRoute[s][0] - ax, dy = kRoute[s][1] - ay;
        float t = ((b.x - ax) * dx + (b.y - ay) * dy) / (dx * dx + dy * dy);
        t = fminf(1, fmaxf(0, t));
        best = fminf(best, hypotf(b.x - ax - t * dx, b.y - ay - t * dy));
      }
      max_off = fmaxf(max_off, best);
    }
    if (b.nav.state() == NAV_ARRIVED) t_done = b.ms;
  }
  b.run(1);
  const float off = hypotf(b.x - kRoute[4][0], b.y - kRoute[4][1]);
  printf("  done in %.1f s, path %.0f mm; speed passing 1-4: %.0f %.0f %.0f %.0f mm/s, slowest 1..3 %.0f mm/s\n",
         t_done / 1000.0, b.path_len, b.pass_v[0], b.pass_v[1], b.pass_v[2], b.pass_v[3], min_v);
  printf("  off the route <= %.0f mm, turned in place %.1f s (before the sharp corner %.1f s), max turn %.0f deg/s,\n"
         "  speed command steps (driving, intermediate) +%.2f / -%.2f mm/s per 5 ms, end %.0f mm off, v %.1f\n",
         max_off, b.spin_ms / 1000.0, spin_early / 1000.0, b.max_turn_cmd, b.max_rise, b.max_drop, off, b.v);
  CHECK(t_done > 0);
  CHECK(b.nav.state() == NAV_ARRIVED && b.nav.index() == 5);
  CHECK(off < P(P_NAV_ARRIVE_MM));
  CHECK(fabsf(b.v) < 5);
  for (int i = 0; i < 3; i++) CHECK(b.pass_v[i] > 1.5f * P(P_NAV_MIN_SPEED));  // never stopped at 1..3
  CHECK(min_v > P(P_NAV_MIN_SPEED));
  CHECK(spin_early == 0);              // no turning in place at the 30 / 45 / 60 deg corners
  CHECK(b.spin_ms > 300);              // ... but at the 110 deg one
  CHECK(b.pass_v[3] < 100);            // slowed down for it
  CHECK(b.max_turn_cmd <= P(P_NAV_MAX_TURN) + 0.01f);
  CHECK(b.max_rise <= P(P_NAV_SPEED_KP) * P(P_NAV_MAX_SPEED) * 0.005f + 0.01f);  // no speed steps
  CHECK(b.max_drop < 5);
  CHECK(max_off < 60);                 // corners cut by less than this
}

static void testPassThroughObstacle() {
  printf("goto: 5-point route, a wall across leg 3 (#103: obstacle stop mid-route)\n");
  reset();
  liveParams();
  Bot b;
  // across the leg 2 -> 3 (75 deg) at its middle
  const float mx = 0.5f * (kRoute[1][0] + kRoute[2][0]), my = 0.5f * (kRoute[1][1] + kRoute[2][1]);
  const float c = cosf(165 * 0.0174533f), s = sinf(165 * 0.0174533f);
  b.world.add(mx - 400 * c, my - 400 * s, mx + 400 * c, my + 400 * s);
  addRoute(b);
  b.run(30);
  CHECK(b.nav.state() == NAV_BLOCKED);
  CHECK(b.nav.obstacle());
  CHECK(b.nav.index() == 2);
  CHECK(fabsf(b.v) < 5);
  const float gap = b.nav.frontMm();
  printf("  blocked at %.0f, %.0f before waypoint 3, front %.0f mm\n", b.x, b.y, gap);
  CHECK(gap > P(P_RNG_OBSTACLE_MM) - 60 && gap < P(P_RNG_OBSTACLE_MM) + 10);
  b.world.n = 0;  // gone: goes on to the end
  b.run(40);
  CHECK(b.nav.state() == NAV_ARRIVED && b.nav.index() == 5);
  CHECK(hypotf(b.x - kRoute[4][0], b.y - kRoute[4][1]) < P(P_NAV_ARRIVE_MM));
}

// A new route while under way (the planner resends the rest): no stop, no turning in place (#103)
static void testPassThroughResend() {
  printf("goto: route resent while driving (#103)\n");
  reset();
  liveParams();
  Bot b, c;  // c: the same without the resend
  addRoute(b);
  addRoute(c);
  b.run(6);
  c.run(6);
  CHECK(b.nav.state() == NAV_DRIVING);
  const int k = b.nav.index();
  b.nav.clearList();
  for (int i = k; i < 5; i++) b.nav.add(kRoute[i][0], kRoute[i][1]);  // the rest, as the bridge sends it
  b.nav.listChanged(true);
  float dv = 0, dturn = 0;
  for (int i = 0; i < 400; i++) {
    b.run(0.005f);
    c.run(0.005f);
    dv = fmaxf(dv, fabsf(b.v - c.v));
    dturn = fmaxf(dturn, fabsf(b.nav.turn() - c.nav.turn()));
    CHECK(b.nav.state() == NAV_DRIVING);
  }
  printf("  at %.0f mm/s: speed differs by <= %.1f mm/s, turn by <= %.1f deg/s from the run without it\n", c.v, dv,
         dturn);
  CHECK(dv < 25);     // (the new leg starts at the pose: a slightly different line and corner)
  CHECK(dturn < 15);
}

// ---- contacts (#104): obstacles below the lidar's scan plane, found by touch
static int g_contacts = 0;
static ContactFrame g_last = {};
static void takeContact(Bot& b) {
  if (b.nav.contactDue()) {
    g_last = b.nav.contactFrame();
    g_contacts++;
    b.nav.contactSent();
  }
}
static void runC(Bot& b, float seconds) {  // run in 5 ms steps, collecting MSG_CONTACT
  for (float t = 0; t < seconds; t += 0.005f) {
    b.run(0.005f);
    takeContact(b);
  }
}

static void contactStart(Bot& b, float speed) {
  g_param[P_NAV_MAX_SPEED] = speed;
  b.dyn = true;
  b.low_x = 1200;
  b.nav.add(2500, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  g_contacts = 0;
  g_last = ContactFrame{};
}

static void testContactBump() {
  printf("goto: drives into a low obstacle at 250 mm/s -> contact bump, back off, wait, try again (#104)\n");
  reset();
  Bot b;
  contactStart(b, 250);
  uint32_t detect = 0;
  for (int i = 0; i < 3000 && !g_contacts; i++) runC(b, 0.005f);
  detect = b.ms;
  CHECK(g_contacts == 1);
  CHECK(g_last.kind == LINK_CONTACT_BUMP);
  CHECK(b.nav.contact());
  CHECK(b.nav.state() == NAV_BLOCKED);
  printf("  impact %.3f s, detected %.3f s (+%u ms), contact at %.0f, %.0f, peak %.0f dps\n", b.impact_ms / 1000.0,
         detect / 1000.0, (unsigned)(detect - b.impact_ms), g_last.x_mm, g_last.y_mm, g_last.value);
  CHECK(detect - b.impact_ms <= 200);
  CHECK(fabsf(g_last.x_mm - 1200) < 25 && fabsf(g_last.y_mm) < 5);  // the body's front edge
  CHECK(fabsf(g_last.heading_deg) < 2);
  runC(b, 0.5f);
  CHECK(b.v < 0);  // backing off
  runC(b, 2.5f);
  CHECK(b.nav.state() == NAV_BLOCKED);  // waits for a new list (the planner's route round the mark)
  printf("  backed off to x %.0f\n", b.x);
  CHECK(1200 - 45 - b.x > 80);
  runC(b, 3.0f);  // no new list within 4 s: GOTO tries again, and finds it again
  CHECK(g_contacts == 2);
  b.nav.clearList();  // the planner's new route
  b.nav.add(b.x, 600);
  b.nav.listChanged(true);
  runC(b, 20);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(!b.nav.contact());
}

static void testContactStuck() {
  printf("goto: pushes into it at 120 mm/s (no bump: too slow) -> contact stuck within 0.5 s\n");
  reset();
  Bot b;
  contactStart(b, 120);
  for (int i = 0; i < 4000 && !g_contacts; i++) runC(b, 0.005f);
  CHECK(g_contacts == 1);
  CHECK(g_last.kind == LINK_CONTACT_STUCK);
  CHECK(b.nav.state() == NAV_BLOCKED);
  printf("  impact %.3f s, detected %.3f s (+%u ms), contact at %.0f, %.0f\n", b.impact_ms / 1000.0, b.ms / 1000.0,
         (unsigned)(b.ms - b.impact_ms), g_last.x_mm, g_last.y_mm);
  CHECK(b.ms - b.impact_ms >= 300 && b.ms - b.impact_ms <= 550);
  runC(b, 0.6f);
  CHECK(b.v < 0);

  printf("  EXPLORE: waits for a new list\n");
  reset();
  Bot e;
  e.mode = MODE_EXPLORE;
  contactStart(e, 120);
  runC(e, 15);
  CHECK(g_contacts == 1);
  CHECK(e.nav.state() == NAV_BLOCKED);

  printf("  a lidar stuck event (#79) confirms it at half the time\n");
  reset();
  Bot l;
  contactStart(l, 120);
  for (int i = 0; i < 4000 && !l.stalled; i++) runC(l, 0.005f);
  runC(l, 0.28f);
  CHECK(g_contacts == 0);
  l.nav.onLidarStuck(l.ms);
  runC(l, 0.005f);
  CHECK(g_contacts == 1);
  CHECK(g_last.kind == LINK_CONTACT_STUCK && (g_last.flags & LINK_CONTACT_LIDAR));
  printf("  lidar-confirmed %u ms after the impact\n", (unsigned)(l.ms - l.impact_ms));
}

static void testContactFell() {
  printf("goto: a hard bump throws it over before the hold time -> contact bump (fell)\n");
  reset();
  Bot b;
  contactStart(b, 250);
  for (int i = 0; i < 4000 && !b.stalled; i++) runC(b, 0.005f);
  runC(b, 0.04f);
  b.upright = false;  // fallen
  runC(b, 0.02f);
  CHECK(g_contacts == 1);
  CHECK(g_last.kind == LINK_CONTACT_BUMP && (g_last.flags & LINK_CONTACT_FELL));
}

static void testContactNone() {
  printf("goto: 3 waypoints, starts / stops / turns, 40 dps pitch-rate noise, no obstacle -> no contact\n");
  reset();
  Bot b;
  b.dyn = true;
  b.noise_dps = 40;
  b.nav.add(800, 0);
  b.nav.add(800, 500);
  b.nav.add(0, 500);
  b.nav.add(0, 0);
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  g_contacts = 0;
  runC(b, 60);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(g_contacts == 0);

  printf("  a door strip at 250 mm/s: the wheels stop 50 ms (pitch kick), then roll over it -> no contact\n");
  reset();
  Bot s;
  contactStart(s, 250);
  s.low_x = 1e9f;
  for (int i = 0; i < 4000 && s.x + 45 < 1200; i++) runC(s, 0.005f);
  const float v0 = s.v;
  for (int k = 0; k < 10; k++) {  // 50 ms stopped at the strip
    s.ms += 5;
    s.speed_meas += (0 - s.speed_meas) * 0.005f / 0.04f;
    s.pitch_rate = k < 6 ? 0.45f * v0 : 0.0f;
    s.nav.update(s.in());
    takeContact(s);
  }
  runC(s, 3);
  printf("    %d contacts, at x %.0f\n", g_contacts, s.x);
  CHECK(g_contacts == 0);
  CHECK(s.nav.state() == NAV_DRIVING);

  printf("  HOLD, held still by hand while it pushes: no contact\n");
  reset();
  Bot h;
  h.dyn = true;
  h.mode = MODE_HOLD;
  h.nav.onModeChange(h.in());
  h.x = -300;          // pushed back 300 mm: it drives forward to its hold point ...
  h.low_x = -200;      // ... but a hand holds it there
  g_contacts = 0;
  runC(h, 5);
  CHECK(h.stalled);
  CHECK(g_contacts == 0);

  printf("  nav.stuck_ms 0: contact checks off too\n");
  reset();
  g_param[P_NAV_STUCK_MS] = 0;
  Bot o;
  contactStart(o, 250);
  runC(o, 15);
  CHECK(g_contacts == 0);
}

static void testContactStation() {
  printf("STATION: driven into it -> reported only (the motion is the driver's)\n");
  reset();
  Bot b;
  b.dyn = true;
  b.low_x = 1200;
  b.mode = MODE_STATION;
  b.nav.onModeChange(b.in());
  b.ms = 4000;  // (one report per 3 s, from boot)
  g_contacts = 0;
  b.station_v = 200;
  runC(b, 12);
  CHECK(g_contacts >= 1 && g_contacts <= 3);  // pushing on: at most one per 3 s (it must get going again first)
  CHECK(g_last.flags & LINK_CONTACT_REPORT);
  CHECK(g_last.mode == MODE_STATION);
  CHECK(b.nav.state() == NAV_IDLE);
  CHECK(b.stalled);  // still pushing: nothing changed its command
  printf("  %d reports in 12 s of pushing\n", g_contacts);
}

// ---- pass-through (#103) + contacts (#104): corner slow-downs are cruising, not a stop
static void testContactPassThrough(bool live) {
  printf("goto: 5-point pass-through route, contact detection on, 40 dps noise, %s params, 4 seeds -> no contact\n",
         live ? "live" : "default");
  for (uint32_t seed = 1; seed <= 4; seed++) {
    reset();
    if (live) liveParams();
    Bot b;
    b.dyn = true;
    b.noise_dps = 40;
    b.rnd = 7919u * seed;
    addRoute(b);
    g_contacts = 0;
    uint32_t t_done = 0;
    for (int i = 0; i < 12000 && !t_done; i++) {
      runC(b, 0.005f);
      if (b.nav.state() == NAV_ARRIVED) t_done = b.ms;
    }
    printf("  seed %u: done in %.1f s, %d contacts, passing 1-3 at %.0f %.0f %.0f mm/s\n", (unsigned)seed,
           t_done / 1000.0, g_contacts, b.pass_v[0], b.pass_v[1], b.pass_v[2]);
    CHECK(t_done > 0);
    CHECK(g_contacts == 0);
    for (int i = 0; i < 3; i++) CHECK(b.pass_v[i] > 1.5f * P(P_NAV_MIN_SPEED));  // still passed through
  }
}

// A low obstacle where the command is cut back for a 60 deg pass-through corner (cap 112 mm/s): still cruising
// (#104 x #103), so the stuck rule finds it in ~0.4 s (with "cruise = at max speed" it would wait 2.5 s for the
// odometry stuck check)
static void testContactCornerSlowdown() {
  printf("goto: low obstacle in a 60 deg corner slow-down (pass-through) -> contact stuck, back off, replan\n");
  reset();
  Bot b;
  b.dyn = true;
  b.low_x = 800;  // the body's front stops at x 800: the axle at 755, ~245 mm before the corner at 1000, 0
  b.low_y0 = -150;
  b.low_y1 = 150;
  b.nav.add(1000, 0);
  b.nav.add(1400, 692.8f);  // 60 deg left
  b.nav.listChanged(true);
  b.nav.onModeChange(b.in());
  g_contacts = 0;
  float cmd_at_impact = 0;
  for (int i = 0; i < 4000 && !g_contacts; i++) {
    runC(b, 0.005f);
    if (b.impact_ms && !cmd_at_impact) cmd_at_impact = b.vt_ramp;
  }
  printf("  impact %.3f s at %.0f mm/s command (max %.0f), detected +%u ms, kind %u\n", b.impact_ms / 1000.0,
         cmd_at_impact, P(P_NAV_MAX_SPEED), (unsigned)(b.ms - b.impact_ms), (unsigned)g_last.kind);
  CHECK(cmd_at_impact >= ContactDetector::kStuckMinCmd && cmd_at_impact < P(P_NAV_MAX_SPEED) - 20);  // slowed for it
  CHECK(g_contacts == 1);
  CHECK(b.ms - b.impact_ms <= 700);
  CHECK(b.nav.contact() && b.nav.state() == NAV_BLOCKED);
  CHECK(b.nav.index() == 0);
  runC(b, 1.0f);
  CHECK(b.x < 755 - 40);  // backed off
  b.nav.clearList();      // the planner's route round the mark
  b.nav.add(b.x, 400);
  b.nav.add(1400, 692.8f);
  b.nav.listChanged(true);
  runC(b, 30);
  CHECK(b.nav.state() == NAV_ARRIVED);
  CHECK(g_contacts == 1);
}
int main() {
  testPassThrough(false);
  testPassThrough(true);
  testPassThroughObstacle();
  testPassThroughResend();
  testThreeWaypoints();
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
  testLidarUnion();
  testLidarDeadZone();
  testLidarLostBlocked();
  testGhostPoseReset();
  testLidarStuck();
  testLidarStuckEvent();
  testContactBump();
  testContactStuck();
  testContactFell();
  testContactNone();
  testContactStation();
  testContactPassThrough(false);
  testContactPassThrough(true);
  testContactCornerSlowdown();
  printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
