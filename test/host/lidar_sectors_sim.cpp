// Host test for the Teensy's own LD19 obstacle sectors (issue #113, src/lidar_sectors.cpp).
//
//   lidar_sectors_sim                      self checks: a simulated LD19 (real 47-byte packets, CRC8, 10 Hz) in a square
//                                          room, chunked like src/lidar_fwd.cpp, poses at 400 Hz; the frames against the
//                                          room's walls, the gates (no config, tilted, stale, no pose), corrupted bytes, a
//                                          pose reset, the corridor rule, and the cost of every service() slice / turn.
//   lidar_sectors_sim --replay FILE --config HEX --half MM
//                                          replays a capture (test/host/make_ld19_replay.py format, uncompressed) and prints
//                                          one "F t_us seq age x0 y0 .. x35 y35" line per frame + counters + timing:
//                                          test/host/lidar_sectors_replay.py compares them with the bridge's frames.
// Build (any C++17 compiler):
//   g++ -std=c++17 -O2 -Wall -Wextra -I src -I include test/host/lidar_sectors_sim.cpp src/lidar_sectors.cpp -o /tmp/ls
//   MSVC: cl /std:c++17 /O2 /EHsc /Zp1 /FI <a header with #define __attribute__(x)> /I src /I include
//         test\host\lidar_sectors_sim.cpp src\lidar_sectors.cpp
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "lidar_fwd.h"
#include "lidar_sectors.h"

static int g_fail = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);               \
      g_fail++;                                                        \
    }                                                                  \
  } while (0)

static const double kPi = 3.14159265358979323846;

// ------------------------------------------------------------------ timing

struct Timing {
  std::vector<double> slice_us, turn_us, feed_us;
  double cur_turn = 0;
  void report(const char* what) const {
    auto stat = [](std::vector<double> v, const char* name) {
      if (v.empty()) return;
      std::sort(v.begin(), v.end());
      double sum = 0;
      for (double x : v) sum += x;
      printf("  %-6s n %6zu  mean %7.2f us  p99 %7.2f us  max %7.2f us\n", name, v.size(), sum / v.size(),
             v[(size_t)(0.99 * (v.size() - 1))], v.back());
    };
    printf("timing (%s, this PC):\n", what);
    stat(feed_us, "feed");
    stat(slice_us, "slice");
    stat(turn_us, "turn");
  }
};

static double nowUs() {
  using namespace std::chrono;
  return duration<double, std::micro>(steady_clock::now().time_since_epoch()).count();
}

// one background pass: a slice while a turn waits / is processed; returns true when a frame came out
static bool serviceAll(LidarSectors& s, uint32_t t_us, float half, Timing& tm, LidarObstacleFrame* out, int* nf) {
  bool any = false;
  while (s.busy()) {
    const double t0 = nowUs();
    s.service(t_us, half);
    const double dt = nowUs() - t0;
    tm.slice_us.push_back(dt);
    tm.cur_turn += dt;
    if (!s.busy()) {
      tm.turn_us.push_back(tm.cur_turn);
      tm.cur_turn = 0;
    }
    LidarObstacleFrame f;
    if (s.takeFrame(f)) {
      any = true;
      if (out) *out = f;
      if (nf) (*nf)++;
    }
  }
  return any;
}

// ------------------------------------------------------------------ replay mode

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static int unhex(const char* s, uint8_t* out, int max) {
  int n = 0;
  while (s[0] && s[1] && n < max) {
    const int a = hexval(s[0]), b = hexval(s[1]);
    if (a < 0 || b < 0) break;
    out[n++] = (uint8_t)(a * 16 + b);
    s += 2;
  }
  return n;
}

static int replay(const char* path, const char* cfg_hex, float half) {
  LidarConfigFrame cfg;
  if (unhex(cfg_hex, (uint8_t*)&cfg, (int)sizeof(cfg)) != (int)sizeof(cfg)) {
    fprintf(stderr, "--config: %d hex bytes expected\n", (int)sizeof(cfg));
    return 2;
  }
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "cannot open %s\n", path);
    return 2;
  }
  static LidarSectors s;
  s.configure(cfg);
  Timing tm;
  std::vector<char> line(1 << 16);
  uint8_t data[512];
  int frames = 0;
  while (fgets(line.data(), (int)line.size(), f)) {
    const char* p = line.data();
    if (p[0] == 'T') {
      unsigned long long us;
      float x, y, h, pitch;
      if (sscanf(p + 2, "%llu %f %f %f %f", &us, &x, &y, &h, &pitch) == 5) {
        s.addPose((uint32_t)us, x, y, h, pitch);
        s.addPitch((uint32_t)us, pitch);
      }
    } else if (p[0] == 'R') {
      unsigned long long us;
      int flags, off = 0;
      if (sscanf(p + 2, "%llu %d %n", &us, &flags, &off) >= 2 && off > 0) {
        char* hx = (char*)p + 2 + off;
        hx[strcspn(hx, "\r\n")] = 0;
        const int n = unhex(hx, data, (int)sizeof(data));
        const double t0 = nowUs();
        s.feed(data, n, (uint32_t)us, (flags & 1) != 0);
        tm.feed_us.push_back(nowUs() - t0);
        LidarObstacleFrame fr;
        int nf = 0;
        if (serviceAll(s, (uint32_t)us, half, tm, &fr, &nf)) {
          frames++;
          printf("F %llu %u %u", us, (unsigned)fr.seq, (unsigned)fr.age_ms);
          for (int i = 0; i < 2 * LINK_LIDAR_SECTORS; i++) printf(" %d", fr.xy_mm[i]);
          printf("\n");
        }
      }
    }
  }
  fclose(f);
  const LidarSectors::Counts& c = s.counts();
  printf("N sent %lu off %lu unbound %lu no_pose %lu stale %lu tilt %lu few %lu busy %lu long %lu packets %lu crc %lu "
         "dropped %lu partial %lu turns %lu\n",
         (unsigned long)c.sent, (unsigned long)c.off, (unsigned long)c.unbound, (unsigned long)c.no_pose,
         (unsigned long)c.stale, (unsigned long)c.tilt, (unsigned long)c.few, (unsigned long)c.busy,
         (unsigned long)c.long_turn, (unsigned long)c.packets, (unsigned long)c.crc_errors,
         (unsigned long)c.bytes_dropped, (unsigned long)c.partial, (unsigned long)c.turns);
  tm.report("replay");
  return 0;
}

// ------------------------------------------------------------------ simulated LD19 in a square room

static uint8_t g_crc[256];
static void crcInit() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    g_crc[i] = c;
  }
}

struct Room {
  double half = 2000;                                // walls at x, y = +-half (odometry frame)
  struct Post {
    double x, y, r;
  };
  std::vector<Post> posts;                           // round posts
  double ray(double x, double y, double th) const {
    const double c = cos(th), s = sin(th);
    double best = 1e9;
    if (c > 1e-9) best = std::min(best, (half - x) / c);
    if (c < -1e-9) best = std::min(best, (-half - x) / c);
    if (s > 1e-9) best = std::min(best, (half - y) / s);
    if (s < -1e-9) best = std::min(best, (-half - y) / s);
    for (const Post& q : posts) {  // ray / circle
      const double dx = x - q.x, dy = y - q.y;
      const double b = dx * c + dy * s, cc = dx * dx + dy * dy - q.r * q.r;
      const double disc = b * b - cc;
      if (disc >= 0) {
        const double t = -b - sqrt(disc);
        if (t > 0) best = std::min(best, t);
      }
    }
    return best;
  }
};

struct Sim {
  // robot state (odometry frame) as functions of time
  double x0 = 0, y0 = 0, h0 = 0, v = 0, w = 0;     // start pose, mm/s, deg/s
  double pitch = -3.0;                             // deg (IMU mount offset when balanced)
  double tilt_from = -1, tilt_to = -1, tilt_deg = 0;  // a lean spike
  double yaw = -90;                                // LD19 mount
  Room room;
  uint32_t t_us0 = 1000000;
  unsigned rnd = 12345;
  double noise() {  // +-3 mm
    rnd = rnd * 1103515245u + 12345u;
    return ((rnd >> 8) % 6001) / 1000.0 - 3.0;
  }
  void pose(double t, double& x, double& y, double& h, double& p) const {
    h = h0 + w * t;
    if (fabs(w) > 1e-9) {
      const double r = v / (w * kPi / 180.0);
      x = x0 + r * (sin(h * kPi / 180) - sin(h0 * kPi / 180));
      y = y0 - r * (cos(h * kPi / 180) - cos(h0 * kPi / 180));
    } else {
      x = x0 + v * t * cos(h0 * kPi / 180);
      y = y0 + v * t * sin(h0 * kPi / 180);
    }
    p = pitch + ((t >= tilt_from && t < tilt_to) ? tilt_deg : 0.0);
  }
  // one 47-byte packet of points n0 .. n0 + 11 (4500 points/s, 450 a turn, 0.8 deg apart, clockwise)
  void packet(long n0, uint8_t* b) {
    const double t_last = (n0 + 11) / 4500.0;
    double x, y, h, p;
    pose(t_last - 5.5 / 4500.0, x, y, h, p);
    const double a0 = (n0 % 450) * 0.8, a1 = a0 + 11 * 0.8;
    b[0] = 0x54;
    b[1] = 0x2C;
    b[2] = 0x10;
    b[3] = 0x0E;  // 3600 deg/s
    const uint16_t s = (uint16_t)lround(fmod(a0, 360.0) * 100), e = (uint16_t)lround(fmod(a1, 360.0) * 100);
    b[4] = s & 0xFF;
    b[5] = s >> 8;
    for (int i = 0; i < 12; i++) {
      const double a = a0 + 0.8 * i;
      const double beam = (h + yaw - a) * kPi / 180.0;  // world direction (LD19 counts clockwise)
      double d = room.ray(x, y, beam) + noise();
      if (d > 12000) d = 0;
      const uint16_t dm = (uint16_t)std::max(0.0, d);
      b[6 + 3 * i] = dm & 0xFF;
      b[7 + 3 * i] = dm >> 8;
      b[8 + 3 * i] = 200;
    }
    b[42] = e & 0xFF;
    b[43] = e >> 8;
    const uint16_t ts = (uint16_t)((long)(t_last * 1000.0) % 30000);
    b[44] = ts & 0xFF;
    b[45] = ts >> 8;
    uint8_t c = 0;
    for (int i = 0; i < 46; i++) c = g_crc[c ^ b[i]];
    b[46] = c;
  }
};

struct RunResult {
  int frames = 0;
  double last_t = 0;  // when the last frame came out
  LidarObstacleFrame last = {};
  std::vector<LidarObstacleFrame> all;
};

// the forwarder (src/lidar_fwd.cpp) between a fake UART and a USB link that may be stalled; its tap feeds the sectors
struct FwdIo : LidarFwdIo {
  std::vector<uint8_t> rx;
  bool stalled = false;
  LidarSectors* s = nullptr;
  uint32_t now_us = 0;
  size_t max_fill = 0;
  int available() override { return (int)rx.size(); }
  int read(uint8_t* dst, int n) override {
    n = std::min<int>(n, (int)rx.size());
    memcpy(dst, rx.data(), (size_t)n);
    rx.erase(rx.begin(), rx.begin() + n);
    return n;
  }
  int room() override { return stalled ? 0 : 4096; }
  bool send(const uint8_t*, uint8_t) override { return !stalled; }
  void tap(const uint8_t* data, int n, bool hole) override { s->feed(data, n, now_us, hole); }
};

// seconds of simulation: poses at 400 Hz (addPitch every 10 ms), packets every 12 points, 180-byte chunks (or through
// a forwarder: fwd + io)
static RunResult run(Sim& sim, LidarSectors& s, double seconds, Timing& tm, float half = 86.0f, int corrupt_every = 0,
                     double reset_at = -1, LidarForwarder* fwd = nullptr, FwdIo* io = nullptr) {
  RunResult r;
  std::vector<uint8_t> pending;
  long next_pt = 0;
  int pk_count = 0;
  double next_pose = 0, next_pitch = 0;
  double first_wait = -1;
  bool reset_done = false;
  for (double t = 0; t < seconds; t += 0.0005) {
    const uint32_t us = sim.t_us0 + (uint32_t)lround(t * 1e6);
    if (t >= next_pose) {
      next_pose += 0.0025;
      double x, y, h, p;
      sim.pose(t, x, y, h, p);
      s.addPose(us, (float)x, (float)y, (float)h, (float)p);
      if (t >= next_pitch) {
        next_pitch += 0.01;
        s.addPitch(us, (float)p);
      }
    }
    if (reset_at >= 0 && t >= reset_at && !reset_done) {
      reset_done = true;
      s.poseReset();
    }
    while ((next_pt + 12) / 4500.0 <= t) {
      uint8_t b[47];
      sim.packet(next_pt, b);
      next_pt += 12;
      if (corrupt_every && ++pk_count % corrupt_every == 0) b[20] ^= 0x5A;  // the CRC catches it
      if (fwd) {
        io->rx.insert(io->rx.end(), b, b + 47);
        continue;
      }
      if (pending.empty()) first_wait = t;
      pending.insert(pending.end(), b, b + 47);
    }
    if (fwd) {
      io->now_us = us;
      io->max_fill = std::max(io->max_fill, io->rx.size());
      fwd->service(us, us / 1000, *io);
    }
    if (!pending.empty() && (pending.size() >= 180 || t - first_wait >= 0.012)) {
      const int n = (int)std::min<size_t>(180, pending.size());
      const double t0 = nowUs();
      s.feed(pending.data(), n, us);
      tm.feed_us.push_back(nowUs() - t0);
      pending.erase(pending.begin(), pending.begin() + n);
      first_wait = t;
    }
    LidarObstacleFrame f;
    int nf = 0;
    if (serviceAll(s, us, half, tm, &f, &nf)) {
      r.frames += nf;
      r.last = f;
      r.last_t = t;
      r.all.push_back(f);
    }
  }
  return r;
}

static LidarConfigFrame config(float yaw = -90, float rmin = 60) {
  LidarConfigFrame c = {};
  c.flags = LINK_LCFG_ENABLED | LINK_LCFG_PITCH_AUTO | LINK_LCFG_FLOOR_CLIP | LINK_LCFG_DESKEW;
  c.yaw_deg = yaw;
  c.height_mm = 208;
  c.min_range_mm = rmin;
  c.max_range_mm = 8000;
  c.min_intensity = 15;
  c.pitch_gate_deg = 6;
  return c;
}

// expected nearest wall distance per sector (robot frame, about the axle), rays at the LD19's beam angles of a robot
// standing still (0.4 + 0.8 k deg with yaw -90), or every 0.1 deg (fine = true)
static void expected(const Sim& sim, double t, double* exp, bool fine = false) {
  double x, y, h, p;
  sim.pose(t, x, y, h, p);
  for (int i = 0; i < 36; i++) exp[i] = 1e9;
  for (int k = 0; k < (fine ? 3600 : 450); k++) {
    const double a = fine ? k * 0.1 : 0.4 + 0.8 * k;
    const int sec = (int)floor(a / 10.0 + 0.5) % 36;
    exp[sec] = std::min(exp[sec], sim.room.ray(x, y, (h + a) * kPi / 180.0));
  }
}

int main(int argc, char** argv) {
  if (argc >= 2 && !strcmp(argv[1], "--replay")) {
    const char* file = nullptr;
    const char* cfg = nullptr;
    float half = 86.0f;
    for (int i = 1; i + 1 < argc; i += 2) {
      if (!strcmp(argv[i], "--replay")) file = argv[i + 1];
      else if (!strcmp(argv[i], "--config")) cfg = argv[i + 1];
      else if (!strcmp(argv[i], "--half")) half = (float)atof(argv[i + 1]);
    }
    if (!file || !cfg) {
      fprintf(stderr, "usage: lidar_sectors_sim --replay FILE --config HEX [--half MM]\n");
      return 2;
    }
    return replay(file, cfg, half);
  }
  crcInit();
  Timing tm;

  // 1. standing in the middle of a 4 m room: ~10 frames/s, every sector at the wall distance
  {
    static LidarSectors s;
    Sim sim;
    s.configure(config());
    RunResult r = run(sim, s, 3.0, tm);
    printf("still: %d frames in 3 s, sent %lu, partial %lu, crc %lu\n", r.frames, (unsigned long)s.counts().sent,
           (unsigned long)s.counts().partial, (unsigned long)s.counts().crc_errors);
    CHECK(r.frames >= 27 && r.frames <= 31);
    CHECK(s.counts().crc_errors == 0);
    double exp[36];
    expected(sim, r.last_t, exp);
    int bad = 0;
    for (int i = 0; i < 36; i++) {
      const double x = r.last.xy_mm[2 * i], y = r.last.xy_mm[2 * i + 1];
      const double d = hypot(x, y);
      if (fabs(d - exp[i]) > 15) {
        printf("  sector %d: %.0f, wall %.0f\n", i, d, exp[i]);
        bad++;
      }
      const double a = atan2(y, x) * 180 / kPi;
      CHECK(fabs(fmod(a - i * 10.0 + 540.0, 360.0) - 180.0) <= 5.01);  // inside its own sector
    }
    CHECK(bad == 0);
    CHECK(r.last.age_ms <= 3);
    CHECK(r.last.sectors == LINK_LIDAR_SECTORS);
    s.noteCycles(600 * 40);  // (a 40 us slice at 600 MHz, for the text)
    char txt[200];
    s.statusText(txt, sizeof(txt), 30000);  // (configured at 0 ms)
    printf("  status: %s\n", txt);
    CHECK(strstr(txt, "sectors 0.9/s, skip tilt 0 pose 0 stale 0 few 0 busy 0, max 40 us/slice") == txt);
  }

  // 2. driving 400 mm/s and turning 90 deg/s: deskewed, the walls stay where they are (frame of the newest pose)
  {
    static LidarSectors s;
    Sim sim;
    sim.x0 = -500;
    sim.v = 400;
    sim.w = 90;
    s.configure(config());
    RunResult r = run(sim, s, 2.0, tm);
    double exp[36];
    expected(sim, r.last_t, exp, true);
    double worst = 0;
    for (int i = 0; i < 36; i++) {
      const double d = hypot(r.last.xy_mm[2 * i], r.last.xy_mm[2 * i + 1]);
      worst = std::max(worst, fabs(d - exp[i]) - 0.02 * exp[i]);  // (0.8 deg beams, the newest pose 0 .. 2.5 ms old)
    }
    printf("moving: %d frames, worst sector error beyond 2 %%: %.0f mm\n", r.frames, worst);
    CHECK(r.frames >= 17);
    CHECK(worst < 30);
  }

  // 3. no MSG_LIDAR_CONFIG: nothing; lidar disabled: nothing
  {
    static LidarSectors s;
    Sim sim;
    RunResult r = run(sim, s, 1.0, tm);
    CHECK(r.frames == 0 && s.counts().unbound >= 8 && !s.active());
    LidarConfigFrame c = config();
    c.flags &= ~LINK_LCFG_ENABLED;
    CHECK(s.configure(c));
    CHECK(!s.configure(c));  // the same again: not a change
    r = run(sim, s, 1.0, tm);
    CHECK(r.frames == 0 && s.counts().off >= 8);
  }

  // 4. a lean beyond the gate for 0.3 s: those turns are "tilted", the others go out
  {
    static LidarSectors s;
    Sim sim;
    sim.tilt_from = 1.0;
    sim.tilt_to = 1.3;
    sim.tilt_deg = 8;
    s.configure(config());
    RunResult r = run(sim, s, 2.5, tm);
    printf("tilt: %d frames, tilt %lu\n", r.frames, (unsigned long)s.counts().tilt);
    CHECK(s.counts().tilt >= 3 && s.counts().tilt <= 5);
    CHECK(r.frames >= 18);
  }

  // 5. every 7th packet corrupted: the CRC drops them (the turns still have >= 25 packets), frames keep coming
  {
    static LidarSectors s;
    Sim sim;
    s.configure(config());
    RunResult r = run(sim, s, 2.0, tm, 86.0f, 7);
    printf("corrupt: %d frames, crc %lu, dropped bytes %lu\n", r.frames, (unsigned long)s.counts().crc_errors,
           (unsigned long)s.counts().bytes_dropped);
    CHECK(s.counts().crc_errors > 0);
    CHECK(r.frames >= 17);
  }

  // 6. no poses for a while (the history is old): no_pose; a pose reset mid-run: that turn is dropped, then frames again
  {
    static LidarSectors s;
    Sim sim;
    s.configure(config());
    RunResult r = run(sim, s, 2.0, tm, 86.0f, 0, 1.0);
    printf("reset: %d frames, partial %lu, no_pose %lu\n", r.frames, (unsigned long)s.counts().partial,
           (unsigned long)s.counts().no_pose);
    CHECK(r.frames >= 16);
  }

  // 7. the corridor rule (#99): in sector 1 (5 .. 15 deg) a post in the path at (800, 75) and a nearer one beside it at
  //    (400, 100), outside the corridor (|y| > 86): the sector holds the one in the path
  {
    static LidarSectors s;
    Sim sim;
    sim.room.half = 1500;
    sim.room.posts.push_back({810, 75, 10});
    sim.room.posts.push_back({410, 100, 10});
    s.configure(config());
    RunResult r = run(sim, s, 1.5, tm);
    const double x1 = r.last.xy_mm[2], y1 = r.last.xy_mm[3];
    printf("corridor: sector 1 = (%.0f, %.0f)\n", x1, y1);
    CHECK(x1 > 780 && x1 < 810 && fabs(y1) <= 86);
    s.configure(config());
    RunResult r2 = run(sim, s, 1.0, tm, 30.0f);  // a corridor narrower than y 75: the nearest point again
    printf("corridor 30: sector 1 = (%d, %d)\n", r2.last.xy_mm[2], r2.last.xy_mm[3]);
    CHECK(r2.last.xy_mm[2] < 420);
  }

  // 8. the Pi's USB link stalled the whole time: the forwarder still reads (and drops) the bytes after at most
  //    setMaxHold (360 B, ~20 ms), so the Teensy's sectors keep coming; without it the bytes would wait ~100 ms
  for (int hold = 0; hold < 2; hold++) {
    static LidarSectors s;
    s = LidarSectors();
    Sim sim;
    s.configure(config());
    LidarForwarder fwd;
    fwd.begin(64 + 2048, 0);
    if (hold) fwd.setMaxHold(2 * LINK_LIDAR_RAW_MAX);
    FwdIo io;
    io.s = &s;
    io.stalled = true;
    RunResult r = run(sim, s, 2.0, tm, 86.0f, 0, -1, &fwd, &io);
    printf("USB stalled, max hold %s: %d frames, UART fill up to %zu bytes (%.0f ms), usb drops %lu\n",
           hold ? "360" : "default", r.frames, io.max_fill, io.max_fill / 17.625, (unsigned long)fwd.drops());
    CHECK(r.frames >= 17);
    if (hold) CHECK(io.max_fill <= 600);
    else CHECK(io.max_fill > 1500);
  }

  tm.report("self checks");
  printf("RAM: sizeof(LidarSectors) %zu bytes\n", sizeof(LidarSectors));
  if (g_fail) {
    printf("%d FAILED\n", g_fail);
    return 1;
  }
  printf("ALL PASS\n");
  return 0;
}
