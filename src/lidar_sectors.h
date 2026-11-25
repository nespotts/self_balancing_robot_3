// LD19 obstacle sectors computed on the Teensy (issue #113): the raw LD19 bytes it already reads from Serial7 (#100) ->
// one LidarObstacleFrame per turn (~10 Hz) straight into the navigator, so obstacle stops no longer depend on the Pi (a
// stalled / rebooting Pi, lost Wi-Fi), with less latency, and without the Pi's 40-80 ms per scan.
//
// A port of the bridge's implementation, which stays the reference (test/host/lidar_sectors_replay.py replays an LD19
// capture through both and compares the frames):
//   station/bridge/ld19.py      Ld19Stream (CRC8 parse, resync, the lidar's ms timestamp anchored to the arrival clock by
//                               the least latency seen, 500 ppm creep) + ScanAssembler (a turn ends when the start angle
//                               wraps; the first partial turn and turns under 25 packets are dropped)
//   station/bridge/lidar.py     Lidar.deskew: per packet the pose interpolated at its time (+ time_offset_ms), the pitch gate
//                               (|pitch - typical| > pitch_gate_deg or |pitch| > 30: the whole turn is "tilted"), range /
//                               intensity / mask / floor-clip filters, range x cos(elevation), the mount (x, y, yaw, flip)
//   station/bridge/lidar_obstacles.py  LidarObstacles.build: the gates (stale turn, no / old pose, tilted, < 30 points),
//                               the points in the robot frame of the newest pose, 36 sectors of 10 deg about the axle: the
//                               nearest point INSIDE the navigator's corridor (by its front / rear distance), else the
//                               nearest (#99); age_ms = how old that pose is.
// The settings are the bridge's lidar.json (the robot profile): MSG_LIDAR_CONFIG (configure()). The corridor half width
// comes from the navigator's own params (nav.corridor_mm / nav.side_min_mm), passed to service().
//
// Cost: feed() (from the forwarder, per UART chunk) only parses: CRC + copy, a few us. A finished turn is processed in
// background slices by service(): the pose / pitch gates, then kSlicePackets packets per call (deskew + filters), then
// kSlicePoints points per call (robot frame + sectors), then the frame. Never inside the balance step; main.cpp calls it
// in the same step-phase window as the forwarder and times every slice (noteCycles) for the "lidar: ..." status line.
//
// Pure C++ (no Arduino): test/host/lidar_sectors_sim.cpp runs this exact code on a PC.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "protocol/link_protocol.h"

class LidarSectors {
 public:
  static constexpr int kPktLen = 47, kPts = 12;
  static constexpr int kMaxPackets = 64;   // per turn (37.5 at 10 Hz); a longer turn is skipped ("long")
  static constexpr int kPoses = 256;       // pose history (640 ms at 400 Hz): a packet's pose is interpolated in it
  static constexpr int kPitch = 300;       // pitch samples for the typical pitch (3 s at 100 Hz, as the bridge)
  static constexpr int kSlicePackets = 4;  // packets deskewed per service() call
  static constexpr int kSlicePoints = 64;  // points put into sectors per service() call (an atan2f each)
  static constexpr int kMinPackets = 25;   // ScanAssembler.MIN_PACKETS
  static constexpr int kMinPoints = 30;    // lidar.MIN_POINTS

  // MSG_LIDAR_CONFIG. Returns true when the settings changed (log line). Until the first one: nothing is computed.
  // now_ms: starts the status line's first period (the first configure).
  bool configure(const LidarConfigFrame& f, uint32_t now_ms = 0);
  bool configured() const { return configured_; }
  bool active() const { return configured_ && enabled_; }  // computing: the bridge's MSG_LIDAR_OBSTACLES are ignored
  const LidarConfigFrame& config() const { return cfg_; }

  // Robot pose of one balance step (t_us = its IMU sample time, the telemetry's t_us), for the per-packet deskew.
  void addPose(uint32_t t_us, float x_mm, float y_mm, float heading_deg, float pitch_deg);
  // Pitch for the typical pitch (median of the last 3 s): every 10 ms (main.cpp), every telemetry row (replay).
  void addPitch(uint32_t t_us, float pitch_deg);
  // EVT_RESET_POSE: the poses jumped; the turns in progress are in the old frame.
  void poseReset();

  // Bytes from the LD19 as the forwarder read them (one UART chunk). hole: bytes before them were lost (UART overflow).
  void feed(const uint8_t* data, int n, uint32_t now_us, bool hole = false);

  // One background slice. half_mm: the navigator's corridor half width. true = a frame is ready (takeFrame).
  bool service(uint32_t now_us, float half_mm);
  bool busy() const { return stage_ != IDLE || queued_; }
  bool takeFrame(LidarObstacleFrame& f);
  // CPU time of the last service() call as measured by the caller (cycles), added to the turn's cost.
  void noteCycles(uint32_t cycles);

  // "sectors 9.9/s (skip tilt 3, pose 0, ...), max 30 us / slice, 140 us / turn" for the forwarder's status line; resets
  // the period's maxima. Returns the length written.
  int statusText(char* out, int n, uint32_t now_ms, uint32_t cycles_per_us = 600);

  // counters (as the bridge's LidarObstacles.n / Ld19Stream / ScanAssembler)
  struct Counts {
    uint32_t sent, off, unbound, no_pose, stale, tilt, few, busy, long_turn;
    uint32_t packets, crc_errors, bytes_dropped, partial, turns;
  };
  const Counts& counts() const { return n_; }
  uint32_t maxSliceCycles() const { return max_slice_; }
  uint32_t lastTurnCycles() const { return last_turn_cycles_; }

  // the last frame's sectors (robot frame, mm) before rounding, and its age (tests)
  float lastAgeMs() const { return last_age_ms_; }

 private:
  struct Packet {
    double t;            // arrival clock, s (the last point's time)
    float start, step;   // deg; start 0 .. 360, step = (end - start) / 11
    uint16_t dist[kPts];
    uint8_t qual[kPts];
  };
  struct Turn {
    Packet pk[kMaxPackets];
    int n = 0;
  };
  struct Pose {
    double t;
    float x, y, h, p;
  };
  enum Stage : uint8_t { IDLE, START, DESKEW, SECTORS, FINAL };

  double sec(uint32_t us);
  void addPacket(const uint8_t* b, double lidar_t);
  void turnDone();
  const Pose& poseAt(uint32_t a) const;  // absolute index (pose_total_ - pose_n_ .. pose_total_ - 1)
  uint32_t seek(double t) const;
  void interp(uint32_t& idx, double t, double& x, double& y, double& h, double& p) const;
  float pitchRef();
  bool deskewPacket(const Packet& pk);
  void buildMask();

  // settings
  LidarConfigFrame cfg_ = {};
  bool configured_ = false, enabled_ = false;
  bool flip_ = false, auto_ref_ = true, clip_ = true, deskew_ = true;
  float sx_ = 0, sy_ = 0, yaw_ = 0, fk_ = 0, rmin_ = 0, rmax_ = 0, qmin_ = 0, gate_ = 0, ref_fixed_ = 0;
  double tco_ = 0;
  bool masked_ = false;
  uint8_t mask_[720] = {};

  // clock (unwrapped micros, s)
  bool have_ref_ = false;
  uint32_t ref_us_ = 0;
  double ref_s_ = 0;

  // Ld19Stream
  uint8_t buf_[kPktLen + 256] = {};
  int buf_n_ = 0;
  bool have_ts_ = false, have_off_ = false, have_now_last_ = false;
  uint16_t ts_last_ = 0;
  int32_t wraps_ = 0;
  double off_ = 0, now_last_ = 0;

  // ScanAssembler: turns_[cur_] is being assembled, turns_[cur_ ^ 1] processed
  Turn turns_[2];
  int cur_ = 0;
  bool have_start_ = false, started_ = false, queued_ = false, too_long_ = false;
  float last_start_ = 0;

  // history
  Pose poses_[kPoses] = {};
  int pose_head_ = 0, pose_n_ = 0;
  uint32_t pose_total_ = 0;  // poses ever added (absolute index of the next one)
  double pitch_t_[kPitch] = {};
  float pitch_v_[kPitch] = {};
  int pitch_head_ = 0, pitch_n_ = 0;

  // processing of turns_[cur_ ^ 1]
  Stage stage_ = IDLE;
  bool turn_end_ = false;  // the turn's processing ended in the last service() call (noteCycles closes its cost)
  int pk_i_ = 0, pt_i_ = 0;
  uint32_t idx_ = 0;
  float ref_ = 0;
  double t1_ = 0;
  double fixed_[4] = {};   // deskew off: the one pose (x, y, heading, pitch)
  Pose newest_ = {};
  float zx_[kMaxPackets * kPts], zy_[kMaxPackets * kPts];
  int zn_ = 0;
  float nc_ = 1, ns_ = 0;  // rotation into the newest pose's robot frame
  float best_d_[LINK_LIDAR_SECTORS], ckey_[LINK_LIDAR_SECTORS];
  int16_t best_i_[LINK_LIDAR_SECTORS], ci_[LINK_LIDAR_SECTORS];
  float half_ = 0;
  uint32_t turn_cycles_ = 0, last_turn_cycles_ = 0, max_turn_ = 0, max_slice_ = 0;

  LidarObstacleFrame frame_ = {};
  bool frame_ready_ = false;
  uint8_t seq_ = 0;
  float last_age_ms_ = 0;
  Counts n_ = {};
  uint32_t stat_ms_ = 0, stat_sent_ = 0;
  Counts stat_n_ = {};
  bool stat_started_ = false;
};
