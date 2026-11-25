#include "lidar_sectors.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>

// station/bridge/ld19.py
static constexpr uint8_t kHeader = 0x54, kVerLen = 0x2C;
static constexpr double kMaxSpanDeg = 45.0;
static constexpr int kTsPeriodMs = 30000;
static constexpr double kPointRate = 4500.0;
// station/bridge/lidar.py, lidar_obstacles.py
static constexpr float kPitchAbsMax = 30.0f;
static constexpr double kTiltRefS = 3.0;
static constexpr double kPoseStaleS = 0.5;
static constexpr double kMaxScanAgeS = 0.25;
static constexpr float kAxisFwd = 26.0f, kFrontOff = 26.0f + 12.9f, kRearOff = 26.0f - 12.9f;  // src/navigator.cpp
static constexpr float kRad = 0.017453292519943295f;
static constexpr float kInf = 1e30f;

static uint8_t g_crc8[256];
static bool g_crc8_ok = false;
static void crcTable() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    g_crc8[i] = c;
  }
  g_crc8_ok = true;
}

static double pmod(double a, double m) {  // Python's a % m (m > 0): 0 <= result < m
  double r = fmod(a, m);
  if (r < 0) r += m;
  return r >= m ? 0.0 : r;
}

static float pmodf(float a, float m) {
  float r = fmodf(a, m);
  if (r < 0) r += m;
  return r >= m ? 0.0f : r;
}

static int16_t i16(float v) {  // Python int(round(v)) (half to even) clamped to int16
  const float r = rintf(v);
  if (r < -32768.0f) return -32768;
  if (r > 32767.0f) return 32767;
  return (int16_t)r;
}

// ------------------------------------------------------------------ settings

bool LidarSectors::configure(const LidarConfigFrame& f, uint32_t now_ms) {
  const bool changed = !configured_ || memcmp(&f, &cfg_, sizeof(f)) != 0;
  if (!configured_ && !stat_started_) {
    stat_started_ = true;
    stat_ms_ = now_ms;
    stat_n_ = n_;
  }
  cfg_ = f;
  configured_ = true;
  enabled_ = (f.flags & LINK_LCFG_ENABLED) != 0;
  flip_ = (f.flags & LINK_LCFG_FLIP) != 0;
  auto_ref_ = (f.flags & LINK_LCFG_PITCH_AUTO) != 0;
  clip_ = (f.flags & LINK_LCFG_FLOOR_CLIP) != 0;
  deskew_ = (f.flags & LINK_LCFG_DESKEW) != 0;
  sx_ = f.x_mm;
  sy_ = f.y_mm;
  yaw_ = f.yaw_deg;
  fk_ = 0.9f * f.height_mm;
  rmin_ = f.min_range_mm;
  rmax_ = f.max_range_mm;
  qmin_ = f.min_intensity;
  gate_ = f.pitch_gate_deg;
  ref_fixed_ = f.pitch_ref_deg;
  tco_ = f.time_offset_ms / 1000.0;
  if (changed) buildMask();
  return changed;
}

// lidar.mask_table: 720 flags (0.5 deg steps, robot frame CCW from forward); a sector [a, b] runs CCW from a to b
void LidarSectors::buildMask() {
  memset(mask_, 0, sizeof(mask_));
  masked_ = false;
  const int n = cfg_.mask_n < LINK_LIDAR_MASK_MAX ? cfg_.mask_n : LINK_LIDAR_MASK_MAX;
  for (int s = 0; s < n; s++) {
    const double a = pmod(cfg_.mask_cdeg[2 * s] / 100.0, 360.0), b = pmod(cfg_.mask_cdeg[2 * s + 1] / 100.0, 360.0);
    if (a == b) continue;
    const int i0 = (int)floor(a * 2), i1 = (int)ceil(b * 2);
    if (b > a) {
      for (int i = i0; i < (i1 < 720 ? i1 : 720); i++) mask_[i] = 1;
    } else {
      for (int i = i0; i < 720; i++) mask_[i] = 1;
      for (int i = 0; i < (i1 < 720 ? i1 : 720); i++) mask_[i] = 1;
    }
  }
  for (int i = 0; i < 720; i++) masked_ = masked_ || mask_[i];
}

// ------------------------------------------------------------------ clock / history

double LidarSectors::sec(uint32_t us) {
  if (!have_ref_) {
    have_ref_ = true;
    ref_us_ = us;
    ref_s_ = us * 1e-6;
  }
  const int32_t d = (int32_t)(us - ref_us_);
  const double s = ref_s_ + d * 1e-6;
  if (d > 0) {
    ref_us_ = us;
    ref_s_ = s;
  }
  return s;
}

void LidarSectors::addPose(uint32_t t_us, float x_mm, float y_mm, float heading_deg, float pitch_deg) {
  const double t = sec(t_us);
  if (pose_n_ && t <= poses_[(pose_head_ + kPoses - 1) % kPoses].t) return;  // PoseHistory.add: in time order only
  Pose& p = poses_[pose_head_];
  p.t = t;
  p.x = x_mm;
  p.y = y_mm;
  p.h = heading_deg;
  p.p = pitch_deg;
  pose_head_ = (pose_head_ + 1) % kPoses;
  if (pose_n_ < kPoses) pose_n_++;
  pose_total_++;
}

void LidarSectors::addPitch(uint32_t t_us, float pitch_deg) {
  pitch_t_[pitch_head_] = sec(t_us);
  pitch_v_[pitch_head_] = pitch_deg;
  pitch_head_ = (pitch_head_ + 1) % kPitch;
  if (pitch_n_ < kPitch) pitch_n_++;
}

void LidarSectors::poseReset() {
  pose_n_ = 0;
  pitch_n_ = 0;
  turns_[cur_].n = 0;  // the turn being assembled: its packets were taken at poses in the old frame
  started_ = false;    // (the next wrap starts a new turn, like the first one after power-up)
  have_start_ = false;
  too_long_ = false;
  if (stage_ != IDLE || queued_) {
    stage_ = IDLE;
    queued_ = false;
    turn_end_ = true;
  }
}

// absolute pose index a (pose_total_ - pose_n_ .. pose_total_ - 1)
const LidarSectors::Pose& LidarSectors::poseAt(uint32_t a) const { return poses_[a % kPoses]; }

// lidar._seek: the last entry at or before t (the oldest if none)
uint32_t LidarSectors::seek(double t) const {
  uint32_t lo = pose_total_ - pose_n_, hi = pose_total_ - 1;
  while (lo < hi) {
    const uint32_t mid = lo + ((hi - lo + 1) >> 1);
    if (poseAt(mid).t <= t) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

// lidar.interp: the pose at t; idx a hint that only moves forward
void LidarSectors::interp(uint32_t& idx, double t, double& x, double& y, double& h, double& p) const {
  const uint32_t first = pose_total_ - pose_n_, end = pose_total_;
  if (idx < first) idx = first;  // (overwritten meanwhile)
  while (idx + 1 < end && poseAt(idx + 1).t <= t) idx++;
  const Pose& a = poseAt(idx);
  if (t <= a.t || idx + 1 >= end) {
    x = a.x;
    y = a.y;
    h = a.h;
    p = a.p;
    return;
  }
  const Pose& b = poseAt(idx + 1);
  const double f = (t - a.t) / (b.t - a.t);
  const double dh = pmod((double)b.h - a.h + 180.0, 360.0) - 180.0;
  x = a.x + f * ((double)b.x - a.x);
  y = a.y + f * ((double)b.y - a.y);
  h = a.h + f * dh;
  p = a.p + f * ((double)b.p - a.p);
}

// lidar.pitch_ref: the median (upper) of the pitch samples of the last 3 s, or the fixed reference
float LidarSectors::pitchRef() {
  if (!auto_ref_) return ref_fixed_;
  if (!pitch_n_) return pose_n_ ? poseAt(pose_total_ - 1).p : 0.0f;
  static float tmp[kPitch];
  const int newest = (pitch_head_ + kPitch - 1) % kPitch;
  const double cut = pitch_t_[newest] - kTiltRefS;
  int m = 0;
  for (int i = 0; i < pitch_n_; i++) {
    const int k = (pitch_head_ + kPitch - pitch_n_ + i) % kPitch;
    if (pitch_t_[k] >= cut) tmp[m++] = pitch_v_[k];
  }
  std::nth_element(tmp, tmp + m / 2, tmp + m);
  return tmp[m / 2];
}

// ------------------------------------------------------------------ LD19 stream (ld19.Ld19Stream + ScanAssembler)

void LidarSectors::feed(const uint8_t* data, int n, uint32_t now_us, bool hole) {
  if (!g_crc8_ok) crcTable();
  const double now = sec(now_us);
  if (hole && buf_n_) {  // Ld19Reader.feed_teensy: never let a packet straddle a hole
    n_.bytes_dropped += (uint32_t)buf_n_;
    buf_n_ = 0;
  }
  // packets of this call (their times are anchored together, on the newest one)
  uint8_t out[16][kPktLen];
  double out_t[16];
  int nout = 0;
  for (;;) {
    const int room = (int)sizeof(buf_) - buf_n_;
    const int take = n < room ? n : room;
    memcpy(buf_ + buf_n_, data, (size_t)take);
    buf_n_ += take;
    data += take;
    n -= take;
    int pos = 0;
    while (buf_n_ - pos >= kPktLen && nout < 16) {
      if (buf_[pos] != kHeader || buf_[pos + 1] != kVerLen) {
        int j = -1;
        for (int i = pos + 1; i + 1 < buf_n_; i++)
          if (buf_[i] == kHeader && buf_[i + 1] == kVerLen) {
            j = i;
            break;
          }
        if (j < 0) j = buf_[buf_n_ - 1] == kHeader ? buf_n_ - 1 : buf_n_;  // a trailing 0x54 may start the next one
        n_.bytes_dropped += (uint32_t)(j - pos);
        pos = j;
        continue;
      }
      const uint8_t* f = buf_ + pos;
      uint8_t c = 0;
      for (int i = 0; i < kPktLen - 1; i++) c = g_crc8[c ^ f[i]];
      bool ok = c == f[kPktLen - 1];
      if (ok) {
        const double s = (f[4] | (f[5] << 8)) / 100.0;
        double e = (f[42] | (f[43] << 8)) / 100.0;
        if (e < s) e += 360.0;  // the packet crosses 360 -> 0
        ok = e - s <= kMaxSpanDeg;
      }
      if (!ok) {
        n_.crc_errors++;
        n_.bytes_dropped++;
        pos++;
        continue;
      }
      // ld19._unwrap: the lidar's ms clock (wraps at 30000) -> s on a continuous lidar clock
      const uint16_t ts = (uint16_t)(f[44] | (f[45] << 8));
      if (have_ts_) {
        if ((int)ts < (int)ts_last_ - kTsPeriodMs / 2) {
          wraps_++;
        } else if ((int)ts < (int)ts_last_ - 200) {  // the lidar restarted: its clock starts over
          wraps_ = 0;
          have_off_ = false;
        }
      }
      have_ts_ = true;
      ts_last_ = ts;
      memcpy(out[nout], f, kPktLen);
      out_t[nout] = ts / 1000.0 + wraps_ * (kTsPeriodMs / 1000.0);
      nout++;
      n_.packets++;
      pos += kPktLen;
    }
    if (pos) {
      memmove(buf_, buf_ + pos, (size_t)(buf_n_ - pos));
      buf_n_ -= pos;
    }
    if (n > 0 && nout < 16) continue;  // more bytes of this call first (a call of more than the buffer holds)
    if (nout) {
      // one read may hold many packets (they all arrive "now"): anchor the offset on the newest one
      const double cand = now - out_t[nout - 1];
      if (!have_off_ || cand < off_) {
        off_ = cand;
      } else {  // let the offset creep up (clock rate difference, 500 ppm)
        const double dt = now - (have_now_last_ ? now_last_ : now);
        off_ = std::min(cand, off_ + 5e-4 * (dt > 0 ? dt : 0.0));
      }
      have_off_ = true;
      now_last_ = now;
      have_now_last_ = true;
      const bool full = nout == 16;
      for (int i = 0; i < nout; i++) addPacket(out[i], out_t[i] + off_);
      nout = 0;
      if (full) continue;  // more packets may be waiting in the buffer
    }
    if (n <= 0) break;
  }
}

void LidarSectors::addPacket(const uint8_t* f, double t) {
  const double s = (f[4] | (f[5] << 8)) / 100.0;
  double e = (f[42] | (f[43] << 8)) / 100.0;
  if (e < s) e += 360.0;
  const float start = (float)pmod(s, 360.0);
  // ScanAssembler.add: a turn ends when the start angle wraps (drops by more than 180 deg)
  Turn& T = turns_[cur_];
  if (have_start_ && start < last_start_ - 180.0f) {
    if (started_ && T.n >= kMinPackets && !too_long_) {
      turnDone();
    } else if (started_ || T.n) {
      if (too_long_) n_.long_turn++;
      else n_.partial++;
    }
    started_ = true;
    turns_[cur_].n = 0;
    too_long_ = false;
  }
  have_start_ = true;
  last_start_ = start;
  Turn& C = turns_[cur_];
  if (C.n >= kMaxPackets) {
    too_long_ = true;
    return;
  }
  Packet& p = C.pk[C.n++];
  p.t = t;
  p.start = start;
  p.step = (float)((e - s) / (kPts - 1));
  for (int i = 0; i < kPts; i++) {
    p.dist[i] = (uint16_t)(f[6 + 3 * i] | (f[7 + 3 * i] << 8));
    p.qual[i] = f[8 + 3 * i];
  }
}

void LidarSectors::turnDone() {
  n_.turns++;
  if (stage_ != IDLE || queued_) {  // the turn before is still being processed: the newest wins
    n_.busy++;
    if (stage_ != IDLE) turn_end_ = true;
    stage_ = IDLE;
  }
  cur_ ^= 1;  // the finished turn is now turns_[cur_ ^ 1]
  queued_ = true;
}

// ------------------------------------------------------------------ processing (background slices)

// lidar.Lidar.deskew for one packet: false = tilted (the whole turn is not sent)
bool LidarSectors::deskewPacket(const Packet& pk) {
  double px, py, ph, pp;
  if (deskew_) {
    interp(idx_, pk.t + tco_, px, py, ph, pp);
  } else {
    px = fixed_[0];
    py = fixed_[1];
    ph = fixed_[2];
    pp = fixed_[3];
  }
  const float dp = (float)(pp - ref_);
  const float adp = fabsf(dp);
  if (adp > gate_ || fabs(pp) > kPitchAbsMax) return false;
  const float hr = (float)(ph * kRad);
  const float ch = cosf(hr), sh = sinf(hr);
  const float ox = (float)px + ch * sx_ - sh * sy_, oy = (float)py + sh * sx_ + ch * sy_;
  const float sgn = flip_ ? 1.0f : -1.0f;  // the LD19 counts clockwise; flipped over it counts CCW
  const float s = pk.start, step = pk.step;
  const float db = sgn * step;
  const float b0 = yaw_ + sgn * s;  // the packet's first beam, robot frame, deg
  const float b1 = b0 + (kPts - 1) * db;
  const float dpr = dp * kRad;
  // range * cos(elevation): horizontal (elevation = lean * cos(azimuth), here at the packet's middle beam)
  const float fh = dpr != 0.0f ? cosf(dpr * cosf(0.5f * kRad * (b0 + b1))) : 1.0f;
  float ux = fh * cosf(hr + b0 * kRad), uy = fh * sinf(hr + b0 * kRad);
  const float rc = cosf(db * kRad), rs = sinf(db * kRad);
  const float lo = b0 <= b1 ? b0 : b1, hi = b0 <= b1 ? b1 : b0;
  uint16_t mxd = 0;
  for (int k = 0; k < kPts; k++) mxd = pk.dist[k] > mxd ? pk.dist[k] : mxd;
  float lo_clip = kInf;  // no point of this packet closer than this can be a floor hit
  if (clip_ && dpr != 0.0f) {
    const float adpr = fabsf(dpr);
    if (adpr > 1e-4f) {
      lo_clip = fk_ / tanf(adpr);  // (the steepest beam of any packet)
      if (mxd > lo_clip) {         // this packet's own steepest beam
        const float cmax = ceilf(lo / 180.0f) * 180.0f <= hi ? 1.0f : fmaxf(fabsf(cosf(lo * kRad)), fabsf(cosf(hi * kRad)));
        lo_clip = adpr * cmax > 1e-4f ? fk_ / tanf(adpr * cmax) : kInf;
      }
    }
  }
  for (int k = 0; k < kPts; k++) {
    const float x = ux, y = uy;
    const float nx = ux * rc - uy * rs;  // the next beam
    uy = ux * rs + uy * rc;
    ux = nx;
    const float d = pk.dist[k];
    if (d <= 0) continue;
    if (d < rmin_ || d > rmax_) continue;
    if (pk.qual[k] < qmin_) continue;
    if (masked_) {
      int mi = (int)(pmodf(b0 + k * db, 360.0f) * 2.0f);
      if (mi > 719) mi = 719;
      if (mask_[mi]) continue;
    }
    if (d > lo_clip) {
      const float e = dpr * cosf((b0 + k * db) * kRad);  // beam elevation at this azimuth
      const float ae = fabsf(e);
      if (ae > 1e-4f && d * cosf(e) > fk_ / tanf(ae)) continue;
    }
    zx_[zn_] = ox + d * x;
    zy_[zn_] = oy + d * y;
    zn_++;
  }
  return true;
}

bool LidarSectors::service(uint32_t now_us, float half_mm) {
  if (stage_ == IDLE) {
    if (!queued_) return false;
    queued_ = false;
    stage_ = START;
    turn_cycles_ = 0;
    turn_end_ = false;
  }
  const Turn& T = turns_[cur_ ^ 1];
  switch (stage_) {
    case START: {
      const double now = sec(now_us);
      stage_ = IDLE;
      turn_end_ = true;
      if (!configured_) {
        n_.unbound++;  // no MSG_LIDAR_CONFIG yet: the mount of this robot is not known
        return false;
      }
      if (!enabled_) {
        n_.off++;
        return false;
      }
      t1_ = T.pk[T.n - 1].t;
      if (now - t1_ > kMaxScanAgeS) {
        n_.stale++;
        return false;
      }
      if (!pose_n_) {
        n_.no_pose++;
        return false;
      }
      newest_ = poseAt(pose_total_ - 1);
      if (newest_.t < t1_ + tco_ - kPoseStaleS || (now - newest_.t) * 1000.0 > LINK_LIDAR_MAX_AGE_MS) {
        n_.no_pose++;
        return false;
      }
      ref_ = pitchRef();
      half_ = half_mm;
      zn_ = 0;
      pk_i_ = 0;
      if (deskew_) {
        idx_ = seek(T.pk[0].t + tco_);
      } else {  // one pose for the whole turn: the one at its end
        uint32_t i = seek(t1_ + tco_);
        interp(i, t1_ + tco_, fixed_[0], fixed_[1], fixed_[2], fixed_[3]);
      }
      stage_ = DESKEW;
      turn_end_ = false;
      return false;
    }
    case DESKEW: {
      for (int k = 0; k < kSlicePackets && pk_i_ < T.n; k++, pk_i_++) {
        if (!deskewPacket(T.pk[pk_i_])) {
          n_.tilt++;  // one tilted packet: the whole turn is not sent
          stage_ = IDLE;
          turn_end_ = true;
          return false;
        }
      }
      if (pk_i_ < T.n) return false;
      if (zn_ < kMinPoints) {
        n_.few++;
        stage_ = IDLE;
        turn_end_ = true;
        return false;
      }
      const float h = newest_.h * kRad;
      nc_ = cosf(h);
      ns_ = sinf(h);
      for (int i = 0; i < LINK_LIDAR_SECTORS; i++) {
        best_d_[i] = kInf;
        best_i_[i] = ci_[i] = -1;
        ckey_[i] = 0;
      }
      pt_i_ = 0;
      stage_ = SECTORS;
      return false;
    }
    case SECTORS: {
      // the points (odometry frame) in the robot frame of the newest pose, about the axle; sector = floor(angle / 10 deg
      // + 0.5) mod 36; the nearest per sector, and the nearest inside the corridor (|y| <= half) by the navigator's front /
      // rear distance, which wins (#99)
      const float k_sec = LINK_LIDAR_SECTORS / (2.0f * 3.14159265358979f);
      const int end = pt_i_ + kSlicePoints < zn_ ? pt_i_ + kSlicePoints : zn_;
      for (int j = pt_i_; j < end; j++) {
        const float dx = zx_[j] - newest_.x, dy = zy_[j] - newest_.y;
        const float xr = dx * nc_ + dy * ns_, yr = -dx * ns_ + dy * nc_;
        zx_[j] = xr;
        zy_[j] = yr;
        const float dist = sqrtf(xr * xr + yr * yr);
        const int sec = (int)(atan2f(yr, xr) * k_sec + (LINK_LIDAR_SECTORS + 0.5f)) % LINK_LIDAR_SECTORS;
        if (dist < best_d_[sec]) {
          best_d_[sec] = dist;
          best_i_[sec] = (int16_t)j;
        }
        if (fabsf(yr) <= half_) {
          const float key = xr >= kAxisFwd ? xr - kFrontOff : kRearOff - xr;
          if (ci_[sec] < 0 || key < ckey_[sec]) {
            ckey_[sec] = key;
            ci_[sec] = (int16_t)j;
          }
        }
      }
      pt_i_ = end;
      if (pt_i_ >= zn_) stage_ = FINAL;
      return false;
    }
    case FINAL: {
      const double now = sec(now_us);
      LidarObstacleFrame& f = frame_;
      f.seq = seq_++;
      f.sectors = LINK_LIDAR_SECTORS;
      last_age_ms_ = (float)((now - newest_.t) * 1000.0);
      const float age = rintf(last_age_ms_);
      f.age_ms = age < 0 ? 0 : (age > 65535.0f ? 65535 : (uint16_t)age);
      for (int i = 0; i < LINK_LIDAR_SECTORS; i++) {
        const int j = ci_[i] >= 0 ? ci_[i] : best_i_[i];
        int16_t x = 0, y = 0;
        if (j >= 0) {
          x = i16(zx_[j]);
          y = i16(zy_[j]);
          if (!x && !y) x = 1;  // a return at the axle, not "none"
        }
        f.xy_mm[2 * i] = x;
        f.xy_mm[2 * i + 1] = y;
      }
      n_.sent++;
      frame_ready_ = true;
      stage_ = IDLE;
      turn_end_ = true;
      return true;
    }
    default:
      stage_ = IDLE;
      return false;
  }
}

bool LidarSectors::takeFrame(LidarObstacleFrame& f) {
  if (!frame_ready_) return false;
  f = frame_;
  frame_ready_ = false;
  return true;
}

void LidarSectors::noteCycles(uint32_t cycles) {
  if (cycles > max_slice_) max_slice_ = cycles;
  turn_cycles_ += cycles;
  if (turn_end_) {
    turn_end_ = false;
    last_turn_cycles_ = turn_cycles_;
    if (turn_cycles_ > max_turn_) max_turn_ = turn_cycles_;
    turn_cycles_ = 0;
  }
}

int LidarSectors::statusText(char* out, int n, uint32_t now_ms, uint32_t cycles_per_us) {
  if (!stat_started_) {
    stat_started_ = true;
    stat_ms_ = now_ms;
    stat_n_ = n_;
  }
  const float s = (now_ms - stat_ms_) * 0.001f;
  const float cpu = cycles_per_us ? (float)cycles_per_us : 1.0f;
  int k;
  if (!configured_) {
    k = snprintf(out, (size_t)n, "sectors: waiting for MSG_LIDAR_CONFIG");
  } else if (!enabled_) {
    k = snprintf(out, (size_t)n, "sectors: off (lidar disabled)");
  } else {
    k = snprintf(out, (size_t)n, "sectors %.1f/s, skip tilt %lu pose %lu stale %lu few %lu busy %lu, max %.0f us/slice %.0f us/turn",
                 (double)(s > 0 ? (n_.sent - stat_n_.sent) / s : 0.0f), (unsigned long)(n_.tilt - stat_n_.tilt),
                 (unsigned long)(n_.no_pose - stat_n_.no_pose), (unsigned long)(n_.stale - stat_n_.stale),
                 (unsigned long)(n_.few - stat_n_.few), (unsigned long)(n_.busy - stat_n_.busy),
                 (double)(max_slice_ / cpu), (double)(max_turn_ / cpu));
  }
  stat_ms_ = now_ms;
  stat_n_ = n_;
  max_slice_ = max_turn_ = 0;
  return k < 0 ? 0 : (k < n ? k : n - 1);
}
