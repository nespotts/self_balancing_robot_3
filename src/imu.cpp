#include "imu.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "config.h"
#include "params.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

static Imu* g_imu = nullptr;

// --- bring-up diagnostics (Imu::debugString, logged by main)
static struct {
  int rc_open = 99, rc_cfg = 99, open_int = -1;
  uint32_t reads = 0, bytes = 0, too_big = 0, writes = 0, blind = 0, stashed = 0, resets = 0, begins = 0;
  uint32_t sensor_ev = 0, decode_err = 0, last_id = 0, events = 0, rode = 0, dropped = 0, repulse = 0;
  uint8_t hdr[4] = {0, 0, 0, 0};
} dbg;

static const SPISettings kSpi(IMU_SPI_HZ, MSBFIRST, SPI_MODE3);

// ---------------------------------------------------------------- SH-2 HAL (SPI, non-blocking)
//
// SHTP over SPI is full duplex: every CS window carries one transfer each way, each starting with
// a 4-byte header (length incl. header, channel, seq). The chip asserts INT when it has a transfer
// for us. Without a WAKE (PS0) line it only listens while INT is asserted, so writes ride on one of
// its transfers; what it sends during our write is kept and handed to SHTP by the next read.

static uint8_t g_stash[SH2_HAL_MAX_TRANSFER_IN];  // chip's transfer received during our write
static uint16_t g_stash_len = 0;
static uint32_t g_stash_us = 0;
static uint8_t g_tx[SH2_HAL_MAX_TRANSFER_OUT];    // write queued while the chip was idle
static uint16_t g_tx_len = 0;
static uint32_t g_tx_ms = 0;
static uint8_t g_wake = 0;          // 1 = chip reset for g_tx, waiting for its init response; 2 = send now
static bool g_own_reset = false;    // the next SH-2 reset event is the one we caused

// Teensy 4 LPSPI applies a new clock polarity only when the first byte is clocked. The nRF24 on
// the same bus uses mode 0 (SCK idles low), the BNO08x mode 3 (idles high): without this the SCK
// flip happens after CS falls, the chip sees an extra edge and every byte is shifted by one bit.
// So: switch modes with one dummy byte while no chip is selected, both ways.
static void spiBegin() {
  SPI.beginTransaction(kSpi);
  SPI.transfer(0);  // nobody selected: just settles SCK at the mode-3 idle level
  digitalWriteFast(pins::IMU_CS, LOW);
}

static void spiEnd() {
  digitalWriteFast(pins::IMU_CS, HIGH);
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0);  // back to the mode-0 idle level for the nRF24
  SPI.endTransaction();
}

static inline uint16_t hdrLen(const uint8_t* h) { return ((uint16_t)h[0] | ((uint16_t)h[1] << 8)) & 0x7FFF; }

static bool waitInt(uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (digitalReadFast(pins::IMU_INT) != LOW) {
    if (micros() - t0 > timeout_us) return false;
  }
  return true;
}

static void pulseReset() {
  digitalWriteFast(pins::IMU_RST, LOW);
  delay(10);
  digitalWriteFast(pins::IMU_RST, HIGH);
}

// One CS window: send tx (txlen may be 0) and receive the chip's transfer into rx.
// Returns the received length, 0 if none / too big for rx.
static int exchange(const uint8_t* tx, unsigned txlen, uint8_t* rx, unsigned rxmax) {
  uint8_t h[4];
  spiBegin();
  for (unsigned i = 0; i < 4; i++) h[i] = SPI.transfer(i < txlen ? tx[i] : 0);
  const uint16_t n = hdrLen(h);
  const bool keep = n >= 4 && n <= rxmax;
  const unsigned total = txlen > n ? txlen : n;
  for (unsigned i = 4; i < total; i++) {
    uint8_t b = SPI.transfer(i < txlen ? tx[i] : 0);
    if (keep && i < n) rx[i] = b;
  }
  spiEnd();
  dbg.reads++;
  memcpy(dbg.hdr, h, 4);
  if (!keep) {
    if (n > rxmax) dbg.too_big++;
    return 0;
  }
  memcpy(rx, h, 4);
  dbg.bytes += n;
  return n;
}

static int halOpen(sh2_Hal_t*) {
  g_stash_len = g_tx_len = 0;
  g_wake = 0;
  g_own_reset = false;
  pulseReset();
  bool up = waitInt(500000);  // chip asserts INT when it has booted (advertisement pending)
  dbg.open_int = up ? 1 : 0;
  return up ? 0 : -1;
}

static void halClose(sh2_Hal_t*) {}

// Returns at once with 0 if the chip has nothing for us.
static int halRead(sh2_Hal_t*, uint8_t* buf, unsigned len, uint32_t* t_us) {
  if (g_stash_len) {
    unsigned n = g_stash_len <= len ? g_stash_len : 0;
    memcpy(buf, g_stash, n);
    *t_us = g_stash_us;
    g_stash_len = 0;
    return (int)n;
  }
  if (digitalReadFast(pins::IMU_INT) != LOW) return 0;
  *t_us = micros();
  const bool send = g_tx_len && g_wake == 2;
  int n = exchange(send ? g_tx : nullptr, send ? g_tx_len : 0, buf, len);
  if (send) {
    g_tx_len = 0;
    g_wake = 0;
    dbg.rode++;
  }
  // after our reset: the control-channel init response means the hub is up; ride on the next one
  if (g_wake == 1 && n >= 5 && buf[2] == 2) g_wake = 2;
  return n;
}

static int halWrite(sh2_Hal_t*, uint8_t* buf, unsigned len) {
  if (g_tx_len || len > sizeof(g_tx)) return 0;
  dbg.writes++;
  // Normally reports are running and INT comes every sample period: ride on that transfer.
  if (waitInt(5000)) {
    int n = exchange(buf, len, g_stash, sizeof(g_stash));
    if (n) {
      g_stash_len = n;
      g_stash_us = micros();
      dbg.stashed++;
    }
    return (int)len;
  }
  // Idle chip (configuration time): queue the write and reset the chip. It sends its boot
  // transfers (advertisement, init response, reset complete); the write rides on the one after
  // the init response, when the hub is ready. (Riding on the advertisement gets ignored.)
  dbg.blind++;
  memcpy(g_tx, buf, len);
  g_tx_len = len;
  g_tx_ms = millis();
  g_wake = 1;
  g_own_reset = true;
  pulseReset();
  return (int)len;
}

// Called from Imu::poll while INT is high. A queued write whose reset produced no boot burst
// (seen on the first init after a Teensy reboot): pulse reset again, give up after 3 tries.
static void halTick() {
  static uint8_t tries = 0;
  if (!g_tx_len) {
    tries = 0;
    return;
  }
  if (millis() - g_tx_ms < 300) return;
  if (++tries > 3) {
    g_tx_len = 0;
    g_wake = 0;
    g_own_reset = false;
    dbg.dropped++;
    return;
  }
  dbg.repulse++;
  g_tx_ms = millis();
  g_wake = 1;
  g_own_reset = true;
  pulseReset();
}

static uint32_t halTime(sh2_Hal_t*) { return micros(); }

static sh2_Hal_t g_hal = {halOpen, halClose, halRead, halWrite, halTime};

static void onSh2Event(void*, sh2_AsyncEvent_t* ev) {
  dbg.events++;
  if (ev->eventId != SH2_RESET) return;
  dbg.resets++;
  if (g_own_reset) {
    g_own_reset = false;  // caused by a queued write; the write goes out with it
    return;
  }
  if (g_imu) g_imu->onReset();
}

static void onSh2Sensor(void*, sh2_SensorEvent_t* ev) {
  sh2_SensorValue_t v;
  dbg.sensor_ev++;
  dbg.last_id = ev->reportId;
  if (sh2_decodeSensorEvent(&v, ev) != SH2_OK) {
    dbg.decode_err++;
    return;
  }
  if (v.sensorId == SH2_GYRO_INTEGRATED_RV && g_imu) {
    const sh2_GyroIntegratedRV_t& r = v.un.gyroIntegratedRV;
    g_imu->onGyroRv(r.real, r.i, r.j, r.k, r.angVelX, r.angVelY, r.angVelZ);
  }
}

// ---------------------------------------------------------------- Imu

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

bool Imu::begin() {
  g_imu = this;
  pinMode(pins::IMU_CS, OUTPUT);
  digitalWriteFast(pins::IMU_CS, HIGH);
  pinMode(pins::IMU_RST, OUTPUT);
  digitalWriteFast(pins::IMU_RST, HIGH);
  pinMode(pins::IMU_INT, INPUT_PULLUP);
  ok_ = false;
  static bool opened = false;
  if (opened) sh2_close();  // retry: free the SHTP instance first
  opened = true;
  dbg.begins++;
  dbg.rc_cfg = 99;
  dbg.rc_open = sh2_open(&g_hal, onSh2Event, nullptr);
  if (dbg.rc_open != SH2_OK) return false;
  // (no sh2_getProdIds: that op has no timeout and hangs forever if the chip doesn't answer)
  sh2_setSensorCallback(onSh2Sensor, nullptr);
  ok_ = enableReports();
  last_retry_ms_ = millis();  // give it time to start before service() re-inits
  return ok_;
}

uint32_t Imu::wantedIntervalUs() const {
  float hz = P(P_CTL_RATE_HZ);
  if (hz < 50.0f) hz = 50.0f;
  return (uint32_t)(1e6f / hz + 0.5f);
}

bool Imu::enableReports() {
  sh2_SensorConfig_t cfg = {};
  interval_us_ = wantedIntervalUs();
  cfg.reportInterval_us = interval_us_;
  need_reports_ = false;
  dbg.rc_cfg = sh2_setSensorConfig(SH2_GYRO_INTEGRATED_RV, &cfg);
  return dbg.rc_cfg == SH2_OK;
}

int Imu::debugString(char* out, int size) const {
  return snprintf(out, size,
                  "imu ok=%d beg=%lu open=%d(int%d) cfg=%d INT=%d rd=%lu B=%lu big=%lu wr=%lu blind=%lu "
                  "stash=%lu rode=%lu drop=%lu rep=%lu ev=%lu rst=%lu sev=%lu derr=%lu id=0x%02lx hdr=%02x%02x.%02x.%02x "
                  "smp=%lu",
                  ok_, dbg.begins, dbg.rc_open, dbg.open_int, dbg.rc_cfg, digitalReadFast(pins::IMU_INT), dbg.reads,
                  dbg.bytes, dbg.too_big, dbg.writes, dbg.blind, dbg.stashed, dbg.rode, dbg.dropped, dbg.repulse, dbg.events,
                  dbg.resets,
                  dbg.sensor_ev, dbg.decode_err, dbg.last_id, dbg.hdr[0], dbg.hdr[1], dbg.hdr[2], dbg.hdr[3],
                  samples_);
}

bool Imu::dataReady() const { return ok_ && (g_stash_len || digitalReadFast(pins::IMU_INT) == LOW); }
bool Imu::poll() {
  if (!dataReady()) {
    if (ok_ && g_tx_len) halTick();
    return false;
  }
  fresh_ = false;
  sh2_service();
  // The chip reset itself: re-enable reports now, right after its boot burst, while it listens.
  if (need_reports_) enableReports();
  return fresh_;
}

bool Imu::healthy(uint32_t now_us) const {
  return ok_ && samples_ > 0 && (now_us - s_.t_us) < IMU_STALE_US;
}

void Imu::service(uint32_t now_ms, bool motors_off) {
  if (!motors_off) return;
  if (!ok_) {
    if (now_ms - last_retry_ms_ > 5000) {
      last_retry_ms_ = now_ms;
      begin();
    }
    return;
  }
  if (wantedIntervalUs() != interval_us_) {  // ctl.rate_hz changed: rides on the running reports
    enableReports();
    return;
  }
  // No samples (never started, or stopped): full re-init. The reset makes the chip listen.
  if (!healthy(micros()) && now_ms - last_retry_ms_ > 2000) {
    last_retry_ms_ = now_ms;
    begin();
  }
}

void Imu::onGyroRv(float qr, float qi, float qj, float qk, float wx, float wy, float wz) {
  // ZYX Euler angles (same convention the v3 firmware used)
  float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
  float yaw = atan2f(2.0f * (qi * qj + qk * qr), sqi - sqj - sqk + sqr);
  float s = -2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  float pitch = asinf(s);
  float roll = atan2f(2.0f * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr);

  // Euler rates from body rates: pitch' = q cos(roll) - r sin(roll),
  // yaw' = (q sin(roll) + r cos(roll)) / cos(pitch)
  float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch);
  if (cp < 0.05f) cp = 0.05f;
  s_.t_us = micros();
  s_.pitch_deg = pitch * RAD_TO_DEG;
  s_.roll_deg = roll * RAD_TO_DEG;
  s_.yaw_deg = yaw * RAD_TO_DEG;
  s_.gx_dps = wx * RAD_TO_DEG;
  s_.gy_dps = wy * RAD_TO_DEG;
  s_.gz_dps = wz * RAD_TO_DEG;
  s_.pitch_rate_dps = (wy * cr - wz * sr) * RAD_TO_DEG;
  s_.yaw_rate_dps = ((wy * sr + wz * cr) / cp) * RAD_TO_DEG;

  // continuous heading
  if (!have_yaw_) {
    last_yaw_ = s_.yaw_deg;
    heading_ = s_.yaw_deg;
    heading_offset_ = s_.yaw_deg;  // heading starts at 0
    have_yaw_ = true;
  } else {
    heading_ += wrap180(s_.yaw_deg - last_yaw_);
    last_yaw_ = s_.yaw_deg;
  }
  samples_++;
  fresh_ = true;
}