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
static const SPISettings kSpi(IMU_SPI_HZ, MSBFIRST, SPI_MODE3);

// ---------------------------------------------------------------- SH-2 HAL (SPI, non-blocking)

static bool waitInt(uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (digitalReadFast(pins::IMU_INT) != LOW) {
    if (micros() - t0 > timeout_us) return false;
  }
  return true;
}

static int halOpen(sh2_Hal_t*) {
  // hardware reset, then wait for the chip to signal it is up
  digitalWriteFast(pins::IMU_RST, LOW);
  delay(10);
  digitalWriteFast(pins::IMU_RST, HIGH);
  delay(10);
  return waitInt(500000) ? 0 : -1;
}

static void halClose(sh2_Hal_t*) {}

// Read one SHTP transfer in a single CS window: 4-byte header, then the rest.
static int halRead(sh2_Hal_t*, uint8_t* buf, unsigned len, uint32_t* t_us) {
  if (digitalReadFast(pins::IMU_INT) != LOW) return 0;  // nothing waiting: never block
  *t_us = micros();
  SPI.beginTransaction(kSpi);
  digitalWriteFast(pins::IMU_CS, LOW);
  for (int i = 0; i < 4; i++) buf[i] = SPI.transfer(0);
  uint16_t n = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) & 0x7FFF;
  int result = 0;
  if (n > 4 && n <= len) {
    memset(buf + 4, 0, n - 4);
    SPI.transfer(buf + 4, n - 4);
    result = n;
  } else if (n > len) {
    for (uint16_t i = 4; i < n; i++) SPI.transfer(0);  // drain a packet we can't hold
  }
  digitalWriteFast(pins::IMU_CS, HIGH);
  SPI.endTransaction();
  return result;
}

// Writes need the chip awake (INT asserted). Only used for configuration.
static int halWrite(sh2_Hal_t*, uint8_t* buf, unsigned len) {
  if (!waitInt(200000)) return 0;
  SPI.beginTransaction(kSpi);
  digitalWriteFast(pins::IMU_CS, LOW);
  for (unsigned i = 0; i < len; i++) SPI.transfer(buf[i]);
  digitalWriteFast(pins::IMU_CS, HIGH);
  SPI.endTransaction();
  return (int)len;
}

static uint32_t halTime(sh2_Hal_t*) { return micros(); }

static sh2_Hal_t g_hal = {halOpen, halClose, halRead, halWrite, halTime};

static void onSh2Event(void*, sh2_AsyncEvent_t* ev) {
  if (ev->eventId == SH2_RESET && g_imu) g_imu->onReset();
}

static void onSh2Sensor(void*, sh2_SensorEvent_t* ev) {
  sh2_SensorValue_t v;
  if (sh2_decodeSensorEvent(&v, ev) != SH2_OK) return;
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
  if (sh2_open(&g_hal, onSh2Event, nullptr) != SH2_OK) return false;
  sh2_ProductIds_t ids;
  if (sh2_getProdIds(&ids) != SH2_OK) return false;
  sh2_setSensorCallback(onSh2Sensor, nullptr);
  ok_ = enableReports();
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
  return sh2_setSensorConfig(SH2_GYRO_INTEGRATED_RV, &cfg) == SH2_OK;
}

bool Imu::dataReady() const { return ok_ && digitalReadFast(pins::IMU_INT) == LOW; }

bool Imu::poll() {
  if (!dataReady()) return false;
  fresh_ = false;
  sh2_service();
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
  bool stalled = samples_ == 0 ? (now_ms - last_retry_ms_ > 1000) : !healthy(micros());
  if (wantedIntervalUs() != interval_us_) need_reports_ = true;  // ctl.rate_hz changed
  if ((need_reports_ || stalled) && now_ms - last_retry_ms_ > 1000) {
    last_retry_ms_ = now_ms;
    enableReports();
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
