// Crash capture that survives the reset (issue #109 follow-up: v9 reset every ~30 s with nothing logged).
//
// A 32-byte record at the very top of OCRAM (0x2027FFE0, the cache row above the core's CrashReport data at
// 0x2027FF80..FFDF), made non-cacheable with MPU region 15 so a plain store reaches the RAM at once: nothing is lost
// in the D-cache when the chip resets, and a phase update costs a few cycles. RAM keeps its contents through every
// warm reset (lockup, SCB_AIRCR, watchdog); a power cycle scrambles it (the magic + build hash then do not match).
//
//   uptime_ms   millis() of the last loop pass        -> "previous run N s" (no SNVS needed)
//   phases      the last 4 phases entered (crash_phase), newest in the low byte -> where a lockup / hang hit
//   flags       reboot requested, RTWDOG interrupt fired, fault captured, ...
//   fault_lr / fault_sp   from our own fault handler (the core's CrashReport has PC, CFSR, HFSR, BFAR, MMFAR)
//
// The fault handler (HardFault / MemManage / BusFault / UsageFault) switches to its own stack first, so even a fault
// on a wrecked stack is recorded; it fills the core's CrashReport block (valid CRC) + this record, drives the motor
// pins low and resets at once (the core's handler waits 8 s, which the 2 s RTWDOG would cut short).
#pragma once
#include <stdint.h>

#include "config.h"

enum CrashPhase : uint8_t {
  PH_NONE = 0,
  PH_SETUP,
  PH_LOOP,       // loop() top
  PH_WDT,        // wdt_service (RTWDOG refresh / reconfigure)
  PH_IMU_POLL,
  PH_STEP,       // controlStep: modes / nav / controller / motors
  PH_ENC_L,      // left AS5600 read (Wire / LPI2C1, shared with the range sweep)
  PH_ENC_R,      // right AS5600 read (Wire1 / LPI2C3)
  PH_ENC_REC,    // I2C bus recovery (bit-banged clocks + Wire.begin)
  PH_SWEEP,
  PH_BG,         // background(): radio poll, modes, motor test
  PH_LINK,       // Pi link poll + output (USB serial)
  PH_ENC_LOG,
  PH_SNVS,       // SNVS marker tick (SBR_SNVS_MARKER)
  PH_BATT,       // battery ADC + reboot button
  PH_IMU_SVC,    // imu.service / radio.service (may block, motors off)
  PH_SLOW,       // sweep.serviceSlow, lidar status, params save / defaults
  PH_LIDAR,      // LD19 forwarding (Serial7)
  PH_WDOG_ISR,   // inside the RTWDOG interrupt
  PH_FAULT,      // inside our fault handler
  PH_SECTORS,    // LD19 obstacle sectors on the Teensy (#113: a turn's deskew / sector slices, the navigator feed)
  PH_COUNT
};

inline const char* crash_phase_name(uint8_t p) {
  static const char* const k[] = {"-",     "setup", "loop",  "wdt",   "imu-poll", "step",   "enc-L",
                                  "enc-R", "i2c-recover", "sweep", "bg", "link", "enc-log", "snvs",
                                  "batt",  "imu/radio-svc", "slow", "lidar", "WDOG-ISR", "FAULT", "sectors"};
  return p < PH_COUNT ? k[p] : "?";
}

// CrashRecord.flags
#define CRF_PLANNED 0x01u      // the firmware asked for this reset (reboot button)
#define CRF_WDOG_ISR 0x02u     // the RTWDOG interrupt ran (timeout): the reset that followed is the watchdog's
#define CRF_WDOG_NORESET 0x04u // ... and the RTWDOG did not reset the chip within 1 ms: our ISR did (SCB_AIRCR)
#define CRF_FAULT 0x08u        // our fault handler ran (fault_lr / fault_sp + the core's CrashReport block valid)
#define CRF_FRAME_BAD 0x10u    // ... with an unreadable exception frame (stack pointer outside RAM / in the guard)

struct CrashRecord {   // 32 bytes = one cache row, at CRASH_RECORD_ADDR
  uint32_t magic;
  uint32_t build_hash;
  uint32_t uptime_ms;
  uint32_t phases;     // last 4 phases, newest in bits 0..7
  uint32_t flags;      // CRF_*; bits 24..31: EXC_RETURN low byte of a captured fault
  uint32_t fault_lr;
  uint32_t fault_sp;
  uint32_t boot_rtc;   // RTC seconds at that run's boot (off time = this boot's RTC - boot_rtc - uptime)
};
static_assert(sizeof(CrashRecord) == 32, "CrashRecord must be one 32-byte cache row");
#define CRASH_RECORD_ADDR 0x2027FFE0u
#define CRASH_RECORD_MAGIC 0x50524253u  // "SBRP"

// What the previous run left (read at boot, before this run's record is started).
struct PrevRun {
  bool valid = false;        // magic found: RAM kept its contents (no power cycle)
  uint32_t build_hash = 0, uptime_ms = 0, phases = 0, flags = 0, fault_lr = 0, fault_sp = 0, boot_rtc = 0;
};

#ifdef ARDUINO
// setup(), first thing: read the previous record, start this run's, install the fault handlers (SBR_CRASH_CAPTURE).
void crash_capture_begin(uint32_t build_hash, uint32_t rtc_now);
const PrevRun& crash_prev();
volatile CrashRecord* crash_record();
// Cheap (one uncached store): call on entering a phase.
inline void crash_phase(uint8_t p) {
#if SBR_CRASH_CAPTURE
  extern bool g_crash_mapped;
  if (!g_crash_mapped) return;
  volatile CrashRecord* r = (volatile CrashRecord*)CRASH_RECORD_ADDR;
  const uint32_t v = r->phases;
  if ((uint8_t)v != p) r->phases = (v << 8) | p;
#else
  (void)p;
#endif
}
void crash_tick(uint32_t now_ms);   // every loop pass: uptime
void crash_set_flag(uint32_t f);    // CRF_*
void crash_clear_flag(uint32_t f);
bool crash_active();                // the record is mapped (crash_capture_begin ran)
#endif
