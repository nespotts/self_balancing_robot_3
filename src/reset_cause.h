// Why the Teensy last reset (issue #109), read once at boot and sent in every HELLO (link v9) + a log line.
//
// SRC_SRSR holds the reset sources (sticky: cleared after reading, so the next boot sees only its own). The RT1062 has
// no brownout flag: a power dip is a power-on reset. The SNVS domain (RTC + 4 general-purpose registers) is told apart:
// every second the firmware writes a marker, its build hash, the RTC seconds and its uptime there. At boot:
//   marker gone                      -> the SNVS lost power too: a real power cycle (cold power-on)
//   marker there, off < 3 s          -> power dip / reset pin: brownout likely
//   another build hash               -> flashed
//   reboot-button flag               -> requested
// The Teensy core's CrashReport (fault address, CFSR, temperature) is logged too when it holds a crash.
#pragma once
#include <stdint.h>

#include "protocol/link_protocol.h"

// SRC_SRSR bits (i.MX RT1060 reference manual)
#define RST_SRSR_POR 0x001u         // IPP_RESET_B: power-on / reset pin (the Teensy's bootloader chip drives it)
#define RST_SRSR_LOCKUP_SW 0x002u   // LOCKUP_SYSRESETREQ: CPU lockup or SCB_AIRCR software reset
#define RST_SRSR_CSU 0x004u
#define RST_SRSR_USER 0x008u        // IPP_USER_RESET_B (power button; not wired on Teensy 4)
#define RST_SRSR_WDOG 0x010u        // WDOG1 / WDOG2
#define RST_SRSR_JTAG 0x020u
#define RST_SRSR_JTAG_SW 0x040u
#define RST_SRSR_WDOG3 0x080u       // RTWDOG: our hardware watchdog
#define RST_SRSR_TEMP 0x100u
#define RST_GPR5_FAULT 0x0BAD00F1u  // SRC_GPR5 set by the core's fault handler before its auto reboot

struct ResetFacts {
  uint32_t srsr = 0, gpr5 = 0;
  bool snvs_ok = false;     // the SNVS marker survived
  bool same_build = false;  // ... and holds this build's hash
  bool planned = false;     // ... and the "reboot requested" flag
  uint32_t off_s = 0xFFFFFFFFu;  // RTC seconds since the previous run's last tick (snvs_ok only)
};

// Pure (host-tested): the reset sources -> LinkResetCause.
inline uint8_t reset_classify(const ResetFacts& f) {
  if (f.srsr & RST_SRSR_TEMP) return LINK_RESET_TEMP;
  if (f.srsr & RST_SRSR_WDOG3) return LINK_RESET_WATCHDOG;
  if (f.srsr & RST_SRSR_LOCKUP_SW) {
    if (f.gpr5 == RST_GPR5_FAULT) return LINK_RESET_FAULT;
    if (f.snvs_ok && !f.same_build) return LINK_RESET_FLASHED;
    if (f.snvs_ok && f.planned) return LINK_RESET_REQUESTED;
    return LINK_RESET_LOCKUP;
  }
  if (f.srsr & (RST_SRSR_WDOG | RST_SRSR_JTAG | RST_SRSR_JTAG_SW | RST_SRSR_CSU)) return LINK_RESET_OTHER;
  if (f.snvs_ok && !f.same_build) return LINK_RESET_FLASHED;
  if (f.srsr & (RST_SRSR_POR | RST_SRSR_USER)) {
    if (f.snvs_ok && f.planned) return LINK_RESET_REQUESTED;
    if (f.snvs_ok && f.off_s < LINK_RESET_DIP_S) return LINK_RESET_POWER_DIP;
    return LINK_RESET_POWER_ON;
  }
  return LINK_RESET_UNKNOWN;
}

inline const char* reset_cause_name(uint8_t c) {
  static const char* const k[] = {"unknown", "power-on", "power dip (brownout?)", "WATCHDOG", "lockup / software reset",
                                  "crash (fault handler)", "requested", "flashed", "over-temperature", "other"};
  return c < LINK_RESET_CAUSE_COUNT ? k[c] : "?";
}

// An unexpected reset: worth a warning on the station (power-on and flashing are normal).
inline bool reset_unexpected(uint8_t c) {
  return c == LINK_RESET_POWER_DIP || c == LINK_RESET_WATCHDOG || c == LINK_RESET_LOCKUP || c == LINK_RESET_FAULT ||
         c == LINK_RESET_TEMP || c == LINK_RESET_OTHER;
}

#ifdef ARDUINO
struct ResetInfo {
  uint8_t cause = LINK_RESET_UNKNOWN;
  uint8_t flags = 0;          // LINK_RESETF_*
  uint32_t srsr = 0;
  uint32_t prev_uptime_s = 0;
  uint32_t off_s = 0xFFFFFFFFu;
  char crash[200] = {};       // short CrashReport text ("" = none)
};
// First thing in setup() (after the motor pins): read + clear SRSR / GPR5, read the SNVS marker, take the CrashReport.
void reset_cause_capture(const char* build_id);
const ResetInfo& reset_info();
void reset_cause_tick(uint32_t now_ms);  // once a second: the SNVS marker (RTC seconds + uptime)
void reset_cause_mark_planned();         // before a software reset the firmware asks for (reboot button)
void reset_cause_set_wdt(bool running);
#endif
