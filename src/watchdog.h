// Hardware watchdog (issue #109): the i.MX RT1062's RTWDOG (WDOG3), clocked from the 32 kHz LPO, so it runs whatever
// the CPU does. Fed once per loop() pass, and only while the loop is healthy (wdt_healthy):
//   - a background() pass (Pi link poll, radio, the stale-IMU motor stop) ran within WDT_BG_MAX_US, and
//   - with the motors driven, a balance step ran within IMU_STALE_US + 10 ms.
// Timeout WDT_RUN_MS (100 ms) while the motors run, WDT_IDLE_MS (2 s) with them off (an EEPROM save or a sensor re-init
// may block that long, and only ever with the motors off). On timeout the RTWDOG interrupt first drives the six motor
// pins low (EN + IN = L298N off), then the chip resets ~2 us later; the boot path holds them low again from the first
// instructions (startup_early_hook, src/motors.cpp). A fault handler (CrashReport) runs with interrupts off: then only
// the reset itself acts (still within the timeout). The next boot reports LINK_RESET_WATCHDOG (src/reset_cause.h).
#pragma once
#include <stdint.h>

#include "config.h"

struct WdtIn {
  uint32_t now_us = 0;
  uint32_t bg_us = 0;      // last background() pass that polled the Pi link
  uint32_t step_us = 0;    // last balance step
  bool motors_active = false;  // the motors may be driven now: armed in a mode, balance loop or motor test running
};

// Pure: feed only if the loop is doing its job (host-tested, test/host/encoder_fault_sim.cpp).
inline bool wdt_healthy(const WdtIn& in) {
  if (in.now_us - in.bg_us > WDT_BG_MAX_US) return false;  // the link / radio / failsafes are starved
  if (in.motors_active && in.now_us - in.step_us > IMU_STALE_US + 10000u) return false;  // driving without a step
  return true;
}

// Timeout the watchdog should have now.
inline uint32_t wdt_timeout_for(bool motors_active) { return motors_active ? WDT_RUN_MS : WDT_IDLE_MS; }

#ifdef ARDUINO
bool wdt_begin();                // at the end of setup(): RTWDOG on (WDT_IDLE_MS). false = could not unlock it
void wdt_service(const WdtIn& in);  // every loop pass: timeout for the motor state, then feed if healthy
bool wdt_running();
// True when a blocking call (motors off) may run now: watchdog off, or on its idle timeout (reconfiguration done);
// feeds it first so the call gets the whole WDT_IDLE_MS.
bool wdt_can_block();
uint32_t wdt_timeout_ms();       // the timeout configured now (0 = off)
#endif
