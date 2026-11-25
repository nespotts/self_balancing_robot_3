#include "watchdog.h"

#include <Arduino.h>

#include "motors.h"

// RTWDOG (WDOG3) CS register bits (i.MX RT1060 reference manual, RTWDOG chapter)
static constexpr uint32_t CS_UPDATE = 1u << 5;    // the config may be changed again later (unlock + rewrite)
static constexpr uint32_t CS_INT = 1u << 6;       // interrupt on timeout, reset 255 bus clocks later
static constexpr uint32_t CS_EN = 1u << 7;
static constexpr uint32_t CS_CLK_LPO = 1u << 8;   // CLK = 01: the 32 kHz low-power oscillator
static constexpr uint32_t CS_RCS = 1u << 10;      // reconfiguration done
static constexpr uint32_t CS_ULK = 1u << 11;      // unlocked
static constexpr uint32_t CS_CMD32EN = 1u << 13;  // 32-bit refresh / unlock words
static constexpr uint32_t CS_FLG = 1u << 14;      // interrupt flag (write 1 to clear)
static constexpr uint32_t UNLOCK_KEY = 0xD928C520u, REFRESH_KEY = 0xB480A602u;
static constexpr uint32_t LPO_HZ = 32000;         // nominal (+-few %): 100 ms = 3200 ticks

static bool g_on = false;
static uint32_t g_timeout_ms = 0;
static uint32_t g_fed_us = 0;

static void wdogIsr() {
  motors_emergency_off();  // EN + IN low before the reset lets the pins float (the keepers then hold them low)
  while (1) {}             // the reset follows within 255 bus clocks
}

static void writeKey(uint32_t key) {
  if (WDOG3_CS & CS_CMD32EN) {
    WDOG3_CNT = key;
  } else {  // 16-bit sequence (CMD32EN off)
    volatile uint16_t* cnt = (volatile uint16_t*)&WDOG3_CNT;
    *cnt = (uint16_t)(key & 0xFFFF);
    *cnt = (uint16_t)(key >> 16);
  }
}

// Unlock + new timeout. Must finish within 128 bus clocks of the unlock: interrupts off.
static bool configure(uint32_t ms) {
  if (g_on && !(WDOG3_CS & CS_RCS)) return false;  // the previous reconfiguration has not finished yet
  uint32_t ticks = ms * (LPO_HZ / 1000u);
  if (ticks > 0xFFFF) ticks = 0xFFFF;               // TOVAL is 16 bits: ~2 s at 32 kHz
  __disable_irq();
  writeKey(UNLOCK_KEY);
  bool ok = false;
  for (int i = 0; i < 64; i++) {
    if (WDOG3_CS & CS_ULK) {
      ok = true;
      break;
    }
  }
  if (ok) {
    WDOG3_WIN = 0;
    WDOG3_TOVAL = ticks;
    WDOG3_CS = CS_CMD32EN | CS_CLK_LPO | CS_EN | CS_UPDATE | CS_INT | CS_FLG;
  }
  __enable_irq();
  if (ok) {
    g_on = true;
    g_timeout_ms = ms;
  }
  return ok;
}

bool wdt_begin() {
  CCM_CCGR5 |= CCM_CCGR5_WDOG3(CCM_CCGR_ON);
  // the WDOG3 reset must reach the chip: SRC_SCR.MASK_WDOG3_RST = 1010 (not masked; 0101 = masked)
  SRC_SCR = (SRC_SCR & ~SRC_SCR_MASK_WDOG3_RST(0xF)) | SRC_SCR_MASK_WDOG3_RST(0xA);
  attachInterruptVector(IRQ_RTWDOG, wdogIsr);
  NVIC_SET_PRIORITY(IRQ_RTWDOG, 0);  // above everything else that can be interrupted
  NVIC_ENABLE_IRQ(IRQ_RTWDOG);
  if (!configure(WDT_IDLE_MS)) return false;
  writeKey(REFRESH_KEY);
  g_fed_us = micros();
  return true;
}

void wdt_service(const WdtIn& in) {
  if (!g_on) return;
  const uint32_t want = wdt_timeout_for(in.motors_active);
  if (want != g_timeout_ms) configure(want);  // retried next pass if a reconfiguration is still running
  if (!wdt_healthy(in)) return;                // starve it: the reset comes within the timeout
  if (in.now_us - g_fed_us < 1000) return;     // once a millisecond is plenty
  if (!(WDOG3_CS & CS_RCS)) return;            // no refresh during a reconfiguration
  g_fed_us = in.now_us;
  __disable_irq();
  writeKey(REFRESH_KEY);
  __enable_irq();
}

bool wdt_can_block() {
  if (!g_on) return true;
  if (g_timeout_ms < WDT_IDLE_MS || !(WDOG3_CS & CS_RCS)) return false;
  g_fed_us = micros();
  __disable_irq();
  writeKey(REFRESH_KEY);
  __enable_irq();
  return true;
}

bool wdt_running() { return g_on; }
uint32_t wdt_timeout_ms() { return g_on ? g_timeout_ms : 0; }
