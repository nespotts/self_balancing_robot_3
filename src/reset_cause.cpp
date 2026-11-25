#include "reset_cause.h"

#include <Arduino.h>
#include <CrashReport.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "crash_capture.h"

static ResetInfo g_info;
static uint32_t g_hash = 0;

static uint32_t fnv1a(const char* s) {
  uint32_t h = 2166136261u;
  while (*s) h = (h ^ (uint8_t)*s++) * 16777619u;
  return h;
}

#if SBR_SNVS_MARKER
// v9 (SBR_SNVS_MARKER 1): SNVS general-purpose registers, kept while the SNVS domain has power
//   LPGPR0 = marker, LPGPR1 = build hash, LPGPR2 = RTC seconds at the last tick,
//   LPGPR3 = uptime seconds at the last tick (bit 31: the firmware asked for this reset)
static constexpr uint32_t kMarker = 0x53425233u;  // "SBR3"
static constexpr uint32_t kPlanned = 0x80000000u;
static constexpr uint32_t kGprZDis = 1u << 24;    // SNVS_LPCR.GPR_Z_DIS: no zeroization on a security violation
#endif

// The main CFSR bit, by name (ARMv7-M: MMFSR 0..7, BFSR 8..15, UFSR 16..31).
static const char* cfsrName(uint32_t c) {
  static const struct { uint32_t bit; const char* n; } k[] = {
      {1u << 0, "IACCVIOL"},   {1u << 1, "DACCVIOL"},  {1u << 3, "MUNSTKERR"}, {1u << 4, "MSTKERR"},
      {1u << 5, "MLSPERR"},    {1u << 8, "IBUSERR"},   {1u << 9, "PRECISERR"}, {1u << 10, "IMPRECISERR"},
      {1u << 11, "UNSTKERR"},  {1u << 12, "STKERR"},   {1u << 13, "LSPERR"},   {1u << 16, "UNDEFINSTR"},
      {1u << 17, "INVSTATE"},  {1u << 18, "INVPC"},    {1u << 19, "NOCP"},     {1u << 24, "UNALIGNED"},
      {1u << 25, "DIVBYZERO"}};
  for (const auto& e : k)
    if (c & e.bit) return e.n;
  return c ? "?" : "none";
}

void reset_cause_capture(const char* build_id) {
  ResetFacts f;
  f.srsr = SRC_SRSR;
  f.gpr5 = SRC_GPR5;
  g_hash = fnv1a(build_id);
  const uint32_t now_s = rtc_get();

  // The core's CrashReport block, raw (written by the core's fault handler or by ours, src/crash_capture.cpp)
  const bool core_crash = (bool)CrashReport;
  arm_fault_info_struct fi;
  memcpy(&fi, (const void*)0x2027FF80, sizeof(fi));

  // The previous run's crash record (uncached RAM), then this run's record + our fault handlers
  crash_capture_begin(g_hash, now_s);
  const PrevRun& p = crash_prev();
  if (p.valid) {
    f.snvs_ok = true;
    f.same_build = p.build_hash == g_hash;
    f.planned = (p.flags & CRF_PLANNED) != 0;
    f.wdog_isr = (p.flags & CRF_WDOG_ISR) != 0;
    f.fault = (p.flags & CRF_FAULT) != 0;
    const uint32_t last = p.boot_rtc + p.uptime_ms / 1000u;
    f.off_s = now_s >= last ? now_s - last : 0xFFFFFFFFu;
    g_info.prev_uptime_s = p.uptime_ms / 1000u;
    g_info.off_s = f.off_s;
    g_info.flags |= LINK_RESETF_SNVS;
    snprintf(g_info.last, sizeof(g_info.last),
             "previous run %lu.%03lu s, last phases (newest first) %s < %s < %s < %s, flags 0x%02lx%s%s%s%s%s",
             (unsigned long)(p.uptime_ms / 1000u), (unsigned long)(p.uptime_ms % 1000u),
             crash_phase_name(p.phases & 0xFF), crash_phase_name((p.phases >> 8) & 0xFF),
             crash_phase_name((p.phases >> 16) & 0xFF), crash_phase_name(p.phases >> 24),
             (unsigned long)(p.flags & 0xFF), (p.flags & CRF_PLANNED) ? " requested" : "",
             (p.flags & CRF_WDOG_ISR) ? " WDOG-IRQ" : "", (p.flags & CRF_WDOG_NORESET) ? " WDOG-NO-RESET" : "",
             (p.flags & CRF_FAULT) ? " FAULT" : "", (p.flags & CRF_FRAME_BAD) ? " BAD-STACK" : "");
  }

#if SBR_SNVS_MARKER
  SNVS_LPCR |= kGprZDis;  // keep the LPGPRs through an SNVS security violation (v9 left them zeroizable)
  const bool m = SNVS_LPGPR0 == kMarker;
  snprintf(g_info.snvs, sizeof(g_info.snvs), "snvs marker %s (uptime %lu s)", m ? "kept" : "LOST",
           m ? (unsigned long)(SNVS_LPGPR3 & ~kPlanned) : 0ul);
  if (!p.valid && m) {  // no RAM record (power cycle, or capture off): the v9 SNVS facts
    f.snvs_ok = true;
    f.same_build = SNVS_LPGPR1 == g_hash;
    f.planned = (SNVS_LPGPR3 & kPlanned) != 0;
    const uint32_t last = SNVS_LPGPR2;
    f.off_s = now_s >= last ? now_s - last : 0xFFFFFFFFu;
    g_info.prev_uptime_s = SNVS_LPGPR3 & ~kPlanned;
    g_info.off_s = f.off_s;
    g_info.flags |= LINK_RESETF_SNVS;
  }
  SNVS_LPGPR0 = kMarker;
  SNVS_LPGPR1 = g_hash;
  SNVS_LPGPR2 = now_s;
  SNVS_LPGPR3 = 0;
#endif

  SRC_SRSR = f.srsr;  // write 1 to clear: the next boot sees only its own cause
  SRC_GPR5 = 0;
  g_info.srsr = f.srsr;
  g_info.cause = reset_classify(f);

  if (core_crash) {
    g_info.flags |= LINK_RESETF_CRASH;
    char lr[12] = "?", sp[12] = "?";  // only our handler saves them
    if (p.valid && (p.flags & CRF_FAULT)) {
      snprintf(lr, sizeof(lr), "%08lx", (unsigned long)p.fault_lr);
      snprintf(sp, sizeof(sp), "%08lx", (unsigned long)p.fault_sp);
    }
    snprintf(g_info.crash, sizeof(g_info.crash),
             "PC %08lx LR %s SP %s exc %02lx | CFSR %08lx (%s) HFSR %08lx MMFAR %08lx BFAR %08lx xPSR %08lx "
             "IPSR %lu | at %lu s, %.1f C",
             (unsigned long)fi.ret, lr, sp, (unsigned long)(p.flags >> 24), (unsigned long)fi.cfsr, cfsrName(fi.cfsr),
             (unsigned long)fi.hfsr, (unsigned long)fi.mmfar, (unsigned long)fi.bfar, (unsigned long)fi.xpsr,
             (unsigned long)fi.ipsr, (unsigned long)fi.time, (double)fi.temp);
    CrashReport.clear();  // consumed (also clears SRSR / GPR5: read above)
  }
}

const ResetInfo& reset_info() { return g_info; }

void reset_cause_tick(uint32_t now_ms) {
#if SBR_SNVS_MARKER
  static uint32_t last_ms = 0;
  if (now_ms - last_ms < 1000) return;
  last_ms = now_ms;
  crash_phase(PH_SNVS);
  SNVS_LPGPR2 = rtc_get();
  SNVS_LPGPR3 = (SNVS_LPGPR3 & kPlanned) | ((now_ms / 1000u) & ~kPlanned);
#else
  (void)now_ms;
#endif
}

void reset_cause_mark_planned() {
  crash_set_flag(CRF_PLANNED);
#if SBR_SNVS_MARKER
  SNVS_LPGPR3 |= kPlanned;
#endif
}

void reset_cause_set_wdt(bool running) {
  if (running) g_info.flags |= LINK_RESETF_WDT;
  else g_info.flags &= (uint8_t)~LINK_RESETF_WDT;
}
