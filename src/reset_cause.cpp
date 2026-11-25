#include "reset_cause.h"

#include <Arduino.h>
#include <CrashReport.h>
#include <string.h>

// SNVS general-purpose registers (kept while the SNVS domain has power, through every reset)
//   LPGPR0 = marker, LPGPR1 = build hash, LPGPR2 = RTC seconds at the last tick,
//   LPGPR3 = uptime seconds at the last tick (bit 31: the firmware asked for this reset)
static constexpr uint32_t kMarker = 0x53425233u;  // "SBR3"
static constexpr uint32_t kPlanned = 0x80000000u;

static ResetInfo g_info;
static uint32_t g_hash = 0;

static uint32_t fnv1a(const char* s) {
  uint32_t h = 2166136261u;
  while (*s) h = (h ^ (uint8_t)*s++) * 16777619u;
  return h;
}

// Print sink that keeps a one-line summary of the CrashReport text: lines joined with " | ", tabs / repeats squeezed.
class CrashSink : public Print {
 public:
  CrashSink(char* dst, size_t cap) : d_(dst), cap_(cap) { d_[0] = 0; }
  size_t write(uint8_t c) override {
    if (c == '\r') return 1;
    if (c == '\n' || c == '\t') {
      pend_sep_ = c == '\n' ? 2 : (pend_sep_ ? pend_sep_ : 1);
      return 1;
    }
    if (c < 0x20 || c > 0x7E) return 1;  // the degree sign etc.
    if (pend_sep_ && n_) {
      if (pend_sep_ == 2) put(' '), put('|'), put(' ');
      else put(' ');
    }
    pend_sep_ = 0;
    put((char)c);
    return 1;
  }

 private:
  void put(char c) {
    if (n_ + 1 < cap_) {
      d_[n_++] = c;
      d_[n_] = 0;
    }
  }
  char* d_;
  size_t cap_, n_ = 0;
  uint8_t pend_sep_ = 0;
};

void reset_cause_capture(const char* build_id) {
  ResetFacts f;
  f.srsr = SRC_SRSR;
  f.gpr5 = SRC_GPR5;
  SRC_SRSR = f.srsr;  // write 1 to clear: the next boot sees only its own cause
  SRC_GPR5 = 0;
  g_hash = fnv1a(build_id);
  const uint32_t now_s = rtc_get();
  if (SNVS_LPGPR0 == kMarker) {
    f.snvs_ok = true;
    f.same_build = SNVS_LPGPR1 == g_hash;
    f.planned = (SNVS_LPGPR3 & kPlanned) != 0;
    const uint32_t last = SNVS_LPGPR2;
    f.off_s = now_s >= last ? now_s - last : 0xFFFFFFFFu;
    g_info.prev_uptime_s = SNVS_LPGPR3 & ~kPlanned;
    g_info.off_s = f.off_s;
    g_info.flags |= LINK_RESETF_SNVS;
  }
  g_info.srsr = f.srsr;
  g_info.cause = reset_classify(f);
  if (CrashReport) {  // a fault was recorded before this boot (CrashReport also clears SRSR / GPR5: read first above)
    g_info.flags |= LINK_RESETF_CRASH;
    CrashSink sink(g_info.crash, sizeof(g_info.crash));
    CrashReport.printTo(sink);
  }
  SNVS_LPGPR0 = kMarker;  // this run's marker from now on
  SNVS_LPGPR1 = g_hash;
  SNVS_LPGPR2 = now_s;
  SNVS_LPGPR3 = 0;
}

const ResetInfo& reset_info() { return g_info; }

void reset_cause_tick(uint32_t now_ms) {
  SNVS_LPGPR2 = rtc_get();
  SNVS_LPGPR3 = (SNVS_LPGPR3 & kPlanned) | ((now_ms / 1000u) & ~kPlanned);
}

void reset_cause_mark_planned() { SNVS_LPGPR3 |= kPlanned; }

void reset_cause_set_wdt(bool running) {
  if (running) g_info.flags |= LINK_RESETF_WDT;
  else g_info.flags &= (uint8_t)~LINK_RESETF_WDT;
}
