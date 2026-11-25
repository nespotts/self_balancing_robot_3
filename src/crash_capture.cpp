#include "crash_capture.h"

#include <Arduino.h>

#include "motors.h"

bool g_crash_mapped = false;
static PrevRun g_prev;

#if SBR_CRASH_CAPTURE
// The fault handler's own stack (DTCM, uncached): a fault on an overflowed or corrupted stack is still recorded.
extern "C" {
uint32_t sbr_fault_stack[256] __attribute__((aligned(8), used));
void sbr_fault_entry(void);
void sbr_fault_c(uint32_t sp, uint32_t exc_return) __attribute__((noreturn, used));
}
extern unsigned long _ebss, _estack;

static inline volatile CrashRecord* rec() { return (volatile CrashRecord*)CRASH_RECORD_ADDR; }

// The 8-word exception frame at sp is readable: inside DTCM (below the stack top) or OCRAM, not in the MPU stack guard.
static bool frameReadable(uint32_t sp) {
  if (sp & 3) return false;
  const bool dtcm = sp >= 0x20000000u && sp + 32u <= (uint32_t)&_estack;
  const bool ocram = sp >= 0x20200000u && sp + 32u <= 0x20280000u;
  if (!dtcm && !ocram) return false;
  const uint32_t guard = (uint32_t)&_ebss & ~31u;  // 32-byte no-access region (startup.c configure_cache)
  if (sp < guard + 32u && sp + 32u > guard) return false;
  return true;
}

// Naked entry: frame pointer + EXC_RETURN in r0 / r1, then onto our own stack before any push.
extern "C" __attribute__((naked)) void sbr_fault_entry(void) {
  asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "ldr r2, =sbr_fault_stack + 1024\n"
      "mov sp, r2\n"
      "b.w sbr_fault_c\n"
      ".ltorg\n");
}

extern "C" void sbr_fault_c(uint32_t sp, uint32_t exc_return) {
  __disable_irq();
  motors_emergency_off();  // register stores only
  uint32_t ipsr;
  asm volatile("mrs %0, ipsr" : "=r"(ipsr));
  uint32_t pc = 0xFFFFFFFFu, lr = 0xFFFFFFFFu, xpsr = 0;
  const bool ok = frameReadable(sp);
  if (ok) {
    const volatile uint32_t* f = (const volatile uint32_t*)sp;
    lr = f[5];
    pc = f[6];
    xpsr = f[7];
  }
  volatile CrashRecord* r = rec();
  // the core's CrashReport block (same layout + CRC as startup.c unused_interrupt_vector): CrashReport stays valid
  struct arm_fault_info_struct* info = (struct arm_fault_info_struct*)0x2027FF80;
  info->ipsr = ipsr;
  info->cfsr = SCB_CFSR;
  info->hfsr = SCB_HFSR;
  info->mmfar = SCB_MMFAR;
  info->bfar = SCB_BFAR;
  info->ret = pc;
  info->xpsr = xpsr;
  info->temp = tempmonGetTemp();
  info->time = r->uptime_ms / 1000u;  // uptime s (the core prints it as a time of day)
  info->len = sizeof(*info) / 4;
  uint32_t crc = 0xFFFFFFFFu;
  const uint32_t* p = (const uint32_t*)info;
  const uint32_t* end = p + (sizeof(*info) / 4 - 1);
  while (p < end) {
    crc ^= *p++;
    for (int i = 0; i < 32; i++) crc = (crc >> 1) ^ (crc & 1) * 0xEDB88320u;
  }
  info->crc = crc;
  arm_dcache_flush_delete(info, sizeof(*info));
  if (g_crash_mapped) {
    r->fault_lr = lr;
    r->fault_sp = sp;
    r->flags = (r->flags & 0x00FFFFFFu) | CRF_FAULT | (ok ? 0u : CRF_FRAME_BAD) | ((exc_return & 0xFFu) << 24);
    r->phases = (r->phases << 8) | PH_FAULT;
  }
  asm volatile("dsb" ::: "memory");
  SRC_GPR5 = 0x0BAD00F1;   // the core's "fault handler rebooted" marker
  SCB_AIRCR = 0x05FA0004;  // reset now (the core's handler waits 8 s; the robot should come back at once)
  while (1) {
  }
}
#endif

void crash_capture_begin(uint32_t build_hash, uint32_t rtc_now) {
#if SBR_CRASH_CAPTURE
  // MPU region 15 (the core uses 0..10): the record's 32 bytes normal memory, NON-cacheable, no execute
  __disable_irq();
  SCB_MPU_RBAR = CRASH_RECORD_ADDR | SCB_MPU_RBAR_VALID | SCB_MPU_RBAR_REGION(15);
  SCB_MPU_RASR = SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_AP(3) | SCB_MPU_RASR_XN | SCB_MPU_RASR_SIZE(4) |
                 SCB_MPU_RASR_ENABLE;
  asm volatile("dsb\nisb" ::: "memory");
  __enable_irq();
  arm_dcache_delete((void*)CRASH_RECORD_ADDR, 32);  // drop any (clean) cached copy: reads go to the RAM now
  g_crash_mapped = true;

  volatile CrashRecord* r = rec();
  if (r->magic == CRASH_RECORD_MAGIC) {
    g_prev.valid = true;
    g_prev.build_hash = r->build_hash;
    g_prev.uptime_ms = r->uptime_ms;
    g_prev.phases = r->phases;
    g_prev.flags = r->flags;
    g_prev.fault_lr = r->fault_lr;
    g_prev.fault_sp = r->fault_sp;
    g_prev.boot_rtc = r->boot_rtc;
  }
  r->build_hash = build_hash;
  r->uptime_ms = millis();
  r->phases = PH_SETUP;
  r->flags = 0;
  r->fault_lr = 0;
  r->fault_sp = 0;
  r->boot_rtc = rtc_now;
  r->magic = CRASH_RECORD_MAGIC;

  _VectorsRam[2] = sbr_fault_entry;  // NMI
  _VectorsRam[3] = sbr_fault_entry;  // HardFault
  _VectorsRam[4] = sbr_fault_entry;  // MemManage
  _VectorsRam[5] = sbr_fault_entry;  // BusFault
  _VectorsRam[6] = sbr_fault_entry;  // UsageFault
  asm volatile("dsb" ::: "memory");
#else
  (void)build_hash;
  (void)rtc_now;
#endif
}

const PrevRun& crash_prev() { return g_prev; }
volatile CrashRecord* crash_record() { return g_crash_mapped ? (volatile CrashRecord*)CRASH_RECORD_ADDR : nullptr; }
bool crash_active() { return g_crash_mapped; }

void crash_tick(uint32_t now_ms) {
  if (g_crash_mapped) ((volatile CrashRecord*)CRASH_RECORD_ADDR)->uptime_ms = now_ms;
}

void crash_set_flag(uint32_t f) {
  if (g_crash_mapped) ((volatile CrashRecord*)CRASH_RECORD_ADDR)->flags |= f;
}

void crash_clear_flag(uint32_t f) {
  if (g_crash_mapped) ((volatile CrashRecord*)CRASH_RECORD_ADDR)->flags &= ~f;
}
