"""Teensy resets (#109): an unexpected reset told from a normal boot, from the HELLO's reset cause (link v9).

The firmware reads SRC_SRSR + its SNVS marker at boot (src/reset_cause.h) and sends the cause in every HELLO of that
boot, with its uptime. A new boot = other reset facts, or an uptime smaller than in the last HELLO. Unexpected = a watchdog reset,
a crash, a lockup, a power dip (brownout), over-temperature - or a cold power-on while this bridge was already talking
to it and had not released the port for flashing.
Pure: time is passed in.
"""
from . import link_protocol as lp

def _dur(s):
    s = int(s)
    if s < 120:
        return "%d s" % s
    if s < 7200:
        return "%d min" % round(s / 60.0)
    return "%.1f h" % (s / 3600.0)


def cause_name(c):
    return lp.RESET_CAUSE_NAMES[c] if 0 <= c < len(lp.RESET_CAUSE_NAMES) else "cause %d" % c


class ResetTracker:
    def __init__(self):
        self.key = None        # the current boot's reset facts (cause, flags, srsr, previous uptime, off time)
        self.uptime_ms = 0     # uptime in its last HELLO: smaller in the next one = it rebooted
        self.boot_t = None     # Pi time the Teensy's current boot started
        self.boot = None       # {"cause", "name", "flags", "srsr", "prev_uptime_s", "off_s"} of the current boot
        self.last = None       # the last unexpected reset this bridge run: {"cause", "name", "text", "t"}
        self.unexpected = 0    # how many this bridge run
        self.released = False  # the serial port was released (flashing) since the last boot

    def note_released(self):
        self.released = True

    def on_hello(self, d, now):
        """A HELLO. -> ("warn" | "info", text) once per new boot; None for the same boot or firmware without the
        reset fields (link v8)."""
        if d.get("link_version", 0) < 9:
            return None
        up = d.get("uptime_ms", 0)
        key = tuple(d.get(k, 0) for k in ("reset_cause", "reset_flags", "srsr", "prev_uptime_s", "off_s"))
        first = self.key is None
        # the same boot: the same reset facts and the uptime still counting up
        if not first and key == self.key and up >= self.uptime_ms:
            self.uptime_ms = up
            return None
        self.key, self.uptime_ms, self.boot_t = key, up, now - up / 1000.0
        cause, flags = d.get("reset_cause", 0), d.get("reset_flags", 0)
        name = cause_name(cause)
        self.boot = {"cause": cause, "name": name, "flags": flags, "srsr": d.get("srsr", 0),
                     "prev_uptime_s": d.get("prev_uptime_s", 0), "off_s": d.get("off_s", 0xFFFFFFFF)}
        extra = []
        if flags & lp.LINK_RESETF_SNVS:
            extra.append("previous run %s" % _dur(d.get("prev_uptime_s", 0)))
            if d.get("off_s", 0xFFFFFFFF) != 0xFFFFFFFF:
                extra.append("off %s" % _dur(d["off_s"]))
        if flags & lp.LINK_RESETF_CRASH:
            extra.append("crash report in the robot log")
        extra.append("SRSR 0x%03x" % d.get("srsr", 0))
        released, self.released = self.released, False
        unexpected = (cause in lp.RESET_UNEXPECTED or bool(flags & lp.LINK_RESETF_CRASH) or
                      (not first and cause == lp.LINK_RESET_POWER_ON and not released))
        if not unexpected:
            return ("info", "Teensy boot: %s (%s)" % (name, ", ".join(extra)))
        if cause == lp.LINK_RESET_POWER_ON:
            name = "power-on (power was lost: battery off, or a long brownout)"
        text = "Teensy reset unexpectedly: %s (%s)" % (name, ", ".join(extra))
        self.last = {"cause": cause, "name": name, "text": text, "t": now}
        self.unexpected += 1
        return ("warn", text)

    def status(self, now):
        """For the Health page: {"boot": ..., "last": ... + "ago_s", "count"}; None before any v9 HELLO."""
        if self.boot is None:
            return None
        last = dict(self.last, ago_s=round(now - self.last["t"])) if self.last else None
        return {"boot": self.boot, "last": last, "count": self.unexpected}
