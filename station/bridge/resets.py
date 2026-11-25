"""Teensy resets (#109): an unexpected reset told from a normal boot, from the HELLO's reset cause (link v9).

The firmware reads SRC_SRSR + its SNVS marker at boot (src/reset_cause.h) and sends the cause in every HELLO of that
boot, with its uptime. A new boot = other reset facts, or an uptime smaller than in the last HELLO. Unexpected = a watchdog reset,
a crash, a lockup, a power dip (brownout), over-temperature - or a cold power-on while this bridge was already talking
to it and had not released the port for flashing.
"While already talking to it" (#110): the robot had been linked steadily (STEADY_S) and the new boot came within
OFF_GAP_S of the last moment it was alive (power lost while running). Not: the whole robot powering up (the bridge's own
first STARTUP_S; the Pi's USB can power the Teensy up, down and up again while it boots), a boot after the robot was
off for a while (switched off and on), or a boot right after a short-lived link.
Pure: time is passed in.
"""
from . import link_protocol as lp

STARTUP_S = 60.0       # a power-on in the bridge's first minute: the robot is powering up
STEADY_S = 10.0        # the previous boot was alive on the link at least this long ...
OFF_GAP_S = 5.0        # ... until at most this long before the new one started: power lost while it was running
SAME_BOOT_SLACK_MS = 3000.0   # a HELLO's uptime this far behind the Pi clock since that boot began: a new boot


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
    def __init__(self, start=None):
        self.start_t = start   # Pi time the bridge started (None: the first HELLO's time)
        self.key = None        # the current boot's reset facts (cause, flags, srsr, previous uptime, off time)
        self.uptime_ms = 0     # uptime in its last HELLO: smaller in the next one = it rebooted
        self.boot_t = None     # Pi time the Teensy's current boot started
        self.boot = None       # {"cause", "name", "flags", "srsr", "prev_uptime_s", "off_s"} of the current boot
        self.last = None       # the last unexpected reset this bridge run: {"cause", "name", "text", "t"}
        self.unexpected = 0    # how many this bridge run
        self.released = False  # the serial port was released (flashing) since the last boot
        self.seen_t = None     # Pi time this bridge first heard the current boot
        self.alive_t = None    # ... and last saw it alive (HELLO / telemetry, note_alive)

    def note_released(self):
        self.released = True

    def note_alive(self, now):
        """The robot is on the link and its telemetry flows (the bridge calls this from its tick)."""
        if self.seen_t is not None:
            self.alive_t = now

    def _while_linked(self, now, up):
        """Was the previous boot running on the link until just before this one started (#110)?"""
        if self.start_t is not None and now - self.start_t < STARTUP_S:
            return False
        if self.seen_t is None or self.alive_t is None:
            return False
        if self.alive_t - self.seen_t < STEADY_S:
            return False
        return (now - up / 1000.0) - self.alive_t <= OFF_GAP_S

    def on_hello(self, d, now):
        """A HELLO. -> ("warn" | "info", text) once per new boot; None for the same boot or firmware without the
        reset fields (link v8)."""
        if d.get("link_version", 0) < 9:
            return None
        if self.start_t is None:
            self.start_t = now
        up = d.get("uptime_ms", 0)
        key = tuple(d.get(k, 0) for k in ("reset_cause", "reset_flags", "srsr", "prev_uptime_s", "off_s"))
        first = self.key is None
        # the same boot: the same reset facts and the uptime still counting up - with the clock (#110: a cold boot has
        # the same facts as the one before; its uptime can still be above that boot's LAST HELLO, minutes ago)
        if not first and key == self.key and up >= self.uptime_ms and \
                up >= (now - self.boot_t) * 1000.0 - SAME_BOOT_SLACK_MS:
            self.uptime_ms = up
            self.alive_t = now
            return None
        linked = not first and self._while_linked(now, up)
        off_for = None if self.alive_t is None else max(0.0, (now - up / 1000.0) - self.alive_t)
        self.key, self.uptime_ms, self.boot_t = key, up, now - up / 1000.0
        self.seen_t = self.alive_t = now
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
                      (cause == lp.LINK_RESET_POWER_ON and linked and not released))
        if not unexpected:
            if cause == lp.LINK_RESET_POWER_ON and not first and off_for is not None and not released:
                extra.insert(0, "the robot was off ~%s or was just powering up" % _dur(off_for))
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
