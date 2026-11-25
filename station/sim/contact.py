"""Line-by-line port of src/contact.h / contact.cpp (issue #104): contact detection for obstacles below the lidar's
scan plane. BUMP: the speed falls far below a steady command with a pitch-rate spike and stays down; STUCK: it got
going, then no new ground for 0.4 s while the command and the velocity loop's lean push on. The sim's Navigator feeds
it every control step."""
from station.bridge import link_protocol as lp

BIN_MS, BINS = 10, 12
BUMP_MIN_CMD = 100.0
BUMP_STEADY = 0.8
BUMP_NEAR = 0.6
BUMP_DROP_MM, BUMP_DROP_FRAC = 70.0, 0.4
BUMP_RATE = 80.0
BUMP_HOLD, BUMP_BACK = 0.5, 0.7
BUMP_HOLD_MS, BUMP_WINDOW_MS = 80, 300
STUCK_MIN_CMD = 100.0
STUCK_MOVED_FRAC, STUCK_MOVED_MM = 0.5, 50.0
STUCK_NEW_MM = 10.0
STUCK_MS, STUCK_START_MS = 400, 500
STUCK_LEAN_DEG = 0.6
FALL_MS = 600


class ContactDetector:
    """update(c) with c: dict now_ms, watch, upright, cmd, speed, pitch_rate, vel_out, lidar_stuck, cruise -> 0 or
    lp.LINK_CONTACT_BUMP / LINK_CONTACT_STUCK; then dir / value / flags describe it."""

    def __init__(self):
        self.bins = []                  # (speed, command at the bin's end, pitch-rate max, min), newest last
        self.bin_ms = 0
        self.cur_max = self.cur_min = 0.0
        self.have = False
        self.last_ms = 0
        self.was_upright = False
        self.dir, self.value, self.flags = 1.0, 0.0, 0
        self.cand_d = self.run_d = 1.0
        self.pos = self.high = 0.0
        self.new_ms = self.run_ms = 0
        self.reset()

    def reset(self):
        self.bump_ms = self.cand_ms = 0
        self.bump_peak = 0.0
        self.hold_ms = 0
        self.run = self.moved = False

    def _fire(self, kind, d, value, flags):
        self.dir, self.value, self.flags = d, value, flags
        self.reset()
        return kind

    def update(self, c):
        now = c["now_ms"]
        dt = min(50, now - self.last_ms) if self.have else 0
        self.last_ms = now
        pr = c["pitch_rate"]
        # signal history (always, also while not watching: a bump needs the speed before it)
        if not self.have:
            self.have = True
            self.bin_ms = now
            self.cur_max = self.cur_min = pr
        self.cur_max = max(self.cur_max, pr)
        self.cur_min = min(self.cur_min, pr)
        if now - self.bin_ms >= BIN_MS:
            self.bins.append((c["speed"], c["cmd"], self.cur_max, self.cur_min))
            if len(self.bins) > BINS:
                del self.bins[0]
            self.bin_ms = now
            self.cur_max = self.cur_min = pr
        fell = self.was_upright and not c["upright"]
        self.was_upright = c["upright"]
        d = 1.0 if c["cmd"] >= 0 else -1.0
        cm = abs(c["cmd"])
        v = d * c["speed"]
        still = now - self.new_ms if self.moved else 0      # ms without new ground

        # a fall right after a candidate confirms it (a hard bump throws it over before the hold time is up)
        if fell:
            if self.cand_ms and now - self.cand_ms <= FALL_MS:
                return self._fire(lp.LINK_CONTACT_BUMP, self.cand_d, self.bump_peak, lp.LINK_CONTACT_FELL)
            if still >= STUCK_MS // 2:
                return self._fire(lp.LINK_CONTACT_STUCK, self.run_d, float(still), lp.LINK_CONTACT_FELL)
        if self.cand_ms and now - self.cand_ms > FALL_MS:
            self.cand_ms = 0
        if not c["watch"] or not c["upright"]:
            self.bump_ms = self.hold_ms = 0
            self.moved = self.run = False
            return 0

        # bump
        if not self.bump_ms and cm >= BUMP_MIN_CMD:
            v_hi, c_hi, r_hi = v, cm, (self.cur_max if d > 0 else -self.cur_min)
            for s, cc, mx, mn in self.bins:
                v_hi = max(v_hi, d * s)
                c_hi = max(c_hi, d * cc)
                r_hi = max(r_hi, mx if d > 0 else -mn)
            drop = max(BUMP_DROP_MM, BUMP_DROP_FRAC * v_hi)
            if (cm >= BUMP_STEADY * c_hi and v_hi >= BUMP_NEAR * cm and v < BUMP_NEAR * cm and v_hi - v >= drop
                    and r_hi >= BUMP_RATE):
                self.bump_ms = self.cand_ms = now or 1
                self.bump_peak = r_hi
                self.cand_d = d
                self.hold_ms = 0
        if self.bump_ms:
            r = self.cur_max if d > 0 else -self.cur_min
            if r > self.bump_peak:
                self.bump_peak = r
            if v >= BUMP_BACK * cm or cm < 0.5 * BUMP_MIN_CMD:
                self.bump_ms = self.cand_ms = 0   # rolled over it (a door strip), or the command went
            elif now - self.bump_ms > BUMP_WINDOW_MS:
                self.bump_ms = 0                  # not confirmed (a fall soon after still counts: cand_ms)
            else:
                if v < BUMP_HOLD * cm:
                    self.hold_ms += dt
                if self.hold_ms >= BUMP_HOLD_MS:
                    return self._fire(lp.LINK_CONTACT_BUMP, d, self.bump_peak, 0)

        # pushing but not moving: no new ground for STUCK_MS since it got going
        if c.get("cruise", True) and cm >= STUCK_MIN_CMD and (not self.run or d == self.run_d):
            if not self.run:
                self.run, self.run_d, self.moved = True, d, False
                self.run_ms = now
                self.pos = self.high = 0.0
            self.pos += v * dt * 0.001        # along the commanded direction, mm (encoder speed integrated)
            # it got going toward it (not in the start: a balancing robot leans in and rolls back first)
            if not self.moved and now - self.run_ms >= STUCK_START_MS and v >= min(STUCK_MOVED_FRAC * cm, STUCK_MOVED_MM):
                self.moved = True
                self.high, self.new_ms = self.pos, now
            elif self.moved and self.pos > self.high + STUCK_NEW_MM:
                self.high, self.new_ms = self.pos, now   # new ground
        else:
            self.run = self.moved = False
        if self.moved and d * c["vel_out"] >= STUCK_LEAN_DEG:
            s = now - self.new_ms
            if s >= STUCK_MS:
                return self._fire(lp.LINK_CONTACT_STUCK, d, float(s), 0)
            if c["lidar_stuck"] and s >= STUCK_MS // 2:
                return self._fire(lp.LINK_CONTACT_STUCK, d, float(s), lp.LINK_CONTACT_LIDAR)
        return 0
