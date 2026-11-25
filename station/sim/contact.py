"""Line-by-line port of src/contact.h / contact.cpp (issue #104): contact detection for obstacles below the lidar's
scan plane. BUMP: the speed falls far below a steady command with a pitch-rate spike and stays down; STUCK: it got
going, then no new ground for 0.4 s while the command and the velocity loop's lean push on; STALL (#111, reported as
STUCK): slow / reverse pushes from 30 mm/s (backing off, stepping away; driving: from 60 mm/s, a 0.8 s window below
100), the wheels (nearly) still for 0.4 s while the lean builds toward the command (or the controller holds it at its
anti-windup limit). The sim's Navigator feeds it every control step."""
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
# stall (#111), see src/contact.h
STALL_MIN_CMD = 30.0
STALL_START_MS = 300
STALL_MS = 400
STALL_NEW_MM, STALL_BACK_MM = 10.0, 15.0
STALL_STILL_MMS = 15.0
STALL_STILL_TAU_MS = 200.0
STALL_BUILD_DEG = 0.08
STALL_DRIVE_MS = 800
STALL_DRIVE_MIN_CMD = 60.0
STALL_DEAD_MMS = 4.0          # dead still (#111): mean |speed| at most ...
STALL_DEAD_MM = 4.0           # ... travel within +- this ...
STALL_DEAD_MS = 800           # ... this long ...
STALL_DEAD_MEAN_MMS = 6.0     # ... (a twitch is fine: the mean |speed| over the window at most this) ...
STALL_DEAD_BUILD_DEG = 0.1    # ... while the lean builds this much
STALL_REV_SETTLE_MS = 1000    # reversing: rear rules only this long after the command came ...
STALL_REV_RATE_DPS = 10.0     # ... the mean |pitch rate| at most this (the rocking died down) ...
STALL_REV_MS = 800            # ... the stall window ...
STALL_REV_BUILD_DEG = 0.15    # ... and the lean build-up backwards
STALL_REV_MEAN = 1.0          # ... the window's mean |speed| at most this x STALL_STILL_MMS (no rocking)


class ContactDetector:
    """update(c) with c: dict now_ms, watch, upright, cmd, speed, pitch_rate, vel_out, lidar_stuck, cruise, ramp, drive,
    held -> 0 or lp.LINK_CONTACT_BUMP / LINK_CONTACT_STUCK; then dir / value / flags / stall describe it."""

    def __init__(self):
        self.bins = []                  # (speed, command at the bin's end, pitch-rate max, min), newest last
        self.bin_ms = 0
        self.cur_max = self.cur_min = 0.0
        self.have = False
        self.last_ms = 0
        self.was_upright = False
        self.dir, self.value, self.flags = 1.0, 0.0, 0
        self.stall = False
        self.s_d = 1.0
        self.s_on_ms = self.s_a_ms = 0
        self.s_pos = self.s_a_pos = self.s_a_lean = self.s_abs = self.s_sum = 0.0
        self.d_a_pos = self.d_a_lean = self.d_sum = self.s_rate = 0.0
        self.d_a_ms = 0
        self.cand_d = self.run_d = 1.0
        self.pos = self.high = 0.0
        self.new_ms = self.run_ms = 0
        self.reset()

    def reset(self):
        self.bump_ms = self.cand_ms = 0
        self.bump_peak = 0.0
        self.hold_ms = 0
        self.run = self.moved = False
        self.s_run = self.s_win = self.d_win = False

    def _fire(self, kind, d, value, flags, stall=False):
        self.dir, self.value, self.flags, self.stall = d, value, flags, stall
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
        s_still = now - self.s_a_ms if self.s_win else 0    # stall: ms in the band

        # a fall right after a candidate confirms it (a hard bump throws it over before the hold time is up)
        if fell:
            if self.cand_ms and now - self.cand_ms <= FALL_MS:
                return self._fire(lp.LINK_CONTACT_BUMP, self.cand_d, self.bump_peak, lp.LINK_CONTACT_FELL)
            if still >= STUCK_MS // 2:
                return self._fire(lp.LINK_CONTACT_STUCK, self.run_d, float(still), lp.LINK_CONTACT_FELL)
            if s_still >= STALL_MS // 2:
                return self._fire(lp.LINK_CONTACT_STUCK, self.s_d, float(s_still), lp.LINK_CONTACT_FELL, True)
        if self.cand_ms and now - self.cand_ms > FALL_MS:
            self.cand_ms = 0
        if not c["watch"] or not c["upright"]:
            self.bump_ms = self.hold_ms = 0
            self.moved = self.run = False
            self.s_run = self.s_win = self.d_win = False
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

        # stall (#111): commanded (>= STALL_MIN_CMD, either way) but the wheels (nearly) still while the lean builds
        if cm >= STALL_MIN_CMD and (not self.s_run or d == self.s_d):
            if not self.s_run:
                self.s_run, self.s_d, self.s_on_ms = True, d, now
                self.s_pos, self.s_win, self.d_win = 0.0, False, False
            self.s_pos += v * dt * 0.001
            self.s_abs += (abs(v) - self.s_abs) * min(1.0, dt / STALL_STILL_TAU_MS)       # mean |speed|, last ~0.2 s
            self.s_rate += (abs(c["pitch_rate"]) - self.s_rate) * min(1.0, dt / STALL_STILL_TAU_MS)  # mean |pitch rate|
            # reversing (backing off, after a bounce or from standing): a balancing robot rocks for 0.5-1 s before the
            # motion shows - the rear rules count only after STALL_REV_SETTLE_MS, with the rocking died down, and a
            # longer window / build-up (2026-11-25 18:36: ~20 false rear contacts in back-offs)
            rev = d < 0
            rev_ok = not rev or (now - self.s_on_ms >= STALL_REV_SETTLE_MS and self.s_rate <= STALL_REV_RATE_DPS)
            # dead still (#111, any command: slowing for the last point, 40 mm/s, ramping up too): the wheels not turning
            # at all for STALL_DEAD_MS while the lean builds (a free robot in its dead band creeps or rocks)
            if now - self.s_on_ms >= STALL_START_MS:
                self.d_sum += abs(v) * dt * 0.001
                if (not self.d_win or abs(self.s_pos - self.d_a_pos) > STALL_DEAD_MM
                        or self.d_sum * 1000.0 > STALL_DEAD_MEAN_MMS * max(STALL_DEAD_MS, now - self.d_a_ms)):
                    # (starts once it is still; a twitch on the way is fine, travel and the mean speed decide)
                    self.d_win = self.s_abs <= STALL_DEAD_MMS
                    self.d_a_pos, self.d_a_ms, self.d_a_lean, self.d_sum = self.s_pos, now, c["vel_out"], 0.0
                elif (rev_ok and now - self.d_a_ms >= STALL_DEAD_MS
                      and (d * (c["vel_out"] - self.d_a_lean) >= (STALL_REV_BUILD_DEG if rev else STALL_DEAD_BUILD_DEG)
                           or c.get("held", False))):
                    return self._fire(lp.LINK_CONTACT_STUCK, d, float(now - self.d_a_ms),
                                      lp.LINK_CONTACT_LIDAR if c.get("lidar_stuck") else 0, True)
            drive = c.get("drive", False)
            # (driving this slowly a balancing robot can stand in its motor dead band: dead still only)
            if (c.get("ramp", False) or now - self.s_on_ms < STALL_START_MS
                    or (drive and (cm < STALL_DRIVE_MIN_CMD or not c.get("cruise", True)))):
                self.s_win = False            # leaning in / ramping up: a balancing robot lags there
            else:
                if not self.s_win or self.s_pos > self.s_a_pos + STALL_NEW_MM or self.s_pos < self.s_a_pos - STALL_BACK_MM:
                    self.s_win = True         # (it moved: the window starts again here)
                    self.s_a_pos, self.s_a_ms, self.s_a_lean, self.s_sum = self.s_pos, now, c["vel_out"], 0.0
                self.s_sum += abs(v) * dt * 0.001                                         # |travel| in the window
                s = now - self.s_a_ms
                # (driving slowly: a balancing robot can stand in its motor dead band a while)
                win = STALL_REV_MS if rev else (STALL_DRIVE_MS if drive and cm < STUCK_MIN_CMD else STALL_MS)
                # (still now; rocking against it before is fine, up to twice that on average)
                if (rev_ok and s >= win and self.s_abs <= STALL_STILL_MMS
                        and self.s_sum * 1000.0 <= (STALL_REV_MEAN if rev else 2.0) * STALL_STILL_MMS * s
                        and (d * (c["vel_out"] - self.s_a_lean) >= (STALL_REV_BUILD_DEG if rev else STALL_BUILD_DEG)
                             or c.get("held", False))):
                    return self._fire(lp.LINK_CONTACT_STUCK, d, float(s),
                                      lp.LINK_CONTACT_LIDAR if c.get("lidar_stuck") else 0, True)
        else:
            self.s_run = self.s_win = self.d_win = False
        return 0
