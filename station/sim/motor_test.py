"""Sim of the wheels-up motor test (mot.test, issue #16): a port of src/motor_test.cpp (same interlocks,
timing and "mt,..." log lines) driving simulated TT gear motors on a stand.

TT motors here: dead zone ~6.0 V to start / 5.6 V to keep turning (static vs sliding friction), above it
speed ~ k * (V - 5.6 V), left wheel 18% weaker, ~50 ms time constant, a little encoder noise.
"""
import math

MM_PER_COUNT = math.pi * 66.0 / 4096.0
# start / safety windows (src/motor_test.cpp)
START_HOLD_S, WAIT_TIMEOUT_S, BLOCK_LOG_S, ENC_STALE_S = 1.0, 60.0, 5.0, 0.05
TILT_DEG, STILL_DPS, ABORT_DPS, MOVE_MMS, STOP_MMS = 20.0, 10.0, 150.0, 20.0, 8.0
A_STEP, A_TICK_S = 10.0, 0.05
B_LEVELS, B_HOLD_S, B_MEAS_S, B_HALF_WIN_MS = 16, 1.2, 0.5, 40.0
C_REPS, C_SPIN_S, C_TIMEOUT_S, C_REV_COUNTS, C_MARGIN, C_DEFAULT = 6, 1.0, 2.0, 23, 300.0, 2000.0
A_LIMIT_S, B_LIMIT_S, C_LIMIT_S = 150.0, 70.0, 60.0
WHEEL = ("left", "right")


def dir_name(d):
    return "fwd" if d > 0 else "rev"


class TTWheel:
    """One gear motor + wheel in the air: volts in, encoder counts out."""

    def __init__(self, strength, v_start=6.0, v_stop=5.6, k=270.0, tau=0.05):
        self.strength, self.v_start, self.v_stop, self.k, self.tau = strength, v_start, v_stop, k, tau
        self.v = 0.0            # true wheel speed mm/s
        self.pos = 0.0          # mm
        self.moving = False

    def step(self, volts, dt):
        a = abs(volts)
        if self.moving and a < self.v_stop and abs(self.v) < 30.0:
            self.moving = False
        if not self.moving and a >= self.v_start:
            self.moving = True
        target = math.copysign(self.strength * self.k * max(0.0, a - self.v_stop), volts) if self.moving else 0.0
        tau = self.tau if self.moving else 0.03
        self.v += (target - self.v) * min(1.0, dt / tau)
        if abs(self.v) < 1.0 and not self.moving:
            self.v = 0.0
        self.pos += self.v * dt

    def counts(self):
        return int(math.floor(self.pos / MM_PER_COUNT))


class SimMotorTest:
    """src/motor_test.cpp in Python. step(inp) once per balance step; lines() / duty() / done()."""

    def __init__(self, emit):
        self.emit = emit
        self.state = "idle"
        self.test = 0
        self.duty = [0.0, 0.0]
        self.done = False
        self.part = None
        self.last_enc_us = None

    # inp: dict now, request, armed, estop, idle, radio_live, imu_ok, enc_ok, pitch, roll, pitch_rate,
    #      enc (l, r), enc_t, speed (l, r), vbatt, max_duty, pwm_hz, drive_mode
    def blocker(self, i):
        if i["estop"]:
            return "estop"
        if not i["armed"]:
            return "arm_the_remote_in_IDLE"
        if not i["idle"]:
            return "mode_not_IDLE"
        if not i["radio_live"]:
            return "no_remote"
        if not i["imu_ok"]:
            return "imu"
        if not i["enc_ok"]:
            return "encoders"
        if abs(i["pitch"]) > TILT_DEG or abs(i["roll"]) > TILT_DEG:
            return "not_upright"
        if abs(i["pitch_rate"]) > STILL_DPS:
            return "not_still"
        if abs(i["speed"][0]) > STOP_MMS or abs(i["speed"][1]) > STOP_MMS:
            return "wheels_turning"
        return None

    def abort_reason(self, i):
        if i["request"] != self.test:
            return "cancelled"
        if i["estop"]:
            return "estop"
        if not i["armed"]:
            return "disarmed"
        if not i["idle"]:
            return "mode_change"
        if not i["radio_live"]:
            return "remote_lost"
        if not i["imu_ok"]:
            return "imu"
        if not i["enc_ok"]:
            return "encoder"
        if i["now"] - i["enc_t"] > ENC_STALE_S:
            return "encoder_stale"
        if abs(i["pitch"]) > TILT_DEG or abs(i["roll"]) > TILT_DEG:
            return "tilt"
        if abs(i["pitch_rate"]) > ABORT_DPS:
            return "pitch_rate"
        if i["now"] >= self.deadline:
            return "timeout"
        return None

    def stop_all(self, done):
        self.duty = [0.0, 0.0]
        self.state = "idle"
        self.holding = False
        self.test = 0
        self.done = done

    def hold(self, s, i):
        self.duty = [0.0, 0.0]
        self.holding = True
        self.hold_until = i["now"] + s

    def volts(self, duty, i):
        return duty / 4095.0 * i["vbatt"]

    def step(self, i):
        new_enc = self.last_enc_us is None or i["enc_t"] != self.last_enc_us
        self.last_enc_us = i["enc_t"]
        now = i["now"]
        if self.state == "idle":
            if i["request"]:
                self.state, self.test, self.t_req, self.ok_t, self.last_block = "wait", i["request"], now, None, None
                self.emit("mt,request,%u" % self.test)
            return
        if self.state == "wait":
            if i["request"] == 0:
                self.emit("mt,cancel,%u" % self.test)
                self.stop_all(False)
                return
            if i["request"] != self.test:
                self.test = i["request"]
                self.emit("mt,request,%u" % self.test)
            b = self.blocker(i)
            if b:
                self.ok_t = None
                if b != self.last_block or now - self.last_block_t >= BLOCK_LOG_S:
                    self.last_block, self.last_block_t = b, now
                    self.emit("mt,wait,%s" % b)
                if now - self.t_req >= WAIT_TIMEOUT_S:
                    self.emit("mt,abort,not_started_in_60s,%s" % b)
                    self.stop_all(True)
                return
            if self.ok_t is None:
                self.ok_t = now
            if now - self.ok_t >= START_HOLD_S:
                self.start(i)
            return
        r = self.abort_reason(i)
        if r:
            self.emit("mt,abort,%s,part=%s,t=%.1f,vb=%.2f" % (r, self.part or "-", now - self.t_start, i["vbatt"]))
            self.stop_all(True)
            return
        self.vb_min, self.vb_max = min(self.vb_min, i["vbatt"]), max(self.vb_max, i["vbatt"])
        if self.holding:
            if now < self.hold_until:
                return
            self.holding = False
        if self.part == "a":
            self.run_a(i, new_enc)
        elif self.part == "b":
            self.run_b(i, new_enc)
        elif self.part == "c":
            self.run_c(i, new_enc)
        else:
            self.next_part(i)

    def start(self, i):
        if self.test > 4:      # tracking tests 5-7 (issue #26): firmware only
            self.emit("mt,abort,not_in_sim,part=-,t=0.0")
            self.stop_all(True)
            return
        self.parts = {1: "a", 2: "b", 3: "c"}.get(self.test, "abc")
        limit = {1: A_LIMIT_S, 2: B_LIMIT_S, 3: C_LIMIT_S}.get(self.test, A_LIMIT_S + B_LIMIT_S + C_LIMIT_S)
        self.state, self.part_i, self.part, self.holding = "run", 0, None, False
        self.t_start, self.deadline = i["now"], i["now"] + limit
        self.vb_min = self.vb_max = i["vbatt"]
        self.max_duty = min(4095.0, max(100.0, i["max_duty"]))
        self.hz, self.drive = i["pwm_hz"], i["drive_mode"]
        self.a_valid = False
        self.emit("mt,begin,%u,max=%.0f,vb=%.2f,pitch=%.1f,hz=%.0f,drive=%u" % (
            self.test, self.max_duty, i["vbatt"], i["pitch"], self.hz, self.drive))
        self.next_part(i)

    def cfg(self):
        return "hz=%.0f,drive=%u" % (self.hz, self.drive)

    def next_part(self, i):
        p = self.parts[self.part_i] if self.part_i < len(self.parts) else None
        self.part_i += 1
        if not p:
            self.emit("mt,end,%u,ok,%.1fs,vb=%.2f..%.2f,%s" % (self.test, i["now"] - self.t_start, self.vb_min,
                                                              self.vb_max, self.cfg()))
            self.stop_all(True)
            return
        self.part, self.dir, self.need_begin = p, 1, True
        if p == "a":
            self.a_start = {}
            self.a_stop = {}
        elif p == "b":
            self.b_level, self.b_prev = 1, [0.0, 0.0]
            self.b_top, self.b_move_v = {}, {}
        else:
            hi = max(self.a_start.values()) if self.a_valid and self.a_start else -1
            self.c_duty = min(self.max_duty, hi + C_MARGIN if hi > 0 else C_DEFAULT)
            self.c_rep, self.c_spin = 0, True
            self.c_sum = {w: [0.0, 0.0, 0.0, 0] for w in (0, 1)}
        getattr(self, "run_" + p)(i, False)

    def end_part(self, i):
        self.duty = [0.0, 0.0]
        self.part = None
        self.holding = True
        self.hold_until = i["now"] + 1.0

    # ---------------- a: thresholds
    def run_a(self, i, new_enc):
        if self.need_begin:
            self.need_begin = False
            self.a_st, self.a_duty, self.a_tick = [0, 0], [0.0, 0.0], i["now"]
        if i["now"] - self.a_tick >= A_TICK_S - 1e-9:
            self.a_tick = i["now"]
            for w in (0, 1):
                if self.a_st[w] == 0:
                    self.a_duty[w] = min(self.a_duty[w] + A_STEP, self.max_duty)
                elif self.a_st[w] == 1:
                    self.a_duty[w] = max(self.a_duty[w] - A_STEP, 0.0)
        d = dir_name(self.dir)
        if new_enc:
            for w in (0, 1):
                sp = i["speed"][w]
                if self.a_st[w] == 0:
                    if abs(sp) > MOVE_MMS:
                        self.a_start[(w, d)] = self.a_duty[w]
                        self.emit("mt,a,%s,%s,start,%.0f,%.3f,%.2f,%.0f" % (WHEEL[w], d, self.a_duty[w],
                                                                          self.volts(self.a_duty[w], i), i["vbatt"], sp))
                        self.a_st[w] = 1
                    elif self.a_duty[w] >= self.max_duty:
                        self.emit("mt,a,%s,%s,start,-1,-1,%.2f,%.0f" % (WHEEL[w], d, i["vbatt"], sp))
                        self.emit("mt,a,%s,%s,stop,-1,-1,%.2f,%.0f" % (WHEEL[w], d, i["vbatt"], sp))
                        self.a_st[w] = 2
                elif self.a_st[w] == 1:
                    if abs(sp) < STOP_MMS or self.a_duty[w] <= 0:
                        self.emit("mt,a,%s,%s,stop,%.0f,%.3f,%.2f,%.0f" % (WHEEL[w], d, self.a_duty[w],
                                                                         self.volts(self.a_duty[w], i), i["vbatt"], sp))
                        self.a_st[w] = 2
        self.duty = [self.dir * self.a_duty[w] if self.a_st[w] < 2 else 0.0 for w in (0, 1)]
        if self.a_st == [2, 2]:
            if self.dir > 0:
                self.dir, self.need_begin = -1, True
                self.hold(1.0, i)
            else:
                self.a_valid = True
                self.emit("mt,sum,a,vb=%.2f,%s" % (i["vbatt"], self.cfg()))
                self.end_part(i)

    # ---------------- b: speed curve
    def run_b(self, i, new_enc):
        if self.need_begin:
            self.need_begin = False
            self.t_phase = i["now"]
            self.b_have0, self.b_trace = [False, False], [[], []]
            self.b_c0, self.b_t0 = [0, 0], [0.0, 0.0]
        duty = self.max_duty * self.b_level / B_LEVELS
        self.duty = [self.dir * duty, self.dir * duty]
        el = i["now"] - self.t_phase
        if not new_enc:
            return
        for w in (0, 1):
            self.b_trace[w].append((el * 1000.0, i["speed"][w]))
            if not self.b_have0[w] and el >= B_HOLD_S - B_MEAS_S:
                self.b_have0[w] = True
                self.b_c0[w], self.b_t0[w] = i["enc"][w], i["enc_t"]
        if el < B_HOLD_S:
            return
        d = dir_name(self.dir)
        for w in (0, 1):
            dt = i["enc_t"] - self.b_t0[w]
            v = (i["enc"][w] - self.b_c0[w]) * MM_PER_COUNT / dt if self.b_have0[w] and dt > 0 else 0.0
            t63, dv = -1.0, v - self.b_prev[w]
            if abs(dv) > 30.0:
                target = self.b_prev[w] + 0.632 * dv
                for ms, sv in self.b_trace[w]:
                    if (sv - target) * (1 if dv > 0 else -1) >= 0:
                        t63 = max(0.0, ms - B_HALF_WIN_MS)
                        break
            self.emit("mt,b,%s,%s,%.0f,%.3f,%.2f,%.1f,%.0f" % (WHEEL[w], d, duty, self.volts(duty, i), i["vbatt"], v, t63))
            self.b_prev[w] = v
        self.need_begin = True
        self.b_level += 1
        if self.b_level > B_LEVELS:
            if self.dir > 0:
                self.dir, self.b_level, self.b_prev = -1, 1, [0.0, 0.0]
                self.hold(1.5, i)
            else:
                self.emit("mt,sum,b,vb=%.2f,%s" % (i["vbatt"], self.cfg()))
                self.end_part(i)

    # ---------------- c: reversal
    def run_c(self, i, new_enc):
        enc = i["enc"]
        if self.need_begin:
            self.need_begin, self.t_phase, self.c_spin = False, i["now"], True
        self.duty = [self.dir * self.c_duty] * 2
        if self.c_spin:
            if i["now"] - self.t_phase >= C_SPIN_S and new_enc:
                self.dir = -self.dir
                self.duty = [self.dir * self.c_duty] * 2
                self.c_spin, self.c_t0 = False, i["now"]
                self.c_c0, self.c_ext, self.c_ext_t, self.c_det = list(enc), list(enc), [i["enc_t"]] * 2, [False, False]
            return
        old = -self.dir
        d = dir_name(self.dir)
        if new_enc:
            for w in (0, 1):
                if self.c_det[w]:
                    continue
                if old * (enc[w] - self.c_ext[w]) > 0:
                    self.c_ext[w], self.c_ext_t[w] = enc[w], i["enc_t"]
                if old * (self.c_ext[w] - enc[w]) >= C_REV_COUNTS:
                    self.c_det[w] = True
                    ts = max(0.0, (self.c_ext_t[w] - self.c_t0) * 1000.0)
                    tr = (i["enc_t"] - self.c_t0) * 1000.0
                    coast = old * (self.c_ext[w] - self.c_c0[w]) * 360.0 / 4096.0
                    self.emit("mt,c,%s,%s,%u,%.0f,%.3f,%.2f,%.0f,%.0f,%.1f" % (
                        WHEEL[w], d, self.c_rep, self.c_duty, self.volts(self.c_duty, i), i["vbatt"], ts, tr, coast))
        if all(self.c_det) or i["now"] - self.c_t0 >= C_TIMEOUT_S:
            for w in (0, 1):
                if not self.c_det[w]:
                    self.emit("mt,c,%s,%s,%u,%.0f,%.3f,%.2f,-1,-1,-1" % (WHEEL[w], d, self.c_rep, self.c_duty,
                                                                       self.volts(self.c_duty, i), i["vbatt"]))
            self.c_rep += 1
            if self.c_rep >= C_REPS:
                self.emit("mt,sum,c,duty=%.0f,V=%.2f,%s" % (self.c_duty, self.volts(self.c_duty, i), self.cfg()))
                self.end_part(i)
            else:
                self.c_spin, self.t_phase = True, i["now"]
