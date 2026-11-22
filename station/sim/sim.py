"""Fake Teensy (firmware v4): speaks the real binary link protocol over TCP so the bridge and UI can be
developed on a PC.

- Param table, arming rules, cfg.save / cfg.defaults and telemetry follow the v4 firmware
  (src/params.h, src/mode_manager.cpp, src/main.cpp).
- The balance / heading / velocity loops are a line-by-line port of src/controller.cpp, running at
  ctl.rate_hz (400) on a wheeled inverted pendulum with TT-style gear motors (dead zone, lag), so the
  bal_p / bal_i / bal_d telemetry and the effect of a gain change look like the real robot.
- A virtual room with walls/boxes and the servo-swept twin VL53L0X range sensor.
- GOTO / EXPLORE / HOLD: Navigator is a line-by-line port of src/navigator.cpp (issue #8): same nav.* params,
  NavStatus states, obstacle stop from the simulated range sweep.
"""
import asyncio
import math
import random
import time

from station.bridge import link_protocol as lp
from station.sim.motor_test import MM_PER_COUNT, SimMotorTest, TTWheel

# Geometry and mass from the Fusion CAD (cad/robot_model.json on master)
TRACK_MM = 126.8          # wheel separation (firmware odometry param says 123)
ROBOT_RADIUS = 90.0       # collision radius: footprint ~152 x 90 mm
SENSOR_FWD_MM = 26.0      # range-sensor servo axis is this far forward of the axle
SENSOR_RADIUS_MM = 12.9   # each VL53L0X sits this far out from the servo axis, facing outwards
RANGE_MAX = 2000.0        # VL53L0X reach in long-range mode (1200 without)
RANGE_MAX_SHORT = 1200.0
RANGE_IO_S = 0.007        # I2C + start per reading (firmware: ~80 ms per sweep position at settle 40, budget 33)
RANGE_DROPOUT = 0.03      # chance of a reading with no valid return

# Physics. Pendulum from the CAD: body 0.77 kg, axle -> CoM 84.5 mm, pitch inertia 0.0073 kg m^2 about
# the axle -> theta'' = (m g l / I) sin(theta) - (m l / I) a cos(theta), fall time constant 0.107 s.
# Motors are made up but plausible (12 V pack, TT gear motors).
G = 9.81
BODY_KG, COM_M, I_AXLE = 0.77, 0.0845, 0.0073
PEND_G = BODY_KG * G * COM_M / I_AXLE     # 1/s^2  (~87, tau = 0.107 s)
PEND_A = BODY_KG * COM_M / I_AXLE         # 1/m    (wheel acceleration coupling)
IMU_MOUNT_DEG = -3.15     # IMU reads this when the robot is truly upright (= -bal.offset_deg default)
LIE_DEG = 55.0            # true lean when lying on the floor (beyond bal.cutoff_deg)
MOTOR_FULL = 4095.0
MOTOR_DEADZONE = 1350.0   # PWM counts below which the wheel does not turn
MOTOR_VMAX = 1100.0       # mm/s at full PWM and 12.4 V
MOTOR_TAU = 0.045         # s, wheel speed lag
LEFT_STRONGER = 1.0 / 0.95  # the left motor is a bit stronger (why mot.left_scale defaults to 0.95)
VEL_LOOP_DT = 0.01        # firmware: velocity loop at 100 Hz

# (name, default, min, max): copy of the v4 firmware table (src/params.h); ids = row order.
# The UI never hardcodes these; it builds everything from ParamInfoFrames.
PARAMS = [
    ("bal.kp", 250.0, 0.0, 2000.0), ("bal.ki", 2010.0, 0.0, 20000.0), ("bal.kd", 4.63, 0.0, 50.0),
    ("bal.offset_deg", 3.15, -15.0, 15.0), ("bal.cutoff_deg", 45.0, 5.0, 80.0),
    ("bal.reengage_deg", 15.0, 1.0, 45.0), ("bal.knob_trim_deg", 0.0, 0.0, 15.0), ("bal.d_src", 0.0, 0.0, 1.0),
    ("str.kp", 20.0, 0.0, 500.0), ("str.ki", 5.9, 0.0, 500.0), ("str.kd", 0.075, 0.0, 10.0),
    ("str.max_err_deg", 45.0, 1.0, 180.0),
    ("vel.kp", 0.006, 0.0, 1.0), ("vel.ki", 0.00869, 0.0, 1.0), ("vel.kd", 0.00007, 0.0, 0.1),
    ("vel.limit_deg", 20.0, 0.0, 30.0),
    ("drv.max_speed", 450.0, 0.0, 1500.0), ("drv.max_turn", 180.0, 0.0, 720.0), ("drv.accel", 1500.0, 50.0, 10000.0),
    ("drv.turn_accel", 1000.0, 50.0, 20000.0), ("drv.turn_deadband", 2.0, 0.0, 30.0),
    ("mot.left_scale", 0.95, 0.5, 1.5), ("mot.right_scale", 1.0, 0.5, 1.5), ("mot.max", 4095.0, 0.0, 4095.0),
    ("mot.curve_exp", 3.92, 1.0, 10.0), ("mot.knee_in", 1250.0, 1.0, 4000.0), ("mot.knee_out", 2750.0, 0.0, 4095.0),
    ("ctl.rate_hz", 400.0, 100.0, 1000.0), ("ctl.loops", 3.0, 0.0, 3.0), ("ctl.settle_ms", 500.0, 0.0, 5000.0),
    ("batt.low_cell", 3.4, 2.5, 4.2), ("batt.adc_per_v", 73.3182, 10.0, 200.0),
    # rng.enable: firmware default 0; the sim starts with the sweep on so the map has points
    ("tlm.hz", 100.0, 1.0, 1000.0), ("rng.enable", 1.0, 0.0, 1.0),
    ("cfg.save", 0.0, 0.0, 1.0), ("cfg.defaults", 0.0, 0.0, 1.0),
    ("rng.servo_trim_deg", 0.0, -45.0, 45.0), ("rng.sweep_deg", 170.0, 0.0, 180.0), ("rng.steps", 16.0, 1.0, 64.0),
    ("rng.settle_ms", 40.0, 0.0, 500.0), ("rng.timing_budget_ms", 33.0, 20.0, 200.0),
    ("rng.long_range", 1.0, 0.0, 1.0), ("rng.max_mm", 2000.0, 50.0, 8190.0),
    ("rng.servo_hold_deg", -1.0, -1.0, 180.0), ("rng.obstacle_mm", 300.0, 0.0, 2000.0),
    ("rng.servo_dir", 1.0, -1.0, 1.0), ("rng.servo_min_deg", 5.0, 0.0, 90.0),
    ("rng.servo_max_deg", 175.0, 90.0, 180.0), ("rng.servo_deg_per_cmd", 1.0, 0.5, 1.5),
    # wheels-up motor test (sim: robot on a stand, TT motors; station/sim/motor_test.py). mot.model / vcomp /
    # v_ref / dz_* are listed (the Tune page applies motor fits to them); the balance sim keeps the v3 curve
    # unless SimRobot.use_floor_wheels() (python -m station.sim --stiction) runs the firmware motor layer.
    ("mot.test", 0.0, 0.0, 7.0), ("mot.test_max", 4095.0, 500.0, 4095.0), ("mot.model", 0.0, 0.0, 1.0),
    ("mot.vcomp", 0.0, 0.0, 1.0), ("mot.v_ref", 12.0, 6.0, 17.0), ("mot.dz_l_fwd", 2.0, 0.0, 8.0),
    ("mot.dz_l_rev", 2.0, 0.0, 8.0), ("mot.dz_r_fwd", 2.0, 0.0, 8.0), ("mot.dz_r_rev", 2.0, 0.0, 8.0),
    ("mot.dz_band", 0.01, 0.0, 0.2), ("rng.idle_off", 1.0, 0.0, 1.0), ("mot.pwm_hz", 36621.09, 100.0, 50000.0),
    ("mot.drive_mode", 0.0, 0.0, 1.0),
    # motor profile (issue #26): one-wheel tests, brake-mode limits, wheel speed loop (not simulated)
    ("mot.test_wheels", 0.0, 0.0, 2.0), ("mot.brake_max_hz", 2000.0, 100.0, 10000.0),
    ("mot.brake_duty_max", 3000.0, 500.0, 4095.0), ("mot.speed_loop", 0.0, 0.0, 1.0), ("mot.sl_kp", 0.006, 0.0, 0.1),
    ("mot.sl_ki", 0.08, 0.0, 2.0), ("mot.sl_imax", 3.0, 0.0, 12.0), ("mot.sl_band", 0.3, 0.0, 3.0),
    ("mot.k_l", 200.0, 10.0, 2000.0), ("mot.k_r", 200.0, 10.0, 2000.0),
    # navigation (issue #8): src/navigator.h, simulated by Navigator below
    ("nav.max_speed", 250.0, 0.0, 1500.0), ("nav.max_turn", 90.0, 0.0, 720.0),
    ("nav.speed_kp", 0.6, 0.0, 10.0), ("nav.min_speed", 40.0, 0.0, 300.0), ("nav.turn_kp", 2.0, 0.0, 20.0),
    ("nav.align_deg", 10.0, 1.0, 60.0), ("nav.arrive_mm", 50.0, 5.0, 500.0), ("nav.stop_speed", 50.0, 1.0, 300.0),
    ("nav.stop_turn_dps", 20.0, 1.0, 180.0), ("nav.dwell_ms", 500.0, 0.0, 10000.0),
    ("nav.end_heading_deg", -1.0, -1.0, 360.0), ("nav.heading_tol_deg", 3.0, 0.5, 45.0),
    ("nav.slow_mm", 800.0, 0.0, 3000.0), ("nav.clear_mm", 100.0, 0.0, 1000.0),
    ("nav.corridor_mm", 220.0, 50.0, 1000.0), ("nav.obst_age_ms", 1500.0, 100.0, 10000.0),
    ("nav.floor_mm", 50.0, 0.0, 200.0), ("nav.backoff_mm", 0.0, 0.0, 500.0),
    ("nav.hold_kp", 2.0, 0.0, 20.0), ("nav.hold_max_speed", 150.0, 0.0, 1000.0), ("nav.hold_db_mm", 15.0, 0.0, 200.0),
    # safety review #38: GOTO without range readings: speed cap (0 = paused); EXPLORE always pauses
    ("nav.blind_speed", 150.0, 0.0, 1500.0),
    ("mot.left_dir", 1.0, -1.0, 1.0),
    ("mot.right_dir", 1.0, -1.0, 1.0),
    # issue #62: keep off side walls, stuck detection + back off
    ("nav.side_min_mm", 120.0, 0.0, 500.0),
    ("nav.side_kp", 0.5, 0.0, 5.0),
    ("nav.stuck_ms", 2500.0, 0.0, 20000.0),
    ("nav.stuck_mm", 40.0, 5.0, 500.0),
    ("nav.stuck_backoff_mm", 100.0, 0.0, 500.0),
    # issue #69: stiction kick (mot.model 1); the balance sim uses it with SimRobot.use_floor_wheels()
    ("mot.bk_l_fwd", 2.46, 0.0, 8.0), ("mot.bk_l_rev", 2.11, 0.0, 8.0),
    ("mot.bk_r_fwd", 2.73, 0.0, 8.0), ("mot.bk_r_rev", 2.49, 0.0, 8.0),
    ("mot.bk_gain", 1.0, 0.0, 1.5), ("mot.bk_ms", 80.0, 0.0, 300.0),
    ("mot.bk_band", 0.03, 0.0, 0.3), ("mot.bk_mms", 15.0, 1.0, 200.0),
    # issue #14: odometry calibration (Station: Tune, Calibrate odometry)
    ("odo.wheel_d_mm", 66.0, 40.0, 100.0), ("odo.track_mm", 126.8, 80.0, 200.0), ("odo.lr_ratio", 1.0, 0.9, 1.1),
]
CELLS = 3
WAYPOINT_MAX = 32         # src/config.h
BLIND_MS = 3000           # Navigator::kBlindMs

NAV_IDLE, NAV_TURNING, NAV_DRIVING, NAV_BLOCKED, NAV_ARRIVED, NAV_HOLDING = (
    lp.NAV_IDLE, lp.NAV_TURNING, lp.NAV_DRIVING, lp.NAV_BLOCKED, lp.NAV_ARRIVED, lp.NAV_HOLDING)
NAV_MODES = (lp.MODE_GOTO, lp.MODE_EXPLORE, lp.MODE_HOLD)
SENSOR_HEIGHT_MM = 189.0  # range sensors above the axle
# Navigator footprint constants (src/navigator.cpp, #62)
HALF_WIDTH_MM = 76.0      # body side from the centre line
SWING_R_MM = 105.0        # turning in place, the corners sweep r 88 (+ margin)
SIDE_BACK_MM, SIDE_AHEAD_MM, SIDE_INNER_MM = 45.0, 300.0, 40.0   # side window: body rear .. 300 ahead
BLOCKED_TURN_DEG = 30.0   # GOTO blocked by a wall alongside: turn away at most this much
STUCK_DEG = 15.0


def _box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


class Room:
    """Walls as line segments (world mm). The robot starts at (0, 0) facing +X."""

    def __init__(self, empty=False):
        """empty: only the outer walls (sim --maze, #70)."""
        s = []
        s += _box(1500, 250, 5000, 3500)          # outer walls: x -1000..4000, y -1500..2000
        if not empty:
            s += _box(1500, 900, 600, 400)            # box
            s += _box(2900, -800, 450, 450)           # box
            s += _box(700, -1300, 1000, 400)          # couch against the wall
            s.append((2300, 2000, 2300, 1100))        # partition wall
            s.append((-1000, 1200, -300, 2000))       # angled corner cabinet
        self.base = s
        self.segs = list(s)

    def set_extra(self, segs):
        """Extra real walls on top of the room: the map's virtual walls and mazes (sim --maze, #70)."""
        self.segs = self.base + [tuple(float(v) for v in s[:4]) for s in segs]

    def raycast(self, ox, oy, ang_rad, max_d=RANGE_MAX * 1.5):
        dx, dy = math.cos(ang_rad), math.sin(ang_rad)
        best = None
        for x1, y1, x2, y2 in self.segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den     # along the ray
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den     # along the segment
            if t > 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        return best if best is not None and best <= max_d else None

    def clearance(self, x, y):
        best = 1e9
        for x1, y1, x2, y2 in self.segs:
            ex, ey = x2 - x1, y2 - y1
            L2 = ex * ex + ey * ey
            u = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / L2)) if L2 else 0.0
            px, py = x1 + u * ex - x, y1 + u * ey - y
            best = min(best, math.hypot(px, py))
        return best


def _wrap180(a):
    """(-180, 180], like the firmware's wrap180 (straight back = +180)."""
    a = math.fmod(a, 360.0)
    if a > 180.0:
        a -= 360.0
    if a <= -180.0:
        a += 360.0
    return a


def _clamp(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


def _slew(cur, target, step):
    if target > cur + step:
        return cur + step
    if target < cur - step:
        return cur - step
    return target


def motor_curve(cmd, exp, knee_in, knee_out):
    """src/motors.cpp: |command| -> PWM counts (dead-zone compensation)."""
    a = min(abs(cmd), MOTOR_FULL)
    if a <= 0:
        return 0.0
    if a <= knee_in:
        out = knee_out * (a / knee_in) ** (1.0 / exp)
    else:
        out = knee_out + (MOTOR_FULL - knee_out) / (MOTOR_FULL - knee_in) * (a - knee_in)
    return min(out, MOTOR_FULL)


def motor_dz_volts(u, dz, v_ref, band):
    """src/motor_model.h: |u| 0..1 -> volts (mot.model 1)."""
    u = min(abs(u), 1.0)
    if u <= 0.0:
        return 0.0
    dz = min(dz, v_ref)
    if u < band:
        return u / band * (dz + (v_ref - dz) * band)
    return dz + (v_ref - dz) * u


class StictionKick:
    """Port of StictionKick (src/motor_model.h, issue #69); times in us."""
    STOP_US, FADE_US, LOCK_US, SUSTAIN_US = 20000, 40000, 150000, 60000
    IDLE, KICK, FADE = 0, 1, 2

    def __init__(self):
        self.reset()

    def reset(self):
        self.state, self.dir, self.sus_dir, self.floor = self.IDLE, 0, 0, 0.0
        self.init = self.kicked = False
        self.t_state = self.t_moving = self.t_sus = self.t_kick = 0

    def apply(self, u, v, speed, fb_ok, now_us, bk_fwd, bk_rev, gain, kick_ms, band, move_mms):
        if gain <= 0.0 or not fb_ok:
            self.reset()
            return v
        if not self.init:
            self.init = True
            self.t_moving = self.t_sus = now_us
            self.sus_dir = 0
        moving = abs(speed) >= move_mms
        if moving:
            self.t_moving = now_us
        d = 1 if u > band else (-1 if u < -band else 0)
        if d != self.sus_dir:
            self.sus_dir, self.t_sus = d, now_us
        if self.state != self.IDLE and d != self.dir:
            self.state = self.IDLE
        if self.state == self.KICK and (moving or now_us - self.t_state >= kick_ms * 1000.0):
            self.state, self.t_state = self.FADE, now_us
        if self.state == self.FADE and now_us - self.t_state >= self.FADE_US:
            self.state = self.IDLE
        if (self.state == self.IDLE and d != 0 and now_us - self.t_moving >= self.STOP_US and
                (not self.kicked or now_us - self.t_kick >= self.LOCK_US or now_us - self.t_sus >= self.SUSTAIN_US)):
            self.state, self.dir, self.t_state, self.t_kick, self.kicked = self.KICK, d, now_us, now_us, True
        bk = gain * (bk_fwd if self.dir > 0 else bk_rev)
        if self.state == self.KICK:
            self.floor = bk
        elif self.state == self.FADE:
            self.floor = bk * (1.0 - (now_us - self.t_state) / self.FADE_US)
        else:
            self.floor = 0.0
        if self.floor <= abs(v):
            return v
        return self.floor if d > 0 else -self.floor


class FloorWheel:
    """Sim-only TT gear motor on the floor with static friction (issue #69): a stopped wheel needs its
    breakaway volts (v_start, per direction) to move, a turning one keeps going down to v_stop (kinetic).
    Defaults: the new motors' motor-test numbers (left wheel; right: 2.73 / 2.49, 0.85 / 0.88)."""
    STICK_MMS = 5.0

    def __init__(self, v_start=(2.46, 2.11), v_stop=(0.82, 0.76), k=124.0, tau=MOTOR_TAU):
        self.v_start, self.v_stop, self.k, self.tau = v_start, v_stop, k, tau
        self.v = 0.0
        self.pos = 0.0
        self.moving = False
        self.hist = []           # (t, encoder mm) of the last 20 ms: WheelSpeedEst

    def step(self, volts, dt):
        a, i = abs(volts), (0 if volts >= 0 else 1)
        if not self.moving and a >= self.v_start[i] and volts != 0.0:
            self.moving = True
        if self.moving:
            target = math.copysign(self.k * max(0.0, a - self.v_stop[i]), volts) if volts else 0.0
            nv = self.v + (target - self.v) * min(1.0, dt / self.tau)
            # slowing through ~0 (or reversing) without the breakaway volts in the drive direction: sticks
            if (abs(nv) < self.STICK_MMS or nv * self.v < 0.0) and (a < self.v_start[i] or volts == 0.0):
                self.moving, nv = False, 0.0
            self.v = nv
        else:
            self.v = 0.0
        self.pos += self.v * dt

    def coast(self, v, dt):
        self.v, self.moving = v, abs(v) >= self.STICK_MMS
        self.pos += v * dt

    def speed_est(self, now):
        """Encoder speed over the last 20 ms (counts quantised), like WheelSpeedEst at 200 Hz."""
        c = math.floor(self.pos / MM_PER_COUNT) * MM_PER_COUNT
        self.hist.append((now, c))
        while len(self.hist) > 2 and now - self.hist[0][0] > 0.0201:
            self.hist.pop(0)
        t0, c0 = self.hist[0]
        return (c - c0) / (now - t0) if now > t0 else 0.0


class Navigator:
    """Line-by-line port of src/navigator.cpp (GOTO / EXPLORE waypoints, HOLD, obstacle check).
    Same params, states, log lines. Inputs `i`: dict now_ms, mode, upright, live, x, y, heading, speed,
    yaw_rate, max_speed, max_turn (see NavIn)."""
    SEEK, STOPPING, END_TURN, DONE = range(4)
    PTS = 48

    def __init__(self, P, log):
        self.P, self._log = P, log
        self.wp = []
        self.idx = 0
        self.mode = lp.MODE_IDLE
        self.phase = self.SEEK
        self.driving = self.blocked = self.latched = self.backing = False
        self.back = (0.0, 0.0, 0)
        self.stop_since = 0
        self.hold = (0.0, 0.0)
        self.state = NAV_IDLE
        self.speed = self.turn = 0.0
        self.tx = self.ty = self.dist = 0.0
        self.front = self.rear = self.obst = 0.0
        self.gap_l = self.gap_r = 0.0     # side gaps (0 = nothing beside), #62
        self.near_mm = self.near_xr = 0.0
        self.back_mm = 0.0
        self.blk_h = 0.0                  # heading when it blocked
        self.stuck = self.stuck_turn = self.stuck_seen = False
        self.stuck_ms = 0
        self.stuck_pose = (0.0, 0.0, 0.0)
        self.away = self.away_done = False
        self.away_pose = (0.0, 0.0, 0)
        self.obst_flag = False
        self.blk_fwd = self.blk_rev = False
        self.close = {"fwd": (0, 0.0), "rev": (0, 0.0)}   # last time / range inside the clear distance
        self.now_ms = 0
        self.pts = []                 # (x, y, t_ms), newest last, at most PTS
        self.last_range_ms = 0
        self.sensed_ms = 0            # last range reading of any kind (valid or not)
        self.warned_no_range = False
        self.sent = None
        self.sent_ms = 0

    # ---- list
    def clear_list(self):
        self.wp = []

    def add(self, x, y):
        if len(self.wp) >= WAYPOINT_MAX or not (math.isfinite(x) and math.isfinite(y)):
            return False
        self.wp.append((float(x), float(y)))
        return True

    def list_changed(self, replaced):
        n = len(self.wp)
        if replaced or self.idx > n:
            self.idx, self.phase, self.driving = 0, self.SEEK, False
        elif self.phase == self.DONE and self.idx < n:
            self.phase, self.driving = self.SEEK, False
        if n == 0:
            self.idx = 0
        self.blocked = self.latched = self.backing = False
        self.blk_fwd = self.blk_rev = False
        self.stop_since = 0
        self.stuck = self.away = self.away_done = False
        self.stuck_ms = 0

    # ---- inputs
    def add_range(self, f, now_ms):
        P = self.P
        self.sensed_ms = now_ms or 1          # the sweep is running (a "no target" reading counts too)
        if not f["valid"] or f["range_mm"] == 0:
            return
        a = f["sensor_angle_deg"]
        r = f["range_mm"] + SENSOR_RADIUS_MM
        lean = f["pitch_deg"] + P("bal.offset_deg")
        tilt = lean * math.cos(math.radians(a))
        if tilt > 0.5 and SENSOR_HEIGHT_MM - r * math.sin(math.radians(tilt)) < P("nav.floor_mm"):
            return
        h = math.radians(f["heading_deg"])
        ax, ay = f["x_mm"] + SENSOR_FWD_MM * math.cos(h), f["y_mm"] + SENSOR_FWD_MM * math.sin(h)
        d = h + math.radians(a)
        self.pts.append((ax + r * math.cos(d), ay + r * math.sin(d), now_ms))
        if len(self.pts) > self.PTS:
            del self.pts[0]
        self.last_range_ms = now_ms or 1

    def on_mode_change(self, i):
        self.mode = i["mode"]
        self.now_ms = i["now_ms"]
        self.speed = self.turn = 0.0
        self.blocked = self.latched = self.backing = False
        self.blk_fwd = self.blk_rev = False
        self.driving = False
        self.stop_since = 0
        self.stuck = self.away = self.away_done = False
        self.stuck_ms = 0
        n = len(self.wp)
        if i["mode"] in (lp.MODE_GOTO, lp.MODE_EXPLORE):
            if self.idx >= n:
                self.idx = 0
            self.phase = self.SEEK if n else self.DONE
        if i["mode"] == lp.MODE_HOLD:
            self.hold = (i["x"], i["y"])
        if i["mode"] in NAV_MODES:
            self.warned_no_range = self.blind(i["now_ms"])
            if self.warned_no_range:
                self._log_blind(i["mode"])
            if i["mode"] != lp.MODE_HOLD:
                self.log("nav: %u waypoints, going to %u" % (n, self.idx + 1))

    def blind(self, now_ms):
        """No range reading for 3 s (rng.enable 0, sensor fault): no obstacle stop (safety review #38)."""
        return not self.sensed_ms or now_ms - self.sensed_ms > BLIND_MS

    def _log_blind(self, mode):
        if mode == lp.MODE_EXPLORE or self.P("nav.blind_speed") <= 0:
            self.log("nav: no range readings - obstacle stop is off: %s paused"
                     % ("EXPLORE" if mode == lp.MODE_EXPLORE else "GOTO"))
        elif mode == lp.MODE_GOTO:
            self.log("nav: no range readings - obstacle stop is off: GOTO max %.0f mm/s" % self.P("nav.blind_speed"))
        else:
            self.log("nav: no range readings - obstacle stop is off")

    def on_pose_reset(self):
        self.wp = []
        self.pts = []
        self.hold = (0.0, 0.0)
        self.list_changed(True)

    # ---- obstacles
    def _update_obstacles(self, i):
        P = self.P
        c, s = math.cos(math.radians(i["heading"])), math.sin(math.radians(i["heading"]))
        # with the side check on (#62), points beside the body (> its half width + 10) steer / slow it down
        # instead of stopping it: a wall alongside or a door jamb is not in the way
        half = (min(0.5 * P("nav.corridor_mm"), HALF_WIDTH_MM + 10.0) if P("nav.side_min_mm") > 0
                else 0.5 * P("nav.corridor_mm"))
        age = P("nav.obst_age_ms")
        front = rear = gl = gr = near = near_xr = 0.0
        for px, py, t in self.pts:
            if i["now_ms"] - t > age:
                continue
            dx, dy = px - i["x"], py - i["y"]
            xr, yr = dx * c + dy * s, -dx * s + dy * c
            if -SIDE_BACK_MM < xr < SIDE_AHEAD_MM and abs(yr) >= SIDE_INNER_MM:   # beside the body: side gap
                g = max(1.0, abs(yr) - HALF_WIDTH_MM)
                if yr > 0:
                    if gl == 0 or g < gl:
                        gl = g
                elif gr == 0 or g < gr:
                    gr = g
            if abs(xr) > 30.0:            # ahead / behind, inside the corners' swing circle
                r = math.hypot(xr, yr)
                if r < SWING_R_MM and (near == 0 or r < near):
                    near, near_xr = r, xr
            if abs(yr) > half:
                continue
            if xr >= SENSOR_FWD_MM:
                d = max(1.0, xr - SENSOR_FWD_MM - SENSOR_RADIUS_MM)
                if front == 0 or d < front:
                    front = d
            else:
                d = max(1.0, SENSOR_FWD_MM - SENSOR_RADIUS_MM - xr)
                if rear == 0 or d < rear:
                    rear = d
        self.front, self.rear = front, rear
        self.gap_l, self.gap_r = gl, gr
        self.near_mm, self.near_xr = near, near_xr

    def _side_turn(self):
        """Side walls (#62): (turn deg/s away from walls closer than nav.side_min_mm to the body side,
        how far the nearer one is inside it)."""
        P = self.P
        smin = P("nav.side_min_mm")
        pl = smin - self.gap_l if smin > 0 and 0 < self.gap_l < smin else 0.0
        pr = smin - self.gap_r if smin > 0 and 0 < self.gap_r < smin else 0.0
        return P("nav.side_kp") * (pr - pl), max(pl, pr)

    def _side_avoid(self, vmax, tmax):
        """Driving: steer away from side walls and slow down (linear to nav.min_speed at gap 0)."""
        P = self.P
        t, inside = self._side_turn()
        if inside <= 0:
            return
        self.turn = _clamp(self.turn + t, tmax)
        smin = P("nav.side_min_mm")
        lim = max(P("nav.min_speed"), vmax * (smin - inside) / smin)
        if self.speed > lim:
            self.speed = lim

    def _stuck_check(self, i, turning, trying):
        """Stuck (#62): commanded to move but no progress for nav.stuck_ms."""
        P = self.P
        if not trying or P("nav.stuck_ms") <= 0:
            return False
        self.stuck_seen = True
        sx, sy, sh = self.stuck_pose
        moved = (abs(i["heading"] - sh) >= STUCK_DEG if turning
                 else math.hypot(i["x"] - sx, i["y"] - sy) >= P("nav.stuck_mm"))
        if not self.stuck_ms or turning != self.stuck_turn or moved:
            self.stuck_ms = i["now_ms"] or 1
            self.stuck_pose = (i["x"], i["y"], i["heading"])
            self.stuck_turn = turning
            return False
        return i["now_ms"] - self.stuck_ms >= P("nav.stuck_ms")

    def _start_stuck(self, i, turning):
        P = self.P
        self.stuck = self.blocked = True
        self.blk_h = i["heading"]
        self.driving = True              # so the BLOCKED branch runs (backs off) even if it was turning
        self.stuck_ms = 0
        if i["mode"] == lp.MODE_EXPLORE:
            self.latched = True
        self.log("nav: stuck, no progress %s for %.1f s (waypoint %u/%u)"
                 % ("turning" if turning else "driving", P("nav.stuck_ms") / 1000.0, self.idx + 1, len(self.wp)))
        self.back_mm = P("nav.stuck_backoff_mm")
        self.backing = self.back_mm > 0
        self.back = (i["x"], i["y"], i["now_ms"])
        self.speed = self.turn = 0.0
        self.state = NAV_BLOCKED
        self.obst = self.front

    def limit_speed(self, v, vmax, apply):
        P = self.P
        self.obst_flag = False
        if abs(v) < 1e-3:
            self.obst = self.front
            return v
        fwd = v > 0
        d = self.front if fwd else self.rear
        key = "fwd" if fwd else "rev"
        blk = self.blk_fwd if fwd else self.blk_rev
        stop = P("rng.obstacle_mm")
        if stop <= 0:
            blk = False
            self.obst = d
        else:
            # blocked until nothing closer than stop + clear (+ backoff) was seen for nav.obst_age_ms
            clear = stop + P("nav.clear_mm") + P("nav.backoff_mm")
            if 0 < d < clear:
                self.close[key] = (self.now_ms or 1, d)
            close_ms, close_mm = self.close[key]
            if 0 < d < stop:
                blk = True
            elif blk and (d <= 0 or d > clear) and self.now_ms - close_ms >= P("nav.obst_age_ms"):
                blk = False
            self.obst = d if d > 0 else (close_mm if blk else 0.0)
        if fwd:
            self.blk_fwd = blk
        else:
            self.blk_rev = blk
        if stop <= 0:
            return v
        if blk:
            self.obst_flag = True
            return 0.0 if apply else v
        if d <= 0:
            return v
        slow = P("nav.slow_mm")
        if apply and slow > stop and d < slow:
            lim = max(P("nav.min_speed"), vmax * (d - stop) / (slow - stop))
            if abs(v) > lim:
                v = lim if fwd else -lim
        return v

    # ---- update
    def _set_target(self, i, x, y):
        self.tx, self.ty = x, y
        self.dist = math.hypot(x - i["x"], y - i["y"])

    def update(self, i):
        self.mode = i["mode"]
        self.now_ms = i["now_ms"]
        self.speed = self.turn = 0.0
        self.obst_flag = False
        self.stuck_seen = False
        self._update_obstacles(i)
        if i["mode"] in (lp.MODE_GOTO, lp.MODE_EXPLORE):
            self._update_goto(i)
        elif i["mode"] == lp.MODE_HOLD:
            self._update_hold(i)
        else:
            self.state = NAV_IDLE
            self.obst = self.front
            if self.wp:
                self._set_target(i, *self.wp[min(self.idx, len(self.wp) - 1)])
            else:
                self.dist = 0.0
        if not self.stuck_seen:           # not trying to move this step: the no-progress window starts over
            self.stuck_ms = 0

    def _start_blocked(self, i):
        self.blocked = True
        self.blk_h = i["heading"]
        if i["mode"] == lp.MODE_EXPLORE:
            self.latched = True
        self.log("nav: blocked, obstacle at %.0f mm (waypoint %u/%u)" % (self.obst, self.idx + 1, len(self.wp)))
        self.back_mm = self.P("nav.backoff_mm")
        if self.back_mm > 0:
            self.backing = True
            self.back = (i["x"], i["y"], i["now_ms"])

    def _update_goto(self, i):
        P = self.P
        n = len(self.wp)
        if n == 0:
            self.state = NAV_IDLE
            self.tx = self.ty = self.dist = 0.0
            self.obst = self.front
            return
        if self.phase == self.DONE and self.idx < n:
            self.phase = self.SEEK
        self._set_target(i, *self.wp[min(self.idx, n - 1)])
        # no range readings: EXPLORE pauses; GOTO is capped at nav.blind_speed (0 = pauses)
        no_range = self.blind(i["now_ms"])
        if no_range != self.warned_no_range:
            self.warned_no_range = no_range
            if no_range:
                self._log_blind(i["mode"])
            else:
                self.log("nav: range readings back - obstacle stop on")
        blind_v = P("nav.blind_speed")
        blind_stop = no_range and (i["mode"] == lp.MODE_EXPLORE or blind_v <= 0)
        run = i["upright"] and i["live"] and not blind_stop   # else paused: stand still, keep the state
        if self.phase == self.DONE:           # arrived: hold the last point
            self.state = NAV_ARRIVED
            self.obst = self.front
            if run:
                self.speed = self._hold_speed(i, self.tx, self.ty)
            return
        vmax = min(P("nav.max_speed"), i["max_speed"], blind_v if no_range else 1e9)
        tmax = min(P("nav.max_turn"), i["max_turn"])
        arrive = P("nav.arrive_mm")
        stopped = abs(i["speed"]) < P("nav.stop_speed") and abs(i["yaw_rate"]) < P("nav.stop_turn_dps")
        if not run:
            self.stop_since = 0
            self.obst = self.front
            return

        if self.phase == self.END_TURN:
            e = _wrap180(P("nav.end_heading_deg") - i["heading"])
            self.turn = _clamp(P("nav.turn_kp") * e, tmax)
            self.speed = self._hold_speed(i, self.tx, self.ty)
            self.state = NAV_TURNING
            self.obst = self.front
            if abs(e) < P("nav.heading_tol_deg") and abs(i["yaw_rate"]) < P("nav.stop_turn_dps"):
                self.turn = 0.0
                self.phase, self.idx, self.state = self.DONE, n, NAV_ARRIVED
                self.log("nav: arrived, heading %.1f" % _wrap180(i["heading"]))
            return

        if self.phase == self.STOPPING:
            self.state = NAV_DRIVING
            self.obst = self.front
            # hold the point (forward / back, no steering) until it has really stopped
            self.speed = self._hold_speed(i, self.tx, self.ty)
            self.obst = self.front
            if self.dist > 2.0 * arrive:
                self.phase, self.driving = self.SEEK, False
            elif not stopped:
                self.stop_since = 0
            else:
                if not self.stop_since:
                    self.stop_since = i["now_ms"] or 1
                if i["now_ms"] - self.stop_since >= P("nav.dwell_ms"):
                    self.log("nav: reached %u/%u, %.0f mm off" % (self.idx + 1, n, self.dist))
                    self.stop_since = 0
                    if self.idx + 1 < n:
                        self.idx += 1
                        self.phase, self.driving = self.SEEK, False
                        self._set_target(i, *self.wp[self.idx])
                        self.state = NAV_TURNING
                    elif P("nav.end_heading_deg") >= 0:
                        self.phase, self.state = self.END_TURN, NAV_TURNING
                    else:
                        self.phase, self.idx, self.state = self.DONE, n, NAV_ARRIVED
                        self.log("nav: arrived")
            return

        # SEEK
        if self.dist < arrive:
            self.phase, self.stop_since = self.STOPPING, 0
            self.blocked = self.latched = self.backing = False
            self.state = NAV_DRIVING
            self.obst = self.front
            return
        bearing = math.degrees(math.atan2(self.ty - i["y"], self.tx - i["x"]))
        e = _wrap180(bearing - i["heading"])
        far = self.dist > 2.0 * arrive
        align = P("nav.align_deg")
        if not self.blocked:
            if not self.driving and abs(e) < align:
                self.driving = True
                self.away = self.away_done = False
            elif self.driving and far and abs(e) > 3.0 * align:
                self.driving = False
        if not self.driving:
            self.state = NAV_TURNING
            self.obst = self.front
            # something inside the corners' swing circle ahead / behind: first step straight away from it (#62)
            if not self.away_done and self.near_mm > 0 and P("nav.stuck_backoff_mm") > 0:
                if not self.away:
                    self.away = True
                    self.away_pose = (i["x"], i["y"], i["now_ms"])
                    self.log("nav: too close to turn (%.0f mm), stepping %s"
                             % (self.near_mm, "back" if self.near_xr > 0 else "forward"))
                step = max(P("nav.min_speed"), 30.0)
                va = self.limit_speed(-step if self.near_xr > 0 else step, vmax, True)
                ax, ay, ams = self.away_pose
                if (not self.obst_flag and math.hypot(i["x"] - ax, i["y"] - ay) < P("nav.stuck_backoff_mm")
                        and i["now_ms"] - ams < 4000):
                    self.speed = va
                    self.obst = self.front
                    return
                self.away_done = True     # far enough (or the way out is blocked too): turn anyway
            self.away = False
            self.obst_flag = False
            self.turn = _clamp(P("nav.turn_kp") * e, tmax)
            # target behind and a wall close at one side: turn the nose away from that wall (#62)
            smin = P("nav.side_min_mm")
            if smin > 0 and abs(e) > 120.0:
                l, r = 0 < self.gap_l < smin, 0 < self.gap_r < smin
                if l and (not r or self.gap_l < self.gap_r):
                    self.turn = -abs(self.turn)
                elif r:
                    self.turn = abs(self.turn)
            if self._stuck_check(i, True, abs(self.turn) >= P("nav.stop_turn_dps")):
                self._start_stuck(i, True)
            return
        v = P("nav.speed_kp") * self.dist * max(0.0, math.cos(math.radians(e)))
        v = min(max(v, P("nav.min_speed")), vmax)
        vl = self.limit_speed(v, vmax, True)
        if self.latched or self.stuck:        # EXPLORE: stays blocked until a new list; stuck: backing off
            self.obst_flag = True
        if self.obst_flag:
            if not self.blocked:
                self._start_blocked(i)
            obst = self.obst
            vl = 0.0
            if self.backing:
                bx, by, bms = self.back
                back = math.hypot(i["x"] - bx, i["y"] - by)
                # stuck: twice as fast (a balancing robot pressed against a wall barely follows the slowest speed)
                vb = self.limit_speed(-max(P("nav.min_speed"), 30.0) * (2.0 if self.stuck else 1.0), vmax, True)
                if self.obst_flag or back >= self.back_mm or i["now_ms"] - bms > 5000:
                    self.backing = False
                    vb = 0.0
                vl = vb
                self.obst = obst
                self.obst_flag = True
            if not self.backing and i["mode"] != lp.MODE_EXPLORE:
                self.stuck = False            # GOTO: backed off, try again
            self.speed, self.turn, self.state = vl, 0.0, NAV_BLOCKED
            # GOTO, blocked with a wall alongside (heading into it): turn away from it a little, in place (#62)
            if (not self.backing and not self.latched and not self.stuck
                    and abs(i["heading"] - self.blk_h) < BLOCKED_TURN_DEG):
                self.turn = _clamp(self._side_turn()[0], tmax)
            return
        if self.blocked:
            self.log("nav: clear, going on")
        self.blocked = self.backing = False
        self.speed = vl
        self.turn = _clamp(P("nav.turn_kp") * e, tmax) if far else 0.0
        self._side_avoid(vmax, tmax)
        self.state = NAV_DRIVING
        # (not in the last 3 x arrive_mm: slow there, and a balancing robot rocks to and fro before it moves)
        if self._stuck_check(i, False, self.dist > 3.0 * arrive and self.speed >= max(20.0, 0.5 * P("nav.min_speed"))):
            self._start_stuck(i, False)

    def _hold_speed(self, i, x, y):
        """nav.hold_* position loop on the forward error."""
        P = self.P
        h = math.radians(i["heading"])
        f = (x - i["x"]) * math.cos(h) + (y - i["y"]) * math.sin(h)
        if abs(f) <= P("nav.hold_db_mm"):
            return 0.0
        vmax = min(P("nav.hold_max_speed"), i["max_speed"])
        return self.limit_speed(_clamp(P("nav.hold_kp") * f, vmax), vmax, True)

    def _update_hold(self, i):
        self._set_target(i, *self.hold)
        self.state = NAV_HOLDING
        self.obst = self.front
        if not i["upright"] or not i["live"]:
            return
        self.speed = self._hold_speed(i, *self.hold)
        if self.obst_flag:
            self.state = NAV_BLOCKED

    # ---- status / log
    def active(self):
        return self.state in (NAV_TURNING, NAV_DRIVING, NAV_BLOCKED)

    def status_due(self, now_ms):
        if (self.state, self.idx, len(self.wp)) != self.sent:
            return True
        return self.mode in NAV_MODES and now_ms - self.sent_ms >= 500

    def status_sent(self, now_ms):
        self.sent = (self.state, self.idx, len(self.wp))
        self.sent_ms = now_ms

    def status(self):
        return {"state": self.state, "waypoint_count": min(255, len(self.wp)), "nav_index": min(255, self.idx),
                "target_x_mm": self.tx, "target_y_mm": self.ty, "distance_mm": self.dist,
                "obstacle_mm": self.obst}

    def log(self, text):
        self._log(text)


class Controller:
    """Port of src/controller.cpp (balance + heading + velocity loops)."""

    def __init__(self, P):
        self.P = P
        self.upright = False
        self.start_s = 0.0
        self.heading_sp = 0.0
        self.snap_heading = False
        self.out = {}
        self._reset(0.0, 0.0, 0.0)

    def _reset(self, heading, speed, pitch):
        self.bal_i = self.str_i = self.vel_i = self.vel_out = self.vel_acc_dt = 0.0
        self.speed_sp = self.turn_sp = 0.0
        self.heading_sp = heading
        self.last_speed = speed
        self.last_pitch = pitch

    def on_mode_change(self):
        self.snap_heading = True

    def step(self, i):
        P = self.P
        o = {"bal_p": 0.0, "bal_i": 0.0, "bal_d": 0.0, "steer": 0.0, "vel_out": 0.0, "speed_sp": 0.0,
             "turn_sp": 0.0, "left": 0.0, "right": 0.0, "active": False, "upright": False}
        offset = P("bal.offset_deg")
        cutoff = P("bal.cutoff_deg")
        mot_max = P("mot.max")
        too_far = abs(i["pitch"]) > cutoff or abs(i["roll"]) > cutoff
        if self.snap_heading:
            self.heading_sp = i["heading"]
            self.str_i = 0.0
            self.snap_heading = False
        if not i["enable"] or (self.upright and too_far):
            self.upright = False
            self._reset(i["heading"], i["speed"], i["pitch"])
        if not self.upright:
            if i["enable"] and not too_far and abs(i["pitch"] + offset) < P("bal.reengage_deg"):
                self.upright = True
                self.start_s = i["now"]
                self._reset(i["heading"], i["speed"], i["pitch"])
            else:
                o["balance_target"] = -offset
                self.heading_sp = i["heading"]
                self.out = o
                return o
        settled = (i["now"] - self.start_s) * 1000.0 >= P("ctl.settle_ms")
        loops = int(round(P("ctl.loops")))
        dt = i["dt"]
        self.speed_sp = _slew(self.speed_sp, i["target_speed"] if settled else 0.0, P("drv.accel") * dt)
        self.turn_sp = _slew(self.turn_sp, i["target_turn"] if settled else 0.0, P("drv.turn_accel") * dt)

        if (loops & 2) and settled and i["speed_ok"]:
            self.vel_acc_dt += dt
            if self.vel_acc_dt >= VEL_LOOP_DT:
                vdt, self.vel_acc_dt = self.vel_acc_dt, 0.0
                lim = P("vel.limit_deg")
                e = self.speed_sp - i["speed"]
                self.vel_i = _clamp(self.vel_i + P("vel.ki") * e * vdt, lim)
                accel = (i["speed"] - self.last_speed) / vdt
                self.last_speed = i["speed"]
                self.vel_out = _clamp(P("vel.kp") * e + self.vel_i - P("vel.kd") * accel, lim)
        else:
            self.vel_out = self.vel_i = self.vel_acc_dt = 0.0
            self.last_speed = i["speed"]

        target = self.vel_out - offset
        e = i["pitch"] - target
        rate = i["pitch_rate"] if P("bal.d_src") >= 0.5 else (i["pitch"] - self.last_pitch) / dt
        self.last_pitch = i["pitch"]
        self.bal_i = _clamp(self.bal_i + P("bal.ki") * e * dt, mot_max)
        o["bal_p"] = P("bal.kp") * e
        o["bal_i"] = self.bal_i
        o["bal_d"] = P("bal.kd") * rate
        bal = _clamp(o["bal_p"] + o["bal_i"] + o["bal_d"], mot_max)

        steer = 0.0
        if (loops & 1) and settled:
            self.heading_sp += self.turn_sp * dt
            max_err = P("str.max_err_deg")
            self.heading_sp = max(i["heading"] - max_err, min(i["heading"] + max_err, self.heading_sp))
            eh = self.heading_sp - i["heading"]
            self.str_i = _clamp(self.str_i + P("str.ki") * eh * dt, mot_max)
            steer = _clamp(P("str.kp") * eh + self.str_i - P("str.kd") * i["yaw_rate"], mot_max)
        else:
            self.heading_sp = i["heading"]
            self.str_i = 0.0

        o["left"] = _clamp((bal - steer) * P("mot.left_scale"), mot_max)
        o["right"] = _clamp((bal + steer) * P("mot.right_scale"), mot_max)
        o.update(balance_target=target, speed_sp=self.speed_sp, turn_sp=self.turn_sp, steer=steer,
                 vel_out=self.vel_out, upright=True, active=True)
        self.out = o
        return o


class SimRobot:
    def __init__(self, remote=False, remote_armed=True, seed=None, head_error_deg=0.0):
        self.rng = random.Random(seed)
        # sim-only: the head is mounted this far CCW of where the firmware thinks (what rng.servo_trim_deg fixes)
        self.head_error_deg = head_error_deg
        self.room = Room()
        self.t = 0.0                  # simulated time, s (advanced by step())
        self.params = [[n, v, lo, hi] for n, v, lo, hi in PARAMS]
        self.pidx = {p[0]: i for i, p in enumerate(self.params)}
        self.eeprom = None            # values saved with cfg.save (this process only)
        self.ctl = Controller(self.P)
        self.dt = 1.0 / self.P("ctl.rate_hz")
        # Mode manager (src/mode_manager.cpp): with the RC remote present its ARMED switch rules;
        # without it, SET_MODE to a non-IDLE mode arms and SET_MODE IDLE disarms.
        self.remote = remote
        self.remote_armed = remote_armed
        self.armed = remote and remote_armed
        self.requested = lp.MODE_IDLE
        self.mode = lp.MODE_IDLE
        self.save_pending = False
        self.save_wait_logged = False
        self.defaults_pending = False     # cfg.defaults waits for the motors off too (issue #42)
        self.defaults_wait_logged = False
        self.link_estop = False           # station E-stop latch (EVT_LINK_ESTOP, issue #41)
        self.arm_req = None               # time of the last EVT_LINK_ARM (explicit arm request), s
        self.pi_clients = 0               # HeartbeatFrame.clients (0 = nobody watching, issue #38)
        self.step_n = 0
        self.last_tlm = -1.0
        self.rate_t0, self.rate_n0, self.loop_hz = 0.0, 0, 0.0
        # true physical state
        self.X = self.Y = 0.0
        self.TH = 0.0                 # deg, continuous, + = CCW
        self.theta = math.radians(LIE_DEG)  # true lean, rad (+ = forward); boots lying on its front
        self.theta_rate = 0.0
        self.vl = self.vr = 0.0       # wheel speeds mm/s
        self.v_prev = 0.0
        self.speed_f = 0.0            # odometry speed (filtered like the 80 ms encoder window)
        self.hand_until = 0.0
        self.engaged_at = None
        # odometry frame (reset with EVT_RESET_POSE)
        self.ox = self.oy = 0.0
        self.oth = 0.0
        # sim-only (issue #14): the real wheels / track / gyro, when they differ from the odo.* params.
        # None = the odometry is exact (the pose is the true pose). See set_hardware().
        self.hw = None
        self.odx = self.ody = 0.0     # odometry pose integrated from the (mis-scaled) wheel travel
        self.gyro_bias = 0.0          # deg/s the gyro heading drifts
        self.gdrift = 0.0             # deg of drift so far
        self.last_hb = -10.0
        self.pi_link = False
        self.cmd = (0.0, 0.0)
        self.cmd_time = -10.0
        self.cmd_valid = False
        self.nav = Navigator(self.P, self.log)   # waypoints in the odom frame
        self.auto = (0.0, 0.0)        # navigator command, used from the next step (ModeManager::setAutoCmd)
        self.auto_valid = False
        # range sweep (model of src/range_sweep.cpp)
        self.rs_on = self.rs_started = False
        self.rs_k, self.rs_dir = 0, 1
        self.rs_last_cmd = None       # servo command last written, deg
        self.rs_head = 0.0            # head angle the firmware reports (sensor 0)
        self.rs_true_head = 0.0       # where the head really points
        self.rs_next = 0.0
        self.rs_measuring = False
        self.motor_l = self.motor_r = 0.0
        self.battery = 12.45
        self.drain = 0.00002          # V/s at no load (x11 at full duty); --drain speeds it up for demos
        self.bench = False            # sim-only: no balance lead, the battery taps read ~0 V (Health page: "bench supply")
        self.fault = 0                # sim-only: telemetry error_code (1 IMU init, 2 IMU stale, 3 / 4 encoder)
        self.radio_log_at = 10.0      # the firmware prints its radio stats line every 10 s while packets arrive
        self.push = 0.0               # sim-only disturbance (rad/s^2), see push()
        self.mtest = SimMotorTest(self.log)   # wheels-up motor test (mot.test)
        self.on_stand = False
        self.tt = []                  # TT motors on the stand (see _motor_test_step)
        self.floor = None             # FloorWheel pair: volts motor layer + stiction (use_floor_wheels, #69)
        self.enc_ring = []
        self.out = []                 # frames to send
        self.log("boot sim v4, params DEFAULTS, imu ok, radio %s" % ("ok" if remote else "ok (no remote)"))

    # ---------------------------------------------------------------- helpers
    def P(self, name):
        return self.params[self.pidx[name]][1]

    def now(self):
        return self.t

    def t_us(self):
        return int(self.t * 1e6) & 0xFFFFFFFF

    def set_hardware(self, wheel_d=66.0, track=TRACK_MM, lr=1.0, gyro_bias_dps=0.0):
        """Sim-only: the REAL mean wheel diameter, track width and right/left diameter ratio, and the gyro drift.
        The robot's odometry then uses its odo.* params against them, so a wrong param gives a wrong pose
        (what the Calibrate odometry tool measures and fixes)."""
        self.hw = {"d": float(wheel_d), "track": float(track), "lr": float(lr)}
        self.gyro_bias = float(gyro_bias_dps)
        self.odx = self.ody = 0.0

    def track_mm(self):
        return self.hw["track"] if self.hw else TRACK_MM

    def wheel_scale(self):
        """(left, right): odometry mm per true mm = param diameter / real diameter, per wheel."""
        if not self.hw:
            return 1.0, 1.0
        d, r = self.P("odo.wheel_d_mm"), self.P("odo.lr_ratio")
        pl = 2.0 * d / (1.0 + r)
        tl = 2.0 * self.hw["d"] / (1.0 + self.hw["lr"])
        return pl / tl, r * pl / (self.hw["lr"] * tl)

    def odom(self):
        """Odometry pose (what the robot reports): world pose relative to the last pose reset."""
        if self.hw:
            return self.odx, self.ody, self.TH - self.oth + self.gdrift
        dx, dy = self.X - self.ox, self.Y - self.oy
        c, s = math.cos(math.radians(-self.oth)), math.sin(math.radians(-self.oth))
        return dx * c - dy * s, dx * s + dy * c, self.TH - self.oth

    def log(self, text):
        self.out.append(lp.encode_log(text))

    def hello(self):
        return lp.encode_hello(param_count=len(self.params), firmware="sim v4 2026-11-20")

    def param_frames(self, ids=None):
        return [lp.encode_param_info(i, p[1], p[2], p[3], p[0])
                for i, p in enumerate(self.params) if ids is None or i in ids]

    def radio_live(self):
        return self.remote

    def motors_active(self):
        return bool(self.ctl.out.get("active"))

    def push_robot(self, deg_s=60.0):
        """Sim-only: a shove (adds pitch rate), e.g. for testing recovery."""
        self.theta_rate += math.radians(deg_s)

    # ---------------------------------------------------------------- link in
    def on_frame(self, t, p):
        d = lp.decode(t, p)
        if t == lp.MSG_HEARTBEAT:
            self.last_hb = self.now()
            self.pi_clients = d["clients"]
            if not self.pi_link:
                self.pi_link = True
                self.log("pi link up (%d clients)" % d["clients"])
        elif t == lp.MSG_CMD_VEL:
            if all(math.isfinite(v) for v in (d["speed_mms"], d["turn_dps"])):
                self.cmd = (d["speed_mms"], d["turn_dps"])
                self.cmd_time = self.now()
                self.cmd_valid = True
        elif t == lp.MSG_SET_MODE:
            r = self.set_mode(d["mode"])
            if r == "estop":
                self.log("SET_MODE %u ignored: station E-stop latched (release it first)" % d["mode"])
            elif r == "not_armed":
                self.log("SET_MODE %u ignored: not armed (arm first)" % d["mode"])
        elif t == lp.MSG_SET_PARAM:
            self.set_param(d["id"], d["value"])
        elif t == lp.MSG_GET_PARAMS:
            self.out.append(self.hello())
            self.out += self.param_frames()
        elif t == lp.MSG_GOTO:
            if d["count"] <= lp.GOTO_MAX_POINTS and len(p) == 2 + d["count"] * 8:
                if d["replace"]:
                    self.nav.clear_list()
                added = sum(1 for x, y in d["points"] if self.nav.add(x, y))
                if added < d["count"]:
                    self.log("waypoints: list full (%u), %u dropped" % (WAYPOINT_MAX, d["count"] - added))
                self.nav.list_changed(bool(d["replace"]))   # also resumes from BLOCKED
        elif t == lp.MSG_EVENT:
            ev = d["event"]
            if ev == lp.EVT_LINK_ESTOP:            # latching station E-stop (main.cpp onLinkFrame)
                if not self.link_estop:
                    self.log("E-STOP (station): latched, motors off")
                self.link_estop, self.armed, self.requested, self.arm_req = True, False, lp.MODE_IDLE, None
                self._update_mode()
            elif ev == lp.EVT_LINK_ESTOP_RELEASE:
                if self.link_estop:
                    self.link_estop, self.armed, self.requested, self.arm_req = False, False, lp.MODE_IDLE, None
                    self.log("E-stop released: re-arm with the remote (ARMED off -> on)" if self.radio_live()
                             else "E-stop released: arm from the station to continue")
            elif ev == lp.EVT_LINK_ARM:
                if not self.link_estop:
                    self.arm_req = self.now()
            else:
                self.on_event(ev)

    def set_param(self, pid, value):
        """params_set(): clamp + apply, echo the applied value. cfg.* are actions and read back 0."""
        if not 0 <= pid < len(self.params) or not math.isfinite(value):
            return
        pr = self.params[pid]
        v = max(pr[2], min(pr[3], value))
        if pr[0] == "cfg.save":
            if v >= 0.5:
                self.save_pending = True        # done in step() once the motors are off
            v = 0.0
        elif pr[0] == "cfg.defaults":
            if v >= 0.5:
                self.defaults_pending = True    # done in step() once the motors are off (issue #42)
            v = 0.0
        pr[1] = v
        self.out += self.param_frames([pid])     # echo = ack

    def on_event(self, ev):
        """handleEvent() in src/main.cpp."""
        x, y, th = self.odom()
        if ev == lp.EVT_MARK_WAYPOINT:
            if self.nav.add(x, y):
                self.nav.list_changed(False)
            else:
                self.log("waypoints: list full (%u)" % WAYPOINT_MAX)
        elif ev == lp.EVT_CLEAR_WAYPOINTS:
            self.nav.clear_list()
            self.nav.list_changed(True)
        elif ev == lp.EVT_GO_HOME:            # list = [first waypoint or origin], then GOTO (never arms)
            hx, hy = self.nav.wp[0] if self.nav.wp else (0.0, 0.0)
            self.nav.clear_list()
            self.nav.add(hx, hy)
            self.nav.list_changed(True)
            if not self.link_estop:            # ModeManager::requestMode: ignored while latched
                self.requested = lp.MODE_GOTO
            self.log("GO_HOME: to %.0f, %.0f" % (hx, hy))
        elif ev == lp.EVT_RESET_POSE:
            h_old = th
            self.ox, self.oy, self.oth = self.X, self.Y, self.TH
            self.odx = self.ody = self.gdrift = 0.0
            self.ctl.heading_sp -= h_old   # keep the heading target where it physically was
            self.nav.on_pose_reset()
            self.log("pose reset (waypoints cleared)")
        elif ev == lp.EVT_START_SCAN:
            self.log("START_SCAN: the sweep runs all the time while rng.enable = 1" if self.P("rng.enable") >= 0.5
                     else "START_SCAN: rng.enable is 0")

    def set_mode(self, mode, arm=False):
        """ModeManager::onPiSetMode: ignored while the link E-stop is latched; with no remote IDLE disarms and
        a mode != IDLE arms only right after an EVT_LINK_ARM (else it changes an armed robot's mode)."""
        if not 0 <= mode < lp.MODE_COUNT:
            return "bad"
        if arm and not self.link_estop:        # Python callers (tour, tests): as if EVT_LINK_ARM came first
            self.arm_req = self.now()
        if self.link_estop:
            return "estop"
        arm_req = self.arm_req is not None and self.now() - self.arm_req <= lp.LINK_ARM_WINDOW_MS / 1000.0
        self.arm_req = None
        if not self.radio_live():
            if mode == lp.MODE_IDLE:
                self.armed = False
            elif not self.armed:
                if not arm_req:
                    return "not_armed"
                self.armed = True
        self.requested = mode
        return "ok"

    def _update_mode(self):
        eff = self.requested if self.armed and not self.link_estop else lp.MODE_IDLE
        if eff == self.mode:
            return
        if eff != lp.MODE_IDLE and self.mode == lp.MODE_IDLE:
            self.log("armed")
        elif eff == lp.MODE_IDLE:
            self.log("disarmed")
        self.mode = eff
        self.cmd_valid = False
        self.auto_valid = False
        self.ctl.on_mode_change()
        self.log("mode %s" % lp.MODE_NAMES[eff])
        self.nav.on_mode_change(self._nav_in(self.now()))

    # ---------------------------------------------------------------- targets (ModeManager::update + navigator)
    def supervised(self):
        """ModeManager::supervised: remote live, or (no remote) the Pi heartbeat with a client connected.
        (The sim's remote never drops out, so the handover rule after a remote loss is not simulated.)"""
        return self.remote or (self.pi_link and self.pi_clients > 0)

    def _nav_in(self, now):
        x, y, th = self.odom()
        return {"now_ms": int(now * 1000.0), "mode": self.mode, "upright": bool(self.ctl.out.get("upright")),
                "live": self.supervised(), "x": x, "y": y, "heading": th, "speed": self.speed_f,
                "yaw_rate": self._wheel_turn_rate(),
                "max_speed": self.P("drv.max_speed"), "max_turn": self.P("drv.max_turn")}

    def _targets(self, now):
        """controlStep() order: ModeManager::update (navigator command from the previous step), then
        Navigator::update, then the obstacle limit for STATION."""
        max_v, max_w = self.P("drv.max_speed"), self.P("drv.max_turn")
        m = self.mode
        v_t = w_t = 0.0
        if m == lp.MODE_STATION:
            if self.pi_link and self.cmd_valid and now - self.cmd_time <= lp.LINK_CMD_VEL_TIMEOUT_MS / 1000.0:
                v_t = _clamp(self.cmd[0], max_v)
                w_t = _clamp(self.cmd[1], max_w)
        elif m in NAV_MODES and self.auto_valid and self.supervised():
            v_t = _clamp(self.auto[0], max_v)
            w_t = _clamp(self.auto[1], max_w)
        if abs(w_t) < self.P("drv.turn_deadband"):
            w_t = 0.0
        nav = self.nav
        nav.update(self._nav_in(now))
        self.auto, self.auto_valid = (nav.speed, nav.turn), True
        if m not in NAV_MODES:
            if m == lp.MODE_STATION and v_t != 0.0:
                v_t = nav.limit_speed(v_t, max_v, True)
            else:
                moving = self.speed_f if abs(self.speed_f) > 50.0 else 0.0
                nav.limit_speed(v_t if v_t != 0.0 else moving, max_v, False)
        return v_t, w_t

    # ---------------------------------------------------------------- one balance step
    def use_floor_wheels(self, right=((2.73, 2.49), (0.85, 0.88)), left=((2.46, 2.11), (0.82, 0.76)), k=124.0):
        """Sim-only (issue #69): balance on TT motors with static friction (breakaway > kinetic volts) through
        a port of the firmware motor layer (Motor::set: mot.model 0 / 1, stiction kick, vcomp, brake cap)."""
        self.floor = [FloorWheel(left[0], left[1], k), FloorWheel(right[0], right[1], k)]
        self.kick = [StictionKick(), StictionKick()]
        self.floor_speed = [0.0, 0.0]
        self.volts = [0.0, 0.0]

    def _motor_volts(self, side, cmd, now, batt):
        """Motor::set() (src/motors.cpp) without the speed loop: command -> volts at the motor."""
        P = self.P
        w = "lr"[side]
        if cmd == 0.0:
            self.kick[side].reset()
            return 0.0
        brake = (P("mot.drive_mode") >= 0.5 and P("mot.pwm_hz") <= P("mot.brake_max_hz") and P("mot.model") >= 0.5)
        if P("mot.model") >= 0.5:
            v_ref = P("mot.v_ref")
            u = cmd / MOTOR_FULL
            v = motor_dz_volts(u, P("mot.dz_%s_%s" % (w, "fwd" if cmd >= 0 else "rev")), v_ref, P("mot.dz_band"))
            v = abs(self.kick[side].apply(u, v if cmd >= 0 else -v, self.floor_speed[side], True, int(now * 1e6),
                                          P("mot.bk_%s_fwd" % w), P("mot.bk_%s_rev" % w), P("mot.bk_gain"),
                                          P("mot.bk_ms"), P("mot.bk_band"), P("mot.bk_mms")))
            pwm = min(v, v_ref) / v_ref * MOTOR_FULL
        else:
            self.kick[side].reset()
            pwm = motor_curve(cmd, P("mot.curve_exp"), P("mot.knee_in"), P("mot.knee_out"))
        if P("mot.vcomp") >= 0.5 and batt > 0:
            pwm *= max(0.5, min(1.3, P("mot.v_ref") / batt))
        duty = math.copysign(min(pwm, MOTOR_FULL), cmd)
        if brake:
            duty = _clamp(duty, P("mot.brake_duty_max"))
        return duty / MOTOR_FULL * batt

    def _wheel_target(self, counts, strength):
        P = self.P
        pwm = motor_curve(counts, P("mot.curve_exp"), P("mot.knee_in"), P("mot.knee_out"))
        eff = max(0.0, pwm - MOTOR_DEADZONE) / (MOTOR_FULL - MOTOR_DEADZONE)
        return math.copysign(eff * MOTOR_VMAX * strength * self.battery / 12.4, counts)

    def step(self, dt=None):
        dt = dt or self.dt
        self.t += dt
        now = self.t
        rng = self.rng
        self.step_n += 1
        if self.pi_link and now - self.last_hb > lp.LINK_HEARTBEAT_TIMEOUT_MS / 1000.0:
            self.pi_link = False
            self.log("pi link lost: station commands stopped")
        self._update_mode()
        v_t, w_t = self._targets(now)

        # sensors (what the firmware sees)
        pitch = math.degrees(self.theta) + IMU_MOUNT_DEG + rng.gauss(0, 0.015)
        pitch_rate = math.degrees(self.theta_rate) + rng.gauss(0, 0.4)
        yaw_rate = math.degrees((self.vr - self.vl) / self.track_mm())
        ctl_in = {"dt": dt, "now": now, "pitch": pitch, "pitch_rate": pitch_rate, "roll": rng.gauss(0, 0.2),
                  "heading": self.TH - self.oth + self.gdrift, "yaw_rate": yaw_rate + rng.gauss(0, 0.3),
                  "speed": self.speed_f, "speed_ok": True, "target_speed": v_t, "target_turn": w_t,
                  "enable": self.mode != lp.MODE_IDLE}
        o = self.ctl.step(ctl_in)
        if o["active"]:
            self.motor_l, self.motor_r = o["left"], o["right"]
            if self.engaged_at is None:
                self.engaged_at = now
        else:
            self.motor_l = self.motor_r = 0.0
            self.engaged_at = None

        # wheels: first-order lag towards the speed the PWM gives (dead zone, battery sag)
        if self.floor is not None and o["active"]:
            for side, (fw, cmd) in enumerate(zip(self.floor, (self.motor_l, self.motor_r))):
                self.volts[side] = self._motor_volts(side, cmd, now, self.battery)
                fw.step(self.volts[side], dt)
            self.vl, self.vr = self.floor[0].v, self.floor[1].v
        else:
            if o["active"]:
                tl = self._wheel_target(self.motor_l, LEFT_STRONGER)
                tr = self._wheel_target(self.motor_r, 1.0)
                k = min(1.0, dt / MOTOR_TAU)
            else:   # coasting / lying on the floor
                tl = tr = 0.0
                k = min(1.0, dt / (0.08 if abs(self.theta) >= math.radians(LIE_DEG) - 1e-6 else 0.4))
            self.vl += (tl - self.vl) * k
            self.vr += (tr - self.vr) * k
            if self.floor is not None:
                self.floor[0].coast(self.vl, dt)
                self.floor[1].coast(self.vr, dt)
                self.kick[0].reset()
                self.kick[1].reset()
        if self.floor is not None:   # encoder read after the step (feeds the next step's kick)
            self.floor_speed = [fw.speed_est(now) for fw in self.floor]
        v = (self.vl + self.vr) / 2
        acc = (v - self.v_prev) / dt / 1000.0    # m/s^2
        self.v_prev = v
        sl, sr = self.wheel_scale()
        self.speed_f += ((self.vl * sl + self.vr * sr) / 2 - self.speed_f) * min(1.0, dt / 0.04)

        # inverted pendulum + a hand that stands the robot up when it is armed but lying down
        lying = abs(self.theta) >= math.radians(LIE_DEG) - 1e-6
        hand = self.mode != lp.MODE_IDLE and (not o["active"] or now - self.engaged_at < 0.35)
        th_acc = PEND_G * math.sin(self.theta) - PEND_A * acc * math.cos(self.theta)
        th_acc += rng.gauss(0, 6.0)               # floor bumps, backlash, air
        if hand:
            th_acc = -60.0 * self.theta - 14.0 * self.theta_rate
        elif lying and not o["active"]:
            th_acc = 0.0
        self.theta_rate += th_acc * dt
        self.theta += self.theta_rate * dt
        lim = math.radians(LIE_DEG)
        if abs(self.theta) >= lim:                 # hit the floor
            self.theta = math.copysign(lim, self.theta)
            self.theta_rate = 0.0

        stand_pose = (self.X, self.Y, self.TH) if self.on_stand else None
        # move (stop at walls)
        w = math.degrees((self.vr - self.vl) / self.track_mm())
        self.TH += w * dt
        self.gdrift += self.gyro_bias * dt
        nx = self.X + v * math.cos(math.radians(self.TH)) * dt
        ny = self.Y + v * math.sin(math.radians(self.TH)) * dt
        if abs(v) < 1e-3 or self.room.clearance(nx, ny) > ROBOT_RADIUS:
            self.X, self.Y = nx, ny
            if self.hw:     # odometry: mis-scaled wheel travel along the (gyro) heading
                hh = math.radians(self.TH - self.oth + self.gdrift)
                dd = (self.vl * sl + self.vr * sr) / 2 * dt
                self.odx += dd * math.cos(hh)
                self.ody += dd * math.sin(hh)
        else:
            self.vl = self.vr = self.v_prev = 0.0
            for fw in self.floor or ():
                fw.v, fw.moving = 0.0, False

        self._motor_test_step(now, dt, pitch, pitch_rate, stand_pose)

        load = (abs(self.motor_l) + abs(self.motor_r)) / (2 * MOTOR_FULL)
        self.battery -= dt * self.drain * (1 + 10 * load)
        batt_v = self.battery - 0.5 * load

        # params: cfg.save waits until the motors are off (src/main.cpp)
        if self.save_pending:
            if not o["active"]:
                self.save_pending = self.save_wait_logged = False
                self.eeprom = [p[1] for p in self.params]
                self.log("params: saved")
            elif not self.save_wait_logged:
                self.save_wait_logged = True
                self.log("params: save waits until the motors are off")
        if self.defaults_pending:                     # cfg.defaults also waits for the motors off (#42)
            if not o["active"] and not any(self.mtest.duty):
                self.defaults_pending = self.defaults_wait_logged = False
                for p, (_n, dflt, _lo, _hi) in zip(self.params, PARAMS):
                    p[1] = dflt
                self.out += self.param_frames()   # firmware re-streams every param
                self.log("params: defaults restored (not saved)")
            elif not self.defaults_wait_logged:
                self.defaults_wait_logged = True
                self.log("params: defaults wait until the motors are off")
        if not o["active"]:
            self.dt = 1.0 / self.P("ctl.rate_hz")   # ctl.rate_hz applies with the motors off

        if now - self.rate_t0 >= 0.5:
            self.loop_hz = (self.step_n - self.rate_n0) / (now - self.rate_t0)
            self.rate_t0, self.rate_n0 = now, self.step_n

        # telemetry at tlm.hz (one snapshot of this step)
        period = 1.0 / max(1.0, self.P("tlm.hz"))
        if now - self.last_tlm >= period - 1e-9:
            self.last_tlm = now
            self.out.append(self._telemetry(o, pitch, pitch_rate, yaw_rate, w, batt_v))

        if self.remote and now >= self.radio_log_at:
            self.radio_log_at = now + 10.0
            self.log("radio: 50.0 pkt/s, gaps>100ms 0 (max 32 ms), bad 0, multi 0, ack fail 0, poll max 61 us")

        self._range_step(now, pitch)

        # nav status: at once on a change (state, index, count), 2 Hz in GOTO / EXPLORE / HOLD
        now_ms = int(now * 1000.0)
        if self.nav.status_due(now_ms):
            self.out.append(lp.encode_nav_status(self.nav.status()))
            self.nav.status_sent(now_ms)

    # ---------------------------------------------------------------- wheels-up motor test (src/motor_test.cpp)
    def _motor_test_step(self, now, dt, pitch, pitch_rate, stand_pose):
        """Sim-only stand: mot.test set in IDLE puts the robot upright on a stand with its wheels in the air;
        arming into a driving mode takes it off. The test itself follows the firmware (interlocks, lines)."""
        req = int(round(self.P("mot.test")))
        if req and self.mode == lp.MODE_IDLE and not self.on_stand:
            self.on_stand = True
            self.theta = self.theta_rate = 0.0
            self.tt = [TTWheel(0.82, v_start=6.12, v_stop=5.66), TTWheel(1.0)]   # left 18% weaker, stiffer
            self.enc_ring = []
            self.log("sim: robot on the stand, wheels in the air")
        elif self.on_stand and self.mode != lp.MODE_IDLE:
            self.on_stand = False
            self.vl = self.vr = 0.0
            self.log("sim: robot off the stand")
        if self.on_stand:
            if stand_pose:
                self.X, self.Y, self.TH = stand_pose
            self.theta = self.theta_rate = 0.0
            vb = self.battery - 0.3 * (abs(self.mtest.duty[0]) + abs(self.mtest.duty[1])) / (2 * MOTOR_FULL)
            for wh, duty in zip(self.tt, self.mtest.duty):
                wh.step(duty / MOTOR_FULL * vb, dt)
            enc = [wh.counts() for wh in self.tt]
            self.enc_ring.append((now, enc))
            while self.enc_ring and now - self.enc_ring[0][0] > 0.08 + 1e-9:
                self.enc_ring.pop(0)
            t0, e0 = self.enc_ring[0]
            sp = [(e - c) * MM_PER_COUNT / (now - t0) if now > t0 else 0.0 for e, c in zip(enc, e0)]
            self.vl, self.vr = self.tt[0].v, self.tt[1].v
        else:
            enc, sp, vb = [0, 0], [self.vl, self.vr], self.battery
        self.mtest.step({
            "now": now, "request": req, "armed": self.armed, "estop": False, "idle": self.mode == lp.MODE_IDLE,
            "radio_live": self.radio_live(), "imu_ok": True, "enc_ok": True, "pitch": pitch,
            "roll": self.rng.gauss(0, 0.2), "pitch_rate": pitch_rate, "enc": enc, "enc_t": now, "speed": sp,
            "vbatt": vb, "max_duty": self.P("mot.test_max"), "pwm_hz": self.P("mot.pwm_hz"),
            "drive_mode": int(self.P("mot.drive_mode"))})
        if self.mtest.done:
            self.mtest.done = False
            pid = self.pidx["mot.test"]
            self.params[pid][1] = 0.0
            self.out += self.param_frames([pid])
        if self.on_stand:
            self.motor_l, self.motor_r = self.mtest.duty

    def _wheel_turn_rate(self):
        """Turn rate from the wheels (firmware: (right - left) / odo.track_mm), deg/s."""
        if not self.hw:
            return math.degrees((self.vr - self.vl) / TRACK_MM)
        sl, sr = self.wheel_scale()
        return math.degrees((self.vr * sr - self.vl * sl) / self.P("odo.track_mm"))

    def _telemetry(self, o, pitch, pitch_rate, yaw_rate, w, batt_v):
        rng = self.rng
        x, y, th = self.odom()
        status = 0
        if self.armed:
            status |= lp.TLM_ARMED
        if o["upright"]:
            status |= lp.TLM_UPRIGHT
        if not self.radio_live():
            status |= lp.TLM_FAILSAFE
        if self.pi_link:
            status |= lp.TLM_PI_LINK
        if self.nav.obst_flag:
            status |= lp.TLM_OBSTACLE
        if not self.bench and batt_v / CELLS < self.P("batt.low_cell"):
            status |= lp.TLM_LOW_BATT
        if self.mode in (lp.MODE_GOTO, lp.MODE_EXPLORE):
            if self.nav.active():
                status |= lp.TLM_NAV_ACTIVE
            if self.nav.state == NAV_ARRIVED:
                status |= lp.TLM_NAV_DONE
        return lp.encode_telemetry({
            "t_us": self.t_us(), "mode": self.mode, "status": status, "error_code": self.fault,
            "radio_ok": 1 if self.radio_live() else 0,
            "pitch_deg": pitch, "pitch_rate_dps": pitch_rate, "roll_deg": rng.gauss(0, 0.2),
            "heading_deg": th, "yaw_rate_dps": yaw_rate,
            "x_mm": x, "y_mm": y, "speed_mms": self.speed_f, "turn_rate_dps": self._wheel_turn_rate() if self.hw else w,
            "cmd_speed_mms": o["speed_sp"], "cmd_turn_dps": o["turn_sp"],
            "wheel_left_mms": self.vl * self.wheel_scale()[0] + rng.gauss(0, 3),
            "wheel_right_mms": self.vr * self.wheel_scale()[1] + rng.gauss(0, 3),
            "balance_target_deg": o["balance_target"],
            "motor_left": self.motor_l / MOTOR_FULL, "motor_right": self.motor_r / MOTOR_FULL,
            "battery_v": 0.0 if self.bench else batt_v + rng.gauss(0, 0.01), "loop_hz": self.loop_hz,
            "bal_p": o["bal_p"], "bal_i": o["bal_i"], "bal_d": o["bal_d"], "steer_out": o["steer"],
            "vel_out_deg": o["vel_out"], "step": self.step_n & 0xFFFFFFFF,
            "remote_flags": (lp.LINK_RF_PRESENT if self.radio_live() else 0) |
                            (lp.LINK_RF_LINK_ESTOP if self.link_estop else 0),
            "nav_state": self.nav.state, "nav_index": min(255, self.nav.idx), "reserved2": 0,
        })

    # ---------------------------------------------------------------- range sweep (src/range_sweep.cpp)
    def _rs_steps(self):
        return max(1, min(64, int(round(self.P("rng.steps")))))

    def _rs_step_deg(self):
        w, n = self.P("rng.sweep_deg"), self._rs_steps()
        if w >= 179.5:
            return w / n          # full circle with two sensors: no repeated end point
        return w / (n - 1) if n > 1 else 180.0

    def _rs_position(self, k):
        w, n = self.P("rng.sweep_deg"), self._rs_steps()
        if n == 1 or w <= 0:
            return 0.0
        return -0.5 * w + k * self._rs_step_deg()

    def _rs_next_position(self):
        n = self._rs_steps()
        if self.rs_k >= n:
            self.rs_k = n - 1
        if n == 1:
            self.rs_k = 0
            return
        if self.rs_k >= n - 1:
            self.rs_dir = -1
        elif self.rs_k <= 0:
            self.rs_dir = 1
        self.rs_k += self.rs_dir

    def _rs_command(self, head):
        """commandServo(): servo command from the head angle; returns the settle time, s."""
        P = self.P
        trim, scale = P("rng.servo_trim_deg"), P("rng.servo_deg_per_cmd")
        sdir = -1 if P("rng.servo_dir") < 0 else 1
        cmd = max(P("rng.servo_min_deg"), min(P("rng.servo_max_deg"), 90.0 + trim + sdir * head / scale))
        self.rs_head = sdir * scale * (cmd - 90.0 - trim)          # reported (true if the firmware is set up right)
        self.rs_true_head = (cmd - 90.0) + self.head_error_deg      # the sim's servo: +1 deg per command deg
        moved = 1e9 if self.rs_last_cmd is None else abs(cmd - self.rs_last_cmd)
        self.rs_last_cmd = cmd
        if moved < 0.01:
            return 0.0
        step = 11.25 if P("rng.servo_hold_deg") >= 0 else self._rs_step_deg()
        f = max(1.0, moved / max(step, 0.5))
        return min(1.5, P("rng.settle_ms") / 1000.0 * f)

    def _range_step(self, now, pitch):
        if self.P("rng.enable") < 0.5:
            if self.rs_on:
                self.rs_on = self.rs_measuring = False
                self.rs_last_cmd = None       # no pulses: the servo goes limp
                self.log("rng: off")
            return
        if not self.rs_on:
            self.log("rng: on" if self.rs_started else "rng: started, sensor0 ok, sensor1 ok")
            self.rs_on, self.rs_started, self.rs_measuring, self.rs_next = True, True, False, now
        if now < self.rs_next:
            return
        if self.rs_measuring:                 # both sensors finish together (pose at that moment)
            x, y, th = self.odom()
            for sid in (0, 1):
                f = self._range(sid, x, y, th, pitch)
                self.out.append(lp.encode_range(f))
                self.nav.add_range(f, int(now * 1000.0))   # RangeSweep::takeNavReading -> Navigator
            self.rs_measuring = False
            self._rs_next_position()
        hold = self.P("rng.servo_hold_deg")
        if hold >= 0:
            sdir = -1 if self.P("rng.servo_dir") < 0 else 1
            head = sdir * self.P("rng.servo_deg_per_cmd") * (hold - 90.0)
        else:
            head = self._rs_position(self.rs_k)
        settle = self._rs_command(head)
        self.rs_next = now + settle + self.P("rng.timing_budget_ms") / 1000.0 + RANGE_IO_S
        self.rs_measuring = True

    def _range(self, sid, x, y, th, pitch):
        h = math.radians(self.TH)
        d = self.room.raycast(self.X + SENSOR_FWD_MM * math.cos(h), self.Y + SENSOR_FWD_MM * math.sin(h),
                              math.radians(self.TH + self.rs_true_head + 180.0 * sid))
        reach = RANGE_MAX if self.P("rng.long_range") >= 0.5 else RANGE_MAX_SHORT
        valid, r = 0, 8190                    # no target (what the VL53L0X reports then)
        if d is not None and d - SENSOR_RADIUS_MM <= reach and self.rng.random() >= RANGE_DROPOUT:
            d -= SENSOR_RADIUS_MM
            noise = (8 + 0.01 * d) * math.sqrt(33.0 / self.P("rng.timing_budget_ms"))
            r = int(max(30, d + self.rng.gauss(0, noise)))
            valid = 1 if r <= self.P("rng.max_mm") else 0
        return {"t_us": self.t_us(), "x_mm": x, "y_mm": y, "heading_deg": th,
                "pitch_deg": pitch, "sensor_angle_deg": _wrap180(self.rs_head + 180.0 * sid),
                "range_mm": r, "sensor_id": sid, "valid": valid}

    def run_until(self, t, max_steps=2000):
        """Advance in balance steps up to simulated time t (catches up after a late wake-up)."""
        n = 0
        while self.t + self.dt <= t and n < max_steps:
            self.step()
            n += 1
        return n


BOOT_TEXT = b"\r\nTeensy 4.1 sim: plain-text boot output before the first frame\r\n"


class SimServer:
    def __init__(self, robot, host="127.0.0.1", port=5761, garbage=0.0, quiet=False):
        self.robot = robot
        self.host, self.port = host, port
        self.garbage = garbage
        self.quiet = quiet
        self.writer = None

    async def _client(self, reader, writer):
        if self.writer:
            try:
                self.writer.close()
            except Exception:
                pass
        writer.write(BOOT_TEXT)          # the real Teensy prints text while booting: the parser must resync
        self.writer = writer
        if not self.quiet:
            print("[sim] bridge connected", writer.get_extra_info("peername"), flush=True)
        self.robot.out.append(self.robot.hello())   # firmware: HELLO when the host opens the port
        parser = lp.FrameParser()
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                for t, p in parser.feed(data):
                    try:
                        self.robot.on_frame(t, p)
                    except (ValueError, KeyError) as e:
                        print("[sim] bad frame", t, e)
        except (ConnectionError, OSError):
            pass
        if self.writer is writer:
            self.writer = None
        if not self.quiet:
            print("[sim] bridge disconnected", flush=True)

    async def physics(self):
        """Balance steps at ctl.rate_hz in simulated time, caught up to the wall clock every few ms."""
        robot = self.robot
        base = time.monotonic() - robot.t
        while True:
            await asyncio.sleep(0.004)
            target = time.monotonic() - base
            if target - robot.t > 0.5:          # fell far behind (PC asleep): skip ahead, don't spin
                base = time.monotonic() - robot.t
                continue
            robot.run_until(target)
            out, robot.out = robot.out, []
            w = self.writer
            if w and out:
                data = b"".join(out)
                if self.garbage and robot.rng.random() < self.garbage * len(out):
                    i = robot.rng.randrange(len(data))
                    data = data[:i] + bytes(robot.rng.randrange(256) for _ in range(5)) + data[i:]
                try:
                    w.write(data)
                except Exception:
                    self.writer = None

    async def run(self):
        srv = await asyncio.start_server(self._client, self.host, self.port)
        if not self.quiet:
            print("[sim] fake Teensy on tcp://%s:%d" % (self.host, self.port), flush=True)
        async with srv:
            await self.physics()
