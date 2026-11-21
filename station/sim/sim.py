"""Fake Teensy (firmware v4): speaks the real binary link protocol over TCP so the bridge and UI can be
developed on a PC.

- Param table, arming rules, cfg.save / cfg.defaults and telemetry follow the v4 firmware
  (src/params.h, src/mode_manager.cpp, src/main.cpp).
- The balance / heading / velocity loops are a line-by-line port of src/controller.cpp, running at
  ctl.rate_hz (400) on a wheeled inverted pendulum with TT-style gear motors (dead zone, lag), so the
  bal_p / bal_i / bal_d telemetry and the effect of a gain change look like the real robot.
- A virtual room with walls/boxes and the servo-swept twin VL53L0X range sensor.
- GOTO / EXPLORE / HOLD drive (sim-only preview of issue #8; firmware v4 stands still in those modes).
"""
import asyncio
import math
import random
import time

from station.bridge import link_protocol as lp

# Geometry and mass from the Fusion CAD (cad/robot_model.json on master)
TRACK_MM = 126.8          # wheel separation (firmware odometry param says 123)
ROBOT_RADIUS = 90.0       # collision radius: footprint ~152 x 90 mm
SENSOR_FWD_MM = 26.0      # range-sensor servo axis is this far forward of the axle
RANGE_MAX = 2000.0
SERVO_STEPS = 16
SERVO_STEP_DEG = 180.0 / SERVO_STEPS   # 11.25 deg: 16 steps x 2 sensors = 32 directions
SERVO_STEP_S = 0.060
ARRIVE_MM = 80.0          # sim-only navigation constants
OBSTACLE_MM = 300.0

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
    ("tlm.hz", 100.0, 1.0, 1000.0), ("cfg.save", 0.0, 0.0, 1.0), ("cfg.defaults", 0.0, 0.0, 1.0),
]
CELLS = 3

NAV_IDLE, NAV_TURNING, NAV_DRIVING, NAV_BLOCKED, NAV_ARRIVED = range(5)


def _box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


class Room:
    """Walls as line segments (world mm). The robot starts at (0, 0) facing +X."""

    def __init__(self):
        s = []
        s += _box(1500, 250, 5000, 3500)          # outer walls: x -1000..4000, y -1500..2000
        s += _box(1500, 900, 600, 400)            # box
        s += _box(2900, -800, 450, 450)           # box
        s += _box(700, -1300, 1000, 400)          # couch against the wall
        s.append((2300, 2000, 2300, 1100))        # partition wall
        s.append((-1000, 1200, -300, 2000))       # angled corner cabinet
        self.segs = s

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
    return (a + 180.0) % 360.0 - 180.0


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
    def __init__(self, remote=False, remote_armed=True, seed=None):
        self.rng = random.Random(seed)
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
        self.last_hb = -10.0
        self.pi_link = False
        self.cmd = (0.0, 0.0)
        self.cmd_time = -10.0
        self.cmd_valid = False
        self.waypoints = []           # odom frame
        self.nav_index = 0
        self.nav_state = NAV_IDLE
        self.nav_sent = None
        self.nav_last_send = 0.0
        self.hold = (0.0, 0.0)
        self.obstacle = False
        self.front_cache = (-1.0, 1e9)
        self.scan_until = 0.0
        self.explore_turn = 0.0
        self.servo_i = 0
        self.servo_dir = 1
        self.next_servo = 0.0
        self.motor_l = self.motor_r = 0.0
        self.battery = 12.45
        self.push = 0.0               # sim-only disturbance (rad/s^2), see push()
        self.out = []                 # frames to send
        self.log("boot sim v4, params DEFAULTS, imu ok, radio %s" % ("ok" if remote else "ok (no remote)"))

    # ---------------------------------------------------------------- helpers
    def P(self, name):
        return self.params[self.pidx[name]][1]

    def now(self):
        return self.t

    def t_us(self):
        return int(self.t * 1e6) & 0xFFFFFFFF

    def odom(self):
        """Odometry pose (what the robot reports): world pose relative to the last pose reset."""
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
            if not self.pi_link:
                self.pi_link = True
                self.log("pi link up (%d clients)" % d["clients"])
        elif t == lp.MSG_CMD_VEL:
            if all(math.isfinite(v) for v in (d["speed_mms"], d["turn_dps"])):
                self.cmd = (d["speed_mms"], d["turn_dps"])
                self.cmd_time = self.now()
                self.cmd_valid = True
        elif t == lp.MSG_SET_MODE:
            self.set_mode(d["mode"])
        elif t == lp.MSG_SET_PARAM:
            self.set_param(d["id"], d["value"])
        elif t == lp.MSG_GET_PARAMS:
            self.out.append(self.hello())
            self.out += self.param_frames()
        elif t == lp.MSG_GOTO:
            if d["replace"]:
                self.waypoints = []
                self.nav_index = 0
            self.waypoints += [tuple(pt) for pt in d["points"]]
            self.log("waypoints: %d" % len(self.waypoints))
            if self.nav_state == NAV_ARRIVED and self.nav_index < len(self.waypoints):
                self.nav_state = NAV_IDLE
        elif t == lp.MSG_EVENT:
            self.on_event(d["event"])

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
                for p, (_n, dflt, _lo, _hi) in zip(self.params, PARAMS):
                    p[1] = dflt
                self.out += self.param_frames()   # firmware re-streams every param
                self.log("params: defaults restored (not saved)")
            v = 0.0
        pr[1] = v
        self.out += self.param_frames([pid])     # echo = ack

    def on_event(self, ev):
        x, y, th = self.odom()
        if ev == lp.EVT_MARK_WAYPOINT:
            self.waypoints.append((x, y))
            self.log("marked waypoint %d at %.0f,%.0f" % (len(self.waypoints), x, y))
        elif ev == lp.EVT_CLEAR_WAYPOINTS:
            self.waypoints = []
            self.nav_index = 0
            self.log("waypoints cleared")
        elif ev == lp.EVT_GO_HOME:
            home = self.waypoints[0] if self.waypoints else (0.0, 0.0)
            self.waypoints = [home]
            self.nav_index = 0
            self.nav_state = NAV_IDLE
            self.set_mode(lp.MODE_GOTO)
            self.log("going home")
        elif ev == lp.EVT_RESET_POSE:
            self.ox, self.oy, self.oth = self.X, self.Y, self.TH
            self.waypoints = []
            self.nav_index = 0
            self.ctl.heading_sp -= self.TH - self.oth   # keep the heading target where it physically was
            self.log("pose reset")
        elif ev == lp.EVT_START_SCAN:
            self.scan_until = self.now() + 2 * SERVO_STEPS * SERVO_STEP_S
            self.log("scan")

    def set_mode(self, mode):
        """ModeManager::onPiSetMode."""
        if not 0 <= mode < lp.MODE_COUNT:
            return
        self.requested = mode
        if not self.radio_live():
            self.armed = mode != lp.MODE_IDLE

    def _update_mode(self):
        eff = self.requested if self.armed else lp.MODE_IDLE
        if eff == self.mode:
            return
        if eff != lp.MODE_IDLE and self.mode == lp.MODE_IDLE:
            self.log("armed")
        elif eff == lp.MODE_IDLE:
            self.log("disarmed")
        self.mode = eff
        self.cmd_valid = False
        self.ctl.on_mode_change()
        if eff == lp.MODE_HOLD:
            x, y, _ = self.odom()
            self.hold = (x, y)
        if eff == lp.MODE_GOTO:
            self.nav_state = NAV_IDLE
            if self.nav_index >= len(self.waypoints):
                self.nav_index = 0
        self.log("mode %s" % lp.MODE_NAMES[eff])

    # ---------------------------------------------------------------- targets (ModeManager::update + sim nav)
    def _front_clear(self):
        now = self.now()
        if now - self.front_cache[0] > 0.02:
            d = self.room.raycast(self.X, self.Y, math.radians(self.TH))
            self.front_cache = (now, d - ROBOT_RADIUS if d is not None else 1e9)
        return self.front_cache[1]

    def _targets(self, now):
        max_v, max_w = self.P("drv.max_speed"), self.P("drv.max_turn")
        m = self.mode
        self.obstacle = False
        v_t = w_t = 0.0
        if m == lp.MODE_IDLE or not self.ctl.upright:
            return 0.0, 0.0
        if m == lp.MODE_STATION:
            if self.pi_link and self.cmd_valid and now - self.cmd_time <= lp.LINK_CMD_VEL_TIMEOUT_MS / 1000.0:
                v_t = _clamp(self.cmd[0], max_v)
                w_t = _clamp(self.cmd[1], max_w)
        elif m == lp.MODE_GOTO:
            v_t, w_t = self._nav(max_v, max_w)
        elif m == lp.MODE_EXPLORE:
            front = self._front_clear()
            if front < OBSTACLE_MM + 150:
                if self.explore_turn == 0.0:
                    left = self.room.raycast(self.X, self.Y, math.radians(self.TH + 60)) or 1e9
                    right = self.room.raycast(self.X, self.Y, math.radians(self.TH - 60)) or 1e9
                    self.explore_turn = 1.0 if left > right else -1.0
                v_t, w_t = 0.0, 0.6 * max_w * self.explore_turn
            else:
                self.explore_turn = 0.0
                v_t = 0.5 * max_v
                w_t = 25.0 * math.sin(now * 0.4)
            self.nav_state = NAV_DRIVING if v_t else NAV_TURNING
        elif m == lp.MODE_HOLD:
            x, y, th = self.odom()
            ex, ey = self.hold[0] - x, self.hold[1] - y
            fwd = ex * math.cos(math.radians(th)) + ey * math.sin(math.radians(th))
            v_t = max(-200.0, min(200.0, 2.0 * fwd))
        if now < self.scan_until:
            v_t = w_t = 0.0
        if v_t > 0 and self._front_clear() < OBSTACLE_MM:   # obstacle stop (forward motion only)
            v_t = 0.0
            self.obstacle = True
            if m == lp.MODE_GOTO:
                self.nav_state = NAV_BLOCKED
        if abs(w_t) < self.P("drv.turn_deadband"):
            w_t = 0.0
        return v_t, w_t

    def _nav(self, max_v, max_w):
        if self.nav_index >= len(self.waypoints):
            self.nav_state = NAV_ARRIVED if self.waypoints else NAV_IDLE
            return 0.0, 0.0
        x, y, th = self.odom()
        tx, ty = self.waypoints[self.nav_index]
        dist = math.hypot(tx - x, ty - y)
        if dist < ARRIVE_MM:
            self.nav_index += 1
            self.log("reached waypoint %d" % self.nav_index)
            if self.nav_index >= len(self.waypoints):
                self.nav_state = NAV_ARRIVED
                self.log("arrived")
            return 0.0, 0.0
        bearing = _wrap180(math.degrees(math.atan2(ty - y, tx - x)) - th)
        w_t = max(-max_w, min(max_w, 3.0 * bearing))
        if abs(bearing) > 30:
            self.nav_state = NAV_TURNING
            return 0.0, w_t
        self.nav_state = NAV_DRIVING
        v_t = min(max_v, 1.2 * dist + 60.0) * math.cos(math.radians(bearing))
        return v_t, w_t

    # ---------------------------------------------------------------- one balance step
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
        if self.mode not in (lp.MODE_GOTO, lp.MODE_EXPLORE):
            self.nav_state = NAV_IDLE

        # sensors (what the firmware sees)
        pitch = math.degrees(self.theta) + IMU_MOUNT_DEG + rng.gauss(0, 0.04)
        pitch_rate = math.degrees(self.theta_rate) + rng.gauss(0, 0.6)
        yaw_rate = math.degrees((self.vr - self.vl) / TRACK_MM)
        ctl_in = {"dt": dt, "now": now, "pitch": pitch, "pitch_rate": pitch_rate, "roll": rng.gauss(0, 0.2),
                  "heading": self.TH - self.oth, "yaw_rate": yaw_rate + rng.gauss(0, 0.3),
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
        if o["active"]:
            tl = self._wheel_target(self.motor_l, LEFT_STRONGER)
            tr = self._wheel_target(self.motor_r, 1.0)
            k = min(1.0, dt / MOTOR_TAU)
        else:   # coasting / lying on the floor
            tl = tr = 0.0
            k = min(1.0, dt / (0.08 if abs(self.theta) >= math.radians(LIE_DEG) - 1e-6 else 0.4))
        self.vl += (tl - self.vl) * k
        self.vr += (tr - self.vr) * k
        v = (self.vl + self.vr) / 2
        acc = (v - self.v_prev) / dt / 1000.0    # m/s^2
        self.v_prev = v
        self.speed_f += (v - self.speed_f) * min(1.0, dt / 0.04)

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

        # move (stop at walls)
        w = math.degrees((self.vr - self.vl) / TRACK_MM)
        self.TH += w * dt
        nx = self.X + v * math.cos(math.radians(self.TH)) * dt
        ny = self.Y + v * math.sin(math.radians(self.TH)) * dt
        if abs(v) < 1e-3 or self.room.clearance(nx, ny) > ROBOT_RADIUS:
            self.X, self.Y = nx, ny
        else:
            self.vl = self.vr = self.v_prev = 0.0

        load = (abs(self.motor_l) + abs(self.motor_r)) / (2 * MOTOR_FULL)
        self.battery -= dt * 0.00002 * (1 + 10 * load)
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

        # range sensor: servo steps every 60 ms, both sensors read at each step
        if now >= self.next_servo:
            x, y, th = self.odom()
            self.next_servo = now + SERVO_STEP_S
            ang = -90.0 + self.servo_i * SERVO_STEP_DEG
            for sid, a in ((0, ang), (1, ang + 180.0)):
                self.out.append(self._range(sid, a, x, y, th, pitch))
            self.servo_i += self.servo_dir
            if self.servo_i in (0, SERVO_STEPS - 1):
                self.servo_dir = -self.servo_dir

        # nav status: on change and at 2 Hz while navigating
        key = (self.nav_state, len(self.waypoints), self.nav_index)
        navigating = self.mode in (lp.MODE_GOTO, lp.MODE_EXPLORE)
        if key != self.nav_sent or (navigating and now - self.nav_last_send > 0.5):
            x, y, th = self.odom()
            self.nav_sent = key
            self.nav_last_send = now
            if self.nav_index < len(self.waypoints):
                tx, ty = self.waypoints[self.nav_index]
            else:
                tx, ty = (self.waypoints[-1] if self.waypoints else (0.0, 0.0))
            self.out.append(lp.encode_nav_status({
                "state": self.nav_state, "waypoint_count": min(255, len(self.waypoints)),
                "nav_index": min(255, self.nav_index), "target_x_mm": tx, "target_y_mm": ty,
                "distance_mm": math.hypot(tx - x, ty - y)}))

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
        if self.obstacle:
            status |= lp.TLM_OBSTACLE
        if batt_v / CELLS < self.P("batt.low_cell"):
            status |= lp.TLM_LOW_BATT
        if self.mode in (lp.MODE_GOTO, lp.MODE_EXPLORE) and self.nav_state not in (NAV_ARRIVED, NAV_IDLE):
            status |= lp.TLM_NAV_ACTIVE
        if self.nav_state == NAV_ARRIVED:
            status |= lp.TLM_NAV_DONE
        return lp.encode_telemetry({
            "t_us": self.t_us(), "mode": self.mode, "status": status, "error_code": 0,
            "radio_ok": 1 if self.radio_live() else 0,
            "pitch_deg": pitch, "pitch_rate_dps": pitch_rate, "roll_deg": rng.gauss(0, 0.2),
            "heading_deg": th, "yaw_rate_dps": yaw_rate,
            "x_mm": x, "y_mm": y, "speed_mms": self.speed_f, "turn_rate_dps": w,
            "cmd_speed_mms": o["speed_sp"], "cmd_turn_dps": o["turn_sp"],
            "wheel_left_mms": self.vl + rng.gauss(0, 3), "wheel_right_mms": self.vr + rng.gauss(0, 3),
            "balance_target_deg": o["balance_target"],
            "motor_left": self.motor_l / MOTOR_FULL, "motor_right": self.motor_r / MOTOR_FULL,
            "battery_v": batt_v + rng.gauss(0, 0.01), "loop_hz": self.loop_hz,
            "bal_p": o["bal_p"], "bal_i": o["bal_i"], "bal_d": o["bal_d"], "steer_out": o["steer"],
            "vel_out_deg": o["vel_out"], "step": self.step_n & 0xFFFFFFFF,
        })

    def _range(self, sid, sensor_ang, x, y, th, pitch):
        h = math.radians(self.TH)
        d = self.room.raycast(self.X + SENSOR_FWD_MM * math.cos(h), self.Y + SENSOR_FWD_MM * math.sin(h),
                              math.radians(self.TH + sensor_ang))
        valid = 1
        if d is None or d > RANGE_MAX or self.rng.random() < 0.03:
            valid, r = 0, 8190
        else:
            r = int(max(30, d + self.rng.gauss(0, 8 + 0.01 * d)))
        return lp.encode_range({"t_us": self.t_us(), "x_mm": x, "y_mm": y, "heading_deg": th,
                                "pitch_deg": pitch, "sensor_angle_deg": sensor_ang,
                                "range_mm": r, "sensor_id": sid, "valid": valid})

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
