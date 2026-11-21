"""Fake Teensy: speaks the real binary link protocol over TCP so the bridge can be developed on a PC.

Unicycle robot + a little pitch wobble, RobotMode/safety behaviour from the headers, params,
a virtual room with walls/boxes and the servo-swept twin VL53L0X range sensor.
"""
import asyncio
import math
import random
import time

from station.bridge import link_protocol as lp

TRACK_MM = 200.0          # wheel separation
ROBOT_RADIUS = 140.0
RANGE_MAX = 2000.0
SERVO_STEPS = 16
SERVO_STEP_DEG = 180.0 / SERVO_STEPS   # 11.25 deg: 16 steps x 2 sensors = 32 directions
SERVO_STEP_S = 0.060
REST_PITCH = 18.0         # leaning on the stand when the motors are off
ARRIVE_MM = 80.0          # sim-only navigation constants
OBSTACLE_MM = 300.0

# (name, default, min, max): a copy of the v4 firmware table (src/params.h) so the UI sees the same
# set. The UI never hardcodes these; it builds everything from ParamInfoFrames.
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
SIM_RATE_HZ = 400         # physics steps per second (telemetry is sent at tlm.hz)

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


class SimRobot:
    def __init__(self, remote=False, remote_armed=True, seed=None):
        self.rng = random.Random(seed)
        self.room = Room()
        self.t0 = time.monotonic()
        self.params = [[n, v, lo, hi] for n, v, lo, hi in PARAMS]
        self.pidx = {p[0]: i for i, p in enumerate(self.params)}
        # Arming (as in the v4 firmware): with the RC remote present its ARMED switch rules;
        # without it, SET_MODE to a non-IDLE mode arms and SET_MODE IDLE disarms.
        self.radio_ok = remote
        self.remote_armed = remote_armed
        self.armed = remote and remote_armed
        self.requested = lp.MODE_IDLE
        self.save_pending = False
        self.step_n = 0
        self.tlm_acc = 0.0
        self.bal_i = 0.0
        self.vel_out = 0.0
        # true world pose
        self.X = self.Y = 0.0
        self.TH = 0.0                 # deg, continuous
        self.v = self.w = 0.0         # mm/s, deg/s
        self.pitch = REST_PITCH
        self.pitch_rate = 0.0
        self.upright = False
        self.bal_target = 0.0
        self.v_t = self.w_t = 0.0
        # odometry frame (reset with EVT_RESET_POSE)
        self.ox = self.oy = 0.0
        self.oth = 0.0
        self.mode = lp.MODE_IDLE
        self.last_hb = -10.0
        self.pi_link = False
        self.cmd = (0.0, 0.0)
        self.cmd_time = -10.0
        self.waypoints = []           # odom frame
        self.nav_index = 0
        self.nav_state = NAV_IDLE
        self.nav_sent = None
        self.nav_last_send = 0.0
        self.hold = (0.0, 0.0)
        self.obstacle = False
        self.scan_until = 0.0
        self.explore_turn = 0.0
        self.servo_i = 0
        self.servo_dir = 1
        self.next_servo = 0.0
        self.motor_l = self.motor_r = 0.0
        self.battery = 12.45
        self.out = []                 # frames to send

    # ---------------------------------------------------------------- helpers
    def P(self, name):
        return self.params[self.pidx[name]][1]

    def now(self):
        return time.monotonic() - self.t0

    def t_us(self):
        return int(self.now() * 1e6) & 0xFFFFFFFF

    def odom(self):
        """Odometry pose (what the robot reports): world pose relative to the last pose reset."""
        dx, dy = self.X - self.ox, self.Y - self.oy
        c, s = math.cos(math.radians(-self.oth)), math.sin(math.radians(-self.oth))
        return dx * c - dy * s, dx * s + dy * c, self.TH - self.oth

    def log(self, text):
        self.out.append(lp.encode_log(text))

    def hello(self):
        return lp.encode_hello(param_count=len(self.params), firmware="sim 2026-11-20")

    # ---------------------------------------------------------------- link in
    def on_frame(self, t, p):
        d = lp.decode(t, p)
        if t == lp.MSG_HEARTBEAT:
            self.last_hb = self.now()
            if not self.pi_link:
                self.pi_link = True
                self.log("pi link up (%d clients)" % d["clients"])
        elif t == lp.MSG_CMD_VEL:
            self.cmd = (d["speed_mms"], d["turn_dps"])
            self.cmd_time = self.now()
        elif t == lp.MSG_SET_MODE:
            self.set_mode(d["mode"])
        elif t == lp.MSG_SET_PARAM:
            if 0 <= d["id"] < len(self.params):
                pr = self.params[d["id"]]
                v = max(pr[2], min(pr[3], d["value"]))
                if pr[0] == "cfg.save" and v >= 0.5:
                    self.save_pending = True      # done in step() once the motors are off
                    v = 0.0
                elif pr[0] == "cfg.defaults" and v >= 0.5:
                    for p, (_n, dflt, _lo, _hi) in zip(self.params, PARAMS):
                        p[1] = dflt
                    self.log("params: defaults restored (not saved)")
                    v = 0.0
                pr[1] = v
                self.out.append(lp.encode_param_info(d["id"], pr[1], pr[2], pr[3], pr[0]))  # echo = ack
        elif t == lp.MSG_GET_PARAMS:
            self.out.append(self.hello())
            for i, (n, v, lo, hi) in enumerate(self.params):
                self.out.append(lp.encode_param_info(i, v, lo, hi, n))
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
            self.log("pose reset")
        elif ev == lp.EVT_START_SCAN:
            self.scan_until = self.now() + 2 * SERVO_STEPS * SERVO_STEP_S
            self.log("scan")

    def set_mode(self, mode):
        if not 0 <= mode < lp.MODE_COUNT:
            return
        self.requested = mode
        if not self.radio_ok:
            self.armed = mode != lp.MODE_IDLE
        self._apply_mode()

    def _apply_mode(self):
        mode = self.requested if self.armed else lp.MODE_IDLE
        if mode == self.mode:
            return
        if mode != lp.MODE_IDLE and self.mode == lp.MODE_IDLE:
            self.log("armed")
        self.mode = mode
        if mode == lp.MODE_HOLD:
            x, y, _ = self.odom()
            self.hold = (x, y)
        if mode == lp.MODE_GOTO:
            self.nav_state = NAV_IDLE
            if self.nav_index >= len(self.waypoints):
                self.nav_index = 0
        self.log("mode %s" % lp.MODE_NAMES[mode])

    # ---------------------------------------------------------------- control
    def _front_clear(self):
        d = self.room.raycast(self.X, self.Y, math.radians(self.TH))
        return d - ROBOT_RADIUS if d is not None else 1e9

    def _targets(self, now):
        max_v, max_w = self.P("drive.max_speed"), self.P("drive.max_turn")
        m = self.mode
        self.obstacle = False
        v_t = w_t = 0.0
        if m == lp.MODE_IDLE or not self.upright:
            return 0.0, 0.0
        if m == lp.MODE_STATION:
            if self.pi_link and now - self.cmd_time < lp.LINK_CMD_VEL_TIMEOUT_MS / 1000.0:
                v_t = max(-max_v, min(max_v, self.cmd[0]))
                w_t = max(-max_w, min(max_w, self.cmd[1]))
        elif m == lp.MODE_GOTO:
            v_t, w_t = self._nav(max_v, max_w)
        elif m == lp.MODE_EXPLORE:
            front = self._front_clear()
            if front < self.P("nav.obstacle_mm") + 150:
                if self.explore_turn == 0.0:
                    left = self.room.raycast(self.X, self.Y, math.radians(self.TH + 60)) or 1e9
                    right = self.room.raycast(self.X, self.Y, math.radians(self.TH - 60)) or 1e9
                    self.explore_turn = 1.0 if left > right else -1.0
                v_t, w_t = 0.0, 0.6 * max_w * self.explore_turn
            else:
                self.explore_turn = 0.0
                v_t = 0.5 * max_v
                w_t = 25.0 * math.sin(now * 0.4) + self.rng.gauss(0, 3)
            self.nav_state = NAV_DRIVING if v_t else NAV_TURNING
        elif m == lp.MODE_HOLD:
            x, y, th = self.odom()
            ex, ey = self.hold[0] - x, self.hold[1] - y
            fwd = ex * math.cos(math.radians(th)) + ey * math.sin(math.radians(th))
            v_t = max(-200.0, min(200.0, 2.0 * fwd))
        if now < self.scan_until:
            v_t = w_t = 0.0
        # obstacle stop (forward motion only)
        if v_t > 0 and self._front_clear() < self.P("nav.obstacle_mm"):
            v_t = 0.0
            self.obstacle = True
            if m == lp.MODE_GOTO:
                self.nav_state = NAV_BLOCKED
        return v_t, w_t

    def _nav(self, max_v, max_w):
        if self.nav_index >= len(self.waypoints):
            if self.waypoints:
                self.nav_state = NAV_ARRIVED
            else:
                self.nav_state = NAV_IDLE
            return 0.0, 0.0
        x, y, th = self.odom()
        tx, ty = self.waypoints[self.nav_index]
        dist = math.hypot(tx - x, ty - y)
        if dist < self.P("nav.arrive_mm"):
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

    def step(self, dt):
        now = self.now()
        rng = self.rng
        if self.pi_link and now - self.last_hb > lp.LINK_HEARTBEAT_TIMEOUT_MS / 1000.0:
            self.pi_link = False
            self.log("pi link lost: station commands stopped")
        self.v_t, self.w_t = self._targets(now)
        if self.mode not in (lp.MODE_GOTO, lp.MODE_EXPLORE):
            self.nav_state = NAV_IDLE

        # speed / turn dynamics (acceleration limited, like the velocity loop)
        acc = max(-900.0, min(900.0, (self.v_t - self.v) * 5.0))
        if self.mode == lp.MODE_IDLE:
            acc = -self.v * 8.0
        self.v += acc * dt
        self.w += max(-720.0, min(720.0, (self.w_t - self.w) * 8.0)) * dt

        # pitch: lean into acceleration, under-damped response, a little wobble
        kp, kd = self.P("bal.kp"), self.P("bal.kd")
        if self.mode == lp.MODE_IDLE:
            target = REST_PITCH
            wn, zeta = 6.0, 1.0
        else:
            self.bal_target = max(-12.0, min(12.0, math.degrees(math.atan2(acc, 9810.0)) * 1.4 + self.v * 0.001))
            target = self.bal_target
            wn = 14.0 * math.sqrt(max(kp, 1.0) / 250.0)
            zeta = max(0.05, min(1.2, 0.35 * math.sqrt(max(kd, 0.01) / 4.63)))
        noise = rng.gauss(0, 150.0 if self.mode != lp.MODE_IDLE else 0.0)
        pacc = wn * wn * (target - self.pitch) - 2 * zeta * wn * self.pitch_rate + noise
        self.pitch_rate += pacc * dt
        self.pitch += self.pitch_rate * dt
        self.upright = self.mode != lp.MODE_IDLE and abs(self.pitch) < 8.0 or (self.upright and self.mode != lp.MODE_IDLE)

        # move (stop at walls)
        self.TH += self.w * dt
        nx = self.X + self.v * math.cos(math.radians(self.TH)) * dt
        ny = self.Y + self.v * math.sin(math.radians(self.TH)) * dt
        if self.room.clearance(nx, ny) > ROBOT_RADIUS:
            self.X, self.Y = nx, ny
        else:
            self.v = 0.0

        wl = self.v - math.radians(self.w) * TRACK_MM / 2
        wr = self.v + math.radians(self.w) * TRACK_MM / 2
        if self.mode == lp.MODE_IDLE:
            self.motor_l = self.motor_r = 0.0
        else:
            base = self.pitch * 0.03 + self.pitch_rate * 0.004
            self.motor_l = max(-1.0, min(1.0, wl / 900.0 + base))
            self.motor_r = max(-1.0, min(1.0, wr / 900.0 + base))
        self.battery -= dt * 0.00002 * (1 + 5 * (abs(self.motor_l) + abs(self.motor_r)))
        sag = 0.35 * (abs(self.motor_l) + abs(self.motor_r)) / 2

        x, y, th = self.odom()
        status = 0
        if self.armed:
            status |= lp.TLM_ARMED
        if self.upright:
            status |= lp.TLM_UPRIGHT
        if not self.radio_ok:
            status |= lp.TLM_FAILSAFE
        if self.pi_link:
            status |= lp.TLM_PI_LINK
        if self.obstacle:
            status |= lp.TLM_OBSTACLE
        if self.battery - sag < 11.1:
            status |= lp.TLM_LOW_BATT
        if self.mode in (lp.MODE_GOTO, lp.MODE_EXPLORE) and self.nav_state not in (NAV_ARRIVED, NAV_IDLE):
            status |= lp.TLM_NAV_ACTIVE
        if self.nav_state == NAV_ARRIVED:
            status |= lp.TLM_NAV_DONE
        self.out.append(lp.encode_telemetry({
            "t_us": self.t_us(), "mode": self.mode, "status": status, "error_code": 0,
            "radio_ok": 1 if self.radio_ok else 0,
            "pitch_deg": self.pitch, "pitch_rate_dps": self.pitch_rate, "roll_deg": rng.gauss(0, 0.2),
            "heading_deg": th, "yaw_rate_dps": self.w + rng.gauss(0, 0.5),
            "x_mm": x, "y_mm": y, "speed_mms": self.v + rng.gauss(0, 4), "turn_rate_dps": self.w + rng.gauss(0, 1),
            "cmd_speed_mms": self.v_t, "cmd_turn_dps": self.w_t,
            "wheel_left_mms": wl + rng.gauss(0, 4), "wheel_right_mms": wr + rng.gauss(0, 4),
            "balance_target_deg": self.bal_target if self.mode != lp.MODE_IDLE else 0.0,
            "motor_left": self.motor_l, "motor_right": self.motor_r,
            "battery_v": self.battery - sag + rng.gauss(0, 0.01), "loop_hz": 2000 + rng.gauss(0, 15),
        }))

        # range sensor: servo steps every 60 ms, both sensors read at each step
        if now >= self.next_servo:
            self.next_servo = now + SERVO_STEP_S
            ang = -90.0 + self.servo_i * SERVO_STEP_DEG
            for sid, a in ((0, ang), (1, ang + 180.0)):
                self.out.append(self._range(sid, a, x, y, th))
            self.servo_i += self.servo_dir
            if self.servo_i in (0, SERVO_STEPS - 1):
                self.servo_dir = -self.servo_dir

        # nav status: on change and at 2 Hz while navigating
        key = (self.nav_state, len(self.waypoints), self.nav_index)
        navigating = self.mode in (lp.MODE_GOTO, lp.MODE_EXPLORE)
        if key != self.nav_sent or (navigating and now - self.nav_last_send > 0.5):
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

    def _range(self, sid, sensor_ang, x, y, th):
        d = self.room.raycast(self.X, self.Y, math.radians(self.TH + sensor_ang))
        valid = 1
        if d is None or d > RANGE_MAX or self.rng.random() < 0.03:
            valid, r = 0, 8190
        else:
            r = int(max(30, d + self.rng.gauss(0, 8 + 0.01 * d)))
        return lp.encode_range({"t_us": self.t_us(), "x_mm": x, "y_mm": y, "heading_deg": th,
                                "pitch_deg": self.pitch, "sensor_angle_deg": sensor_ang,
                                "range_mm": r, "sensor_id": sid, "valid": valid})


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
        self.writer = writer
        if not self.quiet:
            print("[sim] bridge connected", writer.get_extra_info("peername"), flush=True)
        self.robot.out.append(self.robot.hello())
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
        dt = 1.0 / TLM_HZ
        nxt = time.monotonic()
        while True:
            nxt += dt
            delay = nxt - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            elif delay < -0.5:
                nxt = time.monotonic()   # fell far behind (PC asleep): don't spin
            self.robot.step(dt)
            out, self.robot.out = self.robot.out, []
            w = self.writer
            if w and out:
                data = b"".join(out)
                if self.garbage and self.robot.rng.random() < self.garbage * len(out):
                    i = self.robot.rng.randrange(len(data))
                    data = data[:i] + bytes(self.robot.rng.randrange(256) for _ in range(5)) + data[i:]
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
