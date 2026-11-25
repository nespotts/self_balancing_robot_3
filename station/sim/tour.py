"""Sim extras for mapping (issue #12): odometry drift and a waypoint tour around the room.

  python -m station.sim --tour              robot drives a loop around the room (forever)
  python -m station.sim --tour --drift 3    ... with odometry that drifts (3 deg/min heading, 2 % distance)
"""
import asyncio
import math

from station.bridge import link_protocol as lp
from .sim import SimRobot

# A loop through the sim room (odometry frame = world frame at start), around the central box
# and the partition wall; every leg keeps >= 200 mm from walls.
TOUR = [(2000, 300), (3400, -100), (3500, 1400), (2600, 700), (2000, 600), (2000, 1500),
        (0, 1500), (-500, 300), (0, 0)]


CONE_DEG = 8.0             # half-width of the sim range beam (VL53L0X: ~25 deg field of view)
CONE_OFFSETS = [math.radians(k * CONE_DEG / 2) for k in (-2, -1, 0, 1, 2)]


class DriftRobot(SimRobot):
    """SimRobot whose reported odometry drifts: heading gains drift_deg_min degrees per minute
    (gyro bias) and distances read drift_scale too long (wheel size)."""

    def __init__(self, *a, drift_deg_min=0.0, drift_scale=0.0, **kw):
        self.drift_rate = drift_deg_min / 60.0
        self.drift_scale = drift_scale
        self.dx_o = self.dy_o = self.dth_o = 0.0
        super().__init__(*a, **kw)
        self._prev = (self.t, self.X, self.Y, self.TH)

    def _range(self, sid, x, y, th, pitch):
        """Like SimRobot._range, but the beam is a cone (as the VL53L0X): the nearest target within
        +-CONE_DEG counts, not just the one on the centre line."""
        room = self.room
        if not hasattr(room, "_raycast1"):
            room._raycast1 = room.raycast

            def cone_cast(ox, oy, ang, max_d=None, _c=room._raycast1):
                best = None
                for k in CONE_OFFSETS:
                    d = _c(ox, oy, ang + k) if max_d is None else _c(ox, oy, ang + k, max_d)
                    if d is not None and (best is None or d < best):
                        best = d
                return best
            room.cone_cast = cone_cast
        rc = room.raycast
        room.raycast = room.cone_cast
        try:
            return super()._range(sid, x, y, th, pitch)
        finally:
            room.raycast = rc

    # (EXPLORE follows the MSG_GOTO waypoints like GOTO natively in sim.py's Navigator, as the firmware)

    def drifting(self):
        return bool(self.drift_rate or self.drift_scale)

    def odom(self):
        if not self.drifting():
            return super().odom()
        return self.dx_o, self.dy_o, self.dth_o

    def step(self, dt=None):
        super().step(dt)
        t0, x0, y0, th0 = self._prev
        self._prev = (self.t, self.X, self.Y, self.TH)
        if not self.drifting():
            return
        dth = self.TH - th0
        mid = math.radians(th0 + dth / 2)
        ds = (self.X - x0) * math.cos(mid) + (self.Y - y0) * math.sin(mid) + self.slip_ds   # + wheel slip (#79)
        h_mid = math.radians(self.dth_o + dth / 2)
        self.dx_o += ds * (1 + self.drift_scale) * math.cos(h_mid)
        self.dy_o += ds * (1 + self.drift_scale) * math.sin(h_mid)
        self.dth_o += dth + self.drift_rate * (self.t - t0)

    def on_event(self, ev):
        super().on_event(ev)
        if ev == lp.EVT_RESET_POSE:
            self.dx_o = self.dy_o = self.dth_o = 0.0

    def drift_error(self):
        """(position error mm, heading error deg) of the reported pose vs the truth."""
        x, y, th = super().odom()
        ox, oy, oth = self.odom()
        return math.hypot(ox - x, oy - y), oth - th


_HB = lp.encode_heartbeat(0, 1)[4:-2]


def heartbeat(robot):
    """The Pi's 10 Hz heartbeat: GOTO / EXPLORE only move while supervised (remote or Pi link)."""
    robot.on_frame(lp.MSG_HEARTBEAT, _HB)


def set_list(robot, points, replace=True):
    """What a MSG_GOTO does in the firmware (src/main.cpp): fill the navigator's list."""
    nav = robot.nav
    if replace:
        nav.clear_list()
    for x, y in points:
        nav.add(float(x), float(y))
    nav.list_changed(replace)


def drive_offline(robot, on_range, points=TOUR, max_s=240.0, on_step=None):
    """No network, as fast as the CPU goes: arm, stand up, follow the waypoints once; every RangeFrame
    goes to on_range(decoded dict). Returns simulated seconds used (max_s if it never arrived)."""
    parser = lp.FrameParser()

    def pump():
        out, robot.out = robot.out, []
        for t, p in parser.feed(b"".join(out)):
            if t == lp.MSG_RANGE:
                on_range(lp.decode_range(p))

    robot.set_mode(lp.MODE_BALANCE, arm=True)
    t_end = robot.t + 2.5
    while robot.t < t_end:
        if robot.step_n % 40 == 0:
            heartbeat(robot)
        robot.step()
    pump()
    todo = [tuple(p) for p in points]
    set_list(robot, todo)
    robot.set_mode(lp.MODE_GOTO)
    start, blocked = robot.t, 0.0
    nav = robot.nav
    while robot.t - start < max_s:
        heartbeat(robot)
        for _ in range(40):
            robot.step()
        pump()
        if on_step:
            on_step(robot)
        if nav.state == lp.NAV_ARRIVED and nav.idx >= len(nav.wp):
            return robot.t - start
        blocked = blocked + 0.1 if nav.state == lp.NAV_BLOCKED else 0.0
        if blocked > 4.0:                                   # skip a blocked leg
            rest = todo[todo.index(nav.wp[nav.idx]) + 1:] if nav.wp[nav.idx] in todo else []
            if not rest:
                return robot.t - start
            todo = rest
            set_list(robot, todo)
            blocked = 0.0
    return max_s


class OfflineBridge:
    """Just enough of the Bridge for the Planner, wired straight to a SimRobot (no network, sim time)."""

    def __init__(self, robot, mapper, quiet=True):
        from station.bridge.bridge import NAV_STATE_NAMES
        from station.bridge.planner import Planner
        self.robot, self.mapper, self.quiet = robot, mapper, quiet
        self.link, self.estop, self.estop_time, self.robot_estop = True, False, -1.0, False
        self.tlm = self.nav = None
        self.tlm_time = 0.0
        self.waypoints = []
        self.lines = []
        self.robot_logs = []
        self.contacts = []            # (MSG_CONTACT dict, bump mark, merged), #104
        self.planner = Planner(self, NAV_STATE_NAMES, clock=lambda: robot.t, sync=True)

    def say(self, text):
        self.lines.append(text)
        if not self.quiet:
            print("[%.1f] %s" % (self.robot.t, text))

    def broadcast(self, text, droppable=False):
        pass

    def handle_command(self, m, who="?"):
        r = self.robot
        if m["t"] == "goto":
            pts = [(float(x), float(y)) for x, y in m["points"]]
            replace = m.get("replace", True)
            set_list(r, pts, replace)
            self.waypoints = pts if replace else self.waypoints + pts
        elif m["t"] == "set_mode":
            r.set_mode(m["mode"], arm=bool(m.get("arm")))

    def on_contact(self, d):
        """MSG_CONTACT (#104), as Bridge.on_contact: a bump mark on the map, the planner replans round it."""
        mark, merged = self.mapper.add_contact(d)
        self.contacts.append((d, mark, merged))
        self.say("robot: contact %s -> bump mark %s%s" % (d["name"], mark["id"], " (again)" if merged else ""))
        self.planner.on_contact(mark)

    def update(self):
        r = self.robot
        x, y, th = r.odom()
        self.tlm = {"x_mm": x, "y_mm": y, "heading_deg": th, "mode": r.mode,
                    "status": lp.TLM_ARMED if r.armed else 0}
        self.tlm_time = r.t
        self.nav = r.nav.status()


def plan_offline(robot, mapper, command, max_s=300.0, on_step=None):
    """Arm, then run a planner command ({"t": "plan", "action": "explore"} or a goto), or a function
    command(bridge), in sim time until the planner is idle again. Returns (sim seconds, bridge) - bridge.lines has the planner's log."""
    parser = lp.FrameParser()
    b = OfflineBridge(robot, mapper)

    def pump():
        out, robot.out = robot.out, []
        for t, p in parser.feed(b"".join(out)):
            if t == lp.MSG_RANGE:
                mapper.on_range(lp.decode_range(p))
            elif t == lp.MSG_LOG:
                text = lp.decode_log(p)
                b.robot_logs.append(text)
                b.planner.on_log(text)            # as Bridge does ("nav: stuck", #62)
            elif t == lp.MSG_CONTACT:
                b.on_contact(lp.decode_contact(p))   # a bump mark, the planner replans round it (#104)

    robot.set_mode(lp.MODE_BALANCE, arm=True)
    t_end = robot.t + 2.5
    while robot.t < t_end:
        if robot.step_n % 40 == 0:
            heartbeat(robot)
        robot.step()
    pump()
    b.update()

    err = command(b) if callable(command) else b.planner.command(command, "test")
    if err:
        raise RuntimeError(err)
    start = robot.t
    while robot.t - start < max_s:
        heartbeat(robot)
        for _ in range(40):
            robot.step()
        pump()
        mapper.poll(robot.t)
        b.update()
        b.planner.tick(robot.t)
        if on_step:
            on_step(robot, b)
        if b.planner.mode == "idle":
            break
    return robot.t - start, b


async def tour(bridge, points=TOUR, loops=0, quiet=False):
    """Arm (no remote) and drive the waypoint loop through the bridge, like a user pressing Go.
    loops = 0: forever. A leg that stays BLOCKED for 4 s is skipped."""
    while not (bridge.tlm and bridge.hello):
        await asyncio.sleep(0.2)
    await asyncio.sleep(1.0)
    bridge.handle_command({"t": "set_mode", "mode": lp.MODE_BALANCE, "arm": True}, "tour")
    await asyncio.sleep(2.5)
    n = 0
    while loops == 0 or n < loops:
        n += 1
        todo = list(points)
        bridge.handle_command({"t": "goto", "points": todo, "replace": True, "go": True}, "tour")
        blocked_since = None
        await asyncio.sleep(1.0)
        while True:
            await asyncio.sleep(0.25)
            nav = bridge.nav or {}
            if bridge.tlm and bridge.tlm["mode"] == lp.MODE_IDLE:
                return                          # STOP pressed
            if nav.get("state") == lp.NAV_ARRIVED and nav.get("nav_index", 0) >= len(todo):
                break                           # arrived
            if nav.get("state") == lp.NAV_BLOCKED:
                blocked_since = blocked_since or asyncio.get_running_loop().time()
                if asyncio.get_running_loop().time() - blocked_since > 4.0:
                    rest = todo[nav.get("nav_index", 0) + 1:]
                    if not rest:
                        break
                    todo = rest
                    bridge.handle_command({"t": "goto", "points": todo, "replace": True, "go": True}, "tour")
                    blocked_since = None
            else:
                blocked_since = None
