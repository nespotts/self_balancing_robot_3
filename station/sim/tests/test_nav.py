"""Navigation in the simulator (issue #8): the Navigator port of src/navigator.cpp driving the simulated robot.
Run: python -m unittest discover -s station -t ."""
import math
import os
import re
import unittest

from station.bridge import link_protocol as lp
from station.sim import sim as simmod
from station.sim.sim import SimRobot

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
HB = lp.encode_heartbeat(0, 1)[4:-2]     # payloads (strip sync/type/len and crc)


def payload(frame):
    return frame[4:-2]


class NavRun:
    """Steps the sim with Pi heartbeats (no remote: arm request + SET_MODE arms) and collects what it sends."""

    def __init__(self, seed=1):
        self.r = SimRobot(seed=seed)
        self.logs, self.navs, self.tlm = [], [], []
        self.hb = HB                   # heartbeat payload (clients = 1)
        self.lidar = False             # send MSG_LIDAR_OBSTACLES (#77) every 0.1 s, like the bridge with an LD19
        self.lidar_seq = 0
        self.lidar_next = 0.0
        self.ranges = 0                # RangeFrames the sim sent

    def frame(self, f):
        self.r.on_frame(f[2], payload(f))

    def lidar_frame(self):
        """What lidar_obstacles.py sends: nearest return (120..8000 mm, 1 deg rays from the axle) per sector, robot
        frame of the pose now (the sim's odometry is exact, so the true pose is the odometry pose)."""
        r = self.r
        best = [None] * lp.LINK_LIDAR_SECTORS
        for k in range(360):
            d = r.room.raycast(r.X, r.Y, math.radians(r.TH + k), 8000.0)
            if d is None or d < 120.0:
                continue
            i = int(math.floor(k / lp.LIDAR_SECTOR_DEG + 0.5)) % lp.LINK_LIDAR_SECTORS
            if best[i] is None or d < best[i][0]:
                best[i] = (d, d * math.cos(math.radians(k)), d * math.sin(math.radians(k)))
        f = lp.encode_lidar_obstacles(self.lidar_seq, 0, [b and b[1:] for b in best])
        self.lidar_seq += 1
        self.frame(f)

    def run(self, seconds, until=None):
        r = self.r
        end = r.t + seconds
        while r.t + r.dt <= end:
            r.on_frame(lp.MSG_HEARTBEAT, self.hb)
            if self.lidar and r.t >= self.lidar_next:
                self.lidar_next = r.t + 0.1
                self.lidar_frame()
            r.run_until(min(end, r.t + 0.05), max_steps=100000)
            for f in r.out:
                t, p = f[2], payload(f)
                if t == lp.MSG_LOG:
                    self.logs.append(lp.decode_log(p))
                elif t == lp.MSG_NAV_STATUS:
                    self.navs.append(lp.decode_nav_status(p))
                elif t == lp.MSG_TELEMETRY:
                    self.tlm.append(lp.decode_telemetry(p))
                elif t == lp.MSG_RANGE:
                    self.ranges += 1
            r.out = []
            if until and until(self):
                return True
        return False

    def stand_up(self, mode):
        """Balance first (the sim's 'hand' that stands it up throws it around), put it back at the origin
        facing where it faced at boot, then switch to `mode`."""
        r = self.r
        th0 = r.TH
        self.frame(lp.encode_event(lp.EVT_LINK_ARM))   # explicit arm request (no remote)
        self.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        self.assertTrue(self.run(10.0, lambda s: s.r.ctl.upright and s.r.t > 3.0 and abs(s.r.speed_f) < 40))
        r.ctl.heading_sp += th0 - r.TH
        r.X = r.Y = 0.0
        r.TH = th0
        r.nav.pts = []
        self.run(1.0)
        self.frame(lp.encode_set_mode(mode))

    def assertTrue(self, v):
        if not v:
            raise AssertionError("condition not reached")

    def pos(self):
        x, y, _ = self.r.odom()
        return x, y


class TestParamsMatchFirmware(unittest.TestCase):
    def test_same_table(self):
        with open(os.path.join(ROOT, "src", "params.h")) as fh:
            src = fh.read()
        rows = re.findall(r'X\(\s*\w+,\s*"([^"]+)",\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f\)', src)
        self.assertEqual([r[0] for r in rows], [p[0] for p in simmod.PARAMS])
        for (name, d, lo, hi), (sn, sd, slo, shi) in zip(rows, simmod.PARAMS):
            if name != "rng.enable":       # the sim starts with the sweep on
                self.assertAlmostEqual(float(d), sd, places=2, msg=name)
            self.assertAlmostEqual(float(lo), slo, places=4, msg=name)
            self.assertAlmostEqual(float(hi), shi, places=4, msg=name)

    def test_state_names(self):
        self.assertEqual(lp.NAV_STATE_NAMES[simmod.NAV_HOLDING], "holding")


class TestSafety(unittest.TestCase):
    """Safety review rules (issues #38 #41), same as src/mode_manager.cpp / navigator.cpp."""

    def moved(self, n, seconds):
        x0, y0 = n.pos()
        n.run(seconds)
        x, y = n.pos()
        return math.hypot(x - x0, y - y0)

    def test_no_clients_no_motion(self):
        n = NavRun(seed=11)
        n.frame(lp.encode_goto([(1500.0, 0.0)]))
        n.stand_up(lp.MODE_GOTO)
        n.run(1.0)
        n.hb = lp.encode_heartbeat(0, 0)[4:-2]          # the bridge runs, nobody is connected
        n.run(1.0)                                      # (ramp down)
        self.assertLess(self.moved(n, 2.0), 150.0)      # (balancing sway; driving would be ~500 mm)
        self.assertLess(max(abs(t["cmd_speed_mms"]) for t in n.tlm[-100:]), 1.0)
        self.assertEqual(n.r.mode, lp.MODE_GOTO)        # still armed + balancing, just paused
        n.hb = HB                                       # a browser connects again
        self.assertGreater(self.moved(n, 3.0), 150.0)

    def test_explore_paused_without_range(self):
        n = NavRun(seed=12)
        n.r.set_param(n.r.pidx["rng.enable"], 0.0)
        n.frame(lp.encode_goto([(1500.0, 0.0)]))
        n.run(4.0)                                      # no readings for > 3 s
        n.stand_up(lp.MODE_EXPLORE)
        n.run(0.5)
        self.assertLess(self.moved(n, 3.0), 80.0)
        self.assertTrue(any("EXPLORE paused" in l for l in n.logs), n.logs[-5:])

    def test_goto_capped_without_range(self):
        n = NavRun(seed=13)
        n.r.set_param(n.r.pidx["rng.enable"], 0.0)
        n.frame(lp.encode_goto([(2500.0, 0.0)]))
        n.run(4.0)
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(20.0, lambda s: s.r.nav.state == lp.NAV_DRIVING))
        n.run(3.0)
        self.assertLessEqual(max(abs(t["cmd_speed_mms"]) for t in n.tlm[-100:]), n.r.P("nav.blind_speed") + 1)
        self.assertTrue(any("GOTO max 150" in l for l in n.logs), n.logs[-5:])

    def test_link_estop_latches(self):
        n = NavRun(seed=14)
        n.stand_up(lp.MODE_STATION)
        n.run(0.5)
        self.assertEqual(n.r.mode, lp.MODE_STATION)
        n.frame(lp.encode_event(lp.EVT_LINK_ESTOP))
        n.run(0.2)
        self.assertEqual(n.r.mode, lp.MODE_IDLE)
        self.assertTrue(n.tlm[-1]["remote_flags"] & lp.LINK_RF_LINK_ESTOP)
        n.frame(lp.encode_event(lp.EVT_LINK_ARM))       # arm step while latched: ignored
        n.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        n.run(0.3)
        self.assertEqual(n.r.mode, lp.MODE_IDLE)
        self.assertIn("SET_MODE 1 ignored: station E-stop latched (release it first)", n.logs)
        n.frame(lp.encode_event(lp.EVT_LINK_ESTOP_RELEASE))
        n.frame(lp.encode_set_mode(lp.MODE_BALANCE))   # release alone never arms
        n.run(0.3)
        self.assertEqual(n.r.mode, lp.MODE_IDLE)
        self.assertFalse(n.tlm[-1]["remote_flags"] & lp.LINK_RF_LINK_ESTOP)
        n.frame(lp.encode_event(lp.EVT_LINK_ARM))
        n.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        n.run(0.3)
        self.assertEqual(n.r.mode, lp.MODE_BALANCE)


class TestGoto(unittest.TestCase):
    def test_three_waypoints_arrive_and_stop(self):
        n = NavRun(seed=5)
        pts = [(800.0, 0.0), (800.0, 500.0), (0.0, 500.0)]
        n.frame(lp.encode_goto(pts))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(90.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED), "never arrived: %s" % n.logs[-5:])
        reached = [l for l in n.logs if l.startswith("nav: reached")]
        self.assertEqual(len(reached), 3, n.logs)
        self.assertIn("nav: arrived", n.logs)
        x, y = n.pos()
        self.assertLess(math.hypot(x - 0.0, y - 500.0), 2 * n.r.P("nav.arrive_mm"))
        n.run(2.0)                                      # and it stays stopped
        self.assertLess(abs(n.r.speed_f), 60.0)                       # (balancing sway)
        x2, y2 = n.pos()
        self.assertLess(math.hypot(x2 - x, y2 - y), 40.0)
        # status: prompt + every state seen, the final one ARRIVED with nav_index = count
        states = [m["state"] for m in n.navs]
        for st in (lp.NAV_TURNING, lp.NAV_DRIVING, lp.NAV_ARRIVED):
            self.assertIn(st, states)
        self.assertEqual((n.navs[-1]["state"], n.navs[-1]["nav_index"], n.navs[-1]["waypoint_count"]),
                         (lp.NAV_ARRIVED, 3, 3))
        t = n.tlm[-1]
        self.assertTrue(t["status"] & lp.TLM_NAV_DONE)
        self.assertEqual((t["nav_state"], t["nav_index"]), (lp.NAV_ARRIVED, 3))
        self.assertGreater(n.r.room.clearance(n.r.X, n.r.Y), simmod.ROBOT_RADIUS)

    def test_end_heading(self):
        n = NavRun(seed=6)
        n.r.set_param(n.r.pidx["nav.end_heading_deg"], 90.0)
        n.frame(lp.encode_goto([(500.0, 0.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(40.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED))
        _, _, th = n.r.odom()
        self.assertLess(abs(simmod._wrap180(th - 90.0)), 6.0)

    def test_obstacle_stops_and_resumes_when_clear(self):
        # second leg runs along y = 900 straight into the box (x 1200..1800, y 700..1100)
        n = NavRun(seed=7)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED), n.logs[-5:])
        n.run(1.5)
        x0 = n.r.X
        n.run(3.0)
        r = n.r
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        self.assertLess(abs(r.speed_f), 80.0)                          # (balancing sway)
        self.assertLess(abs(r.X - x0), 80.0)                           # stays put
        self.assertLess(r.X, 1200.0 - simmod.ROBOT_RADIUS)            # stopped short of the box
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        blocked = [m for m in n.navs if m["state"] == lp.NAV_BLOCKED]
        self.assertTrue(blocked)
        self.assertGreater(blocked[-1]["obstacle_mm"], 150.0)
        self.assertLess(blocked[-1]["obstacle_mm"], r.P("rng.obstacle_mm") + r.P("nav.clear_mm"))
        self.assertTrue(n.tlm[-1]["status"] & lp.TLM_OBSTACLE)
        self.assertTrue(any(l.startswith("nav: blocked") for l in n.logs))
        # GOTO waits for the way to clear: take the box away
        box = simmod._box(1500, 900, 600, 400)
        r.room.segs = [s for s in r.room.segs if s not in box]
        self.assertTrue(n.run(20.0, lambda s: s.r.nav.state == lp.NAV_DRIVING))
        self.assertTrue(any(l.startswith("nav: clear") for l in n.logs))

    def test_backoff(self):
        n = NavRun(seed=8)
        n.r.set_param(n.r.pidx["nav.backoff_mm"], 120.0)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_GOTO)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED))
        x0 = n.r.X
        n.run(6.0)
        self.assertEqual(n.r.nav.state, lp.NAV_BLOCKED)    # backed off, still blocked (no oscillation)
        self.assertLess(n.r.X, x0 - 60.0)


class TestExplore(unittest.TestCase):
    def test_blocked_waits_for_new_list(self):
        n = NavRun(seed=9)
        n.frame(lp.encode_goto([(600.0, 900.0), (2500.0, 900.0)]))
        n.stand_up(lp.MODE_EXPLORE)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED))
        box = simmod._box(1500, 900, 600, 400)            # even when the way clears ...
        n.r.room.segs = [s for s in n.r.room.segs if s not in box]
        n.run(4.0)
        self.assertEqual(n.r.nav.state, lp.NAV_BLOCKED)    # ... EXPLORE waits for the planner
        n.frame(lp.encode_goto([(600.0, 300.0)]))          # new list (replace) -> resumes
        self.assertTrue(n.run(40.0, lambda s: s.r.nav.state == lp.NAV_ARRIVED))
        x, y = n.pos()
        self.assertLess(math.hypot(x - 600.0, y - 300.0), 100.0)


class TestHold(unittest.TestCase):
    def test_comes_back_after_a_push(self):
        n = NavRun(seed=10)
        n.stand_up(lp.MODE_HOLD)
        n.run(2.0)
        hx, hy = n.r.nav.hold
        self.assertEqual(n.r.nav.state, lp.NAV_HOLDING)
        n.r.X += 250.0                                    # shoved 25 cm forward
        n.run(8.0)
        x, y = n.pos()
        self.assertLess(math.hypot(x - hx, y - hy), 60.0)


class TestStation(unittest.TestCase):
    def test_cmd_vel_obstacle_stop(self):
        n = NavRun(seed=11)
        n.r.TH = -90.0                                    # facing the wall at y = -1500
        n.stand_up(lp.MODE_STATION)
        cmd = lp.encode_cmd_vel(200.0, 0.0)               # the station keeps driving forward
        end = n.r.t + 15.0
        while n.r.t + n.r.dt <= end:
            n.frame(cmd)
            n.run(0.1)
        r = n.r
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        self.assertGreater(r.Y, -1500.0 + simmod.ROBOT_RADIUS)
        self.assertLess(r.Y, -800.0)                      # it did drive
        self.assertLess(abs(r.speed_f), 40.0)
        self.assertTrue(n.tlm[-1]["status"] & lp.TLM_OBSTACLE)


class TestLidarNav(unittest.TestCase):
    """Issue #77: no sweep (rng.enable 0), obstacles only from MSG_LIDAR_OBSTACLES frames."""

    def lidar_run(self, seed, points, mode):
        n = NavRun(seed=seed)
        n.r.set_param(n.r.pidx["rng.enable"], 0.0)
        n.lidar = True
        n.frame(lp.encode_goto(points))
        n.stand_up(mode)
        return n

    def test_explore_lidar_only_stops_for_the_box(self):
        n = self.lidar_run(31, [(600.0, 900.0), (2500.0, 900.0)], lp.MODE_EXPLORE)
        self.assertTrue(n.run(60.0, lambda s: s.r.nav.state == lp.NAV_BLOCKED), n.logs[-5:])
        n.run(2.0)
        r = n.r
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        self.assertLess(r.X, 1200.0 - simmod.ROBOT_RADIUS)               # stopped short of the box
        self.assertGreater(r.X, 600.0)                                    # after driving the first leg + some
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        blocked = [m for m in n.navs if m["state"] == lp.NAV_BLOCKED]
        self.assertGreater(blocked[-1]["obstacle_mm"], 150.0)
        self.assertLess(blocked[-1]["obstacle_mm"], r.P("rng.obstacle_mm") + r.P("nav.clear_mm"))
        self.assertTrue(r.nav.lidar_fresh(int(r.t * 1000)))
        self.assertFalse(any("paused" in l or "no range" in l for l in n.logs), n.logs)     # never blind
        self.assertEqual(n.ranges, 0)                                     # and no sweep at all

    def test_lidar_lost_pauses_explore_until_it_is_back(self):
        n = self.lidar_run(32, [(3000.0, 0.0)], lp.MODE_EXPLORE)
        self.assertTrue(n.run(20.0, lambda s: s.r.nav.state == lp.NAV_DRIVING and s.r.speed_f > 150))
        n.lidar = False
        n.run(1.5)                                                        # stale after 0.5 s, then ramp down
        x0 = n.r.X
        n.run(3.0)
        self.assertLess(abs(n.r.X - x0), 80.0)                            # paused (balancing sway)
        self.assertIn("nav: lidar obstacles lost (no scan for 0.5 s)", n.logs)
        self.assertTrue(any("EXPLORE paused" in l for l in n.logs), n.logs[-5:])
        n.lidar = True
        n.run(3.0)
        self.assertGreater(n.r.X - x0, 200.0)                             # going again

    def test_station_cmd_vel_stops_with_lidar(self):
        n = NavRun(seed=33)
        n.r.set_param(n.r.pidx["rng.enable"], 0.0)
        n.lidar = True
        n.r.TH = -90.0                                    # facing the wall at y = -1500
        n.stand_up(lp.MODE_STATION)
        cmd = lp.encode_cmd_vel(200.0, 0.0)
        end = n.r.t + 15.0
        while n.r.t + n.r.dt <= end:
            n.frame(cmd)
            n.run(0.1)
        r = n.r
        self.assertGreater(r.room.clearance(r.X, r.Y), simmod.ROBOT_RADIUS)
        self.assertLess(r.Y, -800.0)                      # it did drive
        self.assertLess(abs(r.speed_f), 40.0)
        self.assertTrue(n.tlm[-1]["status"] & lp.TLM_OBSTACLE)


class TestWalls(unittest.TestCase):
    """Issue #62: close to and parallel with a wall (the right wall, x = 4000; the sim's body is a 90 mm circle
    that cannot slide along a wall it touches). old = side check and stuck detection off (the old firmware)."""

    def wall_run(self, start, points, old=False, params=None, seconds=40.0, watch=None):
        n = NavRun(seed=21)
        p = dict(params or {})
        if old:
            p.update({"nav.side_min_mm": 0.0, "nav.stuck_ms": 0.0})
        for k, v in p.items():
            n.r.set_param(n.r.pidx[k], v)
        n.frame(lp.encode_goto(points))
        n.stand_up(lp.MODE_BALANCE)
        r = n.r
        x, y, th = start
        r.ctl.heading_sp += th - r.TH
        r.X, r.Y, r.TH = x, y, th
        r.nav.pts = []
        n.run(1.0)
        n.frame(lp.encode_set_mode(lp.MODE_GOTO))
        n.closest = 1e9
        end = r.t + seconds
        while r.t < end and r.nav.state != lp.NAV_ARRIVED:
            n.run(0.25)
            n.closest = min(n.closest, r.room.clearance(r.X, r.Y))
            if watch:
                watch(n)
        return n

    def test_parallel_close_old_sticks_new_keeps_clear(self):
        start, goal = (3895.0, -1200.0, 90.0), [(3750.0, 1500.0)]      # 105 mm off the wall (body side: 29)
        old = self.wall_run(start, goal, old=True)
        self.assertEqual(old.r.nav.state, lp.NAV_BLOCKED)              # the wall alongside "is in the way"
        self.assertLess(old.r.Y, 0.0)                                  # ... and it never gets going
        new = self.wall_run(start, goal)
        self.assertEqual(new.r.nav.state, lp.NAV_ARRIVED, new.logs[-5:])
        x, y = new.pos()
        self.assertLess(math.hypot(x - 3750.0, y - 1500.0), 100.0)
        self.assertGreater(new.closest, simmod.ROBOT_RADIUS + 10.0)    # never touched it
        self.assertFalse(any(l.startswith("nav: blocked") for l in new.logs), new.logs)

    def test_start_touching_the_wall_recovers(self):
        start, goal = (3909.0, -1200.0, 85.0), [(3800.0, 1500.0)]      # touching, 5 deg into the wall
        old = self.wall_run(start, goal, old=True, seconds=20.0)
        self.assertEqual(old.r.nav.state, lp.NAV_BLOCKED)
        self.assertLess(old.r.Y, -1100.0)                              # stuck where it started
        new = self.wall_run(start, goal)
        self.assertEqual(new.r.nav.state, lp.NAV_ARRIVED, new.logs[-5:])
        self.assertGreater(new.r.room.clearance(new.r.X, new.r.Y), simmod.ROBOT_RADIUS + 40.0)

    def test_stuck_backs_off_and_reports(self):
        # obstacle stop and side check off: it drives into the wall, the wheels stop, odometry shows no progress
        seen = {}

        def watch(n):
            if n.r.nav.stuck and "x" not in seen:
                seen["x"], seen["t"] = n.r.X, n.r.t
            if "x" in seen and n.r.t - seen["t"] < 5.5:      # back-off gives up after 5 s
                seen["min_x"] = min(seen.get("min_x", 1e9), n.r.X)
                seen.setdefault("states", set()).add(n.r.nav.state)
        n = self.wall_run((3000.0, 1500.0, 0.0), [(4100.0, 1500.0)], seconds=20.0, watch=watch,
                          params={"rng.obstacle_mm": 0.0, "nav.side_min_mm": 0.0})
        self.assertTrue(any(l.startswith("nav: stuck, no progress driving") for l in n.logs), n.logs)
        self.assertIn("x", seen)
        self.assertGreater(seen["x"], 4000.0 - simmod.ROBOT_RADIUS - 5.0)   # it was at the wall
        self.assertIn(lp.NAV_BLOCKED, seen["states"])
        self.assertGreater(seen["x"] - seen["min_x"], 60.0)                  # backed off (nav.stuck_backoff_mm 100)
        self.assertLess(seen["x"] - seen["min_x"], 160.0)
        blocked = [m for m in n.navs if m["state"] == lp.NAV_BLOCKED]
        self.assertTrue(blocked)


class TestWheelSlip(unittest.TestCase):
    """Sim wheel slip (#79) and the navigator's lidar stuck (EVT_LINK_STUCK), mirror of test/host/nav_sim.cpp."""

    def driving(self, seed=3):
        n = NavRun(seed)
        n.frame(lp.encode_goto([(2000.0, 300.0)], True))     # the tour's first leg: clear
        n.stand_up(lp.MODE_GOTO)
        n.assertTrue(n.run(8.0, lambda s: s.r.nav.state == lp.NAV_DRIVING and s.r.speed_f > 150))
        return n

    def test_slip_moves_the_odometry_not_the_body(self):
        for ratio, lo, hi in ((1.0, 0.0, 0.03), (0.5, 0.4, 0.6)):
            n = self.driving()
            r = n.r
            r.slip_now(1.0, ratio)
            (ox, oy, _), (tx, ty) = r.odom(), (r.X, r.Y)
            n.run(1.0)
            (ox2, oy2, _), (tx2, ty2) = r.odom(), (r.X, r.Y)
            wheels, body = math.hypot(ox2 - ox, oy2 - oy), math.hypot(tx2 - tx, ty2 - ty)
            self.assertGreater(wheels, 150.0)
            self.assertTrue(lo <= body / wheels <= hi, (ratio, wheels, body))
            self.assertAlmostEqual(r.slip_mm, wheels - body, delta=10.0)
            n.run(1.0)                                   # over: both move again
            (ox3, oy3, _), (tx3, ty3) = r.odom(), (r.X, r.Y)
            self.assertAlmostEqual(math.hypot(ox3 - ox2, oy3 - oy2), math.hypot(tx3 - tx2, ty3 - ty2), delta=5.0)
            self.assertFalse(r.slipping())

    def test_lidar_stuck_event_backs_off(self):
        n = self.driving()
        r = n.r
        r.slip_now(10.0, 1.0)                            # wheels spin: the odometry says it is getting there
        n.run(1.0)
        self.assertEqual(r.nav.state, lp.NAV_DRIVING)    # its own check sees progress
        n.frame(lp.encode_event(lp.EVT_LINK_STUCK))
        n.run(0.3)
        self.assertTrue(any(l.startswith("nav: stuck, wheels turn but the lidar sees no motion") for l in n.logs),
                        n.logs[-5:])
        self.assertTrue(r.nav.stuck)
        self.assertEqual(r.nav.state, lp.NAV_BLOCKED)
        self.assertLess(r.nav.speed, 0.0)                # backing off

    def test_lidar_stuck_event_needs_driving(self):
        n = NavRun(3)
        n.stand_up(lp.MODE_BALANCE)
        n.frame(lp.encode_event(lp.EVT_LINK_STUCK))
        n.run(1.0)
        self.assertFalse(n.r.nav.stuck)
        self.assertEqual(n.r.nav.lidar_stuck_ms, 0)      # dropped after 500 ms
        self.assertFalse(any(l.startswith("nav: stuck") for l in n.logs))


if __name__ == "__main__":
    unittest.main()
