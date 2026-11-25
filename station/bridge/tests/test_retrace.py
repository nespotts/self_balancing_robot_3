"""Retrace steps when the pose is lost (retrace.py): breadcrumbs and their map richness, the corridor check on the live
lidar turn, the route / stops, and the planner side (pause the job, drive back in the odometry frame, the local search at
a well-mapped crumb, resume; obstacles, give up, settings). The sim scenario (real LD19 packets, matcher, navigator):
station/sim/tests/test_retrace_sim.py."""
import math
import shutil
import tempfile
import types
import unittest

from station.bridge import link_protocol as lp
from station.bridge import retrace as rt
from station.bridge.bridge import NAV_STATE_NAMES
from station.bridge.mapping import Mapper
from station.bridge.planner import Planner


class FakeLidar:
    def __init__(self, clock):
        self.scan_listeners = []
        self.clock = clock
        self.hist = None


class FakeBridge:
    def __init__(self, mapper, clock):
        self.mapper = mapper
        self.clock = clock
        self.link, self.estop, self.estop_time, self.robot_estop = True, False, -1.0, False
        self.tlm = {"x_mm": 0.0, "y_mm": 0.0, "heading_deg": 0.0, "mode": lp.MODE_GOTO, "speed_mms": 0.0,
                    "status": lp.TLM_ARMED}
        self.tlm_time = 0.0
        self.nav = None
        self.lines, self.cmds = [], []

    def say(self, text):
        self.lines.append(text)

    def broadcast(self, text, droppable=False):
        pass

    def handle_command(self, m, who="?"):
        self.cmds.append(m)


def turn(x, y, h, pts):
    """A lidar scan with its deskewed turn: pts [(x, y)] odometry frame."""
    t = types.SimpleNamespace(z=[complex(a, b) for a, b in pts], mid=(x, y, h), tilted=False)
    return types.SimpleNamespace(turn=t)


def wall_pts(n=120):
    """Points on the two walls of a corridor along x: y = +-600, x -1000 .. 3500 (odometry frame)."""
    return [(-1000.0 + 4500.0 * k / n, y) for y in (-600.0, 600.0) for k in range(n)]


class Env(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.t = [100.0]
        clock = lambda: self.t[0]
        self.m = Mapper(self.tmp)
        self.m.lidar = FakeLidar(clock)
        self.b = FakeBridge(self.m, clock)
        self.p = Planner(self.b, NAV_STATE_NAMES, clock=clock, sync=True)
        self.r = self.p.retrace
        self.jobs = []
        self.p._start_job = lambda fn, go=False, check=False, force=False: self.jobs.append(fn.__name__) or True
        g = self.m.grid
        for x, y in wall_pts(400):                  # the walls in the map (bytes over OCC_BYTE)
            g.cells[g.index(x, y)] = 250
        self.lm = self.m.lmatch
        self.lm.turns = 10 ** 6
        self.fresh = True                           # tick(): the last lidar turn stays fresh
        self.track()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def track(self):
        lm = self.lm
        lm.mode, lm.state, lm.fails, lm.jump, lm.suspect = "track", "matched: pose fine", 0.0, None, None

    def lose(self):
        lm = self.lm
        lm.mode, lm.checking, lm.lost_t0, lm.state = "lost", False, self.t[0], "lost: searching"

    def at(self, x, y, h=0.0, speed=0.0):
        self.b.tlm.update(x_mm=x, y_mm=y, heading_deg=h, speed_mms=speed)
        self.b.tlm_time = self.t[0]

    def feed(self, x, y, h=0.0, pts=None):
        self.r.on_scan(turn(x, y, h, wall_pts() if pts is None else pts))

    def drive_line(self, x0, x1, step=50.0, y=0.0):
        """The robot drives along y from x0 to x1, a turn every step mm (0.1 s)."""
        n = int(abs(x1 - x0) / step)
        for k in range(n + 1):
            x = x0 + (x1 - x0) * k / max(1, n)
            self.t[0] += 0.1
            self.at(x, y)
            self.feed(x, y)

    def tick(self, s=0.04):
        self.t[0] += s
        self.b.tlm_time = self.t[0]
        if self.fresh and self.r.scan is not None:
            self.r.scan = (self.t[0], self.r.scan[1])
        return self.p.tick(self.t[0])

    def gotos(self):
        return [c for c in self.b.cmds if c["t"] == "goto"]


class TestPieces(unittest.TestCase):
    def test_richness_counts_distinct_known_wall_cells(self):
        m = Mapper(None)
        g = m.grid
        for x in range(0, 1000, 25):
            g.cells[g.index(x + 1.0, 500.0)] = 250              # a 1 m wall: 40 cells
        pts = [complex(x + 1.0, 500.0) for x in range(0, 1000, 5)]      # 200 points on it (5 per cell)
        self.assertEqual(rt.richness(pts, [0.0, 0.0, 0.0], g, stride=1), 40)
        self.assertEqual(rt.richness(pts, [0.0, 300.0, 0.0], g, stride=1), 0)       # shifted off it: nothing known
        self.assertEqual(rt.richness([p - 300j for p in pts], [0.0, 300.0, 0.0], g, stride=1), 40)
        self.assertLessEqual(rt.richness(pts, [0.0, 0.0, 0.0], g), 40)              # every 3rd point: fewer lookups

    def test_corridor_hit(self):
        half = 110.0
        path = [(0.0, 0.0), (1000.0, 0.0)]
        self.assertIsNone(rt.corridor_hit([complex(500, 200), complex(500, -150), complex(-300, 0)], path, half))
        self.assertEqual(rt.corridor_hit([complex(500, 200), complex(600, 90)], path, half), (600.0, 90.0))
        self.assertIsNone(rt.corridor_hit([complex(30, 55), complex(-20, -60)], path, half))   # the robot's own posts
        self.assertIsNotNone(rt.corridor_hit([complex(1080, 40)], path, half))                # the body at the end
        self.assertIsNone(rt.corridor_hit([complex(1300, 0)], path, half))
        bent = [(0.0, 0.0), (500.0, 0.0), (500.0, 500.0)]
        self.assertIsNotNone(rt.corridor_hit([complex(450, 300)], bent, half))
        self.assertIsNone(rt.corridor_hit([complex(250, 300)], bent, half))


class TestCrumbs(Env):
    def test_spacing_trust_richness_and_keep(self):
        self.drive_line(0.0, 900.0)
        cs = list(self.r.crumbs)
        self.assertEqual(len(cs), 7)                               # every 150 mm of 900
        self.assertTrue(all(c.corr == [0.0, 0.0, 0.0] for c in cs))
        self.assertAlmostEqual(cs[-1].path, 900.0, delta=1.0)
        self.assertGreater(cs[0].rich, 50)                         # the turn's points on the known walls
        self.lm.state = "kept: mostly new ground"                  # not "fine": a crumb for the way, untrusted
        self.drive_line(900.0, 1200.0)
        self.assertIsNone(self.r.crumbs[-1].corr)
        self.assertEqual(self.r.crumbs[-1].rich, 0)
        # only the last KEEP_MM of path
        old = rt.KEEP_MM
        rt.KEEP_MM = 600.0
        try:
            self.drive_line(1200.0, 1500.0)
            cs = list(self.r.crumbs)
            self.assertLessEqual(cs[-1].path - cs[0].path, 600.0)
        finally:
            rt.KEEP_MM = old

    def test_slip_lost_jump_and_new_map(self):
        self.lm.mode = "slip"
        self.drive_line(0.0, 600.0)
        self.assertEqual(len(self.r.crumbs), 0)                    # the wheels' travel is not the body's
        self.lose()
        self.drive_line(600.0, 900.0)
        self.assertEqual(len(self.r.crumbs), 0)                    # lost with no trail: nothing to start one with
        self.track()
        self.drive_line(900.0, 1200.0)
        n = len(self.r.crumbs)
        self.lose()
        self.drive_line(1200.0, 1500.0)
        self.assertGreater(len(self.r.crumbs), n)                  # lost: the way back is still marked ...
        self.assertTrue(all(c.corr is None for c in list(self.r.crumbs)[n:]))   # ... untrusted
        self.track()
        self.t[0] += 0.1
        self.feed(-2000.0, 3000.0)                                 # the odometry jumped: a reboot / pose reset
        self.assertEqual(len(self.r.crumbs), 1)
        self.m.version += 1                                        # another map
        self.feed(-2000.0, 3000.0)
        self.assertEqual(len(self.r.crumbs), 1)
        self.assertEqual(self.r.crumbs[0].path, 0.0)

    def test_route_and_stops(self):
        C = rt.Crumb
        cs = [C(float(-150 * k), 0.0, 0.0, 3000.0 - 150 * k, [0.0, 0.0, 0.0], r)
              for k, r in enumerate([60, 0, 0, 100, 95, 0, 30, 40, 120, 118, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])]
        self.r.crumbs.extend(reversed(cs))         # oldest first (the robot drove towards +x)
        self.p.settings["retrace_max_mm"] = 2000.0
        route, stops = self.r._route({"x_mm": 100.0, "y_mm": 0.0})
        # newest first: the crumb at x = 0 (path 3000) ... within 2 m of path: up to path >= 1100 (k <= 12)
        self.assertEqual(route[0].x, 0.0)
        self.assertEqual([route[i].rich for i in stops], [100, 120])   # >= 75 % of 120; 100 and 95 are close: 100
        self.assertEqual(len(route), stops[-1] + 1)                     # the route ends at the last stop
        self.p.settings["retrace_max_mm"] = 500.0
        route, stops = self.r._route({"x_mm": 100.0, "y_mm": 0.0})
        self.assertEqual([route[i].rich for i in stops], [60])          # nothing rich in reach: the best one


class TestRetrace(Env):
    def lost_far(self):
        """Mapped and tracked from x = 0 to 1500 (rich), then lost; the robot drove on to x = 2400 (sparse)."""
        self.p.mode, self.p.goals = "goto", [(4000.0, 0.0)]
        self.p.started_at = self.t[0]
        self.drive_line(0.0, 1500.0)
        self.lose()
        self.drive_line(1500.0, 2400.0, step=150.0)
        self.b.cmds.clear()

    def wait_start(self):
        for _ in range(200):
            self.tick(0.05)
            if self.r.active():
                return
        self.fail("no retrace: %s" % self.b.lines[-5:])

    def test_pauses_drives_back_checks_and_resumes(self):
        self.lost_far()
        self.assertFalse(self.tick())                              # lost < AFTER_S: the in-place search first
        self.assertFalse(self.r.active())
        self.wait_start()
        self.assertGreaterEqual(self.t[0] - self.lm.lost_t0, rt.AFTER_S)
        self.assertEqual(self.p.status, "lost - retracing steps")
        self.assertEqual(self.p.mode, "goto")                      # the job is paused, not dropped
        g = self.gotos()[-1]["points"]
        self.assertTrue(all(p[0] < 2400 for p in g), g)            # back along the crumbs (odometry frame)
        self.assertGreater(g[0][0], g[-1][0])                      # newest first
        self.assertLessEqual(math.hypot(g[-1][0] - 2400, g[-1][1]), rt.AHEAD_MM + rt.SPACING_MM)   # a short list
        msg = self.p.msg()
        self.assertIn('"retrace":{"phase":"drive"', msg)
        tr = self.r.msg()
        self.assertGreater(len(tr["trail"]), 5)
        self.assertEqual(tr["trail"][0], [round(self.r.st["route"][self.r.st["j"]].x), 0])
        # the robot follows the lists back to the stop (the last trusted crumbs at x <= 1500)
        x = 2400.0
        for _ in range(400):
            g = self.gotos()[-1]["points"]
            tx, ty = g[0]
            x = x - min(30.0, x - tx) if x > tx else x
            self.at(x, 0.0, 180.0, speed=100.0)
            self.feed(x, 0.0, 180.0, pts=[])                        # sparse: nothing in the way, nothing to match
            self.r.scan = (self.t[0], [complex(x - 3000, 2000)])   # (a far point: fresh, not in the corridor)
            self.tick(0.1)
            if self.r.st["phase"] != "drive":
                break
        self.assertEqual(self.r.st["phase"], "settle")
        c = self.r.st["route"][self.r.st["stops"][0]]
        self.assertIsNotNone(c.corr)
        self.assertLess(abs(x - c.x), rt.STOP_REACH_MM + 1)
        self.at(x, 0.0, 180.0, speed=0.0)
        for _ in range(20):
            self.tick(0.05)
        self.assertEqual(self.r.st["phase"], "check")
        req = self.lm.glob_req
        self.assertEqual(req["by"], "retrace")
        ax, ay, half, hdeg = req["around"]
        self.assertAlmostEqual(ax, x, delta=1.0)                   # where the crumb's trusted correction puts it
        self.assertEqual((half, hdeg), (rt.BOX_MM, rt.BOX_DEG))
        self.assertEqual(self.p.status, "lost - retracing steps: checking the pose here")
        # the matcher confirms a pose: the job is planned again from here
        self.track()
        self.tick()
        self.assertFalse(self.r.active())
        self.assertEqual(self.jobs[-1], "_plan_goto")
        self.assertIsNone(self.r.msg())
        self.assertTrue(any("resuming the route" in ln for ln in self.b.lines), self.b.lines[-3:])
        self.assertEqual(self.r.n["found"], 1)
        # the same lost episode never starts it again; a new one does
        self.lose()
        self.lm.lost_t0 = self.r.done_lost
        for _ in range(150):
            self.tick(0.05)
        self.assertFalse(self.r.active())

    def test_found_while_driving_back(self):
        self.lost_far()
        self.wait_start()
        self.lm.reloc = {"phase": "confirming", "by": "auto", "t": self.t[0]}
        self.tick()
        self.assertEqual(self.r.st["phase"], "drive")              # confirmed by the turns while it drives on
        self.track()
        self.tick()
        self.assertFalse(self.r.active())
        self.assertEqual(self.jobs[-1], "_plan_goto")

    def test_obstacle_skips_then_blocks_then_gives_up(self):
        self.lost_far()
        self.wait_start()
        route, j = self.r.st["route"], self.r.st["j"]
        route[j] = route[j]._replace(y=300.0)                      # (a bend in the trail)
        # something on the next crumb, the one after it clear of it: skipped
        self.r.scan = (self.t[0], [complex(route[j].x, 300.0)])
        self.fresh = False
        self.tick()
        self.assertEqual(self.r.st["j"], j + 1)
        self.assertTrue(any("skipping 1 step" in ln for ln in self.b.lines), self.b.lines[-3:])
        # a wall across the way: stop, wait, look again ... then give up
        wall = [complex(2300.0 - 20 * k, y) for k in range(40) for y in (-200.0, -100.0, 0.0, 100.0, 200.0)]
        for _ in range(2000):
            self.r.scan = (self.t[0], wall)
            self.tick(0.1)
            if not self.r.active():
                break
        self.assertFalse(self.r.active())
        self.assertEqual(self.r.n["gave_up"], 1)
        self.assertTrue(any("retracing gave up: blocked on the way back" in ln for ln in self.b.lines), self.b.lines)
        self.assertEqual(self.lm.glob_req["by"], "auto")           # the whole-map search, the Set position prompt
        self.assertEqual(self.jobs[-1], "_plan_goto")              # ... and the job goes on as without retracing
        last = self.gotos()[-1]["points"]
        self.assertEqual(len(last), 1)                             # it was stopped where it stood first

    def test_no_lidar_data_stops_and_gives_up(self):
        self.lost_far()
        self.wait_start()
        self.fresh = False
        self.r.scan = (self.t[0] - 10.0, [])
        n = len(self.gotos())
        self.tick()
        self.assertEqual(len(self.gotos()), n + 1)
        self.assertEqual(len(self.gotos()[-1]["points"]), 1)       # stopped: no driving blind
        for _ in range(100):
            self.tick(0.1)
        self.assertFalse(self.r.active())
        self.assertTrue(any("no lidar data" in ln for ln in self.b.lines))

    def test_zone_stop_contact_and_set_position(self):
        self.lost_far()
        self.wait_start()
        self.p.on_contact({"id": "b1"})                            # touched something: wait, look again
        self.assertEqual(self.r.st["phase"], "wait")
        self.assertIsNone(self.p.replan_at)
        self.p.zone_stop("in no-go zone A")                         # the watchdog: retracing ends there
        self.assertFalse(self.r.active())
        self.assertTrue(any("zone watchdog" in ln for ln in self.b.lines))
        self.assertEqual(self.p.zone_hits, 0)
        # Set position while retracing: stand still for its confirmation
        self.lose()
        self.r.done_lost = None
        self.r.job_n = 0
        self.wait_start()
        self.lm.reloc = {"phase": "set", "by": "set", "t": self.t[0]}
        self.tick()
        self.assertEqual(self.r.st["phase"], "hold")
        self.assertIn("position set by hand", self.p.status)
        self.track()
        self.tick()
        self.assertFalse(self.r.active())

    def test_setting_off_cap_and_stop(self):
        self.p.command({"t": "plan", "action": "set", "settings": {"retrace_lost": False, "retrace_max_mm": 99999}})
        self.assertFalse(self.p.settings["retrace_lost"])
        self.assertEqual(self.p.settings["retrace_max_mm"], 20000)
        self.lost_far()
        for _ in range(150):
            self.assertFalse(self.tick(0.05))
        self.assertFalse(self.r.active())
        self.p.settings["retrace_lost"] = True
        self.wait_start()
        self.p.stop("stopped by test")
        self.assertFalse(self.r.active())
        self.assertEqual(self.p.mode, "idle")
        # PER_JOB retraces in one job at most
        self.p.mode = "goto"
        self.r.job_n = rt.PER_JOB
        self.r.done_lost = None
        for _ in range(150):
            self.tick(0.05)
        self.assertFalse(self.r.active())

    def test_no_crumbs_gives_up_at_once(self):
        self.p.mode, self.p.goals = "goto", [(4000.0, 0.0)]
        self.lose()
        self.at(500.0, 0.0)
        self.r.scan = (self.t[0], [])
        for _ in range(150):
            self.tick(0.05)
            self.r.scan = (self.t[0], [])
        self.assertFalse(self.r.active())
        self.assertEqual((self.r.n["started"], self.r.n["gave_up"]), (1, 1))
        self.assertTrue(any("no well-mapped spot" in ln for ln in self.b.lines), self.b.lines)


if __name__ == "__main__":
    unittest.main()
