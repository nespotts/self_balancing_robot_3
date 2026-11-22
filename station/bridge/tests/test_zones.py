"""Virtual walls, no-go zones, fence, slow zones, mazes (issue #70). Run: python -m unittest discover -s station -t ."""
import json
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge import zones as zm
from station.bridge.bridge import Bridge
from station.bridge.mapping import Mapper
from station.bridge.occgrid import Grid
from station.bridge.planner import BLOCKED, MARGIN, CostMap, DEFAULTS
from station.bridge.tests.test_hardening import WEB
from station.bridge.zones import Zones, ZoneView, crosses, make_maze, maze_cells_connected
from station.sim.sim import Room
from station.sim.tour import DriftRobot, OfflineBridge, plan_offline


def free_grid(x0=-3000, y0=-3000, x1=3000, y1=3000, walls=()):
    """A grid seen free over the box, with real walls (segments) marked occupied."""
    g = Grid(10.0, 25.0)
    for y in range(int(y0), int(y1), 25):
        for x in range(int(x0), int(x1), 25):
            i = g.index(x, y)
            if i is not None:
                g.cells[i] = 60
    for x1_, y1_, x2_, y2_ in walls:
        n = int(math.hypot(x2_ - x1_, y2_ - y1_) / 10) + 1
        for k in range(n + 1):
            i = g.index(x1_ + (x2_ - x1_) * k / n, y1_ + (y2_ - y1_) * k / n)
            if i is not None:
                g.cells[i] = 250
    g._cover([x0, x1], [y0, y1])
    return g


def route_crosses(route, segs):
    return any(crosses(a[0], a[1], b[0], b[1], s) for a, b in zip(route, route[1:]) for s in segs)


def squeeze_settings():
    s = dict(DEFAULTS)
    s["margin_mm"] = s["squeeze_mm"]
    return s


class TestStore(unittest.TestCase):
    def test_add_update_delete_undo(self):
        z = Zones()
        w = z.add("wall", [[0, 0], [1000, 0]])
        n = z.add("nogo", [[0, 0], [500, 0], [500, 500], [0, 500]])
        s = z.add("slow", [[0, 0], [500, 0], [500, 500]], speed=80)
        self.assertEqual(z.find(s)["speed"], 80)
        f1 = z.add("fence", [[-2000, -2000], [2000, -2000], [2000, 2000], [-2000, 2000]])
        f2 = z.add("fence", [[-3000, -3000], [3000, -3000], [3000, 3000]])
        self.assertIsNone(z.find(f1))                                  # one fence: the new one replaces it
        self.assertEqual([i["kind"] for i in z.items].count("fence"), 1)
        z.update(n, pts=[[0, 0], [800, 0], [800, 800]])
        self.assertEqual(len(z.find(n)["pts"]), 3)
        z.update(s, speed=9999)
        self.assertEqual(z.find(s)["speed"], zm.SLOW_LIMITS[1])        # clamped
        z.delete(w)
        self.assertIsNone(z.find(w))
        z.undo()
        self.assertIsNotNone(z.find(w))
        for bad in ([[0, 0]], [[0, 0], [1, 1], [2, 0]], [[0, 0], ["x", 1]], [[0, 0], [1e9, 0]]):
            with self.assertRaises(ValueError):
                z.add("nogo", bad)
        with self.assertRaises(ValueError):
            z.add("lava", [[0, 0], [1, 1]])
        with self.assertRaises(ValueError):
            z.delete("nope")
        z.clear("slow")
        self.assertIsNone(z.find(s))
        self.assertIsNotNone(z.find(f2))
        z.clear()
        self.assertEqual(z.items, [])

    def test_json_roundtrip_and_bad_entries(self):
        z = Zones()
        z.add("wall", [[0, 0], [1000, 0], [1000, 1000]])
        z.add("slow", [[0, 0], [500, 0], [500, 500]], speed=100)
        mid = z.add_maze(0, 0, 2000, 2000, 500, seed=3)
        q = Zones.from_json(z.to_json())
        self.assertEqual(q.items, z.items)
        self.assertEqual(q.add("wall", [[0, 0], [1, 1]]), "w%d" % (int(mid[1:]) + 1))   # ids carry on
        txt = json.dumps({"format": zm.FORMAT, "items": [{"id": "w1", "kind": "wall", "pts": [[0, 0], [5, 5]]},
                                                         {"id": "n2", "kind": "nogo", "pts": [[0, 0]]},
                                                         {"id": "x", "kind": "lava", "pts": []}, "junk",
                                                         {"id": "m3", "kind": "maze", "walls": [[1, 2, 3]]}]})
        self.assertEqual([i["id"] for i in Zones.from_json(txt).items], ["w1"])
        self.assertEqual(Zones.read("/no/such/file").items, [])
        with self.assertRaises(ValueError):
            Zones.from_json(json.dumps({"format": "other"}))


class TestMapperFiles(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_saved_with_the_map(self):
        m = self.m
        self.assertIsNone(m.zones_command({"action": "add", "kind": "wall", "pts": [[0, 0], [1000, 0]]}))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "autosave.zones.json")))   # autosaved at once
        m.save("flat")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "flat.zones.json")))
        self.assertEqual([x["name"] for x in m.list()], ["flat"])                       # not listed as a map
        m.zones_command({"action": "maze", "x": 0, "y": 2000, "w": 2000, "h": 2000, "cell": 500})
        m.command({"action": "reset"})
        self.assertEqual(m.zones.items, [])
        m.command({"action": "load", "name": "flat"})
        self.assertEqual([i["kind"] for i in m.zones.items], ["wall", "maze"])
        out = [json.loads(s) for s in m.poll(time.monotonic() + 5)]
        zmsg = next(x for x in out if x["t"] == "zones")                                # browsers get them
        self.assertEqual(len(zmsg["items"]), 2)
        self.assertIn("error", json.dumps(m.zones_command({"action": "add", "kind": "nogo", "pts": [[0, 0]]})))
        self.assertIn("error", json.dumps(m.zones_command({"action": "fly"})))
        m.command({"action": "delete", "name": "flat"})
        self.assertFalse(os.path.exists(os.path.join(self.tmp, "flat.zones.json")))

    def test_odometry_frame_in_map_frame_stored(self):
        m = self.m
        m.corr = [100.0, -50.0, 0.0]
        m.zones_command({"action": "add", "kind": "wall", "pts": [[0, 0], [1000, 0]]})
        self.assertEqual(m.zones.items[0]["pts"], [[100.0, -50.0], [1100.0, -50.0]])


class TestRaster(unittest.TestCase):
    def test_layers_in_the_costmap(self):
        g = free_grid()
        z = Zones()
        z.add("wall", [[-500, -1000], [-500, 1000]])
        z.add("nogo", [[500, 500], [1000, 500], [1000, 1000], [500, 1000]])
        z.add("slow", [[500, -1000], [1000, -1000], [1000, -500], [500, -500]], speed=80)
        cm = CostMap(g, DEFAULTS, zones=z.view())
        self.assertEqual(cm.state[cm.cell(-500, 0)], 2)                      # the wall is a wall ...
        self.assertGreaterEqual(cm.cost[cm.cell(-500 + 200, 0)], MARGIN)     # ... inflated (radius + margin = 260)
        self.assertLess(cm.cost[cm.cell(-500 + 600, 0)], MARGIN)
        self.assertEqual(cm.cost[cm.cell(750, 750)], BLOCKED)                # no-go inside
        self.assertGreaterEqual(cm.cost[cm.cell(750, 750 - 400)], MARGIN)    # its edge inflated outwards
        self.assertGreaterEqual(cm.cost[cm.cell(750, -750)], zm.SLOW_COST)   # slow zone costs more ...
        self.assertLess(cm.cost[cm.cell(750, -750)], MARGIN)                 # ... but is passable
        z.add("fence", [[-2000, -2000], [2000, -2000], [2000, 2000], [-2000, 2000]])
        cm = CostMap(g, DEFAULTS, zones=z.view())
        self.assertEqual(cm.cost[cm.cell(2500, 0)], BLOCKED)                 # outside the fence
        self.assertEqual(cm.state[cm.cell(2500, 0)], 2)
        self.assertGreaterEqual(cm.cost[cm.cell(1850, 0)], MARGIN)           # inside, near it
        self.assertLess(cm.cost[cm.cell(0, 1500)], MARGIN)

    def test_costmap_covers_zones_on_unmapped_floor(self):
        g = free_grid(-500, -500, 500, 500)
        z = Zones()
        z.add("wall", [[3000, -500], [3000, 500]])
        cm = CostMap(g, DEFAULTS, zones=z.view())
        self.assertIsNotNone(cm.cell(3000, 0))
        self.assertEqual(cm.state[cm.cell(3000, 0)], 2)


class TestPlannerAvoids(unittest.TestCase):
    def check_both(self, g, z, a, b, segs):
        for name, s, kw in (("normal", DEFAULTS, {}), ("squeeze", squeeze_settings(), {"res_mm": 25.0, "specks": True})):
            cm = CostMap(g, s, zones=z.view(), **kw)
            r = cm.astar(a, b, unknown_ok=True)
            self.assertIsNotNone(r, name)
            self.assertFalse(route_crosses(r, segs), (name, r))
            yield name, r

    def test_wall_across_a_room(self):
        g = free_grid()
        z = Zones()
        z.add("wall", [[0, -3000], [0, 1500]])                       # open only above y 1500
        for name, r in self.check_both(g, z, (-1000, 0), (1000, 0), [(0, -3000, 0, 1500)]):
            self.assertGreater(max(p[1] for p in r), 1500, name)

    def test_squeeze_never_through_a_virtual_wall_in_a_gap(self):
        """A real 400 mm doorway (only squeeze plans fit) closed by a virtual wall: no route at all, not even squeezed."""
        wall_l, wall_r = (0, -3000, 0, -200), (0, 200, 0, 3000)
        g = free_grid(walls=[wall_l, wall_r])
        cm = CostMap(g, squeeze_settings(), res_mm=25.0, specks=True)
        self.assertIsNotNone(cm.astar((-800, 0), (800, 0), unknown_ok=False))         # squeezes through
        z = Zones()
        z.add("wall", [[0, -250], [0, 250]])
        for s, kw in ((DEFAULTS, {}), (squeeze_settings(), {"res_mm": 25.0, "specks": True})):
            cm = CostMap(g, s, zones=z.view(), **kw)
            self.assertIsNone(cm.astar((-800, 0), (800, 0), unknown_ok=True))
            self.assertIsNone(cm.astar((-800, 0), (800, 0), unknown_ok=False))
        z = Zones()
        z.add("nogo", [[-100, -300], [100, -300], [100, 300], [-100, 300]])
        cm = CostMap(g, squeeze_settings(), res_mm=25.0, specks=True, zones=z.view())
        self.assertIsNone(cm.astar((-800, 0), (800, 0), unknown_ok=True))

    def test_nogo_and_fence(self):
        g = free_grid()
        z = Zones()
        box = [[-400, -400], [400, -400], [400, 400], [-400, 400]]
        z.add("nogo", box)
        z.add("fence", [[-1500, -2500], [1500, -2500], [1500, 2500], [-1500, 2500]])
        segs = zm.edges(box)
        for name, r in self.check_both(g, z, (-1000, 0), (1000, 0), segs):
            self.assertTrue(all(abs(p[0]) <= 1500 and abs(p[1]) <= 2500 for p in r), name)
        cm = CostMap(g, DEFAULTS, zones=z.view())
        r = cm.astar((0, -1000), (2500, 0), unknown_ok=True)                # a goal outside: never reached
        self.assertTrue(r is None or all(abs(p[0]) <= 1500 for p in r))

    def test_frontiers_outside_the_fence_or_in_nogo_ignored(self):
        g = free_grid(-1000, -1000, 1000, 1000)                       # unknown beyond 1 m in every direction
        cm = CostMap(g, DEFAULTS)
        self.assertTrue(cm.frontiers((0, 0), 200))
        z = Zones()
        z.add("fence", [[-900, -900], [900, -900], [900, 900], [-900, 900]])
        self.assertEqual(CostMap(g, DEFAULTS, zones=z.view()).frontiers((0, 0), 200), [])
        z = Zones()
        z.add("nogo", [[-2000, 600], [2000, 600], [2000, 2000], [-2000, 2000]])   # the whole top edge
        fr = CostMap(g, DEFAULTS, zones=z.view()).frontiers((0, 0), 200)
        self.assertTrue(fr)
        self.assertTrue(all(goal[1] < 600 for _d, goal, _p, _n in fr))

    def test_planner_refuses_goals_in_zones(self):
        tmp = tempfile.mkdtemp()
        try:
            r = DriftRobot(seed=2)
            m = Mapper(tmp)
            m.zones_command({"action": "add", "kind": "nogo", "pts": [[1000, -300], [1600, -300], [1600, 300], [1000, 300]]})
            b = OfflineBridge(r, m)
            b.update()
            err = b.planner.command({"t": "plan", "action": "goto", "points": [[1300, 0]]})
            self.assertIn("no-go", err["text"])
            self.assertEqual(b.planner.mode, "idle")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestMaze(unittest.TestCase):
    def test_always_solvable(self):
        for seed in range(40):
            for cell, size in ((400, 2000), (500, 3000), (700, 3000), (900, 5000)):
                it = make_maze(0, 0, size, size + 400 * (seed % 3), cell, seed=seed, toward=(seed * 300 - 6000, 2000))
                self.assertTrue(maze_cells_connected(it), (seed, cell))
                self.assertGreaterEqual(it["cols"], 2)
                # a perfect maze: (cells - 1) passages exactly -> walls = all inner edges - passages
                inner = it["cols"] * (it["rows"] - 1) + it["rows"] * (it["cols"] - 1)
                L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in it["walls"]) / cell
                outer = 2 * (it["cols"] + it["rows"]) - 2           # minus entrance and exit gaps
                self.assertAlmostEqual(L, outer + inner - (it["cols"] * it["rows"] - 1), 3)

    def test_planner_solves_at_the_given_cell_width(self):
        """Squeeze A* (radius + squeeze_mm, as the planner falls back to) goes from the entrance through the maze to
        the exit cell, never through a wall and not round the outside (exit gap closed)."""
        g = Grid(10.0, 25.0)
        for seed in range(6):
            for cell in (400, 500, 700):
                z = Zones()
                mid = z.add_maze(0, 0, 2400, 2400, cell, seed=seed, toward=(-3000, 0))
                it = z.find(mid)
                cm = CostMap(g, squeeze_settings(), res_mm=25.0, specks=True, zones=z.view(close_exit=mid))
                r = cm.astar(tuple(it["entry"]), tuple(it["exit"]), unknown_ok=True)
                self.assertIsNotNone(r, (seed, cell))
                segs = [tuple(s) for s in it["walls"]] + [tuple(it["exit_gap"])]
                self.assertFalse(route_crosses(r, segs), (seed, cell))


class FakeLink:
    def __init__(self):
        self.frames = []

    def write(self, f):
        self.frames.append(f)
        return True

    def close(self):
        pass


class TestWatchdog(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.b = b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), map_dir=os.path.join(self.tmp, "maps"),
                            quiet=True, preset_dir=os.path.join(self.tmp, "presets"))
        b.link = FakeLink()
        b.params = {7: {"id": 7, "name": "nav.max_speed", "value": 250.0, "min": 0.0, "max": 1500.0},
                    8: {"id": 8, "name": "cfg.save", "value": 0.0, "min": 0.0, "max": 1.0}}
        b.mapper.zones_command({"action": "add", "kind": "nogo", "pts": [[500, -500], [1500, -500], [1500, 500], [500, 500]]})
        b.mapper.zones_command({"action": "add", "kind": "wall", "pts": [[-500, -1000], [-500, 1000]]})
        b.mapper.zones_command({"action": "add", "kind": "slow", "pts": [[-200, 1000], [400, 1000], [400, 2000], [-200, 2000]],
                                "speed": 90})

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def pose(self, x, y, h=0.0, mode=lp.MODE_GOTO, armed=True, v=0.0):
        self.b.tlm = {"x_mm": x, "y_mm": y, "heading_deg": h, "mode": mode, "speed_mms": v,
                      "status": lp.TLM_ARMED if armed else 0}
        self.b.tlm_time = time.monotonic()

    def sent(self):
        out = []
        p = lp.FrameParser()
        for t, pl in p.feed(b"".join(self.b.link.frames)):
            if t == lp.MSG_SET_MODE:
                out.append(("mode", lp.decode_set_mode(pl)["mode"]))
            elif t == lp.MSG_CMD_VEL:
                out.append(("vel", round(lp.decode_cmd_vel(pl)["speed_mms"])))
            elif t == lp.MSG_EVENT:
                out.append(("event", lp.decode_event(pl)["event"]))
            elif t == lp.MSG_SET_PARAM:
                d = lp.decode_set_param(pl)
                out.append(("param", d["id"], round(d["value"])))
        self.b.link.frames = []
        return out

    def test_autonomous_entering_nogo_holds_never_idles_or_arms(self):
        b = self.b
        self.pose(0, 0)
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])
        self.pose(300, 0)                                       # 200 mm from the zone, footprint 60: fine
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])
        self.pose(470, 0)                                       # footprint touches it
        b._zone_tick(time.monotonic())
        s = self.sent()
        self.assertIn(("mode", lp.MODE_HOLD), s)
        self.assertIn(("vel", 0), s)
        self.assertNotIn(("mode", lp.MODE_IDLE), s)
        self.assertFalse(any(x[0] == "event" for x in s))     # never EVT_LINK_ARM (or anything else)
        self.assertIn("no-go", b.zone_alert)
        self.pose(470, 0, mode=lp.MODE_HOLD)                    # holding: nothing more
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])
        self.pose(475, 0)                                       # replanned, driving out (not deeper): no new stop
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])
        self.pose(530, 0)                                       # 55 mm deeper in: stop again
        b._zone_tick(time.monotonic())
        self.assertIn(("mode", lp.MODE_HOLD), self.sent())

    def test_crossing_a_wall_and_disarmed_or_latched(self):
        b = self.b
        self.pose(-400, 0)
        b._zone_tick(time.monotonic())
        self.sent()
        self.pose(-460, 0, armed=False)                         # disarmed: nothing (motors are off anyway)
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])
        b.zone_hold = {}
        self.pose(-400, 0, mode=lp.MODE_EXPLORE)
        b._zone_tick(time.monotonic())
        self.pose(-640, 0, mode=lp.MODE_EXPLORE)                # jumped across the wall between two frames
        b._zone_tick(time.monotonic())
        self.assertIn(("mode", lp.MODE_HOLD), self.sent())
        self.assertIn("crossed", b.zone_alert)
        b.estop = True                                          # latched: the robot is off; never a mode request
        b.zone_hold = {}
        self.pose(-460, 0)
        b._zone_tick(time.monotonic())
        self.assertEqual(self.sent(), [])

    def test_manual_driving_into_a_zone_is_blocked(self):
        b = self.b
        self.pose(380, 0, h=0.0, mode=lp.MODE_STATION)          # 120 mm short of the zone, facing it
        b.handle_command({"t": "cmd_vel", "speed": 300, "turn": 20}, "test")
        self.assertIn(("vel", 0), self.sent())                  # 300 mm/s x 0.3 s would enter it
        b.handle_command({"t": "cmd_vel", "speed": -300, "turn": 0}, "test")
        self.assertIn(("vel", -300), self.sent())               # backing away is fine
        self.pose(380, 0, h=180.0, mode=lp.MODE_STATION)        # turned round: driving away
        b.handle_command({"t": "cmd_vel", "speed": 300, "turn": 0}, "test")
        self.assertIn(("vel", 300), self.sent())
        self.pose(1000, 0, h=0.0, mode=lp.MODE_MANUAL)          # inside it: out is allowed, deeper is not
        b.handle_command({"t": "cmd_vel", "speed": -200, "turn": 0}, "test")
        self.assertIn(("vel", -200), self.sent())
        self.pose(-300, 0, h=180.0, mode=lp.MODE_STATION)       # towards the virtual wall
        b.handle_command({"t": "cmd_vel", "speed": 400, "turn": 0}, "test")
        self.assertIn(("vel", 0), self.sent())
        b.planner.command({"t": "plan", "action": "set", "settings": {"zones_block_manual": False}})
        b.handle_command({"t": "cmd_vel", "speed": 400, "turn": 0}, "test")
        self.assertIn(("vel", 400), self.sent())                # setting off: not blocked

    def test_slow_zone_caps(self):
        b = self.b
        self.pose(100, 1500, h=90.0, mode=lp.MODE_STATION)
        b.handle_command({"t": "cmd_vel", "speed": 400, "turn": 0}, "test")
        self.assertIn(("vel", 90), self.sent())                 # joystick capped
        self.pose(100, 1500, mode=lp.MODE_GOTO)                 # driving itself: nav.max_speed lowered ...
        b._zone_tick(time.monotonic())
        self.assertIn(("param", 7, 90), self.sent())
        b.handle_command({"t": "cfg", "action": "save"}, "test")
        self.assertEqual(self.sent()[:2], [("param", 7, 250), ("param", 8, 1)])   # never saved capped to EEPROM
        self.pose(100, 1500, mode=lp.MODE_GOTO)
        b._zone_tick(time.monotonic())
        self.assertIn(("param", 7, 90), self.sent())            # capped again
        self.pose(100, 0, mode=lp.MODE_GOTO)                    # ... and put back after
        b._zone_tick(time.monotonic())
        self.assertIn(("param", 7, 250), self.sent())
        self.assertIsNone(b.zone_cap)


class TestSimMaze(unittest.TestCase):
    def test_solve_a_maze_end_to_end(self):
        """A maze generated in front of the sim robot, loaded into the sim as real walls (like --maze): Solve drives
        through it to the exit cell, never through a wall, and the time is kept with the maze."""
        tmp = tempfile.mkdtemp()
        try:
            r = DriftRobot(seed=3)
            r.room = Room(empty=True)
            m = Mapper(tmp)
            m.zones_command({"action": "maze", "x": 1500, "y": 250, "w": 2100, "h": 2100, "cell": 700, "seed": 5},
                            robot_xy=(0.0, 0.0))
            it = m.zones.items[0]
            r.room.set_extra(it["walls"])
            close = [1e9]
            path = []

            def watch(robot, b=None):
                close[0] = min(close[0], robot.room.clearance(robot.X, robot.Y))
                path.append((robot.X, robot.Y))
            s, b = plan_offline(r, m, {"t": "plan", "action": "solve", "maze": it["id"]}, max_s=240, on_step=watch)
            self.assertTrue(b.planner.status.startswith("maze solved in"), (b.planner.status, b.lines[-8:]))
            x, y, _ = r.odom()
            self.assertLess(math.hypot(x - it["exit"][0], y - it["exit"][1]), 200)
            self.assertGreater(close[0], 90)                                     # never touched a wall
            segs = [tuple(w) for w in it["walls"]]
            self.assertFalse(route_crosses(path, segs))
            self.assertIsNotNone(m.zones.items[0]["best_s"])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
