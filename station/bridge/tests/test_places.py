"""Saved places + Go home along a planned route (issue #53). Run: python -m unittest discover -s station -t ."""
import json
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.bridge.places import HOME, MAX_PLACES, Places, clean_name
from station.sim.tour import DriftRobot, drive_offline, plan_offline


class TestPlaces(unittest.TestCase):
    def test_names(self):
        self.assertEqual(clean_name("  Living   room "), "Living room")
        self.assertEqual(clean_name("home"), HOME)
        for bad in ("", "   ", None, "x" * 31):
            with self.assertRaises(ValueError):
                clean_name(bad)

    def test_add_rename_delete(self):
        p = Places()
        p.add("Kitchen", 100, 200)
        with self.assertRaises(ValueError):
            p.add("kitchen", 5, 5)                       # same name, any case
        p.add("Couch", 1, 2)
        self.assertEqual(p.rename("couch", "Sofa"), "Sofa")
        with self.assertRaises(ValueError):
            p.rename("Sofa", "KITCHEN")
        with self.assertRaises(ValueError):
            p.rename("Sofa", "home")                     # Home is special
        with self.assertRaises(ValueError):
            p.rename("nope", "x")
        self.assertEqual(p.delete("sofa"), "Sofa")
        self.assertIsNone(p.find("Sofa"))
        with self.assertRaises(ValueError):
            p.delete("Sofa")
        with self.assertRaises(ValueError):
            p.add("Far", float("nan"), 0)
        for i in range(MAX_PLACES):
            try:
                p.add("p%d" % i, i, i)
            except ValueError:
                break
        with self.assertRaises(ValueError):
            p.add("one too many", 0, 0)

    def test_home(self):
        p = Places()
        self.assertEqual(p.home((7, 8)), (7.0, 8.0, True))          # default: where the pose was reset
        p.origin = [1.0, 2.0]
        self.assertEqual(p.home((7, 8)), (1.0, 2.0, True))
        p.add("HOME", 300, 400)                                      # saving "home" (any case) sets Home
        self.assertEqual(p.home(), (300.0, 400.0, False))
        p.add("Home", 5, 6)                                          # ... and moves it, no "exists" error
        self.assertEqual([x["name"] for x in p.items], [HOME])
        with self.assertRaises(ValueError):
            p.rename("Home", "Base")
        lst = p.listing()
        self.assertTrue(lst[0]["home"] and not lst[0]["default"])
        p.delete("Home")                                             # back to the default
        self.assertEqual(p.listing((7, 8))[0], {"name": HOME, "x": 1, "y": 2, "home": True, "default": True})

    def test_json_roundtrip(self):
        p = Places()
        p.origin = [10.0, -20.0]
        p.add("Kitchen", 1234.56, -789.0)
        p.add("Home", 1, 2)
        q = Places.from_json(p.to_json())
        self.assertEqual(q.origin, [10.0, -20.0])
        self.assertEqual([(x["name"], x["x"], x["y"]) for x in q.items], [("Kitchen", 1234.6, -789.0), ("Home", 1.0, 2.0)])
        with self.assertRaises(ValueError):
            Places.from_json(json.dumps({"format": "other"}))
        # one bad entry does not lose the rest
        txt = json.dumps({"format": "sbr3-places-1", "places": [{"name": "A", "x": 1, "y": 2}, {"name": "", "x": 0, "y": 0},
                                                                  {"x": 1}, {"name": "B", "x": "q", "y": 0}, {"name": "C", "x": 3, "y": 4}]})
        self.assertEqual([x["name"] for x in Places.from_json(txt).items], ["A", "C"])
        self.assertEqual(Places.read("/no/such/file").items, [])


def reading(x=0.0, y=0.0, heading=0.0, angle=0.0, r=1000):
    return {"t_us": 0, "x_mm": x, "y_mm": y, "heading_deg": heading, "pitch_deg": -3.0,
            "sensor_angle_deg": angle, "range_mm": r, "sensor_id": 0, "valid": 1}


class TestMapperPlaces(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def add(self, name, x, y):
        return self.m.places_command({"action": "add", "name": name, "x": x, "y": y})

    def test_saved_with_the_map(self):
        m = self.m
        self.assertIsNone(self.add("Kitchen", 1000, 500))
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "autosave.places.json")))   # unsaved map -> autosave
        m.on_range(reading())
        m.command({"action": "save", "name": "flat"})
        pf = os.path.join(self.tmp, "flat.places.json")
        self.assertTrue(os.path.exists(pf))
        self.assertEqual([x["name"] for x in m.list()], ["flat"])      # the places file is not a map
        self.add("Couch", 2000, 0)                                       # a named map writes at once
        with open(pf, encoding="utf-8") as f:
            self.assertIn("Couch", f.read())
        m.command({"action": "reset"})                                   # a cleared map has no places
        self.assertEqual(m.places.items, [])
        self.assertEqual(m.place_xy("Kitchen"), None)
        m.command({"action": "load", "name": "flat"})
        self.assertEqual(m.place_xy("kitchen"), (1000.0, 500.0))
        self.assertEqual(m.place_xy("Couch"), (2000.0, 0.0))
        out = [json.loads(s) for s in m.poll(time.monotonic() + 5)]
        pl = next(x for x in out if x["t"] == "places")                  # browsers get them after a load
        self.assertEqual([x["name"] for x in pl["items"]], ["Home", "Kitchen", "Couch"])
        self.assertEqual(pl["map"], "flat")
        m.command({"action": "delete", "name": "flat"})
        self.assertFalse(os.path.exists(pf))

    def test_frames_and_errors(self):
        m = self.m
        m.corr = [100.0, 50.0, 90.0]                                     # map = rot90(odometry) + (100, 50)
        self.add("A", 1000, 0)                                           # odometry frame in
        x, y = m.place_xy("A")
        self.assertAlmostEqual(x, 100.0, delta=0.5)
        self.assertAlmostEqual(y, 1050.0, delta=0.5)
        self.assertEqual(m.places_command({"action": "add", "name": "", "x": 0, "y": 0})["t"], "error")
        self.assertEqual(m.places_command({"action": "add", "name": "B", "x": "oops", "y": 0})["t"], "error")
        self.assertEqual(m.places_command({"action": "add", "name": "A", "x": 0, "y": 0})["t"], "error")
        self.assertEqual(m.places_command({"action": "delete", "name": "zzz"})["t"], "error")
        self.assertEqual(m.places_command({"action": "bogus"})["t"], "error")
        self.assertIsNone(m.places_command({"action": "rename", "name": "A", "new": "Alpha"}))
        self.assertIsNone(m.place_xy("A"))

    def test_default_home_is_the_pose_reset(self):
        m = self.m
        self.assertEqual(m.home_xy(), (0.0, 0.0))
        m.corr = [300.0, 0.0, 0.0]
        self.assertEqual(m.home_xy(), (300.0, 0.0))                      # the odometry origin in the map frame
        m.on_pose_reset()
        m.corr = [999.0, 0.0, 0.0]
        self.assertEqual(m.home_xy(), (300.0, 0.0))                      # fixed where it was reset
        self.add("Home", 50, 60)
        self.assertEqual(m.place_xy("home"), (1049.0, 60.0))             # a saved Home wins
        pm = json.loads(m.places_msg())
        self.assertFalse(pm["items"][0]["default"])


class TestGoToPlaces(unittest.TestCase):
    """End to end in the sim room (no network, sim time): map it, save places, drive there around the partition."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.r = DriftRobot(seed=4)
        cls.m = Mapper(cls.tmp)
        drive_offline(cls.r, cls.m.on_range)                    # map the room (the tour ends at 0, 0)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def watch(self):
        close = [1e9]

        def f(robot, b=None):
            close[0] = min(close[0], robot.room.clearance(robot.X, robot.Y))
        return close, f

    def test_a_place_behind_the_partition_then_home(self):
        r, m = self.r, self.m
        self.assertIsNone(m.places_command({"action": "add", "name": "Couch", "x": 2900, "y": 1700}))
        close, watch = self.watch()
        s, b = plan_offline(r, m, {"t": "plan", "action": "goto", "place": "couch"}, max_s=150, on_step=watch)
        self.assertEqual(b.planner.status, "arrived at Couch", b.lines)
        x, y, _ = r.odom()
        self.assertLess(math.hypot(x - 2900, y - 1700), 150)
        self.assertGreater(close[0], 100)                       # went around, never scraped a wall
        # Go home: no Home saved -> the origin (where the pose was reset)
        close, watch = self.watch()
        s, b = plan_offline(r, m, {"t": "plan", "action": "home"}, max_s=150, on_step=watch)
        self.assertEqual(b.planner.status, "arrived at Home", b.lines)
        x, y, _ = r.odom()
        self.assertLess(math.hypot(x, y), 150)
        self.assertGreater(close[0], 100)

    def test_remote_go_home_uses_the_route(self):
        """The remote's GO_HOME (the firmware logs it and drives the straight line): the bridge re-routes it."""
        r, m = self.r, self.m
        plan_offline(r, m, {"t": "plan", "action": "goto", "points": [[2900, 1700]]}, max_s=150)
        close, watch = self.watch()
        s, b = plan_offline(r, m, lambda b: b.planner.robot_go_home((0.0, 0.0)), max_s=150, on_step=watch)
        self.assertEqual(b.planner.status, "arrived at Home", b.lines)
        x, y, _ = r.odom()
        self.assertLess(math.hypot(x, y), 150)
        self.assertGreater(close[0], 100)

    def test_remote_go_home_never_arms_or_overrides_a_latch(self):
        r, m = self.r, self.m
        # latched (station or robot E-stop): nothing is planned
        for attr in ("estop", "robot_estop"):
            def cmd(b, attr=attr):
                setattr(b, attr, True)
                return b.planner.robot_go_home((0.0, 0.0))
            s, b = plan_offline(r, m, cmd, max_s=5)
            self.assertEqual(b.planner.mode, "idle")
            self.assertEqual(b.planner.status, "")
        # disarmed robot: the firmware ignored the request, so nothing is planned (and nothing arms it)
        def cmd(b):
            r.set_mode(lp.MODE_IDLE)
            b.update()
            return b.planner.robot_go_home((0.0, 0.0))
        s, b = plan_offline(r, m, cmd, max_s=5)
        self.assertEqual(b.planner.mode, "idle")
        # an unknown place is an error, not a drive
        self.assertEqual(b.planner.command({"t": "plan", "action": "goto", "place": "Nowhere"})["t"], "error")


class FakeLink:
    def __init__(self):
        self.frames = []

    def write(self, f):
        self.frames.append(f)
        return True


class TestBridgeGoHome(unittest.TestCase):
    """The bridge sees the firmware's "GO_HOME: to x, y" log line and routes the drive."""

    def setUp(self):
        from station.bridge.bridge import Bridge
        self.tmp = tempfile.mkdtemp()
        self.b = Bridge("tcp://127.0.0.1:1", self.tmp, os.path.join(self.tmp, "logs"), quiet=True,
                        map_dir=os.path.join(self.tmp, "maps"))
        self.b.link = FakeLink()
        self.b.planner.sync = True
        self.b.tlm = {"x_mm": 1500.0, "y_mm": 0.0, "heading_deg": 0.0, "mode": lp.MODE_GOTO, "status": lp.TLM_ARMED,
                      "t_us": 0}
        g = self.b.mapper.grid
        for k in range(40):                                    # a little free space around the robot
            g.integrate_wedge(1500, 0, k * 0.157, math.radians(6), 1500, 1600)
        self.b.mapper.grid.take_dirty()

    def tearDown(self):
        self.b.log.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def log(self, text):
        self.b._on_frame(lp.MSG_LOG, text.encode())

    def frames(self):
        return [f for f in self.b.link.frames]

    def test_go_home_is_planned(self):
        b = self.b
        self.log("GO_HOME: to 0, 0")
        self.assertEqual(b.planner.mode, "goto")
        self.assertEqual(b.planner.dest, "Home")
        self.assertTrue(b.link.frames)                          # a MSG_GOTO route went out
        sent = b.waypoints
        self.assertTrue(sent)
        self.assertLess(math.hypot(sent[-1][0], sent[-1][1]), 250)       # the edge of this small test map
        # none of what it sent arms the robot
        self.assertNotIn(lp.encode_event(lp.EVT_LINK_ARM), b.link.frames)

    def test_saved_home_wins(self):
        b = self.b
        b.mapper.places_command({"action": "add", "name": "Home", "x": 700, "y": 100})
        self.log("GO_HOME: to 0, 0")
        sent = b.waypoints
        self.assertLess(math.hypot(sent[-1][0] - 700, sent[-1][1] - 100), 100)

    def test_not_when_latched_or_disarmed_or_no_map(self):
        b = self.b
        b.estop = True
        self.log("GO_HOME: to 0, 0")
        self.assertEqual(b.planner.mode, "idle")
        b.estop = False
        b.robot_estop = True
        self.log("GO_HOME: to 0, 0")
        self.assertEqual(b.planner.mode, "idle")
        b.robot_estop = False
        b.tlm["status"] = 0
        self.log("GO_HOME: to 0, 0")
        self.assertEqual(b.planner.mode, "idle")
        b.tlm["status"] = lp.TLM_ARMED
        b.mapper.reset()
        self.log("GO_HOME: to 0, 0")
        self.assertEqual(b.planner.mode, "idle")                # no map: the straight line stays
        self.log("GO_HOME: nonsense")
        self.assertEqual(b.planner.mode, "idle")

    def test_commands_refused_without_link_or_while_latched(self):
        b = self.b
        b.mapper.places_command({"action": "add", "name": "Couch", "x": 500, "y": 0})
        b.estop = True
        r = b.handle_command({"t": "plan", "action": "home"}, "test")
        self.assertEqual(r["code"], "estop")
        r = b.handle_command({"t": "plan", "action": "goto", "place": "Couch"}, "test")
        self.assertEqual(r["code"], "estop")
        b.estop = False
        b.link = None
        for m in ({"t": "plan", "action": "home"}, {"t": "plan", "action": "goto", "place": "Couch"}):
            self.assertEqual(b.handle_command(m, "test")["code"], "no_link")
        self.assertEqual(b.planner.mode, "idle")
        # places can be edited without the robot
        self.assertIsNone(b.handle_command({"t": "places", "action": "add", "name": "Desk", "x": 0, "y": 0}, "test"))
        self.assertIsNotNone(b.mapper.places.find("Desk"))
        msgs = [json.loads(s) for s in b.mapper.poll(time.monotonic() + 5)]
        self.assertIn("Desk", [x["name"] for x in next(m for m in msgs if m["t"] == "places")["items"]])


if __name__ == "__main__":
    unittest.main()
