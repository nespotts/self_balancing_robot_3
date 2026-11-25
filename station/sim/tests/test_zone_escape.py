"""Zone edges in the simulator (issue #114): the robot touches a drawn maze wall (virtual: nothing the lidar sees), the
bridge watchdog stops it, it backs straight away from the wall and carries on - no "3 times - gave up" loop.
Run: python -m unittest discover -s station -t ."""
import math
import re
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge.mapping import Mapper
from station.bridge.zones import seg_dist
from station.sim.sim import Room
from station.sim.tour import DriftRobot, plan_offline, set_list


class TestZoneEscapeSim(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_touches_a_maze_wall_backs_off_and_solves_it(self):
        r = DriftRobot(seed=3)
        r.room = Room(empty=True)                     # a maze drawn on open floor: only the bridge knows its walls
        m = Mapper(self.tmp)
        m.zones_command({"action": "maze", "x": 1500, "y": 250, "w": 2100, "h": 2100, "cell": 700, "seed": 5},
                        robot_xy=(0.0, 0.0))
        it = m.zones.items[0]
        segs = [tuple(s) for s in it["walls"]]
        st = {"pushed": False, "modes": [], "touch": None, "after": 1e9}

        def step(robot, b):
            x, y, th = robot.odom()
            inside = it["x0"] + 250 < x < it["x0"] + it["cols"] * it["cell"] - 250 and \
                it["y0"] + 250 < y < it["y0"] + it["rows"] * it["cell"] - 250
            if not st["pushed"] and robot.mode == lp.MODE_GOTO and inside:
                # pose error / a cut corner: the robot heads straight on, at the next maze wall
                st["pushed"] = True
                a = math.radians(th)
                set_list(robot, [(x + 2500 * math.cos(a), y + 2500 * math.sin(a))])
            if b.zone_watch() and st["touch"] is None:
                st["touch"] = min(seg_dist(x, y, *s) for s in segs)
            st["modes"].append(robot.mode)
            if st["touch"] is not None and robot.mode == lp.MODE_GOTO and lp.MODE_STATION in st["modes"]:
                st["after"] = min(st["after"], min(seg_dist(x, y, *s) for s in segs))
        s, b = plan_offline(r, m, {"t": "plan", "action": "solve", "maze": it["id"]}, max_s=300, on_step=step)
        stops = getattr(b, "zone_stops", [])
        self.assertTrue(st["pushed"])
        self.assertGreaterEqual(len(stops), 1, b.lines[-10:])          # it did touch a maze wall
        self.assertLessEqual(len(stops), 2, stops)                     # ... and did not keep hitting it
        self.assertIn(lp.MODE_STATION, st["modes"])                    # backed straight away (cmd_vel)
        self.assertTrue(any("backing" in ln for ln in b.lines), b.lines[-10:])
        self.assertFalse(any("gave up" in ln for ln in b.lines))
        self.assertTrue(b.planner.status.startswith("maze solved in"), (b.planner.status, b.lines[-10:]))
        self.assertGreater(st["after"], st["touch"])                   # off the wall once it drove on

    def test_set_position_at_a_wall_escapes_first(self):
        """A pose that starts with the footprint over a virtual wall ahead (Set position / reloc): the plan backs off
        it first (no watchdog stop at all), then drives the route."""
        r = DriftRobot(seed=4)
        r.room = Room(empty=True)
        m = Mapper(self.tmp)
        modes, goal = [], []

        def start(b):
            # a virtual wall square across the robot's heading, 40 mm ahead: its footprint (60 mm) is over it
            x, y, th = r.odom()
            a = math.radians(th)
            fx, fy, lx, ly = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
            wx, wy = x + 40 * fx, y + 40 * fy
            m.zones_command({"action": "add", "kind": "wall",
                             "pts": [[wx - 1500 * lx, wy - 1500 * ly], [wx + 1500 * lx, wy + 1500 * ly]]})
            goal.append((x - 900 * fx + 700 * lx, y - 900 * fy + 700 * ly))   # behind it, to the left
            return b.planner.command({"t": "plan", "action": "goto", "points": [list(goal[0])]}, "test")

        def step(robot, b):
            b.zone_watch()
            modes.append(robot.mode)
        s, b = plan_offline(r, m, start, max_s=120, on_step=step)
        self.assertEqual(getattr(b, "zone_stops", []), [])
        self.assertIn(lp.MODE_STATION, modes)
        self.assertTrue(any(re.match(r"planner: backing \d+ mm straight away from the zone edge", ln) for ln in b.lines),
                        b.lines[-10:])
        self.assertEqual(b.planner.status, "arrived", b.lines[-10:])
        x, y, _ = r.odom()
        self.assertLess(math.hypot(x - goal[0][0], y - goal[0][1]), 200)


if __name__ == "__main__":
    unittest.main()
