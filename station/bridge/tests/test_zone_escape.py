"""Zone edges: direction-aware watchdog + escape first (issue #114). Run: python -m unittest discover -s station -t ."""
import math
import os
import shutil
import tempfile
import time
import unittest

from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.tests.test_hardening import WEB
from station.bridge.tests.test_zones import FakeLink, free_grid
from station.bridge.zones import Zones, ZoneView

FP = 60.0


def view(*items):
    z = Zones()
    for kind, pts in items:
        z.add(kind, pts)
    return z.view()


class TestDirection(unittest.TestCase):
    def setUp(self):
        self.v = view(("wall", [[0, -1000], [0, 1000]]))        # a wall along y at x = 0

    def test_into_along_and_away(self):
        v = self.v
        self.assertEqual(v.into(-40, 0, 1, 0, FP), {v.walls[0][0]})     # towards the wall
        self.assertEqual(v.into(-40, 0, 0, 1, FP), set())                # along it
        self.assertEqual(v.into(-40, 0, -1, 0, FP), set())               # away
        self.assertEqual(v.into(-40, 0, math.cos(math.radians(30)), math.sin(math.radians(30)), FP),
                         {v.walls[0][0]})                                 # slanting in

    def test_watch_stops_only_motion_into(self):
        v = self.v
        why, hold = v.watch(-40, 0, 0.0, 150.0, FP, (-46, 0), {}, 25.0)          # driving at it
        self.assertEqual(why, "at a virtual wall")
        why, hold = v.watch(-40, 0, 180.0, 150.0, FP, (-34, 0), {}, 25.0)        # driving away from it (forward)
        self.assertIsNone(why)
        why, hold = v.watch(-40, 0, 0.0, -150.0, FP, (-34, 0), {}, 25.0)         # backing away (heading at it)
        self.assertIsNone(why)
        why, hold = v.watch(-40, 0, 90.0, 150.0, FP, (-40, -6), {}, 25.0)        # along it
        self.assertIsNone(why)
        why, hold = v.watch(-40, 0, 0.0, 0.0, FP, (-40, 0), {}, 25.0)            # standing at it
        self.assertIsNone(why)
        why, hold = v.watch(-40, 0, 0.0, 150.0, FP, (-46, 0), {}, 25.0, active=False)   # not GOTO / EXPLORE
        self.assertIsNone(why)
        # slow: the direction comes from the pose change
        why, _ = v.watch(-40, 0, 90.0, 5.0, FP, (-45, 0), {}, 25.0)
        self.assertEqual(why, "at a virtual wall")

    def test_watch_crossing_and_hold(self):
        v = self.v
        why, hold = v.watch(30, 0, 180.0, -100.0, FP, (-30, 0), {}, 25.0)        # crossed it (any direction)
        self.assertTrue(why.startswith("crossed"))
        why, hold = v.watch(-40, 0, 0.0, 150.0, FP, (-46, 0), {}, 25.0)
        self.assertTrue(why)
        why, hold = v.watch(-30, 0, 0.0, 150.0, FP, (-40, 0), hold, 25.0)        # 10 mm deeper: no new stop
        self.assertIsNone(why)
        why, hold = v.watch(-10, 0, 0.0, 150.0, FP, (-30, 0), hold, 25.0)        # 30 mm deeper than at the stop
        self.assertTrue(why)

    def test_pose_jump_into_a_zone_is_an_escape_not_a_crossing(self):
        v = view(("nogo", [[0, 0], [1000, 0], [1000, 1000], [0, 1000]]))
        why, _ = v.watch(500, 500, 0.0, 0.0, FP, (-800, 500), {}, 25.0)          # Set position / reloc landed inside
        self.assertTrue(why.startswith("pose moved"), why)


class TestEscape(unittest.TestCase):
    def test_not_needed_away_from_edges(self):
        v = view(("wall", [[0, -1000], [0, 1000]]))
        self.assertIsNone(v.escape(-300, 0, 0.0, FP))
        self.assertIsNone(v.escape(-140, 0, 0.0, FP))           # 80 mm clear of the footprint: no escape

    def test_wall_ahead_backs_straight_away(self):
        v = view(("wall", [[0, -1000], [0, 1000]]))
        e = v.escape(-40, 0, 20.0, FP)                          # facing the wall, 20 deg off square
        self.assertTrue(e["reverse"])
        h = math.radians(20.0)
        self.assertAlmostEqual(e["x"], -40 - e["dist"] * math.cos(h), delta=0.2)   # straight back along the heading
        self.assertAlmostEqual(e["y"], -e["dist"] * math.sin(h), delta=0.2)
        self.assertGreaterEqual(-e["x"], FP + 60 - 1)          # footprint clear by ESCAPE_CLEAR_MM
        self.assertEqual(e["why"], "at a virtual wall")

    def test_wall_beside_or_behind_drives_out_forward(self):
        v = view(("wall", [[0, -1000], [0, 1000]]))
        e = v.escape(-40, 0, 90.0, FP)                          # wall at its right side
        self.assertFalse(e["reverse"])
        self.assertLess(e["x"], -FP - 59)
        self.assertAlmostEqual(e["y"], 0.0, delta=1.0)          # straight off the wall
        e = v.escape(-40, 0, 180.0, FP)                         # wall behind it: forward is the way out
        self.assertFalse(e["reverse"])

    def test_out_of_a_nogo_zone_by_the_nearest_edge(self):
        v = view(("nogo", [[0, 0], [1000, 0], [1000, 1000], [0, 1000]]))
        e = v.escape(950, 500, 0.0, FP)                        # inside, near the east edge, facing it
        self.assertGreater(e["x"], 1000 + FP + 59)
        self.assertFalse(e["reverse"])                         # the way out is ahead
        self.assertIsNone(v.point_blocked(e["x"], e["y"]))

    def test_fence_escape_goes_inwards(self):
        v = view(("fence", [[0, 0], [2000, 0], [2000, 2000], [0, 2000]]))
        e = v.escape(1000, 40, -90.0, FP)                      # at the south fence edge, facing it
        self.assertTrue(e["reverse"])
        self.assertGreater(e["y"], FP + 59)

    def test_maze_corner_and_boxed_in(self):
        v = view(("wall", [[0, 0], [0, 1000]]), ("wall", [[0, 0], [1000, 0]]))
        e = v.escape(40, 40, 225.0, FP)                        # in a corner, facing it
        self.assertTrue(e and not e.get("boxed"))
        self.assertGreater(min(e["x"], e["y"]), FP + 59)
        # a box smaller than the footprint + clear band: no straight way out
        v = view(("wall", [[-50, -50], [50, -50], [50, 50], [-50, 50], [-50, -50]]))
        e = v.escape(0, 0, 0.0, FP)
        self.assertTrue(e["boxed"])

    def test_never_through_a_wall_or_into_an_obstacle(self):
        v = view(("wall", [[0, -1000], [0, 1000]]))
        e = v.escape(-40, 0, 0.0, FP, ok=lambda x, y: y > 50)    # straight back is a real obstacle: angled out
        self.assertTrue(e["y"] > 50)
        self.assertLess(e["x"], 0)


class TestPlannerEscape(unittest.TestCase):
    """Planner side with a real Bridge (no robot): escape-first plans, zone_stop -> escape, boxed in."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.b = b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), map_dir=os.path.join(self.tmp, "maps"),
                            quiet=True, preset_dir=os.path.join(self.tmp, "presets"))
        b.link = FakeLink()
        b.mapper.grid = free_grid()
        b.planner.sync = True
        b.mapper.zones_command({"action": "add", "kind": "wall", "pts": [[0, -1000], [0, 1000]]})

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def pose(self, x, y, h=0.0, mode=lp.MODE_GOTO, v=0.0):
        self.b.tlm = {"x_mm": x, "y_mm": y, "heading_deg": h, "mode": mode, "speed_mms": v, "status": lp.TLM_ARMED}
        self.b.tlm_time = time.monotonic()

    def modes(self):
        out = []
        for t, pl in lp.FrameParser().feed(b"".join(self.b.link.frames)):
            if t == lp.MSG_SET_MODE:
                out.append(lp.decode_set_mode(pl)["mode"])
            elif t == lp.MSG_CMD_VEL:
                out.append(round(lp.decode_cmd_vel(pl)["speed_mms"]))
        self.b.link.frames = []
        return out

    def test_plan_starting_at_a_wall_escapes_first(self):
        p = self.b.planner
        self.pose(-40, 0, h=180.0)                              # wall behind: drive out forward, then the route
        res = p._plan_goto(self.b.mapper.map_pose(-40, 0, 180.0))
        self.assertEqual(res[0], "path")
        route = res[1]
        self.assertLess(route[1][0], -FP - 59)                  # first: straight off the wall
        self.assertAlmostEqual(route[1][1], 0.0, delta=1.0)
        self.assertEqual(p._esc_now["drive"], "ahead")
        p.goals = [(-1500, 500)]
        self.pose(-300, 0)
        res = p._plan_goto(self.b.mapper.map_pose(-300, 0, 0.0))
        self.assertIsNone(p._esc_now)                           # clear of it: a plain route

    def test_zone_stop_backs_away_then_drives_the_route(self):
        b, p = self.b, self.b.planner
        self.pose(-300, 0, h=0.0, mode=lp.MODE_BALANCE)
        p.command({"t": "plan", "action": "goto", "points": [[-300, 900]]})
        self.modes()
        self.pose(-40, 0, h=0.0, v=120.0)                       # drove into the wall (pose noise, a corner cut)
        b.zone_prev = (-46, 0)
        b._zone_tick(time.monotonic())
        s = self.modes()
        self.assertIn(lp.MODE_HOLD, s)                          # the watchdog's stop ...
        self.assertEqual(s[-2:], [lp.MODE_STATION, -80])        # ... then at once straight back with cmd_vel
        self.assertIn("backing 150 mm straight away from it", p.status)
        self.assertIsNotNone(p.backing)
        self.assertEqual(p.path, [])
        self.pose(-30, 0, h=0.0, mode=lp.MODE_STATION, v=200.0)  # still rolling on (braking): no new stop
        b._zone_tick(time.monotonic())
        p.tick(time.monotonic())
        self.assertEqual(self.modes(), [-80])
        self.pose(-40 - 150, 0, h=0.0, mode=lp.MODE_STATION, v=-80.0)
        p.tick(time.monotonic())                                # backed off far enough: the route from there, GOTO
        self.assertIsNone(p.backing)
        s = self.modes()
        self.assertIn(0, s)
        self.assertIn(lp.MODE_GOTO, s)
        self.assertEqual(p.mode, "goto")
        self.assertTrue(p.path and p.path[-1] == (-300.0, 900.0), p.path)
        self.assertIsNone(p._esc_now)                           # clear of the wall: no second escape
        self.assertEqual(p.zone_hits, 1)

    def test_zone_stop_beside_a_wall_replans_with_a_turn_escape(self):
        b, p = self.b, self.b.planner
        self.pose(-300, 0, h=0.0, mode=lp.MODE_BALANCE)
        p.command({"t": "plan", "action": "goto", "points": [[-300, 900]]})
        self.pose(-40, 0, h=85.0, v=100.0)                      # along the wall, slanting in by 5 deg
        p.zone_stop("at a virtual wall")
        self.assertIsNone(p.backing)
        self.assertIn("moving away from it", p.status)
        p.replan_at = time.monotonic() - 0.1                    # (ZONE_SETTLE_S has passed)
        self.pose(-40, 0, h=85.0, v=0.0, mode=lp.MODE_HOLD)
        p.tick(time.monotonic())                                # standing still: not yet (0.4 s still first)
        self.assertIsNotNone(p.replan_at)
        p.zone_still -= 1.0
        p.tick(time.monotonic())
        self.assertIsNone(p.replan_at)
        self.assertEqual(p._esc_now["drive"], "turn")
        self.assertLess(p.path[1][0], -FP - 59)                 # first off the wall (turn, then drive), then on
    def test_boxed_in_gives_up_with_a_clear_message(self):
        b, p = self.b, self.b.planner
        b.mapper.zones_command({"action": "add", "kind": "wall", "pts": [[-100, -50], [-100, 50], [100, 50]]})
        b.mapper.zones_command({"action": "add", "kind": "wall", "pts": [[100, 50], [100, -50], [-100, -50]]})
        self.pose(-300, 0, h=0.0, mode=lp.MODE_BALANCE)
        p.command({"t": "plan", "action": "goto", "points": [[-300, 900]]})
        self.pose(-50, 0, h=0.0)                                # (as if it got in there)
        p.zone_stop("at a virtual wall")
        self.assertEqual(p.mode, "idle")
        self.assertIn("boxed in by zones - edit them", p.status)

    def test_no_give_up_after_three_stops(self):
        b, p = self.b, self.b.planner
        self.pose(-300, 0, h=0.0, mode=lp.MODE_BALANCE)
        p.command({"t": "plan", "action": "goto", "points": [[-300, 900]]})
        self.pose(-40, 0, h=0.0)
        for _ in range(4):
            p.zone_stop("at a virtual wall")
        self.assertEqual(p.mode, "goto")
        self.assertNotIn("gave up", p.status)


if __name__ == "__main__":
    unittest.main()
