"""Bump marks (issue #104): MSG_CONTACT from the navigator -> an "invisible obstacle" mark (zones kind "bump") that the
planner keeps off, saved with the map, listed and cleared. Run: python -m unittest discover -s station -t ."""
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
from station.bridge.planner import BLOCKED, MARGIN, CostMap, DEFAULTS
from station.bridge.tests.test_hardening import WEB
from station.bridge.tests.test_zones import FakeLink, free_grid
from station.bridge.zones import Zones


def contact(kind=lp.LINK_CONTACT_BUMP, x=1000.0, y=0.0, h=0.0, seq=1, flags=0, value=110.0):
    return {"kind": kind, "seq": seq, "mode": lp.MODE_GOTO, "flags": flags, "x_mm": x, "y_mm": y, "heading_deg": h,
            "value": value}


class TestFrame(unittest.TestCase):
    def test_round_trip(self):
        f = lp.encode_contact(contact(lp.LINK_CONTACT_STUCK, 1234.5, -67.0, 90.0, seq=300, flags=lp.LINK_CONTACT_LIDAR,
                                      value=420.0))
        out = lp.FrameParser().feed(f)
        self.assertEqual(out[0][0], lp.MSG_CONTACT)
        d = lp.decode(*out[0])
        self.assertEqual((d["kind"], d["seq"], d["flags"], d["name"]), (lp.LINK_CONTACT_STUCK, 300 & 0xFF,
                                                                        lp.LINK_CONTACT_LIDAR, "stuck"))
        self.assertAlmostEqual(d["x_mm"], 1234.5, places=3)
        self.assertAlmostEqual(d["heading_deg"], 90.0, places=3)
        self.assertEqual(lp.CONTACT_SIZE, 20)
        self.assertGreaterEqual(lp.LINK_PROTOCOL_VERSION, 8)   # MSG_CONTACT since v8 (v9: #109 reset cause)


class TestMarks(unittest.TestCase):
    def test_add_merge_cap(self):
        z = Zones()
        bid, merged = z.add_bump(1000.0, 0.0, 0.0)
        b = z.find(bid)
        self.assertFalse(merged)
        self.assertEqual((b["kind"], b["hits"], b["r"]), ("bump", 1, zm.BUMP_R_MM))
        self.assertAlmostEqual(b["x"], 1000.0 + zm.BUMP_AHEAD_MM)          # past the contact point, along the heading
        self.assertEqual(len(b["pts"]), zm.BUMP_SIDES)
        self.assertTrue(all(abs(math.hypot(p[0] - b["x"], p[1] - b["y"]) - zm.BUMP_R_MM) < 0.2 for p in b["pts"]))
        self.assertEqual(z.history, [])                                     # the robot's marks are no undo steps
        bid2, merged = z.add_bump(1030.0, 40.0, 0.0, what="stuck")          # the same obstacle again
        self.assertTrue(merged)
        self.assertEqual(bid2, bid)
        self.assertEqual(z.find(bid)["hits"], 2)
        self.assertEqual(len(z.bumps()), 1)
        z.add_bump(1500.0, 0.0, 90.0)                                       # another one
        self.assertEqual(len(z.bumps()), 2)
        for k in range(zm.MAX_BUMPS + 5):                                   # the oldest go first
            z.add_bump(10000.0 + 300.0 * k, 0.0)
        self.assertEqual(len(z.bumps()), zm.MAX_BUMPS)
        self.assertIsNone(z.find(bid))
        for k in range(zm.MAX_ITEMS):                                       # drawn zones have their own limit
            z.add("wall", [[0, 1000 * k], [500, 1000 * k]])
        with self.assertRaises(ValueError):
            z.add("wall", [[0, -100], [500, -100]])
        with self.assertRaises(ValueError):
            z.add("bump", [[0, 0], [100, 0], [100, 100]])                   # not drawn: add_bump / zones add x, y
        with self.assertRaises(ValueError):
            z.update(z.bumps()[0]["id"], pts=[[0, 0], [100, 0], [100, 100]])

    def test_clear_undo_json(self):
        z = Zones()
        w = z.add("wall", [[0, 0], [1000, 0]])
        b1, _ = z.add_bump(500.0, 500.0)
        z.add("nogo", [[0, 0], [500, 0], [500, 500], [0, 500]])
        b2, _ = z.add_bump(2000.0, 500.0)                                   # after the no-go zone
        z.undo()                                                            # undo the no-go: the marks stay
        self.assertEqual({b["id"] for b in z.bumps()}, {b1, b2})
        z.clear("bump")                                                     # "Clear bump marks"
        self.assertEqual(z.bumps(), [])
        self.assertIsNotNone(z.find(w))
        z.undo()                                                            # ... undone
        self.assertEqual({b["id"] for b in z.bumps()}, {b1, b2})
        q = Zones.from_json(z.to_json())
        self.assertEqual(q.items, z.items)
        bid, _ = q.add_bump(9000.0, 0.0)
        self.assertNotIn(bid, (b1, b2, w))                                  # ids carry on
        bad = json.dumps({"format": zm.FORMAT, "items": [{"id": "b1", "kind": "bump", "x": "x", "y": 0},
                                                         {"id": "b2", "kind": "bump", "x": 5, "y": 6, "r": 9999}]})
        items = Zones.from_json(bad).items
        self.assertEqual([i["id"] for i in items], ["b2"])
        self.assertEqual(items[0]["r"], 500.0)                              # clamped, polygon rebuilt
        self.assertEqual(len(items[0]["pts"]), zm.BUMP_SIDES)


class TestPlanner(unittest.TestCase):
    def test_costmap_and_route(self):
        g = free_grid()
        z = Zones()
        z.add_bump(0.0, 0.0)
        cm = CostMap(g, DEFAULTS, zones=z.view())
        self.assertGreaterEqual(cm.cost[cm.cell(zm.BUMP_AHEAD_MM, 0.0)], BLOCKED)     # a wall ...
        r = zm.BUMP_R_MM + DEFAULTS["robot_radius_mm"] + DEFAULTS["margin_mm"]
        self.assertGreaterEqual(cm.cost[cm.cell(zm.BUMP_AHEAD_MM, r - 60.0)], MARGIN)  # ... inflated like one
        self.assertLess(cm.cost[cm.cell(zm.BUMP_AHEAD_MM, r + 80.0)], MARGIN)
        route = cm.astar((-1500.0, 0.0), (1500.0, 0.0), unknown_ok=True)
        self.assertIsNotNone(route)
        closest = min(zm.seg_dist(zm.BUMP_AHEAD_MM, 0.0, a[0], a[1], b[0], b[1]) for a, b in zip(route, route[1:]))
        self.assertGreater(closest, zm.BUMP_R_MM + DEFAULTS["robot_radius_mm"])        # round it, with room
        zv = z.view()
        self.assertEqual(zv.depths(zm.BUMP_AHEAD_MM, 0.0, 80.0), {})        # the watchdog never stops it at one


class TestMapper(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.m = Mapper(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_contact_saved_listed_cleared(self):
        m = self.m
        m.corr = [100.0, -50.0, 90.0]                                       # odometry -> map frame
        mark, merged = m.add_contact(lp.decode_contact(lp.encode_contact(contact(x=1000.0, y=0.0, h=0.0))[4:-2]))
        self.assertFalse(merged)
        self.assertAlmostEqual(mark["x"], 100.0, delta=0.5)                 # rotated 90 deg: (1000, 0) -> (100, 950)
        self.assertAlmostEqual(mark["y"], -50.0 + 1000.0 + zm.BUMP_AHEAD_MM, delta=0.5)
        self.assertEqual(mark["what"], "bump")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "autosave.zones.json")))   # saved at once
        out = [json.loads(s) for s in m.poll(time.monotonic() + 5)]
        zmsg = next(x for x in out if x["t"] == "zones")                    # browsers get it (listed)
        self.assertEqual([i["kind"] for i in zmsg["items"]], ["bump"])
        m.save("hall")
        m.command({"action": "reset"})
        self.assertEqual(m.zones.bumps(), [])
        m.command({"action": "load", "name": "hall"})
        self.assertEqual(len(m.zones.bumps()), 1)                           # kept with the map
        self.assertIsNone(m.zones_command({"action": "add", "kind": "bump", "x": 0, "y": 2000}))   # by hand
        self.assertEqual(len(m.zones.bumps()), 2)
        self.assertIsNone(m.zones_command({"action": "clear", "kind": "bump"}))                    # the API
        self.assertEqual(m.zones.bumps(), [])
        self.assertIn("error", json.dumps(m.zones_command({"action": "add", "kind": "bump", "x": "?"})))


class TestBridge(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.b = b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), map_dir=os.path.join(self.tmp, "maps"),
                            quiet=True)
        b.link = FakeLink()
        self.sent = []
        b.broadcast = lambda text, droppable=False: self.sent.append(text)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_msg_contact_marks_and_replans(self):
        b = self.b
        calls = []
        b.planner.on_contact = calls.append
        b._on_frame(lp.MSG_CONTACT, lp.encode_contact(contact(lp.LINK_CONTACT_STUCK, 800.0, 300.0, 0.0))[4:-2])
        marks = b.mapper.zones.bumps()
        self.assertEqual(len(marks), 1)
        self.assertEqual(calls, marks)                                      # the planner replans round it
        ev = [json.loads(t) for t in self.sent if '"contact"' in t]
        self.assertEqual(ev[0]["kind"], "stuck")
        self.assertEqual(ev[0]["mark"], marks[0]["id"])
        self.assertTrue(any("pushing against something" in l["text"] for l in b.logs))
        b._on_frame(lp.MSG_CONTACT, lp.encode_contact(contact(x=820.0, y=310.0))[4:-2])   # again: the same mark
        self.assertEqual(len(b.mapper.zones.bumps()), 1)
        self.assertEqual(b.mapper.zones.bumps()[0]["hits"], 2)
        self.assertIsNone(b.handle_command({"t": "zones", "action": "clear", "kind": "bump"}))
        self.assertEqual(b.mapper.zones.bumps(), [])

    def test_planner_waits_then_replans(self):
        b = self.b
        p = b.planner
        p.mode = "goto"
        p.on_contact({"id": "b1"})
        self.assertIsNotNone(p.replan_at)
        self.assertIn("bump mark b1", p.status)
        p.mode = "idle"
        p.replan_at = None
        p.on_contact({"id": "b2"})                                          # no job: nothing to replan
        self.assertIsNone(p.replan_at)


class TestLoop(unittest.TestCase):
    """#111: stopped again and again at one spot (BLOCKED / stuck / contact, back off, the same route, again): the planner
    marks it, gives that explore goal up and plans another way."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.b = b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), map_dir=os.path.join(self.tmp, "maps"),
                            quiet=True)
        b.link = FakeLink()
        b.broadcast = lambda text, droppable=False: None
        self.now = [100.0]
        self.p = p = b.planner
        p.clock = lambda: self.now[0]
        p.mode, p.goal = "explore", (1500.0, 900.0)
        self.at(-200.0, 250.0, -130.0)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def at(self, x, y, h):
        self.b.tlm = {"x_mm": x, "y_mm": y, "heading_deg": h, "mode": lp.MODE_EXPLORE, "status": 0}

    def later(self, s=2.0):
        self.now[0] += s

    def test_block_then_rear_contact_marks_the_spot(self):
        p, b = self.p, self.b
        p.on_log("nav: blocked, obstacle at 70 mm (waypoint 1/5)")       # (the 2026-11-25 loop at -0.2, 0.25 m)
        p.on_log("nav: obstacle 70 mm ahead, too close for the lidar: backing off")
        self.assertEqual(b.mapper.zones.bumps(), [])
        self.assertEqual(p.blacklist, [])
        self.later(0.8)
        self.at(-205.0, 256.0, -130.0)                                     # the back-off was blocked: a rear contact
        b._on_frame(lp.MSG_CONTACT, lp.encode_contact(contact(lp.LINK_CONTACT_STUCK, -170.0, 294.0, 50.0))[4:-2])
        marks = b.mapper.zones.bumps()
        self.assertEqual(len(marks), 2)                                    # the contact's own + the loop mark ahead
        ahead = min(marks, key=lambda m: math.hypot(m["x"] - (-200.0 + 109.0 * math.cos(math.radians(-130.0))),
                                                    m["y"] - (250.0 + 109.0 * math.sin(math.radians(-130.0)))))
        self.assertLess(math.hypot(ahead["x"] - (-200.0 - 70.0 - 23.0), ahead["y"] - (250.0 - 83.0 - 27.0)), 60.0)
        self.assertEqual(p.blacklist, [(1500.0, 900.0)])                   # that goal is given up
        self.assertEqual(p.boost, 100.0)                                   # more margin
        self.assertIsNotNone(p.replan_at)
        self.assertIn("same spot", p.status)
        self.assertEqual(p.loop_marks, 1)

    def test_three_blocks_and_what_does_not_count(self):
        p, b = self.p, self.b
        for k in range(2):
            p.on_log("nav: blocked, obstacle at 96 mm (waypoint 1/5)")
            self.later()
        self.assertEqual(p.loop_marks, 0)                                  # 2 blocks: not yet
        self.at(-200.0, 450.0, -130.0)                                     # 200 mm away: another spot
        p.on_log("nav: blocked, obstacle at 96 mm (waypoint 1/5)")
        self.later()
        self.assertEqual(p.loop_marks, 0)
        self.at(-190.0, 260.0, -125.0)                                     # back at the first one: the 3rd there
        p.on_log("nav: blocked, obstacle at 44 mm (waypoint 1/5)")
        self.assertEqual(p.loop_marks, 1)
        self.assertEqual(len(b.mapper.zones.bumps()), 1)
        self.later(40.0)                                                   # long after: they have expired
        for k in range(2):
            p.on_log("nav: blocked, obstacle at 96 mm (waypoint 1/5)")
            self.later()
        self.assertEqual(p.loop_marks, 1)
        p.on_log("nav: stuck, no progress driving for 2.5 s (waypoint 1/5)")   # 2 blocks + a stuck: 3
        self.assertEqual(p.loop_marks, 2)

    def test_one_stuck_contact_is_one_stop(self):
        p = self.p
        p.on_log("nav: stuck, no progress driving for 2.5 s (waypoint 1/5)")
        self.b._on_frame(lp.MSG_CONTACT, lp.encode_contact(contact(lp.LINK_CONTACT_STUCK, -140.0, 300.0, -125.0))[4:-2])
        self.assertEqual(p.loop_marks, 0)                                  # one stop, reported twice
        self.assertEqual(len(p.spots), 1)

    def test_idle_or_no_pose(self):
        p = self.p
        p.mode = "idle"
        for _ in range(4):
            p.on_log("nav: blocked, obstacle at 96 mm (waypoint 1/5)")
            self.later()
        self.assertEqual(p.spots, [])
        p.mode, self.b.tlm = "goto", None
        p.on_log("nav: blocked, obstacle at 96 mm (waypoint 1/5)")
        self.assertEqual(p.spots, [])


if __name__ == "__main__":
    unittest.main()
