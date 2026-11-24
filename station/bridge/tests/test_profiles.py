"""Per-robot profiles (#97): identity from HELLO + params, the old flat layout migrated into balancer/, separate
maps / places / zones / lidar + planner settings / presets / falls per robot, nothing written before the robot is known,
and the sim on its own profile."""
import asyncio
import json
import os
import shutil
import tempfile
import time
import unittest
from unittest import mock

from station.bridge import link_protocol as lp
from station.bridge import profiles
from station.bridge.bridge import AUTOSAVE_NAME, Bridge
from station.bridge.tests.test_bridge_sim import until
from station.sim.sim import SimRobot, SimServer

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")


def tree(root):
    """{relative path: bytes} of every file under root."""
    out = {}
    for d, _dirs, files in os.walk(root):
        for f in files:
            p = os.path.join(d, f)
            with open(p, "rb") as fh:
                out[os.path.relpath(p, root).replace("\\", "/")] = fh.read()
    return out


class TestIdentify(unittest.TestCase):
    def test_platform_param_first(self):
        fw = {"firmware": "v4 2026-11-24"}
        self.assertEqual(profiles.identify(fw, {"sys.platform": 1.0}), ("balancer", "sys.platform"))
        self.assertEqual(profiles.identify(fw, {"sys.platform": 2.0}), ("shark", "sys.platform"))
        self.assertEqual(profiles.identify(fw, {"sys.platform": 7.0})[0], "platform7")

    def test_firmware_and_default(self):
        self.assertEqual(profiles.identify({"firmware": "v4 2026-11-24"}, {}), ("balancer", "default"))
        self.assertEqual(profiles.identify({"firmware": "v4"}, {"sys.platform": 0.0})[0], "balancer")
        self.assertEqual(profiles.identify(None, None)[0], "balancer")

    def test_sim_has_its_own_profile(self):
        self.assertEqual(profiles.identify({"firmware": "sim v4 2026-11-20"}, {}), ("sim", "firmware"))
        self.assertEqual(profiles.identify({"firmware": "sim v4"}, {"sys.platform": 1.0})[0], "sim")
        self.assertEqual(profiles.identify({"firmware": "sim v4"}, {"sys.platform": 2.0})[0], "sim-shark")

    def test_names(self):
        self.assertEqual(profiles.clean_name(" Shark "), "shark")
        for bad in ("", "a/b", "..", "x" * 40, "-x", "a b"):
            self.assertIsNone(profiles.clean_name(bad), bad)


class TestMigration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.maps = os.path.join(self.tmp, "sbr3-maps")
        os.makedirs(self.maps)
        self.files = {"living.json": b'{"grid": 1}', "living.png": b"\x89PNG-fake", "living.places.json": b'{"p": 1}',
                      "living.zones.json": b'{"z": 1}', "settings.json": b'{"cone_deg": 9}',
                      "planner.json": b'{"robot_radius_mm": 91}', "lidar.json": b'{"yaw_deg": 12}',
                      "autosave.json": b"x" * 5000}
        for n, data in self.files.items():
            with open(os.path.join(self.maps, n), "wb") as f:
                f.write(data)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_flat_files_move_into_balancer(self):
        os.makedirs(os.path.join(self.maps, "shark"))                       # another robot's folder: untouched
        with open(os.path.join(self.maps, "shark", "keep.json"), "wb") as f:
            f.write(b"shark")
        said = []
        r = profiles.migrate_flat(self.maps, say=said.append)
        self.assertEqual(r["moved"], len(self.files))
        self.assertEqual(r["failed"], [])
        left = [n for n in os.listdir(self.maps) if os.path.isfile(os.path.join(self.maps, n))]
        self.assertEqual(left, [])                                          # nothing flat is left
        self.assertEqual(tree(os.path.join(self.maps, "balancer")), self.files)   # byte for byte
        self.assertEqual(tree(os.path.join(self.maps, "shark")), {"keep.json": b"shark"})
        self.assertTrue(any("moved 8 files" in s for s in said))
        # nothing left half-copied
        self.assertFalse([n for n in os.listdir(os.path.join(self.maps, "balancer")) if n.endswith(".migrating")])
        again = profiles.migrate_flat(self.maps)                            # second start: a no-op
        self.assertEqual((again["moved"], again["dup"], again["kept"]), (0, 0, []))

    def test_failed_copy_loses_nothing(self):
        real = shutil.copy2

        def bad_copy(src, dst, **kw):
            real(src, dst, **kw)
            if src.endswith("lidar.json"):
                with open(dst, "ab") as f:                                  # a corrupt copy
                    f.write(b"junk")
        with mock.patch("station.bridge.profiles.shutil.copy2", bad_copy):
            r = profiles.migrate_flat(self.maps)
        self.assertEqual(r["failed"], ["lidar.json"])
        with open(os.path.join(self.maps, "lidar.json"), "rb") as f:        # the original is still there, intact
            self.assertEqual(f.read(), self.files["lidar.json"])
        self.assertFalse(os.path.exists(os.path.join(self.maps, "balancer", "lidar.json")))
        self.assertFalse(os.path.exists(os.path.join(self.maps, "balancer", "lidar.json.migrating")))
        self.assertEqual(r["moved"], len(self.files) - 1)
        r = profiles.migrate_flat(self.maps)                                # the next start finishes the job
        self.assertEqual(r["moved"], 1)
        self.assertEqual(tree(os.path.join(self.maps, "balancer")), self.files)

    def test_never_overwrites(self):
        bal = os.path.join(self.maps, "balancer")
        os.makedirs(bal)
        with open(os.path.join(bal, "lidar.json"), "wb") as f:              # a different version already there
            f.write(b'{"yaw_deg": 99}')
        with open(os.path.join(bal, "living.json"), "wb") as f:             # an identical copy (interrupted run)
            f.write(self.files["living.json"])
        r = profiles.migrate_flat(self.maps)
        self.assertEqual(r["kept"], ["lidar.json"])
        self.assertEqual(r["dup"], 1)
        with open(os.path.join(bal, "lidar.json"), "rb") as f:
            self.assertEqual(f.read(), b'{"yaw_deg": 99}')
        with open(os.path.join(self.maps, "lidar.json"), "rb") as f:        # the other version stays too
            self.assertEqual(f.read(), self.files["lidar.json"])

    def test_missing_root_and_dirs_only(self):
        self.assertEqual(profiles.migrate_flat(os.path.join(self.tmp, "nope"))["moved"], 0)

    def test_bridge_migrates_all_three_roots_at_start(self):
        presets, falls = os.path.join(self.tmp, "presets"), os.path.join(self.tmp, "falls")
        os.makedirs(presets)
        os.makedirs(falls)
        with open(os.path.join(presets, "mine.json"), "wb") as f:
            f.write(b'{"name": "mine", "values": {"bal.kp": 200}}')
        with open(os.path.join(falls, "fall-20261120-101010.jsonl"), "wb") as f:
            f.write(b'["fall",{}]\n')
        b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True, map_dir=self.maps,
                   preset_dir=presets, falls_dir=falls)
        try:
            self.assertEqual(tree(os.path.join(self.maps, "balancer")), self.files)
            self.assertTrue(os.path.isfile(os.path.join(presets, "balancer", "mine.json")))
            self.assertTrue(os.path.isfile(os.path.join(falls, "balancer", "fall-20261120-101010.jsonl")))
            self.assertIsNone(b.robot)                       # migrated, but nothing is bound before the HELLO
            self.assertEqual(b.mapper.list(), [])
            self._hello(b, {})                               # now the balancer's own files are in use
            self.assertEqual(b.robot, "balancer")
            self.assertEqual(sorted(m["name"] for m in b.mapper.list()), ["autosave", "living"])
            self.assertEqual(b.mapper.settings["cone_deg"], 9.0)
            self.assertEqual(b.planner.settings["robot_radius_mm"], 91.0)
            self.assertEqual([p["name"] for p in b.tools.store.list() if not p["builtin"]], ["mine"])
            self.assertEqual(len(b.falls._files()), 1)
        finally:
            b.log.stop()

    @staticmethod
    def _hello(b, params, firmware="v4 2026-11-24"):
        feed_hello(b, params, firmware)


def feed_hello(b, params, firmware="v4 2026-11-24"):
    """What a robot says after connecting: HELLO, then the whole param list (a new connection first)."""
    b._robot_link_up()
    b._on_data(lp.encode_hello(param_count=len(params), firmware=firmware))
    for i, (n, v) in enumerate(params.items()):
        b._on_data(lp.encode_param_info(i, v, 0.0, 100.0, n))
    if not params:
        b._check_robot(time.monotonic() + 10)             # no params at all: the wait runs out


class Rig(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.maps = os.path.join(self.tmp, "maps")
        self.presets = os.path.join(self.tmp, "presets")
        self.falls = os.path.join(self.tmp, "falls")
        self.b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True, map_dir=self.maps,
                        preset_dir=self.presets, falls_dir=self.falls, lidar_port="tcp://127.0.0.1:1")
        self.b.planner.sync = True

    def tearDown(self):
        self.b.log.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)


class TestNoWritesBeforeHello(Rig):
    def test_nothing_is_written(self):
        b = self.b
        self.assertIsNone(b.robot)
        b.mapper.apply_settings({"cone_deg": 8})
        b.lidar.apply_settings({"yaw_deg": 5})
        b.planner.settings["robot_radius_mm"] = 100.0
        b.planner._save_settings()
        b.mapper.places_command({"action": "add", "name": "Couch", "x": 100, "y": 100})
        r = b.mapper.command({"action": "save", "name": "early"})
        self.assertEqual(r["t"], "error")
        b.mapper.n["used"] = 1
        b.mapper.changed_since_save = True
        b.mapper.last_autosave = -1e9
        b.mapper.poll(time.monotonic())                      # the autosave is due: still nothing
        with self.assertRaises(ValueError):
            b.tools.store.save("early", {"bal.kp": 1.0})
        self.assertIsNone(b.falls.path("fall-20261120-101010.jsonl"))
        self.assertEqual(b.falls.list(), [])
        for root in (self.maps, self.presets, self.falls):
            self.assertEqual(tree(root) if os.path.isdir(root) else {}, {}, root)
        self.assertEqual(b.status()["robot_name"], None)

    def test_hello_without_params_waits_then_uses_firmware(self):
        b = self.b
        b._robot_link_up()
        b._on_data(lp.encode_hello(param_count=5, firmware="v4 2026-11-24"))
        self.assertIsNone(b.robot)                           # the params (sys.platform) may still be on their way
        b._check_robot(time.monotonic() + 1)
        self.assertIsNone(b.robot)
        b._check_robot(time.monotonic() + 10)                # ... the wait ran out: firmware string / default
        self.assertEqual((b.robot, b.robot_source), ("balancer", "default"))

    def test_platform_param_decides(self):
        feed_hello(self.b, {"bal.kp": 1.0, "sys.platform": 2.0})
        self.assertEqual((self.b.robot, self.b.robot_source), ("shark", "sys.platform"))
        self.assertEqual(self.b.status()["robot_name"], "shark")

    def test_override(self):
        b = Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True, map_dir=self.maps,
                   preset_dir=self.presets, falls_dir=self.falls, robot="Shark")
        try:
            self.assertEqual((b.robot, b.robot_source), ("shark", "override"))
            feed_hello(b, {"sys.platform": 1.0})              # the HELLO does not change an explicit choice
            self.assertEqual(b.robot, "shark")
            self.assertEqual(b.mapper.dir, os.path.join(self.maps, "shark"))
        finally:
            b.log.stop()
        with self.assertRaises(ValueError):
            Bridge("tcp://127.0.0.1:1", WEB, os.path.join(self.tmp, "logs"), quiet=True, robot="../x")


class TestSeparateData(Rig):
    def fill(self, tag, radius):
        """Everything a robot keeps: a map with a place and a zone, lidar + planner + map settings, a preset."""
        b = self.b
        b.mapper.grid.integrate_wedge(0, 0, 0.0, 0.1, 1500, 1600)
        b.mapper.n["used"] = 5
        b.mapper.places_command({"action": "add", "name": "Couch-" + tag, "x": 500, "y": 0})
        b.mapper.zones_command({"action": "add", "kind": "nogo", "pts": [[800, -100], [900, -100], [900, 100], [800, 100]]})
        b.mapper.command({"action": "save", "name": "room"})
        b.mapper.apply_settings({"cone_deg": radius / 10.0})
        b.lidar.apply_settings({"yaw_deg": radius / 2.0})
        b.planner.settings["robot_radius_mm"] = float(radius)
        b.planner._save_settings()
        b.params = {1: {"id": 1, "name": "bal.kp", "value": radius, "min": 0.0, "max": 1000.0}}
        b.tools.store.save("tune-" + tag, {"bal.kp": float(radius)})

    def test_two_robots_never_mix(self):
        b = self.b
        feed_hello(b, {"sys.platform": 1.0})
        self.assertEqual(b.robot, "balancer")
        self.fill("bal", 91)
        # the Pi moves to the Shark: new connection, different robot
        b._robot_link_up()
        self.assertTrue(b.mapper.hold)                       # nothing from the new robot is mapped meanwhile
        b.mapper.on_range({"pitch_deg": 0.0, "range_mm": 500, "valid": 1, "x_mm": 0, "y_mm": 0, "heading_deg": 0,
                           "sensor_angle_deg": 0})
        self.assertEqual(b.mapper.n["readings"], 0)
        b._on_data(lp.encode_hello(param_count=1, firmware="v4 shark"))
        b._on_data(lp.encode_param_info(0, 2.0, 0.0, 2.0, "sys.platform"))
        self.assertEqual(b.robot, "shark")
        self.assertFalse(b.mapper.hold)
        self.assertEqual(b.mapper.dir, os.path.join(self.maps, "shark"))
        self.assertEqual(b.mapper.places.items, [])          # empty map, places, zones, defaults: nothing of the balancer
        self.assertEqual(b.mapper.zones.items, [])
        self.assertEqual(b.mapper.list(), [])
        self.assertIsNone(b.mapper.name)
        self.assertEqual(b.mapper.settings["cone_deg"], 12.0)
        self.assertEqual(b.lidar.settings["yaw_deg"], 0.0)
        self.assertEqual(b.planner.settings["robot_radius_mm"], 80.0)       # the default, not the balancer's 91
        self.assertEqual([p["name"] for p in b.tools.store.list() if not p["builtin"]], [])
        self.assertEqual(b.falls.dir, os.path.join(self.falls, "shark"))
        self.fill("shark", 190)
        self.assertEqual([p["name"] for p in b.mapper.places.items if p["name"].startswith("Couch")], ["Couch-shark"])
        # back to the balancer
        feed_hello(b, {"sys.platform": 1.0})
        self.assertEqual(b.robot, "balancer")
        self.assertEqual(sorted(m["name"] for m in b.mapper.list()), ["room"])
        b.mapper.command({"action": "load", "name": "room"})
        self.assertEqual([p["name"] for p in b.mapper.places.items if p["name"].startswith("Couch")], ["Couch-bal"])
        self.assertEqual(len(b.mapper.zones.items), 1)
        self.assertEqual(b.mapper.settings["cone_deg"], 9.1)
        self.assertEqual(b.lidar.settings["yaw_deg"], 45.5)
        self.assertEqual(b.planner.settings["robot_radius_mm"], 91.0)
        self.assertEqual([p["name"] for p in b.tools.store.list() if not p["builtin"]], ["tune-bal"])
        # on disk: two separate trees, nothing flat
        for root in (self.maps, self.presets):
            self.assertEqual(sorted(os.listdir(root)), ["balancer", "shark"])
        with open(os.path.join(self.maps, "shark", "lidar.json")) as f:
            self.assertEqual(json.load(f)["yaw_deg"], 95.0)
        with open(os.path.join(self.maps, "balancer", "lidar.json")) as f:
            self.assertEqual(json.load(f)["yaw_deg"], 45.5)

    def test_the_old_robots_map_is_autosaved_on_a_switch(self):
        b = self.b
        feed_hello(b, {"sys.platform": 1.0})
        b.mapper.grid.integrate_wedge(0, 0, 0.0, 0.1, 1500, 1600)
        b.mapper.n["used"] = 3
        b.mapper.changed_since_save = True
        feed_hello(b, {"sys.platform": 2.0})
        self.assertTrue(os.path.isfile(os.path.join(self.maps, "balancer", AUTOSAVE_NAME + ".json")))
        self.assertFalse(os.path.isfile(os.path.join(self.maps, "shark", AUTOSAVE_NAME + ".json")))
        self.assertEqual(b.mapper.n["used"], 0)               # the Shark starts with an empty map

    def test_same_robot_reconnecting_keeps_its_state(self):
        b = self.b
        feed_hello(b, {"sys.platform": 1.0})
        b.mapper.grid.integrate_wedge(0, 0, 0.0, 0.1, 1500, 1600)
        b.mapper.n["used"] = 3
        places = b.mapper.places
        feed_hello(b, {"sys.platform": 1.0})
        self.assertEqual(b.robot, "balancer")
        self.assertEqual(b.mapper.n["used"], 3)
        self.assertIs(b.mapper.places, places)
        self.assertFalse(b.mapper.hold)

    def test_session_log_records_the_robot(self):
        b = self.b
        self.assertTrue(b.log.start())
        feed_hello(b, {"sys.platform": 2.0})
        b.log.stop()
        with open(os.path.join(b.log.dir, b.log.name), encoding="utf-8") as f:
            rows = [json.loads(line) for line in f if line.startswith('["robot"')]
        self.assertEqual([r[2:] for r in rows], [["shark", "sys.platform"]])


class TestSimProfile(unittest.TestCase):
    def test_sim_gets_its_own_profile(self):
        tmp = tempfile.mkdtemp()
        try:
            asyncio.run(self._run(tmp))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    async def _run(self, tmp):
        robot = SimRobot(seed=1)
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        port = srv.sockets[0].getsockname()[1]
        phys = asyncio.ensure_future(sim.physics())
        maps, presets = os.path.join(tmp, "maps"), os.path.join(tmp, "presets")
        os.makedirs(maps)                                    # the owner's old flat layout, next to the sim
        with open(os.path.join(maps, "lidar.json"), "w") as f:
            f.write('{"yaw_deg": 12}')
        b = Bridge("tcp://127.0.0.1:%d" % port, WEB, os.path.join(tmp, "logs"), "127.0.0.1", 0,
                   release_file=os.path.join(tmp, "release"), quiet=True, map_dir=maps, preset_dir=presets,
                   falls_dir=os.path.join(tmp, "falls"))
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task())]
        try:
            await until(lambda: b.robot is not None, 6)
            self.assertEqual((b.robot, b.robot_source), ("sim", "firmware"))
            self.assertEqual(b.status()["robot_name"], "sim")
            self.assertEqual(b.mapper.dir, os.path.join(maps, "sim"))
            b.mapper.save("simmap")
            self.assertTrue(os.path.isfile(os.path.join(maps, "sim", "simmap.json")))
            self.assertTrue(os.path.isfile(os.path.join(maps, "balancer", "lidar.json")))   # migrated, not the sim's
            self.assertFalse(os.path.exists(os.path.join(maps, "sim", "lidar.json")))
            self.assertEqual(os.listdir(os.path.join(maps, "balancer")), ["lidar.json"])
        finally:
            for t in tasks + [phys]:
                t.cancel()
            srv.close()
            b.log.stop()


if __name__ == "__main__":
    unittest.main()
