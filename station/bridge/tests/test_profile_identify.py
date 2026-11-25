"""Robot identification is robust to every start order and reconnect (#101): the bridge before the Teensy, a Teensy
reboot / reflash while the bridge runs, a HELLO before (or without) the param list, quick reconnects, and another robot
after a reconnect. While no robot is known, settings changes say so and the Health page warns.

A scripted Teensy behind a fake link (open_link patched) and a fake clock: the real _link_step / _on_data / _tick run."""
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

from station.bridge import bridge as bm
from station.bridge import health as H
from station.bridge import link_protocol as lp
from station.bridge.transport import LinkError

WEB = os.path.join(os.path.dirname(__file__), "..", "..", "web")
N_PARAMS = 12
PARAMS = [("p.%02d" % i, float(i)) for i in range(N_PARAMS)]
FW = "v4 2026-11-24"


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class FakeLink:
    def __init__(self, teensy, on_data, on_lost):
        self.teensy, self.on_data, self.on_lost = teensy, on_data, on_lost
        self.parser = lp.FrameParser()
        self.closed = False
        self.get_params = 0

    def write(self, frame):
        if self.closed:
            return False
        for t, _p in self.parser.feed(frame):
            if t == lp.MSG_GET_PARAMS:
                self.get_params += 1
                self.teensy.queue.append(self)      # answered on the next pump (the real Teensy is not re-entrant)
        return True

    def close(self):
        self.closed = True


class Teensy:
    """What the robot does on the other end of the USB cable.
    present: the serial port exists; deaf: GET_PARAMS requests to ignore (still booting / lost);
    params: answer GET_PARAMS with the list (False: HELLO only); cut: send only this many params per answer."""

    def __init__(self, fw=FW, params=PARAMS):
        self.fw, self.params = fw, list(params)
        self.present = True
        self.deaf = 0
        self.answer_params = True
        self.cut = None
        self.link = None
        self.links = []
        self.queue = []

    def pump(self):
        while self.queue:
            link = self.queue.pop(0)
            if link is self.link and not link.closed:
                self.on_get_params(link)

    async def open_link(self, spec, on_data, on_lost, baud=115200):
        if not self.present:
            raise LinkError("%s not present" % spec)
        self.link = FakeLink(self, on_data, on_lost)
        self.links.append(self.link)
        return self.link

    def hello(self):
        self.link.on_data(lp.encode_hello(param_count=len(self.params), firmware=self.fw))

    def stream(self, upto=None):
        for i, (n, v) in enumerate(self.params[:upto]):
            self.link.on_data(lp.encode_param_info(i, v, -10.0, 100.0, n))

    def on_get_params(self, link):
        if self.deaf > 0:
            self.deaf -= 1
            return
        self.hello()
        if self.answer_params:
            self.stream(self.cut)

    def drop(self):
        """The USB port goes away (reboot / reflash): EOF on the tty."""
        link, self.link = self.link, None
        link.closed = True
        link.on_lost()

    def asks(self):
        return sum(l.get_params for l in self.links)


class Base(unittest.TestCase):
    robot = None

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.maps = os.path.join(self.tmp, "maps")
        os.makedirs(os.path.join(self.maps, "balancer"))
        with open(os.path.join(self.maps, "balancer", "lidar.json"), "w") as f:
            json.dump({"yaw_deg": -90}, f)                     # the real robot's saved mount yaw
        self.clk = Clock()
        self.teensy = Teensy()
        self.patches = [mock.patch.object(bm, "_now", self.clk), mock.patch.object(bm, "open_link", self.teensy.open_link)]
        for p in self.patches:
            p.start()
        self.b = bm.Bridge("/dev/ttyACM0", WEB, os.path.join(self.tmp, "logs"), "127.0.0.1", 0,
                           release_file=os.path.join(self.tmp, "release"), quiet=True, map_dir=self.maps,
                           preset_dir=os.path.join(self.tmp, "presets"), falls_dir=os.path.join(self.tmp, "falls"),
                           lidar_port="teensy", robot=self.robot)
        self.b.planner.sync = True

    def tearDown(self):
        self.b.log.stop()
        for p in self.patches:
            p.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_for(self, s, dt=0.25):
        """Real link steps + ticks with the clock moving: s seconds."""
        import asyncio
        end = self.clk.t + s
        while self.clk.t < end - 1e-9:
            self.clk.t += dt
            asyncio.run(self.b._link_step())
            self.teensy.pump()
            self.b._tick(self.clk.t)
            self.teensy.pump()

    def run_until(self, cond, max_s, dt=0.25):
        import asyncio
        end = self.clk.t + max_s
        while not cond():
            self.assertLess(self.clk.t, end, "timed out")
            self.clk.t += dt
            asyncio.run(self.b._link_step())
            self.teensy.pump()
            self.b._tick(self.clk.t)
            self.teensy.pump()

    def assert_bound(self, name="balancer"):
        b = self.b
        self.assertEqual(b.robot, name)
        self.assertFalse(b.robot_pending)
        self.assertFalse(b.mapper.hold)
        self.assertEqual(b.mapper.dir, os.path.join(self.maps, name))
        self.assertEqual(b.tools.store.dir, os.path.join(self.tmp, "presets", name))
        self.assertEqual(b.falls.dir, os.path.join(self.tmp, "falls", name))
        self.assertTrue(b.lidar_obst.ready())                   # obstacle frames to the Teensy flow (#99 gate)
        self.assertEqual(b.status()["robot_wait_s"], 0.0)

    def lidar_set_saves(self, yaw):
        r = self.b.handle_command({"t": "lidar", "action": "set", "settings": {"yaw_deg": yaw}}, "test")
        self.assertFalse(r and r.get("t") == "error", r)
        with open(os.path.join(self.b.mapper.dir, "lidar.json")) as f:
            self.assertEqual(json.load(f)["yaw_deg"], yaw)


class TestStartOrder(Base):
    def test_bridge_starts_before_the_teensy(self):
        b, ty = self.b, self.teensy
        ty.present = False
        self.run_for(3)                                       # no USB port yet
        self.assertEqual(b.link_state, "waiting")
        self.assertIsNone(b.robot)
        ty.present, ty.deaf = True, 2                         # port there, firmware still booting: requests lost
        self.run_for(1.5)
        self.assertEqual(b.link_state, "connected")
        self.assertIsNone(b.robot)
        self.assertTrue(b.robot_pending)
        self.run_for(4)                                       # the bridge keeps asking; the Teensy now answers
        self.assert_bound()
        self.assertGreaterEqual(ty.asks(), 3)
        self.assertEqual(b.lidar.settings["yaw_deg"], -90.0)  # the saved yaw, not the default 0
        self.assertEqual(len(b.params), N_PARAMS)
        self.lidar_set_saves(-91.0)

    def test_teensy_boot_hello_but_the_get_params_was_lost(self):
        """Cold boot: the request at connect is lost, the Teensy's own boot HELLO arrives (within 2 s: the HELLO
        handler does not ask again) - the tick asks for the missing params."""
        b, ty = self.b, self.teensy
        ty.deaf = 1
        self.run_for(1.0)
        self.assertEqual(b.link_state, "connected")
        self.clk.t += 0.5
        ty.hello()                                            # unsolicited boot HELLO, no params behind it
        self.run_for(1.5)
        self.assert_bound()
        self.assertEqual(len(b.params), N_PARAMS)             # asked again, the list came
        self.assertEqual(b.robot_source, "default")

    def test_hello_but_no_param_list_ever(self):
        b, ty = self.b, self.teensy
        ty.answer_params = False
        self.run_for(1.0)
        self.assertIsNotNone(b.ident["hello"])
        self.assertIsNone(b.robot)
        self.run_for(3.0)                                     # ROBOT_WAIT_S after the HELLO: firmware string decides
        self.assert_bound()
        self.assertEqual(b.robot_source, "default")
        self.assertGreaterEqual(ty.asks(), 2)                 # and the params were asked for again meanwhile

    def test_param_list_never_completes(self):
        b, ty = self.b, self.teensy
        ty.cut = 5
        self.run_for(1.0)
        self.assertIsNone(b.robot)
        self.run_for(3.0)
        self.assert_bound()
        n = ty.asks()
        self.run_for(30)
        self.assertLessEqual(ty.asks() - n, 8)                # asks back off: no flood
        self.assertEqual(b.robot, "balancer")

    def test_identified_by_the_whole_list_without_waiting(self):
        self.run_for(0.25)
        self.assert_bound()

    def test_never_asked_and_hello_late(self):
        """The Teensy says nothing for 8 s on an open port (all requests lost), then HELLO + list: identified."""
        b, ty = self.b, self.teensy
        ty.deaf = 1000
        self.run_for(8)
        self.assertIsNone(b.robot)
        self.assertGreater(b.status()["robot_wait_s"], 5)
        ty.deaf = 0
        self.run_for(10.5)                                    # the slow retry reaches it
        self.assert_bound()


class TestReconnect(Base):
    def connect(self):
        self.run_for(0.5)
        self.assert_bound()

    def test_teensy_reflash_same_robot(self):
        b, ty = self.b, self.teensy
        self.connect()
        self.lidar_set_saves(-90.0)
        b.mapper.grid.integrate_wedge(0, 0, 0.0, 0.1, 1500, 1600)
        b.mapper.n["used"] = 4
        places = b.mapper.places
        ty.drop()                                             # reflash: port gone ...
        self.assertEqual(b.link_state, "waiting")
        ty.present = False
        self.run_for(3)
        ty.present, ty.deaf = True, 1                         # ... back; the first request is lost while it boots
        self.run_for(1.0)
        self.assertTrue(b.mapper.hold)                        # held until the robot on the new link is known
        self.run_for(3)
        self.assert_bound()
        self.assertEqual(b.mapper.n["used"], 4)               # same robot: its map stays
        self.assertIs(b.mapper.places, places)
        self.assertEqual(b.lidar.settings["yaw_deg"], -90.0)
        self.lidar_set_saves(-92.0)

    def test_new_firmware_on_the_same_link(self):
        """A reboot / reflash that never dropped the port: a HELLO with other firmware re-identifies."""
        b, ty = self.b, self.teensy
        self.connect()
        ty.fw = "v4 2026-11-25"
        self.clk.t += 0.1
        ty.hello()
        self.assertTrue(b.robot_pending)
        self.assertTrue(b.mapper.hold)
        self.run_for(3.5)
        self.assert_bound()

    def test_folders_rebound_if_lost(self):
        b, ty = self.b, self.teensy
        self.connect()
        b.mapper.dir = None                                   # whatever went wrong before: the next HELLO fixes it
        ty.hello()
        self.assert_bound()
        self.lidar_set_saves(-90.0)

    def test_repeated_reconnects(self):
        b, ty = self.b, self.teensy
        self.connect()
        b.mapper.n["used"] = 7
        for i in range(6):                                    # drops in the middle of identifying, every ~1 s
            ty.deaf, ty.answer_params = (1, True) if i % 2 else (0, False)
            ty.drop()
            self.run_until(lambda: ty.link is not None, 2.0)
            self.run_for(0.5)
            self.assertTrue(b.robot_pending)
            self.assertEqual(b.robot, "balancer")             # never unbound in between
        ty.drop()
        ty.deaf, ty.answer_params = 0, True
        self.run_until(lambda: ty.link is not None, 2.0)
        self.run_for(0.25)
        self.assert_bound()
        self.assertEqual(b.mapper.n["used"], 7)
        self.assertEqual(b.mapper.dir, os.path.join(self.maps, "balancer"))

    def test_another_robot_after_a_reconnect(self):
        b, ty = self.b, self.teensy
        self.connect()
        b.mapper.grid.integrate_wedge(0, 0, 0.0, 0.1, 1500, 1600)
        b.mapper.n["used"] = 3
        b.mapper.changed_since_save = True
        ty.drop()
        ty.params = PARAMS + [("sys.platform", 2.0)]          # the Pi moved to the Shark
        ty.fw = "v4 shark"
        self.run_for(1.5)
        self.assert_bound("shark")
        self.assertTrue(os.path.isfile(os.path.join(self.maps, "balancer", bm.AUTOSAVE_NAME + ".json")))
        self.assertEqual(b.mapper.n["used"], 0)
        self.assertEqual(b.lidar.settings["yaw_deg"], 0.0)    # the Shark's own (default) settings
        self.lidar_set_saves(15.0)
        with open(os.path.join(self.maps, "balancer", "lidar.json")) as f:
            self.assertEqual(json.load(f)["yaw_deg"], -90)     # the balancer's file untouched

    def test_bind_failure_never_leaves_it_unbound(self):
        b = self.b
        with mock.patch.object(b.lidar, "reload_settings", side_effect=RuntimeError("bad file")):
            said = []
            with mock.patch.object(b, "say", side_effect=said.append):
                self.run_for(0.5)
        self.assert_bound()
        self.assertTrue(any("WARNING: robot profile balancer" in s for s in said), said)
        self.lidar_set_saves(-90.0)

    def test_identify_error_retries(self):
        b, ty = self.b, self.teensy
        ty.answer_params = False
        real = bm.profiles.identify
        calls = []

        def flaky(*a):
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("boom")
            return real(*a)
        with mock.patch.object(bm.profiles, "identify", flaky):
            self.run_for(3.5)
            self.assertIsNone(b.robot)
            self.assertTrue(b.robot_pending)
            self.run_for(2.5)
        self.assert_bound()


class TestUnidentified(Base):
    def test_settings_set_says_not_saved(self):
        b, ty = self.b, self.teensy
        ty.deaf = 1000
        self.run_for(1)
        for m in ({"t": "lidar", "action": "set", "settings": {"yaw_deg": 5}},
                  {"t": "lidar", "action": "defaults"},
                  {"t": "map", "action": "set", "settings": {"cone_deg": 8}},
                  {"t": "plan", "action": "set", "settings": {"robot_radius_mm": 90}}):
            r = b.handle_command(m, "test")
            self.assertEqual((r["t"], r["code"], r["text"]), ("error", "no_profile", "robot not identified yet; not saved"))
        self.assertEqual(b.lidar.settings["yaw_deg"], 0.0)
        r = b.handle_command({"t": "lidar", "action": "get"}, "test")
        self.assertFalse(r and r.get("t") == "error", r)      # reading is fine
        self.assertEqual(sorted(os.listdir(os.path.join(self.maps, "balancer"))), ["lidar.json"])
        ty.deaf = 0
        self.run_for(11)
        self.assert_bound()
        self.lidar_set_saves(-80.0)

    def test_health_warns_after_5_s(self):
        b, ty = self.b, self.teensy
        ty.deaf = 1000
        self.run_for(2)
        out = H.evaluate(b.health_ctx())
        self.assertNotIn("profile", [t["id"] for t in out["tiles"]])     # a moment to identify: no alarm yet
        self.run_for(4)
        ctx = b.health_ctx()
        self.assertGreater(ctx["robot_wait_s"], 5)
        out = H.evaluate(ctx)
        t = next(t for t in out["tiles"] if t["id"] == "profile")
        self.assertEqual((t["level"], t["value"]), ("warn", "unidentified"))
        self.assertIn("no HELLO", t["why"])
        self.assertIn("profile", [i["id"] for i in out["preflight"]["items"]])
        st = b.status()
        self.assertTrue(st["robot_pending"])
        self.assertGreater(st["robot_wait_s"], 5)
        ty.deaf = 0
        self.run_for(11)
        self.assertNotIn("profile", [t["id"] for t in H.evaluate(b.health_ctx())["tiles"]])

    def test_health_rule(self):
        base = {"link": "connected", "robot_alive": False}
        self.assertNotIn("profile", [t["id"] for t in H.evaluate(dict(base, robot_wait_s=4.0))["tiles"]])
        self.assertNotIn("profile", [t["id"] for t in H.evaluate(dict(base, link="waiting", robot_wait_s=9.0))["tiles"]])
        out = H.evaluate(dict(base, robot_wait_s=9.0, robot_name="balancer", robot_wait_why="waiting for the param list"))
        t = next(t for t in out["tiles"] if t["id"] == "profile")
        self.assertEqual((t["level"], t["value"]), ("warn", "identifying"))


class TestOverride(Base):
    robot = "shark"

    def test_override_survives_reconnects(self):
        b, ty = self.b, self.teensy
        self.assert_bound("shark")
        ty.deaf = 1000
        self.run_for(6)
        self.assertEqual(ty.asks(), 1)                        # nothing to identify: only the request at connect
        self.assert_bound("shark")
        ty.drop()
        ty.deaf = 0
        self.run_for(1)
        self.assert_bound("shark")
        self.lidar_set_saves(3.0)


if __name__ == "__main__":
    unittest.main()
