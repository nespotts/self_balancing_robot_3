"""Issue #109 (runaway motor after a fall): Teensy reset causes on the bridge + Health, encoder faults end to end.

- ResetTracker: one message per Teensy boot; watchdog / crash / power dip / an unexplained power-on mid-session warn,
  a first power-on or a flash does not.
- Health: the Resets tile, the Enc L / Enc R tiles from fault_flags + the firmware's "enc:" counters line.
- sim + bridge: a watchdog reset reaches the log ("Teensy reset unexpectedly: watchdog"), a teensy_reset message and the
  Resets tile; an encoder fault stops GOTO / STATION (balancing in place), refuses arming and SET_MODE, and shows on
  the Enc tile.
Run: python -m unittest discover -s station -t .
"""
import asyncio
import json
import shutil
import tempfile
import unittest

from station.bridge import health as H
from station.bridge import link_protocol as lp
from station.bridge.resets import ResetTracker
from station.bridge.tests.test_bridge_sim import until, wait_for
from station.bridge.tests.test_hardening import Rig


def hello(**kw):
    d = {"link_version": 9, "radio_version": 2, "param_count": 3, "firmware": "v4 x", "reset_cause": lp.LINK_RESET_POWER_ON,
         "reset_flags": lp.LINK_RESETF_WDT, "srsr": 1, "prev_uptime_s": 0, "off_s": 0xFFFFFFFF, "uptime_ms": 900}
    d.update(kw)
    return d


def tile(h, id):
    return next(t for t in h["tiles"] if t["id"] == id)


class TestResetTracker(unittest.TestCase):
    def test_one_message_per_boot(self):
        r = ResetTracker()
        lvl, text = r.on_hello(hello(), 100.0)
        self.assertEqual(lvl, "info")                       # first boot seen, cold power-on: normal
        self.assertIn("power-on", text)
        self.assertIsNone(r.on_hello(hello(uptime_ms=5000), 104.0))    # GET_PARAMS reply: same boot
        self.assertIsNone(r.on_hello(hello(uptime_ms=9000), 108.0))
        # the watchdog fired: new facts, small uptime
        lvl, text = r.on_hello(hello(reset_cause=lp.LINK_RESET_WATCHDOG, srsr=0x80, uptime_ms=700,
                                     reset_flags=lp.LINK_RESETF_WDT | lp.LINK_RESETF_SNVS, prev_uptime_s=264, off_s=0),
                               200.0)
        self.assertEqual(lvl, "warn")
        self.assertTrue(text.startswith("Teensy reset unexpectedly: watchdog"), text)
        self.assertIn("previous run 4 min, off 0 s", text)
        st = r.status(230.0)
        self.assertEqual((st["count"], st["last"]["cause"], st["last"]["ago_s"]), (1, lp.LINK_RESET_WATCHDOG, 30))

    def test_power_dip_and_crash_warn(self):
        r = ResetTracker()
        lvl, text = r.on_hello(hello(reset_cause=lp.LINK_RESET_POWER_DIP, reset_flags=lp.LINK_RESETF_SNVS, off_s=0,
                                     prev_uptime_s=265), 10.0)
        self.assertEqual(lvl, "warn")                       # even as the first HELLO (the Pi rebooted too)
        self.assertIn("brownout", text)
        r2 = ResetTracker()
        lvl, text = r2.on_hello(hello(reset_cause=lp.LINK_RESET_WATCHDOG, reset_flags=lp.LINK_RESETF_CRASH), 10.0)
        self.assertEqual(lvl, "warn")
        self.assertIn("crash report", text)

    def test_power_on_mid_session_vs_flash(self):
        r = ResetTracker(start=0.0)
        r.on_hello(hello(uptime_ms=60000), 0.0)
        for t in range(1, 100):
            r.note_alive(float(t))                                     # linked, telemetry flowing
        lvl, text = r.on_hello(hello(uptime_ms=800), 100.0)            # rebooted, cold, nobody flashed it
        self.assertEqual(lvl, "warn")
        self.assertIn("power was lost", text)
        r.note_released()                                              # the bridge let go of the port: flashing
        lvl, _ = r.on_hello(hello(reset_cause=lp.LINK_RESET_FLASHED, uptime_ms=600), 200.0)
        self.assertEqual(lvl, "info")
        r.note_released()
        lvl, _ = r.on_hello(hello(uptime_ms=500), 300.0)               # power-on after a release: expected too
        self.assertEqual(lvl, "info")

    def test_whole_robot_power_on_is_not_a_reset(self):
        """#110, real robot 2026-11-25: the Pi and the Teensy power up together. The bridge's first link may see a
        Teensy boot that the Pi's USB then powers down and up again (a second cold boot seconds later): that is the
        robot powering up, not 'Teensy reset unexpectedly' (the Resets tile stays green)."""
        r = ResetTracker(start=0.0)                                    # the bridge started
        lvl, _ = r.on_hello(hello(uptime_ms=1242), 12.0)               # first link: a cold boot
        self.assertEqual(lvl, "info")
        r.note_alive(13.0)
        r.note_alive(14.0)                                             # ... the link drops, the Teensy boots again
        lvl, text = r.on_hello(hello(uptime_ms=1100), 17.0)
        self.assertEqual(lvl, "info", text)
        self.assertIsNone(r.status(20.0)["last"])
        tl = H.reset_tile(r.status(20.0), {"fault_flags": 0}, hello())
        self.assertEqual(tl["level"], "ok")
        # later on: switched off for a minute and on again (the link was gone all that time): a normal boot too
        for t in range(18, 400):
            r.note_alive(float(t))
        lvl, text = r.on_hello(hello(uptime_ms=1500), 470.0)
        self.assertEqual(lvl, "info", text)
        self.assertIn("off ~", text)
        # ... but power lost while it ran (alive until a second before it booted again): unexpected
        for t in range(471, 600):
            r.note_alive(float(t))
        lvl, text = r.on_hello(hello(uptime_ms=900), 600.5)
        self.assertEqual(lvl, "warn", text)
        self.assertEqual(H.reset_tile(r.status(601.0), {"fault_flags": 0}, hello())["level"], "warn")

    def test_power_on_in_the_bridges_first_minute(self):
        r = ResetTracker(start=100.0)
        r.on_hello(hello(uptime_ms=30000), 101.0)                      # the Teensy was up before the Pi's bridge
        for t in range(102, 140):
            r.note_alive(float(t))
        lvl, _ = r.on_hello(hello(uptime_ms=700), 140.5)              # the robot is still powering up
        self.assertEqual(lvl, "info")

    def test_v8_firmware_ignored(self):
        self.assertIsNone(ResetTracker().on_hello({"link_version": 8, "firmware": "old"}, 0.0))


ENC_LINE = ("enc: L FAULT err 140 rej 3 rec 9 resync 2 read 480/1300 us last busy | "
            "R ok err 0 rej 0 rec 0 resync 0 read 120/130 us last ok")


class TestHealthTiles(unittest.TestCase):
    def test_parse_enc(self):
        e = H.parse_enc(ENC_LINE)
        self.assertEqual((e[0]["ok"], e[0]["errors"], e[0]["rejects"], e[0]["recoveries"], e[0]["last"]),
                         (False, 140, 3, 9, "busy"))
        self.assertEqual((e[1]["ok"], e[1]["read_us"]), (True, 120))
        self.assertIsNone(H.parse_enc("enc L: FAULT (last nack; ...)"))

    def test_enc_tiles(self):
        v9 = {"link_version": 9}
        t = {"error_code": 3, "fault_flags": lp.LINK_FAULT_ENC_L | lp.LINK_FAULT_ENC_L_ERR}
        tl = H.enc_tiles(t, H.parse_enc(ENC_LINE), v9)
        self.assertEqual((tl["encl"]["level"], tl["encl"]["value"]), ("bad", "FAULT"))
        self.assertIn("other wheel + gyro", tl["encl"]["why"])
        self.assertIn("140 I2C errors", tl["encl"]["why"])
        self.assertEqual(tl["encr"]["level"], "ok")
        # error_code shows only the first fault (an IMU fault hides it): fault_flags still shows the encoder
        tl = H.enc_tiles({"error_code": 2, "fault_flags": lp.LINK_FAULT_ENC_R}, None, v9)
        self.assertEqual(tl["encr"]["level"], "bad")
        # trouble without a latched fault: amber
        tl = H.enc_tiles({"error_code": 0, "fault_flags": lp.LINK_FAULT_ENC_L_ERR}, None, v9)
        self.assertEqual(tl["encl"]["level"], "warn")
        # v8 firmware: reserved2 is not fault_flags, error_code only
        tl = H.enc_tiles({"error_code": 4, "fault_flags": 0xFF}, None, {"link_version": 8})
        self.assertEqual((tl["encl"]["level"], tl["encr"]["level"]), ("ok", "bad"))

    def test_reset_tile(self):
        r = ResetTracker()
        r.on_hello(hello(), 0.0)
        tl = H.reset_tile(r.status(1.0), {"fault_flags": 0}, hello())
        self.assertEqual(tl["level"], "ok")
        tl = H.reset_tile(r.status(1.0), {"fault_flags": lp.LINK_FAULT_WDT_OFF}, hello())
        self.assertEqual(tl["level"], "warn")
        r.on_hello(hello(reset_cause=lp.LINK_RESET_WATCHDOG, srsr=0x80, uptime_ms=300), 10.0)
        tl = H.reset_tile(r.status(70.0), {"fault_flags": 0}, hello())
        self.assertEqual((tl["level"], tl["value"]), ("warn", "watchdog"))
        self.assertIn("60 s ago", tl["why"])


class TestWithSim(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_watchdog_reset_reported(self):
        asyncio.run(self._reset())

    async def _reset(self):
        rig = await Rig(self.tmp).start()
        try:
            b, robot = rig.b, rig.robot
            ws = await rig.ws()
            h = await wait_for(ws, lambda m: m["t"] == "health" and any(t["id"] == "reset" for t in m["tiles"]), 8)
            self.assertEqual(tile(h, "reset")["level"], "ok")
            # the "Teensy" comes back from a watchdog reset: its next HELLO says so
            robot.reset_cause = lp.LINK_RESET_WATCHDOG
            b.send(lp.encode_get_params())
            m = await wait_for(ws, lambda m: m["t"] == "teensy_reset", 5)
            self.assertEqual(m["level"], "warn")
            self.assertTrue(m["text"].startswith("Teensy reset unexpectedly: watchdog"), m["text"])
            await until(lambda: any(i["src"] == "bridge" and "Teensy reset unexpectedly" in i["text"] for i in b.logs))
            self.assertTrue(any(i["src"] == "bridge" and i["text"].startswith("WARNING: Teensy reset unexpectedly")
                                for i in b.logs))
            h = await wait_for(ws, lambda m: m["t"] == "health" and tile(m, "reset")["level"] == "warn", 4)
            self.assertIn("watchdog", tile(h, "reset")["why"])
        finally:
            await rig.stop()

    def test_encoder_fault_stops_driving(self):
        asyncio.run(self._enc())

    async def _enc(self):
        rig = await Rig(self.tmp).start()
        try:
            b, robot = rig.b, rig.robot
            ws = await rig.ws()
            await until(lambda: robot.pi_link and robot.pi_clients > 0, 5)
            self.assertEqual(robot.set_mode(lp.MODE_BALANCE, arm=True), "ok")
            await until(lambda: robot.mode == lp.MODE_BALANCE, 3)
            self.assertEqual(robot.set_mode(lp.MODE_STATION), "ok")
            await until(lambda: robot.mode == lp.MODE_STATION, 3)
            # the left encoder drops out while driving
            robot.fault = 3
            await until(lambda: robot.mode == lp.MODE_BALANCE, 3)
            self.assertEqual(robot.mode, lp.MODE_BALANCE)          # stopped, still balancing
            self.assertTrue(robot.armed)                            # never dropped
            await until(lambda: any("STATION stopped -> BALANCE" in i["text"] for i in b.logs), 3)
            self.assertTrue(any("STATION stopped -> BALANCE" in i["text"] for i in b.logs))
            # the station asks for GOTO: refused, logged
            ws.send(json.dumps({"t": "set_mode", "mode": lp.MODE_GOTO}))
            await until(lambda: any("ignored: left encoder fault" in i["text"] for i in b.logs), 3)
            self.assertTrue(any("ignored: left encoder fault" in i["text"] for i in b.logs))
            self.assertEqual(robot.mode, lp.MODE_BALANCE)
            h = await wait_for(ws, lambda m: m["t"] == "health" and tile(m, "encl")["level"] == "bad", 4)
            self.assertEqual(tile(h, "encl")["value"], "FAULT")
            t = await wait_for(ws, lambda m: m["t"] == "tlm", 2)
            self.assertEqual(t["fault_flags"] & lp.LINK_FAULT_ENC_L, lp.LINK_FAULT_ENC_L)
            # disarmed, it can't be armed again while faulted
            robot.set_mode(lp.MODE_IDLE)
            await until(lambda: robot.mode == lp.MODE_IDLE, 3)
            self.assertEqual(robot.set_mode(lp.MODE_BALANCE, arm=True), "enc_fault")
            robot.fault = 0
            await until(lambda: any("ok again" in i["text"] for i in b.logs), 3)
            self.assertEqual(robot.set_mode(lp.MODE_BALANCE, arm=True), "ok")
        finally:
            await rig.stop()


if __name__ == "__main__":
    unittest.main()
