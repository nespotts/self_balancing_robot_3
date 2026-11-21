"""The phone as the only controller (issue #64): Disarm, Arm, STOP latch / release, and what happens when the phone goes away.

Against the sim + the real bridge over WebSocket, like the app's page does it:
  - Disarm = SET_MODE IDLE, never a latch. No RC remote: the robot really disarms. With a remote its ARMED switch rules,
    so the robot goes IDLE (motors off) but stays armed.
  - STOP latches in the bridge and the robot; Release E-stop leaves it disarmed; the two-tap Arm works again afterwards.
  - Tab closed / phone out of the Wi-Fi (socket still open but silent): the client count the robot sees drops to 0 and
    GOTO / EXPLORE pause (robot keeps balancing). A phone that answers its pings keeps counting.
Run: python -m unittest discover -s station -t ."""
import asyncio
import json
import os
import time
import unittest

from station.bridge import bridge as bridge_mod
from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.tests.test_hardening import WEB, Base, Rig
from station.sim.sim import SimRobot, SimServer


class PhoneRig(Rig):
    """Rig with an optional RC remote on the simulated robot."""

    def __init__(self, tmp, remote=False, **kw):
        super().__init__(tmp, **kw)
        self.remote = remote

    async def start(self):
        self.robot = SimRobot(seed=1, remote=self.remote)
        sim = SimServer(self.robot, "127.0.0.1", 0, quiet=True)
        self.srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        self.phys = asyncio.ensure_future(sim.physics())
        sim_port = self.srv.sockets[0].getsockname()[1]
        self.b = Bridge("tcp://127.0.0.1:%d" % sim_port, WEB, os.path.join(self.tmp, "logs"), "127.0.0.1", 0,
                        release_file=os.path.join(self.tmp, "release"), quiet=True, map_dir=os.path.join(self.tmp, "maps"),
                        preset_dir=os.path.join(self.tmp, "presets"))
        self.hs = await self.b.build_http().start("127.0.0.1", 0)
        self.port = self.hs.sockets[0].getsockname()[1]
        self.tasks = [asyncio.ensure_future(self.b.link_task()), asyncio.ensure_future(self.b.tick_task())]
        return self


def send(ws, **m):
    ws.send(json.dumps(m))


async def drain(ws, rig, seconds):
    """Keep reading (a page does: it answers pings) for a while."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            await asyncio.wait_for(ws.recv(), 0.1)
        except asyncio.TimeoutError:
            pass


def armed(rig):
    t = rig.b.tlm
    return bool(t and t["status"] & lp.TLM_ARMED)


class TestPhoneOnly(Base):
    def test_disarm_no_remote(self):
        asyncio.run(self._disarm_no_remote())

    async def _disarm_no_remote(self):
        rig = await PhoneRig(self.tmp).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: rig.b.tlm is not None and rig.b.link, "telemetry")
            self.assertFalse(armed(rig))
            self.assertFalse(rig.b.remote_seen)
            send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)                  # the two-tap Arm & Balance
            await rig.until(lambda: armed(rig) and rig.b.tlm["mode"] == lp.MODE_BALANCE, "armed + BALANCE")
            send(ws, t="set_mode", mode=lp.MODE_IDLE)                              # the Disarm button
            await rig.until(lambda: not armed(rig) and rig.b.tlm["mode"] == lp.MODE_IDLE, "disarmed")
            self.assertFalse(rig.b.estop)                                          # no latch
            self.assertFalse(rig.b.robot_estop)
            send(ws, t="set_mode", mode=lp.MODE_BALANCE)                            # a plain mode request does not re-arm
            await asyncio.sleep(0.4)
            self.assertFalse(armed(rig))
            send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)                  # the explicit step does, at once
            await rig.until(lambda: armed(rig), "armed again after a Disarm")
        finally:
            await rig.stop()

    def test_disarm_with_remote_goes_idle_but_stays_armed(self):
        asyncio.run(self._disarm_with_remote())

    async def _disarm_with_remote(self):
        rig = await PhoneRig(self.tmp, remote=True).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: rig.b.tlm is not None and rig.b.remote_seen, "remote seen")
            self.assertTrue(armed(rig))                                            # the remote's ARMED switch is on
            send(ws, t="set_mode", mode=lp.MODE_BALANCE)
            await rig.until(lambda: rig.b.tlm["mode"] == lp.MODE_BALANCE, "BALANCE")
            send(ws, t="set_mode", mode=lp.MODE_IDLE)                              # Disarm: motors off ...
            await rig.until(lambda: rig.b.tlm["mode"] == lp.MODE_IDLE, "IDLE")
            await asyncio.sleep(0.3)
            self.assertTrue(armed(rig))                                            # ... but the remote still rules arming
            self.assertFalse(rig.b.estop)
            self.assertEqual(rig.robot.mode, lp.MODE_IDLE)
            send(ws, t="set_mode", mode=lp.MODE_BALANCE)                            # the phone can still pick a mode
            await rig.until(lambda: rig.b.tlm["mode"] == lp.MODE_BALANCE, "BALANCE again")
        finally:
            await rig.stop()

    def test_stop_latch_and_release(self):
        asyncio.run(self._stop_latch())

    async def _stop_latch(self):
        rig = await PhoneRig(self.tmp).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: rig.b.tlm is not None and rig.b.link, "telemetry")
            send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)
            await rig.until(lambda: armed(rig), "armed")
            send(ws, t="estop")
            await rig.until(lambda: rig.b.estop and rig.b.robot_estop and not armed(rig), "STOP latched, disarmed")
            self.assertEqual(rig.b.tlm["mode"], lp.MODE_IDLE)
            send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)                  # refused while latched
            await asyncio.sleep(0.4)
            self.assertFalse(armed(rig))
            send(ws, t="set_mode", mode=lp.MODE_IDLE)                              # Disarm does not release the latch either
            await asyncio.sleep(0.3)
            self.assertTrue(rig.b.estop)
            send(ws, t="estop_release")
            await rig.until(lambda: not rig.b.estop and not rig.b.robot_estop, "released")
            await asyncio.sleep(0.3)
            self.assertFalse(armed(rig))                                           # release alone never arms
            send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)
            await rig.until(lambda: armed(rig), "armed after the release")
        finally:
            await rig.stop()

    # ------------------------------------------------------------------ the phone goes away
    async def _drive_to_goto(self, rig, ws):
        send(ws, t="set_mode", mode=lp.MODE_BALANCE, arm=True)
        await rig.until(lambda: rig.b.tlm["status"] & lp.TLM_UPRIGHT and rig.robot.t > 3.0, "upright", 15)
        send(ws, t="goto", points=[[4000, 0]], replace=True, go=True)
        await rig.until(lambda: rig.b.tlm["mode"] == lp.MODE_GOTO, "GOTO")
        await rig.until(lambda: abs(rig.b.tlm["cmd_speed_mms"]) > 50, "driving", 10)

    async def _paused(self, rig):
        await rig.until(lambda: abs(rig.b.tlm["cmd_speed_mms"]) < 1.0, "autonomy paused", 6)
        self.assertEqual(rig.b.tlm["mode"], lp.MODE_GOTO)                          # still balancing in GOTO, just not driving
        self.assertTrue(armed(rig))

    def test_tab_closed_pauses_autonomy(self):
        asyncio.run(self._tab_closed())

    async def _tab_closed(self):
        rig = await PhoneRig(self.tmp).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: rig.b.tlm is not None and rig.b.link, "telemetry")
            await self._drive_to_goto(rig, ws)
            self.assertEqual(rig.b.client_count(), 1)
            ws.close()                                                            # tab closed / app killed
            await rig.until(lambda: rig.b.client_count() == 0, "client count 0")
            await self._paused(rig)
            ws2 = await rig.ws()                                                  # a phone connects again: it carries on
            await rig.until(lambda: abs(rig.b.tlm["cmd_speed_mms"]) > 50, "resumed", 8)
            ws2.close()
        finally:
            await rig.stop()

    def test_silent_phone_pauses_autonomy(self):
        asyncio.run(self._silent_phone())

    async def _silent_phone(self):
        old = bridge_mod.WS_PING_S, bridge_mod.WS_LIVE_S
        bridge_mod.WS_PING_S, bridge_mod.WS_LIVE_S = 0.5, 2.0                     # (6 s in real life: faster test)
        rig = await PhoneRig(self.tmp).start()
        try:
            ws = await rig.ws()
            await rig.until(lambda: rig.b.tlm is not None and rig.b.link, "telemetry")
            await self._drive_to_goto(rig, ws)
            # a healthy page keeps answering pings: it stays a supervisor for much longer than WS_LIVE_S
            await drain(ws, rig, 4.0)
            self.assertEqual(rig.b.client_count(), 1)
            self.assertGreater(abs(rig.b.tlm["cmd_speed_mms"]), 50)
            # the phone leaves the Wi-Fi: the socket stays open (no FIN) but nothing answers any more
            await rig.until(lambda: rig.b.client_count() == 0, "silent phone no longer counts", 6)
            self.assertEqual(len(rig.b.clients), 1)                               # still connected as far as TCP knows
            await self._paused(rig)
        finally:
            bridge_mod.WS_PING_S, bridge_mod.WS_LIVE_S = old
            await rig.stop()


if __name__ == "__main__":
    unittest.main()
