"""LD19 through the Teensy (issue #100): raw LD19 bytes in MSG_LIDAR_RAW frames (link v7) -> the same Ld19Stream /
ScanAssembler / Lidar pipeline as a serial port. Real packets (the manual's example + generated ones) split at random
across frames, with lost frames, overflow flags and seq wrap; source selection (Teensy / USB / UART); timestamps that do
not care about the link's delivery jitter; the simulator end to end."""
import asyncio
import os
import random
import shutil
import tempfile
import time
import unittest

from station.bridge import health as health_mod
from station.bridge import ld19
from station.bridge import link_protocol as lp
from station.bridge.bridge import Bridge
from station.bridge.occgrid import UNKNOWN
from station.bridge.tests.test_bridge_sim import WEB, until
from station.bridge.tests.test_ld19 import EXAMPLE, FakeRobot
from station.bridge.tests.test_lidar import cell_max
from station.sim.lidar_sim import LidarSim
from station.sim.sim import SimRobot, SimServer


def lidar_bytes(seconds, seed=1, dt=0.004):
    """seconds of simulated LD19 output (real packets, 375/s) -> bytes, the manual's example packet first."""
    r = FakeRobot()
    sim = LidarSim(seed=seed)
    out = bytearray(EXAMPLE)
    end = seconds
    while r.t < end:
        r.t += dt
        out += sim.step(r)
    return bytes(out)


def chunks(data, rng, sizes=(1, 2, 7, 46, 47, 48, 93, 179, 180)):
    """Arbitrary split points, never respecting the 47-byte packets."""
    i = 0
    while i < len(data):
        n = min(rng.choice(sizes), lp.LINK_LIDAR_RAW_MAX)
        yield data[i:i + n]
        i += n


def frames_of(data, rng, seq0=0):
    """-> [(seq, wire frame)] for data split at random."""
    return [((seq0 + k) & 0xFF, lp.encode_lidar_raw(seq0 + k, c)) for k, c in enumerate(chunks(data, rng))]


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TestFrames(unittest.TestCase):
    def test_encode_decode(self):
        f = lp.encode_lidar_raw(300, b"\x54\x2c\x01", flags=lp.LINK_LIDAR_RAW_OVERFLOW)
        (t, p), = lp.FrameParser().feed(f)
        self.assertEqual(t, lp.MSG_LIDAR_RAW)
        self.assertEqual(lp.decode(t, p), {"seq": 300 & 0xFF, "flags": 1, "data": b"\x54\x2c\x01"})
        self.assertEqual(len(lp.encode_lidar_raw(0, bytes(180))), 6 + 182)
        for bad in (b"", bytes(181)):
            with self.assertRaises(ValueError):
                lp.encode_lidar_raw(0, bad)
        for bad in (b"\x00\x00", bytes(183)):
            with self.assertRaises(ValueError):
                lp.decode_lidar_raw(bad)
        self.assertEqual(lp.MSG_NAMES[0x07], "MSG_LIDAR_RAW")

    def test_crc_is_the_same_as_the_python_table(self):
        rng = random.Random(5)
        for n in (0, 1, 2, 47, 184, 999):
            d = bytes(rng.randrange(256) for _ in range(n))
            self.assertEqual(lp.crc16(d), lp.crc16_py(d))
            self.assertEqual(lp.crc16(d, 0x1D0F), lp.crc16_py(d, 0x1D0F))

    def test_packer_chunks_like_the_firmware(self):
        pk = lp.LidarRawPacker()
        data = lidar_bytes(0.5)
        out, t, i = [], 0.0, 0
        while i < len(data):                        # 17.6 kB/s arriving every 4 ms (~70 bytes)
            out += pk.feed(data[i:i + 70], t)
            i += 70
            t += 0.004
        out += pk.feed(b"", t + 0.02)               # the flush timer sends the rest
        dec = [lp.decode(*lp.FrameParser().feed(f)[0]) for f in out]
        self.assertEqual(b"".join(d["data"] for d in dec), data)
        self.assertEqual([d["seq"] for d in dec], [k & 0xFF for k in range(len(dec))])
        self.assertTrue(all(len(d["data"]) <= 180 for d in dec))
        self.assertGreater(sum(len(d["data"]) == 180 for d in dec), 0.9 * len(dec) - 1)


class TestTeensyPath(unittest.TestCase):
    def reader(self, port=""):
        clk = Clock()
        got = []
        rd = ld19.Ld19Reader(port, got.append, clock=clk, teensy=True)
        return rd, got, clk

    def feed_wire(self, rd, wire, clk, dt=0.0):
        """Wire bytes -> FrameParser -> MSG_LIDAR_RAW decode -> feed_teensy, as the bridge does."""
        p = lp.FrameParser()
        for t, pl in p.feed(wire):
            self.assertEqual(t, lp.MSG_LIDAR_RAW)
            d = lp.decode(t, pl)
            clk.t += dt
            rd.feed_teensy(d["seq"], d["flags"], d["data"])
        return p

    def test_random_split_assembles_whole_turns_with_crc(self):
        data = lidar_bytes(3.0)
        rng = random.Random(7)
        fr = frames_of(data, rng, seq0=250)                 # seq wraps 255 -> 0 at once, then every 256 frames
        rd, got, clk = self.reader()
        packets = []
        orig = rd.teensy.stream.feed

        def spy(d, now=None):
            out = orig(d, now)
            packets.extend(out)
            return out
        rd.teensy.stream.feed = spy
        p = self.feed_wire(rd, b"".join(f for _s, f in fr), clk, dt=0.003)
        self.assertEqual(p.crc_errors, 0)
        st = rd.stats(clk.t)
        self.assertEqual(st["state"], "ok")
        self.assertEqual(st["source"], "teensy")
        self.assertEqual(st["source_label"], "Teensy")
        self.assertEqual(st["teensy"]["seq_gaps"], 0)
        self.assertEqual(st["teensy"]["frames"], len(fr))
        self.assertEqual(st["crc_errors"], 0)
        self.assertEqual(st["bytes_dropped"], 0)
        self.assertEqual(st["packets"], len(data) // 47)          # every packet, the manual's example included
        self.assertEqual(packets[0].speed_dps, 2152)
        self.assertAlmostEqual(packets[0].start, 324.27)
        self.assertEqual(packets[0].pts[0][1:], (224, 228))
        self.assertGreaterEqual(len(got), 28)                      # ~30 turns, the first (partial) dropped
        for sc in got:
            self.assertIn(len(sc.packets), (37, 38, 39))

    def test_lost_frames_and_overflow_only_cost_the_packets_they_touch(self):
        data = lidar_bytes(3.0, seed=2)
        rng = random.Random(11)
        fr = frames_of(data, rng)
        kept, lost_bytes = [], 0
        drop = set(rng.sample(range(10, len(fr) - 10), 12))
        flag_at = sorted(set(rng.sample(range(10, len(fr) - 10), 4)) - drop)
        k = 0
        for c in chunks(data, random.Random(11)):                  # the same split again, to rebuild with gaps / flags
            if k in drop:
                lost_bytes += len(c)
            else:
                kept.append(lp.encode_lidar_raw(k, c, flags=lp.LINK_LIDAR_RAW_OVERFLOW if k in flag_at else 0))
            k += 1
        rd, got, clk = self.reader()
        self.feed_wire(rd, b"".join(kept), clk, dt=0.003)
        st = rd.stats(clk.t)
        self.assertEqual(st["teensy"]["seq_gaps"], len(drop))
        self.assertEqual(st["teensy"]["overflows"], len(flag_at))
        total = len(data) // 47
        holes = len(drop) + len(flag_at)
        self.assertGreaterEqual(st["packets"], total - lost_bytes // 47 - 2 * holes)
        self.assertLess(st["packets"], total)
        self.assertEqual(st["state"], "ok")
        self.assertGreaterEqual(len(got), 20)

    def test_link_jitter_does_not_move_the_packet_times(self):
        """Frames delivered 0-30 ms late at random: the packet times still follow the LD19's own ms clock."""
        data = lidar_bytes(4.0, seed=3)[47:]                       # without the example (its timestamp is unrelated)
        rd, _got, clk = self.reader()
        times = []
        orig = rd.teensy.stream.feed

        def spy(d, now=None):
            out = orig(d, now)
            times.extend((p.t, p.ts_ms) for p in out)
            return out
        rd.teensy.stream.feed = spy
        rng = random.Random(4)
        pk = lp.LidarRawPacker()
        t, i = 0.0, 0
        frames = []
        while i < len(data):                                        # the Teensy chunks in real time (17.6 kB/s)
            for f in pk.feed(data[i:i + 141], t):
                frames.append((t, f))
            i += 141
            t += 0.008
        arrive = 0.0
        for t_sent, f in frames:
            arrive = max(arrive, 100.0 + t_sent + rng.uniform(0.0, 0.030))   # sent + link / scheduling jitter
            clk.t = arrive
            self.feed_wire(rd, f, clk)
        res = [pt - ts / 1000.0 for pt, ts in times if pt > times[0][0] + 1.0]   # after 1 s of settling
        self.assertGreater(len(res), 1000)
        self.assertLess(max(res) - min(res), 0.004)                # vs the 30 ms of arrival jitter

    def test_teensy_takes_over_from_a_uart_that_reads_junk(self):
        """Tonight's H618 case: /dev/ttyS5 open but every byte's top bits wrong; the Teensy's frames win (auto)."""
        data = lidar_bytes(2.0, seed=5)
        rd, got, clk = self.reader("/dev/ttyS5")
        rd.link = object()                                          # as if open
        junk = bytes((b ^ 0xC0) if b & 0x40 else b for b in data)  # "54 2C" reads "94 2C"
        rng = random.Random(1)
        fr = frames_of(data, rng)
        j = 0
        for k, (_s, f) in enumerate(fr):
            clk.t += 0.004
            rd._on_data(junk[j:j + 70])
            j += 70
            self.feed_wire(rd, f, clk)
        st = rd.stats(clk.t)
        self.assertEqual(st["state"], "ok")
        self.assertEqual(st["source"], "teensy")
        self.assertEqual(st["source_label"], "Teensy")
        self.assertGreater(len(got), 10)

    def test_a_working_usb_adapter_keeps_priority_until_it_stops(self):
        data = lidar_bytes(6.0, seed=6)
        port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        said = []
        rd = ld19.Ld19Reader(port, lambda sc: None, say=said.append, clock=Clock(), teensy=True)
        clk = rd.clock
        rd.link = object()
        pk = lp.LidarRawPacker()
        i = 0
        while i < len(data):
            clk.t += 0.004
            if clk.t < 103.0:                                       # the adapter works for 3 s, then is unplugged
                rd._on_data(data[i:i + 70])
            for f in pk.feed(data[i:i + 70], clk.t):                # the Teensy forwards the same lidar all along
                self.feed_wire(rd, f, clk)
            i += 70
            if 102.0 < clk.t < 102.1:
                self.assertEqual(rd.source(), "serial")
                self.assertEqual(rd.source_label(), "USB")
        self.assertEqual(rd.source(), "teensy")                     # took over SILENT_S after the adapter stopped
        self.assertEqual(rd.state(clk.t), "ok")
        self.assertTrue(any("Teensy" in s for s in said), said)
        self.assertEqual(ld19.source_label("/dev/ttyS5"), "UART")
        self.assertEqual(ld19.source_label("/dev/ttyUSB0"), "USB")
        self.assertEqual(ld19.source_label("COM7"), "USB")
        self.assertEqual(ld19.source_label("tcp://127.0.0.1:5762"), "TCP")

    def test_teensy_only_waits_quietly_and_a_serial_only_reader_ignores_frames(self):
        rd, _got, clk = self.reader()
        self.assertEqual(rd.state(), "waiting")
        self.assertEqual(rd.stats()["port"], "teensy")
        tile = health_mod.lidar_tile(dict(rd.stats(), enabled=True, flowing=False))
        self.assertEqual(tile["level"], "na")
        self.assertIn("Teensy", tile["why"])
        plain = ld19.Ld19Reader("/dev/ttyS5", lambda sc: None)
        plain.feed_teensy(0, 0, EXAMPLE)
        self.assertEqual(plain.raw["ignored"], 1)
        self.assertEqual(plain.teensy.bytes, 0)

    def test_health_tile_shows_the_source_and_lost_chunks(self):
        tile = health_mod.lidar_tile({"state": "ok", "pps": 375, "rpm": 600, "crc_errors": 0, "packets": 9000,
                                      "scan_age_s": 0.05, "flowing": True, "source_label": "Teensy",
                                      "teensy": {"frames": 2000, "seq_gaps": 3, "overflows": 0}})
        self.assertEqual(tile["label"], "Lidar · Teensy")
        self.assertEqual(tile["level"], "ok")
        self.assertIn("3 chunks lost", tile["why"])
        bad = health_mod.lidar_tile({"state": "ok", "pps": 300, "rpm": 600, "crc_errors": 0, "packets": 9000,
                                     "scan_age_s": 0.05, "flowing": True, "source_label": "Teensy",
                                     "teensy": {"frames": 1000, "seq_gaps": 100, "overflows": 2}})
        self.assertEqual(bad["level"], "warn")
        usb = health_mod.lidar_tile({"state": "ok", "pps": 375, "rpm": 600, "crc_errors": 0, "packets": 9000,
                                     "scan_age_s": 0.05, "flowing": True, "source_label": "USB"})
        self.assertEqual(usb["label"], "Lidar · USB")
        self.assertNotIn("chunks", usb["why"])


class TestBridge(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_bridge_without_a_lidar_ignores_the_frames(self):
        """A bridge with --lidar off (like a v6 bridge, which does not know the type) just drops them."""
        async def go():
            b = Bridge("tcp://127.0.0.1:1", WEB, self.tmp, "127.0.0.1", 0, quiet=True)
            fr = frames_of(lidar_bytes(0.3), random.Random(2))
            b._on_data(b"".join(f for _s, f in fr) + lp.encode_log("after"))
            self.assertEqual(b.parser.crc_errors, 0)
            self.assertEqual(b.logs[-1]["text"], "after")
            self.assertIsNone(b.lidar)
        asyncio.run(go())

    def test_firmware_log_line_counters_reach_the_status(self):
        async def go():
            b = Bridge("tcp://127.0.0.1:1", WEB, self.tmp, "127.0.0.1", 0, quiet=True, lidar_port="teensy")
            b._on_data(lp.encode_log("lidar: 17.6 kB/s, 98 frames/s, usb drop 4, overflow 1, max 180 B / 21.0 us per pass"))
            st = b.status()["lidar"]
            self.assertEqual(st["teensy"]["fw_usb_drop"], 4)
            self.assertEqual(st["teensy"]["fw_overflow"], 1)
            self.assertEqual(st["port"], "teensy")
        asyncio.run(go())

    def test_cli_sources(self):
        from station.bridge.__main__ import build_parser, lidar_source
        self.assertEqual(lidar_source("teensy"), "teensy")
        self.assertEqual(lidar_source("auto"), "teensy")
        self.assertIsNone(lidar_source("off"))
        self.assertEqual(lidar_source(" /dev/ttyUSB0 "), "/dev/ttyUSB0")
        old = os.environ.pop("SBR3_LIDAR", None)
        try:
            self.assertEqual(build_parser().parse_args([]).lidar, "teensy")
        finally:
            if old is not None:
                os.environ["SBR3_LIDAR"] = old

    def test_sim_end_to_end_through_the_teensy(self):
        """The sim's LD19 on its fake Teensy's Serial7 -> MSG_LIDAR_RAW on the robot link -> the bridge: the map fills."""
        asyncio.run(self._sim())

    async def _sim(self):
        robot = SimRobot(seed=8)
        robot.set_param(robot.pidx["rng.enable"], 0.0)              # like the robot: the sweep shares D28 / D29
        robot.out = []
        sim = SimServer(robot, "127.0.0.1", 0, quiet=True, lidar=LidarSim(seed=8), lidar_teensy=True)
        srv = await asyncio.start_server(sim._client, "127.0.0.1", 0)
        port = srv.sockets[0].getsockname()[1]
        phys = asyncio.ensure_future(sim.physics())
        b = Bridge("tcp://127.0.0.1:%d" % port, WEB, self.tmp, "127.0.0.1", 0,
                   release_file=os.path.join(self.tmp, "release"), quiet=True,
                   preset_dir=os.path.join(self.tmp, "presets"), lidar_port="teensy")
        tasks = [asyncio.ensure_future(b.link_task()), asyncio.ensure_future(b.tick_task()),
                 asyncio.ensure_future(b.lidar.run())]
        try:
            await until(lambda: b.tlm is not None)
            b.handle_command({"t": "set_mode", "mode": lp.MODE_STATION, "arm": True}, "test")
            await until(lambda: b.tlm["status"] & lp.TLM_UPRIGHT, 4.0)
            await until(lambda: b.lidar.n["used"] >= 20, 8.0)
            self.assertGreaterEqual(b.lidar.n["used"], 20)
            st = b.lidar.status()
            self.assertEqual(st["state"], "ok")
            self.assertEqual(st["source"], "teensy")
            self.assertAlmostEqual(st["rpm"], 600, delta=3)
            self.assertAlmostEqual(st["pps"], 375, delta=40)
            self.assertEqual(st["crc_errors"], 0)
            self.assertEqual(st["teensy"]["seq_gaps"], 0)
            self.assertGreater(st["teensy"]["frames"], 300)
            self.assertEqual(b.parser.crc_errors, 0)
            g = b.mapper.grid
            self.assertGreater(cell_max(g, 4000, 0, 150), UNKNOWN + 30)     # far wall
            self.assertLess(g.value(2000, -200), UNKNOWN - 5)               # open floor
            tile = [t for t in b.health.tick(time.monotonic(), b.health_ctx())["tiles"] if t["id"] == "lidar"][0]
            self.assertEqual(tile["label"], "Lidar · Teensy")
            self.assertEqual(tile["level"], "ok", tile)
            self.assertTrue(any(l["text"] == "lidar: reading the LD19 from the Teensy (MSG_LIDAR_RAW)" for l in b.logs))
            await until(lambda: b.lidar_obst.n["sent"] > 5, 5.0)           # obstacles go back to the navigator too
            self.assertGreater(b.lidar_obst.n["sent"], 5)
        finally:
            for t in tasks + [phys]:
                t.cancel()
            b.lidar.reader.close()
            srv.close()


if __name__ == "__main__":
    unittest.main()
