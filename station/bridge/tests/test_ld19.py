"""LD19 packet parser, CRC, resync, scan assembly (issue #76). No hardware needed."""
import asyncio
import random
import unittest

from station.bridge import ld19
from station.sim.lidar_sim import LidarSim

# The manual's CrcTable (LD19 Development Manual V2.3, section 3.1), copied from the PDF text.
MANUAL_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]
# The manual's example (section 3.3): speed 2152 deg/s, start 324.27, end 334.7, first point 224 mm / 228
EXAMPLE = bytes.fromhex(
    "542C6808AB7EE000E4DC00E2D900E5D500E3D300E4D000E9CD00E4CA00E2C700E9C500E5C200E5C000E5BE823A1A50")


class FakeRobot:
    """What LidarSim needs from a SimRobot."""
    def __init__(self, room=None):
        from station.sim.sim import Room
        self.room = room or Room()
        self.t = 0.0
        self.X = self.Y = self.TH = 0.0
        self.theta = 0.0


def packets_of(robot, seconds, sim=None, dt=0.004, stream=None):
    """Run the simulated LD19 for some seconds, return the parsed packets (the stream gets sim time as 'arrival time')."""
    sim = sim or LidarSim(seed=1)
    stream = stream or ld19.Ld19Stream()
    out = []
    end = robot.t + seconds
    while robot.t < end:
        robot.t += dt
        out += stream.feed(sim.step(robot), now=robot.t)
    return out


class TestPacket(unittest.TestCase):
    def test_crc_table_is_the_manuals(self):
        self.assertEqual(list(ld19.CRC_TABLE), MANUAL_TABLE)

    def test_manual_example(self):
        self.assertEqual(len(EXAMPLE), 47)
        p = ld19.parse_packet(EXAMPLE)
        self.assertEqual(p.speed_dps, 2152)
        self.assertAlmostEqual(p.start, 324.27)
        self.assertAlmostEqual(p.end, 334.70)
        self.assertEqual(p.ts_ms, 0x1A3A)
        self.assertEqual(p.pts[0][1:], (224, 228))           # 00E0 mm, intensity E4
        self.assertEqual(p.pts[1][1:], (220, 226))           # DC 00 E2 (the manual's prose says 200 / 176: typos)
        self.assertEqual(p.pts[11][1], 192)
        self.assertEqual(len(p.pts), 12)
        # the manual's interpolation: step = (end - start) / (len - 1)
        step = (334.70 - 324.27) / 11
        for i, (a, _d, _q) in enumerate(p.pts):
            self.assertAlmostEqual(a, 324.27 + step * i, places=6)

    def test_crc_catches_a_flipped_byte(self):
        for k in (0, 1, 5, 20, 40, 45, 46):
            bad = bytearray(EXAMPLE)
            bad[k] ^= 0x01
            with self.assertRaises(ValueError):
                ld19.parse_packet(bytes(bad))
        with self.assertRaises(ValueError):
            ld19.parse_packet(EXAMPLE[:-1])

    def test_roundtrip_and_wrap(self):
        pts = [(100 + i, 50 + i) for i in range(12)]
        p = ld19.parse_packet(ld19.encode_packet(pts, 355.0, 3.8, speed_dps=3600, ts_ms=123))   # crosses 360
        self.assertEqual(p.speed_dps, 3600)
        self.assertEqual(p.ts_ms, 123)
        self.assertAlmostEqual(p.pts[0][0], 355.0)
        self.assertAlmostEqual(p.pts[-1][0], 3.8)
        ang = [a for a, _, _ in p.pts]
        unwrapped = [a + 360.0 if a < 180 else a for a in ang]          # monotone once the wrap is undone
        self.assertEqual(unwrapped, sorted(unwrapped))
        self.assertAlmostEqual(unwrapped[1] - unwrapped[0], 0.8, places=3)
        self.assertEqual([(d, q) for _, d, q in p.pts], pts)
        self.assertAlmostEqual(p.end, 363.8)

    def test_no_wrap_edge_cases(self):
        p = ld19.parse_packet(ld19.encode_packet([(1, 1)] * 12, 0.0, 8.8))
        self.assertAlmostEqual(p.pts[0][0], 0.0)
        self.assertAlmostEqual(p.pts[11][0], 8.8)
        p = ld19.parse_packet(ld19.encode_packet([(1, 1)] * 12, 351.2, 0.0))      # last point exactly at 0 / 360
        self.assertAlmostEqual(p.pts[11][0] % 360.0, 0.0, places=6)
        with self.assertRaises(ValueError):                                       # CRC fine, angles nonsense
            ld19.parse_packet(ld19.encode_packet([(1, 1)] * 12, 10.0, 200.0))


class TestStream(unittest.TestCase):
    def test_resync_on_garbage_and_chunking(self):
        pkts = [ld19.encode_packet([(500 + k, 200)] * 12, a, a + 8.8, ts_ms=1000 + k * 3)
                for k, a in enumerate((0.0, 8.8, 17.6, 26.4, 35.2))]
        junk = bytes([0x54, 0x00, 0x2C, 0x54, 0x54, 0x2C, 0x11, 0xFF])       # includes a fake 54 2C header
        wire = junk + pkts[0] + b"\x00\x01" + pkts[1] + pkts[2][:20]
        wire2 = pkts[2][20:] + junk + pkts[3] + pkts[4]
        s = ld19.Ld19Stream()
        got = []
        rng = random.Random(3)
        for blob in (wire, wire2):                                            # random chunk sizes, even 1 byte
            i = 0
            while i < len(blob):
                n = rng.choice((1, 3, 7, 50))
                got += s.feed(blob[i:i + n], now=1.0)
                i += n
        self.assertEqual(len(got), 5)
        self.assertEqual([round(p.start, 1) for p in got], [0.0, 8.8, 17.6, 26.4, 35.2])
        self.assertGreater(s.bytes_dropped, 8)
        self.assertEqual(s.packets, 5)

    def test_bad_crc_is_counted_and_skipped(self):
        good = ld19.encode_packet([(500, 200)] * 12, 10.0, 18.8)
        bad = bytearray(good)
        bad[10] ^= 0xFF
        s = ld19.Ld19Stream()
        got = s.feed(bytes(bad) + good + bytes(bad) + good, now=0.0)
        self.assertEqual(len(got), 2)
        self.assertEqual(s.crc_errors, 2)

    def test_timestamp_wraps_at_30000(self):
        s = ld19.Ld19Stream()
        t = []
        for k, ts in enumerate((29970, 29990, 10, 30, 50)):          # 30000 -> 0 between the 2nd and 3rd
            t += [p.t for p in s.feed(ld19.encode_packet([(1, 1)] * 12, 0.0, 8.8, ts_ms=ts), now=100.0 + 0.02 * k)]
        self.assertEqual(len(t), 5)
        self.assertEqual(t, sorted(t))
        self.assertAlmostEqual(t[3] - t[1], 0.04, places=3)           # 20 + 20 ms across the wrap

    def test_pi_time_comes_from_the_lidar_clock_not_from_chunking(self):
        s = ld19.Ld19Stream()
        raw = b"".join(ld19.encode_packet([(1, 1)] * 12, 8.8 * k, 8.8 * k + 8.8, ts_ms=1000 + k * 3) for k in range(10))
        got = s.feed(raw, now=50.0)                                   # all 10 arrive in one read
        dts = [b.t - a.t for a, b in zip(got, got[1:])]
        for d in dts:
            self.assertAlmostEqual(d, 0.003, places=6)
        self.assertAlmostEqual(got[-1].t, 50.0, places=6)             # anchored on the newest packet


class TestScans(unittest.TestCase):
    def test_assembles_whole_turns_from_the_sim(self):
        r = FakeRobot()
        pk = packets_of(r, 1.05)
        asm = ld19.ScanAssembler()
        scans = [sc for sc in (asm.add(p) for p in pk) if sc]
        self.assertGreaterEqual(len(scans), 9)                         # ~10 turns, the first one (partial) is dropped
        self.assertLessEqual(len(scans), 10)
        for sc in scans:
            self.assertIn(len(sc.packets), (37, 38, 39))               # 450 points = 37.5 packets
            self.assertAlmostEqual(sc.speed_dps, 3600, delta=15)
            pts = list(sc.points())
            self.assertEqual(len(pts), len(sc))
            ang = sorted(a for a, _, _, _ in pts)
            self.assertLess(ang[0], 10.0)                              # whole circle covered
            self.assertGreater(ang[-1], 350.0)
            gaps = [b - a for a, b in zip(ang, ang[1:])]
            self.assertLess(max(gaps), 2.0)
            ts = [t for _, _, _, t in pts]
            self.assertTrue(all(b >= a - 0.0011 for a, b in zip(ts, ts[1:])))      # ms timestamps: +-1 ms quantisation
            self.assertAlmostEqual(sc.t1 - sc.t0, 0.1, delta=0.01)    # one turn takes 100 ms
        for a, b in zip(scans, scans[1:]):
            self.assertAlmostEqual(b.t1 - a.t1, 0.1, delta=0.006)      # 10 Hz
            self.assertEqual(b.seq, a.seq + 1)

    def test_short_or_first_turn_is_dropped(self):
        asm = ld19.ScanAssembler()
        mk = lambda a: ld19.parse_packet(ld19.encode_packet([(1, 1)] * 12, a, a + 8.8))
        out = [asm.add(mk((k * 8.8) % 360)) for k in range(100)]       # 2.7 turns
        self.assertEqual(len([s for s in out if s]), 1)                # turn 1 is partial-or-whole, never a short scan
        asm = ld19.ScanAssembler()
        for a in (350.0, 10.0, 20.0, 30.0, 5.0, 15.0):                  # tiny stretches that wrap: never emitted
            self.assertIsNone(asm.add(mk(a)))

    def test_lost_packets_still_finish_turns(self):
        r = FakeRobot()
        sim = LidarSim(seed=2, lost_packets=0.1, bad_packets=0.05)
        stream = ld19.Ld19Stream()
        pk = packets_of(r, 2.0, sim, stream=stream)
        asm = ld19.ScanAssembler()
        scans = [sc for sc in (asm.add(p) for p in pk) if sc]
        self.assertGreaterEqual(len(scans), 17)
        self.assertGreater(stream.crc_errors, 0)


class TestReader(unittest.TestCase):
    def test_missing_port_is_quiet_and_does_not_spin(self):
        said = []
        rd = ld19.Ld19Reader("/dev/ttyS5-does-not-exist", lambda sc: None, say=said.append)
        rd.RETRY_S = 0.01

        async def go():
            task = asyncio.ensure_future(rd.run())
            await asyncio.sleep(0.3)
            task.cancel()
        asyncio.run(go())
        self.assertEqual(rd.state(), "waiting")
        self.assertEqual(len(said), 1)                                 # said once, retried silently
        self.assertTrue(rd.error)
        self.assertEqual(rd.stats()["packets"], 0)

    def test_off_without_a_port(self):
        rd = ld19.Ld19Reader("", lambda sc: None)
        self.assertEqual(rd.state(), "off")

    def test_counts_packets_scans_crc_and_rpm(self):
        r = FakeRobot()
        sim = LidarSim(seed=4, bad_packets=0.02)
        got = []
        now = [0.0]
        rd = ld19.Ld19Reader("tcp://x", got.append, clock=lambda: now[0])
        rd.link = object()                                              # as if open
        for _ in range(500):                                            # 2 s of sim time
            r.t += 0.004
            now[0] = r.t
            rd._on_data(sim.step(r))
            rd.tick(now[0])
        st = rd.stats(now[0])
        self.assertEqual(st["state"], "ok")
        self.assertAlmostEqual(st["pps"], 375, delta=25)               # 4500 / 12
        self.assertAlmostEqual(st["scan_hz"], 10, delta=1.5)
        self.assertAlmostEqual(st["rpm"], 600, delta=2)
        self.assertGreater(st["crc_errors"], 0)
        self.assertGreaterEqual(st["scans"], 18)
        self.assertEqual(len(got), st["scans"])
        self.assertLess(st["scan_age_s"], 0.2)
        now[0] += 5.0
        self.assertEqual(rd.state(now[0]), "stale")


if __name__ == "__main__":
    unittest.main()
