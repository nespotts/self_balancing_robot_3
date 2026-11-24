"""LDROBOT LD19 lidar (issue #76): packet parser, resyncing stream, 360 degree scan assembly, serial reader.

Protocol source: "LD19 Development Manual V2.3" (LDROBOT), sections 2 (interface), 3 (data protocol) and 4
(coordinate system); the CRC table, the example packet and the angle formula below are the manual's.
- UART 230400 baud, 8N1, no flow control; the lidar only transmits (3.3 V TTL), no command is ever sent.
- Packet, 47 bytes, little endian:
    0x54 header | 0x2C VerLen (type 1 in the top 3 bits, 12 points in the low 5) | speed u16 (deg/s)
    | start angle u16 (0.01 deg) | 12 x (distance u16 mm, intensity u8) | end angle u16 (0.01 deg)
    | timestamp u16 (ms, counts up to 30000, then starts again from 0) | CRC8 (poly 0x4D, table in the manual) of the 46 bytes before it
- Point i of a packet: angle = start + (end - start) / 11 * i  (end + 360 deg when it wrapped past 360).
- Coordinates: left handed, 0 deg is the front of the sensor, angles INCREASE CLOCKWISE seen from above.
  (The ROS driver turns this into counter-clockwise for rviz.) lidar.py converts to the robot frame.
- Distance 0 = no return. Intensity: ~200 for a white target within 6 m. 4500 points/s, 10 Hz with the PWM pin
  grounded (internal speed control, +-0.1 Hz after ~3 s).
- Connector (ZH1.5T-4P): 1 Tx (out, 3.3 V), 2 PWM (in; to GND for internal speed control), 3 GND, 4 P5V (4.5 - 5.5 V).

Stdlib only: it runs on the Pi inside the bridge (Python 3.8+). No threads: bytes arrive through transport.open_link
(the event loop's add_reader on a raw tty, or the simulator's TCP socket).
"""
import asyncio
import struct
import time

from .transport import LinkError, open_link

BAUD = 230400
HEADER = 0x54
VERLEN = 0x2C
POINTS = 12
PKT_LEN = 47
POINT_RATE = 4500.0          # measurements per second (manual section 1)
NOMINAL_HZ = 10.0
TS_PERIOD_MS = 30000
MAX_SPAN_DEG = 45.0          # a packet covers ~8.8 deg at 10 Hz; a CRC-valid packet beyond this is junk
_ST = struct.Struct("<BBHH" + "HB" * POINTS + "HHB")
_HDR2 = bytes((HEADER, VERLEN))


def _make_table(poly=0x4D):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return bytes(t)


CRC_TABLE = _make_table()     # equals the manual's CrcTable (checked in tests/test_ld19.py)


def crc8(data):
    c = 0
    for b in data:
        c = CRC_TABLE[c ^ b]
    return c


class Packet:
    __slots__ = ("speed_dps", "start", "end", "ts_ms", "pts", "t")

    def __init__(self, speed_dps, start, end, ts_ms, pts):
        self.speed_dps = speed_dps
        self.start, self.end = start, end          # deg, 0 <= start < 360, end may exceed 360 (wrapped)
        self.ts_ms = ts_ms
        self.pts = pts                              # [(angle deg 0..360, distance mm, intensity)] x 12
        self.t = 0.0                                # Pi clock time of the packet's last point (set by Ld19Stream)


def parse_packet(b):
    """47 bytes -> Packet; ValueError on a bad header, CRC or angle span."""
    if len(b) != PKT_LEN:
        raise ValueError("LD19 packet is %d bytes, got %d" % (PKT_LEN, len(b)))
    if b[0] != HEADER or b[1] != VERLEN:
        raise ValueError("bad LD19 header")
    if crc8(b[:PKT_LEN - 1]) != b[PKT_LEN - 1]:
        raise ValueError("LD19 CRC mismatch")
    return _decode(_ST.unpack(b))


def _decode(f):
    speed, start, end, ts = f[2], f[3], f[4 + 2 * POINTS], f[5 + 2 * POINTS]
    s, e = start / 100.0, end / 100.0
    if e < s:
        e += 360.0                                  # the packet crosses 360 -> 0
    if e - s > MAX_SPAN_DEG:
        raise ValueError("LD19 angle span %.1f deg" % (e - s))
    step = (e - s) / (POINTS - 1)
    pts = [((s + step * i) % 360.0, f[4 + 2 * i], f[5 + 2 * i]) for i in range(POINTS)]
    return Packet(speed, s % 360.0, e, ts, pts)


def encode_packet(points, start_deg, end_deg, speed_dps=3600, ts_ms=0):
    """Build a packet (simulator / tests): points = 12 x (distance mm, intensity); angles in deg (end < start = wrapped)."""
    if len(points) != POINTS:
        raise ValueError("12 points per packet")
    vals = [HEADER, VERLEN, int(speed_dps) & 0xFFFF, int(round(start_deg * 100)) % 36000]
    for d, i in points:
        vals += [max(0, min(65535, int(d))), max(0, min(255, int(i)))]
    vals += [int(round(end_deg * 100)) % 36000, int(ts_ms) % TS_PERIOD_MS, 0]
    raw = bytearray(_ST.pack(*vals))
    raw[-1] = crc8(raw[:-1])
    return bytes(raw)


class Ld19Stream:
    """Bytes -> Packets. Resyncs on garbage (looks for 54 2C, checks the CRC) and counts what it skipped.
    Each packet gets .t: the Pi time of its last point, from the lidar's own millisecond timestamp (smooth, no
    jitter from how the bytes were chunked) anchored to the arrival time with a minimum-latency filter."""

    def __init__(self):
        self.buf = bytearray()
        self.packets = 0
        self.crc_errors = 0
        self.bytes_dropped = 0
        self.speed_dps = 0.0
        self._ts_last = None
        self._wraps = 0
        self._off = None
        self._now_last = None

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        buf = self.buf
        buf += data
        out = []
        pos, n = 0, len(buf)
        while n - pos >= PKT_LEN:
            if buf[pos] != HEADER or buf[pos + 1] != VERLEN:
                j = buf.find(_HDR2, pos + 1)
                if j < 0:
                    j = n - 1 if buf[n - 1] == HEADER else n      # a lone trailing 0x54 may start the next packet
                self.bytes_dropped += j - pos
                pos = j
                continue
            frame = bytes(buf[pos:pos + PKT_LEN])
            if crc8(frame[:PKT_LEN - 1]) != frame[PKT_LEN - 1]:
                self.crc_errors += 1
                self.bytes_dropped += 1
                pos += 1
                continue
            try:
                p = _decode(_ST.unpack(frame))
            except ValueError:
                self.crc_errors += 1
                self.bytes_dropped += 1
                pos += 1
                continue
            pos += PKT_LEN
            self.packets += 1
            self.speed_dps = p.speed_dps
            p.t = self._unwrap(p.ts_ms)
            out.append(p)
        del buf[:pos]                            # what is left is under one packet (junk is dropped above)
        if out:
            # one read may hold many packets (they all arrive "now"): anchor the offset on the newest one
            cand = now - out[-1].t
            if self._off is None or cand < self._off:
                self._off = cand
            else:                                 # let the offset creep up (clock rate difference, 500 ppm)
                self._off = min(cand, self._off + 5e-4 * max(0.0, now - (self._now_last or now)))
            self._now_last = now
            for p in out:
                p.t += self._off
        return out

    def _unwrap(self, ts):
        """Lidar timestamp (ms, wraps at 30000) -> seconds on a continuous lidar clock."""
        last = self._ts_last
        if last is not None:
            if ts < last - TS_PERIOD_MS // 2:
                self._wraps += 1                  # 30000 -> 0
            elif ts < last - 200:                 # the lidar restarted: its clock starts over
                self._wraps, self._off = 0, None
        self._ts_last = ts
        return ts / 1000.0 + self._wraps * TS_PERIOD_MS / 1000.0


class Scan:
    """One revolution: packets in rotation order. points() yields (angle deg CW from the sensor front, mm, intensity, t s)."""
    __slots__ = ("seq", "packets", "speed_dps", "t0", "t1")

    def __init__(self, seq, packets):
        self.seq = seq
        self.packets = packets
        self.speed_dps = sum(p.speed_dps for p in packets) / max(1, len(packets))
        self.t0 = packets[0].t - POINTS / POINT_RATE
        self.t1 = packets[-1].t

    def points(self):
        for p in self.packets:
            dt = 1.0 / POINT_RATE
            for i, (a, d, q) in enumerate(p.pts):
                yield a, d, q, p.t - (POINTS - 1 - i) * dt

    def __len__(self):
        return len(self.packets) * POINTS


class ScanAssembler:
    """Packets -> Scans. A scan ends when the start angle wraps (drops by more than 180 deg). The first, partial
    revolution after power-up and revolutions with under MIN_PACKETS packets (a cable pulled, a burst of junk) are dropped."""
    MIN_PACKETS = 25            # ~ 2/3 of the 37.5 packets of a turn

    def __init__(self):
        self.cur = []
        self.last_start = None
        self.seq = 0
        self.started = False
        self.partial = 0

    def add(self, p):
        done = None
        if self.last_start is not None and p.start < self.last_start - 180.0:
            if self.started and len(self.cur) >= self.MIN_PACKETS:
                self.seq += 1
                done = Scan(self.seq, self.cur)
            elif self.started or self.cur:
                self.partial += 1
            self.started = True
            self.cur = []
        self.last_start = p.start
        self.cur.append(p)
        return done


class Ld19Reader:
    """Owns the serial port: opens it (retrying quietly while it is missing), feeds the stream, calls on_scan(scan).
    port: "/dev/ttyS5", "COM7" (pyserial), or "tcp://host:port" (simulator). Counters for the Health page in .stats()."""

    RETRY_S = 3.0
    SILENT_S = 2.0

    def __init__(self, port, on_scan, say=None, clock=time.monotonic):
        self.port = port
        self.on_scan = on_scan
        self.say = say or (lambda text: None)
        self.clock = clock
        self.stream = Ld19Stream()
        self.asm = ScanAssembler()
        self.link = None
        self.error = ""
        self.last_data = None             # clock time of the last bytes
        self.last_pkt = None              # ... of the last valid packet
        self.last_scan = None             # ... of the last whole revolution
        self.scans = 0
        self.bytes = 0
        self.speed_dps = 0.0
        self._rate_t = clock()
        self._rate_pk = 0
        self._rate_sc = 0
        self.pps = 0.0
        self.scan_hz = 0.0
        self._said_open = False
        self._said_err = None

    # ---------------------------------------------------------------- port
    async def run(self):
        while True:
            try:
                await self.step()
            except asyncio.CancelledError:
                raise
            except Exception as e:                       # whatever the port does, the bridge stays up
                self.error = "%s: %s" % (type(e).__name__, e)
            await asyncio.sleep(self.RETRY_S if not self.link else 0.5)

    async def step(self):
        if not self.port or self.link:
            return
        try:
            self.link = await open_link(self.port, self._on_data, self._on_lost, BAUD)
        except LinkError as e:
            self.error = str(e)
            if self._said_err != self.error:               # say it once; the retry stays quiet
                self._said_err = self.error
                self.say("lidar: waiting for %s (%s)" % (self.port, self.error))
            return
        self.error = ""
        self._said_err = None
        self.stream = Ld19Stream()
        self.asm = ScanAssembler()
        self.last_data = self.last_pkt = None
        self.say("lidar: opened %s" % self.port)

    def _on_lost(self):
        if self.link is None:
            return
        self.link = None
        self.error = "port lost"
        self.say("lidar: lost %s - retrying" % self.port)

    def close(self):
        if self.link:
            self.link.close()
            self.link = None

    # ---------------------------------------------------------------- data
    def _on_data(self, data):
        now = self.clock()
        self.last_data = now
        self.bytes += len(data)
        for p in self.stream.feed(data, now):
            self.last_pkt = now
            self.speed_dps = p.speed_dps
            sc = self.asm.add(p)
            if sc is not None:
                self.scans += 1
                self.last_scan = now
                try:
                    self.on_scan(sc)
                except Exception as e:                    # a bug in mapping must not stop the reader
                    self.error = "scan handler: %s: %s" % (type(e).__name__, e)

    def tick(self, now):
        """~1 Hz: packet and scan rates."""
        dt = now - self._rate_t
        if dt >= 0.9:
            pk, sc = self.stream.packets, self.scans
            self.pps = (pk - self._rate_pk) / dt
            self.scan_hz = (sc - self._rate_sc) / dt
            self._rate_t, self._rate_pk, self._rate_sc = now, pk, sc

    def state(self, now=None):
        """off | waiting (port missing) | no data (port open, nothing arrives) | junk (bytes but no packets) | ok | stale"""
        now = self.clock() if now is None else now
        if not self.port:
            return "off"
        if not self.link:
            return "waiting"
        if self.last_pkt is not None and now - self.last_pkt < self.SILENT_S:
            return "ok"
        if self.last_pkt is not None:
            return "stale"
        return "junk" if self.bytes else "no data"

    def stats(self, now=None):
        now = self.clock() if now is None else now
        s = self.stream
        return {"port": self.port, "state": self.state(now), "error": self.error,
                "pps": round(self.pps, 1), "scan_hz": round(self.scan_hz, 1),
                "rpm": round(self.speed_dps / 6.0, 1),                      # deg/s / 360 * 60
                "packets": s.packets, "crc_errors": s.crc_errors, "bytes_dropped": s.bytes_dropped,
                "scans": self.scans, "partial": self.asm.partial,
                "scan_age_s": None if self.last_scan is None else round(now - self.last_scan, 2)}


def main(argv=None):
    """python3 -m station.bridge.ld19 [/dev/ttyS5] [seconds]: wiring check without the bridge (stop the bridge first:
    one reader per port). Prints packets/s, rpm and CRC errors for a few seconds."""
    import sys
    args = list(sys.argv[1:] if argv is None else argv)
    port = args[0] if args else "/dev/ttyS5"
    secs = float(args[1]) if len(args) > 1 else 5.0
    rd = Ld19Reader(port, lambda sc: None, say=lambda t: print(t, flush=True))

    last = {"state": None}

    async def go():
        task = asyncio.ensure_future(rd.run())
        end = time.monotonic() + secs
        while time.monotonic() < end:
            await asyncio.sleep(1.0)
            rd.tick(time.monotonic())
            st = rd.stats()
            last["state"] = st["state"]          # before close(): a closed reader no longer reports "ok"
            print("%-8s %5.0f packets/s  %5.1f rpm  %d scans  %d bad CRC  %d bytes skipped  %s" % (
                st["state"], st["pps"], st["rpm"], st["scans"], st["crc_errors"], st["bytes_dropped"], st["error"]), flush=True)
        task.cancel()
        rd.close()
    asyncio.run(go())
    ok = last["state"] == "ok"
    print("RESULT:", "lidar OK" if ok else "no valid LD19 packets (see docs/ld19.md, Troubleshooting)")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
