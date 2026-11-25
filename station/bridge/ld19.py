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


_CRC2 = [None]                 # two bytes at a time: table[(c << 8) ^ (b0 << 8 | b1)] (64 KB, built on first use)
_WORDS = struct.Struct(">%dH" % ((PKT_LEN - 1) // 2))


def crc8_packet(frame):
    """crc8 of a packet's first 46 bytes, a 16-bit word per step (#110: half the Python steps per packet)."""
    t = _CRC2[0]
    if t is None:
        T = CRC_TABLE
        t = _CRC2[0] = bytes(T[T[i >> 8] ^ (i & 0xFF)] for i in range(65536))
    c = 0
    for w in _WORDS.unpack_from(frame):
        c = t[(c << 8) ^ w]
    return c


class Packet:
    """One packet. dist / qual: the 12 distances (mm) / intensities as tuples (lidar.py works on these directly, #110);
    pts: [(angle deg 0..360, distance mm, intensity)] x 12, built on first use."""
    __slots__ = ("speed_dps", "start", "end", "ts_ms", "dist", "qual", "step", "t", "_pts")

    def __init__(self, speed_dps, start, end, ts_ms, dist, qual):
        self.speed_dps = speed_dps
        self.start, self.end = start, end          # deg, 0 <= start < 360, end may exceed 360 (wrapped)
        self.ts_ms = ts_ms
        self.dist, self.qual = dist, qual
        self.step = (end - start) / (POINTS - 1)    # deg between two points
        self.t = 0.0                                # Pi clock time of the packet's last point (set by Ld19Stream)
        self._pts = None

    @property
    def pts(self):
        if self._pts is None:
            s, step = self.start, self.step
            self._pts = [((s + step * i) % 360.0, d, q) for i, (d, q) in enumerate(zip(self.dist, self.qual))]
        return self._pts


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
    p = Packet(speed, s % 360.0, e, ts, f[4:4 + 2 * POINTS:2], f[5:5 + 2 * POINTS:2])
    p.step = (e - s) / (POINTS - 1)
    return p


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
            if crc8_packet(frame) != frame[PKT_LEN - 1]:
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
    __slots__ = ("seq", "packets", "speed_dps", "t0", "t1", "turn")

    def __init__(self, seq, packets):
        self.seq = seq
        self.packets = packets
        self.turn = False                 # lidar.Turn once deskewed (None: could not be), False: not yet (#110)
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


TEENSY = "teensy"


def source_label(port):
    """Health tile name of a byte source: "Teensy" (MSG_LIDAR_RAW over the robot link, #100), "USB" (a USB serial
    adapter), "UART" (a header UART), "TCP" (the simulator's fake port)."""
    p = (port or "").lower()
    if p == TEENSY:
        return "Teensy"
    if p.startswith("tcp://"):
        return "TCP"
    if p.startswith(("/dev/ttyusb", "/dev/ttyacm", "/dev/serial/by-id", "/dev/serial/by-path", "com")):
        return "USB"
    return "UART"


class _Source:
    """One byte source's own parser (packets never mix across sources): the serial port, or the Teensy's frames."""
    __slots__ = ("name", "stream", "asm", "bytes", "last_data", "last_pkt")

    def __init__(self, name):
        self.name = name
        self.reset()

    def reset(self):
        self.stream = Ld19Stream()
        self.asm = ScanAssembler()
        self.bytes = 0
        self.last_data = None             # clock time of the last bytes
        self.last_pkt = None              # ... of the last valid packet


class Ld19Reader:
    """Owns the serial port: opens it (retrying quietly while it is missing), feeds the stream, calls on_scan(scan).
    port: "/dev/ttyS5", "/dev/serial/by-id/...", "COM7" (pyserial), "tcp://host:port" (simulator), or "" = none.
    teensy: also take the LD19 bytes the Teensy forwards (MSG_LIDAR_RAW, link v7, #100) through feed_teensy().
    With both, scans come from one source at a time: the one that has valid packets, the current one first (a second
    source takes over only once the current one has had no valid packet for SILENT_S). Counters for Health in .stats()."""

    RETRY_S = 3.0
    SILENT_S = 2.0

    def __init__(self, port, on_scan, say=None, clock=time.monotonic, teensy=False):
        self.port = port
        self.teensy_on = bool(teensy)
        self.on_scan = on_scan
        self.say = say or (lambda text: None)
        self.clock = clock
        self.serial = _Source("serial")
        self.teensy = _Source(TEENSY)
        self.active = None                # the _Source scans come from (None until one has valid packets)
        self.link = None
        self.error = ""
        self.last_scan = None             # clock time of the last whole revolution
        self.scans = 0
        self.speed_dps = 0.0
        self._rate_t = clock()
        self._rate_pk = 0
        self._rate_sc = 0
        self.pps = 0.0
        self.scan_hz = 0.0
        self._said_open = False
        self._said_err = None
        # MSG_LIDAR_RAW (#100)
        self.raw = {"frames": 0, "seq_gaps": 0, "overflows": 0, "bad": 0, "ignored": 0}
        self.fw = {}                      # the firmware's own counters from its "lidar: ..." log line (usb drop, overflow)
        self._seq = None

    # the source on show (Health / stats): the active one, else the one that got bytes, else the serial port
    def _shown(self):
        if self.active is not None:
            return self.active
        if self.teensy_on and (self.teensy.bytes or not self.port):
            return self.teensy
        return self.serial

    @property
    def stream(self):
        return self._shown().stream

    @property
    def asm(self):
        return self._shown().asm

    @property
    def bytes(self):
        return self.serial.bytes + self.teensy.bytes

    @property
    def last_data(self):
        return self._shown().last_data

    @property
    def last_pkt(self):
        return self._shown().last_pkt

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
        self.serial.reset()
        if self.active is self.serial:
            self.active = None
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
        """Bytes from the serial port."""
        self._feed(self.serial, data, self.clock())

    def feed_teensy(self, seq, flags, data):
        """One MSG_LIDAR_RAW frame (#100): seq + 1 per chunk the Teensy took from its Serial7 (a gap = chunks it dropped,
        or frames lost on the link), flags LINK_LIDAR_RAW_OVERFLOW = its UART buffer overflowed before this chunk. Either
        way the bytes before this chunk are incomplete: the partial packet waiting in the stream is thrown away (the CRC
        would catch it, this just never lets a packet straddle a hole)."""
        if not self.teensy_on:
            self.raw["ignored"] += 1
            return
        r = self.raw
        r["frames"] += 1
        hole = False
        if self._seq is not None and seq != (self._seq + 1) & 0xFF:
            r["seq_gaps"] += (seq - self._seq - 1) & 0xFF
            hole = True
        self._seq = seq
        if flags & 0x01:                                  # link_protocol.LINK_LIDAR_RAW_OVERFLOW
            r["overflows"] += 1
            hole = True
        st = self.teensy.stream
        if hole and st.buf:
            st.bytes_dropped += len(st.buf)
            del st.buf[:]
        self._feed(self.teensy, data, self.clock())

    def _feed(self, src, data, now):
        src.last_data = now
        src.bytes += len(data)
        pkts = src.stream.feed(data, now)
        if not pkts:
            return
        src.last_pkt = now
        if not self._use(src, now):
            return
        for p in pkts:
            self.speed_dps = p.speed_dps
            sc = src.asm.add(p)
            if sc is not None:
                self.scans += 1
                self.last_scan = now
                try:
                    self.on_scan(sc)
                except Exception as e:                    # a bug in mapping must not stop the reader
                    self.error = "scan handler: %s: %s" % (type(e).__name__, e)

    def _ok(self, src, now):
        return src.last_pkt is not None and now - src.last_pkt < self.SILENT_S

    def _use(self, src, now):
        """Scans from src? It is the active source, or it takes over from one without valid packets for SILENT_S."""
        a = self.active
        if a is src:
            return True
        if a is not None and self._ok(a, now):
            return False                                  # the other source is fine: keep it, never mix the two
        self.active = src
        src.asm = ScanAssembler()                         # start its turns afresh
        if self.teensy_on:                                # two possible sources: say which one is used
            self.say("lidar: reading the LD19 from %s" % (
                "the Teensy (MSG_LIDAR_RAW)" if src is self.teensy else self.port))
        return True

    def source(self):
        """"teensy" | "serial" | None: where the scans come from."""
        a = self.active
        return None if a is None else (TEENSY if a is self.teensy else "serial")

    def source_label(self):
        """"Teensy" | "USB" | "UART" | "TCP" | "" for the Health tile."""
        a = self._shown()
        if a is self.teensy:
            return "Teensy"
        return source_label(self.port) if self.port else ""

    def tick(self, now):
        """~1 Hz: packet and scan rates."""
        dt = now - self._rate_t
        if dt >= 0.9:
            pk, sc = self.serial.stream.packets + self.teensy.stream.packets, self.scans
            self.pps = (pk - self._rate_pk) / dt
            self.scan_hz = (sc - self._rate_sc) / dt
            self._rate_t, self._rate_pk, self._rate_sc = now, pk, sc

    def state(self, now=None):
        """off | waiting (port missing / no Teensy frame yet) | no data (port open, nothing arrives) | junk (bytes but no
        packets) | ok | stale"""
        now = self.clock() if now is None else now
        if not self.port and not self.teensy_on:
            return "off"
        a = self.active
        if a is not None:
            if self._ok(a, now):
                return "ok"
            other = self.serial if a is self.teensy else self.teensy
            if not (other.bytes and now - (other.last_data or -1e9) < self.SILENT_S):
                return "stale"
        open_ = [s for s in ((self.serial if self.port and self.link else None),
                             (self.teensy if self.teensy_on and self.teensy.bytes else None)) if s is not None]
        if not open_:
            return "waiting"
        return "junk" if any(s.bytes for s in open_) else "no data"

    def stats(self, now=None):
        now = self.clock() if now is None else now
        src = self._shown()
        s = src.stream
        out = {"port": self.port or (TEENSY if self.teensy_on else ""), "state": self.state(now), "error": self.error,
               "source": self.source(), "source_label": self.source_label(),
               "pps": round(self.pps, 1), "scan_hz": round(self.scan_hz, 1),
               "rpm": round(self.speed_dps / 6.0, 1),                      # deg/s / 360 * 60
               "packets": s.packets, "crc_errors": s.crc_errors, "bytes_dropped": s.bytes_dropped,
               "scans": self.scans, "partial": src.asm.partial,
               "scan_age_s": None if self.last_scan is None else round(now - self.last_scan, 2)}
        if self.teensy_on:
            t = self.teensy
            out["teensy"] = dict(self.raw, bytes=t.bytes, **{"fw_" + k: v for k, v in self.fw.items()},
                                 age_s=None if t.last_data is None else round(now - t.last_data, 2))
        return out


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
