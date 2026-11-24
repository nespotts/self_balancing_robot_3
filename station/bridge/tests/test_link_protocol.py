"""Unit tests for link_protocol.py. Run: python -m unittest discover -s station -t ."""
import os
import re
import struct
import subprocess
import unittest

from station.bridge import link_protocol as lp

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
REL_LINK = "include/protocol/link_protocol.h"
REL_RADIO = "include/protocol/radio_protocol.h"
# Until the firmware branch is merged, the working tree may still hold an older header; then the
# headers are read from this git ref instead (override with SBR3_HEADER_REF).
HEADER_REF = os.environ.get("SBR3_HEADER_REF", "origin/v4-firmware")

C_SIZES = {"uint8_t": 1, "int8_t": 1, "char": 1, "uint16_t": 2, "int16_t": 2,
           "uint32_t": 4, "int32_t": 4, "float": 4}
C_FMT = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h",
         "uint32_t": "I", "int32_t": "i", "float": "f"}


def _version(txt):
    m = re.search(r"#define\s+LINK_PROTOCOL_VERSION\s+(\d+)", txt or "")
    return int(m.group(1)) if m else None


def _git_show(rel):
    try:
        return subprocess.run(["git", "show", "%s:%s" % (HEADER_REF, rel)], cwd=ROOT, capture_output=True,
                              text=True, timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None


def load_headers():
    """(link_h_text, radio_h_text, source) for the header version this mirror implements."""
    def read(rel):
        p = os.path.join(ROOT, rel)
        if not os.path.exists(p):
            return None
        with open(p) as f:
            return f.read()
    link, radio = read(REL_LINK), read(REL_RADIO)
    if _version(link) == lp.LINK_PROTOCOL_VERSION:
        return link, radio, "working tree"
    link2 = _git_show(REL_LINK)
    if _version(link2) == lp.LINK_PROTOCOL_VERSION:
        return link2, _git_show(REL_RADIO), HEADER_REF
    return link, radio, "working tree (version mismatch)"


LINK_TXT, RADIO_TXT, HEADER_SRC = load_headers()


def _strip_comments(src):
    return re.sub(r"//[^\n]*", "", src)


def parse_structs(src):
    """Very small C struct parser for the protocol headers: name -> [(ctype, field, count)]."""
    src = _strip_comments(src)
    out = {}
    for m in re.finditer(r"struct\s+__attribute__\(\(packed\)\)\s+(\w+)\s*\{(.*?)\n\};", src, re.S):
        name, body = m.group(1), m.group(2)
        body = re.sub(r"struct\s+__attribute__\(\(packed\)\)\s*\{[^}]*\}\s*\w+\[\d+\];", "", body)
        fields = []
        for decl in body.split(";"):
            decl = decl.strip()
            if not decl:
                continue
            ctype, rest = decl.split(None, 1)
            for item in rest.split(","):
                item = item.strip()
                am = re.match(r"(\w+)\s*\[(\d+)\]", item)
                if am:
                    fields.append((ctype, am.group(1), int(am.group(2))))
                else:
                    fields.append((ctype, item, 1))
        out[name] = fields
    return out


def parse_defines(src):
    src = _strip_comments(src)
    d = {}
    for m in re.finditer(r"#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)\b", src):
        d[m.group(1)] = int(m.group(2), 0)
    for m in re.finditer(r"\b(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*,", src):
        d[m.group(1)] = int(m.group(2), 0)
    return d


@unittest.skipUnless(LINK_TXT, "C header not found")
class TestMatchesHeader(unittest.TestCase):
    def test_header_version(self):
        self.assertEqual(_version(LINK_TXT), lp.LINK_PROTOCOL_VERSION, "header source: " + HEADER_SRC)

    def test_struct_layouts(self):
        structs = parse_structs(LINK_TXT)
        for name, (fmt, fields) in lp.STRUCTS.items():
            self.assertIn(name, structs, name)
            cfields = structs[name]
            self.assertEqual([f for _t, f, _n in cfields], list(fields), name + " field order")
            csize = sum(C_SIZES[t] * n for t, _f, n in cfields)
            self.assertEqual(struct.calcsize(fmt), csize, name + " size")
            # same primitive type per field
            exp = "<" + "".join(("%ds" % n) if t == "char" else C_FMT[t] * n for t, _f, n in cfields)
            self.assertEqual(re.sub(r"(\d+)([fBHI])", lambda m: m.group(2) * int(m.group(1)), fmt), exp,
                             name + " types")

    def test_static_asserts(self):
        for name, size in re.findall(r"sizeof\((\w+)\)\s*==\s*(\d+)", LINK_TXT):
            self.assertEqual(struct.calcsize(lp.STRUCTS[name][0]), int(size), name)

    def test_goto_frame(self):
        self.assertEqual(parse_structs(LINK_TXT)["GotoFrame"][:2],
                         [("uint8_t", "replace", 1), ("uint8_t", "count", 1)])
        self.assertIn("points[16]", LINK_TXT)
        self.assertEqual(lp.GOTO_MAX_POINTS, 16)

    def test_constants(self):
        d = parse_defines(LINK_TXT)
        for k, v in d.items():
            if k.startswith(("LINK_", "MSG_", "NAV_")):
                self.assertEqual(getattr(lp, k), v, k)
        self.assertEqual(len([k for k in d if k.startswith("MSG_")]), len(lp.MSG_NAMES))
        self.assertEqual(len(lp.NAV_STATE_NAMES), lp.NAV_STATE_COUNT)
        self.assertEqual(len([k for k in d if k.startswith("NAV_")]), lp.NAV_STATE_COUNT)
        link_events = {k: v for k, v in d.items() if k.startswith("EVT_LINK_")}
        self.assertEqual(sorted(link_events), ["EVT_LINK_ARM", "EVT_LINK_ESTOP", "EVT_LINK_ESTOP_RELEASE", "EVT_LINK_STUCK"])
        for k, v in link_events.items():
            self.assertEqual(getattr(lp, k), v, k)
            self.assertGreaterEqual(v, 0x80)    # never collides with a RadioEvent

    @unittest.skipUnless(RADIO_TXT, "radio header not found")
    def test_radio_enums(self):
        d = parse_defines(RADIO_TXT)
        for k, v in d.items():
            if k.startswith(("MODE_", "EVT_", "TLM_")) or k == "RADIO_PROTOCOL_VERSION":
                self.assertEqual(getattr(lp, k), v, k)
        self.assertEqual(len(lp.MODE_NAMES), lp.MODE_COUNT)
        self.assertEqual(len(lp.EVENT_NAMES), lp.EVT_COUNT)

class TestSizes(unittest.TestCase):
    def test_known_sizes(self):
        self.assertEqual(lp.TELEMETRY_SIZE, 108)
        self.assertEqual(lp.RANGE_SIZE, 28)
        self.assertEqual(lp.PARAM_INFO_SIZE, 38)
        self.assertEqual(lp.HELLO_SIZE, 28)
        self.assertEqual(lp.NAV_STATUS_SIZE, 20)
        for name, (fmt, fields) in lp.STRUCTS.items():
            self.assertLessEqual(struct.calcsize(fmt), lp.LINK_MAX_PAYLOAD)
        self.assertLessEqual(2 + 16 * 8, lp.LINK_MAX_PAYLOAD)


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(lp.crc16(b"123456789"), 0x29B1)  # CRC-16/CCITT-FALSE check value

    def test_empty(self):
        self.assertEqual(lp.crc16(b""), 0xFFFF)

    def test_frame_layout(self):
        f = lp.encode_frame(0x81, b"\x01\x02")
        self.assertEqual(f[:4], b"\xA5\x5A\x81\x02")
        self.assertEqual(struct.unpack("<H", f[-2:])[0], lp.crc16(b"\x81\x02\x01\x02"))
        self.assertEqual(len(f), 4 + 2 + 2)


def sample_tlm(**kw):
    d = {f: 0.0 for f in lp.TELEMETRY_FIELDS}
    d.update(remote_flags=lp.LINK_RF_ESTOP | lp.LINK_RF_PRESENT, nav_state=lp.NAV_BLOCKED, nav_index=2, reserved2=0)
    d.update(t_us=123456, mode=3, status=lp.TLM_ARMED | lp.TLM_UPRIGHT, error_code=0, radio_ok=1)
    d.update(pitch_deg=1.5, heading_deg=-370.25, x_mm=1000.0, battery_v=12.25, loop_hz=1000.0)
    d.update(bal_p=-812.5, bal_i=40.25, bal_d=3.5, step=4000000000)
    d.update(kw)
    return d


class TestRoundTrip(unittest.TestCase):
    def parse_one(self, frame):
        p = lp.FrameParser()
        out = p.feed(frame)
        self.assertEqual(len(out), 1)
        return lp.decode(*out[0])

    def test_telemetry(self):
        d = sample_tlm()
        got = self.parse_one(lp.encode_telemetry(d))
        for k, v in d.items():
            self.assertAlmostEqual(got[k], v, places=4, msg=k)

    def test_range(self):
        d = dict(t_us=1, x_mm=10.5, y_mm=-20.0, heading_deg=90.0, pitch_deg=0.5,
                 sensor_angle_deg=-45.0, range_mm=1234, sensor_id=1, valid=1)
        self.assertEqual(self.parse_one(lp.encode_range(d)), d)

    def test_hello_param_nav_log(self):
        h = self.parse_one(lp.encode_hello(param_count=11, firmware="v4 test"))
        self.assertEqual(h, {"link_version": lp.LINK_PROTOCOL_VERSION, "radio_version": 2, "param_count": 11, "firmware": "v4 test"})
        p = self.parse_one(lp.encode_param_info(3, 4.63, 0.0, 20.0, "bal.kd"))
        self.assertEqual(p["name"], "bal.kd")
        self.assertAlmostEqual(p["value"], 4.63, places=5)
        n = self.parse_one(lp.encode_nav_status(dict(state=2, waypoint_count=3, nav_index=1,
                                                     target_x_mm=1.0, target_y_mm=2.0, distance_mm=3.0)))
        self.assertEqual((n["state"], n["waypoint_count"], n["nav_index"]), (2, 3, 1))
        self.assertEqual(n["obstacle_mm"], 0.0)
        n = self.parse_one(lp.encode_nav_status(dict(state=lp.NAV_BLOCKED, waypoint_count=3, nav_index=1,
                                                     target_x_mm=1.0, target_y_mm=2.0, distance_mm=3.0,
                                                     obstacle_mm=287.5)))
        self.assertEqual((n["state"], n["obstacle_mm"]), (lp.NAV_BLOCKED, 287.5))
        self.assertEqual(self.parse_one(lp.encode_log("hello °")), "hello °")

    def test_pi_to_teensy(self):
        self.assertEqual(self.parse_one(lp.encode_heartbeat(5000, 2)), {"pi_ms": 5000, "clients": 2})
        self.assertEqual(self.parse_one(lp.encode_cmd_vel(100.0, -45.0)), {"speed_mms": 100.0, "turn_dps": -45.0})
        self.assertEqual(self.parse_one(lp.encode_set_mode(4)), {"mode": 4})
        sp = self.parse_one(lp.encode_set_param(7, 0.5))
        self.assertEqual(sp, {"id": 7, "value": 0.5})
        self.assertEqual(self.parse_one(lp.encode_get_params()), {})
        self.assertEqual(self.parse_one(lp.encode_event(lp.EVT_RESET_POSE)), {"event": 4})
        g = self.parse_one(lp.encode_goto([(1.0, 2.0), (3.0, 4.0)], replace=True))
        self.assertEqual(g, {"replace": 1, "count": 2, "points": [(1.0, 2.0), (3.0, 4.0)]})

    def test_goto_len_and_split(self):
        f = lp.encode_goto([(0.0, 0.0)] * 3)
        self.assertEqual(f[3], 2 + 3 * 8)
        frames = lp.goto_frames([(float(i), 0.0) for i in range(40)])
        self.assertEqual(len(frames), 3)
        dec = [lp.decode(*lp.FrameParser().feed(f)[0]) for f in frames]
        self.assertEqual([d["replace"] for d in dec], [1, 0, 0])
        self.assertEqual([d["count"] for d in dec], [16, 16, 8])
        with self.assertRaises(ValueError):
            lp.encode_goto([(0.0, 0.0)] * 17)

    def test_too_long(self):
        with self.assertRaises(ValueError):
            lp.encode_frame(1, b"x" * 201)


class TestResync(unittest.TestCase):
    def test_garbage_between_frames(self):
        a = lp.encode_cmd_vel(1.0, 2.0)
        b = lp.encode_set_mode(3)
        p = lp.FrameParser()
        out = p.feed(b"\x00\xff\xA5garbage\x5A" + a + b"\xA5\xA5\x5A\x01\xF0" + b)
        # "\xA5\x5A\x01\xF0" starts a bogus frame (type 1, len 240 > 200 -> rejected) then b parses
        types = [t for t, _ in out]
        self.assertEqual(types[0], lp.MSG_CMD_VEL)
        self.assertEqual(types[-1], lp.MSG_SET_MODE)

    def test_boot_text_between_frames(self):
        # the Teensy prints plain text on USB serial during setup (lidar init) before/among frames
        text = (b"VL53L0X init... sensor 0 ok\r\nservo center\r\n" + bytes(range(32, 127)) +
                b"\r\nBNO085 found at 0x4A\r\n")
        a, b = lp.encode_hello(param_count=35, firmware="v4"), lp.encode_telemetry(sample_tlm())
        p = lp.FrameParser()
        out = p.feed(text + a + text + b + text[:7])
        self.assertEqual([t for t, _ in out], [lp.MSG_HELLO, lp.MSG_TELEMETRY])
        self.assertEqual(p.crc_errors, 0)
        self.assertEqual(lp.decode(*out[1])["battery_v"], 12.25)

    def test_v1_telemetry_still_decodes(self):
        full = lp._pack("TelemetryFrame", sample_tlm(bal_p=5.0, step=9))
        d = lp.decode_telemetry(full[:lp.TELEMETRY_V1_SIZE])
        self.assertEqual(d["battery_v"], 12.25)
        self.assertEqual((d["bal_p"], d["step"]), (0.0, 0))

    def test_v2_telemetry_and_nav_status_still_decode(self):
        full = lp._pack("TelemetryFrame", sample_tlm(step=9))
        d = lp.decode_telemetry(full[:lp.TELEMETRY_V2_SIZE])
        self.assertEqual((d["step"], d["remote_flags"], d["nav_state"]), (9, 0, 0))
        nav = lp._pack("NavStatusFrame", dict(state=4, waypoint_count=2, nav_index=2, reserved=0,
                                              target_x_mm=5.0, target_y_mm=6.0, distance_mm=7.0, obstacle_mm=8.0))
        n = lp.decode_nav_status(nav[:lp.NAV_STATUS_V2_SIZE])
        self.assertEqual((n["state"], n["distance_mm"], n["obstacle_mm"]), (4, 7.0, 0.0))

    def test_v3_fields_round_trip(self):
        d = lp.decode_telemetry(lp._pack("TelemetryFrame", sample_tlm()))
        self.assertEqual((d["remote_flags"], d["nav_state"], d["nav_index"]),
                         (lp.LINK_RF_ESTOP | lp.LINK_RF_PRESENT, lp.NAV_BLOCKED, 2))

    def test_byte_by_byte(self):
        stream = lp.encode_telemetry(sample_tlm()) + lp.encode_log("abc") + lp.encode_event(1)
        p = lp.FrameParser()
        out = []
        for i in range(len(stream)):
            out += p.feed(stream[i:i + 1])
        self.assertEqual([t for t, _ in out], [lp.MSG_TELEMETRY, lp.MSG_LOG, lp.MSG_EVENT])
        self.assertEqual(p.crc_errors, 0)

    def test_partial_then_rest(self):
        f = lp.encode_telemetry(sample_tlm())
        p = lp.FrameParser()
        self.assertEqual(p.feed(f[:30]), [])
        self.assertEqual(len(p.feed(f[30:])), 1)

    def test_corrupt_frame_skipped(self):
        good1 = lp.encode_event(1)
        bad = bytearray(lp.encode_telemetry(sample_tlm()))
        bad[20] ^= 0xFF
        good2 = lp.encode_event(2)
        p = lp.FrameParser()
        out = p.feed(good1 + bytes(bad) + good2)
        self.assertEqual([lp.decode(t, pl)["event"] for t, pl in out], [1, 2])
        self.assertGreaterEqual(p.crc_errors, 1)

    def test_truncated_frame_then_good(self):
        # a frame cut off mid-way (e.g. Teensy reset) followed by a good frame: the good frame must
        # still be found once the CRC check of the truncated one fails.
        cut = lp.encode_telemetry(sample_tlm())[:40]
        good = lp.encode_event(5)
        p = lp.FrameParser()
        out = p.feed(cut + good + lp.encode_event(5) * 12)
        self.assertGreaterEqual(len([1 for t, _ in out if t == lp.MSG_EVENT]), 1)

    def test_sync_split_across_feeds(self):
        f = lp.encode_event(3)
        p = lp.FrameParser()
        self.assertEqual(p.feed(b"zz" + f[:1]), [])
        self.assertEqual(len(p.feed(f[1:])), 1)

    def test_buffer_does_not_grow_on_noise(self):
        p = lp.FrameParser()
        for _ in range(100):
            p.feed(bytes(range(0, 0xA5)) * 4)
        self.assertLess(len(p.buf), 8)


if __name__ == "__main__":
    unittest.main()
