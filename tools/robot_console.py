#!/usr/bin/env python3
"""Teensy serial console for the Orange Pi (standard library only; no pyserial needed).

Shows boot text / Serial.print lines and decodes link protocol v2 frames
(include/protocol/link_protocol.h): HELLO, LOG text and a few telemetry fields.

    python3 robot_console.py                  # read-only, until Ctrl+C
    python3 robot_console.py --seconds 5      # stop after 5 s
    python3 robot_console.py --release        # ask the ground-station bridge to let go of the port first
    python3 robot_console.py --get-params     # also send MSG_GET_PARAMS (the only thing that writes)

From the PC: tools/robot_console.ps1 (copies this file to the Pi and runs it over ssh).
"""
import argparse
import glob
import os
import select
import struct
import sys
import termios
import time

SYNC = b"\xa5\x5a"
MAX_PAYLOAD = 200
LINK_VERSION = 2
MSG_TELEMETRY, MSG_RANGE, MSG_LOG, MSG_PARAM_INFO, MSG_NAV_STATUS, MSG_HELLO = 1, 2, 3, 4, 5, 6
MSG_GET_PARAMS = 0x84
NAMES = {1: "TELEMETRY", 2: "RANGE", 3: "LOG", 4: "PARAM_INFO", 5: "NAV_STATUS", 6: "HELLO"}
MODES = ["IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"]
TLM_BITS = [(0x01, "ARMED"), (0x02, "UPRIGHT"), (0x04, "FAILSAFE"), (0x08, "PI_LINK"),
            (0x10, "OBSTACLE"), (0x20, "LOW_BATT"), (0x40, "NAV"), (0x80, "NAV_DONE")]
# TelemetryFrame: t_us, mode, status, error_code, radio_ok, 18 floats (v1, 80 bytes),
# v2 adds bal_p, bal_i, bal_d, steer_out, vel_out_deg, step (104 bytes)
TLM_V1 = struct.Struct("<I4B18f")
TLM_V2 = struct.Struct("<I4B18f5fI")
TLM_FIELDS = ("t_us mode status error_code radio_ok pitch_deg pitch_rate_dps roll_deg heading_deg "
              "yaw_rate_dps x_mm y_mm speed_mms turn_rate_dps cmd_speed_mms cmd_turn_dps "
              "wheel_left_mms wheel_right_mms balance_target_deg motor_left motor_right battery_v "
              "loop_hz bal_p bal_i bal_d steer_out vel_out_deg step").split()
HELLO = struct.Struct("<BBH24s")
PARAM = struct.Struct("<Hfff24s")


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(msg_type, payload=b""):
    body = bytes([msg_type, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


def cstr(b):
    return b.split(b"\0", 1)[0].decode("utf-8", "replace")


def ts():
    return time.strftime("%H:%M:%S")


class Decoder:
    """Splits the byte stream into frames (resync on A5 5A, CRC checked) and plain text."""

    def __init__(self, out, tlm_period):
        self.buf = bytearray()
        self.text = bytearray()
        self.out = out
        self.tlm_period = tlm_period
        self.last_tlm = 0.0
        self.counts = {}
        self.bad_crc = 0
        self.junk = 0
        self.params = 0

    def feed(self, data):
        self.buf += data
        buf = self.buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                keep = 1 if buf.endswith(b"\xa5") else 0
                self._text(buf[:len(buf) - keep])
                del buf[:len(buf) - keep]
                return
            if i:
                self._text(buf[:i])
                del buf[:i]
            if len(buf) < 4:
                return
            n = buf[3]
            if n > MAX_PAYLOAD:
                self._text(buf[:1])
                del buf[:1]
                continue
            if len(buf) < 6 + n:
                return
            body = bytes(buf[2:4 + n])
            if struct.unpack_from("<H", buf, 4 + n)[0] != crc16(body):
                self.bad_crc += 1
                self._text(buf[:1])
                del buf[:1]
                continue
            del buf[:6 + n]
            self._flush_text()
            self._frame(body[0], body[2:])

    # ------------------------------------------------------------- text
    def _text(self, b):
        for c in b:
            if c == 10:
                self._flush_text()
            elif c == 13:
                continue
            elif c == 9 or c >= 32:
                self.text.append(c)
                if len(self.text) > 300:
                    self._flush_text()
            else:
                self.junk += 1

    def _flush_text(self):
        if not self.text:
            return
        s = self.text.decode("utf-8", "replace")
        self.text.clear()
        printable = sum(ch.isprintable() and ch != "�" for ch in s)
        if printable < len(s) * 0.8:     # mostly binary noise
            self.junk += len(s)
            return
        self.out("%s  %s" % (ts(), s))

    # ------------------------------------------------------------- frames
    def _frame(self, t, p):
        self.counts[t] = self.counts.get(t, 0) + 1
        if t == MSG_HELLO and len(p) >= HELLO.size:
            lv, rv, pc, fw = HELLO.unpack_from(p)
            warn = "" if lv == LINK_VERSION else "   <-- console expects link v%d" % LINK_VERSION
            self.out("%s  HELLO link v%d radio v%d params %d firmware '%s'%s" % (ts(), lv, rv, pc, cstr(fw), warn))
        elif t == MSG_LOG:
            self.out("%s  [log] %s" % (ts(), p.decode("utf-8", "replace")))
        elif t == MSG_TELEMETRY:
            now = time.monotonic()
            if now - self.last_tlm >= self.tlm_period:
                self.last_tlm = now
                self.out("%s  %s" % (ts(), self._tlm(p)))
        elif t == MSG_PARAM_INFO and len(p) >= PARAM.size:
            pid, v, lo, hi, name = PARAM.unpack_from(p)
            self.params += 1
            self.out("%s  param %3d %-24s = %g  [%g .. %g]" % (ts(), pid, cstr(name), v, lo, hi))
        elif t not in NAMES:
            self.out("%s  frame type 0x%02x len %d (unknown)" % (ts(), t, len(p)))

    @staticmethod
    def _tlm(p):
        if len(p) >= TLM_V2.size:
            v = dict(zip(TLM_FIELDS, TLM_V2.unpack_from(p)))
        elif len(p) >= TLM_V1.size:
            v = dict(zip(TLM_FIELDS, TLM_V1.unpack_from(p)))
        else:
            return "TELEMETRY (short frame, %d bytes)" % len(p)
        mode = MODES[v["mode"]] if v["mode"] < len(MODES) else "mode%d" % v["mode"]
        flags = ",".join(n for bit, n in TLM_BITS if v["status"] & bit) or "-"
        s = "tlm %-7s %-24s pitch %+6.1f  speed %+6.0f mm/s  bat %5.2f V  loop %4.0f Hz" % (
            mode, flags, v["pitch_deg"], v["speed_mms"], v["battery_v"], v["loop_hz"])
        if v["error_code"]:
            s += "  ERR %d" % v["error_code"]
        if not v["radio_ok"]:
            s += "  no-radio"
        return s

    def summary(self):
        parts = ["%s %d" % (NAMES.get(t, "0x%02x" % t), n) for t, n in sorted(self.counts.items())]
        return "frames: %s | crc errors %d | junk bytes %d" % (", ".join(parts) or "none", self.bad_crc, self.junk)


# ----------------------------------------------------------------- port helpers
def find_port():
    for tty in sorted(glob.glob("/sys/bus/usb/devices/*/*:*/tty/tty*")):
        dev = os.path.dirname(os.path.dirname(os.path.dirname(tty)))
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                if f.read().strip() == "16c0":
                    return "/dev/" + os.path.basename(tty)
        except OSError:
            pass
    return "/dev/ttyACM0"


def holders(port):
    """[(pid, cmdline)] of processes we can see with the port open (other users' are invisible)."""
    port = os.path.realpath(port)
    found = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = int(fd_dir.split("/")[2])
        if pid == os.getpid():
            continue
        try:
            for fd in os.listdir(fd_dir):
                if os.readlink(os.path.join(fd_dir, fd)) == port:
                    with open("/proc/%d/cmdline" % pid, "rb") as f:
                        found.append((pid, f.read().replace(b"\0", b" ").decode(errors="replace").strip()[:80]))
                    break
        except OSError:
            continue
    return found


def is_bridge(cmd):
    return "station.bridge" in cmd or "station/bridge" in cmd


def open_port(port, write):
    fd = os.open(port, (os.O_RDWR if write else os.O_RDONLY) | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                  # iflag: raw
    attrs[1] = 0                                  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                  # lflag: no echo / canonical mode
    attrs[4] = attrs[5] = termios.B115200         # ignored by USB serial
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def main():
    ap = argparse.ArgumentParser(description="Teensy serial console (read-only by default)")
    ap.add_argument("--port", help="serial port (default: the Teensy's, else /dev/ttyACM0)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--tlm-period", type=float, default=0.5, help="seconds between telemetry lines (default 0.5)")
    ap.add_argument("--release", action="store_true",
                    help="if the ground-station bridge holds the port, ask it to let go (release file) while we run")
    ap.add_argument("--release-file", default=os.environ.get("SBR3_RELEASE_FILE", "/tmp/sbr3-bridge.release"))
    ap.add_argument("--get-params", action="store_true", help="send MSG_GET_PARAMS once (writes to the Teensy)")
    args = ap.parse_args()

    def out(line):
        print(line, flush=True)

    port = args.port or find_port()
    if not os.path.exists(port):
        out("robot_console: %s not found - is the Teensy plugged into the Pi?" % port)
        return 1

    created_release = False
    h = holders(port)
    if h:
        bridge = [x for x in h if is_bridge(x[1])]
        others = [x for x in h if not is_bridge(x[1])]
        if others:
            out("robot_console: %s is open by another program - not opening it:" % port)
            for pid, cmd in others:
                out("    %d %s" % (pid, cmd))
            return 1
        if not args.release:
            out("robot_console: the ground-station bridge (pid %d) holds %s." % (bridge[0][0], port))
            out("    Use --release to make it let go while the console runs (the station link pauses),")
            out("    or watch the station's Logs page instead.")
            return 1
        if not os.path.exists(args.release_file):
            open(args.release_file, "a").close()
            created_release = True
        out("robot_console: asked the bridge to release %s (%s)" % (port, args.release_file))
        end = time.monotonic() + 10
        while holders(port) and time.monotonic() < end:
            time.sleep(0.2)
        if holders(port):
            out("robot_console: the bridge did not let go within 10 s")
            if created_release:
                os.remove(args.release_file)
            return 1

    dec = Decoder(out, args.tlm_period)
    fd = None
    try:
        fd = open_port(port, args.get_params)
        out("robot_console: %s open %s%s - Ctrl+C to quit" % (
            port, "read/write" if args.get_params else "read-only",
            (", %g s" % args.seconds) if args.seconds else ""))
        if args.get_params:
            os.write(fd, frame(MSG_GET_PARAMS))
            out("robot_console: sent GET_PARAMS")
        end = time.monotonic() + args.seconds if args.seconds else None
        while end is None or time.monotonic() < end:
            wait = 0.5 if end is None else max(0.0, min(0.5, end - time.monotonic()))
            r, _, _ = select.select([fd], [], [], wait)
            if not r:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError as e:
                out("robot_console: port error (%s) - Teensy reset or unplugged?" % e)
                break
            if not data:
                out("robot_console: port closed (Teensy reset or unplugged?)")
                break
            dec.feed(data)
    except KeyboardInterrupt:
        pass
    except OSError as e:
        out("robot_console: cannot open %s: %s" % (port, e))
        return 1
    finally:
        dec._flush_text()
        if fd is not None:
            os.close(fd)
        if created_release:
            try:
                os.remove(args.release_file)
                out("robot_console: release file removed - the bridge reconnects")
            except OSError:
                pass
    out("robot_console: " + dec.summary() + ("" if not args.get_params else " | params %d" % dec.params))
    return 0


if __name__ == "__main__":
    sys.exit(main())
