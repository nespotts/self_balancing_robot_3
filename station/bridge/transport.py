"""Byte transports to the Teensy: USB serial (Linux raw tty or pyserial) or TCP (simulator).

open_link(spec, on_data, on_lost) -> Link with .write(bytes) and .close().
  spec: "/dev/ttyACM0", "COM5", or "tcp://127.0.0.1:5761".
"""
import asyncio
import os
import threading

PENDING_MAX = 4096               # PosixSerialLink: bytes queued behind a partial write before frames are dropped


class LinkError(Exception):
    pass


class TcpLink:
    def __init__(self, reader, writer, on_data, on_lost):
        self.reader, self.writer = reader, writer
        self.on_data, self.on_lost = on_data, on_lost
        self.closed = False
        self.task = asyncio.ensure_future(self._run())

    async def _run(self):
        try:
            while True:
                data = await self.reader.read(4096)
                if not data:
                    break
                self.on_data(data)
        except (ConnectionError, OSError):
            pass
        finally:
            self._lost()

    def _lost(self):
        if not self.closed:
            self.closed = True
            try:
                self.writer.close()
            except Exception:
                pass
            self.on_lost()

    def write(self, data):
        if self.closed:
            return False
        try:
            self.writer.write(data)
            return True
        except Exception:
            self._lost()
            return False

    def close(self):
        self.closed = True
        self.task.cancel()
        try:
            self.writer.close()
        except Exception:
            pass


class PosixSerialLink:
    """Raw tty on Linux using the event loop's add_reader: no threads, no busy polling."""

    def __init__(self, path, baud, on_data, on_lost):
        import termios
        import tty
        self.path = path
        self.on_data, self.on_lost = on_data, on_lost
        self.closed = False
        self.pending = bytearray()       # the unwritten rest of a frame (partial os.write)
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            speed = getattr(termios, "B%d" % baud, termios.B115200)
            attrs[4] = attrs[5] = speed
            attrs[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd)
            raise
        self.loop = asyncio.get_event_loop()
        self.loop.add_reader(self.fd, self._readable)

    def _readable(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if not data:  # EOF / EIO: device gone (Teensy rebooting or unplugged)
            self._lost()
            return
        self.on_data(data)

    def _lost(self):
        if not self.closed:
            self.close()
            self.on_lost()

    def write(self, data):
        """Whole frames only (#99): a partial os.write keeps the rest and sends it first (the event loop's writer),
        so the Teensy never sees a cut frame followed by the next one. Kernel buffer full with nothing of this frame
        written: dropped (heartbeats / cmd_vel are periodic); a frame that does not fit behind the rest: dropped."""
        if self.closed:
            return False
        if self.pending:                 # the rest of an earlier frame goes first, in order
            if len(self.pending) + len(data) > PENDING_MAX:
                return False
            self.pending += data
            return True
        try:
            n = os.write(self.fd, data)
        except BlockingIOError:
            return False
        except OSError:
            self._lost()
            return False
        if n < len(data):
            self.pending = bytearray(data[n:])
            try:
                self.loop.add_writer(self.fd, self._writable)
            except Exception:
                self._lost()
                return False
        return True

    def _writable(self):
        try:
            n = os.write(self.fd, self.pending)
        except BlockingIOError:
            return
        except OSError:
            self._lost()
            return
        del self.pending[:n]
        if not self.pending:
            try:
                self.loop.remove_writer(self.fd)
            except Exception:
                pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.pending = bytearray()
        try:
            self.loop.remove_reader(self.fd)
        except Exception:
            pass
        try:
            self.loop.remove_writer(self.fd)
        except Exception:
            pass
        try:
            os.close(self.fd)
        except OSError:
            pass


class PySerialLink:
    """pyserial with a reader thread (Windows/macOS, or Linux without termios access)."""

    def __init__(self, path, baud, on_data, on_lost):
        import serial  # pyserial (pure Python)
        self.loop = asyncio.get_event_loop()
        self.on_data, self.on_lost = on_data, on_lost
        self.closed = False
        self.ser = serial.Serial(path, baud, timeout=0.2, write_timeout=0.1)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.closed:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                break
            if data:
                self.loop.call_soon_threadsafe(self.on_data, data)
        if not self.closed:
            self.loop.call_soon_threadsafe(self._lost)

    def _lost(self):
        if not self.closed:
            self.close()
            self.on_lost()

    def write(self, data):
        if self.closed:
            return False
        try:
            self.ser.write(data)
            return True
        except Exception:
            self.loop.call_soon(self._lost)
            return False

    def close(self):
        self.closed = True
        try:
            self.ser.close()
        except Exception:
            pass


async def open_link(spec, on_data, on_lost, baud=115200):
    if spec.startswith("tcp://"):
        hostport = spec[6:]
        host, _, port = hostport.rpartition(":")
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_connection(host or "127.0.0.1", int(port)), 3)
        except (OSError, asyncio.TimeoutError) as e:
            raise LinkError(str(e))
        return TcpLink(reader, writer, on_data, on_lost)
    if os.name == "posix" and not os.environ.get("SBR3_USE_PYSERIAL"):
        if not os.path.exists(spec):
            raise LinkError("%s not present" % spec)
        try:
            return PosixSerialLink(spec, baud, on_data, on_lost)
        except OSError as e:
            raise LinkError(str(e))
    try:
        return PySerialLink(spec, baud, on_data, on_lost)
    except ImportError:
        raise LinkError("pyserial not installed (pip install pyserial)")
    except Exception as e:
        raise LinkError(str(e))
