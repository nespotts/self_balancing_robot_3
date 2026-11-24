"""transport.TcpLink / open_link against a local asyncio server (#49)."""
import asyncio
import types
import unittest

from station.bridge import transport
from station.bridge.transport import LinkError, TcpLink, open_link


class TestTcpLink(unittest.TestCase):
    def test_roundtrip_and_partial_reads(self):
        async def go():
            got, lost, conns = bytearray(), [], []

            async def server(r, w):
                conns.append(w)
                w.write(b"he")
                await w.drain()
                await asyncio.sleep(0.05)
                w.write(b"llo")
                await w.drain()
                data = await r.read(100)
                w.write(data.upper())
                await w.drain()
            srv = await asyncio.start_server(server, "127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            link = await open_link("tcp://127.0.0.1:%d" % port, got.extend, lambda: lost.append(1))
            self.assertTrue(link.write(b"abc"))
            for _ in range(100):
                if len(got) >= 8:
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(bytes(got), b"helloABC")
            self.assertEqual(lost, [])
            link.close()
            srv.close()
        asyncio.run(go())

    def test_lost_link_fires_once_and_writes_fail(self):
        async def go():
            lost = []

            async def server(r, w):
                w.write(b"x")
                await w.drain()
                w.close()
            srv = await asyncio.start_server(server, "127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            link = await open_link("tcp://127.0.0.1:%d" % port, lambda d: None, lambda: lost.append(1))
            for _ in range(100):
                if lost:
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(lost, [1])
            self.assertFalse(link.write(b"more"))
            link._lost()                                          # idempotent
            self.assertEqual(lost, [1])
            srv.close()
        asyncio.run(go())

    def test_close_is_quiet_and_write_after_close_is_false(self):
        async def go():
            lost = []

            async def server(r, w):
                await r.read(10)
            srv = await asyncio.start_server(server, "127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            link = await open_link("tcp://127.0.0.1:%d" % port, lambda d: None, lambda: lost.append(1))
            link.close()
            await asyncio.sleep(0.1)
            self.assertFalse(link.write(b"x"))
            self.assertEqual(lost, [])                            # a deliberate close is not a lost link
            srv.close()
        asyncio.run(go())

    def test_reconnect_after_loss(self):
        async def go():
            async def server(r, w):
                w.write(b"k")
                await w.drain()
                await r.read(10)
            srv = await asyncio.start_server(server, "127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            for _ in range(2):
                got = bytearray()
                link = await open_link("tcp://127.0.0.1:%d" % port, got.extend, lambda: None)
                for _ in range(50):
                    if got:
                        break
                    await asyncio.sleep(0.02)
                self.assertEqual(bytes(got), b"k")
                link.close()
            srv.close()
        asyncio.run(go())

    def test_refused_connection_is_a_link_error(self):
        async def go():
            srv = await asyncio.start_server(lambda r, w: None, "127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            srv.close()
            await srv.wait_closed()
            with self.assertRaises(LinkError):
                await open_link("tcp://127.0.0.1:%d" % port, lambda d: None, lambda: None)
        asyncio.run(go())

    def test_missing_serial_device_is_a_link_error(self):
        async def go():
            with self.assertRaises(LinkError):
                await open_link("/dev/definitely-not-here-ttyACM9", lambda d: None, lambda: None)
        asyncio.run(go())


class FakeLoop:
    def __init__(self):
        self.writer = None

    def add_writer(self, fd, cb):
        self.writer = cb

    def remove_writer(self, fd):
        self.writer = None

    def remove_reader(self, fd):
        pass


class TestPosixPartialWrite(unittest.TestCase):
    """#99: a partial os.write on the raw tty must not cut a frame (the rest goes first, in order)."""

    def make(self, os_write):
        link = transport.PosixSerialLink.__new__(transport.PosixSerialLink)   # no tty: just the write path
        link.fd, link.closed, link.pending = 99, False, bytearray()
        link.loop = FakeLoop()
        link.lost = []
        link.on_lost = lambda: link.lost.append(1)
        fake_os = types.SimpleNamespace(write=os_write, close=lambda fd: None)   # never touches a real fd
        self.addCleanup(setattr, transport, "os", transport.os)
        transport.os = fake_os
        return link

    def test_partial_write_finishes_the_frame_first(self):
        wire, room = bytearray(), [5]

        def os_write(fd, data):              # the kernel takes `room` bytes, then would block
            if room[0] <= 0:
                raise BlockingIOError()
            n = min(len(data), room[0])
            room[0] -= n
            wire.extend(bytes(data[:n]))
            return n
        link = self.make(os_write)
        self.assertTrue(link.write(b"AAAAAAAAAA"))               # 5 written, 5 kept
        self.assertEqual(bytes(link.pending), b"AAAAA")
        self.assertIsNotNone(link.loop.writer)
        self.assertTrue(link.write(b"BBB"))                      # queued behind the rest, not written in between
        self.assertEqual(bytes(wire), b"AAAAA")
        link.loop.writer()                                       # still full: nothing
        room[0] = 100
        link.loop.writer()
        self.assertEqual(bytes(wire), b"AAAAAAAAAABBB")          # whole frames, in order
        self.assertIsNone(link.loop.writer)
        self.assertEqual(link.pending, bytearray())
        self.assertTrue(link.write(b"CC"))
        self.assertEqual(bytes(wire), b"AAAAAAAAAABBBCC")

    def test_full_buffer_drops_whole_frames_only(self):
        def os_write(fd, data):
            raise BlockingIOError()
        link = self.make(os_write)
        self.assertFalse(link.write(b"frame"))                   # nothing of it written: dropped whole
        self.assertEqual(link.pending, bytearray())

    def test_too_much_queued_drops_the_new_frame(self):
        def os_write(fd, data):
            return 1
        link = self.make(os_write)
        self.assertTrue(link.write(b"x" * 10))
        self.assertFalse(link.write(b"y" * transport.PENDING_MAX))   # would not fit behind it: dropped whole
        self.assertEqual(bytes(link.pending), b"x" * 9)

    def test_write_error_is_a_lost_link(self):
        calls = [0]

        def os_write(fd, data):
            calls[0] += 1
            if calls[0] == 1:
                return 2
            raise OSError(5, "EIO")
        link = self.make(os_write)
        self.assertTrue(link.write(b"abcdef"))
        link.loop.writer()
        self.assertEqual(link.lost, [1])
        self.assertTrue(link.closed)
        self.assertFalse(link.write(b"x"))


if __name__ == "__main__":
    unittest.main()