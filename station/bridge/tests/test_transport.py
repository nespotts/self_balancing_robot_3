"""transport.TcpLink / open_link against a local asyncio server (#49)."""
import asyncio
import unittest

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


if __name__ == "__main__":
    unittest.main()