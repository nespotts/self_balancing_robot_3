"""httpws: WebSocket framing (fragments, ping/pong, oversize, unmasked), routes, static files, path traversal,
host / origin checks (#49). HTTP request framing limits live in test_hardening."""
import asyncio
import json
import os
import shutil
import struct
import tempfile
import unittest

from station.bridge import httpws
from station.bridge.httpws import HttpServer, Request, Response, WsClient, check_origin, host_ok
from station.bridge.tests.test_hardening import request, raw, WS_UP


def frame(opcode, payload=b"", fin=True, mask=True):
    b0 = (0x80 if fin else 0) | opcode
    n = len(payload)
    head = bytes([b0])
    mbit = 0x80 if mask else 0
    if n < 126:
        head += bytes([mbit | n])
    elif n < 65536:
        head += bytes([mbit | 126]) + struct.pack("!H", n)
    else:
        head += bytes([mbit | 127]) + struct.pack("!Q", n)
    if mask:
        key = b"\x01\x02\x03\x04"
        return head + key + bytes(c ^ key[i & 3] for i, c in enumerate(payload))
    return head + payload


async def read_frame(r, timeout=2):
    b0, b1 = await asyncio.wait_for(r.readexactly(2), timeout)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack("!H", await r.readexactly(2))[0]
    return b0 & 0x0F, await r.readexactly(n)


class TestHttpws(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.static = os.path.join(self.tmp, "web")
        os.makedirs(self.static)
        with open(os.path.join(self.static, "index.html"), "w") as f:
            f.write("<h1>hi</h1>")
        with open(os.path.join(self.tmp, "secret.txt"), "w") as f:
            f.write("secret")
        os.makedirs(self.static + "-old")
        with open(os.path.join(self.static + "-old", "x.txt"), "w") as f:
            f.write("sibling")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    async def serve(self, handler=None, routes=()):
        got = []

        async def echo(ws):
            while True:
                m = await ws.recv()
                if m is None:
                    got.append("closed")
                    return
                got.append(m)
                ws.send("echo:" + m)
        http = HttpServer(self.static, list(routes), "/ws", handler or echo)
        srv = await http.start("127.0.0.1", 0)
        return srv, srv.sockets[0].getsockname()[1], got

    async def upgrade(self, port):
        r, w = await asyncio.open_connection("127.0.0.1", port)
        w.write(request(port, "GET", "/ws", WS_UP).replace(b"Connection: close\r\n", b"").replace(b"Connection: Upgrade", b"Connection: Upgrade"))
        await w.drain()
        head = b""
        while not head.endswith(b"\r\n\r\n"):
            head += await asyncio.wait_for(r.read(1), 2)
        self.assertIn(b" 101 ", head.split(b"\r\n")[0])
        return r, w

    def test_fragmented_message_is_reassembled(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(1, b"hel", fin=False) + frame(0, b"lo ", fin=False) + frame(0, b"world"))
            op, data = await read_frame(r)
            self.assertEqual((op, data), (1, b"echo:hello world"))
            srv.close()
        asyncio.run(go())

    def test_ping_is_answered_with_pong_and_does_not_end_the_message(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(1, b"a", fin=False) + frame(9, b"pp") + frame(0, b"b"))
            op, data = await read_frame(r)
            self.assertEqual((op, data), (0xA, b"pp"))
            op, data = await read_frame(r)
            self.assertEqual(data, b"echo:ab")
            srv.close()
        asyncio.run(go())

    def test_unmasked_frame_is_accepted(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(1, b"plain", mask=False))
            self.assertEqual((await read_frame(r))[1], b"echo:plain")
            srv.close()
        asyncio.run(go())

    def test_oversize_message_ends_that_connection_only(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(1, b"x" * (httpws.MAX_WS_MSG + 1)))
            await w.drain()
            self.assertEqual(await asyncio.wait_for(r.read(1000), 2) is not None, True)
            for _ in range(50):
                if "closed" in got:
                    break
                await asyncio.sleep(0.02)
            self.assertIn("closed", got)
            self.assertFalse([g for g in got if g != "closed"])
            c = await WsClient.connect("127.0.0.1", port)         # the server is still serving
            c.send("still here")
            self.assertEqual(await asyncio.wait_for(c.recv(), 2), "echo:still here")
            srv.close()
        asyncio.run(go())

    def test_fragments_adding_up_to_too_much_are_dropped(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            half = b"y" * (httpws.MAX_WS_MSG // 2 + 10)
            w.write(frame(1, half, fin=False) + frame(0, half, fin=False) + frame(0, b"z"))
            await w.drain()
            for _ in range(50):
                if "closed" in got:
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(got, ["closed"])
            srv.close()
        asyncio.run(go())

    def test_close_frame_is_answered_and_ends_the_handler(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(8, struct.pack("!H", 1000)))
            op, _ = await read_frame(r)
            self.assertEqual(op, 8)
            for _ in range(50):
                if got:
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(got, ["closed"])
            srv.close()
        asyncio.run(go())

    def test_truncated_frame_is_a_close_not_a_crash(self):
        async def go():
            srv, port, got = await self.serve()
            r, w = await self.upgrade(port)
            w.write(frame(1, b"abcdef")[:5])
            await w.drain()
            w.close()
            for _ in range(50):
                if got:
                    break
                await asyncio.sleep(0.02)
            self.assertEqual(got, ["closed"])
            srv.close()
        asyncio.run(go())

    def test_static_files_and_traversal(self):
        async def go():
            srv, port, _ = await self.serve()
            st, head, body = await raw(port, request(port, "GET", "/"))
            self.assertEqual((st, body), (200, b"<h1>hi</h1>"))
            st, _, body = await raw(port, request(port, "HEAD", "/"))
            self.assertEqual((st, body), (200, b""))
            for path in ("/../secret.txt", "/%2e%2e/secret.txt", "/../web-old/x.txt", "/..%2fsecret.txt",
                         "/....//secret.txt", "/..\\secret.txt"):
                st, _, body = await raw(port, request(port, "GET", path))
                self.assertIn(st, (400, 403, 404), path)
                self.assertNotIn(b"secret", body, path)
                self.assertNotIn(b"sibling", body, path)
            st, _, _ = await raw(port, request(port, "GET", "/nope.html"))
            self.assertEqual(st, 404)
            st, _, _ = await raw(port, request(port, "POST", "/"))
            self.assertEqual(st, 405)
            srv.close()
        asyncio.run(go())

    def test_routes_and_handler_errors(self):
        async def go():
            def boom(req):
                raise RuntimeError("bug")

            async def hello(req):
                return Response.json({"you": req.json()["x"], "q": req.path})
            srv, port, _ = await self.serve(routes=[("POST", "/api/hello", hello), ("GET", "/api/boom", boom)])
            st, _, body = await raw(port, request(port, "POST", "/api/hello", body=b'{"x": 5}'))
            self.assertEqual((st, json.loads(body)), (200, {"you": 5, "q": "/api/hello"}))
            st, _, _ = await raw(port, request(port, "GET", "/api/boom"))
            self.assertEqual(st, 500)
            st, _, _ = await raw(port, request(port, "GET", "/api/boom"))      # still serving
            self.assertEqual(st, 500)
            srv.close()
        asyncio.run(go())

    def test_guard_runs_for_api_and_ws_but_not_static(self):
        async def go():
            async def ws_h(ws):
                await ws.recv()
            http = HttpServer(self.static, [("GET", "/api/x", lambda r: Response.text("ok"))], "/ws", ws_h,
                              guard=lambda req: Response(403, "no"))
            srv = await http.start("127.0.0.1", 0)
            port = srv.sockets[0].getsockname()[1]
            self.assertEqual((await raw(port, request(port, "GET", "/")))[0], 200)
            self.assertEqual((await raw(port, request(port, "GET", "/api/x")))[0], 403)
            self.assertEqual((await raw(port, request(port, "GET", "/ws", WS_UP), until_close=False))[0], 403)
            srv.close()
        asyncio.run(go())

    def test_host_and_origin_checks(self):
        self.assertTrue(host_ok("localhost:8080"))
        self.assertTrue(host_ok("127.0.0.1"))
        self.assertFalse(host_ok("evil.example"))
        self.assertTrue(host_ok("pi.local", extra=("pi.local",)))
        req = Request("GET", "/ws", {"host": "127.0.0.1:1", "origin": "http://evil.example"}, b"", None)
        self.assertIsNotNone(check_origin(req))
        req = Request("GET", "/ws", {"host": "127.0.0.1:1", "origin": "http://127.0.0.1:1"}, b"", None)
        self.assertIsNone(check_origin(req))


if __name__ == "__main__":
    unittest.main()