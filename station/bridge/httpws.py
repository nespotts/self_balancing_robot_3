"""Tiny asyncio HTTP/1.1 + WebSocket (RFC 6455) server. Stdlib only, Python 3.8+.

Enough for a LAN ground station: static files, small JSON API, text WebSockets.
"""
import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
import urllib.parse

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_BODY = 64 * 1024
MAX_WS_MSG = 64 * 1024

MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".mjs": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8",
        ".json": "application/json", ".webmanifest": "application/manifest+json",
        ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon",
        ".jsonl": "application/x-ndjson", ".csv": "text/csv; charset=utf-8", ".txt": "text/plain; charset=utf-8"}

STATUS_TEXT = {200: "OK", 204: "No Content", 400: "Bad Request", 403: "Forbidden", 404: "Not Found",
               405: "Method Not Allowed", 413: "Payload Too Large", 500: "Internal Server Error"}


class Request:
    def __init__(self, method, target, headers, body, peer):
        self.method = method
        u = urllib.parse.urlsplit(target)
        self.path = urllib.parse.unquote(u.path)
        self.query = {k: v[-1] for k, v in urllib.parse.parse_qs(u.query).items()}
        self.headers = headers
        self.body = body
        self.peer = peer

    def json(self):
        return json.loads(self.body.decode("utf-8")) if self.body else {}


class Response:
    def __init__(self, status=200, body=b"", ctype="text/plain; charset=utf-8", headers=None, file=None):
        self.status = status
        self.body = body if isinstance(body, bytes) else body.encode("utf-8")
        self.ctype = ctype
        self.headers = headers or {}
        self.file = file  # path to stream instead of body
        self.gen = None   # iterator of str chunks (streamed, connection closes after)

    @staticmethod
    def json(obj, status=200):
        return Response(status, json.dumps(obj, separators=(",", ":")), "application/json")

    @staticmethod
    def text(s, status=200):
        return Response(status, s)


class WebSocket:
    """Server side of one WebSocket connection (text messages)."""

    def __init__(self, reader, writer, peer):
        self.reader = reader
        self.writer = writer
        self.peer = peer
        self.closed = False

    def buffered(self):
        t = self.writer.transport
        return t.get_write_buffer_size() if t and not t.is_closing() else 1 << 30

    def send(self, text):
        """Queue a text message (non-blocking). Returns False if the socket is gone."""
        if self.closed:
            return False
        data = text.encode("utf-8") if isinstance(text, str) else text
        self._send_frame(0x1, data)
        return True

    def _send_frame(self, opcode, data):
        n = len(data)
        if n < 126:
            hdr = struct.pack("!BB", 0x80 | opcode, n)
        elif n < 65536:
            hdr = struct.pack("!BBH", 0x80 | opcode, 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x80 | opcode, 127, n)
        try:
            self.writer.write(hdr + data)
        except Exception:
            self.closed = True

    async def recv(self):
        """Next text message, or None when the connection closes."""
        parts = []
        while True:
            try:
                b0, b1 = await self.reader.readexactly(2)
                fin, opcode = b0 & 0x80, b0 & 0x0F
                masked, n = b1 & 0x80, b1 & 0x7F
                if n == 126:
                    n = struct.unpack("!H", await self.reader.readexactly(2))[0]
                elif n == 127:
                    n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
                if n > MAX_WS_MSG:
                    raise ValueError("ws message too big")
                mask = await self.reader.readexactly(4) if masked else b"\0\0\0\0"
                payload = bytearray(await self.reader.readexactly(n))
            except (asyncio.IncompleteReadError, ConnectionError, ValueError, OSError):
                self.closed = True
                return None
            if masked:
                for i in range(n):
                    payload[i] ^= mask[i & 3]
            if opcode == 0x8:  # close
                self._send_frame(0x8, bytes(payload[:2]))
                self.closed = True
                return None
            if opcode == 0x9:  # ping
                self._send_frame(0xA, bytes(payload))
                continue
            if opcode == 0xA:
                continue
            parts.append(bytes(payload))
            if fin:
                return b"".join(parts).decode("utf-8", "replace")

    def close(self):
        if not self.closed:
            self._send_frame(0x8, struct.pack("!H", 1000))
        self.closed = True
        try:
            self.writer.close()
        except Exception:
            pass


class HttpServer:
    """routes: list of (method, path_prefix, handler(req) -> Response | awaitable Response).
    ws_handler(ws) coroutine is called for GET ws_path upgrade requests."""

    def __init__(self, static_dir, routes, ws_path="/ws", ws_handler=None):
        self.static_dir = os.path.abspath(static_dir)
        self.routes = routes
        self.ws_path = ws_path
        self.ws_handler = ws_handler

    async def start(self, host, port):
        self.server = await asyncio.start_server(self._client, host, port)
        return self.server

    async def _client(self, reader, writer):
        peer = writer.get_extra_info("peername")
        try:
            while True:
                req = await self._read_request(reader, peer)
                if req is None:
                    break
                if isinstance(req, Response):
                    await self._write_response(writer, req, False)
                    break
                if (req.path == self.ws_path and self.ws_handler and
                        req.headers.get("upgrade", "").lower() == "websocket"):
                    await self._upgrade(req, reader, writer)
                    return
                resp = await self._dispatch(req)
                keep = req.headers.get("connection", "").lower() == "keep-alive" or \
                    (req.headers.get("_version") == "HTTP/1.1" and req.headers.get("connection", "").lower() != "close")
                await self._write_response(writer, resp, keep, head=req.method == "HEAD")
                if not keep:
                    break
        except (ConnectionError, asyncio.IncompleteReadError, OSError):
            pass
        finally:
            try:
                writer.close()
            except Exception:
                pass

    async def _read_request(self, reader, peer):
        try:
            line = await asyncio.wait_for(reader.readline(), 30)
        except asyncio.TimeoutError:
            return None
        if not line:
            return None
        try:
            method, target, version = line.decode("latin-1").strip().split(" ", 2)
        except ValueError:
            return Response(400, "bad request line")
        headers = {"_version": version}
        while True:
            h = await reader.readline()
            if not h or h in (b"\r\n", b"\n"):
                break
            k, _, v = h.decode("latin-1").partition(":")
            headers[k.strip().lower()] = v.strip()
        n = int(headers.get("content-length", "0") or 0)
        if n > MAX_BODY:
            return Response(413, "too large")
        body = await reader.readexactly(n) if n else b""
        return Request(method.upper(), target, headers, body, peer)

    async def _dispatch(self, req):
        try:
            for method, prefix, handler in self.routes:
                if (method == req.method or method == "*" or (method == "GET" and req.method == "HEAD")) \
                        and (req.path == prefix or (prefix.endswith("/") and req.path.startswith(prefix))):
                    r = handler(req)
                    if asyncio.iscoroutine(r):
                        r = await r
                    return r
            if req.method in ("GET", "HEAD"):
                return self._static(req.path)
            return Response(405, "method not allowed")
        except Exception as e:  # keep serving on handler bugs
            import traceback
            traceback.print_exc()
            return Response(500, "error: %s" % e)

    def _static(self, path):
        rel = path.lstrip("/") or "index.html"
        full = os.path.abspath(os.path.join(self.static_dir, rel))
        if not full.startswith(self.static_dir):
            return Response(403, "forbidden")
        if os.path.isdir(full):
            full = os.path.join(full, "index.html")
        if not os.path.isfile(full):
            return Response(404, "not found")
        ext = os.path.splitext(full)[1].lower()
        ctype = MIME.get(ext) or mimetypes.guess_type(full)[0] or "application/octet-stream"
        return Response(200, ctype=ctype, file=full, headers={"Cache-Control": "no-cache"})

    async def _write_response(self, writer, resp, keep, head=False):
        if resp.gen is not None:
            hdr = ["HTTP/1.1 %d %s" % (resp.status, STATUS_TEXT.get(resp.status, "OK")),
                   "Content-Type: " + resp.ctype, "Connection: close"]
            hdr += ["%s: %s" % kv for kv in resp.headers.items()]
            writer.write(("\r\n".join(hdr) + "\r\n\r\n").encode("latin-1"))
            buf = []
            for chunk in resp.gen:
                buf.append(chunk)
                if len(buf) >= 500:
                    writer.write("".join(buf).encode("utf-8"))
                    buf = []
                    await writer.drain()
                    await asyncio.sleep(0)  # let telemetry run while exporting big logs
            writer.write("".join(buf).encode("utf-8"))
            await writer.drain()
            raise ConnectionResetError("done")  # close the connection (no content-length)
        size = os.path.getsize(resp.file) if resp.file else len(resp.body)
        lines = ["HTTP/1.1 %d %s" % (resp.status, STATUS_TEXT.get(resp.status, "OK")),
                 "Content-Type: " + resp.ctype, "Content-Length: %d" % size,
                 "Connection: " + ("keep-alive" if keep else "close")]
        for k, v in resp.headers.items():
            lines.append("%s: %s" % (k, v))
        writer.write(("\r\n".join(lines) + "\r\n\r\n").encode("latin-1"))
        if not head:
            if resp.file:
                with open(resp.file, "rb") as f:
                    while True:
                        chunk = f.read(64 * 1024)
                        if not chunk:
                            break
                        writer.write(chunk)
                        await writer.drain()
            else:
                writer.write(resp.body)
        await writer.drain()

    async def _upgrade(self, req, reader, writer):
        key = req.headers.get("sec-websocket-key", "")
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      "Sec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
        await writer.drain()
        ws = WebSocket(reader, writer, req.peer)
        try:
            await self.ws_handler(ws)
        finally:
            ws.close()


# ---- minimal WebSocket client (tests, screenshot tool) ---------------------------------------
class WsClient:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    @classmethod
    async def connect(cls, host, port, path="/ws"):
        reader, writer = await asyncio.open_connection(host, port)
        key = base64.b64encode(os.urandom(16)).decode()
        writer.write(("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (path, host, port, key)).encode())
        await writer.drain()
        status = await reader.readline()
        if b" 101 " not in status:
            raise ConnectionError("ws upgrade failed: %r" % status)
        while (await reader.readline()) not in (b"\r\n", b""):
            pass
        return cls(reader, writer)

    def send(self, text):
        data = text.encode("utf-8")
        mask = os.urandom(4)
        n = len(data)
        if n < 126:
            hdr = struct.pack("!BB", 0x81, 0x80 | n)
        elif n < 65536:
            hdr = struct.pack("!BBH", 0x81, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x81, 0x80 | 127, n)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
        self.writer.write(hdr + mask + masked)

    async def recv(self):
        parts = []
        while True:
            b0, b1 = await self.reader.readexactly(2)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack("!H", await self.reader.readexactly(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
            data = await self.reader.readexactly(n)
            op = b0 & 0x0F
            if op == 0x8:
                return None
            if op in (0x9, 0xA):
                continue
            parts.append(data)
            if b0 & 0x80:
                return b"".join(parts).decode("utf-8")

    def close(self):
        try:
            self.writer.close()
        except Exception:
            pass
