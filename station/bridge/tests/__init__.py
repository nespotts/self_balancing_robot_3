"""Test-suite plumbing (#49): connections a test opens with asyncio.open_connection (WsClient, raw sockets,
the bridge's own TcpLink) are closed when the test's asyncio.run() ends, so the suite stays free of
"unclosed transport / socket" ResourceWarnings without every test needing a tearDown."""
import asyncio
import threading

_real_open_connection = asyncio.open_connection
_real_run = asyncio.run
_real_start_server = asyncio.start_server
_writers = []


async def _open_connection(*a, **kw):
    reader, writer = await _real_open_connection(*a, **kw)
    _writers.append(writer)
    return reader, writer


async def _start_server(cb, *a, **kw):
    async def tracked(reader, writer):
        _writers.append(writer)
        await cb(reader, writer)
    return await _real_start_server(tracked, *a, **kw)


def _run(main, **kw):
    async def wrapped():
        try:
            return await main
        finally:
            while _writers:
                try:
                    _writers.pop().close()
                except Exception:
                    pass
            await asyncio.sleep(0.05)       # let the transports finish closing before the loop goes
    if threading.current_thread() is not threading.main_thread():
        return _real_run(main, **kw)
    return _real_run(wrapped(), **kw)


asyncio.open_connection = _open_connection
asyncio.run = _run
asyncio.start_server = _start_server
