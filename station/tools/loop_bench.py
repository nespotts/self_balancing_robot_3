"""Event-loop stall bench (issue #102): the bridge with the LD19 via the Teensy (MSG_LIDAR_RAW), mapping a big room.

  python -m station.tools.loop_bench                       10 x 10 m room, waypoint tour, 90 s
  python -m station.tools.loop_bench --world room --seconds 60
  python -m station.tools.loop_bench --explore             the planner explores instead (worker-thread jobs too)
  python -m station.tools.loop_bench --kidnap 25           carry the robot away after 25 s: the scan matcher goes lost

The fake Teensy + LD19 (station/sim) runs in a child process, like the real robot's Teensy: its physics and ray casting
do not load the bridge's loop. The bridge runs here with its own event loop, a web client connected (so tiles really go
out over a WebSocket), a robot profile bound (so autosave runs, every --autosave s).

Pi estimate: the bridge's loop profile (loopprof.py) gives the loop work between two ticks; on the Pi Zero 2W the same
work takes --pi-scale times longer (~8-10 x per core), so a tick there is late by about work x scale - TICK_S. The bench
reports the PC numbers and that estimate; --pi-budget also shrinks the time budgets (tile encoding) by the scale, so
the backlog behaves as on the Pi.
"""
import argparse
import asyncio
import gc
import multiprocessing
import os
import shutil
import socket
import struct
import sys
import tempfile
import time

GC = []                              # (generation, s) of every collection during the run
BIG_TOUR = [(3000, 0), (7500, -3500), (7500, 3500), (1000, 3800), (-300, -3500), (4000, -4200), (0, 0)]


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def big_room(room):
    """10 x 10 m: x -1000..9000, y -5000..5000, a few boxes and short partitions (the robot starts at 0, 0)."""
    from station.sim.sim import _box
    s = _box(4000, 0, 10000, 10000)
    s += _box(4500, 2000, 600, 600)
    s += _box(5500, -500, 500, 500)
    s += _box(2000, -1500, 800, 300)
    s.append((6000, 5000, 6000, 4300))
    s.append((9000, -1000, 8300, -1000))
    s.append((-1000, 2000, -400, 2000))
    room.base = s
    room.segs = list(s)


def sim_main(port, world, seed, kidnap=0.0):
    from station.sim.lidar_sim import LidarSim
    from station.sim.sim import SimServer
    from station.sim.tour import DriftRobot
    robot = DriftRobot(seed=seed)
    if world == "big":
        big_room(robot.room)
    robot.set_param(robot.pidx["rng.enable"], 0.0)
    robot.out = []
    server = SimServer(robot, "127.0.0.1", port, 0.0, quiet=True, lidar=LidarSim(seed=seed), lidar_teensy=True)

    async def run():
        jobs = [server.run()]
        if kidnap:
            async def carry():
                from station.sim import extras
                await asyncio.sleep(kidnap)
                extras.teleport(robot, robot.X + 800.0, robot.Y - 600.0, robot.TH + 25.0)   # odometry does not notice
            jobs.append(carry())
        await asyncio.gather(*jobs)
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass


async def ws_client(port, stop):
    """A browser stand-in: reads (and drops) everything, says hi every 2 s so it counts as live."""
    r, w = await asyncio.open_connection("127.0.0.1", port)
    w.write(("GET /ws HTTP/1.1\r\nHost: 127.0.0.1:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
             "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n" % port).encode())
    got = 0

    def frame(text):
        data = text.encode()
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n))
        return head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data))

    async def reader():
        nonlocal got
        while not stop.is_set():
            b = await r.read(65536)
            if not b:
                return
            got += len(b)
    rt = asyncio.ensure_future(reader())
    while not stop.is_set():
        w.write(frame('{"t":"hi","name":"bench"}'))
        await asyncio.sleep(2.0)
    rt.cancel()
    w.close()
    return got


async def bench(a, sim_port, tmp):
    from station.bridge import link_protocol as lp
    from station.bridge import mapping
    from station.bridge.__main__ import DEFAULT_WEB
    from station.bridge.bridge import Bridge
    from station.sim.tour import TOUR, tour
    mapping.AUTOSAVE_S = a.autosave
    if a.pi_budget:
        from station.bridge import lidarmatch
        for mod, name in ((mapping, "TILE_BUDGET_S"), (lidarmatch, "FIELD_BUDGET_S")):
            if hasattr(mod, name):
                setattr(mod, name, getattr(mod, name) / a.pi_scale)
    http_port = _free_port()
    b = Bridge("tcp://127.0.0.1:%d" % sim_port, DEFAULT_WEB, os.path.join(tmp, "logs"), http_host="127.0.0.1",
               http_port=http_port, map_dir=os.path.join(tmp, "maps"), preset_dir=os.path.join(tmp, "presets"),
               falls_dir=os.path.join(tmp, "falls"), lidar_port="teensy", robot="sim", quiet=True, allow_any_origin=True)
    if a.no_match:
        b.mapper.settings["lidar_match"] = False
    prof = b.prof
    windows = []                     # (late s, busy s since the last tick, {part: s})
    orig_tick = prof.tick

    def tick(now, late):
        windows.append((late, sum(prof.since.values()), dict(prof.since)))
        return orig_tick(now, late)
    prof.tick = tick
    gc_t = {}

    def on_gc(phase, info):              # garbage collector pauses (they stall the loop too)
        if phase == "start":
            gc_t["t"] = time.perf_counter()
        elif "t" in gc_t:
            GC.append((info["generation"], time.perf_counter() - gc_t.pop("t")))
    gc.callbacks.append(on_gc)
    stop = asyncio.Event()
    jobs = [asyncio.ensure_future(b.run())]
    await asyncio.sleep(1.0)
    jobs.append(asyncio.ensure_future(ws_client(http_port, stop)))
    if a.explore:
        async def explore():
            while not (b.tlm and b.hello and b.robot):
                await asyncio.sleep(0.2)
            await asyncio.sleep(1.0)
            b.handle_command({"t": "set_mode", "mode": lp.MODE_BALANCE, "arm": True}, "bench")
            await asyncio.sleep(2.5)
            print("explore:", b.handle_command({"t": "plan", "action": "explore"}, "bench"))
        jobs.append(asyncio.ensure_future(explore()))
    else:
        jobs.append(asyncio.ensure_future(tour(b, points=BIG_TOUR if a.world == "big" else TOUR, quiet=True)))
    t_end = time.monotonic() + a.seconds
    t_next = time.monotonic() + 10.0
    while time.monotonic() < t_end:
        await asyncio.sleep(0.5)
        if time.monotonic() >= t_next:
            t_next += 10.0
            st = prof.status()
            g = b.mapper.grid
            print("  [%3.0f s] loop busy %5.1f %%  stall max %4d ms  top %s  threads %s  map %d tiles%s  matcher %s" % (
                a.seconds - (t_end - time.monotonic()), st["busy_pct"], st["stall_max_ms"],
                ", ".join("%s %.1f" % (k, v) for k, v in st["top"][:4]), st["threads"], len(g.touched | g.dirty),
                ", backlog %d" % len(b.mapper.tile_queue) if hasattr(b.mapper, "tile_queue") else "",
                b.mapper.lmatch.mode), flush=True)
    stop.set()
    await asyncio.sleep(2.5)
    for j in jobs:
        j.cancel()
    return b, windows


def report(a, b, windows):
    from station.bridge.bridge import TICK_S
    prof = b.prof
    ws = windows[int(5.0 / TICK_S):]               # skip the start-up (connect, profile, first full map)
    if not ws:
        print("no data")
        return
    secs = sum(max(TICK_S, TICK_S + l) for l, _, _ in ws)
    tot = {}
    peak = {}
    for _, _, d in ws:
        for k, v in d.items():
            tot[k] = tot.get(k, 0.0) + v
            peak[k] = max(peak.get(k, 0.0), v)
    busy = sum(tot.values())
    print("\n== %s room, %s, %.0f s measured (after a 5 s start), Pi scale x%g ==" % (
        a.world, "explore" if a.explore else "tour", secs, a.pi_scale))
    print("loop busy (PC): %.1f %% of a core -> Pi ~%.0f %%" % (100 * busy / secs, 100 * busy / secs * a.pi_scale))
    print("%-22s %10s %12s %16s" % ("part", "ms/s (PC)", "peak ms (PC)", "peak ms (Pi est)"))
    for k, v in sorted(tot.items(), key=lambda kv: -kv[1])[:14]:
        print("%-22s %10.2f %12.1f %16.0f" % (k, 1000 * v / secs, 1000 * peak[k], 1000 * peak[k] * a.pi_scale))
    for k in ("lidar.integrate", "lidar.match", "match.field", "map.tiles"):
        v = sorted(d[k] for _, _, d in ws if d.get(k))
        if v:
            print("  %-16s per tick: n %d, median %.1f ms, p95 %.1f, p99 %.1f, max %.1f (PC)" % (
                k, len(v), 1000 * v[len(v) // 2], 1000 * v[int(0.95 * len(v))], 1000 * v[int(0.99 * len(v))], 1000 * v[-1]))
    late = [l for l, _, _ in ws]
    print("ticks: %d, PC late max %.0f ms, > 50 ms: %d, > 150 ms: %d" % (
        len(ws), 1000 * max(late), sum(1 for l in late if l > 0.05), sum(1 for l in late if l > 0.15)))
    print("longest single loop callback (PC): %.1f ms -> Pi ~%.0f ms (a tick cannot run during one)" % (
        prof.chunk_peak * 1000, prof.chunk_peak * 1000 * a.pi_scale))
    print("  longest callbacks by part (PC ms): " + ", ".join(
        "%s %.1f" % (k, 1000 * v) for k, v in sorted(prof.chunk_peaks.items(), key=lambda kv: -kv[1])[:6]))
    pi = [(s * a.pi_scale - TICK_S, d) for _, s, d in ws]
    worst = max(pi, key=lambda x: x[0])
    print("Pi estimate: max stall %.0f ms; ticks > 50 ms: %d, > 150 ms: %d (of %d); worst: %s" % (
        1000 * max(0.0, worst[0]), sum(1 for p, _ in pi if p > 0.05), sum(1 for p, _ in pi if p > 0.15), len(pi),
        ", ".join("%s %.0f" % (k, 1000 * v * a.pi_scale) for k, v in sorted(worst[1].items(), key=lambda kv: -kv[1])[:4])))
    for gen in (0, 1, 2):
        d = [s for g_, s in GC if g_ == gen]
        if d:
            print("gc gen %d: %d collections, max %.1f ms (PC) -> Pi ~%.0f ms" % (
                gen, len(d), 1000 * max(d), 1000 * max(d) * a.pi_scale))
    m = b.mapper
    print("map: %d tiles touched, lidar scans used %d, map scans %d%s" % (
        len(m.grid.touched | m.grid.dirty), b.lidar.n["used"], m.n["scans"],
        ", tiles sent %d, skipped unchanged %d, backlog %d" % (m.tile_stats["sent"], m.tile_stats["same"], len(m.tile_queue))
        if hasattr(m, "tile_stats") else ""))
    ls = b.lidar.status()
    print("lidar: %s" % {k: ls.get(k) for k in ("state", "rpm", "crc_errors", "cost_ms", "cpu_pct")})
    lm = m.lmatch
    print("matcher: mode %s, state %s" % (lm.mode, getattr(lm, "state", "?")))
    if a.png:
        with open(a.png, "wb") as f:
            f.write(m.grid.png())
        print("map image:", a.png)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.tools.loop_bench", description=__doc__.split("\n")[0])
    ap.add_argument("--world", choices=("big", "room"), default="big", help="big: 10 x 10 m; room: the sim's 5 x 3.5 m")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--explore", action="store_true", help="Explore (planner) instead of the waypoint tour")
    ap.add_argument("--no-match", action="store_true", help="lidar_match off (every turn is mapped)")
    ap.add_argument("--autosave", type=float, default=15.0, help="autosave period, s (the bridge's is 60)")
    ap.add_argument("--pi-scale", type=float, default=9.0, help="Pi Zero 2W / this PC, per core (~8-10)")
    ap.add_argument("--pi-budget", action="store_true", help="time budgets / pi-scale (the Pi's tile throughput)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--kidnap", type=float, default=0.0, metavar="S",
                    help="carry the robot 1 m / 25 deg after S s (odometry does not notice): the matcher goes lost")
    ap.add_argument("--png", default=None, help="write the final map here")
    a = ap.parse_args(argv)
    sim_port = _free_port()
    ctx = multiprocessing.get_context("spawn")
    sim = ctx.Process(target=sim_main, args=(sim_port, a.world, a.seed, a.kidnap), daemon=True)
    sim.start()
    tmp = tempfile.mkdtemp(prefix="sbr3-bench-")
    try:
        time.sleep(1.5)
        b, windows = asyncio.run(bench(a, sim_port, tmp))
        report(a, b, windows)
    finally:
        sim.terminate()
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
