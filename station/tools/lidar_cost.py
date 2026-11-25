"""Per-turn cost of the bridge's lidar path, one thread, no event loop (issue #110).

  python -m station.tools.lidar_cost                 40 s of the sim LD19 via MSG_LIDAR_RAW frames, big room, driving
  python -m station.tools.lidar_cost --pi-scale 20   Pi Zero 2W estimate (the 2026-11-25 robot log: ~20 x this PC)

Parts as the bridge's loop profile names them (loopprof): link:parse (link frames), link:lidar_raw (LD19 parser +
turn assembly), lidar.deskew (once per turn, shared), scan:LidarObstacles (the navigator's frame), lidar.scan (the map
step), match.field, lidar.match, lidar.integrate. Telemetry (100 Hz) is fed to the lidar only.
"""
import argparse
import math
import sys
import tempfile


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.tools.lidar_cost", description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--pi-scale", type=float, default=20.0, help="Pi Zero 2W / this PC (the robot log says ~20)")
    a = ap.parse_args(argv)
    from station.bridge import link_protocol as lp, loopprof, lidar as lidar_mod
    from station.bridge.lidar_obstacles import LidarObstacles
    from station.bridge.mapping import Mapper
    from station.bridge.tests.test_ld19 import FakeRobot
    from station.sim.lidar_sim import LidarSim
    from station.tools.loop_bench import big_room
    r = FakeRobot()
    big_room(r.room)
    sim = LidarSim(seed=1, posts=True, dropout=0.01)
    pack, parser = lp.LidarRawPacker(), lp.FrameParser()
    mapper = Mapper(map_dir=tempfile.mkdtemp())
    prof = loopprof.LoopProf()
    mapper.prof = prof
    L = lidar_mod.Lidar(mapper, "teensy", clock=lambda: r.t)
    L.prof = prof
    LidarObstacles(L, lambda fr: True, clock=lambda: r.t)
    dt, next_tlm, scans0, t0 = 0.002, 0.0, None, 0.0
    while r.t < a.seconds:
        r.t += dt
        w = 45.0 if r.t % 8.0 > 6.0 else 0.0           # straight, then a turn, ...
        r.TH = (r.TH + w * dt) % 360.0
        r.X += 250.0 * dt * math.cos(math.radians(r.TH))
        r.Y += 250.0 * dt * math.sin(math.radians(r.TH))
        if not (-500 < r.X < 8500 and -4500 < r.Y < 4500):
            r.TH = (r.TH + 180.0) % 360.0
        r.theta = math.radians(1.2 * math.sin(2 * math.pi * 1.7 * r.t))     # balancing sway
        if r.t >= next_tlm:
            next_tlm = r.t + 0.01
            L.on_tlm({"x_mm": r.X, "y_mm": r.Y, "heading_deg": r.TH, "pitch_deg": math.degrees(r.theta) - 3.0,
                      "yaw_rate_dps": w, "mode": 5, "t_us": int(r.t * 1e6)}, r.t)
        data = sim.step(r)
        for fr in (pack.feed(data, r.t) if data else ()):
            with prof.part("link:parse"):
                for t, p in parser.feed(fr):
                    if t == lp.MSG_LIDAR_RAW:
                        with prof.part("link:lidar_raw"):
                            L.on_raw(p)
        if scans0 is None and r.t > 5.0:
            scans0, t0 = L.n["scans"], r.t
            prof.win = {}
    n = max(1, L.n["scans"] - scans0)
    print("%d turns (%.0f s), %d matched, %d mapped" % (n, r.t - t0, mapper.lmatch.n["matched"], mapper.n["scans"]))
    print("%-22s %12s %14s" % ("part", "ms/turn (PC)", "ms/turn (Pi est)"))
    tot = 0.0
    for k, s in sorted(prof.win.items(), key=lambda kv: -kv[1]):
        tot += s
        print("%-22s %12.3f %14.1f" % (k, 1000.0 * s / n, 1000.0 * s / n * a.pi_scale))
    print("%-22s %12.3f %14.1f  (= %.0f %% of a Pi core at 10 turns/s)" % (
        "total", 1000.0 * tot / n, 1000.0 * tot / n * a.pi_scale, tot / n * a.pi_scale * 10 * 100))
    return 0


if __name__ == "__main__":
    sys.exit(main())
