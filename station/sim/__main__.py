"""python -m station.sim            fake Teensy + bridge + web UI on http://localhost:8080/
python -m station.sim --no-bridge  only the fake Teensy on tcp://127.0.0.1:5761
                                   (then: python -m station.bridge --port tcp://127.0.0.1:5761)"""
import argparse
import asyncio
import os
import sys

from .sim import SimServer
from .tour import DriftRobot, tour


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.sim", description="SBR3 robot simulator (firmware v4)")
    ap.add_argument("--listen", default="127.0.0.1:5761", help="host:port of the fake Teensy link")
    ap.add_argument("--no-bridge", action="store_true", help="don't start the bridge in this process")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--log-dir", default=os.environ.get("SBR3_LOG_DIR", "~/sbr3-logs"))
    ap.add_argument("--remote", action="store_true",
                    help="pretend the RC remote is on and ARMED (its switch rules arming; default: no remote, "
                         "the station arms with SET_MODE)")
    ap.add_argument("--remote-disarmed", action="store_true", help="RC remote on but its ARMED switch off")
    ap.add_argument("--garbage", type=float, default=0.0,
                    help="probability per frame of injecting junk bytes (tests resync)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--head-error", type=float, default=0.0, metavar="DEG",
                    help="range-sensor head mounted this far CCW (+) of true; rng.servo_trim_deg -DEG fixes it")
    ap.add_argument("--tour", action="store_true", help="arm and drive a waypoint loop around the room (mapping demo)")
    ap.add_argument("--drift", type=float, default=0.0, metavar="DEG_PER_MIN",
                    help="odometry drift: heading gains this many deg/min and distances read 2 %% long")
    a = ap.parse_args(argv)
    host, _, port = a.listen.rpartition(":")
    remote = a.remote or a.remote_disarmed
    robot = DriftRobot(remote=remote, remote_armed=not a.remote_disarmed, seed=a.seed, head_error_deg=a.head_error,
                       drift_deg_min=a.drift, drift_scale=0.02 if a.drift else 0.0)
    server = SimServer(robot, host or "127.0.0.1", int(port), a.garbage)

    async def run():
        jobs = [server.run()]
        if not a.no_bridge:
            from station.bridge.__main__ import DEFAULT_WEB
            from station.bridge.bridge import Bridge
            b = Bridge("tcp://%s:%s" % (host or "127.0.0.1", port), DEFAULT_WEB, a.log_dir,
                       http_port=a.http_port, map_dir=os.environ.get("SBR3_MAP_DIR", "~/sbr3-maps"))
            jobs.append(b.run())
            if a.tour:
                jobs.append(tour(b))
        await asyncio.gather(*jobs)

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
