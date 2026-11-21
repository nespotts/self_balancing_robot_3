"""python -m station.sim            fake Teensy + bridge + web UI on http://localhost:8080/ (this PC only)
python -m station.sim --host 0.0.0.0   same, reachable from the LAN (phones would find it next to the robot)
python -m station.sim --no-bridge  only the fake Teensy on tcp://127.0.0.1:5761
                                   (then: python -m station.bridge --port tcp://127.0.0.1:5761)"""
import argparse
import asyncio
import os
import sys

from .sim import SimRobot, SimServer


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.sim", description="SBR3 robot simulator (firmware v4)")
    ap.add_argument("--listen", default="127.0.0.1:5761", help="host:port of the fake Teensy link")
    ap.add_argument("--no-bridge", action="store_true", help="don't start the bridge in this process")
    ap.add_argument("--host", default="127.0.0.1",
                    help="web UI bind address (default 127.0.0.1: this PC only; 0.0.0.0 = whole LAN)")
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
    a = ap.parse_args(argv)
    host, _, port = a.listen.rpartition(":")
    remote = a.remote or a.remote_disarmed
    robot = SimRobot(remote=remote, remote_armed=not a.remote_disarmed, seed=a.seed, head_error_deg=a.head_error)
    server = SimServer(robot, host or "127.0.0.1", int(port), a.garbage)

    async def run():
        jobs = [server.run()]
        if not a.no_bridge:
            from station.bridge.__main__ import DEFAULT_WEB
            from station.bridge.bridge import Bridge
            b = Bridge("tcp://%s:%s" % (host or "127.0.0.1", port), DEFAULT_WEB, a.log_dir,
                       http_host=a.host, http_port=a.http_port)
            jobs.append(b.run())
        await asyncio.gather(*jobs)

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
