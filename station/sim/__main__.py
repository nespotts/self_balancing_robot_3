"""python -m station.sim            fake Teensy + bridge + web UI on http://localhost:8080/
python -m station.sim --no-bridge  only the fake Teensy on tcp://127.0.0.1:5761
                                   (then: python -m station.bridge --port tcp://127.0.0.1:5761)"""
import argparse
import asyncio
import os
import sys

from .sim import SimRobot, SimServer


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.sim", description="SBR3 robot simulator")
    ap.add_argument("--listen", default="127.0.0.1:5761", help="host:port of the fake Teensy link")
    ap.add_argument("--no-bridge", action="store_true", help="don't start the bridge in this process")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--log-dir", default=os.environ.get("SBR3_LOG_DIR", "~/sbr3-logs"))
    ap.add_argument("--disarmed", action="store_true", help="remote not armed: robot stays IDLE")
    ap.add_argument("--no-radio", action="store_true", help="pretend the RC remote is off (failsafe)")
    ap.add_argument("--garbage", type=float, default=0.0,
                    help="probability per frame of injecting junk bytes (tests resync)")
    ap.add_argument("--seed", type=int, default=None)
    a = ap.parse_args(argv)
    host, _, port = a.listen.rpartition(":")
    robot = SimRobot(armed=not a.disarmed, radio=not a.no_radio, seed=a.seed)
    server = SimServer(robot, host or "127.0.0.1", int(port), a.garbage)

    async def run():
        jobs = [server.run()]
        if not a.no_bridge:
            from station.bridge.__main__ import DEFAULT_WEB
            from station.bridge.bridge import Bridge
            b = Bridge("tcp://%s:%s" % (host or "127.0.0.1", port), DEFAULT_WEB, a.log_dir,
                       http_port=a.http_port)
            jobs.append(b.run())
        await asyncio.gather(*jobs)

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
