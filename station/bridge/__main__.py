"""python -m station.bridge [--port /dev/ttyACM0 | COM5 | tcp://127.0.0.1:5761] [--http-port 8080]"""
import argparse
import asyncio
import os
import sys

from .bridge import DEFAULT_LOG_MAX_MB, DEFAULT_RELEASE_FILE, Bridge

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WEB = os.path.join(os.path.dirname(HERE), "web")


def build_parser():
    ap = argparse.ArgumentParser(prog="python -m station.bridge", description="SBR3 ground-station bridge")
    ap.add_argument("--port", default=os.environ.get("SBR3_PORT", "/dev/ttyACM0"),
                    help="Teensy serial port, or tcp://host:port for the simulator (default /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB serial, kept for real UARTs")
    ap.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--http-port", type=int, default=int(os.environ.get("SBR3_HTTP_PORT", "8080")))
    ap.add_argument("--web", default=DEFAULT_WEB, help="web app directory")
    ap.add_argument("--log-dir", default=os.environ.get("SBR3_LOG_DIR", "~/sbr3-logs"))
    ap.add_argument("--log-max-mb", type=float, default=DEFAULT_LOG_MAX_MB,
                    help="delete the oldest sessions above this total (default 2048 = 2 GB)")
    ap.add_argument("--release-file", default=None,
                    help="while this file exists the serial port is released (default %s)" % DEFAULT_RELEASE_FILE)
    ap.add_argument("--quiet", action="store_true")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    b = Bridge(a.port, a.web, a.log_dir, a.host, a.http_port, a.baud, a.release_file, a.log_max_mb, a.quiet)
    try:
        asyncio.run(b.run())
    except KeyboardInterrupt:
        pass
    finally:
        b.log.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
