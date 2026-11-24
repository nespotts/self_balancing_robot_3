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
    ap.add_argument("--preset-dir", default=os.environ.get("SBR3_PRESET_DIR", "~/sbr3-presets"),
                    help="tuning presets (Tune page)")
    ap.add_argument("--map-dir", default=os.environ.get("SBR3_MAP_DIR", "~/sbr3-maps"), help="saved occupancy maps")
    ap.add_argument("--falls-dir", default=os.environ.get("SBR3_FALLS_DIR", "~/sbr3-falls"),
                    help="black-box fall records (Logs page > Falls), capped at 200 files / 100 MB")
    ap.add_argument("--lidar", default=os.environ.get("SBR3_LIDAR", "/dev/ttyS5"), metavar="PORT",
                    help="LD19 lidar UART: /dev/ttyS5 (Orange Pi Zero 2W header UART5, see docs/ld19.md), COM7, or "
                         "tcp://host:port (simulator); 'off' = no lidar. A missing port is retried quietly (default /dev/ttyS5)")
    ap.add_argument("--allow-any-origin", action="store_true",
                    default=os.environ.get("SBR3_ALLOW_ANY_ORIGIN", "") in ("1", "true", "yes"),
                    help="turn off the Origin / Host checks on /ws and /api/* (default: only pages served by the "
                         "bridge itself and tools without an Origin header, e.g. curl, tune.py, may command the robot)")
    ap.add_argument("--allow-host", action="append", default=[], metavar="NAME",
                    help="also accept this Host header (a public DNS name for the Pi); repeatable")
    ap.add_argument("--allow-origin", action="append", default=[], metavar="ORIGIN",
                    help="also accept pages from this origin, e.g. http://tablet.example:3000; repeatable")
    ap.add_argument("--quiet", action="store_true")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    b = Bridge(a.port, a.web, a.log_dir, a.host, a.http_port, a.baud, a.release_file, a.log_max_mb, a.quiet,
               preset_dir=a.preset_dir, map_dir=a.map_dir, allow_any_origin=a.allow_any_origin,
               allow_hosts=a.allow_host, allow_origins=a.allow_origin, falls_dir=a.falls_dir,
               lidar_port=None if a.lidar.strip().lower() in ("", "off", "none", "0") else a.lidar.strip())
    try:
        asyncio.run(b.run())
    except KeyboardInterrupt:
        pass
    finally:
        b.falls.flush()
        b.log.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
