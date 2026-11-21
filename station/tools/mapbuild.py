"""Build an occupancy map from a session log, offline (also a CPU benchmark for the Pi).

  python -m station.tools.mapbuild ~/sbr3-logs/sbr3-20261121-033113.jsonl -o map.png [--scale 4]

Reads the ["rng", ...] rows (field names from the "schema" line), runs them through the same
Mapper the bridge uses, writes a PNG of the touched area and prints readings/s of CPU.
"""
import argparse
import json
import os
import sys
import tempfile
import time

from station.bridge.mapping import Mapper


def readings(path):
    cols = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith('["schema"'):
                cols = json.loads(line)[1]["rng"]
            elif line.startswith('["rng"') and cols:
                row = json.loads(line)
                yield dict(zip(cols, row[2:]))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m station.tools.mapbuild")
    ap.add_argument("log")
    ap.add_argument("-o", "--out", default="map.png")
    ap.add_argument("--scale", type=int, default=4, help="pixels per cell in the PNG")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="mapper setting")
    a = ap.parse_args(argv)
    rows = list(readings(os.path.expanduser(a.log)))
    m = Mapper(tempfile.mkdtemp(prefix="sbr3-mapbuild-"))
    for kv in a.set:
        k, _, v = kv.partition("=")
        m.apply_settings({k: (v.lower() in ("1", "true", "yes")) if v.lower() in ("true", "false") else v}, save=False)
    t0 = time.process_time()
    for d in rows:
        m.on_range(d)
    m.matcher.flush()
    cpu = time.process_time() - t0
    with open(a.out, "wb") as f:
        f.write(m.grid.png(a.scale))
    occ, free = m.grid.counts()
    span = (rows[-1]["t_us"] - rows[0]["t_us"]) / 1e6 if len(rows) > 1 else 0
    print("%d readings (%.0f s of robot time), %s" % (len(rows), span, m.n))
    print("CPU %.3f s = %.0f us/reading -> %.2f%% of one core at 50 readings/s"
          % (cpu, 1e6 * cpu / max(1, len(rows)), 100 * cpu / max(1, len(rows)) * 50))
    print("cells: %d occupied, %d free; wrote %s" % (occ, free, a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
