"""Find falls in recorded session logs (the same detector + tagger the bridge runs live, issue #50).

  python -m station.tools.falls_scan ~/sbr3-logs/sbr3-20261121-024818.jsonl [more.jsonl ...] [-o falls_dir] [-q]

Replays every telemetry row through station.bridge.falls.FallRecorder and writes one record per fall into the
output directory (default: a temp dir that is printed). Handy to back-fill old sessions and to cut test fixtures:
  --window T0 T1  only replay session seconds T0..T1 (params / hello lines are always read)
"""
import argparse
import gzip
import json
import os
import sys
import tempfile
import time

from station.bridge import falls as F


def _open(path):
    return gzip.open(path, "rt", encoding="utf-8") if path.endswith(".gz") else open(path, encoding="utf-8")


def scan(path, out_dir, window=None, say=None, params=None):
    """Replay one session file. Returns the list of saved metas. `params` {name: value} overrides."""
    state = {"t": 0.0, "wall0": time.time()}
    pvals = dict(params or {})
    names = {}
    rec = F.FallRecorder(out_dir, params=lambda: dict(pvals), session=lambda: os.path.basename(path),
                         say=say, threaded=False, clock=lambda: state["t"], wall=lambda: state["wall0"] + state["t"])
    saved, cols, rcols = [], None, None
    with _open(path) as f:
        for line in f:
            try:
                a = json.loads(line)
            except ValueError:
                continue
            k = a[0]
            if k == "schema":
                cols, rcols = a[1]["tlm"], a[1].get("rng")
                started = a[1].get("started")
                if started:
                    try:
                        state["wall0"] = time.mktime(time.strptime(started, "%Y-%m-%dT%H:%M:%S"))
                    except ValueError:
                        pass
                continue
            if k == "param":
                pvals[a[3]] = a[4]
                continue
            t = a[1]
            if window and not (window[0] <= t <= window[1]):
                continue
            state["t"] = t
            if k == "tlm":
                d = dict(zip(cols, a[2:]))
                d.setdefault("remote_flags", 0)
                d.setdefault("nav_state", 0)
                rec.on_tlm(d, t)
                saved += rec.tick(t)
            elif k == "rng" and rcols:
                rec.on_range(dict(zip(rcols, a[2:])), t)
            elif k in ("log", "bridge"):
                rec.on_event(k, a[2], t)
            elif k == "cmd" and isinstance(a[2], dict):
                m = a[2]
                if m.get("t") == "estop" or (m.get("t") == "set_mode" and m.get("mode") == 0):
                    rec.user_stop(m.get("who", "?"), t)
                else:
                    rec.on_event("cmd", m, t)
    saved += rec.tick(state["t"] + 100.0)       # finish a fall at the very end of the file
    scan.recorder = rec
    return saved


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"))
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)
    out = a.out or tempfile.mkdtemp(prefix="sbr3-falls-")
    total = 0
    for p in a.files:
        metas = scan(p, out, a.window)
        ig = scan.recorder.ignored_total
        for m in metas:
            total += 1
            if not a.quiet:
                print("%s  %-22s %-6s tags: %s" % (m["id"], m["cause_label"], m["trigger"]["kind"],
                                                   ", ".join("%s" % t["tag"] for t in m["tags"])))
        print("%s: %d fall(s), %d ignored (user stops etc.)" % (os.path.basename(p), len(metas), ig))
    print("records in", out)
    return 0 if total or True else 1


if __name__ == "__main__":
    sys.exit(main())
