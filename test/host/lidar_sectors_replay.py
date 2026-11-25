"""Equivalence of the Teensy's LD19 obstacle sectors (src/lidar_sectors.cpp, issue #113) with the bridge's
(station/bridge/lidar_obstacles.py + lidar.Lidar.deskew + ld19.Ld19Stream) on the same LD19 bytes and poses.

  python test/host/lidar_sectors_replay.py --exe build/lidar_sectors_sim.exe [--replay FILE]

Without --replay the replay is made from test/host/data (the robot's real telemetry of 60 s of EXPLORE and its lidar.json;
the LD19 bytes simulated in a seeded room around that path: make_ld19_replay.py). With a capture recorded by the bridge
(SBR3_LOG_LIDAR_RAW=1; make_ld19_replay.py --session) the same comparison runs on real LD19 bytes.

Both sides get every pose (T line) and every UART chunk (R line) in order: the bridge's on the Pi clock (Teensy clock +
a constant link latency, as RobotClock / Ld19Stream anchor them), the C++ on the Teensy clock, a frame after each chunk.
Checks: the same frames (the same chunks produce one, the same skip counters), and in every frame the same sectors
(present in both or neither, the points within TOL_MM: the C++ works in float, the bridge in double). Prints the
C++ cost per slice / turn on this PC.
"""
import argparse
import gzip
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from station.bridge import link_protocol as lp  # noqa: E402
from station.bridge.lidar import Lidar  # noqa: E402
from station.bridge.lidar_obstacles import LidarObstacles, corridor_half  # noqa: E402
from station.bridge.mapping import Mapper  # noqa: E402

LATENCY_S = 0.002           # Teensy clock -> Pi clock (any constant: both anchors take it out)
TOL_MM = 2                  # a sector's point, C++ (float) vs bridge (double), after rounding to mm
MAX_FLIP_FRAC = 0.005       # sectors whose choice may differ (a tie: two points equally near, a point on an edge) ...
TIE_MM = 3.0                # ... their keys within this (the points were rounded to mm), or one on the corridor edge
DATA = os.path.join(HERE, "data")


def make_default_replay(path):
    import make_ld19_replay as mk
    mk.main(["--synth", os.path.join(DATA, "poses_20261125.txt.gz"), "--lidar", os.path.join(DATA, "lidar_balancer.json"),
             "-o", path])


def read_replay(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8") as f:
        lines = f.read().splitlines()
    head = json.loads(lines[0][2:]) if lines and lines[0].startswith("S ") else {"lidar": {}, "params": {}}
    return head, lines[1:]


def bridge_frames(head, lines):
    """The bridge's frames: [(t_us of the chunk that finished the turn, decoded frame)], LidarObstacles.n."""
    now = [0.0]
    tmp = tempfile.mkdtemp()
    try:
        m = Mapper(map_dir=tmp)
        m.settings["enabled"] = False                 # (no grid: only the deskew + the obstacle frame)
        m.lmatch = None
        L = Lidar(m, "teensy", clock=lambda: now[0])
        L.apply_settings(head["lidar"], save=False)
        out, cur = [], [0]

        def send(frame):
            (t, p), = lp.FrameParser().feed(frame)
            out.append((cur[0], lp.decode(t, p)))
            return True
        ob = LidarObstacles(L, send, clock=lambda: now[0], params=lambda: head["params"])
        seq = 0
        for line in lines:
            p = line.split()
            if not p:
                continue
            us = int(p[1])
            now[0] = us / 1e6 + LATENCY_S
            if p[0] == "T":
                L.on_tlm({"t_us": us & 0xFFFFFFFF, "x_mm": float(p[2]), "y_mm": float(p[3]), "heading_deg": float(p[4]),
                          "pitch_deg": float(p[5]), "yaw_rate_dps": float(p[6])}, now[0])
            elif p[0] == "R":
                cur[0] = us
                L.on_raw(bytes((seq & 0xFF, int(p[2]))) + bytes.fromhex(p[3]))
                seq += 1
        return out, dict(ob.n), L.reader.stats()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def cpp_frames(exe, path, head):
    cfg = lp.lidar_config_payload(head["lidar"]).hex()
    half = corridor_half(head["params"])
    r = subprocess.run([exe, "--replay", path, "--config", cfg, "--half", repr(half)], capture_output=True, text=True,
                       check=True)
    frames, counts, timing = [], {}, []
    for line in r.stdout.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "F":
            xy = [int(v) for v in p[4:]]
            frames.append((int(p[1]), {"seq": int(p[2]), "age_ms": int(p[3]),
                                       "pts": [(xy[2 * i], xy[2 * i + 1]) for i in range(lp.LINK_LIDAR_SECTORS)]}))
        elif p[0] == "N":
            counts = {p[i]: int(p[i + 1]) for i in range(1, len(p) - 1, 2)}
        elif line.startswith("  "):
            timing.append(line.strip())
    return frames, counts, timing


def _choice(p, half):
    """What a sector's point won by: ("corridor", the navigator's front / rear distance) or ("nearest", range)."""
    x, y = p
    if abs(y) <= half:
        return "corridor", (x - 38.9 if x >= 26.0 else 13.1 - x)
    return "nearest", math.hypot(x, y)


def _to_boundary(p):
    """mm from the point to the nearest sector boundary ray ((i + 0.5) x 10 deg; the points are rounded to 1 mm)."""
    x, y = p
    a = math.degrees(math.atan2(y, x))
    off = abs((a - 5.0) % 10.0)
    off = min(off, 10.0 - off)
    return math.hypot(x, y) * math.sin(math.radians(off))


def compare(py, cc, half):
    """-> (report lines, ok). Sectors with another point than the bridge's are fine when it is a tie: the two points won
    the same way (corridor / nearest) by keys within TIE_MM, or one of them sits on the corridor edge."""
    rep = []
    n_tie_bad = 0
    tie_worst = 0.0
    by_t = {t: d for t, d in cc}
    common = [(t, d) for t, d in py if t in by_t]
    only_py = [t for t, _d in py if t not in by_t]
    only_cc = [t for t, _d in cc if t not in {t for t, _ in py}]
    n_sec = n_same = n_tol = n_presence = 0
    worst = 0.0
    worst_at = None
    age_diff = 0
    for t, d in common:
        e = by_t[t]
        age_diff = max(age_diff, abs(d["age_ms"] - e["age_ms"]))
        for i, (a, b) in enumerate(zip(d["pts"], e["pts"])):
            n_sec += 1
            if a == b:
                n_same += 1
                continue
            if (a == (0, 0)) != (b == (0, 0)):
                n_presence += 1
                continue
            dd = math.hypot(a[0] - b[0], a[1] - b[1])
            if dd <= TOL_MM:
                n_tol += 1
                continue
            if dd > worst:
                worst, worst_at = dd, (t, i, a, b)
            (ka, va), (kb, vb) = _choice(a, half), _choice(b, half)
            edge = min(abs(abs(a[1]) - half), abs(abs(b[1]) - half)) <= TIE_MM
            if min(_to_boundary(a), _to_boundary(b)) <= 1.5:
                continue                        # a point on a sector boundary: in the next sector on one side
            if ka == kb:
                tie_worst = max(tie_worst, abs(va - vb))
                if abs(va - vb) > TIE_MM:
                    n_tie_bad += 1
            elif not edge:
                n_tie_bad += 1
    n_flip = n_sec - n_same - n_tol
    rep.append("frames: bridge %d, C++ %d, same chunk %d, only bridge %d, only C++ %d" % (
        len(py), len(cc), len(common), len(only_py), len(only_cc)))
    if n_sec:
        rep.append("sectors: %d compared, %d identical (%.2f %%), %d within %d mm, %d other point (%.3f %%; %d present in one "
                   "only), worst %.0f mm %s" % (n_sec, n_same, 100.0 * n_same / n_sec, n_tol, TOL_MM, n_flip,
                                                100.0 * n_flip / n_sec, n_presence, worst, worst_at or ""))
        rep.append("  the other points: all ties (on a sector boundary / the corridor edge, or the same rule with keys within %.1f mm): %s, "
                   "%d not" % (tie_worst, "yes" if not n_tie_bad else "NO", n_tie_bad))
        rep.append("age_ms: largest difference %d ms" % age_diff)
    ok = (len(py) > 0 and not only_py and not only_cc and n_sec > 0 and n_flip <= MAX_FLIP_FRAC * n_sec
          and n_tie_bad == 0 and n_presence == 0 and age_diff <= 1)
    return rep, ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", required=True, help="the built test/host/lidar_sectors_sim")
    ap.add_argument("--replay", help="replay file (default: made from test/host/data)")
    a = ap.parse_args(argv)
    tmp = tempfile.mkdtemp()
    try:
        src = a.replay
        if not src:
            src = os.path.join(tmp, "replay.txt")
            make_default_replay(src)
        head, lines = read_replay(src)
        plain = os.path.join(tmp, "replay_plain.txt")           # the C++ side reads it uncompressed
        with open(plain, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines) + "\n")
        print("replay: %s" % head.get("source", src))
        py, py_n, st = bridge_frames(head, lines)
        cc, cc_n, timing = cpp_frames(a.exe, plain, head)
        print("bridge: sent %d, skipped %s; LD19 packets %d, crc %d, partial %d" % (
            py_n["sent"], {k: v for k, v in py_n.items() if k != "sent" and v}, st["packets"], st["crc_errors"],
            st["partial"]))
        print("C++:    %s" % " ".join("%s %d" % kv for kv in cc_n.items()))
        rep, ok = compare(py, cc, corridor_half(head["params"]))
        for k_py, k_cc in (("stale", "stale"), ("tilt", "tilt"), ("few", "few"), ("no_pose", "no_pose")):
            if py_n.get(k_py, 0) != cc_n.get(k_cc, 0):
                rep.append("counter %s differs: bridge %d, C++ %d" % (k_py, py_n.get(k_py, 0), cc_n.get(k_cc, 0)))
                ok = False
        if st["packets"] != cc_n.get("packets") or st["crc_errors"] != cc_n.get("crc"):
            rep.append("parser counters differ")
            ok = False
        for r in rep:
            print(r)
        print("C++ cost on this PC:")
        for t in timing:
            print("  " + t)
        print("RESULT:", "EQUIVALENT" if ok else "DIFFERENT")
        return 0 if ok else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
