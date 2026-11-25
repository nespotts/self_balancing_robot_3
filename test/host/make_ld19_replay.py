"""Make an LD19 replay for test/host/lidar_sectors_replay.py (issue #113): the Teensy's sectors vs the bridge's.

A replay file (text, gzip ok) holds what the Teensy sees, on its own clock:
  S {"lidar": {...lidar.json settings...}, "params": {"nav.corridor_mm": .., "nav.side_min_mm": ..}, "source": ".."}
  T t_us x_mm y_mm heading_deg pitch_deg yaw_rate_dps      a pose (the telemetry row of one balance step)
  R t_us flags hex                                          one UART chunk the forwarder read (MSG_LIDAR_RAW payload)
in time order. Two sources:

  --session LOG.jsonl     a real bridge session log recorded with SBR3_LOG_LIDAR_RAW=1 (bridge.py): its "tlm" rows and
                          its "lraw" rows (the real LD19 bytes), the chunk times taken back to the Teensy clock with the
                          telemetry's least latency.
  --synth LOG.jsonl [--map MAP.json]
                          the real telemetry of a session (poses, pitch while driving, turns) in a room (a saved
                          occupancy map, or by default walls + boxes / posts around the path, seeded), the LD19 bytes
                          made by station/sim/lidar_sim.py ray casting it at every packet's pose (noise, dropouts,
                          corrupted packets, the cage posts inside the min range), cut into chunks like
                          src/lidar_fwd.cpp does. Until a raw capture exists this is the test data.

The test data in test/host/data was made with
  python test/host/make_ld19_replay.py --synth sbr3-20261125-155822.jsonl --from 245 --to 305
         --poses-out test/host/data/poses_20261125.txt.gz --lidar lidar.json -o replay.txt.gz
(the robot's 60 s of EXPLORE on 2026-11-25: driving ~0.4 m/s, turns, pitch -8 .. +7 deg, and its lidar.json; the house map
stays off this public repo, so the room is the seeded one around the path); lidar_sectors_replay.py makes the replay from
those files again (the same bytes every time).
"""
import argparse
import gzip
import json
import math
import os
import random
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from station.bridge import ld19  # noqa: E402
from station.bridge.lidar import DEFAULTS, clean_settings  # noqa: E402

BYTE_S = 10.0 / 230400.0       # one 8N1 byte on the wire
CHUNK = 180                    # LINK_LIDAR_RAW_MAX
FLUSH_S = 0.012                # LidarForwarder::kFlushUs
GAP_S = 0.002                  # LidarForwarder::kMinGapUs


def _open(path, mode="rt"):
    nl = "\n" if "w" in mode else None
    if path.endswith(".gz"):
        return gzip.open(path, mode, encoding="utf-8", newline=nl)
    return open(path, mode, encoding="utf-8", newline=nl)


def read_poses(path):
    """A poses file (T lines, see write_poses) -> rows (session_t, t_us, x, y, heading, pitch, yaw_rate)."""
    rows = []
    with _open(path) as f:
        for line in f:
            p = line.split()
            if p and p[0] == "T":
                us = int(p[1])
                rows.append((0.0, us) + tuple(float(v) for v in p[2:7]))
    t0 = rows[0][1] if rows else 0
    return [((r[1] - t0) / 1e6,) + r[1:] for r in rows]


def write_poses(path, rows):
    with _open(path, "wt") as f:
        for r in rows:
            f.write("T %d %.2f %.2f %.3f %.3f %.2f\n" % r[1:])


def read_session(path):
    cols, tlm, raw = None, [], []
    with _open(path) as f:
        for line in f:
            r = json.loads(line)
            if r[0] == "schema":
                cols = {c: i + 2 for i, c in enumerate(r[1]["tlm"])}
            elif r[0] == "tlm":
                tlm.append(r)
            elif r[0] == "lraw":
                raw.append(r)
    return cols, tlm, raw


def tlm_rows(cols, tlm, t_from, t_to):
    out, wrap, last = [], 0, None
    for r in tlm:
        if not t_from <= r[1] <= t_to:
            continue
        us = r[cols["t_us"]]
        if last is not None and us < last - (1 << 31):
            wrap += 1 << 32
        last = us
        out.append((r[1], us + wrap, r[cols["x_mm"]], r[cols["y_mm"]], r[cols["heading_deg"]], r[cols["pitch_deg"]],
                    r[cols["yaw_rate_dps"]]))
    return out


class GridRoom:
    """A saved occupancy map as a room: raycast() walks the cells (12.5 mm steps) to the first occupied one."""

    def __init__(self, path, occ=0.65):
        import numpy as np
        from station.bridge.occgrid import Grid, prob
        with open(path, encoding="utf-8") as f:
            g = Grid.from_json(f.read())
        lut = np.array([1 if prob(v) > occ else 0 for v in range(256)], dtype=np.uint8)
        self.occ = lut[np.frombuffer(bytes(g.cells), dtype=np.uint8)].reshape(g.H, g.W)
        self.x0, self.y0, self.res, self.W, self.H = g.x0, g.y0, g.res, g.W, g.H
        self.corr = g.meta.get("corr") or [0.0, 0.0, 0.0]
        self.np = np
        self.steps = np.arange(12.5, 12000.0, 12.5)

    def raycast(self, x, y, th, rmax):
        np = self.np
        s = self.steps[self.steps <= rmax]
        ix = np.floor((x + s * math.cos(th) - self.x0) / self.res).astype(np.int64)
        iy = np.floor((y + s * math.sin(th) - self.y0) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        hit = np.zeros(len(s), dtype=bool)
        hit[ok] = self.occ[iy[ok], ix[ok]] != 0
        k = np.argmax(hit)
        return float(s[k]) if hit[k] else None

    def to_map(self, x, y, h):
        dx, dy, dt = self.corr
        c, sn = math.cos(math.radians(dt)), math.sin(math.radians(dt))
        return c * x - sn * y + dx, sn * x + c * y + dy, h + dt


class SegRoom:
    """A room of wall segments around a path (odometry frame): the bounding box of the poses + a margin, and boxes /
    posts placed off the path (seeded). raycast() is vectorised over the segments (numpy)."""

    def __init__(self, poses, seed=113, margin=700.0, n_boxes=40):
        import numpy as np
        self.np = np
        xs, ys = [p[1] for p in poses], [p[2] for p in poses]
        x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
        segs = self._box(x0, y0, x1, y1)
        rng = random.Random(seed)
        path = np.array([(p[1], p[2]) for p in poses[::10]])
        placed = 0
        for _ in range(4000):
            if placed >= n_boxes:
                break
            w, h = (rng.uniform(30, 60),) * 2 if rng.random() < 0.3 else (rng.uniform(150, 700), rng.uniform(150, 700))
            cx, cy = rng.uniform(x0 + w, x1 - w), rng.uniform(y0 + h, y1 - h)
            if np.min(np.hypot(path[:, 0] - cx, path[:, 1] - cy)) < 250.0 + 0.5 * math.hypot(w, h):
                continue                                   # keep the path itself free (the robot drove there)
            segs += self._box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            placed += 1
        a = np.array(segs, dtype=float)
        self.ax, self.ay = a[:, 0], a[:, 1]
        self.dx, self.dy = a[:, 2] - a[:, 0], a[:, 3] - a[:, 1]
        self.corr = [0.0, 0.0, 0.0]

    @staticmethod
    def _box(x0, y0, x1, y1):
        return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]

    def raycast(self, x, y, th, rmax):
        np = self.np
        c, s = math.cos(th), math.sin(th)
        den = c * self.dy - s * self.dx                    # ray (x, y) + t (c, s) vs segment a + u d
        with np.errstate(divide="ignore", invalid="ignore"):
            ex, ey = self.ax - x, self.ay - y
            t = (ex * self.dy - ey * self.dx) / den
            u = (ex * s - ey * c) / den
            ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        if not ok.any():
            return None
        d = float(t[ok].min())
        return d if d <= rmax else None

    def to_map(self, x, y, h):
        return x, y, h


class Robot:
    pass


def interp(rows, i, t):
    while i + 1 < len(rows) and rows[i + 1][0] <= t:
        i += 1
    a = rows[i]
    if i + 1 >= len(rows) or t <= a[0]:
        return i, a[1:]
    b = rows[i + 1]
    f = (t - a[0]) / (b[0] - a[0])
    dh = (b[3] - a[3] + 180.0) % 360.0 - 180.0
    return i, (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dh, a[4] + f * (b[4] - a[4]))


def synth(rows, room, settings, seed=113, bad=0.003):
    from station.sim.lidar_sim import LidarSim
    t_us0 = rows[0][1]
    poses = [((r[1] - t_us0) / 1e6, r[2], r[3], r[4], r[5]) for r in rows]
    pm = sorted(p[4] for p in poses)[len(poses) // 2]
    sim = LidarSim(seed=seed, yaw_deg=settings["yaw_deg"], flip=settings["flip"], height_mm=settings["height_mm"],
                   x_mm=settings["x_mm"], y_mm=settings["y_mm"], posts=True, dropout=0.01, bad_packets=bad)
    rng = random.Random(seed + 1)
    rob = Robot()
    rob.room = room
    dt = 0.0005
    t, i = 0.0, 0
    end = poses[-1][0]
    pending = bytearray()          # bytes in the UART buffer
    first = None                   # when the oldest of them arrived
    last_chunk = -1.0
    wire_free = 0.0                # when the UART line is free for the next packet's bytes
    arrivals = []                  # (time the packet's bytes are all in, bytes)
    chunks = []
    while t < end:
        t += dt
        i, (x, y, h, p) = interp(poses, i, t)
        rob.X, rob.Y, rob.TH = room.to_map(x, y, h)
        rob.theta = math.radians(p - pm)
        rob.t = t
        b = sim.step(rob)
        for k in range(0, len(b), ld19.PKT_LEN):
            pk = b[k:k + ld19.PKT_LEN]
            start = max(t, wire_free)
            wire_free = start + len(pk) * BYTE_S
            arrivals.append((wire_free, pk))
        # the forwarder's passes (every 0.5 ms here): a chunk of <= 180 bytes when 180 wait or the first waited 12 ms
        while arrivals and arrivals[0][0] <= t:
            ta, pk = arrivals.pop(0)
            if not pending:
                first = ta
            pending += pk
        if pending and (len(pending) >= CHUNK or t - first >= FLUSH_S) and t - last_chunk >= GAP_S:
            n = min(CHUNK, len(pending))
            tr = t + rng.uniform(0.0, 0.0004)
            chunks.append((t_us0 + int(round(tr * 1e6)), bytes(pending[:n])))
            del pending[:n]
            first = t if pending else None
            last_chunk = t
    return chunks


def from_session(cols, tlm, raw, rows):
    """Real lraw rows: Pi times -> Teensy clock with the telemetry's least latency (Pi arrival - t_us)."""
    lat = min(r[0] - r[1] / 1e6 for r in rows)
    lo, hi = rows[0][0], rows[-1][0]
    out, seq = [], None
    for r in raw:                  # ["lraw", t, seq, flags, hex]
        if not lo <= r[1] <= hi:
            continue
        flags = r[3]
        if seq is not None and r[2] != (seq + 1) & 0xFF:
            flags |= 1             # chunks lost on the link: a hole for the Teensy's parser too (as the bridge does)
        seq = r[2]
        out.append((int(round((r[1] - lat) * 1e6)), flags, r[4]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--session", help="session log with lraw rows (SBR3_LOG_LIDAR_RAW=1)")
    g.add_argument("--synth", help="session log (or a poses file: T lines) whose telemetry drives a simulated LD19 in --map")
    ap.add_argument("--poses-out", help="also write the window's poses as a compact poses file (test data)")
    ap.add_argument("--map", help="occupancy map (sbr3-occgrid-1 JSON) for --synth; default: a room of segments around "
                                  "the path (SegRoom)")
    ap.add_argument("--lidar", help="lidar.json of the robot profile (default: lidar.py DEFAULTS)")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="session seconds")
    ap.add_argument("--to", dest="t_to", type=float, default=1e9)
    ap.add_argument("--corridor", type=float, default=220.0, help="nav.corridor_mm")
    ap.add_argument("--side-min", type=float, default=120.0, help="nav.side_min_mm")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args(argv)
    settings = dict(DEFAULTS)
    if a.lidar:
        with open(a.lidar, encoding="utf-8") as f:
            settings = clean_settings(json.load(f), settings)
    src_path = a.session or a.synth
    with _open(src_path) as f:
        is_log = f.read(10).startswith('["schema"')
    if is_log:
        cols, tlm, raw = read_session(src_path)
        rows = tlm_rows(cols, tlm, a.t_from, a.t_to)
    else:
        cols, tlm, raw = None, [], []
        rows = [r for r in read_poses(src_path) if a.t_from <= r[0] <= a.t_to]
    if len(rows) < 100:
        sys.exit("too few telemetry rows in that window")
    if a.poses_out:
        write_poses(a.poses_out, rows)
        rows = read_poses(a.poses_out)        # (the rounded values, so the replay matches one made from that file)
    if a.session:
        if not raw:
            sys.exit("no lraw rows: record the session with SBR3_LOG_LIDAR_RAW=1")
        chunks = [(t, int(fl), bytes.fromhex(h)) for t, fl, h in from_session(cols, tlm, raw, [(r[0], r[1]) for r in rows])]
        src = "real LD19 bytes (lraw) + telemetry of %s %.0f..%.0f s" % (os.path.basename(a.session), rows[0][0], rows[-1][0])
    else:
        if a.map:
            room, where = GridRoom(a.map), os.path.basename(a.map)
        else:
            room, where = SegRoom([r[1:] for r in rows]), "a room around the path (walls + 40 boxes / posts, seeded)"
        chunks = [(t, 0, b) for t, b in synth(rows, room, settings)]
        src = "telemetry of %s %.0f..%.0f s, LD19 simulated in %s" % (os.path.basename(a.synth), rows[0][0], rows[-1][0],
                                                                       where)
    ev = [(r[1], 0, "T %d %.4f %.4f %.4f %.4f %.4f" % (r[1], r[2], r[3], r[4], r[5], r[6])) for r in rows]
    ev += [(t, 1, "R %d %d %s" % (t, fl, b.hex())) for t, fl, b in chunks]
    ev.sort(key=lambda e: (e[0], e[1]))
    head = {"lidar": settings, "params": {"nav.corridor_mm": a.corridor, "nav.side_min_mm": a.side_min}, "source": src}
    with _open(a.out, "wt") as f:
        f.write("S " + json.dumps(head, separators=(",", ":")) + "\n")
        for _t, _k, line in ev:
            f.write(line + "\n")
    print("%s: %d poses, %d chunks (%d bytes), %s" % (a.out, len(rows), len(chunks), sum(len(c[2]) for c in chunks), src))


if __name__ == "__main__":
    main()
