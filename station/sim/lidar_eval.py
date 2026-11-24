"""LD19 scan matching evaluation (issue #78): pose error vs the sim's ground truth, map sharpness and CPU, with the
matcher off and on, odometry drift / miscalibration, real LD19 packets (lidar_sim.py), sim time, no network.

  python -m station.sim.lidar_eval                      drift 3 deg/min + 2 % distance, seeds 1 2, 3 tour loops
  python -m station.sim.lidar_eval --miscal             wheels 69.3 mm (params 66), track 131 (136), right wheel +1.5 %,
                                                        gyro 0.05 deg/s
  python -m station.sim.lidar_eval --drift 6 --seeds 1 2 3 --loops 2

Per run: stand up, drive the TOUR loop as planner gotos (map frame, a short stop at every corner) --loops times, then
go to the saved places (Home = start, "Far" = (3500, 1400)) and measure where the robot really is.
Pose error = |map pose (corrected odometry) - true pose|, every 0.5 s, in the map frame: the map starts where the odometry
is when the first turn is mapped (after standing up: the stand-up lurch already has odometry error), so the truth is
moved into that frame first (as an absolute trajectory error). Map: share of occupied cells within 50 mm of a true wall,
their number and mean distance to it (thick / doubled walls: more cells, farther).
"""
import argparse
import math
import shutil
import tempfile
import time

from station.bridge import ld19
from station.bridge import link_protocol as lp
from station.bridge.lidar import Lidar
from station.bridge.mapping import Mapper
from .lidar_sim import LidarSim
from .tour import TOUR, DriftRobot, OfflineBridge, heartbeat

FAR = (3500.0, 1400.0)
MISCAL = {"wheel_d": 69.3, "track": 131.0, "lr": 1.015, "gyro_bias_dps": 0.05}


def frame(a, b):
    """Rigid transform (dx, dy, dth deg) taking pose a = (x, y, h) to pose b."""
    dt = b[2] - a[2]
    c, s = math.cos(math.radians(dt)), math.sin(math.radians(dt))
    return b[0] - (c * a[0] - s * a[1]), b[1] - (s * a[0] + c * a[1]), dt


def tf(F, x, y):
    c, s = math.cos(math.radians(F[2])), math.sin(math.radians(F[2]))
    return c * x - s * y + F[0], s * x + c * y + F[1]


def wall_stats(m, room, F=(0.0, 0.0, 0.0), occ_byte=150):
    """(share of occupied cells within 50 mm of a true wall, occupied cells, their mean distance to a wall mm).
    F: world -> map frame."""
    g = m.grid
    inv = frame(F, (0.0, 0.0, 0.0)) if any(F) else None
    d = []
    for i, v in enumerate(g.cells):
        if v > occ_byte:
            x, y = g.x0 + (i % g.W + 0.5) * g.res, g.y0 + (i // g.W + 0.5) * g.res
            if inv:
                x, y = tf(inv, x, y)
            d.append(room.clearance(x, y))
    if not d:
        return 0.0, 0, 0.0
    return sum(1 for v in d if v < 50) / len(d), len(d), sum(min(v, 500.0) for v in d) / len(d)


class Run:
    def __init__(self, seed, match, drift=3.0, scale=0.02, miscal=False, settings=None, lidar_settings=None):
        self.tmp = tempfile.mkdtemp()
        r = self.r = DriftRobot(seed=seed, drift_deg_min=0.0 if miscal else drift, drift_scale=0.0 if miscal else scale)
        if miscal:
            r.set_hardware(**MISCAL)
        m = self.m = Mapper(self.tmp)
        m.settings["lidar_match"] = match
        m.apply_settings(settings or {}, save=False)
        m.places.add("Far", *FAR)
        self.lid = Lidar(m, "tcp://sim", clock=lambda: r.t)
        self.lid.apply_settings(lidar_settings or {}, save=False)
        self.sim = LidarSim(seed=seed)
        self.stream, self.asm = ld19.Ld19Stream(), ld19.ScanAssembler()
        self.parser = lp.FrameParser()
        self.b = OfflineBridge(r, m)
        self.errs, self.herrs = [], []
        self.F = None                # world -> map frame (fixed when the first turn is mapped)
        self.next_sample = 0.0
        self.scan_s, self.scans = 0.0, 0
        self.match_ms = []

    def close(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def step(self, n=4):
        """n physics steps (2.5 ms each), then the bridge side: telemetry -> pose history, LD19 packets -> scans."""
        r, m, lid = self.r, self.m, self.lid
        for _ in range(n):           # per physics step: telemetry and packets carry the time they were made at
            r.step()
            if r.out:
                out, r.out = r.out, []
                for t, pl in self.parser.feed(b"".join(out)):
                    if t == lp.MSG_TELEMETRY:
                        lid.on_tlm(lp.decode_telemetry(pl), r.t)
                    elif t == lp.MSG_RANGE:
                        m.on_range(lp.decode_range(pl))
                    elif t == lp.MSG_LOG:
                        self.b.planner.on_log(lp.decode_log(pl))
            data = self.sim.step(r)
            if not data:
                continue
            for p in self.stream.feed(data, now=r.t):
                sc = self.asm.add(p)
                if sc:
                    k = m.lmatch.k
                    t0 = time.perf_counter()
                    lid.on_scan(sc)
                    self.scan_s += time.perf_counter() - t0
                    self.scans += 1
                    if m.lmatch.k != k and m.lmatch.k % max(1, int(m.settings["match_every_n"])) == 0:
                        self.match_ms.append(m.lmatch.cost_ms)
        if self.F is None and lid.n["used"]:
            self.F = frame((r.X, r.Y, r.TH), m.map_pose(*r.odom()))
        if r.t >= self.next_sample and lid.n["used"]:
            self.next_sample = r.t + 0.5
            x, y, h = m.map_pose(*r.odom())
            tx, ty = tf(self.F, r.X, r.Y)
            self.errs.append(math.hypot(x - tx, y - ty))
            self.herrs.append(abs((h - r.TH - self.F[2] + 180.0) % 360.0 - 180.0))

    def stand_up(self):
        r = self.r
        r.set_mode(lp.MODE_BALANCE, arm=True)
        t_end = r.t + 2.5
        while r.t < t_end:
            heartbeat(r)
            for _ in range(10):
                self.step()
        self.b.update()

    def drive(self, command, max_s=90.0):
        """A planner command (dict or callable(bridge)) until the planner is idle again."""
        b, r, m = self.b, self.r, self.m
        err = command(b) if callable(command) else b.planner.command(command, "eval")
        if err:
            raise RuntimeError(err)
        start = r.t
        while r.t - start < max_s:
            heartbeat(r)
            for _ in range(10):
                self.step()
            m.poll(r.t)
            b.update()
            b.planner.tick(r.t)
            if b.planner.mode == "idle":
                break

    def settle(self, s=1.0):
        t_end = self.r.t + s
        while self.r.t < t_end:
            heartbeat(self.r)
            for _ in range(10):
                self.step()
            self.b.update()


def run(seed, match, drift=3.0, scale=0.02, miscal=False, loops=3, settings=None, lidar_settings=None, quiet=True):
    e = Run(seed, match, drift, scale, miscal, settings, lidar_settings)
    try:
        e.stand_up()
        for _ in range(loops):
            for p in TOUR:
                e.drive(lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False), max_s=60.0)
                e.settle(0.5)
        ret = {}
        for name in ("Far", "Home"):
            e.drive({"t": "plan", "action": "goto", "place": name}, max_s=90.0)
            e.settle(1.0)
            tx, ty = e.m.place_xy(name)
            x, y = tf(e.F, e.r.X, e.r.Y)
            ret[name] = math.hypot(x - tx, y - ty)
        fit, n_occ, occ_mm = wall_stats(e.m, e.r.room, e.F)
        lm = e.m.lmatch
        errs, herrs = e.errs, e.herrs
        mm = sorted(e.match_ms)
        out = {"seed": seed, "match": match, "t_s": round(e.r.t), "frame": [round(v, 2) for v in e.F],
               "err_mean_mm": round(sum(errs) / len(errs)), "err_max_mm": round(max(errs)), "err_end_mm": round(errs[-1]),
               "head_mean_deg": round(sum(herrs) / len(herrs), 2), "head_max_deg": round(max(herrs), 2),
               "head_end_deg": round(herrs[-1], 2), "far_mm": round(ret["Far"]), "home_mm": round(ret["Home"]),
               "wall_fit": round(fit, 3), "occ_cells": n_occ, "occ_mm": round(occ_mm, 1),
               "scan_ms": round(1000.0 * e.scan_s / max(1, e.scans), 2), "scans": e.scans,
               "match_ms": round(sum(mm) / len(mm), 2) if mm else 0.0, "match_ms_p95": round(mm[int(0.95 * len(mm))], 2) if mm else 0.0,
               "matched": lm.n["matched"], "rejected": lm.n["rejected"], "jumps": lm.n["jumps"]}
        if not quiet:
            print(out, flush=True)
        return out
    finally:
        e.close()


KEYS = ("err_mean_mm", "err_max_mm", "err_end_mm", "head_mean_deg", "head_max_deg", "head_end_deg", "far_mm", "home_mm",
        "wall_fit", "occ_cells", "occ_mm", "scan_ms", "match_ms", "match_ms_p95")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drift", type=float, default=3.0, help="heading drift, deg/min")
    ap.add_argument("--scale", type=float, default=0.02, help="distance scale error")
    ap.add_argument("--miscal", action="store_true", help="miscalibrated wheels / track / gyro instead of --drift / --scale")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--loops", type=int, default=3)
    ap.add_argument("--every-n", type=int, default=None, help="match_every_n")
    ap.add_argument("--only-on", action="store_true")
    a = ap.parse_args(argv)
    st = {"match_every_n": a.every_n} if a.every_n else {}
    tot = {False: [], True: []}
    for seed in a.seeds:
        for on in ((True,) if a.only_on else (False, True)):
            o = run(seed, on, a.drift, a.scale, a.miscal, a.loops, settings=st)
            tot[on].append(o)
            print(o, flush=True)
    for on in (False, True):
        rows = tot[on]
        if rows:
            print("match %-5s " % on + "  ".join("%s %.3g" % (k, sum(r[k] for r in rows) / len(rows)) for k in KEYS))


if __name__ == "__main__":
    main()
