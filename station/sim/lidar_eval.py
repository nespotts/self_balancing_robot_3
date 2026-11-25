"""LD19 scan matching evaluation (issue #78): pose error vs the sim's ground truth, map sharpness and CPU, with the
matcher off and on, odometry drift / miscalibration, real LD19 packets (lidar_sim.py), sim time, no network.

  python -m station.sim.lidar_eval                      drift 3 deg/min + 2 % distance, seeds 1 2, 3 tour loops
  python -m station.sim.lidar_eval --miscal             wheels 69.3 mm (params 66), track 131 (136), right wheel +1.5 %,
                                                        gyro 0.05 deg/s
  python -m station.sim.lidar_eval --drift 6 --seeds 1 2 3 --loops 2
  python -m station.sim.lidar_eval --slip 0.5 2 5 --lever 170
                                                        wheel slip (#79): one tour loop with one spin-out of each
                                                        length on the first leg, #78 alone vs the #79 handling
                                                        (--ratio 0.5: half slip; --lidar-off: no LD19 data meanwhile)
  python -m station.sim.lidar_eval --reloc --seeds 1 2 3 --kidnaps 4
                                                        relocalisation (#105): after 3 tour legs the robot is carried
                                                        to random poses / headings, and a Teensy reboot (odometry back
                                                        to 0, 0, 0): found?, time to find, pose error after
  python -m station.sim.lidar_eval --retrace --seeds 1 2 3
                                                        retracing steps: lost in a sparse spot (lidar ~7 % of its
                                                        points, bumped 300 mm in a 1 s gap), planner retrace_lost off / on

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
    def __init__(self, seed, match, drift=3.0, scale=0.02, miscal=False, settings=None, lidar_settings=None,
                 sample_s=0.5, lever_mm=0.0):
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
        self.sim = LidarSim(seed=seed, lever_mm=lever_mm)
        self.stream, self.asm = ld19.Ld19Stream(), ld19.ScanAssembler()
        self.parser = lp.FrameParser()
        self.b = OfflineBridge(r, m)
        self.robot_logs = []
        stuck = lp.encode_event(lp.EVT_LINK_STUCK)[4:-2]
        m.lmatch.stuck_listeners.append(lambda: r.on_frame(lp.MSG_EVENT, stuck))   # as the bridge's link does (#79)
        self.errs, self.herrs = [], []
        self.err_t = []              # (sim time, pose error mm) per sample
        self.sample_s = sample_s
        self.F = None                # world -> map frame (fixed when the first turn is mapped)
        self.next_sample = 0.0
        self.scan_s, self.scans = 0.0, 0
        self.match_ms = []
        self.wide_ms = []            # matches while lost (the wide search, #79)
        self.on_step = None          # callable(run) after every step() (tests: start a slip)
        self.lidar_off = False       # no LD19 data (a dropout)

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
                        text = lp.decode_log(pl)
                        self.robot_logs.append((round(r.t, 2), text))
                        self.b.planner.on_log(text)
            data = self.sim.step(r, active=not self.lidar_off)
            if not data:
                continue
            for p in self.stream.feed(data, now=r.t):
                sc = self.asm.add(p)
                if sc:
                    lm = m.lmatch
                    k = lm.n["matched"] + lm.n["rejected"]
                    lost = lm.mode == "lost"
                    t0 = time.perf_counter()
                    lid.on_scan(sc)
                    self.scan_s += time.perf_counter() - t0
                    self.scans += 1
                    if lm.n["matched"] + lm.n["rejected"] != k:
                        (self.wide_ms if lost else self.match_ms).append(lm.cost_ms)
        if self.F is None and lid.n["used"]:
            self.F = frame((r.X, r.Y, r.TH), m.map_pose(*r.odom()))
        if r.t >= self.next_sample and lid.n["used"]:
            self.next_sample = r.t + self.sample_s
            e = self.pose_error()
            self.errs.append(e[0])
            self.herrs.append(e[1])
            self.err_t.append((r.t, e[0]))
        if self.on_step:
            self.on_step(self)

    def pose_error(self):
        """(map pose - true pose mm, heading error deg), in the map frame."""
        r, m = self.r, self.m
        x, y, h = m.map_pose(*r.odom())
        tx, ty = tf(self.F, r.X, r.Y)
        return math.hypot(x - tx, y - ty), abs((h - r.TH - self.F[2] + 180.0) % 360.0 - 180.0)

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


def run(seed, match, drift=3.0, scale=0.02, miscal=False, loops=3, settings=None, lidar_settings=None, quiet=True,
        lever_mm=0.0):
    e = Run(seed, match, drift, scale, miscal, settings, lidar_settings, lever_mm=lever_mm)
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
               "matched": lm.n["matched"], "rejected": lm.n["rejected"], "jumps": lm.n["jumps"],
               "slips": lm.n["slips"], "lost": lm.n["lost"], "stuck": lm.n["stuck"], "checks": lm.n["checks"]}
        if not quiet:
            print(out, flush=True)
        return out
    finally:
        e.close()


REC_MM, REC_HOLD_S = 50.0, 1.0       # recovered: the pose error under 50 mm and staying so for 1 s


def run_slip(seed, dur, ratio=1.0, recover=True, drift=3.0, scale=0.02, leg=0, after_mm=400.0, lever_mm=0.0,
             quiet=True, legs=None, lidar_off=False):
    """One tour loop with one wheel-slip episode (#79): dur s, ratio (1 = wheels spin, body still), starting once the
    robot has driven after_mm into TOUR leg `leg`. recover: the #79 slip / lost handling (False: #78 as merged).
    Pose error sampled every 0.1 s; recovery = from the end of the episode until the error is < REC_MM for REC_HOLD_S.
    legs: drive only the first n legs. lidar_off: no LD19 data during the episode either (nothing to see the slip with:
    the pose comes back out of the matcher's normal window, the #79 "lost" search)."""
    e = Run(seed, True, drift, scale, settings={"match_slip": recover}, sample_s=0.1, lever_mm=lever_mm)
    ep = {"leg": -1}

    def hook(run):
        r = run.r
        if "t0" in ep:
            run.lidar_off = lidar_off and r.t < ep["t1"]
            return
        if ep["leg"] != leg or run.F is None:
            return
        if math.hypot(r.X - ep["start"][0], r.Y - ep["start"][1]) >= after_mm and r.nav.state == lp.NAV_DRIVING:
            ep["t0"], ep["t1"] = r.t, r.t + dur
            ep["e0"] = run.pose_error()[0]
            r.slip_now(dur, ratio)
        run.lidar_off = lidar_off and "t0" in ep and r.t < ep["t1"]
    e.on_step = hook
    try:
        e.stand_up()
        for i, p in enumerate(TOUR[:legs]):
            ep["leg"], ep["start"] = i, (e.r.X, e.r.Y)
            e.drive(lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False), max_s=60.0)
            e.settle(0.5)
        e.settle(3.0)
        lm = e.m.lmatch
        t0, t1 = ep["t0"], ep["t1"]
        during = [v for t, v in e.err_t if t >= t0]
        at_end = min(e.err_t, key=lambda s: abs(s[0] - t1))[1]
        rec = None
        ok_since = None
        for t, v in e.err_t:
            if t < t1:
                continue
            if v < REC_MM:
                ok_since = t if ok_since is None else ok_since
                if t - ok_since >= REC_HOLD_S:
                    rec = max(0.0, ok_since - t1)
                    break
            else:
                ok_since = None
        over = sum(0.1 for t, v in e.err_t if t >= t0 and v >= REC_MM)      # time the error was >= 50 mm
        fit, n_occ, _ = wall_stats(e.m, e.r.room, e.F)
        wm = sorted(e.wide_ms)
        out = {"seed": seed, "dur_s": dur, "ratio": ratio, "recover": recover, "slip_mm": round(e.r.slip_mm),
               "t0": round(t0, 2),
               "err_before_mm": round(ep["e0"]), "err_max_mm": round(max(during)), "err_slip_end_mm": round(at_end),
               "recovery_s": None if rec is None else round(rec, 1), "err_end_mm": round(e.err_t[-1][1]),
               "wall_fit": round(fit, 3), "occ_cells": n_occ,
               "slips": lm.n["slips"], "lost": lm.n["lost"], "found": lm.n["found"], "stuck": lm.n["stuck"],
               "checks": lm.n["checks"],
               "robot_stuck": sum(1 for _, s in e.robot_logs if s.startswith("nav: stuck")),
               "wide_ms": round(sum(wm) / len(wm), 2) if wm else 0.0, "wide_ms_p95": round(wm[int(0.95 * len(wm))], 2) if wm else 0.0,
               "wide_n": len(wm), "t_s": round(e.r.t)}
        if not quiet:
            print(out, flush=True)
        return out
    finally:
        e.close()


def run_kidnap(seed, kind="kidnap", to=None, legs=3, wait_s=14.0, drift=3.0, scale=0.02, gap_s=3.0, quiet=True,
               settings=None):
    """Relocalisation (#105): drive the first `legs` TOUR legs (the map), then
      kind "kidnap": carry the robot to `to` = (x, y, deg) (world; odometry does not notice), or
      kind "reboot": the odometry resets to 0, 0, 0 where it stands, after gap_s s without LD19 data (the Teensy
                     forwards them: a reboot stops them too);
    then stand for wait_s. Found = the pose error < 50 mm (and heading < 2 deg) for 1 s; time from the event."""
    from station.sim import extras
    e = Run(seed, True, drift, scale, settings=settings, sample_s=0.1)
    try:
        e.stand_up()
        for p in TOUR[:legs]:
            e.drive(lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False), max_s=60.0)
            e.settle(0.5)
        r, lm = e.r, e.m.lmatch
        t_ev = r.t
        if kind == "kidnap":
            x, y, h = to or (-300.0, 1200.0, r.TH + 150.0)
            extras.teleport(r, x, y, h)
        else:
            e.lidar_off = True
            e.settle(gap_s)
            extras.teleport(r, r.X, r.Y, r.TH, reset_odom=True)
            e.lidar_off = False
        herr = []
        t_end = r.t + wait_s
        found = ok_since = None
        while r.t < t_end:
            e.settle(0.1)
            pe, he = e.pose_error()
            herr.append(he)
            if pe < 50.0 and he < 2.0:
                ok_since = r.t if ok_since is None else ok_since
                if found is None and r.t - ok_since >= 1.0:
                    found = ok_since - t_ev
            else:
                ok_since = None
        pe, he = e.pose_error()
        g = lm.last_global or {}
        out = {"seed": seed, "kind": kind, "found_s": None if found is None else round(found, 1),
               "err_end_mm": round(pe), "head_end_deg": round(he, 2), "mode": lm.mode,
               "global": lm.n["global"], "relocated": lm.n["relocated"], "lost": lm.n["lost"],
               "search_ms": g.get("ms"), "nodes": g.get("nodes"), "fit": g.get("score"), "margin": g.get("margin"),
               "phase": (lm._reloc_stats(lm._now()) or {}).get("phase"), "t_s": round(r.t),
               "wall_fit": round(wall_stats(e.m, r.room, e.F)[0], 3)}
        if not quiet:
            print(out, flush=True)
        return out
    finally:
        e.close()


SPARSE_X, SPARSE_DROPOUT = 2700.0, 0.93     # run_retrace: west of this the LD19 gets ~7 % of its points


def run_retrace(seed, retrace=True, goal=(500.0, 300.0), shift_mm=300.0, legs=3, max_s=150.0, drift=3.0, scale=0.02,
                quiet=True):
    """Retracing steps (retrace.py): drive the first `legs` TOUR legs (the map), then a planner goto to `goal` into a
    sparse spot: west of SPARSE_X the lidar returns only ~7 % of its points (dark / glossy furniture; turns with too
    few points are skipped, the rest cannot be matched). 250 mm into it the robot is bumped shift_mm sideways during a
    1 s lidar gap (the odometry does not notice): the gap check fails, the pose is lost where nothing can find it.
    retrace: the planner setting retrace_lost. Returns the first lost episode's timeline and the end state."""
    from station.sim import extras
    e = Run(seed, True, drift, scale, sample_s=0.25)
    ev = {"modes": []}

    def hook(run):
        r = run.r
        run.sim.dropout = SPARSE_DROPOUT if ("armed" in ev and r.X < SPARSE_X) else 0.01
        if "armed" in ev and "kid" not in ev and r.X < SPARSE_X - 250.0:
            ev["kid"], ev["kid_x"] = r.t, r.X
            extras.teleport(r, r.X, r.Y - shift_mm, r.TH)
        if "kid" in ev:
            run.lidar_off = r.t < ev["kid"] + 1.0
    e.on_step = hook
    try:
        e.stand_up()
        for p in TOUR[:legs]:
            e.drive(lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False), max_s=60.0)
            e.settle(0.5)
        b, r, m = e.b, e.r, e.m
        pl, lm, rt = b.planner, m.lmatch, b.planner.retrace
        pl.settings["retrace_lost"] = retrace
        ev["armed"] = True
        pl._begin_goto([goal], None, False)
        out = {"seed": seed, "retrace": retrace, "lost_s": None, "retrace_s": None, "found_s": None, "back_mm": 0.0,
               "x_back": None, "trail": 0, "err_found_mm": None, "statuses": []}
        start = r.t
        while r.t - start < max_s:
            heartbeat(r)
            for _ in range(10):
                e.step()
            m.poll(r.t)
            b.update()
            pl.tick(r.t)
            if pl.status not in out["statuses"]:
                out["statuses"].append(pl.status)
            k = ev.get("kid")
            if k is not None and out["lost_s"] is None and lm.mode == "lost":
                out["lost_s"] = round(r.t - k, 1)
            if rt.active() and out["found_s"] is None:
                if out["retrace_s"] is None:
                    out["retrace_s"] = round(r.t - k, 1)
                out["x_back"] = max(out["x_back"] or -1e9, r.X)
                out["trail"] = max(out["trail"], len((rt.msg() or {}).get("trail") or ()))
                out["back_mm"] = rt.st["driven"]
            if out["retrace_s"] is not None and out["found_s"] is None and lm.mode == "track":
                out["found_s"] = round(r.t - k, 1)
                out["err_found_mm"] = round(e.pose_error()[0])
            if pl.mode == "idle":
                break
        gx, gy = goal
        x, y, _ = m.map_pose(*r.odom())
        out.update(status=pl.status, mode=lm.mode, err_end_mm=round(e.pose_error()[0]),
                   goal_mm=round(math.hypot(x - gx, y - gy)), back_mm=round(out["back_mm"]), n=dict(rt.n),
                   kid_x=round(ev.get("kid_x", 0.0)), t_s=round(r.t - start, 1),
                   wall_fit=round(wall_stats(m, r.room, e.F)[0], 3), relocated=lm.n["relocated"], lines=b.lines[-12:])
        if not quiet:
            print(out, flush=True)
        return out
    finally:
        e.close()


KEYS = ("err_mean_mm", "err_max_mm", "err_end_mm", "head_mean_deg", "head_max_deg", "head_end_deg", "far_mm", "home_mm",
        "wall_fit", "occ_cells", "occ_mm", "scan_ms", "match_ms", "match_ms_p95", "slips", "lost", "stuck")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drift", type=float, default=3.0, help="heading drift, deg/min")
    ap.add_argument("--scale", type=float, default=0.02, help="distance scale error")
    ap.add_argument("--miscal", action="store_true", help="miscalibrated wheels / track / gyro instead of --drift / --scale")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--loops", type=int, default=3)
    ap.add_argument("--every-n", type=int, default=None, help="match_every_n")
    ap.add_argument("--only-on", action="store_true")
    ap.add_argument("--slip", type=float, nargs="+", default=None, metavar="SECONDS",
                    help="wheel-slip runs (#79) instead: one tour loop with one episode of each length, old (#78) "
                         "vs new handling")
    ap.add_argument("--ratio", type=float, default=1.0, help="--slip: 1 = wheels spin, body still; 0.5 = half")
    ap.add_argument("--lever", type=float, default=0.0, help="lidar this far above the axle (lean moves it)")
    ap.add_argument("--lidar-off", action="store_true", help="--slip: no LD19 data during the episode (lost + wide search)")
    ap.add_argument("--reloc", action="store_true", help="relocalisation runs (#105): kidnaps + a reboot per seed")
    ap.add_argument("--kidnaps", type=int, default=3, help="--reloc: random kidnaps per seed")
    ap.add_argument("--retrace", action="store_true", help="retracing steps runs: lost in a sparse spot, with and "
                                                               "without the planner's retrace_lost")
    a = ap.parse_args(argv)
    if a.retrace:
        for seed in a.seeds:
            for on in (False, True):
                o = run_retrace(seed, on)
                o.pop("lines")
                print(o, flush=True)
        return
    if a.reloc:
        import random
        from .reloc_eval import free_pose
        from .sim import Room
        rows = []
        for seed in a.seeds:
            rng = random.Random(seed)
            for _ in range(a.kidnaps):
                rows.append(run_kidnap(seed, "kidnap", free_pose(Room(), (-1000, -1500, 4000, 2000), rng, 300.0)))
                print(rows[-1], flush=True)
            rows.append(run_kidnap(seed, "reboot"))
            print(rows[-1], flush=True)
        for kind in ("kidnap", "reboot"):
            rs = [o for o in rows if o["kind"] == kind]
            ok = sorted(o["found_s"] for o in rs if o["found_s"] is not None)
            print("%s: found %d of %d, time to find median %s s, max %s s; wrong poses accepted: %d" % (
                kind, len(ok), len(rs), ok[len(ok) // 2] if ok else "-", ok[-1] if ok else "-",
                sum(1 for o in rs if o["mode"] == "track" and o["err_end_mm"] > 100)))
        return
    if a.slip:
        rows = []
        for dur in a.slip:
            for seed in a.seeds:
                for rec in (False, True):
                    o = run_slip(seed, dur, a.ratio, rec, a.drift, a.scale, lever_mm=a.lever, lidar_off=a.lidar_off)
                    rows.append(o)
                    print(o, flush=True)
        print("dur_s ratio  handling  err_before  err_max  err_at_slip_end  recovery_s  err_end  wall_fit  slips lost stuck")
        for o in rows:
            print("%5.1f %5.2f  %-8s  %10d  %7d  %15d  %10s  %7d  %8.3f  %5d %4d %5d" % (
                o["dur_s"], o["ratio"], "new" if o["recover"] else "old", o["err_before_mm"], o["err_max_mm"],
                o["err_slip_end_mm"], "never" if o["recovery_s"] is None else o["recovery_s"], o["err_end_mm"],
                o["wall_fit"], o["slips"], o["lost"], o["stuck"]))
        return
    st = {"match_every_n": a.every_n} if a.every_n else {}
    tot = {False: [], True: []}
    for seed in a.seeds:
        for on in ((True,) if a.only_on else (False, True)):
            o = run(seed, on, a.drift, a.scale, a.miscal, a.loops, settings=st, lever_mm=a.lever)
            tot[on].append(o)
            print(o, flush=True)
    for on in (False, True):
        rows = tot[on]
        if rows:
            print("match %-5s " % on + "  ".join("%s %.3g" % (k, sum(r[k] for r in rows) / len(rows)) for k in KEYS))


if __name__ == "__main__":
    main()
