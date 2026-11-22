"""Stop-and-scan evaluation (issue #71): pose error, map sharpness and place return accuracy in the sim with
odometry drift, with and without the stop-and-scan pose correction. No network, sim time.

  python -m station.sim.scan_eval                 drift 3 deg/min + 2 % distance, seeds 1-3, explore + tour
  python -m station.sim.scan_eval --drift 6 --seeds 1 2 --loops 3

Per run: explore the room (planner, frontier goals; with stop_scan each goal pauses for a scan), then drive
the TOUR loop as planner gotos (map frame, a stop at every corner) --loops times, then go to the saved
places (Home = start, "Far" = (3500, 1400)) and measure where the robot really is.
Pose error = |map pose (corrected odometry) - true pose|, sampled every 0.5 s.
Map sharpness = share of occupied cells within 50 mm of a true wall (and how many occupied cells).
"""
import argparse
import math
import shutil
import tempfile

from station.bridge.mapping import Mapper
from .tour import TOUR, DriftRobot, plan_offline

FAR = (3500.0, 1400.0)


def wall_fit(m, room, occ_byte=150):
    g = m.grid
    d = [room.clearance(g.x0 + (i % g.W + 0.5) * g.res, g.y0 + (i // g.W + 0.5) * g.res)
         for i, v in enumerate(g.cells) if v > occ_byte]
    return (sum(1 for v in d if v < 50) / len(d) if d else 0.0), len(d)


def settle(r, m, watch, s=4.0):
    """Stand still a few seconds (where a stop-and-scan happens)."""
    from .tour import heartbeat
    from station.bridge import link_protocol as lp
    parser = lp.FrameParser()
    t_end = r.t + s
    while r.t < t_end:
        heartbeat(r)
        for _ in range(40):
            r.step()
        out, r.out = r.out, []
        for t, pl in parser.feed(b"".join(out)):
            if t == lp.MSG_RANGE:
                m.on_range(lp.decode_range(pl))
        watch(r, None)


def run(seed, drift, scale, stop_scan, loops=2, explore_s=500.0, quiet=True):
    tmp = tempfile.mkdtemp()
    try:
        r = DriftRobot(seed=seed, drift_deg_min=drift, drift_scale=scale)
        m = Mapper(tmp)
        m.settings["stop_scan"] = stop_scan
        m.places.add("Far", *FAR)
        errs, herrs = [], []
        last = [0.0]

        def watch(robot, b):
            if robot.t - last[0] >= 0.5:
                last[0] = robot.t
                x, y, h = m.map_pose(*robot.odom())
                errs.append(math.hypot(x - robot.X, y - robot.Y))
                herrs.append(abs((h - robot.TH + 180.0) % 360.0 - 180.0))
        _, b = plan_offline(r, m, {"t": "plan", "action": "explore"}, max_s=explore_s, on_step=watch)
        explore_err = errs[-1] if errs else 0.0
        explored = b.planner.status
        for _ in range(loops):
            for p in TOUR:            # one goto per corner (map frame): the robot stops at each
                plan_offline(r, m, lambda b, p=p: b.planner._begin_goto([tuple(p)], None, False),
                             max_s=60.0, on_step=watch)
                settle(r, m, watch)
        ret = {}
        for name in ("Far", "Home"):
            _, b2 = plan_offline(r, m, {"t": "plan", "action": "goto", "place": name}, max_s=120.0, on_step=watch)
            tx, ty = (0.0, 0.0) if name == "Home" else FAR
            ret[name] = math.hypot(r.X - tx, r.Y - ty)
        fit, n_occ = wall_fit(m, r.room)
        ss = m.stopscan
        h = list(ss.history)
        out = {"seed": seed, "stop_scan": stop_scan, "t_s": round(r.t), "explore": explored,
               "err_explore_mm": round(explore_err), "err_mean_mm": round(sum(errs) / len(errs)),
               "err_max_mm": round(max(errs)), "err_end_mm": round(errs[-1]),
               "head_mean_deg": round(sum(herrs) / len(herrs), 2), "head_end_deg": round(herrs[-1], 2),
               "far_mm": round(ret["Far"]), "home_mm": round(ret["Home"]),
               "wall_fit": round(fit, 3), "occ_cells": n_occ, "scans": ss.seq, "applied": ss.applied,
               "rejected": {}}
        for e in h:
            if not e["ok"]:
                k = e["why"].split(" (")[0]
                out["rejected"][k] = out["rejected"].get(k, 0) + 1
        if not quiet:
            print(out)
        return out
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drift", type=float, default=3.0, help="heading drift, deg/min")
    ap.add_argument("--scale", type=float, default=0.02, help="distance scale error")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--loops", type=int, default=2)
    a = ap.parse_args(argv)
    keys = ("err_explore_mm", "err_mean_mm", "err_max_mm", "err_end_mm", "head_mean_deg", "head_end_deg",
            "far_mm", "home_mm", "wall_fit", "occ_cells")
    tot = {False: [], True: []}
    for seed in a.seeds:
        for on in (False, True):
            o = run(seed, a.drift, a.scale, on, loops=a.loops)
            tot[on].append(o)
            print(o, flush=True)
    for on in (False, True):
        rows = tot[on]
        print("stop_scan %-5s " % on + "  ".join("%s %.3g" % (k, sum(r[k] for r in rows) / len(rows)) for k in keys))


if __name__ == "__main__":
    main()
