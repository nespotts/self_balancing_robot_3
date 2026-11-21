#!/usr/bin/env python3
"""Fit the motor correction from a wheels-up motor test (mot.test, issue #16). Stdlib only.

  python3 tools/motor_fit.py                     newest ~/sbr3-logs/*.jsonl (run on the Pi)
  python3 tools/motor_fit.py --ssh robot         same, fetched from the Pi (run on the PC)
  python3 tools/motor_fit.py FILE [FILE ...]     session .jsonl, or any text with "mt,..." lines
  options: --vref 11.5 (model-1 reference voltage)  --vbatt 12.0 (use if the battery reading was bad)
           --config all | latest | DRIVE:HZ   (default all: every drive config separately + a comparison)

Reads the "mt,..." MSG_LOG lines (format in docs/firmware.md). Runs are grouped by drive config
(mot.drive_mode, mot.pwm_hz from the begin line, issue #22; older logs = 0:36621); per config it
uses the newest completed run of each test (a thresholds, b speed curve, c reversal), and prints:
  dead zone start/stop per wheel + direction (duty and volts), speed vs volts with a linear fit,
  left/right mismatch, reversal lag, motor time constant, v3 curve vs volts model, and the params
  to try.  Everything is in volts (duty / 4095 * pack voltage at that moment), so a fit made at
  one charge level holds at another (with mot.vcomp = 1).
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys

WHEELS = ("left", "right")
DIRS = ("fwd", "rev")
MOVE_MMS = 20.0
V_OK = (6.0, 17.0)


# ------------------------------------------------------------------ input

def mt_lines_from_text(lines):
    """Yield (t, text) for every mt line and ("param", name, value) tuples."""
    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        if ln.startswith("["):
            try:
                row = json.loads(ln)
            except ValueError:
                continue
            if row and row[0] == "log" and len(row) >= 3 and str(row[2]).startswith("mt,"):
                yield ("log", row[1], row[2])
            elif row and row[0] == "param" and len(row) >= 5:
                yield ("param", row[3], row[4])
            continue
        i = ln.find("mt,")
        if i >= 0:
            yield ("log", None, ln[i:])


def read_local(paths):
    if not paths:
        files = sorted(glob.glob(os.path.expanduser("~/sbr3-logs/*.jsonl")), key=os.path.getmtime)
        if not files:
            sys.exit("no session logs in ~/sbr3-logs (give a file, or --ssh robot)")
        paths = [files[-1]]
    out = []
    for p in paths:
        with open(p, encoding="utf-8", errors="ignore") as f:
            out += list(mt_lines_from_text(l for l in f if '"mt,' in l or "mt," in l or l.startswith('["param"')))
    return out, paths


def read_ssh(host, name=None):
    remote = name or "$(ls -t ~/sbr3-logs/*.jsonl | head -1)"
    cmd = "f=%s; echo \"$f\"; grep -h -e '\"mt,' -e '^\\[\"param\"' \"$f\"" % remote
    r = subprocess.run(["ssh", host, cmd], capture_output=True, text=True)
    if r.returncode not in (0, 1):
        sys.exit("ssh failed: " + r.stderr.strip())
    lines = r.stdout.splitlines()
    return list(mt_lines_from_text(lines[1:])), [host + ":" + (lines[0] if lines else "?")]


DEFAULT_HZ = 36621.0  # firmware before issue #22: EN PWM at 36.6 kHz, no hz=/drive= in the lines


def parse_cfg(begin_line):
    """(drive_mode, pwm_hz) from "mt,begin,...,hz=<Hz>,drive=<0|1>" (older firmware: (0, 36621))."""
    m_hz = re.search(r"\bhz=([0-9.]+)", begin_line)
    m_dr = re.search(r"\bdrive=([0-9]+)", begin_line)
    return (int(m_dr.group(1)) if m_dr else 0, round(float(m_hz.group(1))) if m_hz else DEFAULT_HZ)


def cfg_name(cfg):
    return "%d:%.0f" % cfg


def cfg_label(cfg):
    d, hz = cfg
    how = "PWM on EN, coast in the off-time" if d == 0 else "PWM on IN, brake in the off-time"
    return "mot.drive_mode %d (%s), mot.pwm_hz %.0f" % (d, how, hz)


def split_runs(items):
    """Group mt lines into runs (begin .. end/abort). Returns runs and the last param values."""
    runs, cur, params = [], None, {}
    for it in items:
        if it[0] == "param":
            params[it[1]] = it[2]
            continue
        f = it[2].split(",")
        kind = f[1] if len(f) > 1 else ""
        if kind == "begin":
            cur = {"test": f[2], "status": "running", "a": [], "b": [], "c": [], "sum": [], "begin": it[2],
                   "cfg": parse_cfg(it[2])}
            runs.append(cur)
        elif cur is None:
            continue
        elif kind in ("a", "b", "c"):
            cur[kind].append(f)
        elif kind == "sum":
            cur["sum"].append(it[2])
        elif kind == "end":
            cur["status"] = "ok"
            cur = None
        elif kind == "abort":
            cur["status"] = "aborted: " + ",".join(f[2:])
            cur = None
    return runs, params


def num(s):
    try:
        v = float(s)
    except ValueError:
        return None
    return None if v != v else v


def pick(runs, letter):
    """Newest run with results for this test; a completed one beats a newer aborted one."""
    have = [r for r in runs if r[letter]]
    ok = [r for r in have if r["status"] == "ok"]
    return (ok or have or [None])[-1]


# ------------------------------------------------------------------ maths

def linfit(xs, ys):
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    c = my - k * mx
    res = math.sqrt(sum((y - (k * x + c)) ** 2 for x, y in zip(xs, ys)) / n)
    return k, c, res


def interp(pts, x):
    """Piecewise-linear y(x) through sorted (x, y) points, flat outside."""
    if not pts:
        return 0.0
    if x <= pts[0][0]:
        return pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return pts[-1][1]


def v3_curve(cnt, exp=3.92, knee_in=1250.0, knee_out=2750.0):
    cnt = min(abs(cnt), 4095.0)
    if cnt <= 0:
        return 0.0
    if cnt <= knee_in:
        return knee_out * (cnt / knee_in) ** (1.0 / exp)
    return knee_out + (4095.0 - knee_out) / (4095.0 - knee_in) * (cnt - knee_in)


def dz_volts(u, dz, vref, band):
    u = min(abs(u), 1.0)
    if u <= 0:
        return 0.0
    dz = min(dz, vref)
    if u < band:
        return u / band * (dz + (vref - dz) * band)
    return dz + (vref - dz) * u


def median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    m = len(xs) // 2
    return xs[m] if len(xs) % 2 else 0.5 * (xs[m - 1] + xs[m])


def fmt(v, f="%.2f"):
    return "-" if v is None else f % v


# ------------------------------------------------------------------ main

def analyze(runs, params, a, cfg):
    """Print the analysis of the runs of one drive config; return a summary for the comparison."""
    S = {"cfg": cfg}
    warn = []

    def vb_of(s):
        v = num(s)
        if v is not None and V_OK[0] <= v <= V_OK[1]:
            return v
        if a.vbatt:
            return a.vbatt
        warn.append("battery reading %s V invalid; volts unknown (use --vbatt)" % s)
        return None

    def volts(duty, vb_s):
        vb = vb_of(vb_s)
        return None if vb is None or duty is None or duty < 0 else duty / 4095.0 * vb

    # ---------------- test a
    A = {}
    ra = pick(runs, "a")
    if ra:
        for f in ra["a"]:
            # mt,a,wheel,dir,start|stop,duty,volts,vbatt,speed
            w, d, kind, duty = f[2], f[3], f[4], num(f[5])
            A.setdefault((w, d), {})[kind] = (duty if duty is not None and duty >= 0 else None, volts(duty, f[7]))
            A[(w, d)].setdefault("vb", vb_of(f[7]))
        print("\n== a) dead zone (duty of 4095 / volts)            run status: %s" % ra["status"])
        print("%-6s %-4s %12s %12s %8s" % ("wheel", "dir", "start", "stop", "vbatt"))
        for w in WHEELS:
            for d in DIRS:
                e = A.get((w, d), {})
                s, t = e.get("start", (None, None)), e.get("stop", (None, None))
                print("%-6s %-4s %5s %5sV %5s %5sV %7sV" % (w, d, fmt(s[0], "%.0f"), fmt(s[1]), fmt(t[0], "%.0f"),
                                                       fmt(t[1]), fmt(e.get("vb"))))
                S.setdefault("start_v", []).append(s[1])
                S.setdefault("stop_v", []).append(t[1])

    # ---------------- test b
    B, fits, taus = {}, {}, {}
    rb = pick(runs, "b")
    if rb:
        for f in rb["b"]:
            # mt,b,wheel,dir,duty,volts,vbatt,speed,t63
            w, d, duty, sp, t63 = f[2], f[3], num(f[4]), num(f[7]), num(f[8])
            v = volts(duty, f[6])
            B.setdefault((w, d), []).append((v, sp, duty))
            if t63 is not None and t63 >= 0:
                taus.setdefault(w, []).append(t63)
        print("\n== b) steady speed (mm/s) vs motor volts          run status: %s" % rb["status"])
        keys = [(w, d) for w in WHEELS for d in DIRS]
        print("%6s %6s " % ("duty", "volts") + " ".join("%9s" % ("%s_%s" % (w[0], d)) for w, d in keys))
        rows = max(len(B.get(k, [])) for k in keys)
        for i in range(rows):
            first = next((B[k][i] for k in keys if k in B and i < len(B[k])), None)
            line = "%6s %6s " % (fmt(first[2], "%.0f"), fmt(first[0])) if first else ""
            line += " ".join("%9s" % (fmt(B[k][i][1], "%.0f") if k in B and i < len(B[k]) else "-") for k in keys)
            print(line)
        signs = []
        for (w, d), pts in B.items():
            pts = [(v, abs(s)) for v, s, _ in pts if v is not None and s is not None]
            moving = [(v, s) for v, s in pts if s > 3 * MOVE_MMS]
            want = 1 if d == "fwd" else -1
            top = max(B[(w, d)], key=lambda p: p[2] or 0)
            if top[1] is not None and abs(top[1]) > MOVE_MMS and (top[1] > 0) != (want > 0):
                signs.append("%s %s" % (w, d))
            fit = linfit([v for v, _ in moving], [s for _, s in moving])
            if fit and fit[0] > 0:
                k, c, res = fit
                fits[(w, d)] = {"k": k, "v0": -c / k, "res": res, "top": max(s for _, s in pts),
                                "pts": sorted([(0.0, 0.0)] + pts)}
        if signs:
            warn.append("encoder sign opposite to the command on: %s (check ENC direction / motor wiring)" % ", ".join(signs))
        print("\nlinear fit above the dead zone: speed = k * (V - V0)")
        print("%-6s %-4s %10s %7s %10s %9s" % ("wheel", "dir", "k mm/s/V", "V0", "resid", "top mm/s"))
        for w in WHEELS:
            for d in DIRS:
                f = fits.get((w, d))
                if f:
                    print("%-6s %-4s %10.1f %6.2fV %8.1f%% %9.0f" % (w, d, f["k"], f["v0"], 100 * f["res"] / max(1, f["top"]), f["top"]))
                    S.setdefault("v0", []).append(f["v0"])
                    S.setdefault("k", []).append(f["k"])
                    S.setdefault("resid", []).append(100 * f["res"] / max(1, f["top"]))
                    S.setdefault("top", []).append(f["top"])
        for w in WHEELS:
            if taus.get(w):
                print("time constant %s: median %.0f ms (63%% step response, 80 ms speed window removed)" % (w, median(taus[w])))
                S.setdefault("tau", {})[w] = median(taus[w])
        kl = [fits[("left", d)]["k"] for d in DIRS if ("left", d) in fits]
        kr = [fits[("right", d)]["k"] for d in DIRS if ("right", d) in fits]
        if kl and kr:
            print("left/right speed gain: left %.1f, right %.1f mm/s/V -> left is %+.1f%%" %
                  (sum(kl) / len(kl), sum(kr) / len(kr), 100 * (sum(kl) / len(kl) / (sum(kr) / len(kr)) - 1)))
        for w in WHEELS:
            if (w, "fwd") in fits and (w, "rev") in fits:
                kf, kr_ = fits[(w, "fwd")]["k"], fits[(w, "rev")]["k"]
                print("%s fwd/rev gain: %+.1f%%   V0 fwd %.2f / rev %.2f V" % (w, 100 * (kf / kr_ - 1), fits[(w, "fwd")]["v0"], fits[(w, "rev")]["v0"]))

    # ---------------- test c
    rc = pick(runs, "c")
    if rc:
        C = {}
        for f in rc["c"]:
            # mt,c,wheel,newdir,rep,duty,volts,vbatt,t_stop,t_rev,coast_deg
            w, d = f[2], f[3]
            ts, tr, co = num(f[8]), num(f[9]), num(f[10])
            if tr is None or tr < 0:
                C.setdefault((w, d), []).append(None)
            else:
                C.setdefault((w, d), []).append((ts, tr, co))
        f0 = rc["c"][0] if rc["c"] else None
        print("\n== c) reversal at %s duty (%s V)                  run status: %s" %
              (f0[5] if f0 else "?", fmt(volts(num(f0[5]), f0[7])) if f0 else "?", rc["status"]))
        print("%-6s %-8s %4s %9s %9s %9s %10s" % ("wheel", "to dir", "n", "stop ms", "rev ms", "dwell ms", "coast deg"))
        for w in WHEELS:
            for d in DIRS:
                xs = C.get((w, d), [])
                ok = [x for x in xs if x]
                if not xs:
                    continue
                ms = lambda i: sum(x[i] for x in ok) / len(ok) if ok else None
                dwell = (ms(1) - ms(0)) if ok else None
                S.setdefault("dwell", []).append(dwell)
                print("%-6s %-8s %4s %9s %9s %9s %10s%s" % (w, d, "%d/%d" % (len(ok), len(xs)), fmt(ms(0), "%.0f"), fmt(ms(1), "%.0f"),
                                                          fmt(dwell, "%.0f"), fmt(ms(2), "%.1f"), "" if len(ok) == len(xs) else "  (some never reversed in 2 s)"))
        print("stop = command -> wheel at its turning point; rev = until 2 deg back; dwell = rev - stop")
        print("(the encoder is on the gearbox output: free play shows up as dwell, not as an angle)")

    # ---------------- proposal
    if not fits and not A:
        for w_ in warn:
            print("WARNING: " + w_)
        return S
    vref = a.vref or params.get("mot.v_ref") or 11.5
    band = a.band if a.band is not None else params.get("mot.dz_band", 0.01)
    print("\n== proposal (mot.model 1 = volts model, mot.vcomp 1, mot.v_ref %.1f V)" % vref)
    dz = {}
    for w in WHEELS:
        for d in DIRS:
            e = A.get((w, d), {})
            st, sp = e.get("start", (None, None))[1], e.get("stop", (None, None))[1]
            f = fits.get((w, d))
            if f:
                v = f["v0"]
                lo, hi = (sp if sp is not None else -1e9), (st if st is not None else 1e9)
                v = min(max(v, lo), hi) if lo <= hi else v
                how = "fit V0 %.2f" % f["v0"] + (" clamped to stop..start" if abs(v - f["v0"]) > 1e-6 else "")
            elif st is not None and sp is not None:
                v, how = 0.5 * (st + sp), "mid of start/stop"
            elif st is not None:
                v, how = st, "start"
            else:
                continue
            dz[(w, d)] = max(0.0, v)
            print("mot.dz_%s_%s = %.2f V   (%s; start %s, stop %s)" % (w[0], d, dz[(w, d)], how, fmt(st), fmt(sp)))
    # per-wheel scale so u gives the same speed on both wheels (stronger wheel turned down)
    K = {}
    for w in WHEELS:
        ks = [fits[(w, d)]["k"] * (vref - dz.get((w, d), fits[(w, d)]["v0"])) for d in DIRS if (w, d) in fits]
        if ks:
            K[w] = sum(ks) / len(ks)
    scales = {}
    if len(K) == 2:
        lo = min(K.values())
        scales = {w: lo / K[w] for w in WHEELS}
        print("mot.left_scale = %.3f, mot.right_scale = %.3f   (speed at u=1: left %.0f, right %.0f mm/s at %.1f V)" %
              (scales["left"], scales["right"], K["left"], K["right"], vref))

    # v3 curve vs volts model, speed for a command u (both wheels averaged, fwd)
    vb_vals = [p[0] / (p[2] / 4095.0) for pts in B.values() for p in pts if p[0] and p[2]]
    vb_mean = sum(vb_vals) / len(vb_vals) if vb_vals else (a.vbatt or 11.5)
    cur = {"exp": params.get("mot.curve_exp", 3.92), "ki": params.get("mot.knee_in", 1250.0), "ko": params.get("mot.knee_out", 2750.0),
           "ls": params.get("mot.left_scale", 0.95), "rs": params.get("mot.right_scale", 1.0)}
    def wheel_volts(model, w, u):
        f = fits[(w, "fwd")]
        if model == 0:
            sc = cur["ls"] if w == "left" else cur["rs"]
            return v3_curve(u * sc * 4095, cur["exp"], cur["ki"], cur["ko"]) / 4095.0 * vb_mean
        return min(dz_volts(u * scales.get(w, 1.0), dz.get((w, "fwd"), f["v0"]), vref, band), vb_mean)

    def mean_speed(model, u):
        ws = [w for w in WHEELS if (w, "fwd") in fits]
        return sum(interp(fits[(w, "fwd")]["pts"], wheel_volts(model, w, u)) for w in ws) / len(ws)

    gain_ratio = None
    if any((w, "fwd") in fits for w in WHEELS):
        ws = [w for w in WHEELS if (w, "fwd") in fits]
        print("\nspeed (mm/s, fwd) for a controller output u (balancing uses |u| ~ 0.05-0.2):")
        print("%6s | %15s %15s | %15s %15s" % ("u", "v3 volts", "v3 L / R", "model1 volts", "model1 L / R"))
        for u in (0.005, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0):
            row = []
            for model in (0, 1):
                vs = [wheel_volts(model, w, u) for w in ws]
                ss = [interp(fits[(w, "fwd")]["pts"], v) for w, v in zip(ws, vs)]
                row.append("%15s %15s" % ("/".join("%.2f" % v for v in vs), "/".join("%.0f" % s for s in ss)))
            print("%6.3f | %s | %s" % (u, row[0], row[1]))
        print("(v3 at the test's mean pack voltage %.1f V; model 1 with vcomp = same at any charge)" % vb_mean)
        # linearity: rms deviation from a straight line through 0, over u 0.02..0.6
        us = [i / 100.0 for i in range(2, 61)]
        slopes = []
        for model in (0, 1):
            sp = [mean_speed(model, u) for u in us]
            g = sum(u * s for u, s in zip(us, sp)) / sum(u * u for u in us)
            dev = math.sqrt(sum((s - g * u) ** 2 for u, s in zip(us, sp)) / len(us))
            at0 = mean_speed(model, 0.01)
            slopes.append((mean_speed(model, 0.2) - mean_speed(model, 0.05)) / 0.15)
            print("%s: rms deviation from a straight line %.0f mm/s (%.0f%% of top), speed already at u=0.01: %.0f mm/s, "
                  "slope over u 0.05..0.2: %.0f mm/s per unit u" %
                  ("v3 curve   " if model == 0 else "volts model", dev, 100 * dev / max(1, max(sp)), at0, slopes[-1]))
        if slopes[1] > 0:
            gain_ratio = slopes[0] / slopes[1]
            print("-> in the balancing range the volts model gives %.2fx the motor gain of the v3 curve; to start from the"
                  " same loop gain multiply bal.kp / bal.ki / bal.kd by ~%.2f (then retune)" % (1 / gain_ratio, gain_ratio))
    print("\ntry it (on the Pi in ~/self_balancing_robot_3, robot balancing; mot.model 0 switches back;")
    print("  mot.drive_mode / mot.pwm_hz apply when the motors are off: disarm, set, re-arm):")
    cmd = ["mot.drive_mode=%d" % cfg[0], "mot.pwm_hz=%.0f" % cfg[1]]
    cmd += ["mot.model=1", "mot.vcomp=1", "mot.v_ref=%.1f" % vref]
    cmd += ["mot.dz_%s_%s=%.2f" % (w[0], d, v) for (w, d), v in sorted(dz.items())]
    if scales:
        cmd += ["mot.left_scale=%.3f" % scales["left"], "mot.right_scale=%.3f" % scales["right"]]
    print("  python3 tools/tune.py set " + " ".join(cmd))
    print("  (back: mot.model=0 mot.left_scale=%g mot.right_scale=%g; Save to robot / cfg.save keeps it)" % (cur["ls"], cur["rs"]))
    for w_ in sorted(set(warn)):
        print("WARNING: " + w_)
    return S


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="*")
    ap.add_argument("--ssh", metavar="HOST", help="read the newest session log on HOST (e.g. robot)")
    ap.add_argument("--vref", type=float, default=None, help="model-1 reference voltage (default mot.v_ref or 11.5)")
    ap.add_argument("--vbatt", type=float, default=None, help="pack voltage to use when the logged one is invalid")
    ap.add_argument("--band", type=float, default=None, help="mot.dz_band for the comparison (default from log or 0.01)")
    ap.add_argument("--config", default="all", metavar="all|latest|DRIVE:HZ",
                    help="drive config(s) to fit: all (default, each separately + a comparison), latest, "
                         "or e.g. 1:20000 = mot.drive_mode 1 at mot.pwm_hz 20000")
    a = ap.parse_args()

    if a.ssh:
        items, src = read_ssh(a.ssh, a.files[0] if a.files else None)
    else:
        items, src = read_local(a.files)
    runs, params = split_runs(items)
    print("source: %s" % ", ".join(src))
    if not runs:
        sys.exit("no motor test runs (mt,begin ...) found")
    for r in runs:
        print("  run test %s: %-8s %s  (%d a / %d b / %d c lines)" % (r["test"], cfg_name(r["cfg"]), r["status"],
                                                                    len(r["a"]), len(r["b"]), len(r["c"])))

    configs = []  # in order of their newest run
    for r in runs:
        if r["cfg"] in configs:
            configs.remove(r["cfg"])
        configs.append(r["cfg"])
    if a.config == "all":
        sel = configs
    elif a.config == "latest":
        sel = configs[-1:]
    else:
        try:
            d, hz = a.config.split(":")
            want = (int(d), float(hz))
        except ValueError:
            sys.exit("--config: all, latest or DRIVE:HZ (e.g. 1:20000)")
        sel = [c for c in configs if c[0] == want[0] and abs(c[1] - want[1]) <= 0.01 * want[1]]
        if not sel:
            sys.exit("no runs with %s (have: %s)" % (cfg_name(want), ", ".join(cfg_name(c) for c in configs)))
    summaries = []
    for cfg in sel:
        print("\n" + "=" * 100)
        print("=== %s   (newest run of each test with this config)" % cfg_label(cfg))
        summaries.append(analyze([r for r in runs if r["cfg"] == cfg], params, a, cfg))
    if len(summaries) > 1:
        compare(summaries)


def compare(summaries):
    """One row per drive config: dead zone, linearity, time constant, reversal."""
    print("\n" + "=" * 100)
    print("=== comparison (means over wheels and directions; volts = duty / 4095 * pack V)")
    print("%-9s %8s %8s %8s %10s %8s %9s %9s %9s" % ("config", "start V", "stop V", "fit V0", "k mm/s/V",
                                                     "resid", "tau L/R", "top mm/s", "dwell ms"))
    for S in summaries:
        def m(key):
            xs = [x for x in S.get(key, []) if x is not None]
            return sum(xs) / len(xs) if xs else None
        tau = "/".join(fmt(S.get("tau", {}).get(w), "%.0f") for w in WHEELS)
        print("%-9s %8s %8s %8s %10s %7s%% %9s %9s %9s" % (
            cfg_name(S["cfg"]), fmt(m("start_v")), fmt(m("stop_v")), fmt(m("v0")), fmt(m("k"), "%.1f"),
            fmt(m("resid"), "%.1f"), tau, fmt(m("top"), "%.0f"), fmt(m("dwell"), "%.0f")))
    print("lower start/stop V and V0 = smaller dead zone; lower resid = straighter speed vs volts above it;")
    print("drive 0 = PWM on EN (coast in the off-time), drive 1 = PWM on IN (brake in the off-time)")


if __name__ == "__main__":
    main()
