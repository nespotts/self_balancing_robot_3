"""Wheels-up motor test (mot.test, issue #16): parse the "mt,..." log lines and fit the motor correction.
Stdlib only; used by tools/motor_fit.py (text report) and the bridge (Tune page "Motor test" card).

  items = list(mt_items(lines))          # session .jsonl lines or any text with "mt,..." lines
  runs, params = split_runs(items)
  for cfg in configs(runs):
      R = analyze([r for r in runs if r["cfg"] == cfg], params, cfg)   # JSON-ready dict
      print("\n".join(report(R)))

Line formats: docs/firmware.md. Everything is in volts (duty / 4095 * pack voltage at that moment),
so a fit made at one charge level holds at another (with mot.vcomp = 1).
"""
import json
import math
import re

WHEELS = ("left", "right")
DIRS = ("fwd", "rev")
KEYS = tuple("%s_%s" % (w, d) for w in WHEELS for d in DIRS)
MOVE_MMS = 20.0
V_OK = (6.0, 17.0)
DEFAULT_HZ = 36621.0  # firmware before issue #22: EN PWM at 36.6 kHz, no hz=/drive= in the lines
EXPECTED = {"a": 8, "b": 64, "c": 12}   # result lines per part (progress)
PARTS = {1: "a", 2: "b", 3: "c", 4: "abc"}
DURATION_S = {1: 45, 2: 42, 3: 14, 4: 80}  # rough, for a progress estimate


# ------------------------------------------------------------------ input
def mt_items(lines):
    """Yield ("log", t, text) for every mt line and ("param", name, value) for param records."""
    for ln in lines:
        if isinstance(ln, bytes):
            ln = ln.decode("utf-8", "ignore")
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


def read_file(path):
    """mt items of one session log (fast: only lines that can matter are parsed)."""
    with open(path, "rb") as f:
        return list(mt_items(l for l in f if b'"mt,' in l or l.startswith(b'["param"') or
                             (not l.startswith(b"[") and b"mt," in l)))


def parse_cfg(begin_line):
    """(drive_mode, pwm_hz) from "mt,begin,...,hz=<Hz>,drive=<0|1>" (older firmware: (0, 36621))."""
    m_hz = re.search(r"\bhz=([0-9.]+)", begin_line)
    m_dr = re.search(r"\bdrive=([0-9]+)", begin_line)
    return (int(m_dr.group(1)) if m_dr else 0, round(float(m_hz.group(1))) if m_hz else DEFAULT_HZ)


def cfg_name(cfg):
    return "%d:%.0f" % tuple(cfg)


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
                   "cfg": parse_cfg(it[2]), "t": it[1]}
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


def configs(runs):
    """Drive configs in the order of their newest run."""
    out = []
    for r in runs:
        if r["cfg"] in out:
            out.remove(r["cfg"])
        out.append(r["cfg"])
    return out


def num(s):
    try:
        v = float(s)
    except (TypeError, ValueError):
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


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def _r(v, n=4):
    # JSON-sized numbers; 6 decimals so the report's own rounding (%.2f / %.0f) is not a double rounding
    return None if v is None else round(v, max(n, 6))


# ------------------------------------------------------------------ analysis
def analyze(runs, params, cfg, vref=None, vbatt=None, band=None):
    """Fit the runs of one drive config. Returns a JSON-ready dict (see report() for what is in it)."""
    R = {"cfg": list(cfg), "cfg_name": cfg_name(cfg), "cfg_label": cfg_label(cfg), "warnings": [],
         "a": None, "b": None, "c": None, "proposal": None}
    warn = []

    def vb_of(s):
        v = num(s)
        if v is not None and V_OK[0] <= v <= V_OK[1]:
            return v
        if vbatt:
            return vbatt
        warn.append("battery reading %s V invalid; volts unknown (use --vbatt)" % s)
        return None

    def volts(duty, vb_s):
        vb = vb_of(vb_s)
        return None if vb is None or duty is None or duty < 0 else duty / 4095.0 * vb

    # ---------------- test a: dead zone
    A = {}
    ra = pick(runs, "a")
    if ra:
        for f in ra["a"]:
            # mt,a,wheel,dir,start|stop,duty,volts,vbatt,speed
            w, d, kind, duty = f[2], f[3], f[4], num(f[5])
            A.setdefault((w, d), {})[kind] = (duty if duty is not None and duty >= 0 else None, volts(duty, f[7]))
            A[(w, d)].setdefault("vb", vb_of(f[7]))
        rows = []
        for w in WHEELS:
            for d in DIRS:
                e = A.get((w, d), {})
                s, t = e.get("start", (None, None)), e.get("stop", (None, None))
                rows.append({"wheel": w, "dir": d, "start_duty": s[0], "start_v": _r(s[1]), "stop_duty": t[0],
                             "stop_v": _r(t[1]), "vb": e.get("vb")})
        R["a"] = {"status": ra["status"], "rows": rows}

    # ---------------- test b: speed curve
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
        signs = []
        for (w, d), pts in B.items():
            pts = [(v, abs(s)) for v, s, _ in pts if v is not None and s is not None]
            moving = [(v, s) for v, s in pts if s > 3 * MOVE_MMS]
            want = 1 if d == "fwd" else -1
            top = max(B[(w, d)], key=lambda p: p[2] or 0)
            if top[1] is not None and abs(top[1]) > MOVE_MMS and (top[1] > 0) != (want > 0):
                signs.append("%s %s" % (w, d))
            fit = linfit([v for v, _ in moving], [s for _, s in moving])
            if fit and fit[0] > 0 and pts:
                k, c, res = fit
                fits[(w, d)] = {"k": k, "v0": -c / k, "res": res, "top": max(s for _, s in pts),
                                "pts": sorted([(0.0, 0.0)] + pts)}
        if signs:
            warn.append("encoder sign opposite to the command on: %s (check ENC direction / motor wiring)" % ", ".join(signs))
        keys = [(w, d) for w in WHEELS for d in DIRS]
        table = []
        for i in range(max([len(B.get(k, [])) for k in keys] + [0])):
            first = next((B[k][i] for k in keys if k in B and i < len(B[k])), None)
            table.append({"duty": first[2] if first else None, "volts": _r(first[0]) if first else None,
                          "speed": {"%s_%s" % k: (_r(B[k][i][1], 1) if k in B and i < len(B[k]) else None) for k in keys}})
        bfits = {}
        for (w, d), f in fits.items():
            bfits["%s_%s" % (w, d)] = {"k": _r(f["k"], 2), "v0": _r(f["v0"]), "res": _r(f["res"], 2),
                                       "resid_pct": _r(100 * f["res"] / max(1, f["top"]), 2), "top": _r(f["top"], 1)}
        tau = {w: median(taus[w]) for w in WHEELS if taus.get(w)}
        kl = [fits[("left", d)]["k"] for d in DIRS if ("left", d) in fits]
        kr = [fits[("right", d)]["k"] for d in DIRS if ("right", d) in fits]
        lr = None
        if kl and kr:
            lr = {"left": _r(sum(kl) / len(kl), 2), "right": _r(sum(kr) / len(kr), 2),
                  "left_pct": _r(100 * (sum(kl) / len(kl) / (sum(kr) / len(kr)) - 1), 2)}
        fr = {}
        for w in WHEELS:
            if (w, "fwd") in fits and (w, "rev") in fits:
                fr[w] = {"pct": _r(100 * (fits[(w, "fwd")]["k"] / fits[(w, "rev")]["k"] - 1), 2),
                         "v0_fwd": _r(fits[(w, "fwd")]["v0"]), "v0_rev": _r(fits[(w, "rev")]["v0"])}
        R["b"] = {"status": rb["status"], "table": table, "fits": bfits, "tau_ms": tau, "lr": lr, "fr": fr,
                  "points": {"%s_%s" % k: [[_r(v), _r(abs(s), 1)] for v, s, _ in pts if v is not None and s is not None]
                             for k, pts in B.items()}}

    # ---------------- test c: reversal
    rc = pick(runs, "c")
    if rc:
        C = {}
        for f in rc["c"]:
            # mt,c,wheel,newdir,rep,duty,volts,vbatt,t_stop,t_rev,coast_deg
            w, d = f[2], f[3]
            ts, tr, co = num(f[8]), num(f[9]), num(f[10])
            C.setdefault((w, d), []).append(None if tr is None or tr < 0 else (ts, tr, co))
        f0 = rc["c"][0] if rc["c"] else None
        rows = []
        for w in WHEELS:
            for d in DIRS:
                xs = C.get((w, d), [])
                ok = [x for x in xs if x]
                if not xs:
                    continue
                ms = (lambda i: sum(x[i] for x in ok) / len(ok)) if ok else (lambda i: None)
                rows.append({"wheel": w, "dir": d, "n_ok": len(ok), "n": len(xs), "stop_ms": _r(ms(0), 1),
                             "rev_ms": _r(ms(1), 1), "dwell_ms": _r(ms(1) - ms(0), 1) if ok else None,
                             "coast_deg": _r(ms(2), 2)})
        R["c"] = {"status": rc["status"], "duty": num(f0[5]) if f0 else None,
                  "volts": _r(volts(num(f0[5]), f0[7])) if f0 else None, "rows": rows}

    # ---------------- summary for the config comparison
    S = {"start_v": [], "stop_v": [], "v0": [], "k": [], "resid": [], "top": [], "dwell": []}
    for row in (R["a"] or {}).get("rows", []):
        S["start_v"].append(row["start_v"])
        S["stop_v"].append(row["stop_v"])
    for key in KEYS:
        f = (R["b"] or {}).get("fits", {}).get(key)
        if f:
            S["v0"].append(f["v0"])
            S["k"].append(f["k"])
            S["resid"].append(f["resid_pct"])
            S["top"].append(f["top"])
    for row in (R["c"] or {}).get("rows", []):
        S["dwell"].append(row["dwell_ms"])
    R["summary"] = {"start_v": _r(_mean(S["start_v"])), "stop_v": _r(_mean(S["stop_v"])), "v0": _r(_mean(S["v0"])),
                    "k": _r(_mean(S["k"]), 2), "resid": _r(_mean(S["resid"]), 2), "top": _r(_mean(S["top"]), 1),
                    "dwell": _r(_mean(S["dwell"]), 1), "tau_ms": (R["b"] or {}).get("tau_ms", {})}

    if not fits and not A:
        R["warnings"] = sorted(set(warn))
        return R

    # ---------------- proposal
    vref = vref or params.get("mot.v_ref") or 11.5
    band = band if band is not None else params.get("mot.dz_band", 0.01)
    dz, dz_rows = {}, []
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
            dz_rows.append({"param": "mot.dz_%s_%s" % (w[0], d), "v": _r(dz[(w, d)], 2), "how": how,
                            "start": _r(st), "stop": _r(sp)})
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

    vb_vals = [p[0] / (p[2] / 4095.0) for pts in B.values() for p in pts if p[0] and p[2]]
    vb_mean = sum(vb_vals) / len(vb_vals) if vb_vals else (vbatt or 11.5)
    cur = {"exp": params.get("mot.curve_exp", 3.92), "ki": params.get("mot.knee_in", 1250.0),
           "ko": params.get("mot.knee_out", 2750.0), "ls": params.get("mot.left_scale", 0.95),
           "rs": params.get("mot.right_scale", 1.0)}

    def wheel_volts(model, w, u):
        f = fits[(w, "fwd")]
        if model == 0:
            sc = cur["ls"] if w == "left" else cur["rs"]
            return v3_curve(u * sc * 4095, cur["exp"], cur["ki"], cur["ko"]) / 4095.0 * vb_mean
        return min(dz_volts(u * scales.get(w, 1.0), dz.get((w, "fwd"), f["v0"]), vref, band), vb_mean)

    ws = [w for w in WHEELS if (w, "fwd") in fits]

    def mean_speed(model, u):
        return sum(interp(fits[(w, "fwd")]["pts"], wheel_volts(model, w, u)) for w in ws) / len(ws)

    u_table, linearity, gain_ratio = [], [], None
    if ws:
        for u in (0.005, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0):
            row = {"u": u}
            for model, tag in ((0, "v3"), (1, "m1")):
                vs = [wheel_volts(model, w, u) for w in ws]
                row[tag + "_volts"] = [_r(v, 2) for v in vs]
                row[tag + "_speed"] = [_r(interp(fits[(w, "fwd")]["pts"], v), 1) for w, v in zip(ws, vs)]
            u_table.append(row)
        us = [i / 100.0 for i in range(2, 61)]
        slopes = []
        for model in (0, 1):
            sp = [mean_speed(model, u) for u in us]
            g = sum(u * s for u, s in zip(us, sp)) / sum(u * u for u in us)
            dev = math.sqrt(sum((s - g * u) ** 2 for u, s in zip(us, sp)) / len(us))
            slopes.append((mean_speed(model, 0.2) - mean_speed(model, 0.05)) / 0.15)
            linearity.append({"model": model, "dev": _r(dev, 1), "dev_pct": _r(100 * dev / max(1, max(sp)), 1),
                              "at_001": _r(mean_speed(model, 0.01), 1), "slope": _r(slopes[-1], 1)})
        if slopes[1] > 0:
            gain_ratio = slopes[0] / slopes[1]

    pset = {"mot.model": 1.0, "mot.vcomp": 1.0, "mot.v_ref": round(vref, 1)}
    for (w, d), v in sorted(dz.items()):
        pset["mot.dz_%s_%s" % (w[0], d)] = round(v, 2)
    if scales:
        pset["mot.left_scale"] = round(scales["left"], 3)
        pset["mot.right_scale"] = round(scales["right"], 3)
    R["proposal"] = {
        "vref": vref, "band": band, "dz": dz_rows,
        "scales": {w: _r(s, 3) for w, s in scales.items()} if scales else None,
        "speed_at_u1": {w: _r(K[w], 1) for w in K} if len(K) == 2 else None,
        "vb_mean": _r(vb_mean, 2), "u_wheels": ws, "u_table": u_table, "linearity": linearity,
        "gain_ratio": _r(gain_ratio, 3), "params": pset,
        "back": {"mot.model": 0.0, "mot.left_scale": cur["ls"], "mot.right_scale": cur["rs"]},
    }
    R["warnings"] = sorted(set(warn))
    return R


def fit_items(items, vref=None, vbatt=None, band=None, params=None):
    """Everything the UI needs from mt items: runs (newest last) and one result per drive config
    (newest config last). params: current robot params {name: value} (else the logged ones)."""
    runs, logged = split_runs(items)
    p = dict(logged)
    if params:
        p.update(params)
    out = {"runs": [{"test": r["test"], "cfg": cfg_name(r["cfg"]), "status": r["status"], "t": r["t"],
                     "n": {k: len(r[k]) for k in "abc"}} for r in runs], "results": []}
    for cfg in configs(runs):
        out["results"].append(analyze([r for r in runs if r["cfg"] == cfg], p, cfg, vref, vbatt, band))
    return out


# ------------------------------------------------------------------ text report
def report(R):
    """The text tools/motor_fit.py prints for one config."""
    L = []
    p = L.append
    if R["a"]:
        p("\n== a) dead zone (duty of 4095 / volts)            run status: %s" % R["a"]["status"])
        p("%-6s %-4s %12s %12s %8s" % ("wheel", "dir", "start", "stop", "vbatt"))
        for row in R["a"]["rows"]:
            p("%-6s %-4s %5s %5sV %5s %5sV %7sV" % (row["wheel"], row["dir"], fmt(row["start_duty"], "%.0f"),
                                                   fmt(row["start_v"]), fmt(row["stop_duty"], "%.0f"),
                                                   fmt(row["stop_v"]), fmt(row["vb"])))
    b = R["b"]
    if b:
        p("\n== b) steady speed (mm/s) vs motor volts          run status: %s" % b["status"])
        p("%6s %6s " % ("duty", "volts") + " ".join("%9s" % ("%s_%s" % (k[0], k.split("_")[1])) for k in KEYS))
        for row in b["table"]:
            line = "%6s %6s " % (fmt(row["duty"], "%.0f"), fmt(row["volts"])) if row["duty"] is not None else ""
            line += " ".join("%9s" % fmt(row["speed"][k], "%.0f") for k in KEYS)
            p(line)
        p("\nlinear fit above the dead zone: speed = k * (V - V0)")
        p("%-6s %-4s %10s %7s %10s %9s" % ("wheel", "dir", "k mm/s/V", "V0", "resid", "top mm/s"))
        for k in KEYS:
            f = b["fits"].get(k)
            if f:
                w, d = k.split("_")
                p("%-6s %-4s %10.1f %6.2fV %8.1f%% %9.0f" % (w, d, f["k"], f["v0"], f["resid_pct"], f["top"]))
        for w in WHEELS:
            if w in b["tau_ms"]:
                p("time constant %s: median %.0f ms (63%% step response, 80 ms speed window removed)" % (w, b["tau_ms"][w]))
        if b["lr"]:
            p("left/right speed gain: left %.1f, right %.1f mm/s/V -> left is %+.1f%%" %
              (b["lr"]["left"], b["lr"]["right"], b["lr"]["left_pct"]))
        for w in WHEELS:
            if w in b["fr"]:
                f = b["fr"][w]
                p("%s fwd/rev gain: %+.1f%%   V0 fwd %.2f / rev %.2f V" % (w, f["pct"], f["v0_fwd"], f["v0_rev"]))
    c = R["c"]
    if c:
        p("\n== c) reversal at %s duty (%s V)                  run status: %s" %
          (fmt(c["duty"], "%.0f") if c["duty"] is not None else "?", fmt(c["volts"]), c["status"]))
        p("%-6s %-8s %4s %9s %9s %9s %10s" % ("wheel", "to dir", "n", "stop ms", "rev ms", "dwell ms", "coast deg"))
        for row in c["rows"]:
            p("%-6s %-8s %4s %9s %9s %9s %10s%s" % (
                row["wheel"], row["dir"], "%d/%d" % (row["n_ok"], row["n"]), fmt(row["stop_ms"], "%.0f"),
                fmt(row["rev_ms"], "%.0f"), fmt(row["dwell_ms"], "%.0f"), fmt(row["coast_deg"], "%.1f"),
                "" if row["n_ok"] == row["n"] else "  (some never reversed in 2 s)"))
        p("stop = command -> wheel at its turning point; rev = until 2 deg back; dwell = rev - stop")
        p("(the encoder is on the gearbox output: free play shows up as dwell, not as an angle)")
    P = R["proposal"]
    if not P:
        for w in R["warnings"]:
            p("WARNING: " + w)
        return L
    p("\n== proposal (mot.model 1 = volts model, mot.vcomp 1, mot.v_ref %.1f V)" % P["vref"])
    for row in P["dz"]:
        p("%s = %.2f V   (%s; start %s, stop %s)" % (row["param"], row["v"], row["how"], fmt(row["start"]), fmt(row["stop"])))
    if P["scales"]:
        p("mot.left_scale = %.3f, mot.right_scale = %.3f   (speed at u=1: left %.0f, right %.0f mm/s at %.1f V)" %
          (P["scales"]["left"], P["scales"]["right"], P["speed_at_u1"]["left"], P["speed_at_u1"]["right"], P["vref"]))
    if P["u_table"]:
        p("\nspeed (mm/s, fwd) for a controller output u (balancing uses |u| ~ 0.05-0.2):")
        p("%6s | %15s %15s | %15s %15s" % ("u", "v3 volts", "v3 L / R", "model1 volts", "model1 L / R"))
        for row in P["u_table"]:
            cols = ["%15s %15s" % ("/".join("%.2f" % v for v in row[t + "_volts"]),
                                   "/".join("%.0f" % s for s in row[t + "_speed"])) for t in ("v3", "m1")]
            p("%6.3f | %s | %s" % (row["u"], cols[0], cols[1]))
        p("(v3 at the test's mean pack voltage %.1f V; model 1 with vcomp = same at any charge)" % P["vb_mean"])
        for li in P["linearity"]:
            p("%s: rms deviation from a straight line %.0f mm/s (%.0f%% of top), speed already at u=0.01: %.0f mm/s, "
              "slope over u 0.05..0.2: %.0f mm/s per unit u" %
              ("v3 curve   " if li["model"] == 0 else "volts model", li["dev"], li["dev_pct"], li["at_001"], li["slope"]))
        if P["gain_ratio"]:
            g = P["gain_ratio"]
            p("-> in the balancing range the volts model gives %.2fx the motor gain of the v3 curve; to start from the"
              " same loop gain multiply bal.kp / bal.ki / bal.kd by ~%.2f (then retune)" % (1 / g, g))
    p("\ntry it (on the Pi in ~/self_balancing_robot_3, robot balancing; mot.model 0 switches back;")
    p("  mot.pwm_hz applies when the motors are off: disarm, set, re-arm):")
    cmd = ["mot.pwm_hz=%.0f" % R["cfg"][1], "mot.model=1", "mot.vcomp=1", "mot.v_ref=%.1f" % P["vref"]]
    cmd += ["%s=%.2f" % (row["param"], row["v"]) for row in sorted(P["dz"], key=lambda r: r["param"])]
    if P["scales"]:
        cmd += ["mot.left_scale=%.3f" % P["scales"]["left"], "mot.right_scale=%.3f" % P["scales"]["right"]]
    p("  python3 tools/tune.py set " + " ".join(cmd))
    p("  (back: mot.model=0 mot.left_scale=%g mot.right_scale=%g; Save to robot / cfg.save keeps it)" %
      (P["back"]["mot.left_scale"], P["back"]["mot.right_scale"]))
    for w in R["warnings"]:
        p("WARNING: " + w)
    return L


def compare(results):
    """One row per drive config: dead zone, linearity, time constant, reversal."""
    L = ["\n" + "=" * 100, "=== comparison (means over wheels and directions; volts = duty / 4095 * pack V)",
         "%-9s %8s %8s %8s %10s %8s %9s %9s %9s" % ("config", "start V", "stop V", "fit V0", "k mm/s/V",
                                                    "resid", "tau L/R", "top mm/s", "dwell ms")]
    for R in results:
        S = R["summary"]
        tau = "/".join(fmt(S["tau_ms"].get(w), "%.0f") for w in WHEELS)
        L.append("%-9s %8s %8s %8s %10s %7s%% %9s %9s %9s" % (
            R["cfg_name"], fmt(S["start_v"]), fmt(S["stop_v"]), fmt(S["v0"]), fmt(S["k"], "%.1f"),
            fmt(S["resid"], "%.1f"), tau, fmt(S["top"], "%.0f"), fmt(S["dwell"], "%.0f")))
    L.append("lower start/stop V and V0 = smaller dead zone; lower resid = straighter speed vs volts above it;")
    L.append("drive 0 = PWM on EN (coast in the off-time), drive 1 = PWM on IN (brake in the off-time)")
    return L


# ------------------------------------------------------------------ live state (bridge)
class LiveState:
    """Follows the robot's mt lines: waiting (+ reason), running (+ part, progress), done / aborted."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state, self.test, self.wait, self.part, self.result = "idle", 0, None, None, None
        self.counts = {"a": 0, "b": 0, "c": 0}
        self.t_req = self.t_begin = self.t_end = None
        self.last = None

    def feed(self, text, t):
        """One mt line (t = session seconds). Returns True if the state changed."""
        f = text.split(",")
        kind = f[1] if len(f) > 1 else ""
        self.last = text
        if kind == "request":
            self.state, self.test, self.wait, self.part, self.result = "waiting", int(num(f[2]) or 0), None, None, None
            self.counts = {"a": 0, "b": 0, "c": 0}
            self.t_req, self.t_begin, self.t_end = t, None, None
        elif kind == "wait":
            if self.state != "running":
                self.state = "waiting"
            self.wait = f[2] if len(f) > 2 else "?"
        elif kind == "cancel":
            self.state, self.wait, self.result, self.t_end = "idle", None, "cancelled", t
        elif kind == "begin":
            self.state, self.wait, self.t_begin = "running", None, t
            self.test = int(num(f[2]) or self.test or 0)
            self.counts = {"a": 0, "b": 0, "c": 0}
        elif kind in ("a", "b", "c"):
            self.part = kind
            self.counts[kind] += 1
        elif kind == "end":
            self.state, self.result, self.t_end, self.wait = "done", "ok", t, None
        elif kind == "abort":
            self.state, self.result, self.t_end, self.wait = "aborted", ",".join(f[2:]), t, None
        else:
            return False
        return True

    def progress(self):
        parts = PARTS.get(self.test, "")
        if not parts:
            return 0.0
        if self.state == "done":
            return 1.0
        done = sum(min(1.0, self.counts[p] / EXPECTED[p]) for p in parts)
        return round(done / len(parts), 3)

    def to_dict(self):
        return {"state": self.state, "test": self.test, "wait": self.wait, "part": self.part,
                "counts": self.counts, "progress": self.progress(), "result": self.result,
                "t_req": self.t_req, "t_begin": self.t_begin, "t_end": self.t_end,
                "est_s": DURATION_S.get(self.test)}
