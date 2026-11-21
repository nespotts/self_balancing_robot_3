"""Balance scoring from full-rate session logs (issue #16). Stdlib only; used by the bridge (Tune page
"Score" card) and by tools/tune.py (score / measure).

  rows = read_window(path, last_s=20)            # or t0=..., t1=... (session seconds)
  s = score(rows)                                # dict, JSON-ready
  print("\n".join(format_score(s, 20)))

Scores only rows where the robot is armed and upright (status bits 0 and 1) and not in IDLE.
Needs tlm.hz high (400) for good numbers. The metrics are the ones tools/tune.py printed on
2026-11-21: pitch error rms, oscillation (zero crossings), motor rms / saturation / sign flips,
P/I/D rms, D step noise, wheel speed rms, and pushes (> 3 deg) with settle time and overshoot.
"""
import json
import math
import os
import re

KEYS = ("t_us", "mode", "status", "pitch_deg", "balance_target_deg", "pitch_rate_dps", "motor_left", "motor_right",
        "bal_p", "bal_i", "bal_d", "loop_hz", "wheel_left_mms", "wheel_right_mms")
MAX_WINDOW_S = 300.0        # longer windows are cut to their last 5 minutes (memory on the Pi)
MIN_ROWS = 200
_T_RE = re.compile(rb'^\["(\w+)",(-?[0-9][0-9.eE+-]*)')

# metric key -> (label, unit, better): better "low" = smaller is better, None = no ranking
METRICS = [
    ("err_rms", "Pitch error rms", "°", "low"),
    ("rate_rms", "Pitch rate rms", "°/s", "low"),
    ("osc_hz", "Oscillation", "Hz", None),
    ("motor_rms", "Motor rms", "", "low"),
    ("sat_pct", "Saturation", "%", "low"),
    ("flips_s", "Sign flips", "/s", "low"),
    ("d_noise", "D step noise", "", "low"),
    ("wheel_rms", "Wheel speed rms", "mm/s", "low"),
    ("push_n", "Pushes", "", None),
    ("settle_mean", "Settle (mean)", "s", "low"),
    ("over_mean", "Overshoot (mean)", "°", "low"),
    ("err_mean", "Pitch error mean", "°", None),
    ("p_rms", "P rms", "", None),
    ("i_rms", "I rms", "", None),
    ("d_rms", "D rms", "", None),
    ("tlm_hz", "Telemetry", "Hz", None),
]


# ------------------------------------------------------------------ reading
def _line_t(line):
    m = _T_RE.match(line)
    if not m:
        return None, None
    try:
        return m.group(1).decode(), float(m.group(2))
    except ValueError:
        return None, None


def read_schema(path):
    with open(path, "rb") as f:
        first = json.loads(f.readline())
    return first[1]["tlm"]


def _offset_for(f, size, t):
    """Byte offset of a line boundary at or before the first line with time >= t (binary search)."""
    lo, hi = 0, size
    while hi - lo > 65536:
        mid = (lo + hi) // 2
        f.seek(mid)
        f.readline()
        tt = None
        for _ in range(50):
            line = f.readline()
            if not line:
                break
            _k, tt = _line_t(line)
            if tt is not None:
                break
        if tt is None or tt >= t:
            hi = mid
        else:
            lo = mid
    return lo


def last_time(path, kind="tlm"):
    """Session time (s) of the last complete line of this kind (None = any kind)."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        f.seek(max(0, size - 65536))
        lines = f.read().splitlines()
    for line in reversed(lines):
        k, t = _line_t(line)
        if t is not None and (kind is None or k == kind) and line.rstrip().endswith(b"]"):
            return t    # (the last line of a live file may be half written)
    return None


def read_window(path, t0=None, t1=None, last_s=None, max_s=MAX_WINDOW_S, keep_kinds=("param",)):
    """Telemetry rows [(t, {key: value})] with t0 <= t <= t1 (session seconds; None = file start / end).
    last_s: the last N seconds of the file. Also returns other records of keep_kinds as [row, ...].
    Windows longer than max_s keep their end."""
    schema = read_schema(path)
    idx = [(k, schema.index(k)) for k in KEYS if k in schema]
    if t0 is None and t1 is None and last_s is None:
        last_s = max_s
    if last_s is not None:
        t1 = last_time(path)
        if t1 is None:
            return [], []
        t0 = t1 - last_s
    if t1 is not None and t0 is not None and t1 - t0 > max_s:
        t0 = t1 - max_s
    elif t0 is None and t1 is not None:
        t0 = t1 - max_s
    rows, other = [], []
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        f.seek(_offset_for(f, size, t0) if t0 is not None else 0)
        for line in f:
            kind, t = _line_t(line)
            if t is None or (t0 is not None and t < t0 - 1e-6):
                continue
            if t1 is not None and t > t1 + 1e-6:
                break
            if kind == "tlm":
                try:
                    a = json.loads(line)
                except ValueError:
                    continue
                vals = a[2:]
                rows.append((t, {k: vals[i] for k, i in idx}))
            elif kind in keep_kinds:
                try:
                    other.append(json.loads(line))
                except ValueError:
                    pass
    if t0 is None and rows and rows[-1][0] - rows[0][0] > max_s:
        cut = rows[-1][0] - max_s
        rows = [r for r in rows if r[0] >= cut]
    return rows, other


# ------------------------------------------------------------------ maths
def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def _r(v, n=4):
    # >= 4 decimals: format_score rounds again (%.1f ...), and a double rounding would change the text
    return None if v is None or (isinstance(v, float) and not math.isfinite(v)) else round(v, max(n, 4))


def find_pushes(e0, hz):
    """Pushes: |err| peaks > 3 deg; settle = time until |err| stays < 1 deg for 0.3 s (None = not within the
    data); overshoot = largest excursion to the other side before settling (2 s if it never settles)."""
    pushes, i, n = [], 0, len(e0)
    while i < n:
        if abs(e0[i]) > 3.0:
            j, peak = i, i
            while j < n and abs(e0[j]) > 1.0 or j - i < 40:
                if j >= n:
                    break
                if abs(e0[j]) > abs(e0[peak]):
                    peak = j
                j += 1
            k, settle = peak, None
            win = int(0.3 * hz)
            while k + win < n:
                if all(abs(x) < 1.0 for x in e0[k:k + win]):
                    settle = (k - peak) / hz
                    break
                k += 1
            seg = e0[peak:k] if settle is not None else e0[peak:peak + int(2 * hz)]
            sign = e0[peak] > 0
            over = max([(-x if sign else x) for x in seg] + [0.0])
            pushes.append((e0[peak], settle, over))
            i = max(k, peak + int(0.5 * hz)) if settle is not None else peak + int(2 * hz)
        else:
            i += 1
    return pushes


def score(rows, seconds=None):
    """rows: [(t, dict)] or [dict] (telemetry, KEYS at least). Returns a JSON-ready dict; "ok" False
    with "reason" when there is not enough balancing data."""
    rows = [r[1] if isinstance(r, tuple) else r for r in rows]
    up = [r for r in rows if (int(r["status"]) & 3) == 3 and r["mode"] != 0]
    out = {"ok": False, "rows": len(rows), "up": len(up), "up_pct": _r(100.0 * len(up) / max(1, len(rows)), 1),
           "seconds": seconds}
    if len(up) < MIN_ROWS:
        out["reason"] = "not enough balancing data (arm it and let it balance)"
        return out
    dt = max(1e-6, (up[-1]["t_us"] - up[0]["t_us"]) / 1e6) if up[-1]["t_us"] > up[0]["t_us"] else len(up) / 400.0
    hz = len(up) / dt
    err = [r["pitch_deg"] - r["balance_target_deg"] for r in up]
    mean_err = sum(err) / len(err)
    e0 = [e - mean_err for e in err]
    zc = sum(1 for a, b in zip(e0, e0[1:]) if (a < 0) != (b < 0))
    mot = [(r["motor_left"] + r["motor_right"]) / 2 for r in up]
    flips = sum(1 for a, b in zip(mot, mot[1:]) if abs(a) > 0.02 and abs(b) > 0.02 and (a < 0) != (b < 0))
    sat = sum(1 for r in up if max(abs(r["motor_left"]), abs(r["motor_right"])) > 0.98)
    dd = [b["bal_d"] - a["bal_d"] for a, b in zip(up, up[1:])]
    pushes = find_pushes(e0, hz)
    settled = [s for _p, s, _o in pushes if s is not None]
    out.update({
        "ok": True, "balancing_s": _r(dt, 2), "tlm_hz": _r(hz, 0),
        "loop_hz": _r(sum(r["loop_hz"] for r in up) / len(up), 0),
        "err_mean": _r(mean_err), "err_rms": _r(rms(e0)), "rate_rms": _r(rms([r["pitch_rate_dps"] for r in up]), 2),
        "osc_hz": _r(zc / 2 / dt, 2), "motor_rms": _r(rms(mot), 4), "sat_pct": _r(100.0 * sat / len(up), 2),
        "flips_s": _r(flips / dt, 2),
        "p_rms": _r(rms([r["bal_p"] for r in up]), 1), "i_rms": _r(rms([r["bal_i"] for r in up]), 1),
        "d_rms": _r(rms([r["bal_d"] for r in up]), 1), "d_noise": _r(rms(dd), 2),
        "wheel_rms": _r(rms([(r["wheel_left_mms"] + r["wheel_right_mms"]) / 2 for r in up]), 1),
        "lean_rms": _r(rms([r["balance_target_deg"] for r in up])),
        "pushes": [{"peak": _r(p, 2), "settle": _r(s, 3), "over": _r(o, 2)} for p, s, o in pushes],
        "push_n": len(pushes),
        "unsettled": len(pushes) - len(settled),
        "settle_mean": _r(sum(settled) / len(settled)) if settled else None,
        "settle_max": _r(max(settled)) if settled else None,
        "over_mean": _r(sum(o for _p, _s, o in pushes) / len(pushes), 2) if pushes else None,
    })
    return out


def format_score(s, seconds=None):
    """The text tools/tune.py prints."""
    secs = seconds if seconds is not None else s.get("seconds")
    lines = ["rows %d in last %ss, balancing %d (%.0f%%)" % (s["rows"], "%g" % secs if secs is not None else "?",
                                                           s["up"], s["up_pct"] or 0)]
    if not s["ok"]:
        lines.append(s.get("reason", "no score"))
        return lines
    lines.append("telemetry %.0f Hz, loop %.0f Hz" % (s["tlm_hz"], s["loop_hz"]))
    lines.append("pitch err  mean %+.2f  rms %.2f deg   pitch rate rms %.1f deg/s" % (s["err_mean"], s["err_rms"], s["rate_rms"]))
    lines.append("oscillation ~%.1f Hz (zero crossings)   motor rms %.2f  sat %.1f%%  sign flips %.1f/s" % (
        s["osc_hz"], s["motor_rms"], s["sat_pct"], s["flips_s"]))
    lines.append("P rms %.0f  I rms %.0f  D rms %.0f   D step-noise rms %.1f" % (s["p_rms"], s["i_rms"], s["d_rms"], s["d_noise"]))
    lines.append("wheel speed rms %.0f mm/s   lean target rms %.2f deg" % (s["wheel_rms"], s["lean_rms"]))
    if s["pushes"]:
        for p in s["pushes"][:8]:
            lines.append("push %+.1f deg -> settle %s, overshoot %.1f deg" % (
                p["peak"], "%.2f s" % p["settle"] if p["settle"] is not None else ">2 s", p["over"]))
    else:
        lines.append("no pushes (>3 deg) in window")
    return lines
