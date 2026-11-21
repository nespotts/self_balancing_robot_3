#!/usr/bin/env python3
"""Live balance tuning helper - runs ON THE PI next to the ground-station bridge.

  python3 tune.py params                    show tuning params
  python3 tune.py set bal.kd=7 bal.d_src=1  change params live (through the bridge)
  python3 tune.py measure 20                wait 20 s, then score the last 20 s of balancing
  python3 tune.py score 20                  score the last 20 s without waiting

Scores only rows where the robot is armed and upright. Needs tlm.hz high (400) for good numbers.
"""
import glob, json, math, os, sys, time, urllib.request

BRIDGE = os.environ.get("SBR3_BRIDGE", "http://127.0.0.1:8080")
LOGDIR = os.path.expanduser(os.environ.get("SBR3_LOGS", "~/sbr3-logs"))
GROUPS = ("bal.", "vel.", "str.", "drv.", "ctl.", "tlm.", "mot.")


def api_state(params=False):
    with urllib.request.urlopen(BRIDGE + "/api/state" + ("?params=1" if params else ""), timeout=5) as r:
        return json.loads(r.read().decode())


def api_cmd(cmd):
    req = urllib.request.Request(BRIDGE + "/api/cmd", data=json.dumps(cmd).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read().decode()


def params():
    return {p["name"]: p for p in api_state(True).get("params", [])}


def cmd_params():
    for name, p in sorted(params().items(), key=lambda kv: kv[1]["id"]):
        if name.startswith(GROUPS):
            print("%-20s %g" % (name, p["value"]))


def cmd_set(pairs):
    ps = params()
    for pair in pairs:
        name, val = pair.split("=", 1)
        if name not in ps:
            sys.exit("unknown param " + name)
        api_cmd({"t": "set_param", "id": ps[name]["id"], "value": float(val)})
    time.sleep(0.6)
    ps = params()
    print("now: " + "  ".join("%s=%g" % (p.split("=")[0], ps[p.split("=")[0]]["value"]) for p in pairs))


def latest_log():
    files = sorted(glob.glob(os.path.join(LOGDIR, "*.jsonl")), key=os.path.getmtime)
    if not files:
        sys.exit("no logs in " + LOGDIR)
    return files[-1]


def read_rows(seconds):
    path = latest_log()
    with open(path, "rb") as f:
        schema = json.loads(f.readline())[1]["tlm"]
        size = os.path.getsize(path)
        f.seek(max(0, size - int(seconds * 450 * 260) - 4096))
        f.readline()
        lines = f.read().decode(errors="ignore").splitlines()
    rows, t_end = [], None
    for ln in lines:
        if not ln.startswith('["tlm"'):
            continue
        try:
            a = json.loads(ln)
        except ValueError:
            continue
        rows.append((a[1], dict(zip(schema, a[2:]))))
    if not rows:
        return [], schema
    t_end = rows[-1][0]
    return [r for t, r in rows if t >= t_end - seconds], schema


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def score(seconds):
    rows, _ = read_rows(seconds)
    up = [r for r in rows if (r["status"] & 3) == 3 and r["mode"] != 0]
    print("rows %d in last %ds, balancing %d (%.0f%%)" % (len(rows), seconds, len(up), 100.0 * len(up) / max(1, len(rows))))
    if len(up) < 200:
        print("not enough balancing data (arm it and let it balance)")
        return
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
    # pushes: |err| peaks > 3 deg; settle = time until |err| stays < 1 deg for 0.3 s; count overshoots
    pushes, i = [], 0
    while i < len(err):
        if abs(e0[i]) > 3.0:
            j, peak = i, i
            while j < len(err) and abs(e0[j]) > 1.0 or j - i < 40:
                if j >= len(err):
                    break
                if abs(e0[j]) > abs(e0[peak]):
                    peak = j
                j += 1
            # settle: first index after peak where next 0.3 s all < 1 deg
            k, settle = peak, None
            win = int(0.3 * hz)
            while k + win < len(e0):
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
    print("telemetry %.0f Hz, loop %.0f Hz" % (hz, sum(r["loop_hz"] for r in up) / len(up)))
    print("pitch err  mean %+.2f  rms %.2f deg   pitch rate rms %.1f deg/s" % (mean_err, rms(e0), rms([r["pitch_rate_dps"] for r in up])))
    print("oscillation ~%.1f Hz (zero crossings)   motor rms %.2f  sat %.1f%%  sign flips %.1f/s" % (zc / 2 / dt, rms(mot), 100.0 * sat / len(up), flips / dt))
    print("P rms %.0f  I rms %.0f  D rms %.0f   D step-noise rms %.1f" % (rms([r["bal_p"] for r in up]), rms([r["bal_i"] for r in up]), rms([r["bal_d"] for r in up]), rms(dd)))
    print("wheel speed rms %.0f mm/s   lean target rms %.2f deg" % (rms([(r["wheel_left_mms"] + r["wheel_right_mms"]) / 2 for r in up]), rms([r["balance_target_deg"] for r in up])))
    if pushes:
        for p, s, o in pushes[:8]:
            print("push %+.1f deg -> settle %s, overshoot %.1f deg" % (p, "%.2f s" % s if s is not None else ">2 s", o))
    else:
        print("no pushes (>3 deg) in window")


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] == "params":
        cmd_params()
    elif a[0] == "set":
        cmd_set(a[1:])
    elif a[0] == "measure":
        n = int(a[1]) if len(a) > 1 else 20
        time.sleep(n)
        score(n)
    elif a[0] == "score":
        score(int(a[1]) if len(a) > 1 else 20)
    else:
        print(__doc__)
