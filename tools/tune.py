#!/usr/bin/env python3
"""Live balance tuning helper - runs ON THE PI next to the ground-station bridge.

  python3 tune.py params                    show tuning params
  python3 tune.py set bal.kd=7 bal.d_src=1  change params live (through the bridge)
  python3 tune.py measure 20                wait 20 s, then score the last 20 s of balancing
  python3 tune.py score 20                  score the last 20 s without waiting

Scores only rows where the robot is armed and upright. Needs tlm.hz high (400) for good numbers.
The metrics live in station/bridge/scoring.py (the ground station's Tune page Score card uses them too).
"""
import glob, json, os, sys, time, urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from station.bridge import scoring  # noqa: E402  (same metrics as the ground station's Score card)

BRIDGE = os.environ.get("SBR3_BRIDGE", "http://127.0.0.1:8080")
LOGDIR = os.path.expanduser(os.environ.get("SBR3_LOGS", "~/sbr3-logs"))
GROUPS = ("bal.", "vel.", "str.", "drv.", "ctl.", "tlm.", "mot.", "rng.")


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


def score(seconds):
    rows, _ = scoring.read_window(latest_log(), last_s=seconds)
    print("\n".join(scoring.format_score(scoring.score(rows, seconds), seconds)))

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
