"""Contact detection replay (#111): a real session log through the contact detector (station/sim/contact.py, the port
of src/contact.cpp).

  python -m station.tools.contact_replay extract SESSION.jsonl OUT.csv   the detector's inputs, rebuilt from telemetry
  python -m station.tools.contact_replay run IN.csv                      replay them, print each detection

The telemetry (100 Hz) has what the navigator feeds the detector: the controller's ramped command (cmd_speed_mms),
the encoders' speed, the pitch rate, the velocity loop's lean (vel_out_deg). What the navigator decides itself is
rebuilt from the nav state, the 2 Hz nav status and the command:
  watch   GOTO / EXPLORE, upright, DRIVING (not the stop at the last point: nav index = last, < 2 x arrive_mm),
          or BLOCKED while backing off (the command is a back-off speed: nav.min_speed 40 or twice that)
  ramp    DRIVING and the command rose >= 10 mm/s in the last 150 ms (the nav speed rise limit)
  drive   DRIVING (not backing off)
  cruise  DRIVING, not launching (a ramp that began at nav.min_speed or below: from a stop; after a pass-through
          corner it is cruising), the command not falling >= 10 mm/s in 150 ms (slowing for a stop)
test/host/contact_replay.cpp replays the same CSV through the C++ detector."""
import bisect
import csv
import json
import sys

from station.bridge import link_protocol as lp
from station.sim.contact import ContactDetector

FIELDS = ["t_s", "now_ms", "watch", "upright", "cmd", "speed", "pitch_rate", "vel_out", "cruise", "ramp", "drive"]
MIN_SPEED = 40.0                # nav.min_speed (live)
BACK_SPEEDS = (MIN_SPEED, 2 * MIN_SPEED)   # the back-off speeds (a stuck / contact one: twice)
ARRIVE_MM = 120.0               # nav.arrive_mm (live)


def extract(path):
    """Session JSONL -> list of input dicts (only the stretches in GOTO / EXPLORE / STATION, with 1 s around them)."""
    names, tlm, nav = None, [], []
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            if r[0] == "schema":
                names = r[1]["tlm"]
            elif r[0] == "tlm":
                tlm.append(r)
            elif r[0] == "nav":
                nav.append((r[1], r[2]))
    ix = {n: i + 2 for i, n in enumerate(names)}
    nav_t = [t for t, _ in nav]
    nav_modes = (lp.MODE_GOTO, lp.MODE_EXPLORE, lp.MODE_STATION)
    on = [r[1] for r in tlm if r[ix["mode"]] in nav_modes]
    out, hist = [], []
    launch = False
    for r in tlm:
        t = r[1]
        k = bisect.bisect_left(on, t - 1.0)
        if k >= len(on) or on[k] > t + 1.0:
            continue
        mode, ns, cmd = r[ix["mode"]], r[ix["nav_state"]], r[ix["cmd_speed_mms"]]
        up = bool(r[ix["status"]] & lp.TLM_UPRIGHT)
        j = bisect.bisect_right(nav_t, t) - 1
        st = nav[j][1] if j >= 0 else {}
        n = st.get("waypoint_count", 0)
        stopping = n > 0 and st.get("nav_index", 0) >= n - 1 and st.get("distance_mm", 1e9) < 2 * ARRIVE_MM
        backing = ns == lp.NAV_BLOCKED and any(abs(abs(cmd) - b) < 0.6 for b in BACK_SPEEDS)
        driving = ns == lp.NAV_DRIVING and not stopping
        hist.append((t, cmd))
        while t - hist[0][0] > 0.15:
            del hist[0]
        d = 1.0 if cmd >= 0 else -1.0
        rising = driving and d * (cmd - hist[0][1]) >= 10.0
        launch = rising and (launch or abs(hist[0][1]) <= MIN_SPEED + 5.0)   # (a rise that began at a stop)
        falling = d * (cmd - hist[0][1]) <= -10.0
        out.append({"t_s": t, "now_ms": int(r[ix["t_us"]] // 1000),
                    "watch": mode in (lp.MODE_GOTO, lp.MODE_EXPLORE) and up and (driving or backing),
                    "upright": up, "cmd": cmd, "speed": r[ix["speed_mms"]], "pitch_rate": r[ix["pitch_rate_dps"]],
                    "vel_out": r[ix["vel_out_deg"]], "cruise": driving and not launch and not falling,
                    "ramp": rising, "drive": driving})
    return out


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        for x in rows:
            w.writerow(["%.3f" % x["t_s"], x["now_ms"], int(x["watch"]), int(x["upright"]), "%.1f" % x["cmd"],
                        "%.1f" % x["speed"], "%.1f" % x["pitch_rate"], "%.3f" % x["vel_out"], int(x["cruise"]),
                        int(x["ramp"]), int(x["drive"])])


def read_csv(path):
    rows = []
    with open(path, newline="") as f:
        for x in csv.DictReader(f):
            rows.append({"t_s": float(x["t_s"]), "now_ms": int(x["now_ms"]), "watch": x["watch"] == "1",
                         "upright": x["upright"] == "1", "cmd": float(x["cmd"]), "speed": float(x["speed"]),
                         "pitch_rate": float(x["pitch_rate"]), "vel_out": float(x["vel_out"]),
                         "cruise": x["cruise"] == "1", "ramp": x["ramp"] == "1", "drive": x["drive"] == "1",
                         "lidar_stuck": False,
                         "held": False})
    return rows


def replay(rows):
    """-> [(t_s, kind name, dir, value, stall)] for each detection."""
    det = ContactDetector()
    out = []
    for x in rows:
        k = det.update(x)
        if k:
            out.append((x["t_s"], lp.CONTACT_KIND_NAMES.get(k, str(k)), det.dir, det.value, det.stall))
    return out


def main(argv):
    if len(argv) == 3 and argv[0] == "extract":
        rows = extract(argv[1])
        write_csv(rows, argv[2])
        print("%d rows" % len(rows))
    elif len(argv) == 2 and argv[0] == "run":
        for t, kind, d, value, stall in replay(read_csv(argv[1])):
            print("%8.2f s  %-5s %s %s %4.0f ms" % (t, kind, "stall" if stall else "     ",
                                                  "rear " if d < 0 else "front", value))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
