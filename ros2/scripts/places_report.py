#!/usr/bin/env python3
"""Places / patrol test report (sim_e2e_places.sh): where the robot really stopped vs each place (sim truth).

  python3 ros2/scripts/places_report.py OUT_DIR --map MAP.yaml --start X,Y,DEG --place Kitchen=3300,1300 ...
A visit = the true path coming within 0.35 m of a place; per visit: the closest distance, and how long the robot
stood there (within 0.25 m, slower than 2 cm/s).
"""
import argparse
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_png import load_map  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--map", required=True)
    ap.add_argument("--start", default="0,0,0")
    ap.add_argument("--place", action="append", default=[])
    a = ap.parse_args()
    places = {}
    for p in a.place:
        n, v = p.split("=")
        x, y = (float(s) / 1000 for s in v.replace(",", " ").split())
        places[n] = (x, y)
    with open(os.path.join(a.out, "truth.csv")) as f:
        tr = [(float(r["t"]), float(r["X"]) / 1000, float(r["Y"]) / 1000) for r in csv.DictReader(f)]
    pj = os.path.join(a.out, "patrol.json")
    patrol = json.load(open(pj)) if os.path.exists(pj) else {}
    gj = os.path.join(a.out, "goto_kitchen.json")
    goto = json.load(open(gj)) if os.path.exists(gj) else {}
    t_first = goto.get("track", [[None]])[0][0] if goto.get("track") else (patrol.get("track") or [[tr[0][0]]])[0][0]
    visits = []
    for name, (px, py) in places.items():
        cur = None
        for i, (t, x, y) in enumerate(tr):
            if t < t_first:
                continue
            d = math.hypot(x - px, y - py)
            if d <= 0.35:
                if cur is None:
                    cur = {"place": name, "t_in": t, "min_cm": 1e9, "stand_s": 0.0}
                cur["min_cm"] = min(cur["min_cm"], 100 * d)
                if i and d <= 0.25:
                    t0, x0, y0 = tr[i - 1]
                    if t > t0 and math.hypot(x - x0, y - y0) / (t - t0) < 0.02:
                        cur["stand_s"] += t - t0
            elif cur is not None:
                visits.append(cur)
                cur = None
        if cur is not None:
            visits.append(cur)
    visits.sort(key=lambda v: v["t_in"])
    rep = {"goto": {k: goto.get(k) for k in ("result", "seconds")},
           "patrol": {k: patrol.get(k) for k in ("result", "seconds", "loops", "dwell", "missed", "events")},
           "visits": [{"place": v["place"], "at_s": round(v["t_in"] - t_first, 1), "closest_cm": round(v["min_cm"], 1),
                       "stood_s": round(v["stand_s"], 1)} for v in visits]}
    json.dump(rep, open(os.path.join(a.out, "report.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))

    img, res, origin = load_map(a.map)
    w, h = img.size
    fig, ax = plt.subplots(figsize=(8.5, 6.4), dpi=110)
    ax.imshow(img, cmap="gray", extent=[origin[0], origin[0] + w * res, origin[1], origin[1] + h * res],
              origin="upper", vmin=0, vmax=255, interpolation="nearest")
    T = [p for p in tr if p[0] >= t_first]
    ax.plot([p[1] for p in T], [p[2] for p in T], color="#2a7fff", lw=1.6, label="robot (truth)")
    sx, sy, sd = (float(v) for v in a.start.split(","))
    ax.plot(sx / 1000, sy / 1000, "o", color="#2a7fff", ms=8)
    ax.annotate("start / station Home", (sx / 1000, sy / 1000), xytext=(8, -14), textcoords="offset points", fontsize=8)
    for n, (px, py) in places.items():
        vs = [v for v in visits if v["place"] == n]
        ax.plot(px, py, "*", color="#dd0000", ms=15)
        ax.annotate("%s: %d visit%s, %s cm" % (n, len(vs), "" if len(vs) == 1 else "s",
                                               "/".join("%.0f" % v["min_cm"] for v in vs)),
                    (px, py), xytext=(8, 6), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("go_to Kitchen, then patrol Couch -> Window -> Kitchen x %s, dwell %s s: %s" % (
        patrol.get("loops"), patrol.get("dwell"), patrol.get("result")), fontsize=10)
    fig.tight_layout()
    png = os.path.join(a.out, "places.png")
    fig.savefig(png)
    print("wrote", png)


if __name__ == "__main__":
    main()
