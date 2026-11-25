#!/usr/bin/env python3
"""SIMULATOR ONLY: the #106 end-to-end checks, as the station UI does them (sim_e2e_station.sh starts the stack).

Talks to the local sim's bridge like a phone: ROS commands are {"t": "ros", "action": ...}; what ROS says comes back
as ros_nav / ros_plan / ros_pose / ros_status. Arms the sim and taps Manual (as the owner would), like sim_arm.py, so
it refuses any bridge that is not on this machine.

  python3 ros2/scripts/station_e2e.py --url ws://127.0.0.1:8095/ws [--start 1000,500,90] [--out result.json]
"""
import argparse
import json
import math
import sys
import threading
import time
from urllib.parse import urlparse

import websocket

MODES = ["IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"]
WORLD_PLACES = {"Kitchen": (3300, 1300), "Couch": (700, -700), "Window": (-400, 400)}   # world mm (sim room)


class Station:
    def __init__(self, url):
        if urlparse(url).hostname not in ("127.0.0.1", "localhost"):
            sys.exit("station_e2e.py: refusing %s - this is for the local simulator only" % url)
        end = time.time() + 60                       # the sim may still be starting
        while True:
            try:
                self.ws = websocket.create_connection(url, timeout=5, suppress_origin=True)
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(1.0)
        self.ws.settimeout(None)
        self.ws.send(json.dumps({"t": "hi", "name": "station_e2e"}))
        self.last = {}
        self.errors = []
        self.replies = []
        self.plans = []
        self.poses = []
        self.lock = threading.RLock()
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        while True:
            try:
                m = json.loads(self.ws.recv())
            except Exception:
                return
            t = m.get("t")
            with self.lock:
                self.last[t] = m
                if t == "error":
                    self.errors.append(m)
                elif t == "ros_reply":
                    self.replies.append(m)
                elif t == "ros_plan" and m.get("pts"):
                    self.plans.append((time.time(), m))
                elif t == "ros_pose":
                    self.poses.append((time.time(), m["p"]))
                    del self.poses[:-600]

    def send(self, **m):
        self.ws.send(json.dumps(m))

    def get(self, t):
        with self.lock:
            return self.last.get(t)

    def wait(self, pred, secs, what):
        end = time.time() + secs
        while time.time() < end:
            with self.lock:
                r = pred()
            if r:
                return r
            time.sleep(0.1)
        raise TimeoutError("timed out (%d s): %s" % (secs, what))

    def nav(self):
        return self.last.get("ros_nav") or {}

    def mode(self):
        t = self.last.get("tlm")
        return (MODES[t["mode"]], bool(t["status"] & 1)) if t else (None, False)


def odom_of(start, x, y):
    """world mm -> odometry mm (what the phone sends, the robot having started at start)."""
    sx, sy, sd = start
    c, s = math.cos(math.radians(-sd)), math.sin(math.radians(-sd))
    dx, dy = x - sx, y - sy
    return round(c * dx - s * dy), round(s * dx + c * dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://127.0.0.1:8095/ws")
    ap.add_argument("--start", default="1000,500,90")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    start = [float(v) for v in a.start.split(",")]
    S = Station(a.url)
    res = {"checks": {}, "runs": {}}
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        res["checks"][name] = {"ok": bool(cond), "detail": detail}
        print("%s %s %s" % ("PASS" if cond else "FAIL", name, detail), flush=True)
        ok = ok and bool(cond)

    def run_nav(name, seq0, secs):
        """wait for a new ros_nav run to end -> (msg, seconds, plan points seen)"""
        t0 = time.time()
        n_plans = len(S.plans)
        S.wait(lambda: S.nav().get("seq", -1) > seq0 and S.nav().get("state") == "navigating", 20, name + " starts")
        m = S.wait(lambda: S.nav() if S.nav().get("state") in ("succeeded", "failed", "cancelled") and
                   S.nav().get("seq", -1) > seq0 + 1 else None, secs, name + " ends")
        plans = S.plans[n_plans:]
        return m, round(time.time() - t0, 1), plans

    try:
        S.wait(lambda: ((S.get("status") or {}).get("ros") or {}).get("connected"), 90, "ROS connected (status.ros)")
        st = S.get("status")
        check("ros_connected", True, json.dumps(st["ros"]))
        S.wait(lambda: S.get("ros_map"), 30, "ros_map")
        mp = S.get("ros_map")
        check("ros_map", mp["w"] > 10 and mp["rle"], "%dx%d cells at %.3f m, %d B" % (mp["w"], mp["h"], mp["res"],
                                                                                     len(mp["rle"])))
        # not armed / not in Manual: refused, nothing sent to ROS
        S.send(t="ros", action="goal", x=1.0, y=0.0)
        e = S.wait(lambda: next((x for x in S.errors if x.get("code") == "not_manual"), None), 5, "not_manual refusal")
        check("refused_without_manual", "Manual" in e["text"], e["text"])
        # arm + Manual, as the owner on the phone
        S.send(t="set_mode", mode=MODES.index("BALANCE"), arm=True)
        time.sleep(3.0)
        S.send(t="set_mode", mode=MODES.index("STATION"))
        S.wait(lambda: S.mode() == ("STATION", True), 8, "armed STATION")
        S.wait(lambda: ((S.get("ros_status") or {}).get("loc") or {}).get("state") == "ok" and S.get("ros_pose"), 90,
               "localized")
        st = S.get("ros_status")
        check("localized", True, json.dumps(st["loc"]))
        for n, (x, y) in WORLD_PLACES.items():
            ox, oy = odom_of(start, x, y)
            S.send(t="places", action="add", name=n, x=ox, y=oy)
        S.wait(lambda: len((S.get("places") or {}).get("items") or []) >= 4, 10, "places saved")
        time.sleep(2.0)

        # 1. a Nav2 goal in the ROS map frame (the station's ROS view tap)
        seq = S.nav().get("seq", -1)
        S.send(t="ros", action="goal", frame="map", x=3.3, y=1.3, yaw=0.0)
        m, secs, plans = run_nav("goal", seq, 150)
        p = S.get("ros_pose")["p"]
        d = math.hypot(p[0] - 3.3, p[1] - 1.3)
        res["runs"]["goal"] = {"result": m, "seconds": secs, "plan_msgs": len(plans),
                               "plan_first": plans[0][1] if plans else None, "end_pose": p, "dist_m": round(d, 3)}
        check("goal_reached", m["state"] == "succeeded" and d < 0.25,
              "%s in %.0f s, %.2f m from the goal" % (m["state"], secs, d))
        check("plan_shown", len(plans) >= 1 and len(plans[0][1]["pts"]) >= 2,
              "%d plan messages, first %d points / %.2f m" % (len(plans), len(plans[0][1]["pts"]) if plans else 0,
                                                             plans[0][1]["len_m"] if plans else 0))
        check("plan_cleared", S.wait(lambda: S.get("ros_plan") and not S.get("ros_plan")["pts"], 5, "plan cleared"))

        # 2. cancel a goal on the way
        seq = S.nav().get("seq", -1)
        S.send(t="ros", action="goal", frame="map", x=0.0, y=0.0, yaw=math.pi)
        S.wait(lambda: S.nav().get("seq", -1) > seq and S.nav().get("state") == "navigating" and
               (S.nav().get("dist_m") or 0) > 0, 20, "cancel run starts")
        S.wait(lambda: (S.get("status") or {}).get("cmd_active"), 15, "ROS drives (cmd_vel)")
        time.sleep(4.0)
        v0 = abs((S.get("tlm") or {}).get("speed_mms", 0))
        t_cancel = time.time()
        S.send(t="ros", action="cancel")
        m = S.wait(lambda: S.nav() if S.nav().get("state") == "cancelled" else None, 10, "cancelled")
        # cmd_vel stops: the bridge's cmd_active drops 300 ms after the last one (the adapter's deadman: 0.5 s)
        S.wait(lambda: not (S.get("status") or {}).get("cmd_active"), 5, "cmd_vel stops")
        t_stop = time.time() - t_cancel
        time.sleep(2.0)                               # braking (velocity smoother 0.3 m/s^2, then the balancer)
        p0 = S.get("ros_pose")["p"]
        time.sleep(3.0)
        p1 = S.get("ros_pose")["p"]
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        spd = abs((S.get("tlm") or {}).get("speed_mms", 0))
        res["runs"]["cancel"] = {"result": m, "speed_before_mms": v0, "cmd_vel_stopped_s": round(t_stop, 2),
                                 "moved_after_m": round(moved, 3), "speed_mms": spd}
        # a balancing robot standing in STATION creeps a little (~2 cm/s); driving it would cover ~0.5 m in 3 s
        check("cancel_stops", moved < 0.12 and t_stop < 3.0,
              "cmd_vel stopped %.1f s after the cancel (was %.0f mm/s); then %.3f m in 3 s, %.0f mm/s" % (
                  t_stop, v0, moved, spd))

        # 3. go to a station place (station frame -> ROS map through TF map <- station_map)
        seq = S.nav().get("seq", -1)
        S.send(t="ros", action="place", name="couch")
        m, secs, _ = run_nav("place", seq, 150)
        p = S.get("ros_pose")["p"]
        wx, wy = WORLD_PLACES["Couch"]
        d = math.hypot(p[0] - wx / 1000.0, p[1] - wy / 1000.0)
        res["runs"]["place"] = {"result": m, "seconds": secs, "end_pose": p, "dist_m": round(d, 3)}
        check("place_reached", m["state"] == "succeeded" and m.get("place") == "Couch" and d < 0.3,
              "%s %s in %.0f s, %.2f m from the true spot" % (m.get("place"), m["state"], secs, d))

        # 4. a patrol: Window -> Kitchen, 1 loop, wait 2 s
        seq = S.nav().get("seq", -1)
        S.send(t="ros", action="patrol", places=["Window", "Kitchen"], loops=1, dwell=2)
        t0 = time.time()
        seen = []

        def done():
            n = S.nav()
            if n.get("seq", -1) <= seq:
                return None
            k = (n.get("state"), n.get("place"))
            if not seen or seen[-1] != k:
                seen.append(k)
            return n if n.get("state") in ("succeeded", "failed", "cancelled") and n.get("kind") == "patrol" else None
        m = S.wait(done, 300, "patrol ends")
        res["runs"]["patrol"] = {"result": m, "seconds": round(time.time() - t0, 1), "states": seen}
        check("patrol_done", m["state"] == "succeeded" and not (m.get("patrol") or {}).get("missed") and
              ("dwell", "Window") in seen and ("dwell", "Kitchen") in seen,
              "%s in %.0f s: %s" % (m["state"], time.time() - t0, " > ".join("%s %s" % s for s in seen)))

        # 5. leaving Manual stops a running goal (and the station says why)
        m = {}
        for attempt in range(3):              # (a loaded PC: Nav2 may abort on a TF hiccup before the switch: again)
            seq = S.nav().get("seq", -1)
            S.send(t="ros", action="goal", frame="map", x=0.0, y=0.0)
            S.wait(lambda: S.nav().get("seq", -1) > seq and S.nav().get("state") == "navigating", 20, "goal starts")
            time.sleep(3.0)
            if S.nav().get("state") != "navigating":
                continue
            S.send(t="set_mode", mode=MODES.index("BALANCE"))
            m = S.wait(lambda: S.nav() if S.nav().get("state") in ("cancelled", "failed", "succeeded") else None, 12,
                       "cancelled off Manual")
            S.send(t="set_mode", mode=MODES.index("STATION"))
            S.wait(lambda: S.mode() == ("STATION", True), 8, "back in STATION")
            if m.get("state") == "cancelled":
                break
        check("off_manual_cancels", m.get("state") == "cancelled" and "Manual" in (m.get("reason") or ""),
              "%s: %s" % (m.get("state"), m.get("reason")))
        st = S.get("ros_status")
        check("ros_status_nodes", st["nodes"].get("nav2") == "active" and st["nodes"].get("slam"),
              json.dumps(st["nodes"]))
        h = S.get("health")
        tile = next((t for t in (h or {}).get("tiles", []) if t["id"] == "ros"), None)
        check("health_tile", tile and tile["level"] == "ok", json.dumps(tile))
    except TimeoutError as e:
        check("timeout", False, str(e))
    res["ok"] = ok
    res["errors"] = S.errors[-10:]
    res["replies"] = S.replies[-20:]
    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)
    print("ALL PASS" if ok else "SOME FAILED", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
