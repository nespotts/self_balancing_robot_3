#!/usr/bin/env python3
"""Watch an explore run (sim tests): log /explore/status, snapshot /map + the robot pose, exit when it is over.

  python3 ros2/scripts/explore_watch.py OUT_DIR [--timeout 1200] [--snap 30,90,180]
Writes OUT_DIR/status.jsonl (every status message, with the wall time) and OUT_DIR/snap_<t>s.npz (map, res, origin,
track so far, the target). Exits 0 once the state is done / stopped / error and the auto-save finished (or after
--timeout: exit 2, having asked the explorer to stop).
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class Watch(Node):
    def __init__(self, out, snaps):
        super().__init__("explore_watch")
        self.out = out
        self.snaps = sorted(snaps)
        self.t0 = time.time()
        self.f = open(os.path.join(out, "status.jsonl"), "w")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "explore/status", self.on_status, latched)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.cmd = self.create_publisher(String, "station/explore_cmd", 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.map = None
        self.status = None
        self.track = []
        self.started = False
        self.finished = False
        self.create_timer(0.5, self.tick)

    def on_status(self, m):
        s = json.loads(m.data)
        self.status = s
        s["_wall"] = round(time.time() - self.t0, 2)
        self.f.write(json.dumps(s) + "\n")
        self.f.flush()
        if s.get("running"):
            self.started = True
        if self.started and s["state"] in ("done", "stopped", "error") and s.get("ok") is not None:
            self.finished = True

    def on_map(self, m):
        self.map = (np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width), m.info.resolution,
                    (m.info.origin.position.x, m.info.origin.position.y))

    def tick(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
            p = (t.transform.translation.x, t.transform.translation.y)
            if not self.track or math.hypot(p[0] - self.track[-1][0], p[1] - self.track[-1][1]) > 0.05:
                self.track.append(p)
        except Exception:
            pass
        el = (self.status or {}).get("elapsed_s") or 0.0
        if self.snaps and self.map is not None and self.started and el >= self.snaps[0]:
            s = self.snaps.pop(0)
            g, res, org = self.map
            tgt = (self.status or {}).get("target_map")
            np.savez_compressed(os.path.join(self.out, "snap_%03ds.npz" % s), grid=g, res=res, origin=np.array(org),
                                track=np.array(self.track), target=np.array(tgt if tgt else [np.nan, np.nan]),
                                elapsed=el)

    def stop_explore(self):
        self.cmd.publish(String(data=json.dumps({"cmd": "explore", "run": False})))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--timeout", type=float, default=1200.0)
    ap.add_argument("--snap", default="30,90,180")
    a = ap.parse_args()
    rclpy.init()
    w = Watch(a.out, [float(v) for v in a.snap.split(",") if v])
    end = time.time() + a.timeout
    code = 0
    while rclpy.ok() and not w.finished:
        rclpy.spin_once(w, timeout_sec=0.2)
        if time.time() > end:
            print("explore_watch: timeout, stopping the explorer")
            w.stop_explore()
            t = time.time() + 40
            while time.time() < t and not w.finished:
                rclpy.spin_once(w, timeout_sec=0.2)
            code = 2
            break
    if w.map is not None:
        g, res, org = w.map
        np.savez_compressed(os.path.join(a.out, "final_map.npz"), grid=g, res=res, origin=np.array(org),
                            track=np.array(w.track))
    print("explore_watch: %s" % json.dumps(w.status))
    w.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
