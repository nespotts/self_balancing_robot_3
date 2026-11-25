"""Frontier explorer (issue #107): drive Nav2 to the frontiers of slam_toolbox's live /map until none is left.

  ros2 launch sbr3_ros explore.launch.py              (adapter + slam_toolbox mapping + Nav2 + this node)
  ros2 run sbr3_ros explore --ros-args -p autostart:=true

Start / stop:
  /station/explore_cmd (std_msgs/String JSON, from the station through the adapter, #106):
      {"cmd": "explore", "run": true|false}      {"cmd": "save_map", "name": "explore-20261124-1530"}
  services /explore/start, /explore/stop (std_srvs/Trigger); autostart:=true starts at once.
Status: /explore/status (std_msgs/String JSON, latched; on every change and at least every status_period s):
  {"state": "waiting|idle|exploring|navigating|dwell|done|stopped|saving|error",
   "target": [x_mm, y_mm] (station map frame) | null, "target_map": [x, y] (m, map frame) | null,
   "frontiers": N, "area_m2": explored free area, "reason": "...", "running": bool,
   "goals_ok": N, "goals_failed": N, "dist_m": driven, "elapsed_s": ..., "saved": name | null, "ok": bool | null}

Loop: plan (frontier.py) -> NavigateToPose to the best frontier's viewpoint -> on the way, re-plan as the map grows
and move on as soon as the target frontier is seen (gone) -> short dwell at a reached goal (a still robot = a clean
LD19 turn) -> next. A goal that aborts, fails, times out or stops making progress blacklists its frontier and
viewpoint; so does a frontier still there after visits_max visits. Done when no reachable frontier is left for
done_checks plans in a row (or max_goals / max_duration_s). Then (or when stopped after > 1 m^2) the map is saved
(auto_save) as map_dir/explore-<YYYYmmdd-HHMM>: .posegraph/.data (slam_toolbox), .yaml/.pgm (map_saver format),
.png and .explore.json (track, goals, stats).
Safety: this node only sends Nav2 goals. /cmd_vel reaches the robot only through the adapter, only while it is armed
in STATION (Manual); with require_station (default) it does not even send goals otherwise.
"""
import json
import math
import os
import re
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import frontier as fr

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
END_STATES = ("done", "stopped", "error")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self):
        super().__init__("explore")
        P = lambda n, v: self.declare_parameter(n, v).value
        self.p = fr.Params()
        for k in vars(self.p):
            setattr(self.p, k, type(getattr(self.p, k))(P(k, getattr(self.p, k))))
        self.map_frame = P("map_frame", "map")
        self.base_frame = P("base_frame", "base_link")
        self.station_frame = P("station_frame", "station_map")
        self.autostart = P("autostart", False)
        self.require_station = P("require_station", True)
        self.plan_period = float(P("plan_period", 1.0))        # s between plans while choosing
        self.replan_period = float(P("replan_period", 2.0))    # s between plans while driving
        self.dwell_s = float(P("dwell_s", 1.5))
        self.done_checks = int(P("done_checks", 3))
        self.progress_s = float(P("progress_s", 30.0))         # no 0.2 m of progress in this long: give up the goal
        self.goal_timeout_min = float(P("goal_timeout_min", 40.0))
        self.speed_est = float(P("speed_est", 0.12))           # m/s, for the goal timeout
        self.visits_max = int(P("visits_max", 2))
        self.max_goals = int(P("max_goals", 150))
        self.max_duration_s = float(P("max_duration_s", 0.0))  # 0: no limit
        self.switch_margin = float(P("switch_margin", 1.0))    # preempt the current goal for a frontier this much better
        self.status_period = float(P("status_period", 1.0))
        self.auto_save = P("auto_save", True)
        self.map_dir = os.path.expanduser(P("map_dir", "~/maps"))
        self.return_home = P("return_home", False)

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.on_costmap, latched)
        self.create_subscription(String, "robot_state", self.on_robot_state, 5)
        self.create_subscription(String, "station/explore_cmd", self.on_cmd, 10)
        self.pub = self.create_publisher(String, "explore/status", latched)
        self.create_service(Trigger, "explore/start", lambda rq, rs: self.srv(rs, True))
        self.create_service(Trigger, "explore/stop", lambda rq, rs: self.srv(rs, False))
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.ser = None                                         # slam_toolbox serialize client (lazy)

        self.map = None            # (grid, res, origin, stamp)
        self.map_seq = 0
        self.cost = None
        self.robot_state = None
        self.running = False
        self.state, self.reason = "idle", "ready"
        self.target = None         # Frontier
        self.goal = None           # dict: handle, t0, timeout, vp, point, prog_xy, prog_t
        self.goal_seq = 0
        self.rejects = 0
        self.retry_at = 0.0
        self.dwell_until = 0.0
        self.blacklist = []
        self.visits = []           # [(x, y, n)]
        self.empty = 0
        self.last_plan = 0.0
        self.frontiers = []
        self.stats = self.new_stats()
        self.saved, self.save_ok = None, None
        self.saving = None         # dict while a save waits for slam_toolbox
        self.last_status, self.last_pub = None, 0.0
        self.home = None
        self.going_home = False
        self.create_timer(0.25, self.tick)
        self.get_logger().info("explore ready (autostart %s, maps -> %s)" % (self.autostart, self.map_dir))

    def new_stats(self):
        return {"t0": None, "t_end": None, "goals_ok": 0, "goals_failed": 0, "goals_left": 0, "dist_m": 0.0,
                "track": [], "goals": [], "plans": 0, "plan_ms": 0.0, "last_xy": None}

    # ------------------------------------------------------------ inputs
    def on_map(self, m):
        g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        o = m.info.origin.position
        self.map = (g, m.info.resolution, (o.x, o.y), time.monotonic())
        self.map_seq += 1

    def on_costmap(self, m):
        g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        o = m.info.origin.position
        self.cost = (g, m.info.resolution, (o.x, o.y))

    def on_robot_state(self, m):
        try:
            self.robot_state = json.loads(m.data)
            self.robot_state["_rx"] = time.monotonic()
        except ValueError:
            pass

    def on_cmd(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            self.get_logger().warn("explore_cmd: not JSON: %r" % m.data[:80])
            return
        if not isinstance(c, dict):
            return
        if c.get("cmd") == "explore":
            self.set_running(bool(c.get("run")))
        elif c.get("cmd") == "save_map":
            self.save_map(c.get("name") or None)
        else:
            self.get_logger().warn("explore_cmd: unknown %r" % c)

    def srv(self, rs, run):
        self.set_running(run)
        rs.success, rs.message = True, "explore %s" % ("started" if run else "stopped")
        return rs

    # ------------------------------------------------------------ start / stop
    def set_running(self, run):
        if run and not self.running:
            self.running = True
            self.blacklist, self.visits, self.empty = [], [], 0
            self.stats = self.new_stats()
            self.stats["t0"] = time.monotonic()
            self.saved = self.save_ok = None
            self.target, self.going_home, self.home = None, False, None
            self.set_state("exploring", "looking for frontiers")
            self.last_plan = 0.0
            self.get_logger().info("explore: started")
        elif not run and self.running:
            self.cancel_goal()
            self.finish("stopped", "stopped by the user")

    def finish(self, state, reason):
        self.cancel_goal()
        self.running = False
        self.target = None
        self.stats["t_end"] = time.monotonic()
        self.set_state(state, reason)
        self.get_logger().info("explore %s: %s (%d goals ok, %d failed, %.1f m, %.0f s, %.1f m2)" % (
            state, reason, self.stats["goals_ok"], self.stats["goals_failed"], self.stats["dist_m"],
            self.elapsed(), self.area()))
        if self.auto_save and (state == "done" or self.area() > 1.0):
            self.save_map(None)

    def set_state(self, state, reason):
        self.state, self.reason = state, reason

    # ------------------------------------------------------------ helpers
    def robot_pose(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def to_station(self, xy):
        """map (m) -> station map frame (mm), like the places / zones; None without that frame."""
        if not self.station_frame or xy is None:
            return None
        try:
            t = self.tf.lookup_transform(self.station_frame, self.map_frame, Time())
        except Exception:
            return None
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        x = t.transform.translation.x + c * xy[0] - s * xy[1]
        y = t.transform.translation.y + s * xy[0] + c * xy[1]
        return [round(x * 1000.0), round(y * 1000.0)]

    def area(self):
        if self.map is None:
            return 0.0
        g, res = self.map[0], self.map[1]
        return float(((g >= 0) & (g <= self.p.free_max)).sum()) * res * res

    def elapsed(self):
        t0 = self.stats["t0"]
        if t0 is None:
            return 0.0
        return (self.stats["t_end"] or time.monotonic()) - t0

    def station_ok(self):
        """-> None if Nav2 may drive the robot, else why not."""
        if not self.require_station:
            return None
        s = self.robot_state
        if s is None or time.monotonic() - s.get("_rx", 0) > 3.0:
            return "no robot state (is the adapter connected to the bridge?)"
        if s.get("estop") or s.get("robot_estop"):
            return "E-stop"
        if not s.get("armed") or s.get("mode") != "STATION":
            return "arm the robot and select Manual (STATION)"
        return None

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = time.monotonic()
        pose = self.robot_pose()
        if pose is not None and self.running:
            self.track(pose)
        if self.saving is not None:
            self.save_poll()
        if self.running:
            self.step(now, pose)
        self.publish_status(now)

    def track(self, pose):
        st = self.stats
        xy = (pose[0], pose[1])
        if st["last_xy"] is None:
            st["last_xy"] = xy
            st["track"].append([round(xy[0], 3), round(xy[1], 3)])
            if self.home is None:
                self.home = pose
            return
        d = math.hypot(xy[0] - st["last_xy"][0], xy[1] - st["last_xy"][1])
        if d >= 0.05:
            st["dist_m"] += d
            st["last_xy"] = xy
            st["track"].append([round(xy[0], 3), round(xy[1], 3)])

    def step(self, now, pose):
        if self.max_duration_s > 0 and self.elapsed() > self.max_duration_s:
            return self.finish("done", "time limit (%.0f s)" % self.max_duration_s)
        why = self.station_ok()
        if why is None and self.map is None:
            why = "waiting for /map (slam_toolbox)"
        if why is None and pose is None:
            why = "waiting for tf %s -> %s" % (self.map_frame, self.base_frame)
        if why is None and not self.nav.server_is_ready():
            why = "waiting for Nav2 (navigate_to_pose)"
        if why is not None:
            if self.goal is not None:
                self.cancel_goal()
            self.target = None
            self.set_state("waiting", why)
            return
        if self.state == "waiting":
            self.set_state("exploring", "looking for frontiers")
            self.last_plan = 0.0
        if self.state == "dwell":
            if now < self.dwell_until:
                return
            self.set_state("exploring", "looking for frontiers")
            self.last_plan = 0.0
        if self.state == "navigating":
            if self.goal is not None:
                return self.watch_goal(now, pose)
            self.set_state("exploring", "looking for frontiers")
        if self.state == "exploring" and now - self.last_plan >= self.plan_period and now >= self.retry_at:
            self.choose(now, pose)

    def plan(self, pose):
        g, res, origin, _ = self.map
        cost = None
        if self.cost is not None:
            cg, cres, corg = self.cost
            cost = fr.resample(cg, cres, corg, g.shape, res, origin, fill=-1)
        t = time.monotonic()
        fs, info = fr.find_frontiers(g, res, origin, pose[:2], cost=cost, blacklist=self.blacklist, p=self.p)
        dt = (time.monotonic() - t) * 1000.0
        st = self.stats
        st["plans"] += 1
        st["plan_ms"] = max(st["plan_ms"], dt)
        for f in fs:                                   # visited before and still there: a bit less attractive
            n = self.visit_count(f.point)
            if n:
                f.score -= 0.5 * n
        fs.sort(key=lambda f: -f.score)
        self.frontiers = fs
        if dt > 1500:
            self.get_logger().warn("frontier plan took %.0f ms" % dt, throttle_duration_sec=30.0)
        return fs, info

    def choose(self, now, pose):
        self.last_plan = now
        if self.stats["goals_ok"] + self.stats["goals_failed"] >= self.max_goals:
            return self.finish("done", "goal limit (%d)" % self.max_goals)
        fs, info = self.plan(pose)
        if not fs:
            self.target = None
            self.empty += 1
            if self.empty >= self.done_checks:
                if self.return_home and self.home is not None and not self.going_home and \
                        math.hypot(pose[0] - self.home[0], pose[1] - self.home[1]) > 0.3:
                    self.going_home = True
                    return self.send_goal(self.home[:2], self.home[2], None, "returning to the start")
                why = "no reachable frontier left" if info.get("reachable") else info.get("why", "no frontier")
                return self.finish("done", why)
            self.set_state("exploring", "checking for frontiers (%d/%d)" % (self.empty, self.done_checks))
            return
        self.empty = 0
        f = fs[0]
        yaw = math.atan2(f.point[1] - f.viewpoint[1], f.point[0] - f.viewpoint[0])
        if math.hypot(f.point[0] - f.viewpoint[0], f.point[1] - f.viewpoint[1]) < 0.05:
            yaw = math.atan2(f.point[1] - pose[1], f.point[0] - pose[0])
        self.send_goal(f.viewpoint, yaw, f, "to a frontier %.1f m away (%.1f m wide%s)" % (
            f.dist_m, f.size_m, ", opens up" if f.opens else ""))

    def send_goal(self, xy, yaw, f, reason):
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(xy[0]), float(xy[1])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        self.goal_seq += 1
        seq = self.goal_seq
        now = time.monotonic()
        dist = f.dist_m if f is not None else 3.0
        self.goal = {"seq": seq, "handle": None, "t0": now, "vp": tuple(xy), "point": f.point if f else None,
                     "timeout": max(self.goal_timeout_min, 20.0 + 2.5 * dist / self.speed_est),
                     "prog_xy": None, "prog_t": now, "f": f, "last_replan": now}
        self.target = f
        self.set_state("navigating", reason)
        self.get_logger().info("goal %d: (%.2f, %.2f) %s" % (seq, xy[0], xy[1], reason))
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda fu, s=seq: self.on_goal_response(fu, s))

    def on_goal_response(self, fut, seq):
        g = self.goal
        h = fut.result()
        if g is None or g["seq"] != seq:                # superseded meanwhile (Nav2 preempted it with the newer one)
            return
        if h is None or not h.accepted:
            # Nav2 not active yet (lifecycle still starting) or busy: try again shortly, no blacklist
            self.goal = None
            self.target = None
            self.rejects += 1
            self.retry_at = time.monotonic() + 2.0
            self.set_state("exploring", "Nav2 rejected the goal (still starting?), retrying")
            if self.rejects in (1, 10, 100):
                self.get_logger().warn("goal %d rejected by Nav2 (%d in a row), retrying" % (seq, self.rejects))
            return
        self.rejects = 0
        g["handle"] = h
        h.get_result_async().add_done_callback(lambda fu, s=seq: self.on_result(fu, s))

    def on_result(self, fut, seq):
        g = self.goal
        if g is None or g["seq"] != seq:
            return
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_reached()
        elif status == GoalStatus.STATUS_CANCELED:
            self.goal = None
            if self.running and self.state == "navigating":
                self.set_state("exploring", "looking for frontiers")
        else:
            self.goal_failed("Nav2: %s" % {GoalStatus.STATUS_ABORTED: "aborted"}.get(status, "status %d" % status))

    def visit_count(self, xy):
        for x, y, n in self.visits:
            if math.hypot(x - xy[0], y - xy[1]) < self.p.black_r:
                return n
        return 0

    def add_visit(self, xy):
        for i, (x, y, n) in enumerate(self.visits):
            if math.hypot(x - xy[0], y - xy[1]) < self.p.black_r:
                self.visits[i] = (x, y, n + 1)
                return n + 1
        self.visits.append((xy[0], xy[1], 1))
        return 1

    def log_goal(self, result):
        g = self.goal
        self.stats["goals"].append({"vp": [round(v, 3) for v in g["vp"]],
                                    "point": [round(v, 3) for v in g["point"]] if g["point"] else None,
                                    "result": result, "s": round(time.monotonic() - g["t0"], 1),
                                    "t": round(self.elapsed(), 1)})

    def goal_reached(self):
        g = self.goal
        self.stats["goals_ok"] += 1
        self.log_goal("reached")
        self.goal = None
        if self.going_home:
            return self.finish("done", "no reachable frontier left (back at the start)")
        if g["point"] is not None and self.add_visit(g["point"]) >= self.visits_max:
            self.blacklist.append(tuple(g["point"]))     # reached it twice and it is still a frontier: unobservable
        self.target = None
        self.dwell_until = time.monotonic() + self.dwell_s
        self.set_state("dwell", "scanning at the goal")

    def goal_failed(self, why):
        g = self.goal
        if g is None:
            return
        self.stats["goals_failed"] += 1
        self.log_goal("failed: " + why)
        if g["point"] is not None:
            self.blacklist.append(tuple(g["point"]))
        self.blacklist.append(tuple(g["vp"]))
        self.get_logger().warn("goal %d failed (%s): blacklisted (%.2f, %.2f)" % (g["seq"], why, g["vp"][0], g["vp"][1]))
        self.cancel_goal()
        self.target = None
        if self.going_home:
            return self.finish("done", "no reachable frontier left (could not return to the start)")
        self.set_state("exploring", "goal failed (%s), choosing another" % why)
        self.last_plan = 0.0

    def cancel_goal(self):
        g, self.goal = self.goal, None
        if g is not None and g["handle"] is not None:
            try:
                g["handle"].cancel_goal_async()
            except Exception:
                pass

    def watch_goal(self, now, pose):
        g = self.goal
        if now - g["t0"] > g["timeout"]:
            return self.goal_failed("timeout %.0f s" % g["timeout"])
        if g["prog_xy"] is None or math.hypot(pose[0] - g["prog_xy"][0], pose[1] - g["prog_xy"][1]) > 0.2:
            g["prog_xy"], g["prog_t"] = pose[:2], now
        elif now - g["prog_t"] > self.progress_s:
            return self.goal_failed("no progress for %.0f s" % self.progress_s)
        if g["point"] is None or now - g["last_replan"] < self.replan_period:
            return
        g["last_replan"] = now
        fs, _ = self.plan(pose)
        cur = None
        for f in fs:
            if math.hypot(f.point[0] - g["point"][0], f.point[1] - g["point"][1]) < self.p.black_r or \
                    math.hypot(f.viewpoint[0] - g["vp"][0], f.viewpoint[1] - g["vp"][1]) < 0.25:
                cur = f
                break
        if cur is None:
            # the target frontier is mapped already (or no longer reachable): next one, no need to get there
            self.stats["goals_ok"] += 1
            self.log_goal("seen")
            return self.replace_goal(now, pose, "target frontier mapped, choosing the next")
        self.target = cur
        if fs[0] is not cur and fs[0].score > cur.score + self.switch_margin:
            self.log_goal("switched")
            return self.replace_goal(now, pose, "a better frontier appeared")

    def replace_goal(self, now, pose, why):
        """Next goal without cancelling the current one first: a new NavigateToPose goal preempts it in Nav2
        (cancel + send back to back made bt_navigator abort the new goal). Cancel only when there is no next."""
        old, self.goal = self.goal, None
        self.target = None
        self.set_state("exploring", why)
        self.choose(now, pose)
        if self.goal is None and old is not None and old["handle"] is not None:
            try:
                old["handle"].cancel_goal_async()
            except Exception:
                pass

    # ------------------------------------------------------------ status
    def status(self):
        t = self.target
        tm = None if t is None else [round(t.viewpoint[0], 3), round(t.viewpoint[1], 3)]
        if self.goal is not None and tm is None:
            tm = [round(self.goal["vp"][0], 3), round(self.goal["vp"][1], 3)]
        return {"state": self.state, "target": self.to_station(tm), "target_map": tm,
                "frontiers": len(self.frontiers),
                "area_m2": round(self.area(), 2), "reason": self.reason, "running": self.running,
                "goals_ok": self.stats["goals_ok"], "goals_failed": self.stats["goals_failed"],
                "dist_m": round(self.stats["dist_m"], 2), "elapsed_s": round(self.elapsed(), 1),
                "saved": self.saved, "ok": self.save_ok}

    def publish_status(self, now):
        s = self.status()
        if self.saving is not None:
            s["state"] = "saving"
        key = json.dumps({k: v for k, v in s.items() if k not in ("elapsed_s", "dist_m", "area_m2")}, sort_keys=True)
        if key == self.last_status and now - self.last_pub < self.status_period:
            return
        self.last_status, self.last_pub = key, now
        self.pub.publish(String(data=json.dumps(s, separators=(",", ":"))))

    # ------------------------------------------------------------ save the map
    def save_map(self, name):
        if self.saving is not None:
            self.reason = "already saving %s" % self.saving["name"]
            return
        name = name or time.strftime("explore-%Y%m%d-%H%M")
        if not NAME_RE.match(name):
            self.saved, self.save_ok = name, False
            self.reason = "map name: letters, digits, _ and - only"
            return
        if self.map is None:
            self.saved, self.save_ok = name, False
            self.reason = "no /map to save"
            return
        try:
            os.makedirs(self.map_dir, exist_ok=True)
        except OSError as e:
            self.saved, self.save_ok, self.reason = name, False, "map dir: %s" % e
            return
        base = os.path.join(self.map_dir, name)
        g, res, origin, _ = self.map
        notes = []
        try:
            fr.write_map(base, g, res, origin)
            goals = [(x["vp"][0], x["vp"][1], not x["result"].startswith("failed")) for x in self.stats["goals"]]
            try:
                fr.map_png(base + ".png", g, res, origin, self.stats["track"], goals)
            except Exception as e:                     # no Pillow: the map files still count
                notes.append("no png (%s)" % e)
            with open(base + ".explore.json", "w") as f:
                json.dump({"name": name, "status": self.status(), "track": self.stats["track"],
                           "goals": self.stats["goals"], "blacklist": [list(b) for b in self.blacklist],
                           "plans": self.stats["plans"], "plan_ms_max": round(self.stats["plan_ms"], 1),
                           "resolution": res, "origin": list(origin)}, f)
        except OSError as e:
            self.saved, self.save_ok, self.reason = name, False, "save failed: %s" % e
            return
        from slam_toolbox.srv import SerializePoseGraph
        if self.ser is None:
            self.ser = self.create_client(SerializePoseGraph, "slam_toolbox/serialize_map")
        self.saving = {"name": name, "base": base, "t0": time.monotonic(), "fut": None, "notes": notes}
        self.get_logger().info("saving the map as %s" % base)

    def save_poll(self):
        from slam_toolbox.srv import SerializePoseGraph
        s = self.saving
        if s["fut"] is None:
            if self.ser.service_is_ready():
                rq = SerializePoseGraph.Request()
                rq.filename = s["base"]
                s["fut"] = self.ser.call_async(rq)
            elif time.monotonic() - s["t0"] > 5.0:
                return self.save_done(s, False, "no slam_toolbox serialize_map: grid only")
            return
        if s["fut"].done():
            r = s["fut"].result()
            ok = r is not None and getattr(r, "result", 1) == 0
            return self.save_done(s, ok, None if ok else "pose graph not saved: grid only")
        if time.monotonic() - s["t0"] > 30.0:
            return self.save_done(s, False, "slam_toolbox serialize_map timed out: grid only")

    def save_done(self, s, graph_ok, note):
        self.saving = None
        self.saved, self.save_ok = s["name"], True
        notes = s["notes"] + ([note] if note else [])
        self.reason = "map saved: %s%s" % (s["name"], " (%s)" % "; ".join(notes) if notes else "")
        self.get_logger().info("%s -> %s.{yaml,pgm,png%s}" % (self.reason, s["base"],
                                                              ",posegraph,data" if graph_ok else ""))


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    if node.autostart:
        node.set_running(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
