"""ROS inside the station (issue #106): the adapter's half. The station UI shows the ROS map, pose, plan and Nav2 status
and sends goals / places / patrols / cancel / initial pose / explore through the bridge; no RViz needed.

Attached to the adapter node (one WebSocket, one process). Everything ROS happens on the executor thread: commands
from the WebSocket thread wait in a queue for the 10 Hz tick. Messages: see stationlink.py.

Safety is unchanged: goals only make Nav2 publish /cmd_vel, which the adapter's gate forwards only while the robot is
armed in STATION (Manual). Goals are refused while it is not, and a running goal / patrol is cancelled when the robot
leaves Manual for longer than station_cancel_off_manual_s. Nothing here arms the robot or changes its mode.
"""
import json
import math
import queue
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_action_status_default
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from . import convert as cv
from . import stationlink as sl

ACTIVE = (1, 2, 3)           # GoalStatus ACCEPTED, EXECUTING, CANCELING
TERMINAL = (4, 5, 6)         # SUCCEEDED, CANCELED, ABORTED
OWN_KINDS = ("goal", "place", "patrol")
MANUAL_TEXT = "Switch to Manual to let ROS drive"


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf_pose(t):
    return t.transform.translation.x, t.transform.translation.y, quat_to_yaw(t.transform.rotation)


class StationNav:
    def __init__(self, node, send, robot_state, station_frame="station_map"):
        self.node = node
        self.send = send                      # dict -> bool (the adapter's WebSocket)
        self.robot_state = robot_state        # -> the newest ros_state dict (or None)
        P = node.declare_parameter
        self.map_frame = node.map_frame
        self.base_frame = node.base_frame
        self.station_frame = station_frame
        self.map_period = float(P("station_map_period_s", 2.0).value)
        self.pose_period = 1.0 / float(P("station_pose_hz", 5.0).value)
        self.off_manual_s = float(P("station_cancel_off_manual_s", 3.0).value)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, P("station_map_topic", "map").value, self.on_map, latched)
        node.create_subscription(Path, "plan", self.on_plan, 5)
        node.create_subscription(String, "localization_status", self.on_loc, 5)
        node.create_subscription(Float32, "localization_quality", self.on_quality, 5)
        node.create_subscription(String, "explore/status", self.on_explore, 10)
        node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self.on_goal_status,
                                 qos_profile_action_status_default)
        node.create_subscription(NavigateToPose.Impl.FeedbackMessage, "navigate_to_pose/_action/feedback",
                                 self.on_feedback, 10)
        self.pub_init = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 5)
        self.pub_explore = node.create_publisher(String, "station/explore_cmd", 5)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.cancel_clients = [node.create_client(CancelGoal, a + "/_action/cancel_goal")
                               for a in ("navigate_to_pose", "navigate_through_poses", "follow_waypoints")]
        self.bt_client = node.create_client(GetState, "bt_navigator/get_state")

        self.cmds = queue.Queue()
        self.places = []                      # station places (m, station_map) from the bridge's places message
        self.nav = sl.NavTracker()
        self.goal_handle = None
        self.goal_uuid = None
        self.own_uuids = []                   # every goal this node sent (their late status must not look external)
        self.goal_token = 0
        self.ext_uuid = None                  # a goal from elsewhere (RViz, go_to.ps1, patrol.ps1)
        self.off_manual_since = None
        self.map_in = None
        self.map_new = False
        self.map_digest = None
        self.map_seq = 0
        self.map_info = None
        self.map_t = -1e9
        self.plan_in = None
        self.plan_new = False
        self.plan_t = -1e9
        self.plan_shown = False
        self.pose_t = -1e9
        self.tf_ok = False
        self.T_live = None                    # map <- station_map now
        self.T_t = -1e9
        self.T_good = None                    # ... the last one while the ROS pose was good (initial pose from the station)
        self.loc = None
        self.loc_t = -1e9
        self.quality = None
        self.explore_t = -1e9
        self.bt_state = None
        self.bt_t = -1e9
        self.status_t = -1e9
        self.nav_sent_seq = -1
        self.nav_t = -1e9
        node.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ helpers
    def now(self):
        return time.monotonic()

    def lookup(self, target, source):
        try:
            return tf_pose(self.tf.lookup_transform(target, source, Time()))
        except Exception:
            return None

    def reply(self, m, ok, text=""):
        self.send({"t": "ros_reply", "id": m.get("id"), "cmd": m.get("cmd"), "ok": bool(ok), "text": text})
        if not ok:
            self.node.get_logger().warn("station %s: %s" % (m.get("cmd"), text))

    def drivable(self):
        st = self.robot_state()
        ok, why = cv.drivable(st, self.node.ros_now())
        if not ok and st and st.get("armed") and st.get("mode") not in cv.DRIVE_MODES and not st.get("estop"):
            why = "%s (the robot is in %s)" % (MANUAL_TEXT, st.get("mode"))
        return ok, why

    # ------------------------------------------------------------ inputs (WebSocket thread)
    def command(self, m):
        self.cmds.put(m)

    def on_places(self, m):
        self.places = cv.places_from_msg(m)

    def on_connect(self):
        """(Re)connected to the bridge: everything goes out again."""
        self.map_digest = None
        self.map_new = self.map_in is not None
        self.map_t = -1e9
        self.nav_sent_seq = -1
        self.plan_new = True

    # ------------------------------------------------------------ inputs (ROS)
    def on_map(self, m):
        self.map_in = m
        self.map_new = True

    def on_plan(self, m):
        self.plan_in = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        self.plan_new = True

    def on_loc(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.loc = {"state": d.get("state"), "quality": d.get("quality")}
        self.loc_t = self.now()

    def on_quality(self, m):
        self.quality = round(float(m.data), 3)

    def on_explore(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if not isinstance(d, dict):
            return
        self.explore_t = self.now()
        self.send(sl.explore_fill(d, self.T_live))

    def on_goal_status(self, m):
        by_id = {bytes(s.goal_info.goal_id.uuid): s.status for s in m.status_list}
        if self.ext_uuid is not None and by_id.get(self.ext_uuid) in TERMINAL:
            if self.nav.kind == "external" and self.nav.busy():
                self.nav.finish(sl.NAV2_RESULT.get(by_id[self.ext_uuid], "failed"), now=self.now())
                self.clear_plan()
            self.ext_uuid = None
        if self.nav.busy() and self.nav.kind in OWN_KINDS:
            return
        for uid, st in by_id.items():
            if st in ACTIVE and st != 3 and uid not in self.own_uuids and uid != self.ext_uuid:
                self.ext_uuid = uid
                self.nav.start("external", None, now=self.now())
                break

    def on_feedback(self, m):
        uid = bytes(m.goal_id.uuid)
        if uid not in (self.goal_uuid, self.ext_uuid) or not self.nav.busy():
            return
        f = m.feedback
        eta = f.estimated_time_remaining.sec + f.estimated_time_remaining.nanosec * 1e-9
        self.nav.feedback(f.distance_remaining, eta, f.number_of_recoveries)

    # ------------------------------------------------------------ commands
    def run_command(self, m):
        c = m.get("cmd")
        try:
            if c == "goal":
                frame, x, y, yaw = sl.parse_pose(m)
                if frame == "station":
                    T = self.T_live
                    if T is None:
                        return self.reply(m, False, "no transform %s <- %s yet" % (self.map_frame, self.station_frame))
                    x, y = sl.station_to_map(T, x, y)
                    yaw = None if yaw is None else cv.wrap_pi(yaw + T[2])
                if not self.check_drive(m):
                    return
                if yaw is None:
                    p = self.lookup(self.map_frame, self.base_frame)
                    yaw = math.atan2(y - p[1], x - p[0]) if p and math.hypot(x - p[0], y - p[1]) > 0.05 else (p[2] if p else 0.0)
                self.nav_to(m, x, y, yaw, "goal")
            elif c == "place":
                if not self.check_drive(m):
                    return
                r = self.resolve(m.get("name"))
                if isinstance(r, str):
                    return self.reply(m, False, r)
                name, x, y = r
                p = self.lookup(self.map_frame, self.base_frame)
                yaw = cv.route_yaws([(x, y)], start=p[:2] if p else None)[0]
                self.nav_to(m, x, y, yaw, "place", place=name)
            elif c == "patrol":
                names, loops, dwell = sl.parse_patrol(m)
                if not self.check_drive(m):
                    return
                self.start_patrol(m, names, loops, dwell)
            elif c == "cancel":
                self.cancel("cancelled from the station")
                self.reply(m, True, "cancelled")
            elif c == "initial_pose":
                frame, x, y, yaw = sl.parse_pose(m)
                if yaw is None:
                    return self.reply(m, False, "initial_pose needs a heading (yaw)")
                if frame == "station":
                    T = self.T_good or self.T_live
                    if T is None:
                        return self.reply(m, False, "ROS and the station have not been aligned yet: set the pose on the "
                                                    "ROS map instead")
                    x, y, yaw = sl.station_to_map(T, x, y, yaw)
                self.publish_initial(x, y, yaw)
                self.reply(m, True, "initial pose sent: (%.2f, %.2f) m, %.0f deg" % (x, y, math.degrees(yaw)))
            elif c in ("explore", "save_map"):
                if self.pub_explore.get_subscription_count() == 0:
                    return self.reply(m, False, "the ROS explorer is not running")
                d = {"cmd": c}
                if c == "explore":
                    d["run"] = bool(m.get("run", True))
                    if d["run"] and not self.check_drive(m):
                        return
                else:
                    d["name"] = sl.clean_map_name(m.get("name"))
                self.pub_explore.publish(String(data=json.dumps(d)))
                self.reply(m, True, "sent to the explorer")
            else:
                self.reply(m, False, "unknown ROS command %r" % c)
        except (KeyError, TypeError, ValueError) as e:
            self.reply(m, False, "bad %s: %s" % (c, e))

    def check_drive(self, m):
        ok, why = self.drivable()
        if not ok:
            self.reply(m, False, why)
        return ok

    def resolve(self, name):
        """station place -> (name, x, y) in the ROS map (TF map <- station_map, as places.py), or an error text."""
        p = cv.find_place(self.places, name)
        if p is None:
            return "no place called %r (places: %s)" % (name, ", ".join(q["name"] for q in self.places) or "none")
        T = self.T_live or self.lookup(self.map_frame, self.station_frame)
        if T is None:
            return "no transform %s <- %s (is localization up?)" % (self.map_frame, self.station_frame)
        x, y = sl.station_to_map(T, p["x"] * 1000.0, p["y"] * 1000.0)
        return p["name"], x, y

    def nav_to(self, m, x, y, yaw, kind, place=None, keep_patrol=False):
        if not self.nav_client.server_is_ready():
            if m is not None:
                self.reply(m, False, "Nav2 is not running (no navigate_to_pose server)")
            if keep_patrol:
                self.nav.finish("failed", "Nav2 went away", self.now())
            return False
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.map_frame
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        q = cv.quat_yaw(yaw)
        o = g.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        self.goal_token += 1
        tok = self.goal_token
        old = self.goal_handle
        self.goal_handle = self.goal_uuid = None
        if old is not None:
            old.cancel_goal_async()
        self.nav.start(kind, (x, y, yaw), place, self.now(), keep_patrol=keep_patrol)
        self.off_manual_since = None
        self.ext_uuid = None
        fut = self.nav_client.send_goal_async(g)
        fut.add_done_callback(lambda f: self.on_accepted(f, tok))
        if m is not None:
            self.reply(m, True, "%s (%.2f, %.2f) sent to Nav2" % (place or kind, x, y))
        self.node.get_logger().info("station %s: %s (%.2f, %.2f, %.0f deg)" % (
            kind, place or "", x, y, math.degrees(yaw)))
        return True

    def on_accepted(self, f, tok):
        h = f.result()
        if h is not None and h.accepted:
            self.own_uuids = self.own_uuids[-50:] + [bytes(h.goal_id.uuid)]
        if tok != self.goal_token:                       # superseded / cancelled meanwhile
            if h is not None and h.accepted:
                h.cancel_goal_async()
            return
        if h is None or not h.accepted:
            self.goal_done("failed", "Nav2 rejected the goal")
            return
        self.goal_handle = h
        self.goal_uuid = bytes(h.goal_id.uuid)
        h.get_result_async().add_done_callback(lambda r: self.on_result(r, tok))

    def on_result(self, f, tok):
        if tok != self.goal_token:
            return
        r = f.result()
        self.goal_handle = self.goal_uuid = None
        if r is None:
            return self.goal_done("failed", "no result from Nav2")
        res = r.result
        reason = sl.nav2_reason(getattr(res, "error_code", 0), getattr(res, "error_msg", ""))
        self.goal_done(sl.NAV2_RESULT.get(r.status, "failed"), reason)

    def goal_done(self, outcome, reason=""):
        now = self.now()
        p = self.nav.patrol
        if p is not None and self.nav.kind == "patrol" and outcome != "cancelled":
            name = p["places"][p["i"]]
            if outcome == "succeeded":
                self.nav.dwell(now + p["dwell"])
                self.node.get_logger().info("patrol: at %s, waiting %.0f s" % (name, p["dwell"]))
            else:
                p["missed"].append([p["loop"], name, reason or outcome])
                self.node.get_logger().warn("patrol: missed %s (%s)" % (name, reason or outcome))
                self.patrol_advance()
            return
        self.nav.finish(outcome, reason, now)
        self.clear_plan()

    def start_patrol(self, m, names, loops, dwell):
        pts = []
        for n in names:
            r = self.resolve(n)
            if isinstance(r, str):
                return self.reply(m, False, r)
            pts.append(r)
        xy = [(x, y) for _, x, y in pts]
        rp = self.lookup(self.map_frame, self.base_frame)
        yaws = cv.route_yaws(xy, start=rp[:2] if rp else None)
        later = list(yaws)
        if len(xy) > 1:
            later[0] = cv.route_yaws([xy[-1], xy[0]])[1]
        self.nav.patrol = {"places": [n for n, _, _ in pts], "i": 0, "loop": 0, "loops": loops, "dwell": dwell,
                           "missed": [], "dwell_end": None, "xy": xy, "yaws": yaws, "later": later}
        if self.patrol_go(m):
            self.node.get_logger().info("patrol: %s, %s, dwell %.0f s" % (
                " -> ".join(self.nav.patrol["places"]), "forever" if loops <= 0 else "%d loop(s)" % loops, dwell))

    def patrol_go(self, m=None):
        p = self.nav.patrol
        i = p["i"]
        yaw = (p["later"] if p["loop"] else p["yaws"])[i]
        x, y = p["xy"][i]
        return self.nav_to(m, x, y, yaw, "patrol", place=p["places"][i], keep_patrol=True)

    def patrol_advance(self):
        p = self.nav.patrol
        nxt = sl.patrol_next(p)
        if nxt is None:
            miss = len(p["missed"])
            self.nav.finish("succeeded" if not miss or miss < len(p["places"]) * max(1, p["loop"] + 1) else "failed",
                            "missed %d" % miss if miss else "", self.now())
            self.clear_plan()
            return
        p["i"], p["loop"] = nxt
        p["dwell_end"] = None
        self.patrol_go()

    def cancel(self, reason):
        self.goal_token += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = self.goal_uuid = None
        for c in self.cancel_clients:                    # zero goal id + zero stamp = every goal of that action
            if c.service_is_ready():
                c.call_async(CancelGoal.Request())
        if self.nav.busy():
            self.nav.finish("cancelled", reason, self.now())
        self.clear_plan()

    def publish_initial(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        q = cv.quat_yaw(yaw)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        c = [0.0] * 36
        c[0] = c[7] = 0.25 ** 2
        c[35] = math.radians(10.0) ** 2
        m.pose.covariance = c
        self.pub_init.publish(m)
        self.node.get_logger().info("station: initial pose (%.2f, %.2f, %.0f deg)" % (x, y, math.degrees(yaw)))

    def clear_plan(self):
        self.plan_in = []
        self.plan_new = True

    # ------------------------------------------------------------ periodic (executor thread)
    def tick(self):
        now = self.now()
        while True:
            try:
                m = self.cmds.get_nowait()
            except queue.Empty:
                break
            try:
                self.run_command(m)
            except Exception as e:                       # never let a station command take the adapter down
                self.reply(m, False, "%s: %s" % (type(e).__name__, e))
        if now - self.pose_t >= self.pose_period:
            self.pose_t = now
            self.send_pose()
        if self.map_new and now - self.map_t >= self.map_period:
            self.send_map(now)
        if self.plan_new and now - self.plan_t >= 1.0:
            self.plan_new = False
            self.plan_t = now
            pts = self.plan_in or []
            if pts or self.plan_shown:
                if not self.nav.busy():
                    pts = []
                self.plan_shown = bool(pts)
                self.send(sl.plan_msg(pts))
        if self.nav.state == "dwell" and self.nav.patrol and now >= (self.nav.patrol.get("dwell_end") or 0):
            self.patrol_advance()
        self.watch_manual(now)
        if self.nav.seq != self.nav_sent_seq or (self.nav.busy() and now - self.nav_t >= 1.0) or now - self.nav_t >= 5.0:
            if self.send(self.nav.msg(now)):
                self.nav_sent_seq = self.nav.seq
                self.nav_t = now
        if now - self.bt_t >= 3.0:
            self.bt_t = now
            self.poll_bt()
        if now - self.status_t >= 1.0:
            self.status_t = now
            self.send(self.status_msg(now))

    def watch_manual(self, now):
        if not (self.nav.busy() and self.nav.kind in OWN_KINDS):
            self.off_manual_since = None
            return
        ok, why = self.drivable()
        if ok:
            self.off_manual_since = None
            return
        if self.off_manual_since is None:
            self.off_manual_since = now
        elif now - self.off_manual_since >= self.off_manual_s:
            self.node.get_logger().warn("station goal cancelled: %s" % why)
            self.cancel("stopped: " + why)

    def send_pose(self):
        p = self.lookup(self.map_frame, self.base_frame)
        self.tf_ok = p is not None
        T = self.lookup(self.map_frame, self.station_frame) if self.station_frame else None
        now = self.now()
        if T is not None:
            self.T_t = now
        elif self.T_live is not None and now - self.T_t < 5.0:
            T = self.T_live                              # one missed lookup (TF hiccup): keep the last one a while
        self.T_live = T
        loc_state = self.loc["state"] if self.loc and self.now() - self.loc_t < 3.0 else None
        if T is not None and loc_state in (None, "ok"):
            self.T_good = T
        if p is None:
            return
        self.send({"t": "ros_pose", "p": [round(p[0], 3), round(p[1], 3), round(p[2], 4)],
                   "T": None if T is None else [round(T[0], 4), round(T[1], 4), round(T[2], 5)]})

    def send_map(self, now):
        m = self.map_in
        i = m.info
        origin = (i.origin.position.x, i.origin.position.y, quat_to_yaw(i.origin.orientation))
        dig = sl.grid_digest(i.width, i.height, i.resolution, origin, m.data)
        self.map_new = False
        if dig == self.map_digest:
            return
        t0 = time.perf_counter()
        msg = sl.map_msg(i.width, i.height, i.resolution, origin, m.data, self.map_seq + 1,
                         frame=m.header.frame_id or self.map_frame)
        if self.send(msg):
            self.map_seq += 1
            self.map_digest = dig
            self.map_t = now
            self.map_info = {"w": i.width, "h": i.height, "res": round(i.resolution, 4), "seq": self.map_seq,
                             "kb": round(len(msg["rle"]) / 1024.0, 1),
                             "ms": round(1000 * (time.perf_counter() - t0), 1)}
        else:
            self.map_new = True                          # not connected: try again later

    def poll_bt(self):
        if not self.bt_client.service_is_ready():
            self.bt_state = None
            return

        def done(f):
            try:
                self.bt_state = f.result().current_state.label
            except Exception:
                self.bt_state = None
        self.bt_client.call_async(GetState.Request()).add_done_callback(done)

    def status_msg(self, now):
        try:
            names = set(self.node.get_node_names())
        except Exception:
            names = set()
        loc = self.loc if self.loc and now - self.loc_t < 3.0 else None
        if loc is not None and loc.get("quality") is None and self.quality is not None:
            loc = dict(loc, quality=self.quality)
        ok, why = self.drivable()
        return {"t": "ros_status", "v": sl.LINK_VERSION,
                "loc": loc, "tf": self.tf_ok, "aligned": self.T_live is not None,
                "nodes": {"nav2": self.bt_state, "nav_server": self.nav_client.server_is_ready(),
                          "slam": "slam_toolbox" in names, "amcl": "amcl" in names,
                          "map_server": "map_server" in names, "loc_monitor": "loc_monitor" in names,
                          "explore": now - self.explore_t < 10.0},
                "map": self.map_info, "drivable": ok, "why": why}
