"""sbr3_ros adapter: the SBR3 bridge's ROS stream (WebSocket) <-> ROS 2 topics (issue #96).

Publishes /scan (LaserScan, laser frame), /odom (Odometry), /tf odom -> base_link, /tf_static base_link -> laser,
/imu (Imu: pitch / roll / heading + rates), /robot_state (std_msgs/String, JSON), /battery (BatteryState),
and with use_bridge_correction:=true /tf map -> odom from the bridge's own LD19 scan matcher.
Station data (#98): /tf odom -> station_map (the station's map frame, from the bridge's correction), and latched
/station/places and /station/zones (std_msgs/String, JSON as the bridge sends them, station_map frame).
Subscribes /cmd_vel (Twist) -> station cmd_vel, only while the robot is armed in STATION mode, with a deadman.
Station link (#106, station_link:=true): the ROS map / pose / Nav2 plan + status go to the bridge for the station UI,
and the station's goals / places / patrols / cancel / initial pose / explore come back (station_nav.py).
It never arms the robot, never changes its mode and never releases an E-stop.

  ros2 run sbr3_ros adapter --ros-args -p url:=ws://127.0.0.1:8080/ws
"""
import json
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

import websocket

from . import convert as cv

DEFAULT_URL = "ws://192.168.0.209:8080/ws"


class Adapter(Node):
    def __init__(self):
        super().__init__("sbr3_adapter")
        P = self.declare_parameter
        self.url = P("url", DEFAULT_URL).value
        self.odom_frame = P("odom_frame", "odom").value
        self.base_frame = P("base_frame", "base_link").value
        self.laser_frame = P("laser_frame", "laser").value
        self.map_frame = P("map_frame", "map").value
        self.publish_odom_tf = P("publish_odom_tf", True).value
        self.use_bridge_correction = P("use_bridge_correction", False).value
        self.max_tilt_deg = float(P("max_tilt_deg", 6.0).value)
        self.cmd_rate_hz = float(P("cmd_rate_hz", 20.0).value)
        self.enable_cmd_vel = P("enable_cmd_vel", True).value
        self.gate = cv.CmdGate(max_speed=float(P("max_speed", 0.25).value), max_turn=float(P("max_turn", 1.0).value),
                               max_accel=float(P("max_accel", 0.4).value),
                               max_turn_accel=float(P("max_turn_accel", 2.0).value),
                               deadman_s=float(P("deadman_s", 0.5).value))
        self.laser_height_override = float(P("laser_height_m", -1.0).value)   # < 0: from the bridge's lidar.json
        self.station_frame = P("station_frame", "station_map").value          # "" = no station frame / places / zones

        self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_imu = self.create_publisher(Imu, "imu", 20)
        self.pub_state = self.create_publisher(String, "robot_state", 5)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 5)
        self.tfb = TransformBroadcaster(self)
        self.static_tfb = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_places = self.create_publisher(String, "station/places", latched)
        self.pub_zones = self.create_publisher(String, "station/zones", latched)
        self.station_corr = None                         # newest bridge correction (station map <- odom)
        self.station_seen = {"places": None, "zones": None}
        self.station_nav = None
        if self.station_frame and P("station_link", True).value:
            from .station_nav import StationNav          # (Nav2 / tf2 imports only when it is on)
            self.station_nav = StationNav(self, self.send_station, self.robot_state, self.station_frame)

        self.clock_sync = cv.ClockSync()
        self.hello = None
        self.layout = None
        self.state = None
        self.ws = None
        self.lock = threading.Lock()
        self.n = {"scans": 0, "tilted": 0, "odom": 0, "cmd_sent": 0, "cmd_ignored": 0}
        self.last_why = None
        self.last_corr = None
        self.create_timer(1.0 / self.cmd_rate_hz, self.cmd_tick)
        self.create_timer(10.0, self.report)
        self.ws_thread = threading.Thread(target=self.ws_loop, daemon=True)
        self.ws_thread.start()
        self.get_logger().info("connecting to %s (cmd_vel %s, bridge correction %s)" % (
            self.url, "on" if self.enable_cmd_vel else "off", "on" if self.use_bridge_correction else "off"))

    # ------------------------------------------------------------ time
    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, ts):
        t = self.clock_sync.to_ros(ts)
        if t is None:
            t = self.ros_now()
        sec, ns = cv.split_stamp(t)
        return Time(sec=sec, nanosec=ns)

    # ------------------------------------------------------------ WebSocket
    def ws_loop(self):
        while rclpy.ok():
            try:
                ws = websocket.WebSocketApp(self.url, on_open=self.on_open, on_message=self.on_message,
                                            on_error=self.on_error, on_close=self.on_close)
                self.ws = ws
                ws.run_forever(ping_interval=5, ping_timeout=3, suppress_origin=True)
            except Exception as e:                       # never let the thread die
                self.get_logger().warn("websocket: %s" % e)
            self.ws = None
            with self.lock:
                self.state = None                        # no state: cmd_vel is ignored until we are back
            time.sleep(2.0)

    def on_open(self, ws):
        self.get_logger().info("connected to the bridge")
        self.hello = None                                # this bridge's ros_hello follows
        ws.send(json.dumps({"t": "hi", "name": "ros2"}))
        ws.send(json.dumps({"t": "ros", "action": "subscribe"}))
        if self.station_nav is not None:
            self.station_nav.on_connect()

    def on_error(self, ws, err):
        self.get_logger().warn("bridge websocket: %s" % err, throttle_duration_sec=10.0)

    def on_close(self, ws, code, msg):
        self.get_logger().warn("bridge connection closed (%s %s), retrying" % (code, msg))

    def send(self, m):
        ws = self.ws
        if ws is None or not ws.sock or not ws.sock.connected:
            return False
        try:
            ws.send(json.dumps(m))
            return True
        except Exception:
            return False

    def send_station(self, m):
        """Station link messages (#106): only to a bridge that relays them (ros_hello "station_link"); an older one would
        take them for commands."""
        if not (self.hello or {}).get("station_link"):
            return False
        return self.send(m)

    def on_message(self, ws, text):
        try:
            m = json.loads(text)
        except ValueError:
            return
        t = m.get("t")
        if "now" in m and t in ("ros_scan", "ros_odom", "ros_state", "ros_hello"):
            self.clock_sync.update(m["now"], self.ros_now())
        try:
            if t == "ros_scan":
                self.on_scan(m)
            elif t == "ros_odom":
                self.on_odom(m)
            elif t == "ros_state":
                self.on_state(m)
            elif t == "ros_hello":
                self.on_hello(m)
            elif t in ("places", "zones") and self.station_frame:
                self.on_station(t, m)
                if t == "places" and self.station_nav is not None:
                    self.station_nav.on_places(m)
            elif t == "ros_cmd" and self.station_nav is not None:
                self.station_nav.command(m)
            elif t == "error":
                self.get_logger().warn("bridge: %s" % m.get("text"))
        except Exception as e:
            self.get_logger().error("%s: %s: %s" % (t, type(e).__name__, e), throttle_duration_sec=5.0)

    # ------------------------------------------------------------ stream -> ROS
    def on_hello(self, m):
        if not m.get("subscribed"):
            return
        self.hello = m
        if not m.get("lidar"):
            self.get_logger().warn("the bridge has no lidar: /scan stays silent")
        self.layout = cv.scan_layout(m) if m.get("scan") else None
        x, y, z, yaw = cv.laser_mount(m)
        if self.laser_height_override >= 0:
            z = self.laser_height_override
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.laser_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        q = cv.quat_yaw(yaw)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.static_tfb.sendTransform(tf)
        self.get_logger().info("ROS stream v%s: %d bins, laser at (%.3f, %.3f, %.3f) m" % (
            m.get("v"), self.layout["bins"] if self.layout else 0, x, y, z))

    def on_scan(self, m):
        L = self.layout
        if L is None:
            return
        self.n["scans"] += 1
        if not cv.keep_scan(m, self.max_tilt_deg):
            self.n["tilted"] += 1
            return
        s = LaserScan()
        s.header.stamp = self.stamp(m["ts"])
        s.header.frame_id = self.laser_frame
        s.angle_min, s.angle_max, s.angle_increment = L["angle_min"], L["angle_max"], L["angle_increment"]
        s.time_increment = 0.0                        # deskewed by the bridge: every range is at the stamp
        s.scan_time = L["scan_time"]
        s.range_min, s.range_max = L["range_min"], L["range_max"]
        s.ranges = cv.scan_ranges(m, L["range_min"], L["range_max"])
        self.pub_scan.publish(s)
        if m.get("corr") is not None:
            self.station_corr = m["corr"]
        if self.use_bridge_correction and m.get("corr") is not None:
            self.publish_corr(m["corr"], m["ts"])

    def publish_corr(self, corr, ts):
        x, y, yaw = cv.corr_to_tf(corr)
        tf = TransformStamped()
        tf.header.stamp = self.stamp(ts + 0.1)        # a little ahead, like slam_toolbox's transform_timeout
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y = x, y
        q = cv.quat_yaw(yaw)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tfb.sendTransform(tf)
        self.last_corr = corr

    def on_odom(self, m):
        cols = (self.hello or {}).get("odom", {}).get("cols")
        if not cols:
            return
        tfs = []
        for r in cv.odom_rows(m, cols):
            self.n["odom"] += 1
            st = self.stamp(r["t"])
            q = cv.quat_yaw(r["yaw"])
            o = Odometry()
            o.header.stamp = st
            o.header.frame_id = self.odom_frame
            o.child_frame_id = self.base_frame
            o.pose.pose.position.x, o.pose.pose.position.y = r["x"], r["y"]
            o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
            o.twist.twist.linear.x = r["v"]
            o.twist.twist.angular.z = r["w"]
            pc = [0.0] * 36
            pc[0] = pc[7] = 0.01 ** 2                 # wheel odometry: ~1 cm, ~2 deg; z / roll / pitch unused
            pc[14] = pc[21] = pc[28] = 1e6
            pc[35] = math.radians(2.0) ** 2
            o.pose.covariance = pc
            tc = [0.0] * 36
            tc[0], tc[7], tc[14], tc[21], tc[28], tc[35] = 0.02 ** 2, 1e-4, 1e6, 1e6, 1e6, math.radians(3.0) ** 2
            o.twist.covariance = tc
            self.pub_odom.publish(o)
            imu = Imu()
            imu.header.stamp = st
            imu.header.frame_id = self.base_frame
            qi = cv.quat_from_rpy(r["roll"], r["pitch"], r["yaw"])
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qi
            imu.orientation_covariance = [math.radians(1.0) ** 2, 0.0, 0.0, 0.0, math.radians(1.0) ** 2, 0.0,
                                          0.0, 0.0, math.radians(3.0) ** 2]
            imu.angular_velocity.y = r["pitch_rate"]
            imu.angular_velocity.z = r["yaw_rate"]
            imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
            imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8     # not measured
            self.pub_imu.publish(imu)
            if self.publish_odom_tf:
                tf = TransformStamped()
                tf.header.stamp = st
                tf.header.frame_id = self.odom_frame
                tf.child_frame_id = self.base_frame
                tf.transform.translation.x, tf.transform.translation.y = r["x"], r["y"]
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
                tfs.append(tf)
        if tfs and self.station_frame:
            tfs.append(self.station_tf(tfs[-1].header.stamp))
        if tfs:
            self.tfb.sendTransform(tfs)
        if self.use_bridge_correction and self.last_corr is not None and tfs:
            self.publish_corr(self.last_corr, cv.odom_rows(m, cols)[-1]["t"])

    def on_state(self, m):
        m["_rx"] = self.ros_now()
        with self.lock:
            self.state = m
        self.pub_state.publish(String(data=json.dumps({k: v for k, v in m.items() if not k.startswith("_")})))
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = float(m.get("battery_v") or 0.0)
        b.present = b.voltage > 1.0
        b.percentage = float("nan")
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.pub_batt.publish(b)
        if self.use_bridge_correction and m.get("corr") is not None:
            self.last_corr = m["corr"]
        if m.get("corr") is not None:
            self.station_corr = m["corr"]

    def robot_state(self):
        with self.lock:
            return self.state

    # ------------------------------------------------------------ station places / zones (#98)
    def station_tf(self, stamp):
        """odom -> station_map from the bridge's correction (identity until the first one arrives)."""
        x, y, yaw = cv.station_frame_tf(self.station_corr)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.station_frame
        tf.transform.translation.x, tf.transform.translation.y = x, y
        q = cv.quat_yaw(yaw)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        return tf

    def on_station(self, t, m):
        """The bridge sends places / zones to every client on connect and after each change: republish (latched)
        when they changed. The points stay in the station's map frame (mm), as sent."""
        m = {k: v for k, v in m.items() if k in ("t", "map", "items", "undo")}
        m["frame"] = self.station_frame
        text = json.dumps(m, separators=(",", ":"))
        if text == self.station_seen[t]:
            return
        self.station_seen[t] = text
        (self.pub_places if t == "places" else self.pub_zones).publish(String(data=text))
        self.get_logger().info("station %s: %d on map %s" % (t, len(m.get("items") or []), m.get("map")))

    # ------------------------------------------------------------ cmd_vel -> station
    def on_cmd_vel(self, tw):
        with self.lock:
            self.gate.on_twist(tw.linear.x, tw.angular.z, self.ros_now())

    def cmd_tick(self):
        if not self.enable_cmd_vel:
            return
        with self.lock:
            out = self.gate.step(self.ros_now(), self.state)
            why = self.gate.why
            fresh = self.gate.t_cmd is not None and self.ros_now() - self.gate.t_cmd <= self.gate.deadman_s
        if why and fresh:
            self.n["cmd_ignored"] += 1
            if why != self.last_why:
                self.get_logger().warn("ignoring /cmd_vel: %s" % why)
        self.last_why = why if fresh else None
        if out is not None and self.send(out):
            self.n["cmd_sent"] += 1

    def report(self):
        st = (self.state or {}).get("stats", {})
        self.get_logger().info("scans %d (tilted %d), odom %d, cmd sent %d / ignored %d; bridge stream %.1f KB/s, "
                               "%.2f %% CPU, %.2f ms/scan; clock offset %s" % (
                                   self.n["scans"], self.n["tilted"], self.n["odom"], self.n["cmd_sent"],
                                   self.n["cmd_ignored"], st.get("kbps", 0), st.get("cpu_pct", 0),
                                   st.get("scan_ms", 0), "%.3f" % self.clock_sync.offset
                                   if self.clock_sync.offset is not None else "?"))


def main(args=None):
    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send({"t": "cmd_vel", "speed": 0, "turn": 0})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
