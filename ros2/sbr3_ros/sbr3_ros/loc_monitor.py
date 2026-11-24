"""Localization monitor (issue #98): how well does the current pose explain the scan, and find the robot when it does not.

  ros2 run sbr3_ros loc_monitor                                  global relocalization at startup, kidnap recovery
  ros2 run sbr3_ros loc_monitor --ros-args -p initial_pose:="0.0,0.0,0.0"     start from this pose instead

- /localization_quality (std_msgs/Float32, 2 Hz): share of the scan's end points within 10 cm of a wall of /map at
  the current map -> laser pose (1.0 = perfect; a room with furniture that moved: ~0.7; lost: < 0.3);
- /localization_status (std_msgs/String, JSON): quality, state (waiting / searching / ok / lost), last relocalization;
- startup: no initial_pose -> a global scan match over the whole saved map (scanfit.global_search) -> /initialpose,
  which AMCL (or slam_toolbox localization) takes as its start;
- kidnapped / lost: quality < lost_fit for lost_s -> global match again; the new pose is used only when it explains
  the scan clearly better than the current one (and well: >= good_fit), at most every cooldown_s.
It never drives the robot.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from . import scanfit as sf
from .convert import quat_yaw


def tf_pose(t):
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


class LocMonitor(Node):
    def __init__(self):
        super().__init__("loc_monitor")
        P = self.declare_parameter
        self.global_frame = P("global_frame", "map").value
        self.base_frame = P("base_frame", "base_link").value
        self.initial_pose = P("initial_pose", "").value          # "x,y,yaw" (m, m, rad); "" = global search
        self.auto_init = P("auto_init", True).value
        self.auto_recover = P("auto_recover", True).value
        self.good_fit = float(P("good_fit", 0.6).value)
        self.lost_fit = float(P("lost_fit", 0.35).value)
        self.lost_s = float(P("lost_s", 3.0).value)
        self.margin = float(P("margin", 0.2).value)               # new pose must beat the current fit by this much
        self.cooldown_s = float(P("cooldown_s", 8.0).value)
        self.tol = float(P("tol", 0.10).value)
        self.rate = float(P("rate_hz", 2.0).value)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.create_subscription(LaserScan, "scan", self.on_scan, 5)
        self.pub_q = self.create_publisher(Float32, "localization_quality", 5)
        self.pub_s = self.create_publisher(String, "localization_status", 5)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 5)
        self.field = None
        self.scan = None
        self.laser_off = None                                      # base_link -> laser (x, y, yaw)
        self.state = "waiting"
        self.started = False
        self.bad_since = None
        self.last_reloc = None                                     # (wall time, info dict)
        self.last_try = -1e9
        self.quality = None
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info("loc_monitor: %s" % ("initial pose %s" % self.initial_pose if self.initial_pose
                                                   else "global relocalization at start"))

    # ------------------------------------------------------------ inputs
    def on_map(self, m):
        t0 = time.time()
        first = self.field is None
        self.field = sf.MapField.from_msg(m)
        if first:
            self.get_logger().info("map %dx%d at %.2f m, %d wall cells (%.0f ms)" % (
                m.info.width, m.info.height, m.info.resolution, self.field.n_occ, 1000 * (time.time() - t0)))

    def on_scan(self, s):
        self.scan = s
        if self.laser_off is None:
            try:
                self.laser_off = tf_pose(self.tf.lookup_transform(self.base_frame, s.header.frame_id, Time()))
            except Exception:
                pass

    def scan_pts(self):
        s = self.scan
        return sf.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, min(s.range_max, 8.0))

    def laser_pose(self):
        s = self.scan
        try:
            t = self.tf.lookup_transform(self.global_frame, s.header.frame_id, Time.from_msg(s.header.stamp))
        except Exception:
            try:
                t = self.tf.lookup_transform(self.global_frame, s.header.frame_id, Time())
            except Exception:
                return None
        return tf_pose(t)

    # ------------------------------------------------------------ actions
    def publish_initial(self, base_pose, why, cov_xy=0.05, cov_yaw=0.05):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.scan.header.stamp if self.scan else self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = base_pose[0], base_pose[1]
        q = quat_yaw(base_pose[2])
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
        c = [0.0] * 36
        c[0] = c[7] = cov_xy
        c[35] = cov_yaw
        m.pose.covariance = c
        self.pub_init.publish(m)
        self.get_logger().info("initial pose (%.2f, %.2f, %.0f deg): %s" % (
            base_pose[0], base_pose[1], math.degrees(base_pose[2]), why))

    def search(self):
        """Global match of the newest scan -> (score, fit, base pose) or None."""
        if self.field is None or self.scan is None or self.laser_off is None:
            return None
        pts = self.scan_pts()
        t0 = time.time()
        res = sf.global_search(self.field, pts)
        if not res:
            return None
        score, lp = res[0]
        second = res[1][0] if len(res) > 1 else 0.0
        fit = sf.fit(self.field, pts, lp, self.tol)
        base = sf.compose(lp, sf.inverse(self.laser_off))
        self.get_logger().info("global match: (%.2f, %.2f, %.0f deg) score %.2f (next best %.2f), fit %.2f, %.2f s" % (
            base[0], base[1], math.degrees(base[2]), score, second, fit, time.time() - t0))
        return score, fit, base, second

    def tick(self):
        if self.field is None or self.scan is None:
            return
        if not self.started:
            if self.initial_pose:
                try:
                    x, y, yaw = (float(v) for v in self.initial_pose.replace(",", " ").split())
                except ValueError:
                    self.get_logger().error("initial_pose: 'x,y,yaw' (m, m, rad), got %r" % self.initial_pose)
                    self.initial_pose = ""
                    return
                self.publish_initial((x, y, yaw), "given")
                self.started = True
            elif self.auto_init:
                self.state = "searching"
                r = self.search()
                if r is None:
                    return
                score, fit, base, _ = r
                self.publish_initial(base, "global match at start, fit %.2f" % fit)
                self.last_reloc = (time.time(), {"at": "start", "pose": [round(v, 3) for v in base], "fit": round(fit, 3)})
                self.last_try = time.time()
                self.started = True
            else:
                self.started = True
            return
        lp = self.laser_pose()
        if lp is None:
            self.state = "no_tf"
            self.publish_status()
            return
        pts = self.scan_pts()
        q = sf.fit(self.field, pts, lp, self.tol)
        self.quality = q
        self.pub_q.publish(Float32(data=float(q)))
        now = time.time()
        if q >= self.lost_fit:
            self.bad_since = None
            self.state = "ok" if q >= self.good_fit else "weak"
        else:
            self.bad_since = self.bad_since or now
            self.state = "lost" if now - self.bad_since >= self.lost_s else "weak"
        if self.state == "lost" and self.auto_recover and now - self.last_try >= self.cooldown_s:
            self.last_try = now
            r = self.search()
            if r is not None:
                score, fit, base, _ = r
                if fit >= self.good_fit and fit >= q + self.margin:
                    cur = sf.compose(lp, sf.inverse(self.laser_off))
                    jump = math.hypot(base[0] - cur[0], base[1] - cur[1])
                    self.publish_initial(base, "lost (fit %.2f) -> global match fit %.2f, jump %.2f m" % (q, fit, jump))
                    self.last_reloc = (now, {"at": "lost", "pose": [round(v, 3) for v in base], "fit": round(fit, 3),
                                             "was_fit": round(q, 3), "jump_m": round(jump, 3)})
                    self.bad_since = None
                else:
                    self.get_logger().warn("lost (fit %.2f), global match not convincing (fit %.2f): keeping the pose"
                                           % (q, fit))
        self.publish_status()

    def publish_status(self):
        d = {"state": self.state, "quality": None if self.quality is None else round(self.quality, 3)}
        if self.last_reloc:
            d["last_relocalization"] = dict(self.last_reloc[1], age_s=round(time.time() - self.last_reloc[0], 1))
        self.pub_s.publish(String(data=json.dumps(d)))
        self.get_logger().info("localization %s, quality %s" % (self.state, d["quality"]), throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    n = LocMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
