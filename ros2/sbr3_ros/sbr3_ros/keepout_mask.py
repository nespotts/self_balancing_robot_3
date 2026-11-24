"""Station zones -> Nav2 keepout filter (issue #98).

  ros2 run sbr3_ros keepout_mask

Reads /station/zones (the adapter republishes the bridge's virtual walls, no-go zones, fence and mazes; station_map
frame, mm), puts them in the ROS map frame with TF map <- station_map, and publishes
  /keepout_filter_mask   nav_msgs/OccupancyGrid (latched, map frame): 100 = keep out (shape + inflate), 0 = free
  /costmap_filter_info   nav2_msgs/CostmapFilterInfo (latched): type 0 (keepout), the mask topic
which the KeepoutFilter in config/nav2.yaml (global + local costmap) reads. The mask is rebuilt when the zones
change, when /map grows past it, or when map <- station_map moves (relocalization) by > move_m / move_deg.
Also /station/markers (MarkerArray, latched) for RViz: the saved places (name + dot) and the zone outlines.
"""
import json
import math
import time

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import convert as cv
from . import keepout as ko
from .places import tf_pose

ZONE_RGB = {"wall": (1.0, 0.2, 0.2), "nogo": (1.0, 0.2, 0.2), "fence": (1.0, 0.6, 0.0), "maze": (0.8, 0.3, 0.8),
            "slow": (1.0, 0.85, 0.0)}


class KeepoutMask(Node):
    def __init__(self):
        super().__init__("keepout_mask")
        P = self.declare_parameter
        self.global_frame = P("global_frame", "map").value
        self.inflate = float(P("inflate", 0.15).value)
        self.move_m = float(P("move_m", 0.03).value)
        self.move_deg = float(P("move_deg", 1.0).value)
        mask_topic = P("mask_topic", "/keepout_filter_mask").value
        info_topic = P("filter_info_topic", "/costmap_filter_info").value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_mask = self.create_publisher(OccupancyGrid, mask_topic, latched)
        self.pub_info = self.create_publisher(CostmapFilterInfo, info_topic, latched)
        self.create_subscription(String, "station/zones", self.on_zones, latched)
        self.create_subscription(String, "station/places", self.on_places, latched)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.pub_markers = self.create_publisher(MarkerArray, "station/markers", latched)
        self.places = []
        self.markers_T = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.items = None
        self.frame = "station_map"
        self.map_info = None
        self.built = None                       # (signature, T, map_info) of the published mask
        self.no_tf_since = None
        self.dirty = True
        self.info_sent = False
        self.mask_topic = mask_topic
        self.create_timer(1.0, self.tick)
        self.get_logger().info("station zones -> %s (inflate %.2f m)" % (mask_topic, self.inflate))

    def on_zones(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.items = d.get("items") or []
        self.frame = d.get("frame") or self.frame
        self.dirty = True
        self.markers_T = None

    def on_places(self, m):
        try:
            self.places = cv.places_from_msg(json.loads(m.data))
        except ValueError:
            return
        self.markers_T = None

    def markers(self, T):
        """Places and zone outlines in the map frame, for RViz."""
        stamp = self.get_clock().now().to_msg()
        out = [Marker(action=Marker.DELETEALL)]
        mid = 0

        def mk(kind, ns):
            nonlocal mid
            mid += 1
            m = Marker(type=kind, ns=ns, id=mid, action=Marker.ADD)
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.pose.orientation.w = 1.0
            return m
        for p in self.places:
            x, y, _ = cv.pose_compose(T, (p["x"], p["y"], 0.0))
            s = mk(Marker.SPHERE, "places")
            s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, 0.05
            s.scale.x = s.scale.y = s.scale.z = 0.12
            s.color.r, s.color.g, s.color.b, s.color.a = 0.1, 0.9, 0.3, 1.0
            t = mk(Marker.TEXT_VIEW_FACING, "places")
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y + 0.15, 0.2
            t.scale.z = 0.16
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = p["name"]
            out += [s, t]
        for it in self.items or []:
            k = it.get("kind")
            if k not in ZONE_RGB:
                continue
            if k == "slow":
                segs, _ = ko.segments(dict(it, kind="nogo"), T)
            else:
                segs, _ = ko.segments(it, T)
            ln = mk(Marker.LINE_LIST, "zones")
            ln.scale.x = 0.04
            ln.color.r, ln.color.g, ln.color.b = ZONE_RGB[k]
            ln.color.a = 1.0
            for s in segs:
                ln.points += [Point(x=s[0], y=s[1], z=0.02), Point(x=s[2], y=s[3], z=0.02)]
            out.append(ln)
        self.pub_markers.publish(MarkerArray(markers=out))
        self.markers_T = T

    def on_map(self, m):
        i = m.info
        info = (i.origin.position.x, i.origin.position.y, i.resolution, i.width, i.height)
        if self.map_info is None or info[3] > self.map_info[3] or info[4] > self.map_info[4] or \
                abs(info[0] - self.map_info[0]) > 0.5 or abs(info[1] - self.map_info[1]) > 0.5:
            self.dirty = True
        self.map_info = info

    def lookup(self):
        try:
            T = tf_pose(self.tf.lookup_transform(self.global_frame, self.frame, Time()))
            self.no_tf_since = None
            return T
        except Exception:
            self.no_tf_since = self.no_tf_since or time.time()
            if time.time() - self.no_tf_since > 10.0:
                self.get_logger().warn("no tf %s <- %s for 10 s: using the identity" % (self.global_frame, self.frame),
                                       throttle_duration_sec=60.0)
                return (0.0, 0.0, 0.0)
            return None

    def tick(self):
        if not self.info_sent:
            info = CostmapFilterInfo()
            info.header.frame_id = self.global_frame
            info.header.stamp = self.get_clock().now().to_msg()
            info.type = 0                    # keepout
            info.filter_mask_topic = self.mask_topic
            info.base, info.multiplier = 0.0, 1.0
            self.pub_info.publish(info)
            self.info_sent = True
        if self.items is None and not self.places:
            return
        T = self.lookup()
        if T is None:
            return
        if self.markers_T is None or math.hypot(T[0] - self.markers_T[0], T[1] - self.markers_T[1]) > \
                self.move_m or abs(math.remainder(T[2] - self.markers_T[2], 2 * math.pi)) > math.radians(self.move_deg):
            self.markers(T)
        if self.items is None:
            return
        if self.built and not self.dirty:
            _, T0, _ = self.built
            moved = math.hypot(T[0] - T0[0], T[1] - T0[1]) > self.move_m or \
                abs(math.degrees(math.remainder(T[2] - T0[2], 2 * math.pi))) > self.move_deg
            if not moved:
                return
        self.build(T)

    def build(self, T):
        t0 = time.time()
        ext = ko.mask_extent(self.map_info, self.items, T)
        if ext is None:                      # no map yet and no zones: an empty 1 cell mask
            ext = ((0.0, 0.0), 1, 1)
        (x0, y0), w, h = ext
        res = self.map_info[2] if self.map_info else 0.05
        data, used = ko.build_mask(self.items, T, (x0, y0), res, w, h, self.inflate)
        m = OccupancyGrid()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = res
        m.info.width, m.info.height = w, h
        m.info.origin.position.x, m.info.origin.position.y = x0, y0
        m.info.origin.orientation.w = 1.0
        m.data = data.ravel().tolist()
        self.pub_mask.publish(m)
        self.built = (ko.zones_signature(self.items), T, self.map_info)
        self.dirty = False
        self.get_logger().info("keepout mask: %d zone(s), %d x %d cells, %d keep-out, station->map (%.2f, %.2f, %.1f deg)"
                               ", %.0f ms" % (used, w, h, int((data == 100).sum()), T[0], T[1], math.degrees(T[2]),
                                              1000 * (time.time() - t0)))


def main(args=None):
    rclpy.init(args=args)
    n = KeepoutMask()
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
