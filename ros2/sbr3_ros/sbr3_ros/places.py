"""The station's saved places in ROS (issue #98).

The adapter republishes the bridge's places list as /station/places (latched JSON, station_map frame, mm) and
broadcasts /tf odom -> station_map. A place in the ROS map frame = TF map <- station_map applied to it, so places
saved on the phone work with whatever localization ROS uses (slam_toolbox, AMCL on a saved map ...).
Places belong to the robot profile and map the bridge has loaded (#97): the list follows it.
"""
import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from . import convert as cv


def tf_pose(t):
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


class StationPlaces:
    def __init__(self, node, tf_buffer, global_frame="map"):
        self.node = node
        self.tf = tf_buffer
        self.global_frame = global_frame
        self.places = None
        self.frame = "station_map"
        self.map_name = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, "station/places", self.on_places, latched)

    def on_places(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.places = cv.places_from_msg(d)
        self.frame = d.get("frame") or self.frame
        self.map_name = d.get("map")

    def wait(self, timeout=15.0):
        end = time.time() + timeout
        while rclpy.ok() and self.places is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.places is not None

    def names(self):
        return [p["name"] for p in self.places or []]

    def to_map(self, x, y, timeout=10.0):
        """station_map (m) -> global frame (m)."""
        end = time.time() + timeout
        while rclpy.ok():
            try:
                T = tf_pose(self.tf.lookup_transform(self.global_frame, self.frame, Time()))
                break
            except Exception:
                if time.time() > end:
                    raise RuntimeError("no transform %s <- %s (is the adapter running, and localization up?)" % (
                        self.global_frame, self.frame))
                rclpy.spin_once(self.node, timeout_sec=0.1)
        p = cv.pose_compose(T, (x, y, 0.0))
        return p[0], p[1]

    def resolve(self, name):
        """-> (name as saved, x, y) in the global frame; raises KeyError with the known names."""
        p = cv.find_place(self.places or [], name)
        if p is None:
            raise KeyError("no place called %r on map %s (places: %s)" % (name, self.map_name, ", ".join(self.names())))
        x, y = self.to_map(p["x"], p["y"])
        return p["name"], x, y

    def robot_pose(self):
        try:
            return tf_pose(self.tf.lookup_transform(self.global_frame, "base_link", Time()))
        except Exception:
            return None
