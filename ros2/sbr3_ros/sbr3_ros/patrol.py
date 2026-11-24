"""Patrol the station's saved places with the Nav2 waypoint follower (issue #98).

  ros2 run sbr3_ros patrol --ros-args -p places:="Kitchen,Couch,Home" -p loops:=3 -p dwell:=5.0
  ros2 run sbr3_ros patrol --ros-args -p places:="Kitchen,Couch" -p loops:=0          (until Ctrl+C)

- places: comma-separated names, as saved on the phone (case does not matter; "home" = Home);
- loops: passes through the list (0 = forever); dwell: seconds to wait at each place;
- the places are put in the map frame once at the start (TF map <- station_map, see places.py), each one facing
  the direction it is approached from;
- each place is one follow_waypoints goal, then this node waits `dwell` s itself (Jazzy's wait_at_waypoint reads
  its pause only at startup, so a per-patrol dwell has to live here); a place Nav2 cannot reach is skipped;
- waits for loc_monitor to report a good pose (if it runs) before it starts;
- out: optional JSON log (arrival times per place, the driven track, result). Ctrl+C cancels the patrol.
"""
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from . import convert as cv
from .places import StationPlaces

RESULTS = {4: "succeeded", 5: "canceled", 6: "aborted"}


class Patrol(Node):
    def __init__(self):
        super().__init__("sbr3_patrol")
        P = self.declare_parameter
        self.names = [n.strip() for n in str(P("places", "").value).split(",") if n.strip()]
        self.loops = int(P("loops", 1).value)
        self.dwell = float(P("dwell", 3.0).value)
        self.out = P("out", "").value
        self.wait_loc = float(P("wait_localization_s", 30.0).value)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.places = StationPlaces(self, self.tf)
        self.loc_state = None
        self.create_subscription(String, "localization_status", self.on_loc, 5)
        self.client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.track, self.events, self.missed, self.pts = [], [], [], []
        self.goal_handle = None               # (Node.handle is rclpy's own)
        self.t0 = None
        self.create_timer(0.2, self.sample)

    def on_loc(self, m):
        try:
            self.loc_state = json.loads(m.data).get("state")
        except ValueError:
            pass

    def sample(self):
        p = self.places.robot_pose()
        if p and self.t0:
            self.track.append([round(time.time(), 2), round(p[0], 3), round(p[1], 3), round(p[2], 3)])

    def spin_for(self, secs):
        end = time.time() + secs
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def event(self, what, name):
        self.events.append([round(time.time() - self.t0, 1), what, name])
        self.get_logger().info("%s %s (%.0f s)" % (what, name, time.time() - self.t0))

    def go(self, pose):
        """One follow_waypoints goal with this pose -> 'succeeded' | 'missed' | 'aborted' | ..."""
        g = FollowWaypoints.Goal()
        g.poses = [pose]
        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        h = fut.result()
        if h is None or not h.accepted:
            return "rejected"
        self.goal_handle = h
        rf = h.get_result_async()
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.goal_handle = None
        r = rf.result()
        if r is None:
            return "no_result"
        if r.status == 4 and r.result.missed_waypoints:
            return "missed"
        return RESULTS.get(r.status, "status %d" % r.status)

    def run(self):
        if not self.names:
            self.get_logger().error("places:=\"Kitchen,Couch,...\" is empty")
            return "no_places"
        if not self.places.wait(15.0):
            self.get_logger().error("no /station/places (is the adapter connected to the bridge?)")
            return "no_places"
        end = time.time() + self.wait_loc
        while rclpy.ok() and time.time() < end and self.loc_state not in (None, "ok"):
            self.get_logger().info("waiting for a good pose (localization %s)" % self.loc_state, throttle_duration_sec=5.0)
            rclpy.spin_once(self, timeout_sec=0.2)
        try:
            self.pts = [self.places.resolve(n) for n in self.names]
        except (KeyError, RuntimeError) as e:
            self.get_logger().error(str(e))
            return "no_place"
        rp = self.places.robot_pose()
        xy = [(x, y) for _, x, y in self.pts]
        yaws = cv.route_yaws(xy, start=rp[:2] if rp else None)
        later = list(yaws)
        if len(xy) > 1:                       # later loops come back to the first place from the last one
            later[0] = cv.route_yaws([xy[-1], xy[0]])[1]

        def pose(i, yaw):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = xy[i]
            q = cv.quat_yaw(yaw)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            return ps
        for name, x, y in self.pts:
            self.get_logger().info("  %s: map (%.2f, %.2f)" % (name, x, y))
        if not self.client.wait_for_server(timeout_sec=60.0):
            return "no_server"
        self.t0 = time.time()
        self.get_logger().info("patrol: %s, %s, dwell %.1f s" % (
            " -> ".join(n for n, _, _ in self.pts), "forever" if self.loops <= 0 else "%d loop(s)" % self.loops,
            self.dwell))
        loop = 0
        while rclpy.ok() and (self.loops <= 0 or loop < self.loops):
            for i, (name, _, _) in enumerate(self.pts):
                self.event("heading to", name)
                r = self.go(pose(i, (later if loop else yaws)[i]))
                if r in ("rejected", "no_result", "canceled"):
                    return r
                if r != "succeeded":
                    self.missed.append([loop, name, r])
                    self.event("missed (%s)" % r, name)
                    continue
                self.event("at", name)
                self.spin_for(self.dwell)
            loop += 1
        return "succeeded" if not self.missed else "succeeded, missed %d" % len(self.missed)

    def save(self, result):
        self.get_logger().info("patrol %s after %.0f s, missed %s" % (
            result, time.time() - (self.t0 or time.time()), self.missed))
        if self.out:
            with open(self.out, "w") as f:
                json.dump({"places": [[n, round(x, 3), round(y, 3)] for n, x, y in self.pts],
                           "loops": self.loops, "dwell": self.dwell, "result": result,
                           "seconds": round(time.time() - (self.t0 or time.time()), 1),
                           "events": self.events, "missed": self.missed, "track": self.track}, f)


def main(args=None):
    rclpy.init(args=args)
    n = Patrol()
    r = "interrupted"
    try:
        r = n.run()
    except KeyboardInterrupt:
        if n.goal_handle is not None:
            n.goal_handle.cancel_goal_async()
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.save(r)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
