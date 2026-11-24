"""Send one Nav2 goal and log what happened (for tests and docs).

  ros2 run sbr3_ros goal_test --ros-args -p x:=2.0 -p y:=0.5 -p yaw:=0.0 -p out:=/tmp/nav_goal.json

Records the robot pose in the map frame (tf map -> base_link, 5 Hz) and the newest global plan (/plan), and writes them
with the result to `out` (JSON): {"goal": [x, y, yaw], "result": "succeeded|aborted|canceled|timeout|rejected",
"seconds": ..., "track": [[t, x, y, yaw], ...], "plan": [[x, y], ...], "first_plan": [[x, y], ...]}.
"""
import json
import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .convert import quat_yaw

RESULTS = {4: "succeeded", 5: "canceled", 6: "aborted"}


class GoalTest(Node):
    def __init__(self):
        super().__init__("sbr3_goal_test")
        P = self.declare_parameter
        self.goal = (float(P("x", 1.0).value), float(P("y", 0.0).value), float(P("yaw", 0.0).value))
        self.timeout = float(P("timeout", 180.0).value)
        self.out = P("out", "/tmp/sbr3_nav_goal.json").value
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.track, self.plan, self.first_plan = [], [], None
        self.create_subscription(Path, "plan", self.on_plan, 5)
        self.create_timer(0.2, self.sample)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.t0 = None
        self.result = None

    def on_plan(self, p):
        pts = [[round(s.pose.position.x, 3), round(s.pose.position.y, 3)] for s in p.poses]
        self.plan = pts
        if self.first_plan is None and pts:
            self.first_plan = pts

    def sample(self):
        try:
            tf = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.track.append([round(time.time(), 2), round(tf.transform.translation.x, 3),
                           round(tf.transform.translation.y, 3), round(yaw, 3)])

    def run(self):
        self.get_logger().info("waiting for navigate_to_pose ...")
        if not self.client.wait_for_server(timeout_sec=120.0):
            return "no_server"
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = self.goal[0], self.goal[1]
        qx, qy, qz, qw = quat_yaw(self.goal[2])
        g.pose.pose.orientation.x, g.pose.pose.orientation.y = qx, qy
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = qz, qw
        self.t0 = time.time()
        fut = self.client.send_goal_async(g, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return "rejected"
        self.get_logger().info("goal (%.2f, %.2f) accepted" % self.goal[:2])
        rf = handle.get_result_async()
        end = time.time() + self.timeout
        while rclpy.ok() and not rf.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rf.done():
            handle.cancel_goal_async()
            return "timeout"
        return RESULTS.get(rf.result().status, "status %d" % rf.result().status)

    def on_feedback(self, fb):
        f = fb.feedback
        self.get_logger().info("distance remaining %.2f m" % f.distance_remaining, throttle_duration_sec=5.0)

    def save(self, result):
        d = {"goal": list(self.goal), "result": result, "seconds": round(time.time() - (self.t0 or time.time()), 1),
             "track": self.track, "plan": self.plan, "first_plan": self.first_plan or []}
        with open(self.out, "w") as f:
            json.dump(d, f)
        last = self.track[-1] if self.track else None
        err = math.hypot(last[1] - self.goal[0], last[2] - self.goal[1]) if last else float("nan")
        self.get_logger().info("result %s after %.1f s, final position error %.3f m -> %s" % (
            result, d["seconds"], err, self.out))


def main(args=None):
    rclpy.init(args=args)
    n = GoalTest()
    try:
        r = n.run()
        for _ in range(10):                       # a last pose or two after arriving
            rclpy.spin_once(n, timeout_sec=0.1)
        n.save(r)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
