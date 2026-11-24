"""Log the robot's map pose and motion to CSV (tests, tuning plots).

  ros2 run sbr3_ros pose_logger --ros-args -p out:=/tmp/track.csv -p hz:=10.0

Columns: t (wall s), x, y, yaw (tf map -> base_link; empty while there is none), q (/localization_quality),
v, w (/odom twist), cmd_v, cmd_w (/cmd_vel, what reaches the adapter), pitch (deg, /imu).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener


class PoseLogger(Node):
    def __init__(self):
        super().__init__("pose_logger")
        self.out = open(self.declare_parameter("out", "/tmp/sbr3_track.csv").value, "w")
        self.frame = self.declare_parameter("frame", "map").value
        hz = float(self.declare_parameter("hz", 10.0).value)
        self.out.write("t,x,y,yaw,q,v,w,cmd_v,cmd_w,pitch\n")
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.q = self.v = self.w = self.cv = self.cw = self.pitch = None
        self.create_subscription(Float32, "localization_quality", lambda m: setattr(self, "q", m.data), 5)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 20)
        self.create_subscription(Imu, "imu", self.on_imu, 20)
        self.create_timer(1.0 / hz, self.tick)

    def on_odom(self, m):
        self.v, self.w = m.twist.twist.linear.x, m.twist.twist.angular.z

    def on_cmd(self, m):
        self.cv, self.cw = m.linear.x, m.angular.z

    def on_imu(self, m):
        q = m.orientation
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))

    def tick(self):
        x = y = yaw = None
        try:
            t = self.tf.lookup_transform(self.frame, "base_link", Time())
            q = t.transform.rotation
            x, y = t.transform.translation.x, t.transform.translation.y
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            pass
        f = lambda v, n=4: "" if v is None else ("%." + str(n) + "f") % v
        self.out.write(",".join([f(time.time(), 3), f(x), f(y), f(yaw), f(self.q, 3), f(self.v), f(self.w),
                                 f(self.cv), f(self.cw), f(self.pitch, 2)]) + "\n")
        self.out.flush()


def main(args=None):
    rclpy.init(args=args)
    n = PoseLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.out.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
