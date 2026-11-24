"""Replay helper: relay a bag's /tf (played as /bag/tf) to /tf without the transforms the offline stack makes itself.

  ros2 run sbr3_ros bag_tf_filter --ros-args -p drop:="['map>odom']"

`drop` entries are "parent>child"; the default drops map -> odom (slam_toolbox / AMCL in the recording), so an
offline slam_toolbox or AMCL can publish its own. odom -> base_link and everything else pass through unchanged.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def keep(transform, drop):
    return "%s>%s" % (transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/")) not in drop


class BagTfFilter(Node):
    def __init__(self):
        super().__init__("bag_tf_filter")
        self.drop = set(self.declare_parameter("drop", ["map>odom"]).value)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TFMessage, "/tf", qos)
        self.create_subscription(TFMessage, "/bag/tf", self.on_tf, qos)
        self.n_in = self.n_drop = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info("relaying /bag/tf -> /tf, dropping %s" % sorted(self.drop))

    def on_tf(self, m):
        out = [t for t in m.transforms if keep(t, self.drop)]
        self.n_in += len(m.transforms)
        self.n_drop += len(m.transforms) - len(out)
        if out:
            self.pub.publish(TFMessage(transforms=out))

    def report(self):
        self.get_logger().info("tf relayed %d, dropped %d" % (self.n_in - self.n_drop, self.n_drop))


def main(args=None):
    rclpy.init(args=args)
    n = BagTfFilter()
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
