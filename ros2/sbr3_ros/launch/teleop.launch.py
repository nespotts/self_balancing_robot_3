"""Gamepad teleop: joy_node + teleop_twist_joy -> /cmd_vel, through the adapter's safety gate (it only drives when the
robot is armed in STATION, stops 0.5 s after the last /cmd_vel, caps 0.25 m/s / 1 rad/s and slews).

  ros2 launch sbr3_ros teleop.launch.py                     next to run_mapping / run_localize (the adapter is there)
  ros2 launch sbr3_ros teleop.launch.py adapter:=true url:=ws://192.168.0.209:8080/ws     on its own
Hold LB (deadman) and push the left stick; RB = faster. Mapping: config/teleop_joy.yaml. Don't drive while Nav2 has a
goal: both publish /cmd_vel. In WSL the pad must be attached to Linux first (usbipd, see docs/ros2.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    cfg = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "teleop_joy.yaml")),
        DeclareLaunchArgument("adapter", default_value="false", description="also start the adapter"),
        DeclareLaunchArgument("url", default_value="ws://192.168.0.209:8080/ws"),
        Node(package="joy", executable="joy_node", name="joy_node", output="screen", parameters=[cfg]),
        Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node", output="screen",
             parameters=[cfg], remappings=[("cmd_vel", "/cmd_vel")]),
        Node(package="sbr3_ros", executable="adapter", name="sbr3_adapter", output="screen",
             parameters=[{"url": LaunchConfiguration("url")}], condition=IfCondition(LaunchConfiguration("adapter"))),
    ])
