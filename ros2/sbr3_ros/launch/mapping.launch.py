"""Mapping: adapter + slam_toolbox (online async) + RViz.

  ros2 launch sbr3_ros mapping.launch.py url:=ws://192.168.0.209:8080/ws
  ros2 launch sbr3_ros mapping.launch.py url:=ws://127.0.0.1:8080/ws rviz:=false     (sim in WSL, headless)
Drive the robot (phone joystick, or Nav2) and save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/room
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    slam_share = get_package_share_directory("slam_toolbox")
    url = LaunchConfiguration("url")
    rviz = LaunchConfiguration("rviz")
    slam_params = LaunchConfiguration("slam_params")
    enable_cmd_vel = LaunchConfiguration("enable_cmd_vel")
    return LaunchDescription([
        DeclareLaunchArgument("url", default_value="ws://192.168.0.209:8080/ws",
                              description="bridge WebSocket (robot), or ws://127.0.0.1:8080/ws for the sim"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("enable_cmd_vel", default_value="true",
                              description="forward /cmd_vel (e.g. teleop_twist_keyboard) to the robot"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        Node(package="sbr3_ros", executable="adapter", name="sbr3_adapter", output="screen",
             parameters=[{"url": url, "enable_cmd_vel": enable_cmd_vel}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_params, "use_sim_time": "false",
                              "autostart": "true"}.items()),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(share, "rviz", "sbr3.rviz")], condition=IfCondition(rviz)),
    ])
