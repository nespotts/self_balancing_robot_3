"""Explore (issue #107): adapter + slam_toolbox online mapping + Nav2 (no map_server: slam_toolbox's /map) +
keep-out zones + the frontier explorer. RViz off by default (the station shows the map and the explore status).

  ros2 launch sbr3_ros explore.launch.py url:=ws://192.168.0.209:8080/ws
  ros2 launch sbr3_ros explore.launch.py url:=ws://127.0.0.1:8080/ws autostart:=true rviz:=true
Start / stop: the station's "Explore (ROS)" button (#106), or
  ros2 service call /explore/start std_srvs/srv/Trigger       ros2 service call /explore/stop std_srvs/srv/Trigger
The robot must be armed and in Manual (STATION) on the phone; nothing here arms it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    slam_share = get_package_share_directory("slam_toolbox")
    nav_share = get_package_share_directory("nav2_bringup")
    L = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("url", default_value="ws://192.168.0.209:8080/ws"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("keepout", default_value="true", description="station zones -> Nav2 keepout filter"),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(share, "config", "nav2.yaml")),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("explore_params", default_value=os.path.join(share, "config", "explore.yaml")),
        DeclareLaunchArgument("autostart", default_value="false", description="start exploring at once"),
        DeclareLaunchArgument("map_dir", default_value="~/maps", description="where explore saves maps"),
        DeclareLaunchArgument("max_duration_s", default_value="0.0", description="stop (done) after this; 0 = no limit"),
        DeclareLaunchArgument("return_home", default_value="false"),
        Node(package="sbr3_ros", executable="adapter", name="sbr3_adapter", output="screen",
             parameters=[{"url": L("url")}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": L("slam_params"), "use_sim_time": "false",
                              "autostart": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": "false", "autostart": "true", "params_file": L("nav_params")}.items()),
        Node(package="sbr3_ros", executable="keepout_mask", name="keepout_mask", output="screen",
             condition=IfCondition(L("keepout"))),
        Node(package="sbr3_ros", executable="explore", name="explore", output="screen",
             parameters=[L("explore_params"), {
                 "autostart": ParameterValue(L("autostart"), value_type=bool),
                 "map_dir": L("map_dir"),
                 "max_duration_s": ParameterValue(L("max_duration_s"), value_type=float),
                 "return_home": ParameterValue(L("return_home"), value_type=bool)}]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(share, "rviz", "sbr3.rviz")], condition=IfCondition(L("rviz"))),
    ])
