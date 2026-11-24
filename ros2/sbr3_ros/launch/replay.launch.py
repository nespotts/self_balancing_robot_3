"""Replay a recorded run (record_run.ps1) for offline tuning, adapter off, clock from the bag (use_sim_time).

  ros2 launch sbr3_ros replay.launch.py bag:=$HOME/sbr3-bags/20261124_201500                    rebuild the map (slam_toolbox)
  ros2 launch sbr3_ros replay.launch.py bag:=... mode:=amcl map:=$HOME/maps/room.yaml           localize on a saved map
  ros2 launch sbr3_ros replay.launch.py bag:=... slam_params:=/tmp/try.yaml rate:=2.0 rviz:=false

The bag's own map -> odom, /map and localization topics are moved under /bag/ (bag_tf_filter relays the rest of /tf),
so the offline slam_toolbox / AMCL publishes its own. Save the rebuilt map with save_map.ps1 while it plays.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# topics the offline stack makes itself: the bag's copies go under /bag/
REMAP = ["/tf", "/map", "/map_metadata", "/amcl_pose", "/initialpose", "/localization_quality", "/cmd_vel",
         "/slam_toolbox/graph_visualization", "/particle_cloud", "/keepout_filter_mask", "/costmap_filter_info"]


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    slam_share = get_package_share_directory("slam_toolbox")
    nav_share = get_package_share_directory("nav2_bringup")
    bag = LaunchConfiguration("bag")
    sim_time = {"use_sim_time": True}
    play = ["ros2", "bag", "play", bag, "--clock", "50", "--rate", LaunchConfiguration("rate"),
            "--delay", LaunchConfiguration("delay"), "--remap"] + ["%s:=/bag%s" % (t, t) for t in REMAP]
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="bag folder from record_run.ps1"),
        DeclareLaunchArgument("mode", default_value="slam", choices=["slam", "amcl"]),
        DeclareLaunchArgument("map", default_value="", description="map yaml (mode:=amcl)"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("delay", default_value="4.0", description="s before the bag starts (nodes come up)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(share, "config", "nav2.yaml")),
        ExecuteProcess(cmd=play, output="screen", name="bag_play"),
        Node(package="sbr3_ros", executable="bag_tf_filter", name="bag_tf_filter", output="screen",
             parameters=[sim_time]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": LaunchConfiguration("slam_params"), "use_sim_time": "true",
                              "autostart": "true"}.items(),
            condition=LaunchConfigurationEquals("mode", "slam")),
        IncludeLaunchDescription(        # map_server + AMCL, and loc_monitor for the start pose
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "localization_launch.py")),
            launch_arguments={"use_sim_time": "true", "autostart": "true", "map": LaunchConfiguration("map"),
                              "params_file": LaunchConfiguration("nav_params")}.items(),
            condition=LaunchConfigurationEquals("mode", "amcl")),
        Node(package="sbr3_ros", executable="loc_monitor", name="loc_monitor", output="screen",
             parameters=[sim_time], condition=LaunchConfigurationEquals("mode", "amcl")),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log", parameters=[sim_time],
             arguments=["-d", os.path.join(share, "rviz", "sbr3.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
