"""Navigation: adapter + Nav2 + RViz, localised by one of
  localization:=slam    (default) slam_toolbox maps while Nav2 drives (map -> odom from slam_toolbox)
  localization:=amcl    a saved map (map:=/path/room.yaml) with map_server + AMCL
  localization:=bridge  a saved map with map_server; map -> odom from the bridge's own LD19 scan matcher

  ros2 launch sbr3_ros navigation.launch.py url:=ws://127.0.0.1:8080/ws
  ros2 launch sbr3_ros navigation.launch.py localization:=amcl map:=$HOME/maps/room.yaml
Goals: RViz "2D Goal Pose", or ros2 run sbr3_ros goal_test --ros-args -p x:=2.0 -p y:=0.5
The robot must be armed and in Manual (STATION) on the phone; the adapter never arms it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    slam_share = get_package_share_directory("slam_toolbox")
    nav_share = get_package_share_directory("nav2_bringup")
    url = LaunchConfiguration("url")
    loc = LaunchConfiguration("localization")
    nav_params = LaunchConfiguration("nav_params")
    map_yaml = LaunchConfiguration("map")
    common = {"use_sim_time": "false", "autostart": "true", "params_file": nav_params}
    return LaunchDescription([
        DeclareLaunchArgument("url", default_value="ws://192.168.0.209:8080/ws"),
        DeclareLaunchArgument("localization", default_value="slam", choices=["slam", "amcl", "bridge"]),
        DeclareLaunchArgument("map", default_value="", description="map yaml for amcl / bridge"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(share, "config", "nav2.yaml")),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        Node(package="sbr3_ros", executable="adapter", name="sbr3_adapter", output="screen",
             parameters=[{"url": url,
                          "use_bridge_correction": PythonExpression(["'", loc, "' == 'bridge'"])}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": LaunchConfiguration("slam_params"), "use_sim_time": "false",
                              "autostart": "true"}.items(),
            condition=LaunchConfigurationEquals("localization", "slam")),
        IncludeLaunchDescription(        # map_server + AMCL
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "localization_launch.py")),
            launch_arguments=dict(common, map=map_yaml).items(),
            condition=LaunchConfigurationEquals("localization", "amcl")),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": map_yaml}], condition=LaunchConfigurationEquals("localization", "bridge")),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             parameters=[{"autostart": True, "node_names": ["map_server"]}],
             condition=LaunchConfigurationEquals("localization", "bridge")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "navigation_launch.py")),
            launch_arguments=common.items()),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(share, "rviz", "sbr3.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
