"""Localize on a saved map (save_map.ps1) and navigate: adapter + slam_toolbox localization (or AMCL) + loc_monitor
+ Nav2 + the station's keep-out zones + RViz.

  ros2 launch sbr3_ros localize.launch.py map:=room                         ~/maps/room.posegraph (+ .yaml)
  ros2 launch sbr3_ros localize.launch.py map:=room localization:=amcl      AMCL + map_server on room.yaml
  ros2 launch sbr3_ros localize.launch.py map:=room initial_pose:="0,0,0"   skip the global match at start

slam_toolbox localization is the default: in the sim it held 1.5-2 cm (AMCL 4-6 cm) on the same map and runs.

loc_monitor finds the robot on the map at start (global scan match -> /initialpose) and again when it is lost
(kidnapped, carried). Nothing here arms the robot: Arm & Balance + Manual on the phone, as for run_nav.ps1.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def map_base(m):
    """'room' -> ~/maps/room; '/x/room.yaml' -> /x/room"""
    m = os.path.expanduser(m)
    if "/" not in m:
        m = os.path.join(os.path.expanduser("~/maps"), m)
    for ext in (".yaml", ".posegraph", ".pgm", ".data"):
        if m.endswith(ext):
            m = m[:-len(ext)]
    return m


def setup(context):
    share = get_package_share_directory("sbr3_ros")
    nav_share = get_package_share_directory("nav2_bringup")
    A = lambda k: context.launch_configurations[k]
    base = map_base(A("map"))
    loc = A("localization")
    nav_params = A("nav_params")
    on = lambda k: A(k).lower() in ("true", "1", "yes")
    if not os.path.exists(base + ".yaml"):
        raise RuntimeError("no map %s.yaml (save one with save_map.ps1)" % base)
    if loc == "slam" and not os.path.exists(base + ".posegraph"):
        raise RuntimeError("no pose graph %s.posegraph for localization:=slam (use localization:=amcl)" % base)
    common = {"use_sim_time": "false", "autostart": "true", "params_file": nav_params}
    acts = [
        Node(package="sbr3_ros", executable="adapter", name="sbr3_adapter", output="screen",
             parameters=[{"url": A("url")}]),
        Node(package="sbr3_ros", executable="loc_monitor", name="loc_monitor", output="screen",
             parameters=[{"initial_pose": A("initial_pose"), "auto_recover": on("auto_recover")}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "navigation_launch.py")),
            launch_arguments=common.items()),
    ]
    if loc == "amcl":
        acts.append(IncludeLaunchDescription(            # map_server + AMCL
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "localization_launch.py")),
            launch_arguments=dict(common, map=base + ".yaml").items()))
    else:
        from launch.actions import EmitEvent, RegisterEventHandler
        from launch.events import matches_action
        from launch_ros.event_handlers import OnStateTransition
        from launch_ros.events.lifecycle import ChangeState
        from lifecycle_msgs.msg import Transition
        # slam_toolbox localization keeps adding recent scans to the map it publishes (wherever it thinks the robot
        # is, also before the first pose). Nav2, loc_monitor and keepout_mask get the saved map from map_server
        # on /map instead; slam_toolbox's own goes to /slam_toolbox/map.
        node = LifecycleNode(package="slam_toolbox", executable="localization_slam_toolbox_node",
                             name="slam_toolbox", namespace="", output="screen",
                             parameters=[os.path.join(share, "config", "slam_toolbox.yaml"),
                                         {"mode": "localization", "map_file_name": base, "map_start_at_dock": True,
                                          "use_lifecycle_manager": False, "map_name": "/slam_toolbox/map"}])
        acts += [Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                      parameters=[{"yaml_filename": base + ".yaml"}]),
                 Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
                      parameters=[{"autostart": True, "node_names": ["map_server"]}]),
                 node,
                 EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                             transition_id=Transition.TRANSITION_CONFIGURE)),
                 RegisterEventHandler(OnStateTransition(
                     target_lifecycle_node=node, start_state="configuring", goal_state="inactive",
                     entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                                           transition_id=Transition.TRANSITION_ACTIVATE))]))]
    if on("keepout"):
        acts.append(Node(package="sbr3_ros", executable="keepout_mask", name="keepout_mask", output="screen"))
    if on("rviz"):
        acts.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                         arguments=["-d", os.path.join(share, "rviz", "sbr3.rviz")]))
    return acts


def generate_launch_description():
    share = get_package_share_directory("sbr3_ros")
    return LaunchDescription([
        DeclareLaunchArgument("url", default_value="ws://192.168.0.209:8080/ws"),
        DeclareLaunchArgument("map", description="saved map: a name in ~/maps, or a path"),
        DeclareLaunchArgument("localization", default_value="slam", choices=["slam", "amcl"]),
        DeclareLaunchArgument("initial_pose", default_value="",
                              description="x,y,yaw (m, m, rad) in the map; empty = global match at start"),
        DeclareLaunchArgument("auto_recover", default_value="true", description="relocalize when lost (kidnapped)"),
        DeclareLaunchArgument("keepout", default_value="false", description="station zones -> Nav2 keepout filter"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(share, "config", "nav2.yaml")),
        OpaqueFunction(function=setup),
    ])
