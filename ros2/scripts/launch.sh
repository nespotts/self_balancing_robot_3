#!/bin/bash
# Start one sbr3_ros launch file next to a running one (used by teleop.ps1):
#   bash ros2/scripts/launch.sh teleop.launch.py [args]
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
exec ros2 launch sbr3_ros "$@"
