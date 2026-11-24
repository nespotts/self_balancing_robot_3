#!/bin/bash
# Run one sbr3_ros node next to a running launch (used by go_to.ps1 / patrol.ps1):
#   bash ros2/scripts/node.sh go_to --ros-args -p place:=Kitchen
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
exec ros2 run sbr3_ros "$@"
