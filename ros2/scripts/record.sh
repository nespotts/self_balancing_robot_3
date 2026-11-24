#!/bin/bash
# Record a run into ~/sbr3-bags/<date>_<time>[_NAME] (rosbag2, mcap). Ctrl+C stops and closes the bag.
#   bash ros2/scripts/record.sh [NAME] [SECONDS]        SECONDS: stop by itself after that long (tests)
# Records whatever of these is live: the adapter's topics, /map, /tf, /tf_static, and Nav2 / localization topics.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
NAME="${1:-}"
DUR="${2:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/sbr3-bags/${STAMP}${NAME:+_$NAME}"
mkdir -p "$HOME/sbr3-bags"
TOPICS=(
  /scan /odom /imu /tf /tf_static /robot_state /battery /cmd_vel           # adapter
  /map /map_metadata /slam_toolbox/graph_visualization                      # SLAM
  /amcl_pose /initialpose /localization_quality                             # localization
  /plan /local_plan /goal_pose /cmd_vel_nav /cmd_vel_smoothed /collision_monitor_state
  /station/places /station/zones /keepout_filter_mask /costmap_filter_info
  /joy
)
echo "recording to $OUT"
echo "  Windows: \\\\wsl.localhost\\$WSL_DISTRO_NAME${OUT//\//\\}"
echo "  replay:  powershell -File ros2\\replay.ps1 -Bag $OUT"
if [ -n "$DUR" ]; then
  timeout -s INT "$DUR" ros2 bag record -s mcap -o "$OUT" --topics "${TOPICS[@]}" || true
  sleep 1
  ros2 bag info "$OUT" | grep -E "Duration|Messages|Topic:" | head -30
else
  exec ros2 bag record -s mcap -o "$OUT" --topics "${TOPICS[@]}"
fi
