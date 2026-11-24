#!/bin/bash
# Save the live slam_toolbox map as ~/maps/NAME: the pose graph (NAME.posegraph + NAME.data, for slam_toolbox
# localization / continuing the map) and the occupancy grid (NAME.yaml + NAME.pgm, for AMCL / map_server), + NAME.png.
#   bash ros2/scripts/save_map.sh NAME [DIR]
# Works while run_mapping.ps1 / run_nav.ps1 (slam) / replay.ps1 runs.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
NAME="${1:?usage: save_map.sh NAME [DIR]}"
DIR="${2:-$HOME/maps}"
case "$NAME" in *[!A-Za-z0-9_-]*) echo "map name: letters, digits, _ and - only"; exit 2 ;; esac
source /opt/ros/jazzy/setup.bash
mkdir -p "$DIR"
P="$DIR/$NAME"
SIM=()
# replay (bag clock): the saver must use sim time too, or it waits for a /map newer than its wall clock
if ros2 topic list 2>/dev/null | grep -qx /clock; then SIM=(-p use_sim_time:=true); fi
echo "pose graph -> $P.posegraph"
if ! timeout 30 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '$P'}" | grep -q "result=0"; then
  echo "  (no slam_toolbox serialize_map service, or it failed: grid only)"
fi
echo "grid -> $P.yaml / .pgm"
ros2 run nav2_map_server map_saver_cli -f "$P" --ros-args -p save_map_timeout:=10.0 "${SIM[@]}" 2>&1 | grep -E "Map saved|error|Failed" || true
[ -f "$P.yaml" ] || { echo "no map saved (is /map published?)"; exit 1; }
python3 "$HERE/map_png.py" "$P.yaml" "$P.png" --title "map $NAME" | tail -1
ls -la "$P".* | awk '{print "  " $5 "\t" $9}'
echo "localize on it:  powershell -File ros2\\run_localize.ps1 -Map $NAME"
