#!/bin/bash
# End-to-end mapping test with the simulator, headless (no RViz):
#   sim (--lidar --tour) -> bridge ROS stream -> adapter -> slam_toolbox; map snapshots every SNAP_S seconds.
#   bash ros2/scripts/sim_e2e_mapping.sh [OUT_DIR] [SECONDS]      (SIM_EXTRA="--drift 3": odometry drift;
#   SLAM_PARAMS=/path/to.yaml: other slam_toolbox params)
# Leaves OUT_DIR/map_<t>s.{yaml,pgm,png}, map_final.*, topic_hz.txt, frames.gv/.png, *.log
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HOME/sbr3-e2e/mapping}"
DUR="${2:-150}"
SNAPS="${SNAPS:-20 45 90}"
mkdir -p "$OUT"
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
pkill -f "station.sim" ; pkill -f "sbr3_ros/adapter"; pkill -f async_slam_toolbox; sleep 1
cleanup() { kill $LAUNCH_PID $SIM_PID 2>/dev/null; sleep 2; pkill -f "station.sim"; pkill -f async_slam_toolbox; pkill -f "sbr3_ros/adapter"; }
trap cleanup EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --tour $SIM_EXTRA --http-port 8080 --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
SIM_PID=$!
sleep 3
ros2 launch sbr3_ros mapping.launch.py url:=ws://127.0.0.1:8080/ws rviz:=false ${SLAM_PARAMS:+slam_params:=$SLAM_PARAMS} \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
T0=$(date +%s)
snap() {
  ros2 run nav2_map_server map_saver_cli -f "$OUT/$1" --ros-args -p save_map_timeout:=10.0 > "$OUT/saver_$1.log" 2>&1
  python3 "$HERE/map_png.py" "$OUT/$1.yaml" "$OUT/$1.png" --truth --title "$2" | tail -1
}
for s in $SNAPS; do
  while [ $(( $(date +%s) - T0 )) -lt "$s" ]; do sleep 1; done
  snap "map_${s}s" "slam_toolbox map from the sim, t = ${s} s"
done
# rates, tf tree (while it runs)
{
  for t in /scan /odom /imu /tf /map /robot_state; do
    echo "== $t"; timeout 6 ros2 topic hz "$t" 2>&1 | grep -E "average rate" | tail -1
  done
  echo "== /scan bandwidth"; timeout 6 ros2 topic bw /scan 2>&1 | grep -E "B/s" | tail -1
} > "$OUT/topic_hz.txt"
(cd "$OUT" && timeout 15 ros2 run tf2_tools view_frames -o frames > view_frames.log 2>&1)
command -v dot >/dev/null && [ -f "$OUT/frames.gv" ] && dot -Tpng "$OUT/frames.gv" -o "$OUT/frames.png"
while [ $(( $(date +%s) - T0 )) -lt "$DUR" ]; do sleep 1; done
snap map_final "slam_toolbox map from the sim after ${DUR} s (tour loop, LD19 10 Hz)"
grep -E "scans [0-9]+" "$OUT/launch.log" | tail -1
echo "done: $OUT"
