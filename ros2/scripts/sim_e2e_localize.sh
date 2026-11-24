#!/bin/bash
# End-to-end localization test with the simulator, headless:
#   saved map MAP (map frame = the sim world: it was mapped from the sim's start pose), robot started elsewhere
#   (START, world mm/deg) -> localize.launch.py (AMCL or slam_toolbox localization + loc_monitor + Nav2);
#   1. relocalize at start (no initial pose: global match), 2. drive a Nav2 goal, 3. kidnap (teleport) -> recover,
#   4. drive a goal again. Ground truth: the sim's --truth-log; the ROS pose: pose_logger.
#   bash ros2/scripts/sim_e2e_localize.sh MAP_BASE [OUT_DIR] [amcl|slam] [START] [KIDNAP] [extra launch args]
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
MAP="${1:?map base path (no extension)}"
OUT="${2:-$HOME/sbr3-e2e/localize}"
LOC="${3:-amcl}"
START="${4:-2600,-200,135}"
KIDNAP="${5:-3300 1300 -90}"
shift 5 2>/dev/null || shift $#
mkdir -p "$OUT"
rm -f "$OUT"/*.csv "$OUT"/goal*.json
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
# every ROS node runs from /opt/ros/jazzy/lib or ~/sbr3_ws/install (patterns that cannot match this script's args)
stop_all() { pkill -f "ros2 launch"; pkill -f "station.sim"; pkill -f "/opt/ros/jazzy/lib/"; pkill -f "sbr3_ws/install/"; \
             sleep 2; }
stop_all
trap stop_all EXIT
TP="$OUT/teleport"
rm -f "$TP"
(cd "$REPO" && exec python3 -m station.sim --lidar --start "$START" --teleport-file "$TP" --truth-log "$OUT/truth.csv" \
   $SIM_EXTRA --http-port 8080 --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
ros2 launch sbr3_ros localize.launch.py url:=ws://127.0.0.1:8080/ws map:="$MAP" localization:="$LOC" rviz:=false "$@" \
  > "$OUT/launch.log" 2>&1 &
ros2 run sbr3_ros pose_logger --ros-args -p out:="$OUT/track.csv" > /dev/null 2>&1 &
sleep 10
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
date +%s.%N > "$OUT/t_armed"
echo "armed; waiting for the start relocalization"
sleep 25
grep -E "global match|initial pose" "$OUT/launch.log" | tail -3
ros2 run sbr3_ros goal_test --ros-args -p x:=0.0 -p y:=1.0 -p yaw:=1.57 -p timeout:=120.0 -p out:="$OUT/goal1.json" 2>&1 \
  | grep -E "result" | tail -1
date +%s.%N > "$OUT/t_kidnap"
echo "$KIDNAP" > "$TP"
echo "kidnapped -> $KIDNAP (world mm deg)"
sleep 30
grep -E "lost|global match|initial pose" "$OUT/launch.log" | tail -3
ros2 run sbr3_ros goal_test --ros-args -p x:=1.0 -p y:=-0.6 -p yaw:=3.14 -p timeout:=120.0 -p out:="$OUT/goal2.json" 2>&1 \
  | grep -E "result" | tail -1
sleep 2
python3 "$HERE/loc_report.py" "$OUT" --map "$MAP.yaml" --title "$LOC" | tail -12
echo "done: $OUT"
