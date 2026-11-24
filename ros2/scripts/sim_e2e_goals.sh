#!/bin/bash
# Drive a fixed goal list in the sim on a saved map (AMCL, known start pose) with one Nav2 params file, headless;
# for comparing controller / smoother configs on the same goals (nav_metrics.py makes the table and plots).
#   bash ros2/scripts/sim_e2e_goals.sh MAP_BASE NAV_PARAMS OUT_DIR ["x y yaw" ...]
#   SIM_EXTRA="--box 2000,100,300,300": an obstacle that is not on the saved map
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
MAP="${1:?map base}"; NAV="${2:?nav2 params yaml}"; OUT="${3:?out dir}"; shift 3
GOALS=("$@"); [ ${#GOALS[@]} -eq 0 ] && GOALS=("3.4 0.3 0.0" "3.3 1.4 1.57" "0.0 0.0 3.14")
mkdir -p "$OUT"
rm -f "$OUT"/*.csv "$OUT"/goal*.json
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
stop_all() { pkill -f "ros2 launch"; pkill -f "station.sim"; pkill -f "/opt/ros/jazzy/lib/"; pkill -f "sbr3_ws/install/"; \
             sleep 2; }
stop_all
trap stop_all EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --truth-log "$OUT/truth.csv" $SIM_EXTRA --http-port 8080 \
   --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
ros2 launch sbr3_ros localize.launch.py url:=ws://127.0.0.1:8080/ws map:="$MAP" initial_pose:="0,0,0" \
  nav_params:="$NAV" rviz:=false $LAUNCH_EXTRA > "$OUT/launch.log" 2>&1 &
ros2 run sbr3_ros pose_logger --ros-args -p out:="$OUT/track.csv" -p hz:=20.0 > /dev/null 2>&1 &
sleep 10
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
sleep 8
i=0
for g in "${GOALS[@]}"; do
  i=$((i + 1))
  set -- $g
  ros2 run sbr3_ros goal_test --ros-args -p x:=$1 -p y:=$2 -p yaw:=$3 -p timeout:=150.0 -p out:="$OUT/goal$i.json" 2>&1 \
    | grep -E "result" | tail -1
  sleep 3
done
grep -E "ignoring /cmd_vel" "$OUT/launch.log" | tail -3
echo "done: $OUT"
