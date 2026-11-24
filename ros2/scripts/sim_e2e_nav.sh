#!/bin/bash
# End-to-end Nav2 test with the simulator, headless (no RViz):
#   sim (--lidar) -> adapter -> slam_toolbox + Nav2; sim_arm.py arms it in STATION (as the phone would);
#   goal_test sends Nav2 goals; Nav2's /cmd_vel drives the sim robot through the adapter.
#   bash ros2/scripts/sim_e2e_nav.sh [OUT_DIR] ["x y yaw" ...]
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HOME/sbr3-e2e/nav}"; shift || true
GOALS=("$@"); [ ${#GOALS[@]} -eq 0 ] && GOALS=("3.3 1.3 1.57" "0.0 0.0 3.14")
mkdir -p "$OUT"
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
pkill -f "station.sim"; pkill -f "sbr3_ros/adapter"; pkill -f async_slam_toolbox; pkill -f nav2; sleep 1
cleanup() { kill $LAUNCH_PID $SIM_PID 2>/dev/null; sleep 3; pkill -f "station.sim"; pkill -f async_slam_toolbox; pkill -f "sbr3_ros/adapter"; pkill -f nav2; pkill -f opennav; }
trap cleanup EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --http-port 8080 --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
SIM_PID=$!
sleep 3
ros2 launch sbr3_ros navigation.launch.py url:=ws://127.0.0.1:8080/ws rviz:=false > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 12
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
sleep 8      # lying down, every turn is flagged tilted: slam_toolbox starts the map once the robot stands
i=0
for g in "${GOALS[@]}"; do
  i=$((i + 1))
  set -- $g
  ros2 run sbr3_ros goal_test --ros-args -p x:=$1 -p y:=$2 -p yaw:=$3 -p timeout:=150.0 -p out:="$OUT/goal$i.json" 2>&1 \
    | grep -E "result|accepted|rejected|no_server" | tail -2
  ros2 run nav2_map_server map_saver_cli -f "$OUT/map_goal$i" --ros-args -p save_map_timeout:=10.0 > /dev/null 2>&1
  python3 "$HERE/map_png.py" "$OUT/map_goal$i.yaml" "$OUT/nav_goal$i.png" --truth --nav "$OUT/goal$i.json" \
    --title "Nav2 goal $i ($1, $2) on the slam_toolbox map, sim robot driven by /cmd_vel" | tail -1
  sleep 2
done
grep -E "scans [0-9]+" "$OUT/launch.log" | tail -1
grep -E "ignoring /cmd_vel" "$OUT/launch.log" | tail -3
echo "done: $OUT"
