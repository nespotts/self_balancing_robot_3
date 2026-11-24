#!/bin/bash
# End-to-end keep-out test with the simulator, headless: station zones (drawn as on the phone) -> keepout_mask ->
# Nav2 KeepoutFilter. Same goal three times:
#   1. a virtual wall across the room's south half: the plan must go around its end;
#   2. more walls close that gap and the one north of the box: unreachable, must fail cleanly (no crossing);
#   3. both walls deleted: the direct route again (the mask follows the zones live).
#   bash ros2/scripts/sim_e2e_keepout.sh MAP_BASE [OUT_DIR]
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
MAP="${1:?map base}"
OUT="${2:-$HOME/sbr3-e2e/keepout}"
GOAL="${GOAL:-3.0 -0.6 0.0}"
mkdir -p "$OUT"
rm -f "$OUT"/*.csv "$OUT"/*.json
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
stop_all() { pkill -f "ros2 launch"; pkill -f "station.sim"; pkill -f "/opt/ros/jazzy/lib/"; pkill -f "sbr3_ws/install/"; \
             sleep 2; }
stop_all
trap stop_all EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --truth-log "$OUT/truth.csv" --http-port 8080 \
   --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
ros2 launch sbr3_ros localize.launch.py url:=ws://127.0.0.1:8080/ws map:="$MAP" initial_pose:="0,0,0" keepout:=true \
  rviz:=false > "$OUT/launch.log" 2>&1 &
sleep 10
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
sleep 6
set -- $GOAL
goal() {   # $1 = run index
  ros2 run sbr3_ros goal_test --ros-args -p x:=$GX -p y:=$GY -p yaw:=$GYAW -p timeout:=120.0 -p out:="$OUT/goal$1.json" \
    2>&1 | grep -E "result" | tail -1
}
GX=$1; GY=$2; GYAW=$3
echo "--- 1. virtual wall x = 1.5 m from the south wall up to y = -0.2 m"
python3 "$HERE/sim_station.py" zone wall "1500,-1600 1500,-200" | tail -1
sleep 4; grep -E "keepout mask" "$OUT/launch.log" | tail -1
goal 1
ros2 run sbr3_ros goal_test --ros-args -p x:=0.0 -p y:=0.0 -p yaw:=0.0 -p timeout:=120.0 -p out:="$OUT/back.json" \
  2>&1 | grep -E "result" | tail -1     # back to the west side (around the wall again)
echo "--- 2. walls close the gap up to the box and the gap north of it: unreachable"
python3 "$HERE/sim_station.py" zone wall "1500,-250 1500,750" | tail -1
python3 "$HERE/sim_station.py" zone wall "1500,1050 1500,2100" | tail -1
sleep 4; grep -E "keepout mask" "$OUT/launch.log" | tail -1
T0=$(date +%s.%N)
goal 2
python3 -c "import time; print('failed after %.1f s' % (time.time() - $T0))"
echo "--- 3. walls deleted"
python3 "$HERE/sim_station.py" clear | tail -1
sleep 4; grep -E "keepout mask" "$OUT/launch.log" | tail -1
goal 3
python3 "$HERE/sim_station.py" list > "$OUT/station.json"
python3 "$HERE/keepout_report.py" "$OUT" --map "$MAP.yaml" | tail -8
echo "done: $OUT"
