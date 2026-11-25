#!/bin/bash
# End-to-end test of ROS inside the station (#106), headless, isolated from any other ROS / sim running here:
# its own workspace (~/sbr3_ws_station), ROS_DOMAIN_ID 42, the sim on http port 8095 (link 5771, LD19 5772), and it
# only stops the processes it started. sim + localize (slam_toolbox on a saved map, loc_monitor, Nav2, keep-out) +
# the adapter with the station link; then station_e2e.py drives it all through the station's own API (a goal, the
# plan, cancel, go to a place, a patrol, the Manual check).
#   bash ros2/scripts/sim_e2e_station.sh MAP [OUT_DIR]          e.g. simroom (~/maps/simroom.posegraph)
#   HOLD=1 ...       keep everything running after the test (screenshots: station/tools/ros_shots.py), until killed
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
MAP="${1:?map name (in ~/maps) or path}"
OUT="${2:-$HOME/sbr3-e2e/station}"
PORT="${PORT:-8095}"
START="${START:-1000,500,90}"                 # world mm, deg
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export SBR3_WS="${SBR3_WS:-$HOME/sbr3_ws_station}"
mkdir -p "$OUT"
rm -f "$OUT"/*.json "$OUT"/*.log
bash "$HERE/build.sh" > "$OUT/build.log" 2>&1 || { tail -20 "$OUT/build.log"; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$SBR3_WS/install/setup.bash"
SIMDIR="$HOME/sbr3-sim-$PORT"
export SBR3_MAP_DIR="$SIMDIR/maps" SBR3_PRESET_DIR="$SIMDIR/presets" SBR3_FALLS_DIR="$SIMDIR/falls"
rm -rf "$SBR3_MAP_DIR"
PIDS=()
stop_all() {
  for p in "${PIDS[@]}"; do pkill -INT -P "$p" 2>/dev/null; kill -INT "$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do pkill -KILL -P "$p" 2>/dev/null; kill -KILL "$p" 2>/dev/null; done
}
trap stop_all EXIT INT TERM
(cd "$REPO" && exec python3 -m station.sim --lidar 5772 --listen 127.0.0.1:5771 --start "$START" --host 0.0.0.0 \
   --http-port "$PORT" --log-dir "$SIMDIR/logs") > "$OUT/sim.log" 2>&1 &
PIDS+=($!)
sleep 3
setsid ros2 launch sbr3_ros localize.launch.py url:="ws://127.0.0.1:$PORT/ws" map:="$MAP" rviz:=false \
  > "$OUT/launch.log" 2>&1 &
PIDS+=($!)
echo "sim on :$PORT, ROS domain $ROS_DOMAIN_ID, workspace $SBR3_WS; logs in $OUT"
python3 "$HERE/station_e2e.py" --url "ws://127.0.0.1:$PORT/ws" --start "$START" --out "$OUT/result.json"
RC=$?
if [ -n "$HOLD" ]; then
  echo "HOLD: still running (web UI http://localhost:$PORT/ from Windows); kill this script to stop"
  while true; do sleep 5; done
fi
exit $RC
