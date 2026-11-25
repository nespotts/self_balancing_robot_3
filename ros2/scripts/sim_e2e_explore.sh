#!/bin/bash
# End-to-end explore test with the simulator, headless, and ISOLATED from a live ROS session on this PC:
#   its own ROS_DOMAIN_ID (43), colcon workspace (~/sbr3_ws_explore) and sim ports (http 8096, link 5861, LD19 5862);
#   it only stops the processes it started (their process groups), never by name.
#   sim (--lidar [--maze]) -> adapter -> slam_toolbox (mapping) + Nav2 + explore (autostart); sim_arm.py arms it;
#   explore_watch.py logs the status + map snapshots until the explorer is done and saved the map;
#   explore_report.py: coverage vs the true free area, time, distance, goals, PNGs.
#   bash ros2/scripts/sim_e2e_explore.sh [OUT_DIR] [room|maze] [MAX_S]
#   KEEPOUT=false: no keepout filter (the maze: the sim builds its walls for real, so they need not be keep-out too)
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HOME/sbr3-e2e/explore}"
WORLD="${2:-room}"
MAXS="${3:-900}"
export ROS_DOMAIN_ID="${SBR3_TEST_DOMAIN:-43}"
export SBR3_WS="${SBR3_WS:-$HOME/sbr3_ws_explore}"
HTTP="${SBR3_TEST_HTTP:-8096}"; LINK="${SBR3_TEST_LINK:-5861}"; LIDAR="${SBR3_TEST_LIDAR:-5862}"
for p in $HTTP $LINK $LIDAR; do
  if ss -ltn 2>/dev/null | grep -q ":$p "; then echo "port $p is in use: another test running?"; exit 1; fi
done
rm -rf "$OUT"; mkdir -p "$OUT/maps" "$OUT/sim"
bash "$HERE/build.sh" > "$OUT/build.log" 2>&1 || { cat "$OUT/build.log"; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$SBR3_WS/install/setup.bash"
export SBR3_MAP_DIR="$OUT/sim/maps" SBR3_PRESET_DIR="$OUT/sim/presets" SBR3_FALLS_DIR="$OUT/sim/falls"
SIMOPT=""; [ "$WORLD" = "maze" ] && SIMOPT="--maze"
PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done
  sleep 6
  for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
}
trap cleanup EXIT
(cd "$REPO" && exec setsid python3 -m station.sim --lidar "$LIDAR" --listen "127.0.0.1:$LINK" --http-port "$HTTP" \
   $SIMOPT --truth-log "$OUT/truth.csv" --log-dir "$OUT/sim/logs") > "$OUT/sim.log" 2>&1 &
PIDS+=($!)
sleep 4
URL="ws://127.0.0.1:$HTTP/ws"
if [ "$WORLD" = "maze" ]; then
  python3 - "$URL" "$OUT/zones.json" <<'EOF'
import json, sys, time
import websocket
ws = websocket.create_connection(sys.argv[1], timeout=5, suppress_origin=True)
ws.send(json.dumps({"t": "hi", "name": "e2e_zones"}))
end = time.time() + 8
while time.time() < end:
    m = json.loads(ws.recv())
    if m.get("t") == "zones" and m.get("items"):
        json.dump(m, open(sys.argv[2], "w"))
        print("maze: %d zone items" % len(m["items"]))
        break
ws.close()
EOF
fi
setsid ros2 launch sbr3_ros explore.launch.py url:="$URL" rviz:=false autostart:=true map_dir:="$OUT/maps" keepout:="${KEEPOUT:-true}" \
  max_duration_s:="$MAXS" > "$OUT/launch.log" 2>&1 &
PIDS+=($!)
sleep 12
python3 "$HERE/sim_arm.py" "$URL" || exit 1
T0=$(date +%s)
python3 "$HERE/explore_watch.py" "$OUT" --timeout $((MAXS + 120)) --snap "${SNAPS:-20,60,120}" | tail -1
echo "wall time: $(( $(date +%s) - T0 )) s"
grep -cE "goal [0-9]+:" "$OUT/launch.log" | sed 's/^/goals sent: /'
grep -E "\[explore\].*(failed|done|stopped|saved)" "$OUT/launch.log" | tail -8
python3 "$HERE/explore_report.py" "$OUT" $([ "$WORLD" = "maze" ] && echo --maze) | tail -30
echo "done: $OUT"
