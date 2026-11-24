#!/bin/bash
# End-to-end rosbag test with the simulator, headless:
#   1. live: sim --tour -> adapter -> slam_toolbox, record_run (record.sh) for DUR s, save the live map;
#   2. offline: everything off, replay the bag (replay.launch.py, adapter off, bag clock), rebuild the map, save it.
#   bash ros2/scripts/sim_e2e_bag.sh [OUT_DIR] [SECONDS] [RATE]
# Leaves OUT_DIR/live.{yaml,pgm,png}, offline.*, bag path in bag.txt, *.log
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HOME/sbr3-e2e/bag}"
DUR="${2:-150}"
RATE="${3:-1.0}"
mkdir -p "$OUT"
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
rm -rf "$SBR3_MAP_DIR"
stop_all() { pkill -f "station.sim"; pkill -f "sbr3_ros/adapter"; pkill -f async_slam_toolbox; pkill -f "ros2 bag"; \
             pkill -f bag_tf_filter; pkill -f "ros2 launch"; sleep 2; }
stop_all
trap stop_all EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --tour $SIM_EXTRA --http-port 8080 --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
ros2 launch sbr3_ros mapping.launch.py url:=ws://127.0.0.1:8080/ws rviz:=false > "$OUT/live_launch.log" 2>&1 &
sleep 4
bash "$HERE/record.sh" e2e "$DUR" > "$OUT/record.log" 2>&1
BAG=$(grep -m1 "recording to" "$OUT/record.log" | awk '{print $3}')
echo "$BAG" > "$OUT/bag.txt"
bash "$HERE/save_map.sh" live "$OUT" > "$OUT/save_live.log" 2>&1
python3 "$HERE/map_png.py" "$OUT/live.yaml" "$OUT/live.png" --truth --title "live slam_toolbox map (sim, ${DUR} s)" | tail -1
grep -E "Duration|Messages" "$OUT/record.log"
stop_all
echo "--- offline replay of $BAG at x$RATE (adapter off)"
ros2 launch sbr3_ros replay.launch.py bag:="$BAG" rate:="$RATE" rviz:=false > "$OUT/replay_launch.log" 2>&1 &
# the bag plays DUR / RATE s (+ the start delay); save just before it ends
sleep $(python3 -c "print(int($DUR / $RATE) + 2)")
bash "$HERE/save_map.sh" offline "$OUT" > "$OUT/save_offline.log" 2>&1
python3 "$HERE/map_png.py" "$OUT/offline.yaml" "$OUT/offline.png" --truth --title "offline: same bag replayed, slam_toolbox rebuilt (x$RATE)" | tail -1
grep -E "tf relayed" "$OUT/replay_launch.log" | tail -1
ls "$OUT"/offline.posegraph >/dev/null 2>&1 && echo "offline pose graph saved"
echo "done: $OUT"
