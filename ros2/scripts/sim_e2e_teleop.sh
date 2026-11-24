#!/bin/bash
# Gamepad teleop wiring test with the simulator (no real pad): teleop.launch.py (joy_node finds no device) + adapter;
# fake /joy messages stand in for the pad: stick forward WITHOUT the deadman (must not move), then with LB held
# (moves), then LB released (stops). Checks /cmd_vel and the sim robot's true motion.
#   bash ros2/scripts/sim_e2e_teleop.sh [OUT_DIR]
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HOME/sbr3-e2e/teleop}"
mkdir -p "$OUT"
bash "$HERE/build.sh" >/dev/null || exit 1
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
stop_all() { pkill -f "ros2 launch"; pkill -f "station.sim"; pkill -f "/opt/ros/jazzy/lib/"; pkill -f "sbr3_ws/install/"; \
             pkill -f "ros2 topic"; sleep 2; }
stop_all
trap stop_all EXIT
(cd "$REPO" && exec python3 -m station.sim --lidar --truth-log "$OUT/truth.csv" --http-port 8080 \
   --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
ros2 launch sbr3_ros teleop.launch.py adapter:=true url:=ws://127.0.0.1:8080/ws > "$OUT/launch.log" 2>&1 &
sleep 8
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
sleep 3
ros2 node list > "$OUT/nodes.txt"
ros2 topic info -v /cmd_vel | grep -E "Node name|Publisher count|Subscription count" > "$OUT/cmd_vel_info.txt"
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --csv > "$OUT/cmd_vel.csv" 2>/dev/null &
pos() { tail -1 "$OUT/truth.csv" | awk -F, '{printf "%.0f %.0f", $2, $3}'; }
joy() {   # $1 = LB (0/1), $2 = left stick up (0..1), seconds
  timeout "$3" ros2 topic pub -r 20 /joy sensor_msgs/msg/Joy \
    "{axes: [0.0, $2, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, $1, 0, 0, 0, 0, 0, 0]}" > /dev/null 2>&1
}
P0=$(pos); joy 0 1.0 3; sleep 1; P1=$(pos)
echo "stick forward, NO deadman, 3 s:  $P0 -> $P1"
joy 1 1.0 4; P2=$(pos); sleep 2; P3=$(pos)
echo "stick forward + LB held, 4 s:    $P1 -> $P2 (then released, 2 s later: $P3)"
pkill -f "ros2 topic echo"
echo "/cmd_vel messages: $(wc -l < "$OUT/cmd_vel.csv"), max linear.x: $(awk -F, 'BEGIN{m=0}{if($1>m)m=$1}END{print m}' "$OUT/cmd_vel.csv")"
cat "$OUT/cmd_vel_info.txt"
grep -E "joy_node|No joystick|device" "$OUT/launch.log" | head -3
grep -E "cmd sent" "$OUT/launch.log" | tail -1
echo "done: $OUT"
