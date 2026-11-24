#!/bin/bash
# End-to-end places test with the simulator, headless: the robot starts away from the saved map's origin (so the
# station's odometry / map frame and the ROS map frame differ), places are saved in the station as the phone does
# (odometry mm), then go_to Kitchen and a patrol (loops, dwell) drive there with Nav2.
#   bash ros2/scripts/sim_e2e_places.sh MAP_BASE [OUT_DIR]
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
MAP="${1:?map base}"
OUT="${2:-$HOME/sbr3-e2e/places}"
START="${START:-1000,500,90}"                 # world mm, deg
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
(cd "$REPO" && exec python3 -m station.sim --lidar --start "$START" --truth-log "$OUT/truth.csv" --http-port 8080 \
   --log-dir "$HOME/sbr3-sim/logs") > "$OUT/sim.log" 2>&1 &
sleep 3
# places in WORLD mm -> odometry mm (what the phone would send, standing at the start pose)
odom() { python3 -c "
import math,sys
sx,sy,sd=[float(v) for v in '$START'.split(',')]
x,y=float(sys.argv[1]),float(sys.argv[2]); c,s=math.cos(math.radians(-sd)),math.sin(math.radians(-sd))
dx,dy=x-sx,y-sy; print('%.0f %.0f'%(c*dx-s*dy,s*dx+c*dy))" "$1" "$2"; }
declare -A WORLD=([Kitchen]="3300 1300" [Couch]="700 -700" [Window]="-400 400")
ros2 launch sbr3_ros localize.launch.py url:=ws://127.0.0.1:8080/ws map:="$MAP" rviz:=false > "$OUT/launch.log" 2>&1 &
ros2 run sbr3_ros pose_logger --ros-args -p out:="$OUT/track.csv" > /dev/null 2>&1 &
sleep 10
python3 "$HERE/sim_arm.py" ws://127.0.0.1:8080/ws || exit 1
sleep 8
# saved once the robot stands on the station's map, as on the phone
for n in Kitchen Couch Window; do
  python3 "$HERE/sim_station.py" place "$n" $(odom ${WORLD[$n]}) | tail -1
done
sleep 7
timeout 10 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /station/places \
  std_msgs/msg/String > "$OUT/places_topic.txt" 2>&1
echo "--- go_to Kitchen"
ros2 run sbr3_ros go_to --ros-args -p place:=kitchen -p timeout:=120.0 -p out:="$OUT/goto_kitchen.json" 2>&1 \
  | grep -E "place|result" | tail -2
echo "--- patrol Couch, Window, Kitchen x2, dwell 3 s"
ros2 run sbr3_ros patrol --ros-args -p places:="Couch,Window,Kitchen" -p loops:=2 -p dwell:=3.0 \
  -p out:="$OUT/patrol.json" 2>&1 | grep -E "map \(|dwell|patrol|left" | tail -14
sleep 2
python3 "$HERE/places_report.py" "$OUT" --map "$MAP.yaml" --start "$START" \
  --place "Kitchen=${WORLD[Kitchen]}" --place "Couch=${WORLD[Couch]}" --place "Window=${WORLD[Window]}" | tail -16
echo "done: $OUT"
