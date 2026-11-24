#!/bin/bash
# Start the ROS side in WSL (used by run_mapping.ps1 / run_nav.ps1):
#   bash ros2/scripts/run.sh mapping|nav [URL|sim] [extra launch args...]
#   URL "sim": start the simulator inside WSL too (python3 -m station.sim --lidar, port 8080) and use
#   ws://127.0.0.1:8080/ws; mapping adds --tour (the sim drives a loop by itself).
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WHAT="${1:-mapping}"; shift || true
URL="${1:-ws://192.168.0.209:8080/ws}"; shift || true
bash "$HERE/build.sh" >/dev/null
source /opt/ros/jazzy/setup.bash
source "$HOME/sbr3_ws/install/setup.bash"
SIM_PID=""
cleanup() { [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
if [ "$URL" = "sim" ]; then
  EXTRA=""; [ "$WHAT" = "mapping" ] && EXTRA="--tour"
  export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_LOG_DIR="$HOME/sbr3-sim/logs" \
         SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
  (cd "$REPO" && exec python3 -m station.sim --lidar $EXTRA --host 0.0.0.0 --http-port 8080 \
      --log-dir "$SBR3_LOG_DIR") > /tmp/sbr3_sim.log 2>&1 &
  SIM_PID=$!
  echo "sim started (pid $SIM_PID, log /tmp/sbr3_sim.log); web UI: http://localhost:8080/ from Windows"
  URL="ws://127.0.0.1:8080/ws"
  sleep 3
fi
case "$WHAT" in
  mapping) ros2 launch sbr3_ros mapping.launch.py url:="$URL" "$@" ;;
  nav)     ros2 launch sbr3_ros navigation.launch.py url:="$URL" "$@" ;;
  adapter) ros2 run sbr3_ros adapter --ros-args -p url:="$URL" ;;
  *) echo "usage: run.sh mapping|nav|adapter [URL|sim] [launch args]"; exit 2 ;;
esac
