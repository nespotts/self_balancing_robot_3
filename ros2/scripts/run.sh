#!/bin/bash
# Start the ROS side in WSL (used by the ros2/*.ps1 scripts):
#   bash ros2/scripts/run.sh mapping|nav|localize|teleop|adapter [URL|sim] [extra launch args...]
#   bash ros2/scripts/run.sh replay none bag:=... [launch args]
#   URL "sim": start the simulator inside WSL too (python3 -m station.sim --lidar, port 8080) and use
#   ws://127.0.0.1:8080/ws; mapping adds --tour (the sim drives a loop by itself). SBR3_SIM_ARGS: more sim options
#   (e.g. "--start 1200,-400,90"; the PowerShell scripts pass it through WSLENV).
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WHAT="${1:-mapping}"; shift || true
URL="${1:-ws://192.168.0.209:8080/ws}"; shift || true
bash "$HERE/build.sh" >/dev/null
source /opt/ros/jazzy/setup.bash
source "${SBR3_WS:-$HOME/sbr3_ws}/install/setup.bash"
# RViz on the GPU (WSLg's Mesa d3d12 driver -> the NVIDIA card) instead of llvmpipe software rendering (~3 CPU cores).
# SBR3_SOFT_GL=1 keeps software rendering (if the GPU path ever misbehaves).
if [ -z "$SBR3_SOFT_GL" ]; then
  export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
fi
SIM_PID=""
cleanup() { [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
if [ "$URL" = "sim" ]; then
  EXTRA="$SBR3_SIM_ARGS"; [ "$WHAT" = "mapping" ] && EXTRA="--tour $EXTRA"
  export SBR3_MAP_DIR="$HOME/sbr3-sim/maps" SBR3_LOG_DIR="$HOME/sbr3-sim/logs" \
         SBR3_PRESET_DIR="$HOME/sbr3-sim/presets" SBR3_FALLS_DIR="$HOME/sbr3-sim/falls"
  (cd "$REPO" && exec python3 -m station.sim --lidar $EXTRA --host 0.0.0.0 --http-port 8080 \
      --log-dir "$SBR3_LOG_DIR") > /tmp/sbr3_sim.log 2>&1 &
  SIM_PID=$!
  echo "sim started (pid $SIM_PID, log /tmp/sbr3_sim.log, options: ${EXTRA:-none}); web UI: http://localhost:8080/ from Windows"
  URL="ws://127.0.0.1:8080/ws"
  sleep 3
fi
case "$WHAT" in
  mapping)  ros2 launch sbr3_ros mapping.launch.py url:="$URL" "$@" ;;
  nav)      ros2 launch sbr3_ros navigation.launch.py url:="$URL" "$@" ;;
  localize) ros2 launch sbr3_ros localize.launch.py url:="$URL" "$@" ;;
  teleop)   ros2 launch sbr3_ros teleop.launch.py adapter:=true url:="$URL" "$@" ;;
  replay)   ros2 launch sbr3_ros replay.launch.py "$@" ;;
  adapter)  ros2 run sbr3_ros adapter --ros-args -p url:="$URL" ;;
  *) echo "usage: run.sh mapping|nav|localize|teleop|replay|adapter [URL|sim|none] [launch args]"; exit 2 ;;
esac
