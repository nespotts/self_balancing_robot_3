#!/bin/bash
# Copy ros2/sbr3_ros into ~/sbr3_ws/src and build it (a copy, so colcon never writes into the Windows checkout).
# SBR3_WS=/path: another workspace (e.g. the sim tests, next to a live session on ~/sbr3_ws).
#   bash ros2/scripts/build.sh
#   SBR3_WS=~/other_ws bash ros2/scripts/build.sh      another workspace (e.g. to test without touching a running one)
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PKG="$HERE/../sbr3_ros"
WS="${SBR3_WS:-$HOME/sbr3_ws}"
mkdir -p "$WS/src"
rm -rf "$WS/src/sbr3_ros"
cp -r "$PKG" "$WS/src/sbr3_ros"
find "$WS/src/sbr3_ros" -name __pycache__ -prune -exec rm -rf {} +
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select sbr3_ros 2>&1 | tail -3
echo "built: source $WS/install/setup.bash"
