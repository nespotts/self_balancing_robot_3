#!/bin/sh
# Flash the Teensy from the Pi while the bridge keeps running (issue #2). No sudo needed.
#   station/bridge/install/flash-teensy.sh firmware.hex
#
# Release-file contract (keep stable, see station/README.md):
#   - while /tmp/sbr3-bridge.release exists the bridge closes /dev/ttyACM0 (checked every 0.25 s,
#     so wait >= 1 s after creating it); it re-opens the port by itself once the file is gone
#   - a file older than 15 min is ignored as stale: touch it again to hold the port longer
#   - optional check: GET http://localhost:8080/api/serial -> {"released": true, ...}
set -e
HEX=${1:?usage: flash-teensy.sh firmware.hex}
REL=${SBR3_RELEASE_FILE:-/tmp/sbr3-bridge.release}
touch "$REL"
trap 'rm -f "$REL"' EXIT
sleep 1.0
teensy_loader_cli --mcu=TEENSY41 -w -s -v "$HEX"
