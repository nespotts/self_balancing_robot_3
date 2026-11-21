#!/bin/sh
# Flash the Teensy from the Pi while the bridge keeps running (issue #2).
#   station/bridge/install/flash-teensy.sh firmware.hex
# The bridge lets go of /dev/ttyACM0 while the release file exists (checked once a second)
# and reconnects by itself when the Teensy comes back.
set -e
HEX=${1:?usage: flash-teensy.sh firmware.hex}
REL=${SBR3_RELEASE_FILE:-/tmp/sbr3-bridge.release}
touch "$REL"
trap 'rm -f "$REL"' EXIT
sleep 1.5
teensy_loader_cli --mcu=TEENSY41 -w -s -v "$HEX"