#!/bin/sh
# Install the bridge as a systemd USER service on the Orange Pi (no sudo needed).
#   cd ~/self_balancing_robot_3 && sh station/bridge/install/install.sh
#
# One-time setup that does need sudo (ask the owner first; this script never runs sudo):
#   sudo loginctl enable-linger $USER     # start user services at boot, keep them after logout
#   sudo usermod -aG dialout $USER        # only if `id -nG` lacks dialout: access to /dev/ttyACM0
#
# Afterwards, no sudo:
#   systemctl --user status sbr3-bridge       journalctl --user -u sbr3-bridge -f
#   systemctl --user restart sbr3-bridge      (e.g. after git pull)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
USER_NAME=$(id -un)
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}

python3 -c 'import sys; assert sys.version_info >= (3, 8), "python 3.8+ needed"'
id -nG | grep -qw dialout || echo "WARNING: $USER_NAME is not in group dialout: sudo usermod -aG dialout $USER_NAME (then log in again)"
loginctl show-user "$USER_NAME" -p Linger 2>/dev/null | grep -q yes || \
  echo "NOTE: linger is off, so the bridge stops at logout and does not start at boot. One-time: sudo loginctl enable-linger $USER_NAME"

UNIT_DIR=${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user
mkdir -p "$UNIT_DIR" "$HOME/sbr3-logs"
sed -e "s|@USER@|$USER_NAME|g" -e "s|@REPO@|$REPO|g" "$HERE/sbr3-bridge.service" > "$UNIT_DIR/sbr3-bridge.service"
systemctl --user daemon-reload
systemctl --user enable sbr3-bridge
systemctl --user restart sbr3-bridge
sleep 2
systemctl --user --no-pager --lines=5 status sbr3-bridge || true
echo
echo "Ground station: http://$(hostname).local:8080/   (logs: journalctl --user -u sbr3-bridge -f)"
