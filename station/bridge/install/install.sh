#!/bin/sh
# Install the bridge as a systemd service on the Orange Pi.
#   cd ~/self_balancing_robot_3 && sudo station/bridge/install/install.sh [user]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
USER_NAME=${1:-${SUDO_USER:-$(id -un)}}
HOME_DIR=$(getent passwd "$USER_NAME" | cut -d: -f6)

python3 -c 'import sys; assert sys.version_info >= (3, 8), "python 3.8+ needed"'
usermod -aG dialout "$USER_NAME"            # access to /dev/ttyACM0
sed -e "s|@USER@|$USER_NAME|g" -e "s|@REPO@|$REPO|g" -e "s|@HOME@|$HOME_DIR|g" \
    "$HERE/sbr3-bridge.service" > /etc/systemd/system/sbr3-bridge.service
systemctl daemon-reload
systemctl enable sbr3-bridge
systemctl restart sbr3-bridge
sleep 2
systemctl --no-pager --lines=5 status sbr3-bridge || true
echo
echo "Ground station: http://$(hostname).local/   (logs: journalctl -u sbr3-bridge -f)"