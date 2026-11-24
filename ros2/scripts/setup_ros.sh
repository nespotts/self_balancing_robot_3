#!/bin/bash
# One-time (idempotent) setup of ROS 2 Jazzy in the WSL distro. Run as root:
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/.../ros2/scripts/setup_ros.sh
# ros2/setup_wsl.ps1 does this for you.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
USER_NAME="${SBR3_USER:-sbr3}"

# 1. normal user with passwordless sudo, and the WSL default user
if ! id "$USER_NAME" >/dev/null 2>&1; then
  useradd -m -s /bin/bash -G sudo "$USER_NAME"
fi
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-$USER_NAME
chmod 440 /etc/sudoers.d/90-$USER_NAME
if ! grep -q "^default=$USER_NAME" /etc/wsl.conf 2>/dev/null; then
  cat > /etc/wsl.conf <<EOF
[user]
default=$USER_NAME
EOF
fi

# 2. ROS 2 apt repo (official ros-apt-source package)
apt-get update -q
apt-get install -y -q software-properties-common curl ca-certificates locales
locale-gen en_US.UTF-8 >/dev/null
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe >/dev/null
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}')
  . /etc/os-release
  curl -sSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${UBUNTU_CODENAME}_all.deb"
  apt-get install -y -q /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi
apt-get update -q

# 3. ROS 2 Jazzy + SLAM + Nav2 + build tools + the WebSocket client the adapter uses
apt-get install -y -q \
  ros-jazzy-desktop ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  python3-colcon-common-extensions python3-rosdep python3-websocket python3-pil

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi
sudo -u "$USER_NAME" rosdep update >/dev/null 2>&1 || true

# 4. source ROS in the user's shells
BRC=/home/$USER_NAME/.bashrc
grep -q "/opt/ros/jazzy/setup.bash" "$BRC" || echo "source /opt/ros/jazzy/setup.bash" >> "$BRC"
grep -q "sbr3_ws/install/setup.bash" "$BRC" || \
  echo '[ -f ~/sbr3_ws/install/setup.bash ] && source ~/sbr3_ws/install/setup.bash' >> "$BRC"
echo "setup_ros.sh: done"
