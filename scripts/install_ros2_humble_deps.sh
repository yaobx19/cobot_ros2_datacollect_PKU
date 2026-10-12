#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-std-srvs

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

python3 -m pip install --user -r "$(dirname "$0")/../src/cobot_magic_ros2/requirements.txt"

echo "ROS2 Humble dependencies are installed. Build with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd ~/Documents/rm-sia/cobot_ros2_datacollect_PKU"
echo "  colcon build --symlink-install"
