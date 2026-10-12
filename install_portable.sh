#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$ROS_SETUP" ]] || { echo "缺少 ROS2 Humble: $ROS_SETUP" >&2; exit 1; }
python3 -m pip install --user -r "$ROOT/requirements-ros.txt"
python3 -m pip install --user -r "$ROOT/requirements-export.txt"
set +u
source "$ROS_SETUP"
set -u
cd "$ROOT"
colcon build --symlink-install
cat <<EOF3
安装完成。启动前执行：
  source $ROS_SETUP
  source $ROOT/install/setup.bash
CAN配置脚本：
  sudo bash $ROOT/tools/can_config_auto.sh
EOF3
