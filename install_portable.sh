#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$ROS_SETUP" ]] || { echo "缺少 ROS2 Humble: $ROS_SETUP" >&2; exit 1; }
PYTHON_BIN="${PYTHON_BIN:-python3}"
PYTHON_BIN="$(command -v "$PYTHON_BIN" 2>/dev/null || true)"
[[ -x "$PYTHON_BIN" ]] || { echo "找不到 Python: ${PYTHON_BIN:-python3}" >&2; exit 1; }
"$PYTHON_BIN" -m pip install --user -r "$ROOT/requirements-ros.txt"
"$PYTHON_BIN" -m pip install --user -r "$ROOT/requirements-export.txt"
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
