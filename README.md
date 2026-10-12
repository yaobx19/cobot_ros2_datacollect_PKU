# Cobot ROS2 数采工作空间

此目录包含 ROS2 Humble 数采源码、30 Hz 采集入口、HDF5 导出工具和 CAN 配置脚本。`build/`、`install/`、`log/` 不随包迁移，由目标主机重新生成。

## 目录

- `src/cobot_magic_ros2/`：ROS2 包源码、双臂节点、RealSense 节点和 launch 文件
- `scripts/interactive_collect_data.sh`：推荐的 30 Hz 交互采集入口
- `scripts/collect_data_until_key.py`：单条采集器
- `tools/export_hdf5_episode.py`：HDF5 视频/诊断导出
- `tools/can_config_auto.sh`：CAN 接口配置
- `requirements-ros.txt`、`requirements-export.txt`：Python 依赖

## 目标主机安装

要求 Ubuntu 22.04、ROS2 Humble、可用的 RealSense 和双 CAN 设备。将整个目录复制到目标主机后执行：

```bash
cd cobot_ros2_datacollect_PKU_portable
bash install_portable.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如目标主机没有 ROS2 Humble，先安装官方 ROS2 Humble，再运行安装脚本。若目标主机使用已有虚拟环境，可设置 `EXPORT_PYTHON=/path/to/python` 后运行采集入口。

## 启动硬件

先按现场安全流程准备机械臂，然后配置 CAN：

```bash
sudo bash tools/can_config_auto.sh
ip -br link show type can
```

确认 `can_left`、`can_right` 均为 UP。启动硬件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch cobot_magic_ros2 collect_stack_split_cameras.launch.py \
  mode:=0 auto_enable:=false \
  camera_f_serial:=<front-camera-serial> \
  camera_l_serial:=<left-camera-serial> \
  camera_r_serial:=<right-camera-serial> \
  camera_width:=640 camera_height:=480 camera_fps:=30 \
  publish_width:=640 publish_height:=480 publish_fps:=30
```

换相机或换机器时，先核对三个序列号；换 CAN 适配器时设置 `CAN_LEFT_SERIAL`、`CAN_RIGHT_SERIAL` 或对应 `CAN_LEFT_BUS`、`CAN_RIGHT_BUS`。

## 采集

另开一个终端：

```bash
cd cobot_ros2_datacollect_PKU_portable
export EXPORT_PYTHON=python3
bash scripts/interactive_collect_data.sh \
  --dataset-dir ~/data \
  --export-dir ~/data_mp4 \
  --task-name DLO_Stretch_30Hz \
  --frame-rate 30 \
  --max-timesteps 1000
```

采集时按 `S` 保存；完成导出后输入 `C` 继续、`R` 重采、`Q` 退出。开始正式采集前，用 `ros2 topic hz` 检查三路相机和四路关节话题。

## 迁移验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix cobot_magic_ros2
python3 -c 'import h5py, numpy, rclpy'

python3 -c 'import cv2, matplotlib, h5py, numpy'  # 导出依赖
bash -n scripts/interactive_collect_data.sh
```

再录制一条短 episode，确认 HDF5、三个视频和摘要文件均生成。此包不包含已有数据集，也不包含 pi0.5 推理模型和权重。
