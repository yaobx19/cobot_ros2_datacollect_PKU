#!/usr/bin/env python3
import argparse
import collections
import math
import os
import sys
import time
from collections import deque

import h5py
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, JointState


REQUIRED_STREAMS = (
    ("img_front", "前相机"),
    ("img_left", "左腕相机"),
    ("img_right", "右腕相机"),
    ("master_left", "主臂左侧"),
    ("master_right", "主臂右侧"),
    ("puppet_left", "从臂左侧"),
    ("puppet_right", "从臂右侧"),
)
STREAM_STALE_AFTER_SECONDS = 2.0


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_to_numpy(msg):
    dtype = np.uint8
    channels = 3
    if msg.encoding in ("mono8", "8UC1"):
        channels = 1
    elif msg.encoding in ("16UC1", "mono16"):
        dtype = np.uint16
        channels = 1
    elif msg.encoding not in ("rgb8", "bgr8", "8UC3"):
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        return arr.reshape((msg.height, msg.width))
    return arr.reshape((msg.height, msg.width, channels))


def save_data(args, timesteps, actions, dataset_path):
    data_size = len(actions)
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/action": [],
        "/base_action": [],
    }

    for cam_name in args.camera_names:
        data_dict[f"/observations/images/{cam_name}"] = []

    for ts, action in zip(timesteps, actions):
        data_dict["/observations/qpos"].append(ts["qpos"])
        data_dict["/observations/qvel"].append(ts["qvel"])
        data_dict["/observations/effort"].append(ts["effort"])
        data_dict["/action"].append(action)
        data_dict["/base_action"].append(ts["base_vel"])
        for cam_name in args.camera_names:
            data_dict[f"/observations/images/{cam_name}"].append(ts["images"][cam_name])

    t0 = time.time()
    height, width = args.image_height, args.image_width
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["compress"] = False
        obs = root.create_group("observations")
        image = obs.create_group("images")
        for cam_name in args.camera_names:
            image.create_dataset(
                cam_name,
                (data_size, height, width, 3),
                dtype="uint8",
                chunks=(1, height, width, 3),
            )
        obs.create_dataset("qpos", (data_size, 14), dtype="float32")
        obs.create_dataset("qvel", (data_size, 14), dtype="float32")
        obs.create_dataset("effort", (data_size, 14), dtype="float32")
        root.create_dataset("action", (data_size, 14), dtype="float32")
        root.create_dataset("base_action", (data_size, 2), dtype="float32")

        for name, array in data_dict.items():
            root[name][...] = array
    print(f"\033[32m\nSaving: {time.time() - t0:.1f} secs. {dataset_path}\033[0m\n")


class Ros2Collector(Node):
    def __init__(self, args):
        super().__init__("record_episodes")
        self.args = args
        self.queues = {
            "img_left": deque(),
            "img_right": deque(),
            "img_front": deque(),
            "master_left": deque(),
            "master_right": deque(),
            "puppet_left": deque(),
            "puppet_right": deque(),
            "base": deque(),
        }
        self.last_received_at = {name: None for name, _ in REQUIRED_STREAMS}
        self.create_subscription(Image, args.img_left_topic, self.callback("img_left"), qos_profile_sensor_data)
        self.create_subscription(Image, args.img_right_topic, self.callback("img_right"), qos_profile_sensor_data)
        self.create_subscription(Image, args.img_front_topic, self.callback("img_front"), qos_profile_sensor_data)
        self.create_subscription(JointState, args.master_arm_left_topic, self.callback("master_left"), 1000)
        self.create_subscription(JointState, args.master_arm_right_topic, self.callback("master_right"), 1000)
        self.create_subscription(JointState, args.puppet_arm_left_topic, self.callback("puppet_left"), 1000)
        self.create_subscription(JointState, args.puppet_arm_right_topic, self.callback("puppet_right"), 1000)
        if getattr(args, "use_robot_base", False):
            self.create_subscription(Odometry, args.robot_base_topic, self.callback("base"), 1000)

    def callback(self, queue_name):
        def _callback(msg):
            queue = self.queues[queue_name]
            if len(queue) >= 2000:
                queue.popleft()
            queue.append(msg)
            if queue_name in self.last_received_at:
                self.last_received_at[queue_name] = time.monotonic()

        return _callback

    def missing_streams(self):
        """Return human-readable names for streams that have not published yet."""
        return [label for name, label in REQUIRED_STREAMS if not self.queues[name]]

    def unhealthy_streams(self, max_age=STREAM_STALE_AFTER_SECONDS):
        now = time.monotonic()
        return [
            label
            for name, label in REQUIRED_STREAMS
            if not self.queues[name]
            or self.last_received_at[name] is None
            or now - self.last_received_at[name] > max_age
        ]

    def newest_frame_time(self):
        required = ["img_left", "img_right", "img_front"]
        if any(len(self.queues[name]) == 0 for name in required):
            return None
        return min(stamp_to_sec(self.queues[name][-1].header.stamp) for name in required)

    def pop_at_or_after(self, name, frame_time):
        queue = self.queues[name]
        if len(queue) == 0 or stamp_to_sec(queue[-1].header.stamp) < frame_time:
            return None
        while len(queue) > 1 and stamp_to_sec(queue[0].header.stamp) < frame_time:
            queue.popleft()
        return queue.popleft()

    def latest(self, name):
        queue = self.queues[name]
        if len(queue) == 0:
            return None
        msg = queue[-1]
        queue.clear()
        queue.append(msg)
        return msg

    def get_frame(self):
        if self.unhealthy_streams():
            return None
        names = ["img_left", "img_right", "img_front", "master_left", "master_right", "puppet_left", "puppet_right"]
        if getattr(self.args, "use_robot_base", False):
            names.append("base")
        popped = {name: self.latest(name) for name in names}
        if any(value is None for value in popped.values()):
            return None

        images = {
            self.args.camera_names[0]: image_to_numpy(popped["img_front"]),
            self.args.camera_names[1]: image_to_numpy(popped["img_left"]),
            self.args.camera_names[2]: image_to_numpy(popped["img_right"]),
        }
        puppet_left = popped["puppet_left"]
        puppet_right = popped["puppet_right"]
        master_left = popped["master_left"]
        master_right = popped["master_right"]

        obs = collections.OrderedDict()
        obs["images"] = images
        obs["qpos"] = np.concatenate((np.array(puppet_left.position), np.array(puppet_right.position)), axis=0)
        obs["qvel"] = np.concatenate((np.array(puppet_left.velocity), np.array(puppet_right.velocity)), axis=0)
        obs["effort"] = np.concatenate((np.array(puppet_left.effort), np.array(puppet_right.effort)), axis=0)
        if getattr(self.args, "use_robot_base", False):
            base = popped["base"]
            obs["base_vel"] = [base.twist.twist.linear.x, base.twist.twist.angular.z]
        else:
            obs["base_vel"] = [0.0, 0.0]
        action = np.concatenate((np.array(master_left.position), np.array(master_right.position)), axis=0)
        return obs, action

    def process(self):
        timesteps = []
        actions = []
        count = 0
        sleep_sec = 1.0 / self.args.frame_rate
        sync_issue_started = None
        last_sync_report = 0.0

        while rclpy.ok() and count < self.args.max_timesteps + 1:
            spin_until = time.time() + min(0.02, sleep_sec)
            while rclpy.ok() and time.time() < spin_until:
                rclpy.spin_once(self, timeout_sec=0.001)
            result = self.get_frame()
            if result is None:
                now = time.monotonic()
                if sync_issue_started is None:
                    sync_issue_started = now
                    print("sync fail: waiting for camera and arm data...", flush=True)
                if now - last_sync_report >= 2.0:
                    missing = self.unhealthy_streams()
                    detail = "、".join(missing) if missing else "消息尚未组成完整帧"
                    print(f"Still waiting for synchronized data: {detail}", flush=True)
                    last_sync_report = now
                if now - sync_issue_started >= self.args.sync_timeout:
                    missing = self.unhealthy_streams()
                    detail = "、".join(missing) if missing else "消息尚未组成完整帧"
                    raise RuntimeError(
                        f"同步等待超过 {self.args.sync_timeout:g} 秒：{detail}。"
                        "请检查 collection stack、相机连接、CAN 和话题名称。"
                    )
                time.sleep(sleep_sec)
                continue

            sync_issue_started = None
            count += 1
            obs, action = result
            if count == 1:
                timesteps.append(obs)
                continue
            timesteps.append(obs)
            actions.append(action)
            print("Frame data: ", count, flush=True)
            time.sleep(sleep_sec)

        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        return timesteps[1:], actions


def parse_args(argv=None):
    argv = remove_ros_args(args=argv if argv is not None else sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_dir", default="./data")
    parser.add_argument("--task_name", default="aloha_mobile_dummy")
    parser.add_argument("--episode_idx", type=int, default=0)
    parser.add_argument("--max_timesteps", type=int, default=500)
    parser.add_argument("--camera_names", nargs="+", default=["cam_high", "cam_left_wrist", "cam_right_wrist"])
    parser.add_argument("--img_front_topic", default="/camera_f/color/image_raw")
    parser.add_argument("--img_left_topic", default="/camera_l/color/image_raw")
    parser.add_argument("--img_right_topic", default="/camera_r/color/image_raw")
    parser.add_argument("--master_arm_left_topic", default="/master/joint_left")
    parser.add_argument("--master_arm_right_topic", default="/master/joint_right")
    parser.add_argument("--puppet_arm_left_topic", default="/puppet/joint_left")
    parser.add_argument("--puppet_arm_right_topic", default="/puppet/joint_right")
    parser.add_argument("--robot_base_topic", default="/odom")
    parser.add_argument("--use_robot_base", action="store_true")
    parser.add_argument("--frame_rate", type=int, default=30)
    parser.add_argument("--image_width", type=int, default=640)
    parser.add_argument("--image_height", type=int, default=480)
    parser.add_argument(
        "--sync-timeout",
        type=float,
        default=30.0,
        help="abort when required ROS topics stay unavailable (default: 30 seconds)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not math.isfinite(args.sync_timeout) or args.sync_timeout <= 0:
        raise SystemExit("--sync-timeout must be a positive finite number")
    rclpy.init(args=argv)
    collector = Ros2Collector(args)
    try:
        try:
            timesteps, actions = collector.process()
        except RuntimeError as exc:
            print(f"Error: {exc}", file=sys.stderr, flush=True)
            return 1
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    if len(actions) < args.max_timesteps:
        print(f"\033[31m\nSave failure, please record {args.max_timesteps} timesteps of data.\033[0m\n")
        raise SystemExit(1)

    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    os.makedirs(dataset_dir, exist_ok=True)
    dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
    save_data(args, timesteps, actions, dataset_path)


if __name__ == "__main__":
    main()
