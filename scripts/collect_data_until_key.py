#!/usr/bin/env python3
"""Collect one episode and allow the operator to stop and save with the S key."""

import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy

from cobot_magic_ros2.collect_data import Ros2Collector, save_data


STOP_KEY = "s"
SYNC_STATUS_INTERVAL_SECONDS = 2.0


class StopKeyReader:
    def __init__(self):
        self.fd = None
        self.original_settings = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("interactive collection requires a terminal on standard input")
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.original_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def stop_requested(self):
        requested = False
        while select.select([self.fd], [], [], 0.0)[0]:
            key = os.read(self.fd, 1).decode(errors="ignore")
            if key.lower() == STOP_KEY:
                requested = True
        return requested


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Collect one Cobot Magic episode; press S to stop early and save."
    )
    parser.add_argument("--dataset_dir", "--dataset-dir", dest="dataset_dir", default="./data")
    parser.add_argument("--task_name", "--task-name", dest="task_name", default="DLO_Stretch")
    parser.add_argument("--episode_idx", "--episode-idx", dest="episode_idx", type=int, default=0)
    parser.add_argument(
        "--max_timesteps", "--max-timesteps", dest="max_timesteps", type=int, default=600
    )
    parser.add_argument(
        "--camera_names",
        "--camera-names",
        dest="camera_names",
        nargs="+",
        default=["cam_high", "cam_left_wrist", "cam_right_wrist"],
    )
    parser.add_argument(
        "--img_front_topic",
        "--img-front-topic",
        dest="img_front_topic",
        default="/camera_f/color/image_raw",
    )
    parser.add_argument(
        "--img_left_topic",
        "--img-left-topic",
        dest="img_left_topic",
        default="/camera_l/color/image_raw",
    )
    parser.add_argument(
        "--img_right_topic",
        "--img-right-topic",
        dest="img_right_topic",
        default="/camera_r/color/image_raw",
    )
    parser.add_argument(
        "--master_arm_left_topic",
        "--master-arm-left-topic",
        dest="master_arm_left_topic",
        default="/master/joint_left",
    )
    parser.add_argument(
        "--master_arm_right_topic",
        "--master-arm-right-topic",
        dest="master_arm_right_topic",
        default="/master/joint_right",
    )
    parser.add_argument(
        "--puppet_arm_left_topic",
        "--puppet-arm-left-topic",
        dest="puppet_arm_left_topic",
        default="/puppet/joint_left",
    )
    parser.add_argument(
        "--puppet_arm_right_topic",
        "--puppet-arm-right-topic",
        dest="puppet_arm_right_topic",
        default="/puppet/joint_right",
    )
    parser.add_argument(
        "--robot_base_topic", "--robot-base-topic", dest="robot_base_topic", default="/odom"
    )
    parser.add_argument("--use_robot_base", "--use-robot-base", dest="use_robot_base", action="store_true")
    parser.add_argument("--frame_rate", "--frame-rate", dest="frame_rate", type=int, default=30)
    parser.add_argument("--image_width", "--image-width", dest="image_width", type=int, default=640)
    parser.add_argument(
        "--image_height", "--image-height", dest="image_height", type=int, default=480
    )
    parser.add_argument(
        "--sync-timeout",
        dest="sync_timeout",
        type=float,
        default=30.0,
        help="abort when required ROS topics stay unavailable (default: 30 seconds)",
    )
    return parser.parse_args(argv)


def validate_args(args):
    if args.episode_idx < 0:
        raise ValueError("episode index must be non-negative")
    if args.max_timesteps <= 0:
        raise ValueError("max timesteps must be positive")
    if args.frame_rate <= 0:
        raise ValueError("frame rate must be positive")
    if args.image_width <= 0 or args.image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if len(args.camera_names) != 3:
        raise ValueError("exactly three camera names are required")
    if not (args.sync_timeout > 0):
        raise ValueError("sync timeout must be positive")


def collect_until_stop(collector, args):
    timesteps = []
    actions = []
    warmed_up = False
    stop_requested = False
    sleep_sec = 1.0 / args.frame_rate
    next_sample_at = time.monotonic() + sleep_sec
    sync_issue_started = None
    last_sync_report = 0.0

    print(
        f"Collecting up to {args.max_timesteps} timesteps at {args.frame_rate} Hz. "
        "Press S to stop and save.",
        flush=True,
    )

    with StopKeyReader() as key_reader:
        while rclpy.ok() and len(actions) < args.max_timesteps:
            if key_reader.stop_requested():
                stop_requested = True
                if actions:
                    break
                print("Stop requested; waiting for the first valid sample before saving.", flush=True)

            # Receive ROS data within the frame period instead of adding a
            # separate spin interval on top of a full-period sleep.
            while rclpy.ok() and time.monotonic() < next_sample_at:
                remaining = max(0.0, next_sample_at - time.monotonic())
                rclpy.spin_once(collector, timeout_sec=min(0.001, remaining))
                if key_reader.stop_requested():
                    stop_requested = True

            next_sample_at += sleep_sec
            if next_sample_at < time.monotonic():
                next_sample_at = time.monotonic() + sleep_sec

            result = collector.get_frame()
            if result is None:
                now = time.monotonic()
                if sync_issue_started is None:
                    sync_issue_started = now
                    print("sync fail: waiting for camera and arm data...", flush=True)
                if now - last_sync_report >= SYNC_STATUS_INTERVAL_SECONDS:
                    missing = collector.unhealthy_streams()
                    detail = "、".join(missing) if missing else "消息尚未组成完整帧"
                    print(f"Still waiting for synchronized data: {detail}", flush=True)
                    last_sync_report = now
                if now - sync_issue_started >= args.sync_timeout:
                    missing = collector.unhealthy_streams()
                    detail = "、".join(missing) if missing else "消息尚未组成完整帧"
                    raise RuntimeError(
                        f"同步等待超过 {args.sync_timeout:g} 秒：{detail}。"
                        "请检查 collection stack、相机连接、CAN 和话题名称。"
                    )
                continue

            sync_issue_started = None
            obs, action = result
            if not warmed_up:
                warmed_up = True
                print("Collector warmed up; valid samples are now being recorded.", flush=True)
            else:
                timesteps.append(obs)
                actions.append(action)
                print(f"Frame data: {len(actions)}", flush=True)

            if stop_requested and actions:
                break

    reason = "stop_key" if stop_requested else "max_timesteps"
    print(f"Recorded {len(actions)} timesteps; stop reason: {reason}.", flush=True)
    return timesteps, actions


def main(argv=None):
    args = parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    dataset_dir = os.path.join(os.path.abspath(os.path.expanduser(args.dataset_dir)), args.task_name)
    dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
    hdf5_path = dataset_path + ".hdf5"
    if os.path.exists(hdf5_path):
        print(f"Error: refusing to overwrite existing HDF5: {hdf5_path}", file=sys.stderr)
        return 2

    rclpy.init(args=[])
    collector = None
    try:
        collector = Ros2Collector(args)
        timesteps, actions = collect_until_stop(collector, args)
    except KeyboardInterrupt:
        print("\nInterrupted with Ctrl+C; the current episode was not saved.", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    finally:
        if collector is not None:
            collector.destroy_node()
        rclpy.shutdown()

    if not actions:
        print("Error: no valid timesteps were collected; nothing was saved.", file=sys.stderr)
        return 1

    os.makedirs(dataset_dir, exist_ok=True)
    save_data(args, timesteps, actions, dataset_path)
    print(f"Saved {len(actions)} timesteps to {hdf5_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
