#!/usr/bin/env python3
import argparse
import sys
import threading
import time

import numpy as np
from PIL import Image as PILImage
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


DEFAULT_SERIALS = {
    "camera_f": "346522070769",
    "camera_l": "419622072836",
    "camera_r": "420222071136",
}

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
MAX_CONSECUTIVE_FRAME_TIMEOUTS = 3


def rs_format_from_name(name):
    formats = {
        "rgb8": rs.format.rgb8,
        "bgr8": rs.format.bgr8,
    }
    if name not in formats:
        raise ValueError(f"unsupported color format: {name}")
    return formats[name]


def resize_with_pad(image, height, width):
    if image.shape[:2] == (height, width):
        return np.ascontiguousarray(image)

    current_height, current_width = image.shape[:2]
    ratio = max(current_width / width, current_height / height)
    resized_height = int(current_height / ratio)
    resized_width = int(current_width / ratio)
    resized = np.asarray(
        PILImage.fromarray(image).resize((resized_width, resized_height), resample=PILImage.BILINEAR)
    )
    output = np.zeros((height, width, 3), dtype=np.uint8)
    pad_height = max(0, (height - resized_height) // 2)
    pad_width = max(0, (width - resized_width) // 2)
    output[pad_height : pad_height + resized_height, pad_width : pad_width + resized_width] = resized
    return output


def image_msg_from_array(arr, stamp, frame_id, encoding):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.shape[1] * 3
    msg.data = arr.tobytes()
    return msg


class CameraWorker(threading.Thread):
    def __init__(
        self,
        node,
        name,
        serial,
        width,
        height,
        fps,
        publish_width,
        publish_height,
        publish_fps,
        timeout_ms,
        color_format,
        stats_period,
    ):
        super().__init__(daemon=True)
        self.node = node
        self.name = name
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.publish_width = publish_width
        self.publish_height = publish_height
        self.publish_fps = publish_fps
        self.timeout_ms = timeout_ms
        self.color_format = color_format
        self.stats_period = stats_period
        self.stop_event = threading.Event()
        self.pipeline = None
        self.publisher = node.create_publisher(Image, f"/{name}/color/image_raw", IMAGE_QOS)

    def run(self):
        restart_count = 0
        while rclpy.ok() and not self.stop_event.is_set():
            cfg = rs.config()
            cfg.enable_device(self.serial)
            cfg.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs_format_from_name(self.color_format),
                self.fps,
            )
            self.pipeline = rs.pipeline()
            self.node.get_logger().info(
                f"starting {self.name} serial={self.serial} capture={self.width}x{self.height}@{self.fps} "
                f"publish={self.publish_width}x{self.publish_height}@{self.publish_fps:g} "
                f"{self.color_format} topic=/{self.name}/color/image_raw qos=best_effort/depth1"
            )
            timeout_count = 0
            captured_since_log = 0
            published_since_log = 0
            log_start = time.monotonic()
            next_publish_at = log_start
            publish_period = 1.0 / self.publish_fps
            try:
                self.pipeline.start(cfg)
                while rclpy.ok() and not self.stop_event.is_set():
                    try:
                        frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
                    except RuntimeError as exc:
                        if "Frame didn't arrive" in str(exc):
                            timeout_count += 1
                            if timeout_count == 1:
                                self.node.get_logger().warning(
                                    f"{self.name} frame timeout x{timeout_count}; still waiting"
                                )
                            if timeout_count >= MAX_CONSECUTIVE_FRAME_TIMEOUTS:
                                self.node.get_logger().error(
                                    f"{self.name} frame timeout x{timeout_count}; "
                                    "restarting RealSense pipeline"
                                )
                                break
                            continue
                        raise
                    timeout_count = 0
                    color = frames.get_color_frame()
                    if color:
                        captured_since_log += 1
                        now = time.monotonic()
                        if now >= next_publish_at:
                            image = np.asanyarray(color.get_data())
                            image = resize_with_pad(image, self.publish_height, self.publish_width)
                            msg = image_msg_from_array(
                                image,
                                self.node.get_clock().now().to_msg(),
                                f"{self.name}_color_optical_frame",
                                self.color_format,
                            )
                            self.publisher.publish(msg)
                            published_since_log += 1
                            next_publish_at += publish_period
                            if next_publish_at < now - publish_period:
                                next_publish_at = now + publish_period

                        elapsed = now - log_start
                        if self.stats_period > 0 and elapsed >= self.stats_period:
                            self.node.get_logger().info(
                                f"{self.name} capture_fps={captured_since_log / elapsed:.2f} "
                                f"publish_fps={published_since_log / elapsed:.2f}"
                            )
                            captured_since_log = 0
                            published_since_log = 0
                            log_start = now
            except Exception as exc:
                self.node.get_logger().error(f"{self.name} failed: {exc!r}")
            finally:
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
                self.pipeline = None
            if rclpy.ok() and not self.stop_event.is_set():
                restart_count += 1
                self.node.get_logger().warning(f"{self.name} restarting RealSense pipeline x{restart_count}")
                time.sleep(1.0)

    def stop(self):
        self.stop_event.set()
        pipeline = self.pipeline
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


class RealSenseColorNode(Node):
    def __init__(self, args):
        super().__init__("realsense_color_node")
        self.workers = []
        for name, serial in [
            ("camera_f", args.camera_f),
            ("camera_l", args.camera_l),
            ("camera_r", args.camera_r),
        ]:
            if serial:
                self.workers.append(
                    CameraWorker(
                        self,
                        name,
                        serial,
                        args.width,
                        args.height,
                        args.fps,
                        args.publish_width,
                        args.publish_height,
                        args.publish_fps,
                        args.timeout_ms,
                        args.format,
                        args.stats_period,
                    )
                )

    def start(self):
        for worker in self.workers:
            worker.start()
            time.sleep(0.5)

    def stop(self):
        for worker in self.workers:
            worker.stop()
        for worker in self.workers:
            worker.join(timeout=2.0)


def parse_args(argv=None):
    argv = remove_ros_args(args=argv if argv is not None else sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-f", default=DEFAULT_SERIALS["camera_f"])
    parser.add_argument("--camera-l", default=DEFAULT_SERIALS["camera_l"])
    parser.add_argument("--camera-r", default=DEFAULT_SERIALS["camera_r"])
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--publish-width", type=int, default=0, help="0 uses --width")
    parser.add_argument("--publish-height", type=int, default=0, help="0 uses --height")
    parser.add_argument("--publish-fps", type=float, default=0.0, help="0 uses --fps")
    parser.add_argument("--format", choices=["rgb8", "bgr8"], default="rgb8")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--stats-period", type=float, default=5.0)
    args = parser.parse_args(argv)
    args.publish_width = args.publish_width or args.width
    args.publish_height = args.publish_height or args.height
    args.publish_fps = args.publish_fps or float(args.fps)
    numeric_values = (
        args.width,
        args.height,
        args.fps,
        args.publish_width,
        args.publish_height,
        args.publish_fps,
    )
    if any(value <= 0 for value in numeric_values):
        parser.error("capture and publish dimensions/frequencies must be positive")
    if args.publish_fps > args.fps:
        parser.error("--publish-fps cannot exceed the RealSense --fps")
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=argv)
    node = RealSenseColorNode(args)
    node.start()
    try:
        while rclpy.ok() and any(worker.is_alive() for worker in node.workers):
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.01)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


def probe_parse_args(argv=None):
    argv = argv if argv is not None else sys.argv
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-f", default=DEFAULT_SERIALS["camera_f"])
    parser.add_argument("--camera-l", default=DEFAULT_SERIALS["camera_l"])
    parser.add_argument("--camera-r", default=DEFAULT_SERIALS["camera_r"])
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--format", choices=["rgb8", "bgr8"], default="rgb8")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--seconds", type=float, default=15.0)
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def probe_main(argv=None):
    args = probe_parse_args(argv)
    pipelines = []
    counts = {}
    serials = [
        ("camera_f", args.camera_f),
        ("camera_l", args.camera_l),
        ("camera_r", args.camera_r),
    ]
    try:
        for name, serial in serials:
            if not serial:
                continue
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(
                rs.stream.color,
                args.width,
                args.height,
                rs_format_from_name(args.format),
                args.fps,
            )
            pipeline = rs.pipeline()
            pipeline.start(cfg)
            pipelines.append((name, serial, pipeline))
            counts[name] = 0
            print(f"started {name} serial={serial} {args.width}x{args.height}@{args.fps} {args.format}")

        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            for name, _, pipeline in pipelines:
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=args.timeout_ms)
                except RuntimeError as exc:
                    print(f"{name} timeout/error: {exc}")
                    continue
                if frames.get_color_frame():
                    counts[name] += 1

        elapsed = time.monotonic() - t0
        for name, serial, _ in pipelines:
            print(f"{name} serial={serial} direct fps={counts[name] / elapsed:.2f}")
    finally:
        for _, _, pipeline in pipelines:
            try:
                pipeline.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
