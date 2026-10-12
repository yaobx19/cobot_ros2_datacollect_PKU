#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    from piper_sdk import C_PiperInterface
except ImportError:
    C_PiperInterface = None


JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ANGLE_TO_RAD = 0.017444
RAD_TO_PIPER = 57324.840764


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


def validate_joint_command(requested):
    """Validate a command without changing any requested value."""
    if len(requested) < 7:
        raise ValueError("PiPER commands must contain seven positions")
    requested = [float(value) for value in requested[:7]]
    if not all(math.isfinite(value) for value in requested):
        raise ValueError("PiPER command contains NaN/Inf")
    return requested


def empty_joint_state(stamp):
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(JOINT_NAMES)
    msg.position = [0.0] * 7
    msg.velocity = [0.0] * 7
    msg.effort = [0.0] * 7
    return msg


class PiperMsNode(Node):
    def __init__(self, args):
        super().__init__(f"piper_{args.side}")
        if C_PiperInterface is None:
            raise RuntimeError("piper_sdk is not installed. Install it with: pip install piper_sdk")

        self.can_port = args.can_port
        self.side = args.side
        self.mode = args.mode
        self.auto_enable = args.auto_enable and self.mode == 1
        self.enable_flag = False
        self.motion_speed = max(1, min(100, int(args.motion_speed)))
        self.command_timeout = float(args.command_timeout)
        self.last_state_positions = None
        self.last_command_time = None
        self.watchdog_tripped = False
        self.last_warning_time = 0.0
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        self.puppet_pub = self.create_publisher(JointState, f"/puppet/joint_{self.side}", 1)
        self.master_pub = self.create_publisher(JointState, f"/master/joint_{self.side}", 1)
        self.enable_sub = self.create_subscription(Bool, "/enable_flag", self.enable_callback, 1)
        if self.mode == 1:
            self.master_sub = self.create_subscription(
                JointState, f"/master/joint_{self.side}", self.joint_callback, 1
            )
        else:
            self.master_sub = None

        self.create_service(Trigger, f"/{self.can_port}/go_zero_master", self.go_zero_master)
        self.create_service(Trigger, f"/{self.can_port}/go_zero_master_slave", self.go_zero_master_slave)
        self.create_service(Trigger, f"/{self.can_port}/restore_ms_mode", self.restore_ms_mode)
        self.timer = self.create_timer(1.0 / args.publish_rate, self.publish_once)
        self.get_logger().info(
            f"connected can_port={self.can_port} side={self.side} mode={self.mode} "
            f"auto_enable={self.auto_enable} speed={self.motion_speed} "
            f"watchdog={self.command_timeout}s"
        )

    def publish_once(self):
        if (
            self.auto_enable
            and not self.watchdog_tripped
            and not self.enable_flag
        ):
            self.try_auto_enable()

        if not self.piper.isOk():
            self.warn_throttled(f"{self.can_port} is lost; no joint state published")
            return

        state_msg = self.read_slave_joint()
        self.last_state_positions = list(state_msg.position)
        self.puppet_pub.publish(state_msg)
        if self.mode == 0:
            self.master_pub.publish(self.read_master_joint())
        self.check_command_watchdog()

    def try_auto_enable(self):
        status = self.piper.GetArmLowSpdInfoMsgs()
        enabled = (
            status.motor_1.foc_status.driver_enable_status
            and status.motor_2.foc_status.driver_enable_status
            and status.motor_3.foc_status.driver_enable_status
            and status.motor_4.foc_status.driver_enable_status
            and status.motor_5.foc_status.driver_enable_status
            and status.motor_6.foc_status.driver_enable_status
        )
        if enabled:
            self.enable_flag = True
            self.auto_enable = False
            self.get_logger().info(f"{self.can_port} enabled")
            return
        self.piper.EnableArm(7)
        self.piper.GripperCtrl(0, 1000, 0x02, 0)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

    def warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 1.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def disable_motion(self):
        self.enable_flag = False
        self.piper.DisableArm(7)
        self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def check_command_watchdog(self):
        if (
            self.mode != 1
            or not self.enable_flag
            or self.command_timeout <= 0.0
            or self.last_command_time is None
        ):
            return
        elapsed = time.monotonic() - self.last_command_time
        if elapsed <= self.command_timeout:
            return
        self.disable_motion()
        self.watchdog_tripped = True
        self.get_logger().error(
            f"{self.can_port} command timeout after {elapsed:.3f}s; arm disabled"
        )

    def read_slave_joint(self):
        msg = empty_joint_state(self.get_clock().now().to_msg())
        joint = self.piper.GetArmJointMsgs().joint_state
        gripper = self.piper.GetArmGripperMsgs().gripper_state
        high = self.piper.GetArmHighSpdInfoMsgs()
        msg.position = [
            (joint.joint_1 / 1000) * ANGLE_TO_RAD,
            (joint.joint_2 / 1000) * ANGLE_TO_RAD,
            (joint.joint_3 / 1000) * ANGLE_TO_RAD,
            (joint.joint_4 / 1000) * ANGLE_TO_RAD,
            (joint.joint_5 / 1000) * ANGLE_TO_RAD,
            (joint.joint_6 / 1000) * ANGLE_TO_RAD,
            gripper.grippers_angle / 1000000,
        ]
        msg.velocity = [
            high.motor_1.motor_speed / 1000,
            high.motor_2.motor_speed / 1000,
            high.motor_3.motor_speed / 1000,
            high.motor_4.motor_speed / 1000,
            high.motor_5.motor_speed / 1000,
            high.motor_6.motor_speed / 1000,
            0.0,
        ]
        msg.effort = [
            high.motor_1.effort / 1000,
            high.motor_2.effort / 1000,
            high.motor_3.effort / 1000,
            high.motor_4.effort / 1000,
            high.motor_5.effort / 1000,
            high.motor_6.effort / 1000,
            gripper.grippers_effort / 1000,
        ]
        return msg

    def read_master_joint(self):
        msg = empty_joint_state(self.get_clock().now().to_msg())
        joint = self.piper.GetArmJointCtrl().joint_ctrl
        gripper = self.piper.GetArmGripperCtrl().gripper_ctrl
        msg.position = [
            (joint.joint_1 / 1000) * ANGLE_TO_RAD,
            (joint.joint_2 / 1000) * ANGLE_TO_RAD,
            (joint.joint_3 / 1000) * ANGLE_TO_RAD,
            (joint.joint_4 / 1000) * ANGLE_TO_RAD,
            (joint.joint_5 / 1000) * ANGLE_TO_RAD,
            (joint.joint_6 / 1000) * ANGLE_TO_RAD,
            gripper.grippers_angle / 1000000,
        ]
        return msg

    def joint_callback(self, joint_data):
        if not self.enable_flag:
            return
        if len(joint_data.position) < 7:
            self.warn_throttled("Rejected command with fewer than seven positions")
            return
        if self.last_state_positions is None:
            self.warn_throttled("Rejected command before first measured arm state")
            return

        try:
            command = validate_joint_command(joint_data.position)
        except ValueError as exc:
            self.warn_throttled(f"Rejected unsafe command: {exc}")
            return

        joints = [round(value * RAD_TO_PIPER) for value in command[:6]]
        gripper = round(command[6] * 1000000)
        self.piper.MotionCtrl_2(0x01, 0x01, self.motion_speed)
        self.piper.JointCtrl(*joints)
        self.piper.GripperCtrl(gripper, 1000, 0x01, 0)
        self.last_command_time = time.monotonic()

    def enable_callback(self, msg):
        requested_enable = bool(msg.data)
        self.auto_enable = False
        self.last_command_time = None
        self.watchdog_tripped = False
        if requested_enable:
            self.enable_flag = True
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0, 1000, 0x02, 0)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.disable_motion()

    def shutdown(self):
        if self.mode == 1:
            self.disable_motion()
            self.get_logger().info(f"{self.can_port} disabled on shutdown")

    def go_zero_master(self, request, response):
        self.piper.ReqMasterArmMoveToHome(1)
        response.success = True
        response.message = f"{self.can_port} go_zero_master sent"
        return response

    def go_zero_master_slave(self, request, response):
        self.piper.ReqMasterArmMoveToHome(2)
        response.success = True
        response.message = f"{self.can_port} go_zero_master_slave sent"
        return response

    def restore_ms_mode(self, request, response):
        self.piper.ReqMasterArmMoveToHome(0)
        response.success = True
        response.message = f"{self.can_port} restore_ms_mode sent"
        return response


def parse_args(argv=None):
    argv = remove_ros_args(args=argv if argv is not None else sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--can-port", required=True)
    parser.add_argument("--side", choices=["left", "right"], required=True)
    parser.add_argument("--mode", type=int, default=0, choices=[0, 1])
    parser.add_argument("--auto-enable", default="false", type=str_to_bool)
    parser.add_argument("--publish-rate", type=float, default=200.0)
    parser.add_argument("--motion-speed", type=int, default=20)
    parser.add_argument("--command-timeout", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=argv)
    node = PiperMsNode(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.001)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
