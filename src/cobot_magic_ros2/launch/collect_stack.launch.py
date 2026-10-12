from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    auto_enable = LaunchConfiguration("auto_enable")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="0"),
            DeclareLaunchArgument("auto_enable", default_value="false"),
            Node(
                package="cobot_magic_ros2",
                executable="realsense_color_node",
                name="realsense_color_node",
                output="screen",
            ),
            Node(
                package="cobot_magic_ros2",
                executable="piper_ms_node",
                name="piper_left",
                output="screen",
                arguments=[
                    "--can-port",
                    "can_left",
                    "--side",
                    "left",
                    "--mode",
                    mode,
                    "--auto-enable",
                    auto_enable,
                    "--publish-rate",
                    "200",
                ],
            ),
            Node(
                package="cobot_magic_ros2",
                executable="piper_ms_node",
                name="piper_right",
                output="screen",
                arguments=[
                    "--can-port",
                    "can_right",
                    "--side",
                    "right",
                    "--mode",
                    mode,
                    "--auto-enable",
                    auto_enable,
                    "--publish-rate",
                    "200",
                ],
            ),
        ]
    )
