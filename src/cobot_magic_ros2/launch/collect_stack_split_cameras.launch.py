from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_node(
    name,
    serial_arg,
    camera_width,
    camera_height,
    camera_fps,
    publish_width,
    publish_height,
    publish_fps,
):
    return Node(
        package="cobot_magic_ros2",
        executable="realsense_color_node",
        name=f"{name}_node",
        output="screen",
        arguments=[
            "--camera-f",
            serial_arg if name == "camera_f" else "",
            "--camera-l",
            serial_arg if name == "camera_l" else "",
            "--camera-r",
            serial_arg if name == "camera_r" else "",
            "--width",
            camera_width,
            "--height",
            camera_height,
            "--fps",
            camera_fps,
            "--publish-width",
            publish_width,
            "--publish-height",
            publish_height,
            "--publish-fps",
            publish_fps,
        ],
    )


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    auto_enable = LaunchConfiguration("auto_enable")
    motion_speed = LaunchConfiguration("motion_speed")
    command_timeout = LaunchConfiguration("command_timeout")
    camera_f_serial = LaunchConfiguration("camera_f_serial")
    camera_l_serial = LaunchConfiguration("camera_l_serial")
    camera_r_serial = LaunchConfiguration("camera_r_serial")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    publish_width = LaunchConfiguration("publish_width")
    publish_height = LaunchConfiguration("publish_height")
    publish_fps = LaunchConfiguration("publish_fps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="0"),
            DeclareLaunchArgument("auto_enable", default_value="false"),
            DeclareLaunchArgument("motion_speed", default_value="20"),
            DeclareLaunchArgument("command_timeout", default_value="1.0"),
            DeclareLaunchArgument("camera_f_serial", default_value="346522070769"),
            DeclareLaunchArgument("camera_l_serial", default_value="419622072836"),
            DeclareLaunchArgument("camera_r_serial", default_value="420222071136"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument("publish_width", default_value="640"),
            DeclareLaunchArgument("publish_height", default_value="480"),
            DeclareLaunchArgument("publish_fps", default_value="30"),
            camera_node(
                "camera_f",
                camera_f_serial,
                camera_width,
                camera_height,
                camera_fps,
                publish_width,
                publish_height,
                publish_fps,
            ),
            camera_node(
                "camera_l",
                camera_l_serial,
                camera_width,
                camera_height,
                camera_fps,
                publish_width,
                publish_height,
                publish_fps,
            ),
            camera_node(
                "camera_r",
                camera_r_serial,
                camera_width,
                camera_height,
                camera_fps,
                publish_width,
                publish_height,
                publish_fps,
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
                    "--motion-speed",
                    motion_speed,
                    "--command-timeout",
                    command_timeout,
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
                    "--motion-speed",
                    motion_speed,
                    "--command-timeout",
                    command_timeout,
                ],
            ),
        ]
    )
