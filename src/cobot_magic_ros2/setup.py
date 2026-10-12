from setuptools import setup

package_name = "cobot_magic_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/collect_stack.launch.py",
                "launch/collect_stack_split_cameras.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pc",
    maintainer_email="pc@localhost.localdomain",
    description="ROS 2 data collection utilities for Cobot Magic.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "piper_ms_node = cobot_magic_ros2.piper_ms_node:main",
            "realsense_color_node = cobot_magic_ros2.realsense_color_node:main",
            "realsense_fps_probe = cobot_magic_ros2.realsense_color_node:probe_main",
            "collect_data = cobot_magic_ros2.collect_data:main",
        ],
    },
)
