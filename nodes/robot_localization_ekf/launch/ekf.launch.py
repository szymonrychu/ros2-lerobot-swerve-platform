#!/usr/bin/env python3
"""Launch EKF node with config from env or default path."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

EKF_CONFIG_PATH = os.environ.get(
    "ROBOT_LOCALIZATION_EKF_CONFIG",
    "/etc/ros2/robot_localization_ekf/ekf.yaml",
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[EKF_CONFIG_PATH],
            ),
        ]
    )
