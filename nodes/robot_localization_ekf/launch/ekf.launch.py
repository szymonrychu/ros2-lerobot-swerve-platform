#!/usr/bin/env python3
"""Launch robot_localization's EKF node with the deployed config.

The config path comes from env ROBOT_LOCALIZATION_EKF_CONFIG (set by Ansible to the deployed
/etc/ros2/robot_localization_ekf/config.yaml); the repo copy is config/ekf.yaml.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

EKF_CONFIG_ENV = "ROBOT_LOCALIZATION_EKF_CONFIG"
DEFAULT_EKF_CONFIG_PATH = "/etc/ros2/robot_localization_ekf/config.yaml"


def generate_launch_description() -> LaunchDescription:
    """Build the EKF launch description.

    Returns:
        LaunchDescription: The ekf_filter_node with the config from ROBOT_LOCALIZATION_EKF_CONFIG.
    """
    config_path = os.environ.get(EKF_CONFIG_ENV, DEFAULT_EKF_CONFIG_PATH)
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[config_path],
            ),
        ]
    )
