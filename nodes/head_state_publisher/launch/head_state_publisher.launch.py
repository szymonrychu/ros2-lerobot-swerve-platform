#!/usr/bin/env python3
"""Launch robot_state_publisher on the pan/tilt head URDF.

The URDF is nodes/web_ui/urdf/head.urdf, resolved relative to this file so it works from the deployed checkout.
Joint states come from /head/joint_states (published by base_servos' head group).
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

HEAD_URDF_PATH = Path(__file__).resolve().parents[2] / "web_ui" / "urdf" / "head.urdf"
REMAPPINGS = [
    ("joint_states", "/head/joint_states"),
    ("robot_description", "/head/robot_description"),
]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        LaunchDescription: robot_state_publisher with the head URDF and the /head/* remappings.
    """
    robot_description = HEAD_URDF_PATH.read_text()
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="head_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=REMAPPINGS,
            ),
        ]
    )
