#!/usr/bin/env python3
"""Launch robot_state_publisher on the pan/tilt head URDF.

The URDF is nodes/web_ui/urdf/head.urdf, resolved relative to this file so it works from the deployed checkout.
If /var/lib/ros2/head_rgbd_calibration/applied/head_chain.json exists, its origins replace the URDF head_mount_joint,
head_tilt and head_camera_joint origins; tof_joint.json there replaces the tof_joint origin (head_description.py), so an
applied calibration takes effect after a restart without editing the repo URDF.
Joint states come from /head/joint_states (published by base_servos' head group).
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

HEAD_URDF_PATH = Path(__file__).resolve().parents[2] / "web_ui" / "urdf" / "head.urdf"
APPLIED_DIR = Path("/var/lib/ros2/head_rgbd_calibration/applied")
URDF_TEXT_PATH = (
    Path(__file__).resolve().parents[3] / "shared" / "ros2_common" / "head_rgbd_calibration" / "urdf_text.py"
)
HEAD_DESCRIPTION_PATH = Path(__file__).resolve().parent / "head_description.py"
REMAPPINGS = [
    ("joint_states", "/head/joint_states"),
    ("robot_description", "/head/robot_description"),
]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        LaunchDescription: robot_state_publisher with the head URDF and the /head/* remappings.
    """
    spec = importlib.util.spec_from_file_location("head_description", HEAD_DESCRIPTION_PATH)
    head_description = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(head_description)
    robot_description = head_description.robot_description(HEAD_URDF_PATH, APPLIED_DIR, URDF_TEXT_PATH)
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
