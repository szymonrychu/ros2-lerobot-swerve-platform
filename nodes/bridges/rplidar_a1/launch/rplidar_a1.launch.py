#!/usr/bin/env python3
"""Headless launch for RPLidar A1 (node only, no RViz). Uses rplidar_composition from ros-jazzy-rplidar-ros."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Must match the static TF child published by static_tf_publisher (base_link -> laser_frame).
DEFAULT_FRAME_ID = "laser_frame"
FRAME_ID_ENV = "RPLIDAR_FRAME_ID"
SERIAL_PORT_ENV = "RPLIDAR_SERIAL_PORT"
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"


def generate_launch_description() -> LaunchDescription:
    """Build the headless RPLidar launch description.

    Returns:
        LaunchDescription: Launch arguments plus the rplidar_composition node.
    """
    serial_port = LaunchConfiguration("serial_port", default=os.environ.get(SERIAL_PORT_ENV, DEFAULT_SERIAL_PORT))
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="115200")
    frame_id = LaunchConfiguration("frame_id", default=os.environ.get(FRAME_ID_ENV, DEFAULT_FRAME_ID))
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value=serial_port, description="USB port for lidar"),
            DeclareLaunchArgument(
                "serial_baudrate", default_value=serial_baudrate, description="Baud rate (115200 for A1)"
            ),
            DeclareLaunchArgument("frame_id", default_value=frame_id, description="Frame ID for LaserScan (static TF child of base_link)"),
            DeclareLaunchArgument("inverted", default_value=inverted, description="Invert scan data"),
            DeclareLaunchArgument("angle_compensate", default_value=angle_compensate, description="Angle compensation"),
            Node(
                package="rplidar_ros",
                executable="rplidar_composition",
                name="rplidar_composition",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": inverted,
                        "angle_compensate": angle_compensate,
                    }
                ],
                output="screen",
            ),
        ]
    )
