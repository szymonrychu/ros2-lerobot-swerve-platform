#!/usr/bin/env python3
"""Headless launch for RPLidar A1 (node only, no RViz). Uses rplidar_composition from ros-jazzy-rplidar-ros."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port", default=os.environ.get("RPLIDAR_SERIAL_PORT", "/dev/ttyUSB0"))
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="115200")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value=serial_port, description="USB port for lidar"),
            DeclareLaunchArgument(
                "serial_baudrate", default_value=serial_baudrate, description="Baud rate (115200 for A1)"
            ),
            DeclareLaunchArgument("frame_id", default_value=frame_id, description="Frame ID for LaserScan"),
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
