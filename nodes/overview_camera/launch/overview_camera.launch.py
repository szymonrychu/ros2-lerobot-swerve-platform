#!/usr/bin/env python3
"""Launch the overhead camera: one camera_ros `camera::CameraNode` (Raspberry Pi Camera Module 3, IMX708).

The node runs in namespace /overview_camera inside one component container and publishes /overview_camera/image_raw,
/overview_camera/image_raw/compressed (JPEG, quality from the settings) and /overview_camera/camera_info. No camera
calibration is configured: camera_ros publishes an uncalibrated camera_info (no placeholder intrinsics).
The camera is selected by its libcamera ID; without one, camera index 0 is used and a warning is logged.

Settings: config/params.yaml (env OVERVIEW_CAMERA_PARAMS replaces the path), overridden by the deployed
/etc/ros2/overview_camera/config.yaml (env OVERVIEW_CAMERA_CONFIG) when it exists and is non-empty.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, str(Path(__file__).resolve().parent))

from overview_camera_config import camera_parameters, camera_selector, load_settings  # noqa: E402

PARAMS_ENV = "OVERVIEW_CAMERA_PARAMS"
CONFIG_ENV = "OVERVIEW_CAMERA_CONFIG"
NODE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS_PATH = NODE_DIR / "config" / "params.yaml"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/overview_camera/config.yaml")
NAMESPACE = "/overview_camera"
TOPICS = ("image_raw", "image_raw/compressed", "camera_info")
INTRA_PROCESS = [{"use_intra_process_comms": True}]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description from the merged settings.

    Returns:
        LaunchDescription: Container with the camera node, preceded by a warning when no camera ID is set.
    """
    settings = load_settings(
        Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)),
        Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)),
    )
    camera, warning = camera_selector(settings["launch"]["camera_id"])
    node = ComposableNode(
        package="camera_ros",
        plugin="camera::CameraNode",
        name="camera",
        namespace=NAMESPACE,
        parameters=[camera_parameters(settings, camera)],
        remappings=[(f"~/{name}", name) for name in TOPICS],
        extra_arguments=INTRA_PROCESS,
    )
    container = ComposableNodeContainer(
        name="overview_container",
        namespace=NAMESPACE,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[node],
        output="screen",
    )
    warnings = [LogInfo(msg=f"WARNING: overview_camera: {warning}")] if warning else []
    return LaunchDescription([*warnings, container])
