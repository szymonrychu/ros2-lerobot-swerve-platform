#!/usr/bin/env python3
"""Launch the IMX219 stereo pair: two camera_ros `camera::CameraNode`s in one component container.

The left camera (cam0) keeps the overview role in namespace /overview_camera and publishes
/overview_camera/image_raw, /overview_camera/image_raw/compressed and /overview_camera/camera_info. The right camera
(cam1) publishes the same topics under /stereo/right. libcamera SyncMode pairs the frames: left is the sync server,
right the client. Each camera is selected by its libcamera ID; without one, index 0 (left) or 1 (right) is used and a
warning is logged. camera_info_url is set only when the stereo calibration file of that side has a valid projection
matrix; otherwise camera_ros publishes an uncalibrated camera_info and the reason is logged.

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

from overview_camera_config import (  # noqa: E402
    SIDE_INDEX,
    camera_info_url,
    camera_parameters,
    camera_selector,
    load_settings,
    side_namespace,
)

PARAMS_ENV = "OVERVIEW_CAMERA_PARAMS"
CONFIG_ENV = "OVERVIEW_CAMERA_CONFIG"
NODE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS_PATH = NODE_DIR / "config" / "params.yaml"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/overview_camera/config.yaml")
CONTAINER_NAMESPACE = "/overview_camera"
TOPICS = ("image_raw", "image_raw/compressed", "camera_info")
INTRA_PROCESS = [{"use_intra_process_comms": True}]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description from the merged settings.

    Returns:
        LaunchDescription: Container with the left and right camera nodes, preceded by the warnings (camera picked
        by index, calibration file unusable).
    """
    settings = load_settings(
        Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)),
        Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)),
    )
    calibration_dir = Path(settings["launch"]["calibration_dir"])
    nodes = []
    warnings = []
    for side in ("left", "right"):
        camera, warning = camera_selector(settings["launch"][f"{side}_camera_id"], SIDE_INDEX[side])
        info_url, reason = camera_info_url(calibration_dir, side)
        warnings += [f"{side}: {warning}"] if warning else []
        warnings += [f"{side}: uncalibrated camera_info ({reason})"] if reason else []
        nodes.append(
            ComposableNode(
                package="camera_ros",
                plugin="camera::CameraNode",
                name="camera",
                namespace=side_namespace(settings, side),
                parameters=[camera_parameters(settings, side, camera, info_url)],
                remappings=[(f"~/{name}", name) for name in TOPICS],
                extra_arguments=INTRA_PROCESS,
            )
        )
    container = ComposableNodeContainer(
        name="overview_container",
        namespace=CONTAINER_NAMESPACE,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=nodes,
        output="screen",
    )
    return LaunchDescription([*(LogInfo(msg=f"WARNING: overview_camera: {text}") for text in warnings), container])
