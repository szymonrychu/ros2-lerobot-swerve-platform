#!/usr/bin/env python3
"""Launch the Camera Module 3 (IMX708) previews: one camera_ros `camera::CameraNode` per role in one component container.

The camera on the front_port CSI connector (default cam0) runs in namespace /front_camera, the other one in
/back_camera; `launch.roles` selects which of the two are launched. Each publishes image_raw, image_raw/compressed and camera_info (uncalibrated: nothing consumes more than
the preview yet).

Settings: config/params.yaml (env PI_CAMERAS_PARAMS replaces the path), overridden by the deployed
/etc/ros2/pi_cameras/config.yaml (env PI_CAMERAS_CONFIG) when it exists and is non-empty.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pi_cameras_config import (  # noqa: E402
    camera_parameters,
    enabled_roles,
    load_settings,
    role_camera_ids,
    role_namespace,
)

PARAMS_ENV = "PI_CAMERAS_PARAMS"
CONFIG_ENV = "PI_CAMERAS_CONFIG"
NODE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS_PATH = NODE_DIR / "config" / "params.yaml"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/pi_cameras/config.yaml")
CONTAINER_NAMESPACE = "/pi_cameras"
TOPICS = ("image_raw", "image_raw/compressed", "camera_info")
INTRA_PROCESS = [{"use_intra_process_comms": True}]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description from the merged settings.

    Returns:
        LaunchDescription: Container with one camera node per enabled role.
    """
    settings = load_settings(
        Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)),
        Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)),
    )
    camera_ids = role_camera_ids(settings)
    nodes = [
        ComposableNode(
            package="camera_ros",
            plugin="camera::CameraNode",
            name="camera",
            namespace=role_namespace(role),
            parameters=[camera_parameters(settings, role, camera_ids[role])],
            remappings=[(f"~/{name}", name) for name in TOPICS],
            extra_arguments=INTRA_PROCESS,
        )
        for role in enabled_roles(settings)
    ]
    container = ComposableNodeContainer(
        name="pi_cameras_container",
        namespace=CONTAINER_NAMESPACE,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=nodes,
        output="screen",
    )
    return LaunchDescription([container])
