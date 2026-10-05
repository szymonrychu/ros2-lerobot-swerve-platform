#!/usr/bin/env python3
"""Launch the stereo camera pipeline in one multi-threaded component container with intra-process transport.

Two camera_ros `camera::CameraNode` (namespaces /stereo/left and /stereo/right, selected by libcamera ID) publish
/stereo/{left,right}/image_raw and camera_info. Only when BOTH calibration files (calibration/left.yaml and
right.yaml) exist and hold a non-zero projection matrix P, image_proc rectifies both images
(/stereo/{left,right}/image_rect) and stereo_image_proc computes /stereo/disparity. Without a calibration only raw
images are published and a warning is logged: no placeholder rectification is ever produced. The point cloud
(/stereo/points2) additionally needs publish_points: true (and the stereo mount TF).

Settings: config/params.yaml (env STEREO_CAMERA_PARAMS replaces the path), overridden by the deployed
/etc/ros2/stereo_camera/config.yaml (env STEREO_CAMERA_CONFIG) when it exists and is non-empty.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, str(Path(__file__).resolve().parent))

from stereo_camera_config import (  # noqa: E402
    SIDE_INDEX,
    calibration_ready,
    camera_info_url,
    camera_parameters,
    camera_selector,
    disparity_parameters,
    load_settings,
    stage_plan,
)

PARAMS_ENV = "STEREO_CAMERA_PARAMS"
CONFIG_ENV = "STEREO_CAMERA_CONFIG"
NODE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS_PATH = NODE_DIR / "config" / "params.yaml"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/stereo_camera/config.yaml")
CALIBRATION_DIR = NODE_DIR / "calibration"
STEREO_NS = "/stereo"
INTRA_PROCESS = [{"use_intra_process_comms": True}]


def camera_node(side: str, settings: dict) -> tuple[ComposableNode, list[str]]:
    """camera_ros node of one side.

    Args:
        side: "left" or "right".
        settings: Merged settings.

    Returns:
        tuple[ComposableNode, list[str]]: The node and the warnings to log.
    """
    camera_id, warning = camera_selector(settings["launch"][f"{side}_camera_id"], SIDE_INDEX[side])
    params = camera_parameters(settings, side, camera_id, camera_info_url(CALIBRATION_DIR, side))
    topics = [(f"~/{name}", name) for name in ("image_raw", "image_raw/compressed", "camera_info")]
    node = ComposableNode(
        package="camera_ros",
        plugin="camera::CameraNode",
        name="camera",
        namespace=f"{STEREO_NS}/{side}",
        parameters=[params],
        remappings=topics,
        extra_arguments=INTRA_PROCESS,
    )
    return node, [f"stereo_camera ({side}): {warning}"] if warning else []


def rectify_node(side: str) -> ComposableNode:
    """image_proc RectifyNode of one side: /stereo/<side>/image_raw -> image_rect.

    Args:
        side: "left" or "right".

    Returns:
        ComposableNode: The node.
    """
    return ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify",
        namespace=f"{STEREO_NS}/{side}",
        remappings=[("image", "image_raw")],
        extra_arguments=INTRA_PROCESS,
    )


def stereo_nodes(settings: dict, publish_points: bool) -> list[ComposableNode]:
    """stereo_image_proc nodes in /stereo (inputs resolve to /stereo/{left,right}/image_rect and camera_info).

    Args:
        settings: Merged settings.
        publish_points: Add the PointCloudNode.

    Returns:
        list[ComposableNode]: DisparityNode, plus PointCloudNode when requested.
    """
    nodes = [
        ComposableNode(
            package="stereo_image_proc",
            plugin="stereo_image_proc::DisparityNode",
            name="disparity_node",
            namespace=STEREO_NS,
            parameters=[disparity_parameters(settings)],
            extra_arguments=INTRA_PROCESS,
        )
    ]
    if publish_points:
        nodes.append(
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::PointCloudNode",
                name="point_cloud_node",
                namespace=STEREO_NS,
                parameters=[{"approximate_sync": settings["disparity"]["approximate_sync"], "use_color": True}],
                remappings=[
                    ("left/image_rect_color", "left/image_rect"),
                    ("right/image_rect_color", "right/image_rect"),
                ],
                extra_arguments=INTRA_PROCESS,
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    """Build the launch description from the merged settings and the calibration state.

    Returns:
        LaunchDescription: Container with the camera nodes and, when calibrated, the rectify / stereo nodes.
    """
    settings = load_settings(
        Path(os.environ.get(PARAMS_ENV, DEFAULT_PARAMS_PATH)),
        Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)),
    )
    publish_points = bool(settings["launch"]["publish_points"])
    calibrated, reasons = calibration_ready(CALIBRATION_DIR / "left.yaml", CALIBRATION_DIR / "right.yaml")
    stages = stage_plan(calibrated, publish_points)
    warnings: list[str] = []
    nodes: list[ComposableNode] = []
    for side in ("left", "right"):
        node, side_warnings = camera_node(side, settings)
        nodes.append(node)
        warnings += side_warnings
    if calibrated:
        nodes += [rectify_node("left"), rectify_node("right")]
        nodes += stereo_nodes(settings, "points" in stages)
    else:
        warnings.append(
            "stereo_camera: no usable calibration (" + "; ".join(reasons) + "): publishing raw images only, "
            "no rectified images, disparity or point cloud. See nodes/stereo_camera/calibration/README.md."
        )
    if publish_points and not calibrated:
        warnings.append("stereo_camera: publish_points is set but needs the calibration: no point cloud.")
    container = ComposableNodeContainer(
        name="stereo_container",
        namespace=STEREO_NS,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=nodes,
        output="screen",
    )
    return LaunchDescription([LogInfo(msg=f"WARNING: {text}") for text in warnings] + [container])
