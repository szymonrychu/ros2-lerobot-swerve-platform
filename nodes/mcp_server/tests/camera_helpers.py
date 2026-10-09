"""Synthetic camera setups with known mounts shared by the camera tool tests."""

import math

import numpy as np
from ros2_common.camera_geometry import MountPose

from mcp_server.config import McpServerConfig

WIDTH = 640
HEIGHT = 480
HFOV_DEG = 90.0
FRONT_PITCH = 1.0  # rad, looking down
FRONT_MOUNT = {
    "parent_frame": "base_link",
    "x": 0.25,
    "y": 0.0,
    "z": 0.8,
    "roll": 0.0,
    "pitch": FRONT_PITCH,
    "yaw": 0.0,
}
GRIPPER_MOUNT = {"parent_frame": "gripper_link", "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
HFOV = {"hfov_deg": HFOV_DEG, "width": WIDTH, "height": HEIGHT}


def camera_config(
    front: bool = True, gripper: bool = True, arm_offset: bool = False, **extra: object
) -> McpServerConfig:
    """Config with hfov-based intrinsics and known mounts for the requested cameras."""
    cameras: dict[str, object] = {}
    if front:
        cameras["front"] = {"intrinsics": HFOV, "mount": FRONT_MOUNT}
    if gripper:
        cameras["gripper"] = {"intrinsics": HFOV, "mount": GRIPPER_MOUNT}
    data: dict[str, object] = {"cameras": cameras, **extra}
    # These tests use their own synthetic arm mount (16.5 cm high), or none at all.
    data["arm"] = {"arm_base_height_m": 0.165, "base_in_base_link": None}
    if arm_offset:
        data["arm"]["base_in_base_link"] = {"x": 0.1, "y": 0.0, "z": 0.165, "yaw": 0.0}
    return McpServerConfig.model_validate(data)


def front_mount() -> MountPose:
    return MountPose(**FRONT_MOUNT)


def front_ground_of_center() -> tuple[float, float]:
    """Floor point seen at the image centre of the synthetic front camera (analytic)."""
    return 0.25 + 0.8 / math.tan(FRONT_PITCH), 0.0


def jpeg_blank() -> np.ndarray:
    return np.full((HEIGHT, WIDTH, 3), 90, np.uint8)
