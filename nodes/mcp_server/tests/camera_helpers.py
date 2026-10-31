"""Synthetic camera setups with known mounts shared by the camera tool tests."""

import math

import numpy as np
from ros2_common.camera_geometry import MountPose

from mcp_server.config import McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController

from .fakes import FakeHeadBackend

WIDTH = 640
HEIGHT = 480
HFOV_DEG = 90.0
FRONT_PITCH = 1.0  # rad, looking down
HEAD_MOUNT = {
    "parent_frame": "base_link",
    "x": 0.25,
    "y": 0.0,
    "z": 0.8,
    "roll": 0.0,
    "pitch": FRONT_PITCH,
    "yaw": 0.0,
}
HFOV = {"hfov_deg": HFOV_DEG, "width": WIDTH, "height": HEIGHT}


def camera_config(front: bool = True, arm_offset: bool = False, **extra: object) -> McpServerConfig:
    """Config with hfov-based intrinsics and the known HEAD_MOUNT for the head camera (none when front is False)."""
    cameras: dict[str, object] = {"head": {"intrinsics": HFOV, "mount": HEAD_MOUNT}} if front else {}
    data: dict[str, object] = {"cameras": cameras, **extra}
    # These tests use their own synthetic arm mount (16.5 cm high), or none at all.
    data["arm"] = {"arm_base_height_m": 0.165, "base_in_base_link": None}
    if arm_offset:
        data["arm"]["base_in_base_link"] = {"x": 0.1, "y": 0.0, "z": 0.165, "yaw": 0.0}
    return McpServerConfig.model_validate(data)


def fresh_head(cfg: McpServerConfig, pan: float = 0.0, tilt: float = 0.0) -> HeadController:
    """Head controller whose fresh /head/joint_states report (pan, tilt) rad (the head camera scene needs it)."""
    backend = FakeHeadBackend({"head_pan": pan, "head_tilt": tilt})
    return HeadController(backend, AppliedHeadKinematics(cfg.head.urdf_path, None), cfg.head)


def sync_head_history(head: HeadController, around_s: float, span_s: float = 0.5, rate_hz: float = 50.0) -> None:
    """Make the head joint-state history report the backend's current pose around a frame stamp (a still head).

    A real robot's /head/joint_states history holds the pose the head had when the frame was captured; the fake
    camera frames carry a fixed stamp, so the tests that move the fake head set the history from it per frame.

    Args:
        head (HeadController): Head whose history is replaced.
        around_s (float): Frame stamp (s).
        span_s (float): History kept on each side of the stamp (s).
        rate_hz (float): Joint-state rate (Hz).
    """
    positions = head.backend.head_sample().positions
    head.history.stamps.clear(), head.history.pans.clear(), head.history.tilts.clear()
    for i in range(round(2 * span_s * rate_hz) + 1):
        head.history.add(around_s - span_s + i / rate_hz, dict(positions))


def front_mount() -> MountPose:
    return MountPose(**HEAD_MOUNT)


def front_ground_of_center() -> tuple[float, float]:
    """Floor point seen at the image centre of the synthetic head camera (analytic)."""
    return 0.25 + 0.8 / math.tan(FRONT_PITCH), 0.0


def jpeg_blank() -> np.ndarray:
    return np.full((HEIGHT, WIDTH, 3), 90, np.uint8)
