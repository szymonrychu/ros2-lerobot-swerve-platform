"""Synthetic D435 world for the pixel_to_point / camera_point_to_arm tests.

The truth is the calibration library's chain (UrdfChain + apply_overrides + arm_in_robot_base, the model the head-arm
fit solved); the robot under test is the MCP server stack on a FakeRobot whose head runs the applied chain.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual, head_chain_json

from mcp_server.camera_scene import CameraInfoData
from mcp_server.config import McpServerConfig
from mcp_server.depth_source import DepthFrame
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.head_kinematics import CAMERA_FRAME
from mcp_server.models import CameraFrame

from .camera_helpers import jpeg_blank  # noqa: F401  (kept for tests that need a blank image)
from .fakes import FakeHeadBackend
from .test_camera_arm_chain import APPLIED, ARM_BASE, calibration_chain, calibration_t_arm_optical
from .test_camera_tools import CameraRobot

STAMP = 50.0
WIDTH, HEIGHT = 640, 360
INFO_TOPIC = "/head/d435/color/camera_info"
INFO = CameraInfoData(
    width=WIDTH,
    height=HEIGHT,
    k=(455.2, 0.0, 321.7, 0.0, 455.4, 180.9, 0.0, 0.0, 1.0),
    d=(0.0, 0.0, 0.0, 0.0, 0.0),
    distortion_model="plumb_bob",
    stamp=12.5,
)
FX, FY, CX, CY = 455.2, 455.4, 321.7, 180.9
D435_MOUNT = {"parent_frame": "d435_color_frame", "x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
HEAD_RATE_HZ = 50.0
OBJECT_RADIUS_PX = 14
BACKGROUND_OFFSET_M = 0.3
# Tool points 0.25-0.37 m from the camera that the head can aim at (right, ahead and left of the arm base).
ARM_JOINT_SETS = (
    {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0},
    {"shoulder_pan": -0.3, "shoulder_lift": 0.3, "elbow_flex": 0.0, "wrist_flex": 0.4, "wrist_roll": 0.2},
    {"shoulder_pan": 0.3, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": -0.3},
    {"shoulder_pan": -0.6, "shoulder_lift": 0.3, "elbow_flex": 0.3, "wrist_flex": -0.6, "wrist_roll": 0.0},
)


def depth_config(**arm: object) -> McpServerConfig:
    """Config with the deployed head camera (CameraInfo intrinsics, zero mount on d435_color_frame) and the arm mount."""
    data: dict[str, object] = {
        "cameras": {"head": {"intrinsics": {"camera_info_topic": INFO_TOPIC}, "mount": D435_MOUNT}},
        "arm": {"base_in_base_link": ARM_BASE, **arm},
    }
    return McpServerConfig.model_validate(data)


def write_applied(
    directory: Path, residual: ChainResidual | None, source: str = "head_arm", result: str = "r-1"
) -> Path:
    """Write the applied head chain (synthetic fit-sized origins) with its applied.json record, marker last."""
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "head_chain.json").write_text(head_chain_json(APPLIED, "synthetic", "calib-1", result, residual))
    record = {"session": "calib-1", "result": result, "applied_at": "2026-10-29T10:00:00+00:00", "source": source}
    (directory / "applied.json").write_text(json.dumps({"head_chain": record}))
    return directory


def install_head(
    robot: CameraRobot, cfg: McpServerConfig, applied: Path | None, pan: float, tilt: float
) -> HeadController:
    """Replace the robot's head by one on the applied chain at (pan, tilt) with a static joint-state history."""
    backend = FakeHeadBackend({"head_pan": pan, "head_tilt": tilt})
    head = HeadController(backend, AppliedHeadKinematics(cfg.head.urdf_path, applied), cfg.head)
    robot.head_backend = backend
    robot.head = head
    feed_history(head, pan, tilt)
    return head


def feed_history(head: HeadController, pan: float, tilt: float, around: float = STAMP, span_s: float = 1.0) -> None:
    """Static head joint states at HEAD_RATE_HZ over around +- span_s."""
    steps = round(2 * span_s * HEAD_RATE_HZ)
    for i in range(steps + 1):
        head.history.add(around - span_s + i / HEAD_RATE_HZ, {"head_pan": pan, "head_tilt": tilt})


class DepthRobot(CameraRobot):
    """CameraRobot with D435 sized colour frames, the colour CameraInfo and a settable aligned depth frame."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.camera_infos["head"] = INFO

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        self.calls.append(("camera_image", (camera, max_px)))
        ok, jpg = cv2.imencode(".jpg", np.full((HEIGHT, WIDTH, 3), 90, np.uint8))
        assert ok
        return CameraFrame(
            camera=camera, topic="/x", jpeg=jpg.tobytes(), width=WIDTH, height=HEIGHT, stamp_s=STAMP, age_s=0.05
        )


def depth_frame(depth: np.ndarray, stamp_s: float = STAMP, info: CameraInfoData = INFO) -> DepthFrame:
    """A DepthFrame paired with a colour frame of the same stamp."""
    return DepthFrame(depth=depth, stamp_s=stamp_s, color_stamp_s=stamp_s, age_s=0.05, info=info)


def flat_depth(z_m: float) -> np.ndarray:
    """A fronto-parallel wall at z_m (uint16 mm)."""
    return np.full((HEIGHT, WIDTH), round(z_m * 1000), dtype=np.uint16)


def object_depth(
    u: float, v: float, z_m: float, noise_sigma_m: float = 0.0, seed: int = 0, background_m: float | None = None
) -> np.ndarray:
    """Depth image of a small flat-faced object centred on pixel (u, v) at depth z_m in front of a farther wall."""
    rng = np.random.default_rng(seed)
    back = z_m + BACKGROUND_OFFSET_M if background_m is None else background_m
    ys, xs = np.mgrid[0:HEIGHT, 0:WIDTH]
    inside = (xs - u) ** 2 + (ys - v) ** 2 <= OBJECT_RADIUS_PX**2
    depth_m = np.where(inside, z_m, back)
    if noise_sigma_m:
        depth_m = depth_m + rng.normal(0.0, noise_sigma_m, size=depth_m.shape)
    return np.round(depth_m * 1000).astype(np.uint16)


def project_arm_point(applied: Path, p_arm: np.ndarray, pan: float, tilt: float) -> tuple[float, float, float]:
    """(u, v, z) of an arm-frame point through the calibration chain and the colour intrinsics (truth)."""
    t_arm_optical = calibration_t_arm_optical(calibration_chain(applied), CAMERA_FRAME, pan, tilt)
    x, y, z = (np.linalg.inv(t_arm_optical) @ np.append(p_arm, 1.0))[:3]
    return FX * x / z + CX, FY * y / z + CY, float(z)


def d435_sigma_m(z_m: float) -> float:
    """The z^2 depth error model of the D435 (subpixel 0.08 px, baseline 50 mm)."""
    return z_m * z_m * 0.08 / (FX * 0.05)


def deg(rad: float) -> float:
    return math.degrees(rad)
