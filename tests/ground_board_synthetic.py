"""Synthetic ground board scenes: the FULL ChArUco board lying face up on the floor at one or more placements, seen by
the D435 colour camera (`CAMERA_FRAME`, head.urdf) through a known (truth) head chain over a pan/tilt grid around the
board, plus the floor plane in the D435 depth frame at every head pose.

The applied chain is `synth_head_urdf()` (the 2026-10-22 floor-fit chain); the truth moves it by `GROUND_TRUTH_DELTAS`
(head_arm_fit parameter names, m and deg) and the pan axis by `GROUND_PAN_AXIS_M`, so a fit started from the applied
chain has to recover the truth. A head pose is kept only when every board corner lands inside the image with
`IMAGE_MARGIN_PX` to spare. Used by the ground board fit tests and the node tests.
"""

import importlib.util
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.charuco import Detection, board_corners_3d
from ros2_common.head_rgbd_calibration.config import FULL_BOARD, BoardConfig
from ros2_common.head_rgbd_calibration.floor_fit import FloorObservation, PlaneFit, predicted_floor
from ros2_common.head_rgbd_calibration.ground_board_fit import GroundBoardView, board_on_floor
from ros2_common.head_rgbd_calibration.head_arm_fit import PAN_JOINT, TILT_JOINT, aim_head, camera_pose
from ros2_common.head_rgbd_calibration.urdf_text import Origin

# Loaded by path, not as tests.head_arm_synthetic: the node tests load this module by path too, and there `tests` is
# the node's own test package.
HEAD_ARM_SPEC = importlib.util.spec_from_file_location(
    "head_arm_synthetic", Path(__file__).with_name("head_arm_synthetic.py")
)
assert HEAD_ARM_SPEC is not None and HEAD_ARM_SPEC.loader is not None
head_arm_synthetic = importlib.util.module_from_spec(HEAD_ARM_SPEC)
HEAD_ARM_SPEC.loader.exec_module(head_arm_synthetic)
CAMERA_FRAME = head_arm_synthetic.CAMERA_FRAME
DEPTH_FRAME = head_arm_synthetic.DEPTH_FRAME

# D435 colour at 640x360 (the robot's stream), factory-like intrinsics.
GROUND_IMAGE_SIZE = (640, 360)
GROUND_CAMERA_MATRIX = np.array([[455.0, 0.0, 320.0], [0.0, 455.0, 180.0], [0.0, 0.0, 1.0]])
GROUND_DIST = np.zeros(5)
IMAGE_MARGIN_PX = 5.0
# Truth of the standard scene: the applied chain moved by these amounts (m and deg).
GROUND_TRUTH_DELTAS = {
    "mount_z_m": 0.005,
    "mount_roll_deg": 0.4,
    "mount_pitch_deg": -0.5,
    "tilt_zero_deg": 0.6,
    "tilt_roll_deg": -0.4,
    "camera_x_m": 0.004,
    "camera_z_m": -0.003,
    "camera_roll_deg": 0.5,
    "camera_yaw_deg": 0.7,
}
GROUND_PAN_AXIS_M = (-0.02, 0.008)
# Board placements (x, y of the board centre in base_link, m; yaw, rad): close, in front, three different turns.
GROUND_PLACEMENTS = ((0.55, 0.10, 0.3), (0.45, -0.15, -0.6), (0.75, 0.05, 1.2))
GROUND_PAN_OFFSETS_DEG = (-25.0, -12.0, 0.0, 12.0, 25.0)
GROUND_TILT_OFFSETS_DEG = (-12.0, -6.0, 0.0, 6.0, 12.0)
MAX_TILT_DEG = 45.0
FLOOR_NOISE_DEG = 0.15
FLOOR_NOISE_MM = 1.5
FLOOR_INLIERS = 5000


@dataclass(frozen=True)
class GroundScene:
    """A synthetic ground board session.

    Attributes:
        views: Board views (one per kept head pose and frame).
        planes: Floor planes in the depth frame, one per kept head pose.
        truth: Truth origins (head_mount_joint, head_tilt, head_pan, d435_joint).
        placements: Truth (x, y, yaw) per placement.
        applied_urdf: Applied head URDF the fit starts from.
    """

    views: list[GroundBoardView]
    planes: list[FloorObservation]
    truth: dict[str, Origin]
    placements: tuple[tuple[float, float, float], ...]
    applied_urdf: str


def ground_truth(
    applied_urdf: str,
    deltas: dict[str, float] | None = None,
    pan_axis_m: tuple[float, float] = GROUND_PAN_AXIS_M,
) -> dict[str, Origin]:
    """Truth origins: the applied chain moved by `deltas` (head_arm_fit names, m and deg) and the pan axis shift.

    Args:
        applied_urdf: Applied head URDF text.
        deltas: Changes per parameter; `GROUND_TRUTH_DELTAS` when None.
        pan_axis_m: Pan axis shift (x, y) in head_pan_base (m).

    Returns:
        dict[str, Origin]: Origins of head_mount_joint, head_tilt, d435_joint and head_pan.
    """
    moved = head_arm_synthetic.perturbed_origins(applied_urdf, GROUND_TRUTH_DELTAS if deltas is None else deltas)
    axis = head_arm_synthetic.moved_pan_axis(applied_urdf, *pan_axis_m)
    moved["head_tilt"] = Origin(axis["head_tilt"].xyz, moved["head_tilt"].rpy)
    moved[PAN_JOINT] = axis[PAN_JOINT]
    return moved


def ground_scene(
    deltas: dict[str, float] | None = None,
    placements: tuple[tuple[float, float, float], ...] = GROUND_PLACEMENTS,
    pan_offsets_deg: tuple[float, ...] = GROUND_PAN_OFFSETS_DEG,
    tilt_offsets_deg: tuple[float, ...] = GROUND_TILT_OFFSETS_DEG,
    noise_px: float = 0.3,
    seed: int = 1,
    pan_axis_m: tuple[float, float] = GROUND_PAN_AXIS_M,
    board: BoardConfig = FULL_BOARD,
    frames_per_pose: int = 1,
) -> GroundScene:
    """Project the board through the truth chain at a pan/tilt grid around each placement.

    The head aims at the board centre with `aim_head` on the APPLIED chain (as the node does), then visits the grid of
    offsets around that aim (tilt capped at `MAX_TILT_DEG`); a pose is kept when every corner lands inside the image.

    Args:
        deltas: Truth changes from the applied chain (`GROUND_TRUTH_DELTAS` when None).
        placements: (x, y, yaw) of the board centre per placement.
        pan_offsets_deg: Pan offsets around the aim (deg).
        tilt_offsets_deg: Tilt offsets around the aim (deg).
        noise_px: Gaussian corner noise (px).
        seed: Noise seed.
        pan_axis_m: Truth pan axis shift.
        board: Board geometry.
        frames_per_pose: Frames per kept pose (each with its own noise).

    Returns:
        GroundScene: The scene.
    """
    applied = head_arm_synthetic.synth_head_urdf()
    chain = UrdfChain.from_text(applied)
    truth = ground_truth(applied, deltas, pan_axis_m)
    corners = board_corners_3d(board)
    ids = np.arange(len(corners), dtype=np.int32)
    rng = np.random.default_rng(seed)
    width, height = GROUND_IMAGE_SIZE
    views: list[GroundBoardView] = []
    planes: list[FloorObservation] = []
    pose = 0
    for placement, (x, y, yaw) in enumerate(placements):
        board_pose = board_on_floor(board, x, y, yaw)
        centre = (board_pose @ np.append(corners.mean(axis=0), 1.0))[:3]
        pan0, tilt0 = aim_head(chain, centre, CAMERA_FRAME)
        for d_tilt in tilt_offsets_deg:
            for d_pan in pan_offsets_deg:
                pan = pan0 + math.radians(d_pan)
                tilt = min(tilt0 + math.radians(d_tilt), math.radians(MAX_TILT_DEG))
                joints = {PAN_JOINT: pan, TILT_JOINT: tilt}
                in_camera = np.linalg.solve(camera_pose(chain, joints, truth, CAMERA_FRAME), board_pose)
                depth = (in_camera[:3, :3] @ corners.T).T + in_camera[:3, 3]
                points = cv2.projectPoints(
                    corners, cv2.Rodrigues(in_camera[:3, :3])[0], in_camera[:3, 3], GROUND_CAMERA_MATRIX, GROUND_DIST
                )[0].reshape(-1, 2)
                inside = (
                    (points[:, 0] >= IMAGE_MARGIN_PX)
                    & (points[:, 0] <= width - IMAGE_MARGIN_PX)
                    & (points[:, 1] >= IMAGE_MARGIN_PX)
                    & (points[:, 1] <= height - IMAGE_MARGIN_PX)
                )
                if (depth[:, 2] <= 0).any() or not inside.all():
                    continue
                for frame in range(frames_per_pose):
                    noisy = points + rng.normal(0.0, noise_px, points.shape)
                    views.append(
                        GroundBoardView(
                            view_id=f"p{placement}-{pose:03d}-{frame}",
                            placement=placement,
                            pose=pose,
                            pan_rad=pan,
                            tilt_rad=tilt,
                            detection=Detection(ids=ids, points=noisy.astype(np.float32)),
                        )
                    )
                normal, plane_height = predicted_floor(chain, pan, tilt, truth, DEPTH_FRAME)
                turn = cv2.Rodrigues(rng.normal(0.0, math.radians(FLOOR_NOISE_DEG), 3))[0]
                planes.append(
                    FloorObservation(
                        pose=pose,
                        pan_rad=pan,
                        tilt_rad=tilt,
                        plane=PlaneFit(
                            normal=turn @ normal,
                            height_m=plane_height + rng.normal(0.0, FLOOR_NOISE_MM / 1000.0),
                            inliers=FLOOR_INLIERS,
                            candidates=FLOOR_INLIERS,
                            rms_m=0.005,
                        ),
                    )
                )
                pose += 1
    return GroundScene(views, planes, truth, placements, applied)
