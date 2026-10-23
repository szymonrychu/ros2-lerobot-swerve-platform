"""Synthetic head-from-arm scenes: the gripper board seen by the head camera through a known (truth) head chain.

The arm is exact (the measured joints plus `cfg.joint_offsets_rad` through the arm URDF and `cfg.base_in_base_link`),
the head is aimed at the true board centre with `aim_head` on the truth chain plus each head offset, and the ChArUco
corners are projected through the truth chain, so a fit started from the applied chain has to recover the truth. Used
by the fit tests, the arm pose set tests and the node tests.
"""

import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import MIN_CORNERS, Detection, board_corners_3d
from ros2_common.head_rgbd_calibration.config import BoardMountConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    CAMERA_FRAME,
    GRIPPER_LINK,
    MOUNT_JOINT,
    PAN_JOINT,
    TILT_JOINT,
    aim_head,
    board_centre,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin, rewrite_joint_origin
from ros2_common.head_rgbd_calibration.views import ArmBoardView

URDF_DIR = Path(__file__).resolve().parents[1] / "nodes" / "web_ui" / "urdf"
APPLIED_NOTE = "floor fit 2026-10-22"
# The applied floor-fit chain of 2026-10-22 (floor_fit.PARAM_NAMES, m and deg): the start chain of every scene.
APPLIED_FLOOR = {
    "mount_z_m": 0.246,
    "mount_roll_deg": -0.32,
    "mount_pitch_deg": 1.50,
    "tilt_zero_deg": -6.50,
    "tilt_roll_deg": 0.92,
}
# Truth of the standard scene: the start chain moved by these amounts (head_arm_fit.PARAM_NAMES, m and deg).
SYNTH_TRUTH_DELTAS = {
    "mount_x_m": 0.008,
    "mount_y_m": -0.006,
    "mount_z_m": 0.005,
    "mount_roll_deg": 0.4,
    "mount_pitch_deg": -0.5,
    "mount_yaw_deg": 1.5,
    "tilt_zero_deg": 0.6,
    "tilt_roll_deg": -0.4,
}
MOUNT_INIT_SHIFT_M = 0.010
MOUNT_INIT_TURN_DEG = 8.0
IMAGE_SIZE = (576, 324)
SYNTH_CAMERA_MATRIX = np.array([[440.0, 0.0, 288.0], [0.0, 440.0, 162.0], [0.0, 0.0, 1.0]])
SYNTH_DIST = np.zeros(5)
TOOL_POINT = (-0.0079, -0.000218121, -0.0981274)
BOARD_STANDOFF_M = 0.02
TOOL_AXIS = (0.0, 0.0, -1.0)  # tool direction in gripper_link (gripper_frame_joint lies along -z)
# The user's mount (2026-10-23): the 250 x 210 mm sheet glued at the middle of its lower long edge to a stick held in
# the closed gripper; the lower edge this far above the tool axis when parked (the stick offset).
BOARD_OUTER_HEIGHT_M = 0.21
BOARD_EDGE_ABOVE_AXIS_M = 0.015
MIN_FACING_COS = 0.2
# Board centre about 0.37 m in front of the head camera, lifted to about the camera height, clear of the body.
BASE_ARM_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.5,
    "elbow_flex": 0.0,
    "wrist_flex": -0.5,
    "wrist_roll": 0.0,
}
# (shoulder_pan, elbow_flex, wrist_flex, wrist_roll) changes from the base pose (rad).
ARM_POSE_CHANGES = (
    (0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.6),
    (0.0, 0.0, 0.0, -0.6),
    (0.0, 0.0, 0.0, 1.2),
    (0.0, 0.0, 0.0, -1.2),
    (0.0, 0.0, 0.15, 0.0),
    (0.0, 0.0, -0.15, 0.0),
    (0.0, 0.0, 0.15, 0.8),
    (0.0, 0.0, -0.15, -0.8),
    (0.0, 0.15, -0.15, 0.0),
    (0.0, 0.15, 0.0, 0.5),
    (0.0, -0.15, 0.15, 0.0),
    (0.0, -0.15, 0.0, -0.5),
    (0.15, 0.0, 0.0, 0.0),
    (0.15, 0.0, 0.15, 0.6),
    (0.15, 0.15, -0.15, -0.6),
    (0.15, -0.15, 0.15, 1.0),
    (-0.15, 0.0, 0.0, 0.0),
    (-0.15, 0.0, -0.15, -0.6),
    (-0.15, 0.15, 0.0, 0.6),
    (-0.15, -0.15, 0.15, -1.0),
    (0.075, 0.075, 0.075, 0.3),
    (-0.075, -0.075, -0.075, -0.3),
    (0.075, -0.075, 0.15, -0.9),
)
SYNTH_ARM_POSES: list[dict[str, float]] = [
    {
        **BASE_ARM_POSE,
        "shoulder_pan": BASE_ARM_POSE["shoulder_pan"] + pan,
        "elbow_flex": BASE_ARM_POSE["elbow_flex"] + elbow,
        "wrist_flex": BASE_ARM_POSE["wrist_flex"] + flex,
        "wrist_roll": BASE_ARM_POSE["wrist_roll"] + roll,
    }
    for pan, elbow, flex, roll in ARM_POSE_CHANGES
]
DEFAULT_HEAD_OFFSETS_DEG: list[tuple[float, float]] = [(0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, 6.0)]


@dataclass(frozen=True)
class HeadArmTruth:
    """Truth of a synthetic scene.

    Attributes:
        origins: Head origin overrides (joint -> Origin) of the given head URDF; joints not named keep the URDF.
        board_xyz: Board frame origin in gripper_link (m).
        board_rpy: Board frame fixed-axis xyz rotation in gripper_link (rad).
    """

    origins: dict[str, Origin] = field(default_factory=dict)
    board_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    board_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def board_pose(self) -> np.ndarray:
        """4x4 pose of the board frame in gripper_link."""
        return pose_from_xyz_rpy(self.board_xyz, self.board_rpy)


def applied_head_urdf(head_urdf: str, floor: dict[str, float] | None = None) -> str:
    """Head URDF text with a floor-fit chain written into head_mount_joint (z, roll, pitch) and head_tilt (origin
    pitch and roll); mount x/y/yaw and the head_tilt xyz/yaw keep the URDF values.

    Args:
        head_urdf: Head URDF text (e.g. nodes/web_ui/urdf/head.urdf).
        floor: Floor-fit values per floor_fit.PARAM_NAMES entry (m and deg); `APPLIED_FLOOR` when None.

    Returns:
        str: The rewritten URDF text.
    """
    floor = APPLIED_FLOOR if floor is None else floor
    chain = UrdfChain.from_text(head_urdf)
    (mx, my, _), (_, _, myaw) = chain.origin(MOUNT_JOINT)
    txyz, (_, _, tyaw) = chain.origin(TILT_JOINT)
    mount = Origin(
        (mx, my, floor["mount_z_m"]),
        (math.radians(floor["mount_roll_deg"]), math.radians(floor["mount_pitch_deg"]), myaw),
    )
    tilt = Origin(txyz, (math.radians(floor["tilt_roll_deg"]), math.radians(floor["tilt_zero_deg"]), tyaw))
    text = rewrite_joint_origin(head_urdf, MOUNT_JOINT, mount, APPLIED_NOTE)
    return rewrite_joint_origin(text, TILT_JOINT, tilt, APPLIED_NOTE)


def synth_head_urdf() -> str:
    """nodes/web_ui/urdf/head.urdf with the applied floor-fit chain of 2026-10-22 (`APPLIED_FLOOR`)."""
    return applied_head_urdf((URDF_DIR / "head.urdf").read_text())


def synth_arm_urdf() -> str:
    """nodes/web_ui/urdf/so101_arm.urdf text."""
    return (URDF_DIR / "so101_arm.urdf").read_text()


def nominal_board_mount(cfg: HeadArmFitConfig) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A board mount that faces back along the tool axis: the board plane normal to gripper_link z, its centre on the
    tool axis `BOARD_STANDOFF_M` beyond the gripper_frame_link point, its front toward the arm.

    Args:
        cfg: Fit config (board geometry).

    Returns:
        tuple[xyz, rpy]: Board frame origin (m) and fixed-axis rotation (rad) in gripper_link.
    """
    # Board z (into the board, away from a viewer behind the gripper) along -z of gripper_link (the tool direction).
    rotation = pose_from_xyz_rpy((0.0, 0.0, 0.0), (math.pi, 0.0, 0.0))[:3, :3]
    centre = np.array([TOOL_POINT[0], TOOL_POINT[1], TOOL_POINT[2] - BOARD_STANDOFF_M])
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = centre - rotation @ board_centre(cfg.board)
    return xyz_rpy_from_pose(pose)


def park_up_in_gripper(cfg: HeadArmFitConfig, park_pose: dict[str, float]) -> np.ndarray:
    """Unit vector in gripper_link, normal to the tool axis, that points up (base_link +z) at the parked arm.

    Args:
        cfg: Fit config (`joint_offsets_rad`; the arm base is only yawed in base_link, so its z is up).
        park_pose: Measured (follower) joints of the parked arm (rad).

    Returns:
        np.ndarray: (3,) unit vector in gripper_link.
    """
    gripper = UrdfChain.from_text(synth_arm_urdf()).transform(GRIPPER_LINK, park_pose, offsets=cfg.joint_offsets_rad)
    axis = np.array(TOOL_AXIS)
    up = gripper[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    up -= (up @ axis) * axis
    return up / np.linalg.norm(up)


def measured_board_mount(
    cfg: HeadArmFitConfig, park_pose: dict[str, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """The user's board mount: the plane across the tool axis `BOARD_STANDOFF_M` past the gripper_frame_link point,
    the print (board -z) toward the arm, the long edges level and board y down when parked, the centre
    `BOARD_EDGE_ABOVE_AXIS_M` plus half `BOARD_OUTER_HEIGHT_M` above the tool axis on the side up when parked.

    Args:
        cfg: Fit config (board geometry, joint offsets).
        park_pose: Measured (follower) joints of the parked arm (rad), which define "up".

    Returns:
        tuple[xyz, rpy]: Board frame origin (m) and fixed-axis rotation (rad) in gripper_link.
    """
    axis = np.array(TOOL_AXIS)
    up = park_up_in_gripper(cfg, park_pose)
    rotation = np.column_stack([np.cross(-up, axis), -up, axis])
    centre = np.array(TOOL_POINT) + BOARD_STANDOFF_M * axis + (BOARD_EDGE_ABOVE_AXIS_M + BOARD_OUTER_HEIGHT_M / 2) * up
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = centre - rotation @ board_centre(cfg.board)
    return xyz_rpy_from_pose(pose)


def perturbed_board_mount(
    xyz: tuple[float, float, float], rpy: tuple[float, float, float], shift_m: float, turn_deg: float
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A board mount moved by `shift_m` along (1, 1, 1)/sqrt(3) and turned by `turn_deg` about (1, -1, 1)/sqrt(3).

    Args:
        xyz: Mount origin (m).
        rpy: Mount fixed-axis rotation (rad).
        shift_m: Translation length (m).
        turn_deg: Rotation angle (deg), applied in the gripper frame.

    Returns:
        tuple[xyz, rpy]: The perturbed mount.
    """
    pose = pose_from_xyz_rpy(xyz, rpy)
    turn = np.eye(4)
    axis = np.array([1.0, -1.0, 1.0]) / math.sqrt(3.0)
    turn[:3, :3] = cv2.Rodrigues(axis * math.radians(turn_deg))[0]
    moved = turn @ pose
    moved[:3, 3] = pose[:3, 3] + shift_m * np.ones(3) / math.sqrt(3.0)
    return xyz_rpy_from_pose(moved)


def perturbed_origins(head_urdf: str, deltas: dict[str, float]) -> dict[str, Origin]:
    """head_mount_joint and head_tilt origins of `head_urdf` moved by `deltas` (head_arm_fit.PARAM_NAMES units).

    Args:
        head_urdf: Head URDF text (the start chain).
        deltas: Per parameter name the change from the URDF value (m for `*_m`, deg for `*_deg`); missing names are 0.

    Returns:
        dict[str, Origin]: Origins of head_mount_joint and head_tilt.
    """
    chain = UrdfChain.from_text(head_urdf)
    (mx, my, mz), (mroll, mpitch, myaw) = chain.origin(MOUNT_JOINT)
    txyz, (troll, tpitch, tyaw) = chain.origin(TILT_JOINT)

    def deg(name: str) -> float:
        return math.radians(deltas.get(name, 0.0))

    return {
        MOUNT_JOINT: Origin(
            (mx + deltas.get("mount_x_m", 0.0), my + deltas.get("mount_y_m", 0.0), mz + deltas.get("mount_z_m", 0.0)),
            (mroll + deg("mount_roll_deg"), mpitch + deg("mount_pitch_deg"), myaw + deg("mount_yaw_deg")),
        ),
        TILT_JOINT: Origin(txyz, (troll + deg("tilt_roll_deg"), tpitch + deg("tilt_zero_deg"), tyaw)),
    }


def synth_truth(
    head_urdf: str,
    deltas: dict[str, float] | None = None,
    extra: dict[str, Origin] | None = None,
    cfg: HeadArmFitConfig | None = None,
    mount: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
) -> HeadArmTruth:
    """Truth of a scene: the chain of `head_urdf` moved by `deltas`, plus `extra` origins, and the board mount.

    Args:
        head_urdf: Head URDF text (the start chain).
        deltas: Per head_arm_fit.PARAM_NAMES entry the change from the URDF value (m or deg); `SYNTH_TRUTH_DELTAS`
            when None.
        extra: Further origin overrides (e.g. a head_pan origin), applied after `deltas`.
        cfg: Fit config whose board geometry places the nominal mount; the default config when None.
        mount: Board mount (xyz m, rpy rad) in gripper_link; `nominal_board_mount` when None.

    Returns:
        HeadArmTruth: The truth.
    """
    xyz, rpy = mount or nominal_board_mount(cfg or HeadArmFitConfig())
    moved = perturbed_origins(head_urdf, SYNTH_TRUTH_DELTAS if deltas is None else deltas)
    return HeadArmTruth(origins={**moved, **(extra or {})}, board_xyz=xyz, board_rpy=rpy)


def synth_fit_config(**overrides: object) -> HeadArmFitConfig:
    """Fit config whose board mount start is `MOUNT_INIT_SHIFT_M` / `MOUNT_INIT_TURN_DEG` off the nominal (truth)
    mount.

    Args:
        **overrides: Further HeadArmFitConfig fields.

    Returns:
        HeadArmFitConfig: The config.
    """
    xyz, rpy = perturbed_board_mount(*nominal_board_mount(HeadArmFitConfig()), MOUNT_INIT_SHIFT_M, MOUNT_INIT_TURN_DEG)
    return HeadArmFitConfig(board_mount=BoardMountConfig(xyz_init=xyz, rpy_init=rpy), **overrides)


def truth_chain(chain: UrdfChain, origins: dict[str, Origin]) -> UrdfChain:
    """`chain` with the given joint origins replaced.

    Args:
        chain: Parsed head URDF.
        origins: Origin per joint name.

    Returns:
        UrdfChain: A new chain; joints not named keep their URDF origin.
    """
    return UrdfChain(
        [
            replace(j, xyz=origins[j.name].xyz, rpy=origins[j.name].rpy) if j.name in origins else j
            for j in chain.joints.values()
        ]
    )


def arm_board_scene(
    truth: HeadArmTruth,
    head_urdf: str,
    arm_urdf: str,
    arm_poses: list[dict[str, float]],
    head_offsets_deg: list[tuple[float, float]],
    noise_px: float,
    seed: int,
    cfg: HeadArmFitConfig,
    tiers: list[int] | None = None,
) -> list[ArmBoardView]:
    """Views of the gripper board through the truth chain.

    For every arm pose the head is aimed at the true board centre with `aim_head` on the truth chain, then moved by
    each (pan, tilt) offset (deg) and clamped to the head limits. A corner counts when it is in front of the camera,
    inside the image and the board front faces the camera; views with fewer than `MIN_CORNERS` corners are dropped.

    Args:
        truth: Truth head origins and board mount.
        head_urdf: Head URDF text the truth origins override.
        arm_urdf: Arm URDF text.
        arm_poses: Measured arm joints (rad) per pose; FK applies `cfg.joint_offsets_rad`.
        head_offsets_deg: (pan, tilt) offsets (deg) added to the aimed head angles, one view each.
        noise_px: Standard deviation of the Gaussian pixel noise.
        seed: Random seed of the noise.
        cfg: Fit config (board, gauge, joint offsets).
        tiers: Tier of each arm pose; 0 for every pose when None.

    Returns:
        list[ArmBoardView]: Views, id `p<pose>-h<offset>`, `arm_pose` the index into `arm_poses`.
    """
    rng = np.random.default_rng(seed)
    head = UrdfChain.from_text(head_urdf)
    arm = UrdfChain.from_text(arm_urdf)
    truth_head = truth_chain(head, truth.origins)
    base = arm_in_robot_base(cfg.base_in_base_link.model_dump())
    corners = board_corners_3d(cfg.board)
    homogeneous = np.column_stack([corners, np.ones(len(corners))])
    pan_limits = head.limits(PAN_JOINT) or (-math.pi, math.pi)
    tilt_limits = head.limits(TILT_JOINT) or (-math.pi, math.pi)
    views = []
    for pose_index, joints in enumerate(arm_poses):
        board_in_base = base @ arm.transform(GRIPPER_LINK, joints, offsets=cfg.joint_offsets_rad) @ truth.board_pose()
        centre = (board_in_base @ np.append(board_centre(cfg.board), 1.0))[:3]
        pan0, tilt0 = aim_head(truth_head, centre)
        for k, (dpan, dtilt) in enumerate(head_offsets_deg):
            pan = float(np.clip(pan0 + math.radians(dpan), *pan_limits))
            tilt = float(np.clip(tilt0 + math.radians(dtilt), *tilt_limits))
            camera = truth_head.transform(CAMERA_FRAME, {PAN_JOINT: pan, TILT_JOINT: tilt})
            board_in_camera = np.linalg.inv(camera) @ board_in_base
            if board_in_camera[2, 2] < MIN_FACING_COS:
                continue
            points_camera = (board_in_camera @ homogeneous.T).T[:, :3]
            pixels = (SYNTH_CAMERA_MATRIX @ points_camera.T).T
            pixels = pixels[:, :2] / pixels[:, 2:3]
            visible = (
                (points_camera[:, 2] > 0)
                & (pixels[:, 0] >= 0)
                & (pixels[:, 0] <= IMAGE_SIZE[0] - 1)
                & (pixels[:, 1] >= 0)
                & (pixels[:, 1] <= IMAGE_SIZE[1] - 1)
            )
            if visible.sum() < MIN_CORNERS:
                continue
            ids = np.flatnonzero(visible).astype(np.int32)
            points = pixels[ids] + rng.normal(0.0, noise_px, size=(len(ids), 2))
            views.append(
                ArmBoardView(
                    view_id=f"p{pose_index:02d}-h{k}",
                    arm_pose=pose_index,
                    tier=0 if tiers is None else tiers[pose_index],
                    joints={**joints, PAN_JOINT: pan, TILT_JOINT: tilt},
                    detection=Detection(ids=ids, points=points.astype(np.float64)),
                )
            )
    return views
