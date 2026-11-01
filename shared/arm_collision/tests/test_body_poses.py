"""Arm poses against the packaged robot body, with the deployed mcp_server arm calibration (client.yml, 2026-11-01).

Every pose is FK-checked (the tool point is where the test says) before its clearance is asserted.
"""

import math

import numpy as np
import pytest

from arm_collision.head import HeadPose
from arm_collision.ik import ArmKinematics
from arm_collision.jaw import JawModel, yaw_matrix
from arm_collision.self_collision import SelfCollisionChecker
from arm_collision.settings import REPO_ROOT, ArmModelSettings, BodyCheckSettings, load_robot_body

# mcp_server arm calibration of ansible/group_vars/client.yml (arm: block) and its limits.arm_limit_margin_rad default.
ARM = ArmModelSettings(
    urdf_path=REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf",
    arm_base_height_m=0.100,
    joint_offsets_rad={
        "shoulder_pan": 0.0182,
        "shoulder_lift": -0.0345,
        "elbow_flex": -0.1070,
        "wrist_flex": 0.0988,
        "wrist_roll": -0.0710,
    },
    tool_offset_m={"x": 0.0010, "y": -0.0056, "z": -0.0014},
    joint_limit_overrides_rad={"shoulder_lift": (-1.745, 1.9)},
    base_in_base_link={"x": 0.0592, "y": -0.05, "z": 0.100, "yaw": 0.0},
)
ARM_LIMIT_MARGIN_RAD = 0.05
KIN = ArmKinematics(
    ARM.urdf_path,
    margin=ARM_LIMIT_MARGIN_RAD,
    joint_offsets=ARM.joint_offsets_rad.model_dump(),
    tool_offset=tuple(ARM.tool_offset_m.model_dump().values()),
    limit_overrides=ARM.joint_limit_overrides_rad,
)
JAW = JawModel(KIN, ARM.jaw_open_axis, ARM.gripper_closed_rad)
CHECK = BodyCheckSettings()
CHECKER = SelfCollisionChecker.from_settings(KIN, load_robot_body(), CHECK, ARM, JAW)
MOUNT = ARM.base_in_base_link
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
# Top of the physical wheel (60 mm radius on a 60 mm axle height): the wheel obstacle reaches 3 cm above it.
WHEEL_TOP_Z_M = 0.12
WHEEL_FR_XY = (0.1525, -0.1333)
HEAD_POST_XY = (0.09, 0.10)
HEAD_POST_RADIUS_M = 0.012
# Parked home (scripts/gen_nav_footprint.yaml parked_arm_joints) and the drive pose (client.yml arm.drive_pose.joints).
PARKED_HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
DRIVE_POSE = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.711,
    "elbow_flex": -1.483,
    "wrist_flex": 1.543,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
FK_TOLERANCE_M = 1e-3
# The posed head parts of head.urdf (head.HeadModel.parts): bracket and D435 boxes, pan base and pan link cylinders.
HEAD_PARTS = ("head_link", "d435_link", "head_pan_base", "head_pan_link")
# mcp_server head home (config.head home_pan_rad, home_tilt_rad): straight ahead, 30 deg down.
HEAD_HOME = HeadPose(pan=0.0, tilt=0.5236)
# Arm swung far left with the wrist folded up beside the head: inside the full head envelope, clear of the head at
# home (the envelope holds every pan/tilt, the posed head only the measured one).
BESIDE_THE_HEAD = {
    "shoulder_pan": 1.2,
    "shoulder_lift": -1.2,
    "elbow_flex": -0.9,
    "wrist_flex": 1.6,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


def tool_base_link(joints: dict[str, float]) -> np.ndarray:
    """FK tool point in base_link.

    Args:
        joints (dict[str, float]): Joint configuration (measured space, rad).

    Returns:
        np.ndarray: (3,) tool point (m).
    """
    pose = KIN.forward(joints)
    return yaw_matrix(MOUNT.yaw) @ np.array([pose.x, pose.y, pose.z]) + np.array([MOUNT.x, MOUNT.y, MOUNT.z])


def solve_base_link(x: float, y: float, z: float, pitch: float) -> dict[str, float]:
    """IK for a base_link tool point, FK-checked to 1 mm.

    Args:
        x (float): Tool point x in base_link (m).
        y (float): Tool point y in base_link (m).
        z (float): Tool point z in base_link (m).
        pitch (float): Approach pitch (rad, pi/2 = pointing down).

    Returns:
        dict[str, float]: Joint configuration.
    """
    target = yaw_matrix(MOUNT.yaw).T @ (np.array([x, y, z]) - np.array([MOUNT.x, MOUNT.y, MOUNT.z]))
    joints = KIN.inverse(*target, pitch, seed=SEED)
    assert np.linalg.norm(tool_base_link(joints) - np.array([x, y, z])) < FK_TOLERANCE_M
    return joints


def gripper_above_wheel_fr(height_m: float) -> dict[str, float]:
    """Gripper pointing down over wheel_fr, its lowest point height_m above the wheel top (FK-checked to 5 mm).

    Args:
        height_m (float): Height of the gripper's lowest point above the wheel top (m).

    Returns:
        dict[str, float]: Joint configuration.
    """
    joints = solve_base_link(*WHEEL_FR_XY, WHEEL_TOP_Z_M + height_m, math.pi / 2)
    lowest = min(float(points[..., 2].min()) for points in CHECKER.part_points(joints).values())
    assert lowest == pytest.approx(WHEEL_TOP_Z_M + height_m, abs=0.005)
    return joints


def test_gripper_2_cm_above_the_wheel_top_is_rejected() -> None:
    """Gripper pointing down over wheel_fr, its lowest point 2 cm above the wheel top: inside the 3 cm wheel envelope."""
    assert CHECKER.pose(gripper_above_wheel_fr(0.02)).per_obstacle["wheel_fr"][0] < CHECK.hard_margin_m


def test_gripper_4_cm_above_the_wheel_top_clears_the_wheel() -> None:
    """Gripper 4 cm above the wheel top: 1 cm over the 3 cm wheel envelope, clear of the 5 mm keep-away."""
    assert CHECKER.pose(gripper_above_wheel_fr(0.04)).per_obstacle["wheel_fr"][0] > CHECK.hard_margin_m


def test_touching_the_head_post_at_z_0_2_is_rejected() -> None:
    """Tool point on the head post surface (the side facing the arm mount) at z 0.2: the post is in the body."""
    toward_mount = np.array([MOUNT.x, MOUNT.y]) - np.array(HEAD_POST_XY)
    surface = np.array(HEAD_POST_XY) + HEAD_POST_RADIUS_M * toward_mount / np.linalg.norm(toward_mount)
    joints = solve_base_link(float(surface[0]), float(surface[1]), 0.2, 1.0)
    check = CHECKER.pose(joints)
    assert check.per_obstacle["head_post"][0] < CHECK.hard_margin_m


@pytest.mark.parametrize("joints", [PARKED_HOME, DRIVE_POSE], ids=["parked_home", "drive_pose"])
def test_parked_home_and_drive_pose_clear_the_hard_margin(joints: dict[str, float]) -> None:
    """The parked home and the drive pose stay clear of every obstacle by more than the hard margin."""
    assert tool_base_link(joints)[2] > WHEEL_TOP_Z_M  # FK check: both keep the tool above the wheels
    check = CHECKER.pose(joints)
    assert check.min_clearance > CHECK.hard_margin_m, (check.obstacle, check.part, check.min_clearance)


def test_a_measured_head_pose_replaces_the_envelope_by_the_posed_head() -> None:
    """Beside the head: rejected against the full envelope (no head pose), clear of the body with the head posed at
    home (the envelope gives way to the posed parts, boxes before cylinders as BodyModel orders them)."""
    assert CHECKER.posed(None) is CHECKER
    posed = CHECKER.posed(HEAD_HOME)
    assert CHECKER.pose(BESIDE_THE_HEAD).per_obstacle["head"][0] < CHECK.hard_margin_m
    assert posed.pose(BESIDE_THE_HEAD).min_clearance > CHECK.hard_margin_m
    names = [o.name for o in posed.body.obstacles]
    assert names == [*(n for n in (o.name for o in CHECKER.body.obstacles) if n != "head"), *HEAD_PARTS]


def test_the_posed_checker_is_reused_for_the_same_head_pose() -> None:
    """Asking again for the same head pose returns the same checker; another pose builds a new one."""
    first = CHECKER.posed(HeadPose(pan=0.1, tilt=0.2))
    assert CHECKER.posed(HeadPose(pan=0.1, tilt=0.2)) is first
    assert CHECKER.posed(HeadPose(pan=0.1, tilt=0.3)) is not first
