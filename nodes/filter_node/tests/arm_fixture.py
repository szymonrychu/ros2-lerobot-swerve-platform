"""The real arm checker for the guard tests: the packaged robot body with the deployed mcp_server arm calibration
(ansible/group_vars/client.yml arm: block, 2026-11-01), and arm poses FK-checked against it."""

import numpy as np
from arm_collision.ik import ArmKinematics
from arm_collision.jaw import JawModel, yaw_matrix
from arm_collision.self_collision import SelfCollisionChecker
from arm_collision.settings import REPO_ROOT, ArmModelSettings, BodyCheckSettings, load_robot_body

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
HARD_MARGIN_M = 0.005
KIN = ArmKinematics(
    ARM.urdf_path,
    margin=ARM_LIMIT_MARGIN_RAD,
    joint_offsets=ARM.joint_offsets_rad.model_dump(),
    tool_offset=tuple(ARM.tool_offset_m.model_dump().values()),
    limit_overrides=ARM.joint_limit_overrides_rad,
)
JAW = JawModel(KIN, ARM.jaw_open_axis, ARM.gripper_closed_rad)
CHECKER = SelfCollisionChecker.from_settings(
    KIN, load_robot_body(), BodyCheckSettings(hard_margin_m=HARD_MARGIN_M), ARM, JAW
)
MOUNT = ARM.base_in_base_link
FK_TOLERANCE_M = 1e-3
# Top of the physical wheel_fr (60 mm radius on a 60 mm axle height) and its steer axis in base_link.
WHEEL_TOP_Z_M = 0.12
WHEEL_FR_XY = (0.1525, -0.1333)
# Parked home (scripts/gen_nav_footprint.yaml parked_arm_joints) and the drive pose (client.yml arm.drive_pose.joints).
PARKED_HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
# Gripper pointing down over the wheel_fr steer axis (IK solutions rounded to 1e-4 rad, FK-checked in test_guard.py):
# the tool point 2 cm below the wheel top (pressed into the wheel, the incident) and 1 cm above it (inside the 3 cm
# wheel envelope).
WHEEL_FR_PRESSED = {
    "shoulder_pan": 0.9253,
    "shoulder_lift": -0.5382,
    "elbow_flex": 1.2508,
    "wrist_flex": 0.9008,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
WHEEL_FR_PRESSED_TOOL_Z_M = WHEEL_TOP_Z_M - 0.02
WHEEL_FR_INSIDE = {
    "shoulder_pan": 0.9253,
    "shoulder_lift": -0.6764,
    "elbow_flex": 1.0984,
    "wrist_flex": 1.1915,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
WHEEL_FR_INSIDE_TOOL_Z_M = WHEEL_TOP_Z_M + 0.01
DRIVE_POSE = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.711,
    "elbow_flex": -1.483,
    "wrist_flex": 1.543,
    "wrist_roll": -1.637,
    "gripper": 0.1,
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
