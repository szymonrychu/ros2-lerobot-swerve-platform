"""Arm link geometry shared by the surface checks (grasp planner) and the body self-collision check (self_collision).

Link bodies are capsules between FK link origins (radius = half the STS3215 servo body each link carries, about 4 cm
across); the gripper body and fixed jaw are a hull of points in gripper_link; the jaw points come from the floor guard
(FloorGuard.checked_points). The body check adds the arm collision boxes of the grasp_sim MuJoCo model
(LINK_COLLISION_BOXES) as points on their faces (box_surface_points), the upper arm boxes contact-only.
"""

import math
from dataclasses import dataclass

import numpy as np

# Capsule radius of the arm links (m): half the STS3215 servo body.
LINK_CAPSULE_RADIUS_M = 0.02
UPPER_ARM_CAPSULE = ("upper arm", "shoulder", "elbow", LINK_CAPSULE_RADIUS_M)
# Link bodies checked against surface regions as capsules between FK link origins (radius = half the STS3215 servo
# body that each link carries, about 4 cm across).
FOREARM_CAPSULE = ("forearm", "elbow", "wrist", LINK_CAPSULE_RADIUS_M)
WRIST_CAPSULE = ("wrist link", "wrist", "gripper", LINK_CAPSULE_RADIUS_M)
JAW_POINTS = ("jaw_tip", "tool_point", "moving_jaw_tip")
GRIPPER_BODY = "gripper body"
# Hull of the gripper body and fixed jaw in gripper_link (m): the corners of the housing collision boxes and the extreme
# points of the fixed finger mesh per 1 cm slice, taken from the grasp_sim MuJoCo model with the jaws calibrated to
# the 2026-10-10 tool point (its gripper body frame is gripper_link). Checked against surfaces with
# surface_jaw_clearance_m: at a diagonal step edge the jaw body, not its tip, is what touches first.
GRIPPER_BODY_POINTS = np.array(
    [
        (-0.038, -0.015, -0.038),
        (-0.037, -0.021, -0.040),
        (-0.037, -0.021, -0.030),
        (-0.037, 0.009, -0.040),
        (-0.037, 0.009, -0.030),
        (-0.035, -0.015, -0.037),
        (-0.035, -0.015, -0.007),
        (-0.035, 0.015, -0.037),
        (-0.035, 0.015, -0.007),
        (-0.034, -0.012, -0.049),
        (-0.034, 0.000, -0.049),
        (-0.023, -0.002, -0.081),
        (-0.020, -0.022, -0.033),
        (-0.020, 0.010, -0.033),
        (-0.019, -0.009, -0.091),
        (-0.018, 0.005, -0.039),
        (-0.017, -0.021, -0.040),
        (-0.017, -0.021, -0.030),
        (-0.017, 0.009, -0.040),
        (-0.017, 0.009, -0.030),
        (-0.016, -0.015, -0.056),
        (-0.016, -0.014, -0.054),
        (-0.016, 0.002, -0.058),
        (-0.016, 0.002, -0.054),
        (-0.015, -0.012, -0.091),
        (-0.014, -0.013, -0.087),
        (-0.014, 0.000, -0.090),
        (-0.014, 0.001, -0.083),
        (-0.013, -0.001, -0.090),
        (-0.012, -0.006, -0.099),
        (0.030, -0.015, -0.037),
        (0.030, -0.015, -0.007),
        (0.030, 0.015, -0.037),
        (0.030, 0.015, -0.007),
    ]
)
# FK link frame whose origin each capsule end names: the shoulder_lift axis, the elbow, wrist_flex axis, gripper_link.
LINK_ORIGINS = {
    "shoulder": "upper_arm_link",
    "elbow": "lower_arm_link",
    "wrist": "wrist_link",
    "gripper": "gripper_link",
}


@dataclass(frozen=True)
class LinkBox:
    """A collision box of an arm link: centre and orientation in the URDF link frame, half sizes (m); a contact-only
    box rejects a motion only when it would touch an obstacle and never slows one."""

    part: str
    link: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]  # MuJoCo (w, x, y, z), not necessarily normalised (as in the MJCF)
    half_size: tuple[float, float, float]
    contact_only: bool = False


# Collision boxes of the upper arm, forearm and wrist in sim/grasp_sim/assets/so101/so101.xml (its upper_arm,
# lower_arm and wrist bodies are the URDF links' frames), in MJCF order. The body check covers them in addition to the
# link capsules: the boxes sit off the line between the link origins (the forearm box reaches 4.8 cm from it), where
# the 2 cm capsules alone let plans through that the sim arm ran into the lidar dome. The upper arm boxes are
# contact-only: with the first dome model (radius 74 mm at the laser_frame position) their shoulder end stood 7 mm from
# the dome at the folded home pose and the slab passes 2..7 mm over the dome top edge at most forward reaches, so held
# to the margins they would reject home and slow nearly every motion; the sim upper arm touched the dome only in the
# low stair reaches, which the contact check rejects.
LINK_COLLISION_BOXES = (
    LinkBox(
        "upper arm",
        "upper_arm_link",
        (-0.06, 0.0, 0.02),
        (0.0, -1.0, 1.0, 0.0),
        (0.01, 0.07, 0.03),
        contact_only=True,
    ),
    LinkBox(
        "upper arm",
        "upper_arm_link",
        (-0.12, -0.014, 0.0182),
        (0.0, 1.0, 0.0, 0.0),
        (0.01, 0.02, 0.015),
        contact_only=True,
    ),
    LinkBox("forearm", "lower_arm_link", (-0.05, 0.0, 0.0182), (0.0, 1.0, 0.0, 0.0), (0.07, 0.01, 0.03)),
    LinkBox("forearm", "lower_arm_link", (-0.125, 0.005, 0.018), (0.0, -1.0, 0.0, 0.0), (0.023, 0.013, 0.018)),
    LinkBox("wrist link", "wrist_link", (0.0, -0.0424, 0.0256), (1.0, 1.0, 1.0, -1.0), (0.029, 0.015, 0.018)),
    LinkBox("wrist link", "wrist_link", (0.0, -0.02, 0.0191), (1.0, -1.0, -1.0, -1.0), (0.027, 0.01, 0.03)),
)
# Spacing of the sample points on the box faces (m). Between the points a sharp obstacle edge can come up to
# spacing / sqrt(2) = 1.8 mm closer than the points say, within the 2 mm body hard margin (a test pins this invariant).
BOX_POINT_SPACING_M = 0.0025


def quat_matrix(quat: tuple[float, float, float, float]) -> np.ndarray:
    """Rotation matrix of a (w, x, y, z) quaternion.

    Args:
        quat (tuple[float, float, float, float]): Quaternion (normalised here).

    Returns:
        np.ndarray: (3, 3) rotation.
    """
    w, x, y, z = np.asarray(quat, dtype=np.float64) / np.linalg.norm(quat)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def box_surface_points(box: LinkBox, spacing: float = BOX_POINT_SPACING_M) -> np.ndarray:
    """Points on every face of the box (a grid at most spacing apart, corners and edges included), link frame.

    Args:
        box (LinkBox): Collision box.
        spacing (float): Largest grid step (m).

    Returns:
        np.ndarray: (K, 3) points in the link frame.
    """
    half = np.asarray(box.half_size, dtype=np.float64)
    axes = [np.linspace(-h, h, max(2, math.ceil(2 * h / spacing) + 1)) for h in half]
    grid = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
    on_face = (np.abs(np.abs(grid) - half) < 1e-12).any(axis=1)
    return grid[on_face] @ quat_matrix(box.quat).T + np.asarray(box.pos, dtype=np.float64)
