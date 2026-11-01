"""Gripper jaw model of the SO-101 arm: the moving jaw opening about the gripper pivot, the jaw points (fixed jaw tip,
tool point, moving jaw tip) and the arm mount yaw rotation. Pure geometry (numpy), no ROS, no OpenCV."""

import math

import numpy as np

from .ik import ArmKinematics
from .jaw_profile import MOVING_JAW_INNER_PROFILE
from .settings import ArmBaseOffset

# Moving jaw pivot and rotation axis in gripper_link (URDF joint 'gripper': origin xyz 0.0202 0.0188 -0.0234, rpy
# 1.5708 0 0, axis z -> the gripper_link -y axis). The rotation sign is resolved against the jaw opening direction.
JAW_PIVOT_IN_GRIPPER_LINK = (0.0202, 0.0188, -0.0234)
JAW_AXIS_IN_GRIPPER_LINK = (0.0, -1.0, 0.0)
JAW_SIGN_PROBE_RAD = 0.1
# Upper end (rad above closed) of the bisection that maps a jaw gap to a gripper angle.
JAW_SEARCH_SPAN_RAD = 2.0
JAW_BISECTION_STEPS = 60
GRIPPER_LINK = "gripper_link"
TOOL_FRAME_LINK = "gripper_frame_link"


def yaw_matrix(yaw: float) -> np.ndarray:
    """Rotation about z.

    Args:
        yaw (float): Angle (rad).

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotate(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation of vectors about a unit axis.

    Args:
        vector (np.ndarray): 3-vector, or (K, 3) vectors rotated at once.
        axis (np.ndarray): Unit rotation axis.
        angle (float): Angle (rad).

    Returns:
        np.ndarray: Rotated vector(s), the shape of vector.
    """
    c, s = math.cos(angle), math.sin(angle)
    along = (np.asarray(vector) @ axis)[..., None]
    return vector * c + np.cross(axis, vector) * s + axis * along * (1.0 - c)


class JawModel:
    """Approximate moving-jaw geometry: the closed moving jaw tip sits on the tool point (fixed jaw inner face) and
    opens by rotating about the URDF gripper joint pivot. Gives the jaw gap (m, across the jaws) for a gripper angle
    and back, the moving jaw tip position for the floor check, and the narrower gap deeper into the jaws
    (inner_gap, from the moving jaw's inner silhouette MOVING_JAW_INNER_PROFILE)."""

    def __init__(self, kin: ArmKinematics, jaw_open_axis: tuple[float, float, float], closed_rad: float) -> None:
        """Derive the closed tip and the opening direction in gripper_link from the URDF chain.

        Args:
            kin (ArmKinematics): Kinematics (tool offset = fixed jaw inner face).
            jaw_open_axis (tuple[float, float, float]): Unit opening direction in gripper_frame_link.
            closed_rad (float): Gripper joint position of a closed jaw (rad).
        """
        frames = kin.link_frames({})
        t_gl_tool = np.linalg.inv(frames[GRIPPER_LINK]) @ frames[TOOL_FRAME_LINK]
        self.closed_rad = closed_rad
        self.closed_tip = t_gl_tool[:3, :3] @ kin.tool_offset + t_gl_tool[:3, 3]
        self.open_axis = t_gl_tool[:3, :3] @ np.array(jaw_open_axis, dtype=np.float64)
        self.pivot = np.array(JAW_PIVOT_IN_GRIPPER_LINK)
        self.axis = np.array(JAW_AXIS_IN_GRIPPER_LINK)
        probe = rotate(self.closed_tip - self.pivot, self.axis, JAW_SIGN_PROBE_RAD) + self.pivot
        self.sign = 1.0 if float((probe - self.closed_tip) @ self.open_axis) > 0.0 else -1.0
        # into the jaws = against the approach (tool frame z) axis
        self.depth_axis = -(t_gl_tool[:3, :3] @ np.array([0.0, 0.0, 1.0]))
        self.profile = np.array(
            [
                self.closed_tip + offset * self.open_axis + depth * self.depth_axis
                for offset, depth in MOVING_JAW_INNER_PROFILE
            ]
        )

    def tip_in_gripper_link(self, angle: float) -> np.ndarray:
        """Moving jaw tip in gripper_link for a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            np.ndarray: Tip position (m).
        """
        turned = rotate(self.closed_tip - self.pivot, self.axis, self.sign * (angle - self.closed_rad))
        return turned + self.pivot

    def gap(self, angle: float) -> float:
        """Jaw opening across the jaws (m) for a gripper angle.

        Args:
            angle (float): Gripper joint position (rad).

        Returns:
            float: Distance of the moving jaw tip from the fixed jaw inner face along the opening direction.
        """
        return float((self.tip_in_gripper_link(angle) - self.closed_tip) @ self.open_axis)

    def inner_gap(self, angle: float, depth: float) -> float:
        """Narrowest jaw opening from the tips to a depth into the jaws, for a gripper angle.

        The moving jaw's inner silhouette (closed) is rotated about the pivot; its points from beyond the tips up to
        depth (along the jaws, from the closed tips toward the palm) give the opening, measured from the fixed jaw
        inner face along the opening direction. The open jaw is tilted, so deeper means narrower.

        Args:
            angle (float): Gripper joint position (rad).
            depth (float): How far the object reaches into the jaws from the tips (m, >= 0).

        Returns:
            float: Opening (m); inf when the jaw has no material up to that depth.
        """
        turned = rotate(self.profile - self.pivot, self.axis, self.sign * (angle - self.closed_rad)) + self.pivot
        depths = (turned - self.closed_tip) @ self.depth_axis
        offsets = (turned - self.closed_tip) @ self.open_axis
        inside = depths <= depth
        best = float(offsets[inside].min()) if inside.any() else math.inf
        for k in range(len(depths) - 1):  # the silhouette edge crossing the depth limit
            a, b = depths[k], depths[k + 1]
            if (a - depth) * (b - depth) < 0.0:
                best = min(best, float(offsets[k] + (depth - a) / (b - a) * (offsets[k + 1] - offsets[k])))
        return best

    def angle_for_inner_gap(self, gap: float, depth: float) -> float:
        """Gripper angle whose narrowest opening up to depth is gap (bisection; it grows with the angle).

        Args:
            gap (float): Wanted opening (m), >= 0.
            depth (float): Object depth into the jaws (m).

        Returns:
            float: Gripper joint position (rad); the widest-search angle when the gap is not reachable.
        """
        lo, hi = self.closed_rad, self.closed_rad + JAW_SEARCH_SPAN_RAD
        for _ in range(JAW_BISECTION_STEPS):
            mid = (lo + hi) / 2.0
            if self.inner_gap(mid, depth) < gap:
                lo = mid
            else:
                hi = mid
        return hi

    def angle_for_gap(self, gap: float) -> float:
        """Gripper angle giving a jaw gap (bisection; the gap grows monotonically over the useful range).

        Args:
            gap (float): Wanted opening (m), >= 0.

        Returns:
            float: Gripper joint position (rad); the widest-gap angle when the gap is not reachable.
        """
        lo, hi = self.closed_rad, self.closed_rad + JAW_SEARCH_SPAN_RAD
        for _ in range(JAW_BISECTION_STEPS):
            mid = (lo + hi) / 2.0
            if self.gap(mid) < gap:
                lo = mid
            else:
                hi = mid
        return hi

    def moving_tip(self, t_gripper_link: np.ndarray, angle: float) -> np.ndarray:
        """Moving jaw tip in the arm base frame.

        Args:
            t_gripper_link (np.ndarray): 4x4 pose of gripper_link in the arm base frame.
            angle (float): Gripper joint position (rad).

        Returns:
            np.ndarray: Tip position (m).
        """
        return t_gripper_link[:3, :3] @ self.tip_in_gripper_link(angle) + t_gripper_link[:3, 3]


def jaw_points(
    kin: ArmKinematics, jaw: JawModel, frames: dict[str, np.ndarray], gripper_angle: float
) -> dict[str, np.ndarray]:
    """Jaw points in the arm base frame: the one definition shared by self_collision and the jaw crop.

    Args:
        kin (ArmKinematics): Arm kinematics on the URDF (tool_offset).
        jaw (JawModel): Moving jaw geometry.
        frames (dict[str, np.ndarray]): kin.link_frames(joints).
        gripper_angle (float): Measured gripper angle (rad); it moves the moving jaw tip.

    Returns:
        dict[str, np.ndarray]: 'jaw_tip', 'tool_point' and 'moving_jaw_tip' -> (3,) positions.
    """
    tool = frames[TOOL_FRAME_LINK]
    return {
        "jaw_tip": tool[:3, 3],
        "tool_point": tool[:3, :3] @ kin.tool_offset + tool[:3, 3],
        "moving_jaw_tip": jaw.moving_tip(frames[GRIPPER_LINK], gripper_angle),
    }


def tool_point_in_base(
    kin: ArmKinematics, jaw: JawModel, joints: dict[str, float], offset: ArmBaseOffset
) -> np.ndarray:
    """FK tool point in base_link (the point the head aims the head camera at for the grasp image).

    Args:
        kin (ArmKinematics): Arm kinematics on the URDF.
        jaw (JawModel): Moving jaw geometry.
        joints (dict[str, float]): Measured joint name -> rad.
        offset (ArmBaseOffset): arm.base_in_base_link.

    Returns:
        np.ndarray: (3,) tool point in base_link (m).
    """
    point = jaw_points(kin, jaw, kin.link_frames(joints), joints.get("gripper", jaw.closed_rad))["tool_point"]
    return yaw_matrix(offset.yaw) @ point + np.array([offset.x, offset.y, offset.z])
