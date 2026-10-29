"""Camera-frame 3D points in base_link and the arm base frame: the one transform helper the camera tools share.

A point in a D435 optical frame converts with the pose of that frame in base_link (the head chain at the pan/tilt of
the image stamp) and the pose of base_link in the arm base frame (``arm.base_in_base_link``). The calibration library
(``ros2_common.head_rgbd_calibration.chain``: ``UrdfChain`` plus ``arm_in_robot_base``) composes the same two poses
when the head-from-arm fit solves the chain; ``tests/test_camera_arm_chain.py`` pins that they agree.
"""

from dataclasses import dataclass

import numpy as np

from .camera_scene import transform_point

COLOR_OPTICAL_FRAME = "d435_color_optical_frame"
DEPTH_OPTICAL_FRAME = "d435_depth_optical_frame"
CAMERA_FRAMES = (COLOR_OPTICAL_FRAME, DEPTH_OPTICAL_FRAME)
ARM_FRAME = "arm"
BASE_FRAME = "base_link"


@dataclass(frozen=True)
class PointFrames:
    """One point expressed in every frame the grasp tools use.

    Attributes:
        camera (np.ndarray): The point in its camera optical frame (3,).
        base_link (np.ndarray): The point in base_link (3,).
        arm (np.ndarray | None): The point in the arm base frame (3,); None while arm.base_in_base_link is not set.
    """

    camera: np.ndarray
    base_link: np.ndarray
    arm: np.ndarray | None


def frames_of_point(
    t_base_optical: np.ndarray, t_arm_base: np.ndarray | None, point_optical: np.ndarray
) -> PointFrames:
    """A camera-frame point in base_link and the arm base frame.

    Args:
        t_base_optical (np.ndarray): 4x4 pose of the camera optical frame in base_link (head chain at the image stamp).
        t_arm_base (np.ndarray | None): 4x4 pose of base_link in the arm base frame; None when the arm mount is unknown.
        point_optical (np.ndarray): Point (3,) in the optical frame (x right, y down, z forward).

    Returns:
        PointFrames: The point in the camera frame, base_link and (when known) the arm base frame.
    """
    point = np.asarray(point_optical, dtype=np.float64)
    in_base = transform_point(t_base_optical, point)
    in_arm = None if t_arm_base is None else transform_point(t_arm_base, in_base)
    return PointFrames(point, in_base, in_arm)
