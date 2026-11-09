"""Pure view maths for the board renders: the view directions, the camera and the grip fit scene (no pyvista)."""

from typing import Any

import numpy as np
import trimesh
from ros2_common.head_rgbd_calibration.chain import pose_from_xyz_rpy

from scripts.markers.handle import board_in_mesh
from scripts.markers.jaw_fit import Gripper, moving_jaw

MM_PER_M = 1000.0
GRIP_VIEW = "grip_fit"
VIEW_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
    "iso_front": (1.0, -1.0, 1.0),
    "iso_back": (-1.0, 1.0, 1.0),
}
DISTANCE_PER_DIAGONAL = 2.0  # camera distance as a multiple of the bounds diagonal
UP_PARALLEL_TOL = 0.99  # |direction . z| above this uses +y as the view up
ARM_PATTERN_FACE = "top"  # the ARM board prints back down (pattern up, user m540)
COLOUR_BASE = "white"
COLOUR_MODULES = "black"
COLOUR_FIXED_JAW = "lightsteelblue"
COLOUR_MOVING_JAW = "orange"


def camera_for(
    direction: tuple[float, float, float], bounds: np.ndarray
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """A camera looking at the bounds centre from the given direction.

    Args:
        direction (tuple[float, float, float]): Vector from the centre toward the camera (need not be unit).
        bounds (np.ndarray): 2x3 min and max corner of the scene.

    Returns:
        tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]: (position, focal_point, view_up), each
        three floats. View up is +z, or +y when the camera looks along z.
    """
    centre = (bounds[0] + bounds[1]) / 2
    unit = np.asarray(direction, dtype=float)
    unit /= np.linalg.norm(unit)
    distance = DISTANCE_PER_DIAGONAL * float(np.linalg.norm(bounds[1] - bounds[0]))
    up = (0.0, 1.0, 0.0) if abs(unit[2]) > UP_PARALLEL_TOL else (0.0, 0.0, 1.0)
    return tuple(float(v) for v in centre + unit * distance), tuple(float(v) for v in centre), up


def grip_scene(
    gripper: Gripper, meta: dict[str, Any], base: trimesh.Trimesh, modules: trimesh.Trimesh
) -> list[tuple[trimesh.Trimesh, str]]:
    """The board seated in the gripper, everything in gripper_link (mm).

    Args:
        gripper (Gripper): Loaded gripper.
        meta (dict[str, Any]): Board metadata JSON (plate_mm, board, handle_in_board, grip).
        base (trimesh.Trimesh): White base body in print coordinates (mm).
        modules (trimesh.Trimesh): Black module body in print coordinates (mm).

    Returns:
        list[tuple[trimesh.Trimesh, str]]: Posed copies named "base", "modules", "fixed_jaw" and "moving_jaw" (at
        grip.grip_rad), in that order.
    """
    plate_w, plate_h = meta["plate_mm"]
    board = meta["board"]
    margin = (plate_w - board["squares_x"] * board["square_mm"]) / 2
    hib = meta["handle_in_board"]
    board_in_gripper = np.linalg.inv(pose_from_xyz_rpy(hib["gripper_link_xyz_m"], hib["gripper_link_rpy_rad"]))
    board_in_gripper[:3, 3] *= MM_PER_M
    pose = board_in_gripper @ np.linalg.inv(
        board_in_mesh((plate_w, plate_h), margin, ARM_PATTERN_FACE, board["thickness_mm"])
    )
    return [
        (base.copy().apply_transform(pose), "base"),
        (modules.copy().apply_transform(pose), "modules"),
        (gripper.fixed.copy(), "fixed_jaw"),
        (moving_jaw(gripper, meta["grip"]["grip_rad"]), "moving_jaw"),
    ]
