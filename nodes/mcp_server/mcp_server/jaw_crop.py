"""Jaw crop: the region between the jaws in the head D435 colour image, from the arm model and the head
camera scene.

The head camera hangs on the pan/tilt head: its pose comes from the head chain at the measured head angles
(camera_scene.build_scene), and the jaw points convert into it from the arm base frame through arm.base_in_base_link.
Pure geometry and image cropping, no camera I/O: the caller supplies the scene and the JPEG bytes.
"""

import math

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.arm_pose_check import D435_MIN_Z_M

from .camera_overlay import MAX_PROJECTED_PX, MIN_DEPTH, project_points
from .camera_scene import Scene, offset_matrix, transform_point
from .config import ArmBaseOffset, GraspCheckSettings
from .floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, JawModel
from .ik import ArmKinematics

JPEG_QUALITY = 85
NO_ARM_MOUNT = "arm.base_in_base_link is not configured: the jaws cannot be placed in the head camera"
OUTSIDE_IMAGE = "the jaw is outside the head camera image"


class JawCropError(ValueError):
    """The jaw region cannot be cropped from the head image; the message is the reason (the full frame is used)."""


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
    return transform_point(offset_matrix(offset), point)


def jaw_crop(
    scene: Scene, kin: ArmKinematics, jaw: JawModel, joints: dict[str, float], s: GraspCheckSettings
) -> tuple[int, int, int, int]:
    """Pixel box of the region between the jaws in the head camera image of the scene.

    Projects the jaw points plus the same points moved s.crop_depth_m into the jaws (toward gripper_link) through the
    scene (head chain pose at the measured head angles), takes their bounding box, adds s.crop_margin_px on every side
    and clamps it to the image.

    Args:
        scene (Scene): Head camera scene (base_link reference, intrinsics at the calibrated image size).
        kin (ArmKinematics): Arm kinematics on the URDF.
        jaw (JawModel): Moving jaw geometry.
        joints (dict[str, float]): Measured joint name -> rad (including the gripper angle).
        s (GraspCheckSettings): crop_depth_m, crop_margin_px and crop_min_px.

    Returns:
        tuple[int, int, int, int]: (x0, y0, x1, y1) in pixels of the scene's image size.

    Raises:
        JawCropError: When the arm mount is unknown, part of the region lies nearer the camera than the D435 min-Z
            (D435_MIN_Z_M of depth), the region is outside the image, or the box is smaller than s.crop_min_px.
    """
    frames = kin.link_frames(joints)
    front = np.array(list(jaw_points(kin, jaw, frames, joints.get("gripper", jaw.closed_rad)).values()))
    into_jaws = frames[GRIPPER_LINK][:3, :3] @ jaw.depth_axis
    pts_arm = np.vstack([front, front + s.crop_depth_m * into_jaws])
    if scene.t_arm_ref is None:
        raise JawCropError(NO_ARM_MOUNT)
    pts = np.array([scene.arm_to_ref(p) for p in pts_arm])
    uv, depth = project_points(scene.intr, scene.t_ref_optical, pts)
    if depth.max() <= MIN_DEPTH:
        raise JawCropError(OUTSIDE_IMAGE)
    nearest = float(depth.min())
    if nearest < D435_MIN_Z_M:
        raise JawCropError(
            f"the jaw is {max(nearest, 0.0):.3f} m deep from the head camera, inside the D435 min-Z "
            f"({D435_MIN_Z_M:g} m)"
        )
    bounded = np.all(np.isfinite(uv), axis=1) & np.all(np.abs(uv) < MAX_PROJECTED_PX, axis=1)
    width, height = scene.intr.width, scene.intr.height
    if not bounded.any():
        raise JawCropError(OUTSIDE_IMAGE)
    low, high = uv[bounded].min(axis=0), uv[bounded].max(axis=0)
    if high[0] < 0 or high[1] < 0 or low[0] >= width or low[1] >= height:
        raise JawCropError(OUTSIDE_IMAGE)
    x0 = max(0, math.floor(low[0] - s.crop_margin_px))
    y0 = max(0, math.floor(low[1] - s.crop_margin_px))
    x1 = min(width, math.ceil(high[0] + s.crop_margin_px))
    y1 = min(height, math.ceil(high[1] + s.crop_margin_px))
    if min(x1 - x0, y1 - y0) < s.crop_min_px:
        raise JawCropError(
            f"the jaw crop ({x1 - x0}x{y1 - y0} px) is smaller than grasp_check.crop_min_px {s.crop_min_px}"
        )
    return x0, y0, x1, y1


def crop_jpeg(jpeg: bytes, box: tuple[int, int, int, int], max_px: int) -> bytes:
    """Crop a JPEG to a box and shrink it so the longest side is at most max_px.

    Args:
        jpeg (bytes): Source JPEG.
        box (tuple[int, int, int, int]): (x0, y0, x1, y1) in source pixels.
        max_px (int): Longest side of the result (a smaller crop is not upscaled).

    Returns:
        bytes: The cropped JPEG.

    Raises:
        ValueError: When the bytes are not a decodable image or the box is empty.
    """
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("jaw crop: the frame is not a decodable image")
    x0, y0, x1, y1 = box
    crop = img[max(0, y0) : y1, max(0, x0) : x1]
    if crop.size == 0:
        raise ValueError(f"jaw crop: box {box} is empty in a {img.shape[1]}x{img.shape[0]} frame")
    longest = max(crop.shape[:2])
    if longest > max_px:
        scale = max_px / longest
        size = (max(1, round(crop.shape[1] * scale)), max(1, round(crop.shape[0] * scale)))
        crop = cv2.resize(crop, size, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError("jaw crop: JPEG encoding failed")
    return buf.tobytes()
