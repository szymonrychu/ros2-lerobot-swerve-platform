"""Jaw crop: the gripper camera region between the jaws, from the arm model and the camera calibration.

Pure geometry and image cropping, no camera I/O: the caller supplies the frame size and the JPEG bytes.
"""

import math

import cv2
import numpy as np
from ros2_common.camera_geometry import optical_from_mount

from .camera_overlay import MAX_PROJECTED_PX, MIN_DEPTH, project_points
from .camera_scene import CameraSetup
from .config import GraspCheckSettings
from .floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, JawModel
from .ik import ArmKinematics

JPEG_QUALITY = 85


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


def jaw_crop(
    setup: CameraSetup | None,
    kin: ArmKinematics,
    jaw: JawModel,
    joints: dict[str, float],
    frame_w: int,
    frame_h: int,
    s: GraspCheckSettings,
) -> tuple[int, int, int, int] | None:
    """Pixel box of the region between the jaws in the gripper camera frame.

    Projects the jaw points plus the same points moved s.crop_depth_m into the jaws (toward gripper_link), takes
    their bounding box, adds s.crop_margin_px on every side and clamps it to the frame.

    Args:
        setup (CameraSetup | None): Gripper camera calibration; None or without a mount means uncalibrated.
        kin (ArmKinematics): Arm kinematics on the URDF.
        jaw (JawModel): Moving jaw geometry.
        joints (dict[str, float]): Measured joint name -> rad (including the gripper angle).
        frame_w (int): Frame width in pixels (the size the intrinsics refer to, or the frame to crop).
        frame_h (int): Frame height in pixels.
        s (GraspCheckSettings): crop_depth_m, crop_margin_px and crop_min_px.

    Returns:
        tuple[int, int, int, int] | None: (x0, y0, x1, y1), or None when the camera is uncalibrated, no point lies in
            front of the camera, or the box is smaller than s.crop_min_px (callers then use the full frame).
    """
    if setup is None or setup.mount is None:
        return None
    t_gripper = kin.link_frame(joints, setup.parent_frame)
    t_optical = optical_from_mount(t_gripper @ setup.mount.to_matrix())
    frames = kin.link_frames(joints)
    front = np.array(list(jaw_points(kin, jaw, frames, joints.get("gripper", jaw.closed_rad)).values()))
    into_jaws = frames[GRIPPER_LINK][:3, :3] @ jaw.depth_axis
    pts = np.vstack([front, front + s.crop_depth_m * into_jaws])
    uv, depth = project_points(setup.intrinsics, t_optical, pts)
    visible = (depth > MIN_DEPTH) & np.all(np.isfinite(uv), axis=1) & np.all(np.abs(uv) < MAX_PROJECTED_PX, axis=1)
    if not visible.any():
        return None
    low, high = uv[visible].min(axis=0), uv[visible].max(axis=0)
    x0 = max(0, math.floor(low[0] - s.crop_margin_px))
    y0 = max(0, math.floor(low[1] - s.crop_margin_px))
    x1 = min(frame_w, math.ceil(high[0] + s.crop_margin_px))
    y1 = min(frame_h, math.ceil(high[1] + s.crop_margin_px))
    if min(x1 - x0, y1 - y0) < s.crop_min_px:
        return None
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
