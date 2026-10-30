"""Camera-frame 3D points in base_link and the arm base frame: the one transform helper the camera tools share.

A point in a D435 optical frame converts with the pose of that frame in base_link (the head chain at the pan/tilt of
the image stamp) and the pose of base_link in the arm base frame (``arm.base_in_base_link``). The calibration library
(``ros2_common.head_rgbd_calibration.chain``: ``UrdfChain`` plus ``arm_in_robot_base``) composes the same two poses
when the head-from-arm fit solves the chain; ``tests/test_camera_arm_chain.py`` pins that they agree.

The reports name the frame of every position and carry the calibration residual the applied head chain was measured
with, and whether it is within the grasp gate (``camera_arm.max_residual_mm``).
"""

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .camera_scene import transform_point
from .head import CalibrationInfo, HeadPoseAtStamp

COLOR_OPTICAL_FRAME = "d435_color_optical_frame"
DEPTH_OPTICAL_FRAME = "d435_depth_optical_frame"
CAMERA_FRAMES = (COLOR_OPTICAL_FRAME, DEPTH_OPTICAL_FRAME)
ARM_FRAME = "arm"
BASE_FRAME = "base_link"
MAP_FRAME = "map"
MM_PER_M = 1000.0


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


def position(frame: str, point: np.ndarray | None) -> dict[str, Any] | None:
    """A position block naming its frame, millimetre rounded.

    Args:
        frame (str): Frame name.
        point (np.ndarray | None): Point (3,) in that frame.

    Returns:
        dict[str, Any] | None: {frame, x, y, z} in metres, None for a point that is not known.
    """
    if point is None:
        return None
    return {
        "frame": frame,
        "x": round(float(point[0]), 3),
        "y": round(float(point[1]), 3),
        "z": round(float(point[2]), 3),
    }


def gate_state(info: CalibrationInfo, gate_mm: float) -> bool | None:
    """Whether the applied calibration's arm-frame residual is within the grasp gate.

    Args:
        info (CalibrationInfo): The applied head calibration.
        gate_mm (float): camera_arm.max_residual_mm.

    Returns:
        bool | None: True/False for a known residual; None when the applied chain carries none.
    """
    residual = info.residual_mm
    return None if residual is None else residual <= gate_mm


def gate_refusal(info: CalibrationInfo, gate_mm: float) -> str | None:
    """Why a camera-derived grasp position is not allowed, None when it is.

    Args:
        info (CalibrationInfo): The applied head calibration.
        gate_mm (float): camera_arm.max_residual_mm.

    Returns:
        str | None: Refusal text naming the residual and the gate, None when the residual is within the gate.
    """
    state = gate_state(info, gate_mm)
    if state is True:
        return None
    if state is None:
        return (
            f"no arm-frame residual to check against the {gate_mm:g} mm gate ({info.describe()}); "
            "run the head-from-arm calibration on the web UI Calibrate page and Apply it"
        )
    return (
        f"the applied head calibration's camera-to-arm residual {info.residual_mm:.1f} mm is above the {gate_mm:g} mm "
        "gate; improve the head-from-arm calibration on the web UI Calibrate page (more arm poses, a better board "
        "mount) and Apply it"
    )


def calibration_block(info: CalibrationInfo, gate_mm: float) -> dict[str, Any]:
    """The calibration part of a camera-to-arm report.

    Args:
        info (CalibrationInfo): The applied head calibration.
        gate_mm (float): camera_arm.max_residual_mm.

    Returns:
        dict[str, Any]: applied, source, session, result, applied_at, residual_mm (null when unknown),
            holdout_rms_px, gate_mm, within_gate (null when the residual is unknown) and description.
    """
    residual = info.residual
    return {
        "applied": info.applied,
        "source": info.source,
        "session": info.session,
        "result": info.result,
        "applied_at": info.applied_at,
        "residual_mm": None if residual is None else round(residual.holdout_board_mm, 2),
        "holdout_rms_px": None if residual is None else round(residual.holdout_rms_px, 2),
        "gate_mm": gate_mm,
        "within_gate": gate_state(info, gate_mm),
        "description": info.describe(),
    }


def head_pose_block(pose: HeadPoseAtStamp) -> dict[str, Any]:
    """The head pose a camera frame was placed with.

    Args:
        pose (HeadPoseAtStamp): Pan/tilt at the frame stamp with the surrounding motion.

    Returns:
        dict[str, Any]: pan_deg, tilt_deg, stamp_s, nearest_sample_gap_s and window_motion_deg.
    """
    return {
        "pan_deg": round(math.degrees(pose.pan), 4),
        "tilt_deg": round(math.degrees(pose.tilt), 4),
        "stamp_s": pose.stamp_s,
        "nearest_sample_gap_s": round(pose.nearest_gap_s, 4),
        "window_motion_deg": round(math.degrees(pose.window_motion_rad), 4),
    }


def residual_notes(info: CalibrationInfo, gate_mm: float, arm_known: bool) -> list[str]:
    """Notes on what the result may be used for, from the calibration residual and the arm mount.

    Args:
        info (CalibrationInfo): The applied head calibration.
        gate_mm (float): camera_arm.max_residual_mm.
        arm_known (bool): Whether the arm frame point could be computed.

    Returns:
        list[str]: Notes (empty when the point is grasp ready).
    """
    notes: list[str] = []
    if not arm_known:
        notes.append("arm is null: arm.base_in_base_link is not configured, so the arm frame is unknown")
    refusal = gate_refusal(info, gate_mm)
    if refusal is not None:
        notes.append(f"not grasp ready: {refusal}")
    return notes
