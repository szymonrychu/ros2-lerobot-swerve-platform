"""The head RGB-D calibration fit from per-pose detections: no bag, no files, no ssh.

PoseView carries what one still pose yielded (RGB detection, ToF detection, depth frame);
calibrate_detections fits RGB intrinsics, ToF intrinsics, the stereo extrinsic and the plane depth bias.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.charuco import Detection, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.extrinsics import (
    ExtrinsicResult,
    PlaneCheck,
    plane_depth_bias,
    stereo_calibrate,
)
from ros2_common.head_rgbd_calibration.frames import CameraInfoData, normalise_amplitude
from ros2_common.head_rgbd_calibration.intrinsics import (
    IntrinsicsResult,
    calibrate,
    calibrate_tof,
    compare_sdk,
)


@dataclass(frozen=True)
class CalibrationRun:
    """Everything calibration computes before any file is written."""

    rgb: IntrinsicsResult
    tof: IntrinsicsResult
    extrinsic: ExtrinsicResult
    plane: PlaneCheck
    sdk_diff_pct: dict[str, float]
    pair_count: int


@dataclass(frozen=True)
class PoseView:
    """One still pose: the board detections in each camera (None when not found) and the ToF depth (uint16 mm)."""

    rgb: Detection | None
    tof: Detection | None
    depth: np.ndarray | None


def detect_tof(amplitude: np.ndarray, board: cv2.aruco.CharucoBoard, sel: SelectionConfig) -> Detection | None:
    """Detect the board in a ToF amplitude frame: normalised by sel.amplitude_percentiles, enlarged sel.tof_upscale.

    Args:
        amplitude: ToF amplitude image (uint16).
        board: The ChArUco board.
        sel: Selection settings (percentiles and upscale).

    Returns:
        The Detection in amplitude pixels, or None (empty frame or too few corners).
    """
    image = normalise_amplitude(amplitude, sel.amplitude_percentiles)
    return None if image is None else detect(image, board, sel.tof_upscale)


def calibrate_detections(
    views: list[PoseView],
    rgb_size: tuple[int, int],
    tof_size: tuple[int, int],
    tof_info: CameraInfoData | None,
    board_cfg: BoardConfig,
    limits: LimitsConfig,
) -> CalibrationRun:
    """Fit RGB intrinsics, ToF intrinsics and the extrinsic from the detections of the still poses.

    Args:
        views: One PoseView per still pose.
        rgb_size: RGB image size (width, height).
        tof_size: ToF image size (width, height).
        tof_info: The SDK ToF camera_info to compare the fitted intrinsics with.
        board_cfg: The ChArUco board.
        limits: Acceptance limits.

    Returns:
        The CalibrationRun; pair_count is len(views).

    Raises:
        ValueError: no ToF camera_info, too few views or pairs (the message names the count), or no plane measurement.
    """
    if tof_info is None:
        raise ValueError("the bag has no ToF camera_info; cannot compare with the SDK")
    board = make_board(board_cfg)
    rgb = calibrate([v.rgb for v in views if v.rgb is not None], board, rgb_size, limits)
    tof = calibrate_tof([v.tof for v in views if v.tof is not None], board, tof_size, limits)
    both = [(v.rgb, v.tof, v.depth) for v in views if v.rgb is not None and v.tof is not None]
    extrinsic = stereo_calibrate([(r, t) for r, t, _ in both], board, rgb, tof, limits)
    with_depth = [(r, t, depth) for r, t, depth in both if depth is not None]
    plane = plane_depth_bias(with_depth, board, rgb, tof, extrinsic, limits)
    return CalibrationRun(rgb, tof, extrinsic, plane, compare_sdk(tof, tof_info), len(views))


def failed_limits(run: CalibrationRun, limits: LimitsConfig) -> list[str]:
    """Descriptions of every exceeded limit, each naming its limits.* key."""
    checks = [
        ("limits.rgb_max_rms_px", "RGB RMS", run.rgb.rms, limits.rgb_max_rms_px),
        ("limits.tof_max_rms_px", "ToF RMS", run.tof.rms, limits.tof_max_rms_px),
        ("limits.stereo_max_rms_px", "stereo RMS", run.extrinsic.stereo_rms, limits.stereo_max_rms_px),
    ]
    failures = [
        f"{key}: {label} {value:.3f} px exceeds {limit} px" for key, label, value, limit in checks if value > limit
    ]
    bias = run.plane.median_mm
    if abs(bias) > limits.plane_max_bias_mm:
        failures.append(
            f"limits.plane_max_bias_mm: plane depth bias {bias:.1f} mm exceeds {limits.plane_max_bias_mm} mm"
        )
    return failures


def report_markdown(run: CalibrationRun, limits: LimitsConfig, session: str, lens: float, date: str) -> str:
    """The calibration run report as markdown."""
    lines = [
        f"# Head RGB-D calibration {session}",
        "",
        f"Date: {date}",
        f"Head camera lens position: {lens:g} dioptres (changing lens_position invalidates the RGB calibration)",
        f"Still pairs: {run.pair_count}",
        "",
        "| Item | Value | Limit |",
        "|---|---|---|",
        f"| RGB intrinsic RMS | {run.rgb.rms:.3f} px | {limits.rgb_max_rms_px} px |",
        f"| RGB views | {run.rgb.view_count} | >= {limits.min_views} |",
        f"| ToF intrinsic RMS | {run.tof.rms:.3f} px | {limits.tof_max_rms_px} px |",
        f"| ToF views | {run.tof.view_count} | >= {limits.min_views} |",
        f"| ToF zero-distortion RMS | {run.tof.zero_distortion_rms:.3f} px | {limits.tof_zero_distortion_rms_px} px |",
        f"| Stereo RMS | {run.extrinsic.stereo_rms:.3f} px | {limits.stereo_max_rms_px} px |",
        f"| Stereo pairs | {run.extrinsic.pair_count} | >= {limits.min_views} |",
        f"| Board plane depth bias (median) | {run.plane.median_mm:.1f} mm | {limits.plane_max_bias_mm} mm |",
        f"| Board plane pairs rejected (background) | {run.plane.rejected_pairs} | "
        f"spread {limits.plane_max_spread_mm} mm |",
        "",
        "ToF intrinsics versus the SDK /tof/camera_info (percent difference):",
        "",
        *[f"- {name}: {value:+.2f} %" for name, value in run.sdk_diff_pct.items()],
        "",
        "Extrinsic (tof_optical_frame in head_camera_optical_frame), metres: "
        + ", ".join(f"{v:.5f}" for v in run.extrinsic.t),
        "",
    ]
    return "\n".join(lines)
