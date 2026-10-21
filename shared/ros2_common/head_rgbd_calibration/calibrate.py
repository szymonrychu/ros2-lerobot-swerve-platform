"""The head RGB-D calibration fit from per-pose detections: no bag, no files, no ssh.

PoseView carries what one still pose yielded (RGB detection, ToF detection, depth frame);
calibrate_detections fits RGB intrinsics, ToF intrinsics, the stereo extrinsic and the plane depth bias.
"""

from dataclasses import dataclass
from typing import Literal

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

OUTLIER_MEDIAN_FACTOR = 3.0  # a pose is an outlier above max(this x the median per-pose error, the fit's limit)
FITS = ("rgb", "tof", "stereo")
FIT_LABELS = {"rgb": "RGB", "tof": "ToF", "stereo": "stereo"}


@dataclass(frozen=True)
class PoseErrors:
    """Reprojection error (px) of one pose in each fit; None means the pose was not in that fit.

    outlier: the subset of ("rgb", "tof", "stereo"), in that order, of the fits in which the pose is flagged.
    """

    pose_id: str
    rgb_px: float | None
    tof_px: float | None
    stereo_px: float | None
    outlier: tuple[str, ...]


@dataclass(frozen=True)
class CalibrationRun:
    """Everything calibration computes before any file is written."""

    rgb: IntrinsicsResult
    tof: IntrinsicsResult
    extrinsic: ExtrinsicResult
    plane: PlaneCheck
    sdk_diff_pct: dict[str, float]
    pair_count: int
    poses: tuple[PoseErrors, ...] = ()


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


def flag_outliers(errors: list[float | None], limit: float) -> list[bool]:
    """Flag the errors above max(OUTLIER_MEDIAN_FACTOR x median of the present errors, limit); None is never flagged.

    Args:
        errors: One reprojection error (px) per pose, None for a pose not in the fit.
        limit: The fit's RMS limit (px).

    Returns:
        One bool per error.
    """
    present = [e for e in errors if e is not None]
    if not present:
        return [False] * len(errors)
    threshold = max(OUTLIER_MEDIAN_FACTOR * float(np.median(present)), limit)
    return [e is not None and e > threshold for e in errors]


def pose_errors(
    view_count: int,
    pose_ids: list[str],
    per_fit: dict[str, list[float | None]],
    limits: LimitsConfig,
) -> tuple[PoseErrors, ...]:
    """One PoseErrors per view from the per-view error lists of each fit, flagged by flag_outliers."""
    fit_limits = {"rgb": limits.rgb_max_rms_px, "tof": limits.tof_max_rms_px, "stereo": limits.stereo_max_rms_px}
    flags = {fit: flag_outliers(per_fit[fit], fit_limits[fit]) for fit in FITS}
    return tuple(
        PoseErrors(
            pose_ids[i],
            per_fit["rgb"][i],
            per_fit["tof"][i],
            per_fit["stereo"][i],
            tuple(fit for fit in FITS if flags[fit][i]),
        )
        for i in range(view_count)
    )


def outlier_pose_ids(run: CalibrationRun) -> list[str]:
    """Ids of the poses flagged in any fit, in view order."""
    return [p.pose_id for p in run.poses if p.outlier]


def calibrate_detections(
    views: list[PoseView],
    rgb_size: tuple[int, int],
    tof_size: tuple[int, int],
    tof_info: CameraInfoData | None,
    board_cfg: BoardConfig,
    limits: LimitsConfig,
    pose_ids: list[str] | None = None,
) -> CalibrationRun:
    """Fit RGB intrinsics, ToF intrinsics and the extrinsic from the detections of the still poses.

    Args:
        views: One PoseView per still pose.
        rgb_size: RGB image size (width, height).
        tof_size: ToF image size (width, height).
        tof_info: The SDK ToF camera_info to compare the fitted intrinsics with.
        board_cfg: The ChArUco board.
        limits: Acceptance limits.
        pose_ids: One id per view (default "001", "002", ...).

    Returns:
        The CalibrationRun; pair_count is len(views); poses holds the per-pose errors and outlier flags (hints only,
        no view is dropped).

    Raises:
        ValueError: pose_ids length differs from len(views), no ToF camera_info, too few views or pairs (the
            message names the count), or no plane measurement.
    """
    if pose_ids is None:
        pose_ids = [f"{i + 1:03d}" for i in range(len(views))]
    if len(pose_ids) != len(views):
        raise ValueError(f"pose_ids has {len(pose_ids)} entries for {len(views)} views")
    if tof_info is None:
        raise ValueError("the bag has no ToF camera_info; cannot compare with the SDK")
    board = make_board(board_cfg)
    rgb = calibrate([v.rgb for v in views if v.rgb is not None], board, rgb_size, limits)
    tof = calibrate_tof([v.tof for v in views if v.tof is not None], board, tof_size, limits)
    both = [(v.rgb, v.tof, v.depth) for v in views if v.rgb is not None and v.tof is not None]
    extrinsic = stereo_calibrate([(r, t) for r, t, _ in both], board, rgb, tof, limits)
    with_depth = [(r, t, depth) for r, t, depth in both if depth is not None]
    plane = plane_depth_bias(with_depth, board, rgb, tof, extrinsic, limits)
    rgb_index = [i for i, v in enumerate(views) if v.rgb is not None]
    tof_index = [i for i, v in enumerate(views) if v.tof is not None]
    both_index = [i for i, v in enumerate(views) if v.rgb is not None and v.tof is not None]
    per_fit: dict[str, list[float | None]] = {fit: [None] * len(views) for fit in FITS}
    for fit, result, index in (("rgb", rgb, rgb_index), ("tof", tof, tof_index)):
        for i, error in zip(index, result.per_view_errors, strict=True):
            per_fit[fit][i] = float(error)
    for pair, error in zip(extrinsic.pair_indices, extrinsic.per_pair_errors, strict=True):
        per_fit["stereo"][both_index[pair]] = error
    poses = pose_errors(len(views), pose_ids, per_fit, limits)
    return CalibrationRun(rgb, tof, extrinsic, plane, compare_sdk(tof, tof_info), len(views), poses)


def fit_error(pose: PoseErrors, fit: Literal["rgb", "tof", "stereo"]) -> float | None:
    """The pose's error in the named fit."""
    return {"rgb": pose.rgb_px, "tof": pose.tof_px, "stereo": pose.stereo_px}[fit]


def outlier_suffix(run: CalibrationRun, fit: str) -> str:
    """The failed_limits suffix naming the poses flagged in the fit (empty when none)."""
    flagged = [p for p in run.poses if fit in p.outlier]
    if not flagged:
        return ""
    named = ", ".join(f"{p.pose_id} ({fit_error(p, fit):.3f} px)" for p in flagged)
    ids = ", ".join(p.pose_id for p in flagged)
    if len(flagged) == 1:
        return f" - outlier pose {named}: delete pose {ids} and Compute again"
    return f" - outlier poses {named}: delete poses {ids} and Compute again"


def failed_limits(run: CalibrationRun, limits: LimitsConfig) -> list[str]:
    """Descriptions of every exceeded limit, each naming its limits.* key."""
    checks = [
        ("limits.rgb_max_rms_px", "RGB RMS", run.rgb.rms, limits.rgb_max_rms_px),
        ("limits.tof_max_rms_px", "ToF RMS", run.tof.rms, limits.tof_max_rms_px),
        ("limits.stereo_max_rms_px", "stereo RMS", run.extrinsic.stereo_rms, limits.stereo_max_rms_px),
    ]
    failures = [
        f"{key}: {label} {value:.3f} px exceeds {limit} px" + outlier_suffix(run, fit)
        for fit, (key, label, value, limit) in zip(FITS, checks, strict=True)
        if value > limit
    ]
    bias = run.plane.median_mm
    if abs(bias) > limits.plane_max_bias_mm:
        failures.append(
            f"limits.plane_max_bias_mm: plane depth bias {bias:.1f} mm exceeds {limits.plane_max_bias_mm} mm"
        )
    return failures


def pose_report_lines(run: CalibrationRun) -> list[str]:
    """The per-pose error table and the flagged-pose list of the report."""

    def cell(value: float | None) -> str:
        return "-" if value is None else f"{value:.3f}"

    lines = [
        "## Per-pose reprojection errors",
        "",
        "| Pose | RGB px | ToF px | Stereo px | Flag |",
        "|---|---|---|---|---|",
    ]
    for p in run.poses:
        flag = "outlier: " + ", ".join(FIT_LABELS[f] for f in p.outlier) if p.outlier else ""
        lines.append(f"| {p.pose_id} | {cell(p.rgb_px)} | {cell(p.tof_px)} | {cell(p.stereo_px)} | {flag} |")
    lines.append("")
    flagged = [p for p in run.poses if p.outlier]
    if not flagged:
        return [*lines, "Flagged poses: none", ""]
    lines.append(f"Flagged poses (error > max({OUTLIER_MEDIAN_FACTOR:g} x median, limit)):")
    lines.append("")
    for p in flagged:
        errors = ", ".join(f"{FIT_LABELS[f]} {fit_error(p, f):.3f} px" for f in p.outlier)
        lines.append(f"- pose {p.pose_id}: {errors} - delete pose {p.pose_id} and Compute again")
    return [*lines, ""]


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
    if run.poses:
        lines += pose_report_lines(run)
    return "\n".join(lines)
