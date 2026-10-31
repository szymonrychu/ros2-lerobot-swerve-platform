"""The D435 colour intrinsics check: the colour fit from ChArUco views compared with the factory camera_info, and the
board-plane depth bias of the aligned depth.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import LimitsConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.intrinsics import IntrinsicsResult, calibrate, compare_sdk

MIN_BOARD_PIXELS = 200  # fewer valid board pixels than this give no plane bias
MM_PER_M = 1000.0
MAD_TO_SIGMA = 1.4826  # scales a median absolute deviation to a standard deviation for normal data


@dataclass(frozen=True)
class IntrinsicsCheck:
    """Colour intrinsics fitted from ChArUco views, compared with the factory camera_info.

    k, d: fitted matrix (3x3) and plumb_bob coefficients. rms_px: fit RMS. focal_diff_pct: larger of |fx| and |fy|
    percent difference to the factory. principal_diff_px: distance between the fitted and factory principal points.
    per_view_errors: reprojection error (px) per view. failed_limits: one description per exceeded limit, each naming
    its limits.* key.
    """

    k: np.ndarray
    d: np.ndarray
    rms_px: float
    focal_diff_pct: float
    principal_diff_px: float
    per_view_errors: tuple[float, ...]
    failed_limits: tuple[str, ...]
    passed: bool


def check_intrinsics(
    detections: list[Detection], board: cv2.aruco.CharucoBoard, factory: CameraInfoData, limits: LimitsConfig
) -> IntrinsicsCheck:
    """Fit the colour intrinsics and compare them with the factory camera_info.

    Args:
        detections: One Detection per view.
        board: The ChArUco board from make_board.
        factory: The factory camera_info (its width and height are the image size).
        limits: Acceptance limits (min_views and the intrinsics_max_* limits).

    Returns:
        The IntrinsicsCheck; passed is True when no limit is exceeded.

    Raises:
        ValueError: fewer than limits.min_views views (the message names the count).
    """
    result = calibrate(detections, board, (factory.width, factory.height), limits)
    diff = compare_sdk(result, factory)
    focal_diff = max(abs(diff["fx"]), abs(diff["fy"]))
    factory_k = np.asarray(factory.k, dtype=np.float64).reshape(3, 3)
    principal_diff = float(np.hypot(result.k[0, 2] - factory_k[0, 2], result.k[1, 2] - factory_k[1, 2]))
    checks = [
        ("limits.intrinsics_max_rms_px", "RMS", result.rms, limits.intrinsics_max_rms_px, "px"),
        (
            "limits.intrinsics_max_focal_diff_pct",
            "focal difference",
            focal_diff,
            limits.intrinsics_max_focal_diff_pct,
            "%",
        ),
        (
            "limits.intrinsics_max_principal_diff_px",
            "principal point difference",
            principal_diff,
            limits.intrinsics_max_principal_diff_px,
            "px",
        ),
    ]
    failures = tuple(
        f"{key}: {label} {value:.3f} {unit} exceeds {limit} {unit}"
        for key, label, value, limit, unit in checks
        if value > limit
    )
    return IntrinsicsCheck(
        k=result.k,
        d=result.d,
        rms_px=result.rms,
        focal_diff_pct=focal_diff,
        principal_diff_px=principal_diff,
        per_view_errors=tuple(float(e) for e in result.per_view_errors),
        failed_limits=failures,
        passed=not failures,
    )


def board_plane_bias_mm(
    depth_mm: np.ndarray,
    corners_px: np.ndarray,
    board_pose: tuple[np.ndarray, np.ndarray],
    intrinsics: IntrinsicsResult,
) -> tuple[float, float] | None:
    """Median and spread of (aligned depth - PnP board plane depth) over the board pixels.

    Args:
        depth_mm: Depth aligned to the colour image, uint16 millimetres (0 is invalid).
        corners_px: (N, 2) board outline polygon in colour image pixels (the projected outer board corners).
        board_pose: (r, t) with p_cam = r @ p_board + t, r (3, 3) and t (3,), in the colour optical frame (PnP).
        intrinsics: Colour intrinsics (K, D) of the aligned depth.

    Returns:
        (median_mm, spread_mm), the spread being the scaled median absolute deviation; None when fewer than
        MIN_BOARD_PIXELS valid pixels lie inside the polygon.
    """
    r, t = np.asarray(board_pose[0], dtype=np.float64), np.asarray(board_pose[1], dtype=np.float64).reshape(3)
    mask = np.zeros(depth_mm.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(np.asarray(corners_px)).astype(np.int32).reshape(-1, 2), 1)
    v, u = np.nonzero((mask > 0) & (depth_mm > 0))
    if len(u) < MIN_BOARD_PIXELS:
        return None
    normal = r[:, 2]
    offset = float(normal @ t)
    points = np.stack([u, v], axis=1).astype(np.float64).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(points, intrinsics.k, intrinsics.d).reshape(-1, 2)
    rays = np.concatenate([rays, np.ones((len(rays), 1))], axis=1)
    residual = depth_mm[v, u].astype(np.float64) - offset / (rays @ normal) * MM_PER_M
    median = float(np.median(residual))
    return median, float(MAD_TO_SIGMA * np.median(np.abs(residual - median)))


def intrinsics_report_markdown(
    check: IntrinsicsCheck, bias: tuple[float, float] | None, session: str, date: str
) -> str:
    """The intrinsics check report as markdown.

    Args:
        check: The check result.
        bias: (median_mm, spread_mm) from board_plane_bias_mm, None when not measured.
        session: Session name.
        date: Report date.

    Returns:
        The markdown text.
    """
    bias_text = (
        "not measured (too few valid board pixels)" if bias is None else f"{bias[0]:.1f} mm (spread {bias[1]:.1f} mm)"
    )
    lines = [
        f"# D435 intrinsics check {session}",
        "",
        f"Date: {date}",
        f"Result: {'PASS' if check.passed else 'FAIL'}",
        "",
        "| Item | Value |",
        "|---|---|",
        f"| RMS | {check.rms_px:.3f} px |",
        f"| Focal difference to factory | {check.focal_diff_pct:.2f} % |",
        f"| Principal point difference to factory | {check.principal_diff_px:.2f} px |",
        f"| Board plane depth bias | {bias_text} |",
        f"| Views | {len(check.per_view_errors)} |",
        "",
        "Fitted K (row-major): " + ", ".join(f"{v:.3f}" for v in check.k.reshape(-1)),
        "Fitted D: " + ", ".join(f"{v:.5f}" for v in check.d),
        "",
        "Per-view reprojection errors (px): " + ", ".join(f"{e:.3f}" for e in check.per_view_errors),
        "",
    ]
    if check.failed_limits:
        lines += ["Failed limits:", "", *[f"- {f}" for f in check.failed_limits], ""]
    return "\n".join(lines)
