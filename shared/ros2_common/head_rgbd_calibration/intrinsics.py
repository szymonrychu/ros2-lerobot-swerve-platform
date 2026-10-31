"""Camera intrinsics from ChArUco detections: calibration and the comparison with a factory camera_info."""

from dataclasses import dataclass

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import LimitsConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData

DISTORTION_COEFFS = 5
PERCENT = 100.0


@dataclass(frozen=True)
class IntrinsicsResult:
    """Calibration outcome: K, plumb_bob D, RMS and per-view reprojection errors (px), view count, image size."""

    k: np.ndarray
    d: np.ndarray
    rms: float
    per_view_errors: np.ndarray
    view_count: int
    image_size: tuple[int, int]


def require_views(detections: list[Detection], limits: LimitsConfig) -> None:
    """Raise ValueError naming the view count when it is below limits.min_views."""
    if len(detections) < limits.min_views:
        raise ValueError(f"calibration needs at least {limits.min_views} views, got {len(detections)} views")


def fit(detections: list[Detection], board: cv2.aruco.CharucoBoard, image_size: tuple[int, int]) -> IntrinsicsResult:
    """Run cv2.calibrateCamera (plumb_bob) over the detections."""
    corners = board.getChessboardCorners()
    object_points = [corners[d.ids] for d in detections]
    image_points = [d.points.reshape(-1, 1, 2) for d in detections]
    rms, k, d, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        object_points, image_points, image_size, None, np.zeros(DISTORTION_COEFFS)
    )
    return IntrinsicsResult(
        k=k,
        d=d.reshape(-1)[:DISTORTION_COEFFS],
        rms=float(rms),
        per_view_errors=per_view.reshape(-1),
        view_count=len(detections),
        image_size=image_size,
    )


def calibrate(
    detections: list[Detection],
    board: cv2.aruco.CharucoBoard,
    image_size: tuple[int, int],
    limits: LimitsConfig | None = None,
) -> IntrinsicsResult:
    """Calibrate a camera with the plumb_bob model.

    Args:
        detections: One Detection per view.
        board: Board from make_board.
        image_size: (width, height).
        limits: Acceptance limits (default LimitsConfig()).

    Returns:
        The IntrinsicsResult (K, D, RMS, per-view errors, view count).

    Raises:
        ValueError: fewer than limits.min_views views (the message names the count).
    """
    require_views(detections, limits or LimitsConfig())
    return fit(detections, board, image_size)


def compare_sdk(result: IntrinsicsResult, sdk_camera_info: CameraInfoData) -> dict[str, float]:
    """Percent difference of fx, fy, cx, cy relative to a reference camera_info (the D435 factory one).

    Args:
        result: Measured calibration.
        sdk_camera_info: The reference camera_info (k row-major 3x3).

    Returns:
        {"fx", "fy", "cx", "cy"}: 100 * (measured - sdk) / sdk.
    """
    sdk_k = np.asarray(sdk_camera_info.k, dtype=np.float64).reshape(3, 3)
    entries = {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}
    return {name: float(PERCENT * (result.k[idx] - sdk_k[idx]) / sdk_k[idx]) for name, idx in entries.items()}
