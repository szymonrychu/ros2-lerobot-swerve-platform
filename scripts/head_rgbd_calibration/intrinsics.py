"""Camera intrinsics from ChArUco detections: calibration, ToF zero-distortion choice, SDK comparison, YAML output."""

from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
import yaml

from scripts.head_rgbd_calibration.charuco import Detection
from scripts.head_rgbd_calibration.config import LimitsConfig
from scripts.head_rgbd_calibration.frames import CameraInfoData

DISTORTION_MODEL = "plumb_bob"
DISTORTION_COEFFS = 5
ZERO_DISTORTION_FLAGS = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
KIND_RGB = "rgb"
KIND_TOF = "tof"
PERCENT = 100.0


@dataclass(frozen=True)
class IntrinsicsResult:
    """Calibration outcome. kind is "rgb" or "tof" (selects the RMS acceptance limit when writing).

    zero_distortion_rms: RMS of the fit with distortion fixed at zero (ToF only, None for RGB).
    """

    k: np.ndarray
    d: np.ndarray
    rms: float
    per_view_errors: np.ndarray
    view_count: int
    image_size: tuple[int, int]
    kind: str = KIND_RGB
    zero_distortion_rms: float | None = None


def require_views(detections: list[Detection], limits: LimitsConfig) -> None:
    """Raise ValueError naming the view count when it is below limits.min_views."""
    if len(detections) < limits.min_views:
        raise ValueError(f"calibration needs at least {limits.min_views} views, got {len(detections)} views")


def fit(
    detections: list[Detection], board: cv2.aruco.CharucoBoard, image_size: tuple[int, int], flags: int, kind: str
) -> IntrinsicsResult:
    """Run cv2.calibrateCamera (plumb_bob) over the detections with the given flags."""
    corners = board.getChessboardCorners()
    object_points = [corners[d.ids] for d in detections]
    image_points = [d.points.reshape(-1, 1, 2) for d in detections]
    rms, k, d, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        object_points, image_points, image_size, None, np.zeros(DISTORTION_COEFFS), flags=flags
    )
    return IntrinsicsResult(
        k=k,
        d=d.reshape(-1)[:DISTORTION_COEFFS],
        rms=float(rms),
        per_view_errors=per_view.reshape(-1),
        view_count=len(detections),
        image_size=image_size,
        kind=kind,
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
    return fit(detections, board, image_size, 0, KIND_RGB)


def calibrate_tof(
    detections: list[Detection], board: cv2.aruco.CharucoBoard, image_size: tuple[int, int], limits: LimitsConfig
) -> IntrinsicsResult:
    """Calibrate the ToF camera, preferring the zero-distortion model when it fits well enough.

    Args:
        detections: One Detection per view (from the normalised amplitude).
        board: Board from make_board.
        image_size: (width, height).
        limits: Acceptance limits (min_views, tof_zero_distortion_rms_px).

    Returns:
        The zero-distortion result when its RMS <= limits.tof_zero_distortion_rms_px, else the free-distortion one;
        zero_distortion_rms records the zero-distortion RMS either way.

    Raises:
        ValueError: fewer than limits.min_views views (the message names the count).
    """
    require_views(detections, limits)
    zero = fit(detections, board, image_size, ZERO_DISTORTION_FLAGS, KIND_TOF)
    chosen = zero if zero.rms <= limits.tof_zero_distortion_rms_px else fit(detections, board, image_size, 0, KIND_TOF)
    return replace(chosen, zero_distortion_rms=zero.rms)


def compare_sdk(result: IntrinsicsResult, sdk_camera_info: CameraInfoData) -> dict[str, float]:
    """Percent difference of fx, fy, cx, cy relative to the SDK /tof/camera_info.

    Args:
        result: Measured calibration.
        sdk_camera_info: The SDK camera_info (k row-major 3x3).

    Returns:
        {"fx", "fy", "cx", "cy"}: 100 * (measured - sdk) / sdk.
    """
    sdk_k = np.asarray(sdk_camera_info.k, dtype=np.float64).reshape(3, 3)
    entries = {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}
    return {name: float(PERCENT * (result.k[idx] - sdk_k[idx]) / sdk_k[idx]) for name, idx in entries.items()}


def matrix_block(matrix: np.ndarray) -> dict[str, object]:
    """ROS camera_calibration matrix mapping (rows, cols, data)."""
    matrix = np.atleast_2d(matrix)
    return {"rows": matrix.shape[0], "cols": matrix.shape[1], "data": [float(v) for v in matrix.reshape(-1)]}


def write_camera_calibration_yaml(
    path: Path,
    result: IntrinsicsResult,
    camera_name: str,
    image_size: tuple[int, int],
    limits: LimitsConfig | None = None,
) -> None:
    """Write the ROS camera_calibration YAML that camera_calibration_parsers reads.

    Args:
        path: Output file.
        result: Calibration to write.
        camera_name: camera_name field.
        image_size: (width, height) of the calibrated images.
        limits: Acceptance limits (default LimitsConfig()); rgb_max_rms_px or tof_max_rms_px applies by result.kind.

    Raises:
        ValueError: result.rms exceeds the camera's limit; nothing is written.
    """
    limits = limits or LimitsConfig()
    max_rms = limits.tof_max_rms_px if result.kind == KIND_TOF else limits.rgb_max_rms_px
    if result.rms > max_rms:
        raise ValueError(
            f"{camera_name} calibration RMS {result.rms:.3f} px exceeds the limit {max_rms} px; not written"
        )
    projection = np.hstack([result.k, np.zeros((3, 1))])
    data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": camera_name,
        "camera_matrix": matrix_block(result.k),
        "distortion_model": DISTORTION_MODEL,
        "distortion_coefficients": matrix_block(result.d.reshape(1, -1)),
        "rectification_matrix": matrix_block(np.eye(3)),
        "projection_matrix": matrix_block(projection),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
