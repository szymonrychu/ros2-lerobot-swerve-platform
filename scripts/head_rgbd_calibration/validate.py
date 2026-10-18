"""Validation metrics for the head RGB-D pipeline, computed from recorded data (methodology: validation targets).

Depth is uint16 millimetres with 0 = invalid. Pixel coordinates follow OpenCV (pixel centres at integers).
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

from scripts.head_rgbd_calibration.charuco import Detection, detect
from scripts.head_rgbd_calibration.extrinsics import ExtrinsicResult
from scripts.head_rgbd_calibration.intrinsics import IntrinsicsResult

MM_PER_M = 1000.0
STEP_THRESHOLD_MM = 300.0
OUTLINE_BAND_PX = 8.0
MIN_SPEED_PX_S = 20.0
MAX_BRACKET_S = 0.1
MAD_TO_SIGMA = 1.4826
P90 = 90.0
LONG_RANGE_M = 2.5
RGB_RMS_MAX_PX = 0.5
TOF_RMS_MAX_PX = 0.3
STEREO_RMS_MAX_PX = 1.0
PLANE_BIAS_MAX_MM = 15.0
DEPTH_LIMIT_MM = 20.0
DEPTH_LIMIT_LONG_MM = 40.0
REGISTRATION_BIAS_MAX_MM = 5.0
EDGE_MEDIAN_MAX_PX = 1.0
VALID_RATIO_MIN = 0.85
TIME_OFFSET_MAX_MS = 10.0
MOTION_EDGE_MAX_PX = 1.5
CPU_P95_MAX_CORES = 0.2


@dataclass(frozen=True)
class DepthStats:
    """Depth accuracy over the ROI: bias and RMSE against the reference (mm), valid_fraction of ROI pixels."""

    bias_mm: float
    rmse_mm: float
    valid_fraction: float


@dataclass(frozen=True)
class AlignmentStats:
    """Distance (output pixels) from registered depth-step points to the RGB board outline."""

    median_px: float
    p90_px: float
    count: int


@dataclass(frozen=True)
class OffsetEstimate:
    """ToF-RGB time offset: median and scaled-MAD spread in ms over pair_count corner pairs.

    Positive: the ToF content was captured later than its stamp says.
    """

    median_ms: float
    spread_ms: float
    pair_count: int


@dataclass(frozen=True)
class RgbSample:
    """RGB corner detection at stamp_s (seconds)."""

    stamp_s: float
    detection: Detection


@dataclass(frozen=True)
class TofSample:
    """ToF amplitude corner detection at stamp_s with the ToF depth (m) of each corner, in detection order."""

    stamp_s: float
    detection: Detection
    depth_m: np.ndarray


@dataclass(frozen=True)
class ValidationResults:
    """Measured values for check_targets; None means not measured.

    depth: raw ToF DepthStats keyed by target range in metres. cpu_p95_cores and placeholder_messages come from Grafana
    and a topic echo with the ToF stopped, not from this tool.
    """

    rgb_rms_px: float | None = None
    tof_rms_px: float | None = None
    stereo_rms_px: float | None = None
    plane_bias_mm: float | None = None
    depth: dict[float, DepthStats] = field(default_factory=dict)
    registration_bias_mm: float | None = None
    edge_median_px: float | None = None
    valid_ratio: float | None = None
    time_offset_ms: float | None = None
    motion_edge_median_px: float | None = None
    cpu_p95_cores: float | None = None
    placeholder_messages: int | None = None


@dataclass(frozen=True)
class TargetResult:
    """One methodology target: measured value (None = not measured, never a pass) against its limit."""

    name: str
    measured: float | None
    limit: float
    unit: str
    passed: bool
    lower_bound: bool = False


def central_roi(frame: np.ndarray, roi_px: int) -> np.ndarray:
    """The central roi_px x roi_px window of frame."""
    top = (frame.shape[0] - roi_px) // 2
    left = (frame.shape[1] - roi_px) // 2
    return frame[top : top + roi_px, left : left + roi_px]


def depth_accuracy(depth_frames: list[np.ndarray], reference_m: float, roi_px: int) -> DepthStats:
    """Bias and RMSE of the central ROI against the reference distance, invalid (0) pixels excluded.

    Args:
        depth_frames: uint16 mm frames of a flat target.
        reference_m: Tape-measured distance.
        roi_px: Side of the central square ROI.

    Returns:
        DepthStats (RMSE includes the bias).

    Raises:
        ValueError: no valid pixel in the ROI.
    """
    window = np.stack([central_roi(frame, roi_px) for frame in depth_frames])
    valid = window > 0
    if not valid.any():
        raise ValueError("no valid depth pixels in the ROI")
    error = window[valid].astype(np.float64) - reference_m * MM_PER_M
    return DepthStats(float(error.mean()), float(np.sqrt((error**2).mean())), float(valid.mean()))


def registration_bias(
    raw_frames: list[np.ndarray],
    registered_frames: list[np.ndarray],
    roi: tuple[int, int, int, int],
    registered_roi: tuple[int, int, int, int] | None = None,
) -> float:
    """Median registered depth minus median raw depth over the matching ROI (mm), zeros excluded.

    Args:
        raw_frames: Raw ToF frames (uint16 mm).
        registered_frames: Registered frames of the same target.
        roi: (x, y, width, height) in the raw frames.
        registered_roi: Same region in the registered frames (default roi).

    Returns:
        Bias in mm.

    Raises:
        ValueError: no valid pixel in either ROI.
    """

    def median(frames: list[np.ndarray], window: tuple[int, int, int, int]) -> float:
        x, y, w, h = window
        values = np.concatenate([frame[y : y + h, x : x + w].ravel() for frame in frames])
        values = values[values > 0]
        if values.size == 0:
            raise ValueError("no valid depth pixels in the ROI")
        return float(np.median(values))

    return median(registered_frames, registered_roi or roi) - median(raw_frames, roi)


def valid_ratio(registered_depth: np.ndarray, fov_mask: np.ndarray) -> float:
    """Fraction of the pixels inside fov_mask that hold a valid (non-zero) depth."""
    return float((registered_depth[fov_mask] > 0).mean())


def depth_step_points(depth: np.ndarray) -> np.ndarray:
    """Edge points (x, y) between each valid pixel and a 4-neighbour farther by more than STEP_THRESHOLD_MM.

    The point is the midpoint of the two pixel centres.
    """
    padded = np.pad(depth.astype(np.float64), 1)
    centre = padded[1:-1, 1:-1]
    points = []
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        neighbour = padded[1 + dy : padded.shape[0] - 1 + dy, 1 + dx : padded.shape[1] - 1 + dx]
        ys, xs = np.nonzero((centre > 0) & (neighbour - centre > STEP_THRESHOLD_MM))
        points.append(np.stack([xs + dx / 2, ys + dy / 2], axis=1))
    return np.concatenate(points).astype(np.float32)


def edge_alignment(
    registered_depth: np.ndarray,
    rgb: np.ndarray,
    board: cv2.aruco.CharucoBoard,
    rgb_camera: IntrinsicsResult,
    depth_scale: int,
) -> AlignmentStats:
    """Distance from registered depth-step pixels to the board outline seen in the RGB image.

    Args:
        registered_depth: uint16 mm depth in the output image (RGB size divided by depth_scale).
        rgb: Full-resolution RGB frame of the same instant, showing the ChArUco board.
        board: Board from make_board.
        rgb_camera: RGB intrinsics at full resolution.
        depth_scale: Output image downscale versus the RGB frame.

    Returns:
        AlignmentStats in output pixels, over step points within OUTLINE_BAND_PX of the outline.

    Raises:
        ValueError: the board is not detected, or no depth step lies near its outline.
    """
    detection = detect(rgb, board)
    if detection is None:
        raise ValueError("board not detected in the RGB frame")
    corners = board.getChessboardCorners()
    ok, rvec, tvec = cv2.solvePnP(corners[detection.ids], detection.points, rgb_camera.k, rgb_camera.d)
    if not ok:
        raise ValueError("board pose estimation failed")
    size = np.array(board.getChessboardSize(), dtype=np.float64) * board.getSquareLength()
    rectangle = np.array([[0, 0, 0], [size[0], 0, 0], [size[0], size[1], 0], [0, size[1], 0]], dtype=np.float64)
    projected = cv2.projectPoints(rectangle, rvec, tvec, rgb_camera.k, rgb_camera.d)[0].reshape(-1, 2)
    outline = ((projected + 0.5) / depth_scale - 0.5).astype(np.float32)
    distances = np.array(
        [abs(cv2.pointPolygonTest(outline, (float(x), float(y)), True)) for x, y in depth_step_points(registered_depth)]
    )
    distances = distances[distances <= OUTLINE_BAND_PX]
    if distances.size == 0:
        raise ValueError("no depth step near the board outline")
    return AlignmentStats(float(np.median(distances)), float(np.percentile(distances, P90)), int(distances.size))


def time_offset(
    rgb_tracks: list[RgbSample],
    tof_tracks: list[TofSample],
    extrinsic: ExtrinsicResult,
    rgb_camera: IntrinsicsResult,
    tof_camera: IntrinsicsResult,
    min_speed_px_s: float = MIN_SPEED_PX_S,
) -> OffsetEstimate:
    """ToF-RGB time offset from a pan sweep.

    Each ToF corner (with its depth) is projected into the RGB image with the extrinsic and compared with the RGB track
    of the same corner id, linearly interpolated at the ToF stamp: offset = (u_projected - u_rgb) / (du_rgb/dt).

    Args:
        rgb_tracks: RGB detections in time order.
        tof_tracks: ToF detections with corner depths.
        extrinsic: ToF-to-RGB extrinsic (p_rgb = r @ p_tof + t).
        rgb_camera: RGB intrinsics.
        tof_camera: ToF intrinsics.
        min_speed_px_s: Pairs with |du/dt| below this are ignored.

    Returns:
        OffsetEstimate in ms.

    Raises:
        ValueError: no usable pairs.
    """
    tracks: dict[int, tuple[list[float], list[float]]] = {}
    for sample in rgb_tracks:
        for corner_id, point in zip(sample.detection.ids, sample.detection.points, strict=True):
            times, us = tracks.setdefault(int(corner_id), ([], []))
            times.append(sample.stamp_s)
            us.append(float(point[0]))
    offsets = []
    for sample in tof_tracks:
        depth = np.asarray(sample.depth_m, dtype=np.float64)
        normalised = cv2.undistortPoints(
            sample.detection.points.reshape(-1, 1, 2).astype(np.float64), tof_camera.k, tof_camera.d
        ).reshape(-1, 2)
        points_rgb = (np.column_stack([normalised * depth[:, None], depth]) @ extrinsic.r.T) + extrinsic.t.reshape(3)
        projected = cv2.projectPoints(points_rgb, np.zeros(3), np.zeros(3), rgb_camera.k, rgb_camera.d)[0].reshape(
            -1, 2
        )
        for corner_id, z, u_projected in zip(sample.detection.ids, depth, projected[:, 0], strict=True):
            if int(corner_id) not in tracks or not np.isfinite(z) or z <= 0:
                continue
            times, us = tracks[int(corner_id)]
            index = int(np.searchsorted(times, sample.stamp_s))
            if index == 0 or index >= len(times) or times[index] - times[index - 1] > MAX_BRACKET_S:
                continue
            slope = (us[index] - us[index - 1]) / (times[index] - times[index - 1])
            if abs(slope) < min_speed_px_s:
                continue
            u_rgb = us[index - 1] + slope * (sample.stamp_s - times[index - 1])
            offsets.append((u_projected - u_rgb) / slope * MM_PER_M)
    if not offsets:
        raise ValueError("no corner pairs above the minimum speed")
    values = np.array(offsets)
    median = float(np.median(values))
    return OffsetEstimate(median, float(MAD_TO_SIGMA * np.median(np.abs(values - median))), len(values))


def check_targets(results: ValidationResults) -> list[TargetResult]:
    """Compare every methodology target with its measured value; an unmeasured target fails.

    Args:
        results: Measured values.

    Returns:
        One TargetResult per target (one depth accuracy target per measured range, 0.5/1/2/3 m when none measured).
    """

    def upper(name: str, measured: float | None, limit: float, unit: str, use_abs: bool = False) -> TargetResult:
        value = abs(measured) if use_abs and measured is not None else measured
        return TargetResult(name, measured, limit, unit, value is not None and value <= limit)

    ranges = sorted(results.depth) or [0.5, 1.0, 2.0, 3.0]
    targets = [
        upper("RGB intrinsic RMS", results.rgb_rms_px, RGB_RMS_MAX_PX, "px"),
        upper("ToF intrinsic RMS", results.tof_rms_px, TOF_RMS_MAX_PX, "px"),
        upper("stereo RMS", results.stereo_rms_px, STEREO_RMS_MAX_PX, "px"),
        upper("plane depth bias", results.plane_bias_mm, PLANE_BIAS_MAX_MM, "mm", use_abs=True),
    ]
    for range_m in ranges:
        stats = results.depth.get(range_m)
        limit = DEPTH_LIMIT_LONG_MM if range_m >= LONG_RANGE_M else DEPTH_LIMIT_MM
        worst = None if stats is None else max(abs(stats.bias_mm), stats.rmse_mm)
        targets.append(upper(f"depth accuracy {range_m:g} m", worst, limit, "mm"))
    targets += [
        upper("registration bias", results.registration_bias_mm, REGISTRATION_BIAS_MAX_MM, "mm", use_abs=True),
        upper("edge alignment", results.edge_median_px, EDGE_MEDIAN_MAX_PX, "px"),
        TargetResult(
            "valid-pixel ratio",
            results.valid_ratio,
            VALID_RATIO_MIN,
            "fraction",
            results.valid_ratio is not None and results.valid_ratio >= VALID_RATIO_MIN,
            lower_bound=True,
        ),
        upper("time offset", results.time_offset_ms, TIME_OFFSET_MAX_MS, "ms", use_abs=True),
        upper("motion edge alignment", results.motion_edge_median_px, MOTION_EDGE_MAX_PX, "px"),
        upper("CPU p95", results.cpu_p95_cores, CPU_P95_MAX_CORES, "cores"),
        upper("placeholder messages", results.placeholder_messages, 0, "messages"),
    ]
    return targets
