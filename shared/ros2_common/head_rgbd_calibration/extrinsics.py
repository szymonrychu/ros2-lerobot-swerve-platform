"""ToF-to-RGB extrinsic from ChArUco pairs: stereo calibration, board-plane depth check, YAML output."""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import LimitsConfig
from ros2_common.head_rgbd_calibration.intrinsics import IntrinsicsResult

PARENT_FRAME = "head_camera_optical_frame"
CHILD_FRAME = "tof_optical_frame"
MM_PER_M = 1000.0
MIN_PLANE_PIXELS = 20
MAX_OFF_PLANE_FRACTION = 0.5  # a pair with more of its board pixels this far off its own median has background in it


MIN_COMMON_CORNERS = 6  # cv2.stereoCalibrate runs a DLT pose per view (findExtrinsicCameraParams2): >= 6 points


@dataclass(frozen=True)
class ExtrinsicResult:
    """Pose of tof_optical_frame in head_camera_optical_frame: p_rgb = r @ p_tof + t (metres).

    pair_indices: index into the stereo_calibrate input pairs of every pair used, in order.
    per_pair_errors: per used pair sqrt((e_tof**2 + e_rgb**2) / 2) of its per-camera reprojection RMS (px).
    """

    r: np.ndarray
    t: np.ndarray
    stereo_rms: float
    pair_count: int
    pair_indices: tuple[int, ...] = ()
    per_pair_errors: tuple[float, ...] = ()


@dataclass(frozen=True)
class PlaneCheck:
    """Plane depth check: per_pair_mm is the per-pair median (measured - predicted) ToF depth, median_mm overall.

    rejected_pairs counts the pairs left out because most of their pixels inside the board outline were farther than
    limits.plane_max_spread_mm from the pair median (background or a hand inside the outline).
    """

    per_pair_mm: list[float]
    median_mm: float
    rejected_pairs: int = 0


def common_points(
    rgb: Detection, tof: Detection, corners: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Object points and both image point sets for corner ids seen in both images, or None if under 6 are shared."""
    ids = np.intersect1d(rgb.ids, tof.ids)
    if len(ids) < MIN_COMMON_CORNERS:
        return None
    rgb_lookup = {int(i): p for i, p in zip(rgb.ids, rgb.points, strict=True)}
    tof_lookup = {int(i): p for i, p in zip(tof.ids, tof.points, strict=True)}
    rgb_points = np.array([rgb_lookup[int(i)] for i in ids], dtype=np.float32)
    tof_points = np.array([tof_lookup[int(i)] for i in ids], dtype=np.float32)
    return corners[ids].astype(np.float32), tof_points, rgb_points


def stereo_calibrate(
    pairs: list[tuple[Detection, Detection]],
    board: cv2.aruco.CharucoBoard,
    rgb: IntrinsicsResult,
    tof: IntrinsicsResult,
    limits: LimitsConfig | None = None,
) -> ExtrinsicResult:
    """Estimate the ToF-to-RGB extrinsic with cv2.stereoCalibrate(CALIB_FIX_INTRINSIC).

    Args:
        pairs: (rgb detection, tof detection) of the same board pose; only corner ids seen in both are used and pairs
            with fewer than 6 are skipped.
        board: Board from make_board.
        rgb: RGB intrinsics.
        tof: ToF intrinsics.
        limits: Acceptance limits (default LimitsConfig()); min_views applies.

    Returns:
        ExtrinsicResult mapping ToF points into the RGB frame (ToF is camera 1, RGB camera 2).

    Raises:
        ValueError: fewer than limits.min_views usable pairs (the message names the count).
    """
    limits = limits or LimitsConfig()
    corners = board.getChessboardCorners()
    found = [(i, common_points(rgb_det, tof_det, corners)) for i, (rgb_det, tof_det) in enumerate(pairs)]
    indices = [i for i, c in found if c is not None]
    used = [c for _, c in found if c is not None]
    if len(used) < limits.min_views:
        raise ValueError(f"extrinsic calibration needs at least {limits.min_views} pairs, got {len(used)} pairs")
    rms, _, _, _, _, r, t, _, _, _, _, per_view = cv2.stereoCalibrateExtended(
        [u[0] for u in used],
        [u[1].reshape(-1, 1, 2) for u in used],
        [u[2].reshape(-1, 1, 2) for u in used],
        tof.k,
        tof.d,
        rgb.k,
        rgb.d,
        rgb.image_size,
        None,
        None,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    per_pair = np.sqrt(np.mean(np.square(per_view.reshape(-1, 2)), axis=1))
    return ExtrinsicResult(
        r=r,
        t=t.reshape(3),
        stereo_rms=float(rms),
        pair_count=len(used),
        pair_indices=tuple(indices),
        per_pair_errors=tuple(float(e) for e in per_pair),
    )


def plane_depth_bias(
    pairs_with_depth: list[tuple[Detection, Detection, np.ndarray]],
    board: cv2.aruco.CharucoBoard,
    rgb: IntrinsicsResult,
    tof: IntrinsicsResult,
    extrinsic: ExtrinsicResult,
    limits: LimitsConfig | None = None,
) -> PlaneCheck:
    """Compare measured ToF depth with the board plane found from RGB PnP and carried through the extrinsic.

    Args:
        pairs_with_depth: (rgb detection, tof detection, tof depth uint16 millimetres) per board pose.
        board: Board from make_board.
        rgb: RGB intrinsics (PnP).
        tof: ToF intrinsics (rays).
        extrinsic: ToF-to-RGB extrinsic under test.

    Returns:
        PlaneCheck: per pair the median of (measured - predicted) depth in mm over valid pixels inside the projected
        board outline, and the median over pairs. Pairs with too few valid pixels are skipped.

    Raises:
        ValueError: no pair yields a plane measurement.
    """
    limits = limits or LimitsConfig()
    corners = board.getChessboardCorners()
    width, height = (
        board.getChessboardSize()[0] * board.getSquareLength(),
        (board.getChessboardSize()[1] * board.getSquareLength()),
    )
    outline = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]], dtype=np.float64)
    per_pair: list[float] = []
    rejected = 0
    for rgb_det, _, depth in pairs_with_depth:
        ok, rvec, tvec = cv2.solvePnP(corners[rgb_det.ids], rgb_det.points, rgb.k, rgb.d)
        if not ok:
            continue
        r_board = cv2.Rodrigues(rvec)[0]
        normal_rgb = r_board[:, 2]
        offset_rgb = float(normal_rgb @ tvec.reshape(3))
        normal_tof = extrinsic.r.T @ normal_rgb
        offset_tof = offset_rgb - float(normal_rgb @ extrinsic.t)
        outline_tof = (r_board @ outline.T + tvec.reshape(3, 1)).T
        outline_tof = (outline_tof - extrinsic.t) @ extrinsic.r
        if np.any(outline_tof[:, 2] <= 0):
            continue
        polygon = cv2.projectPoints(outline_tof, np.zeros(3), np.zeros(3), tof.k, tof.d)[0].reshape(-1, 2)
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 1)
        v, u = np.nonzero((mask > 0) & (depth > 0))
        if len(u) < MIN_PLANE_PIXELS:
            continue
        rays = cv2.undistortPoints(np.stack([u, v], axis=1).astype(np.float64).reshape(-1, 1, 2), tof.k, tof.d).reshape(
            -1, 2
        )
        rays = np.concatenate([rays, np.ones((len(rays), 1))], axis=1)
        predicted_mm = offset_tof / (rays @ normal_tof) * MM_PER_M
        residual_mm = depth[v, u].astype(np.float64) - predicted_mm
        bias_mm = float(np.median(residual_mm))
        if float(np.mean(np.abs(residual_mm - bias_mm) > limits.plane_max_spread_mm)) > MAX_OFF_PLANE_FRACTION:
            rejected += 1
            continue
        per_pair.append(bias_mm)
    if not per_pair:
        raise ValueError("plane depth check found no pair with valid ToF depth inside the board")
    return PlaneCheck(per_pair_mm=per_pair, median_mm=float(np.median(per_pair)), rejected_pairs=rejected)


def write_extrinsics_yaml(
    path: Path,
    result: ExtrinsicResult,
    plane_check: PlaneCheck,
    meta: dict[str, object],
    limits: LimitsConfig | None = None,
) -> None:
    """Write the extrinsics report YAML.

    Args:
        path: Output file.
        result: Extrinsic to write.
        plane_check: Plane depth bias result (its median is recorded).
        meta: lens_position, board, date, rgb_rms_px, tof_rms_px, tof_sdk_diff_pct.
        limits: Acceptance limits (default LimitsConfig()); stereo_max_rms_px applies.

    Raises:
        ValueError: result.stereo_rms exceeds limits.stereo_max_rms_px; nothing is written.
    """
    limits = limits or LimitsConfig()
    if result.stereo_rms > limits.stereo_max_rms_px:
        raise ValueError(
            f"stereo RMS {result.stereo_rms:.3f} px exceeds the limit {limits.stereo_max_rms_px} px; not written"
        )
    data = {
        "parent_frame": PARENT_FRAME,
        "child_frame": CHILD_FRAME,
        "translation_m": [float(v) for v in result.t],
        "rotation_xyzw": [float(v) for v in Rotation.from_matrix(result.r).as_quat()],
        "stereo_rms_px": float(result.stereo_rms),
        "pairs": int(result.pair_count),
        "plane_bias_mm_median": float(plane_check.median_mm),
        **meta,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))


def load_extrinsics_yaml(path: Path) -> ExtrinsicResult:
    """Read an extrinsics YAML written by write_extrinsics_yaml.

    Args:
        path: File to read.

    Returns:
        The ExtrinsicResult (rotation as a 3x3 matrix, translation in metres).

    Raises:
        ValueError: the frames are not head_camera_optical_frame <- tof_optical_frame.
    """
    data = yaml.safe_load(path.read_text())
    if data["parent_frame"] != PARENT_FRAME or data["child_frame"] != CHILD_FRAME:
        raise ValueError(f"{path} is not a {CHILD_FRAME} -> {PARENT_FRAME} extrinsic")
    return ExtrinsicResult(
        r=Rotation.from_quat(data["rotation_xyzw"]).as_matrix(),
        t=np.asarray(data["translation_m"], dtype=np.float64),
        stereo_rms=float(data["stereo_rms_px"]),
        pair_count=int(data["pairs"]),
    )
