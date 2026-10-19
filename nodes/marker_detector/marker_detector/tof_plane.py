"""Marker plane from the registered ToF depth, used to settle the IPPE ambiguity (design D10b).

The depth image (``/head_rgbd/depth/image_raw``, 16UC1 mm, ``head_camera_optical_frame``, 288x162) is registered to
the head RGB camera, so a marker polygon found in the RGB image covers the same surface in the depth image once scaled
to the depth size. Pixel centres sit at integer coordinates in both images: a point p scales as (p + 0.5) * s - 0.5.

Normal convention (shared with ``pose.Solution.normal``): the plane normal points out of the surface towards the camera,
exactly like the marker's +z axis (out of the printed face). So a correct IPPE solution has its +z close to the plane
normal and the planar flip does not.
"""

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from marker_detector.config import TofConfig
from marker_detector.pose import Solution, angle_between_deg

REASON_TOO_FEW_PX = "too_few_px"
REASON_RESIDUAL = "residual"
MIN_PLANE_POINTS = 3
NS_PER_S = 1_000_000_000


@dataclass
class Plane:
    """Least-squares plane in the depth camera frame: points p (mm) with normal . p + distance_mm = 0.

    normal: (3,) unit normal oriented towards the camera. distance_mm: distance from the camera centre to the plane (mm,
    >= 0). residual_mm: RMS point-to-plane distance of the fitted pixels (mm). n_px: number of valid pixels fitted.
    """

    normal: np.ndarray
    distance_mm: float
    residual_mm: float
    n_px: int


@dataclass
class PlaneFit:
    """Outcome of ``fit_plane_detailed``.

    plane: the accepted plane, None when rejected. reason: ``too_few_px`` or ``residual`` when rejected, else None.
    n_px: valid pixels inside the polygon. residual_mm: RMS residual of the fit (nan when too few pixels to fit).
    """

    plane: Plane | None
    reason: str | None
    n_px: int
    residual_mm: float = math.nan


def scale_polygon(polygon_px: np.ndarray, image_size: tuple[int, int], depth_size: tuple[int, int]) -> np.ndarray:
    """Scale a polygon from the RGB image to the depth image (pixel centres at integers in both).

    Args:
        polygon_px: (N, 2) polygon vertices in RGB pixels.
        image_size: (width, height) of the RGB image.
        depth_size: (width, height) of the depth image.

    Returns:
        (N, 2) float vertices in depth pixels.
    """
    scale = np.array([depth_size[0] / image_size[0], depth_size[1] / image_size[1]])
    return (np.asarray(polygon_px, dtype=float).reshape(-1, 2) + 0.5) * scale - 0.5


def polygon_mask(polygon_px: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Pixels whose centre lies inside a polygon (even-odd rule).

    Args:
        polygon_px: (N, 2) polygon vertices in pixels of the target image.
        shape: (height, width) of the target image.

    Returns:
        (height, width) bool mask.
    """
    vertices = np.asarray(polygon_px, dtype=float).reshape(-1, 2)
    mask = np.zeros(shape, dtype=bool)
    if len(vertices) < MIN_PLANE_POINTS or not np.all(np.isfinite(vertices)):
        return mask
    u0 = max(0, math.floor(vertices[:, 0].min()))
    u1 = min(shape[1] - 1, math.ceil(vertices[:, 0].max()))
    v0 = max(0, math.floor(vertices[:, 1].min()))
    v1 = min(shape[0] - 1, math.ceil(vertices[:, 1].max()))
    if u0 > u1 or v0 > v1:
        return mask
    u, v = np.meshgrid(np.arange(u0, u1 + 1, dtype=float), np.arange(v0, v1 + 1, dtype=float))
    inside = np.zeros(u.shape, dtype=bool)
    for (xa, ya), (xb, yb) in zip(vertices, np.roll(vertices, -1, axis=0), strict=True):
        if ya == yb:
            continue
        crosses = (ya > v) != (yb > v)
        x_cross = xa + (v - ya) * (xb - xa) / (yb - ya)
        inside ^= crosses & (u < x_cross)
    mask[v0 : v1 + 1, u0 : u1 + 1] = inside
    return mask


def fit_plane_detailed(
    depth_mm: np.ndarray,
    depth_K: np.ndarray,  # noqa: N803
    polygon_px: np.ndarray,
    image_size: tuple[int, int],
    cfg: TofConfig,
) -> PlaneFit:
    """Least-squares plane (SVD) of the valid depth pixels inside the marker polygon, with the rejection reason.

    Args:
        depth_mm: (H, W) depth image in mm (16UC1); 0 (and any non-finite or negative value) is invalid.
        depth_K: 3x3 intrinsics of the depth image.
        polygon_px: (N, 2) marker polygon in RGB pixels (e.g. the four symbol corners).
        image_size: (width, height) of the RGB image the polygon comes from.
        cfg: ToF thresholds (``min_valid_px``, ``max_residual_mm``).

    Returns:
        PlaneFit: the plane, or None with ``too_few_px`` (fewer than ``min_valid_px`` valid pixels) or ``residual``
        (RMS residual above ``max_residual_mm``).
    """
    depth = np.asarray(depth_mm, dtype=float)
    height, width = depth.shape[:2]
    inside = polygon_mask(scale_polygon(polygon_px, image_size, (width, height)), (height, width))
    valid = inside & np.isfinite(depth) & (depth > 0.0)
    rows, cols = np.nonzero(valid)
    n_px = len(rows)
    if n_px < max(cfg.min_valid_px, MIN_PLANE_POINTS):
        return PlaneFit(None, REASON_TOO_FEW_PX, n_px)
    z = depth[rows, cols]
    k = np.asarray(depth_K, dtype=float)
    points = np.column_stack([(cols - k[0, 2]) * z / k[0, 0], (rows - k[1, 2]) * z / k[1, 1], z])
    centroid = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1] / np.linalg.norm(vt[-1])
    if normal @ centroid > 0.0:
        normal = -normal
    residual_mm = float(singular[-1] / math.sqrt(n_px))
    if residual_mm > cfg.max_residual_mm:
        return PlaneFit(None, REASON_RESIDUAL, n_px, residual_mm)
    plane = Plane(normal=normal, distance_mm=float(-(normal @ centroid)), residual_mm=residual_mm, n_px=n_px)
    return PlaneFit(plane, None, n_px, residual_mm)


def fit_plane(
    depth_mm: np.ndarray,
    depth_K: np.ndarray,  # noqa: N803
    polygon_px: np.ndarray,
    image_size: tuple[int, int],
    cfg: TofConfig,
) -> Plane | None:
    """``fit_plane_detailed`` without the reason: the plane, or None when rejected.

    Args:
        depth_mm: (H, W) depth image in mm. depth_K: 3x3 depth intrinsics. polygon_px: (N, 2) polygon in RGB pixels.
        image_size: (width, height) of the RGB image. cfg: ToF thresholds.

    Returns:
        Plane or None (``fit_plane_detailed`` gives the reason).
    """
    return fit_plane_detailed(depth_mm, depth_K, polygon_px, image_size, cfg).plane


def plane_chooser(plane: Plane, cfg: TofConfig) -> Callable[[list[Solution]], Solution | None]:
    """Disambiguation callback for ``pose.estimate_pose``: the solution whose +z agrees with the ToF plane normal.

    Args:
        plane: ToF plane of the marker (normal towards the camera, the marker +z convention).
        cfg: ToF thresholds (``max_angle_deg``).

    Returns:
        Callable taking the IPPE solutions and returning the one whose marker +z (camera frame) is within
        ``max_angle_deg`` of the plane normal when exactly one is, else None.
    """

    def choose(solutions: list[Solution]) -> Solution | None:
        near = [s for s in solutions if angle_between_deg(s.normal, plane.normal) <= cfg.max_angle_deg]
        return near[0] if len(near) == 1 else None

    return choose


def nearest_depth(
    frames: Sequence[tuple[int, np.ndarray]], stamp_ns: int, max_dt_s: float
) -> tuple[int, np.ndarray] | None:
    """The depth frame nearest in time to ``stamp_ns``, when within ``max_dt_s``.

    Args:
        frames: (stamp_ns, depth image) pairs in any order.
        stamp_ns: Stamp of the RGB image in ns.
        max_dt_s: Largest accepted time difference in seconds (inclusive).

    Returns:
        The (stamp_ns, depth image) pair nearest in time, or None when there is none within the limit.
    """
    if not frames:
        return None
    nearest = min(frames, key=lambda frame: abs(frame[0] - stamp_ns))
    return nearest if abs(nearest[0] - stamp_ns) <= round(max_dt_s * NS_PER_S) else None
