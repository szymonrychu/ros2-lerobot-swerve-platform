"""Marker pose from the 24 refined finder corners: IPPE, LM refinement and an ambiguity check (design D9).

Marker frame (design D3): origin at the symbol centre, +x along the top edge (left to right as read), +y along the left
edge towards the top, +z out of the printed face. The declared side length S (outer finder edge to the opposite outer
edge, quiet zone excluded) sets the scale: module edge = S / (17 + 4 * version).

Finder point order (``finder_model_points``, ``predict_image_points``): finders top-left, top-right, bottom-left; per
finder the outer 7x7 square (module offsets 0 and 7) then the inner 3x3 square (offsets 2 and 5); per square the corners
top-left, top-right, bottom-right, bottom-left. Index = 8 * finder + 4 * (0 outer, 1 inner) + corner.

Pipeline: the homography of the four decoder corners predicts the 24 finder corners; ``refine_corners``
(``cv2.cornerSubPix``) refines them; ``cv2.solvePnPGeneric(SOLVEPNP_IPPE)`` gives both planar solutions, each refined
with ``cv2.solvePnPRefineLM``; corners that converged onto the wrong structure are dropped (``solve_robust``); a second
pass restarts the refinement from the best solution's reprojection (``refine_and_solve``).

Acceptance: the best (lowest RMS) IPPE solution is accepted when the second solution's RMS is at least
``ambiguity_ratio`` times the best's, or when both solutions' normals agree within ``SAME_POSE_DEG`` (a near-frontal
view, where the planar flip barely changes the orientation and the RMS ratio is ~1 by construction). Otherwise the
optional chooser (the ToF plane) settles it, else the pose is rejected ``ambiguous``.

Confidence of an accepted pose, in (0, 1], is the product of three terms, each in [0.5, 1] for an accepted pose:

- reprojection: ``1 - 0.5 * min(rms_px / max_rms_px, 1)`` (1 at zero error, 0.5 at the RMS limit);
- ambiguity: ``max(min(1, ratio / (2 * ambiguity_ratio)), 1 - 0.5 * min(separation_deg / SAME_POSE_DEG, 1))``
  (0.5 at the ratio threshold, 1 at twice it; 1 when both solutions coincide; a pose settled by the chooser gets 0.5);
- module size: ``min(1, module_px / (2 * min_module_px))`` (0.5 at the gate, 1 at twice it).
"""

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import cv2
import numpy as np

from marker_detector.config import PoseConfig
from marker_detector.decode import MODULES_PER_VERSION, SYMBOL_BASE_MODULES, Candidate

REASON_AMBIGUOUS = "ambiguous"
REASON_RMS = "rms"
REASON_MODULE_TOO_SMALL = "module_too_small"
REASON_BEHIND_CAMERA = "behind_camera"
REASON_PNP_FAILED = "pnp_failed"
FINDER_SIZE = 7
FINDER_ORIGINS = ((0, 0), (1, 0), (0, 1))  # (col, row) per finder: 0 = symbol start, 1 = symbol end - FINDER_SIZE
SQUARE_OFFSETS = ((0, 7), (2, 5))  # outer 7x7 edge, inner 3x3 square, in modules from the finder origin
SUBPIX_UPSAMPLE = 4
MIN_WINDOW_PX = 2
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
SAME_POSE_DEG = 10.0
MIN_RMS_PX = 1e-9
OUTLIER_FACTOR = 3.0
OUTLIER_FLOOR_PX = 0.3
OUTLIER_ROUNDS = 3
MIN_INLIERS = 12
REFINE_PASSES = 2


@dataclass
class Solution:
    """One IPPE solution after LM refinement.

    rvec, tvec: (3,) rotation vector and translation of the marker frame in the camera frame (metres).
    rms_px: RMS reprojection error of the (inlier) finder corners in pixels.
    """

    rvec: np.ndarray
    tvec: np.ndarray
    rms_px: float

    @property
    def normal(self) -> np.ndarray:
        """Marker +z axis in the camera frame, shape (3,), unit length."""
        rotation, _ = cv2.Rodrigues(np.asarray(self.rvec, dtype=float))
        return rotation[:, 2]


@dataclass
class PoseResult:
    """Outcome of ``estimate_pose``.

    ok: pose accepted. reason: rejection category (``ambiguous``, ``rms``, ``module_too_small``, ``behind_camera``,
    ``pnp_failed``) or None when ok. rvec, tvec: (3,) accepted pose of the marker frame in the camera frame, None when
    rejected. rms_px: RMS reprojection error of the accepted (or best) solution, nan before PnP. ratio: second RMS over
    best RMS, nan before PnP, inf with a single solution. separation_deg: angle between the two solutions' normals
    (0 with a single solution, nan before PnP). confidence: in (0, 1] when ok (module docstring), 0 when rejected.
    solutions: every refined solution, best RMS first (empty before PnP). inliers: finder corners kept by the outlier
    test (0 before PnP).
    """

    ok: bool
    reason: str | None = None
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    rms_px: float = math.nan
    ratio: float = math.nan
    separation_deg: float = math.nan
    confidence: float = 0.0
    solutions: list[Solution] = field(default_factory=list)
    inliers: int = 0


def symbol_modules(version: int) -> int:
    """Module count along one symbol edge for a QR version (21 for version 1)."""
    return SYMBOL_BASE_MODULES + MODULES_PER_VERSION * version


def quad_is_valid(corners: np.ndarray) -> bool:
    """True when ``corners`` is a finite, convex quadrilateral with positive area (a usable symbol outline).

    Args:
        corners: (4, 2) symbol corners in pixels.

    Returns:
        Whether a homography from the symbol square to ``corners`` is well defined.
    """
    quad = np.asarray(corners, dtype=float)
    if quad.shape != (4, 2) or not np.all(np.isfinite(quad)):
        return False
    contour = quad.astype(np.float32).reshape(-1, 1, 2)
    return bool(cv2.contourArea(contour) > 0.0 and cv2.isContourConvex(contour))


def finder_module_points(version: int) -> np.ndarray:
    """The 24 finder corners in module coordinates (col right, row down, origin at the symbol's top-left corner).

    Args:
        version: QR version.

    Returns:
        (24, 2) float (col, row) in the documented finder point order.
    """
    end = symbol_modules(version) - FINDER_SIZE
    points = []
    for col_end, row_end in FINDER_ORIGINS:
        col0, row0 = col_end * end, row_end * end
        for low, high in SQUARE_OFFSETS:
            points += [(col0 + low, row0 + low), (col0 + high, row0 + low)]
            points += [(col0 + high, row0 + high), (col0 + low, row0 + high)]
    return np.array(points, dtype=float)


def finder_model_points(size_mm: float, version: int) -> np.ndarray:
    """The 24 finder corners in the marker frame (design D3), z = 0.

    Args:
        size_mm: Declared symbol edge S in mm.
        version: QR version.

    Returns:
        (24, 3) float metres in the documented finder point order.
    """
    modules = symbol_modules(version)
    module_m = size_mm / 1000.0 / modules
    grid = finder_module_points(version)
    x = (grid[:, 0] - modules / 2.0) * module_m
    y = (modules / 2.0 - grid[:, 1]) * module_m
    return np.column_stack([x, y, np.zeros(len(grid))])


def predict_image_points(corners: np.ndarray, version: int) -> np.ndarray:
    """Predict the 24 finder corners in the image from the four symbol corners.

    Args:
        corners: (4, 2) symbol corners in pixels, top-left, top-right, bottom-right, bottom-left (symbol orientation).
        version: QR version.

    Returns:
        (24, 2) float pixels in the documented finder point order (nan when the corners give no homography).
    """
    modules = symbol_modules(version)
    square = np.array([[0, 0], [modules, 0], [modules, modules], [0, modules]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(square, np.asarray(corners, dtype=np.float32))
    if not np.all(np.isfinite(homography)):
        return np.full((len(SQUARE_OFFSETS) * 4 * len(FINDER_ORIGINS), 2), np.nan)
    grid = finder_module_points(version).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(grid, homography.astype(float)).reshape(-1, 2)


def refine_corners(grey: np.ndarray, points: np.ndarray, module_px: float) -> np.ndarray:
    """Refine corner positions with ``cv2.cornerSubPix``.

    Each point is refined on a 4x bicubic upsampled patch around it, so small modules still get a window that stays
    inside one module: half window module_px / 2 original pixels, at least 2 pixels of the upsampled patch.
    Termination: 30 iterations or 0.01 px.

    Args:
        grey: Grey uint8 image.
        points: (N, 2) initial corner positions in pixels (pixel centres at integers).
        module_px: Module edge in pixels.

    Returns:
        (N, 2) float refined positions.
    """
    scale = SUBPIX_UPSAMPLE
    half = max(MIN_WINDOW_PX, round(module_px / 2.0 * scale))
    radius = math.ceil(module_px) + 3
    side = 2 * radius + 1
    refined = []
    for x, y in np.asarray(points, dtype=float):
        centre = (round(x), round(y))
        patch = cv2.getRectSubPix(grey, (side, side), (float(centre[0]), float(centre[1])))
        big = cv2.resize(patch, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        origin = np.array(centre, dtype=float) - radius
        start = ((np.array([x, y]) - origin + 0.5) * scale - 0.5).astype(np.float32).reshape(1, 1, 2)
        cv2.cornerSubPix(big, start, (half, half), (-1, -1), SUBPIX_CRITERIA)
        refined.append((start.reshape(2).astype(float) + 0.5) / scale - 0.5 + origin)
    return np.array(refined, dtype=float).reshape(-1, 2)


def reprojection_rms(
    model: np.ndarray,
    image: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
) -> float:
    """RMS pixel distance between ``image`` and the projection of ``model`` under (rvec, tvec).

    Args:
        model: (N, 3) marker-frame points. image: (N, 2) observed pixels. rvec, tvec: pose. K: 3x3 intrinsics.
        D: distortion coefficients or None.

    Returns:
        RMS reprojection error in pixels.
    """
    projected, _ = cv2.projectPoints(model, rvec, tvec, K, D)
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image) ** 2, axis=1))))


def solve_ippe(
    model: np.ndarray,
    image: np.ndarray,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
) -> list[Solution]:
    """Both IPPE solutions, each LM-refined, sorted by RMS (best first); empty when PnP fails.

    Args:
        model: (N, 3) coplanar marker-frame points (N >= 4). image: (N, 2) pixels. K: 3x3 intrinsics.
        D: distortion coefficients or None.

    Returns:
        Refined solutions, best RMS first.
    """
    if not np.all(np.isfinite(image)):
        return []
    try:
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(model, image, K, D, flags=cv2.SOLVEPNP_IPPE)
        solutions = []
        for rvec, tvec in zip(rvecs[:count], tvecs[:count], strict=True):
            rvec, tvec = cv2.solvePnPRefineLM(model, image, K, D, rvec.copy(), tvec.copy())
            rvec, tvec = rvec.reshape(3), tvec.reshape(3)
            if np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec)):
                solutions.append(Solution(rvec, tvec, reprojection_rms(model, image, rvec, tvec, K, D)))
    except cv2.error:
        return []
    return sorted(solutions, key=lambda s: s.rms_px)


def point_residuals(
    model: np.ndarray,
    image: np.ndarray,
    solution: Solution,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
) -> np.ndarray:
    """Per-point reprojection distance in pixels, shape (N,), of ``model`` under ``solution`` against ``image``."""
    projected, _ = cv2.projectPoints(model, solution.rvec, solution.tvec, K, D)
    return np.linalg.norm(projected.reshape(-1, 2) - image, axis=1)


def solve_robust(
    model: np.ndarray,
    image: np.ndarray,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
) -> tuple[list[Solution], np.ndarray]:
    """``solve_ippe`` with corners that converged onto the wrong structure dropped.

    A corner is an outlier when its residual under the best solution exceeds both ``OUTLIER_FACTOR`` times the median
    inlier residual and ``OUTLIER_FLOOR_PX``; the solve repeats on the inliers (at most ``OUTLIER_ROUNDS`` times) as
    long as at least ``MIN_INLIERS`` remain. RMS values then cover the inliers only.

    Args:
        model: (N, 3) coplanar marker-frame points. image: (N, 2) pixels. K: 3x3 intrinsics.
        D: distortion coefficients or None.

    Returns:
        (solutions best RMS first, (N,) bool inlier mask); solutions empty when PnP fails.
    """
    keep = np.ones(len(model), dtype=bool)
    solutions = solve_ippe(model, image, K, D)
    for _ in range(OUTLIER_ROUNDS):
        if not solutions:
            break
        residuals = point_residuals(model, image, solutions[0], K, D)
        limit = max(OUTLIER_FACTOR * float(np.median(residuals[keep])), OUTLIER_FLOOR_PX)
        inliers = residuals <= limit
        if inliers.sum() < MIN_INLIERS or np.array_equal(inliers, keep):
            break
        refit = solve_ippe(model[inliers], image[inliers], K, D)
        if not refit:
            break
        solutions, keep = refit, inliers
    return solutions, keep


def refine_and_solve(
    grey: np.ndarray,
    model: np.ndarray,
    predicted: np.ndarray,
    module_px: float,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
) -> tuple[list[Solution], np.ndarray]:
    """Refine the finder corners and solve, ``REFINE_PASSES`` times.

    The first pass starts from the homography prediction; each later pass restarts ``refine_corners`` from the
    reprojection of the best solution, which is consistent across all 24 corners and so much closer than a prediction
    from four decoder corners that may be a pixel off.

    Args:
        grey: Grey uint8 image. model: (24, 3) finder model points. predicted: (24, 2) predicted pixels.
        module_px: module edge in pixels. K: 3x3 intrinsics. D: distortion coefficients or None.

    Returns:
        ``solve_robust`` output of the last successful pass (solutions empty when the first solve fails).
    """
    solutions, keep = solve_robust(model, refine_corners(grey, predicted, module_px), K, D)
    for _ in range(REFINE_PASSES - 1):
        if not solutions:
            break
        start, _ = cv2.projectPoints(model, solutions[0].rvec, solutions[0].tvec, K, D)
        again, again_keep = solve_robust(model, refine_corners(grey, start.reshape(-1, 2), module_px), K, D)
        if not again:
            break
        solutions, keep = again, again_keep
    return solutions, keep


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees between two 3-vectors."""
    cosine = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def pose_confidence(rms_px: float, ratio: float, separation_deg: float, module_px: float, cfg: PoseConfig) -> float:
    """Confidence in (0, 1] of an accepted pose: product of the reprojection, ambiguity and module terms.

    Args:
        rms_px: RMS reprojection error of the accepted solution. ratio: second RMS over best RMS.
        separation_deg: angle between the two solutions' normals. module_px: module edge in pixels.
        cfg: pose thresholds.

    Returns:
        Confidence (see the module docstring for the terms).
    """
    rms_term = 1.0 - 0.5 * min(rms_px / cfg.max_rms_px, 1.0)
    ratio_part = min(1.0, ratio / (2.0 * cfg.ambiguity_ratio))
    agreement_part = 1.0 - 0.5 * min(separation_deg / SAME_POSE_DEG, 1.0)
    module_term = min(1.0, module_px / (2.0 * cfg.min_module_px))
    return rms_term * max(ratio_part, agreement_part) * module_term


def estimate_pose(
    grey: np.ndarray,
    candidate: Candidate,
    K: np.ndarray,  # noqa: N803
    D: np.ndarray | None,  # noqa: N803
    cfg: PoseConfig,
    choose: Callable[[list[Solution]], Solution | None] | None = None,
) -> PoseResult:
    """6-DoF pose of a decoded marker from its 24 refined finder corners (design D9).

    Args:
        grey: Grey uint8 image the candidate was decoded from.
        candidate: Decoded symbol (corners, version, module_px, payload size S).
        K: 3x3 intrinsics of ``grey``.
        D: Distortion coefficients of ``grey`` (or None).
        cfg: Pose thresholds.
        choose: Optional disambiguation (e.g. the ToF ``plane_chooser``), consulted only when the solutions are
            ambiguous (RMS ratio below ``cfg.ambiguity_ratio`` and normals further apart than ``SAME_POSE_DEG``);
            returns one of the solutions or None.

    Returns:
        PoseResult: accepted pose with confidence, or a rejection reason.
    """
    if candidate.module_px < cfg.min_module_px:
        return PoseResult(ok=False, reason=REASON_MODULE_TOO_SMALL)
    if not quad_is_valid(candidate.corners):
        return PoseResult(ok=False, reason=REASON_PNP_FAILED)
    model = finder_model_points(candidate.payload.size_mm, candidate.version)
    predicted = predict_image_points(candidate.corners, candidate.version)
    if not np.all(np.isfinite(predicted)):
        return PoseResult(ok=False, reason=REASON_PNP_FAILED)
    solutions, keep = refine_and_solve(grey, model, predicted, candidate.module_px, K, D)
    if not solutions:
        return PoseResult(ok=False, reason=REASON_PNP_FAILED)
    best = solutions[0]
    if len(solutions) > 1:
        ratio = solutions[1].rms_px / max(best.rms_px, MIN_RMS_PX)
        separation = angle_between_deg(best.normal, solutions[1].normal)
    else:
        ratio, separation = math.inf, 0.0
    inliers = int(keep.sum())
    rejected = PoseResult(False, None, None, None, best.rms_px, ratio, separation, 0.0, solutions, inliers)
    if best.rms_px > cfg.max_rms_px:
        rejected.reason = REASON_RMS
        return rejected
    chosen: Solution | None = best
    if ratio < cfg.ambiguity_ratio and separation >= SAME_POSE_DEG:
        chosen = choose(solutions) if choose is not None else None
    if chosen is None:
        rejected.reason = REASON_AMBIGUOUS
        return rejected
    if chosen.rms_px > cfg.max_rms_px:
        rejected.reason = REASON_RMS
        return rejected
    if chosen.tvec[2] <= 0.0:
        rejected.reason = REASON_BEHIND_CAMERA
        return rejected
    confidence = pose_confidence(chosen.rms_px, ratio, separation, candidate.module_px, cfg)
    return PoseResult(
        True, None, chosen.rvec, chosen.tvec, chosen.rms_px, ratio, separation, confidence, solutions, inliers
    )
