"""Relocalisation against a loaded map: refine the pose hint by a candidate search, then judge the match (no ROS).

slam_toolbox's near-region match after ``deserialize_map`` with ``START_AT_GIVEN_POSE`` only searches a 0.8 m window
and +-0.349 rad around the given pose and never rejects a match (design D5), so before the load the latest scan is
scored against the bundle's occupancy grid at every candidate pose around the hint and the best one is handed to
slam_toolbox. Scoring uses the shared scan-to-map agreement rules (``ros2_common.scan_agreement``): an endpoint agrees
when it lands within the hit radius of an occupied cell, endpoints in unknown space are not evaluated. After the load
``LocalisationJudge`` turns the live agreement samples into the verdict (design D6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from ros2_common.maps import Pose2D
from ros2_common.scan_agreement import PreparedMap, agreement, transform_points, yaw_quaternion

TOO_FEW_SAMPLES = "too few agreement samples"
FULL_TURN_DEG = 360.0
STEP_EPSILON = 1e-9

Verdict = Literal["localised", "not_localised", "ambiguous"]


class SearchSettings(BaseModel):
    """Candidate search window and acceptance rules.

    Attributes:
        radius_m: Half-width of the square translation window around the hint, m.
        step_m: Translation step, m.
        yaw_range_deg: Half-range of the yaw window around the hint, deg (180 searches the full turn).
        yaw_step_deg: Yaw step, deg.
        occupied_threshold: Occupancy from which a grid cell is occupied (1..100).
        hit_radius_cells: Distance to an occupied cell within which an endpoint agrees, cells.
        min_points: Minimum evaluated endpoints for a candidate to count.
        min_evaluated_fraction: Minimum fraction of the valid scan endpoints (within ``max_range_m``) that must be
            evaluated (inside the known map) for a candidate to count, so a pose scored on part of the scan is never
            reported as a match.
        max_range_m: Endpoints farther than this from the base are dropped, m.
        candidate_min_ratio: Minimum agreement ratio of the best candidate for ``ok``.
        ambiguity_distance_m: A rival must be farther than this from the best candidate, m ...
        ambiguity_yaw_deg: ... or turned by more than this, deg ...
        ambiguity_margin: ... and score within this ratio margin of the best ...
        ambiguity_min_support: ... and agree with at least this fraction of the best candidate's agreeing endpoints
            (a rival that only has a few endpoints inside the known map scores a high ratio on little evidence) to
            make the result ambiguous.
    """

    model_config = ConfigDict(extra="forbid")

    radius_m: float = Field(default=1.0, gt=0.0)
    step_m: float = Field(default=0.1, gt=0.0)
    yaw_range_deg: float = Field(default=180.0, gt=0.0, le=180.0)
    yaw_step_deg: float = Field(default=5.0, gt=0.0)
    occupied_threshold: int = Field(default=65, gt=0, le=100)
    hit_radius_cells: int = Field(default=1, gt=0)
    min_points: int = Field(default=40, gt=0)
    min_evaluated_fraction: float = Field(default=0.6, gt=0.0, le=1.0)
    max_range_m: float = Field(default=8.0, gt=0.0)
    candidate_min_ratio: float = Field(default=0.5, ge=0.0, le=1.0)
    ambiguity_distance_m: float = Field(default=0.5, gt=0.0)
    ambiguity_yaw_deg: float = Field(default=30.0, gt=0.0)
    ambiguity_margin: float = Field(default=0.05, ge=0.0, le=1.0)
    ambiguity_min_support: float = Field(default=0.5, ge=0.0, le=1.0)


@dataclass(frozen=True)
class SearchResult:
    """Outcome of the candidate search.

    Attributes:
        pose: Best candidate pose; the unchanged hint when too few points were evaluated.
        ratio: Agreement ratio of the best candidate, None when no candidate had enough evaluated points.
        evaluated: Evaluated endpoints of the best candidate (the most seen by any candidate when none had enough).
        agreeing: Agreeing endpoints of the best candidate (0 when none had enough).
        ok: ratio >= candidate_min_ratio and evaluated >= min_points.
        ambiguous: A rival farther than the ambiguity distance or yaw scores within the margin of the best ratio.
        reason: Human readable explanation.
        rival: The best-scoring such rival when ambiguous, else None.
    """

    pose: Pose2D
    ratio: float | None
    evaluated: int
    agreeing: int
    ok: bool
    ambiguous: bool
    reason: str
    rival: Pose2D | None = None


def yaw_difference(a: float, b: float) -> float:
    """Signed angle a - b wrapped to [-pi, pi].

    Args:
        a (float): Angle, rad.
        b (float): Angle, rad.

    Returns:
        float: Wrapped difference, rad.
    """
    return math.remainder(a - b, 2 * math.pi)


def translation_offsets(settings: SearchSettings) -> np.ndarray:
    """Square grid of translation offsets around the hint.

    Args:
        settings (SearchSettings): Search window.

    Returns:
        np.ndarray: (M, 2) dx, dy offsets, m, including (0, 0).
    """
    n = int(math.floor(settings.radius_m / settings.step_m + STEP_EPSILON))
    steps = np.arange(-n, n + 1) * settings.step_m
    dx, dy = np.meshgrid(steps, steps)
    return np.column_stack((dx.ravel(), dy.ravel()))


def yaw_offsets(settings: SearchSettings) -> np.ndarray:
    """Yaw offsets around the hint, without a duplicate when the window covers the full turn.

    Args:
        settings (SearchSettings): Search window.

    Returns:
        np.ndarray: (K,) offsets, rad, including 0.
    """
    n = int(math.floor(settings.yaw_range_deg / settings.yaw_step_deg + STEP_EPSILON))
    degrees = np.arange(-n, n + 1) * settings.yaw_step_deg
    if degrees[-1] - degrees[0] >= FULL_TURN_DEG - STEP_EPSILON:
        degrees = degrees[:-1]
    return np.radians(degrees)


def score_translations(
    prepared: PreparedMap, points_map: np.ndarray, offsets: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Evaluated and agreeing endpoint counts for every translation of map-frame points (shared agreement rules).

    Args:
        prepared (PreparedMap): Map with its hit and known masks.
        points_map (np.ndarray): (N, 2) endpoints already rotated and placed at the reference translation.
        offsets (np.ndarray): (M, 2) translations added to every point, m.

    Returns:
        tuple[np.ndarray, np.ndarray]: (M,) evaluated counts and (M,) agreeing counts.
    """
    grid = prepared.grid
    xs = points_map[None, :, 0] + offsets[:, 0, None]
    ys = points_map[None, :, 1] + offsets[:, 1, None]
    dx = xs - grid.origin_x
    dy = ys - grid.origin_y
    cos_yaw, sin_yaw = np.cos(grid.origin_yaw), np.sin(grid.origin_yaw)
    cols = np.floor((cos_yaw * dx + sin_yaw * dy) / grid.resolution).astype(int)
    rows = np.floor((-sin_yaw * dx + cos_yaw * dy) / grid.resolution).astype(int)
    height, width = grid.cells.shape
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    rows = np.where(inside, rows, 0)
    cols = np.where(inside, cols, 0)
    evaluated = inside & prepared.known_mask[rows, cols]
    agreeing = evaluated & prepared.hit_mask[rows, cols]
    return evaluated.sum(axis=1), agreeing.sum(axis=1)


def search_candidates(
    prepared: PreparedMap, points_base: np.ndarray, hint: Pose2D, settings: SearchSettings
) -> SearchResult:
    """Find the pose around hint where the scan best agrees with the map.

    Every candidate (hint + translation offset, hint yaw + yaw offset) projects the base-frame endpoints into the map
    and counts agreeing endpoints; candidates with fewer than ``min_points`` evaluated endpoints, or fewer than
    ``min_evaluated_fraction`` of the scan, do not count.
    Candidates rank by agreeing count, ties by ratio. The reported score is the shared ``agreement`` at the best pose.

    Args:
        prepared (PreparedMap): Loaded map, prepared with the settings' threshold and hit radius.
        points_base (np.ndarray): (N, 2) scan endpoints in the base frame, m.
        hint (Pose2D): Pose hint (map -> base).
        settings (SearchSettings): Search window and acceptance rules.

    Returns:
        SearchResult: Best pose and its score; the unchanged hint with ``ok`` false when too few points evaluate.
    """
    points = np.asarray(points_base, dtype=float).reshape(-1, 2)
    points = points[np.hypot(points[:, 0], points[:, 1]) <= settings.max_range_m]
    offsets = translation_offsets(settings)
    yaws = np.array([math.remainder(hint.yaw + offset, 2 * math.pi) for offset in yaw_offsets(settings)])
    reference = np.array([hint.x, hint.y])

    evaluated = np.zeros((yaws.size, len(offsets)), dtype=int)
    agreeing = np.zeros_like(evaluated)
    if len(points) >= settings.min_points and prepared.known_cells > 0:
        for index, yaw in enumerate(yaws):
            placed = transform_points(points, (hint.x, hint.y, 0.0), yaw_quaternion(float(yaw)))
            evaluated[index], agreeing[index] = score_translations(prepared, placed, offsets)

    needed = settings.min_evaluated_fraction * len(points) - STEP_EPSILON
    enough = evaluated >= settings.min_points
    valid = enough & (evaluated >= needed)
    if not valid.any():
        most = int(evaluated.max()) if evaluated.size else 0
        if enough.any():
            return SearchResult(
                pose=hint,
                ratio=None,
                evaluated=most,
                agreeing=0,
                ok=False,
                ambiguous=False,
                reason=(
                    f"evaluated {most}/{len(points)} = {most / len(points):.2f} < "
                    f"{settings.min_evaluated_fraction:.2f}; hint kept"
                ),
            )
        return SearchResult(
            pose=hint,
            ratio=None,
            evaluated=most,
            agreeing=0,
            ok=False,
            ambiguous=False,
            reason=f"too few scan points evaluated against the map ({most} < {settings.min_points}); hint kept",
        )

    ratios = np.where(valid, agreeing / np.maximum(evaluated, 1), -1.0)
    flat_agreeing = np.where(valid, agreeing, -1).ravel()
    flat_ratios = ratios.ravel()
    best_flat = int(np.lexsort((-flat_ratios, -flat_agreeing))[0])
    best_yaw_index, best_offset_index = divmod(best_flat, len(offsets))
    best_xy = reference + offsets[best_offset_index]
    best = Pose2D(x=float(best_xy[0]), y=float(best_xy[1]), yaw=float(yaws[best_yaw_index]))

    shared = agreement(transform_points(points, (best.x, best.y, 0.0), yaw_quaternion(best.yaw)), prepared, 1)
    if shared is not None:
        best_ratio, best_evaluated, best_agreeing = shared.ratio, shared.evaluated, shared.agreeing
    else:
        best_ratio = float(flat_ratios[best_flat])
        best_evaluated = int(evaluated.ravel()[best_flat])
        best_agreeing = int(flat_agreeing[best_flat])

    candidate_xy = reference + offsets
    distance = np.hypot(candidate_xy[:, 0] - best.x, candidate_xy[:, 1] - best.y)
    turn = np.abs(np.remainder(yaws - best.yaw + math.pi, 2 * math.pi) - math.pi)
    apart = (distance[None, :] > settings.ambiguity_distance_m) | (
        turn[:, None] > math.radians(settings.ambiguity_yaw_deg)
    )
    supported = agreeing >= settings.ambiguity_min_support * best_agreeing
    rivals = valid & apart & supported & (ratios >= best_ratio - settings.ambiguity_margin)

    ok = best_ratio >= settings.candidate_min_ratio and best_evaluated >= settings.min_points
    reasons = [
        f"best agreement {best_ratio:.2f} over {best_evaluated} points"
        + ("" if ok else f" below {settings.candidate_min_ratio:.2f}")
    ]
    rival: Pose2D | None = None
    if rivals.any():
        rival_flat = int(np.argmax(np.where(rivals, ratios, -1.0).ravel()))
        rival_yaw_index, rival_offset_index = divmod(rival_flat, len(offsets))
        rival_xy = candidate_xy[rival_offset_index]
        rival = Pose2D(x=float(rival_xy[0]), y=float(rival_xy[1]), yaw=float(yaws[rival_yaw_index]))
        reasons.append(
            f"ambiguous: candidate at ({rival.x:.2f}, {rival.y:.2f}, {math.degrees(rival.yaw):.0f} deg) scores "
            f"{flat_ratios[rival_flat]:.2f}, within {settings.ambiguity_margin:.2f} of the best"
        )
    return SearchResult(
        pose=best,
        ratio=best_ratio,
        evaluated=best_evaluated,
        agreeing=best_agreeing,
        ok=ok,
        ambiguous=rival is not None,
        reason="; ".join(reasons),
        rival=rival,
    )


@dataclass
class LocalisationJudge:
    """Median-of-window verdict over live agreement samples after a load (design D6).

    Attributes:
        localised_ratio: Minimum median agreement for ``localised`` (0..1).
        min_samples: Minimum number of samples for any positive verdict (>= 1).
        samples: Agreement ratios added so far.
    """

    localised_ratio: float = 0.6
    min_samples: int = 5
    samples: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Validate the thresholds.

        Raises:
            ValueError: localised_ratio outside [0, 1] or min_samples < 1.
        """
        if not 0.0 <= self.localised_ratio <= 1.0:
            raise ValueError(f"localised_ratio must be in [0, 1], got {self.localised_ratio}")
        if self.min_samples < 1:
            raise ValueError(f"min_samples must be >= 1, got {self.min_samples}")

    def add(self, ratio: float) -> None:
        """Record one agreement sample.

        Args:
            ratio (float): Agreement ratio in [0, 1].

        Raises:
            ValueError: ratio is not a number in [0, 1].
        """
        if not 0.0 <= ratio <= 1.0:
            raise ValueError(f"agreement ratio must be in [0, 1], got {ratio}")
        self.samples.append(float(ratio))

    def verdict(self, ambiguous: bool) -> tuple[Verdict, float | None, str]:
        """Judge the samples so far.

        Args:
            ambiguous (bool): The candidate search found a rival pose (``SearchResult.ambiguous``).

        Returns:
            tuple[Verdict, float | None, str]: State, median ratio (None without samples) and reason.
        """
        median = float(np.median(self.samples)) if self.samples else None
        if median is None or len(self.samples) < self.min_samples:
            return "not_localised", median, TOO_FEW_SAMPLES
        if median < self.localised_ratio:
            return "not_localised", median, f"median agreement {median:.2f} below {self.localised_ratio:.2f}"
        if ambiguous:
            return "ambiguous", median, f"median agreement {median:.2f} but another pose fits the map as well"
        return "localised", median, f"median agreement {median:.2f} over {len(self.samples)} samples"
