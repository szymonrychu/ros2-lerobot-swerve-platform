"""Approach pose planner (pure, no ROS): where to put the base so the arm reaches a target.

Pipeline (suggest):

1. Target: exactly one of a map point, a remembered object, a POI or a camera pixel (ApproachTarget), resolved to a map
   point by injected lookups (resolve_target). Objects and POIs carry no height, so z_m is required for them.
2. Candidates (generate_seeds): for every base yaw (every yaw_step_deg) and every reachable reach map cell (heading h
   from the pan axis, distance r) at the target height, the pan axis must stand at P = T - r (cos(yaw + h),
   sin(yaw + h)) and the base at B = P - R(yaw) a (a = pan axis in base_link). The expected margin is the cell margin
   minus the pose error: margin - (pose_error_xy_m + rad(pose_error_yaw_deg) |T - B|); cells below
   min_expected_margin_m are dropped and the best cells_per_yaw (by expected margin) kept per yaw.
3. Costmap (score_candidates): the footprint is rasterised once per yaw (footprint_mask, cv2.fillPoly at the costmap
   resolution) and read at the dock pose, at the staging pose (staging_offset_m back along the yaw) and at
   corridor_samples poses evenly between them. A pose is blocked when a footprint cell is lethal (100) or unknown (-1,
   also any cell outside the grid) or its centre cell is at least inscribed (99).
4. Score (score_value): w_margin min(expected, cap) / cap - w_cost mean_cost / 100 - w_travel travel_m
   - w_heading |h| / 180; ranked best first (rank).
5. Hooks: the best verify_top_k are re-checked with an injected point_reach (target in the candidate's base_link;
   failures dropped) and the best path_check_k planned to their staging pose with an injected planner (no path:
   dropped; a found path replaces the straight-line travel by its length plus the staging leg).

Nothing is invented: a missing or stale costmap raises ApproachUnavailable, an unknown object / POI / pixel too, and
with no robot pose travel is not scored and paths are reported unavailable.
"""

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Literal

import cv2
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .config import ApproachSettings
from .models import BasePose
from .reach_map import CellSet
from .topdown import map_to_base

MAP_FRAME = "map"
FULL_TURN_DEG = 360.0
HALF_TURN_DEG = 180.0
# nav2 OccupancyGrid translation of the costmap: 100 lethal, 99 inscribed, -1 unknown, 0..98 free to near-inscribed.
LETHAL_COST = 100
INSCRIBED_COST = 99
UNKNOWN_COST = -1
MAX_COST = 100.0
# Footprint raster: cv2.fillPoly on a grid FOOTPRINT_SUPERSAMPLE times finer than the costmap; a costmap cell belongs
# to the footprint when more than FOOTPRINT_MIN_COVERAGE of it is covered (area-true within 5 % at 2.5 cm).
FOOTPRINT_SUPERSAMPLE = 16
FOOTPRINT_MIN_COVERAGE = 0.5
FILL_SHIFT_BITS = 8
MIN_POLYGON_POINTS = 3
YAW_KEY_DECIMALS = 9

ObjectLookup = Callable[[str], tuple[float, float] | None]
HeightLookup = Callable[[str], float | None]
PixelLookup = Callable[[str, float, float, float], tuple[float, float] | None]
VerifyFn = Callable[[dict[str, float | None]], dict[str, Any]]


class ApproachUnavailable(RuntimeError):
    """Raised when the planner lacks real input (no or stale costmap, unknown target): it never guesses."""


class XyzTarget(BaseModel):
    """A point in the map frame (m); z is the height above the floor."""

    model_config = ConfigDict(extra="forbid")

    x: float
    y: float
    z: float


class PixelTarget(BaseModel):
    """A camera pixel on a surface at a known height above the floor."""

    model_config = ConfigDict(extra="forbid")

    camera: str
    u: float
    v: float
    surface_height_m: float = 0.0


class ApproachTarget(BaseModel):
    """What to reach: exactly one of xyz, object_id, poi_id or pixel; z_m (m above the floor) for object / poi."""

    model_config = ConfigDict(extra="forbid")

    xyz: XyzTarget | None = None
    object_id: str | None = None
    poi_id: str | None = None
    pixel: PixelTarget | None = None
    z_m: float | None = Field(
        default=None,
        description="Target height above the floor for object_id / poi_id (m); optional for a marker poi_id (its z)",
    )

    @model_validator(mode="after")
    def one_form(self) -> "ApproachTarget":
        """Require exactly one target form, z_m for objects and POIs and no z_m where the form has its own height.

        Returns:
            ApproachTarget: The validated target.
        """
        given = [f for f in ("xyz", "object_id", "poi_id", "pixel") if getattr(self, f) is not None]
        if len(given) != 1:
            raise ValueError(f"give exactly one of xyz, object_id, poi_id, pixel (got {given or 'none'})")
        needs_z = given[0] in ("object_id", "poi_id")
        # A poi_id may name a marker POI, which carries its own height: resolve_target checks z_m for the other kinds.
        if given[0] == "object_id" and self.z_m is None:
            raise ValueError("z_m is required for a object_id target: stored objects have no height")
        if not needs_z and self.z_m is not None:
            raise ValueError(f"z_m is only for object_id / poi_id targets; a {given[0]} target carries its own height")
        return self


class TargetPoint(BaseModel):
    """A resolved target in the map frame."""

    x: float
    y: float
    z: float
    frame: str = MAP_FRAME
    source: str


class ReachInfo(BaseModel):
    """The reach map cell a candidate uses."""

    heading_deg: float
    r_m: float
    z_m: float
    pitches_deg: list[float]
    clearance_m: float
    margin_m: float
    expected_margin_m: float


class CostInfo(BaseModel):
    """Global costmap values under the footprint (0..100)."""

    max_cost: int
    mean_cost: float
    staging_max_cost: int
    corridor_max_cost: int | None = Field(description="None with corridor_samples 0")


class PathInfo(BaseModel):
    """Planned path to the staging pose (planning only, nothing moves)."""

    status: Literal["ok", "no_path", "skipped", "unavailable"]
    length_m: float | None = None
    reason: str | None = None

    @model_validator(mode="after")
    def ok_has_length(self) -> "PathInfo":
        """Require a length for a found path.

        Returns:
            PathInfo: The validated result.
        """
        if self.status == "ok" and self.length_m is None:
            raise ValueError("a path with status ok needs length_m")
        return self


class Candidate(BaseModel):
    """One ranked base pose."""

    pose: BasePose
    staging_pose: BasePose
    reach: ReachInfo
    costmap: CostInfo
    verified: dict[str, Any] | None = Field(default=None, description="point_reach at the pose; None = not checked")
    path: PathInfo = PathInfo(status="skipped")
    travel_m: float | None = Field(description="Path or straight-line travel from the robot; None = no robot pose")
    score: float


class RejectedCounts(BaseModel):
    """Why candidates were dropped (each counted once, in the order of the checks)."""

    out_of_reach: int = Field(default=0, description="Candidate yaws with no reachable cell at the target height")
    collision: int = 0
    unknown_space: int = 0
    staging_blocked: int = 0
    corridor_blocked: int = 0
    below_margin: int = 0
    not_verified: int = 0
    no_path: int = 0


class ApproachSuggestion(BaseModel):
    """suggest's result; reach_map and docking are filled in by the tool."""

    target: TargetPoint
    candidates: list[Candidate]
    rejected: RejectedCounts
    costmap_age_s: float
    reach_map: dict[str, Any] | None = None
    docking: dict[str, Any] | None = None
    notes: list[str] = []


PlanFn = Callable[[BasePose], PathInfo]


@dataclass(frozen=True)
class CostGrid:
    """An OccupancyGrid-like costmap: data[row, col], row along +y of the grid origin, col along +x.

    Attributes:
        data (np.ndarray): (H, W) costs, -1 unknown, 0..100 (converted to int16).
        resolution (float): Cell size (m).
        origin_x (float): Map x of the corner of cell (0, 0) (m).
        origin_y (float): Map y of that corner (m).
        age_s (float): Age of the grid (s).
        origin_yaw (float): Rotation of the grid in the map (rad).
    """

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    age_s: float
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        """Convert the data to int16 and check the shape and resolution.

        Raises:
            ValueError: For data that is not 2D or a non-positive resolution.
        """
        data = np.asarray(self.data, dtype=np.int16)
        if data.ndim != 2 or self.resolution <= 0.0:
            raise ValueError(f"costmap needs 2D data and a positive resolution (got {data.shape}, {self.resolution})")
        object.__setattr__(self, "data", data)

    def world_to_cell(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Cells holding map points (may lie outside the grid).

        Args:
            xy (np.ndarray): (N, 2) map points (m).

        Returns:
            tuple[np.ndarray, np.ndarray]: Row and column indices (int).
        """
        d = np.asarray(xy, dtype=np.float64) - (self.origin_x, self.origin_y)
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx = d[:, 0] * c + d[:, 1] * s
        gy = -d[:, 0] * s + d[:, 1] * c
        return np.floor(gy / self.resolution).astype(np.int64), np.floor(gx / self.resolution).astype(np.int64)


@dataclass(frozen=True)
class FootprintMask:
    """A footprint rasterised at one yaw: mask[i, j] is the cell (i + row0, j + col0) relative to the base's cell.

    Attributes:
        mask (np.ndarray): (H, W) bool.
        row0 (int): Row offset of mask row 0.
        col0 (int): Column offset of mask column 0.
    """

    mask: np.ndarray
    row0: int
    col0: int

    def offsets(self) -> tuple[np.ndarray, np.ndarray]:
        """Row and column offsets of the footprint cells.

        Returns:
            tuple[np.ndarray, np.ndarray]: (K,) row offsets, (K,) column offsets.
        """
        rows, cols = np.nonzero(self.mask)
        return rows + self.row0, cols + self.col0


@dataclass(frozen=True)
class PoseCosts:
    """Costmap values under the footprint at N poses.

    Attributes:
        max_cost (np.ndarray): (N,) highest cost (-1 when every cell is unknown).
        mean_cost (np.ndarray): (N,) mean of the known costs (0 when none).
        blocked (np.ndarray): (N,) lethal, unknown / outside or inscribed centre.
        collision (np.ndarray): (N,) lethal cell or inscribed centre.
    """

    max_cost: np.ndarray
    mean_cost: np.ndarray
    blocked: np.ndarray
    collision: np.ndarray


@dataclass(frozen=True)
class Seed:
    """A candidate base pose before the costmap check.

    Attributes:
        x (float): Base map x (m).
        y (float): Base map y (m).
        yaw (float): Base yaw (rad, wrapped).
        heading_deg (float): Cell heading from the pan axis (deg, base_link).
        r_m (float): Cell distance from the pan axis (m).
        z_m (float): Cell height (m).
        pitches (list[float]): Working pitches of the cell (deg).
        clearance (float): Cell body clearance (m).
        margin_m (float): Cell margin (m).
        expected_margin_m (float): Margin left after the pose error (m).
    """

    x: float
    y: float
    yaw: float
    heading_deg: float
    r_m: float
    z_m: float
    pitches: list[float]
    clearance: float
    margin_m: float
    expected_margin_m: float


def resolve_target(
    target: ApproachTarget,
    object_xy: ObjectLookup,
    poi_xy: ObjectLookup,
    pixel_xy: PixelLookup,
    poi_z: HeightLookup | None = None,
) -> TargetPoint:
    """Resolve a target to a map point with the injected lookups.

    Args:
        target (ApproachTarget): The target.
        object_xy (ObjectLookup): object_id -> map (x, y), None when unknown.
        poi_xy (ObjectLookup): poi_id -> map (x, y) (point or area centroid), None when unknown.
        pixel_xy (PixelLookup): (camera, u, v, surface_height_m) -> map (x, y) of the ground point, None when the
            pixel does not hit the surface.
        poi_z (HeightLookup | None): poi_id -> own height (m) of a marker POI, None for POIs without one.

    Returns:
        TargetPoint: The map point and its source.

    Raises:
        ApproachUnavailable: When a lookup has no answer.
    """
    if target.xyz is not None:
        return TargetPoint(x=target.xyz.x, y=target.xyz.y, z=target.xyz.z, source="xyz")
    if target.pixel is not None:
        px = target.pixel
        xy = pixel_xy(px.camera, float(px.u), float(px.v), float(px.surface_height_m))
        if xy is None:
            raise ApproachUnavailable(f"pixel ({px.u:g}, {px.v:g}) of camera {px.camera} does not hit the surface")
        return TargetPoint(x=xy[0], y=xy[1], z=px.surface_height_m, source=f"pixel:{px.camera}")
    if target.object_id is not None:
        assert target.z_m is not None
        xy = object_xy(target.object_id)
        if xy is None:
            raise ApproachUnavailable(f"unknown object {target.object_id!r}")
        return TargetPoint(x=xy[0], y=xy[1], z=target.z_m, source=f"object:{target.object_id}")
    assert target.poi_id is not None
    xy = poi_xy(target.poi_id)
    if xy is None:
        raise ApproachUnavailable(f"unknown POI {target.poi_id!r}")
    z = target.z_m if target.z_m is not None else (poi_z(target.poi_id) if poi_z is not None else None)
    if z is None:
        raise ValueError("z_m is required for a poi_id target: only marker POIs carry a height")
    return TargetPoint(x=xy[0], y=xy[1], z=z, source=f"poi:{target.poi_id}")


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi).

    Args:
        angle (np.ndarray): Angles (rad).

    Returns:
        np.ndarray: Wrapped angles (rad).
    """
    return (np.asarray(angle, dtype=np.float64) + math.pi) % (2.0 * math.pi) - math.pi


def candidate_yaws(step_deg: float) -> np.ndarray:
    """Base yaws 0 .. 360 deg every step, wrapped and without the duplicate 360.

    Args:
        step_deg (float): Step (deg).

    Returns:
        np.ndarray: Yaws (rad).
    """
    return wrap_angle(np.radians(np.arange(0.0, FULL_TURN_DEG - 1e-9, step_deg)))


def invert(target_xy: tuple[float, float], cells: CellSet, yaw: float, pan_axis_xy: tuple[float, float]) -> np.ndarray:
    """Base poses that put each cell on the target at one yaw.

    Args:
        target_xy (tuple[float, float]): Target map (x, y) (m).
        cells (CellSet): Reachable cells (heading from the pan axis in base_link, r).
        yaw (float): Base yaw (rad).
        pan_axis_xy (tuple[float, float]): Pan axis in base_link (m), from the arm mount config.

    Returns:
        np.ndarray: (N, 3) base x, y (m) and wrapped yaw (rad).
    """
    angle = yaw + np.radians(cells.heading_deg)
    px = target_xy[0] - cells.r_m * np.cos(angle)
    py = target_xy[1] - cells.r_m * np.sin(angle)
    c, s = math.cos(yaw), math.sin(yaw)
    bx = px - (c * pan_axis_xy[0] - s * pan_axis_xy[1])
    by = py - (s * pan_axis_xy[0] + c * pan_axis_xy[1])
    return np.column_stack([bx, by, np.full(len(bx), float(wrap_angle(np.array([yaw]))[0]))])


def target_in_base(target: TargetPoint, pose: tuple[float, float, float]) -> tuple[float, float, float]:
    """The target in the base_link frame of a base pose.

    Args:
        target (TargetPoint): Map target.
        pose (tuple[float, float, float]): Base map x, y (m) and yaw (rad).

    Returns:
        tuple[float, float, float]: base_link x, y, z (m).
    """
    x, y = map_to_base(target.x, target.y, pose)
    return x, y, target.z


def staging_pose_of(x: float, y: float, yaw: float, offset_m: float) -> tuple[float, float, float]:
    """The docking server's staging pose: the dock pose moved offset_m back along its yaw.

    Args:
        x (float): Dock x (m).
        y (float): Dock y (m).
        yaw (float): Dock yaw (rad).
        offset_m (float): Staging distance (m).

    Returns:
        tuple[float, float, float]: Staging x, y (m) and yaw (rad).
    """
    return x - offset_m * math.cos(yaw), y - offset_m * math.sin(yaw), yaw


def expected_margin(margin_m: np.ndarray, distance_m: np.ndarray, settings: ApproachSettings) -> np.ndarray:
    """Cell margin left after the base pose error.

    Args:
        margin_m (np.ndarray): Cell margins (m).
        distance_m (np.ndarray): Horizontal target-to-base distances |T - B| (m).
        settings (ApproachSettings): Pose error model.

    Returns:
        np.ndarray: margin - (pose_error_xy_m + rad(pose_error_yaw_deg) * |T - B|) (m).
    """
    return margin_m - (settings.pose_error_xy_m + math.radians(settings.pose_error_yaw_deg) * distance_m)


def generate_seeds(
    target: TargetPoint,
    cells: CellSet,
    pan_axis_xy: tuple[float, float],
    settings: ApproachSettings,
    rejected: RejectedCounts,
) -> list[Seed]:
    """Candidate base poses over every yaw (inverse reachability), best cells_per_yaw per yaw.

    Args:
        target (TargetPoint): Map target.
        cells (CellSet): Reachable cells at the target height (and pitch).
        pan_axis_xy (tuple[float, float]): Pan axis in base_link (m).
        settings (ApproachSettings): Planner settings.
        rejected (RejectedCounts): Counters (out_of_reach, below_margin) updated in place.

    Returns:
        list[Seed]: The seeds.
    """
    yaws = candidate_yaws(settings.yaw_step_deg)
    if len(cells.r_m) == 0:
        rejected.out_of_reach += len(yaws)
        return []
    seeds: list[Seed] = []
    for yaw in yaws:
        poses = invert((target.x, target.y), cells, float(yaw), pan_axis_xy)
        dist = np.hypot(target.x - poses[:, 0], target.y - poses[:, 1])
        em = expected_margin(cells.margin_m, dist, settings)
        ok = np.nonzero(em >= settings.min_expected_margin_m)[0]
        rejected.below_margin += len(em) - len(ok)
        best = ok[np.argsort(-em[ok], kind="stable")][: settings.cells_per_yaw]
        seeds.extend(
            Seed(
                x=float(poses[i, 0]),
                y=float(poses[i, 1]),
                yaw=float(poses[i, 2]),
                heading_deg=float(cells.heading_deg[i]),
                r_m=float(cells.r_m[i]),
                z_m=float(cells.z_m),
                pitches=[float(p) for p in cells.pitches[i]],
                clearance=float(cells.clearance[i]),
                margin_m=float(cells.margin_m[i]),
                expected_margin_m=float(em[i]),
            )
            for i in best
        )
    return seeds


def footprint_mask(points: Sequence[Sequence[float]], yaw: float, resolution: float) -> FootprintMask:
    """Rasterise a base_link footprint polygon at a yaw, relative to the cell holding the base origin.

    The polygon is filled with cv2.fillPoly on a FOOTPRINT_SUPERSAMPLE times finer grid; a cell belongs to the
    footprint when more than FOOTPRINT_MIN_COVERAGE of it is covered. The base origin is taken at its cell centre (up to
    half a cell off).

    Args:
        points (Sequence[Sequence[float]]): Footprint polygon (x, y) in base_link (m).
        yaw (float): Footprint yaw relative to the grid axes (rad).
        resolution (float): Cell size (m).

    Returns:
        FootprintMask: The raster.

    Raises:
        ValueError: For fewer than 3 points or a polygon that covers no cell.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < MIN_POLYGON_POINTS or pts.shape[1] != 2:
        raise ValueError(f"footprint needs at least {MIN_POLYGON_POINTS} (x, y) points, got {pts.shape}")
    c, s = math.cos(yaw), math.sin(yaw)
    cells = np.column_stack([pts[:, 0] * c - pts[:, 1] * s, pts[:, 0] * s + pts[:, 1] * c]) / resolution
    lo = np.floor(cells.min(axis=0) + 0.5).astype(np.int64) - 1
    hi = np.ceil(cells.max(axis=0) + 0.5).astype(np.int64) + 1
    ncols, nrows = (hi - lo + 1).tolist()
    ss = FOOTPRINT_SUPERSAMPLE
    # Cell k spans [k - 0.5, k + 0.5) in cell units; subpixel q of mask column j = k - lo has its centre at q.
    pixels = (cells - lo + 0.5) * ss - 0.5
    fine = np.zeros((nrows * ss, ncols * ss), dtype=np.uint8)
    cv2.fillPoly(fine, [np.round(pixels * (1 << FILL_SHIFT_BITS)).astype(np.int32)], 1, cv2.LINE_8, FILL_SHIFT_BITS)
    coverage = fine.reshape(nrows, ss, ncols, ss).mean(axis=(1, 3))
    mask = coverage > FOOTPRINT_MIN_COVERAGE
    if not mask.any():
        raise ValueError("footprint polygon covers no costmap cell")
    return FootprintMask(mask=mask, row0=int(lo[1]), col0=int(lo[0]))


def check_grid(grid: CostGrid | None, settings: ApproachSettings) -> CostGrid:
    """Refuse a missing or stale costmap.

    Args:
        grid (CostGrid | None): The latest global costmap, None when none was received.
        settings (ApproachSettings): costmap_stale_s and the topic name.

    Returns:
        CostGrid: The grid, when fresh.

    Raises:
        ApproachUnavailable: With the reason.
    """
    if grid is None:
        raise ApproachUnavailable(f"no global costmap received on {settings.global_costmap_topic}")
    if grid.age_s > settings.costmap_stale_s:
        raise ApproachUnavailable(
            f"global costmap on {settings.global_costmap_topic} is stale: {grid.age_s:.1f} s old "
            f"(limit {settings.costmap_stale_s:g} s)"
        )
    return grid


def footprint_costs(grid: CostGrid, mask: FootprintMask, xy: np.ndarray) -> PoseCosts:
    """Costs under the footprint at N poses of the mask's yaw.

    Args:
        grid (CostGrid): Costmap.
        mask (FootprintMask): Footprint raster at the poses' yaw.
        xy (np.ndarray): (N, 2) base map positions (m).

    Returns:
        PoseCosts: Per-pose max / mean cost and blocked / collision flags.
    """
    height, width = grid.data.shape
    rows0, cols0 = grid.world_to_cell(xy)
    drow, dcol = mask.offsets()

    def values(rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        cells = grid.data[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)]
        return np.where(inside, cells, UNKNOWN_COST)

    vals = values(rows0[:, None] + drow[None, :], cols0[:, None] + dcol[None, :])
    centre = values(rows0, cols0)
    known = vals >= 0
    collision = (vals >= LETHAL_COST).any(axis=1) | (centre >= INSCRIBED_COST)
    unknown = (~known).any(axis=1) | (centre < 0)
    mean = np.where(known, vals, 0).sum(axis=1) / np.maximum(known.sum(axis=1), 1)
    return PoseCosts(max_cost=vals.max(axis=1), mean_cost=mean, blocked=collision | unknown, collision=collision)


def score_value(
    expected_margin_m: float, mean_cost: float, travel_m: float | None, heading_deg: float, settings: ApproachSettings
) -> float:
    """Weighted candidate score (higher is better).

    Args:
        expected_margin_m (float): Margin after the pose error (m); saturates at margin_cap_m.
        mean_cost (float): Mean footprint cost (0..100).
        travel_m (float | None): Path or straight-line travel (m); None (no robot pose) adds nothing.
        heading_deg (float): Cell heading from the pan axis (deg); 0 is straight ahead.
        settings (ApproachSettings): Weights and margin cap.

    Returns:
        float: w_margin min(em, cap) / cap - w_cost cost / 100 - w_travel travel - w_heading |h| / 180.
    """
    w = settings.weights
    cap = settings.margin_cap_m
    return (
        w.margin * min(expected_margin_m, cap) / cap
        - w.cost * mean_cost / MAX_COST
        - w.travel * (travel_m or 0.0)
        - w.heading * abs(heading_deg) / HALF_TURN_DEG
    )


def score_candidates(
    grid: CostGrid,
    footprint_points: Sequence[Sequence[float]],
    seeds: list[Seed],
    settings: ApproachSettings,
    robot_xy: tuple[float, float] | None,
    rejected: RejectedCounts,
) -> list[Candidate]:
    """Costmap-check the seeds at the dock, staging and corridor poses and score the survivors.

    A seed is counted once, in this order: collision (dock footprint lethal or centre inscribed), unknown_space
    (dock footprint unknown or outside the grid), staging_blocked, corridor_blocked.

    Args:
        grid (CostGrid): Global costmap.
        footprint_points (Sequence[Sequence[float]]): Current footprint polygon in base_link (m).
        seeds (list[Seed]): Candidate poses.
        settings (ApproachSettings): Planner settings.
        robot_xy (tuple[float, float] | None): Robot map position for the travel term, None when unknown.
        rejected (RejectedCounts): Counters updated in place.

    Returns:
        list[Candidate]: The accepted candidates (unranked).
    """
    groups: dict[float, list[Seed]] = {}
    for seed in seeds:
        groups.setdefault(round(seed.yaw, YAW_KEY_DECIMALS), []).append(seed)
    n_corr = settings.corridor_samples
    fractions = np.arange(1, n_corr + 1) / (n_corr + 1)
    out: list[Candidate] = []
    for yaw, group in groups.items():
        mask = footprint_mask(footprint_points, yaw - grid.origin_yaw, grid.resolution)
        dock = np.array([(s.x, s.y) for s in group])
        back = settings.staging_offset_m * np.array([math.cos(yaw), math.sin(yaw)])
        staging = dock - back
        d = footprint_costs(grid, mask, dock)
        st = footprint_costs(grid, mask, staging)
        corr_blocked = np.zeros(len(group), dtype=bool)
        corr_max: np.ndarray | None = None
        if n_corr:
            corridor = (staging[None, :, :] + fractions[:, None, None] * back[None, None, :]).reshape(-1, 2)
            co = footprint_costs(grid, mask, corridor)
            corr_blocked = co.blocked.reshape(n_corr, len(group)).any(axis=0)
            corr_max = co.max_cost.reshape(n_corr, len(group)).max(axis=0)
        for i, seed in enumerate(group):
            if d.collision[i]:
                rejected.collision += 1
            elif d.blocked[i]:
                rejected.unknown_space += 1
            elif st.blocked[i]:
                rejected.staging_blocked += 1
            elif corr_blocked[i]:
                rejected.corridor_blocked += 1
            else:
                out.append(make_candidate(seed, staging[i], d, st, corr_max, i, robot_xy, settings))
    return out


def make_candidate(
    seed: Seed,
    staging: np.ndarray,
    dock_costs: PoseCosts,
    staging_costs: PoseCosts,
    corridor_max: np.ndarray | None,
    index: int,
    robot_xy: tuple[float, float] | None,
    settings: ApproachSettings,
) -> Candidate:
    """Build a scored candidate from an accepted seed.

    Args:
        seed (Seed): The seed.
        staging (np.ndarray): Staging map (x, y) (m).
        dock_costs (PoseCosts): Dock pose costs of the seed's yaw group.
        staging_costs (PoseCosts): Staging pose costs of the group.
        corridor_max (np.ndarray | None): Corridor max cost per group seed, None without corridor samples.
        index (int): The seed's index in the group.
        robot_xy (tuple[float, float] | None): Robot map position, None when unknown.
        settings (ApproachSettings): Planner settings.

    Returns:
        Candidate: The candidate with its straight-line travel and score.
    """
    travel = None if robot_xy is None else math.hypot(seed.x - robot_xy[0], seed.y - robot_xy[1])
    mean_cost = float(dock_costs.mean_cost[index])
    return Candidate(
        pose=BasePose(frame=MAP_FRAME, x=seed.x, y=seed.y, yaw=seed.yaw),
        staging_pose=BasePose(frame=MAP_FRAME, x=float(staging[0]), y=float(staging[1]), yaw=seed.yaw),
        reach=ReachInfo(
            heading_deg=seed.heading_deg,
            r_m=seed.r_m,
            z_m=seed.z_m,
            pitches_deg=seed.pitches,
            clearance_m=seed.clearance,
            margin_m=seed.margin_m,
            expected_margin_m=seed.expected_margin_m,
        ),
        costmap=CostInfo(
            max_cost=int(dock_costs.max_cost[index]),
            mean_cost=mean_cost,
            staging_max_cost=int(staging_costs.max_cost[index]),
            corridor_max_cost=None if corridor_max is None else int(corridor_max[index]),
        ),
        travel_m=travel,
        score=score_value(seed.expected_margin_m, mean_cost, travel, seed.heading_deg, settings),
    )


def rank(candidates: list[Candidate]) -> list[Candidate]:
    """Best score first (stable for ties).

    Args:
        candidates (list[Candidate]): Candidates.

    Returns:
        list[Candidate]: Sorted copy.
    """
    return sorted(candidates, key=lambda c: -c.score)


def verify_top_k(
    candidates: list[Candidate],
    target: TargetPoint,
    k: int,
    pitch_deg: float | None,
    verify: VerifyFn,
    rejected: RejectedCounts,
) -> list[Candidate]:
    """Re-check the best k candidates with point_reach; drop and count the failures, keep the rest unverified.

    Args:
        candidates (list[Candidate]): Ranked candidates.
        target (TargetPoint): Map target.
        k (int): How many to check.
        pitch_deg (float | None): Requested approach pitch (deg), None for any.
        verify (VerifyFn): point_reach on {x, y, z (base_link, m), pitch (rad) | None}; the result's reachable and
            collision_free decide.
        rejected (RejectedCounts): not_verified updated in place.

    Returns:
        list[Candidate]: The ranked survivors; checked ones carry verified {reachable, collision_free,
        pitch_range_deg}.
    """
    pitch = None if pitch_deg is None else math.radians(pitch_deg)
    out: list[Candidate] = []
    for i, cand in enumerate(candidates):
        if i >= k:
            out.append(cand)
            continue
        x, y, z = target_in_base(target, (cand.pose.x, cand.pose.y, cand.pose.yaw))
        result = verify({"x": x, "y": y, "z": z, "pitch": pitch})
        if result.get("reachable") and result.get("collision_free"):
            verified = {f: result.get(f) for f in ("reachable", "collision_free", "pitch_range_deg")}
            out.append(cand.model_copy(update={"verified": verified}))
        else:
            rejected.not_verified += 1
    return out


def path_check(
    candidates: list[Candidate], k: int, plan: PlanFn, settings: ApproachSettings, rejected: RejectedCounts
) -> list[Candidate]:
    """Plan to the staging pose of the best k candidates (planning only) and re-rank.

    no_path drops the candidate (counted); ok replaces the travel by the path length plus the staging leg and
    re-scores; unavailable keeps the candidate as it was, with that status. Candidates past k stay "skipped".

    Args:
        candidates (list[Candidate]): Ranked candidates.
        k (int): How many to plan.
        plan (PlanFn): Planner on the staging pose.
        settings (ApproachSettings): Weights and staging offset.
        rejected (RejectedCounts): no_path updated in place.

    Returns:
        list[Candidate]: Re-ranked survivors.
    """
    out: list[Candidate] = []
    for i, cand in enumerate(candidates):
        if i >= k:
            out.append(cand)
            continue
        info = plan(cand.staging_pose)
        if info.status == "no_path":
            rejected.no_path += 1
            continue
        update: dict[str, Any] = {"path": info}
        if info.status == "ok":
            assert info.length_m is not None
            travel = info.length_m + settings.staging_offset_m
            update |= {
                "travel_m": travel,
                "score": score_value(
                    cand.reach.expected_margin_m, cand.costmap.mean_cost, travel, cand.reach.heading_deg, settings
                ),
            }
        out.append(cand.model_copy(update=update))
    return rank(out)


def suggest(
    target: TargetPoint,
    cells: CellSet,
    pan_axis_xy: tuple[float, float],
    footprint_points: Sequence[Sequence[float]],
    grid: CostGrid | None,
    settings: ApproachSettings,
    robot_xy: tuple[float, float] | None,
    pitch_deg: float | None = None,
    max_results: int = 5,
    verify: VerifyFn | None = None,
    plan: PlanFn | None = None,
) -> ApproachSuggestion:
    """Ranked approach poses for a target (see the module docstring).

    Args:
        target (TargetPoint): Resolved map target.
        cells (CellSet): ReachMap.cells(target.z, pitch_deg).
        pan_axis_xy (tuple[float, float]): Pan axis in base_link (m).
        footprint_points (Sequence[Sequence[float]]): Current footprint polygon in base_link (m).
        grid (CostGrid | None): Latest global costmap, None when none was received.
        settings (ApproachSettings): Planner settings.
        robot_xy (tuple[float, float] | None): Robot map position, None when unknown.
        pitch_deg (float | None): Requested approach pitch (deg) for the verification, None for any.
        max_results (int): Candidates returned.
        verify (VerifyFn | None): point_reach hook, None to skip the verification.
        plan (PlanFn | None): Path planner hook, None to skip path checks.

    Returns:
        ApproachSuggestion: Ranked candidates, rejection counters and notes.

    Raises:
        ApproachUnavailable: When the costmap is missing or stale.
    """
    grid = check_grid(grid, settings)
    rejected = RejectedCounts()
    notes: list[str] = []
    seeds = generate_seeds(target, cells, pan_axis_xy, settings, rejected)
    if not len(cells.r_m):
        pitch = "" if pitch_deg is None else f" at pitch {pitch_deg:g} deg"
        notes.append(f"out_of_reach: no reachable reach map cell at z {target.z:g} m{pitch}")
    candidates = rank(score_candidates(grid, footprint_points, seeds, settings, robot_xy, rejected))
    if verify is not None:
        candidates = verify_top_k(candidates, target, settings.verify_top_k, pitch_deg, verify, rejected)
    else:
        notes.append("not verified: no point_reach check was run")
    if robot_xy is None:
        notes.append("no robot pose: travel is not scored and paths are not planned")
        if plan is not None:
            unavailable = PathInfo(status="unavailable", reason="no robot pose")
            candidates = [
                c.model_copy(update={"path": unavailable}) if i < settings.path_check_k else c
                for i, c in enumerate(candidates)
            ]
    elif plan is not None:
        candidates = path_check(candidates, settings.path_check_k, plan, settings, rejected)
    else:
        notes.append("paths not checked")
    if not candidates:
        notes.append("no candidate passed the checks; see rejected for the reasons")
    return ApproachSuggestion(
        target=target,
        candidates=candidates[:max_results],
        rejected=rejected,
        costmap_age_s=grid.age_s,
        notes=notes,
    )
