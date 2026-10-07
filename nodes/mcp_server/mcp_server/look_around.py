"""look_around: rotate in place in equal steps, capture a camera frame and the lidar summary at every stop (no ROS)."""

import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import FootprintSettings, LookAroundSettings
from .geometry import normalize_angle
from .models import MapSummary, NavigationResult, RobotError, SectorObstacle
from .perception import encode_jpeg
from .perception_models import HeadingSummary, LookAroundResult, StepRecord
from .tool_context import RobotApi

TILE_BG = 90
TILE_LABEL_H = 18
TILE_TEXT_COLOR = (255, 255, 255)
SCAN_RADIUS_M = 1.0  # the PNG crop is not requested; the radius is a required argument only
SCAN_PNG_PX = 32


@dataclass(frozen=True)
class LookPlan:
    """Equal rotation steps covering 360 degrees."""

    captures: int
    step_rad: float
    headings_rad: list[float]  # relative heading of every stop, counter-clockwise from the start heading

    @property
    def total_rad(self) -> float:
        """Total rotation including the closing step back to the start heading.

        Returns:
            float: 2 * pi.
        """
        return self.step_rad * self.captures


@dataclass
class LookAroundRun:
    """Outcome of run_look_around: the result model plus the raw frames (label, JPEG or None when missing)."""

    result: LookAroundResult
    frames: list[tuple[str, bytes | None]] = field(default_factory=list)


def plan_look_around(captures: int, min_captures: int = 3, max_captures: int = 12) -> LookPlan:
    """Plan the stops of a full turn.

    Args:
        captures (int): Number of stops (equal steps of 360 / captures degrees).
        min_captures (int): Smallest accepted number of stops (steps stay below 180 degrees).
        max_captures (int): Largest accepted number of stops.

    Returns:
        LookPlan: Step size and relative headings of the stops.
    """
    if not min_captures <= captures <= max_captures:
        raise ValueError(f"captures must be between {min_captures} and {max_captures}")
    step = 2 * math.pi / captures
    return LookPlan(captures=captures, step_rad=step, headings_rad=[i * step for i in range(captures)])


def required_clearance_m(length_m: float, width_m: float, margin_m: float) -> float:
    """Free radius needed to spin the footprint in place: circumscribed radius plus a margin.

    Args:
        length_m (float): Footprint length (m).
        width_m (float): Footprint width (m).
        margin_m (float): Extra clearance (m).

    Returns:
        float: Required distance from base_link to the nearest obstacle (m).
    """
    return math.hypot(length_m / 2, width_m / 2) + margin_m


def check_clearance(obstacles: list[SectorObstacle] | None, required_m: float) -> str | None:
    """Refusal reason when the lidar shows an obstacle inside the rotation circle (or no lidar at all).

    Args:
        obstacles (list[SectorObstacle] | None): Nearest return per sector, None when the scan is unavailable.
        required_m (float): Free radius needed (m).

    Returns:
        str | None: Reason to refuse, None when the robot may rotate.
    """
    if obstacles is None:
        return "lidar scan missing or stale: cannot verify the rotation clearance"
    close = [o for o in obstacles if o.nearest_m is not None and o.nearest_m < required_m]
    if not close:
        return None
    worst = min(close, key=lambda o: o.nearest_m or 0.0)
    return (
        f"obstacle too close for an in-place rotation: {worst.nearest_m:.2f} m in sector {worst.sector} "
        f"(needs {required_m:.2f} m = footprint circumscribed radius + margin)"
    )


def heading_label(heading_rad: float) -> str:
    """Label of a stop: degrees turned counter-clockwise from the start heading.

    Args:
        heading_rad (float): Relative heading (rad).

    Returns:
        str: "0 deg (start)" or "+90 deg".
    """
    deg = round(math.degrees(heading_rad)) % 360
    return "0 deg (start)" if deg == 0 else f"+{deg} deg"


def sector_summary(
    index: int, heading_rad: float, obstacles: list[SectorObstacle] | None, captured: bool
) -> HeadingSummary:
    """Per-heading lidar summary.

    Args:
        index (int): Stop number.
        heading_rad (float): Relative heading (rad).
        obstacles (list[SectorObstacle] | None): Sector returns, None when the scan was unavailable.
        captured (bool): Whether a camera frame was taken.

    Returns:
        HeadingSummary: Nearest obstacle overall and per sector.
    """
    sectors = {o.sector: o.nearest_m for o in obstacles or []}
    seen = [o for o in obstacles or [] if o.nearest_m is not None]
    nearest = min(seen, key=lambda o: o.nearest_m or 0.0) if seen else None
    return HeadingSummary(
        index=index,
        heading_deg=round(math.degrees(heading_rad)),
        label=heading_label(heading_rad),
        nearest_m=nearest.nearest_m if nearest else None,
        nearest_sector=nearest.sector if nearest else None,
        sectors=sectors,
        image_captured=captured,
    )


def build_montage(tiles: list[tuple[str, bytes | None]], tile_width: int) -> np.ndarray:
    """Grid of labelled camera frames; a missing frame is a flat grey tile saying so (nothing is invented).

    Args:
        tiles (list[tuple[str, bytes | None]]): (label, JPEG or None) per stop.
        tile_width (int): Width of one tile (pixels).

    Returns:
        np.ndarray: BGR montage.
    """
    decoded = [
        (label, None if data is None else cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR))
        for label, data in tiles
    ]
    ratios = [img.shape[0] / img.shape[1] for _, img in decoded if img is not None]
    tile_h = round(tile_width * (ratios[0] if ratios else 0.75))
    cells: list[np.ndarray] = []
    for label, img in decoded:
        if img is None:
            cell = np.full((tile_h, tile_width, 3), TILE_BG, np.uint8)
            cv2.putText(
                cell, "no frame", (6, tile_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TILE_TEXT_COLOR, 1, cv2.LINE_AA
            )
        else:
            cell = cv2.resize(img, (tile_width, tile_h), interpolation=cv2.INTER_AREA)
        canvas = np.full((tile_h + TILE_LABEL_H, tile_width, 3), 0, np.uint8)
        canvas[TILE_LABEL_H:] = cell
        cv2.putText(
            canvas, label, (4, TILE_LABEL_H - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TILE_TEXT_COLOR, 1, cv2.LINE_AA
        )
        cells.append(canvas)
    cols = min(len(cells), 2 if len(cells) <= 4 else 3)
    rows = []
    for start in range(0, len(cells), cols):
        row = cells[start : start + cols]
        row += [np.zeros_like(cells[0])] * (cols - len(row))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def montage_jpeg(tiles: list[tuple[str, bytes | None]], tile_width: int, quality: int = 80) -> bytes:
    """JPEG of build_montage.

    Args:
        tiles (list[tuple[str, bytes | None]]): (label, JPEG or None) per stop.
        tile_width (int): Width of one tile (pixels).
        quality (int): JPEG quality.

    Returns:
        bytes: JPEG data.
    """
    montage = build_montage(tiles, tile_width)
    return encode_jpeg(montage, max(montage.shape[:2]), quality)


def read_sectors(robot: RobotApi) -> tuple[list[SectorObstacle] | None, MapSummary]:
    """Nearest lidar return per sector from the robot's scan.

    Args:
        robot (RobotApi): Robot.

    Returns:
        tuple[list[SectorObstacle] | None, MapSummary]: Sectors (None when no fresh scan) and the raw summary.
    """
    summary, _ = robot.map_summary(False, SCAN_RADIUS_M, SCAN_PNG_PX)
    return summary.obstacles, summary


def yaw_delta_deg(previous: float, current: float) -> float:
    """Counter-clockwise rotation between two yaws, wrapped into [-180, 180) degrees.

    Args:
        previous (float): Earlier yaw (rad).
        current (float): Later yaw (rad).

    Returns:
        float: Rotation (degrees).
    """
    return math.degrees(normalize_angle(current - previous))


def run_look_around(
    robot: RobotApi, settings: LookAroundSettings, footprint: FootprintSettings, captures: int, camera: str
) -> LookAroundRun:
    """Turn in place through `captures` equal steps (full circle), capturing at each stop, then return to the start.

    Uses the robot's normal base motion (move_relative with only dyaw), so early return on critical events applies:
    an interrupted or failed step ends the sequence at once (no further rotation, no return attempt). Before every
    rotation the lidar is checked against the footprint's circumscribed radius plus a margin; a stop() issued while
    the tool runs ends it before the next rotation.

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): Step timeout, frame size, margins and capture limits.
        footprint (FootprintSettings): Robot outer frame.
        captures (int): Number of stops.
        camera (str): 'front' or 'gripper'.

    Returns:
        LookAroundRun: Result and frames. RobotError is raised (robot untouched) when rotation is not safe.
    """
    plan = plan_look_around(captures, settings.min_captures, settings.max_captures)
    required = required_clearance_m(footprint.length_m, footprint.width_m, settings.clearance_margin_m)
    obstacles, _ = read_sectors(robot)
    refusal = check_clearance(obstacles, required)
    if refusal is not None:
        raise RobotError(f"look_around refused: {refusal}")
    start = robot.robot_pose()
    if start is None:
        raise RobotError("robot pose (map->base_link) unavailable; look_around needs it to verify the turn")
    started = time.monotonic()
    stop_mark = robot.stop_count()
    result = LookAroundResult(
        status="completed",
        expected={
            "rotation_deg": 360,
            "stops": captures,
            "step_deg": round(math.degrees(plan.step_rad), 3),
            "final_yaw_rad": start.yaw,
        },
    )
    run = LookAroundRun(result=result)
    last_yaw = start.yaw
    rotated_deg = 0.0

    def rotate(index: int, heading_rad: float) -> NavigationResult | None:
        """One rotation step; records it and ends the run when it did not succeed."""
        nonlocal last_yaw, rotated_deg
        if robot.stop_count() != stop_mark:
            result.status, result.message = "stopped", "stop was called; look_around ended before the next rotation"
            return None
        nav = robot.move_relative(0.0, 0.0, plan.step_rad, settings.step_timeout_s)
        expected_yaw = normalize_angle(start.yaw + heading_rad)
        achieved_yaw = nav.final_pose.yaw if nav.final_pose is not None else None
        result.steps.append(
            StepRecord(
                index=index,
                heading_deg=round(math.degrees(heading_rad)) % 360,
                status=nav.status,
                expected_yaw=expected_yaw,
                achieved_yaw=achieved_yaw,
                message=nav.message,
            )
        )
        if achieved_yaw is not None:
            rotated_deg += yaw_delta_deg(last_yaw, achieved_yaw)
            last_yaw = achieved_yaw
        if nav.status == "interrupted":
            result.status, result.interrupted_by = "interrupted", nav.interrupted_by
            result.message = f"rotation interrupted by {nav.interrupted_by}; robot stopped where it was"
            return None
        if nav.status != "succeeded":
            result.status, result.message = (
                "failed",
                f"rotation step {index} ended with status {nav.status}: {nav.message}",
            )
            return None
        return nav

    for index, heading in enumerate(plan.headings_rad):
        if index > 0 and rotate(index, heading) is None:
            break
        label = heading_label(heading)
        try:
            frame = robot.camera_image(camera, settings.frame_max_px)
            run.frames.append((label, frame.jpeg))
        except RobotError as exc:
            run.frames.append((label, None))
            result.notes.append(f"{label}: no camera frame ({exc})")
        obstacles, _ = read_sectors(robot)
        result.headings.append(sector_summary(index, heading, obstacles, run.frames[-1][1] is not None))
        if obstacles is None:
            result.notes.append(f"{label}: lidar scan missing or stale")
        refusal = check_clearance(obstacles, required)
        if refusal is not None:
            result.status, result.message = "aborted_obstacle", f"rotation not continued: {refusal}"
            break
    else:
        # All stops captured: close the circle back to the start heading.
        if rotate(plan.captures, 2 * math.pi) is not None:
            result.returned_to_start = True
    end = robot.robot_pose()
    result.achieved = {
        "rotation_deg": round(rotated_deg, 3),
        "final_pose": end.model_dump() if end else None,
        "heading_error_deg": None if end is None else round(yaw_delta_deg(start.yaw, end.yaw), 3),
    }
    result.duration_s = round(time.monotonic() - started, 3)
    return run
