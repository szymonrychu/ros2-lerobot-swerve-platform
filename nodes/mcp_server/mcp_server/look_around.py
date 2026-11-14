"""look_around: rotate in place in equal steps, capture a camera frame and the lidar summary at every stop, or pan the
D435 head without moving the base (head mode) (no ROS)."""

import math
import time
from dataclasses import dataclass, field
from typing import Literal

import cv2
import numpy as np

from .camera_scene import Camera
from .config import FootprintSettings, LookAroundSettings
from .geometry import normalize_angle
from .head import HeadController, HeadError, HeadHistoryError, HeadMoveResult
from .models import BaseMotionBusyError, BasePose, MapSummary, NavigationResult, RobotError, SectorObstacle
from .perception import encode_jpeg
from .perception_models import HeadingSummary, LookAroundResult, LookHeadAim, StepRecord
from .tool_context import RobotApi

TILE_BG = 90
TILE_LABEL_H = 18
TILE_TEXT_COLOR = (255, 255, 255)
SCAN_RADIUS_M = 1.0  # the PNG crop is not requested; the radius is a required argument only
SCAN_PNG_PX = 32
DEFAULT_YAW_TOLERANCE_DEG = 5.0  # nav.goal_yaw_tolerance_deg default
LEVEL_PAN_RAD = 0.0  # head aim for the base turn: straight ahead ...
LEVEL_TILT_RAD = 0.0  # ... and level (tilt + is down)
# spin mode without a map pose (localization stalled): the turn runs on odometry, the tiles are relative to the start
NO_MAP_POSE_NOTE = "robot pose (map) unavailable: tiles have no map heading; rotation measured from odometry"

BaseLookMode = Literal["spin", "steps"]  # the modes that turn the base
LookMode = Literal["spin", "steps", "head"]  # 'head' pans the D435 head, the base never moves


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


def nearest_inside(obstacles: list[SectorObstacle], required_m: float) -> SectorObstacle | None:
    """Nearest lidar return inside the rotation circle.

    Args:
        obstacles (list[SectorObstacle]): Nearest return per sector.
        required_m (float): Free radius needed (m).

    Returns:
        SectorObstacle | None: The nearest return closer than required_m, None when the circle is free.
    """
    close = [o for o in obstacles if o.nearest_m is not None and o.nearest_m < required_m]
    return min(close, key=lambda o: o.nearest_m or 0.0) if close else None


def check_clearance(obstacles: list[SectorObstacle] | None, required_m: float) -> str | None:
    """Refusal reason when the lidar shows an obstacle inside the rotation circle (or no lidar at all).

    Args:
        obstacles (list[SectorObstacle] | None): Nearest return per sector, None when the scan is unavailable.
        required_m (float): Free radius needed (m).

    Returns:
        str | None: Reason to refuse (naming the nearest return's distance, bearing and sector), None when the robot
            may rotate.
    """
    if obstacles is None:
        return "lidar scan missing or stale: cannot verify the rotation clearance"
    worst = nearest_inside(obstacles, required_m)
    if worst is None:
        return None
    bearing = "" if worst.bearing_rad is None else f" at bearing {math.degrees(worst.bearing_rad):+.0f} deg"
    return (
        f"obstacle too close for an in-place rotation: {worst.nearest_m:.2f} m{bearing} in sector {worst.sector} "
        f"(needs {required_m:.2f} m = footprint circumscribed radius + margin)"
    )


def escape_translation(obstacle: SectorObstacle, required_m: float, extra_m: float) -> tuple[float, float] | None:
    """Straight translation (base_link) directly away from a return that clears the rotation circle by extra_m.

    Args:
        obstacle (SectorObstacle): Return inside the rotation circle.
        required_m (float): Free radius needed (m).
        extra_m (float): Clearance beyond required_m after the move (m).

    Returns:
        tuple[float, float] | None: (dx, dy) in m, None when the return has no distance or bearing.
    """
    if obstacle.nearest_m is None or obstacle.bearing_rad is None:
        return None
    distance = max(0.0, required_m - obstacle.nearest_m) + extra_m
    return -distance * math.cos(obstacle.bearing_rad), -distance * math.sin(obstacle.bearing_rad)


def heading_label(heading_rad: float) -> str:
    """Label of a stop: degrees turned counter-clockwise from the start heading.

    Args:
        heading_rad (float): Relative heading (rad).

    Returns:
        str: "0 deg (start)" or "+90 deg".
    """
    deg = round(math.degrees(heading_rad)) % 360
    return "0 deg (start)" if deg == 0 else f"+{deg} deg"


def head_pan_label(pan_deg: float) -> str:
    """Label of a head mode tile: the signed head pan, + left (counter-clockwise), rounded to whole degrees.

    Args:
        pan_deg (float): Head pan (deg, + left).

    Returns:
        str: "pan +90 deg (left)", "pan 0 deg (ahead)" or "pan -45 deg (right)".
    """
    deg = round(pan_deg)
    if deg == 0:
        return "pan 0 deg (ahead)"
    return f"pan {deg:+d} deg ({'left' if deg > 0 else 'right'})"


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


def head_aim(head: HeadController) -> tuple[float, float] | None:
    """Measured (pan, tilt) of the head from a fresh joint state.

    Args:
        head (HeadController): Head.

    Returns:
        tuple[float, float] | None: (pan, tilt) in rad, None when there is no fresh sample.
    """
    measured = head.measured()
    if measured is None:
        return None
    pan, tilt = (measured[name] for name in head.settings.joint_names)
    return pan, tilt


def head_at(head: HeadController, aim: tuple[float, float] | None, pan_rad: float, tilt_rad: float) -> bool:
    """Whether a measured aim is within the head tolerance of (pan_rad, tilt_rad).

    Args:
        head (HeadController): Head (its settings carry the tolerance).
        aim (tuple[float, float] | None): Measured (pan, tilt) in rad, None when unknown.
        pan_rad (float): Target pan (rad).
        tilt_rad (float): Target tilt (rad).

    Returns:
        bool: True when both joints are within tolerance_rad; False when the aim is unknown.
    """
    tolerance = head.settings.tolerance_rad
    return aim is not None and abs(aim[0] - pan_rad) <= tolerance and abs(aim[1] - tilt_rad) <= tolerance


def head_move_note(what: str, outcome: HeadMoveResult) -> str | None:
    """Note for a head move that did not converge.

    Args:
        what (str): What the move was for (starts the note).
        outcome (HeadMoveResult): Result of HeadController.move / home.

    Returns:
        str | None: The note, None when the move converged.
    """
    if outcome.status == "converged":
        return None
    reason = "no fresh head joint state" if outcome.status == "no_feedback" else "the head did not reach the target"
    return f"{what}: head move ended with status {outcome.status} ({reason})"


def level_head(head: HeadController, pan_rad: float, tilt_rad: float, notes: list[str]) -> tuple[float, float] | None:
    """Aim the head at (pan_rad, tilt_rad) for the base turn, remembering where it pointed before.

    No command is sent when the measured aim is already within the head tolerance. A move that times out, has no
    feedback or is refused (outside the limits) is a note, never an error: look_around goes on with the head where it is.

    Args:
        head (HeadController): Head.
        pan_rad (float): Target pan (rad, + left).
        tilt_rad (float): Target tilt (rad, + down).
        notes (list[str]): Notes are appended here.

    Returns:
        tuple[float, float] | None: The measured (pan, tilt) before the move (rad), also when no move was needed; None
            when there was no fresh joint state.
    """
    before = head_aim(head)
    if head_at(head, before, pan_rad, tilt_rad):
        return before
    try:
        outcome = head.move(pan_rad, tilt_rad)
    except HeadError as exc:
        notes.append(f"head not levelled for the turn: {exc}")
        return before
    note = head_move_note("head levelling for the turn", outcome)
    if note is not None:
        notes.append(note)
    return before


def restore_head(head: HeadController, before: tuple[float, float] | None, notes: list[str]) -> bool:
    """Aim the head back where level_head found it, or home it (with a note) when that aim was unknown.

    Args:
        head (HeadController): Head.
        before (tuple[float, float] | None): Aim returned by level_head (rad), None when unknown.
        notes (list[str]): Notes are appended here.

    Returns:
        bool: True when the head converged on the restored aim.
    """
    try:
        if before is None:
            notes.append("head aim before look_around unknown (no fresh joint state): head sent home instead")
            outcome = head.home()
        elif head_at(head, head_aim(head), *before):
            return True
        else:
            outcome = head.move(*before)
    except HeadError as exc:
        notes.append(f"head aim not restored: {exc}")
        return False
    note = head_move_note("head aim restore", outcome)
    if note is not None:
        notes.append(note)
    return note is None


def run_look_around(
    robot: RobotApi,
    settings: LookAroundSettings,
    footprint: FootprintSettings,
    captures: int,
    camera: Camera,
    yaw_tolerance_deg: float = DEFAULT_YAW_TOLERANCE_DEG,
    mode: BaseLookMode = "steps",
    return_to_start: bool = True,
) -> LookAroundRun:
    """Turn in place through `captures` equal headings, capturing at each one, optionally returning to the start.

    mode 'spin' turns once continuously (RobotApi.spin, see run_spin_mode); mode 'steps' (described below) makes one
    precise Nav2 rotation per heading (head mode, which never moves the base, is run_head_mode). Both capture the same headings and labels (the same montage). Without
    return_to_start the turn ends at the last heading (no closing rotation, no corrective rotation after a failure).

    Uses the robot's normal base motion (move_relative with only dyaw), so early return on critical events applies
    within a step. One event mark is taken for the whole run and checked before every rotation, so a critical event
    raised while capturing between two steps ends the run too: no further rotation and no return to the start (the
    robot stays where the event found it). An interrupted or failed step likewise ends the sequence at once. A
    RobotError from a step (stale pose, ...) ends it as status 'failed' with the frames captured so far; a single
    corrective rotation back to the start heading is then attempted, except when another base motion was running
    (nothing is sent against it). Before every rotation the lidar is checked against the footprint's circumscribed
    radius plus a margin; a stop() issued while the tool runs ends it before the next rotation.

    Without a fresh map pose (map->base_link stale, localization stalled) 'steps' is refused (its Nav2 goals need it)
    and 'spin' runs anyway with NO_MAP_POSE_NOTE: its rotation is measured from odometry.

    With settings.level_head the D435 head is levelled (pan 0, tilt 0, level_head) once the turn is allowed, before the
    first capture, and its previous aim is restored when the run ends (restore_head; home when that aim was unknown),
    except after a stop() (the head stays where it is). Head problems are notes in result.notes, never errors.

    `returned_to_start` is true only when the closing rotation (or the corrective one) succeeded and the measured final
    heading is within `yaw_tolerance_deg` of the start heading; `achieved.heading_error_deg` always carries the error.

    Args:
        mode (BaseLookMode): 'spin' (continuous) or 'steps' (one Nav2 rotation per heading).
        return_to_start (bool): Turn back to the start heading at the end.
        robot (RobotApi): Robot.
        settings (LookAroundSettings): Step timeout, frame size, margins and capture limits.
        footprint (FootprintSettings): Robot outer frame.
        captures (int): Number of stops.
        camera (Camera): 'head', the D435 colour camera on the pan/tilt head.
        yaw_tolerance_deg (float): Largest final heading error still counted as back at the start (nav.goal_yaw_tolerance_deg).

    Returns:
        LookAroundRun: Result and frames. RobotError is raised (robot untouched) when rotation is not safe or, in
            'steps' mode, without a map pose.
    """
    plan = plan_look_around(captures, settings.min_captures, settings.max_captures)
    required = required_clearance_m(footprint.length_m, footprint.width_m, settings.clearance_margin_m)
    obstacles, _ = read_sectors(robot)
    refusal = check_clearance(obstacles, required)
    if refusal is not None:
        raise RobotError(f"look_around refused: {refusal}")
    start = robot.robot_pose()
    if start is None and mode == "steps":
        raise RobotError(
            "robot pose (map->base_link) unavailable; look_around mode 'steps' needs it for its Nav2 rotations "
            "(mode 'spin' turns on odometry, mode 'head' does not move the base)"
        )
    final_yaw = None  # no map heading without the map pose
    if start is not None:
        final_yaw = start.yaw if return_to_start else normalize_angle(start.yaw + plan.headings_rad[-1])
    started = time.monotonic()
    stop_mark = robot.stop_count()
    event_mark = robot.event_seq()  # both marks precede the head move, which can block: a stop during it still counts
    result = LookAroundResult(
        status="completed",
        expected={
            "mode": mode,
            "rotation_deg": 360 if return_to_start else round(math.degrees(plan.headings_rad[-1])),
            "stops": captures,
            "step_deg": round(math.degrees(plan.step_rad), 3),
            "final_yaw_rad": final_yaw,
        },
    )
    if start is None:
        result.notes.append(NO_MAP_POSE_NOTE)
    run = LookAroundRun(result=result)
    before = level_head(robot.head, LEVEL_PAN_RAD, LEVEL_TILT_RAD, result.notes) if settings.level_head else None
    try:
        if mode == "spin":
            return run_spin_mode(
                robot,
                settings,
                plan,
                required,
                camera,
                start,
                run,
                return_to_start,
                yaw_tolerance_deg,
                started,
                stop_mark,
                event_mark,
            )
        assert start is not None  # steps mode is refused above without the map pose
        return run_steps_mode(
            robot,
            settings,
            plan,
            required,
            camera,
            start,
            run,
            return_to_start,
            yaw_tolerance_deg,
            started,
            stop_mark,
            event_mark,
        )
    finally:
        if settings.level_head:
            stopped = robot.stop_count() != stop_mark
            if stopped:
                result.notes.append("stop was called: the head was left where it is (aim not restored)")
            result.head = LookHeadAim(
                before_deg=None
                if before is None
                else (round(math.degrees(before[0]), 2), round(math.degrees(before[1]), 2)),
                restored=not stopped and restore_head(robot.head, before, result.notes),
            )
            result.duration_s = round(time.monotonic() - started, 3)


def run_steps_mode(
    robot: RobotApi,
    settings: LookAroundSettings,
    plan: LookPlan,
    required: float,
    camera: Camera,
    start: BasePose,
    run: LookAroundRun,
    return_to_start: bool,
    yaw_tolerance_deg: float,
    started: float,
    stop_mark: int,
    event_mark: int,
) -> LookAroundRun:
    """look_around 'steps' mode: one precise Nav2 rotation per heading with a frame and the lidar at every stop.

    With settings.level_head the head is levelled again before every capture after the first (head_aim homes the head
    when each Nav2 goal ends). See run_look_around for the stop, event, failure and return_to_start rules.

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): Step timeout, frame size and level_head.
        plan (LookPlan): Headings.
        required (float): Free radius needed for the rotation (m).
        camera (Camera): 'head', the D435 colour camera on the pan/tilt head.
        start (BasePose): Start pose.
        run (LookAroundRun): Result and frames to fill.
        return_to_start (bool): Turn back to the start heading at the end.
        yaw_tolerance_deg (float): Heading tolerance for returned_to_start.
        started (float): Monotonic start time (s).
        stop_mark (int): robot.stop_count() taken before the head was levelled.
        event_mark (int): robot.event_seq() taken before the head was levelled.

    Returns:
        LookAroundRun: The filled run.
    """
    result = run.result
    last_yaw = start.yaw
    rotated_deg = 0.0
    closed = False  # the closing (or corrective) rotation succeeded

    def record(index: int, heading_rad: float, nav: NavigationResult) -> None:
        """Append the step record and account for the rotation actually achieved."""
        nonlocal last_yaw, rotated_deg
        achieved_yaw = nav.final_pose.yaw if nav.final_pose is not None else None
        result.steps.append(
            StepRecord(
                index=index,
                heading_deg=round(math.degrees(heading_rad)) % 360,
                status=nav.status,
                expected_yaw=normalize_angle(start.yaw + heading_rad),
                achieved_yaw=achieved_yaw,
                message=nav.message,
            )
        )
        if achieved_yaw is not None:
            rotated_deg += yaw_delta_deg(last_yaw, achieved_yaw)
            last_yaw = achieved_yaw

    def return_home(index: int) -> None:
        """One corrective rotation back to the start heading after a failed step."""
        nonlocal closed
        delta = normalize_angle(start.yaw - last_yaw)
        if abs(math.degrees(delta)) <= yaw_tolerance_deg:
            return
        try:
            nav = robot.move_relative(0.0, 0.0, delta, settings.step_timeout_s, precise=True)
        except RobotError as exc:
            result.notes.append(f"return to start failed: {exc}")
            return
        record(index, 2 * math.pi, nav)
        closed = nav.status == "succeeded"
        if not closed:
            result.notes.append(f"return to start ended with status {nav.status}: {nav.message}")

    def rotate(index: int, heading_rad: float) -> NavigationResult | None:
        """One rotation step; records it and ends the run when it did not succeed."""
        interrupt = robot.interrupt_since(event_mark)
        if interrupt is not None:
            result.status, result.interrupted_by = "interrupted", interrupt
            result.message = (
                f"critical event {interrupt} raised before rotation step {index}; look_around ended without "
                "rotating further and did NOT return to the start heading (the robot stayed where it was)"
            )
            return None
        if robot.stop_count() != stop_mark:
            result.status, result.message = "stopped", "stop was called; look_around ended before the next rotation"
            return None
        try:
            nav = robot.move_relative(0.0, 0.0, plan.step_rad, settings.step_timeout_s, precise=True)
        except RobotError as exc:
            result.status = "failed"
            result.message = f"rotation step {index} could not start: {exc}"
            if isinstance(exc, BaseMotionBusyError):
                result.message += "; no corrective rotation sent while another base motion runs"
            elif return_to_start:
                return_home(index + 1)
            return None
        record(index, heading_rad, nav)
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
        if index > 0 and settings.level_head:
            level_head(robot.head, LEVEL_PAN_RAD, LEVEL_TILT_RAD, result.notes)
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
        # All stops captured: close the circle back to the start heading (only when asked).
        if return_to_start:
            closed = rotate(plan.captures, 2 * math.pi) is not None
    end = robot.robot_pose()
    heading_error = None if end is None else yaw_delta_deg(start.yaw, end.yaw)
    result.returned_to_start = closed and heading_error is not None and abs(heading_error) <= yaw_tolerance_deg
    if closed and not result.returned_to_start:
        result.notes.append(
            "final heading unknown: not confirmed back at the start"
            if heading_error is None
            else f"heading error {heading_error:.1f} deg exceeds the {yaw_tolerance_deg:g} deg tolerance: "
            "not back at the start heading"
        )
    result.achieved = {
        "rotation_deg": round(rotated_deg, 3),
        "final_pose": end.model_dump() if end else None,
        "heading_error_deg": None if heading_error is None else round(heading_error, 3),
    }
    result.duration_s = round(time.monotonic() - started, 3)
    return run


def run_spin_mode(
    robot: RobotApi,
    settings: LookAroundSettings,
    plan: LookPlan,
    required: float,
    camera: Camera,
    start: BasePose | None,
    run: LookAroundRun,
    return_to_start: bool,
    yaw_tolerance_deg: float,
    started: float,
    stop_mark: int,
    event_mark: int,
) -> LookAroundRun:
    """look_around 'spin' mode: capture at the start heading, then one continuous slow rotation (RobotApi.spin, base
    path with stop, critical events and a timeout) capturing a frame and the lidar at every further heading.

    The spin covers the last heading, or the full circle with return_to_start (then one precise corrective rotation
    removes a final heading error beyond the tolerance). An obstacle inside the rotation circle at a heading ends the
    spin there ('aborted_obstacle'); a critical event, a stop or a failure ends it without any corrective rotation.

    The spin measures its rotation from odometry, so it also runs without a map pose (start None, NO_MAP_POSE_NOTE):
    the tiles keep their headings relative to the start, but the steps carry no map yaws, there is no corrective
    rotation and returned_to_start stays false (the map heading is not verified).

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): Spin speed, rate, timeout margin and frame size.
        plan (LookPlan): Headings.
        required (float): Free radius needed for the rotation (m).
        camera (Camera): 'head', the D435 colour camera on the pan/tilt head.
        start (BasePose | None): Start map pose, None when it is unavailable (the turn runs on odometry).
        run (LookAroundRun): Result and frames to fill.
        return_to_start (bool): Turn the full circle back to the start heading.
        yaw_tolerance_deg (float): Heading tolerance for returned_to_start.
        started (float): Monotonic start time (s).
        stop_mark (int): robot.stop_count() taken before the head was levelled.
        event_mark (int): robot.event_seq() taken before the head was levelled.

    Returns:
        LookAroundRun: The filled run.
    """
    result = run.result

    def capture(index: int) -> bool:
        """Frame + lidar at heading `index`; False (and status aborted_obstacle) when the rotation must not go on."""
        heading = plan.headings_rad[index]
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
        pose = None if start is None else robot.robot_pose()  # map yaws only with a map start heading
        result.steps.append(
            StepRecord(
                index=index,
                heading_deg=round(math.degrees(heading)) % 360,
                status="captured",
                expected_yaw=None if start is None else normalize_angle(start.yaw + heading),
                achieved_yaw=None if pose is None else pose.yaw,
            )
        )
        refusal = check_clearance(obstacles, required)
        if refusal is not None:
            result.status, result.message = "aborted_obstacle", f"rotation not continued: {refusal}"
            return False
        return True

    rotated_deg = 0.0
    closed = False
    if capture(0):
        interrupt = robot.interrupt_since(event_mark)
        angle = 2 * math.pi if return_to_start else plan.headings_rad[-1]
        if interrupt is not None:
            result.status, result.interrupted_by = "interrupted", interrupt
            result.message = f"critical event {interrupt} raised before the spin; the robot did not turn"
        elif robot.stop_count() != stop_mark:
            result.status, result.message = "stopped", "stop was called; look_around ended before turning"
        else:
            try:
                outcome = robot.spin(
                    settings.spin_speed_rps,
                    angle,
                    plan.headings_rad[1:],
                    lambda i: capture(i + 1),
                    angle / settings.spin_speed_rps + settings.spin_timeout_margin_s,
                )
            except RobotError as exc:
                result.status, result.message = "failed", f"spin could not start: {exc}"
            else:
                rotated_deg = math.degrees(outcome.rotated_rad)
                if outcome.status == "interrupted":
                    result.status, result.interrupted_by = "interrupted", outcome.interrupted_by
                    result.message = f"spin interrupted by {outcome.interrupted_by}; robot stopped where it was"
                elif outcome.status == "stopped":
                    result.status, result.message = "stopped", "stop was called; the spin ended"
                elif outcome.status in ("failed", "timeout"):
                    result.status, result.message = (
                        "failed",
                        f"spin ended with status {outcome.status}: {outcome.message}",
                    )
                elif outcome.status == "completed" and return_to_start:
                    closed = True
                    if start is not None:
                        rotated_deg += correct_heading(robot, settings, start, yaw_tolerance_deg, result)
    end = robot.robot_pose()
    heading_error = None if end is None or start is None else yaw_delta_deg(start.yaw, end.yaw)
    result.returned_to_start = closed and heading_error is not None and abs(heading_error) <= yaw_tolerance_deg
    result.achieved = {
        "rotation_deg": round(rotated_deg, 3),
        "final_pose": end.model_dump() if end else None,
        "heading_error_deg": None if heading_error is None else round(heading_error, 3),
    }
    result.duration_s = round(time.monotonic() - started, 3)
    return run


def correct_heading(
    robot: RobotApi, settings: LookAroundSettings, start: BasePose, yaw_tolerance_deg: float, result: LookAroundResult
) -> float:
    """One precise rotation back to the start heading when the spin ended outside the tolerance.

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): Step timeout.
        start (BasePose): Start pose.
        yaw_tolerance_deg (float): Heading tolerance.
        result (LookAroundResult): Notes are added here.

    Returns:
        float: Rotation of the correction (deg; 0 without one).
    """
    end = robot.robot_pose()
    if end is None:
        result.notes.append("final heading unknown: no corrective rotation")
        return 0.0
    delta = normalize_angle(start.yaw - end.yaw)
    if abs(math.degrees(delta)) <= yaw_tolerance_deg:
        return 0.0
    try:
        nav = robot.move_relative(0.0, 0.0, delta, settings.step_timeout_s, precise=True)
    except RobotError as exc:
        result.notes.append(f"corrective rotation failed: {exc}")
        return 0.0
    if nav.status != "succeeded":
        result.notes.append(f"corrective rotation ended with status {nav.status}: {nav.message}")
    return math.degrees(delta)


def run_head_mode(
    robot: RobotApi, settings: LookAroundSettings, pans_deg: list[float], tilt_deg: float, camera: Camera
) -> LookAroundRun:
    """look_around 'head' mode: pan the D435 head through pans_deg at tilt_deg with a frame at every pan; the base
    never moves (no lidar clearance check, no robot pose needed).

    The aim before the run is recorded and restored in the end (home with a note when it was unknown), except after a
    stop() (the head stays where it is). The stop and event marks are taken before the first head move and checked
    before every pan and again before every frame: a stop ends the run as 'stopped', a critical event as 'interrupted',
    with no further pan or capture. After each pan the head settles for settings.head_settle_s, then a frame is taken;
    the head pose at the frame stamp (head.pose_at) labels the tile. A frame taken while the head was still moving is
    retaken once, a second moving frame is kept with a note. A head move that times out, has no feedback or is refused
    is a note, never an error; a missing frame is a grey tile with a note. Each pan also records the lidar sectors.

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): frame_max_px and head_settle_s.
        pans_deg (list[float]): Head pans to visit in order (deg, + left), validated by the caller.
        tilt_deg (float): Head tilt held while panning (deg, + down).
        camera (Camera): 'head', the D435 colour camera on the pan/tilt head.

    Returns:
        LookAroundRun: Result and frames; result.headings[i].heading_deg is the signed pan measured at the frame (the
            commanded pan when the joint-state history does not cover the frame stamp).
    """
    head = robot.head
    started = time.monotonic()
    stop_mark = robot.stop_count()
    event_mark = robot.event_seq()  # both marks precede the first head move, which blocks
    result = LookAroundResult(
        status="completed", expected={"mode": "head", "rotation_deg": 0, "pans_deg": list(pans_deg)}
    )
    run = LookAroundRun(result=result)
    measured_pans: list[float | None] = []
    before = head_aim(head)

    def halted(what: str) -> bool:
        """End the run on a critical event or a stop raised since the marks."""
        interrupt = robot.interrupt_since(event_mark)
        if interrupt is not None:
            result.status, result.interrupted_by = "interrupted", interrupt
            result.message = f"critical event {interrupt} raised {what}; look_around ended without panning further"
            return True
        if robot.stop_count() != stop_mark:
            result.status, result.message = "stopped", f"stop was called; look_around ended {what}"
            return True
        return False

    try:
        for index, pan_deg in enumerate(pans_deg):
            if halted(f"before pan {index}"):
                break
            commanded = head_pan_label(pan_deg)
            try:
                outcome = head.move(math.radians(pan_deg), math.radians(tilt_deg))
            except HeadError as exc:
                result.steps.append(
                    StepRecord(index=index, heading_deg=round(pan_deg), status="refused", message=str(exc))
                )
                result.notes.append(f"{commanded}: head move refused ({exc}); no frame")
                run.frames.append((commanded, None))
                result.headings.append(head_heading_summary(index, pan_deg, commanded, None, False))
                measured_pans.append(None)
                continue
            note = head_move_note(commanded, outcome)
            result.steps.append(
                StepRecord(index=index, heading_deg=round(pan_deg), status=outcome.status, message=note or "")
            )
            if note is not None:
                result.notes.append(note)
            head.backend.sleep(settings.head_settle_s)
            if halted(f"before the frame at pan {index}"):
                break
            jpeg, measured = capture_head_frame(robot, settings, camera, commanded, result.notes)
            label = commanded if measured is None else head_pan_label(math.degrees(measured))
            run.frames.append((label, jpeg))
            measured_pans.append(None if measured is None else round(math.degrees(measured), 2))
            try:
                obstacles, _ = read_sectors(robot)
            except RobotError as exc:
                obstacles = None
                result.notes.append(f"{label}: lidar summary unavailable ({exc})")
            heading = pan_deg if measured is None else math.degrees(measured)
            result.headings.append(head_heading_summary(index, heading, label, obstacles, jpeg is not None))
    finally:
        stopped = robot.stop_count() != stop_mark
        if stopped:
            result.notes.append("stop was called: the head was left where it is (aim not restored)")
        result.head = LookHeadAim(
            before_deg=None
            if before is None
            else (round(math.degrees(before[0]), 2), round(math.degrees(before[1]), 2)),
            restored=not stopped and restore_head(head, before, result.notes),
        )
        result.achieved = {"pans_deg": measured_pans}
        result.duration_s = round(time.monotonic() - started, 3)
    return run


def head_heading_summary(
    index: int, pan_deg: float, label: str, obstacles: list[SectorObstacle] | None, captured: bool
) -> HeadingSummary:
    """Per-pan summary of head mode: the signed pan as heading_deg and the lidar sectors (the base does not turn).

    Args:
        index (int): Pan number.
        pan_deg (float): Head pan at the frame (deg, + left).
        label (str): Tile label (head_pan_label).
        obstacles (list[SectorObstacle] | None): Sector returns, None when unavailable.
        captured (bool): Whether a camera frame was taken.

    Returns:
        HeadingSummary: The summary.
    """
    summary = sector_summary(index, 0.0, obstacles, captured)
    return summary.model_copy(update={"heading_deg": round(pan_deg), "label": label})


def capture_head_frame(
    robot: RobotApi, settings: LookAroundSettings, camera: Camera, label: str, notes: list[str]
) -> tuple[bytes | None, float | None]:
    """Take the frame of one pan, retaking it once when the head was still moving at its stamp.

    Args:
        robot (RobotApi): Robot.
        settings (LookAroundSettings): frame_max_px and head_settle_s (waited again before the retake).
        camera (Camera): 'head'.
        label (str): Commanded pan label, starts the notes.
        notes (list[str]): Notes are appended here.

    Returns:
        tuple[bytes | None, float | None]: The JPEG (None when the camera failed, noted) and the head pan at its stamp
            (rad; None without a frame or when the joint-state history does not cover the stamp, noted).
    """
    for attempt in range(2):
        try:
            frame = robot.camera_image(camera, settings.frame_max_px)
        except RobotError as exc:
            notes.append(f"{label}: no camera frame ({exc})")
            return None, None
        try:
            pose = robot.head.pose_at(frame.stamp_s)
        except HeadHistoryError as exc:
            notes.append(f"{label}: head pan at the frame unknown ({exc}); labelled with the commanded pan")
            return frame.jpeg, None
        if not pose.moving:
            break
        if attempt == 0:
            robot.head.backend.sleep(settings.head_settle_s)
        else:
            notes.append(
                f"{label}: the head was still moving at the retaken frame ({math.degrees(pose.window_motion_rad):.1f} "
                "deg around its stamp): the tile may be blurred"
            )
    return frame.jpeg, pose.pan
