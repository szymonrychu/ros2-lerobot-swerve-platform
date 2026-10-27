"""Timed base velocity streaming (drive tool), the straight translation of move_relative and the stop sequence: clamp,
publish at a fixed rate, always finish with a zero twist."""

import math
from collections.abc import Callable
from typing import Literal, Protocol

from pydantic import BaseModel

from .geometry import clamp_twist, compose_relative, normalize_angle, relative_pose
from .metrics import NAV_DURATION, NAV_GOALS
from .models import BasePose, NavigationResult, RobotError, StopResult

# move_relative near obstacles (README "StopBox on backing and turning"): a translation up to STRAIGHT_MOVE_MAX_M with
# dyaw 0 that backs (dx < 0) or slides (dy != 0) runs as one straight twist on cmd_vel_nav (velocity_smoother and
# collision_monitor downstream, so StopBox checks the sub-polygon of the travel direction) at STRAIGHT_MOVE_SPEED_MPS
# (the drive speed that freed the robot on 2026-10-22), never as a Nav2 goal that would turn the robot to face it first.
STRAIGHT_MOVE_MAX_M = 0.5
STRAIGHT_MOVE_SPEED_MPS = 0.1
# nav2_params.yaml FollowPath.angular_dist_threshold: RotationShimController turns in place first when the path heading
# is further off than this.
SHIM_TURN_THRESHOLD_RAD = 0.785


class DriveError(ValueError):
    """Raised for an invalid drive request."""


class DriveOutcome(BaseModel):
    """Result of a timed drive."""

    commanded: tuple[float, float, float]
    clamped: bool
    aborted: bool
    published: int
    status: Literal["completed", "interrupted", "stopped"] = "completed"
    interrupted_by: str | None = None
    duration_s: float = 0.0
    expected: dict[str, float] | None = None
    achieved: dict[str, float] | None = None


def run_drive(
    publish: Callable[[float, float, float], None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    should_abort: Callable[[], bool],
    vx: float,
    vy: float,
    wz: float,
    duration_s: float,
    rate_hz: float,
    max_linear: float,
    max_angular: float,
    max_duration: float,
    interrupt: Callable[[], str | None] = lambda: None,
) -> DriveOutcome:
    """Publish a clamped twist at rate_hz for duration_s, then a zero twist (also after an abort).

    Args:
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps the given seconds.
        should_abort (Callable[[], bool]): Polled each cycle; True stops early (stop tool).
        vx (float): Forward velocity (m/s).
        vy (float): Lateral velocity (m/s).
        wz (float): Yaw rate (rad/s).
        duration_s (float): Drive time, in (0, max_duration].
        rate_hz (float): Publish rate.
        max_linear (float): Linear clamp (m/s).
        max_angular (float): Angular clamp (rad/s).
        max_duration (float): Longest allowed duration (s).
        interrupt (Callable[[], str | None]): Polled each cycle; a critical event type ends the drive early.

    Returns:
        DriveOutcome: Commanded (clamped) twist, whether it was clamped/aborted, status, interrupting event, duration
            and messages published.
    """
    if not math.isfinite(duration_s) or duration_s <= 0.0 or duration_s > max_duration:
        raise DriveError(f"duration_s must be in (0, {max_duration}] s, got {duration_s}")
    try:
        cmd = clamp_twist(vx, vy, wz, max_linear, max_angular)
    except ValueError as exc:
        raise DriveError(str(exc)) from exc
    period = 1.0 / rate_hz
    start = now()
    end = start + duration_s
    next_tick = start
    published = 0
    aborted = False
    interrupted_by: str | None = None
    try:
        while now() < end - 1e-9:
            if should_abort():
                aborted = True
                break
            interrupted_by = interrupt()
            if interrupted_by is not None:
                break
            publish(*cmd)
            published += 1
            next_tick += period
            sleep(max(0.0, next_tick - now()))
    finally:
        publish(0.0, 0.0, 0.0)
        published += 1
    return DriveOutcome(
        commanded=cmd,
        clamped=cmd != (vx, vy, wz),
        aborted=aborted,
        published=published,
        status="interrupted" if interrupted_by else "stopped" if aborted else "completed",
        interrupted_by=interrupted_by,
        duration_s=round(now() - start, 3),
    )


class SpinOutcome(BaseModel):
    """Result of a continuous in-place rotation (run_spin)."""

    status: Literal["completed", "stopped", "interrupted", "aborted", "failed", "timeout"]
    message: str = ""
    commanded_wz: float
    rotated_rad: float
    marks_reached: int
    interrupted_by: str | None = None
    duration_s: float = 0.0


def run_spin(
    publish: Callable[[float, float, float], None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    should_abort: Callable[[], bool],
    interrupt: Callable[[], str | None],
    yaw: Callable[[], float | None],
    wz: float,
    angle_rad: float,
    marks_rad: list[float],
    on_mark: Callable[[int], bool],
    rate_hz: float,
    max_angular: float,
    timeout_s: float,
) -> SpinOutcome:
    """Rotate in place at a constant (clamped) yaw rate until the measured rotation reaches angle_rad; always zero.

    The rotation is integrated from the measured heading (wrapped deltas), so a slow or slipping base still turns
    the full angle. on_mark(i) is called once the rotation passed marks_rad[i] (in order); it returns False to end
    the spin (e.g. an obstacle seen at that heading). A stop request, a critical event, a lost pose or the timeout
    also end it. A zero twist is published at the end in every case.

    Args:
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps.
        should_abort (Callable[[], bool]): True after the stop tool.
        interrupt (Callable[[], str | None]): Critical event type that ends the spin, or None.
        yaw (Callable[[], float | None]): Current fresh heading (rad), None when unknown.
        wz (float): Yaw rate (rad/s); its sign is the turn direction (+ = counter-clockwise).
        angle_rad (float): Rotation to turn (rad, > 0).
        marks_rad (list[float]): Rotations (rad, ascending) at which on_mark fires.
        on_mark (Callable[[int], bool]): Mark callback; False ends the spin ('aborted').
        rate_hz (float): Publish rate.
        max_angular (float): Yaw rate clamp (rad/s).
        timeout_s (float): Longest spin (s).

    Returns:
        SpinOutcome: Status, rotation achieved, marks reached and duration.

    Raises:
        DriveError: For a non-positive angle or a zero yaw rate.
    """
    if not math.isfinite(angle_rad) or angle_rad <= 0.0:
        raise DriveError(f"angle_rad must be positive, got {angle_rad}")
    if not math.isfinite(wz) or wz == 0.0:
        raise DriveError("wz must be non-zero")
    cmd = math.copysign(min(abs(wz), max_angular), wz)
    direction = math.copysign(1.0, wz)
    period = 1.0 / rate_hz
    start = now()
    rotated = 0.0
    reached = 0
    status: Literal["completed", "stopped", "interrupted", "aborted", "failed", "timeout"] = "completed"
    message = ""
    event: str | None = None
    previous = yaw()
    try:
        while True:
            if should_abort():
                status, message = "stopped", "stop requested"
                break
            event = interrupt()
            if event is not None:
                status, message = "interrupted", f"stopped early: critical event {event}"
                break
            current = yaw()
            if current is None or previous is None:
                status, message = "failed", "robot pose unavailable: cannot measure the rotation"
                break
            rotated += normalize_angle(current - previous) * direction
            previous = current
            refused = False
            while reached < len(marks_rad) and rotated >= marks_rad[reached] - 1e-9:
                reached += 1
                if not on_mark(reached - 1):
                    refused = True
                    break
            if refused:
                status, message = "aborted", f"ended at mark {reached - 1}"
                break
            if rotated >= angle_rad - 1e-9:
                break
            if now() - start >= timeout_s:
                status, message = "timeout", f"rotation not finished after {timeout_s:g} s"
                break
            publish(0.0, 0.0, cmd)
            sleep(period)
    finally:
        publish(0.0, 0.0, 0.0)
    return SpinOutcome(
        status=status,
        message=message,
        commanded_wz=cmd,
        rotated_rad=round(rotated, 4),
        marks_reached=reached,
        interrupted_by=event,
        duration_s=round(now() - start, 3),
    )


def run_stop(
    signal_base_stop: Callable[[], None],
    publish: Callable[[float, float, float], None],
    cancel_all_goals: Callable[[], bool],
    stop_arm: Callable[[], tuple[bool, str]],
) -> StopResult:
    """Stop tool sequence: every part is attempted regardless of the others; the base is always zeroed.

    Args:
        signal_base_stop (Callable[[], None]): Tells running navigate/drive calls to abort.
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        cancel_all_goals (Callable[[], bool]): Cancels every Nav2 goal; True when the cancel service answered.
        stop_arm (Callable[[], tuple[bool, str]]): Arm part (ArmController.stop_hold): held flag and note.

    Returns:
        StopResult: What succeeded.
    """
    signal_base_stop()
    publish(0.0, 0.0, 0.0)
    arm_held, arm_note = stop_arm()
    cancelled = cancel_all_goals()
    publish(0.0, 0.0, 0.0)
    notes = [] if cancelled else ["Nav2 cancel service unavailable"]
    if arm_note:
        notes.append(arm_note)
    return StopResult(nav_goals_cancelled=cancelled, base_zeroed=True, arm_held=arm_held, message="; ".join(notes))


def run_base_stop(
    signal_base_stop: Callable[[], None],
    publish: Callable[[float, float, float], None],
    cancel_all_goals: Callable[[], bool],
) -> bool:
    """Base half of the stop sequence (release_control base reset): abort base calls, zero, cancel Nav2, zero again.

    Args:
        signal_base_stop (Callable[[], None]): Tells running navigate/drive calls to abort.
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz).
        cancel_all_goals (Callable[[], bool]): Cancels every Nav2 goal; True when the cancel service answered.

    Returns:
        bool: True when the Nav2 cancel service answered.
    """
    signal_base_stop()
    publish(0.0, 0.0, 0.0)
    cancelled = cancel_all_goals()
    publish(0.0, 0.0, 0.0)
    return cancelled


class NavPort(Protocol):
    """One NavigateToPose goal as run_nav needs it (implemented over rclpy futures in ros_iface, faked in tests)."""

    def server_ready(self) -> bool:
        """Whether the Nav2 action server is available."""
        ...

    def send_goal(self, pose: BasePose) -> bool | None:
        """Send the goal; True accepted, False rejected, None when Nav2 did not answer."""
        ...

    def result_ready(self) -> bool:
        """Whether the goal has finished."""
        ...

    def result(self) -> tuple[str, str]:
        """Final goal status name and error message."""
        ...

    def cancel(self) -> None:
        """Cancel the goal and wait briefly for the result."""
        ...

    def zero_velocity(self) -> None:
        """Publish a zero twist."""
        ...

    def pose(self) -> BasePose | None:
        """Current fresh map pose, if known."""
        ...


def nav_result(
    status: str,
    message: str,
    goal: BasePose,
    final: BasePose | None,
    duration_s: float,
    interrupted_by: str | None = None,
) -> NavigationResult:
    """Build a NavigationResult with goal/final_pose mirrored as expected/achieved.

    Args:
        status (str): Result status.
        message (str): Explanation.
        goal (BasePose): Requested pose.
        final (BasePose | None): Final measured pose.
        duration_s (float): Seconds from goal start to finish.
        interrupted_by (str | None): Critical event type for status 'interrupted'.

    Returns:
        NavigationResult: Result.
    """
    NAV_GOALS.labels(status).inc()
    NAV_DURATION.observe(duration_s)
    return NavigationResult(
        status=status,
        message=message,
        goal=goal,
        final_pose=final,
        interrupted_by=interrupted_by,
        expected=goal,
        achieved=final,
        duration_s=round(duration_s, 3),
    )


def within_tolerance(pose: BasePose, goal: BasePose, xy_m: float, yaw_rad: float) -> bool:
    """Whether a measured pose is within a position and heading tolerance of the goal (same frame only).

    Args:
        pose (BasePose): Measured pose.
        goal (BasePose): Goal pose.
        xy_m (float): Position tolerance (m).
        yaw_rad (float): Heading tolerance (rad).

    Returns:
        bool: True when both hold; False for poses in different frames.
    """
    if pose.frame != goal.frame:
        return False
    yaw_error = math.atan2(math.sin(pose.yaw - goal.yaw), math.cos(pose.yaw - goal.yaw))
    return math.hypot(pose.x - goal.x, pose.y - goal.y) <= xy_m and abs(yaw_error) <= yaw_rad


def run_nav(
    port: NavPort,
    goal: BasePose,
    timeout_s: float,
    stop_requested: Callable[[], bool],
    interrupt: Callable[[], str | None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    poll_s: float,
    early_xy_m: float | None = None,
    early_yaw_rad: float | None = None,
) -> NavigationResult:
    """Run one navigation goal to its end: result, stop request, timeout or a critical event (cancel + zero twist).

    With both early tolerances set the goal also ends as soon as the measured pose (same frame as the goal) is within
    early_xy_m and early_yaw_rad of it: the goal is cancelled, the base zeroed and the result is 'succeeded'. Nav2's
    own goal checker is much tighter (1 cm / 2 deg) and its final approach is the slow part of a move.

    Args:
        port (NavPort): Nav2 goal access.
        goal (BasePose): Goal pose.
        timeout_s (float): Give up after this many seconds (goal cancelled).
        stop_requested (Callable[[], bool]): True after the stop tool.
        interrupt (Callable[[], str | None]): Critical event type that ends the goal early, or None.
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps.
        poll_s (float): Poll period.
        early_xy_m (float | None): Position tolerance (m) that ends the goal early; None waits for Nav2's result.
        early_yaw_rad (float | None): Heading tolerance (rad) that ends the goal early; None waits for Nav2's result.

    Returns:
        NavigationResult: Outcome with expected (goal), achieved (final pose) and duration.
    """
    if not port.server_ready():
        raise RobotError("Nav2 action server not available")
    accepted = port.send_goal(goal)
    if accepted is None:
        raise RobotError("Nav2 did not answer the goal request")
    started = now()
    if not accepted:
        return nav_result("rejected", "Nav2 rejected the goal", goal, None, 0.0)
    deadline = started + timeout_s
    while not port.result_ready():
        event = interrupt()
        if event is not None:
            port.cancel()
            port.zero_velocity()
            return nav_result(
                "interrupted",
                f"stopped early: critical event {event}; goal cancelled, base zeroed",
                goal,
                port.pose(),
                now() - started,
                event,
            )
        stopped = stop_requested()
        if stopped or now() >= deadline:
            port.cancel()
            reason = "stop requested" if stopped else f"timeout after {timeout_s:.0f} s"
            return nav_result(
                "canceled" if stopped else "timeout", f"{reason}; goal cancelled", goal, port.pose(), now() - started
            )
        if early_xy_m is not None and early_yaw_rad is not None:
            pose = port.pose()
            if pose is not None and within_tolerance(pose, goal, early_xy_m, early_yaw_rad):
                port.cancel()
                port.zero_velocity()
                message = (
                    f"within the intermediate tolerance ({early_xy_m * 100:g} cm, {math.degrees(early_yaw_rad):g} deg); "
                    "goal ended early. Pass precise=true for the tight Nav2 tolerance"
                )
                return nav_result("succeeded", message, goal, port.pose() or pose, now() - started)
        sleep(poll_s)
    status, error = port.result()
    return nav_result(status, error, goal, port.pose(), now() - started)


def straight_move(dx: float, dy: float, dyaw: float) -> bool:
    """Whether a relative move runs as one straight twist instead of a Nav2 goal: no heading change, a translation of
    at most STRAIGHT_MOVE_MAX_M, backing (dx < 0) or with a sideways component (dy != 0).

    Args:
        dx (float): Forward (m).
        dy (float): Left (m).
        dyaw (float): Yaw change (rad).

    Returns:
        bool: True for a straight backing or sideways move.
    """
    return dyaw == 0.0 and 0.0 < math.hypot(dx, dy) <= STRAIGHT_MOVE_MAX_M and (dx < 0.0 or dy != 0.0)


def needs_in_place_turn(dx: float, dy: float, dyaw: float) -> bool:
    """Whether a relative Nav2 goal turns the robot in place: a heading change (RotationShimController
    rotate_to_goal_heading), or a goal direction more than SHIM_TURN_THRESHOLD_RAD off the heading (the shim turns to
    face the path first; the straight line to the goal stands in for the path heading).

    Args:
        dx (float): Forward (m).
        dy (float): Left (m).
        dyaw (float): Yaw change (rad).

    Returns:
        bool: True when the goal starts or ends with an in-place rotation.
    """
    return dyaw != 0.0 or (math.hypot(dx, dy) > 0.0 and abs(math.atan2(dy, dx)) > SHIM_TURN_THRESHOLD_RAD)


def run_translate(
    publish: Callable[[float, float, float], None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    should_abort: Callable[[], bool],
    interrupt: Callable[[], str | None],
    pose: Callable[[], BasePose | None],
    start: BasePose,
    dx: float,
    dy: float,
    speed_mps: float,
    rate_hz: float,
    xy_tolerance_m: float,
    yaw_tolerance_rad: float,
    timeout_s: float,
) -> NavigationResult:
    """Translate straight by (dx, dy) in the start base_link frame with a constant twist (no rotation), measured on the
    pose: the twist stops once the progress along the direction is within xy_tolerance_m of the distance. A stop
    request, a critical event, a lost pose or the timeout end it early; a zero twist always follows.

    Args:
        publish (Callable[[float, float, float], None]): Sends one twist (vx, vy, wz) on cmd_vel_nav.
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps.
        should_abort (Callable[[], bool]): True after the stop tool.
        interrupt (Callable[[], str | None]): Critical event type that ends the move, or None.
        pose (Callable[[], BasePose | None]): Current fresh map pose, None when unknown.
        start (BasePose): Pose at the start (map).
        dx (float): Forward displacement (m).
        dy (float): Left displacement (m).
        speed_mps (float): Translation speed (m/s, > 0).
        rate_hz (float): Publish rate.
        xy_tolerance_m (float): Position tolerance of the goal (m).
        yaw_tolerance_rad (float): Heading tolerance of the goal (rad).
        timeout_s (float): Longest move (s).

    Returns:
        NavigationResult: succeeded (the final pose within the tolerances of the goal), aborted (lost pose, or ended
            outside the tolerances), canceled (stop), interrupted (critical event) or timeout.
    """
    distance = math.hypot(dx, dy)
    ux, uy = dx / distance, dy / distance
    gx, gy, gyaw = compose_relative(start.x, start.y, start.yaw, dx, dy, 0.0)
    goal = BasePose(frame=start.frame, x=gx, y=gy, yaw=gyaw)
    period = 1.0 / rate_hz
    started = now()
    status, message, event = "succeeded", "", None
    try:
        while True:
            if should_abort():
                status, message = "canceled", "stop requested; base zeroed"
                break
            event = interrupt()
            if event is not None:
                status, message = "interrupted", f"stopped early: critical event {event}; base zeroed"
                break
            current = pose()
            if current is None:
                status, message = "aborted", "robot pose unavailable: cannot measure the translation; base zeroed"
                break
            rx, ry, _ = relative_pose(start.x, start.y, start.yaw, current.x, current.y, current.yaw)
            if rx * ux + ry * uy >= distance - xy_tolerance_m:
                break
            if now() - started >= timeout_s:
                status, message = "timeout", f"translation not finished after {timeout_s:g} s; base zeroed"
                break
            publish(ux * speed_mps, uy * speed_mps, 0.0)
            sleep(period)
    finally:
        publish(0.0, 0.0, 0.0)
    final = pose()
    if status == "succeeded":
        if final is None or not within_tolerance(final, goal, xy_tolerance_m, yaw_tolerance_rad):
            error = "unknown" if final is None else f"{math.hypot(final.x - goal.x, final.y - goal.y):.3f} m"
            status = "aborted"
            message = f"straight translation ended outside the goal tolerance ({error} from the goal)"
        else:
            message = "straight translation on cmd_vel_nav without turning (velocity smoother + collision monitor)"
    return nav_result(status, message, goal, final, now() - started, event)
