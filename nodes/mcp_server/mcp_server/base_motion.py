"""Timed base velocity streaming (drive tool) and the stop sequence: clamp, publish at a fixed rate, always finish
with a zero twist."""

import math
from collections.abc import Callable
from typing import Literal, Protocol

from pydantic import BaseModel

from .geometry import clamp_twist
from .models import BasePose, NavigationResult, RobotError, StopResult


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


def run_nav(
    port: NavPort,
    goal: BasePose,
    timeout_s: float,
    stop_requested: Callable[[], bool],
    interrupt: Callable[[], str | None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    poll_s: float,
) -> NavigationResult:
    """Run one navigation goal to its end: result, stop request, timeout or a critical event (cancel + zero twist).

    Args:
        port (NavPort): Nav2 goal access.
        goal (BasePose): Goal pose.
        timeout_s (float): Give up after this many seconds (goal cancelled).
        stop_requested (Callable[[], bool]): True after the stop tool.
        interrupt (Callable[[], str | None]): Critical event type that ends the goal early, or None.
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps.
        poll_s (float): Poll period.

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
        sleep(poll_s)
    status, error = port.result()
    return nav_result(status, error, goal, port.pose(), now() - started)
