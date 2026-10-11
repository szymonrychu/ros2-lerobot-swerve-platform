"""Blocking DockRobot loop (nav2_msgs/action/DockRobot) over a DockPort, mirroring base_motion.run_nav.

The loop never publishes a velocity itself except a zero twist after a critical event; the docking server drives.
"""

from collections.abc import Callable
from typing import Literal, Protocol

from pydantic import BaseModel

from .metrics import DOCK_DURATION, DOCK_GOALS
from .models import BasePose, RobotError

DOCK_ERROR_NAMES: dict[int, str] = {
    0: "NONE",
    901: "DOCK_NOT_IN_DB",
    902: "DOCK_NOT_VALID",
    903: "FAILED_TO_STAGE",
    904: "FAILED_TO_DETECT_DOCK",
    905: "FAILED_TO_CONTROL",
    906: "FAILED_TO_CHARGE",
    999: "UNKNOWN",
}
DOCK_STATES = ("NAV_TO_STAGING_POSE", "INITIAL_PERCEPTION", "CONTROLLING", "WAIT_FOR_CHARGE", "RETRY")

DockStatus = Literal["succeeded", "failed", "rejected", "interrupted", "canceled", "timeout"]


class DockGoal(BaseModel):
    """One DockRobot goal addressed by pose (use_dock_id false)."""

    dock_pose: BasePose
    dock_type: str
    navigate_to_staging_pose: bool = True
    max_staging_time_s: float = 1000.0


class DockResult(BaseModel):
    """Outcome of a dock run."""

    status: DockStatus
    error_code: int = 0
    error_name: str = "NONE"
    num_retries: int = 0
    message: str = ""
    goal: DockGoal
    final_pose: BasePose | None = None
    duration_s: float = 0.0
    last_state: str | None = None
    interrupted_by: str | None = None


class DockPort(Protocol):
    """One DockRobot goal as run_dock needs it (implemented over rclpy futures in ros_dock, faked in tests)."""

    def server_ready(self) -> bool:
        """Whether the docking action server is available."""
        ...

    def send_goal(self, goal: DockGoal) -> bool | None:
        """Send the goal; True accepted, False rejected, None when the server did not answer."""
        ...

    def result_ready(self) -> bool:
        """Whether the goal has finished."""
        ...

    def result(self) -> tuple[str, int, int, str]:
        """Final goal status name (succeeded, aborted, canceled), error code, retries and error message."""
        ...

    def feedback_state(self) -> str | None:
        """Latest feedback state name (one of DOCK_STATES), or None when none arrived."""
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


def dock_error_name(code: int) -> str:
    """Name of a DockRobot error code.

    Args:
        code (int): DockRobot error code.

    Returns:
        str: Known name, or UNKNOWN_<code>.
    """
    return DOCK_ERROR_NAMES.get(code, f"UNKNOWN_{code}")


def dock_result(
    status: DockStatus,
    message: str,
    goal: DockGoal,
    final: BasePose | None,
    duration_s: float,
    last_state: str | None,
    error_code: int = 0,
    num_retries: int = 0,
    interrupted_by: str | None = None,
) -> DockResult:
    """Build a DockResult and record the dock metrics.

    Args:
        status (DockStatus): Result status.
        message (str): Explanation.
        goal (DockGoal): Requested goal.
        final (BasePose | None): Final measured pose.
        duration_s (float): Seconds from goal start to finish.
        last_state (str | None): Last feedback state seen.
        error_code (int): DockRobot error code.
        num_retries (int): Retries the server made.
        interrupted_by (str | None): Critical event type for status 'interrupted'.

    Returns:
        DockResult: Result.
    """
    DOCK_GOALS.labels(status).inc()
    DOCK_DURATION.observe(duration_s)
    return DockResult(
        status=status,
        error_code=error_code,
        error_name=dock_error_name(error_code),
        num_retries=num_retries,
        message=message,
        goal=goal,
        final_pose=final,
        duration_s=round(duration_s, 3),
        last_state=last_state,
        interrupted_by=interrupted_by,
    )


def run_dock(
    port: DockPort,
    goal: DockGoal,
    timeout_s: float,
    stop_requested: Callable[[], bool],
    interrupt: Callable[[], str | None],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    poll_s: float,
) -> DockResult:
    """Run one docking goal to its end: result, stop request, timeout or a critical event (cancel + zero twist).

    Args:
        port (DockPort): Docking server access.
        goal (DockGoal): Goal to send.
        timeout_s (float): Give up after this many seconds (goal cancelled).
        stop_requested (Callable[[], bool]): True after the stop tool.
        interrupt (Callable[[], str | None]): Critical event type that ends the goal early, or None.
        now (Callable[[], float]): Monotonic clock (s).
        sleep (Callable[[float], None]): Sleeps.
        poll_s (float): Poll period.

    Returns:
        DockResult: Outcome with error code and name, retries, final pose, duration and last feedback state.

    Raises:
        RobotError: Docking server unavailable or it did not answer the goal request.
    """
    if not port.server_ready():
        raise RobotError("docking server not available")
    accepted = port.send_goal(goal)
    if accepted is None:
        raise RobotError("docking server did not answer the goal request")
    started = now()
    if not accepted:
        return dock_result("rejected", "docking server rejected the goal", goal, None, 0.0, None)
    deadline = started + timeout_s
    state: str | None = None
    while not port.result_ready():
        state = port.feedback_state() or state
        event = interrupt()
        if event is not None:
            port.cancel()
            port.zero_velocity()
            message = f"stopped early: critical event {event}; dock cancelled, base zeroed"
            return dock_result("interrupted", message, goal, port.pose(), now() - started, state, interrupted_by=event)
        stopped = stop_requested()
        if stopped or now() >= deadline:
            port.cancel()
            reason = "stop requested" if stopped else f"timeout after {timeout_s:.0f} s"
            return dock_result(
                "canceled" if stopped else "timeout",
                f"{reason}; dock cancelled",
                goal,
                port.pose(),
                now() - started,
                state,
            )
        sleep(poll_s)
    state = port.feedback_state() or state
    status_name, error_code, num_retries, error_msg = port.result()
    status: DockStatus
    if status_name == "canceled":
        status = "canceled"
    elif status_name == "succeeded" and error_code == 0:
        status = "succeeded"
    else:
        status = "failed"
    message = error_msg or ("" if status == "succeeded" else f"docking {status_name}: {dock_error_name(error_code)}")
    return dock_result(
        status, message, goal, port.pose(), now() - started, state, error_code=error_code, num_retries=num_retries
    )
