"""Tests for mcp_server.base_dock.run_dock (blocking DockRobot loop over a DockPort)."""

import pytest

from mcp_server.base_dock import DOCK_ERROR_NAMES, DockGoal, DockResult, dock_error_name, run_dock
from mcp_server.models import BasePose, RobotError

POSE = BasePose(frame="map", x=1.0, y=2.0, yaw=0.5)
GOAL = DockGoal(dock_pose=POSE, dock_type="grasp_servoed", max_staging_time_s=30.0)


class FakeDockPort:
    """Scripted DockPort: finishes after finish_after result_ready polls; records every call."""

    def __init__(self, clock: "Clock") -> None:
        self.clock = clock
        self.ready = True
        self.accepted: bool | None = True
        self.finish_after: int | None = 3
        self.polls = 0
        self.outcome: tuple[str, int, int, str] = ("succeeded", 0, 0, "")
        self.states = ["NAV_TO_STAGING_POSE", "CONTROLLING", None]
        self.calls: list[str] = []
        self.twists: list[tuple[float, float, float]] = []
        self.sent: list[DockGoal] = []

    def server_ready(self) -> bool:
        return self.ready

    def send_goal(self, goal: DockGoal) -> bool | None:
        self.calls.append("send_goal")
        self.sent.append(goal)
        return self.accepted

    def result_ready(self) -> bool:
        self.polls += 1
        return self.finish_after is not None and self.polls > self.finish_after

    def result(self) -> tuple[str, int, int, str]:
        return self.outcome

    def feedback_state(self) -> str | None:
        return self.states[min(self.polls - 1, len(self.states) - 1)] if self.polls else None

    def cancel(self) -> None:
        self.calls.append("cancel")

    def zero_velocity(self) -> None:
        self.calls.append("zero")
        self.twists.append((0.0, 0.0, 0.0))

    def pose(self) -> BasePose | None:
        return POSE


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s


def run(port: FakeDockPort, clock: Clock, timeout_s: float = 60.0, stop=lambda: False, interrupt=lambda: None):
    return run_dock(port, GOAL, timeout_s, stop, interrupt, clock.now, clock.sleep, 0.5)


def make() -> tuple[FakeDockPort, Clock]:
    clock = Clock()
    return FakeDockPort(clock), clock


def test_server_down_raises_before_sending():
    port, clock = make()
    port.ready = False
    with pytest.raises(RobotError, match="docking server"):
        run(port, clock)
    assert port.calls == []


def test_no_answer_raises():
    port, clock = make()
    port.accepted = None
    with pytest.raises(RobotError, match="did not answer"):
        run(port, clock)


def test_rejected():
    port, clock = make()
    port.accepted = False
    result = run(port, clock)
    assert result.status == "rejected"
    assert result.goal == GOAL
    assert "cancel" not in port.calls


def test_success_maps_result_and_tracks_state():
    port, clock = make()
    result = run(port, clock)
    assert isinstance(result, DockResult)
    assert result.status == "succeeded"
    assert result.error_code == 0
    assert result.error_name == "NONE"
    assert result.last_state == "CONTROLLING"
    assert result.final_pose == POSE
    assert result.duration_s == pytest.approx(1.5)
    assert port.sent == [GOAL]
    assert port.calls == ["send_goal"]


@pytest.mark.parametrize(
    ("code", "name"),
    [
        (901, "DOCK_NOT_IN_DB"),
        (902, "DOCK_NOT_VALID"),
        (903, "FAILED_TO_STAGE"),
        (904, "FAILED_TO_DETECT_DOCK"),
        (905, "FAILED_TO_CONTROL"),
        (906, "FAILED_TO_CHARGE"),
        (999, "UNKNOWN"),
    ],
)
def test_failure_error_codes(code, name):
    port, clock = make()
    port.outcome = ("aborted", code, 2, "boom")
    result = run(port, clock)
    assert result.status == "failed"
    assert result.error_code == code
    assert result.error_name == name
    assert result.num_retries == 2
    assert result.message == "boom"


def test_unknown_code_name():
    assert dock_error_name(123) == "UNKNOWN_123"
    assert DOCK_ERROR_NAMES[0] == "NONE"


def test_succeeded_with_error_code_is_failed():
    port, clock = make()
    port.outcome = ("succeeded", 904, 0, "")
    assert run(port, clock).status == "failed"


def test_server_canceled_result():
    port, clock = make()
    port.outcome = ("canceled", 0, 0, "")
    assert run(port, clock).status == "canceled"


def test_interrupt_cancels_zeroes_and_reports_event():
    port, clock = make()
    port.finish_after = None
    events = iter([None, "tilt_warning"])
    result = run(port, clock, interrupt=lambda: next(events))
    assert result.status == "interrupted"
    assert "tilt_warning" in result.message
    assert port.calls == ["send_goal", "cancel", "zero"]
    assert result.last_state == "CONTROLLING"


def test_stop_cancels():
    port, clock = make()
    port.finish_after = None
    result = run(port, clock, stop=lambda: clock.t >= 1.0)
    assert result.status == "canceled"
    assert port.calls == ["send_goal", "cancel"]


def test_timeout_cancels():
    port, clock = make()
    port.finish_after = None
    result = run(port, clock, timeout_s=2.0)
    assert result.status == "timeout"
    assert port.calls == ["send_goal", "cancel"]
    assert result.duration_s == pytest.approx(2.0)


def test_interrupt_wins_over_stop():
    port, clock = make()
    port.finish_after = None
    result = run(port, clock, stop=lambda: True, interrupt=lambda: "tilt_warning")
    assert result.status == "interrupted"


@pytest.mark.parametrize("branch", ["interrupt", "stop", "timeout"])
def test_cancel_always_called_on_early_exit_and_only_zero_twists(branch):
    port, clock = make()
    port.finish_after = None
    run(
        port,
        clock,
        timeout_s=1.0 if branch == "timeout" else 60.0,
        stop=lambda: branch == "stop",
        interrupt=lambda: "tilt_warning" if branch == "interrupt" else None,
    )
    assert "cancel" in port.calls
    assert all(t == (0.0, 0.0, 0.0) for t in port.twists)


def test_metrics_recorded():
    from mcp_server.metrics import DOCK_GOALS

    before = DOCK_GOALS.labels("succeeded")._value.get()
    port, clock = make()
    run(port, clock)
    assert DOCK_GOALS.labels("succeeded")._value.get() == before + 1
