"""Tests for mcp_server.motion_queue: FIFO executor, blend groups, preconditions, events, stop/cancel and waits."""

import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
from pydantic import TypeAdapter

from mcp_server.arm import ArmError
from mcp_server.config import McpServerConfig
from mcp_server.models import BasePose, NavigationResult
from mcp_server.motion_queue import (
    LiveState,
    MotionQueue,
    MotionStep,
    QueueError,
    blend_groups,
    evaluate_precondition,
    step_floor,
)

from .test_tools import FakeRobot

STEPS = TypeAdapter(list[MotionStep])
WAIT = 5.0


def steps(*raw: dict[str, Any]) -> list[MotionStep]:
    return STEPS.validate_python(list(raw))


class Cutoff:
    """BatteryGuard double."""

    def __init__(self, cutoff: bool = False) -> None:
        self.cutoff = cutoff

    def is_cutoff(self) -> bool:
        return self.cutoff

    def rejection_message(self) -> str:
        return "battery below cut-off: 9.00 V; motion refused"

    def state(self) -> dict[str, Any]:
        return {"voltage": 9.0 if self.cutoff else 12.0, "stale": False, "cutoff": self.cutoff}


class GateRobot(FakeRobot):
    """FakeRobot whose base motions block until released (or stopped)."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.gate = threading.Event()
        self.entered = threading.Event()
        self.stopped = threading.Event()

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("navigate", (x, y, yaw, frame, timeout_s, precise)))
        self.entered.set()
        while not self.gate.is_set():
            if self.stopped.is_set():
                return NavigationResult(status="canceled", message="stop requested")
            self.gate.wait(0.01)
        return NavigationResult(status="succeeded", goal=BasePose(frame=frame, x=x, y=y, yaw=yaw))

    def stop(self) -> Any:
        self.stops += 1
        self.stopped.set()
        return super().stop()


@pytest.fixture
def robot(tmp_path: Path) -> GateRobot:
    r = GateRobot(tmp_path)
    r.gate.set()
    return r


def make_queue(robot: FakeRobot, **kwargs: Any) -> MotionQueue:
    return MotionQueue(robot, robot.arm.cfg, **kwargs)


def types(events: list[Any]) -> list[str]:
    return [e.type for e in events]


def spy_blends(robot: FakeRobot) -> list[list[dict[str, float]]]:
    calls: list[list[dict[str, float]]] = []
    original = robot.arm.move_blend

    def spy(targets: list[dict[str, float]], *args: Any, **kwargs: Any) -> Any:
        calls.append([dict(t) for t in targets])
        return original(targets, *args, **kwargs)

    robot.arm.move_blend = spy  # type: ignore[method-assign]
    return calls


# --- step models and grouping -------------------------------------------------------------------------------------


def test_step_models_validate_kinds_and_reject_unknown_fields() -> None:
    parsed = steps(
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}},
        {"kind": "gripper", "open_fraction": 0.5},
        {"kind": "wait_s", "seconds": 0.5},
        {"kind": "base_relative", "dx": 0.1},
        {"kind": "navigate_to_pose", "x": 1.0, "y": 2.0},
        {"kind": "arm_cartesian", "x": 0.2, "y": 0.0, "z": 0.0, "precondition": {"type": "gripper_open"}},
    )
    assert [s.kind for s in parsed] == [
        "arm_joints",
        "gripper",
        "wait_s",
        "base_relative",
        "navigate_to_pose",
        "arm_cartesian",
    ]
    assert parsed[0].on_fail == "stop_queue" and parsed[0].precondition.type == "none"
    with pytest.raises(ValueError):
        steps({"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}, "bogus": 1})
    with pytest.raises(ValueError):
        steps({"kind": "teleport"})
    with pytest.raises(ValueError):
        steps({"kind": "gripper"})  # neither open_fraction nor close_until_effort
    with pytest.raises(ValueError):
        steps({"kind": "wait_s", "seconds": 0.0})


def test_arm_gripper_and_grasp_steps_carry_surfaces_into_the_floor_override() -> None:
    surface = {"name": "stair", "height_m": -0.1, "edge": {"point": [0.2, 0.0], "direction": [0.0, -1.0]}}
    arm, gripper = steps(
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}, "surfaces": [surface]},
        {"kind": "gripper", "open_fraction": 0.5, "surfaces": [surface]},
    )
    floor = step_floor(arm)
    assert floor is not None and floor.surfaces is not None and floor.surfaces[0].name == "stair"
    assert step_floor(gripper) == floor
    assert step_floor(steps({"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}})[0]) is None


def test_blend_groups_split_on_settle_final_gripper_precondition_and_speed(robot: GateRobot) -> None:
    parsed = steps(
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}, "settle": "final"},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.3}},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
        {"kind": "gripper", "open_fraction": 0.3},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.5}},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.6}, "precondition": {"type": "battery_ok"}},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.7}, "speed_scale": 0.2},
        {"kind": "arm_joints", "targets": {"gripper": 0.7}},
        {"kind": "arm_joints", "targets": {"shoulder_pan": 0.1}},
        {"kind": "wait_s", "seconds": 0.1},
    )
    groups = blend_groups(parsed, robot.arm.cfg)
    assert [len(g) for g in groups] == [2, 2, 1, 1, 1, 1, 1, 1, 1]


# --- execution ------------------------------------------------------------------------------------------------------


def test_enqueue_returns_at_once_and_runs_everything_in_order(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    result = queue.enqueue(
        steps(
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
            {"kind": "wait_s", "seconds": 0.01},
        )
    )
    assert len(result.job_ids) == 3 and result.queue_length == 3
    assert robot.entered.wait(WAIT)
    assert queue.busy()
    robot.gate.set()
    done = queue.wait(WAIT, "queue_empty")
    assert not done.timed_out
    kinds = [(e.type, e.job_id) for e in done.events if e.type in ("step_started", "step_done")]
    assert kinds == [(t, j) for j in result.job_ids for t in ("step_started", "step_done")]
    assert done.events[-1].type == "queue_empty"
    assert not queue.busy()


def test_consecutive_arm_steps_run_as_one_blended_motion(robot: GateRobot) -> None:
    calls = spy_blends(robot)
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.6}, "settle": "final"},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.3}},
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert [len(c) for c in calls] == [3, 1]
    assert types(result.events).count("step_done") == 4
    assert robot.arm_backend.commands[-1]["shoulder_pan"] == pytest.approx(0.3)


def test_cartesian_steps_are_solved_at_enqueue_and_blended(robot: GateRobot) -> None:
    calls = spy_blends(robot)
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {"kind": "arm_cartesian", "x": 0.2, "y": 0.0, "z": 0.05},
            {"kind": "arm_cartesian", "x": 0.2, "y": 0.0, "z": 0.0},
        )
    )
    queue.wait(WAIT, "queue_empty")
    assert len(calls) == 1 and len(calls[0]) == 2
    assert set(calls[0][0]) >= {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"}


def test_infeasible_cartesian_target_is_refused_at_enqueue_with_reasons(robot: GateRobot) -> None:
    queue = make_queue(robot)
    with pytest.raises(QueueError, match="step 1.*unreachable"):
        queue.enqueue(
            steps(
                {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
                {"kind": "arm_cartesian", "x": 2.0, "y": 0.0, "z": 0.0},
            )
        )
    assert queue.status().queue == [] and not queue.busy()
    assert robot.arm_backend.commands == []


def test_invalid_joint_names_are_refused_at_enqueue(robot: GateRobot) -> None:
    queue = make_queue(robot)
    with pytest.raises(QueueError, match="knee"):
        queue.enqueue(steps({"kind": "arm_joints", "targets": {"knee": 0.2}}))


def test_queue_length_is_capped(robot: GateRobot) -> None:
    queue = make_queue(robot)
    too_many = [{"kind": "wait_s", "seconds": 0.01}] * (robot.arm.cfg.motion_queue.max_steps + 1)
    with pytest.raises(QueueError, match="at most"):
        queue.enqueue(steps(*too_many))


# --- preconditions --------------------------------------------------------------------------------------------------


def test_failed_precondition_stops_the_queue_by_default(robot: GateRobot) -> None:
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}, "precondition": {"type": "gripper_holding"}},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert "precondition_failed" in types(result.events)
    assert "queue_stopped" in types(result.events)
    assert robot.arm_backend.commands == []
    assert not queue.busy()


def test_failed_precondition_with_skip_continues(robot: GateRobot) -> None:
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {
                "kind": "arm_joints",
                "targets": {"shoulder_pan": 0.2},
                "precondition": {"type": "gripper_holding"},
                "on_fail": "skip",
            },
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert "step_skipped" in types(result.events)
    assert robot.arm_backend.commands[-1]["shoulder_pan"] == pytest.approx(0.4)


def test_skipped_precondition_does_not_wake_a_queue_empty_wait_while_steps_remain(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {
                "kind": "arm_joints",
                "targets": {"shoulder_pan": 0.2},
                "precondition": {"type": "gripper_holding"},
                "on_fail": "skip",
            },
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
        )
    )
    assert robot.entered.wait(WAIT)
    early = queue.wait(0.2, "queue_empty")
    assert early.timed_out and early.status.running
    assert queue.wait(0.2, "failure", since_seq=0).reason == "precondition_failed"
    robot.gate.set()
    done = queue.wait(WAIT, "queue_empty", since_seq=0)
    assert done.reason == "queue_empty" and not done.status.running
    assert {"precondition_failed", "step_skipped"} <= set(types(done.events))


def test_skipped_step_failure_does_not_wake_a_queue_empty_wait_while_steps_remain(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)

    def refuse(*args: Any, **kwargs: Any) -> Any:
        raise ArmError("refused for the test")

    robot.arm.move_blend = refuse  # type: ignore[method-assign]
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}, "on_fail": "skip"},
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
        )
    )
    assert robot.entered.wait(WAIT)
    early = queue.wait(0.2, "queue_empty")
    assert early.timed_out and early.status.running
    robot.gate.set()
    done = queue.wait(WAIT, "queue_empty", since_seq=0)
    assert done.reason == "queue_empty" and not done.status.running
    assert {"step_failed", "step_skipped"} <= set(types(done.events))


def test_precondition_predicates() -> None:
    cfg = McpServerConfig()
    open_jaw = LiveState(joints={"shoulder_pan": 0.1, "gripper": 1.0}, gripper_effort=0.0, battery_ok=True)
    holding = LiveState(joints={"shoulder_pan": 0.1, "gripper": 0.3}, gripper_effort=400.0, battery_ok=True)
    pre = TypeAdapter(MotionStep).validate_python
    check: Callable[[dict[str, Any], LiveState], tuple[bool, str]] = lambda p, s: evaluate_precondition(  # noqa: E731
        pre({"kind": "wait_s", "seconds": 1.0, "precondition": p}).precondition, s, cfg
    )
    assert check({"type": "none"}, open_jaw)[0]
    assert check({"type": "gripper_open", "min_fraction": 0.5}, open_jaw)[0]
    assert not check({"type": "gripper_open", "min_fraction": 0.9}, open_jaw)[0]
    assert check({"type": "gripper_holding"}, holding)[0]
    assert not check({"type": "gripper_holding"}, open_jaw)[0]
    assert check({"type": "arm_near", "joints": {"shoulder_pan": 0.12}, "tol": 0.05}, open_jaw)[0]
    ok, reason = check({"type": "arm_near", "joints": {"shoulder_pan": 0.5}, "tol": 0.05}, open_jaw)
    assert not ok and "shoulder_pan" in reason
    assert check({"type": "base_still"}, LiveState(base_twist=(0.0, 0.0, 0.01)))[0]
    assert not check({"type": "base_still"}, LiveState(base_twist=(0.2, 0.0, 0.0)))[0]
    assert not check({"type": "base_still"}, LiveState())[0]  # unknown is not still
    assert check({"type": "battery_ok"}, LiveState(battery_ok=True))[0]
    assert not check({"type": "battery_ok"}, LiveState(battery_ok=False))[0]
    assert not check({"type": "gripper_open"}, LiveState())[0]  # no joint states


# --- failures, contact, stop and cancel ------------------------------------------------------------------------------


def test_guard_abort_is_a_failure_event_and_stops_the_queue(robot: GateRobot) -> None:
    robot.arm_backend.sag = {"shoulder_pan": -0.6}
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.9}},
            {"kind": "wait_s", "seconds": 0.01},
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    failed = [e for e in result.events if e.type == "step_failed"]
    assert failed and failed[0].data["status"] == "aborted_tracking"
    assert "queue_stopped" in types(result.events)
    assert not any(c[0] == "navigate" for c in robot.calls)


def test_gripper_contact_produces_a_contact_event(robot: GateRobot) -> None:
    be = robot.arm_backend
    be.positions["gripper"] = 1.0

    def squeeze(backend: Any) -> None:
        if backend.positions["gripper"] < 0.5:
            backend.follow = False
            backend.efforts["gripper"] = 500.0

    be.on_sleep = squeeze
    queue = make_queue(robot)
    queue.enqueue(steps({"kind": "gripper", "close_until_effort": True}))
    result = queue.wait(WAIT, "queue_empty")
    assert "contact" in types(result.events)
    assert "step_done" in types(result.events)


def test_stop_clears_the_queue_immediately(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(
        steps(
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
        )
    )
    assert robot.entered.wait(WAIT)
    dropped = queue.halt_for_stop()
    assert len(dropped) == 1
    assert queue.status().queue == []
    robot.stop()
    result = queue.wait(WAIT, "queue_empty")
    assert "stopped" in types(result.events)
    assert robot.arm_backend.commands == []
    assert not queue.busy()


def test_external_stop_between_steps_halts_the_queue(robot: GateRobot) -> None:
    queue = make_queue(robot)

    def stop_after_first(backend: Any) -> None:
        if len(backend.commands) == 3:
            robot.stops += 1

    robot.arm_backend.on_sleep = stop_after_first
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}, "settle": "final"},
            {"kind": "wait_s", "seconds": 0.01},
            {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert "stopped" in types(result.events)
    assert not any(c[0] == "navigate" for c in robot.calls)


def test_cancel_aborts_the_running_step_and_drops_the_rest(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(
        steps({"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}, {"kind": "wait_s", "seconds": 0.01}),
    )
    assert robot.entered.wait(WAIT)
    cancelled = queue.cancel()
    assert len(cancelled.dropped) == 1 and cancelled.running is not None
    result = queue.wait(WAIT, "queue_empty")
    assert "cancelled" in types(result.events)
    assert robot.stopped.is_set()
    assert not queue.busy()


def test_cancel_when_idle_does_not_stop_the_robot(robot: GateRobot) -> None:
    queue = make_queue(robot)
    result = queue.cancel()
    assert result.dropped == [] and result.running is None
    assert not robot.stopped.is_set()


def test_replace_drops_pending_steps_and_keeps_the_running_one(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    first = queue.enqueue(
        steps({"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}, {"kind": "navigate_to_pose", "x": 2.0, "y": 0.0})
    )
    assert robot.entered.wait(WAIT)
    second = queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}), replace=True)
    assert second.replaced == [first.job_ids[1]]
    robot.gate.set()
    result = queue.wait(WAIT, "queue_empty")
    assert [c[1][0] for c in robot.calls if c[0] == "navigate"] == [1.0]
    assert first.job_ids[0] in [e.job_id for e in result.events if e.type == "step_done"]


def test_battery_cutoff_at_dispatch_fails_the_step(robot: GateRobot) -> None:
    guard = Cutoff(cutoff=True)
    queue = make_queue(robot, guard=guard)
    queue.enqueue(steps({"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}}))
    result = queue.wait(WAIT, "queue_empty")
    failed = [e for e in result.events if e.type == "step_failed"]
    assert failed and "battery" in failed[0].message
    assert robot.arm_backend.commands == []


def test_enqueue_refused_while_a_blocking_motion_tool_runs(robot: GateRobot) -> None:
    queue = make_queue(robot, external_busy=lambda: "move_arm_joints")
    with pytest.raises(QueueError, match="move_arm_joints"):
        queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}))


def test_enqueue_refused_while_another_arm_motion_holds_the_arm(robot: GateRobot) -> None:
    queue = make_queue(robot)
    with robot.arm.exclusive_motion(), pytest.raises(QueueError, match="arm motion"):
        queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}))


def test_grasp_step_runs_through_the_runner(robot: GateRobot) -> None:
    seen: list[Any] = []

    def runner(step: Any, stop_requested: Callable[[], bool]) -> tuple[str, list[str]]:
        seen.append(step)
        assert stop_requested() is False
        return ("grasped", [])

    queue = make_queue(robot, grasp_runner=runner)
    queue.enqueue(
        steps(
            {
                "kind": "grasp",
                "object": {"x": 0.2, "y": 0.0, "support_z": -0.15, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.03},
            }
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert len(seen) == 1
    assert {"step_done", "contact"} <= set(types(result.events))


def test_missed_grasp_is_a_failure(robot: GateRobot) -> None:
    queue = make_queue(robot, grasp_runner=lambda step, stop: ("missed", ["closed on nothing"]))
    queue.enqueue(
        steps(
            {
                "kind": "grasp",
                "object": {"x": 0.2, "y": 0.0, "support_z": -0.15, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.03},
            }
        )
    )
    result = queue.wait(WAIT, "queue_empty")
    assert "step_failed" in types(result.events)


def test_grasp_step_refused_without_a_runner(robot: GateRobot) -> None:
    queue = make_queue(robot)
    with pytest.raises(QueueError, match="grasp"):
        queue.enqueue(
            steps(
                {
                    "kind": "grasp",
                    "object": {
                        "x": 0.2,
                        "y": 0.0,
                        "support_z": -0.15,
                        "width_m": 0.03,
                        "depth_m": 0.03,
                        "height_m": 0.03,
                    },
                }
            )
        )


# --- waiting and status ------------------------------------------------------------------------------------------------


def test_wait_returns_immediately_when_idle(robot: GateRobot) -> None:
    queue = make_queue(robot)
    for until in ("queue_empty", "failure", "step_done", "any"):
        result = queue.wait(0.5, until)
        assert result.reason == "idle" and not result.timed_out


def test_wait_step_done_wakes_after_the_first_step(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}, {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}))
    result = queue.wait(WAIT, "step_done")
    assert result.reason == "step_done" and not result.timed_out
    assert queue.busy()
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")


def test_wait_times_out_while_the_queue_is_still_busy(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(steps({"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}))
    assert robot.entered.wait(WAIT)
    result = queue.wait(0.1, "queue_empty")
    assert result.timed_out
    robot.gate.set()
    assert not queue.wait(WAIT, "queue_empty").timed_out


def test_wait_only_returns_events_not_seen_by_an_earlier_wait(robot: GateRobot) -> None:
    queue = make_queue(robot)
    queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}))
    first = queue.wait(WAIT, "queue_empty")
    assert first.events
    queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}))
    second = queue.wait(WAIT, "queue_empty")
    assert min(e.seq for e in second.events) > max(e.seq for e in first.events)


def test_status_reports_running_step_progress_and_queue(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = make_queue(robot)
    queue.enqueue(steps({"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}, {"kind": "wait_s", "seconds": 0.01}))
    assert robot.entered.wait(WAIT)
    status = queue.status()
    assert status.running and status.current is not None and status.current.kind == "navigate_to_pose"
    assert status.current.elapsed_s >= 0.0
    assert [s.kind for s in status.queue] == ["wait_s"]
    assert status.last_events
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")
