"""Tests for motion queue watch constraints: the watcher thread, for_s debounce, abort and pause, resume, events."""

import statistics
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
from prometheus_client import REGISTRY
from pydantic import TypeAdapter, ValidationError

from mcp_server.config import McpServerConfig
from mcp_server.motion_queue import (
    FAILURE_EVENTS,
    WAKE_EVENTS,
    MotionQueue,
    MotionStep,
    QueueError,
    WatchConstraint,
)

from .test_motion_queue import Cutoff, GateRobot

STEPS = TypeAdapter(list[MotionStep])
WAIT = 5.0
NAV1 = {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}
NAV2 = {"kind": "navigate_to_pose", "x": 2.0, "y": 0.0}


def steps(*raw: dict[str, Any]) -> list[MotionStep]:
    return STEPS.validate_python(list(raw))


def watch(**raw: Any) -> WatchConstraint:
    return WatchConstraint.model_validate(raw)


def block_forever(_seconds: float) -> None:
    """watch_sleep that parks the watcher thread: the test drives watch_tick() itself."""
    threading.Event().wait()


class Clock:
    """Controllable monotonic clock."""

    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


class FakeSampler:
    """WatchSampler double: one value per (metric, joint); a hook runs on every sample."""

    def __init__(self) -> None:
        self.values: dict[tuple[str, str | None], float | None] = {}
        self.calls = 0
        self.hook: Callable[[], None] | None = None

    def set(self, metric: str, value: float | None, joint: str | None = None) -> None:
        self.values[(metric, joint)] = value

    def sample(self, metric: str, joint: str | None) -> float | None:
        self.calls += 1
        if self.hook is not None:
            hook, self.hook = self.hook, None
            hook()
        return self.values.get((metric, joint))


@pytest.fixture
def robot(tmp_path: Path) -> GateRobot:
    r = GateRobot(tmp_path)
    r.gate.clear()  # navigate steps run until the test releases them
    return r


def make_queue(
    robot: GateRobot,
    sampler: FakeSampler,
    clock: Clock | None = None,
    watch_sleep: Callable[[float], None] = block_forever,
    config: McpServerConfig | None = None,
    **kwargs: Any,
) -> tuple[MotionQueue, list[dict[str, Any]]]:
    reports: list[dict[str, Any]] = []
    queue = MotionQueue(
        robot,
        config or robot.arm.cfg,
        sampler=sampler.sample,
        on_watch_trigger=reports.append,
        clock=clock or time.monotonic,
        watch_sleep=watch_sleep,
        **kwargs,
    )
    return queue, reports


def types(events: list[Any]) -> list[str]:
    return [e.type for e in events]


def stops(robot: GateRobot) -> int:
    return sum(1 for c in robot.calls if c[0] == "stop")


def triggers(metric: str, action: str) -> float:
    return REGISTRY.get_sample_value("mcp_motion_watch_triggers_total", {"metric": metric, "action": action}) or 0.0


def wait_idle(queue: MotionQueue) -> None:
    deadline = time.monotonic() + WAIT
    while queue.busy() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert not queue.busy()


def wait_running(queue: MotionQueue, robot: GateRobot) -> None:
    assert robot.entered.wait(WAIT)
    deadline = time.monotonic() + WAIT
    while queue.status().current is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert queue.status().current is not None


# --- the model ----------------------------------------------------------------------------------------------------


def test_watch_constraint_defaults() -> None:
    w = watch(metric="gripper_effort", op="gt", value=60)
    assert (w.name, w.joint, w.for_s, w.action, w.on_unknown) == (None, None, 0.0, "abort", "ignore")


@pytest.mark.parametrize(
    "raw",
    [
        {"metric": "joint_position", "op": "gt", "value": 0.1},  # joint required
        {"metric": "joint_effort", "op": "gt", "value": 1.0, "joint": "tail"},  # not an arm joint
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "joint": "wrist_roll"},  # joint on a joint-less metric
        {"metric": "tilt_deg", "op": "gt", "value": float("nan")},
        {"metric": "tilt_deg", "op": "gt", "value": float("inf")},
        {"metric": "tilt_deg", "op": "eq", "value": 5.0},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "for_s": 5.5},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "for_s": -0.1},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "name": "x" * 41},
        {"metric": "speed", "op": "gt", "value": 5.0},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "action": "stop"},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "on_unknown": "abort"},
        {"metric": "tilt_deg", "op": "gt", "value": 5.0, "extra": 1},
    ],
)
def test_watch_constraint_rejects_invalid(raw: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        WatchConstraint.model_validate(raw)


def test_joint_watch_accepts_an_arm_joint() -> None:
    w = watch(metric="joint_position", joint="wrist_roll", op="gt", value=0.1, action="pause", for_s=0.2)
    assert w.joint == "wrist_roll" and w.action == "pause"


def test_watch_events_wake_wait_for_event() -> None:
    assert {"watch_triggered", "paused"} <= FAILURE_EVENTS
    for until in ("queue_empty", "step_done", "failure"):
        assert {"watch_triggered", "paused"} <= WAKE_EVENTS[until]


def test_enqueue_refuses_more_watches_than_watch_max_per_call(robot: GateRobot) -> None:
    queue, _ = make_queue(robot, FakeSampler())
    many = [watch(metric="tilt_deg", op="gt", value=5.0)] * (robot.arm.cfg.motion_queue.watch_max_per_call + 1)
    with pytest.raises(QueueError, match="watch_constraints"):
        queue.enqueue(steps(NAV1), watch_constraints=many)
    assert not queue.busy()


# --- trips --------------------------------------------------------------------------------------------------------


def test_trip_only_after_for_s(robot: GateRobot) -> None:
    sampler, clock = FakeSampler(), Clock()
    queue, _ = make_queue(robot, sampler, clock)
    sampler.set("gripper_effort", 80.0)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="gripper_effort", op="gt", value=60, for_s=0.5)])
    wait_running(queue, robot)
    queue.watch_tick()
    clock.t += 0.3
    queue.watch_tick()
    assert "watch_triggered" not in types(queue.status().last_events)
    sampler.set("gripper_effort", 50.0)  # back below: the debounce restarts
    clock.t += 0.3
    queue.watch_tick()
    sampler.set("gripper_effort", 80.0)
    clock.t += 0.1
    queue.watch_tick()
    clock.t += 0.4
    queue.watch_tick()
    assert "watch_triggered" not in types(queue.status().last_events)
    clock.t += 0.1
    queue.watch_tick()
    assert "watch_triggered" in types(queue.status().last_events)


def test_abort_drops_pending_and_stops_once(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, reports = make_queue(robot, sampler)
    sampler.set("tilt_deg", 12.0)
    before = triggers("tilt_deg", "abort")
    result = queue.enqueue(
        steps(NAV1, NAV2), watch_constraints=[watch(name="tilt", metric="tilt_deg", op="ge", value=10.0)]
    )
    wait_running(queue, robot)
    queue.watch_tick()
    queue.watch_tick()  # a second tick after the trip does nothing
    done = queue.wait(WAIT, "queue_empty")
    wait_idle(queue)
    events = queue.status().last_events
    tripped = next(e for e in events if e.type == "watch_triggered")
    assert tripped.data == {
        "watch": "tilt",
        "metric": "tilt_deg",
        "value": 12.0,
        "op": "ge",
        "threshold": 10.0,
        "action": "abort",
    }
    cancelled = next(e for e in events if e.type == "cancelled")
    assert cancelled.data["dropped"] == [result.job_ids[1]]
    assert "step_aborted" in types(events)
    assert done.reason == "watch_triggered"
    assert stops(robot) == 1
    assert [c[1][0] for c in robot.calls if c[0] == "navigate"] == [1.0]
    assert triggers("tilt_deg", "abort") == before + 1
    assert len(reports) == 1 and reports[0]["watch"] == "tilt" and reports[0]["action"] == "abort"


def test_pause_keeps_pending_and_resume_continues(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, reports = make_queue(robot, sampler)
    sampler.set("joint_position", 0.2, "wrist_roll")
    before = triggers("joint_position", "pause")
    result = queue.enqueue(
        steps(NAV1, NAV2),
        watch_constraints=[watch(metric="joint_position", joint="wrist_roll", op="gt", value=0.1, action="pause")],
    )
    wait_running(queue, robot)
    queue.watch_tick()
    paused = queue.wait(WAIT, "failure")
    assert paused.reason == "watch_triggered"
    deadline = time.monotonic() + WAIT
    while queue.status().current is not None and time.monotonic() < deadline:
        time.sleep(0.005)
    status = queue.status()
    assert status.paused is not None and status.paused.by == "joint_position[wrist_roll] gt 0.1"
    assert [q.job_id for q in status.queue] == [result.job_ids[1]]
    assert status.running  # a paused queue is busy
    evs = types(status.last_events)
    assert "paused" in evs and "step_aborted" in evs and "cancelled" not in evs
    assert stops(robot) == 1
    assert triggers("joint_position", "pause") == before + 1
    assert reports[0]["action"] == "pause"
    time.sleep(0.1)  # the worker must not dispatch while paused
    assert [c[1][0] for c in robot.calls if c[0] == "navigate"] == [1.0]
    sampler.set("joint_position", 0.05, "wrist_roll")
    robot.stopped.clear()
    robot.gate.set()
    seq = queue.status().event_seq
    resumed = queue.resume()
    assert resumed.resumed and resumed.queue_length == 1
    done = queue.wait(WAIT, "queue_empty", since_seq=seq)
    assert types(done.events)[0] == "resumed"
    assert [c[1][0] for c in robot.calls if c[0] == "navigate"] == [1.0, 2.0]
    assert result.job_ids[1] in [e.job_id for e in done.events if e.type == "step_done"]
    assert queue.status().paused is None


def test_resume_refused_while_a_watch_of_the_pending_steps_violates(robot: GateRobot) -> None:
    sampler = FakeSampler()
    guard = Cutoff()
    queue, _ = make_queue(robot, sampler, guard=guard)
    sampler.set("tilt_deg", 9.0)
    queue.enqueue(steps(NAV1, NAV2), watch_constraints=[watch(metric="tilt_deg", op="gt", value=5, action="pause")])
    wait_running(queue, robot)
    queue.watch_tick()
    with pytest.raises(QueueError, match="tilt_deg"):
        queue.resume()
    assert queue.status().paused is not None
    sampler.set("tilt_deg", 1.0)
    guard.cutoff = True
    with pytest.raises(QueueError, match="battery"):
        queue.resume()
    assert queue.status().paused is not None
    guard.cutoff = False
    robot.gate.set()
    robot.stopped.clear()
    assert queue.resume().resumed


def test_resume_when_not_paused_is_a_no_op(robot: GateRobot) -> None:
    queue, _ = make_queue(robot, FakeSampler())
    result = queue.resume()
    assert not result.resumed and "not paused" in result.message
    assert "resumed" not in types(queue.status().last_events)


def test_stop_and_cancel_clear_the_pause(robot: GateRobot) -> None:
    for end in ("stop", "cancel"):
        robot.stopped.clear()
        sampler = FakeSampler()
        queue, _ = make_queue(robot, sampler)
        sampler.set("tilt_deg", 9.0)
        robot.entered.clear()
        queue.enqueue(steps(NAV1, NAV2), watch_constraints=[watch(metric="tilt_deg", op="gt", value=5, action="pause")])
        wait_running(queue, robot)
        queue.watch_tick()
        assert queue.status().paused is not None
        dropped = queue.halt_for_stop() if end == "stop" else queue.cancel().dropped
        assert len(dropped) == 1
        wait_idle(queue)
        status = queue.status()
        assert status.paused is None and status.queue == []


def test_pause_then_enqueue_appends_and_waits_for_resume(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler)
    sampler.set("tilt_deg", 9.0)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="tilt_deg", op="gt", value=5, action="pause")])
    wait_running(queue, robot)
    queue.watch_tick()
    queue.wait(WAIT, "failure")
    later = queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}))
    time.sleep(0.05)
    assert [q.job_id for q in queue.status().queue] == later.job_ids
    replaced = queue.enqueue(steps({"kind": "wait_s", "seconds": 0.01}), replace=True)
    assert replaced.replaced == later.job_ids
    assert queue.status().paused is not None  # replace keeps the pause
    seq = queue.status().event_seq
    queue.resume()
    done = queue.wait(WAIT, "queue_empty", since_seq=seq)
    assert replaced.job_ids[0] in [e.job_id for e in done.events if e.type == "step_done"]


# --- metrics and unknown values -----------------------------------------------------------------------------------


def test_distance_travelled_integrates_fresh_base_speed_since_activation(robot: GateRobot) -> None:
    sampler, clock = FakeSampler(), Clock()
    queue, _ = make_queue(robot, sampler, clock)
    sampler.set("base_speed_mps", 0.2)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="distance_travelled_m", op="gt", value=0.5)])
    wait_running(queue, robot)
    queue.watch_tick()  # activation: distance 0
    assert queue.status().watches[0].last_value == 0.0
    clock.t += 1.0
    queue.watch_tick()
    clock.t += 1.0
    queue.watch_tick()
    assert queue.status().watches[0].last_value == pytest.approx(0.4)
    sampler.set("base_speed_mps", None)  # stale odometry: unknown, the gap is not integrated
    clock.t += 5.0
    queue.watch_tick()
    assert queue.status().watches[0].last_value is None
    sampler.set("base_speed_mps", 0.2)
    clock.t += 1.0
    queue.watch_tick()  # first fresh sample after the gap restarts the integration
    assert queue.status().watches[0].last_value == pytest.approx(0.4)
    assert "watch_triggered" not in types(queue.status().last_events)
    clock.t += 1.0
    queue.watch_tick()
    assert "watch_triggered" in types(queue.status().last_events)
    tripped = next(e for e in queue.status().last_events if e.type == "watch_triggered")
    assert tripped.data["value"] == pytest.approx(0.6)


@pytest.mark.parametrize(("on_unknown", "trips"), [("ignore", False), ("trigger", True)])
def test_unknown_value_handling(robot: GateRobot, on_unknown: str, trips: bool) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler)
    sampler.set("gripper_effort", None)  # NaN efforts arrive as None from the sampler
    queue.enqueue(
        steps(NAV1),
        watch_constraints=[watch(metric="gripper_effort", op="gt", value=60, on_unknown=on_unknown)],
    )
    wait_running(queue, robot)
    queue.watch_tick()
    events = queue.status().last_events
    assert ("watch_triggered" in types(events)) is trips
    if trips:
        assert next(e for e in events if e.type == "watch_triggered").data["value"] is None
    else:
        robot.gate.set()
        assert queue.wait(WAIT, "queue_empty").reason in ("queue_empty", "idle")


def test_unknown_ignore_keeps_the_debounce_running(robot: GateRobot) -> None:
    sampler, clock = FakeSampler(), Clock()
    queue, _ = make_queue(robot, sampler, clock)
    sampler.set("gripper_effort", 80.0)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="gripper_effort", op="gt", value=60, for_s=0.2)])
    wait_running(queue, robot)
    queue.watch_tick()
    sampler.set("gripper_effort", None)
    clock.t += 0.1
    queue.watch_tick()
    sampler.set("gripper_effort", 80.0)
    clock.t += 0.1
    queue.watch_tick()
    assert "watch_triggered" in types(queue.status().last_events)


# --- scope, races, status -----------------------------------------------------------------------------------------


def test_trip_racing_the_step_end_is_dropped(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, reports = make_queue(robot, sampler)
    sampler.set("tilt_deg", 9.0)
    queue.enqueue(
        steps(NAV1, {"kind": "wait_s", "seconds": 0.0001}),
        watch_constraints=[watch(metric="tilt_deg", op="gt", value=5)],
    )
    wait_running(queue, robot)

    def end_step() -> None:  # the step ends between the sample and the trip
        robot.gate.set()
        queue.wait(WAIT, "queue_empty")

    sampler.hook = end_step
    queue.watch_tick()
    events = queue.status().last_events
    assert "watch_triggered" not in types(events)
    assert stops(robot) == 0 and reports == []
    assert "step_done" in types(events)


def test_watch_is_bound_to_the_job_ids_of_its_call(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler)
    sampler.set("tilt_deg", 9.0)
    queue.enqueue(steps(NAV1))
    later = queue.enqueue(steps(NAV2), watch_constraints=[watch(name="later", metric="tilt_deg", op="gt", value=5)])
    wait_running(queue, robot)
    queue.watch_tick()  # NAV1 runs: the set of the second call is not active
    status = queue.status()
    assert "watch_triggered" not in types(status.last_events)
    view = status.watches[0]
    assert (view.name, view.metric, view.op, view.value, view.job_ids, view.active, view.triggered) == (
        "later",
        "tilt_deg",
        "gt",
        5.0,
        later.job_ids,
        False,
        False,
    )
    assert sampler.calls == 0  # nothing active: no cache read


def test_replace_drops_orphaned_watch_sets(robot: GateRobot) -> None:
    queue, _ = make_queue(robot, FakeSampler())
    queue.enqueue(steps(NAV1))
    queue.enqueue(steps(NAV2), watch_constraints=[watch(name="orphan", metric="tilt_deg", op="gt", value=5)])
    wait_running(queue, robot)
    assert [w.name for w in queue.status().watches] == ["orphan"]
    queue.enqueue(
        steps({"kind": "wait_s", "seconds": 0.01}),
        replace=True,
        watch_constraints=[watch(name="kept", metric="tilt_deg", op="gt", value=5)],
    )
    assert [w.name for w in queue.status().watches] == ["kept"]


def test_finished_sets_are_dropped_and_status_reports_the_active_one(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler)
    sampler.set("tilt_deg", 1.0)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="tilt_deg", op="gt", value=5)])
    wait_running(queue, robot)
    queue.watch_tick()
    view = queue.status().watches[0]
    assert view.active and not view.triggered and view.last_value == 1.0
    assert queue.status().paused is None
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")
    assert queue.status().watches == []


# --- the thread ---------------------------------------------------------------------------------------------------


def test_watcher_thread_wakes_wait_for_event(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler, watch_sleep=time.sleep)
    sampler.set("gripper_effort", 10.0)
    queue.enqueue(steps(NAV1), watch_constraints=[watch(metric="gripper_effort", op="gt", value=60)])
    wait_running(queue, robot)
    names = {t.name for t in threading.enumerate()}
    assert "motion-watch" in names
    sampler.set("gripper_effort", 90.0)
    result = queue.wait(WAIT, "failure")
    assert result.reason == "watch_triggered"
    assert stops(robot) == 1


def test_watcher_thread_sleeps_while_no_set_is_active(robot: GateRobot) -> None:
    sampler = FakeSampler()
    queue, _ = make_queue(robot, sampler, watch_sleep=time.sleep)
    queue.enqueue(steps(NAV1))
    wait_running(queue, robot)
    time.sleep(0.3)
    assert sampler.calls == 0
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")


def stream_periods(seconds: float, rate_hz: float = 25.0) -> list[float]:
    """A fake arm setpoint stream: a deadline-scheduled loop; returns its tick periods (s)."""
    period = 1.0 / rate_hz
    stamps: list[float] = []
    deadline = time.perf_counter()
    end = deadline + seconds
    while deadline < end:
        stamps.append(time.perf_counter())
        deadline += period
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
    return [b - a for a, b in zip(stamps, stamps[1:], strict=False)]


def p95(values: list[float]) -> float:
    return statistics.quantiles(values, n=20)[-1]


def test_watcher_at_20hz_keeps_the_25hz_arm_stream_period(robot: GateRobot) -> None:
    """Best p95 of two runs each (OS sleep jitter), watcher-on no more than 1 ms worse than watcher-off."""
    runs, seconds = 2, 1.5
    baseline = min(p95(stream_periods(seconds)) for _ in range(runs))
    sampler = FakeSampler()
    sampler.set("gripper_effort", 10.0)
    sampler.set("tilt_deg", 1.0)
    sampler.set("base_speed_mps", 0.0)
    cfg = robot.arm.cfg
    fast = cfg.model_copy(update={"motion_queue": cfg.motion_queue.model_copy(update={"watch_rate_hz": 20.0})})
    queue, _ = make_queue(robot, sampler, watch_sleep=time.sleep, config=fast)
    constraints = [
        watch(metric="gripper_effort", op="gt", value=60),
        watch(metric="tilt_deg", op="gt", value=20),
        watch(metric="distance_travelled_m", op="gt", value=50),
    ]
    queue.enqueue(steps(NAV1), watch_constraints=constraints)
    wait_running(queue, robot)
    calls = sampler.calls
    watched = min(p95(stream_periods(seconds)) for _ in range(runs))
    assert sampler.calls - calls >= 3 * 20 * runs  # the watcher really ticked (3 samples per tick) meanwhile
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")
    assert watched - baseline < 0.001, (baseline, watched)
