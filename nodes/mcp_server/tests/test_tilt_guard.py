"""Tilt guard consumer (/tilt_guard/state from swerve_drive_controller): robot events, Nav2 cancel, tool refusals,
body state and vitals (no ROS)."""

import json
import math
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig, MonitorSettings
from mcp_server.models import RobotEvent
from mcp_server.monitor import BASE_INTERRUPTS, RobotMonitor
from mcp_server.tool_context import TILT_MANUAL_BASE_TOOLS, TILT_NAV_TOOLS
from mcp_server.tools import MOTION_TOOLS, build_mcp_server

from .test_motion_queue import WAIT, GateRobot, make_queue, steps
from .test_tools import TOKEN, FakeRobot, call, take

LIMITS = {"roll_warn_deg": 8.0, "roll_error_deg": 15.0, "pitch_warn_deg": 12.0, "pitch_error_deg": 20.0}
MANUAL_HINT = "drive back to level ground manually with short base moves"
INHIBITED = "base inhibited by the tilt guard"


class Clock:
    """Controllable monotonic + wall clock."""

    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def wall(self) -> float:
        return 1_700_000_000.0 + self.t


def state(level: str, roll: float = 0.5, pitch: float = 1.0, imu_ok: bool = True, inhibited: bool | None = None) -> str:
    return json.dumps(
        {
            "level": level,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "limits": LIMITS,
            "since": 1_700_000_000.0,
            "base_drive_inhibited": level == "error" if inhibited is None else inhibited,
            "imu_ok": imu_ok,
        }
    )


def make(settings: MonitorSettings | None = None) -> tuple[RobotMonitor, Clock, list[RobotEvent], list[int]]:
    clock = Clock()
    mon = RobotMonitor(
        settings or MonitorSettings(),
        clock=clock,
        wall=clock.wall,
        cpu_temp_reader=lambda: 50.0,
        throttled_reader=lambda: None,
        spawn=lambda fn: fn(),  # run the Nav2 cancel (and its retries) inline
        sleep=lambda seconds: None,
    )
    seen: list[RobotEvent] = []
    cancels: list[int] = []
    mon.set_sink(seen.append)

    def cancel() -> bool:
        cancels.append(1)
        return True

    mon.set_nav_canceller(cancel)
    return mon, clock, seen, cancels


def kinds(events: list[RobotEvent]) -> list[tuple[str, str]]:
    return [(e.type, e.severity) for e in events]


# --- monitor: events and Nav2 cancel ---------------------------------------------------------------------------


def test_warning_entry_is_a_critical_event_with_tilt_vs_limits_and_cancels_nav() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("ok"))
    assert seen == [] and cancels == []
    mon.on_tilt_guard(state("warning", roll=9.3, pitch=2.0))
    assert kinds(seen) == [("tilt_warning", "critical")]
    ev = seen[0]
    assert ev.source == "tilt_guard"
    assert "roll 9.3 deg" in ev.message and "pitch 2.0 deg" in ev.message
    assert "8" in ev.message and "12" in ev.message
    assert "automatic navigation is disabled" in ev.message and MANUAL_HINT in ev.message
    assert "wait for the guard to clear" in ev.message
    assert ev.data["level"] == "warning" and ev.data["limits"] == LIMITS
    assert ev.data["roll_deg"] == pytest.approx(9.3) and ev.data["pitch_deg"] == pytest.approx(2.0)
    assert cancels == [1]


def test_repeated_state_at_the_same_level_emits_and_cancels_once() -> None:
    mon, clock, seen, cancels = make()
    for _ in range(5):
        clock.t += 1.0
        mon.on_tilt_guard(state("warning", roll=9.0))
    assert len(seen) == 1 and cancels == [1]


def test_error_entry_says_base_drive_inhibited_and_cancels_nav() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("warning", roll=9.0))
    mon.on_tilt_guard(state("error", roll=16.0))
    assert kinds(seen) == [("tilt_warning", "critical"), ("tilt_error", "critical")]
    msg = seen[1].message
    assert "base drive is inhibited until the IMU is back within limits" in msg
    assert "no base motion tools work" in msg and "roll 16.0 deg" in msg
    assert cancels == [1, 1]


def test_error_straight_from_ok_cancels_nav() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("error", pitch=21.0))
    assert kinds(seen) == [("tilt_error", "critical")] and cancels == [1]


def test_error_back_to_warning_tells_the_agent_to_drive_back_manually() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("error", roll=16.0))
    mon.on_tilt_guard(state("warning", roll=12.0, inhibited=False))
    assert kinds(seen)[-1] == ("tilt_warning", "critical")
    assert MANUAL_HINT in seen[-1].message and "base drive enabled again" in seen[-1].message
    assert cancels == [1, 1]


def test_error_back_to_warning_with_the_base_still_inhibited_says_so() -> None:
    mon, _, seen, _ = make()
    mon.on_tilt_guard(state("error", roll=16.0))
    mon.on_tilt_guard(state("warning", roll=12.0, inhibited=True))
    assert kinds(seen)[-1] == ("tilt_warning", "critical")
    msg = seen[-1].message
    assert "base drive enabled again" not in msg
    assert "base drive still inhibited" in msg and "no base motion tools work" in msg
    assert seen[-1].data["base_drive_inhibited"] is True


def test_inhibit_released_while_still_at_warning_is_an_info_event() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("error", roll=16.0))
    mon.on_tilt_guard(state("warning", roll=12.0, inhibited=True))
    mon.on_tilt_guard(state("warning", roll=11.0, inhibited=True))
    assert len(seen) == 2
    mon.on_tilt_guard(state("warning", roll=10.0, inhibited=False))
    assert kinds(seen)[-1] == ("tilt_drive_enabled", "info")
    assert "base drive enabled again" in seen[-1].message and MANUAL_HINT in seen[-1].message
    assert cancels == [1, 1]


def test_clear_is_an_info_event_without_cancel() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard(state("warning", roll=9.0))
    mon.on_tilt_guard(state("ok", roll=3.0))
    assert kinds(seen) == [("tilt_warning", "critical"), ("tilt_clear", "info")]
    assert "roll 3.0 deg" in seen[1].message
    assert cancels == [1]


def test_malformed_state_is_ignored() -> None:
    mon, _, seen, cancels = make()
    mon.on_tilt_guard("not json")
    mon.on_tilt_guard(json.dumps({"level": "panic", "roll_deg": 1.0}))
    assert seen == [] and cancels == [] and mon.body_state().tilt_guard is None


def test_tilt_events_interrupt_running_base_motions() -> None:
    assert {"tilt_warning", "tilt_error"} <= BASE_INTERRUPTS
    mon, _, _, _ = make()
    watch = mon.watch(BASE_INTERRUPTS)
    mon.on_tilt_guard(state("warning", roll=9.0))
    assert watch.check() == "tilt_warning"


def test_canceller_failure_does_not_break_detection() -> None:
    mon, _, seen, _ = make()

    def broken() -> bool:
        raise RuntimeError("no executor")

    mon.set_nav_canceller(broken)
    mon.on_tilt_guard(state("warning", roll=9.0))
    assert kinds(seen) == [("tilt_warning", "critical"), ("tilt_nav_cancel_failed", "critical")]


# --- monitor: refusals -----------------------------------------------------------------------------------------


def test_refusals_by_level() -> None:
    mon, _, _, _ = make()
    assert mon.tilt_refusal("nav") is None and mon.tilt_refusal("manual") is None
    mon.on_tilt_guard(state("ok"))
    assert mon.tilt_refusal("nav") is None and mon.tilt_refusal("manual") is None
    mon.on_tilt_guard(state("warning", roll=9.0))
    reason = mon.tilt_refusal("nav")
    assert reason is not None and "automatic navigation is disabled" in reason and MANUAL_HINT in reason
    assert mon.tilt_refusal("manual") is None
    mon.on_tilt_guard(state("error", roll=16.0))
    for kind in ("nav", "manual"):
        reason = mon.tilt_refusal(kind)
        assert reason is not None and INHIBITED in reason
    mon.on_tilt_guard(state("ok"))
    assert mon.tilt_refusal("nav") is None


def test_manual_drive_follows_base_drive_inhibited_not_only_the_level() -> None:
    mon, _, _, _ = make()
    mon.on_tilt_guard(state("error", roll=16.0))
    mon.on_tilt_guard(state("warning", roll=12.0, inhibited=True))
    reason = mon.tilt_refusal("manual")
    assert reason is not None and INHIBITED in reason and "level warning" in reason
    nav = mon.tilt_refusal("nav")
    assert nav is not None and INHIBITED in nav
    guard = mon.body_state().tilt_guard
    assert guard is not None and guard.manual_drive_allowed is False and guard.navigation_allowed is False
    assert "base inhibited" in mon.vitals_line()
    mon.on_tilt_guard(state("warning", roll=10.0, inhibited=False))
    assert mon.tilt_refusal("manual") is None
    guard = mon.body_state().tilt_guard
    assert guard is not None and guard.manual_drive_allowed is True
    assert "base inhibited" not in mon.vitals_line()


def test_stale_state_keeps_refusing_at_the_last_level() -> None:
    mon, clock, _, _ = make()
    mon.on_tilt_guard(state("warning", roll=9.0))
    clock.t += 60.0
    reason = mon.tilt_refusal("nav")
    assert reason is not None and "stale" in reason
    body = mon.body_state()
    assert body.tilt_guard is not None and body.tilt_guard.stale is True
    assert any("tilt guard" in note for note in body.notes)


def test_stale_ok_refuses_nav_allows_manual() -> None:
    mon, clock, _, _ = make()
    mon.on_tilt_guard(state("ok"))
    clock.t += MonitorSettings().tilt_guard_stale_s + 1.0
    reason = mon.tilt_refusal("nav")
    assert reason is not None and "stale" in reason and "navigation" in reason
    assert mon.tilt_refusal("manual") is None
    guard = mon.body_state().tilt_guard
    assert guard is not None and guard.navigation_allowed is False and guard.manual_drive_allowed is True
    mon.on_tilt_guard(state("ok"))  # the guard reports again
    assert mon.tilt_refusal("nav") is None


def test_imu_ok_false_at_ok_refuses_nav() -> None:
    mon, _, _, _ = make()
    mon.on_tilt_guard(state("ok", imu_ok=False))
    reason = mon.tilt_refusal("nav")
    assert reason is not None and "IMU" in reason
    assert mon.tilt_refusal("manual") is None
    guard = mon.body_state().tilt_guard
    assert guard is not None and guard.navigation_allowed is False
    mon.on_tilt_guard(state("ok", imu_ok=True))
    assert mon.tilt_refusal("nav") is None


def test_missed_nav_cancel_is_retried() -> None:
    mon, _, seen, _ = make()
    answers = [False, False, True]
    calls: list[int] = []

    def flaky() -> bool:
        calls.append(1)
        return answers[len(calls) - 1]

    mon.set_nav_canceller(flaky)
    mon.on_tilt_guard(state("warning", roll=9.0))
    assert len(calls) == 3
    assert kinds(seen) == [("tilt_warning", "critical")]


def test_nav_cancel_failing_every_attempt_raises_an_event() -> None:
    mon, _, seen, _ = make()
    calls: list[int] = []

    def dead() -> bool:
        calls.append(1)
        return False

    mon.set_nav_canceller(dead)
    mon.on_tilt_guard(state("error", roll=16.0))
    assert len(calls) == MonitorSettings().tilt_cancel_attempts
    assert kinds(seen) == [("tilt_error", "critical"), ("tilt_nav_cancel_failed", "critical")]
    assert "Nav2" in seen[-1].message and "stop" in seen[-1].message


# --- monitor: IMU fallback tilt warning ------------------------------------------------------------------------


def roll_quaternion(deg: float) -> tuple[float, float, float, float]:
    half = math.radians(deg) / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def test_imu_tilt_fallback_is_silent_while_the_guard_reports() -> None:
    mon, _, seen, _ = make()
    mon.on_tilt_guard(state("ok", roll=0.5))
    mon.on_imu(0.0, 0.0, 9.81, *roll_quaternion(12.0))
    assert [e.type for e in seen] == []


def test_imu_tilt_fallback_warns_when_the_guard_state_is_stale() -> None:
    mon, clock, seen, _ = make()
    mon.on_tilt_guard(state("ok", roll=0.5))
    clock.t += 60.0
    mon.on_imu(0.0, 0.0, 9.81, *roll_quaternion(12.0))
    assert kinds(seen) == [("tilt", "warning")]


# --- monitor: body state and vitals ----------------------------------------------------------------------------


def test_body_state_reports_level_roll_pitch_and_limits() -> None:
    mon, clock, _, _ = make()
    body = mon.body_state()
    assert body.tilt_guard is None and any("/tilt_guard/state" in note for note in body.notes)
    mon.on_tilt_guard(state("warning", roll=9.0, pitch=2.5))
    clock.t += 0.5
    guard = mon.body_state().tilt_guard
    assert guard is not None
    assert guard.level == "warning" and guard.roll_deg == 9.0 and guard.pitch_deg == 2.5
    assert guard.limits.model_dump() == LIMITS
    assert guard.base_drive_inhibited is False and guard.imu_ok is True and guard.stale is False
    assert guard.navigation_allowed is False and guard.manual_drive_allowed is True
    assert guard.age_s == pytest.approx(0.5)
    mon.on_tilt_guard(state("error", roll=16.0))
    guard = mon.body_state().tilt_guard
    assert guard is not None and guard.manual_drive_allowed is False and guard.base_drive_inhibited is True


def test_vitals_line_shows_guard_level_tilt_and_limits() -> None:
    mon, _, _, _ = make()
    assert "tilt guard" not in mon.vitals_line()
    mon.on_tilt_guard(state("warning", roll=9.0, pitch=2.5))
    line = mon.vitals_line()
    assert "tilt guard warning: roll 9.0 deg (limits 8/15), pitch 2.5 deg (limits 12/20)" in line


# --- tools ------------------------------------------------------------------------------------------------------

NAV_ARGS: dict[str, dict[str, Any]] = {
    "navigate_to_pose": {"x": 1.0, "y": 0.0},
    "move_relative": {"dx": 0.5},
    "look_around": {},
    "go_to_frontier": {"id": 1},
    "look_toward_frontier": {"id": 1},
    "approach_for_grasp": {"x": 1.5, "y": 2.0, "z": 0.05},
}


def guarded(tmp_path: Path, level: str) -> tuple[FakeRobot, Any, RobotMonitor]:
    robot = FakeRobot(tmp_path)
    config = McpServerConfig()
    monitor = RobotMonitor(config.monitor, cpu_temp_reader=lambda: 50.0, throttled_reader=lambda: None)
    monitor.on_tilt_guard(state(level, roll=9.0 if level == "warning" else 16.0))
    server = build_mcp_server(robot, config, TOKEN, monitor=monitor)
    take(server)  # both leases: the tilt guard is what refuses
    return robot, server, monitor


def test_tool_classification_covers_every_base_motion_tool() -> None:
    assert TILT_NAV_TOOLS == {
        "navigate_to_pose",
        "move_relative",
        "look_around",
        "go_to_frontier",
        "look_toward_frontier",
        "approach_for_grasp",
    }
    assert TILT_MANUAL_BASE_TOOLS == {"drive"}
    assert (TILT_NAV_TOOLS | TILT_MANUAL_BASE_TOOLS) <= MOTION_TOOLS


@pytest.mark.parametrize("level", ["warning", "error"])
@pytest.mark.parametrize("name", sorted(NAV_ARGS))
def test_nav_tools_refused_with_the_reason(tmp_path: Path, name: str, level: str) -> None:
    robot, server, _ = guarded(tmp_path, level)
    with pytest.raises(ToolError) as info:
        call(server, name, NAV_ARGS[name])
    text = str(info.value)
    assert f"{name} refused" in text and "tilt guard" in text
    assert ("automatic navigation is disabled" if level == "warning" else INHIBITED) in text
    assert not [c for c in robot.calls if c[0] in ("navigate", "move_relative", "spin")]


def test_manual_drive_allowed_in_warning(tmp_path: Path) -> None:
    robot, server, _ = guarded(tmp_path, "warning")
    res = call(server, "drive", {"vx": -0.1, "duration_s": 0.5})
    assert res.structured_content["published"] == 3
    assert ("drive", (-0.1, 0.0, 0.0, 0.5)) in robot.calls


def test_manual_drive_refused_in_warning_while_the_base_stays_inhibited(tmp_path: Path) -> None:
    robot, server, monitor = guarded(tmp_path, "error")
    monitor.on_tilt_guard(state("warning", roll=12.0, inhibited=True))
    with pytest.raises(ToolError, match=INHIBITED):
        call(server, "drive", {"vx": -0.1})
    assert not [c for c in robot.calls if c[0] == "drive"]


def test_manual_drive_refused_in_error(tmp_path: Path) -> None:
    robot, server, _ = guarded(tmp_path, "error")
    with pytest.raises(ToolError, match=INHIBITED):
        call(server, "drive", {"vx": -0.1})
    assert not [c for c in robot.calls if c[0] == "drive"]


def test_arm_tools_unaffected_in_error(tmp_path: Path) -> None:
    robot, server, _ = guarded(tmp_path, "error")
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5})
    assert res.structured_content["status"] == "converged"


@pytest.mark.parametrize("level", ["warning", "error"])
def test_enqueue_with_a_base_step_refused(tmp_path: Path, level: str) -> None:
    _, server, _ = guarded(tmp_path, level)
    with pytest.raises(ToolError, match="enqueue_motions refused.*tilt guard"):
        call(server, "enqueue_motions", {"steps": [{"kind": "base_relative", "dx": 0.2}]})


def test_enqueue_arm_only_steps_allowed_in_error(tmp_path: Path) -> None:
    _, server, _ = guarded(tmp_path, "error")
    res = call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.01}]})
    assert res.structured_content["job_ids"]


def test_digest_vitals_carry_the_guard(tmp_path: Path) -> None:
    _, server, _ = guarded(tmp_path, "warning")
    res = call(server, "get_body_state", {})
    assert "tilt guard warning" in res.structured_content["vitals"]
    assert res.structured_content["tilt_guard"]["level"] == "warning"


def test_queued_base_step_refused_at_dispatch(tmp_path: Path) -> None:
    robot = GateRobot(tmp_path)
    robot.gate.set()
    queue = make_queue(robot, tilt_gate=lambda: "tilt guard warning: automatic navigation is disabled")
    queue.enqueue(steps({"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}))
    result = queue.wait(WAIT, "queue_empty")
    failed = [e for e in result.events if e.type == "step_failed"]
    assert failed and "tilt guard" in failed[0].message
    assert not [c for c in robot.calls if c[0] == "navigate"]
