"""Guard engine wiring: tool entry (RobotMCPServer.call_tool_with_digest), queue dispatch (MotionQueue.dispatch) and
arm motion start (ArmController.check_motion_guards). Fakes only - no base motion anywhere."""

import logging
import re
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from prometheus_client import REGISTRY

from mcp_server.arm import ArmController, ArmError
from mcp_server.config import McpServerConfig
from mcp_server.guard_rules import GUARD_ORIGIN, GUARD_REFUSAL_PATTERN
from mcp_server.guard_state import GuardState
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.monitor import RobotMonitor
from mcp_server.tools import build_mcp_server

from .fakes import FakeArmBackend
from .test_tools import TOKEN, FakeRobot, call, take

HIT = "mcp_guard_rule_hits_total"
WAIT = {"timeout_s": 5.0}
OVERREACH = {"shoulder_pan": 0.0, "shoulder_lift": 1.9, "elbow_flex": -0.1, "wrist_flex": 0.0, "wrist_roll": 0.0}
CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)


def rule(name: str, mode: str = "enforce", **spec: Any) -> dict[str, Any]:
    """One guard rule dict (message names the way out like the shipped rules)."""
    return {"name": name, "mode": mode, "message": f"{name} test rule; do something else", **spec}


NAV_RULE = rule(
    "no_nav_with_base_lease",
    applies_to={"tools": ["navigate_to_pose"]},
    all=[{"field": "base.lease_held", "op": "eq", "value": True}],
)
RETAKE_RULE = rule(
    "no_arm_retake_from_human",
    applies_to={"tools": ["take_control"]},
    all=[{"field": "args.subsystem", "op": "eq", "value": "arm"}],
    any=[
        {"field": "arm.active_source", "op": "eq", "value": "web_ui"},
        {"field": "events.human_takeover_age_s", "op": "lt", "value": 30.0},
    ],
    when_unknown="allow",
)
STEP_RULE = rule(
    "no_queued_base_relative",
    applies_to={"steps": ["base_relative"]},
    all=[{"field": "base.lease_held", "op": "eq", "value": True}],
)
OVERREACH_RULE = rule(
    "no_stretched_shoulder_overreach",
    applies_to={"arm_motion_from": ["move_arm_joints", "move_arm_cartesian", "arm_joints", "arm_cartesian"]},
    all=[
        {"field": "target.joint.shoulder_lift", "op": "gt", "value": 1.85},
        {"field": "target.joint.elbow_flex", "op": "lt", "value": 0.0},
    ],
)


def hits(rule_name: str, mode: str, phase: str, basis: str = "match") -> float:
    labels = {"rule": rule_name, "mode": mode, "phase": phase, "basis": basis}
    return REGISTRY.get_sample_value(HIT, labels) or 0.0


def make_server(
    robot: FakeRobot, rules: list[dict[str, Any]], monitor: RobotMonitor | None = None, lease: bool = True
) -> Any:
    cfg = McpServerConfig(
        objects={"store_path": robot.arm.cfg.arm.home_file.parent / "objects.json"},
        guard_rules={"rules": rules},
    )
    server = build_mcp_server(robot, cfg, TOKEN, monitor=monitor)
    if lease:
        take(server)
    return server


def allow_overreach(arm: ArmController) -> None:
    """Measured-space limits that reach shoulder_lift 1.9 (the deployed calibration does; the bare URDF does not), and
    no body model: with zero offsets the stretched arm would reach the wheels, and only the guard rule is under test."""
    arm.limits["shoulder_lift"] = (-2.2, 2.2)
    arm.body_check.settings.enabled = False


# --- (1) tool entry ------------------------------------------------------------------------------------------------


def test_enforce_rule_refuses_navigate_to_pose_and_the_base_receives_nothing(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, [NAV_RULE])
    before = hits("no_nav_with_base_lease", "enforce", "tool")
    with pytest.raises(ToolError) as exc:
        call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    text = str(exc.value)
    assert text.startswith("guard rule 'no_nav_with_base_lease' refused navigate_to_pose: ")
    assert "base.lease_held=true" in text
    assert "robot_events_since_last_call" in text  # the digest is appended by the existing error path
    assert not any(name == "navigate" for name, _ in robot.calls)
    assert hits("no_nav_with_base_lease", "enforce", "tool") == before + 1
    assert server.blocking_tool() is None  # refused before the blocking counter


def test_dry_run_rule_lets_navigate_through_and_counts(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, [{**NAV_RULE, "mode": "dry_run"}])
    before = hits("no_nav_with_base_lease", "dry_run", "tool")
    with caplog.at_level(logging.WARNING, logger="mcp_server.guard_rules"):
        res = call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert res.structured_content["status"] == "succeeded"
    assert any(name == "navigate" for name, _ in robot.calls)
    assert hits("no_nav_with_base_lease", "dry_run", "tool") == before + 1
    assert any("guard_rule_would_refuse" in r.getMessage() for r in caplog.records)


def test_tool_without_rules_never_calls_a_provider(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    read: list[str] = []
    original = GuardState.get

    def recording(self: GuardState, field: str, ctx: Any) -> Any:
        read.append(field)
        return original(self, field, ctx)

    monkeypatch.setattr(GuardState, "get", recording)
    robot = FakeRobot(tmp_path)
    server = make_server(robot, [NAV_RULE])
    call(server, "get_robot_state", {})
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    call(server, "set_gripper", {"open_fraction": 0.4})
    call(server, "move_relative", {"dx": 0.1})
    assert read == []
    with pytest.raises(ToolError):
        call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert read == ["base.lease_held"]  # the patched provider is the one the engine reads


def test_take_control_arm_refused_after_human_takeover_base_allowed(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    monitor = RobotMonitor(CONFIG.monitor)
    server = make_server(robot, [RETAKE_RULE], monitor=monitor, lease=False)
    call(server, "take_control", {"subsystem": "arm"})  # no human takeover yet: allowed
    call(server, "release_control", {"subsystem": "arm", "keep_position": True})
    with monitor.lock:
        monitor.takeover("web_ui")
    with pytest.raises(ToolError, match=r"^guard rule 'no_arm_retake_from_human' refused take_control: "):
        call(server, "take_control", {"subsystem": "arm"})
    assert robot.arm.control_held is False
    res = call(server, "take_control", {"subsystem": "base"})
    assert res.structured_content["base"]["held"] is True


def test_guard_origin_is_the_tool_name_inside_the_tool_body(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, [])
    seen: list[str | None] = []
    robot.arm.motion_guard = lambda start, target: seen.append(GUARD_ORIGIN.get())
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    assert seen == ["move_arm_joints"]
    assert GUARD_ORIGIN.get() is None  # reset after the call


# --- (2) queue dispatch --------------------------------------------------------------------------------------------


def test_queued_base_relative_refused_stops_the_queue_even_with_on_fail_skip(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, [STEP_RULE])
    out = call(
        server,
        "enqueue_motions",
        {
            "steps": [
                {"kind": "base_relative", "dx": 0.1, "on_fail": "skip"},
                {"kind": "wait_s", "seconds": 0.01, "on_fail": "skip"},
            ]
        },
    ).structured_content
    base_id, wait_id = out["job_ids"]
    waited = call(server, "wait_for_event", WAIT).structured_content
    events = waited["events"]
    failed = [e for e in events if e["type"] == "step_failed"]
    assert len(failed) == 1 and failed[0]["job_id"] == base_id
    assert failed[0]["data"]["status"] == "guard_rule"
    assert failed[0]["data"]["rules"] == ["no_queued_base_relative"]
    assert "on_fail" not in failed[0]["data"]  # never skipped
    assert failed[0]["message"].startswith("guard rule 'no_queued_base_relative' refused queued base_relative step: ")
    stopped = [e for e in events if e["type"] == "queue_stopped"]
    assert len(stopped) == 1 and wait_id in stopped[0]["data"]["dropped"]
    assert not any(e["type"] == "step_started" and e["job_id"] == base_id for e in events)
    assert not any(e["job_id"] == wait_id and e["type"] in ("step_started", "step_skipped") for e in events)
    assert not any(name == "move_relative" for name, _ in robot.calls)


def test_queued_arm_step_runs_with_its_kind_as_guard_origin(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    allow_overreach(robot.arm)
    server = make_server(robot, [OVERREACH_RULE])
    commands = len(robot.arm_backend.commands)
    call(
        server,
        "enqueue_motions",
        {"steps": [{"kind": "arm_joints", "targets": {"shoulder_lift": 1.9, "elbow_flex": -0.1}}]},
    )
    events = call(server, "wait_for_event", WAIT).structured_content["events"]
    failed = [e for e in events if e["type"] == "step_failed"]
    assert len(failed) == 1 and failed[0]["data"]["status"] == "refused"
    assert failed[0]["message"].startswith(
        "guard rule 'no_stretched_shoulder_overreach' refused arm motion of arm_joints: "
    )
    assert len(robot.arm_backend.commands) == commands  # nothing streamed


# --- (3) arm motion start ------------------------------------------------------------------------------------------


def test_move_arm_cartesian_with_overreaching_ik_target_is_refused(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = FakeRobot(tmp_path)
    allow_overreach(robot.arm)
    monkeypatch.setattr(robot.arm, "solve_cartesian", lambda *a, **k: dict(OVERREACH))
    server = make_server(robot, [OVERREACH_RULE])
    commands = len(robot.arm_backend.commands)
    before = hits("no_stretched_shoulder_overreach", "enforce", "arm")
    with pytest.raises(ToolError) as exc:
        call(server, "move_arm_cartesian", {"x": 0.25, "y": 0.0, "z": 0.3})
    text = str(exc.value)
    # An ArmError from the tool body (the framework prefixes "Error executing tool ..."), parsed by the pattern.
    assert "guard rule 'no_stretched_shoulder_overreach' refused arm motion of move_arm_cartesian: " in text
    assert re.search(GUARD_REFUSAL_PATTERN, text)
    assert "target.joint.shoulder_lift=1.9" in text and "target.joint.elbow_flex=-0.1" in text
    assert len(robot.arm_backend.commands) == commands
    assert hits("no_stretched_shoulder_overreach", "enforce", "arm") == before + 1


def test_same_pose_from_arm_home_and_park_is_not_checked(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    allow_overreach(robot.arm)
    robot.arm_backend.positions.update(OVERREACH)
    server = make_server(robot, [OVERREACH_RULE])
    call(server, "arm_set_home", {})
    before = hits("no_stretched_shoulder_overreach", "enforce", "arm")
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})  # leaves the pose (elbow not stretched)
    assert call(server, "arm_home", {}).structured_content["status"] == "converged"
    assert robot.arm_backend.positions["shoulder_lift"] == pytest.approx(1.9, abs=1e-3)
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(-0.1, abs=1e-3)
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    res = call(server, "release_control", {"subsystem": "arm"}).structured_content  # parks at the home pose
    assert res["state"]["arm"]["held"] is False
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(-0.1, abs=1e-3)
    assert hits("no_stretched_shoulder_overreach", "enforce", "arm") == before


def make_arm(tmp_path: Path) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def test_motion_guard_runs_once_per_motion_before_the_first_setpoint(tmp_path: Path) -> None:
    arm, be = make_arm(tmp_path)
    calls: list[tuple[dict[str, float], dict[str, float], int]] = []
    arm.motion_guard = lambda start, target: calls.append((dict(start), dict(target), len(be.commands)))
    arm.acquire()
    published = len(be.commands)
    start_elbow = arm.command_base(arm.require_sample())["elbow_flex"]
    arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    assert len(calls) == 1
    start, target, commands_then = calls[0]
    assert commands_then == published  # before the first setpoint
    assert start["elbow_flex"] == pytest.approx(start_elbow)
    assert target["elbow_flex"] == pytest.approx(0.2)
    assert set(target) == set(start)  # the full target pose (unnamed joints keep their start value)


def test_motion_guard_checks_every_leg_of_a_blend_and_every_path_sample(tmp_path: Path) -> None:
    arm, _be = make_arm(tmp_path)
    calls: list[tuple[dict[str, float], dict[str, float]]] = []
    arm.motion_guard = lambda start, target: calls.append((dict(start), dict(target)))
    arm.acquire()
    arm.move_blend([{"elbow_flex": 0.2}, {"wrist_flex": 0.3}], speed_scale=0.5)
    assert len(calls) == 2
    assert calls[1][0]["elbow_flex"] == pytest.approx(0.2)  # the second leg starts where the first ends
    assert calls[1][1]["wrist_flex"] == pytest.approx(0.3) and calls[1][1]["elbow_flex"] == pytest.approx(0.2)
    calls.clear()
    arm.move_path([{"elbow_flex": 0.1}, {"elbow_flex": 0.0}], speed_scale=0.5)
    assert len(calls) == 2


def test_motion_guard_rejection_moves_nothing(tmp_path: Path) -> None:
    arm, be = make_arm(tmp_path)

    def refuse(start: dict[str, float], target: dict[str, float]) -> None:
        raise ArmError("guard rule 'x_rule' refused arm motion of move_arm_joints: no (target.joint.elbow_flex=0.2)")

    arm.motion_guard = refuse
    arm.acquire()
    published = len(be.commands)
    with pytest.raises(ArmError, match="guard rule 'x_rule'"):
        arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    assert len(be.commands) == published


def test_roll_guard_runs_before_the_motion_guard(tmp_path: Path) -> None:
    arm, be = make_arm(tmp_path)
    calls: list[int] = []
    arm.motion_guard = lambda start, target: calls.append(1)
    arm.acquire()
    be.positions[arm.gripper] = arm.cfg.limits.roll_max_gripper_open_rad + 0.3
    with pytest.raises(ArmError, match="wrist_roll changes by"):
        arm.move_joints({"wrist_roll": 0.5}, speed_scale=0.5)
    assert calls == []
