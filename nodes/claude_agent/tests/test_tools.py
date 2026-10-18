"""Tool classification, budget gating (ro/rw counting) and permission denial."""

import asyncio
import logging
from pathlib import Path

import pytest
from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from claude_agent.config import ClaudeAgentConfig
from claude_agent.tools import (
    BUILTIN_TOOLS,
    NOTES_TOOLS,
    EffectorGate,
    classify_tool,
    short_name,
)


def ctx(tool_use_id: str = "tu1") -> ToolPermissionContext:
    return ToolPermissionContext(tool_use_id=tool_use_id)


def test_short_name() -> None:
    assert short_name("mcp__robot__drive") == "drive"
    assert short_name("Bash") == "Bash"


def test_classify(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "mcp__robot__drive") == "effector"
    assert classify_tool(config, "mcp__robot__arm_home") == "effector"
    assert classify_tool(config, "mcp__robot__stop") == "uncapped"
    assert classify_tool(config, "mcp__robot__release_control") == "uncapped"
    assert classify_tool(config, "mcp__robot__get_camera_image") == "sensor"
    assert classify_tool(config, "mcp__robot__get_robot_state") == "sensor"
    assert classify_tool(config, "mcp__robot__get_control_state") == "sensor"
    assert classify_tool(config, "mcp__robot__take_control") == "uncapped"
    assert classify_tool(config, "mcp__robot__arm_set_home") == "denied"


def test_unknown_robot_tool_is_capped_as_effector(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "mcp__robot__new_motion_tool") == "effector"


def test_non_robot_tool_is_not_classified(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "Bash") is None
    assert classify_tool(config, "mcp__other__drive") is None


def test_builtin_tools_cover_the_dangerous_ones() -> None:
    for name in (
        "Bash",
        "WebFetch",
        "WebSearch",
        "Task",
        "Agent",
        "TodoWrite",
    ):
        assert name in BUILTIN_TOOLS
    for name in ("NotebookEdit", "AskUserQuestion", "ExitPlanMode", "EnterPlanMode"):
        assert name in BUILTIN_TOOLS


PLAN_TOOLS = [
    "mcp__agent__set_task_plan",
    "mcp__agent__complete_phase",
    "mcp__agent__revise_plan",
    "mcp__agent__raise_phase_budget",
    "mcp__agent__answer_condition",
    "mcp__agent__start_workflow",
]


def one_phase(rw: int = 2, turns: int = 20, name: str = "Work") -> dict:
    return {"name": name, "goal": "goal reached", "rw_cap": rw, "turn_cap": turns}


def gate_with_plan(rw: int = 2, turns: int = 20, **kwargs) -> EffectorGate:
    gate = EffectorGate(ClaudeAgentConfig(), **kwargs)
    gate.plan.set_plan("simple", "test", [one_phase(rw, turns)])
    return gate


@pytest.mark.parametrize("name", PLAN_TOOLS)
def test_classify_planning_tools(config: ClaudeAgentConfig, name: str) -> None:
    assert classify_tool(config, name) == "plan"
    assert short_name(name) == name.removeprefix("mcp__agent__")


def test_classify_unknown_agent_tool_is_none(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "mcp__agent__other") is None
    assert classify_tool(config, "mcp__agent__set_task_budget") is None


async def test_effector_tools_denied_until_a_plan_is_set_but_sensors_allowed() -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_denied=denied.append)
    for name in ("get_robot_state", "get_camera_image"):
        assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx("t")), PermissionResultAllow)
    for name in ("drive", "move_arm_joints"):
        result = await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx("t"))
        assert isinstance(result, PermissionResultDeny)
        assert result.message == "call agent.set_task_plan first"
    assert len(denied) == 2 and denied[0] == {"id": "t", "name": "drive", "reason": result.message}
    assert gate.plan.ro_used == 2 and gate.plan.rw_used == 0


async def test_planning_and_notes_tools_allowed_without_a_plan(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    for name in PLAN_TOOLS:
        assert isinstance(await gate.can_use_tool(name, {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)


async def test_stop_and_control_tools_always_allowed_and_never_counted() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    for name in ("stop", "take_control", "release_control"):
        assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx()), PermissionResultAllow)
    gate.plan.set_plan("simple", "x", [one_phase(1, 5)])
    for _ in range(20):
        assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)


async def test_sensors_are_counted_never_denied_and_effectors_denied_at_the_cap() -> None:
    denied: list[dict] = []
    gate = gate_with_plan(rw=1, on_denied=denied.append)
    for _ in range(50):
        assert isinstance(await gate.can_use_tool("mcp__robot__get_camera_image", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)
    result = await gate.can_use_tool("mcp__robot__set_gripper", {}, ctx("d"))
    assert isinstance(result, PermissionResultDeny) and result.message.startswith(
        "phase 1 'Work' rw budget exhausted (1/1)"
    )
    assert "complete_phase" in result.message and "raise_phase_budget" in result.message
    assert (gate.plan.ro_used, gate.plan.rw_used) == (50, 1)
    assert denied[0]["id"] == "d" and denied[0]["name"] == "set_gripper"
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)


async def test_unknown_robot_tool_counts_as_rw() -> None:
    gate = gate_with_plan(rw=1)
    assert isinstance(await gate.can_use_tool("mcp__robot__new_motion_tool", {}, ctx()), PermissionResultAllow)
    assert gate.plan.rw_used == 1


async def test_notes_tools_are_not_counted(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    gate.plan.set_plan("simple", "x", [one_phase(1, 5)])
    for _ in range(10):
        assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)


async def test_calls_count_against_the_active_phase_and_the_next_phase_starts_fresh() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    gate.plan.set_plan("simple", "x", [one_phase(1, 20, "A"), one_phase(1, 20, "B")])
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultDeny)
    gate.plan.complete_phase("done", "ok")
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


async def test_phase_turn_cap_denies_robot_tools_but_not_planning_tools(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    gate.plan.set_plan("simple", "x", [one_phase(5, 1)])
    gate.plan.count_turn()
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("d"))
    assert isinstance(result, PermissionResultDeny) and "turn budget exhausted (1/1)" in result.message
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool(PLAN_TOOLS[1], {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)


async def test_effector_tools_denied_but_sensors_allowed_after_the_last_phase_is_completed() -> None:
    gate = gate_with_plan()
    gate.plan.complete_phase("done", "all done")
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    assert isinstance(result, PermissionResultDeny) and "all phases are completed" in result.message
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)


async def test_reset_clears_the_plan_and_counters() -> None:
    gate = gate_with_plan(rw=1)
    await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultDeny)
    gate.reset()
    assert gate.plan.plan is None and gate.plan.rw_used == 0
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    assert isinstance(result, PermissionResultDeny) and result.message == "call agent.set_task_plan first"


@pytest.mark.parametrize("name", ["Bash", "WebFetch", "Task", "mcp__other__thing", "mcp__claude_ai__x"])
async def test_everything_else_denied(name: str) -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_denied=denied.append)
    result = await gate.can_use_tool(name, {}, ctx("z"))
    assert isinstance(result, PermissionResultDeny)
    assert "not available" in result.message
    assert denied[0]["name"] == name and denied[0]["id"] == "z"
    assert gate.plan.rw_used == 0


async def test_count_change_callback() -> None:
    seen: list[None] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_count=lambda: seen.append(None))
    gate.plan.set_plan("simple", "x", [one_phase(5, 5)])
    seen.clear()
    await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx())
    assert len(seen) == 2


# --- file tools for notes, sandboxed to the workdir ---------------------------------------------------------------


@pytest.fixture
def workdir(tmp_path: Path) -> Path:
    path = tmp_path / "workspace"
    path.mkdir()
    return path


def notes_gate(workdir: Path, **kwargs) -> EffectorGate:
    return EffectorGate(ClaudeAgentConfig(workdir=str(workdir)), **kwargs)


def test_notes_tools_constant_and_builtin_list() -> None:
    assert set(NOTES_TOOLS) == {"Read", "Write", "Edit", "Glob", "Grep"}
    assert not set(NOTES_TOOLS) & set(BUILTIN_TOOLS)
    for name in ("Bash", "WebFetch", "WebSearch", "Task", "Agent", "NotebookEdit", "TodoWrite", "AskUserQuestion"):
        assert name in BUILTIN_TOOLS


def test_classify_notes_tools(config: ClaudeAgentConfig) -> None:
    for name in NOTES_TOOLS:
        assert classify_tool(config, name) == "notes"
    assert classify_tool(config, "Bash") is None


@pytest.mark.parametrize(
    "tool,field,value",
    [
        ("Read", "file_path", "NOTES.md"),
        ("Write", "file_path", "sub/dir/new.md"),
        ("Edit", "file_path", "NOTES.md"),
        ("Glob", "path", "."),
        ("Grep", "path", "sub"),
    ],
)
async def test_notes_inside_workdir_allowed_relative_and_absolute(
    workdir: Path, tool: str, field: str, value: str
) -> None:
    gate = notes_gate(workdir)
    assert isinstance(await gate.can_use_tool(tool, {field: value}, ctx()), PermissionResultAllow)
    absolute = str(workdir / value)
    assert isinstance(await gate.can_use_tool(tool, {field: absolute}, ctx()), PermissionResultAllow)


@pytest.mark.parametrize("tool", ["Glob", "Grep"])
async def test_glob_grep_without_path_default_to_workdir(workdir: Path, tool: str) -> None:
    gate = notes_gate(workdir)
    assert isinstance(await gate.can_use_tool(tool, {"pattern": "*.md"}, ctx()), PermissionResultAllow)


async def test_notes_tools_never_counted(workdir: Path) -> None:
    gate = notes_gate(workdir)
    gate.plan.set_plan("simple", "x", [one_phase(5, 20)])
    for _ in range(20):
        for tool in NOTES_TOOLS:
            await gate.can_use_tool(tool, {"file_path": "NOTES.md", "path": "."}, ctx())
    assert gate.plan.rw_used == 0
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


@pytest.mark.parametrize(
    "tool,field",
    [("Read", "file_path"), ("Write", "file_path"), ("Edit", "file_path"), ("Glob", "path"), ("Grep", "path")],
)
async def test_notes_outside_workdir_denied(workdir: Path, tmp_path: Path, tool: str, field: str) -> None:
    denied: list[dict] = []
    gate = notes_gate(workdir, on_denied=denied.append)
    for bad in (
        "/etc/passwd",
        str(tmp_path / "other.md"),
        "../outside.md",
        "sub/../../outside.md",
        "~/x",
        str(workdir) + "-evil/x",
    ):
        result = await gate.can_use_tool(tool, {field: bad}, ctx("t9"))
        assert isinstance(result, PermissionResultDeny), bad
        assert "workdir" in result.message
    assert denied[0]["id"] == "t9" and denied[0]["name"] == tool
    assert gate.plan.rw_used == 0


async def test_notes_symlink_escape_denied(workdir: Path, tmp_path: Path) -> None:
    outside = tmp_path / "secret.txt"
    outside.write_text("s")
    (workdir / "link").symlink_to(outside)
    (workdir / "dirlink").symlink_to(tmp_path)
    gate = notes_gate(workdir)
    for bad in ("link", "dirlink/secret.txt", "dirlink/new.txt"):
        for tool in ("Read", "Write", "Edit"):
            assert isinstance(await gate.can_use_tool(tool, {"file_path": bad}, ctx()), PermissionResultDeny), bad
    assert isinstance(await gate.can_use_tool("Grep", {"path": "dirlink"}, ctx()), PermissionResultDeny)


async def test_notes_symlink_inside_workdir_allowed(workdir: Path) -> None:
    (workdir / "real.md").write_text("x")
    (workdir / "alias.md").symlink_to(workdir / "real.md")
    assert isinstance(
        await notes_gate(workdir).can_use_tool("Read", {"file_path": "alias.md"}, ctx()), PermissionResultAllow
    )


@pytest.mark.parametrize("tool", ["Read", "Write", "Edit"])
async def test_notes_missing_or_invalid_path_denied(workdir: Path, tool: str) -> None:
    gate = notes_gate(workdir)
    for payload in ({}, {"file_path": ""}, {"file_path": 5}, {"file_path": None}):
        assert isinstance(await gate.can_use_tool(tool, payload, ctx()), PermissionResultDeny)


@pytest.mark.parametrize(
    "payload", [{"pattern": "/etc/*"}, {"pattern": "../*"}, {"pattern": "~/*"}, {"pattern": "a/../../b"}]
)
async def test_glob_pattern_escaping_workdir_denied(workdir: Path, payload: dict) -> None:
    assert isinstance(await notes_gate(workdir).can_use_tool("Glob", payload, ctx()), PermissionResultDeny)


async def test_grep_glob_filter_escaping_workdir_denied(workdir: Path) -> None:
    gate = notes_gate(workdir)
    assert isinstance(await gate.can_use_tool("Grep", {"pattern": "x", "glob": "../*"}, ctx()), PermissionResultDeny)
    assert isinstance(await gate.can_use_tool("Grep", {"pattern": "x", "glob": "*.md"}, ctx()), PermissionResultAllow)


@pytest.mark.parametrize(
    "name",
    ["Bash", "WebFetch", "WebSearch", "Task", "Agent", "NotebookEdit", "TodoWrite", "AskUserQuestion", "ExitPlanMode"],
)
async def test_other_builtins_still_denied(workdir: Path, name: str) -> None:
    result = await notes_gate(workdir).can_use_tool(name, {"file_path": "NOTES.md"}, ctx())
    assert isinstance(result, PermissionResultDeny) and "not available" in result.message


async def test_parallel_sensor_and_effector_requests_both_get_an_explicit_decision() -> None:
    gate = gate_with_plan(rw=5)
    move, camera = await asyncio.gather(
        gate.can_use_tool("mcp__robot__move_arm_cartesian", {"x": 0.25}, ctx("m")),
        gate.can_use_tool("mcp__robot__get_annotated_camera_image", {"camera": "gripper"}, ctx("c")),
    )
    assert isinstance(move, PermissionResultAllow) and isinstance(camera, PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (1, 1)


async def test_parallel_requests_past_the_rw_cap_deny_the_effector_with_the_budget_reason() -> None:
    denied: list[dict] = []
    gate = gate_with_plan(rw=1, on_denied=denied.append)
    results = await asyncio.gather(
        *(gate.can_use_tool("mcp__robot__drive", {}, ctx(f"d{i}")) for i in range(3)),
        gate.can_use_tool("mcp__robot__get_camera_image", {}, ctx("c")),
    )
    assert [type(r) for r in results[:3]].count(PermissionResultAllow) == 1
    assert all("rw budget exhausted" in r.message for r in results[:3] if isinstance(r, PermissionResultDeny))
    assert isinstance(results[3], PermissionResultAllow)
    assert len(denied) == 2 and gate.plan.rw_used == 1


async def test_exception_inside_the_gate_becomes_an_explicit_internal_error_denial(
    caplog: pytest.LogCaptureFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    denied: list[dict] = []
    gate = gate_with_plan(on_denied=denied.append)

    def broken(kind: str) -> str | None:
        raise RuntimeError("tracker exploded")

    monkeypatch.setattr(gate.plan, "charge", broken)
    with caplog.at_level(logging.ERROR, logger="claude_agent"):
        result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("x"))
    assert isinstance(result, PermissionResultDeny)
    assert result.message.startswith("internal error in the permission gate: RuntimeError: tracker exploded")
    assert "not the user" in result.message
    assert denied == [{"id": "x", "name": "drive", "reason": result.message}]
    assert "tracker exploded" in caplog.text


async def test_a_failing_denial_callback_still_returns_the_explicit_denial(caplog: pytest.LogCaptureFixture) -> None:
    def explode(info: dict) -> None:
        raise ValueError("ui sink down")

    gate = EffectorGate(ClaudeAgentConfig(), on_denied=explode)
    with caplog.at_level(logging.ERROR, logger="claude_agent"):
        result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("y"))
    assert isinstance(result, PermissionResultDeny) and result.message == "call agent.set_task_plan first"
    assert "ui sink down" in caplog.text


MIN_LEFT_S = 60.0


def gate_with_time_left(left: float, **kwargs) -> EffectorGate:
    gate = EffectorGate(ClaudeAgentConfig(min_effector_time_left_s=MIN_LEFT_S), time_left=lambda: left, **kwargs)
    gate.plan.set_plan("simple", "test", [one_phase(rw=20)])
    return gate


@pytest.mark.parametrize("name", ["drive", "grasp_object", "arm_home", "navigate_to_pose", "enqueue_motions"])
async def test_effector_denied_below_the_minimum_instruction_time_left(name: str) -> None:
    denied: list[dict] = []
    gate = gate_with_time_left(59.0, on_denied=denied.append)
    result = await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx("t"))
    assert isinstance(result, PermissionResultDeny)
    assert "instruction time left: 59 s" in result.message
    assert "below the 60 s minimum" in result.message
    assert "report" in result.message and "end the instruction" in result.message
    assert denied == [{"id": "t", "name": name, "reason": result.message}]
    assert gate.plan.rw_used == 0


async def test_effector_allowed_at_or_above_the_minimum_time_left() -> None:
    gate = gate_with_time_left(60.0)
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


@pytest.mark.parametrize("name", ["stop", "cancel_motions", "release_object", "take_control", "release_control"])
async def test_safety_tools_allowed_with_no_time_left(name: str) -> None:
    gate = gate_with_time_left(1.0)
    assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx()), PermissionResultAllow)


async def test_set_gripper_open_allowed_but_close_denied_with_no_time_left() -> None:
    gate = gate_with_time_left(1.0)
    allowed = [{"open_fraction": 1.0}, {"open_fraction": 0.5}]
    for args in allowed:
        assert isinstance(await gate.can_use_tool("mcp__robot__set_gripper", args, ctx()), PermissionResultAllow)
    for args in (
        {"open_fraction": 0.2},
        {"close_until_effort": True},
        {"open_fraction": 1.0, "close_until_effort": True},
        {},
    ):
        result = await gate.can_use_tool("mcp__robot__set_gripper", args, ctx())
        assert isinstance(result, PermissionResultDeny), args


async def test_sensors_and_planning_tools_ignore_the_time_left() -> None:
    gate = gate_with_time_left(1.0)
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__agent__complete_phase", {}, ctx()), PermissionResultAllow)


async def test_gate_without_a_time_source_never_denies_on_time() -> None:
    gate = gate_with_plan()
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


async def test_answer_condition_is_a_planning_tool_and_a_pending_ask_blocks_effectors() -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_denied=denied.append)
    assert isinstance(await gate.can_use_tool("mcp__agent__answer_condition", {}, ctx()), PermissionResultAllow)
    tree = {
        "type": "if",
        "id": "check",
        "condition": {"ask": "is the gripper empty?"},
        "then": {"type": "phase", "id": "a", "name": "A", "goal": "g", "rw_cap": 2, "turn_cap": 5},
    }
    gate.plan.set_plan("simple", "x", plan=tree)
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("d"))
    assert isinstance(result, PermissionResultDeny) and "answer condition 'check' first" in result.message
    assert isinstance(await gate.can_use_tool("mcp__robot__get_camera_image", {}, ctx()), PermissionResultAllow)
    gate.plan.answer_condition("check", True, "empty jaws in the gripper camera")
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)
    assert denied[0]["name"] == "drive"


async def test_gate_forwards_the_condition_callbacks() -> None:
    seen: list[tuple[str, str]] = []
    gate = EffectorGate(
        ClaudeAgentConfig(),
        on_condition_pending=lambda c: seen.append(("pending", c["node_id"])),
        on_condition_answered=lambda c: seen.append(("answered", c["node_id"])),
    )
    tree = {
        "type": "if",
        "id": "check",
        "condition": {"ask": "q?"},
        "then": {"type": "phase", "id": "a", "name": "A", "goal": "g", "rw_cap": 2, "turn_cap": 5},
    }
    gate.plan.set_plan("simple", "x", plan=tree)
    gate.plan.answer_condition("check", False, "no")
    assert seen == [("pending", "check"), ("answered", "check")]


ARM_SET_HOME_REASON = "arm_set_home is reserved for humans: the job-end check compares against the stored home"


async def test_arm_set_home_denied_with_reason_even_with_a_plan_and_never_counted() -> None:
    denied: list[dict] = []
    gate = gate_with_plan(rw=5, on_denied=denied.append)
    result = await gate.can_use_tool("mcp__robot__arm_set_home", {}, ctx())
    assert isinstance(result, PermissionResultDeny)
    assert ARM_SET_HOME_REASON in result.message
    assert denied == [{"id": "tu1", "name": "arm_set_home", "reason": result.message}]
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)


async def test_arm_set_home_denied_without_a_plan_with_the_denial_reason_not_the_plan_reason() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    result = await gate.can_use_tool("mcp__robot__arm_set_home", {}, ctx())
    assert isinstance(result, PermissionResultDeny)
    assert ARM_SET_HOME_REASON in result.message


async def test_take_and_release_control_never_counted_never_time_refused_and_get_control_state_is_a_sensor() -> None:
    gate = gate_with_time_left(1.0)
    for name in ("take_control", "release_control"):
        for _ in range(20):
            assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)
    assert isinstance(await gate.can_use_tool("mcp__robot__get_control_state", {}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (1, 0)


# --- H9: putting an object down is never refused by the plan or the caps --------------------------------------------

RELEASES = [("release_object", {}), ("set_gripper", {"open_fraction": 1.0})]


def finished_gate() -> EffectorGate:
    gate = gate_with_plan()
    gate.plan.complete_phase("done", "all done")
    return gate


def pending_gate() -> EffectorGate:
    gate = EffectorGate(ClaudeAgentConfig())
    gate.plan.set_plan(
        "simple",
        "t",
        plan={
            "type": "if",
            "id": "check",
            "condition": {"ask": "red?"},
            "then": {"type": "phase", "id": "a", **one_phase()},
        },
    )
    return gate


def exhausted_gate() -> EffectorGate:
    gate = gate_with_plan(rw=1)
    gate.plan.charge("effector")
    return gate


@pytest.mark.parametrize(
    "make_gate", [finished_gate, pending_gate, exhausted_gate, lambda: EffectorGate(ClaudeAgentConfig())]
)
@pytest.mark.parametrize(("name", "args"), RELEASES)
async def test_release_object_and_opening_the_gripper_are_never_refused_by_the_plan(
    make_gate, name: str, args: dict
) -> None:
    gate = make_gate()
    before = gate.plan.rw_used
    assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", args, ctx()), PermissionResultAllow)
    assert gate.plan.rw_used == before + 1, "the release is still counted"


@pytest.mark.parametrize("make_gate", [finished_gate, pending_gate, exhausted_gate])
async def test_closing_the_gripper_is_still_refused_by_the_plan(make_gate) -> None:
    gate = make_gate()
    result = await gate.can_use_tool("mcp__robot__set_gripper", {"open_fraction": 0.1}, ctx())
    assert isinstance(result, PermissionResultDeny)


async def test_instruction_denied_tools_are_refused_with_the_arm_only_reason_and_not_counted() -> None:
    denied: list[dict] = []
    gate = gate_with_plan(rw=5, on_denied=denied.append)
    gate.set_instruction_denied(["drive", "navigate_to_pose"])
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("t1"))
    assert isinstance(result, PermissionResultDeny)
    assert result.message == "drive is disabled for this instruction (evaluation arm-only mode)"
    assert denied == [{"id": "t1", "name": "drive", "reason": result.message}]
    assert gate.plan.rw_used == 0
    assert isinstance(await gate.can_use_tool("mcp__robot__move_arm_joints", {}, ctx()), PermissionResultAllow)


async def test_instruction_denied_tools_never_deny_releases() -> None:
    gate = gate_with_plan(rw=5)
    gate.set_instruction_denied(["release_object", "set_gripper", "drive"])
    assert isinstance(await gate.can_use_tool("mcp__robot__release_object", {}, ctx()), PermissionResultAllow)
    opening = {"open_fraction": 1.0, "close_until_effort": False}
    assert isinstance(await gate.can_use_tool("mcp__robot__set_gripper", opening, ctx()), PermissionResultAllow)
    closing = {"open_fraction": 0.0, "close_until_effort": True}
    assert isinstance(await gate.can_use_tool("mcp__robot__set_gripper", closing, ctx()), PermissionResultDeny)


async def test_instruction_denied_tools_are_cleared_by_reset() -> None:
    gate = gate_with_plan(rw=5)
    gate.set_instruction_denied(["drive"])
    gate.reset()
    gate.plan.set_plan("simple", "test", [one_phase(5)])
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


async def test_instruction_denied_tools_cannot_unlock_config_denied_tools() -> None:
    gate = gate_with_plan()
    gate.set_instruction_denied(["drive"])
    result = await gate.can_use_tool("mcp__robot__arm_set_home", {}, ctx())
    assert isinstance(result, PermissionResultDeny)
    assert "evaluation arm-only mode" not in result.message


async def test_instruction_denied_subsystem_entry_refuses_take_control_base_only() -> None:
    """Arm-only mode blocks the base by mechanism: no base lease means every base step is refused by mcp_server."""
    gate = gate_with_plan(rw=5)
    gate.set_instruction_denied(["take_control:base"])
    base = await gate.can_use_tool("mcp__robot__take_control", {"subsystem": "base"}, ctx())
    assert isinstance(base, PermissionResultDeny)
    assert base.message == "take_control(subsystem='base') is disabled for this instruction (evaluation arm-only mode)"
    arm = await gate.can_use_tool("mcp__robot__take_control", {"subsystem": "arm"}, ctx())
    assert isinstance(arm, PermissionResultAllow)
