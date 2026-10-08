"""Tool classification, budget gating (ro/rw counting) and permission denial."""

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
]


def one_phase(ro: int = 3, rw: int = 2, turns: int = 20, name: str = "Work") -> dict:
    return {"name": name, "goal": "goal reached", "ro_cap": ro, "rw_cap": rw, "turn_cap": turns}


def gate_with_plan(ro: int = 3, rw: int = 2, turns: int = 20, **kwargs) -> EffectorGate:
    gate = EffectorGate(ClaudeAgentConfig(), **kwargs)
    gate.plan.set_plan("simple", "test", [one_phase(ro, rw, turns)])
    return gate


@pytest.mark.parametrize("name", PLAN_TOOLS)
def test_classify_planning_tools(config: ClaudeAgentConfig, name: str) -> None:
    assert classify_tool(config, name) == "plan"
    assert short_name(name) == name.removeprefix("mcp__agent__")


def test_classify_unknown_agent_tool_is_none(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "mcp__agent__other") is None
    assert classify_tool(config, "mcp__agent__set_task_budget") is None


async def test_every_robot_tool_denied_until_a_plan_is_set() -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_denied=denied.append)
    for name in ("get_robot_state", "get_camera_image", "drive", "move_arm_joints"):
        result = await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx("t"))
        assert isinstance(result, PermissionResultDeny)
        assert result.message == "call agent.set_task_plan first"
    assert len(denied) == 4 and denied[0] == {"id": "t", "name": "get_robot_state", "reason": result.message}
    assert gate.plan.ro_used == 0 and gate.plan.rw_used == 0


async def test_planning_and_notes_tools_allowed_without_a_plan(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    for name in PLAN_TOOLS:
        assert isinstance(await gate.can_use_tool(name, {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)


async def test_stop_and_control_tools_always_allowed_and_never_counted() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    for name in ("stop", "acquire_control", "release_control"):
        assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx()), PermissionResultAllow)
    gate.plan.set_plan("simple", "x", [one_phase(1, 1, 5)])
    for _ in range(20):
        assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)


async def test_ro_and_rw_counted_separately_then_denied() -> None:
    denied: list[dict] = []
    gate = gate_with_plan(ro=2, rw=1, on_denied=denied.append)
    for name in ("get_robot_state", "get_arm_state"):
        assert isinstance(await gate.can_use_tool(f"mcp__robot__{name}", {}, ctx()), PermissionResultAllow)
    result = await gate.can_use_tool("mcp__robot__get_camera_image", {}, ctx("c"))
    assert isinstance(result, PermissionResultDeny)
    assert result.message.startswith("phase 1 'Work' ro budget exhausted (2/2)")
    assert "complete_phase" in result.message and "raise_phase_budget" in result.message
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)
    result = await gate.can_use_tool("mcp__robot__set_gripper", {}, ctx("d"))
    assert isinstance(result, PermissionResultDeny) and result.message.startswith(
        "phase 1 'Work' rw budget exhausted (1/1)"
    )
    assert (gate.plan.ro_used, gate.plan.rw_used) == (2, 1)
    assert denied[0]["id"] == "c" and denied[0]["name"] == "get_camera_image"
    assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)


async def test_unknown_robot_tool_counts_as_rw() -> None:
    gate = gate_with_plan(rw=1)
    assert isinstance(await gate.can_use_tool("mcp__robot__new_motion_tool", {}, ctx()), PermissionResultAllow)
    assert gate.plan.rw_used == 1


async def test_notes_tools_are_not_counted(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    gate.plan.set_plan("simple", "x", [one_phase(1, 1, 5)])
    for _ in range(10):
        assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used) == (0, 0)


async def test_calls_count_against_the_active_phase_and_the_next_phase_starts_fresh() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    gate.plan.set_plan("simple", "x", [one_phase(1, 1, 20, "A"), one_phase(1, 1, 20, "B")])
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultDeny)
    gate.plan.complete_phase("done", "ok")
    assert isinstance(await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx()), PermissionResultAllow)


async def test_phase_turn_cap_denies_robot_tools_but_not_planning_tools(workdir: Path) -> None:
    gate = EffectorGate(ClaudeAgentConfig(workdir=str(workdir)))
    gate.plan.set_plan("simple", "x", [one_phase(5, 5, 1)])
    gate.plan.count_turn()
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("d"))
    assert isinstance(result, PermissionResultDeny) and "turn budget exhausted (1/1)" in result.message
    assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool(PLAN_TOOLS[1], {}, ctx()), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("Read", {"file_path": "NOTES.md"}, ctx()), PermissionResultAllow)


async def test_robot_tools_denied_after_the_last_phase_is_completed() -> None:
    gate = gate_with_plan()
    gate.plan.complete_phase("done", "all done")
    result = await gate.can_use_tool("mcp__robot__get_robot_state", {}, ctx())
    assert isinstance(result, PermissionResultDeny) and "all phases are completed" in result.message


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
    gate.plan.set_plan("simple", "x", [one_phase(5, 5, 5)])
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
    gate.plan.set_plan("simple", "x", [one_phase(5, 5, 20)])
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
