"""Tool classification, effector cap counting and permission denial."""

from pathlib import Path

import pytest
from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from claude_agent.config import ClaudeAgentConfig
from claude_agent.tools import (
    BUILTIN_TOOLS,
    CAP_MESSAGE,
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


async def test_sensor_and_uncapped_never_counted(config: ClaudeAgentConfig) -> None:
    gate = EffectorGate(ClaudeAgentConfig(effector_call_cap=1))
    for _ in range(50):
        assert isinstance(await gate.can_use_tool("mcp__robot__get_camera_image", {}, ctx()), PermissionResultAllow)
        assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)
    assert gate.used == 0


async def test_effector_calls_counted_then_denied() -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(effector_call_cap=2), on_denied=denied.append)
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {"x": 0.1}, ctx("a")), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__set_gripper", {}, ctx("b")), PermissionResultAllow)
    assert gate.used == 2
    result = await gate.can_use_tool("mcp__robot__drive", {}, ctx("c"))
    assert isinstance(result, PermissionResultDeny)
    assert result.message == "effector call cap reached (2 per instruction); stop and report to the user"
    assert gate.used == 2
    assert denied == [{"id": "c", "name": "drive", "reason": result.message}]
    # stop still allowed at the cap
    assert isinstance(await gate.can_use_tool("mcp__robot__stop", {}, ctx()), PermissionResultAllow)


def test_cap_message_default_text() -> None:
    assert CAP_MESSAGE.format(cap=30) == "effector call cap reached (30 per instruction); stop and report to the user"


async def test_reset_clears_counter() -> None:
    gate = EffectorGate(ClaudeAgentConfig(effector_call_cap=1))
    await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultDeny)
    gate.reset()
    assert gate.used == 0
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, ctx()), PermissionResultAllow)


@pytest.mark.parametrize("name", ["Bash", "WebFetch", "Task", "mcp__other__thing", "mcp__claude_ai__x"])
async def test_everything_else_denied(name: str) -> None:
    denied: list[dict] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_denied=denied.append)
    result = await gate.can_use_tool(name, {}, ctx("z"))
    assert isinstance(result, PermissionResultDeny)
    assert "not available" in result.message
    assert denied[0]["name"] == name and denied[0]["id"] == "z"
    assert gate.used == 0


async def test_count_change_callback() -> None:
    seen: list[int] = []
    gate = EffectorGate(ClaudeAgentConfig(), on_count=seen.append)
    await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    await gate.can_use_tool("mcp__robot__drive", {}, ctx())
    assert seen == [1, 2]


# --- file tools for notes, sandboxed to the workdir ---------------------------------------------------------------


@pytest.fixture
def workdir(tmp_path: Path) -> Path:
    path = tmp_path / "workspace"
    path.mkdir()
    return path


def notes_gate(workdir: Path, **kwargs) -> EffectorGate:
    return EffectorGate(ClaudeAgentConfig(workdir=str(workdir), effector_call_cap=1), **kwargs)


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
    for _ in range(20):
        for tool in NOTES_TOOLS:
            await gate.can_use_tool(tool, {"file_path": "NOTES.md", "path": "."}, ctx())
    assert gate.used == 0
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
    assert gate.used == 0


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
