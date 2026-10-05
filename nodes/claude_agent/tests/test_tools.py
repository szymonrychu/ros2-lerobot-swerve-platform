"""Tool classification, effector cap counting and permission denial."""

import pytest
from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from claude_agent.config import ClaudeAgentConfig
from claude_agent.tools import (
    BUILTIN_TOOLS,
    CAP_MESSAGE,
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
        "Read",
        "Write",
        "Edit",
        "Glob",
        "Grep",
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


@pytest.mark.parametrize("name", ["Bash", "Read", "WebFetch", "Task", "mcp__other__thing", "mcp__claude_ai__x"])
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
