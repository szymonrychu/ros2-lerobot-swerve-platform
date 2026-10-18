"""Named map tools: classification and the prompt rules around them."""

import pytest

from claude_agent.config import (
    DEFAULT_DENIED_TOOLS,
    DEFAULT_EFFECTOR_TOOLS,
    DEFAULT_SENSOR_TOOLS,
    DEFAULT_UNCAPPED_TOOLS,
    ClaudeAgentConfig,
)
from claude_agent.prompt import build_system_prompt

MAP_TOOLS = ["save_map", "list_maps", "load_map", "delete_map", "clear_pois"]


def prompt_text() -> str:
    return " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())


@pytest.mark.parametrize("tool", MAP_TOOLS)
def test_map_tool_classification_is_sensor_only(tool: str) -> None:
    assert tool in DEFAULT_SENSOR_TOOLS
    assert tool not in DEFAULT_EFFECTOR_TOOLS
    assert tool not in DEFAULT_UNCAPPED_TOOLS
    assert tool not in DEFAULT_DENIED_TOOLS
    assert tool in ClaudeAgentConfig().sensor_tools


@pytest.mark.parametrize(
    "phrase",
    [
        "after load_map, read the verdict",
        "do not navigate unless the state is localised",
        "retry load_map with a pose hint only when the robot's real position is known",
        "after a load call list_pois and check the zones",
        "clear_pois defaults to your own pois and keeps zones",
        "only when they asked",
        "a map holds its pois",
        "loading replaces them",
        "deleting removes them",
        "call delete_map only after the person asked and confirmed (confirm=true)",
        "wait for the verdict",
        "recovered means carry on",
        "clears the live map and pois",
        "offer load_map of a saved map",
        "saved maps are never deleted by a move",
    ],
)
def test_prompt_contains_rule(phrase: str) -> None:
    assert phrase in prompt_text()
