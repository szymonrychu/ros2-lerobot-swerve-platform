"""System prompt is built from the config values."""

from claude_agent.config import ClaudeAgentConfig
from claude_agent.prompt import build_system_prompt


def test_prompt_states_caps_from_config() -> None:
    text = build_system_prompt(ClaudeAgentConfig(max_turns=17, effector_call_cap=9))
    assert "9 effector" in text
    assert "17 turns" in text
    assert "unlimited" in text.lower()
    assert "30 effector" not in text
    assert "50 turns" not in text


def test_prompt_lists_tools_by_kind(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config)
    for name in config.effector_tools + config.uncapped_tools + config.sensor_tools:
        assert name in text


def test_prompt_safety_rules(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config).lower()
    for phrase in ("look before", "small", "call stop", "release_control", "battery", "never assume"):
        assert phrase in text


def test_prompt_persona_and_extra() -> None:
    base = build_system_prompt(ClaudeAgentConfig())
    assert "chat" in base.lower()
    extra = build_system_prompt(ClaudeAgentConfig(system_prompt_extra="Name: Rover"))
    assert extra.endswith("Name: Rover")
    assert "Name: Rover" not in base
