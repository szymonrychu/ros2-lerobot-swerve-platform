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


def test_prompt_working_method_in_order(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config)
    lower = text.lower()
    top = lower.index("top-level view")
    explore = lower.index("gently explore")
    perform = lower.index("then perform the task")
    assert top < explore < perform
    assert "map summary" in lower and "camera" in lower


def test_prompt_notes_instructions_use_workdir(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config)
    assert "NOTES.md" in text and "workdir" in text.lower()
    assert "read it at the start of each instruction" in text.lower()
    assert "update" in text.lower() and "never store secrets" in text.lower()
    assert "no shell" in text.lower()


def test_prompt_hardware_facts_from_config(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config)
    assert "SO-101" in text
    assert f"{config.arm_reach_cm:g} cm" in text
    assert "16.5 cm above the floor" in text
    assert "left to right" in text and "upright" in text
    custom = build_system_prompt(ClaudeAgentConfig(arm_reach_cm=33, arm_base_height_m=0.2, camera_note="faces down"))
    assert "33 cm" in custom and "20 cm above the floor" in custom and "faces down" in custom
    assert "16.5 cm" not in custom


def test_arm_reach_default_matches_urdf_link_lengths() -> None:
    import re
    from pathlib import Path

    from claude_agent.config import ARM_REACH_CM

    urdf = (Path(__file__).resolve().parents[3] / "nodes/web_ui/urdf/so101_arm.urdf").read_text()

    def offset(joint: str) -> float:
        block = re.search(rf'<joint name="{joint}".*?<origin xyz="([^"]+)"', urdf, re.S).group(1).split()
        return sum(float(v) ** 2 for v in block) ** 0.5

    total = offset("elbow_flex") + offset("wrist_flex") + offset("wrist_roll") + offset("gripper_frame_joint")
    assert abs(total * 100 - ARM_REACH_CM) < 1.0
    assert ClaudeAgentConfig().arm_reach_cm == ARM_REACH_CM
