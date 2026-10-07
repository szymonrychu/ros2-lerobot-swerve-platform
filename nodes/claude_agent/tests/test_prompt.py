"""System prompt is built from the config values."""

from claude_agent.config import ClaudeAgentConfig
from claude_agent.prompt import build_system_prompt


def test_prompt_explains_the_budget_workflow() -> None:
    text = build_system_prompt(ClaudeAgentConfig(max_ro_cap=200, max_rw_cap=60, max_turn_cap=90))
    lower = text.lower()
    assert "set_task_budget" in text and "agent" in text
    first = lower.index("first judge")
    assert first < lower.index("working method")
    for phrase in ("trivial", "simple", "moderate", "complex", "very_complex", "without waiting for approval"):
        assert phrase in lower
    for guidance in ("ro 5-10", "rw 0", "ro 10-20", "rw 3-8", "ro 40-80", "rw 25-50", "turns 40-80"):
        assert guidance in lower
    assert "200" in text and "60" in text and "90" in text
    assert "once" in lower and "raise" in lower
    assert "call agent.set_task_budget first" in lower or "refused until" in lower


def test_prompt_has_no_old_static_caps() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    assert "30 effector" not in text and "50 turns" not in text
    assert "unlimited" not in text.lower()


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
    assert "front" in text and "overhead" in text and "realsense" not in text.lower() and "stereo" not in text.lower()
    assert "overview first" in text and "arm-to-object distance" in text
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


def test_prompt_states_nav_goal_precision_from_config() -> None:
    default = build_system_prompt(ClaudeAgentConfig())
    assert "within 1 cm and 2 deg" in default
    assert "sideways" in default and "rotate first" in default
    custom = build_system_prompt(ClaudeAgentConfig(nav_goal_xy_tolerance_cm=2.5, nav_goal_yaw_tolerance_deg=4))
    assert "within 2.5 cm and 4 deg" in custom
    assert "within 1 cm" not in custom


def test_prompt_teaches_body_awareness() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    for needle in ("robot_events_since_last_call", "vitals", "get_body_state", "interrupted_by", "ROBOT EVENT"):
        assert needle in text, needle


def test_prompt_teaches_spatial_perception_workflow() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    for needle in (
        "get_topdown_view",
        "look_around",
        "get_annotated_camera_image",
        "mark_candidate_points",
        "resolve_candidate",
        "pixel_to_ground",
        "not calibrated",
    ):
        assert needle in text, needle
    section = text[text.index("Spatial perception:") :]
    assert section.index("get_topdown_view") < section.index("mark_candidate_points")


def test_prompt_teaches_memory_pois_and_calibration() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    for needle in ("remember_object", "list_objects", "list_pois", "add_poi", "update_poi", "NOTES.md"):
        assert needle in text, needle
    lower = text.lower()
    assert "calibrat" in lower and "only when the person asks" in lower
    assert "capture_calibration_sample" in text and "solve_camera_calibration" in text


def test_every_default_tool_is_named_in_the_prompt() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    cfg = ClaudeAgentConfig()
    for name in cfg.sensor_tools + cfg.effector_tools:
        assert name in text
