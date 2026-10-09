"""System prompt is built from the config values."""

from claude_agent.config import ClaudeAgentConfig
from claude_agent.prompt import build_system_prompt


def test_prompt_explains_the_phase_plan_workflow() -> None:
    text = build_system_prompt(ClaudeAgentConfig(max_rw_cap=60, max_turn_cap=90, max_phase_rw_cap=22))
    lower = text.lower()
    for tool_name in ("set_task_plan", "complete_phase", "revise_plan", "raise_phase_budget"):
        assert f"agent.{tool_name}" in text
    assert "set_task_budget" not in text
    assert lower.index("first split") < lower.index("working method")
    for phrase in ("trivial", "very_complex", "without waiting for approval", "goal", "within 10 cm", "phases"):
        assert phrase in lower
    assert "call agent.set_task_plan first" in lower or "refused until" in lower
    assert "60" in text and "90" in text and "22" in text


def test_prompt_has_the_tomato_example_and_phase_guidance() -> None:
    lower = build_system_prompt(ClaudeAgentConfig()).lower()
    assert "put plushie tomato into toy car" in lower
    for step in (
        "locate mentioned objects",
        "drive towards tomato",
        "pick up tomato",
        "drop tomato",
        "get back to home",
    ):
        assert step in lower
    for guidance in ("rw 5-10", "rw 8-20", "rw 20-40", "rw 10-20", "rw 2-6"):
        assert guidance in lower
    assert "look_around" in lower and "counts 1" in lower


def test_prompt_has_no_ro_budget_and_says_sensors_are_unlimited() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    lower = text.lower()
    assert "ro_cap" not in lower and "ro 10" not in lower and "max_ro" not in lower
    assert "sensor calls are unlimited" in lower and "never counted against a budget" in lower
    assert "look as much as needed" in lower


def test_prompt_demands_generous_rw_and_turn_caps() -> None:
    lower = build_system_prompt(ClaudeAgentConfig()).lower()
    assert "roughly double" in lower and "retries" in lower
    assert "raise_phase_budget" in lower and "early" in lower


def test_prompt_budgets_for_retries_and_never_gives_up_at_a_cap() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "room for retries in rw_cap and turn_cap" in lower
    assert "2-4 attempts" in lower
    assert "before the budget runs out" in lower
    assert "agent.revise_plan to add a retry phase" in lower
    assert "never give up only because a cap is near" in lower


def test_prompt_teaches_accurate_grasping() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    section = text[text.index("Grasping:") :]
    lower = section.lower()
    assert "directly above" in lower and "several viewpoints" in lower and "changing the wrist roll" in lower
    assert "pixel_to_ground" in section and "mark_candidate_points" in section and "resolve_candidate" in section
    assert "centre" in lower and "not its edge" in lower
    assert "within about 1 cm" in lower and "average" in lower and "take another picture" in lower
    assert "left of the object centre" in lower and "observed offset" in lower
    assert "NOTES.md" in section


def test_prompt_demands_explicit_honest_phase_completion() -> None:
    lower = build_system_prompt(ClaudeAgentConfig()).lower()
    assert "complete every phase" in lower or "always complete" in lower
    assert "failed" in lower and "skipped" in lower and "honest" in lower
    assert "once per instruction" in lower and "once per phase" in lower


def test_prompt_has_no_old_static_caps() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    assert "30 effector" not in text and "50 turns" not in text
    assert "ro_cap" not in text


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
    assert "15 cm above the floor" in text
    assert "left to right" in text and "upright" in text
    assert "front" in text and "overhead" in text and "realsense" not in text.lower() and "stereo" not in text.lower()
    assert "overview first" in text and "arm-to-object distance" in text
    custom = build_system_prompt(ClaudeAgentConfig(arm_reach_cm=33, arm_base_height_m=0.2, camera_note="faces down"))
    assert "33 cm" in custom and "20 cm above the floor" in custom and "faces down" in custom
    assert "15 cm above" not in custom


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
    assert "objects are pois" in lower and "new session" in lower
    assert "your own pois and objects are removed" in lower and "person's pois stay" in lower
    assert "calibrat" in lower and "only when the person asks" in lower
    assert "capture_calibration_sample" in text and "solve_camera_calibration" in text


def test_every_default_tool_is_named_in_the_prompt() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    cfg = ClaudeAgentConfig()
    for name in cfg.sensor_tools + cfg.effector_tools:
        assert name in text


def prompt_lower() -> str:
    return " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())


def test_prompt_explains_the_fixed_and_moving_jaw() -> None:
    lower = prompt_lower()
    assert "one fixed jaw and one moving jaw" in lower
    assert "dark shape at the lower right" in lower and "closes in from the top" in lower
    assert "roll 0" in lower
    assert "beside or under the object's side, never onto the object" in lower
    assert "closes the object against it" in lower
    assert "object_width_m" in lower and "move_arm_cartesian" in lower
    assert "both object edges" in lower and "object centre" in lower


def test_prompt_makes_the_agent_choose_the_grasp_roll() -> None:
    lower = prompt_lower()
    assert "before each grasp" in lower and "choose the wrist roll" in lower
    assert "-90 deg = -1.57 rad" in lower and "narrow side" in lower
    assert "pass it as wrist_roll to move_arm_cartesian" in lower


def test_prompt_describes_camera_views_by_roll_in_the_grasping_paragraph() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    lower = " ".join(text.lower().split())
    assert "nearly straight down" in lower and "parallel to the ground" in lower
    assert "-154 deg" in lower and "upside down" in lower and "180 is not reachable" in lower
    assert "pixel_to_ground works at any roll" in lower
    assert lower.count("directly above") == 1  # merged into the one Grasping paragraph


def test_prompt_has_the_rolling_protocol() -> None:
    lower = prompt_lower()
    assert "before changing the roll" in lower
    assert "about half open" in lower and "keep the finger out of the picture" in lower
    assert "lift the arm clear of the robot body" in lower
    assert "open wider only for the grasp" in lower
    assert "refuse a roll with a wide open gripper" in lower


def test_prompt_teaches_surfaces_and_below_floor_reach() -> None:
    lower = prompt_lower()
    assert "surface_height_m" in lower and "above or below the robot's floor" in lower
    assert "below floor level" in lower and "joint limits" in lower and "unreachable" in lower


def test_prompt_says_the_arm_is_fast_by_default() -> None:
    lower = prompt_lower()
    assert "default is full speed" in lower
    assert "lower speed_scale only for the last few centimetres of a grasp or near obstacles" in lower
    assert "0.5 rad/s" not in lower and "slow speed" not in lower


def test_prompt_explains_grasp_macros_and_the_slow_zone() -> None:
    lower = prompt_lower()
    assert lower.index("plan_grasp") < lower.index("grasp_object")
    assert "plan_grasp first" in lower
    for word in ("scoop", "angled", "top_down", "auto", "release_object", "missed", "infeasible"):
        assert word in lower
    assert "surface_z_m" in lower and "stair" in lower and "hole" in lower
    assert "slow zone" in lower and "never blocks" in lower
    assert "tilt_override_deg" in lower and "imu" in lower
    assert "radially" in lower


def test_default_tool_lists_include_the_grasp_tools() -> None:
    cfg = ClaudeAgentConfig()
    assert {"grasp_object", "release_object"} <= set(cfg.effector_tools)
    assert "plan_grasp" in cfg.sensor_tools and "plan_grasp" not in cfg.effector_tools
