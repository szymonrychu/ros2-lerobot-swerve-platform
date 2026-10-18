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
    assert "once per instruction" in lower


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
    for phrase in ("look before", "small", "call stop", "release_control", "battery", "checkpoint"):
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
    assert "10 cm above the floor" in text
    assert "left to right" in text and "upright" in text
    assert (
        "front" in text
        and "head" in text
        and "overhead" not in text.lower()
        and "realsense" not in text.lower()
        and "stereo" not in text.lower()
    )
    assert "overview first" in text and "arm-to-object distance" in text
    custom = build_system_prompt(ClaudeAgentConfig(arm_reach_cm=33, arm_base_height_m=0.2, camera_note="faces down"))
    assert "33 cm" in custom and "20 cm above the floor" in custom and "faces down" in custom
    assert "10 cm above" not in custom


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
    default = " ".join(build_system_prompt(ClaudeAgentConfig()).split())
    assert "within 3 cm and 5 deg" in default
    assert "precise=true" in default and "within 1 cm and 2 deg" in default
    assert "sideways" in default and "rotate first" in default
    custom = " ".join(
        build_system_prompt(
            ClaudeAgentConfig(
                nav_goal_xy_tolerance_cm=2.5,
                nav_goal_yaw_tolerance_deg=4,
                nav_intermediate_xy_tolerance_cm=6,
                nav_intermediate_yaw_tolerance_deg=9,
            )
        ).split()
    )
    assert "within 6 cm and 9 deg" in custom and "within 2.5 cm and 4 deg" in custom
    assert "within 1 cm" not in custom


def test_prompt_explains_the_arm_settle_policy() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "settle" in lower and "trajectory_end" in lower and "settling" in lower
    assert "settle='final'" in lower


def test_prompt_teaches_body_awareness() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    for needle in ("robot_events_since_last_call", "vitals", "get_body_state", "interrupted_by", "ROBOT EVENT"):
        assert needle in text, needle


def test_prompt_teaches_the_robot_body_and_arm_reach() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    lower = " ".join(text.lower().split())
    section = lower[lower.index("robot body:") :]
    section = section[: section.index("spatial perception:")]
    for needle in ("letter h", "wheels", "side frames", "crossbar", "lidar", "slowed", "refused", "get_arm_reach"):
        assert needle in section, needle
    assert "self_collision" in section and "raise the arm" in section


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
    assert "objects are pois" in lower
    assert "removed" not in lower[lower.index("memory:") : lower.index("lookups:")]
    assert "vanish at a new session" not in lower and "removed at a new session" not in lower
    assert "calibrat" in lower and "only when the person asks" in lower
    assert "capture_calibration_sample" in text and "solve_camera_calibration" in text


def test_prompt_has_map_marking_section_with_triggers() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    assert text.index("Memory:") < text.index("Safety rules:") < text.index("Map marking:") < text.index("Arm settle:")
    section = text[text.index("Map marking:") : text.index("Arm settle:")]
    lower = section.lower()
    assert "look_around" in section and "camera image" in lower and "object" in lower
    assert "collision_stop" in section and "nav2" in lower and "abort" in lower
    assert "stuck point" in lower and "keepout" in lower and "slow" in lower
    assert "complete_phase" in section
    assert "moved" in lower
    assert "pois persist across sessions" in lower
    assert "notes.md holds no map coordinates" in lower


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


def test_prompt_teaches_describing_stairs_tables_and_holes_as_surfaces() -> None:
    """Step-edge model (2026-10-10): when the robot floor and the object's surface differ, pass surfaces with the step
    edge position and height instead of one surface_z_m, so the planner keeps the wrist clear of the edge."""
    lower = prompt_lower()
    assert "surfaces" in lower and "step edge" in lower
    for word in ("edge", "point", "direction", "polygon", "height_m", "table", "hole"):
        assert word in lower, word
    assert "instead of a single surface_z_m" in lower
    assert "support_z" in lower and "must match" in lower


def test_prompt_states_grasp_auto_order_and_scoop_gap_rule() -> None:
    """Sim validation (2026-10-09): scoop shoves objects resting flat; it needs a gap under the object."""
    lower = prompt_lower()
    assert "top_down, then angled, then scoop" in lower
    assert "gap_below_m" in lower
    assert "good for flat or low objects" not in lower


def test_prompt_explains_grip_profiles_and_when_to_choose_them() -> None:
    lower = prompt_lower()
    assert "grip_profile" in lower
    for profile in ("gentle", "normal", "firm", "soft"):
        assert profile in lower
    para = lower[lower.index("grip strength") :]
    para = para[: para.index("surfaces and speed:")]
    assert para.index("gentle") < para.index("fragile")
    for word in ("soft", "light", "ordinary", "heavy", "slippery", "tools"):
        assert word in para
    assert "holding_load" in para and "slipping" in para and "crush_risk" in para
    assert "firmer" in para and "gentler" in para and "retry" in para
    assert "set_gripper" in para and "grasp_object" in para


def test_prompt_sends_plush_and_soft_toys_to_the_soft_profile() -> None:
    lower = prompt_lower()
    para = lower[lower.index("grip strength") :]
    para = para[: para.index("surfaces and speed:")]
    assert "plush" in para and "soft toys" in para and "grip_profile soft" in para
    assert "prefer gentle or normal" not in para
    assert "retightened" in para


def test_default_tool_lists_include_the_grasp_tools() -> None:
    cfg = ClaudeAgentConfig()
    assert {"grasp_object", "release_object"} <= set(cfg.effector_tools)
    assert "plan_grasp" in cfg.sensor_tools and "plan_grasp" not in cfg.effector_tools
    assert "approach_for_grasp" in cfg.effector_tools and "approach_for_grasp" not in cfg.sensor_tools
    assert "suggest_approach_pose" in cfg.sensor_tools and "suggest_approach_pose" not in cfg.effector_tools


def test_prompt_orders_the_approach_tools_before_the_grasp_macros() -> None:
    lower = prompt_lower()
    line = lower[lower.index("suggest_approach_pose before") :]
    line = line[: line.index("grasp_object") + len("grasp_object")]
    assert line.index("suggest_approach_pose") < line.index("approach_for_grasp") < line.index("plan_grasp")
    assert "base lease" in line


def test_prompt_uses_checkpoint_verification_not_a_check_after_every_motion() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "after every motion confirm" not in lower
    assert "checkpoint" in lower
    for phrase in ("phase boundar", "irreversible", "grasp", "release", "tool reports a problem", "trust"):
        assert phrase in lower
    assert "never assume success without checking" not in lower


def test_prompt_keeps_the_motion_safety_rules() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "an open moving finger has jammed" in lower
    assert "call stop whenever you are unsure" in lower
    assert "do not retry" in lower


def test_prompt_asks_for_terse_batched_calls_and_small_images() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "terse" in lower and "do not restate" in lower
    assert "short tool arguments" in lower
    assert "independent" in lower and "one turn" in lower
    assert "max_px" in lower and "aiming" in lower


def test_prompt_teaches_the_motion_queue_pattern() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    for phrase in (
        "enqueue_motions",
        "wait_for_event",
        "cancel_motions",
        "get_motion_status",
        "precondition",
        "at least 2 steps deep",
        "instead of polling",
        "replace=true",
        "blend",
        "gripper_holding",
        "on_fail",
    ):
        assert phrase in lower, phrase
    # Verification stays at checkpoints and the safety rules hold for queued motions too.
    assert "checkpoint" in lower and "queued" in lower
    assert "roll" in lower and "elbow_flex" in lower and "negative" in lower and "slow zone" in lower


def test_prompt_states_the_negative_elbow_rule() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "negative elbow_flex stretches the arm" in lower


def test_prompt_mentions_the_fast_look_around() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "return_to_start" in lower


def test_prompt_tells_the_agent_to_refine_poi_positions_with_better_estimates() -> None:
    """remember_object averages comparable sightings; a clearly better estimate or a moved object replaces the position."""
    lower = prompt_lower()
    assert "better location estimate" in lower
    assert "update_poi" in lower and "averages" in lower
    assert "position refined from" in lower
    assert "person's pois" in lower and "only their position" in lower


def test_prompt_says_to_pass_the_image_size_with_pixels() -> None:
    """2026-10-10: get_camera_image returns 384x288 but the pixel tools expect the calibrated 640x480 pixels."""
    lower = prompt_lower()
    assert "image_width" in lower and "image_height" in lower
    assert "384x288" in lower and "get_annotated_camera_image" in lower
    para = lower[lower.index("pixel sizes:") :]
    para = para[: para.index("gripper: it has")]
    assert "pixel_to_ground" in para and "mark_candidate_points" in para
    assert "calibrated" in para


def test_prompt_explains_front_camera_zoom() -> None:
    """The front camera is the head camera: 384x216 by default, zoom crops on demand."""
    lower = prompt_lower()
    para = lower[lower.index("pixel sizes:") :]
    para = para[: para.index("gripper: it has")]
    assert "384x216" in para and "384x288" in para
    assert "zoom={u, v, factor}" in para and "fractions" in para
    assert "2/4/8" in para and "1024" in para and "af_failed" in para
    assert "max_px" in para and "stream" in para
    assert "overhead" not in lower and "front (head) camera" in lower


def test_prompt_explains_the_centred_grasp_shift_and_residual_reports() -> None:
    """2026-10-10: the centring shift of grasp_object was read as a sideways drift."""
    lower = prompt_lower()
    para = lower[lower.index("grasp macros:") :]
    para = para[: para.index("grip strength:")]
    assert "grasp_shift" in para and "tool_point" in para and "jaw centre" in para
    assert "not a drift" in para
    assert "residual_error" in para and "warnings" in para and "held_pose" in para
    assert "approach_speed_scale" in para
    # 2026-10-10 JC1: the opening is centred on the object, so the fixed jaw clears the side face too
    assert "half the object width beside it" not in para
    assert "half the object width plus the fixed jaw clearance" in para and "shift_m" in para


def test_prompt_states_soft_objects_get_squashed_whatever_the_load() -> None:
    lower = prompt_lower()
    para = lower[lower.index("grip strength:") :]
    para = para[: para.index("surfaces and speed:")]
    assert "squash" in para and "plush" in para


def test_prompt_explains_the_unverified_grasp_outcome() -> None:
    """E1: every held close returns 'unverified' with a jaw crop; the agent confirms it with confirm_grasp."""
    lower = prompt_lower()
    macros = lower[lower.index("grasp_object reports") :]
    macros = macros[: macros.index("grip strength:")]
    assert "unverified" in macros and "jaw crop" in macros
    assert "confirm_grasp(grasp_id, yes|no|unsure, evidence)" in macros
    assert "before complete_phase" in macros
    assert "only a fused 'grasped'" in macros
    assert "another wrist roll or get_camera_image" in macros
    assert "missed means release_object" in macros
    assert "image alone never proves" in macros
    assert "means look and decide" not in lower
    assert "treat it as held" not in lower
    para = lower[lower.index("grip strength:") :]
    para = para[: para.index("surfaces and speed:")]
    assert "unverified" in para and "plush" in para
    assert "grip_profile soft" in para  # 2026-10-11 12:18: plush slipped out of a normal hold
    assert "an empty close (nothing between the jaws) is reported as missed" not in para


def test_prompt_says_tool_rejections_never_come_from_the_user() -> None:
    lower = build_system_prompt(ClaudeAgentConfig()).lower()
    assert "the user doesn't want to proceed with this tool use" in lower
    assert "never from the person" in lower
    assert "budget gate" in lower
    assert "re-run" in lower


def test_prompt_explains_the_instruction_time_left_rule() -> None:
    text = build_system_prompt(ClaudeAgentConfig(min_effector_time_left_s=45))
    assert "instruction time left" in text
    assert "45 s" in text


def test_prompt_has_the_tilt_guard_section() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    section = lower[lower.index("tilt guard:") :]
    for needle in (
        "tilt_warning",
        "tilt_clear",
        "tilt_error",
        "nav2",
        "refused",
        "manually",
        "short base moves",
        "get_body_state",
        "base motion is disabled",
        "arm still and compact",
        "report to the user",
        "never try to work around",
    ):
        assert needle in section, needle


def assert_phrases(lower: str, phrases: tuple[str, ...]) -> None:
    for phrase in phrases:
        assert phrase in lower, phrase


def test_prompt_teaches_control_leases() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "take_control(subsystem 'arm' or 'base')",
            "before any arm or base motion",
            "release_control(subsystem, keep_position=false)",
            "arm homed",
            "base steering neutral",
            "get_control_state",
            "head tools (set_head_angles, head_look_at, head_home) work under either the arm or the base lease",
            "arm_set_home is denied to you",
        ),
    )
    assert "acquire_control" not in lower
    assert "use arm_set_home" not in lower


def test_prompt_replaces_the_old_release_rule_with_the_end_of_job_rule() -> None:
    lower = prompt_lower()
    assert "when you are done with the arm so the leader arm and web ui work again" not in lower
    assert_phrases(
        lower,
        (
            "end of the job",
            "release_control(subsystem='all')",
            "arm is parked",
            "leases are released",
            "plan is finished",
            "job end refused",
            "after 2 refusals",
            "force-parks",
        ),
    )


def test_prompt_teaches_the_dynamic_footprint_and_carrying() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "release_control(subsystem='arm', keep_position=true, carried_object_radius_m=<radius of the held object",
            "0 to 0.3",
            "arm shadow",
            "carried circle",
            "taking arm control back reverts to the static footprint",
            "refused while you carry an object",
            "put it down first with release_object",
            "cannot park with an object in the gripper",
        ),
    )


def test_prompt_teaches_the_plan_tree() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "plan tree",
            "'seq'",
            "'if'",
            "'while'",
            "'phase'",
            "max_iterations",
            "max_attempts",
            "pickloop",
            "outcome_of",
            "{ask:",
            "answer_condition(node_id, value, evidence)",
            "while an ask is pending",
            "effector tools are refused",
        ),
    )
    assert "agent.answer_condition" in build_system_prompt(ClaudeAgentConfig())


def test_prompt_teaches_retry_estimates() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "p_retry_success",
            "0 to 1",
            "honest estimate",
            "retry_change",
            "automatic retry",
            "threshold",
            "attempts remain",
            "automatic retry n/m - apply: ...",
        ),
    )


def test_prompt_asks_for_more_pois() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "mark more pois",
            "objects and areas of interest",
            "scratchpad for future assignments",
            "critical obstacles",
            "area poi",
            "with a note",
        ),
    )


def test_prompt_verifies_every_pick_up_with_sensors() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "verify every pick-up",
            "gripper camera",
            "holding_load",
            "slipping",
            "jaw position",
            "get_arm_state",
            "before complete_phase done",
            "close it as failed with a p_retry_success estimate",
        ),
    )


def test_prompt_teaches_lookups() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "before re-searching an area or object",
            "list_lookups",
            "near",
            "poi_id",
            "text",
            "since",
            "get_lookup_image",
            "annotate_lookup",
            "lookup_id",
            "photo poi",
            "camera view",
            "delete_photos",
            "delete_poi",
            "only when asked",
            "survive",
        ),
    )


def test_prompt_teaches_bump_and_lift_interrupts() -> None:
    lower = prompt_lower()
    assert_phrases(
        lower,
        (
            "warning bump interrupts you after the tools in flight finish",
            "lifted",
            "carried",
            "critical interrupts",
            "stop and report",
        ),
    )


def test_prompt_states_the_job_end_followups_and_retry_threshold_from_config() -> None:
    text = " ".join(
        build_system_prompt(ClaudeAgentConfig(job_end_max_followups=3, retry_success_threshold=0.7)).split()
    )
    assert "after 3 refusals" in text and "after 2 refusals" not in text
    assert "retry threshold (0.7)" in text
    once = " ".join(build_system_prompt(ClaudeAgentConfig(job_end_max_followups=1)).split())
    assert "after 1 refusal," in once
    at_once = " ".join(build_system_prompt(ClaudeAgentConfig(job_end_max_followups=0)).split())
    assert "force-parks the arm at once" in at_once


def test_prompt_explains_failure_typing() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    for phrase in (
        "failure_type",
        "execution_failure",
        "the step did not achieve its effect",
        "plan_failure",
        "the steps worked but the plan was wrong",
        "revise the plan instead of retrying",
        "eai_error_type",
        "explanation",
        "what you observed",
        "evidence_lookup_ids",
        "lookup_id",
    ):
        assert phrase in lower


def test_prompt_teaches_keepout_and_slow_zones() -> None:
    text = " ".join(build_system_prompt(ClaudeAgentConfig()).split()).lower()
    assert "zone 'keepout'" in text and "zone 'slow'" in text and "speed_limit_pct" in text
    assert "never send a navigation goal inside a keepout" in text
    assert "ask the person before setting the zone status to cancelled" in text
    assert "vanish" not in text
    assert "draw permanent zones in the web ui" in text
    assert "after a map reset" in text and "check the zones still sit where intended" in text
    assert "a place the person says not to go" in text


def test_prompt_teaches_frontier_exploration() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    lower = " ".join(text.lower().split())
    section = lower[lower.index("exploration:") :]
    section = section[: section.index("pixel sizes:")]
    for needle in (
        "list_frontiers",
        "get_topdown_view",
        "cyan",
        "last_lookup_facing",
        "nobody has looked",
        "look_toward_frontier",
        "plan_frontier_visit",
        "go_to_frontier",
        "stable",
        "successor",
        "lease",
    ):
        assert needle in section, needle
    assert lower.index("spatial perception:") < lower.index("exploration:")


def test_prompt_teaches_the_time_budget_and_repeatable_raises() -> None:
    cfg = ClaudeAgentConfig(max_instruction_time_min=90, instruction_timeout_s=600)
    lower = " ".join(build_system_prompt(cfg).lower().split())
    assert "time_budget_min" in lower and "whole task" in lower
    assert "at most 90 minutes" in lower
    assert "600 s" in lower and "until the plan is set" in lower
    assert "raise_phase_budget" in lower and "more than once per phase" in lower
    assert "(once per phase, with a" not in lower
    assert "never past 90 minutes" in lower


def test_prompt_plans_time_and_caps_at_twice_the_honest_estimate() -> None:
    lower = " ".join(build_system_prompt(ClaudeAgentConfig()).lower().split())
    assert "time_budget_min is required" in lower
    assert "time_budget_min and the caps at about twice your honest estimate" in lower
    assert "prefer overshooting and delivering to undershooting and stopping" in lower
    assert "raise budgets early with agent.raise_phase_budget rather than end unfinished" in lower


def norm_prompt() -> str:
    return " ".join(build_system_prompt(ClaudeAgentConfig()).split())


def test_prompt_has_skills_and_workflows_paragraph_after_working_method() -> None:
    text = norm_prompt()
    lower = text.lower()
    assert "Skills and workflows:" in text
    assert lower.index("then perform the task") < lower.index("skills and workflows:")
    section = text[text.index("Skills and workflows:") :]
    section = section[: section.index("Notes:")] if "Notes:" in section else section[:3000]
    for needle in (
        "Skill tool",
        "robot-skills:",
        "read-only",
        "robot-skills:planning",
        "agent.start_workflow",
        "discovery",
        "drive_to_target",
        "pick_place",
        "dry_run",
        "set_task_plan",
        "Mandatory steps",
        "RETROSPECTIVE",
        "agent.record_retrospective",
        "RECENT RETROSPECTIVES",
    ):
        assert needle in section, needle


def test_prompt_points_exploration_to_sightseeing_and_status_tools() -> None:
    text = norm_prompt()
    section = text[text.index("Exploration:") : text.index("Pixel sizes:")]
    for needle in (
        "robot-skills:sightseeing",
        "get_exploration_status",
        "mark_survey_spot",
        "list_frontiers",
        "look_toward_frontier",
        "go_to_frontier",
    ):
        assert needle in section, needle


def test_prompt_teaches_poi_categories_and_removal_rules() -> None:
    text = norm_prompt()
    lower = text.lower()
    assert "robot-skills:poi-categories" in text
    for needle in ("target", "slow zone", "keep-out", "problem spot", "detail", "survey=true"):
        assert needle in lower, needle
    for needle in ("window", "mirror", "glass", "lidar plane"):
        assert needle in lower, needle
    assert "new session deletes no pois" in lower
    assert "clear_pois" in lower and "delete marked" in lower
    assert "after the person asked" in lower
    assert "creating a keep-out is free" in lower
    assert "cancelling or deleting one needs the person's consent" in lower


def test_prompt_states_photo_rules_for_maps_and_cleanups() -> None:
    lower = norm_prompt().lower()
    assert "a map holds its pois and photos" in lower
    assert "save_map stores both" in lower
    assert "load_map replaces both" in lower
    assert "delete_map removes both" in lower
    assert "clear_pois defaults to your own pois and photos and keeps zones" in lower
    assert "web ui reset map clears the live pois and photos" in lower
    assert "saved maps are never deleted by a move or a reset" in lower
    assert "travel with saved maps" in lower
    assert "belong to the live map" in lower
    assert "photos survive sessions and restarts" in lower
    assert "photos are never cleared" not in lower
    assert "photos survive sessions and restarts." not in lower


def test_prompt_teaches_head_tools_lease_rule_and_home_on_park(config: ClaudeAgentConfig) -> None:
    text = build_system_prompt(config)
    assert "head_look_at" in text
    assert "either the arm or the base lease" in text
    assert "reset.head" in text
    assert "A head tool (future)" not in text
    assert "Zoom never moves the head" in text
