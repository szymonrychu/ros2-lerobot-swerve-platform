"""Tests for mcp_server.config (pydantic YAML config, env lookup, bearer token)."""

import re
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mcp_server.config import (
    DEFAULT_CONFIG_PATH,
    REPO_ROOT,
    GripProfileOverride,
    McpServerConfig,
    MissingTokenError,
    MotionQueueSettings,
    config_path_from_env,
    load_config,
    token_from_env,
)
from mcp_server.grip import resolve_grip_profile


def test_defaults_match_documented_server_endpoint() -> None:
    cfg = McpServerConfig()
    assert (cfg.server.host, cfg.server.port, cfg.server.path) == ("0.0.0.0", 18200, "/mcp")


def test_default_topics_follow_the_robot_graph() -> None:
    t = McpServerConfig().topics
    assert t.autonomy_command == "/filter/autonomy_joint_commands"
    assert t.autonomy_release == "/filter/autonomy_release"
    assert t.active_source == "/filter/active_source"
    assert t.follower_joint_states == "/follower/joint_states"
    assert t.cmd_vel == "/cmd_vel_nav"
    assert t.scan == "/scan_filtered"
    assert t.gripper_camera == "/camera_0/image_raw/compressed"
    assert t.head_camera == "/head_camera/image_raw/compressed"
    assert t.navigate_action == "/navigate_to_pose"


def test_retired_camera_topic_key_is_rejected() -> None:
    """The retired front camera topic key is no longer accepted (extra keys are forbidden)."""
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"topics": {"front_camera": "/retired/image_raw/compressed"}})


def test_default_limits_are_conservative() -> None:
    lim = McpServerConfig().limits
    assert lim.max_linear_mps == pytest.approx(0.25)
    assert lim.max_angular_rps == pytest.approx(0.5)
    assert lim.max_drive_duration_s == pytest.approx(2.0)
    assert lim.drive_rate_hz == pytest.approx(20.0)
    assert lim.arm_rate_hz == pytest.approx(25.0)
    assert lim.arm_max_joint_velocity_rps == pytest.approx(1.0)
    assert lim.arm_max_speed_scale == pytest.approx(0.5)
    assert lim.max_image_px == 1024


def test_gripper_closes_fully_within_its_own_small_limit_margin() -> None:
    cfg = McpServerConfig()
    assert cfg.limits.arm_limit_margin_rad == 0.05
    assert cfg.limits.arm_limit_margin_overrides == {"gripper": 0.005}
    assert cfg.limits.margin_for("gripper") == 0.005
    assert cfg.limits.margin_for("elbow_flex") == 0.05
    assert cfg.arm.gripper_closed_rad == -0.165
    assert cfg.limits.gripper_effort_ignore_s == 0.3
    assert cfg.limits.gripper_contact_travel_rad == 0.03


def test_default_timeouts() -> None:
    to = McpServerConfig().timeouts
    assert to.follower_stale_s == pytest.approx(0.3)
    assert to.image_max_age_s == pytest.approx(1.0)


def test_default_urdf_path_is_repo_relative_and_exists() -> None:
    cfg = McpServerConfig()
    assert cfg.arm.urdf_path == REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"
    assert cfg.arm.urdf_path.is_file()
    assert cfg.arm.home_file == Path("/var/lib/ros2/arm/home.yaml")


def test_relative_urdf_path_resolves_against_repo_root(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("arm:\n  urdf_path: nodes/web_ui/urdf/so101_arm.urdf\n")
    assert load_config(path).arm.urdf_path == REPO_ROOT / "nodes/web_ui/urdf/so101_arm.urdf"


def test_load_config_overrides(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("server:\n  port: 19000\n  path: /robot\nlimits:\n  max_linear_mps: 0.1\n")
    cfg = load_config(path)
    assert cfg.server.port == 19000
    assert cfg.server.path == "/robot"
    assert cfg.limits.max_linear_mps == pytest.approx(0.1)
    assert cfg.limits.max_angular_rps == pytest.approx(0.5)


def test_empty_file_gives_defaults(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("")
    assert load_config(path) == McpServerConfig()


def test_unknown_keys_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("servr:\n  port: 1\n")
    with pytest.raises(ValidationError):
        load_config(path)


def test_limits_cannot_exceed_hard_caps() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"max_linear_mps": 1.0}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_max_speed_scale": 0.9}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"max_image_px": 4096}})


def test_path_must_start_with_slash() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"server": {"path": "mcp"}})


def test_config_path_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MCP_SERVER_CONFIG", raising=False)
    assert config_path_from_env() == DEFAULT_CONFIG_PATH == Path("/etc/ros2/mcp_server/config.yaml")
    monkeypatch.setenv("MCP_SERVER_CONFIG", "/tmp/x.yaml")
    assert config_path_from_env() == Path("/tmp/x.yaml")


def test_token_from_env_refuses_missing_or_blank(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MCP_SERVER_TOKEN", raising=False)
    with pytest.raises(MissingTokenError):
        token_from_env()
    monkeypatch.setenv("MCP_SERVER_TOKEN", "   ")
    with pytest.raises(MissingTokenError):
        token_from_env()


def test_token_from_env_refuses_short_tokens(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MCP_SERVER_TOKEN", "short")
    with pytest.raises(MissingTokenError):
        token_from_env()


def test_token_from_env_strips(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MCP_SERVER_TOKEN", "  " + "a" * 48 + "\n")
    assert token_from_env() == "a" * 48


def test_arm_base_height_defaults_to_the_measured_10_4_cm_mount_and_gives_floor_z() -> None:
    arm = McpServerConfig().arm
    assert arm.arm_base_height_m == 0.100
    assert arm.floor_z_m == pytest.approx(-0.100)


def test_arm_mount_measured_on_the_robot_is_enabled_by_default() -> None:
    mount = McpServerConfig().arm.base_in_base_link
    assert mount is not None
    assert (mount.x, mount.y, mount.z, mount.yaw) == (0.0592, -0.05, 0.100, 0.0)


def test_arm_mount_height_must_match_arm_base_height() -> None:
    with pytest.raises(ValidationError, match="arm_base_height_m"):
        McpServerConfig.model_validate({"arm": {"base_in_base_link": {"x": 0.1, "z": 0.2}}})
    cfg = McpServerConfig.model_validate({"arm": {"arm_base_height_m": 0.2, "base_in_base_link": {"z": 0.2}}})
    assert cfg.arm.floor_z_m == pytest.approx(-0.2)
    assert McpServerConfig.model_validate({"arm": {"base_in_base_link": None}}).arm.base_in_base_link is None


def test_floor_guard_defaults_and_validation() -> None:
    guard = McpServerConfig().floor_guard
    assert guard.enabled is True
    assert guard.margin_m == 0.02
    assert guard.slow_speed_scale == 0.2
    assert guard.surface_z_m == 0.0
    assert guard.imu_max_age_s == 1.0
    for bad in ({"slow_speed_scale": 0.0}, {"slow_speed_scale": 1.5}, {"margin_m": -0.01}, {"imu_max_age_s": 0}):
        with pytest.raises(ValidationError):
            McpServerConfig.model_validate({"floor_guard": bad})


def test_grasp_defaults_and_validation() -> None:
    grasp = McpServerConfig().grasp
    assert grasp.interpolation_step_m == 0.005
    assert grasp.max_object_width_m == 0.08
    assert [entry.strategy for entry in grasp.auto_order] == ["top_down", "angled", "scoop"]
    assert grasp.auto_order[1].approach_pitch_deg == 45.0
    assert grasp.scoop_gap_margin_m == pytest.approx(0.004)
    assert grasp.tall_ratio == pytest.approx(1.5)
    assert grasp.tall_grasp_height_fraction == pytest.approx(0.3)
    assert 0.0 < grasp.lift_speed_scale < grasp.slide_speed_scale
    assert grasp.min_object_width_m == pytest.approx(0.01)
    assert grasp.scoop_max_pitch_deg == pytest.approx(40.0)  # sim matrix: 9/9 gap scoops lift at <= 40 deg
    assert 0.0 < grasp.slide_speed_scale <= 0.5
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grasp": {"slide_speed_scale": 0.9}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grasp": {"auto_order": [{"strategy": "teleport"}]}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grasp": {"interpolation_step_m": 0.0}})


def test_arm_base_height_must_be_positive() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"arm_base_height_m": 0.0}})


def test_nav_goal_tolerance_defaults_and_validation() -> None:
    nav = McpServerConfig().nav
    assert nav.goal_xy_tolerance_m == 0.01 and nav.goal_yaw_tolerance_deg == 2.0
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"nav": {"goal_xy_tolerance_m": 0.0}})


def test_settle_tolerance_defaults_between_converge_tolerance_and_tracking_abort() -> None:
    lim = McpServerConfig().limits
    assert lim.arm_settle_tolerance_rad == pytest.approx(0.08)
    assert (
        lim.arm_tracking_error_rad == pytest.approx(0.25) and lim.arm_tracking_lag_s == 0.0
    )  # the delay carries the lag
    assert lim.arm_tracking_delay_s == pytest.approx(0.35)
    assert lim.arm_converge_tolerance_rad < lim.arm_settle_tolerance_rad < lim.arm_tracking_error_rad


def test_settle_tolerance_must_stay_below_tracking_abort() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.4}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.01}})


def test_settle_hold_and_grasp_squeeze_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.limits.arm_settle_hold_s == pytest.approx(2.0)
    assert cfg.grip_profiles.default.squeeze_rad == pytest.approx(0.03)


def test_perception_config_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.local_costmap == "/local_costmap/costmap" and cfg.topics.plan == "/plan"
    assert (cfg.topics.poi_list, cfg.topics.poi_command, cfg.topics.poi_result) == (
        "/poi/list",
        "/poi/command",
        "/poi/result",
    )
    assert cfg.objects.store_path == Path("/var/lib/ros2/objects/objects.json")
    assert cfg.objects.merge_radius_m == 0.25
    assert (cfg.footprint.length_m, cfg.footprint.width_m) == (0.47, 0.386)
    assert cfg.topdown.default_radius_m == 2.5 and cfg.topdown.default_px == 480
    assert cfg.look_around.default_captures == 4 and cfg.look_around.clearance_margin_m > 0
    assert cfg.poi.request_timeout_s == 3.0


def test_perception_config_rejects_unknown_keys_and_bad_values(tmp_path: Path) -> None:
    for text in ("objects:\n  nope: 1\n", "look_around:\n  default_captures: 2\n", "poi:\n  request_timeout_s: 0\n"):
        path = tmp_path / "c.yaml"
        path.write_text(text)
        with pytest.raises(ValidationError):
            load_config(path)


def test_deployed_ansible_config_validates_against_the_model() -> None:
    nodes = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "mcp_server")
    cfg = McpServerConfig.model_validate(yaml.safe_load(entry["config"]))
    assert cfg.objects.store_path == Path("/var/lib/ros2/objects/objects.json")
    assert cfg.topics.poi_command == "/poi/command"


def test_joint_offsets_default_to_zero_and_reject_unknown_joints() -> None:
    offsets = McpServerConfig().arm.joint_offsets_rad
    assert offsets.model_dump() == dict.fromkeys(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"), 0.0
    )
    cfg = McpServerConfig.model_validate({"arm": {"joint_offsets_rad": {"wrist_flex": 0.2}}})
    assert cfg.arm.joint_offsets_rad.wrist_flex == 0.2
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"joint_offsets_rad": {"gripper": 0.2}}})


def test_tool_offset_defaults_to_zero_and_rejects_unknown_axes() -> None:
    assert McpServerConfig().arm.tool_offset_m.model_dump() == {"x": 0.0, "y": 0.0, "z": 0.0}
    cfg = McpServerConfig.model_validate({"arm": {"tool_offset_m": {"x": 0.01, "y": -0.028, "z": -0.002}}})
    assert cfg.arm.tool_offset_m.y == -0.028
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"tool_offset_m": {"w": 0.2}}})


def test_arm_velocity_default_is_one_rad_per_second_with_a_hard_cap() -> None:
    assert McpServerConfig().limits.arm_max_joint_velocity_rps == pytest.approx(1.0)
    assert McpServerConfig().limits.gripper_velocity_rps == pytest.approx(0.5)
    assert McpServerConfig.model_validate({"limits": {"arm_max_joint_velocity_rps": 1.5}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_max_joint_velocity_rps": 1.6}})


def test_roll_guard_defaults_and_validation() -> None:
    lim = McpServerConfig().limits
    assert lim.roll_guard_min_change_rad == pytest.approx(0.1)
    assert lim.roll_max_gripper_open_rad == pytest.approx(0.8)
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"roll_guard_min_change_rad": 0.0}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"roll_max_gripper_open_rad": -0.1}})


def test_jaw_open_axis_defaults_and_is_normalised() -> None:
    assert McpServerConfig().arm.jaw_open_axis == pytest.approx((-1.0, 0.0, 0.0))
    cfg = McpServerConfig.model_validate({"arm": {"jaw_open_axis": [0.0, -3.0, 4.0]}})
    assert cfg.arm.jaw_open_axis == pytest.approx((0.0, -0.6, 0.8))
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"jaw_open_axis": [0.0, 0.0, 0.0]}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"jaw_open_axis": [1.0, 0.0]}})


def test_joint_limit_overrides_default_empty_and_are_validated() -> None:
    assert McpServerConfig().arm.joint_limit_overrides_rad == {}
    cfg = McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"shoulder_lift": [-1.7, 2.6]}}})
    assert cfg.arm.joint_limit_overrides_rad == {"shoulder_lift": (-1.7, 2.6)}
    assert McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"gripper": [-0.2, 1.8]}}})
    with pytest.raises(ValidationError, match="unknown joint"):
        McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"elbow": [-1.0, 1.0]}}})
    with pytest.raises(ValidationError, match="lower"):
        McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"elbow_flex": [1.0, -1.0]}}})


def test_grasp_service_topics_default() -> None:
    topics = McpServerConfig().topics
    assert topics.grasp_command == "/grasp/command"
    assert topics.grasp_result == "/grasp/result"


def test_nav_intermediate_tolerances_are_looser_than_the_precise_ones() -> None:
    nav = McpServerConfig().nav
    assert (nav.intermediate_xy_tolerance_m, nav.intermediate_yaw_tolerance_deg) == (0.03, 5.0)
    assert (nav.goal_xy_tolerance_m, nav.goal_yaw_tolerance_deg) == (0.01, 2.0)
    with pytest.raises(ValidationError):
        McpServerConfig(nav={"intermediate_xy_tolerance_m": 0.005})
    with pytest.raises(ValidationError):
        McpServerConfig(nav={"intermediate_yaw_tolerance_deg": 1.0})


def swerve_level_reference_defaults() -> tuple[float, float]:
    """Code defaults of the swerve tilt guard level reference (read from its source: another node's venv)."""
    source = (REPO_ROOT / "nodes" / "swerve_drive_controller" / "swerve_drive_controller" / "tilt_guard.py").read_text()
    roll = re.search(r"^DEFAULT_LEVEL_REFERENCE_ROLL_DEG = ([-0-9.]+)$", source, re.MULTILINE)
    pitch = re.search(r"^DEFAULT_LEVEL_REFERENCE_PITCH_DEG = ([-0-9.]+)$", source, re.MULTILINE)
    assert roll is not None and pitch is not None
    return float(roll.group(1)), float(pitch.group(1))


def test_imu_level_reference_code_default_matches_the_swerve_tilt_guard() -> None:
    """Both nodes subtract the same BNO055 mounting bias; equal code defaults keep them equal without Ansible too."""
    reference = McpServerConfig().floor_guard.imu_level_reference
    assert (reference.roll_deg, reference.pitch_deg) == swerve_level_reference_defaults()


def test_imu_level_reference_is_validated_and_the_deployed_config_carries_the_mounting_bias() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"floor_guard": {"imu_level_reference": {"yaw_deg": 1.0}}})
    nodes = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == "mcp_server")
    reference = McpServerConfig.model_validate(yaml.safe_load(entry["config"])).floor_guard.imu_level_reference
    assert (reference.roll_deg, reference.pitch_deg) == (0.7, 1.3)  # 2026-10-11 investigation (BNO055 bias)


def test_steer_states_lazy_subscription_defaults_and_bounds() -> None:
    """control.steer_states_wait_s (first-message wait) and steer_states_idle_s (destroy after idle), review M5."""
    control = McpServerConfig().control
    assert (control.steer_states_wait_s, control.steer_states_idle_s) == (0.5, 30.0)
    for bad in ({"steer_states_wait_s": 0.0}, {"steer_states_idle_s": 0.0}):
        with pytest.raises(ValidationError):
            McpServerConfig.model_validate({"control": bad})


def test_guard_rules_default_is_enabled_without_rules() -> None:
    guard = McpServerConfig().guard_rules
    assert guard.enabled is True
    assert guard.rules == []


def test_guard_rules_load_from_yaml(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    rule = {
        "name": "base_motion_carrying_without_object_footprint",
        "mode": "dry_run",
        "message": "release_control keep_position=true carried_object_radius_m=... first",
        "applies_to": {"tools": ["navigate_to_pose"], "steps": ["base_relative"]},
        "all": [
            {"field": "arm.carrying", "op": "eq", "value": True},
            {"field": "footprint.carried_radius_m", "op": "le", "value": 0.0},
        ],
    }
    path.write_text(yaml.safe_dump({"guard_rules": {"rules": [rule]}}))
    loaded = load_config(path).guard_rules.rules[0]
    assert (loaded.name, loaded.mode, loaded.when_unknown) == (rule["name"], "dry_run", "reject")
    assert [c.field for c in loaded.all_of] == ["arm.carrying", "footprint.carried_radius_m"]


@pytest.mark.parametrize(
    "bad",
    [
        {"name": "never_gated_rule", "applies_to": {"tools": ["stop"]}},
        {"name": "unknown_field_rule", "all": [{"field": "arm.speed", "op": "gt", "value": 1.0}]},
    ],
)
def test_invalid_guard_rule_fails_config_load(tmp_path: Path, bad: dict) -> None:
    rule = {
        "message": "m",
        "applies_to": {"tools": ["drive"]},
        "all": [{"field": "battery.cutoff", "op": "eq", "value": True}],
    } | bad
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump({"guard_rules": {"rules": [rule]}}))
    with pytest.raises(ValueError, match="guard_rules"):
        load_config(path)


def test_motion_queue_watch_settings_defaults_and_bounds() -> None:
    settings = MotionQueueSettings()
    assert settings.watch_rate_hz == 10.0 and settings.watch_max_per_call == 8
    MotionQueueSettings(watch_rate_hz=20.0, watch_max_per_call=16)
    MotionQueueSettings(watch_max_per_call=1)
    for bad in ({"watch_rate_hz": 0.0}, {"watch_rate_hz": 20.5}, {"watch_max_per_call": 0}, {"watch_max_per_call": 17}):
        with pytest.raises(ValidationError):
            MotionQueueSettings(**bad)


def test_frontier_settings_defaults_match_the_design_table() -> None:
    f = McpServerConfig().frontiers
    assert (f.min_length_m, f.map_max_age_s, f.geodesic_downsample) == (0.30, 60.0, 2)
    assert (f.pass_clearance_margin_m, f.view_clearance_margin_m) == (0.03, 0.05)
    assert (f.standoff_min_m, f.standoff_max_m, f.los_candidates) == (0.4, 1.2, 5)
    assert (f.match_radius_m, f.epoch_reset_known_fraction, f.retired_keep, f.max_listed) == (0.6, 0.5, 50, 15)
    assert (f.poi_radius_m, f.facing_half_angle_deg, f.facing_max_distance_m) == (2.0, 30.0, 6.0)
    assert (f.compute_path_timeout_s, f.look_timeout_s, f.look_camera) == (5.0, 20.0, "front")


def test_compute_path_topics_default_to_the_nav2_planner_server() -> None:
    t = McpServerConfig().topics
    assert (t.compute_path_action, t.planner_id) == ("/compute_path_to_pose", "GridBased")


@pytest.mark.parametrize(
    "bad",
    [
        {"standoff_min_m": 1.2, "standoff_max_m": 1.2},
        {"standoff_min_m": 1.5, "standoff_max_m": 1.0},
        {"geodesic_downsample": 0},
        {"min_length_m": 0.0},
        {"epoch_reset_known_fraction": 1.5},
        {"los_candidates": 0},
        {"look_camera": "rear"},
        {"unknown_key": 1},
    ],
)
def test_frontier_settings_reject_invalid_values(bad: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"frontiers": bad})


def test_frontier_settings_accept_a_valid_override() -> None:
    f = McpServerConfig.model_validate(
        {
            "frontiers": {
                "standoff_min_m": 0.5,
                "standoff_max_m": 2.0,
                "geodesic_downsample": 3,
                "look_camera": "gripper",
            }
        }
    ).frontiers
    assert (f.standoff_min_m, f.standoff_max_m, f.geodesic_downsample, f.look_camera) == (0.5, 2.0, 3, "gripper")


def test_follower_joint_currents_topic_default() -> None:
    """E1: Present_Current of the gripper from the feetech bridge (sensor_msgs/JointState, effort in A)."""
    assert McpServerConfig().topics.follower_joint_currents == "/follower/joint_currents"
    cfg = McpServerConfig.model_validate({"topics": {"follower_joint_currents": "/x/joint_currents"}})
    assert cfg.topics.follower_joint_currents == "/x/joint_currents"


def test_contact_delta_settings_defaults_and_bounds() -> None:
    """E1 (arXiv 2608.23983): baseline over N_warm=10 samples, contact persists N_persist=3, current off by default."""
    gp = McpServerConfig().grip_profiles
    assert (gp.contact_baseline_samples, gp.contact_persist_samples, gp.current_contact_delta_a) == (10, 3, None)
    ok = McpServerConfig.model_validate(
        {
            "grip_profiles": {
                "contact_baseline_samples": 3,
                "contact_persist_samples": 1,
                "current_contact_delta_a": 0.05,
            }
        }
    ).grip_profiles
    assert (ok.contact_baseline_samples, ok.contact_persist_samples, ok.current_contact_delta_a) == (3, 1, 0.05)
    for bad in (
        {"contact_baseline_samples": 2},
        {"contact_persist_samples": 0},
        {"current_contact_delta_a": 0.0},
        {"current_contact_delta_a": -0.1},
    ):
        with pytest.raises(ValidationError):
            McpServerConfig.model_validate({"grip_profiles": bad})


def test_grip_profile_contact_load_delta_defaults_off_and_must_be_positive() -> None:
    """Delta contact is off by default: empty closes on the robot rose 35-124 above the free-closing baseline."""
    gp = McpServerConfig().grip_profiles
    assert all(p.contact_load_delta is None for p in gp.presets.values())
    preset = {"squeeze_rad": 0.03, "torque_limit": 500, "close_speed_rps": 0.5, "contact_effort_threshold": 300.0}
    cfg = McpServerConfig.model_validate(
        {"grip_profiles": {"presets": {"normal": {**preset, "contact_load_delta": 20.0}}}}
    )
    assert cfg.grip_profiles.presets["normal"].contact_load_delta == 20.0
    for bad in (0.0, -5.0):
        with pytest.raises(ValidationError):
            McpServerConfig.model_validate(
                {"grip_profiles": {"presets": {"normal": {**preset, "contact_load_delta": bad}}}}
            )


def test_grip_profile_override_accepts_contact_load_delta() -> None:
    assert GripProfileOverride().contact_load_delta is None
    assert GripProfileOverride.model_validate({"contact_load_delta": 25.0}).contact_load_delta == 25.0
    with pytest.raises(ValidationError):
        GripProfileOverride.model_validate({"contact_load_delta": 0.0})
    cfg = McpServerConfig.model_validate({"grasp": {"grip_profile": {"base": "soft", "contact_load_delta": 15.0}}})
    assert isinstance(cfg.grasp.grip_profile, GripProfileOverride)
    assert cfg.grasp.grip_profile.contact_load_delta == 15.0


def test_grasp_check_settings_defaults_and_validation() -> None:
    gc = McpServerConfig().grasp_check
    assert (gc.crop_depth_m, gc.crop_margin_px, gc.crop_min_px, gc.image_px) == (0.045, 24, 64, 640)
    for bad in ({"crop_depth_m": -0.01}, {"crop_margin_px": -1}, {"crop_min_px": 0}, {"image_px": 0}):
        with pytest.raises(ValidationError):
            McpServerConfig.model_validate({"grasp_check": bad})


@pytest.mark.parametrize(
    "data",
    [
        {"grasp_check": {"crop_depth": 0.05}},
        {"grip_profiles": {"contact_baseline": 10}},
        {"grasp": {"grip_profile": {"contact_delta": 20.0}}},
        {"topics": {"follower_currents": "/x"}},
    ],
)
def test_new_e1_sections_reject_unknown_keys(data: dict) -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate(data)


def test_contact_load_delta_appears_in_the_resolved_grip_report() -> None:
    cfg = McpServerConfig()
    assert resolve_grip_profile(cfg.grip_profiles, cfg.limits, "firm").report()["contact_load_delta"] is None
    custom = resolve_grip_profile(cfg.grip_profiles, cfg.limits, {"base": "firm", "contact_load_delta": 20.0})
    assert custom.report()["contact_load_delta"] == 20.0
    assert custom.name == "firm+custom"


def test_zoom_config_defaults_and_validation(tmp_path: Path) -> None:
    cfg = McpServerConfig()
    assert cfg.topics.head_camera_zoom_command == "/head_camera/zoom/command"
    assert cfg.topics.head_camera_zoom_result == "/head_camera/zoom/result"
    assert cfg.topics.head_camera_zoom_image == "/head_camera/zoom/image"
    assert cfg.timeouts.zoom_timeout_s == 8.0 and cfg.timeouts.zoom_image_grace_s == 1.0
    for key in ("zoom_timeout_s", "zoom_image_grace_s"):
        path = tmp_path / "c.yaml"
        path.write_text(f"timeouts:\n  {key}: 0\n")
        with pytest.raises(ValidationError):
            load_config(path)


def test_head_defaults_and_home_validation() -> None:
    head = McpServerConfig().head
    assert head.urdf_path == REPO_ROOT / "nodes/web_ui/urdf/head.urdf"
    assert (head.home_pan_rad, head.home_tilt_rad, head.tolerance_rad) == (0.0, 0.5236, 0.03)
    with pytest.raises(ValidationError, match="head_tilt"):
        McpServerConfig.model_validate({"head": {"home_tilt_rad": 1.2}})
