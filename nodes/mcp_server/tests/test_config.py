"""Tests for mcp_server.config (pydantic YAML config, env lookup, bearer token)."""

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mcp_server.config import (
    DEFAULT_CONFIG_PATH,
    REPO_ROOT,
    McpServerConfig,
    MissingTokenError,
    config_path_from_env,
    load_config,
    token_from_env,
)


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
    assert t.front_camera == "/overview_camera/image_raw/compressed"
    assert t.navigate_action == "/navigate_to_pose"


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


def test_arm_base_height_defaults_to_the_15_cm_mount_estimate_and_gives_floor_z() -> None:
    arm = McpServerConfig().arm
    assert arm.arm_base_height_m == 0.15
    assert arm.floor_z_m == pytest.approx(-0.15)


def test_arm_mount_estimate_is_enabled_by_default() -> None:
    mount = McpServerConfig().arm.base_in_base_link
    assert mount is not None
    assert (mount.x, mount.y, mount.z, mount.yaw) == (0.15, -0.04, 0.15, 0.0)


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
    assert [entry.strategy for entry in grasp.auto_order] == ["scoop", "angled", "top_down"]
    assert grasp.auto_order[1].approach_pitch_deg == 45.0
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
    assert lim.arm_tracking_error_rad == pytest.approx(0.25) and lim.arm_tracking_lag_s == pytest.approx(0.25)
    assert lim.arm_converge_tolerance_rad < lim.arm_settle_tolerance_rad < lim.arm_tracking_error_rad


def test_settle_tolerance_must_stay_below_tracking_abort() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.4}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.01}})


def test_settle_hold_and_grasp_squeeze_defaults() -> None:
    lim = McpServerConfig().limits
    assert lim.arm_settle_hold_s == pytest.approx(2.0)
    assert lim.gripper_grasp_squeeze_rad == pytest.approx(0.03)


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
