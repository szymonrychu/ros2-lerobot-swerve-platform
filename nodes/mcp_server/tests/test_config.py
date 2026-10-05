"""Tests for mcp_server.config (pydantic YAML config, env lookup, bearer token)."""

from pathlib import Path

import pytest
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
    assert t.realsense_camera == "/camera/camera/color/image_raw"
    assert t.navigate_action == "/navigate_to_pose"


def test_default_limits_are_conservative() -> None:
    lim = McpServerConfig().limits
    assert lim.max_linear_mps == pytest.approx(0.25)
    assert lim.max_angular_rps == pytest.approx(0.5)
    assert lim.max_drive_duration_s == pytest.approx(2.0)
    assert lim.drive_rate_hz == pytest.approx(20.0)
    assert lim.arm_rate_hz == pytest.approx(25.0)
    assert lim.arm_max_joint_velocity_rps == pytest.approx(0.5)
    assert lim.arm_max_speed_scale == pytest.approx(0.5)
    assert lim.max_image_px == 1024


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


def test_arm_base_height_defaults_to_measured_16_5_cm_and_gives_floor_z() -> None:
    arm = McpServerConfig().arm
    assert arm.arm_base_height_m == 0.165
    assert arm.floor_z_m == pytest.approx(-0.165)


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
    assert lim.arm_converge_tolerance_rad < lim.arm_settle_tolerance_rad < lim.arm_tracking_error_rad


def test_settle_tolerance_must_stay_below_tracking_abort() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.4}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"limits": {"arm_settle_tolerance_rad": 0.01}})


def test_settle_hold_and_grasp_squeeze_defaults() -> None:
    lim = McpServerConfig().limits
    assert lim.arm_settle_hold_s == pytest.approx(2.0)
