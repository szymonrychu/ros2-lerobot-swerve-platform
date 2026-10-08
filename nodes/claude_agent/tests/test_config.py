"""Config validation, YAML loading and MCP token reading."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from claude_agent.config import (
    CONFIG_ENV_VAR,
    TURN_MARGIN,
    ClaudeAgentConfig,
    MissingTokenError,
    config_path_from_env,
    load_config,
    read_mcp_token,
)


def test_defaults() -> None:
    cfg = ClaudeAgentConfig()
    assert cfg.model == "opus"
    assert (cfg.max_rw_cap, cfg.max_turn_cap) == (150, 200)
    assert cfg.max_turns == cfg.max_turn_cap + TURN_MARGIN
    assert (cfg.max_phase_rw_cap, cfg.max_phase_turn_cap) == (40, 40)
    assert not hasattr(cfg, "max_ro_cap") and not hasattr(cfg, "max_phase_ro_cap")
    assert cfg.robot_events_topic == "/robot_events"
    assert cfg.poi_command_topic == "/poi/command"
    assert cfg.robot_events_history == 50
    assert cfg.robot_event_debounce_s == 2.0
    assert cfg.mcp_url == "http://127.0.0.1:18200/mcp"
    assert cfg.mcp_token_file == "/etc/ros2/mcp_server/token"
    assert cfg.http_port == 18300
    assert cfg.history_size == 500
    assert cfg.image_thumbnail_max_px == 480
    assert cfg.system_prompt_extra == ""
    assert {"drive", "move_relative", "navigate_to_pose", "move_arm_joints", "move_arm_cartesian"} <= set(
        cfg.effector_tools
    )
    assert {"set_gripper", "arm_home", "arm_set_home"} <= set(cfg.effector_tools)
    assert {"stop", "acquire_control", "release_control"} <= set(cfg.uncapped_tools)
    assert "look_around" in cfg.effector_tools
    assert {"get_body_state", "get_topdown_view", "list_pois", "add_poi", "remember_object", "pixel_to_ground"} <= set(
        cfg.sensor_tools
    )
    assert not set(cfg.effector_tools) & set(cfg.uncapped_tools)
    assert not set(cfg.effector_tools) & set(cfg.sensor_tools)


def test_persistence_and_hardware_defaults() -> None:
    cfg = ClaudeAgentConfig()
    assert cfg.workdir == "/var/lib/claude_agent/workspace"
    assert cfg.state_dir == "/var/lib/claude_agent"
    assert cfg.session_log_max_bytes == 50 * 1024 * 1024
    assert cfg.arm_base_height_m == 0.165
    assert 30 <= cfg.arm_reach_cm <= 50
    assert "left to right" in cfg.camera_note and "upright" in cfg.camera_note
    assert not hasattr(cfg, "work_dir")


def test_stop_cannot_be_an_effector() -> None:
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(effector_tools=["drive", "stop"])


@pytest.mark.parametrize(
    "field,value",
    [
        ("max_rw_cap", -1),
        ("max_turn_cap", 0),
        ("max_phase_rw_cap", 0),
        ("max_phase_turn_cap", 0),
        ("robot_events_history", 0),
        ("robot_event_debounce_s", -1),
        ("http_port", 70000),
        ("history_size", 0),
        ("image_thumbnail_max_px", 8),
    ],
)
def test_invalid_numbers_rejected(field: str, value: int) -> None:
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(**{field: value})


def test_unknown_key_rejected() -> None:
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(bogus=1)


def test_load_config_from_yaml(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("max_turn_cap: 7\nmax_rw_cap: 3\nmodel: sonnet\n")
    cfg = load_config(path)
    assert (cfg.max_turn_cap, cfg.max_rw_cap, cfg.model) == (7, 3, "sonnet")
    assert cfg.max_turns == 7 + TURN_MARGIN


@pytest.mark.parametrize("key", ["max_turns", "effector_call_cap"])
def test_removed_static_cap_keys_rejected(key: str) -> None:
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(**{key: 5})


def test_load_config_none_gives_defaults_and_empty_file_too(tmp_path: Path) -> None:
    assert load_config(None) == ClaudeAgentConfig()
    path = tmp_path / "empty.yaml"
    path.write_text("")
    assert load_config(path) == ClaudeAgentConfig()


def test_config_path_from_env() -> None:
    assert config_path_from_env({}) is None
    assert config_path_from_env({CONFIG_ENV_VAR: "/x/c.yaml"}) == Path("/x/c.yaml")
    assert CONFIG_ENV_VAR == "CLAUDE_AGENT_CONFIG"


def test_read_mcp_token_env_file_format_and_bare(tmp_path: Path) -> None:
    path = tmp_path / "token"
    path.write_text("MCP_SERVER_TOKEN=abc123\n")
    assert read_mcp_token(path) == "abc123"
    path.write_text("rawtoken\n")
    assert read_mcp_token(path) == "rawtoken"


def test_read_mcp_token_missing_or_empty(tmp_path: Path) -> None:
    with pytest.raises(MissingTokenError):
        read_mcp_token(tmp_path / "nope")
    path = tmp_path / "token"
    path.write_text("MCP_SERVER_TOKEN=\n")
    with pytest.raises(MissingTokenError):
        read_mcp_token(path)


def test_missing_token_error_never_contains_token(tmp_path: Path) -> None:
    path = tmp_path / "token"
    path.write_text("")
    with pytest.raises(MissingTokenError) as exc:
        read_mcp_token(path)
    assert str(path) in str(exc.value)


def test_watchdog_and_stop_timeouts_defaults_and_bounds() -> None:
    cfg = ClaudeAgentConfig()
    assert (cfg.instruction_timeout_s, cfg.connect_timeout_s, cfg.stop_timeout_s) == (900, 240, 5)
    for key in ("instruction_timeout_s", "connect_timeout_s", "stop_timeout_s"):
        with pytest.raises(ValidationError):
            ClaudeAgentConfig(**{key: 0})


def test_nav_goal_precision_defaults_and_validation() -> None:
    cfg = ClaudeAgentConfig()
    assert cfg.nav_goal_xy_tolerance_cm == 1.0 and cfg.nav_goal_yaw_tolerance_deg == 2.0
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(nav_goal_xy_tolerance_cm=0)
