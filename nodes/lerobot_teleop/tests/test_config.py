"""Unit tests for teleop config (env-based)."""

import pytest

from config import DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC, DEFAULT_LEADER_JOINT_STATES_TOPIC, get_config


def test_get_config_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    """With no env set, get_config returns default leader and follower topics."""
    monkeypatch.delenv("TELEOP_LEADER_JOINT_STATES_TOPIC", raising=False)
    monkeypatch.delenv("TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC", raising=False)
    leader, follower = get_config()
    assert leader == DEFAULT_LEADER_JOINT_STATES_TOPIC
    assert follower == DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC


def test_get_config_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """get_config reads topic env vars."""
    monkeypatch.setenv("TELEOP_LEADER_JOINT_STATES_TOPIC", "/custom/leader/states")
    monkeypatch.setenv("TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC", "/custom/follower/cmds")
    leader, follower = get_config()
    assert leader == "/custom/leader/states"
    assert follower == "/custom/follower/cmds"


def test_get_config_empty_env_falls_back_to_default(monkeypatch: pytest.MonkeyPatch) -> None:
    """Empty or whitespace env values fall back to defaults."""
    monkeypatch.setenv("TELEOP_LEADER_JOINT_STATES_TOPIC", "")
    monkeypatch.setenv("TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC", "   ")
    leader, follower = get_config()
    assert leader == DEFAULT_LEADER_JOINT_STATES_TOPIC
    assert follower == DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC
