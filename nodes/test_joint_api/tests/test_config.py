"""Unit tests for test_joint_api config."""

from pathlib import Path

import pytest

from test_joint_api.config import load_config, load_config_from_env


def test_load_config_nonexistent() -> None:
    """load_config returns None for missing file."""
    assert load_config(Path("/nonexistent/api.yaml")) is None


def test_load_config_defaults(tmp_path: Path) -> None:
    """load_config uses default host, port, topic."""
    p = tmp_path / "api.yaml"
    p.write_text("host: 0.0.0.0\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.host == "0.0.0.0"
    assert cfg.port == 8080
    assert cfg.topic == "/filter/input_joint_updates"


def test_load_config_explicit(tmp_path: Path) -> None:
    """load_config parses host, port, topic."""
    p = tmp_path / "api.yaml"
    p.write_text("host: 127.0.0.1\nport: 9090\ntopic: /custom/input\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.host == "127.0.0.1"
    assert cfg.port == 9090
    assert cfg.topic == "/custom/input"


def test_load_config_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env uses TEST_JOINT_API_CONFIG path."""
    config_file = tmp_path / "env.yaml"
    config_file.write_text("port: 8888\n")
    monkeypatch.setenv("TEST_JOINT_API_CONFIG", str(config_file))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.port == 8888
