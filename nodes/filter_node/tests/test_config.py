"""Unit tests for filter node config loading."""

from pathlib import Path

import pytest

from filter_node.config import load_config, load_config_from_env


def test_load_config_nonexistent_path() -> None:
    """load_config returns None for missing file."""
    assert load_config(Path("/nonexistent/filter.yaml")) is None


def test_load_config_empty_file(tmp_path: Path) -> None:
    """load_config returns None for empty or invalid YAML."""
    p = tmp_path / "empty.yaml"
    p.write_text("")
    assert load_config(p) is None


def test_load_config_defaults(tmp_path: Path) -> None:
    """load_config uses default input/output topic and algorithm when minimal YAML."""
    p = tmp_path / "min.yaml"
    p.write_text("algorithm: kalman\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.input_topic == "/filter/input_joint_updates"
    assert cfg.output_topic == "/follower/joint_commands"
    assert cfg.algorithm == "kalman"
    assert cfg.control_loop_hz == 100.0
    assert cfg.joint_names == []


def test_load_config_explicit(tmp_path: Path) -> None:
    """load_config parses explicit topics, algorithm, params, joint_names."""
    p = tmp_path / "full.yaml"
    p.write_text(
        "input_topic: /in/joints\n"
        "output_topic: /out/joints\n"
        "algorithm: kalman\n"
        "control_loop_hz: 150\n"
        "joint_names:\n  - joint_1\n  - joint_2\n"
        "algorithm_params:\n  process_noise_pos: 0.0003\n  prediction_lead_s: 0.04\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.input_topic == "/in/joints"
    assert cfg.output_topic == "/out/joints"
    assert cfg.algorithm == "kalman"
    assert cfg.control_loop_hz == 150.0
    assert cfg.joint_names == ["joint_1", "joint_2"]
    assert cfg.algorithm_params.get("process_noise_pos") == 0.0003
    assert cfg.algorithm_params.get("prediction_lead_s") == 0.04


def test_load_config_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env uses FILTER_NODE_CONFIG path."""
    config_file = tmp_path / "env_config.yaml"
    config_file.write_text("input_topic: /custom/input\noutput_topic: /custom/output\n")
    monkeypatch.setenv("FILTER_NODE_CONFIG", str(config_file))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.input_topic == "/custom/input"
    assert cfg.output_topic == "/custom/output"
