"""Tests for the ``executor`` config switch and ``make_executor`` in the node module."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from marker_detector.config import MarkerDetectorConfig, load_config
from tests.ros_stubs import install_ros_stubs


def test_executor_defaults_to_single_threaded(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("")
    assert load_config(path).executor == "single_threaded"
    assert MarkerDetectorConfig().executor == "single_threaded"


def test_executor_accepts_events() -> None:
    assert MarkerDetectorConfig(executor="events").executor == "events"


def test_unknown_executor_is_rejected() -> None:
    with pytest.raises(ValidationError):
        MarkerDetectorConfig(executor="multi")


def test_make_executor_builds_the_configured_kind(monkeypatch: pytest.MonkeyPatch) -> None:
    install_ros_stubs()
    import marker_detector.node as node_module

    events, single = MagicMock(name="events"), MagicMock(name="single")
    monkeypatch.setattr(node_module, "EventsExecutor", events)
    monkeypatch.setattr(node_module, "SingleThreadedExecutor", single)
    assert node_module.make_executor("events") is events.return_value
    assert node_module.make_executor("single_threaded") is single.return_value
    with pytest.raises(ValueError):
        node_module.make_executor("bogus")


def test_run_spins_the_configured_executor(monkeypatch: pytest.MonkeyPatch) -> None:
    install_ros_stubs()
    import marker_detector.node as node_module

    executor = MagicMock(name="executor")
    make = MagicMock(return_value=executor)
    rclpy = MagicMock()
    monkeypatch.setattr(node_module, "make_executor", make)
    monkeypatch.setattr(node_module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(node_module, "MarkerDetectorRos", MagicMock())
    monkeypatch.setattr(node_module, "MarkerPipeline", MagicMock())
    monkeypatch.setattr(node_module, "rclpy", rclpy)
    node_module.run_node(MarkerDetectorConfig(executor="events"))
    make.assert_called_once_with("events")
    executor.add_node.assert_called_once()
    executor.spin.assert_called_once_with()
    rclpy.spin.assert_not_called()
