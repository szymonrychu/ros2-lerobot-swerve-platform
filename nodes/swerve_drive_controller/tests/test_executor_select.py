"""The `executor` config key selects the rclpy executor the swerve controller spins on."""

import importlib
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from swerve_drive_controller.config import load_config

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs(
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "tf2_ros",
)


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import the node module with ROS stubbed, the executor classes replaced by MagicMocks and metrics silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module.
    """
    module = importlib.import_module("swerve_drive_controller.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "EventsExecutor", MagicMock(name="EventsExecutor"))
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock(name="SingleThreadedExecutor"))
    monkeypatch.setattr(module, "rclpy", FakeRclpy())
    return module


def write_config(tmp_path: Path, extra: str) -> Path:
    path = tmp_path / "config.yaml"
    path.write_text(extra)
    return path


def test_executor_defaults_to_single_threaded(tmp_path: Path) -> None:
    config = load_config(write_config(tmp_path, "half_length_m: 0.15\n"))
    assert config is not None
    assert config.executor == "single_threaded"


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_executor_value_is_loaded(tmp_path: Path, kind: str) -> None:
    config = load_config(write_config(tmp_path, f"executor: {kind}\n"))
    assert config is not None
    assert config.executor == kind


@pytest.mark.parametrize("raw", ["multi", "''", "3", "Events"])
def test_unknown_executor_is_a_config_error(tmp_path: Path, raw: str) -> None:
    assert load_config(write_config(tmp_path, f"executor: {raw}\n")) is None


def test_make_executor_events(node_module: ModuleType) -> None:
    assert node_module.make_executor("events") is node_module.EventsExecutor.return_value


def test_make_executor_single_threaded(node_module: ModuleType) -> None:
    assert node_module.make_executor("single_threaded") is node_module.SingleThreadedExecutor.return_value


def test_make_executor_rejects_unknown(node_module: ModuleType) -> None:
    with pytest.raises(ValueError):
        node_module.make_executor("multi")


@pytest.mark.parametrize(("kind", "cls"), [("events", "EventsExecutor"), ("single_threaded", "SingleThreadedExecutor")])
def test_run_spins_the_selected_executor(node_module: ModuleType, tmp_path: Path, kind: str, cls: str) -> None:
    config = load_config(write_config(tmp_path, f"executor: {kind}\nmetrics_port: 9\n"))
    assert config is not None

    node_module.run_swerve_controller(config)

    executor = getattr(node_module, cls).return_value
    executor.add_node.assert_called_once_with(node_module.Node.return_value)
    executor.spin.assert_called_once_with()
    node_module.Node.return_value.destroy_node.assert_called_once()
