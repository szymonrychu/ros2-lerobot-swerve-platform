"""The executor key picks the rclpy executor of the bridge: single_threaded (default) or events."""

import importlib
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, external_shutdown, install_ros_stubs
from .test_shutdown import make_config, signal_during_spin

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

BASE_YAML = "namespace: base\njoint_names:\n  - {name: wheel, id: 2}\n"


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import feetech_servos.bridge with both executors, the node and the metrics server mocked.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched bridge module.
    """
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    monkeypatch.setattr(module, "EventsExecutor", MagicMock())
    return module


def load(tmp_path: Path, extra: str) -> object:
    """Load a minimal bridge YAML plus extra top-level lines.

    Args:
        tmp_path (Path): Test directory.
        extra (str): Extra YAML.

    Returns:
        object: BridgeConfig or None.
    """
    path = tmp_path / "c.yaml"
    path.write_text(BASE_YAML + extra)
    return importlib.import_module("feetech_servos.config").load_config(path)


def test_executor_defaults_to_single_threaded(tmp_path: Path) -> None:
    assert load(tmp_path, "").executor == "single_threaded"


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_executor_value_parsed(tmp_path: Path, kind: str) -> None:
    assert load(tmp_path, f"executor: {kind}\n").executor == kind


@pytest.mark.parametrize("value", ["multi", "''", "3", "Events"])
def test_unknown_executor_is_rejected(tmp_path: Path, value: str) -> None:
    assert load(tmp_path, f"executor: {value}\n") is None


@pytest.mark.parametrize(("kind", "cls"), [("events", "EventsExecutor"), ("single_threaded", "SingleThreadedExecutor")])
def test_make_executor_builds_the_configured_kind(bridge_module: ModuleType, kind: str, cls: str) -> None:
    assert bridge_module.make_executor(kind) is getattr(bridge_module, cls).return_value


def test_make_executor_unknown_raises(bridge_module: ModuleType) -> None:
    with pytest.raises(ValueError):
        bridge_module.make_executor("multi")


@pytest.mark.parametrize(
    ("kind", "used", "unused"),
    [
        ("events", "EventsExecutor", "SingleThreadedExecutor"),
        ("single_threaded", "SingleThreadedExecutor", "EventsExecutor"),
    ],
)
def test_run_spins_the_configured_executor(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch, kind: str, used: str, unused: str
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    executor = getattr(bridge_module, used).return_value
    executor.spin_once.side_effect = signal_during_spin(fake, external_shutdown())
    config = make_config(bridge_module, None)
    config.executor = kind
    bridge_module.run_bridge(config)
    executor.add_node.assert_called_once_with(bridge_module.Node.return_value)
    executor.spin_once.assert_called_with(timeout_sec=0.0)
    getattr(bridge_module, unused).assert_not_called()
    assert "try_shutdown" in fake.calls
