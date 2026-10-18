"""Several bridges on one host: node_name picks the ROS node and metrics labels, battery interval 0 publishes nothing."""

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
    """Import feetech_servos.bridge with the metrics server, node and executor mocked (as test_shutdown does).

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched bridge module.
    """
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    return module


def write_config(tmp_path: Path, extra: str) -> Path:
    """Write a minimal bridge YAML plus extra top-level lines.

    Args:
        tmp_path (Path): Test directory.
        extra (str): Extra YAML appended to the base config.

    Returns:
        Path: The config file path.
    """
    path = tmp_path / "c.yaml"
    path.write_text(BASE_YAML + extra)
    return path


def load(path: Path) -> object:
    """Call load_config on a path.

    Args:
        path (Path): YAML path.

    Returns:
        object: BridgeConfig or None.
    """
    return importlib.import_module("feetech_servos.config").load_config(path)


def test_node_name_parsed_and_stripped(tmp_path: Path) -> None:
    assert load(write_config(tmp_path, "node_name: '  base_servos '\n")).node_name == "base_servos"


@pytest.mark.parametrize("extra", ["", "node_name: ''\n", "node_name: '   '\n"])
def test_node_name_defaults_to_none(tmp_path: Path, extra: str) -> None:
    assert load(write_config(tmp_path, extra)).node_name is None


@pytest.mark.parametrize("name", ["9bus", "a/b", "a-b"])
def test_invalid_node_name_rejects_config(tmp_path: Path, name: str) -> None:
    assert load(write_config(tmp_path, f"node_name: '{name}'\n")) is None


def run_with_name(module: ModuleType, monkeypatch: pytest.MonkeyPatch, node_name: str | None) -> None:
    """Run the bridge (no servo bus) until a shutdown signal, with the given node_name.

    Args:
        module (ModuleType): Patched bridge module.
        monkeypatch (pytest.MonkeyPatch): Patches rclpy.
        node_name (str | None): Config node_name.
    """
    fake = FakeRclpy()
    monkeypatch.setattr(module, "rclpy", fake)
    module.SingleThreadedExecutor.return_value.spin_once.side_effect = signal_during_spin(fake, external_shutdown())
    config = make_config(module, None)
    config.node_name = node_name
    module.run_bridge(config)


def test_node_name_names_node_and_metrics(bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    run_with_name(bridge_module, monkeypatch, "base_servos")
    bridge_module.Node.assert_called_once_with("base_servos")
    assert bridge_module.start_metrics_server.call_args.args[1] == "base_servos"


def test_unset_node_name_keeps_defaults(bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    run_with_name(bridge_module, monkeypatch, None)
    bridge_module.Node.assert_called_once_with("feetech_servos_bridge")
    assert bridge_module.start_metrics_server.call_args.args[1] == "lerobot_follower"


def battery_publishers(module: ModuleType) -> list[object]:
    """create_publisher calls made with BatteryState.

    Args:
        module (ModuleType): Patched bridge module.

    Returns:
        list[object]: Matching calls.
    """
    calls = module.Node.return_value.create_publisher.call_args_list
    return [c for c in calls if c.args and c.args[0] is module.BatteryState]


def test_battery_interval_zero_creates_no_publisher(bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(bridge_module, "BatteryState", MagicMock(name="BatteryState"))
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    bridge_module.SingleThreadedExecutor.return_value.spin_once.side_effect = signal_during_spin(
        fake, external_shutdown()
    )
    config = make_config(bridge_module, None)
    config.battery_interval_s = 0.0
    bridge_module.run_bridge(config)
    assert battery_publishers(bridge_module) == []


def test_battery_interval_positive_creates_publisher(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(bridge_module, "BatteryState", MagicMock(name="BatteryState"))
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    bridge_module.SingleThreadedExecutor.return_value.spin_once.side_effect = signal_during_spin(
        fake, external_shutdown()
    )
    config = make_config(bridge_module, None)
    config.battery_interval_s = 1.0
    bridge_module.run_bridge(config)
    assert len(battery_publishers(bridge_module)) == 1
