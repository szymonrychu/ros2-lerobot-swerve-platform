"""executor config key and make_executor / run() executor wiring against ROS doubles."""

import importlib
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from laser_filter.config import LaserFilterConfig, load_config

DEPLOYED = Path(__file__).resolve().parents[1] / "config" / "footprint_filter.yaml"
POLYGON = "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
ROS_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.duration",
    "rclpy.executors",
    "rclpy.experimental",
    "rclpy.parameter",
    "rclpy.qos",
    "rclpy.task",
    "rclpy.time",
    "geometry_msgs",
    "geometry_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "sensor_msgs_py",
    "sensor_msgs_py.point_cloud2",
    "tf2_ros",
)


class FakeNodeBase:
    """rclpy.node.Node double that accepts anything the node constructor does."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        self.destroyed = False

    def __getattr__(self, name: str) -> MagicMock:
        return MagicMock(name=name)

    def destroy_node(self) -> None:
        self.destroyed = True


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """laser_filter.node imported against the ROS doubles."""
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, MagicMock(name=name))
    sys.modules["rclpy.node"].Node = FakeNodeBase
    sys.modules.pop("laser_filter.node", None)
    yield importlib.import_module("laser_filter.node")
    sys.modules.pop("laser_filter.node", None)


def test_executor_defaults_to_single_threaded() -> None:
    assert LaserFilterConfig(polygon=POLYGON).executor == "single_threaded"


def test_executor_events_accepted_and_deployed_config_valid() -> None:
    assert LaserFilterConfig(polygon=POLYGON, executor="events").executor == "events"
    assert load_config(DEPLOYED).executor in ("events", "single_threaded")


def test_executor_unknown_value_rejected() -> None:
    with pytest.raises(ValidationError):
        LaserFilterConfig(polygon=POLYGON, executor="multi")


def test_make_executor_picks_class(node_module: ModuleType) -> None:
    assert node_module.make_executor("events") is sys.modules["rclpy.experimental"].EventsExecutor.return_value
    assert node_module.make_executor("single_threaded") is sys.modules["rclpy.executors"].SingleThreadedExecutor.return_value
    with pytest.raises(ValueError):
        node_module.make_executor("bogus")


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_run_spins_selected_executor_not_global(node_module: ModuleType, kind: str) -> None:
    rclpy = sys.modules["rclpy"]
    executor = node_module.make_executor(kind)
    node_module.run(LaserFilterConfig(polygon=POLYGON, executor=kind))
    executor.add_node.assert_called_once()
    executor.spin.assert_called_once_with()
    rclpy.spin.assert_not_called()
    rclpy.try_shutdown.assert_called_once_with()


def test_run_keyboard_interrupt_still_shuts_down(node_module: ModuleType) -> None:
    rclpy = sys.modules["rclpy"]
    node_module.make_executor("single_threaded").spin.side_effect = KeyboardInterrupt
    node_module.run(LaserFilterConfig(polygon=POLYGON))
    rclpy.try_shutdown.assert_called_once_with()
