"""The executor config key selects the rclpy executor; the 100 Hz loop spins it with the control period as timeout."""

import importlib
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from filter_node.config import FilterConfig, load_config

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

LOOP_ITERATIONS = 3


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import filter_node.node with ROS stubbed.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module.
    """
    module = importlib.import_module("filter_node.node")
    monkeypatch.setattr(module, "EventsExecutor", MagicMock(name="EventsExecutor"))
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock(name="SingleThreadedExecutor"))
    return module


def write_config(tmp_path: Path, extra: str = "") -> Path:
    path = tmp_path / "config.yaml"
    path.write_text(f"algorithm: kalman\n{extra}")
    return path


def test_executor_defaults_to_single_threaded(tmp_path: Path) -> None:
    config = load_config(write_config(tmp_path))
    assert config is not None
    assert config.executor == "single_threaded"


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_executor_key_is_loaded(tmp_path: Path, kind: str) -> None:
    config = load_config(write_config(tmp_path, f"executor: {kind}\n"))
    assert config is not None
    assert config.executor == kind


def test_unknown_executor_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="executor"):
        load_config(write_config(tmp_path, "executor: threaded\n"))


def test_make_executor_builds_the_configured_kind(node_module: ModuleType) -> None:
    assert node_module.make_executor("events") is node_module.EventsExecutor.return_value
    assert node_module.make_executor("single_threaded") is node_module.SingleThreadedExecutor.return_value


def test_make_executor_rejects_unknown_kind(node_module: ModuleType) -> None:
    with pytest.raises(ValueError, match="executor"):
        node_module.make_executor("threaded")


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def run_loop(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, kind: str
) -> tuple[list[str], MagicMock, list[float]]:
    """Run serve_filter_node against a stub executor for LOOP_ITERATIONS spins.

    Returns:
        tuple[list[str], MagicMock, list[float]]: Executor kinds requested, the stub executor, spin_once timeouts.
    """
    fake = FakeRclpy()
    executor = MagicMock()
    timeouts: list[float] = []

    def spin_once(timeout_sec: float) -> None:
        timeouts.append(timeout_sec)
        fake.alive = len(timeouts) < LOOP_ITERATIONS

    executor.spin_once.side_effect = spin_once
    made: list[str] = []
    monkeypatch.setattr(node_module, "make_executor", lambda k: made.append(k) or executor)
    monkeypatch.setattr(node_module, "rclpy", fake)
    monkeypatch.setattr(node_module, "FilterMetrics", MagicMock())  # the real gauges register globally once
    config = FilterConfig(
        input_topic="/in",
        output_topic="/out",
        algorithm="kalman",
        algorithm_params={},
        control_loop_hz=100.0,
        joint_names=[],
        executor=kind,
    )

    node_module.serve_filter_node(config, MagicMock())
    return made, executor, timeouts


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_run_spins_the_configured_executor(node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, kind: str) -> None:
    made, executor, timeouts = run_loop(node_module, monkeypatch, kind)
    assert made == [kind]
    executor.add_node.assert_called_once()
    assert len(timeouts) == LOOP_ITERATIONS


@pytest.mark.parametrize("kind", ["events", "single_threaded"])
def test_control_loop_spins_once_per_period(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, kind: str
) -> None:
    _, _, timeouts = run_loop(node_module, monkeypatch, kind)
    assert timeouts == [pytest.approx(0.01)] * LOOP_ITERATIONS
