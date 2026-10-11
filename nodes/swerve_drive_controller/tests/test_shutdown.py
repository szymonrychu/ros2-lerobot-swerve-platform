"""run_swerve_controller exits cleanly when systemd stops the service (context already shut down by rclpy)."""

import importlib
from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from swerve_drive_controller.tilt_guard import TiltGuardConfig

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

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

CONFIG = SimpleNamespace(
    metrics_port=None,
    joint_commands_topic="/cmd",
    odom_topic="/odom",
    publish_tf=False,
    cmd_vel_topic="/cmd_vel",
    joint_states_topic="/js",
    control_loop_hz=50.0,
    half_length_m=0.1,
    half_width_m=0.1,
    wheel_radius_m=0.03,
    max_steer_angle_rad=1.0,
    max_wheel_angular_velocity_rad_s=10.0,
    joint_names=["fl_drive", "fl_steer", "fr_drive", "fr_steer", "rl_drive", "rl_steer", "rr_drive", "rr_steer"],
    tilt_guard=TiltGuardConfig(),
)


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import swerve_drive_controller.node with ROS stubbed and the metrics server silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a MagicMock class.
    """
    module = importlib.import_module("swerve_drive_controller.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_returns_after_external_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(node_module, "rclpy", fake)

    node_module.run_swerve_controller(CONFIG)

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy(context_invalid())
    fake.alive_after_spin = True
    monkeypatch.setattr(node_module, "rclpy", fake)

    with pytest.raises(type(fake.spin_error)):
        node_module.run_swerve_controller(CONFIG)

    node_module.Node.return_value.destroy_node.assert_called_once()
