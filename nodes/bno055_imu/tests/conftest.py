"""Pytest configuration: stub out ROS2 modules so non-ROS helpers can be tested without rclpy."""

import sys
import types


def _make_stub_module(name: str) -> types.ModuleType:
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _stub_ros2_modules() -> None:
    """Register lightweight stubs for rclpy and sensor_msgs so node.py can be imported."""
    if "rclpy" in sys.modules:
        return  # already available (real ROS2 env), no stubs needed

    # rclpy top-level
    rclpy = _make_stub_module("rclpy")
    rclpy.init = lambda *_a: None  # type: ignore[attr-defined]
    rclpy.ok = lambda: False  # type: ignore[attr-defined]
    rclpy.shutdown = lambda: None  # type: ignore[attr-defined]
    rclpy.spin_once = lambda *_a: None  # type: ignore[attr-defined]

    # rclpy.node
    node_mod = _make_stub_module("rclpy.node")
    node_mod.Node = object  # type: ignore[attr-defined]

    # rclpy.qos
    qos_mod = _make_stub_module("rclpy.qos")

    class _FakeQoS:
        def __init__(self, **_kw: object) -> None:
            pass

    class _FakeEnum:
        RELIABLE = "RELIABLE"
        KEEP_LAST = "KEEP_LAST"

    qos_mod.QoSProfile = _FakeQoS  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = _FakeEnum  # type: ignore[attr-defined]
    qos_mod.HistoryPolicy = _FakeEnum  # type: ignore[attr-defined]

    # sensor_msgs.msg
    sensor_msgs = _make_stub_module("sensor_msgs")
    sensor_msgs_msg = _make_stub_module("sensor_msgs.msg")
    sensor_msgs.msg = sensor_msgs_msg  # type: ignore[attr-defined]
    sensor_msgs_msg.Imu = object  # type: ignore[attr-defined]


_stub_ros2_modules()
