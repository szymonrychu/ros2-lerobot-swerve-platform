"""Tests for the ROS wiring helpers of node.py with the ROS packages stubbed (no ROS in the dev environment)."""

import math
import sys
import types
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest


class StubRequest:
    """Request message accepting keyword fields."""

    def __init__(self, **fields: Any) -> None:
        self.__dict__.update(fields)


def stub_srv(name: str, **constants: int) -> type:
    """Service type with Request and Response (carrying the result constants)."""
    return type(name, (), {"Request": StubRequest, "Response": type("Response", (), constants)})


def install_stubs() -> None:
    """Inject stub ROS modules so map_guard.node imports."""
    names = [
        "rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "rclpy.time", "rclpy.callback_groups",
        "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
        "rcl_interfaces", "rcl_interfaces.msg", "rcl_interfaces.srv", "slam_toolbox", "slam_toolbox.srv",
        "std_msgs", "std_msgs.msg", "tf2_ros",
    ]  # fmt: skip
    for name in names:
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["rclpy.node"].Node = type("Node", (), {})
    sys.modules["rclpy.executors"].MultiThreadedExecutor = MagicMock
    sys.modules["rclpy.callback_groups"].ReentrantCallbackGroup = MagicMock
    qos = sys.modules["rclpy.qos"]
    qos.QoSProfile = StubRequest
    qos.DurabilityPolicy = qos.HistoryPolicy = qos.ReliabilityPolicy = MagicMock()
    sys.modules["rclpy.time"].Time = MagicMock
    for module, classes in (
        ("geometry_msgs.msg", ["Twist"]),
        ("nav_msgs.msg", ["OccupancyGrid", "Odometry"]),
        ("sensor_msgs.msg", ["Imu", "LaserScan"]),
        ("std_msgs.msg", ["Empty", "String"]),
    ):
        for cls in classes:
            setattr(sys.modules[module], cls, MagicMock())
    sys.modules["rcl_interfaces.msg"].ParameterType = SimpleNamespace(PARAMETER_BOOL=1)
    sys.modules["rcl_interfaces.srv"].GetParameters = stub_srv("GetParameters")
    slam = sys.modules["slam_toolbox.srv"]
    slam.Pause = stub_srv("Pause")
    slam.Reset = stub_srv("Reset", RESULT_SUCCESS=0)
    slam.SerializePoseGraph = stub_srv("SerializePoseGraph", RESULT_SUCCESS=0, RESULT_FAILED_TO_WRITE_FILE=255)
    tf2 = sys.modules["tf2_ros"]
    tf2.Buffer = tf2.TransformListener = MagicMock
    tf2.TransformException = type("TransformException", (Exception,), {})


install_stubs()

from map_guard.config import MapGuardConfig  # noqa: E402
from map_guard.node import PAUSED_PARAMETER, SlamServices, grid_from_msg  # noqa: E402


class FakeFuture:
    def __init__(self, response: Any) -> None:
        self.response = response

    def add_done_callback(self, callback: Any) -> None:
        if self.response is not None:
            callback(self)

    def result(self) -> Any:
        return self.response


class FakeClient:
    def __init__(self, name: str, response: Any, ready: bool = True) -> None:
        self.srv_name = name
        self.response = response
        self.ready = ready
        self.requests: list[Any] = []

    def wait_for_service(self, **_kwargs: float) -> bool:
        return self.ready

    def call_async(self, request: Any) -> FakeFuture:
        self.requests.append(request)
        return FakeFuture(self.response)


class FakeNode:
    def __init__(self, responses: dict[str, Any]) -> None:
        self.responses = responses
        self.clients: dict[str, FakeClient] = {}

    def get_logger(self) -> Any:
        return MagicMock()

    def create_client(self, _srv_type: Any, name: str, **_kwargs: Any) -> FakeClient:
        client = FakeClient(name, self.responses.get(name))
        self.clients[name] = client
        return client


def services(responses: dict[str, Any]) -> tuple[SlamServices, FakeNode]:
    node = FakeNode(responses)
    return SlamServices(node, MapGuardConfig(service_timeout_s=0.05)), node


def test_grid_from_msg() -> None:
    half = math.radians(90) / 2
    msg = SimpleNamespace(
        info=SimpleNamespace(
            width=3,
            height=2,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=-2.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
            ),
        ),
        data=[0, 100, -1, 50, 0, 0],
    )
    grid = grid_from_msg(msg)
    assert grid.cells.shape == (2, 3)
    assert grid.cells[0, 1] == 100 and grid.cells[0, 2] == -1 and grid.cells[1, 0] == 50
    assert (grid.origin_x, grid.origin_y) == (1.0, -2.0)
    assert grid.origin_yaw == pytest.approx(math.pi / 2)
    assert grid.cells.dtype == np.int16


def test_query_paused_reads_the_parameter() -> None:
    value = SimpleNamespace(type=1, bool_value=True)
    slam, node = services({"/slam_toolbox/get_parameters": SimpleNamespace(values=[value])})
    assert slam.query_paused() is True
    assert node.clients["/slam_toolbox/get_parameters"].requests[0].names == [PAUSED_PARAMETER]


def test_query_paused_unknown_parameter_type() -> None:
    slam, _ = services({"/slam_toolbox/get_parameters": SimpleNamespace(values=[SimpleNamespace(type=0)])})
    assert slam.query_paused() is None


def test_unavailable_service_returns_none_or_false() -> None:
    slam, node = services({})
    for client in node.clients.values():
        client.ready = False
    assert slam.query_paused() is None
    assert slam.toggle_pause() is False
    assert slam.serialize_map("/tmp/x") is False
    assert slam.reset() is False


def test_timed_out_call_returns_false() -> None:
    slam, _ = services({})  # futures never complete
    assert slam.toggle_pause() is False


def test_toggle_pause_checks_status() -> None:
    slam, _ = services({"/slam_toolbox/pause_new_measurements": SimpleNamespace(status=True)})
    assert slam.toggle_pause() is True
    slam, _ = services({"/slam_toolbox/pause_new_measurements": SimpleNamespace(status=False)})
    assert slam.toggle_pause() is False


def test_serialize_and_reset_check_result_codes() -> None:
    slam, node = services(
        {"/slam_toolbox/serialize_map": SimpleNamespace(result=0), "/slam_toolbox/reset": SimpleNamespace(result=0)}
    )
    assert slam.serialize_map("/var/lib/ros2/maps/b") is True
    assert node.clients["/slam_toolbox/serialize_map"].requests[0].filename == "/var/lib/ros2/maps/b"
    assert slam.reset() is True
    assert node.clients["/slam_toolbox/reset"].requests[0].pause_new_measurements is False
    slam, _ = services({"/slam_toolbox/serialize_map": SimpleNamespace(result=255)})
    assert slam.serialize_map("/x") is False
