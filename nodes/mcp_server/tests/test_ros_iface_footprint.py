"""ros_iface.RosRobot footprint sink (footprint.FootprintSink) without ROS: the ROS packages are replaced by mocks so
the timer, parameter-service, published_footprint and readiness bookkeeping can be checked against a fake node.

Covers: the re-send timer runs at the period NavFootprint asks for; parameter sets report ok / unavailable / refused /
no response and a request still unanswered at the next tick is cancelled and reported as a timeout; double values
for the costmap inflation; `<costmap>/published_footprint` polygons are kept per topic; a parameter service coming
up (unavailable -> ready edge) calls the consumer-ready callback once; the arm pose accessors.
"""

import importlib
import sys
from collections.abc import Callable, Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig

from .test_footprint import HOME, deployed_config, footprint_robot, move_arm

ROS_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.callback_groups",
    "rclpy.duration",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.signals",
    "rclpy.time",
    "action_msgs",
    "action_msgs.msg",
    "action_msgs.srv",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "nav2_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "rcl_interfaces.srv",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
    "tf2_ros",
)
LASER = "/laser_filter"
LOCAL_COSTMAP = "/local_costmap/local_costmap"


@pytest.fixture
def ros_iface(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """mcp_server.ros_iface imported against mocked ROS packages (removed again afterwards)."""
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, MagicMock())
    monkeypatch.delitem(sys.modules, "mcp_server.ros_iface", raising=False)
    module = importlib.import_module("mcp_server.ros_iface")
    yield module
    sys.modules.pop("mcp_server.ros_iface", None)


class FakeFuture:
    """rclpy Future stand-in: done callbacks, result, cancel."""

    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], None]] = []
        self.response: Any = None
        self.finished = False
        self.cancelled = False

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        self.callbacks.append(callback)

    def done(self) -> bool:
        return self.finished or self.cancelled

    def result(self) -> Any:
        return self.response

    def cancel(self) -> None:
        self.cancelled = True

    def finish(self, response: Any) -> None:
        self.response = response
        self.finished = True
        for callback in self.callbacks:
            callback(self)


def make_robot(module: ModuleType, tmp_path: Path, clock: list[float]) -> Any:
    """RosRobot without __init__: a mock node, the deployed nav_footprint config and a fake arm."""
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.node = MagicMock()
    robot.node.create_client.side_effect = lambda *_args, **_kwargs: MagicMock()
    robot.group = None
    robot.log = MagicMock()
    fake = footprint_robot(tmp_path)
    robot.arm = fake.arm
    robot.arm_backend = fake.arm_backend
    config = deployed_config()
    config.arm.urdf_path = McpServerConfig().arm.urdf_path
    robot.cfg = config
    robot.init_footprint_io(config)
    robot.param_clock = lambda: clock[0]
    return robot


def client_of(robot: Any, node: str) -> MagicMock:
    return robot.param_clients[node]


def test_every_creates_the_timer_with_the_requested_period(ros_iface: ModuleType, tmp_path: Path) -> None:
    robot = make_robot(ros_iface, tmp_path, [0.0])
    robot.node.create_timer.reset_mock()
    ticks: list[int] = []
    robot.every(2.5, lambda: ticks.append(1))
    (period, callback), _kwargs = robot.node.create_timer.call_args
    assert period == 2.5
    callback()
    assert ticks == [1]


def test_parameter_outcomes_are_reported(ros_iface: ModuleType, tmp_path: Path) -> None:
    robot = make_robot(ros_iface, tmp_path, [0.0])
    client = client_of(robot, LASER)
    outcomes: list[str] = []
    client.service_is_ready.return_value = False
    robot.set_node_parameter(LASER, "polygon", "[[0, 0], [1, 0], [1, 1]]", outcomes.append)
    assert outcomes == ["unavailable: /laser_filter/set_parameters not ready"]
    client.service_is_ready.return_value = True
    futures = [FakeFuture(), FakeFuture(), FakeFuture()]
    client.call_async.side_effect = futures
    for _ in futures:
        robot.set_node_parameter(LASER, "polygon", "[[0, 0], [1, 0], [1, 1]]", outcomes.append)
        # each set is answered before the next one is sent
        future = futures[len(outcomes) - 1]
        if future is futures[0]:
            future.finish(SimpleNamespace(results=[SimpleNamespace(successful=True, reason="")]))
        elif future is futures[1]:
            future.finish(SimpleNamespace(results=[SimpleNamespace(successful=False, reason="not a polygon")]))
        else:
            future.finish(None)
    assert outcomes[1:] == ["ok", "refused: not a polygon", "refused: no response"]


def test_unanswered_parameter_set_times_out_at_the_next_tick(ros_iface: ModuleType, tmp_path: Path) -> None:
    clock = [100.0]
    robot = make_robot(ros_iface, tmp_path, clock)
    client = client_of(robot, LASER)
    client.service_is_ready.return_value = True
    future = FakeFuture()
    client.call_async.side_effect = [future]
    outcomes: list[str] = []
    robot.every(5.0, lambda: None)
    robot.set_node_parameter(LASER, "polygon", "[[0, 0], [1, 0], [1, 1]]", outcomes.append)
    robot.on_footprint_tick()  # too young to give up on
    assert outcomes == [] and not future.cancelled
    clock[0] += robot.cfg.nav_footprint.param_timeout_s + 0.1
    robot.on_footprint_tick()
    assert future.cancelled
    client.remove_pending_request.assert_called_once_with(future)
    assert outcomes == [f"timeout: no answer within {robot.cfg.nav_footprint.param_timeout_s:g} s"]


def test_double_parameters_for_the_inflation(ros_iface: ModuleType) -> None:
    ros_iface.parameter_msg("inflation_layer.inflation_radius", 0.35)
    _args, kwargs = ros_iface.ParameterValue.call_args
    assert kwargs["double_value"] == 0.35
    assert kwargs["type"] == ros_iface.ParameterType.PARAMETER_DOUBLE


def test_published_footprints_are_kept_per_topic(ros_iface: ModuleType, tmp_path: Path) -> None:
    robot = make_robot(ros_iface, tmp_path, [0.0])
    topics = [call.args[1] for call in robot.node.create_subscription.call_args_list]
    assert topics == ["/local_costmap/published_footprint", "/global_costmap/published_footprint"]
    assert robot.observed_polygon("/local_costmap/published_footprint") is None
    msg = SimpleNamespace(
        polygon=SimpleNamespace(points=[SimpleNamespace(x=1.0, y=2.0), SimpleNamespace(x=3.0, y=4.0)])
    )
    callback = robot.node.create_subscription.call_args_list[0].args[2]
    callback(msg)
    assert robot.observed_polygon("/local_costmap/published_footprint") == [(1.0, 2.0), (3.0, 4.0)]
    assert robot.observed_polygon("/global_costmap/published_footprint") is None


def test_consumer_ready_edge_calls_back_once(ros_iface: ModuleType, tmp_path: Path) -> None:
    robot = make_robot(ros_iface, tmp_path, [0.0])
    ready: list[str] = []
    robot.on_consumer_ready(ready.append)
    for client in robot.param_clients.values():
        client.service_is_ready.return_value = False
    robot.poll_consumers()
    assert ready == []
    client_of(robot, LASER).service_is_ready.return_value = True
    robot.poll_consumers()
    robot.poll_consumers()
    assert ready == [LASER]
    client_of(robot, LASER).service_is_ready.return_value = False
    robot.poll_consumers()
    client_of(robot, LASER).service_is_ready.return_value = True
    client_of(robot, LOCAL_COSTMAP).service_is_ready.return_value = True
    robot.poll_consumers()
    assert sorted(ready[1:]) == [LASER, LOCAL_COSTMAP]


def test_arm_pose_accessors(ros_iface: ModuleType, tmp_path: Path) -> None:
    robot = make_robot(ros_iface, tmp_path, [0.0])
    move_arm(robot, HOME)
    assert robot.arm_lease_held() is False
    parts = robot.arm_parts()
    assert parts is not None and "tool_point" in parts
    mount = robot.cfg.arm.base_in_base_link
    assert robot.arm_mount_xy() == (mount.x, mount.y)
    robot.arm_backend.stale_after = robot.arm_backend.t
    robot.arm_backend.t += 100.0
    assert robot.arm_parts() is None
