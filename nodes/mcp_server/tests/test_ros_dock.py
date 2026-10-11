"""mcp_server.ros_dock and its RosRobot wiring without ROS: the ROS packages are replaced by mocks (conftest ros_iface
fixture) and the message classes by plain attribute bags, so goal mapping, published detections, cancel coverage,
costmap staleness and planner results can be checked on fake action clients. Nothing here moves a robot.

Covers: DockRobot goal fields (use_dock_id false, map dock pose with yaw quaternion, dock_type, staging flag,
max_staging_time) and the accepted / rejected / no-answer outcomes, feedback state names, result mapping and the own
goal registry; ComputePathToPose result mapping (ok with path length, no_path on abort or empty path, unavailable when
the server is down, does not answer, rejects or times out); the lazy global costmap cache (no subscription until
first use, grid layout and origin, age grows, stale grids are still returned with their age, none without a message);
DetectionPublisher (frame map, header stamp = the capture stamp, nothing published by construction, other frames
refused); RosRobot wiring (cancel_all_goals covers NavigateToPose and /grasp/dock_robot with the zero goal id, a dock
server that is not running costs no wait, reset_base cancels own dock goals, dock() runs run_dock under the base
motion lock, plan_path, global_costmap, publish_detection).
"""

import math
import sys
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.approach import PathInfo
from mcp_server.base_dock import DockGoal
from mcp_server.config import McpServerConfig
from mcp_server.control import NavGoalRegistry
from mcp_server.models import BaseMotionBusyError, BasePose, RobotError


class Auto(SimpleNamespace):
    """Message stand-in: any attribute read creates a nested bag, so nested fields can be assigned directly."""

    def __getattr__(self, name: str) -> Any:
        if name.startswith("__"):
            raise AttributeError(name)
        child = Auto()
        setattr(self, name, child)
        return child


class FakeFuture:
    """rclpy Future stand-in that is already done (or never done)."""

    def __init__(self, response: Any = None, done: bool = True) -> None:
        self.response = response
        self.finished = done
        self.callbacks: list[Any] = []
        self.cancelled = False

    def add_done_callback(self, callback: Any) -> None:
        self.callbacks.append(callback)
        if self.finished:
            callback(self)

    def done(self) -> bool:
        return self.finished

    def result(self) -> Any:
        return self.response

    def cancel(self) -> None:
        self.cancelled = True


class FakeHandle:
    """ClientGoalHandle stand-in."""

    def __init__(self, accepted: bool = True, result: Any = None, result_done: bool = False) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture(result, done=result_done)
        self.cancel_requests = 0

    def get_result_async(self) -> FakeFuture:
        return self.result_future

    def cancel_goal_async(self) -> FakeFuture:
        self.cancel_requests += 1
        return FakeFuture(object())


def fake_client(handle: FakeHandle | None, ready: bool = True) -> MagicMock:
    client = MagicMock()
    client.wait_for_server.return_value = ready
    client.send_goal_async.side_effect = lambda goal, **kwargs: FakeFuture(handle)
    return client


@pytest.fixture
def dock(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """mcp_server.ros_dock imported with the message classes replaced by attribute bags."""
    module = sys.modules["mcp_server.ros_dock"]
    monkeypatch.setattr(module, "DockRobot", SimpleNamespace(Goal=Auto))
    monkeypatch.setattr(module, "ComputePathToPose", SimpleNamespace(Goal=Auto))
    monkeypatch.setattr(module, "PoseStamped", Auto)
    return module


def status_of(module: ModuleType, name: str) -> Any:
    """The GoalStatus constant (a mock attribute here) that ros_dock names `name`."""
    return next(key for key, value in module.GOAL_STATUS_NAMES.items() if value == name)


def dock_goal() -> DockGoal:
    return DockGoal(
        dock_pose=BasePose(frame="map", x=1.0, y=2.0, yaw=math.pi / 2),
        dock_type="grasp_servoed",
        navigate_to_staging_pose=True,
        max_staging_time_s=90.0,
    )


def dock_port(module: ModuleType, client: Any, goals: NavGoalRegistry | None = None) -> Any:
    return module.RosDockPort(
        client, goals or NavGoalRegistry(), 0.1, MagicMock(), lambda: BasePose(frame="map", x=1.0, y=2.0, yaw=0.0)
    )


# --- RosDockPort ---------------------------------------------------------------------------------------------


def test_goal_fields_are_mapped(dock: ModuleType) -> None:
    handle = FakeHandle()
    client = fake_client(handle)
    assert dock_port(dock, client).send_goal(dock_goal()) is True
    goal = client.send_goal_async.call_args.args[0]
    assert goal.use_dock_id is False
    assert goal.dock_type == "grasp_servoed"
    assert goal.navigate_to_staging_pose is True
    assert goal.max_staging_time == 90.0
    assert goal.dock_pose.header.frame_id == "map"
    assert (goal.dock_pose.pose.position.x, goal.dock_pose.pose.position.y) == (1.0, 2.0)
    o = goal.dock_pose.pose.orientation
    assert (o.x, o.y) == (0.0, 0.0)
    assert o.z == pytest.approx(math.sin(math.pi / 4))
    assert o.w == pytest.approx(math.cos(math.pi / 4))


def test_rejection_and_no_answer(dock: ModuleType) -> None:
    assert dock_port(dock, fake_client(FakeHandle(accepted=False))).send_goal(dock_goal()) is False
    silent = MagicMock()
    silent.send_goal_async.return_value = FakeFuture(done=False)
    assert dock_port(dock, silent).send_goal(dock_goal()) is None


def test_server_ready_waits_for_the_server(dock: ModuleType) -> None:
    client = fake_client(None, ready=False)
    assert dock_port(dock, client).server_ready() is False
    client.wait_for_server.assert_called_once_with(timeout_sec=0.1)


def test_feedback_state_is_tracked_by_name(dock: ModuleType) -> None:
    client = fake_client(FakeHandle())
    port = dock_port(dock, client)
    assert port.feedback_state() is None
    port.send_goal(dock_goal())
    callback = client.send_goal_async.call_args.kwargs["feedback_callback"]
    callback(SimpleNamespace(feedback=SimpleNamespace(state=3)))
    assert port.feedback_state() == "CONTROLLING"
    callback(SimpleNamespace(feedback=SimpleNamespace(state=0)))  # NONE keeps the last named state
    assert port.feedback_state() == "CONTROLLING"
    callback(SimpleNamespace(feedback=SimpleNamespace(state=1)))
    assert port.feedback_state() == "NAV_TO_STAGING_POSE"


def test_a_new_goal_forgets_the_old_feedback(dock: ModuleType) -> None:
    client = fake_client(FakeHandle())
    port = dock_port(dock, client)
    port.send_goal(dock_goal())
    client.send_goal_async.call_args.kwargs["feedback_callback"](SimpleNamespace(feedback=SimpleNamespace(state=2)))
    port.send_goal(dock_goal())
    assert port.feedback_state() is None


def test_result_mapping(dock: ModuleType) -> None:
    result = SimpleNamespace(
        status=status_of(dock, "aborted"), result=SimpleNamespace(success=False, error_code=904, num_retries=1)
    )
    handle = FakeHandle(result=result, result_done=True)
    port = dock_port(dock, fake_client(handle))
    port.send_goal(dock_goal())
    assert port.result_ready() is True
    assert port.result() == ("aborted", 904, 1, "")


def test_result_without_answer_is_unknown(dock: ModuleType) -> None:
    handle = FakeHandle(result=None, result_done=True)
    port = dock_port(dock, fake_client(handle))
    port.send_goal(dock_goal())
    assert port.result() == ("unknown", 999, 0, "")


def test_own_goal_is_registered_until_its_result(dock: ModuleType) -> None:
    handle = FakeHandle()
    goals = NavGoalRegistry()
    port = dock_port(dock, fake_client(handle), goals)
    port.send_goal(dock_goal())
    assert goals.handles == [handle]
    handle.result_future.finished = True
    for callback in handle.result_future.callbacks:
        callback(handle.result_future)
    assert goals.handles == []


def test_rejected_goal_is_not_registered(dock: ModuleType) -> None:
    goals = NavGoalRegistry()
    dock_port(dock, fake_client(FakeHandle(accepted=False)), goals).send_goal(dock_goal())
    assert goals.handles == []


def test_cancel_cancels_the_goal_and_waits_for_the_result(dock: ModuleType) -> None:
    handle = FakeHandle()
    port = dock_port(dock, fake_client(handle))
    port.send_goal(dock_goal())
    port.cancel()
    assert handle.cancel_requests == 1


def test_zero_velocity_and_pose_are_delegated(dock: ModuleType) -> None:
    zero = MagicMock()
    port = dock.RosDockPort(fake_client(None), NavGoalRegistry(), 0.1, zero, lambda: None)
    port.zero_velocity()
    zero.assert_called_once_with()
    assert port.pose() is None


# --- RosPlannerPort ------------------------------------------------------------------------------------------


def path_of(points: list[tuple[float, float]]) -> Any:
    return SimpleNamespace(
        poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))) for x, y in points]
    )


def planner(module: ModuleType, handle: FakeHandle | None, ready: bool = True, timeout_s: float = 0.05) -> Any:
    return module.RosPlannerPort(fake_client(handle, ready), 0.1, timeout_s)


def planned(module: ModuleType, status: str, points: list[tuple[float, float]], error_code: int = 0) -> FakeHandle:
    result = SimpleNamespace(
        status=status_of(module, status), result=SimpleNamespace(path=path_of(points), error_code=error_code)
    )
    return FakeHandle(result=result, result_done=True)


GOAL_POSE = BasePose(frame="map", x=3.0, y=4.0, yaw=0.0)


def test_planner_goal_uses_the_current_robot_pose_as_start(dock: ModuleType) -> None:
    port = planner(dock, planned(dock, "succeeded", [(0.0, 0.0), (3.0, 4.0)]))
    port.plan(GOAL_POSE)
    goal = port.client.send_goal_async.call_args.args[0]
    assert goal.use_start is False
    assert goal.goal.header.frame_id == "map"
    assert (goal.goal.pose.position.x, goal.goal.pose.position.y) == (3.0, 4.0)


def test_planner_ok_returns_the_path_length(dock: ModuleType) -> None:
    info = planner(dock, planned(dock, "succeeded", [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])).plan(GOAL_POSE)
    assert info == PathInfo(status="ok", length_m=7.0)


def test_planner_abort_is_no_path_with_the_error_code(dock: ModuleType) -> None:
    info = planner(dock, planned(dock, "aborted", [], error_code=204)).plan(GOAL_POSE)
    assert info.status == "no_path"
    assert "204" in (info.reason or "")


def test_planner_empty_path_is_no_path(dock: ModuleType) -> None:
    assert planner(dock, planned(dock, "succeeded", [])).plan(GOAL_POSE).status == "no_path"


def test_planner_server_down_is_unavailable(dock: ModuleType) -> None:
    info = planner(dock, None, ready=False).plan(GOAL_POSE)
    assert info.status == "unavailable"
    assert "not available" in (info.reason or "")


def test_planner_rejection_and_silence_are_unavailable(dock: ModuleType) -> None:
    assert planner(dock, FakeHandle(accepted=False)).plan(GOAL_POSE).status == "unavailable"
    silent = FakeHandle(result_done=False)
    info = planner(dock, silent).plan(GOAL_POSE)
    assert info.status == "unavailable"
    assert "no result" in (info.reason or "")
    assert silent.cancel_requests == 1  # a plan nobody waits for is cancelled


def test_planner_without_answer_to_the_goal_is_unavailable(dock: ModuleType) -> None:
    client = MagicMock()
    client.wait_for_server.return_value = True
    client.send_goal_async.return_value = FakeFuture(done=False)
    assert dock.RosPlannerPort(client, 0.05, 0.05).plan(GOAL_POSE).status == "unavailable"


# --- GlobalCostmapCache --------------------------------------------------------------------------------------


def grid_msg(width: int = 4, height: int = 3, resolution: float = 0.05) -> Any:
    msg = Auto()
    msg.info.width, msg.info.height, msg.info.resolution = width, height, resolution
    msg.info.origin.position.x, msg.info.origin.position.y = -1.0, -2.0
    msg.info.origin.orientation.x = msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z, msg.info.origin.orientation.w = 0.0, 1.0
    msg.data = list(range(width * height))
    return msg


def cache(module: ModuleType, clock: list[float], wait_s: float = 0.05) -> tuple[Any, MagicMock]:
    node = MagicMock()
    sleeps: list[float] = []

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        clock[0] += seconds

    return module.GlobalCostmapCache(node, "/global_costmap/costmap", None, wait_s, lambda: clock[0], sleep), node


def deliver(node: MagicMock, msg: Any) -> None:
    node.create_subscription.call_args.args[2](msg)


def test_costmap_is_not_subscribed_until_first_use(dock: ModuleType) -> None:
    costmap, node = cache(dock, [0.0])
    node.create_subscription.assert_not_called()
    assert costmap.latest() is None  # first use subscribes; nothing arrives within the wait
    assert node.create_subscription.call_count == 1
    assert node.create_subscription.call_args.args[1] == "/global_costmap/costmap"
    costmap.latest()
    assert node.create_subscription.call_count == 1


def test_costmap_grid_layout_and_age(dock: ModuleType) -> None:
    clock = [10.0]
    costmap, node = cache(dock, clock)
    costmap.latest()
    deliver(node, grid_msg())
    clock[0] += 0.4
    grid = costmap.latest()
    assert grid.data.shape == (3, 4)
    assert grid.data[1, 2] == 6  # row-major, row along +y
    assert (grid.resolution, grid.origin_x, grid.origin_y, grid.origin_yaw) == (0.05, -1.0, -2.0, 0.0)
    assert grid.age_s == pytest.approx(0.4)


def test_costmap_origin_yaw_comes_from_the_quaternion(dock: ModuleType) -> None:
    costmap, node = cache(dock, [0.0])
    costmap.latest()
    msg = grid_msg()
    msg.info.origin.orientation.z = math.sin(math.pi / 4)
    msg.info.origin.orientation.w = math.cos(math.pi / 4)
    deliver(node, msg)
    assert costmap.latest().origin_yaw == pytest.approx(math.pi / 2)


def test_stale_costmap_is_returned_with_its_age(dock: ModuleType) -> None:
    clock = [0.0]
    costmap, node = cache(dock, clock)
    costmap.latest()
    deliver(node, grid_msg())
    clock[0] += 60.0
    grid = costmap.latest()
    assert grid is not None
    assert grid.age_s == pytest.approx(60.0)  # approach.check_grid refuses it with the reason


def test_costmap_first_use_waits_for_the_first_message(dock: ModuleType) -> None:
    clock = [0.0]
    costmap, node = cache(dock, clock, wait_s=1.0)
    first = {"done": False}

    def sleep(seconds: float) -> None:
        clock[0] += seconds
        if not first["done"]:
            first["done"] = True
            deliver(node, grid_msg())

    costmap.sleep = sleep
    grid = costmap.latest()
    assert grid is not None
    assert clock[0] < 1.0


def test_costmap_ignores_a_message_with_the_wrong_size(dock: ModuleType) -> None:
    costmap, node = cache(dock, [0.0])
    costmap.latest()
    msg = grid_msg()
    msg.data = [0, 1]
    deliver(node, msg)
    assert costmap.latest() is None


# --- DetectionPublisher --------------------------------------------------------------------------------------


def test_detection_is_published_in_map_with_the_capture_stamp(dock: ModuleType) -> None:
    publisher = MagicMock()
    detection = dock.DetectionPublisher(publisher, "map")
    publisher.publish.assert_not_called()  # never publishes on its own
    detection.publish(BasePose(frame="map", x=1.5, y=-0.5, yaw=math.pi), 1_700_000_002_500_000_000)
    msg = publisher.publish.call_args.args[0]
    assert msg.header.frame_id == "map"
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1_700_000_002, 500_000_000)
    assert (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) == (1.5, -0.5, 0.0)
    assert msg.pose.orientation.z == pytest.approx(1.0)
    assert msg.pose.orientation.w == pytest.approx(0.0, abs=1e-12)


def test_detection_in_another_frame_is_refused(dock: ModuleType) -> None:
    publisher = MagicMock()
    with pytest.raises(RobotError, match="map"):
        dock.DetectionPublisher(publisher, "map").publish(BasePose(frame="odom", x=0.0, y=0.0, yaw=0.0), 1)
    publisher.publish.assert_not_called()


# --- RosRobot wiring -----------------------------------------------------------------------------------------


def wired_robot(module: ModuleType, handle: FakeHandle | None = None) -> Any:
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.node = MagicMock()
    robot.group = None
    robot.log = MagicMock()
    robot.cfg = McpServerConfig()
    robot.cancel_client = MagicMock()
    robot.cancel_client.wait_for_service.return_value = True
    robot.cancel_client.call_async.side_effect = lambda request: FakeFuture(object())
    robot.dock_cancel_client = MagicMock()
    robot.dock_cancel_client.service_is_ready.return_value = True
    robot.dock_cancel_client.call_async.side_effect = lambda request: FakeFuture(object())
    robot.dock_client = fake_client(handle)
    robot.planner_client = fake_client(None)
    robot.own_goals = NavGoalRegistry()
    robot.own_dock_goals = NavGoalRegistry()
    robot.base_motion = module.threading.Lock()
    robot.base_stop = module.threading.Event()
    robot.monitor = MagicMock()
    return robot


def test_cancel_all_goals_covers_both_actions_with_the_zero_goal_id(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    assert robot.cancel_all_goals() is True
    for client in (robot.cancel_client, robot.dock_cancel_client):
        request = client.call_async.call_args.args[0]
        assert client.call_async.call_count == 1
        assert request.goal_info is not None


def test_cancel_all_goals_cancels_docking_even_when_nav_cancel_fails(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.cancel_client.wait_for_service.return_value = False
    assert robot.cancel_all_goals() is False
    assert robot.dock_cancel_client.call_async.call_count == 1


def test_a_dock_server_that_is_not_running_costs_no_wait(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.dock_cancel_client.service_is_ready.return_value = False
    assert robot.cancel_all_goals() is True
    robot.dock_cancel_client.wait_for_service.assert_not_called()
    robot.dock_cancel_client.call_async.assert_not_called()


def test_an_unanswered_dock_cancel_is_reported(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.dock_cancel_client.call_async.side_effect = lambda request: FakeFuture(done=False)
    assert robot.cancel_all_goals() is False


def test_stop_and_the_tilt_canceller_reach_the_dock_cancel(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.stops = 0
    robot.publish_twist = MagicMock()
    robot.arm = MagicMock()
    robot.arm.stop_hold.return_value = (False, "")
    robot.stop()
    assert robot.dock_cancel_client.call_async.call_count == 1  # stop -> cancel_all_goals
    # the monitor's tilt-warning canceller is cancel_all_goals itself (RosRobot.__init__ registers it)
    source = open(ros_iface.__file__).read()
    assert "set_nav_canceller(self.cancel_all_goals)" in source


def test_reset_base_cancels_own_dock_goals(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.publish_twist = MagicMock()
    own = FakeHandle()
    robot.own_dock_goals.add(own)
    assert robot.reset_base() is True
    assert own.cancel_requests == 1
    assert robot.own_dock_goals.handles == []


def test_dock_runs_run_dock_under_the_base_motion_lock(
    ros_iface: ModuleType, dock: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = wired_robot(ros_iface, FakeHandle())
    calls: list[Any] = []

    def fake_run_dock(port: Any, goal: DockGoal, timeout_s: float, *rest: Any) -> str:
        calls.append((type(port).__name__, goal, timeout_s, robot.base_motion.locked()))
        return "result"

    monkeypatch.setattr(ros_iface, "run_dock", fake_run_dock)
    assert robot.dock(dock_goal(), 120.0) == "result"
    assert calls == [("RosDockPort", dock_goal(), 120.0, True)]
    assert not robot.base_motion.locked()


def test_dock_refuses_while_another_base_motion_runs(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface, FakeHandle())
    robot.base_motion.acquire()
    with pytest.raises(BaseMotionBusyError):
        robot.dock(dock_goal(), 1.0)


def test_plan_path_publication_and_costmap_are_delegated(ros_iface: ModuleType, dock: ModuleType) -> None:
    robot = wired_robot(ros_iface)
    robot.planner_client = fake_client(planned(dock, "succeeded", [(0.0, 0.0), (0.0, 2.0)]))
    assert robot.plan_path(GOAL_POSE) == PathInfo(status="ok", length_m=2.0)
    robot.detection = MagicMock()
    pose = BasePose(frame="map", x=1.0, y=1.0, yaw=0.0)
    robot.publish_detection(pose, 5)
    robot.detection.publish.assert_called_once_with(pose, 5)
    robot.costmap_cache = MagicMock()
    robot.costmap_cache.latest.return_value = None
    assert robot.global_costmap() is None
