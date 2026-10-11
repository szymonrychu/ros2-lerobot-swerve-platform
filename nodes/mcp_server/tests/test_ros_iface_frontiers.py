"""ros_iface.RosRobot frontier support (map_grid, compute_path, nav_active) against stubbed ROS types.

Covers: the map_grid version key (stamp sec / nanosec, size, origin), staleness against frontiers.map_max_age_s, a
map in another frame moved into the map frame through TF (and None without that TF), no map at all; every
compute_path outcome (succeeded with the summed segment length, planner error code, goal rejected, no answer,
no result in time -> cancelled, empty path, action server not ready -> RobotError) and the goal it sends
(use_start false, planner_id, map frame pose); nav_active from the base_motion lock and the nav_status goal states.
"""

import math
import time
from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.frontiers import MapGrid
from mcp_server.models import BasePose, PathPlan, RobotError
from mcp_server.staleness import Stamped

STATUS = SimpleNamespace(
    STATUS_UNKNOWN=0, STATUS_ACCEPTED=1, STATUS_EXECUTING=2, STATUS_CANCELING=3, STATUS_SUCCEEDED=4, STATUS_CANCELED=5,
    STATUS_ABORTED=6,
)  # fmt: skip


class ReadyFuture:
    """rclpy Future that is done from the start (or never, when finished is False)."""

    def __init__(self, response: Any = None, finished: bool = True) -> None:
        self.response = response
        self.finished = finished
        self.cancelled = False

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        if self.finished:
            callback(self)

    def done(self) -> bool:
        return self.finished

    def result(self) -> Any:
        return self.response

    def cancel(self) -> None:
        self.cancelled = True


def pose_stub() -> Any:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=""),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )


def goal_stub() -> Any:
    return SimpleNamespace(goal=None, start=None, planner_id="", use_start=True)


def make_robot(module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Any:
    """RosRobot without __init__, with stub message types and a mock compute_path client."""
    monkeypatch.setattr(module, "GoalStatus", STATUS)
    monkeypatch.setattr(module, "PoseStamped", pose_stub)
    monkeypatch.setattr(module, "ComputePathToPose", SimpleNamespace(Goal=goal_stub))
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.cfg = McpServerConfig()
    robot.cfg.timeouts.action_server_wait_s = 0.05
    robot.lock = module.threading.Lock()
    robot.latest = {}
    robot.base_motion = module.threading.Lock()
    robot.compute_client = MagicMock()
    robot.compute_client.wait_for_server.return_value = True
    robot.lookup_pose = lambda frame, child: None
    return robot


def map_msg(frame: str = "map", sec: int = 10, nanosec: int = 5, origin: tuple[float, float] = (-1.0, -2.0)) -> Any:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        info=SimpleNamespace(
            width=3,
            height=2,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[-1, 0, 100, 0, 0, -1],
    )


def put_map(robot: Any, msg: Any, age_s: float = 0.0) -> None:
    robot.latest["map"] = Stamped(msg, time.monotonic() - age_s)


# --- map_grid ---------------------------------------------------------------------------------------------------------


def test_map_grid_version_key_and_cells(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    put_map(robot, map_msg())
    grid = robot.map_grid()
    assert isinstance(grid, MapGrid)
    assert grid.version == (10, 5, 3, 2, -1.0, -2.0)
    assert grid.data.shape == (2, 3)
    assert grid.data.tolist() == [[-1, 0, 100], [0, 0, -1]]
    assert (grid.resolution, grid.origin_x, grid.origin_y, grid.origin_yaw) == (0.05, -1.0, -2.0, 0.0)
    assert grid.known_cells == 4


def test_map_grid_version_changes_with_stamp_size_and_origin(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    versions = []
    for msg in (
        map_msg(),
        map_msg(sec=11),
        map_msg(nanosec=6),
        map_msg(origin=(0.0, -2.0)),
        map_msg(origin=(-1.0, 0.0)),
    ):
        put_map(robot, msg)
        versions.append(robot.map_grid().version)
    assert len(set(versions)) == len(versions)
    put_map(robot, map_msg())
    assert robot.map_grid().version == versions[0]


def test_map_grid_is_none_without_a_map_or_when_stale(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    assert robot.map_grid() is None
    max_age = robot.cfg.frontiers.map_max_age_s
    put_map(robot, map_msg(), age_s=max_age + 1.0)
    assert robot.map_grid() is None
    put_map(robot, map_msg(), age_s=max_age - 1.0)
    assert robot.map_grid() is not None


def test_map_grid_in_another_frame_is_moved_through_tf(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    put_map(robot, map_msg(frame="slam", origin=(1.0, 0.0)))
    assert robot.map_grid() is None  # no TF map <- slam
    robot.lookup_pose = lambda frame, child: BasePose(frame=frame, x=10.0, y=20.0, yaw=math.pi / 2)
    grid = robot.map_grid()
    assert grid.origin_x == pytest.approx(10.0)
    assert grid.origin_y == pytest.approx(21.0)
    assert grid.origin_yaw == pytest.approx(math.pi / 2)
    assert grid.version == (10, 5, 3, 2, 1.0, 0.0)


# --- compute_path -----------------------------------------------------------------------------------------------------


def result_msg(
    poses: list[tuple[float, float]],
    error_code: int = 0,
    error_msg: str = "",
    status: int = 4,
    planning_s: float = 0.25,
) -> Any:
    path = SimpleNamespace(
        poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))) for x, y in poses]
    )
    result = SimpleNamespace(
        path=path,
        planning_time=SimpleNamespace(sec=int(planning_s), nanosec=int((planning_s % 1) * 1e9)),
        error_code=error_code,
        error_msg=error_msg,
    )
    return SimpleNamespace(status=status, result=result)


def arm_client(robot: Any, result: Any, accepted: bool = True, result_done: bool = True) -> SimpleNamespace:
    handle = SimpleNamespace(
        accepted=accepted,
        get_result_async=lambda: ReadyFuture(result, finished=result_done),
        cancel_goal_async=lambda: ReadyFuture(None),
    )
    robot.compute_client.send_goal_async.return_value = ReadyFuture(handle)
    return handle


def test_compute_path_success_sums_segments_and_sends_the_configured_goal(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    arm_client(robot, result_msg([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]))
    plan = robot.compute_path(3.0, 4.0, 0.5, 1.0)
    assert isinstance(plan, PathPlan)
    assert plan.status == "succeeded"
    assert plan.length_m == pytest.approx(7.0)
    assert plan.planning_time_s == pytest.approx(0.25)
    assert plan.error_code == 0
    assert plan.goal == BasePose(frame="map", x=3.0, y=4.0, yaw=0.5)
    (goal,), _ = robot.compute_client.send_goal_async.call_args
    assert goal.use_start is False
    assert goal.planner_id == robot.cfg.topics.planner_id
    assert goal.goal.header.frame_id == robot.cfg.topics.map_frame
    assert (goal.goal.pose.position.x, goal.goal.pose.position.y) == (3.0, 4.0)
    assert goal.goal.pose.orientation.z == pytest.approx(math.sin(0.25))
    assert goal.goal.pose.orientation.w == pytest.approx(math.cos(0.25))


def test_compute_path_planner_error_is_failed_with_its_code(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    arm_client(robot, result_msg([], error_code=204, error_msg="no valid path", status=6))
    plan = robot.compute_path(1.0, 1.0, 0.0, 1.0)
    assert (plan.status, plan.error_code, plan.error_msg) == ("failed", 204, "no valid path")
    assert plan.length_m is None


def test_compute_path_empty_path_is_failed(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    arm_client(robot, result_msg([]))
    plan = robot.compute_path(1.0, 1.0, 0.0, 1.0)
    assert plan.status == "failed"
    assert plan.length_m is None
    assert "empty" in plan.error_msg


def test_compute_path_rejected_goal_is_failed(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    arm_client(robot, None, accepted=False)
    plan = robot.compute_path(1.0, 1.0, 0.0, 1.0)
    assert (plan.status, plan.error_msg) == ("failed", "goal rejected")


def test_compute_path_without_an_answer_is_unavailable(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    robot.compute_client.send_goal_async.return_value = ReadyFuture(finished=False)
    plan = robot.compute_path(1.0, 1.0, 0.0, 0.05)
    assert plan.status == "unavailable"
    assert plan.length_m is None


def test_compute_path_without_a_result_in_time_times_out_and_cancels(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    cancelled: list[bool] = []
    handle = arm_client(robot, None, result_done=False)
    handle.cancel_goal_async = lambda: cancelled.append(True) or ReadyFuture(None)
    plan = robot.compute_path(1.0, 1.0, 0.0, 0.05)
    assert plan.status == "timeout"
    assert plan.planning_time_s is not None
    assert cancelled == [True]


def test_compute_path_server_not_ready_raises(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    robot.compute_client.wait_for_server.return_value = False
    with pytest.raises(RobotError, match="compute_path_to_pose"):
        robot.compute_path(1.0, 1.0, 0.0, 1.0)
    robot.compute_client.send_goal_async.assert_not_called()


def test_compute_path_rejects_a_non_finite_goal(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    with pytest.raises(RobotError, match="finite"):
        robot.compute_path(float("nan"), 1.0, 0.0, 1.0)


# --- nav_active -------------------------------------------------------------------------------------------------------


def put_status(robot: Any, *states: int) -> None:
    msg = SimpleNamespace(status_list=[SimpleNamespace(status=s) for s in states])
    robot.latest["nav_status"] = Stamped(msg, time.monotonic())


def test_nav_active_is_false_when_idle(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    assert robot.nav_active() is False
    put_status(robot)
    assert robot.nav_active() is False
    put_status(robot, STATUS.STATUS_SUCCEEDED, STATUS.STATUS_CANCELED, STATUS.STATUS_ABORTED)
    assert robot.nav_active() is False


@pytest.mark.parametrize("state", [STATUS.STATUS_ACCEPTED, STATUS.STATUS_EXECUTING])
def test_nav_active_while_a_goal_is_accepted_or_executing(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch, state: int
) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    put_status(robot, STATUS.STATUS_SUCCEEDED, state)
    assert robot.nav_active() is True


def test_nav_active_while_the_base_motion_lock_is_held(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = make_robot(ros_iface, monkeypatch)
    with robot.base_motion:
        assert robot.nav_active() is True
    assert robot.nav_active() is False
