"""ros_iface.RosRobot drive pose wiring without ROS: the ROS packages are replaced by mocks (conftest ros_iface).

Covers: the 5 Hz timer ticks DrivePose with nav_active(); /arm/drive_pose/state (std_msgs/String JSON, reliable +
transient_local, depth 1) is published on every change and at 1 Hz, nothing before the controller is bound;
/arm/set_drive_pose stores the measured pose only after check_drive_pose from the stored home passes;
/arm/drive_pose/enable toggles at runtime (not persisted); both refused without fresh joint states; navigate asks for
the lift (request_lift, the goal edge) before sending its goal and sends it whatever the outcome.
"""

import contextlib
import json
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.control import ControlManager
from mcp_server.drive_pose import DrivePose
from mcp_server.drive_pose_check import DrivePoseViolation
from mcp_server.models import DrivePoseState

from .test_drive_pose import DRIVE, FakeArm
from .test_tools import FakeRobot

STATE_TOPIC = "/arm/drive_pose/state"


class StringMsg(SimpleNamespace):
    """std_msgs/String stand-in that keeps its data."""


def make_robot(module: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Any:
    """RosRobot without __init__: a mock node, the default config and a FakeRobot arm; ROS IO of the drive pose set up.

    Args:
        module (ModuleType): The mocked ros_iface module.
        tmp_path (Path): Test dir.
        monkeypatch (pytest.MonkeyPatch): Patcher.

    Returns:
        Any: The robot.
    """
    monkeypatch.setattr(module, "String", lambda data: StringMsg(data=data))
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.node = MagicMock()
    robot.group = None
    robot.log = MagicMock()
    fake = FakeRobot(tmp_path)
    robot.fake = fake
    robot.arm = fake.arm
    robot.arm_backend = fake.arm_backend
    robot.cfg = fake.arm.cfg  # home_file in tmp_path
    robot.cfg.arm.drive_pose.stored_file = tmp_path / "drive_pose.yaml"
    robot.leader = MagicMock()
    robot.leader.moving.return_value = False
    robot.goal = [False]
    robot.nav_active = lambda: robot.goal[0]
    robot.init_drive_pose_io(robot.cfg)
    return robot


def bind(robot: Any) -> ControlManager:
    """Bind a ControlManager (creates the DrivePose) and return it."""
    control = ControlManager(robot.arm, robot.fake, robot.cfg)
    robot.bind_control(control)
    return control


def timer(robot: Any, period: float) -> Any:
    """The callback of the timer created with period."""
    callbacks = [c.args[1] for c in robot.node.create_timer.call_args_list if c.args[0] == pytest.approx(period)]
    assert len(callbacks) == 1, f"one timer at {period} s expected"
    return callbacks[0]


def published(robot: Any) -> list[dict[str, Any]]:
    return [json.loads(c.args[0].data) for c in robot.drive_pose_pub.publish.call_args_list]


def test_state_topic_is_latched_reliable_depth_one(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    (msg_type, topic, qos), _kw = next(
        c for c in robot.node.create_publisher.call_args_list if c.args[1] == STATE_TOPIC
    )
    assert topic == STATE_TOPIC and qos is ros_iface.DRIVE_POSE_STATE_QOS
    assert msg_type is ros_iface.String
    qos_kwargs = [c.kwargs for c in ros_iface.QoSProfile.call_args_list]
    assert {
        "reliability": ros_iface.ReliabilityPolicy.RELIABLE,
        "durability": ros_iface.DurabilityPolicy.TRANSIENT_LOCAL,
        "history": ros_iface.HistoryPolicy.KEEP_LAST,
        "depth": 1,
    } in qos_kwargs


def test_nothing_is_published_before_the_controller_is_bound(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    timer(robot, 0.2)()
    timer(robot, 1.0)()
    robot.drive_pose_pub.publish.assert_not_called()


def test_five_hz_timer_ticks_with_nav_active(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.drive_pose = MagicMock()
    robot.drive_pose.state.return_value = DrivePoseState(state="idle", reason="", enabled=True, stamp_ns=1)
    tick = timer(robot, 0.2)
    robot.goal[0] = True
    tick()
    (now, active), _ = robot.drive_pose.tick.call_args
    assert active is True and now > 0.0
    robot.goal[0] = False
    tick()
    assert robot.drive_pose.tick.call_args.args[1] is False


def test_state_is_published_on_change_and_at_one_hz(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    control = bind(robot)
    assert control.drive_pose is robot.drive_pose
    tick, every_second = timer(robot, 0.2), timer(robot, 1.0)
    tick()
    first = published(robot)
    assert first == [{"state": "disabled", "reason": "", "enabled": False, "stamp_ns": first[0]["stamp_ns"]}]
    tick()
    assert len(published(robot)) == 1, "unchanged: not republished by the tick"
    every_second()
    assert published(robot)[-1] == first[0] and len(published(robot)) == 2
    robot.drive_pose.set_enabled(True)
    tick()
    assert published(robot)[-1]["state"] == "idle" and published(robot)[-1]["enabled"] is True
    assert len(published(robot)) == 3


# --- /arm/set_drive_pose and /arm/drive_pose/enable ------------------------------------------------------------------


def service(robot: Any, name: str) -> Any:
    """The callback of the service created on name."""
    return next(c.args[2] for c in robot.node.create_service.call_args_list if c.args[1] == name)


def response() -> SimpleNamespace:
    return SimpleNamespace(success=None, message=None)


def test_set_drive_pose_stores_the_measured_pose_checked_from_the_stored_home(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.arm.set_home()
    home = robot.arm.stored_home()
    robot.arm_backend.positions["elbow_flex"] = 0.4
    checks: list[tuple[Any, dict[str, float], dict[str, float]]] = []

    def check(config: Any, start: dict[str, float], pose: dict[str, float]) -> list[Any]:
        checks.append((config, dict(start), dict(pose)))
        return []

    monkeypatch.setattr(ros_iface, "check_drive_pose", check)
    answer = service(robot, "/arm/set_drive_pose")(None, response())
    assert answer.success is True
    assert checks == [(robot.cfg, home, robot.arm.measured(robot.arm.fresh_sample()))]
    stored = robot.cfg.arm.drive_pose.stored_file
    assert stored == tmp_path / "drive_pose.yaml" and stored.is_file()
    assert robot.drive_pose.joints["elbow_flex"] == pytest.approx(0.4)


def test_set_drive_pose_refuses_naming_the_failed_checks(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.arm.set_home()
    monkeypatch.setattr(
        ros_iface, "check_drive_pose", lambda *_a: [DrivePoseViolation("footprint", "lift leg 1 sample 4")]
    )
    answer = service(robot, "/arm/set_drive_pose")(None, response())
    assert answer.success is False and "footprint" in answer.message
    assert not robot.cfg.arm.drive_pose.stored_file.exists()


def test_set_drive_pose_refused_without_fresh_joint_states(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.arm.set_home()
    robot.arm_backend.stale_after = robot.arm_backend.t
    robot.arm_backend.t += 100.0
    answer = service(robot, "/arm/set_drive_pose")(None, response())
    assert answer.success is False and "joint states" in answer.message


def test_enable_toggles_at_runtime_and_is_not_persisted(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    enable = service(robot, "/arm/drive_pose/enable")
    answer = enable(SimpleNamespace(data=True), response())
    assert answer.success is True and robot.drive_pose.state().enabled is True
    assert robot.cfg.arm.drive_pose.enabled is False, "the config stays the boot default"
    answer = enable(SimpleNamespace(data=False), response())
    assert answer.success is True and robot.drive_pose.state().state == "disabled"
    bind(robot)  # a controller restart boots from the config again
    assert robot.drive_pose.state().enabled is False


def test_enable_refused_without_fresh_joint_states(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.arm_backend.stale_after = robot.arm_backend.t
    robot.arm_backend.t += 100.0
    answer = service(robot, "/arm/drive_pose/enable")(SimpleNamespace(data=True), response())
    assert answer.success is False and "joint states" in answer.message
    assert robot.drive_pose.state().enabled is False


def test_services_refused_before_the_controller_is_bound(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    assert service(robot, "/arm/set_drive_pose")(None, response()).success is False
    assert service(robot, "/arm/drive_pose/enable")(SimpleNamespace(data=True), response()).success is False


# --- navigate lifts before sending its goal --------------------------------------------------------------------------


class FakeDrive:
    """DrivePose stand-in recording request_lift and answering with a fixed state."""

    def __init__(self, events: list[str], state: str) -> None:
        self.events = events
        self.answer = state
        self.enabled = True

    def request_lift(self, timeout_s: float) -> DrivePoseState:
        self.events.append(f"request_lift {timeout_s:g}")
        return DrivePoseState(state=self.answer, enabled=True, stamp_ns=1)


def nav_robot(module: ModuleType, tmp_path: Path, monkeypatch: Any, events: list[str]) -> Any:
    """A robot whose navigate runs against a recording run_nav (no Nav2)."""
    robot = make_robot(module, tmp_path, monkeypatch)
    robot.base_motion = threading.Lock()
    robot.base_stop = threading.Event()
    robot.monitor = MagicMock()
    robot.motion_rate = lambda *_subsystems: contextlib.nullcontext()

    def run_nav(_port: Any, goal: Any, *_args: Any) -> Any:
        events.append(f"goal {goal.x:g}")
        return "nav result"

    monkeypatch.setattr(module, "run_nav", run_nav)
    return robot


@pytest.mark.parametrize("answer", ["lifted", "skipped: torque off", "lifting"], ids=["lifted", "skipped", "timeout"])
def test_navigate_requests_the_lift_then_sends_the_goal_whatever_the_outcome(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any, answer: str
) -> None:
    events: list[str] = []
    robot = nav_robot(ros_iface, tmp_path, monkeypatch, events)
    robot.drive_pose = FakeDrive(events, answer)
    assert robot.navigate(1.5, 0.0, 0.0, "map", 30.0) == "nav result"
    assert events == [f"request_lift {robot.cfg.arm.drive_pose.lift_timeout_s:g}", "goal 1.5"]


def test_navigate_with_the_controller_disabled_does_not_wait(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    events: list[str] = []
    robot = nav_robot(ros_iface, tmp_path, monkeypatch, events)
    bind(robot)
    robot.cfg.arm.drive_pose.lift_timeout_s = 30.0
    assert robot.drive_pose.state().state == "disabled"
    started = time.monotonic()
    robot.navigate(2.0, 0.0, 0.0, "map", 30.0)
    assert time.monotonic() - started < 1.0 and events == ["goal 2"]


def test_navigate_lift_counts_as_the_goal_edge(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    """The lift starts at the request: the BT wait sees lifted at once (the worker runs the lift synchronously)."""
    events: list[str] = []
    robot = nav_robot(ros_iface, tmp_path, monkeypatch, events)
    control = bind(robot)
    drive = DrivePose(
        robot.cfg.arm.drive_pose.model_copy(update={"enabled": True, "joints": {**DRIVE}}),
        FakeArm(),
        control,
        publish_static=lambda: None,
        submit=lambda job: job(),
    )
    robot.drive_pose = drive
    robot.navigate(1.0, 0.0, 0.0, "map", 30.0)
    assert drive.state().state == "lifted" and events == ["goal 1"]


def test_navigate_without_a_controller_sends_the_goal(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    events: list[str] = []
    robot = nav_robot(ros_iface, tmp_path, monkeypatch, events)
    robot.navigate(1.0, 0.0, 0.0, "map", 30.0)
    assert events == ["goal 1"]
