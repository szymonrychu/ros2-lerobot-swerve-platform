"""The self-collision guard sits in front of both publish sites of serve_filter_node (direct web UI / autonomy commands
and the filtered leader output), seeded by the follower feedback and posing the head from /head/joint_states."""

import importlib
import json
from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from arm_collision.head import HeadPose

from filter_node.config import FilterConfig, GuardSettings, LeaderGripperSettings
from filter_node.guard import ArmGuard

from .arm_fixture import ARM, CHECKER, DRIVE_POSE, HARD_MARGIN_M, PARKED_HOME, WHEEL_FR_PRESSED
from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

LEADER = "/in"
OUTPUT = "/out"
WEB_UI = "/web_ui"
AUTONOMY = "/autonomy"
GUARD_STATE = "/filter/guard_state"
FEEDBACK = "/follower/joint_states"
HEAD = "/head/joint_states"
ARM_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
GUARD_LOG_PERIOD_S = 5.0


def joint_msg(pose: dict[str, float]) -> SimpleNamespace:
    """A JointState-like message of a pose.

    Args:
        pose (dict[str, float]): Joint name -> rad.

    Returns:
        SimpleNamespace: name and position lists.
    """
    return SimpleNamespace(name=list(pose), position=list(pose.values()))


class Harness:
    """Drives serve_filter_node with a fake clock; records what goes out on the output topic."""

    def __init__(self, node_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
        self.now = 0.0
        self.callbacks: dict[str, Callable[[Any], None]] = {}
        self.steps: list[Callable[[], None]] = []
        self.published: list[tuple[str, dict[str, float]]] = []
        self.guards: list[ArmGuard] = []
        self.guard_states: list[dict[str, Any]] = []
        self.node = MagicMock()
        self.node.create_subscription.side_effect = lambda _t, topic, cb, _q: (
            self.callbacks.__setitem__(topic, cb) or topic
        )
        publishers: dict[str, MagicMock] = {}

        def create_publisher(_type: Any, topic: str, _qos: Any) -> MagicMock:
            pub = publishers.setdefault(topic, MagicMock())
            if topic == OUTPUT:
                pub.publish.side_effect = lambda m: self.published.append(
                    (m.header.frame_id, dict(zip(m.name, m.position, strict=True)))
                )
            if topic == GUARD_STATE:
                pub.publish.side_effect = lambda m: self.guard_states.append(json.loads(m.data))
            return pub

        self.node.create_publisher.side_effect = create_publisher
        fake = FakeRclpy()
        executor = MagicMock()

        def spin_once(timeout_sec: float) -> None:
            if self.steps:
                self.steps.pop(0)()
            fake.alive = bool(self.steps)

        executor.spin_once.side_effect = spin_once
        build = node_module.build_arm_guard

        def recorded_build(*args: Any) -> ArmGuard:
            guard = build(*args)
            self.guards.append(guard)
            return guard

        monkeypatch.setattr(node_module, "build_arm_guard", recorded_build)
        monkeypatch.setattr(node_module, "make_executor", lambda _k: executor)
        monkeypatch.setattr(node_module, "rclpy", fake)
        monkeypatch.setattr(node_module, "FilterMetrics", MagicMock())
        monkeypatch.setattr(node_module, "time", SimpleNamespace(monotonic=lambda: self.now))
        self.module = node_module

    @property
    def log(self) -> MagicMock:
        """The node logger."""
        return self.node.get_logger.return_value

    def run(self) -> None:
        """Serve with the guard configured until the steps run out."""
        config = FilterConfig(
            input_topic=LEADER,
            output_topic=OUTPUT,
            algorithm="kalman",
            algorithm_params={},
            control_loop_hz=100.0,
            joint_names=list(ARM_NAMES),
            idle_timeout_s=0.5,
            web_ui_input_topic=WEB_UI,
            web_ui_timeout_s=0.5,
            follower_feedback_topic=FEEDBACK,
            autonomy_input_topic=AUTONOMY,
            guard_state_topic=GUARD_STATE,
            self_collision_guard=GuardSettings(hard_margin_m=HARD_MARGIN_M, head_state_topic=HEAD, arm=ARM),
        )
        self.module.serve_filter_node(config, self.node)

    def at(self, now: float, topic: str | None = None, msg: Any = None) -> Callable[[], None]:
        """A step that sets the clock and delivers a message.

        Args:
            now (float): Clock (s).
            topic (str | None): Topic to deliver on; None only moves the clock.
            msg (Any): Message.

        Returns:
            Callable[[], None]: The step.
        """

        def step() -> None:
            self.now = now
            if topic is not None:
                self.callbacks[topic](msg)

        return step

    def call(self, now: float, action: Callable[[], Any]) -> Callable[[], None]:
        """A step that sets the clock and runs an action.

        Args:
            now (float): Clock (s).
            action (Callable[[], Any]): Run after the clock is set.

        Returns:
            Callable[[], None]: The step.
        """

        def step() -> None:
            self.now = now
            action()

        return step


@pytest.fixture
def harness(monkeypatch: pytest.MonkeyPatch) -> Harness:
    """Harness on the real node module.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        Harness: The harness.
    """
    return Harness(importlib.import_module("filter_node.node"), monkeypatch)


def assert_clamped(pose: dict[str, float], request: dict[str, float]) -> None:
    """The pose is a clamp short of the request, clear of the body.

    Args:
        pose (dict[str, float]): Published pose.
        request (dict[str, float]): Requested pose.
    """
    assert pose != request
    assert CHECKER.pose(pose).min_clearance >= HARD_MARGIN_M


@pytest.mark.parametrize(("topic", "source"), [(WEB_UI, "web_ui"), (AUTONOMY, "autonomy")])
def test_a_direct_command_into_a_wheel_publishes_the_clamp(harness: Harness, topic: str, source: str) -> None:
    """Web UI and autonomy commands pressing into wheel_fr go out clamped to the last safe sample."""
    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.2, topic, joint_msg(WHEEL_FR_PRESSED)),
        harness.at(0.3),
    ]
    harness.run()
    assert len(harness.published) == 1
    frame_id, pose = harness.published[0]
    assert frame_id == source
    assert_clamped(pose, WHEEL_FR_PRESSED)
    warnings = [str(c.args[0]) for c in harness.log.warning.call_args_list]
    assert any(source in w and "frame_right" in w for w in warnings)


def test_a_blocked_command_publishes_the_guard_state_and_a_clear_one_unblocks_it(harness: Harness) -> None:
    """The guard state topic reports the obstacle on a clamp and blocked false after the next clear command."""
    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.2, WEB_UI, joint_msg(WHEEL_FR_PRESSED)),
        harness.at(0.25),
        harness.at(0.3, WEB_UI, joint_msg(PARKED_HOME)),
        harness.at(0.35),
    ]
    harness.run()
    states = harness.guard_states
    assert [s["blocked"] for s in states] == [True, False]
    assert states[0]["source"] == "web_ui" and states[0]["obstacle"]


def test_a_clear_command_publishes_the_request(harness: Harness) -> None:
    """Home to the drive pose is clear: the web UI command goes out unchanged."""
    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.2, WEB_UI, joint_msg(DRIVE_POSE)),
        harness.at(0.3),
    ]
    harness.run()
    assert harness.published == [("web_ui", DRIVE_POSE)]


def test_a_blocked_leader_prediction_publishes_the_clamp(harness: Harness) -> None:
    """The filtered leader output goes through the guard: a prediction into the wheel goes out clamped."""
    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.2, LEADER, joint_msg(WHEEL_FR_PRESSED)),
        harness.at(0.2),
    ]
    harness.run()
    assert harness.published
    frame_id, pose = harness.published[0]
    assert frame_id == "leader"
    assert_clamped(pose, WHEEL_FR_PRESSED)


def test_a_request_whose_clamp_is_the_start_publishes_nothing(harness: Harness) -> None:
    """The second command into the wheel starts at the published clamp: nothing goes out."""
    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.2, WEB_UI, joint_msg(WHEEL_FR_PRESSED)),
        harness.at(0.21, WEB_UI, joint_msg(WHEEL_FR_PRESSED)),
        harness.at(0.3),
    ]
    harness.run()
    assert len(harness.published) == 1


def test_follower_feedback_seeds_the_guard(harness: Harness) -> None:
    """A gripper-only command needs the arm pose: refused before any feedback, published after it."""
    gripper = {"gripper": 0.3}
    harness.steps = [
        harness.at(0.1, WEB_UI, joint_msg(gripper)),
        harness.at(0.2, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.at(0.3, WEB_UI, joint_msg(gripper)),
        harness.at(0.4),
    ]
    harness.run()
    assert harness.published == [("web_ui", gripper)]


def test_a_guard_exception_publishes_nothing_and_logs_an_error_once_per_5_s(harness: Harness) -> None:
    """A failing check publishes nothing; the error is logged at most once per 5 s."""

    def broken() -> None:
        harness.guards[0].checker = SimpleNamespace(posed=MagicMock(side_effect=RuntimeError("kernel gone")))

    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.call(0.15, broken),
        harness.at(0.2, WEB_UI, joint_msg(DRIVE_POSE)),
        harness.at(0.3, WEB_UI, joint_msg(DRIVE_POSE)),
        harness.at(0.2 + GUARD_LOG_PERIOD_S + 0.1, WEB_UI, joint_msg(DRIVE_POSE)),
        harness.at(6.0),
    ]
    harness.run()
    assert harness.published == []
    errors = [str(c.args[0]) for c in harness.log.error.call_args_list]
    assert len(errors) == 2 and all("kernel gone" in e for e in errors)


def test_the_head_state_poses_the_head_while_fresh(harness: Harness) -> None:
    """A fresh /head/joint_states sample is the guard's head pose; a stale one (> 0.5 s) falls back to the envelope."""
    seen: list[HeadPose | None] = []
    harness.steps = [
        harness.at(1.0, HEAD, SimpleNamespace(name=["head_pan", "head_tilt"], position=[0.2, 0.5])),
        harness.call(1.4, lambda: seen.append(harness.guards[0].head_source())),
        harness.call(1.6, lambda: seen.append(harness.guards[0].head_source())),
        harness.at(1.7),
    ]
    harness.run()
    assert seen == [HeadPose(pan=0.2, tilt=0.5), None]


def test_build_arm_guard_maps_leader_grippers_with_the_configured_mapping() -> None:
    """The guard built from the config converts leader grippers with its leader_gripper block."""
    mapping = LeaderGripperSettings(
        source_min_steps=2045, source_max_steps=3289, command_min_steps=1900, command_max_steps=3377
    )
    module = importlib.import_module("filter_node.node")
    guard = module.build_arm_guard(GuardSettings(arm=ARM, leader_gripper=mapping), lambda: None)
    assert guard.leader_gripper == mapping
