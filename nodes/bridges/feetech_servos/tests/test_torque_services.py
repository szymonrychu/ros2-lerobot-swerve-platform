"""Torque services /<ns>/servo/{release,enable}_torque and the /<ns>/torque_state topic on an emulated bus."""

import importlib
import json
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from feetech_servos.bridge_cycle import drain_callbacks
from feetech_servos.config import JointGroup
from feetech_servos.registers import STS_TORQUE_ENABLE, read_all_registers
from feetech_servos.torque_control import TorqueControl, torque_state_payload

from .ros_stubs import install_service_stubs
from .test_torque_control import ARM, GROUPS, HEAD, SWERVE, Rig, make_rig

install_service_stubs()

INTERVAL_S = 1.0


class RecordingNode:
    """rclpy node double recording the services and publishers the bridge wiring creates."""

    def __init__(self) -> None:
        self.services: dict[str, Any] = {}
        self.publishers: dict[str, list[Any]] = {}
        self.publisher_qos: dict[str, Any] = {}

    def create_service(self, _srv_type: Any, name: str, callback: Any) -> MagicMock:
        """Record a service callback by name."""
        self.services[name] = callback
        return MagicMock()

    def create_publisher(self, _msg_type: Any, name: str, qos: Any) -> MagicMock:
        """Record a publisher; its published messages land in publishers[name]."""
        self.publishers[name] = []
        self.publisher_qos[name] = qos
        publisher = MagicMock()
        publisher.publish.side_effect = self.publishers[name].append
        return publisher


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import feetech_servos.bridge with ROS stubbed and String replaced by a plain message class."""
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "String", lambda data: SimpleNamespace(data=data))
    return module


class Wiring:
    """The torque wiring on one rig, with a ROS callback queue like the executor's."""

    def __init__(self, bridge_module: ModuleType, rig: Rig, groups: list[JointGroup], interval_s: float) -> None:
        self.rig = rig
        self.node = RecordingNode()
        self.callbacks_run = [0]
        self.queue: list[Any] = []
        self.control = TorqueControl(rig.control.servo, groups, rig.last_written, rig.cycle, verify_sleep_s=0.0)
        # The startup torque sequence read torque 1 back from every position joint.
        self.control.joint_torque.update({j.id: 1 for g in groups for j in g.joints if j.mode == "position"})
        self.reporter = bridge_module.setup_torque_interfaces(
            self.node, groups, self.control, self.callbacks_run, interval_s
        )

    def call(self, service: str) -> Any:
        """Queue a service request; it runs when spin drains the callbacks."""
        response = SimpleNamespace(success=None, message=None)
        self.queue.append(lambda: self.node.services[service](SimpleNamespace(), response))
        return response

    def spin(self) -> None:
        """Run the queued callbacks the way the loop's drain_callbacks does."""

        def spin_ready() -> bool:
            before = self.callbacks_run[0]
            if self.queue:
                self.queue.pop(0)()
            return self.callbacks_run[0] != before

        drain_callbacks(spin_ready)

    def states(self, namespace: str) -> list[dict[str, Any]]:
        """Decoded torque_state payloads published so far for a namespace."""
        return [json.loads(m.data) for m in self.node.publishers[f"/{namespace}/torque_state"]]


def make_wiring(bridge_module: ModuleType, groups: list[JointGroup], interval_s: float = INTERVAL_S) -> Wiring:
    return Wiring(bridge_module, make_rig(), groups, interval_s)


def test_services_exist_per_namespace_for_one_group(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [ARM])
    assert set(wiring.node.services) == {"/follower/servo/release_torque", "/follower/servo/enable_torque"}
    assert set(wiring.node.publishers) == {"/follower/torque_state"}


def test_services_exist_per_namespace_for_extra_groups(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, GROUPS)
    for ns in ("follower", "swerve_drive", "head"):
        assert f"/{ns}/servo/release_torque" in wiring.node.services
        assert f"/{ns}/servo/enable_torque" in wiring.node.services
        assert f"/{ns}/torque_state" in wiring.node.publishers


def test_torque_state_topic_is_reliable_transient_local_depth_one(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [ARM])
    qos = wiring.node.publisher_qos["/follower/torque_state"]
    assert qos is bridge_module.QoSProfile.return_value
    kwargs = bridge_module.QoSProfile.call_args.kwargs
    assert kwargs["depth"] == 1
    assert kwargs["reliability"] is bridge_module.ReliabilityPolicy.RELIABLE
    assert kwargs["durability"] is bridge_module.DurabilityPolicy.TRANSIENT_LOCAL


def test_release_answers_success_and_next_state_is_released(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, GROUPS)
    wiring.reporter.step(0.0, 100)
    assert wiring.states("follower")[-1]["released"] is False

    response = wiring.call("/follower/servo/release_torque")
    wiring.spin()
    assert response.success is True
    assert "torque off" in response.message
    assert wiring.callbacks_run[0] == 1

    wiring.reporter.step(0.1, 200)
    state = wiring.states("follower")[-1]
    assert state["namespace"] == "follower"
    assert state["released"] is True
    assert state["stamp_ns"] == 200
    assert state["joints"] == {"shoulder_pan": 0, "wrist_roll": 0, "gripper": 0}
    assert wiring.states("head")[-1]["released"] is False  # other groups untouched


def test_enable_answers_success_and_state_returns_to_held(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, GROUPS)
    wiring.call("/head/servo/release_torque")
    wiring.spin()
    wiring.reporter.step(0.0, 1)
    assert wiring.states("head")[-1]["released"] is True

    response = wiring.call("/head/servo/enable_torque")
    wiring.spin()
    assert response.success is True
    wiring.reporter.step(0.1, 2)
    assert wiring.states("head")[-1] == {
        "namespace": "head",
        "released": False,
        "joints": {"head_pan": 1, "head_tilt": 1},
        "stamp_ns": 2,
    }


def test_velocity_joints_are_never_listed_or_released(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, GROUPS)
    wiring.call("/swerve_drive/servo/release_torque")
    wiring.spin()
    wiring.reporter.step(0.0, 1)
    state = wiring.states("swerve_drive")[-1]
    assert state["released"] is True
    assert set(state["joints"]) == {"fl_steer", "fr_steer", "rl_steer", "rr_steer"}
    drive_ids = {32, 35, 36, 39}
    assert all(wiring.rig.port.servos[sid][STS_TORQUE_ENABLE] == 1 for sid in drive_ids)


def test_without_a_bus_services_refuse_and_nothing_is_published(bridge_module: ModuleType) -> None:
    rig = make_rig()
    wiring = Wiring(bridge_module, rig, [ARM], INTERVAL_S)
    wiring.control.servo = None
    response = wiring.call("/follower/servo/release_torque")
    wiring.spin()
    assert (response.success, response.message) == (False, "no servo bus")
    wiring.reporter.step(0.0, 1)
    assert wiring.node.publishers["/follower/torque_state"] == []


def test_payload_omits_a_joint_without_a_reading() -> None:
    payload = torque_state_payload(ARM, {1: 1, 6: 0}, set(), 5)
    assert payload == {
        "namespace": "follower",
        "released": False,
        "joints": {"shoulder_pan": 1, "gripper": 0},
        "stamp_ns": 5,
    }


def test_payload_released_when_any_position_joint_is_gated() -> None:
    payload = torque_state_payload(SWERVE, {33: 1}, {34}, 1)
    assert payload["released"] is True
    assert payload["joints"] == {"fl_steer": 1}


def test_no_reading_means_nothing_published(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [HEAD])
    wiring.control.joint_torque.clear()
    wiring.reporter.step(0.0, 1)
    assert wiring.node.publishers["/head/torque_state"] == []


def test_publishes_on_change_and_every_interval(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [HEAD])
    wiring.control.joint_torque.update({21: 1, 22: 1})
    wiring.reporter.step(0.0, 1)
    wiring.reporter.step(0.5, 2)  # unchanged, interval not elapsed
    assert len(wiring.states("head")) == 1
    wiring.reporter.step(1.0, 3)  # interval elapsed
    assert len(wiring.states("head")) == 2


def test_interval_zero_publishes_on_change_only(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [HEAD], interval_s=0.0)
    wiring.control.joint_torque.update({21: 1, 22: 1})
    wiring.reporter.step(0.0, 1)
    wiring.reporter.step(100.0, 2)
    assert len(wiring.states("head")) == 1
    wiring.control.joint_torque[21] = 0
    wiring.reporter.step(101.0, 3)
    assert len(wiring.states("head")) == 2


def test_register_dump_showing_torque_zero_changes_the_state(bridge_module: ModuleType) -> None:
    wiring = make_wiring(bridge_module, [ARM])
    wiring.control.joint_torque.update({1: 1, 5: 1, 6: 1})
    wiring.reporter.step(0.0, 1)
    assert wiring.states("follower")[-1]["joints"]["gripper"] == 1

    wiring.rig.port.servos[6][STS_TORQUE_ENABLE] = 0  # the servo dropped torque by itself
    dump = {j.name: read_all_registers(wiring.control.servo, j.id) for j in ARM.joints}
    wiring.control.note_dump(dump)
    wiring.reporter.step(0.1, 2)  # well inside the interval: published because it changed
    states = wiring.states("follower")
    assert len(states) == 2
    assert states[-1]["joints"] == {"shoulder_pan": 1, "wrist_roll": 1, "gripper": 0}
    assert states[-1]["released"] is False
