"""serve_bridge loop wiring of the torque state: startup seeding, per-read register dump feed, state publishing."""

import importlib
import json
import sys
from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from feetech_servos.sync_read import ServoState

from .ros_stubs import FakeRclpy, install_service_stubs

install_service_stubs()

IDS = {"shoulder_pan": 1, "wrist_roll": 5, "gripper": 6}
STAMP_NS = 77


class LoopRclpy(FakeRclpy):
    """rclpy double whose context stays valid for a fixed number of loop iterations."""

    def __init__(self, cycles: int) -> None:
        super().__init__()
        self.cycles = cycles
        self.cycle = 0

    def ok(self) -> bool:
        self.cycle += 1
        return self.cycle <= self.cycles


class Harness:
    """serve_bridge on an emulated torque register file; callbacks scheduled per loop cycle."""

    def __init__(self, monkeypatch: pytest.MonkeyPatch, cycles: int, startup: bool = True) -> None:
        self.module: ModuleType = importlib.import_module("feetech_servos.bridge")
        config_module = importlib.import_module("feetech_servos.config")
        self.torque_module = importlib.import_module("feetech_servos.torque_control")
        self.torque = {sid: 1 for sid in IDS.values()}
        self.rclpy = LoopRclpy(cycles)
        self.scheduled: dict[int, list[Callable[[], None]]] = {}
        self.services: dict[str, Any] = {}
        self.states: list[dict[str, Any]] = []
        module = self.module
        for name in ("start_metrics_server", "use_blocking_reads", "_wait_for_servos"):
            monkeypatch.setattr(module, name, MagicMock())
        monkeypatch.setattr(module, "String", lambda data: SimpleNamespace(data=data))
        monkeypatch.setattr(module, "rclpy", self.rclpy)
        monkeypatch.setattr(
            module,
            "SingleThreadedExecutor",
            lambda: SimpleNamespace(add_node=lambda _n: None, spin_once=self.spin_once),
        )
        monkeypatch.setattr(module.time, "sleep", lambda _s: None)
        monkeypatch.setattr(module, "write_register", lambda *_a, **_k: True)
        monkeypatch.setattr(
            module, "read_register_raw", lambda _s, sid, _e: self.torque[sid] if _e.name == "torque_enable" else 2000
        )
        monkeypatch.setattr(module, "read_all_registers", lambda _s, sid: {"torque_enable": self.torque[sid]})
        monkeypatch.setattr(
            module,
            "set_startup_torque_state",
            lambda **kw: [sid for sid in kw["joint_ids"] if kw["read_once"](sid) is None],
        )
        monkeypatch.setattr(module, "read_servo_states", self.read_states)
        monkeypatch.setattr(self.torque_module, "write_register", self.write_torque)
        monkeypatch.setattr(self.torque_module, "read_register", lambda _s, sid, _e: self.torque[sid])
        monkeypatch.setattr(sys.modules["st3215"], "ST3215", MagicMock())
        self.node = MagicMock()
        self.node.get_clock.return_value.now.return_value.nanoseconds = STAMP_NS
        self.node.create_service.side_effect = lambda _t, name, cb: self.services.setdefault(name, cb)
        self.node.create_publisher.side_effect = self.make_publisher
        self.config = config_module.BridgeConfig(
            namespace="follower",
            joints=[config_module.JointEntry(name, sid) for name, sid in IDS.items()],
            device="/dev/ttyUSB0",
            enable_torque_on_start=startup,
            register_publish_interval_s=0.001,
            battery_interval_s=0.0,
            torque_state_interval_s=0.0,
        )

    def write_torque(self, _servo: Any, sid: int, entry: Any, value: int, _cache: dict[str, int]) -> bool:
        self.torque[sid] = value
        return True

    def read_states(self, *_a: Any, **_k: Any) -> dict[int, ServoState]:
        return {sid: ServoState(2000, 0, 0, STAMP_NS, 3) for sid in IDS.values()}

    def make_publisher(self, _type: Any, topic: str, _qos: Any) -> MagicMock:
        publisher = MagicMock()
        if topic == "/follower/torque_state":
            publisher.publish.side_effect = lambda msg: self.states.append(json.loads(msg.data))
        return publisher

    def spin_once(self, **_kwargs: Any) -> None:
        """Run one scheduled callback of the current loop cycle, like the executor."""
        todo = self.scheduled.get(self.rclpy.cycle, [])
        if todo:
            todo.pop(0)()

    def call(self, cycle: int, service: str) -> SimpleNamespace:
        """Schedule a service request for the drain of a loop cycle."""
        response = SimpleNamespace(success=None, message=None)
        self.scheduled.setdefault(cycle, []).append(lambda: self.services[service](SimpleNamespace(), response))
        return response

    def run(self) -> None:
        self.module.serve_bridge(self.config, self.node)


def test_startup_torque_seeds_the_state_published_on_the_first_cycle(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = Harness(monkeypatch, cycles=1)
    harness.run()
    assert harness.states[0]["joints"] == IDS_HELD
    assert harness.states[0]["released"] is False


def test_nothing_is_published_before_the_startup_torque_sequence(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = Harness(monkeypatch, cycles=1, startup=False)
    harness.config.register_publish_interval_s = 0.0  # no dump either: no reading at all
    harness.run()
    assert harness.states == []


def test_release_through_the_loop_publishes_released(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = Harness(monkeypatch, cycles=2)
    response = harness.call(2, "/follower/servo/release_torque")
    harness.run()
    assert response.success is True
    assert harness.states[-1]["released"] is True
    assert harness.states[-1]["joints"] == {name: 0 for name in IDS}


def test_register_dump_in_the_loop_shows_a_servo_that_dropped_torque(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = Harness(monkeypatch, cycles=6)
    harness.scheduled[2] = [lambda: harness.torque.__setitem__(6, 0)]
    harness.run()
    assert harness.states[-1]["joints"]["gripper"] == 0
    assert harness.states[-1]["released"] is False


def test_release_mid_dump_round_is_not_overwritten_by_older_dump_readings(monkeypatch: pytest.MonkeyPatch) -> None:
    # Cycle 1 dumps shoulder_pan at torque 1; the release lands in cycle 2; the round ends in cycle 3.
    harness = Harness(monkeypatch, cycles=3)
    harness.call(2, "/follower/servo/release_torque")
    harness.run()
    assert harness.states[-1]["released"] is True
    assert harness.states[-1]["joints"] == {name: 0 for name in IDS}


IDS_HELD = {name: 1 for name in IDS}
