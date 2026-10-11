"""Node wiring of the tilt guard: /imu/data in, /tilt_guard/state out, drive commands zeroed while inhibited."""

import importlib
import json
import math
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from swerve_drive_controller.config import SwerveControllerConfig, load_config

from .ros_stubs import FakeRclpy, install_ros_stubs

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

IMU_PERIOD_S = 0.02
DRIVE_INDICES = (0, 2, 4, 6)
NAMES = ["fl_drive", "fl_steer", "fr_drive", "fr_steer", "rl_drive", "rl_steer", "rr_drive", "rr_steer"]


class Harness:
    """Runs run_swerve_controller against a MagicMock Node and exposes its callbacks and published messages."""

    def __init__(self, module: ModuleType, monkeypatch: pytest.MonkeyPatch, config: SwerveControllerConfig) -> None:
        self.now = 100.0
        self.published: dict[str, list[Any]] = {}
        self.subscriptions: dict[str, Any] = {}
        self.timer: Any = None
        node = module.Node.return_value

        def create_publisher(_msg_type: Any, topic: str, _qos: Any) -> Any:
            pub = MagicMock(name=f"pub{topic}")
            self.published[topic] = []

            def record(msg: Any) -> None:
                self.published[topic].append(
                    SimpleNamespace(
                        data=getattr(msg, "data", None),
                        velocity=list(getattr(msg, "velocity", []) or []),
                        position=list(getattr(msg, "position", []) or []),
                    )
                )

            pub.publish.side_effect = record
            return pub

        def create_subscription(_msg_type: Any, topic: str, callback: Any, _qos: Any) -> Any:
            self.subscriptions[topic] = callback
            return MagicMock()

        def create_timer(_period: float, callback: Any) -> Any:
            self.timer = callback
            return MagicMock()

        node.create_publisher.side_effect = create_publisher
        node.create_subscription.side_effect = create_subscription
        node.create_timer.side_effect = create_timer
        self.logger = node.get_logger.return_value
        node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
        monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: self.now))
        monkeypatch.setattr(module, "rclpy", FakeRclpy())
        module.run_swerve_controller(config)

    def joint_states(self) -> None:
        msg = SimpleNamespace(name=NAMES, position=[0.0] * 8, velocity=[0.0] * 8)
        self.subscriptions["/swerve_drive/joint_states"](msg)

    def cmd_vel(self, vx: float) -> None:
        msg = SimpleNamespace(linear=SimpleNamespace(x=vx, y=0.0), angular=SimpleNamespace(z=0.0))
        self.subscriptions["/cmd_vel"](msg)

    def imu(self, roll_deg: float, duration: float) -> None:
        """Feed roll-only orientation samples at 50 Hz for duration seconds, running the timer each sample."""
        half = math.radians(roll_deg) / 2.0
        end = self.now + duration
        while self.now <= end + 1e-9:
            orientation = SimpleNamespace(x=math.sin(half), y=0.0, z=0.0, w=math.cos(half))
            self.subscriptions["/imu/data"](SimpleNamespace(orientation=orientation, orientation_covariance=[0.0] * 9))
            self.now += IMU_PERIOD_S

    def cycle(self) -> list[float]:
        """Refresh joint states, run one control cycle and return the drive velocities it published."""
        self.joint_states()
        self.timer()
        last = self.published["/swerve_drive/joint_commands"][-1]
        return [last.velocity[i] for i in DRIVE_INDICES]

    def states(self) -> list[dict[str, Any]]:
        return [json.loads(m.data) for m in self.published["/tilt_guard/state"]]

    def torque_writes(self) -> list[dict[str, Any]]:
        return [json.loads(m.data) for m in self.published.get("/follower/set_register", [])]


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    module = importlib.import_module("swerve_drive_controller.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


def make_config(tmp_path: Path, tilt_yaml: str = "") -> SwerveControllerConfig:
    path = tmp_path / "config.yaml"
    path.write_text(("tilt_guard:\n" + tilt_yaml) if tilt_yaml else "{}\n")
    cfg = load_config(path)
    assert cfg is not None
    return cfg


def test_subscribes_imu_and_publishes_state(node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    h = Harness(node_module, monkeypatch, make_config(tmp_path))
    assert "/imu/data" in h.subscriptions
    assert h.published["/tilt_guard/state"] == []  # nothing before the first IMU sample
    h.imu(0.0, 0.1)
    h.timer()
    states = h.states()
    assert states and states[-1]["level"] == "ok" and states[-1]["base_drive_inhibited"] is False


def test_drive_passes_when_level_and_is_zeroed_on_error(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from prometheus_client import REGISTRY

    h = Harness(node_module, monkeypatch, make_config(tmp_path))
    h.imu(0.0, 0.1)
    h.cmd_vel(0.1)
    assert all(v > 0.0 for v in h.cycle())

    h.imu(25.0, 0.6)  # roll beyond the 15 deg error limit for longer than the 0.5 s persistence
    assert h.states()[-1]["level"] == "error"  # the transition is published immediately
    h.logger.error.assert_called()
    before = REGISTRY.get_sample_value("tilt_guard_inhibited_commands_total") or 0.0
    h.cmd_vel(0.1)
    assert REGISTRY.get_sample_value("tilt_guard_inhibited_commands_total") == before + 1
    assert h.cycle() == [0.0, 0.0, 0.0, 0.0]
    assert h.torque_writes() == []  # brake: torque stays on

    h.imu(0.0, 2.1)  # level again for the 2 s recovery hold
    assert h.states()[-1]["level"] == "ok"
    h.cmd_vel(0.1)
    assert all(v > 0.0 for v in h.cycle())


def test_torque_off_action_disables_and_restores_drive_torque(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    h = Harness(node_module, monkeypatch, make_config(tmp_path, "  error_action: torque_off\n"))
    h.imu(25.0, 0.6)
    writes = h.torque_writes()
    assert [w["joint_name"] for w in writes] == ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]
    assert {w["register"] for w in writes} == {"torque_enable"} and {w["value"] for w in writes} == {0}
    h.imu(0.0, 2.1)
    assert [w["value"] for w in h.torque_writes()[4:]] == [1, 1, 1, 1]


def test_state_heartbeat_reports_stale_imu(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    cfg = make_config(tmp_path)
    cfg = replace(cfg, control_loop_hz=50.0)
    h = Harness(node_module, monkeypatch, cfg)
    h.imu(0.0, 0.1)
    h.now += 5.0
    h.timer()
    assert h.states()[-1]["imu_stale"] is True
    assert any("IMU" in str(c) for c in h.logger.warn.call_args_list)
