"""Startup goal (startup_position_rad): config validation, queueing through BridgeCycle, bridge startup wiring."""

import importlib
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from feetech_servos.bridge_cycle import BridgeCycle
from feetech_servos.command_mapping import position_to_raw_steps
from feetech_servos.config import JointEntry, load_config
from feetech_servos.registers import STS_ACC, get_register_entry_by_name

from .ros_stubs import FakeRclpy, context_invalid, install_ros_stubs
from .sts_bus import make_bus, register_table

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

HOME_RAD = -0.5
HOME_STEPS = position_to_raw_steps(HOME_RAD)
PAN = JointEntry("pan", 1, startup_position_rad=HOME_RAD, max_speed_rad_s=1.0, max_acceleration_rad_s2=2.0)
TILT = JointEntry("tilt", 2, startup_position_rad=HOME_RAD)
PLAIN = JointEntry("plain", 3)


def make_cycle() -> tuple[BridgeCycle, object]:
    """Cycle over an emulated bus with servos 1..3."""
    servo, port = make_bus({1: register_table(), 2: register_table(), 3: register_table()})
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={},
        last_written={1: {}, 2: {}, 3: {}},
        velocity_command_timeout_s=0.3,
        goal_refresh_interval_s=1.0,
    )
    return cycle, port


def test_goal_written_once_to_expected_steps_capped_and_plain() -> None:
    cycle, port = make_cycle()
    queued = cycle.queue_startup_goals([PAN, TILT, PLAIN], {1, 2, 3}, MagicMock())
    assert queued == [1, 2]
    cycle.write_pending_commands()
    cycle.write_pending_commands()
    writes = sorted((w.servo_id, w.address, w.data) for w in port.sync_writes)
    lo, hi = HOME_STEPS & 0xFF, HOME_STEPS >> 8
    assert writes == [(1, STS_ACC, [14, lo, hi, 0, 0, 652 & 0xFF, 652 >> 8]), (2, 42, [lo, hi])]


def test_no_goal_for_servo_without_torque() -> None:
    cycle, port = make_cycle()
    log = MagicMock()
    assert cycle.queue_startup_goals([PAN, TILT], {2}, log) == [2]
    cycle.write_pending_commands()
    assert [w.servo_id for w in port.sync_writes] == [2]
    assert "pan" in str(log.call_args_list)


def test_startup_goal_is_logged_once_per_joint() -> None:
    cycle, _ = make_cycle()
    log = MagicMock()
    cycle.queue_startup_goals([PAN, PLAIN], {1, 3}, log)
    assert log.call_count == 1
    text = str(log.call_args)
    assert "pan" in text and str(HOME_STEPS) in text


def cfg_text(extra: str) -> str:
    return f"namespace: a\njoint_names:\n  - name: x\n    id: 1\n    {extra}\n"


def load(tmp_path: Path, extra: str):
    p = tmp_path / "c.yaml"
    p.write_text(cfg_text(extra))
    return load_config(p)


def test_config_parses_startup_position(tmp_path: Path) -> None:
    cfg = load(tmp_path, "startup_position_rad: -0.5\n    command_min_steps: 1000\n    command_max_steps: 3000")
    assert cfg is not None and cfg.joints[0].startup_position_rad == -0.5
    other = load(tmp_path, "inverted: true")
    assert other is not None and other.joints[0].startup_position_rad is None


@pytest.mark.parametrize(
    "extra",
    [
        "mode: velocity\n    startup_position_rad: 0.1",
        "startup_position_rad: 0.1\n    source_min_steps: 100",
        "startup_position_rad: 0.1\n    source_max_steps: 4000",
        "startup_position_rad: .nan",
        "startup_position_rad: .inf",
        "startup_position_rad: abc",
        "startup_position_rad: 1.0\n    command_min_steps: 1000\n    command_max_steps: 2100",
        "startup_position_rad: -1.0\n    command_min_steps: 1000\n    command_max_steps: 3000",
        "startup_position_rad: 5.0",
    ],
)
def test_config_invalid_startup_position_rejected(tmp_path: Path, extra: str, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level("WARNING"):
        assert load(tmp_path, extra) is None
    assert "startup_position_rad" in caplog.text


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    return module


@pytest.mark.parametrize(("failed", "expected"), [([], {1, 2}), ([1], {2})])
def test_bridge_queues_goals_only_for_torqued_servos(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch, failed: list[int], expected: set[int]
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    monkeypatch.setattr(bridge_module, "_wait_for_servos", MagicMock())
    monkeypatch.setattr(bridge_module, "read_all_registers", lambda *_a: {})
    monkeypatch.setattr(bridge_module, "write_register", MagicMock())
    monkeypatch.setattr(bridge_module, "set_startup_torque_state", lambda **_kw: failed)
    monkeypatch.setattr(bridge_module, "read_register_raw", lambda *_a: 2048)
    cycle_class = MagicMock()
    monkeypatch.setattr(bridge_module, "BridgeCycle", cycle_class)

    def spin_once(**_kw: float) -> None:
        fake.alive = False
        raise context_invalid()

    bridge_module.SingleThreadedExecutor.return_value.spin_once.side_effect = spin_once
    config_mod = importlib.import_module("feetech_servos.config")
    config = config_mod.BridgeConfig(
        namespace="head", joints=[PAN, TILT, PLAIN], device="/dev/ttyUSB0", enable_torque_on_start=True
    )
    bridge_module.run_bridge(config)

    joints, torque_ids, _log = cycle_class.return_value.queue_startup_goals.call_args.args
    assert {j.id for j in joints} == {1, 2, 3}
    assert set(torque_ids) == expected | ({3} if 3 not in failed else set())
