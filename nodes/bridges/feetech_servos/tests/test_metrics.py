"""Tests for the feetech_servos Prometheus metrics: metrics_port config and updates at each code point."""

import json
from pathlib import Path

import pytest
from prometheus_client import REGISTRY

from feetech_servos.bridge_cycle import BridgeCycle, end_cycle
from feetech_servos.config import BridgeConfig, JointEntry, load_config
from feetech_servos.metrics import init_joints, record_read_cycle, record_register_dump
from feetech_servos.registers import STS_GOAL_POSITION_L, get_register_entry_by_name
from feetech_servos.set_register import apply_set_register

LOAD_RAW_NEGATIVE = 0x400 | 120  # direction bit set, magnitude 120 -> -120


def value(name: str, labels: dict[str, str] | None = None) -> float | None:
    return REGISTRY.get_sample_value(name, labels or {})


def make_config() -> BridgeConfig:
    return BridgeConfig(namespace="follower", joints=[JointEntry("gripper", 6), JointEntry("wrist_roll", 5)])


class RecordingServo:
    """Servo whose writes succeed or fail on demand."""

    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.writes: list[tuple[int, int, int]] = []

    def write1ByteTxRx(self, sts_id: int, address: int, val: int) -> tuple[int, int]:  # noqa: N802 - st3215 API
        if self.fail:
            return (-1001, 0)
        self.writes.append((sts_id, address, val))
        return (0, 0)

    write2ByteTxRx = write1ByteTxRx  # noqa: N815 - st3215 API


def test_metrics_port_default_and_parsed(tmp_path: Path) -> None:
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\njoint_names:\n  - name: gripper\n    id: 6\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.metrics_port is None
    p.write_text("namespace: follower\nmetrics_port: 19101\njoint_names:\n  - name: gripper\n    id: 6\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.metrics_port == 19101


def test_read_cycle_counts_failures_per_joint_and_bus_up() -> None:
    joints = [("m_gripper", 6), ("m_wrist", 5)]
    init_joints([name for name, _sid in joints])
    record_read_cycle(joints, {6: (1, 2), 5: (3, 4)})
    assert value("servo_bus_up") == 1.0
    assert value("servo_bus_read_failures_total", {"joint": "m_gripper"}) == 0.0
    record_read_cycle(joints, {6: (1, 2)})
    assert value("servo_bus_up") == 1.0  # one servo still answered
    assert value("servo_bus_read_failures_total", {"joint": "m_wrist"}) == 1.0
    assert value("servo_bus_read_failures_total", {"joint": "m_gripper"}) == 0.0
    record_read_cycle(joints, {})
    assert value("servo_bus_up") == 0.0
    assert value("servo_bus_read_failures_total", {"joint": "m_wrist"}) == 2.0
    assert value("servo_bus_read_failures_total", {"joint": "m_gripper"}) == 1.0


def test_register_dump_sets_per_joint_gauges() -> None:
    dump = {
        "d_inv": {
            "present_load": LOAD_RAW_NEGATIVE,
            "present_temperature": 41,
            "present_voltage": 74,
            "torque_enable": 1,
        },
        "d_norm": {
            "present_load": LOAD_RAW_NEGATIVE,
            "present_temperature": 38,
            "present_voltage": 73,
            "torque_enable": 0,
        },
        "d_partial": {"present_temperature": 30},
    }
    record_register_dump(dump, {"d_inv": True, "d_norm": False})
    assert value("servo_present_load", {"joint": "d_norm"}) == -120.0
    assert value("servo_present_load", {"joint": "d_inv"}) == 120.0  # inverted joint flips the sign
    assert value("servo_temperature_celsius", {"joint": "d_inv"}) == 41.0
    assert value("servo_voltage_volts", {"joint": "d_inv"}) == pytest.approx(7.4)
    assert value("servo_torque_enabled", {"joint": "d_inv"}) == 1.0
    assert value("servo_torque_enabled", {"joint": "d_norm"}) == 0.0
    assert value("servo_temperature_celsius", {"joint": "d_partial"}) == 30.0
    # registers that were not read stay unset instead of defaulting to 0
    assert value("servo_present_load", {"joint": "d_partial"}) is None
    assert value("servo_voltage_volts", {"joint": "d_partial"}) is None


def test_end_cycle_observes_duration_and_returns_sleep() -> None:
    before = value("servo_cycle_duration_seconds_count") or 0.0
    assert end_cycle(0.01, 0.004) == pytest.approx(0.006)
    assert end_cycle(0.01, 0.02) == 0.0
    assert value("servo_cycle_duration_seconds_count") == before + 2
    assert value("servo_cycle_duration_seconds_bucket", {"le": "0.005"}) is not None
    assert value("servo_cycle_duration_seconds_sum") is not None


def test_commands_total_counts_written_targets() -> None:
    servo = RecordingServo()
    cycle = BridgeCycle(
        servo=servo,
        goal_entry=get_register_entry_by_name("goal_position"),
        speed_goal_entry=get_register_entry_by_name("goal_speed"),
        velocity_joints=[],
        command_limits={},
        last_written={},
        velocity_command_timeout_s=0.3,
    )
    group = make_config().groups[0]
    before = value("servo_commands_total") or 0.0
    cycle.handle_command(group, ["gripper", "wrist_roll"], [0.1, 0.2], [])
    assert cycle.write_pending_commands() == 2
    assert value("servo_commands_total") == before + 2
    assert cycle.write_pending_commands() == 0
    assert value("servo_commands_total") == before + 2
    assert any(addr == STS_GOAL_POSITION_L for _sid, addr, _v in servo.writes)


@pytest.mark.parametrize(
    ("payload", "reason"),
    [
        ("{not json", "invalid_json"),
        (json.dumps({"joint_name": "gripper"}), "missing_field"),
        (json.dumps({"joint_name": "gripper", "register": "present_load", "value": 1}), "unknown_register"),
        (json.dumps({"joint_name": "nope", "register": "torque_enable", "value": 1}), "unknown_joint"),
        (json.dumps({"joint_name": "gripper", "register": "torque_enable", "value": "x"}), "bad_value"),
        (json.dumps({"joint_name": "gripper", "register": "p_coefficient", "value": 5}), "eprom_rejected"),
    ],
)
def test_set_register_failure_reasons(payload: str, reason: str) -> None:
    labels = {"reason": reason}
    before = value("servo_set_register_failures_total", labels) or 0.0
    warnings: list[str] = []
    assert apply_set_register(RecordingServo(), make_config(), {}, payload, warnings.append) is False
    assert value("servo_set_register_failures_total", labels) == before + 1
    assert len(warnings) == 1


def test_set_register_write_failed_and_success() -> None:
    payload = json.dumps({"joint_name": "gripper", "register": "torque_enable", "value": 1})
    labels = {"reason": "write_failed"}
    before = value("servo_set_register_failures_total", labels) or 0.0
    assert apply_set_register(RecordingServo(fail=True), make_config(), {}, payload, lambda _m: None) is False
    assert value("servo_set_register_failures_total", labels) == before + 1
    ok_servo = RecordingServo()
    assert apply_set_register(ok_servo, make_config(), {}, payload, lambda _m: None) is True
    assert ok_servo.writes
    assert value("servo_set_register_failures_total", labels) == before + 1


def test_set_register_without_bus_counts_no_bus() -> None:
    labels = {"reason": "no_bus"}
    before = value("servo_set_register_failures_total", labels) or 0.0
    assert apply_set_register(None, make_config(), {}, "{}", lambda _m: None) is False
    assert value("servo_set_register_failures_total", labels) == before + 1
