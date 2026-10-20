"""Tests for the set_register guard: torque_enable 0/1 only, range checks, released joints."""

import json
from typing import Any
from unittest.mock import MagicMock

import pytest
from feetech_servos.metrics import SET_REGISTER_FAILURES
from feetech_servos.set_register import apply_set_register

JOINT_ID = 3


def make_config() -> MagicMock:
    config = MagicMock()
    config.servo_id_for_joint_name.side_effect = lambda n: JOINT_ID if n == "elbow" else None
    return config


def make_servo() -> MagicMock:
    servo = MagicMock()
    servo.write1ByteTxRx.return_value = (0, 0)
    servo.write2ByteTxRx.return_value = (0, 0)
    return servo


def run(register: str, value: Any, released: set[int] | None = None) -> tuple[bool, MagicMock, list[str]]:
    servo = make_servo()
    warnings: list[str] = []
    payload = json.dumps({"joint_name": "elbow", "register": register, "value": value})
    ok = apply_set_register(servo, make_config(), {}, payload, warnings.append, released or set())
    return ok, servo, warnings


def failures(reason: str) -> float:
    return SET_REGISTER_FAILURES.labels(reason=reason)._value.get()


def writes(servo: MagicMock) -> int:
    return servo.write1ByteTxRx.call_count + servo.write2ByteTxRx.call_count


@pytest.mark.parametrize("value", [128, 2, -1, 255])
def test_torque_enable_other_than_0_1_rejected(value: int) -> None:
    before = failures("bad_value")
    ok, servo, _ = run("torque_enable", value)
    assert not ok
    assert writes(servo) == 0
    assert servo.UnLockEprom.call_count == 0
    assert failures("bad_value") == before + 1


@pytest.mark.parametrize("value", [0, 1])
def test_torque_enable_0_and_1_accepted(value: int) -> None:
    ok, servo, _ = run("torque_enable", value)
    assert ok
    assert writes(servo) == 1


@pytest.mark.parametrize(
    ("register", "value"),
    [("torque_limit", 65536), ("torque_limit", -1), ("goal_position", 70000), ("acceleration", 256), ("acceleration", -1)],
)
def test_out_of_range_rejected(register: str, value: int) -> None:
    before = failures("bad_value")
    ok, servo, _ = run(register, value)
    assert not ok
    assert writes(servo) == 0
    assert failures("bad_value") == before + 1


def test_range_edges_accepted() -> None:
    assert run("torque_limit", 65535)[0]
    assert run("acceleration", 255)[0]


@pytest.mark.parametrize(
    ("register", "value"),
    [("torque_enable", 1), ("goal_position", 2048), ("goal_speed", 100), ("goal_time", 10), ("acceleration", 5)],
)
def test_released_joint_refuses_motion_and_enable(register: str, value: int) -> None:
    before = failures("released")
    ok, servo, _ = run(register, value, {JOINT_ID})
    assert not ok
    assert writes(servo) == 0
    assert failures("released") == before + 1


@pytest.mark.parametrize(("register", "value"), [("torque_enable", 0), ("torque_limit", 500)])
def test_released_joint_accepts_torque_off_and_limit(register: str, value: int) -> None:
    ok, servo, _ = run(register, value, {JOINT_ID})
    assert ok
    assert writes(servo) == 1


def test_unreleased_joint_accepts_goal_position() -> None:
    ok, servo, _ = run("goal_position", 2048, {99})
    assert ok
    assert writes(servo) == 1
