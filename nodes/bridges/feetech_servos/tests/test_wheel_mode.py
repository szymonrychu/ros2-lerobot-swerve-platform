"""Unit tests for velocity (wheel mode) encoding, inverted joints, sync read and the velocity watchdog."""

import math

import pytest

from feetech_servos.command_mapping import (
    SPEED_SIGN_BIT,
    STEP_CENTER,
    STEPS_PER_RADIAN,
    is_finite_command,
    map_position_to_steps,
    position_to_raw_steps,
    speed_register_to_velocity,
    steps_to_radians,
    velocity_to_speed_register,
)
from feetech_servos.sync_read import PRESENT_POSITION_ADDRESS, read_positions_and_speeds
from feetech_servos.velocity_watchdog import expired_velocity_joints

# --- velocity_to_speed_register ---


def test_velocity_zero_encodes_zero() -> None:
    assert velocity_to_speed_register(0.0, max_velocity_rad_s=4.0) == 0


def test_velocity_positive_encodes_steps_per_second() -> None:
    assert velocity_to_speed_register(1.0, max_velocity_rad_s=4.0) == round(STEPS_PER_RADIAN)


def test_velocity_negative_sets_sign_bit() -> None:
    raw = velocity_to_speed_register(-1.0, max_velocity_rad_s=4.0)
    assert raw & SPEED_SIGN_BIT
    assert raw & ~SPEED_SIGN_BIT == round(STEPS_PER_RADIAN)


def test_velocity_inverted_flips_sign() -> None:
    raw = velocity_to_speed_register(1.0, max_velocity_rad_s=4.0, inverted=True)
    assert raw == SPEED_SIGN_BIT | round(STEPS_PER_RADIAN)


def test_velocity_clamped_to_max() -> None:
    raw = velocity_to_speed_register(-100.0, max_velocity_rad_s=2.0)
    assert raw == SPEED_SIGN_BIT | round(2.0 * STEPS_PER_RADIAN)


# --- non-finite (NaN / inf) command values ---

NON_FINITE = [float("nan"), float("inf"), float("-inf")]


@pytest.mark.parametrize("value", NON_FINITE)
def test_is_finite_command_rejects_non_finite(value: float) -> None:
    assert is_finite_command(value) is False


def test_is_finite_command_accepts_numbers() -> None:
    assert is_finite_command(0.0) is True
    assert is_finite_command(-3.5) is True


@pytest.mark.parametrize("value", NON_FINITE)
def test_velocity_to_speed_register_refuses_non_finite(value: float) -> None:
    # Unchecked, NaN used to clamp to full speed (min(limit, nan) == limit).
    with pytest.raises(ValueError):
        velocity_to_speed_register(value, max_velocity_rad_s=4.0)


@pytest.mark.parametrize("value", NON_FINITE)
def test_position_to_raw_steps_refuses_non_finite(value: float) -> None:
    with pytest.raises(ValueError):
        position_to_raw_steps(value)


@pytest.mark.parametrize("value", NON_FINITE)
def test_map_position_to_steps_refuses_non_finite(value: float) -> None:
    with pytest.raises(ValueError):
        map_position_to_steps(value, 0, 4095, 1000, 3000)


# --- speed_register_to_velocity ---


def test_speed_register_decodes_positive() -> None:
    assert speed_register_to_velocity(round(STEPS_PER_RADIAN)) == pytest.approx(1.0, abs=1e-3)


def test_speed_register_decodes_sign_bit_as_negative() -> None:
    raw = SPEED_SIGN_BIT | round(STEPS_PER_RADIAN)
    assert speed_register_to_velocity(raw) == pytest.approx(-1.0, abs=1e-3)


def test_speed_register_inverted() -> None:
    assert speed_register_to_velocity(round(STEPS_PER_RADIAN), inverted=True) == pytest.approx(-1.0, abs=1e-3)


def test_speed_roundtrip() -> None:
    for v in (-3.0, -0.5, 0.0, 0.25, 2.0):
        raw = velocity_to_speed_register(v, max_velocity_rad_s=4.0, inverted=True)
        assert speed_register_to_velocity(raw, inverted=True) == pytest.approx(v, abs=2e-3)


# --- inverted positions ---


def test_steps_to_radians_inverted() -> None:
    ticks = STEP_CENTER + round(STEPS_PER_RADIAN * 0.5)
    assert steps_to_radians(ticks, inverted=True) == pytest.approx(-0.5, abs=2e-3)


def test_position_to_raw_steps_inverted() -> None:
    assert position_to_raw_steps(math.pi / 2, inverted=True) == STEP_CENTER - 1024


def test_position_roundtrip_inverted() -> None:
    steps = position_to_raw_steps(0.3, inverted=True)
    assert steps_to_radians(steps, inverted=True) == pytest.approx(0.3, abs=2e-3)


# --- read_positions_and_speeds ---


class FakeGroupSyncRead:
    """Mimics st3215.GroupSyncRead: data_dict[id] = [error, pos_lo, pos_hi, spd_lo, spd_hi].

    getData raises AttributeError like the real library (it calls a non-existent scs_makeword).
    """

    def __init__(self, replies: dict[int, tuple[int, int]], comm_result: int = 0) -> None:
        self.replies = replies
        self.comm_result = comm_result
        self.params: list[int] = []
        self.data_dict: dict[int, list[int]] = {}

    def addParam(self, sts_id: int) -> bool:  # noqa: N802 - st3215 API
        self.params.append(sts_id)
        self.data_dict[sts_id] = []
        return True

    def txRxPacket(self) -> int:  # noqa: N802 - st3215 API
        for sid, (pos, spd) in self.replies.items():
            self.data_dict[sid] = [0, pos & 0xFF, pos >> 8, spd & 0xFF, spd >> 8]
        return self.comm_result

    def isAvailable(self, sts_id: int, address: int, data_length: int) -> tuple[bool, int]:  # noqa: N802
        assert address == PRESENT_POSITION_ADDRESS and data_length == 4
        return (len(self.data_dict.get(sts_id, [])) >= data_length + 1, 0)

    def getData(self, sts_id: int, address: int, data_length: int) -> int:  # noqa: N802 - st3215 API
        raise AttributeError("'ST3215' object has no attribute 'scs_makeword'")


def test_sync_read_returns_all_servos() -> None:
    fake = FakeGroupSyncRead({1: (2048, 0), 32: (100, 0x8010)})
    result = read_positions_and_speeds([1, 32], lambda: fake, fallback_read=lambda _sid: None)
    assert fake.params == [1, 32]
    assert result == {1: (2048, 0), 32: (100, 0x8010)}


def test_sync_read_falls_back_per_servo_for_missing_reply() -> None:
    fake = FakeGroupSyncRead({1: (2048, 0)})
    result = read_positions_and_speeds([1, 32], lambda: fake, fallback_read=lambda sid: (55, 7) if sid == 32 else None)
    assert result == {1: (2048, 0), 32: (55, 7)}


def test_sync_read_comm_failure_uses_fallback_and_omits_unreadable() -> None:
    fake = FakeGroupSyncRead({1: (2048, 0)}, comm_result=-1001)
    result = read_positions_and_speeds([1, 32], lambda: fake, fallback_read=lambda sid: (10, 0) if sid == 1 else None)
    assert result == {1: (10, 0)}


# --- expired_velocity_joints ---


def test_watchdog_reports_stale_moving_joints_only() -> None:
    last_cmd = {"fl": (1.0, 0.5), "fr": (1.0, 0.0), "rl": (9.9, 0.5)}
    assert expired_velocity_joints(last_cmd, now=10.0, timeout_s=0.3) == ["fl"]


def test_watchdog_nothing_expired_within_timeout() -> None:
    assert expired_velocity_joints({"fl": (9.8, -1.0)}, now=10.0, timeout_s=0.3) == []


# --- apply_velocity_command ---


def test_apply_velocity_command_records_only_successful_writes() -> None:
    from feetech_servos.velocity_watchdog import apply_velocity_command

    last: dict[int, tuple[float, float]] = {32: (1.0, 0.5)}
    assert apply_velocity_command(lambda: False, 32, 0.0, 10.0, last) is False
    assert last[32] == (1.0, 0.5)  # failed stop stays "moving" so the watchdog retries it
    assert expired_velocity_joints(last, now=10.0, timeout_s=0.3) == [32]
    assert apply_velocity_command(lambda: True, 32, 0.0, 10.1, last) is True
    assert last[32] == (10.1, 0.0)


def test_apply_received_command_refreshes_time_but_keeps_applied_velocity_on_failed_write() -> None:
    from feetech_servos.velocity_watchdog import apply_velocity_command

    last: dict[int, tuple[float, float]] = {32: (1.0, 0.5)}
    assert apply_velocity_command(lambda: False, 32, 2.0, 1.2, last, received=True) is False
    assert last[32] == (1.2, 0.5)  # command arrived (watchdog fed) but the servo still runs at 0.5
    assert expired_velocity_joints(last, now=1.4, timeout_s=0.3) == []
    assert expired_velocity_joints(last, now=1.6, timeout_s=0.3) == [32]


def test_apply_received_command_without_history_records_commanded_velocity_on_failed_write() -> None:
    from feetech_servos.velocity_watchdog import apply_velocity_command

    last: dict[int, tuple[float, float]] = {}
    assert apply_velocity_command(lambda: False, 32, 2.0, 1.0, last, received=True) is False
    assert last[32] == (1.0, 2.0)


# --- RegisterDumpScheduler ---


def test_register_dump_reads_one_servo_per_step_and_publishes_when_complete() -> None:
    from feetech_servos.register_dump import RegisterDumpScheduler

    reads: list[int] = []

    def read_all(sid: int) -> dict[str, int]:
        reads.append(sid)
        return {"id": sid}

    dump = RegisterDumpScheduler([("a", 1), ("b", 2)], interval_s=10.0)
    assert dump.step(0.0, read_all) is None  # starts a dump, reads servo 1
    assert reads == [1]
    assert dump.step(0.01, read_all) == {"a": {"id": 1}, "b": {"id": 2}}
    assert reads == [1, 2]
    assert dump.step(5.0, read_all) is None  # interval not elapsed: no reads
    assert reads == [1, 2]
    assert dump.step(10.5, read_all) is None
    assert reads == [1, 2, 1]


def test_register_dump_disabled_with_zero_interval() -> None:
    from feetech_servos.register_dump import RegisterDumpScheduler

    dump = RegisterDumpScheduler([("a", 1)], interval_s=0.0)
    assert dump.step(100.0, lambda _sid: {"x": 1}) is None
