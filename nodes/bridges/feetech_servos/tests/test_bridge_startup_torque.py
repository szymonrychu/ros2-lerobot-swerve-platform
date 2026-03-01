"""Unit tests for startup torque apply/verify behavior."""

from feetech_servos.startup_torque import set_startup_torque_state


def test_set_startup_torque_state_success() -> None:
    writes: list[tuple[int, int]] = []
    readbacks: dict[int, int] = {1: 0, 2: 0}

    def fake_write_register(sid: int, value: int) -> bool:
        writes.append((sid, value))
        return True

    def fake_read_register(sid: int) -> int:
        return readbacks[sid]

    failed = set_startup_torque_state(
        joint_ids=[1, 2],
        torque_value=0,
        write_once=fake_write_register,
        read_once=fake_read_register,
        attempts=2,
        verify_sleep_s=0.0,
    )

    assert failed == []
    assert writes == [(1, 0), (2, 0)]


def test_set_startup_torque_state_reports_failed_servo() -> None:
    attempts_per_sid: dict[int, int] = {1: 0, 2: 0}

    def fake_write_register(sid: int, _value: int) -> bool:
        attempts_per_sid[sid] += 1
        return True

    def fake_read_register(sid: int) -> int:
        # Servo 1 verifies, servo 2 never reaches desired value.
        return 0 if sid == 1 else 1

    failed = set_startup_torque_state(
        joint_ids=[1, 2],
        torque_value=0,
        write_once=fake_write_register,
        read_once=fake_read_register,
        attempts=3,
        verify_sleep_s=0.0,
    )

    assert failed == [2]
    assert attempts_per_sid[1] == 1
    assert attempts_per_sid[2] == 3
