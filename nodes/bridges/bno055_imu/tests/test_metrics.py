"""Unit tests for the BNO055 Prometheus metrics helpers (names, labels and the recovery policy hooks)."""

import time

import pytest
from prometheus_client import REGISTRY

from bno055_imu import metrics
from bno055_imu.recovery import Action, Decision


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    """Return the current sample value, 0.0 when the series does not exist yet."""
    value = REGISTRY.get_sample_value(name, labels or {})
    return 0.0 if value is None else value


def test_soft_restore_decision_counts_soft_restore() -> None:
    before = sample("imu_soft_restores_total")
    metrics.record_decision(Decision(Action.SOFT_RESTORE))
    assert sample("imu_soft_restores_total") == before + 1


def test_continue_decision_counts_nothing() -> None:
    before = sample("imu_soft_restores_total")
    metrics.record_decision(Decision(Action.CONTINUE))
    assert sample("imu_soft_restores_total") == before


@pytest.mark.parametrize("reason", ["watchdog", "i2c_hard_errors", "soft_exhausted"])
def test_full_reinit_decision_counts_by_reason_code(reason: str) -> None:
    labels = {"reason": reason}
    before = sample("imu_reinits_total", labels)
    metrics.record_decision(Decision(Action.FULL_REINIT, "free text 12 s", code=reason))
    assert sample("imu_reinits_total", labels) == before + 1


def test_calibration_levels_per_subsystem() -> None:
    metrics.record_calibration((3, 2, 1, 0))
    assert sample("imu_calibration_level", {"subsystem": "sys"}) == 3
    assert sample("imu_calibration_level", {"subsystem": "gyro"}) == 2
    assert sample("imu_calibration_level", {"subsystem": "accel"}) == 1
    assert sample("imu_calibration_level", {"subsystem": "mag"}) == 0


def test_incomplete_calibration_status_is_not_exported() -> None:
    metrics.record_calibration((1, 1, 1, 1))
    metrics.record_calibration((None, 3, 3, 3))
    metrics.record_calibration(None)
    assert sample("imu_calibration_level", {"subsystem": "sys"}) == 1


def test_publish_counts_and_resets_seconds_since_publish() -> None:
    before = sample("imu_published_total")
    metrics.record_publish()
    assert sample("imu_published_total") == before + 1
    assert sample("imu_seconds_since_publish") < 1.0


def test_seconds_since_publish_grows() -> None:
    metrics.record_publish()
    metrics.last_publish_s -= 5.0
    assert sample("imu_seconds_since_publish") >= 5.0
    assert metrics.last_publish_s <= time.monotonic()


def test_read_error_init_attempt_and_mode() -> None:
    errors, attempts = sample("imu_read_errors_total"), sample("imu_init_attempts_total")
    metrics.record_read_error()
    metrics.record_init_attempt()
    metrics.record_mode(0x08)
    assert sample("imu_read_errors_total") == errors + 1
    assert sample("imu_init_attempts_total") == attempts + 1
    assert sample("imu_mode") == 0x08
