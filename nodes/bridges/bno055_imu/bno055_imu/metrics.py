"""Prometheus metrics of the BNO055 node, registered once in the default registry.

The node loop calls the record_* helpers at the real code points; the helpers are plain functions so they can be
tested without hardware or ROS2.
"""

import time

from prometheus_client import Counter, Gauge

from .recovery import Action, Decision

CALIBRATION_SUBSYSTEMS = ("sys", "gyro", "accel", "mag")

READ_ERRORS = Counter("imu_read_errors_total", "BNO055 I2C read errors (RuntimeError/OSError)")
SOFT_RESTORES = Counter("imu_soft_restores_total", "Soft operation-mode restores issued by the recovery policy")
REINITS = Counter("imu_reinits_total", "Full BNO055 re-initialisations by escalation reason", ["reason"])
INIT_ATTEMPTS = Counter("imu_init_attempts_total", "BNO055 driver creation attempts (start-up and re-init)")
CALIBRATION_LEVEL = Gauge("imu_calibration_level", "BNO055 calibration level 0-3 per subsystem", ["subsystem"])
PUBLISHED = Counter("imu_published_total", "sensor_msgs/Imu samples published")
MODE = Gauge("imu_mode", "BNO055 OPR_MODE register value as last read")
SECONDS_SINCE_PUBLISH = Gauge("imu_seconds_since_publish", "Seconds since the last published sample")

last_publish_s: float = time.monotonic()
SECONDS_SINCE_PUBLISH.set_function(lambda: time.monotonic() - last_publish_s)


def record_decision(decision: Decision) -> None:
    """Count a recovery policy verdict.

    Args:
        decision (Decision): Verdict returned by RecoveryPolicy.decide; CONTINUE counts nothing.
    """
    if decision.action is Action.SOFT_RESTORE:
        SOFT_RESTORES.inc()
    elif decision.action is Action.FULL_REINIT:
        REINITS.labels(decision.code).inc()


def record_calibration(status: tuple | None) -> None:
    """Export the calibration levels; an unavailable or incomplete status leaves the gauges untouched.

    Args:
        status (tuple | None): (sys, gyro, accel, mag), each 0-3, as read from the driver.
    """
    if not status or len(status) < len(CALIBRATION_SUBSYSTEMS) or any(v is None for v in status[:4]):
        return
    for name, level in zip(CALIBRATION_SUBSYSTEMS, status, strict=False):
        CALIBRATION_LEVEL.labels(name).set(int(level))


def record_publish() -> None:
    """Count a published sample and restart the seconds-since-publish clock."""
    global last_publish_s
    PUBLISHED.inc()
    last_publish_s = time.monotonic()


def record_read_error() -> None:
    """Count an I2C read error."""
    READ_ERRORS.inc()


def record_init_attempt() -> None:
    """Count a driver creation attempt."""
    INIT_ATTEMPTS.inc()


def record_mode(value: int) -> None:
    """Export the operation mode register value.

    Args:
        value (int): OPR_MODE register value.
    """
    MODE.set(value)
