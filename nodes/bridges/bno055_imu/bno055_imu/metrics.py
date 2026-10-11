"""Prometheus metrics of the BNO055 node, registered once in the default registry.

The node loop calls the record_* helpers at the real code points; the helpers are plain functions so they can be
tested without hardware or ROS2.
"""

import time

from prometheus_client import Counter, Gauge, Histogram

from .recovery import Action, Decision

CALIBRATION_SUBSYSTEMS = ("sys", "gyro", "accel", "mag")
GYRO_AXES = ("x", "y", "z")

READ_ERRORS = Counter("imu_read_errors_total", "BNO055 I2C read errors (RuntimeError/OSError)")
SOFT_RESTORES = Counter("imu_soft_restores_total", "Soft operation-mode restores issued by the recovery policy")
REINITS = Counter("imu_reinits_total", "Full BNO055 re-initialisations by escalation reason", ["reason"])
INIT_ATTEMPTS = Counter("imu_init_attempts_total", "BNO055 driver creation attempts (start-up and re-init)")
CALIBRATION_LEVEL = Gauge("imu_calibration_level", "BNO055 calibration level 0-3 per subsystem", ["subsystem"])
PUBLISHED = Counter("imu_published_total", "sensor_msgs/Imu samples published")
MODE = Gauge("imu_mode", "BNO055 OPR_MODE register value as last read")
SECONDS_SINCE_PUBLISH = Gauge("imu_seconds_since_publish", "Seconds since the last published sample")

GYRO_BIAS = Gauge("imu_gyro_bias_rad_s", "Learned gyro zero-rate bias subtracted from angular_velocity", ["axis"])
GYRO_BIAS_LEARNING = Gauge("imu_gyro_bias_learning", "1 while the stationary gyro bias estimator is learning, else 0")

CYCLE_SECONDS = Histogram(
    "imu_cycle_seconds",
    "Work time of one IMU cycle (burst read, decode, publish, periodic calibration)",
    buckets=(0.002, 0.005, 0.01, 0.02, 0.03, 0.05, 0.1),
)
CYCLE_OVERRUNS = Counter("imu_cycle_overruns_total", "IMU cycles that finished after their next slot (slots skipped)")

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


def record_cycle(seconds: float) -> None:
    """Observe the work time of one cycle.

    Args:
        seconds (float): Read plus publish work time.
    """
    CYCLE_SECONDS.observe(seconds)


def record_overrun() -> None:
    """Count a cycle that overran its slot."""
    CYCLE_OVERRUNS.inc()


def record_gyro_bias(bias: tuple[float, float, float], learning: bool) -> None:
    """Export the learned gyro bias and whether the estimator is learning right now.

    Args:
        bias (tuple[float, float, float]): Bias (x, y, z) in rad/s.
        learning (bool): True when the last gyro sample was used to update the bias.
    """
    for axis, value in zip(GYRO_AXES, bias, strict=True):
        GYRO_BIAS.labels(axis).set(value)
    GYRO_BIAS_LEARNING.set(1 if learning else 0)
