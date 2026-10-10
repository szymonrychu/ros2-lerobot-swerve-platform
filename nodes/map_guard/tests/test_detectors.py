"""Tests for the lifted / carried / rotated detectors: sustained conditions trigger, spikes and commanded motion do not."""

import math
from collections.abc import Callable

import pytest

from map_guard.config import MapGuardConfig
from map_guard.detectors import (
    REASON_CARRIED,
    REASON_LIFTED,
    REASON_ROTATED,
    Disturbances,
    angle_diff_deg,
    odom_speed,
    roll_pitch_deg,
    world_vector,
)

DT = 0.05  # 20 Hz simulation step
LEVEL = (0.0, 0.0, 0.0, 1.0)
STILL = (0.0, 0.0, 0.0)
GRAVITY = 9.80665


def tilt_quaternion(roll_deg: float) -> tuple[float, float, float, float]:
    """Quaternion of a pure roll."""
    half = math.radians(roll_deg) / 2
    return (math.sin(half), 0.0, 0.0, math.cos(half))


Inputs = Callable[[float], dict]


def run(det: Disturbances, start: float, end: float, inputs: Inputs) -> list[tuple[float, frozenset[str]]]:
    """Feed every input stream at 20 Hz in [start, end) and evaluate after each step.

    inputs(t) returns optional keys: orientation, gyro, accel (IMU), lidar (speed), wheel (speed, yaw rate),
    cmd (linear, angular). Missing keys mean the stream is silent at that step.
    """
    out = []
    steps = int(round((end - start) / DT))
    for i in range(steps):
        t = start + i * DT
        values = inputs(t)
        det.on_imu(t, values.get("orientation", LEVEL), values.get("gyro", STILL), values.get("accel", STILL))
        if "lidar" in values:
            det.on_lidar_odom(t, values["lidar"])
        det.on_wheel_odom(t, *values.get("wheel", (0.0, 0.0)))
        if "cmd" in values:
            det.on_cmd(t, *values["cmd"])
        out.append((t, det.evaluate(t).reasons))
    return out


def first_fire(results: list[tuple[float, frozenset[str]]], reason: str) -> float | None:
    """Time of the first evaluation reporting reason, or None."""
    return next((t for t, reasons in results if reason in reasons), None)


def level_then(det: Disturbances, duration: float = 4.0) -> float:
    """Feed a level, still robot for duration seconds (establishes the tilt baseline); returns the end time."""
    run(det, 0.0, duration, lambda t: {"lidar": 0.0})
    return duration


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def test_roll_pitch_deg() -> None:
    roll, pitch = roll_pitch_deg(tilt_quaternion(20.0))
    assert roll == pytest.approx(20.0)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_angle_diff_wraps() -> None:
    assert angle_diff_deg(179.0, -179.0) == pytest.approx(-2.0)
    assert angle_diff_deg(10.0, 0.0) == pytest.approx(10.0)


def test_world_vector_rotates_into_the_world_frame() -> None:
    # Rolled 90 deg: the sensor y axis points up.
    v = world_vector(tilt_quaternion(90.0), (0.0, 1.0, 0.0))
    assert v == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_odom_speed_uses_the_larger_of_twist_and_pose_delta() -> None:
    # rf2o's twist is laser-frame x only: a sideways carry shows up only in the pose difference.
    assert odom_speed((0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.05, 0.5)) == pytest.approx(0.1)
    assert odom_speed((0.3, 0.0), (0.0, 0.0, 0.0), (0.0, 0.05, 0.5)) == pytest.approx(0.3)
    assert odom_speed((0.0, 0.0), None, (0.0, 0.05, 0.5)) == 0.0
    assert odom_speed((0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 1.0)) == 0.0  # no time step: twist only


# ---------------------------------------------------------------------------
# LIFTED by tilt
# ---------------------------------------------------------------------------


def test_sustained_tilt_triggers_lifted_after_lift_s() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 2.0, lambda t: {"orientation": tilt_quaternion(20.0), "lidar": 0.0})
    fired = first_fire(results, REASON_LIFTED)
    assert fired is not None
    assert start + 1.0 - 1e-6 <= fired <= start + 1.0 + DT + 1e-6
    assert det.evaluate(start + 2.0).tilt_deg == pytest.approx(20.0, abs=0.01)


def test_tilt_spike_of_0_2_s_never_triggers() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(
        det,
        start,
        start + 3.0,
        lambda t: {"orientation": tilt_quaternion(25.0 if t < start + 0.2 else 0.0), "lidar": 0.0},
    )
    assert first_fire(results, REASON_LIFTED) is None


def test_no_tilt_trigger_without_a_level_baseline() -> None:
    det = Disturbances(MapGuardConfig())
    results = run(det, 0.0, 0.5, lambda t: {"orientation": tilt_quaternion(30.0)})
    assert first_fire(results, REASON_LIFTED) is None


def test_small_tilt_below_threshold_never_triggers() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"orientation": tilt_quaternion(8.0), "lidar": 0.0})
    assert first_fire(results, REASON_LIFTED) is None


def test_baseline_follows_a_level_but_inclined_floor() -> None:
    det = Disturbances(MapGuardConfig())
    # Standing still on a 5 deg slope: the baseline is that slope, so a further 10 deg is under the 12 deg threshold.
    run(det, 0.0, 4.0, lambda t: {"orientation": tilt_quaternion(5.0), "lidar": 0.0})
    results = run(det, 4.0, 7.0, lambda t: {"orientation": tilt_quaternion(15.0), "lidar": 0.0})
    assert first_fire(results, REASON_LIFTED) is None


# ---------------------------------------------------------------------------
# LIFTED by vertical acceleration
# ---------------------------------------------------------------------------


def test_vertical_acceleration_then_tilt_triggers_lifted() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)

    def inputs(t: float) -> dict:
        if t < start + 0.6:
            return {"accel": (0.0, 0.0, 3.0), "lidar": 0.0}
        return {"orientation": tilt_quaternion(6.0), "lidar": 0.0}

    results = run(det, start, start + 1.0, inputs)
    fired = first_fire(results, REASON_LIFTED)
    assert fired is not None and fired < start + 1.0


def test_vertical_acceleration_then_lidar_motion_triggers_lifted() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)

    def inputs(t: float) -> dict:
        if t < start + 0.6:
            return {"accel": (0.0, 0.0, -2.5), "lidar": 0.0}
        return {"lidar": 0.15}

    results = run(det, start, start + 0.8, inputs)
    assert first_fire(results, REASON_LIFTED) is not None


def test_vertical_acceleration_alone_never_triggers() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"accel": (0.0, 0.0, 3.0), "lidar": 0.0})
    assert first_fire(results, REASON_LIFTED) is None


def test_vertical_acceleration_spike_then_tilt_never_triggers() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)

    def inputs(t: float) -> dict:
        if t < start + 0.2:
            return {"accel": (0.0, 0.0, 4.0), "lidar": 0.0}
        return {"orientation": tilt_quaternion(6.0), "lidar": 0.0}

    results = run(det, start, start + 0.9, inputs)
    assert first_fire(results, REASON_LIFTED) is None


def test_gravity_included_fallback_samples_are_ignored() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)

    def inputs(t: float) -> dict:
        if t < start + 1.0:
            return {"accel": (0.0, 0.0, GRAVITY), "lidar": 0.0}
        return {"orientation": tilt_quaternion(6.0), "lidar": 0.0}

    results = run(det, start, start + 1.5, inputs)
    assert first_fire(results, REASON_LIFTED) is None


# ---------------------------------------------------------------------------
# CARRIED / PUSHED
# ---------------------------------------------------------------------------


def test_sustained_unexplained_lidar_motion_triggers_carried() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 1.5, lambda t: {"lidar": 0.2})
    fired = first_fire(results, REASON_CARRIED)
    assert fired is not None
    assert start + 1.0 - 1e-6 <= fired <= start + 1.0 + DT + 1e-6
    assert det.evaluate(start + 1.5).unexplained_speed_mps == pytest.approx(0.2)


def test_lidar_motion_spike_never_triggers_carried() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"lidar": 0.3 if t < start + 0.2 else 0.0})
    assert first_fire(results, REASON_CARRIED) is None


def test_commanded_motion_suppresses_carried() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    # Wheels report still (e.g. slipping) but a drive command is active: explained.
    results = run(det, start, start + 3.0, lambda t: {"lidar": 0.2, "cmd": (0.2, 0.0)})
    assert first_fire(results, REASON_CARRIED) is None
    assert det.evaluate(start + 3.0).unexplained_speed_mps == 0.0


def test_command_older_than_cmd_timeout_does_not_suppress() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 2.0, lambda t: {"lidar": 0.2, **({"cmd": (0.2, 0.0)} if t < start + 0.1 else {})})
    assert first_fire(results, REASON_CARRIED) is not None


def test_zero_command_does_not_suppress() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 2.0, lambda t: {"lidar": 0.2, "cmd": (0.0, 0.0)})
    assert first_fire(results, REASON_CARRIED) is not None


def test_moving_wheels_suppress_carried() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"lidar": 0.2, "wheel": (0.18, 0.0)})
    assert first_fire(results, REASON_CARRIED) is None


def test_stale_lidar_odometry_never_triggers_carried() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    det.on_lidar_odom(start, 0.3)
    results = run(det, start, start + 3.0, lambda t: {})
    assert first_fire(results, REASON_CARRIED) is None
    assert det.evaluate(start + 3.0).unexplained_speed_mps is None


# ---------------------------------------------------------------------------
# ROTATED
# ---------------------------------------------------------------------------


def test_sustained_unexplained_yaw_rate_triggers_rotated() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 1.5, lambda t: {"gyro": (0.0, 0.0, 0.5), "lidar": 0.0})
    fired = first_fire(results, REASON_ROTATED)
    assert fired is not None
    assert start + 0.8 - 1e-6 <= fired <= start + 0.8 + DT + 1e-6


def test_yaw_rate_spike_never_triggers_rotated() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"gyro": (0.0, 0.0, 1.0 if t < start + 0.2 else 0.0)})
    assert first_fire(results, REASON_ROTATED) is None


def test_commanded_rotation_suppresses_rotated() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"gyro": (0.0, 0.0, 0.5), "cmd": (0.0, 0.5)})
    assert first_fire(results, REASON_ROTATED) is None


def test_wheel_yaw_rate_suppresses_rotated() -> None:
    det = Disturbances(MapGuardConfig())
    start = level_then(det)
    results = run(det, start, start + 3.0, lambda t: {"gyro": (0.0, 0.0, 0.5), "wheel": (0.0, 0.45)})
    assert first_fire(results, REASON_ROTATED) is None


def test_quiet_robot_reports_nothing() -> None:
    det = Disturbances(MapGuardConfig())
    results = run(det, 0.0, 6.0, lambda t: {"lidar": 0.01})
    assert all(not reasons for _, reasons in results)
