"""Unit tests for the tilt guard: quaternion -> roll/pitch, level state machine, staleness, /tilt_guard/state JSON."""

import json
import math

import pytest
from prometheus_client import REGISTRY
from pydantic import ValidationError

from swerve_drive_controller.tilt_guard import (
    STATE_KEYS,
    TiltGuard,
    TiltGuardConfig,
    TiltLevel,
    drive_torque_payloads,
    quaternion_to_roll_pitch_deg,
)

IMU_PERIOD_S = 0.02  # 50 Hz BNO055 samples
DRIVE_JOINTS = ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]


def quaternion_from_roll_pitch_deg(roll_deg: float, pitch_deg: float, yaw_deg: float = 0.0) -> tuple:
    """Build an (x, y, z, w) quaternion from ZYX Euler angles in degrees (test helper)."""
    r, p, y = (math.radians(a) / 2.0 for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def feed(guard: TiltGuard, roll: float, pitch: float, start: float, duration: float) -> float:
    """Feed constant roll/pitch samples at 50 Hz for duration seconds; return the time after the last sample."""
    t = start
    end = start + duration
    while t <= end + 1e-9:
        guard.update(roll, pitch, t)
        t += IMU_PERIOD_S
    return t


# --- quaternion -> roll/pitch ---------------------------------------------------------------------------------------


def test_identity_quaternion_is_level() -> None:
    roll, pitch = quaternion_to_roll_pitch_deg(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0) and pitch == pytest.approx(0.0)


@pytest.mark.parametrize(("roll", "pitch", "yaw"), [(10.0, 0.0, 0.0), (0.0, -12.0, 0.0), (7.0, 5.0, 135.0)])
def test_quaternion_roundtrip_independent_of_yaw(roll: float, pitch: float, yaw: float) -> None:
    got_roll, got_pitch = quaternion_to_roll_pitch_deg(*quaternion_from_roll_pitch_deg(roll, pitch, yaw))
    assert got_roll == pytest.approx(roll, abs=1e-6)
    assert got_pitch == pytest.approx(pitch, abs=1e-6)


def test_unnormalised_quaternion_is_normalised() -> None:
    x, y, z, w = quaternion_from_roll_pitch_deg(6.0, -4.0)
    roll, pitch = quaternion_to_roll_pitch_deg(2 * x, 2 * y, 2 * z, 2 * w)
    assert roll == pytest.approx(6.0, abs=1e-6) and pitch == pytest.approx(-4.0, abs=1e-6)


@pytest.mark.parametrize("quat", [(0.0, 0.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0), (0.0, math.inf, 0.0, 1.0)])
def test_invalid_quaternion_returns_none(quat: tuple) -> None:
    assert quaternion_to_roll_pitch_deg(*quat) is None


def test_level_reference_is_subtracted() -> None:
    guard = TiltGuard(TiltGuardConfig(level_reference_roll_deg=2.0, level_reference_pitch_deg=-1.5))
    guard.update_from_quaternion(*quaternion_from_roll_pitch_deg(2.0, -1.5), now=1.0)
    assert guard.roll_deg == pytest.approx(0.0, abs=1e-6)
    assert guard.pitch_deg == pytest.approx(0.0, abs=1e-6)
    guard.update_from_quaternion(*quaternion_from_roll_pitch_deg(12.0, 3.5), now=1.02)
    assert guard.roll_deg == pytest.approx(10.0, abs=1e-6)
    assert guard.pitch_deg == pytest.approx(5.0, abs=1e-6)


def test_invalid_quaternion_is_not_a_sample() -> None:
    guard = TiltGuard(TiltGuardConfig())
    assert guard.update_from_quaternion(0.0, 0.0, 0.0, 0.0, now=1.0) is None
    assert guard.roll_deg is None and guard.last_sample_s is None


# --- rise with persistence ------------------------------------------------------------------------------------------


def test_rises_to_warning_only_after_persistence() -> None:
    guard = TiltGuard(TiltGuardConfig())
    feed(guard, 0.0, 13.0, 0.0, 0.4)
    assert guard.level is TiltLevel.OK
    feed(guard, 0.0, 13.0, 0.42, 0.1)
    assert guard.level is TiltLevel.WARNING


def test_short_spike_does_not_rise() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 0.0, 0.0, 0.0, 0.5)
    t = feed(guard, 30.0, 0.0, t, 0.2)
    t = feed(guard, 0.0, 0.0, t, 1.0)
    assert guard.level is TiltLevel.OK
    # A second short spike starts the persistence over instead of accumulating.
    t = feed(guard, 30.0, 0.0, t, 0.2)
    feed(guard, 0.0, 0.0, t, 0.1)
    assert guard.level is TiltLevel.OK


def test_jump_straight_to_error_after_persistence() -> None:
    guard = TiltGuard(TiltGuardConfig())
    transition = None
    t = 0.0
    while t <= 0.6:
        transition = guard.update(25.0, 0.0, t) or transition
        t += IMU_PERIOD_S
    assert guard.level is TiltLevel.ERROR
    assert guard.base_drive_inhibited
    assert transition is not None and transition.previous is TiltLevel.OK and transition.current is TiltLevel.ERROR
    assert transition.axis == "roll"


@pytest.mark.parametrize(
    ("roll", "pitch", "expected"),
    [
        (13.0, 0.0, TiltLevel.WARNING),  # roll warn 8 < 13 < error 15
        (0.0, 13.0, TiltLevel.WARNING),  # pitch warn 12 < 13
        (0.0, 18.0, TiltLevel.WARNING),  # pitch 18 < error 20: the longer axis tolerates more
        (18.0, 0.0, TiltLevel.ERROR),  # roll 18 > error 15
        (0.0, -21.0, TiltLevel.ERROR),  # negative pitch counts by magnitude
        (7.0, 11.0, TiltLevel.OK),
    ],
)
def test_asymmetric_axis_limits(roll: float, pitch: float, expected: TiltLevel) -> None:
    guard = TiltGuard(TiltGuardConfig())
    feed(guard, roll, pitch, 0.0, 1.0)
    assert guard.level is expected


def test_default_limits_are_larger_on_the_longer_pitch_axis() -> None:
    cfg = TiltGuardConfig()
    assert cfg.pitch_warning_deg > cfg.roll_warning_deg
    assert cfg.pitch_error_deg > cfg.roll_error_deg
    assert cfg.error_action == "brake"


# --- recovery: hysteresis and hold ----------------------------------------------------------------------------------


def test_error_recovers_only_below_hysteresis_after_hold() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 18.0, 0.0, 0.0, 1.0)
    assert guard.level is TiltLevel.ERROR
    # 14 deg is below the 15 deg error limit but inside the 2 deg hysteresis band: stays in error.
    t = feed(guard, 14.0, 0.0, t, 5.0)
    assert guard.level is TiltLevel.ERROR
    # 12 deg is below 15 - 2: warning after the 2 s hold, not before.
    t = feed(guard, 12.0, 0.0, t, 1.8)
    assert guard.level is TiltLevel.ERROR
    t = feed(guard, 12.0, 0.0, t, 0.3)
    assert guard.level is TiltLevel.WARNING
    assert not guard.base_drive_inhibited
    # 7 deg is inside the warning hysteresis band (8 - 2 = 6): stays warning.
    t = feed(guard, 7.0, 0.0, t, 3.0)
    assert guard.level is TiltLevel.WARNING
    t = feed(guard, 1.0, 0.0, t, 2.1)
    assert guard.level is TiltLevel.OK


def test_error_recovers_directly_to_ok_when_level() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 0.0, 25.0, 0.0, 1.0)
    assert guard.level is TiltLevel.ERROR
    feed(guard, 0.0, 0.0, t, 2.1)
    assert guard.level is TiltLevel.OK


def test_recovery_hold_restarts_when_tilt_returns() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 18.0, 0.0, 0.0, 1.0)
    t = feed(guard, 0.0, 0.0, t, 1.5)
    t = feed(guard, 14.0, 0.0, t, 0.1)
    t = feed(guard, 0.0, 0.0, t, 1.5)
    assert guard.level is TiltLevel.ERROR
    feed(guard, 0.0, 0.0, t, 0.6)
    assert guard.level is TiltLevel.OK


# --- stale IMU ------------------------------------------------------------------------------------------------------


def test_stale_imu_keeps_error_and_inhibition() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 18.0, 0.0, 0.0, 1.0)
    assert guard.base_drive_inhibited
    assert not guard.imu_stale(t)
    assert guard.imu_stale(t + 1.0)
    # No samples for 10 s: still error, the robot never drives on unknown attitude after an error.
    assert guard.level is TiltLevel.ERROR and guard.base_drive_inhibited


def test_stale_gap_restarts_persistence() -> None:
    guard = TiltGuard(TiltGuardConfig())
    feed(guard, 0.0, 15.0, 0.0, 0.4)
    # Next sample after a 2 s gap: the 0.4 s before the gap must not count towards the 0.5 s persistence.
    feed(guard, 0.0, 15.0, 2.4, 0.2)
    assert guard.level is TiltLevel.OK


def test_stale_gap_restarts_recovery_hold() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 18.0, 0.0, 0.0, 1.0)
    feed(guard, 0.0, 0.0, t, 1.5)
    feed(guard, 0.0, 0.0, t + 5.0, 0.6)
    assert guard.level is TiltLevel.ERROR


def test_no_sample_is_stale() -> None:
    guard = TiltGuard(TiltGuardConfig())
    assert guard.imu_stale(0.0)
    assert guard.state_dict(0.0, stamp_ns=0) is None


# --- disabled ---------------------------------------------------------------------------------------------------------


def test_disabled_guard_never_inhibits() -> None:
    guard = TiltGuard(TiltGuardConfig(enabled=False))
    feed(guard, 40.0, 40.0, 0.0, 2.0)
    assert guard.level is TiltLevel.OK and not guard.base_drive_inhibited


# --- config validation ----------------------------------------------------------------------------------------------


def test_config_rejects_warning_above_error() -> None:
    with pytest.raises(ValidationError):
        TiltGuardConfig(roll_warning_deg=16.0, roll_error_deg=15.0)


def test_config_rejects_hysteresis_wider_than_warning() -> None:
    with pytest.raises(ValidationError):
        TiltGuardConfig(hysteresis_deg=9.0)


def test_config_rejects_unknown_error_action_and_keys() -> None:
    with pytest.raises(ValidationError):
        TiltGuardConfig(error_action="coast")
    with pytest.raises(ValidationError):
        TiltGuardConfig.model_validate({"roll_warn": 5.0})


# --- state JSON (contract) ------------------------------------------------------------------------------------------


def test_state_json_matches_contract() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 18.0, -3.0, 0.0, 1.0)
    state = guard.state_dict(t, stamp_ns=123)
    assert state is not None
    assert tuple(state) == STATE_KEYS
    assert state["level"] == "error"
    assert state["level_code"] == 2
    assert state["roll_deg"] == pytest.approx(18.0)
    assert state["pitch_deg"] == pytest.approx(-3.0)
    assert state["axis"] == "roll"
    assert state["base_drive_inhibited"] is True
    assert state["error_action"] == "brake"
    assert state["imu_stale"] is False
    assert state["limits"] == {
        "roll_warning_deg": 8.0,
        "roll_error_deg": 15.0,
        "pitch_warning_deg": 12.0,
        "pitch_error_deg": 20.0,
        "hysteresis_deg": 2.0,
    }
    assert state["level_age_s"] == pytest.approx(0.5, abs=0.05)
    assert state["stamp_ns"] == 123
    assert "roll 18.0 deg" in state["message"] and "15.0" in state["message"]
    json.dumps(state)


def test_state_json_warning_and_ok_messages() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 0.0, 0.0, 0.0, 0.1)
    ok = guard.state_dict(t, stamp_ns=1)
    assert ok is not None and ok["level"] == "ok" and ok["axis"] is None and ok["base_drive_inhibited"] is False
    t = feed(guard, 0.0, 13.0, t, 1.0)
    warn = guard.state_dict(t, stamp_ns=2)
    assert warn is not None and warn["level"] == "warning" and warn["level_code"] == 1 and warn["axis"] == "pitch"
    assert "manually" in warn["message"]


def test_state_json_stale_flag() -> None:
    guard = TiltGuard(TiltGuardConfig())
    t = feed(guard, 0.0, 0.0, 0.0, 0.1)
    state = guard.state_dict(t + 5.0, stamp_ns=1)
    assert state is not None and state["imu_stale"] is True


# --- metrics ----------------------------------------------------------------------------------------------------------


def test_metrics_follow_samples_and_level() -> None:
    guard = TiltGuard(TiltGuardConfig())
    feed(guard, 18.0, -2.0, 0.0, 1.0)
    assert REGISTRY.get_sample_value("tilt_guard_level") == 2.0
    assert REGISTRY.get_sample_value("tilt_roll_deg") == pytest.approx(18.0)
    assert REGISTRY.get_sample_value("tilt_pitch_deg") == pytest.approx(-2.0)


# --- torque_off payloads --------------------------------------------------------------------------------------------


def test_drive_torque_payloads() -> None:
    payloads = [json.loads(p) for p in drive_torque_payloads(DRIVE_JOINTS, enable=False)]
    assert payloads == [{"joint_name": j, "register": "torque_enable", "value": 0} for j in DRIVE_JOINTS]
    assert json.loads(drive_torque_payloads(DRIVE_JOINTS, enable=True)[0])["value"] == 1
