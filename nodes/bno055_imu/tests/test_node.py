"""Unit tests for BNO055 IMU node validation helper functions."""

from unittest.mock import MagicMock

import pytest

from bno055_imu.node import all_zero, coerce, has_valid_tuple, valid_quat, warmup_check

# ---------------------------------------------------------------------------
# coerce
# ---------------------------------------------------------------------------


def test_coerce_none_returns_default() -> None:
    """None input returns the default value."""
    assert coerce(None) == 0.0


def test_coerce_none_returns_custom_default() -> None:
    """None input returns a custom default value."""
    assert coerce(None, default=99.0) == 99.0


def test_coerce_float_returns_float() -> None:
    """A float value is returned as-is."""
    assert coerce(3.14) == pytest.approx(3.14)


def test_coerce_int_returns_float() -> None:
    """An integer is coerced to float."""
    result = coerce(5)
    assert result == 5.0
    assert isinstance(result, float)


def test_coerce_negative_float() -> None:
    """A negative float is handled correctly."""
    assert coerce(-1.5) == pytest.approx(-1.5)


# ---------------------------------------------------------------------------
# all_zero
# ---------------------------------------------------------------------------


def test_all_zero_all_zeros_returns_true() -> None:
    """All-zero sequence returns True."""
    assert all_zero((0.0, 0.0, 0.0)) is True


def test_all_zero_non_zero_returns_false() -> None:
    """Sequence with non-zero value returns False."""
    assert not all_zero((0.0, 1.0, 0.0))


def test_all_zero_empty_sequence_is_falsy() -> None:
    """Empty sequence returns a falsy value."""
    assert not all_zero(())


def test_all_zero_within_tolerance_returns_true() -> None:
    """Values within the default tolerance of zero return True."""
    assert all_zero((1e-7, 0.0, -1e-7))


def test_all_zero_outside_tolerance_returns_false() -> None:
    """Values outside the default tolerance return False."""
    assert not all_zero((1e-5, 0.0, 0.0))


def test_all_zero_custom_tolerance() -> None:
    """Custom tolerance is respected."""
    assert all_zero((0.05, 0.04, -0.03), tol=0.1)
    assert not all_zero((0.05, 0.04, -0.03), tol=0.01)


# ---------------------------------------------------------------------------
# has_valid_tuple
# ---------------------------------------------------------------------------


def test_has_valid_tuple_none_returns_false() -> None:
    """None input returns False."""
    assert has_valid_tuple(None, 3) is False


def test_has_valid_tuple_too_short_returns_false() -> None:
    """Tuple shorter than min_len returns False."""
    assert has_valid_tuple((1.0, 2.0), 3) is False


def test_has_valid_tuple_with_none_elements_returns_false() -> None:
    """Tuple containing None elements returns False."""
    assert has_valid_tuple((1.0, None, 3.0), 3) is False


def test_has_valid_tuple_valid_non_zero_returns_true() -> None:
    """Valid tuple with non-zero values returns True."""
    assert has_valid_tuple((1.0, 2.0, 3.0), 3) is True


def test_has_valid_tuple_all_zero_allow_zeros_false_returns_false() -> None:
    """All-zero tuple with allow_zeros=False returns False."""
    assert has_valid_tuple((0.0, 0.0, 0.0), 3, allow_zeros=False) is False


def test_has_valid_tuple_all_zero_allow_zeros_true_returns_true() -> None:
    """All-zero tuple with allow_zeros=True returns True."""
    assert has_valid_tuple((0.0, 0.0, 0.0), 3, allow_zeros=True) is True


def test_has_valid_tuple_exactly_at_min_len() -> None:
    """Tuple exactly at min_len works correctly."""
    assert has_valid_tuple((1.0, 2.0, 3.0), 3) is True
    assert has_valid_tuple((1.0, 2.0), 2) is True


def test_has_valid_tuple_longer_than_min_len() -> None:
    """Tuple longer than min_len is accepted."""
    assert has_valid_tuple((1.0, 2.0, 3.0, 4.0), 3) is True


# ---------------------------------------------------------------------------
# valid_quat
# ---------------------------------------------------------------------------


def test_valid_quat_none_returns_false() -> None:
    """None input returns False."""
    assert valid_quat(None) is False


def test_valid_quat_all_zeros_returns_false() -> None:
    """All-zeros quaternion returns False."""
    assert valid_quat((0.0, 0.0, 0.0, 0.0)) is False


def test_valid_quat_identity_returns_true() -> None:
    """Valid unit quaternion (1, 0, 0, 0) returns True."""
    assert valid_quat((1.0, 0.0, 0.0, 0.0)) is True


def test_valid_quat_unit_norm_returns_true() -> None:
    """Valid quaternion with norm ~1.0 returns True."""
    # norm² = 0.5² + 0.5² + 0.5² + 0.5² = 1.0
    assert valid_quat((0.5, 0.5, 0.5, 0.5)) is True


def test_valid_quat_norm_too_large_returns_false() -> None:
    """Quaternion with norm² > 1.1 returns False."""
    assert valid_quat((2.0, 0.0, 0.0, 0.0)) is False


def test_valid_quat_norm_too_small_returns_false() -> None:
    """Quaternion with norm² < 0.9 returns False."""
    assert valid_quat((0.1, 0.0, 0.0, 0.0)) is False


def test_valid_quat_with_none_elements_returns_false() -> None:
    """Quaternion with None elements returns False."""
    assert valid_quat((1.0, None, 0.0, 0.0)) is False


def test_valid_quat_too_short_returns_false() -> None:
    """Tuple with fewer than 4 elements returns False."""
    assert valid_quat((1.0, 0.0, 0.0)) is False


# ---------------------------------------------------------------------------
# warmup_check
# ---------------------------------------------------------------------------


def test_warmup_check_oserror_on_gyro_returns_false() -> None:
    """Returns False when bno.gyro raises OSError."""
    bno = MagicMock()
    bno.gyro = MagicMock(side_effect=OSError("I2C error"))
    assert warmup_check(bno) is False


def test_warmup_check_gyro_has_none_values_returns_false() -> None:
    """Returns False when gyro values contain None."""
    bno = MagicMock()
    type(bno).gyro = property(lambda self: (None, 0.0, 0.0))
    type(bno).linear_acceleration = property(lambda self: (0.1, 0.2, 0.3))
    assert warmup_check(bno) is False


def test_warmup_check_all_gyro_none_returns_false() -> None:
    """Returns False when all gyro values are None."""
    bno = MagicMock()
    type(bno).gyro = property(lambda self: (None, None, None))
    type(bno).linear_acceleration = property(lambda self: (0.1, 0.2, 0.3))
    assert warmup_check(bno) is False


def test_warmup_check_valid_gyro_and_linear_accel_returns_true() -> None:
    """Returns True when gyro and linear_acceleration are valid."""
    bno = MagicMock()
    type(bno).gyro = property(lambda self: (0.01, 0.02, 0.03))
    type(bno).linear_acceleration = property(lambda self: (0.1, 0.2, 9.81))
    assert warmup_check(bno) is True


def test_warmup_check_falls_back_to_acceleration_when_linear_invalid() -> None:
    """Falls back to bno.acceleration when linear_acceleration is invalid but acceleration is valid."""
    bno = MagicMock()
    type(bno).gyro = property(lambda self: (0.01, 0.02, 0.03))
    type(bno).linear_acceleration = property(lambda self: (None, None, None))
    type(bno).acceleration = property(lambda self: (0.1, 0.2, 9.81))
    assert warmup_check(bno) is True


def test_warmup_check_falls_back_acceleration_also_invalid_returns_false() -> None:
    """Returns False when both linear_acceleration and acceleration are invalid."""
    bno = MagicMock()
    type(bno).gyro = property(lambda self: (0.01, 0.02, 0.03))
    type(bno).linear_acceleration = property(lambda self: (None, None, None))
    type(bno).acceleration = property(lambda self: (None, None, None))
    assert warmup_check(bno) is False


def test_warmup_check_acceleration_raises_oserror_returns_false() -> None:
    """Returns False when fallback bno.acceleration also raises OSError."""

    def raise_oserror(self: object) -> None:
        raise OSError("I2C error")

    bno = MagicMock()
    type(bno).gyro = property(lambda self: (0.01, 0.02, 0.03))
    type(bno).linear_acceleration = property(lambda self: (None, None, None))
    type(bno).acceleration = property(raise_oserror)
    assert warmup_check(bno) is False
