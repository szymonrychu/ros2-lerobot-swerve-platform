"""Unit tests for joint command position-to-steps mapping."""

import math

import pytest
from feetech_servos.command_mapping import (
    STEP_CENTER,
    STEPS_PER_RADIAN,
    map_position_to_steps,
    position_to_raw_steps,
    steps_to_radians,
)

# --- steps_to_radians ---


def test_steps_to_radians_center() -> None:
    """Servo centre (step 2048) maps to 0 radians."""
    assert steps_to_radians(2048) == pytest.approx(0.0)


def test_steps_to_radians_min() -> None:
    """Step 0 maps to approximately -π radians."""
    assert steps_to_radians(0) == pytest.approx(-math.pi, abs=1e-3)


def test_steps_to_radians_max() -> None:
    """Step 4095 maps near +π (one step short of full revolution, so within 2 mrad)."""
    assert steps_to_radians(4095) == pytest.approx(math.pi, abs=2e-3)


def test_steps_to_radians_quarter() -> None:
    """Step 3072 (centre + 1024) maps to approximately +π/2."""
    assert steps_to_radians(3072) == pytest.approx(math.pi / 2, abs=0.01)


# --- position_to_raw_steps ---


def test_position_to_raw_steps_center() -> None:
    """0 radians maps to servo centre (step 2048)."""
    assert position_to_raw_steps(0.0) == STEP_CENTER


def test_position_to_raw_steps_positive_pi() -> None:
    """+π radians maps near step 4095."""
    result = position_to_raw_steps(math.pi)
    assert 4090 <= result <= 4095


def test_position_to_raw_steps_negative_pi() -> None:
    """-π radians maps near step 0."""
    result = position_to_raw_steps(-math.pi)
    assert 0 <= result <= 5


def test_position_to_raw_steps_clamp_high() -> None:
    """Values beyond +π are clamped to 4095."""
    assert position_to_raw_steps(math.pi * 2) == 4095


def test_position_to_raw_steps_clamp_low() -> None:
    """Values beyond -π are clamped to 0."""
    assert position_to_raw_steps(-math.pi * 2) == 0


# --- roundtrip ---


@pytest.mark.parametrize("ticks", [0, 512, 1024, 2048, 2560, 3072, 4095])
def test_roundtrip(ticks: int) -> None:
    """steps_to_radians → position_to_raw_steps roundtrip is lossless to nearest step."""
    rad = steps_to_radians(ticks)
    result = position_to_raw_steps(rad)
    assert result == ticks


# --- map_position_to_steps ---


def test_map_position_full_range_min() -> None:
    """-π (step 0) with source 0-4095 maps to cmd_min=0."""
    result = map_position_to_steps(-math.pi, 0, 4095, 0, 4095)
    assert result == 0


def test_map_position_full_range_max() -> None:
    """+π (step 4095) with source 0-4095 maps to cmd_max=4095."""
    result = map_position_to_steps(math.pi, 0, 4095, 0, 4095)
    assert result == 4095


def test_map_position_midpoint() -> None:
    """0 radians (centre, step 2048) with source 0-4095 maps to midpoint of command range."""
    result = map_position_to_steps(0.0, 0, 4095, 0, 4095)
    assert result in (2047, 2048)


def test_map_position_source_degenerate_returns_cmd_min() -> None:
    """When source_max == source_min, avoid division by zero; return cmd_min."""
    result = map_position_to_steps(0.0, 1000, 1000, 1951, 3377)
    assert result == 1951


def test_map_position_clamped_below_source() -> None:
    """Raw steps below source_min normalize to 0 -> cmd_min."""
    # source is steps 1000-3000; input is 0.0 rad = step 2048, which is inside range,
    # so use a clearly below-source value: steps_to_radians(500) => below source_min=1000
    rad_below = steps_to_radians(500)
    result = map_position_to_steps(rad_below, 1000, 3000, 1951, 3377)
    assert result == 1951


def test_map_position_clamped_above_source() -> None:
    """Raw steps above source_max normalize to 1 -> cmd_max."""
    rad_above = steps_to_radians(3500)  # above source_max=3000
    result = map_position_to_steps(rad_above, 0, 3000, 1951, 3377)
    assert result == 3377


def test_map_position_inverted_source() -> None:
    """With source_inverted, source_max maps to cmd_min and source_min maps to cmd_max."""
    rad_max = steps_to_radians(3000)
    rad_min = steps_to_radians(0)
    assert map_position_to_steps(rad_max, 0, 3000, 1951, 3377, source_inverted=True) == 1951
    assert map_position_to_steps(rad_min, 0, 3000, 1951, 3377, source_inverted=True) == 3377


# --- gripper range mapping (matches follower config) ---


def test_map_position_gripper_source_min() -> None:
    """Leader gripper at source_min_steps=2045 maps to command_min=1900."""
    rad = steps_to_radians(2045)
    result = map_position_to_steps(rad, 2045, 3289, 1900, 4095)
    assert result == 1900


def test_map_position_gripper_source_max() -> None:
    """Leader gripper at source_max_steps=3289 maps to command_max."""
    rad = steps_to_radians(3289)
    result = map_position_to_steps(rad, 2045, 3289, 1900, 4095)
    assert result == 4095


def test_map_position_gripper_midpoint() -> None:
    """Leader gripper at midpoint of source range maps to midpoint of command range."""
    source_mid = (2045 + 3289) // 2  # 2667
    rad = steps_to_radians(source_mid)
    result = map_position_to_steps(rad, 2045, 3289, 1900, 4095)
    expected_mid = round(1900 + (4095 - 1900) / 2)
    assert abs(result - expected_mid) <= 2


# --- steps_to_radians / STEPS_PER_RADIAN consistency ---


def test_scale_consistency() -> None:
    """STEPS_PER_RADIAN × RADIANS_PER_STEP == 1 (inverse constants)."""
    from feetech_servos.command_mapping import RADIANS_PER_STEP

    assert STEPS_PER_RADIAN * RADIANS_PER_STEP == pytest.approx(1.0)
