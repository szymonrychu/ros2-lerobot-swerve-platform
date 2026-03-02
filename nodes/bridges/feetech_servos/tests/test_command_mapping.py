"""Unit tests for joint command position-to-steps mapping (range mapping)."""

from feetech_servos.command_mapping import map_position_to_steps, position_to_raw_steps


def test_map_position_full_range_identity() -> None:
    """When source and command ranges are 0-4095, 0 and 4.095 map to 0 and 4095."""
    assert map_position_to_steps(0.0, 0, 4095, 0, 4095) == 0
    assert map_position_to_steps(4.095, 0, 4095, 0, 4095) == 4095


def test_map_position_midpoint() -> None:
    """Position 2.0475 (half of 4095) maps to midpoint of command range."""
    result = map_position_to_steps(2.0475, 0, 4095, 0, 4095)
    assert result in (2047, 2048)  # 2047.5 rounds to 2048


def test_map_position_to_narrow_command_range() -> None:
    """Position 3.0 with source 0-4095 maps into command range 1951-3377."""
    result = map_position_to_steps(3.0, 0, 4095, 1951, 3377)
    # 3000/4095 ~= 0.7326; 1951 + 0.7326 * (3377-1951) ~= 2996
    assert 1951 <= result <= 3377
    assert result == 2996


def test_map_position_leader_max_to_follower_max() -> None:
    """When leader sends its max (e.g. 3.0), with source_max 3000, follower gets cmd_max."""
    # Leader "fully closed" = 3000 steps = 3.0; source range 0-3000; command range 1951-3377.
    result = map_position_to_steps(3.0, 0, 3000, 1951, 3377)
    assert result == 3377


def test_map_position_leader_min_to_follower_min() -> None:
    """When leader sends min (0), normalized 0 -> cmd_min."""
    result = map_position_to_steps(0.0, 0, 3000, 1951, 3377)
    assert result == 1951


def test_map_position_source_degenerate_returns_cmd_min() -> None:
    """When source_max == source_min, avoid division by zero; return cmd_min."""
    result = map_position_to_steps(1.5, 1000, 1000, 1951, 3377)
    assert result == 1951


def test_map_position_clamped_below_source() -> None:
    """Raw steps below source_min normalize to 0 -> cmd_min."""
    result = map_position_to_steps(0.5, 1000, 3000, 1951, 3377)
    # 500 < 1000 => normalized 0
    assert result == 1951


def test_map_position_clamped_above_source() -> None:
    """Raw steps above source_max normalize to 1 -> cmd_max."""
    result = map_position_to_steps(4.0, 0, 3000, 1951, 3377)
    # 4000 > 3000 => normalized 1
    assert result == 3377


def test_position_to_raw_steps_passthrough_clamped() -> None:
    """Raw pass-through converts pseudo-radians to steps with 0..4095 clamp."""
    assert position_to_raw_steps(3.0) == 3000
    assert position_to_raw_steps(-0.2) == 0
    assert position_to_raw_steps(9.0) == 4095
