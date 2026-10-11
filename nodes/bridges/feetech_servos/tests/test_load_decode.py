"""Present_Load is sign-magnitude on STS3215: magnitude in bits 0-9, direction in bit 10."""

import pytest

from feetech_servos.registers import CURRENT_LSB_A, CURRENT_MAGNITUDE_MASK, decode_present_current, decode_present_load


@pytest.mark.parametrize(
    ("raw", "expected"),
    [(0, 0), (52, 52), (1023, 1023), (1024, 0), (1056, -32), (1076, -52), (2047, -1023)],
)
def test_decode_present_load_sign_magnitude(raw: int, expected: int) -> None:
    assert decode_present_load(raw) == expected


def test_decode_present_load_inverted_joint_flips_the_direction() -> None:
    assert decode_present_load(1056, inverted=True) == 32
    assert decode_present_load(52, inverted=True) == -52


def test_current_constants() -> None:
    assert CURRENT_MAGNITUDE_MASK == 0x7FFF
    assert CURRENT_LSB_A == 0.0065


@pytest.mark.parametrize(
    ("raw", "amps"),
    [(0, 0.0), (1, 0.0065), (100, 0.65), (0x7FFF, 0x7FFF * 0.0065), (0x8000 | 100, 0.65)],
)
def test_decode_present_current_is_an_unsigned_magnitude_in_amperes(raw: int, amps: float) -> None:
    assert decode_present_current(raw) == pytest.approx(amps)
