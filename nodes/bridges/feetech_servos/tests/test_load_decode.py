"""Present_Load is sign-magnitude on STS3215: magnitude in bits 0-9, direction in bit 10."""

import pytest

from feetech_servos.registers import decode_present_load


@pytest.mark.parametrize(
    ("raw", "expected"),
    [(0, 0), (52, 52), (1023, 1023), (1024, 0), (1056, -32), (1076, -52), (2047, -1023)],
)
def test_decode_present_load_sign_magnitude(raw: int, expected: int) -> None:
    assert decode_present_load(raw) == expected


def test_decode_present_load_inverted_joint_flips_the_direction() -> None:
    assert decode_present_load(1056, inverted=True) == 32
    assert decode_present_load(52, inverted=True) == -52
