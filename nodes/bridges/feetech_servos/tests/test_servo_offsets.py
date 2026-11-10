"""Pure offset helpers and the bus-level offset functions of servo_offsets."""

import pytest
from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION
from st3215.values import INST_WRITE

from feetech_servos.registers import (
    STS_LOCK,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_TORQUE_ENABLE,
)
from feetech_servos.servo_offsets import (
    OFFSET_MAX_STEPS,
    decode_offset,
    encode_offset,
    offset_delta_steps,
    read_offset,
    steps_to_degrees,
    wrap_offset_steps,
    write_limits,
)

from .offset_bus import OffsetPort
from .sts_bus import put_word, register_table
from .test_neutral import FakeServo, word

SERVO_ID = 1
EEPROM_END_ADDRESS = 40  # the lock register (55) lies above the EEPROM block


def one_servo_bus(limits: tuple[int, int], offset_raw: int = 0) -> tuple[FakeServo, OffsetPort]:
    """A single-servo OffsetPort bus with the given angle limits and raw offset register.

    Args:
        limits (tuple[int, int]): (min, max) angle limits.
        offset_raw (int): Raw offset register value.

    Returns:
        tuple[FakeServo, OffsetPort]: The servo handle and its emulated port.
    """
    port = OffsetPort(servos={})
    table = register_table({STS_TORQUE_ENABLE: 0, STS_LOCK: 1})
    put_word(table, STS_MIN_ANGLE_LIMIT_L, limits[0])
    put_word(table, STS_MAX_ANGLE_LIMIT_L, limits[1])
    put_word(table, STS_OFS_L, offset_raw)
    port.servos[SERVO_ID] = table
    port.encoder[SERVO_ID] = 2048
    port.refresh_present(SERVO_ID)
    return FakeServo(port), port


def test_write_limits_skips_unchanged_registers() -> None:
    servo, port = one_servo_bus((100, 4000))
    assert write_limits(servo, {}, SERVO_ID, (100, 3900)) == (100, 3900)
    eeprom_writes = [
        p.params[0] for p in port.packets if p.instruction == INST_WRITE and p.params[0] < EEPROM_END_ADDRESS
    ]
    assert eeprom_writes == [STS_MAX_ANGLE_LIMIT_L]
    assert word(port.servos[SERVO_ID], STS_MAX_ANGLE_LIMIT_L) == 3900


def test_read_offset_decodes_sign_magnitude() -> None:
    servo, _ = one_servo_bus((0, 4095), offset_raw=0x805)
    assert read_offset(servo, SERVO_ID) == -5


@pytest.mark.parametrize(
    ("steps", "raw"),
    [(0, 0x000), (1, 0x001), (-1, 0x801), (901, 0x385), (-1029, 0xC05), (2047, 0x7FF), (-2047, 0xFFF)],
)
def test_offset_encoding_round_trip(steps: int, raw: int) -> None:
    assert encode_offset(steps) == raw
    assert decode_offset(raw) == steps


def test_negative_zero_decodes_to_zero() -> None:
    assert decode_offset(0x800) == 0


@pytest.mark.parametrize("steps", [2048, -2048, 4000])
def test_encode_rejects_magnitude_over_2047(steps: int) -> None:
    with pytest.raises(ValueError):
        encode_offset(steps)


@pytest.mark.parametrize("raw", [0x1000, 0xF000])
def test_decode_rejects_bits_above_sign(raw: int) -> None:
    with pytest.raises(ValueError):
        decode_offset(raw)


def test_offset_max_is_2047() -> None:
    assert OFFSET_MAX_STEPS == 2047


def test_offset_delta_is_present_minus_target() -> None:
    assert offset_delta_steps(2949, 2048) == 901
    assert offset_delta_steps(1019, 2048) == -1029


def test_offset_delta_takes_the_short_way_across_the_wrap() -> None:
    assert offset_delta_steps(10, 4090) == 16
    assert offset_delta_steps(4090, 10) == -16


@pytest.mark.parametrize(
    ("steps", "wrapped"),
    [(2469, -1627), (-2242, 1854), (-2601, 1495), (0, 0), (2047, 2047), (-2047, -2047), (2048, -2048), (-2048, -2048)],
)
def test_wrap_offset_steps(steps: int, wrapped: int) -> None:
    assert wrap_offset_steps(steps) == wrapped
    assert (steps - wrapped) % STEPS_PER_REVOLUTION == 0


@pytest.mark.parametrize("steps", [2469, -2242, -2601])
def test_wrapped_user_offsets_encode(steps: int) -> None:
    assert decode_offset(encode_offset(wrap_offset_steps(steps))) == wrap_offset_steps(steps)


def test_wrapped_2048_is_not_encodable() -> None:
    with pytest.raises(ValueError):
        encode_offset(wrap_offset_steps(2048))


def test_steps_to_degrees() -> None:
    assert steps_to_degrees(1024) == 90.0
    assert steps_to_degrees(-901) == pytest.approx(-79.189, abs=1e-3)
