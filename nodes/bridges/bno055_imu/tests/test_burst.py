"""Tests for the single-transaction BNO055 burst read and its decoding against the adafruit_bno055 conversion."""

import struct
from typing import Any

import pytest
from adafruit_bno055 import BNO055_I2C

from bno055_imu.burst import BURST_LENGTH, BURST_START_REGISTER, ImuSample, decode_burst, read_burst, read_sample

MODE_REGISTER = 0x3D
IMUPLUS = 0x08

# int16 raw values in register order: gyro(3) euler(3) quaternion(4) linear acceleration(3)
RAW = (-123, 456, 32767, 1600, -800, 5760, 16384, -1, 2, -16384, 250, -9806, 77)


class FakeI2CDevice:
    """Register-map backed I2CDevice stand-in that counts transactions."""

    def __init__(self, registers: bytearray) -> None:
        self.registers = registers
        self.transactions = 0
        self.entered = 0

    def __enter__(self) -> "FakeI2CDevice":
        self.entered += 1
        return self

    def __exit__(self, *_exc: Any) -> None:
        return None

    def write_then_readinto(
        self,
        out_buffer: Any,
        in_buffer: Any,
        *,
        out_end: int | None = None,
        in_start: int = 0,
        in_end: int | None = None,
    ) -> None:
        assert out_end == 1  # only the register address is written
        self.transactions += 1
        register = out_buffer[0]
        end = len(in_buffer) if in_end is None else in_end
        n = end - in_start
        in_buffer[in_start:end] = self.registers[register : register + n]


def make_bno(raw: tuple[int, ...] = RAW, mode: int = IMUPLUS) -> tuple[Any, FakeI2CDevice]:
    registers = bytearray(256)
    registers[BURST_START_REGISTER : BURST_START_REGISTER + BURST_LENGTH] = struct.pack("<13h", *raw)
    registers[MODE_REGISTER] = mode
    device = FakeI2CDevice(registers)
    bno = BNO055_I2C.__new__(BNO055_I2C)
    bno.i2c_device = device
    bno.buffer = bytearray(2)
    return bno, device


def block(raw: tuple[int, ...] = RAW) -> bytes:
    return struct.pack("<13h", *raw)


def test_burst_covers_gyro_to_linear_acceleration() -> None:
    assert BURST_START_REGISTER == 0x14
    assert BURST_LENGTH == 26


def test_decode_matches_library_conversion_exactly() -> None:
    bno, _ = make_bno()
    sample = decode_burst(block())
    assert sample is not None
    assert sample.gyro == bno.gyro
    assert sample.euler == bno.euler
    assert sample.quaternion == bno.quaternion
    assert sample.linear_acceleration == bno.linear_acceleration


@pytest.mark.parametrize("seed", range(5))
def test_decode_matches_library_for_varied_raw_bytes(seed: int) -> None:
    import random

    rng = random.Random(seed)
    raw = tuple(rng.randint(-32768, 32767) for _ in range(13))
    bno, _ = make_bno(raw)
    sample = decode_burst(block(raw))
    assert sample == ImuSample(bno.gyro, bno.euler, bno.quaternion, bno.linear_acceleration)


def test_decode_known_values() -> None:
    sample = decode_burst(block())
    assert sample is not None
    assert sample.quaternion[0] == pytest.approx(1.0)
    assert sample.linear_acceleration[0] == pytest.approx(2.5)
    assert sample.linear_acceleration[1] == pytest.approx(-98.06)
    assert sample.gyro[0] == pytest.approx(-123 * 0.001090830782496456)


def test_all_zero_block_is_invalid() -> None:
    assert decode_burst(bytes(BURST_LENGTH)) is None


def test_wrong_length_block_is_invalid() -> None:
    assert decode_burst(bytes(10)) is None
    assert decode_burst(b"") is None


def test_read_burst_is_one_i2c_transaction() -> None:
    bno, device = make_bno()
    data = read_burst(bno)
    assert bytes(data) == block()
    assert device.transactions == 1
    assert device.entered == 1


def test_read_burst_uses_register_0x14_and_26_bytes() -> None:
    bno, device = make_bno()
    seen: list[tuple[int, int]] = []
    original = device.write_then_readinto

    def spy(out_buffer: Any, in_buffer: Any, **kw: Any) -> None:
        seen.append((out_buffer[0], (kw.get("in_end") or len(in_buffer)) - kw.get("in_start", 0)))
        original(out_buffer, in_buffer, **kw)

    device.write_then_readinto = spy  # type: ignore[method-assign]
    read_burst(bno)
    assert seen == [(0x14, 26)]


def test_read_burst_propagates_oserror() -> None:
    bno, device = make_bno()

    def boom(*_a: Any, **_k: Any) -> None:
        raise OSError("bus")

    device.write_then_readinto = boom  # type: ignore[method-assign]
    with pytest.raises(OSError):
        read_burst(bno)


def test_read_sample_is_one_transaction_and_decoded() -> None:
    bno, device = make_bno()
    sample = read_sample(bno)
    assert device.transactions == 1
    assert sample is not None
    assert sample.quaternion == bno.quaternion


def test_read_sample_all_zero_registers_is_invalid_sample() -> None:
    bno, device = make_bno(raw=(0,) * 13, mode=0x00)
    assert read_sample(bno) is None
    assert device.transactions == 1
