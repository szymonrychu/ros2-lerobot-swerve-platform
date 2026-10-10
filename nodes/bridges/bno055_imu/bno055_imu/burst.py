"""Single-transaction BNO055 data read and its decoding.

The adafruit_bno055 properties cost one operation-mode read plus one register read each (two I2C transactions per
property). On the kernel bit-banged i2c-gpio bus that time is burnt as system CPU. The data registers gyro, euler,
quaternion and linear acceleration are contiguous, so one write-register-then-read of 26 bytes replaces all of them.
The scale factors below are the ones adafruit_bno055.BNO055_I2C applies, and are applied with the same expression so
the decoded values are bit-identical to the library properties.
"""

import struct
from typing import Any, NamedTuple

BURST_START_REGISTER = 0x14  # GYR_DATA_X_LSB
BURST_LENGTH = 26  # gyro 6 + euler 6 + quaternion 8 + linear acceleration 6 bytes
BURST_FORMAT = "<13h"
GYRO_SCALE = 0.001090830782496456  # LSB to rad/s (library default units)
EULER_SCALE = 1 / 16  # LSB to degrees
QUATERNION_SCALE = 1 / (1 << 14)  # LSB to unit quaternion
LINEAR_ACCEL_SCALE = 1 / 100  # LSB to m/s^2


class ImuSample(NamedTuple):
    """Decoded burst: gyro (rad/s), euler (deg: heading, roll, pitch), quaternion (w, x, y, z), linear accel (m/s^2)."""

    gyro: tuple[float, float, float]
    euler: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    linear_acceleration: tuple[float, float, float]


def decode_burst(block: bytes | bytearray | memoryview) -> ImuSample | None:
    """Decode the 26-byte data block exactly as the adafruit_bno055 properties would.

    Args:
        block (bytes | bytearray | memoryview): Registers 0x14..0x2D as read from the chip.

    Returns:
        ImuSample | None: Decoded sample, or None when the block has the wrong length or is all zero (what the chip
            returns outside a fusion mode, or on a failed read); the caller treats None as an invalid sample.
    """
    if len(block) != BURST_LENGTH or not any(block):
        return None
    raw = struct.unpack(BURST_FORMAT, block)
    return ImuSample(
        gyro=tuple(GYRO_SCALE * v for v in raw[0:3]),
        euler=tuple(EULER_SCALE * v for v in raw[3:6]),
        quaternion=tuple(QUATERNION_SCALE * v for v in raw[6:10]),
        linear_acceleration=tuple(LINEAR_ACCEL_SCALE * v for v in raw[10:13]),
    )


def read_burst(bno: Any) -> bytes:
    """Read registers 0x14..0x2D in one I2C transaction.

    Args:
        bno (Any): adafruit_bno055.BNO055_I2C driver (its i2c_device is used).

    Returns:
        bytes: The 26 data bytes.

    Raises:
        OSError: On a bus error (propagated from the I2C device).
        RuntimeError: On a bus error (propagated from the I2C device).
    """
    buffer = bytearray(BURST_LENGTH + 1)
    buffer[0] = BURST_START_REGISTER
    with bno.i2c_device as i2c:
        i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)
    return bytes(buffer[1:])


def read_sample(bno: Any) -> ImuSample | None:
    """Read and decode one sample with a single I2C transaction.

    Args:
        bno (Any): adafruit_bno055.BNO055_I2C driver.

    Returns:
        ImuSample | None: Decoded sample, or None for an invalid (all-zero or short) block.

    Raises:
        OSError: On a bus error.
        RuntimeError: On a bus error.
    """
    return decode_burst(read_burst(bno))
