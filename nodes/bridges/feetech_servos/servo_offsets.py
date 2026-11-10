"""STS3215 offset register helpers shared by the arm zero restore and the servo calibration flow (no rclpy).

Offset register (address 31): sign-magnitude, magnitude in bits 0-10, sign (negative) in bit 11. present = (encoder -
offset) mod 4096, so the offset only matters modulo 4096 (wrap_offset_steps). Pure helpers (encoding, wrap, deltas,
still spread) plus bus-level helpers (retried reads, offset and angle limit writes, present_position checks).
"""

from collections.abc import Callable
from typing import Any

from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION

from .config import JointEntry
from .neutral import MAX_LIMIT_REGISTER, MIN_LIMIT_REGISTER, names_text
from .registers import get_register_entry_by_name, read_register, write_register
from .torque_control import PRESENT_REGISTER, TORQUE_OFF, TORQUE_REGISTER

DEGREES_PER_TURN = 360.0
HALF_TURN_STEPS = STEPS_PER_REVOLUTION // 2
# STS3215 offset register: sign-magnitude, magnitude in bits 0-10, sign (negative) in bit 11.
OFFSET_SIGN_BIT = 0x800
OFFSET_MAGNITUDE_MASK = 0x7FF
OFFSET_MAX_STEPS = OFFSET_MAGNITUDE_MASK
OFFSET_VALID_BITS = OFFSET_SIGN_BIT | OFFSET_MAGNITUDE_MASK
OFFSET_REGISTER = "offset"
READ_ATTEMPTS = 3
SETTLE_S = 0.5
VERIFY_READ_ATTEMPTS = 10
VERIFY_RETRY_SLEEP_S = 0.1
STILL_SAMPLE_INTERVAL_S = 0.1


def encode_offset(steps: int) -> int:
    """Encode an offset in steps as the STS3215 sign-magnitude register value.

    Args:
        steps (int): Offset, -2047..2047.

    Returns:
        int: Raw register value (bit 11 set for negative, magnitude in bits 0-10).

    Raises:
        ValueError: |steps| > 2047 (never clamped).
    """
    if abs(steps) > OFFSET_MAX_STEPS:
        raise ValueError(f"offset {steps} outside +-{OFFSET_MAX_STEPS}")
    return (-steps | OFFSET_SIGN_BIT) if steps < 0 else steps


def wrap_offset_steps(steps: int) -> int:
    """Wrap an offset into one turn around zero; the offset only matters modulo 4096.

    Args:
        steps (int): Offset in steps, any value.

    Returns:
        int: The equivalent offset in -2048..2047 (-2048, i.e. +-2048, is not encodable: encode_offset refuses it).
    """
    return (steps + HALF_TURN_STEPS) % STEPS_PER_REVOLUTION - HALF_TURN_STEPS


def decode_offset(raw: int) -> int:
    """Decode a raw STS3215 offset register value.

    Args:
        raw (int): Raw 2-byte value.

    Returns:
        int: Offset in steps (0x800, negative zero, is 0).

    Raises:
        ValueError: Bits above the sign bit are set (not an offset value).
    """
    if raw & ~OFFSET_VALID_BITS:
        raise ValueError(f"offset register 0x{raw:04x} has bits above bit 11 set")
    magnitude = raw & OFFSET_MAGNITUDE_MASK
    return -magnitude if raw & OFFSET_SIGN_BIT else magnitude


def steps_to_degrees(steps: float) -> float:
    """Convert servo steps to degrees.

    Args:
        steps (float): Steps (4096 per turn).

    Returns:
        float: Degrees.
    """
    return steps * DEGREES_PER_TURN / STEPS_PER_REVOLUTION


def offset_delta_steps(present: int, target: int) -> int:
    """Offset change that makes a resting servo reading present read target (present = encoder - offset).

    Args:
        present (int): Current present_position (0..4095).
        target (int): Wanted present_position (0..4095).

    Returns:
        int: present - target, the short way round the 0/4095 wrap (-2048..2047).
    """
    return (present - target + HALF_TURN_STEPS) % STEPS_PER_REVOLUTION - HALF_TURN_STEPS


def read_retried(servo: Any, servo_id: int, register: str, attempts: int = READ_ATTEMPTS) -> int | None:
    """Read one register by name, retrying a failed read.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        register (str): Register name (feetech_servos.registers).
        attempts (int): Maximum reads.

    Returns:
        int | None: Raw value, or None when every read failed.
    """
    entry = get_register_entry_by_name(register)
    for _ in range(attempts):
        value = read_register(servo, servo_id, entry)
        if value is not None:
            return value
    return None


def read_limits(servo: Any, servo_id: int) -> tuple[int, int] | None:
    """Read (min_angle_limit, max_angle_limit).

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.

    Returns:
        tuple[int, int] | None: The limits, or None when a read failed.
    """
    low = read_retried(servo, servo_id, MIN_LIMIT_REGISTER)
    high = read_retried(servo, servo_id, MAX_LIMIT_REGISTER)
    return None if low is None or high is None else (low, high)


def read_torques(servo: Any, joints: list[JointEntry]) -> dict[str, int | None]:
    """Read torque_enable of each joint from the bus (never the cache).

    Args:
        servo (Any): ST3215-compatible bus.
        joints (list[JointEntry]): Joints to read.

    Returns:
        dict[str, int | None]: joint name -> torque_enable, None when the read failed.
    """
    return {j.name: read_retried(servo, j.id, TORQUE_REGISTER) for j in joints}


def torque_refusals(torques: dict[str, int | None]) -> list[str]:
    """Name every joint whose torque_enable is not verified 0.

    Args:
        torques (dict[str, int | None]): joint name -> torque_enable reading (read_torques).

    Returns:
        list[str]: "<name> (torque_enable N)" or "<name> (torque read failed)" per refused joint.
    """
    refused = []
    for name, torque in torques.items():
        if torque is None:
            refused.append(f"{name} (torque read failed)")
        elif torque != TORQUE_OFF:
            refused.append(f"{name} (torque_enable {torque})")
    return refused


def still_spreads(samples: list[dict[str, int]]) -> dict[str, int]:
    """Per joint, the largest wrap-aware distance of any sample from the first one.

    Args:
        samples (list[dict[str, int]]): present_position readings over the still window, oldest first.

    Returns:
        dict[str, int]: joint name -> spread in steps.
    """
    first = samples[0]
    return {name: max(abs(offset_delta_steps(s[name], first[name])) for s in samples) for name in first}


def read_offset(servo: Any, servo_id: int) -> int | None:
    """Read and decode the offset register.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.

    Returns:
        int | None: Offset in steps, None when every read failed or the raw value has invalid bits.
    """
    raw = read_retried(servo, servo_id, OFFSET_REGISTER)
    if raw is None:
        return None
    try:
        return decode_offset(raw)
    except ValueError:
        return None


def write_offset(servo: Any, cache: dict[str, int], servo_id: int, steps: int) -> bool:
    """Write the offset register (write_register unlocks and relocks the EEPROM around it).

    Args:
        servo (Any): ST3215-compatible bus.
        cache (dict[str, int]): The servo's last-written register cache (the offset entry is dropped first).
        servo_id (int): Servo ID.
        steps (int): Offset in steps (-2047..2047).

    Returns:
        bool: True when the write was acknowledged.
    """
    cache.pop(OFFSET_REGISTER, None)
    entry = get_register_entry_by_name(OFFSET_REGISTER)
    return write_register(servo, servo_id, entry, encode_offset(steps), cache)


def verify_present(servo: Any, servo_id: int, target: int, tolerance: int, sleep: Callable[[float], Any]) -> int | None:
    """Read present_position until it is within tolerance of target.

    Args:
        servo (Any): ST3215-compatible bus.
        servo_id (int): Servo ID.
        target (int): Wanted present_position.
        tolerance (int): Allowed |present - target| (wrap-aware).
        sleep (Callable[[float], Any]): Sleep between reads, in seconds.

    Returns:
        int | None: The last reading (in tolerance when verified), None when every read failed.
    """
    entry = get_register_entry_by_name(PRESENT_REGISTER)
    last = None
    for attempt in range(VERIFY_READ_ATTEMPTS):
        reading = read_register(servo, servo_id, entry)
        if reading is not None:
            last = reading
            if abs(offset_delta_steps(reading, target)) <= tolerance:
                return reading
        if attempt < VERIFY_READ_ATTEMPTS - 1:
            sleep(VERIFY_RETRY_SLEEP_S)
    return last


def write_limits(servo: Any, cache: dict[str, int], servo_id: int, wanted: tuple[int, int]) -> tuple[int, int] | None:
    """Write the angle limits that read differently from wanted (as neutral.py after a centre), then read back.

    Args:
        servo (Any): ST3215-compatible bus.
        cache (dict[str, int]): The servo's last-written register cache (written entries are dropped first).
        servo_id (int): Servo ID.
        wanted (tuple[int, int]): Wanted (min, max) angle limits.

    Returns:
        tuple[int, int] | None: The limits read at the end (None when a read failed).
    """
    limits = read_limits(servo, servo_id)
    if limits == wanted:
        return limits
    current = limits if limits is not None else (None, None)
    for name, value, now in ((MIN_LIMIT_REGISTER, wanted[0], current[0]), (MAX_LIMIT_REGISTER, wanted[1], current[1])):
        if value == now:
            continue
        cache.pop(name, None)
        write_register(servo, servo_id, get_register_entry_by_name(name), value, cache)
    return read_limits(servo, servo_id)


def sample_still(
    servo: Any, joints: list[JointEntry], window_s: float, tolerance: int, sleep: Callable[[float], Any]
) -> tuple[dict[str, int] | None, str]:
    """Sample present_position over the still window; the last sample is the reading.

    Args:
        servo (Any): ST3215-compatible bus.
        joints (list[JointEntry]): Joints to sample.
        window_s (float): Still window in seconds.
        tolerance (int): Largest allowed spread in steps.
        sleep (Callable[[float], Any]): Sleep between samples, in seconds.

    Returns:
        tuple[dict[str, int] | None, str]: (joint name -> steps, "") or (None, refusal reason).
    """
    count = round(window_s / STILL_SAMPLE_INTERVAL_S) + 1
    samples: list[dict[str, int]] = []
    for index in range(count):
        if index:
            sleep(STILL_SAMPLE_INTERVAL_S)
        sample = {j.name: read_retried(servo, j.id, PRESENT_REGISTER) for j in joints}
        unread = [n for n, v in sample.items() if v is None]
        if unread:
            return None, f"present_position read failed: {names_text(unread)}"
        samples.append(sample)
    moved = {n: s for n, s in still_spreads(samples).items() if s > tolerance}
    if moved:
        text = ", ".join(f"{n} moved {s} steps" for n, s in moved.items())
        return None, f"arm not still: {text} (tolerance {tolerance}) over {window_s:g} s; rest it and try again"
    return samples[-1], ""
