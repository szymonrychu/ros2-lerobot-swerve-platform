"""BNO055 calibration profile persistence: validate, read and write offsets, save and load the profile file.

The adafruit_bno055 offset/radius properties are plain register accesses; the chip only accepts offset writes (and
returns reliable reads) in CONFIG mode, so every access here switches to CONFIG mode explicitly.
"""

import json
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

CONFIG_MODE_VALUE = 0x00
MODE_SETTLE_S = 0.05  # CONFIG mode switch time (datasheet 19 ms), with margin for the slow bit-banged bus
MODE_SWITCH_DELAY_S = 1.5  # fusion needs time after switching back to produce valid data
CONFIG_VERIFY_RETRIES = 3
INT16_MIN = -32768
INT16_MAX = 32767
MIN_RADIUS = 1
MAX_RADIUS = 2000  # datasheet: accel radius <= 1000, mag radius <= 960
CALIBRATED = 3
PROFILE_KEYS = ("accel_offset", "gyro_offset", "mag_offset", "accel_radius", "mag_radius")


@dataclass(frozen=True)
class CalibrationProfile:
    """BNO055 sensor offsets and radii.

    Attributes:
        accel_offset: Accelerometer offsets (x, y, z), int16.
        gyro_offset: Gyroscope offsets (x, y, z), int16.
        mag_offset: Magnetometer offsets (x, y, z), int16.
        accel_radius: Accelerometer radius.
        mag_radius: Magnetometer radius.
    """

    accel_offset: tuple[int, int, int]
    gyro_offset: tuple[int, int, int]
    mag_offset: tuple[int, int, int]
    accel_radius: int
    mag_radius: int


def is_int(value: Any) -> bool:
    """Return True for a real int (bool excluded)."""
    return isinstance(value, int) and not isinstance(value, bool)


def parse_offset(value: Any, key: str) -> tuple[int, int, int]:
    """Validate a 3-element int16 offset.

    Args:
        value: Parsed JSON value.
        key: Key name for error messages.

    Returns:
        tuple[int, int, int]: The offset.

    Raises:
        ValueError: If value is not three ints within int16 range.
    """
    if not isinstance(value, (list, tuple)) or len(value) != 3 or not all(is_int(v) for v in value):
        raise ValueError(f"{key}: expected three integers, got {value!r}")
    if not all(INT16_MIN <= v <= INT16_MAX for v in value):
        raise ValueError(f"{key}: value outside int16 range: {value!r}")
    return (int(value[0]), int(value[1]), int(value[2]))


def parse_radius(value: Any, key: str) -> int:
    """Validate a plausible radius.

    Args:
        value: Parsed JSON value.
        key: Key name for error messages.

    Returns:
        int: The radius.

    Raises:
        ValueError: If value is not an int in [MIN_RADIUS, MAX_RADIUS].
    """
    if not is_int(value) or not MIN_RADIUS <= value <= MAX_RADIUS:
        raise ValueError(f"{key}: implausible radius {value!r}")
    return int(value)


def profile_to_dict(profile: CalibrationProfile) -> dict[str, Any]:
    """Convert a profile to a JSON-serialisable dict.

    Args:
        profile: Profile to convert.

    Returns:
        dict[str, Any]: Dict with PROFILE_KEYS (offsets as lists).
    """
    return {
        "accel_offset": list(profile.accel_offset),
        "gyro_offset": list(profile.gyro_offset),
        "mag_offset": list(profile.mag_offset),
        "accel_radius": profile.accel_radius,
        "mag_radius": profile.mag_radius,
    }


def profile_from_dict(data: Any) -> CalibrationProfile:
    """Validate a dict and build a profile.

    Args:
        data: Parsed JSON value, expected to hold all PROFILE_KEYS.

    Returns:
        CalibrationProfile: The validated profile.

    Raises:
        ValueError: If data is not a dict, a key is missing, or a value is invalid.
    """
    if not isinstance(data, dict):
        raise ValueError(f"profile must be an object, got {type(data).__name__}")
    missing = [k for k in PROFILE_KEYS if k not in data]
    if missing:
        raise ValueError(f"profile missing keys: {missing}")
    return CalibrationProfile(
        accel_offset=parse_offset(data["accel_offset"], "accel_offset"),
        gyro_offset=parse_offset(data["gyro_offset"], "gyro_offset"),
        mag_offset=parse_offset(data["mag_offset"], "mag_offset"),
        accel_radius=parse_radius(data["accel_radius"], "accel_radius"),
        mag_radius=parse_radius(data["mag_radius"], "mag_radius"),
    )


def should_save(
    status: tuple | None,
    last_save_monotonic: float,
    now: float,
    interval: float,
    current_profile: CalibrationProfile | None,
    saved_profile: CalibrationProfile | None,
) -> bool:
    """Decide whether the calibration profile should be read from the chip and saved now.

    Args:
        status: (sys, gyro, accel, mag) calibration status; sys is ignored.
        last_save_monotonic: Monotonic time of the last save attempt.
        now: Current monotonic time.
        interval: Minimum seconds between attempts.
        current_profile: Profile just read from the chip, or None when not read yet.
        saved_profile: Profile known to be on disk, or None.

    Returns:
        bool: True when gyro, accel and mag are all 3, the interval elapsed, and the profile is not known to
            equal the saved one.
    """
    if not status or len(status) < 4 or any(v is None for v in status[:4]):
        return False
    if not all(status[i] == CALIBRATED for i in (1, 2, 3)):
        return False
    if now - last_save_monotonic < interval:
        return False
    return current_profile is None or current_profile != saved_profile


def enter_config_mode(bno: Any) -> None:
    """Switch the chip to CONFIG mode and verify it took effect.

    Args:
        bno: BNO055 driver instance.

    Raises:
        RuntimeError: If the chip does not report CONFIG mode after retries.
    """
    for _ in range(CONFIG_VERIFY_RETRIES):
        bno.mode = CONFIG_MODE_VALUE
        time.sleep(MODE_SETTLE_S)
        if bno.mode == CONFIG_MODE_VALUE:
            return
    raise RuntimeError("BNO055 did not enter CONFIG mode")


def apply_profile(bno: Any, profile: CalibrationProfile) -> None:
    """Write a profile to the chip. Leaves the chip in CONFIG mode; the caller selects the operation mode next.

    Args:
        bno: BNO055 driver instance.
        profile: Profile to write.
    """
    enter_config_mode(bno)
    bno.offsets_accelerometer = profile.accel_offset
    bno.offsets_gyroscope = profile.gyro_offset
    bno.offsets_magnetometer = profile.mag_offset
    bno.radius_accelerometer = profile.accel_radius
    bno.radius_magnetometer = profile.mag_radius


def read_profile(bno: Any, operation_mode: int) -> CalibrationProfile:
    """Read the current offsets from the chip (CONFIG mode), then restore the operation mode.

    Args:
        bno: BNO055 driver instance.
        operation_mode: Mode register value to switch back to.

    Returns:
        CalibrationProfile: Raw values read (not validated).
    """
    enter_config_mode(bno)
    try:
        return CalibrationProfile(
            accel_offset=tuple(bno.offsets_accelerometer),
            gyro_offset=tuple(bno.offsets_gyroscope),
            mag_offset=tuple(bno.offsets_magnetometer),
            accel_radius=int(bno.radius_accelerometer),
            mag_radius=int(bno.radius_magnetometer),
        )
    finally:
        bno.mode = operation_mode
        time.sleep(MODE_SWITCH_DELAY_S)


def write_profile(path: Path, profile: CalibrationProfile) -> None:
    """Atomically write a profile as JSON (temp file in the same directory, then os.replace).

    Args:
        path: Destination file; the parent directory is created if missing.
        profile: Profile to write.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as handle:
            json.dump(profile_to_dict(profile), handle, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    except BaseException:
        Path(tmp_name).unlink(missing_ok=True)
        raise


def load_profile(path: Path) -> CalibrationProfile | None:
    """Load and validate a profile file.

    Args:
        path: Profile file.

    Returns:
        CalibrationProfile | None: The profile, or None when the file does not exist.

    Raises:
        ValueError: If the file is unreadable, not JSON, or not a valid profile.
    """
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc
    return profile_from_dict(data)


def save_if_changed(
    bno: Any, path: Path, operation_mode: int, saved_profile: CalibrationProfile | None
) -> CalibrationProfile | None:
    """Read the chip profile and write it when valid and different from the saved one.

    Args:
        bno: BNO055 driver instance.
        path: Profile file.
        operation_mode: Mode register value to switch back to after reading.
        saved_profile: Profile known to be on disk, or None.

    Returns:
        CalibrationProfile | None: The profile now on disk (unchanged saved_profile when nothing was written).
    """
    current = read_profile(bno, operation_mode)
    try:
        current = profile_from_dict(profile_to_dict(current))
    except ValueError:
        return saved_profile
    if current == saved_profile:
        return saved_profile
    write_profile(path, current)
    return current
