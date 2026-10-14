"""Save-now and reset calibration service logic, free of ROS so it is testable on a fake driver.

The node wraps these in std_srvs/Trigger callbacks that run on its loop thread (spin_once inside the read loop), so
every I2C access here is serialised with the read cycle.
"""

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .calibration import (
    CALIBRATED,
    CalibrationProfile,
    profile_from_dict,
    profile_to_dict,
    read_profile,
    write_profile,
)

SYS_TRIGGER_REGISTER = 0x3F  # datasheet section 4.3.63
SYS_TRIGGER_RST_SYS = 0x20  # reset the whole chip; it reboots into CONFIG mode with no calibration
CHIP_RESET_WAIT_S = 0.65  # BNO055 boot time after RST_SYS
CODE_CALIBRATION_RESET = "calibration_reset"
STATUS_KEYS = ("sys", "gyro", "accel", "mag")
MOVING = "robot is moving"


@dataclass(frozen=True)
class SaveOutcome:
    """Result of a save-now request.

    Attributes:
        success: True when the chip profile was written to the calibration file.
        message: Saved statuses and file, or the refusal or error.
        profile: The profile now on disk, or None when nothing was written.
        chip_read: True when the CONFIG-mode round trip ran (the calibration episode counts as read).
    """

    success: bool
    message: str
    profile: CalibrationProfile | None = None
    chip_read: bool = False


@dataclass(frozen=True)
class ResetOutcome:
    """Result of a reset request.

    Attributes:
        success: True when the file is gone and the chip was reset.
        message: What was done, or the refusal or error.
        reinit: True when the caller must run the full re-init path without restoring a profile.
    """

    success: bool
    message: str
    reinit: bool = False


def write_reset(bno: Any) -> None:
    """Set RST_SYS in SYS_TRIGGER and wait for the chip to reboot.

    Args:
        bno (Any): Driver instance exposing _write_register (adafruit_bno055.BNO055_I2C).

    Raises:
        Exception: Whatever the register write raised (OSError on a bus error).
    """
    bno._write_register(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)
    time.sleep(CHIP_RESET_WAIT_S)


def chip_reset(bno: Any) -> bool:
    """Reset the BNO055 via SYS_TRIGGER RST_SYS and wait for it to reboot; never raises.

    Args:
        bno (Any): Driver instance exposing _write_register (adafruit_bno055.BNO055_I2C).

    Returns:
        bool: True if the reset was issued, False if the driver lacks _write_register or the write failed.
    """
    try:
        write_reset(bno)
    except Exception:  # noqa: BLE001
        return False
    return True


def status_text(status: tuple) -> str:
    """Format a calibration status for a service message.

    Args:
        status (tuple): (sys, gyro, accel, mag).

    Returns:
        str: e.g. "sys 3, gyro 3, accel 1, mag 3".
    """
    return ", ".join(f"{key} {int(status[i])}" for i, key in enumerate(STATUS_KEYS))


def save_calibration(
    bno: Any, path: Path | None, operation_mode: int, status: tuple | None, stationary: bool | None
) -> SaveOutcome:
    """Read the chip profile and write it to the calibration file, even when unchanged.

    Args:
        bno (Any): BNO055 driver instance.
        path (Path | None): Calibration file; None means persistence is disabled.
        operation_mode (int): Mode register value to switch back to after the CONFIG-mode read.
        status (tuple | None): Latest (sys, gyro, accel, mag), or None when unreadable.
        stationary (bool | None): Robot stillness from wheel odometry; None when unknown (gyro bias estimator off).

    Returns:
        SaveOutcome: success with the saved statuses and file, or the reason nothing was written.
    """
    if path is None:
        return SaveOutcome(False, "calibration_file disabled")
    if not status or len(status) < len(STATUS_KEYS) or any(v is None for v in status[: len(STATUS_KEYS)]):
        return SaveOutcome(False, "calibration status unavailable")
    if status[3] != CALIBRATED:
        return SaveOutcome(False, f"mag is {int(status[3])}, needs {CALIBRATED}")
    if stationary is False:
        return SaveOutcome(False, MOVING)
    try:
        profile = profile_from_dict(profile_to_dict(read_profile(bno, operation_mode)))
    except (RuntimeError, OSError, ValueError, TypeError) as exc:
        return SaveOutcome(False, f"reading the chip profile failed: {exc}", chip_read=True)
    try:
        write_profile(path, profile)
    except OSError as exc:
        return SaveOutcome(False, f"writing {path} failed: {exc}", chip_read=True)
    return SaveOutcome(True, f"saved calibration ({status_text(status)}) to {path}", profile, chip_read=True)


def reset_calibration(bno: Any, path: Path | None, stationary: bool | None) -> ResetOutcome:
    """Delete the calibration file and reset the chip; the caller then re-initialises it without a profile.

    Args:
        bno (Any): BNO055 driver instance.
        path (Path | None): Calibration file; None means persistence is disabled (only the chip is reset).
        stationary (bool | None): Robot stillness from wheel odometry; None when unknown (gyro bias estimator off).

    Returns:
        ResetOutcome: success with reinit=True, or the refusal or error (reinit=False, chip untouched unless the
            message says otherwise).
    """
    if stationary is False:
        return ResetOutcome(False, MOVING)
    if path is not None:
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            return ResetOutcome(False, f"deleting {path} failed: {exc}")
    try:
        write_reset(bno)
    except Exception as exc:  # noqa: BLE001
        deleted = f"{path} deleted, but " if path is not None else ""
        return ResetOutcome(False, f"{deleted}chip reset failed: {exc}")
    removed = f"deleted {path}, " if path is not None else ""
    return ResetOutcome(True, f"{removed}chip reset; re-initialising uncalibrated", reinit=True)
