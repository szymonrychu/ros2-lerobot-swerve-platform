"""Feetech STS3215 register map: read all / write with EPROM safety (unlock, write, lock)."""

from dataclasses import dataclass
from typing import Any

# STS register addresses (decimal) from Feetech docs; L = low byte for 2-byte registers.
STS_MODEL_L = 3
STS_ID = 5
STS_BAUD_RATE = 6
STS_MIN_ANGLE_LIMIT_L = 9
STS_MAX_ANGLE_LIMIT_L = 11
STS_MAX_TEMPERATURE_LIMIT = 13
STS_MAX_VOLTAGE_LIMIT = 14
STS_MIN_VOLTAGE_LIMIT = 15
STS_MAX_TORQUE_LIMIT_L = 16
STS_PHASE = 18
STS_UNLOADING_CONDITION = 19
STS_LED_ALARM_CONDITION = 20
STS_P_COEFFICIENT = 21
STS_D_COEFFICIENT = 22
STS_I_COEFFICIENT = 23
STS_MIN_STARTUP_FORCE = 24
STS_CW_DEAD = 26
STS_CCW_DEAD = 27
STS_PROTECTION_CURRENT = 28
STS_ANGULAR_RESOLUTION = 30
STS_OFS_L = 31
STS_MODE = 33
STS_PROTECTIVE_TORQUE = 34
STS_PROTECTION_TIME_L = 35
STS_OVERLOAD_TORQUE = 36
STS_SPEED_P_COEFFICIENT = 37
STS_OVER_CURRENT_PROTECTION_TIME_L = 38
STS_VELOCITY_I_COEFFICIENT_L = 39
STS_TORQUE_ENABLE = 40
STS_ACC = 41
STS_GOAL_POSITION_L = 42
STS_GOAL_TIME_L = 44
STS_GOAL_SPEED_L = 46
STS_LOCK = 55
STS_PRESENT_POSITION_L = 56
STS_PRESENT_SPEED_L = 58
STS_PRESENT_LOAD_L = 60
STS_PRESENT_VOLTAGE = 62
STS_PRESENT_TEMPERATURE = 63
STS_STATUS = 65
STS_MOVING = 66
STS_PRESENT_CURRENT_L = 69


@dataclass
class RegisterEntry:
    """Single register metadata.

    Includes address, size, name, read-only status, and EPROM flag
    (EPROM writes require unlock/write/lock sequence).
    """

    address: int
    size: int  # 1 or 2
    name: str
    read_only: bool
    eprom: bool  # if writable and in EPROM, use UnLockEprom -> write -> LockEprom


# Full register map: address, size, name, read_only, eprom. Order matches typical docs.
REGISTER_MAP: list[RegisterEntry] = [
    RegisterEntry(STS_MODEL_L, 2, "model", True, False),
    RegisterEntry(STS_ID, 1, "id", False, True),
    RegisterEntry(STS_BAUD_RATE, 1, "baudrate", False, True),
    RegisterEntry(STS_MIN_ANGLE_LIMIT_L, 2, "min_angle_limit", False, True),
    RegisterEntry(STS_MAX_ANGLE_LIMIT_L, 2, "max_angle_limit", False, True),
    RegisterEntry(STS_MAX_TEMPERATURE_LIMIT, 1, "max_temperature_limit", False, True),
    RegisterEntry(STS_MAX_VOLTAGE_LIMIT, 1, "max_voltage_limit", False, True),
    RegisterEntry(STS_MIN_VOLTAGE_LIMIT, 1, "min_voltage_limit", False, True),
    RegisterEntry(STS_MAX_TORQUE_LIMIT_L, 2, "max_torque_limit", False, True),
    RegisterEntry(STS_PHASE, 1, "phase", False, True),
    RegisterEntry(STS_UNLOADING_CONDITION, 1, "unloading_condition", False, True),
    RegisterEntry(STS_LED_ALARM_CONDITION, 1, "led_alarm_condition", False, True),
    RegisterEntry(STS_P_COEFFICIENT, 1, "p_coefficient", False, True),
    RegisterEntry(STS_D_COEFFICIENT, 1, "d_coefficient", False, True),
    RegisterEntry(STS_I_COEFFICIENT, 1, "i_coefficient", False, True),
    RegisterEntry(STS_MIN_STARTUP_FORCE, 1, "min_startup_force", False, True),
    RegisterEntry(STS_CW_DEAD, 1, "cw_dead_zone", False, True),
    RegisterEntry(STS_CCW_DEAD, 1, "ccw_dead_zone", False, True),
    RegisterEntry(STS_PROTECTION_CURRENT, 1, "protection_current", False, True),
    RegisterEntry(STS_ANGULAR_RESOLUTION, 1, "angular_resolution", False, True),
    RegisterEntry(STS_OFS_L, 2, "offset", False, True),
    RegisterEntry(STS_MODE, 1, "mode", False, False),  # RAM
    RegisterEntry(STS_PROTECTIVE_TORQUE, 1, "protective_torque", False, True),
    RegisterEntry(STS_PROTECTION_TIME_L, 2, "protection_time", False, True),
    RegisterEntry(STS_OVERLOAD_TORQUE, 1, "overload_torque", False, True),
    RegisterEntry(STS_SPEED_P_COEFFICIENT, 1, "speed_p_coefficient", False, True),
    RegisterEntry(STS_OVER_CURRENT_PROTECTION_TIME_L, 2, "over_current_protection_time", False, True),
    RegisterEntry(STS_VELOCITY_I_COEFFICIENT_L, 2, "velocity_i_coefficient", False, True),
    RegisterEntry(STS_TORQUE_ENABLE, 1, "torque_enable", False, False),  # RAM
    RegisterEntry(STS_ACC, 1, "acceleration", False, False),  # RAM
    RegisterEntry(STS_GOAL_POSITION_L, 2, "goal_position", False, False),  # RAM
    RegisterEntry(STS_GOAL_TIME_L, 2, "goal_time", False, False),  # RAM
    RegisterEntry(STS_GOAL_SPEED_L, 2, "goal_speed", False, False),  # RAM
    RegisterEntry(STS_LOCK, 1, "lock", False, False),  # special: used internally for EPROM lock
    RegisterEntry(STS_PRESENT_POSITION_L, 2, "present_position", True, False),
    RegisterEntry(STS_PRESENT_SPEED_L, 2, "present_speed", True, False),
    RegisterEntry(STS_PRESENT_LOAD_L, 2, "present_load", True, False),
    RegisterEntry(STS_PRESENT_VOLTAGE, 1, "present_voltage", True, False),
    RegisterEntry(STS_PRESENT_TEMPERATURE, 1, "present_temperature", True, False),
    RegisterEntry(STS_STATUS, 1, "status", True, False),
    RegisterEntry(STS_MOVING, 1, "moving", True, False),
    RegisterEntry(STS_PRESENT_CURRENT_L, 2, "present_current", True, False),
]

# Registers we expose for writing via ROS2 (exclude 'lock' and read-only).
WRITABLE_REGISTER_NAMES: set[str] = {r.name for r in REGISTER_MAP if not r.read_only and r.name != "lock"}


def read_register(
    servo: Any,
    sts_id: int,
    entry: RegisterEntry,
) -> int | None:
    """Read one register from the servo. Returns raw value or None on error."""
    if entry.size == 1:
        val, comm, err = servo.read1ByteTxRx(sts_id, entry.address)
    else:
        val, comm, err = servo.read2ByteTxRx(sts_id, entry.address)
    if comm != 0 or err != 0:
        return None
    return val


def read_all_registers(servo: Any, sts_id: int) -> dict[str, int]:
    """Read all registers for one servo. Returns dict name -> raw value; missing/error as None (omitted)."""
    out: dict[str, int] = {}
    for entry in REGISTER_MAP:
        val = read_register(servo, sts_id, entry)
        if val is not None:
            out[entry.name] = val
    return out


def write_register(
    servo: Any,
    sts_id: int,
    entry: RegisterEntry,
    value: int,
    last_written: dict[str, int],
) -> bool:
    """Write one register. Uses EPROM unlock/write/lock when entry.eprom is True. Skips write if value unchanged.

    Args:
        servo: ST3215 instance.
        sts_id: Servo ID.
        entry: Register definition.
        value: Value to write (raw).
        last_written: Cache of last written values per name; updated on success.

    Returns:
        True if write succeeded or was skipped (unchanged), False on error.
    """
    if entry.read_only or entry.name == "lock":
        return False
    if last_written.get(entry.name) == value:
        return True  # avoid unnecessary write
    if entry.eprom:
        if servo.UnLockEprom(sts_id) != 0:
            return False
    try:
        if entry.size == 1:
            comm, err = servo.write1ByteTxRx(sts_id, entry.address, value)
        else:
            comm, err = servo.write2ByteTxRx(sts_id, entry.address, value)
        if comm != 0 or err != 0:
            return False
        last_written[entry.name] = value
        return True
    finally:
        if entry.eprom:
            servo.LockEprom(sts_id)


def get_register_entry_by_name(name: str) -> RegisterEntry | None:
    """Return RegisterEntry for a given name, or None."""
    for r in REGISTER_MAP:
        if r.name == name:
            return r
    return None
