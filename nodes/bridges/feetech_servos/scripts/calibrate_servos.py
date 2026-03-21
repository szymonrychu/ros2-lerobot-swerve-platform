#!/usr/bin/env python3
"""Feetech servo calibration and register tool (argparse subcommands).

Supports: interactive calibration (multi-joint), single-servo register read/write,
min/max limit set/clear, and listing register map. Uses st3215 and registers from
the feetech_servos package.
"""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

# Allow importing registers when script is run from scripts/ or tests
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from registers import (  # noqa: E402
    REGISTER_MAP,
    get_register_entry_by_name,
    read_all_registers,
    read_register,
    write_register,
)
from st3215 import ST3215  # noqa: E402

# Legacy cal.json keys mapped to register names when loading config
_LEGACY_CONFIG_KEYS: dict[str, str] = {"min": "min_angle_limit", "max": "max_angle_limit"}
# "center" is not written to a register; omit or use offset in extended format
READ_ONLY_REGISTER_NAMES: set[str] = {r.name for r in REGISTER_MAP if r.read_only}

# Constants
DEFAULT_EXPECTED_JOINT_COUNT = 6
DEFAULT_CENTER = 2048
SAMPLING_DURATION_S = 3.0
LIMIT_MIN_STEPS = 0
LIMIT_MAX_STEPS = 4095
LIMITS_CLEAR_MIN = 0
LIMITS_CLEAR_MAX = 4095


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    """Add --device, --baudrate, --id (default 1) to a subparser."""
    parser.add_argument(
        "--device",
        required=True,
        help="Serial port to the servo bus (e.g. /dev/ttyUSB0 or /dev/serial/by-id/...).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="Serial baudrate for STS3215 bus (default: 1000000).",
    )
    parser.add_argument(
        "--id",
        type=int,
        default=1,
        dest="servo_id",
        metavar="ID",
        help="Target servo ID on the bus (0-253). Default: 1.",
    )


def _open_servo(device: str, baudrate: int = 1_000_000) -> Any:
    """Open ST3215 on device. Raises ValueError/OSError on failure."""
    return ST3215(device)


def cmd_calibrate(servo: Any, args: argparse.Namespace) -> int:
    """Run interactive calibration: neutral then min/max per joint; write JSON to --output."""
    ids = servo.ListServos()
    if ids is None:
        print("Error: could not scan for servos", file=sys.stderr)
        return 1
    expected = getattr(args, "expected_joints", DEFAULT_EXPECTED_JOINT_COUNT)
    if len(ids) < expected:
        print(
            f"Error: expected at least {expected} joints, found {len(ids)}: {ids}",
            file=sys.stderr,
        )
        return 1
    joint_ids = sorted(ids)[:expected]

    for jid in joint_ids:
        servo.StopServo(jid)
    time.sleep(0.3)

    result: dict[int, dict[str, int]] = {}

    print("\n--- Neutral position (center) ---")
    for jid in joint_ids:
        input(f"Move joint (servo ID {jid}) to neutral position, then press Enter...")
        if servo.DefineMiddle(jid) is not True:
            print(f"Error: failed to set center for servo {jid}", file=sys.stderr)
            return 1
        center = servo.ReadPosition(jid)
        if center is None:
            center = DEFAULT_CENTER
        result[jid] = {"min": 0, "center": center, "max": 0}
        print(f"  Servo {jid}: center = {center}")

    print("\n--- Full motion range (min/max) ---")
    last_written: dict[str, int] = {}
    min_entry = get_register_entry_by_name("min_angle_limit")
    max_entry = get_register_entry_by_name("max_angle_limit")
    if not min_entry or not max_entry:
        print("Error: min/max angle limit registers not found", file=sys.stderr)
        return 1
    for jid in joint_ids:
        print(f"Move joint (servo ID {jid}) through its full range, then press Enter...")
        input()
        positions: list[int] = []
        print("  Sampling position for 3 seconds...")
        deadline = time.monotonic() + SAMPLING_DURATION_S
        while time.monotonic() < deadline:
            pos = servo.ReadPosition(jid)
            if pos is not None:
                positions.append(pos)
            time.sleep(0.05)
        if not positions:
            print(f"Error: could not read position for servo {jid}", file=sys.stderr)
            return 1
        min_pos = min(positions)
        max_pos = max(positions)
        if not write_register(servo, jid, min_entry, min_pos, last_written):
            print(f"Error: failed to write min angle to servo {jid}", file=sys.stderr)
            return 1
        if not write_register(servo, jid, max_entry, max_pos, last_written):
            print(f"Error: failed to write max angle to servo {jid}", file=sys.stderr)
            return 1
        result[jid]["min"] = min_pos
        result[jid]["max"] = max_pos
        print(f"  Servo {jid}: min={min_pos}, center={result[jid]['center']}, max={max_pos}")

    for jid in joint_ids:
        servo.StartServo(jid)

    out_path = Path(args.output)
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nCalibration saved to {out_path}")
    return 0


def cmd_read(servo: Any, args: argparse.Namespace) -> int:
    """Read one register by name or all registers (--all)."""
    sid = args.servo_id
    if getattr(args, "read_all", False):
        data = read_all_registers(servo, sid)
        print(json.dumps(data, indent=2))
        return 0
    reg_name = getattr(args, "register", None)
    if not reg_name:
        print("Error: --register <name> required (or use --all)", file=sys.stderr)
        return 1
    entry = get_register_entry_by_name(reg_name)
    if entry is None:
        print(f"Error: unknown register '{reg_name}'", file=sys.stderr)
        return 1
    val = read_register(servo, sid, entry)
    if val is None:
        print(f"Error: failed to read register '{reg_name}' from servo {sid}", file=sys.stderr)
        return 1
    print(val)
    return 0


def cmd_get(servo: Any, args: argparse.Namespace) -> int:
    """Get one register or all registers (RO or RW). Same behavior as read."""
    return cmd_read(servo, args)


def cmd_set(servo: Any, args: argparse.Namespace) -> int:
    """Set one register by name (RW only). Refuses read-only with detailed message."""
    reg_name = args.register
    value = args.value
    entry = get_register_entry_by_name(reg_name)
    if entry is None:
        print(f"Error: unknown register '{reg_name}'", file=sys.stderr)
        return 1
    if entry.read_only:
        print(
            f"Error: cannot set '{reg_name}': register is read-only. Use 'get' to read it.",
            file=sys.stderr,
        )
        return 1
    if entry.name == "lock":
        print("Error: writing 'lock' register is not allowed via this tool", file=sys.stderr)
        return 1
    last_written: dict[str, int] = {}
    if not write_register(servo, args.servo_id, entry, value, last_written):
        print(f"Error: failed to write register '{reg_name}' = {value}", file=sys.stderr)
        return 1
    print("OK")
    return 0


def cmd_write(servo: Any, args: argparse.Namespace) -> int:
    """Write one register by name."""
    reg_name = args.register
    value = args.value
    entry = get_register_entry_by_name(reg_name)
    if entry is None:
        print(f"Error: unknown register '{reg_name}'", file=sys.stderr)
        return 1
    if entry.read_only:
        print(f"Error: register '{reg_name}' is read-only", file=sys.stderr)
        return 1
    if entry.name == "lock":
        print("Error: writing 'lock' register is not allowed via this tool", file=sys.stderr)
        return 1
    last_written: dict[str, int] = {}
    if not write_register(servo, args.servo_id, entry, value, last_written):
        print(f"Error: failed to write register '{reg_name}' = {value}", file=sys.stderr)
        return 1
    print("OK")
    return 0


def cmd_limits_set(servo: Any, args: argparse.Namespace) -> int:
    """Set min/max angle limits (0..4095, min <= max)."""
    min_pos = args.min_steps
    max_pos = args.max_steps
    if min_pos < LIMIT_MIN_STEPS or max_pos > LIMIT_MAX_STEPS:
        print(
            f"Error: min and max must be in [{LIMIT_MIN_STEPS}, {LIMIT_MAX_STEPS}]",
            file=sys.stderr,
        )
        return 1
    if min_pos > max_pos:
        print("Error: min must be <= max", file=sys.stderr)
        return 1
    min_entry = get_register_entry_by_name("min_angle_limit")
    max_entry = get_register_entry_by_name("max_angle_limit")
    if not min_entry or not max_entry:
        print("Error: min/max angle limit registers not found", file=sys.stderr)
        return 1
    last_written: dict[str, int] = {}
    if not write_register(servo, args.servo_id, min_entry, min_pos, last_written):
        print("Error: failed to write min_angle_limit", file=sys.stderr)
        return 1
    if not write_register(servo, args.servo_id, max_entry, max_pos, last_written):
        print("Error: failed to write max_angle_limit", file=sys.stderr)
        return 1
    print("OK")
    return 0


def cmd_limits_get(servo: Any, args: argparse.Namespace) -> int:
    """Read min and max angle limits from one servo; print as JSON."""
    min_entry = get_register_entry_by_name("min_angle_limit")
    max_entry = get_register_entry_by_name("max_angle_limit")
    if not min_entry or not max_entry:
        print("Error: min/max angle limit registers not found", file=sys.stderr)
        return 1
    min_val = read_register(servo, args.servo_id, min_entry)
    max_val = read_register(servo, args.servo_id, max_entry)
    if min_val is None:
        print("Error: failed to read min_angle_limit", file=sys.stderr)
        return 1
    if max_val is None:
        print("Error: failed to read max_angle_limit", file=sys.stderr)
        return 1
    print(json.dumps({"min": min_val, "max": max_val}))
    return 0


def cmd_limits_clear(servo: Any, args: argparse.Namespace) -> int:
    """Clear min/max limits to full range (0, 4095)."""
    min_entry = get_register_entry_by_name("min_angle_limit")
    max_entry = get_register_entry_by_name("max_angle_limit")
    if not min_entry or not max_entry:
        print("Error: min/max angle limit registers not found", file=sys.stderr)
        return 1
    last_written: dict[str, int] = {}
    if not write_register(servo, args.servo_id, min_entry, LIMITS_CLEAR_MIN, last_written):
        print("Error: failed to write min_angle_limit", file=sys.stderr)
        return 1
    if not write_register(servo, args.servo_id, max_entry, LIMITS_CLEAR_MAX, last_written):
        print("Error: failed to write max_angle_limit", file=sys.stderr)
        return 1
    print("OK")
    return 0


def _register_name_from_config_key(key: str) -> str:
    """Return register name for a config key; legacy 'min'/'max' map to angle limits."""
    return _LEGACY_CONFIG_KEYS.get(key, key)


def _validate_config_no_readonly(config: dict[str, dict[str, int]]) -> list[str]:
    """
    Return list of read-only register names that appear in the config.
    Config is { servo_id_str: { register_key: value } }. Keys can be legacy (min/max) or register names.
    """
    ro_found: list[str] = []
    for servo_regs in config.values():
        if not isinstance(servo_regs, dict):
            continue
        for key in servo_regs:
            reg_name = _register_name_from_config_key(key)
            if reg_name in READ_ONLY_REGISTER_NAMES:
                ro_found.append(reg_name)
    return sorted(set(ro_found))


def cmd_load_config(servo: Any, args: argparse.Namespace) -> int:
    """Load extended cal.json and apply to servos; refuse if file contains read-only registers."""
    path = Path(args.config_file)
    if not path.exists():
        print(f"Error: config file not found: {path}", file=sys.stderr)
        return 1
    try:
        with open(path) as f:
            config = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error: invalid JSON in {path}: {e}", file=sys.stderr)
        return 1
    if not isinstance(config, dict):
        print("Error: config root must be an object (servo_id -> registers)", file=sys.stderr)
        return 1
    ro_found = _validate_config_no_readonly(config)
    if ro_found:
        print(
            "Error: refusing to load config because the following registers are read-only "
            "and cannot be set from file:",
            file=sys.stderr,
        )
        for name in ro_found:
            print(f"  - {name}", file=sys.stderr)
        print("Remove them from the file or use 'get' to read their values.", file=sys.stderr)
        return 1
    for servo_id_str, regs in config.items():
        if not isinstance(regs, dict):
            continue
        try:
            sid = int(servo_id_str)
        except (ValueError, TypeError):
            print(f"Error: invalid servo ID in config: {servo_id_str}", file=sys.stderr)
            return 1
        last_written: dict[str, int] = {}
        for key, value in regs.items():
            if not isinstance(value, int):
                print(f"Error: value for servo {sid} register '{key}' must be integer", file=sys.stderr)
                return 1
            reg_name = _register_name_from_config_key(key)
            if reg_name == "center":
                continue
            entry = get_register_entry_by_name(reg_name)
            if entry is None:
                print(f"Error: unknown register '{reg_name}' (from key '{key}') in config", file=sys.stderr)
                return 1
            if entry.read_only or entry.name == "lock":
                continue
            if not write_register(servo, sid, entry, value, last_written):
                print(f"Error: failed to write {reg_name}={value} to servo {sid}", file=sys.stderr)
                return 1
    print("OK")
    return 0


def cmd_dump_config(servo: Any, args: argparse.Namespace) -> int:
    """Read all registers for each specified servo and print extended JSON to stdout or --output."""
    ids = getattr(args, "servo_ids", None)
    if ids is None:
        ids = [getattr(args, "servo_id", 1)]
    result: dict[str, dict[str, int]] = {}
    for sid in ids:
        data = read_all_registers(servo, sid)
        result[str(sid)] = data
    out = getattr(args, "output", None)
    if out:
        with open(Path(out), "w") as f:
            json.dump(result, f, indent=2)
        print(f"Config dumped to {out}")
    else:
        print(json.dumps(result, indent=2))
    return 0


def cmd_list_registers(_servo: Any, _args: argparse.Namespace) -> int:
    """Print register map (name, address, size, read_only, eprom)."""
    rows = []
    for r in REGISTER_MAP:
        rows.append(
            {
                "name": r.name,
                "address": r.address,
                "size": r.size,
                "read_only": r.read_only,
                "eprom": r.eprom,
            }
        )
    print(json.dumps(rows, indent=2))
    return 0


def _build_parser() -> argparse.ArgumentParser:
    """Build and return the argument parser (for testing and main)."""
    parser = argparse.ArgumentParser(
        description=(
            "Feetech STS3215 servo calibration and register tool. "
            "Use subcommands to run interactive multi-joint calibration, read/write "
            "registers by name, set or clear min/max angle limits, or list the register map. "
            "Single-servo commands use --device and optionally --id (default 1)."
        ),
        epilog=(
            "Examples:\n"
            "  %(prog)s calibrate --device /dev/ttyUSB0 --output cal.json\n"
            "  %(prog)s read --device /dev/ttyUSB0 --register present_position\n"
            "  %(prog)s write --device /dev/ttyUSB0 --id 2 --register goal_position --value 2048\n"
            "  %(prog)s limits-set --device /dev/ttyUSB0 --min 100 --max 4000\n"
            "  %(prog)s limits-get --device /dev/ttyUSB0 --id 1\n"
            "  %(prog)s get --device /dev/ttyUSB0 --register present_position\n"
            "  %(prog)s set --device /dev/ttyUSB0 --register goal_position --value 2048\n"
            "  %(prog)s load-config --device /dev/ttyUSB0 --file cal.json\n"
            "  %(prog)s dump-config --device /dev/ttyUSB0 --id 1 2 --output config.json\n"
            "  %(prog)s list-registers\n"
            "Register names (e.g. present_position, min_angle_limit) are defined in registers.py; "
            "use list-registers to print the full map."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
        help="Choose one of the commands below.",
    )

    # calibrate
    p_cal = subparsers.add_parser(
        "calibrate",
        help="Interactive calibration: set neutral (center) and min/max range per joint, write to EEPROM and JSON.",
        description=(
            "Run interactive calibration for a multi-joint arm. Disables torque, prompts you to move "
            "each joint to neutral then through full range; samples positions and writes min/max limits "
            "to each servo EEPROM. Outputs a JSON file with min/center/max per servo ID."
        ),
    )
    p_cal.add_argument(
        "--device",
        required=True,
        help="Serial port to the servo bus (e.g. /dev/ttyUSB0).",
    )
    p_cal.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="Serial baudrate (default: 1000000).",
    )
    p_cal.add_argument(
        "--output",
        required=True,
        help="Path for the output JSON file (min/center/max per servo ID).",
    )
    p_cal.add_argument(
        "--expected-joints",
        type=int,
        default=DEFAULT_EXPECTED_JOINT_COUNT,
        help=f"Joints to calibrate; fails if fewer found (default: {DEFAULT_EXPECTED_JOINT_COUNT}).",
    )
    p_cal.set_defaults(_cmd=cmd_calibrate)

    # read
    p_read = subparsers.add_parser(
        "read",
        help="Read a single register or all registers from one servo.",
        description=(
            "Read register value(s) from the specified servo. Use --register <name> for one register "
            "(prints the raw integer value), or --all for a JSON map of all registers. "
            "Register names match the map in registers.py (e.g. present_position, goal_position)."
        ),
    )
    _add_connection_args(p_read)
    read_group = p_read.add_mutually_exclusive_group(required=True)
    read_group.add_argument(
        "--register",
        dest="register",
        help="Name of the register to read (e.g. present_position, min_angle_limit).",
    )
    read_group.add_argument(
        "--all",
        dest="read_all",
        action="store_true",
        help="Read every register and print as a JSON object (name -> value).",
    )
    p_read.set_defaults(_cmd=cmd_read)

    # get (alias for read: any register RO or RW)
    p_get = subparsers.add_parser(
        "get",
        help="Get one or all registers from one servo (read-only and read-write).",
        description=(
            "Read register value(s). Use --register <name> for one register, or --all for all. "
            "Works for both read-only and read-write registers. Use 'set' to write writable registers."
        ),
    )
    _add_connection_args(p_get)
    get_group = p_get.add_mutually_exclusive_group(required=True)
    get_group.add_argument("--register", dest="register", help="Register name to read.")
    get_group.add_argument("--all", dest="read_all", action="store_true", help="Read every register (JSON).")
    p_get.set_defaults(_cmd=cmd_get)

    # set (write one RW register; refuse RO with detailed error)
    p_set = subparsers.add_parser(
        "set",
        help="Set one register on one servo (read-write registers only).",
        description=(
            "Write a value to a writable register. Read-only registers are refused with a clear error; "
            "use 'get' to read them. The 'lock' register cannot be set."
        ),
    )
    _add_connection_args(p_set)
    p_set.add_argument("--register", required=True, help="Register name to write (must be read-write).")
    p_set.add_argument("--value", type=int, required=True, help="Integer value to write.")
    p_set.set_defaults(_cmd=cmd_set)

    # write
    p_write = subparsers.add_parser(
        "write",
        help="Write a value to a single register on one servo.",
        description=(
            "Write an integer value to a writable register. EPROM registers (e.g. min_angle_limit) "
            "are written with unlock/write/lock. Read-only registers and 'lock' are rejected."
        ),
    )
    _add_connection_args(p_write)
    p_write.add_argument(
        "--register",
        required=True,
        help="Register name to write (e.g. goal_position, torque_enable, min_angle_limit).",
    )
    p_write.add_argument(
        "--value",
        type=int,
        required=True,
        help="Integer value to write (range depends on register size).",
    )
    p_write.set_defaults(_cmd=cmd_write)

    # limits-set
    p_limits_set = subparsers.add_parser(
        "limits-set",
        help="Set min and max angle limits (in steps 0..4095) for one servo.",
        description=(
            "Write min_angle_limit and max_angle_limit to the servo EEPROM. "
            "Values must be in [0, 4095] with min <= max. Affects the allowed goal_position range."
        ),
    )
    _add_connection_args(p_limits_set)
    p_limits_set.add_argument(
        "--min",
        dest="min_steps",
        type=int,
        required=True,
        help="Minimum angle in steps (0..4095).",
    )
    p_limits_set.add_argument(
        "--max",
        dest="max_steps",
        type=int,
        required=True,
        help="Maximum angle in steps (0..4095). Must be >= --min.",
    )
    p_limits_set.set_defaults(_cmd=cmd_limits_set)

    # limits-get
    p_limits_get = subparsers.add_parser(
        "limits-get",
        help="Read min and max angle limits from one servo (output as JSON).",
        description=(
            "Read min_angle_limit and max_angle_limit from the specified servo and print "
            'as JSON: {"min": <steps>, "max": <steps>}. Complementary to limits-set and limits-clear.'
        ),
    )
    _add_connection_args(p_limits_get)
    p_limits_get.set_defaults(_cmd=cmd_limits_get)

    # limits-clear
    p_limits_clear = subparsers.add_parser(
        "limits-clear",
        help="Reset min/max angle limits to full range (0 and 4095).",
        description=(
            "Write min_angle_limit=0 and max_angle_limit=4095 to the servo EEPROM, "
            "restoring the full mechanical range."
        ),
    )
    _add_connection_args(p_limits_clear)
    p_limits_clear.set_defaults(_cmd=cmd_limits_clear)

    # load-config
    p_load = subparsers.add_parser(
        "load-config",
        help="Load extended cal.json and apply to servos (refuses read-only registers in file).",
        description=(
            "Load a JSON config file (extended cal.json: servo_id -> { register_name: value }). "
            "Applies only writable registers. If the file contains any read-only register key, "
            "exits with an error listing them. Legacy keys 'min' and 'max' map to "
            "min_angle_limit and max_angle_limit; 'center' is ignored."
        ),
    )
    p_load.add_argument("--device", required=True, help="Serial port to the servo bus.")
    p_load.add_argument("--baudrate", type=int, default=1_000_000, help="Serial baudrate (default: 1000000).")
    p_load.add_argument(
        "--file",
        dest="config_file",
        required=True,
        metavar="PATH",
        help="Path to the JSON config file to load.",
    )
    p_load.set_defaults(_cmd=cmd_load_config)

    # dump-config
    p_dump = subparsers.add_parser(
        "dump-config",
        help="Read all registers for one or more servos and write extended JSON.",
        description=(
            "Read every register for each specified servo and output extended config JSON "
            "(servo_id -> { register_name: value }). Write to --output file or stdout."
        ),
    )
    p_dump.add_argument("--device", required=True, help="Serial port to the servo bus.")
    p_dump.add_argument("--baudrate", type=int, default=1_000_000, help="Serial baudrate (default: 1000000).")
    p_dump.add_argument(
        "--id",
        dest="servo_ids",
        type=int,
        nargs="*",
        default=None,
        metavar="ID",
        help="Servo ID(s) to dump; default 1 if omitted.",
    )
    p_dump.add_argument(
        "--output",
        dest="output",
        default=None,
        metavar="PATH",
        help="Write JSON to this file; if omitted, print to stdout.",
    )
    p_dump.set_defaults(_cmd=cmd_dump_config)

    # list-registers
    p_list = subparsers.add_parser(
        "list-registers",
        help="Print the Feetech register map as JSON (no serial device required).",
        description=(
            "Output the full register map (name, address, size, read_only, eprom) as JSON. "
            "Use this to see valid register names for read/write and limits-set/limits-clear."
        ),
    )
    p_list.set_defaults(_cmd=cmd_list_registers, device=None, baudrate=1_000_000, servo_id=1)

    return parser


def main() -> int:
    """Dispatch to subcommand. Returns 0 on success, 1 on error."""
    parser = _build_parser()
    args = parser.parse_args()
    cmd_fn = args._cmd

    if args.command == "list-registers":
        return cmd_list_registers(None, args)

    try:
        servo = _open_servo(args.device, getattr(args, "baudrate", 1_000_000))
    except (ValueError, OSError) as e:
        print(f"Error opening device {args.device}: {e}", file=sys.stderr)
        return 1

    return cmd_fn(servo, args)


if __name__ == "__main__":
    sys.exit(main())
