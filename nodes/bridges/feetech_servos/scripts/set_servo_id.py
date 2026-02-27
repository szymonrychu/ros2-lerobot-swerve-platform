#!/usr/bin/env python3
"""Set a single Feetech servo's ID. Requires exactly one servo on the given device/baudrate.

Uses the st3215 library. Baudrate is passed for future use; st3215 uses a fixed default (1Mbps)
unless the library is extended to accept it.
"""

import argparse
import sys

from st3215 import ST3215

VALID_ID_MIN = 0
VALID_ID_MAX = 253


def main() -> int:
    """Run CLI: require exactly one servo on device, then set its ID to --new-id.

    Returns:
        int: 0 on success, 1 on validation or device/library error.
    """
    parser = argparse.ArgumentParser(
        description="Set new ID on a single Feetech servo (exactly one must be on the bus)."
    )
    parser.add_argument("--device", required=True, help="Serial device path (e.g. /dev/ttyUSB0)")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Baudrate (default: 1000000)",
    )
    parser.add_argument(
        "--new-id",
        type=int,
        required=True,
        help="New servo ID to assign (0-253)",
    )
    args = parser.parse_args()

    if not (VALID_ID_MIN <= args.new_id <= VALID_ID_MAX):
        print(
            f"Error: new-id must be between {VALID_ID_MIN} and {VALID_ID_MAX}",
            file=sys.stderr,
        )
        return 1

    try:
        servo = ST3215(args.device)
    except (ValueError, OSError) as e:
        print(f"Error opening device {args.device}: {e}", file=sys.stderr)
        return 1

    try:
        ids = servo.ListServos()
    except (OSError, ValueError, RuntimeError) as e:
        print(f"Error scanning for servos: {e}", file=sys.stderr)
        return 1

    if ids is None:
        print("Error: could not scan for servos", file=sys.stderr)
        return 1

    if len(ids) != 1:
        print(
            f"Error: expected exactly one Feetech servo on {args.device}, found {len(ids)}: {ids}",
            file=sys.stderr,
        )
        return 1

    current_id = ids[0]
    if current_id == args.new_id:
        print(f"Servo already has ID {args.new_id}. Nothing to do.")
        return 0

    err = servo.ChangeId(current_id, args.new_id)
    if err is not None:
        print(f"Error changing ID: {err}", file=sys.stderr)
        return 1

    print(f"Servo ID changed from {current_id} to {args.new_id}.")
    return 0
