#!/usr/bin/env python3
"""Interactive calibration for Feetech servo arms (e.g. 6-joint arm).

Procedure: connect and validate all joints; disable torque; for each joint set neutral (center),
then capture min/max range and write to servo; output JSON with min/center/max per servo ID.
Uses the st3215 library. Baudrate is passed for future use; st3215 uses a fixed default (1Mbps).
"""

import argparse
import json
import sys
import time
from pathlib import Path

# STS3215 EEPROM register addresses (from Feetech docs)
STS_MIN_ANGLE = 0x09
STS_MAX_ANGLE = 0x0B

# Default expected joint count for a typical 6-DOF arm
DEFAULT_EXPECTED_JOINT_COUNT = 6


def _write_min_max_angles(servo: object, sts_id: int, min_pos: int, max_pos: int) -> bool:
    """Write min and max angle limits to servo EEPROM. Uses library's UnLockEprom, write2ByteTxRx, LockEprom."""
    if servo.UnLockEprom(sts_id) != 0:  # COMM_SUCCESS is 0
        return False
    try:
        comm, err = servo.write2ByteTxRx(sts_id, STS_MIN_ANGLE, min_pos)
        if comm != 0 or err != 0:
            return False
        comm, err = servo.write2ByteTxRx(sts_id, STS_MAX_ANGLE, max_pos)
        if comm != 0 or err != 0:
            return False
    finally:
        servo.LockEprom(sts_id)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Interactive calibration for Feetech servo arm (neutral + min/max per joint)."
    )
    parser.add_argument("--device", required=True, help="Serial device path (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--output", required=True, help="Output JSON path for min/center/max per servo ID")
    parser.add_argument(
        "--expected-joints",
        type=int,
        default=DEFAULT_EXPECTED_JOINT_COUNT,
        help=f"Expected number of joints (default: {DEFAULT_EXPECTED_JOINT_COUNT})",
    )
    args = parser.parse_args()

    try:
        from st3215 import ST3215
    except ImportError:
        print("Error: st3215 library not installed (pip install st3215)", file=sys.stderr)
        return 1

    try:
        servo = ST3215(args.device)
    except (ValueError, OSError) as e:
        print(f"Error opening device {args.device}: {e}", file=sys.stderr)
        return 1

    ids = servo.ListServos()
    if ids is None:
        print("Error: could not scan for servos", file=sys.stderr)
        return 1

    if len(ids) < args.expected_joints:
        print(
            f"Error: expected at least {args.expected_joints} joints, found {len(ids)}: {ids}",
            file=sys.stderr,
        )
        return 1

    joint_ids = sorted(ids)[: args.expected_joints]

    # Disable torque for all
    for jid in joint_ids:
        servo.StopServo(jid)
    time.sleep(0.3)

    result: dict[int, dict[str, int]] = {}

    # Step 1: neutral (center) for each joint
    print("\n--- Neutral position (center) ---")
    for jid in joint_ids:
        input(f"Move joint (servo ID {jid}) to neutral position, then press Enter...")
        if servo.DefineMiddle(jid) is not True:
            print(f"Error: failed to set center for servo {jid}", file=sys.stderr)
            return 1
        center = servo.ReadPosition(jid)
        if center is None:
            center = 2048  # default center
        result[jid] = {"min": 0, "center": center, "max": 0}
        print(f"  Servo {jid}: center = {center}")

    # Step 2: min/max range for each joint
    print("\n--- Full motion range (min/max) ---")
    for jid in joint_ids:
        print(f"Move joint (servo ID {jid}) through its full range, then press Enter...")
        input()
        positions: list[int] = []
        print("  Sampling position for 3 seconds...")
        deadline = time.monotonic() + 3.0
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
        if not _write_min_max_angles(servo, jid, min_pos, max_pos):
            print(f"Error: failed to write min/max to servo {jid}", file=sys.stderr)
            return 1
        result[jid]["min"] = min_pos
        result[jid]["max"] = max_pos
        print(f"  Servo {jid}: min={min_pos}, center={result[jid]['center']}, max={max_pos}")

    # Re-enable torque
    for jid in joint_ids:
        servo.StartServo(jid)

    # Output JSON: { servo_id: { "min", "center", "max" } }
    out_path = Path(args.output)
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nCalibration saved to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
