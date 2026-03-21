#!/usr/bin/env python3
"""One-time survey-in calibration for LC29H(BS) base station.

Sends NMEA commands to the module over serial: factory reset, start survey-in,
poll until complete, then save ECEF position to flash. Run on the server (RPi
with BS hat). After calibration, power cycle the module.

Usage:
  python scripts/calibrate_rtk_base.py --port /dev/ttyAMA0 --samples 3600 --accuracy 15

See RTK_NOTES.md for calibration flow and PQTMCFGSVIN command details.
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def nmea_checksum(sentence: str) -> str:
    """XOR checksum for NMEA (sentence after $ and before *)."""
    checksum = 0
    for c in sentence:
        checksum ^= ord(c)
    return f"{checksum:02X}"


def _read_one_nmea_line(ser: serial.Serial, deadline: float) -> str | None:
    """Read one complete NMEA line (starts with $). Returns None on timeout."""
    buf = bytearray()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            b = ser.read(1)
            if not b:
                continue
            buf.append(b[0])
            if b[0] in (ord("\r"), ord("\n")) and buf:
                try:
                    line = buf.decode("ascii", errors="strict").strip()
                except UnicodeDecodeError:
                    buf.clear()
                    continue
                if line.startswith("$"):
                    return line
                buf.clear()
        else:
            time.sleep(0.05)
    return None


def send_cmd(ser: serial.Serial, cmd: str, timeout_s: float = 5.0) -> str | None:
    """Send NMEA command (with $ and *XX if missing), read one $ line response."""
    cmd = cmd.strip()
    if not cmd.startswith("$"):
        cmd = "$" + cmd
    if "*" not in cmd:
        cmd = cmd + "*" + nmea_checksum(cmd[1:])
    ser.write((cmd + "\r\n").encode("ascii"))
    deadline = time.monotonic() + timeout_s
    return _read_one_nmea_line(ser, deadline)


def send_cmd_wait_for(
    ser: serial.Serial,
    cmd: str,
    expect_substring: str,
    timeout_s: float = 10.0,
) -> str | None:
    """Send command and read lines until one contains expect_substring (e.g. 'PQTMCFGSVIN')."""
    cmd = cmd.strip()
    if not cmd.startswith("$"):
        cmd = "$" + cmd
    if "*" not in cmd:
        cmd = cmd + "*" + nmea_checksum(cmd[1:])
    ser.write((cmd + "\r\n").encode("ascii"))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = _read_one_nmea_line(ser, time.monotonic() + 2.0)
        if line and expect_substring in line:
            return line
    return None


def read_lines(ser: serial.Serial, duration_s: float) -> list[str]:
    """Read NMEA lines (lines starting with $) for up to duration_s."""
    deadline = time.monotonic() + duration_s
    lines: list[str] = []
    buf = bytearray()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
        while True:
            idx = buf.find(ord("$"))
            if idx < 0:
                buf.clear()
                break
            buf = buf[idx:]
            end = buf.find(ord("\n"))
            if end < 0:
                break
            line = buf[: end + 1].decode("ascii", errors="ignore").strip()
            buf = buf[end + 1 :]  # noqa: E203
            if line.startswith("$"):
                lines.append(line)
        time.sleep(0.1)
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate LC29H(BS) base station survey-in (one-time).")
    parser.add_argument(
        "--port",
        default="/dev/ttyAMA0",
        help="Serial port (default: /dev/ttyAMA0 on RPi 4 with disable-bt)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=3600,
        help="Survey-in sample count (default: 3600)",
    )
    parser.add_argument(
        "--accuracy",
        type=float,
        default=15.0,
        help="Accuracy limit in meters (default: 15)",
    )
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="Skip factory restore (PQTMRESTOREPAR)",
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.5,
            write_timeout=2.0,
        )
    except serial.SerialException as e:
        print(f"Failed to open {args.port}: {e}", file=sys.stderr)
        return 1

    try:
        if not args.no_restore:
            print("Sending factory restore (PQTMRESTOREPAR)...")
            r = send_cmd_wait_for(ser, "$PQTMRESTOREPAR", "PQTMRESTOREPAR", timeout_s=8.0)
            if r:
                print(f"  -> {r}")
            else:
                print("  (no PQTMRESTOREPAR response in time)")
            time.sleep(1.0)

        print(f"Starting survey-in: samples={args.samples}, accuracy={args.accuracy}m")
        cmd = f"$PQTMCFGSVIN,W,1,{args.samples},{int(args.accuracy)},0,0,0"
        r = send_cmd_wait_for(ser, cmd, "PQTMCFGSVIN", timeout_s=10.0)
        if r:
            print(f"  -> {r}")
        if r and "PQTMCFGSVIN" in r and "OK" in r:
            print("Survey-in started. This may take 1–2 hours.")
            print("Polling for completion (check every 60s)...")
        elif r:
            print("Unexpected response above; survey may already be running. Polling anyway.")
        else:
            print("No PQTMCFGSVIN response in time; polling for status anyway.")

        last_status = ""
        while True:
            time.sleep(60.0)
            r = send_cmd_wait_for(ser, "$PQTMCFGSVIN,R", "PQTMCFGSVIN", timeout_s=15.0)
            if not r:
                print("  (no PQTMCFGSVIN response)")
                continue
            print(f"  -> {r}")
            # Response: OK,1,<samples>,1,X,Y,Z (done) or OK,1,<samples>,<accuracy>,X,Y,Z (e.g. 15.0,0,0,0 = in progress)
            # Accept completion only when we have non-zero ECEF (doc format: OK,1,3600,1,X,Y,Z)
            m = re.search(
                r"PQTMCFGSVIN,OK,1,\d+,[\d.]+,([-\d.]+),([-\d.]+),([-\d.]+)",
                r,
            )
            if m:
                x, y, z = m.group(1), m.group(2), m.group(3)
                try:
                    xf, yf, zf = float(x), float(y), float(z)
                    if (xf != 0.0 or yf != 0.0 or zf != 0.0) and abs(xf) > 1e-6:
                        print(f"Survey complete. ECEF X={x}, Y={y}, Z={z}")
                        print("Saving to module...")
                        save_cmd = f"$PQTMCFGSVIN,W,2,0,0,{x},{y},{z}"
                        send_cmd_wait_for(ser, save_cmd, "PQTMCFGSVIN", timeout_s=5.0)
                        time.sleep(0.5)
                        send_cmd_wait_for(ser, "$PQTMSAVEPAR", "PQTM", timeout_s=5.0)
                        print("Done. Power cycle the module (unplug/plug or reboot).")
                        return 0
                except ValueError:
                    pass
            if r != last_status:
                last_status = r

    except KeyboardInterrupt:
        print("\nInterrupted. Survey may still be running on the module.")
        return 130
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
