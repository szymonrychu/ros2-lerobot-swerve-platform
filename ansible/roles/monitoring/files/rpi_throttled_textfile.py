#!/usr/bin/env python3
"""Write the Raspberry Pi throttling flags (vcgencmd get_throttled) into the Alloy textfile collector.

Run every 15 s by rpi-throttled.timer. Produces rpi_throttled_flags (raw value) and rpi_throttled{bit=...} (0/1 per
named bit). The file is replaced atomically (temp file + rename) so the collector never reads half a file; when
vcgencmd fails the old file is removed instead, so no stale value is published, and the run exits non-zero.

Environment:
    RPI_THROTTLED_TEXTFILE: output file (default /var/lib/alloy/textfile/rpi_throttled.prom).
    VCGENCMD: vcgencmd binary (default "vcgencmd", looked up on PATH).
"""

import logging
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

DEFAULT_TEXTFILE = "/var/lib/alloy/textfile/rpi_throttled.prom"
DEFAULT_VCGENCMD = "vcgencmd"
VCGENCMD_TIMEOUT_S = 5.0
THROTTLED_RE = re.compile(r"throttled=(0x[0-9a-fA-F]+)")

# Bit number -> name (https://www.raspberrypi.com/documentation/computers/os.html#get_throttled).
THROTTLE_BITS: dict[int, str] = {
    0: "under_voltage_now",
    1: "freq_capped_now",
    2: "throttled_now",
    3: "soft_temp_limit_now",
    16: "under_voltage_occurred",
    17: "freq_capped_occurred",
    18: "throttled_occurred",
    19: "soft_temp_limit_occurred",
}

LOG = logging.getLogger("rpi_throttled_textfile")


def parse_throttled(output: str) -> int:
    """Parse the output of `vcgencmd get_throttled`.

    Args:
        output (str): Command output, e.g. "throttled=0x50005".

    Returns:
        int: The flags value.

    Raises:
        ValueError: The output has no throttled=0x... value.
    """
    match = THROTTLED_RE.search(output)
    if match is None:
        raise ValueError(f"unexpected vcgencmd output: {output.strip()!r}")
    return int(match.group(1), 16)


def throttled_metrics(flags: int) -> str:
    """Render the flags in the Prometheus text exposition format.

    Args:
        flags (int): get_throttled value.

    Returns:
        str: Metrics text, newline terminated.
    """
    lines = [
        "# HELP rpi_throttled_flags Raw value of vcgencmd get_throttled.",
        "# TYPE rpi_throttled_flags gauge",
        f"rpi_throttled_flags {flags}",
        "# HELP rpi_throttled One get_throttled bit: 1 when set (_now: currently, _occurred: since boot).",
        "# TYPE rpi_throttled gauge",
    ]
    lines += [f'rpi_throttled{{bit="{name}"}} {(flags >> bit) & 1}' for bit, name in THROTTLE_BITS.items()]
    return "\n".join(lines) + "\n"


def write_atomic(path: Path, text: str) -> None:
    """Replace a file atomically: write a temp file in the same directory, then rename it over the target.

    Args:
        path (Path): Target file.
        text (str): New content.
    """
    fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(text)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def read_throttled(vcgencmd: str) -> int:
    """Run vcgencmd get_throttled and parse it.

    Args:
        vcgencmd (str): vcgencmd binary.

    Returns:
        int: The flags value.

    Raises:
        OSError: The binary could not be run.
        subprocess.SubprocessError: It failed or timed out.
        ValueError: Its output could not be parsed.
    """
    proc = subprocess.run(
        [vcgencmd, "get_throttled"], capture_output=True, text=True, check=True, timeout=VCGENCMD_TIMEOUT_S
    )
    return parse_throttled(proc.stdout)


def main() -> int:
    """Read the flags once and update the textfile.

    Returns:
        int: Exit code (0 written, 1 vcgencmd failed and the old file was removed).
    """
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    path = Path(os.environ.get("RPI_THROTTLED_TEXTFILE", DEFAULT_TEXTFILE))
    vcgencmd = os.environ.get("VCGENCMD", DEFAULT_VCGENCMD)
    try:
        flags = read_throttled(vcgencmd)
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        LOG.error("vcgencmd get_throttled failed, removing %s: %s", path, exc)
        path.unlink(missing_ok=True)
        return 1
    write_atomic(path, throttled_metrics(flags))
    return 0


if __name__ == "__main__":
    sys.exit(main())
