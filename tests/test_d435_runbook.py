"""Static pins of the D435 switchover runbook (docs/runbooks/d435-switchover.md).

The "Bring-up" section records the live bring-up of the head D435 on the client: every command with the summary line
of its real output. These tests pin that the commands are there and that the evidence is filled in (firmware, USB
speed, the 30 min soak with zero errors, camera_info rates), with no placeholder left.
"""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "d435-switchover.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
MIN_FIRMWARE = (5, 17, 3, 10)
MIN_RATE_HZ = 14.0
# Regexes of the bring-up commands (the flash ones as run on the client: backup, signed flash, recovery).
COMMANDS = (
    r"\./scripts/deploy-nodes\.sh client realsense_d435",
    r"\./scripts/d435_diag\.sh --device",
    r"rs-fw-update -l",
    r"rs-fw-update -s 935723024187 -b \S+",
    r"rs-fw-update -s 935723024187 -f \S+Signed_Image_UVC_5_17_3_10\.bin",
    r"rs-fw-update -r -f \S+",
    r"\./scripts/d435_diag\.sh --since",
    r"\./scripts/d435_diag\.sh --topics",
    r"\./scripts/d435_diag\.sh --rates",
    r"\./scripts/deploy-nodes\.sh client topic_scraper_api",
)
ERROR_PATTERNS = ("VIDIOC_QBUF", "No such device", "Frame didn't arrive")
PLACEHOLDER = re.compile(r"TODO|TBD|FIXME|XXX|\?\?\?|<fill|\bpending\b|to be (?:filled|measured)", re.IGNORECASE)
FIRMWARE = re.compile(r"\b(\d+)\.(\d+)\.(\d+)\.(\d+)\b")
RATE_LINE = re.compile(r"/head/d435/(color|depth|aligned_depth_to_color)/camera_info\s+(\d+(?:\.\d+)?) Hz")


def section(name: str) -> str:
    """Return the body of the `## name` section of the runbook.

    Args:
        name (str): Section title.

    Returns:
        str: Section text after the heading line.

    Raises:
        AssertionError: The section is missing.
    """
    for part in re.split(r"^## ", RUNBOOK.read_text(), flags=re.MULTILINE):
        if part.startswith(name + "\n"):
            return part
    raise AssertionError(f"missing section: {name}")


def test_bring_up_section_is_recorded() -> None:
    """Bring-up names every command and records firmware >= 5.17.3.10, 5000M, the 30 min soak and >= 14 Hz rates."""
    body = section("Bring-up")
    for command in COMMANDS:
        assert re.search(command, body), command
    assert not PLACEHOLDER.search(body), PLACEHOLDER.search(body)
    versions = [tuple(int(part) for part in match) for match in FIRMWARE.findall(body)]
    assert any(version >= MIN_FIRMWARE for version in versions), versions
    assert "8086:0b07" in body
    assert re.search(r"speed: 5000 Mb/s", body)
    assert "Device USB type: 3." in body
    assert "usb_max_current_enable=1" in body
    assert "30 min" in body
    for pattern in ERROR_PATTERNS:
        assert re.search(rf"{re.escape(pattern)}\s+0\b", body), pattern
    rates = {stream: float(rate) for stream, rate in RATE_LINE.findall(body)}
    for stream in ("color", "depth"):
        assert rates.get(stream, 0.0) >= MIN_RATE_HZ, (stream, rates)


def test_tests_readme_documents_the_bring_up_tests() -> None:
    """tests/README.md has a section for both bring-up test files."""
    readme = TESTS_README.read_text()
    for name in ("test_d435_topics.py", "test_d435_runbook.py"):
        assert re.search(rf"^### `?{re.escape(name)}`?$", readme, flags=re.MULTILINE), name
