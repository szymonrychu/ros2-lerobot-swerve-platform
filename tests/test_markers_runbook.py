"""Static invariants of the printable markers runbook (docs/runbooks/printable-markers.md) and docs/markers.md."""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "printable-markers.md"
MARKERS_DOC = REPO_ROOT / "docs" / "markers.md"

FORBIDDEN_RUNBOOK = ("head_camera_cpp", "/head_camera/", "1152", "head_camera_calibration.yaml", "zoom")
THRESHOLDS = ("100 ms", "640x360", "0.3 core", "20 sightings")


def section(text: str, heading: str) -> str:
    """Return the body of a '## heading' section.

    Args:
        text: Markdown document.
        heading: Heading text without the leading '## '.

    Returns:
        The section body up to the next '## ' heading.
    """
    match = re.search(rf"^## {re.escape(heading)}\n(.*?)(?=^## |\Z)", text, re.S | re.M)
    assert match, f"section {heading!r} missing"
    return match.group(1)


def numbered_steps(text: str) -> list[int]:
    """Return the numbers of the numbered steps of the Steps section.

    Args:
        text: Runbook markdown.

    Returns:
        Step numbers in document order.
    """
    return [int(n) for n in re.findall(r"^(\d+)\. ", section(text, "Steps"), re.M)]


def test_runbook_reads_the_d435() -> None:
    text = RUNBOOK.read_text()
    assert "/head/d435/color" in text
    for word in FORBIDDEN_RUNBOOK:
        assert word.lower() not in text.lower(), word


def test_prerequisites_are_the_d435_calibration() -> None:
    prereq = section(RUNBOOK.read_text(), "Prerequisites")
    for name in ("docs/calibration/", "docs/runbooks/d435-switchover.md", "./scripts/d435_diag.sh"):
        assert name in prereq, name


def test_pose_distances_are_in_d435_range() -> None:
    steps = section(RUNBOOK.read_text(), "Steps")
    pose = re.search(r"^5\. .*?(?=^6\. )", steps, re.S | re.M)
    assert pose
    body = pose.group(0)
    for token in ("0.4", "0.55", "0.7 m", "30 deg", "3 cm", "5 deg", "finding"):
        assert token in body, token
    assert "1.5 m" not in body
    assert "1.5 m" not in RUNBOOK.read_text()


def test_thresholds_kept() -> None:
    text = RUNBOOK.read_text()
    for token in THRESHOLDS:
        assert token in text, token


def test_results_rows_match_steps() -> None:
    text = RUNBOOK.read_text()
    steps = numbered_steps(text)
    rows = [
        line for line in section(text, "Results").splitlines() if line.startswith("|") and not line.startswith("|---")
    ][1:]
    assert len(steps) == len(rows)
    for number, row in zip(steps, rows, strict=True):
        first_cell = row.split("|")[1].strip()
        assert first_cell.startswith(f"{number} "), (number, first_cell)
