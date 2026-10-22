"""Guards for docs/runbooks/ground-robustness.md: required sections, report-config thresholds, tracked commands."""

import re
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK_PATH = REPO_ROOT / "docs" / "runbooks" / "ground-robustness.md"
REPORT_CONFIG_PATH = REPO_ROOT / "scripts" / "ground_noise_report.yaml"
REQUIRED_SECTIONS = ("Safety", "Scene", "Prerequisites", "Procedure", "Acceptance", "Results")
SCRIPT_PATTERN = re.compile(r"\b(scripts/[\w./-]+\.(?:py|sh|yaml))")
PERCENT_TO_FRACTION = 100.0


def read_runbook() -> str:
    """Read the runbook.

    Returns:
        The runbook markdown text.
    """
    return RUNBOOK_PATH.read_text()


def section_text(name: str) -> str:
    """Return the body of one `## ` section.

    Args:
        name: Section heading without the `## ` prefix.

    Returns:
        The text between this heading and the next `## ` heading.
    """
    match = re.search(rf"^## {re.escape(name)}\n(.*?)(?=^## |\Z)", read_runbook(), re.S | re.M)
    assert match, f"missing section {name}"
    return match.group(1)


def test_runbook_sections() -> None:
    headings = re.findall(r"^## (.+)$", read_runbook(), re.M)
    assert tuple(h for h in headings if h in REQUIRED_SECTIONS) == REQUIRED_SECTIONS


def test_runbook_thresholds_match_report_config() -> None:
    thresholds = yaml.safe_load(REPORT_CONFIG_PATH.read_text())["thresholds"]
    acceptance = section_text("Acceptance")
    percents = [int(v) for v in re.findall(r"(\d+)%", acceptance)]
    assert round(thresholds["box_obstacle_share_min"] * PERCENT_TO_FRACTION) in percents
    assert round(thresholds["ramp_warning_share_min"] * PERCENT_TO_FRACTION) in percents
    assert round(thresholds["warning_churn_share_max"] * PERCENT_TO_FRACTION) in percents
    assert f"+-{thresholds['ramp_slope_abs_error_p50_max']:g} deg" in acceptance
    assert f"{thresholds['board_lethal_states_max']} lethal" in acceptance or "no lethal" in acceptance
    assert f"{thresholds['empty_lethal_points_p95_max']} lethal points at p95" in acceptance


def test_runbook_commands_exist() -> None:
    listing = subprocess.run(["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    tracked = set(listing.stdout.splitlines())
    named = set(SCRIPT_PATTERN.findall(read_runbook()))
    assert {"scripts/topic_scraper_collect.py", "scripts/ground_noise_report.py", "scripts/deploy-nodes.sh"} <= named
    assert named <= tracked, sorted(named - tracked)


def test_runbook_forbids_driving() -> None:
    safety = section_text("Safety").lower()
    for phrase in ("parked", "charging", "no drive commands", "no arm motion", "pan/tilt"):
        assert phrase in safety
