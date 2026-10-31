"""Static invariants of the 3D mapping runbook (docs/runbooks/map3d-mapping.md) re-based on the D435."""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "map3d-mapping.md"

RETIRED_INPUTS = (
    "head_camera_calibration.yaml",
    "tof_calibration.yaml",
    "head_rgbd_extrinsics.yaml",
    "arducam_tof",
    "head_camera_cpp",
    "camera_info_file",
    "focus lock",
)
STEP_RE = re.compile(r"^(\d+)\. ", re.MULTILINE)
ROW_RE = re.compile(r"^\| (\d+) ", re.MULTILINE)


def read_runbook() -> str:
    """Return the runbook text.

    Returns:
        The full markdown of docs/runbooks/map3d-mapping.md.
    """
    return RUNBOOK.read_text(encoding="utf-8")


def step_text(number: int) -> str:
    """Return the text of one numbered step.

    Args:
        number: Step number (1-based) in the Steps section.

    Returns:
        The step's lines up to the next numbered step or the next heading.
    """
    text = read_runbook()
    steps = text.split("## Steps", 1)[1].split("## Results", 1)[0]
    match = re.search(rf"^{number}\. .*?(?=^\d+\. |\Z)", steps, re.MULTILINE | re.DOTALL)
    assert match, f"step {number} missing"
    return match.group(0)


def test_runbook_has_no_cm3_or_tof_inputs() -> None:
    """None of the CM3 camera or ToF calibration inputs remain."""
    text = read_runbook()
    for needle in RETIRED_INPUTS:
        assert needle not in text, needle


def test_head_chain_check_is_the_d435() -> None:
    """Step 1 checks the D435 calibration results, the diag script and the /head_rgbd pair rate."""
    step = step_text(1)
    assert "docs/calibration/" in step
    assert "-d435" in step
    assert "./scripts/d435_diag.sh" in step
    assert "/head_rgbd/depth/image_raw" in step
    assert "mapping rate" in step


def test_deploy_and_placeholder_name_the_d435() -> None:
    """Deploy, CPU and placeholder steps use the D435 node names."""
    assert "./scripts/deploy-nodes.sh client head_rgbd_cpp map3d_uplink web_ui" in step_text(2)
    cpu = step_text(5)
    for name in ("realsense_d435", "head_rgbd_cpp", "map3d_uplink"):
        assert name in cpu, name
    placeholder = step_text(7)
    assert "realsense_d435" in placeholder
    assert "waiting_for_inputs" in placeholder


def test_thresholds_kept() -> None:
    """The moved requirement's thresholds are unchanged."""
    text = read_runbook()
    for needle in ("0.8", "0.15 m/s", "0.3 rad/s", "2x", "budget_bytes"):
        assert needle in text, needle


def test_runs_with_d435_s72() -> None:
    """The header says when it runs: after D435 S59 and S84, in the session of S72."""
    header = read_runbook().split("## Steps", 1)[0]
    for needle in ("S59", "S84", "S72", "docs/runbooks/d435-switchover.md"):
        assert needle in header, needle


def test_results_rows_match_steps() -> None:
    """The Results table has one row per numbered step, each starting with its number."""
    text = read_runbook()
    steps = text.split("## Steps", 1)[1].split("## Results", 1)[0]
    results = text.split("## Results", 1)[1]
    step_numbers = STEP_RE.findall(steps)
    row_numbers = ROW_RE.findall(results)
    assert step_numbers == row_numbers
    assert len(step_numbers) == 7
