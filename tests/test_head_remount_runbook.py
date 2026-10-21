"""Static invariants of the head remount floor-check runbook (docs/runbooks/head-remount-floor-check.md).

The runbook is executed on the robot; these tests pin its sections, the deploy command form, the floor thresholds and
the numbers it quotes from nav2_params.yaml and ground_estimator.yaml.
"""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "head-remount-floor-check.md"
GROUND_RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "ground-estimation.md"
FUSION_RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "head-rgbd-fusion.md"
METHODOLOGY = REPO_ROOT / "docs" / "head-rgbd-fusion-methodology.md"
MEMORY = REPO_ROOT / "MEMORY.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
GROUND_CONFIG = REPO_ROOT / "nodes" / "ground_estimator_cpp" / "config" / "ground_estimator.yaml"

SECTIONS = (
    "Preconditions",
    "Deploy",
    "Head pan direction",
    "Head re-zero",
    "Head RGB-D recalibration",
    "Floor numbers",
    "Costmap",
    "Results",
)
DEPLOY_NODES = (
    "arducam_tof",
    "head_rgbd_calibration",
    "base_servos",
    "ground_estimator_cpp",
    "nav2_bringup",
    "mcp_server",
)


def runbook_text() -> str:
    """Return the runbook text. Output: str."""
    return RUNBOOK.read_text()


def section(name: str) -> str:
    """Return the body of the `## name` section. Input: section title. Output: str."""
    parts = re.split(r"^## ", runbook_text(), flags=re.MULTILINE)
    for part in parts:
        if part.startswith(name + "\n"):
            return part
    raise AssertionError(f"missing section: {name}")


def test_runbook_has_every_section_in_order() -> None:
    headings = re.findall(r"^## (.+)$", runbook_text(), flags=re.MULTILINE)
    assert tuple(headings) == SECTIONS


def test_deploy_uses_the_script_never_ansible_playbook() -> None:
    text = runbook_text()
    assert "ansible-playbook" not in text.replace("never `ansible-playbook`", "")
    deploy = section("Deploy")
    assert "scripts/deploy-nodes.sh" in deploy
    assert ".logs/" in deploy and "tee" in deploy
    assert "ansible-deploy" in deploy
    for node in DEPLOY_NODES:
        assert node in deploy


def test_preconditions_name_the_setup_and_the_agent_session_check() -> None:
    pre = section("Preconditions")
    for phrase in ("not on blocks", "flat floor", "ChArUco", "agent session", "mcp_server"):
        assert phrase in pre


def test_pan_direction_check() -> None:
    pan = section("Head pan direction")
    assert "+0.3 rad" in pan and "left" in pan and "CCW" in pan and "confirm" in pan


def test_rezero_and_recalibration_reference_their_runbooks() -> None:
    assert "docs/runbooks/servo-torque-neutral.md" in section("Head re-zero")
    recal = section("Head RGB-D recalibration")
    assert "docs/runbooks/head-rgbd-fusion.md" in recal
    assert "plane_bias_mm_median" in recal and "10 mm" in recal and "plane_max_bias_mm" in recal and "40" in recal


def test_floor_thresholds_and_scraper_workflow() -> None:
    floor = section("Floor numbers")
    assert "topic_scraper_collect.py" in floor
    assert "+-1 cm" in floor
    assert "0.6 deg" in floor
    assert "ground_temporal_cells" in floor
    assert "60 s" in floor


def test_costmap_band_equals_nav2_tof_mark() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    layer = params["local_costmap"]["local_costmap"]["ros__parameters"]["tof_voxel_layer"]
    band = layer["tof_mark"]["min_obstacle_height"]
    assert band == 0.10
    costmap = section("Costmap")
    assert f"{band:.2f}" in costmap and "10 cm" in costmap
    assert "30 s" in costmap


def test_temporal_defaults_match_ground_estimator_config() -> None:
    temporal = yaml.safe_load(GROUND_CONFIG.read_text())["temporal"]
    floor = section("Floor numbers")
    assert f"window_frames {temporal['window_frames']}" in floor
    assert f"min_frames {temporal['min_frames']}" in floor
    assert f"cell_size_m {temporal['cell_size_m']:.2f}" in floor


def test_every_scripts_path_exists() -> None:
    paths = set(re.findall(r"scripts/[A-Za-z0-9_./-]+\.(?:sh|py)", runbook_text()))
    assert "scripts/deploy-nodes.sh" in paths and "scripts/topic_scraper_collect.py" in paths
    for path in paths:
        assert (REPO_ROOT / path).is_file(), path


def test_every_referenced_doc_exists() -> None:
    for path in set(re.findall(r"docs/[A-Za-z0-9_./-]+\.md", runbook_text())):
        assert (REPO_ROOT / path).is_file(), path


def test_results_section_has_a_check_table() -> None:
    results = section("Results")
    assert "| Check |" in results
    for check in ("Pan direction", "Plane height", "plane_vs_base", "Drop and obstacle points", "Costmap"):
        assert check in results


def test_other_docs_link_and_state_z_depth() -> None:
    assert "head-remount-floor-check.md" in GROUND_RUNBOOK.read_text()
    for doc in (FUSION_RUNBOOK, METHODOLOGY):
        text = doc.read_text()
        assert "z-depth" in text
        assert "radial" in text and "ray norm" in text
        assert "plane bias" in text.lower() or "plane_bias" in text


def test_memory_records_the_decision() -> None:
    text = MEMORY.read_text()
    assert "## Head remount: ToF z-depth and ground noise (2026-10-21)" in text
    entry = text.split("## Head remount: ToF z-depth and ground noise (2026-10-21)")[1]
    for phrase in (
        "radial",
        "z in arducam_tof",
        "calibrated K",
        "head_pan",
        "reversed",
        "temporal median",
        "10 cm",
        "rule unchanged",
    ):
        assert phrase in entry
    assert "docs/runbooks/head-remount-floor-check.md" in entry


def test_tests_readme_lists_this_file() -> None:
    text = TESTS_README.read_text()
    assert "test_head_remount_runbook.py" in text
