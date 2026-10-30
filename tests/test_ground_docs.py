"""Docs of the ground estimation epic: runbook, MEMORY decision, topic flow diagram, tests README.

The runbook is executed on the robot after merge. These tests keep it consistent with the shipped config and the repo:
required sections, design thresholds, the Threshold validation defaults equal to ground_estimator.yaml, existing script
paths, /ground topics that the node publishes, and the other docs naming the node.
"""

import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parent.parent
RUNBOOK = REPO / "docs" / "runbooks" / "ground-estimation.md"
CONFIG = REPO / "nodes" / "ground_estimator_cpp" / "config" / "ground_estimator.yaml"
MEMORY = REPO / "MEMORY.md"
TOPIC_FLOW = REPO / "docs" / "diagrams" / "topic_flow.puml"
TESTS_README = REPO / "tests" / "README.md"

REQUIRED_SECTIONS = [
    "Preconditions",
    "Deploy",
    "Topic checks",
    "Level floor",
    "Scenes",
    "CPU and memory",
    "Build and CI validation",
    "Threshold validation",
    "Switch-over checklist",
    "Results",
]
GROUND_TOPICS = ["obstacle_points", "drop_points", "labelled_points", "state", "classified_points"]
DESIGN_THRESHOLDS = ["0.10 core", "1 cm", "1 deg"]
DEPLOY_COMMAND = (
    "./scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones nav2_bringup web_ui "
    "2>&1 | tee .logs/deploy-client-ground.log"
)
MEMORY_TERMS = [
    "ground_estimator_cpp",
    "IMU prior",
    "second STVL layer",
    "one third",
    "8 deg",
    "costmap_zones",
    "floor_guard",
    "0.10 core",
]
THRESHOLD_KEYS = [
    "fit.distance_m",
    "fit.max_normal_angle_deg",
    "fit.max_offset_m",
    "traversability.wheel_radius_m",
    "traversability.max_slope_deg",
    "edges.step_max_height_m",
    "colour.max_mahalanobis",
]


def runbook_text() -> str:
    return RUNBOOK.read_text()


def section(name: str) -> str:
    """Body of the `## name` section of the runbook.

    Args:
        name: Heading text without the leading hashes.
    Returns:
        The text up to the next level-2 heading.
    """
    match = re.search(rf"^## {re.escape(name)}\n(.*?)(?=^## |\Z)", runbook_text(), re.S | re.M)
    assert match, f"section {name} missing"
    return match.group(1)


def flat_config() -> dict[str, float]:
    """Shipped ground_estimator.yaml with nested keys dotted."""
    out: dict[str, float] = {}

    def walk(prefix: str, node: dict) -> None:
        for key, value in node.items():
            if isinstance(value, dict):
                walk(f"{prefix}{key}.", value)
            else:
                out[f"{prefix}{key}"] = value

    walk("", yaml.safe_load(CONFIG.read_text()))
    return out


@pytest.mark.parametrize("name", REQUIRED_SECTIONS)
def test_runbook_has_section(name: str) -> None:
    assert section(name).strip()


def test_runbook_thresholds_equal_the_design() -> None:
    text = runbook_text()
    for value in DESIGN_THRESHOLDS:
        assert value in text, value
    assert "<= 0.10 core" in section("CPU and memory")
    assert "within 1 cm" in section("Level floor")
    assert "below 1 deg" in section("Level floor")


def test_threshold_validation_defaults_equal_the_shipped_config() -> None:
    config = flat_config()
    rows = re.findall(r"^\| `([a-z_.]+)` \| ([0-9.]+) \|", section("Threshold validation"), re.M)
    found = {key: float(value) for key, value in rows}
    for key in THRESHOLD_KEYS:
        assert key in found, f"{key} not in the Threshold validation table"
    for key, value in found.items():
        if key in config:
            assert config[key] == pytest.approx(value), key
    # One third of the wheel diameter = 0.04 m, up and down (user decision 2026-10-29; 0.5 was the temporary value),
    # separate keys since task T3.
    for key in ("max_step_up_fraction", "max_step_down_fraction"):
        assert config[f"traversability.{key}"] == pytest.approx(0.3333, abs=1e-4)
        assert key in section("Threshold validation")
    assert "`ground_warning_speed_pct` | 40 |" in section("Threshold validation")
    assert "`ground_warning_mask_value` | 40 |" in section("Threshold validation")


def test_runbook_scripts_exist() -> None:
    tracked = set(
        subprocess.run(["git", "ls-files"], cwd=REPO, capture_output=True, text=True, check=True).stdout.split("\n")
    )
    paths = set(re.findall(r"scripts/[A-Za-z0-9_./-]+\.(?:py|sh|yaml)", runbook_text()))
    assert paths
    for path in paths:
        assert path in tracked, path


def test_runbook_ground_topics_are_node_outputs() -> None:
    config = yaml.safe_load(CONFIG.read_text())
    namespace = config["output_namespace"]
    readme = (REPO / "nodes" / "ground_estimator_cpp" / "README.md").read_text()
    named = set(re.findall(r"/ground/[a-z_]+", runbook_text()))
    assert {f"{namespace}/{t}" for t in GROUND_TOPICS} <= named
    for topic in named:
        assert topic.startswith(namespace + "/")
        assert f"| out | `{topic}` |" in readme, topic


def test_runbook_pins_the_rates_and_the_deploy_command() -> None:
    topics = section("Topic checks")
    for text in ["~10 Hz", "~1 Hz", "2 Hz", "TF drop", "head joint_states"]:
        assert text in topics, text
    deploy = section("Deploy")
    assert DEPLOY_COMMAND in deploy
    assert "ansible-deploy" in deploy
    assert "mcp_server" in deploy


def test_runbook_scenes_cover_both_sides_of_the_rule() -> None:
    scenes = section("Scenes")
    for text in [
        "3 cm sill",
        "5 cm sill",
        "3 cm drop",
        "5 cm drop",
        "10 cm drop",
        "box",
        "13 deg ramp",
        "red sheet",
        "0.04 m",
        "15 deg",
        "CliffStop",
        "ground_voxel_layer.enabled",
        "depth_voxel_layer.enabled",
        "/speed_limit",
    ]:
        assert text in scenes, text


def test_memory_has_the_decision() -> None:
    text = MEMORY.read_text()
    match = re.search(r"^## Ground plane[^\n]*\n(.*?)(?=^## |\Z)", text, re.S | re.M)
    assert match, "MEMORY.md has no Ground plane entry"
    for term in MEMORY_TERMS:
        assert term in match.group(1), term


def test_topic_flow_names_the_node_and_the_ground_topics() -> None:
    text = TOPIC_FLOW.read_text()
    assert "ground_estimator_cpp" in text
    for topic in GROUND_TOPICS:
        assert f"/ground/{topic}" in text, topic
    for topic in ["/tof/points_filtered", "/imu/data", "/head_rgbd/points"]:
        assert topic in text, topic


def test_tests_readme_lists_every_ground_test_file() -> None:
    readme = TESTS_README.read_text()
    files = sorted(p.name for p in (REPO / "tests").glob("test_*ground*.py"))
    assert "test_ground_docs.py" in files
    for name in files:
        assert f"`{name}`" in readme or f"### {name}" in readme, name
    for name in [
        "test_traversability.cpp",
        "test_plane_fit.cpp",
        "test_node.cpp",
        "test_ground_warnings.py",
        "test_floor_guard_ground.py",
        "test_ground_summary.py",
        "test_ground_layer.py",
        "groundLayer.test.ts",
    ]:
        assert name in readme, name
