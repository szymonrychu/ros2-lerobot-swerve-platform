"""Static invariants of the client CPU budget runbook (docs/runbooks/client-cpu-budget.md) and its config."""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNBOOK = REPO_ROOT / "docs" / "runbooks" / "client-cpu-budget.md"
CPU_COMPARE = REPO_ROOT / "scripts" / "node_cpu_compare.yaml"

SECTIONS = ("Before", "Deploy", "After", "Checks", "Expected savings", "Rollback")
CHANGED_UNITS = (
    "bno055_imu",
    "swerve_controller",
    "laser_filter",
    "marker_detector",
    "filter_node",
    "lerobot_follower",
    "base_servos",
    "mcp_server",
    "costmap_zones",
    "nav2_bringup",
)
SAVINGS = ("-0.18", "-0.3", "-0.12", "-0.07", "-0.08", "-0.08", "-0.04", "-0.06")


def test_runbook_has_every_section() -> None:
    """The six sections exist, in order, and the savings table lists the eight items."""
    text = RUNBOOK.read_text()
    headings = re.findall(r"^## (.+)$", text, flags=re.MULTILINE)
    positions = [headings.index(s) for s in SECTIONS]
    assert positions == sorted(positions)
    for saving in SAVINGS:
        assert saving in text
    assert "1.0-1.2" in text
    assert "date +%s" in text
    assert "vmstat 5" in text
    assert "node_load1" in text
    assert "transform_tolerance" in text


def test_runbook_deploys_only_through_deploy_nodes() -> None:
    """Every deploy line names scripts/deploy-nodes.sh with a tee to .logs/; no direct ansible-playbook."""
    text = RUNBOOK.read_text()
    assert "ansible-playbook" not in text.replace("never run `ansible-playbook`", "")
    deploys = [line for line in text.splitlines() if "deploy-nodes.sh" in line and line.strip().startswith(("./", "$"))]
    assert deploys
    assert "./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-cpu-budget.log" in text
    assert "./scripts/deploy-nodes.sh client monitoring 2>&1 | tee .logs/deploy-client-monitoring.log" in text
    assert "./scripts/deploy-nodes.sh server monitoring" in text
    for line in deploys:
        assert "tee .logs/" in line


def test_cpu_compare_lists_every_changed_unit() -> None:
    """node_cpu_compare.yaml has an entry per changed unit and every switch_time is the placeholder 1 or a real time."""
    config = yaml.safe_load(CPU_COMPARE.read_text())
    units = {e["after_unit"] for e in config["entries"]}
    for name in CHANGED_UNITS:
        assert f"ros2-{name}.service" in units, name
    # head_camera_cpp is retired by the D435 switchover; the head D435 units are measured after only (no before series).
    assert "ros2-head_camera_cpp.service" not in units
    for unit in ("ros2-realsense_d435.service", "ros2-d435_obstacle_cloud.service"):
        assert unit in CPU_COMPARE.read_text() and unit not in units, unit
    assert "client-cpu-budget.md" in CPU_COMPARE.read_text()
    assert config["window_h"] * 3600 <= 86400
    for existing in ("map_guard", "web_ui"):
        assert any(e["name"] == existing for e in config["entries"])
