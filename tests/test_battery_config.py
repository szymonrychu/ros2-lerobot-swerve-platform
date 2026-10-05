"""Wiring of the battery voltage chain in Ansible group_vars: follower publisher, web_ui guard, leader off."""

import ast
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

BATTERY_TOPIC = "/battery_state"
CELLS = 3
CUTOFF_CELL_V = 2.8
RESUME_CELL_V = 2.9
EXPECTED_CUTOFF_V = 8.4


def node_config(path: Path, name: str) -> dict:
    """Parse the `config: |` block of one ros2_nodes entry.

    Args:
        path: group_vars YAML file.
        name: Node name.

    Returns:
        dict: Parsed config YAML of the node.
    """
    nodes = yaml.safe_load(path.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def test_follower_publishes_battery_state() -> None:
    """lerobot_follower reads the pack voltage once per second on the standard topic."""
    cfg = node_config(CLIENT_VARS, "lerobot_follower")
    assert cfg["battery_topic"] == BATTERY_TOPIC
    assert cfg["battery_interval_s"] == 1.0
    assert cfg["battery_cells"] == CELLS


def test_web_ui_battery_matches_follower() -> None:
    """web_ui subscribes to the follower's battery topic with the same cell count."""
    follower = node_config(CLIENT_VARS, "lerobot_follower")
    battery = node_config(CLIENT_VARS, "web_ui")["battery"]
    assert battery["topic"] == follower["battery_topic"]
    assert battery["cells"] == follower["battery_cells"]


def test_web_ui_cutoff_is_8v4_for_three_cells() -> None:
    """Cut-off 2.8 V/cell x 3 cells = 8.4 V, resume above 2.9 V/cell, readings stale after 5 s."""
    battery = node_config(CLIENT_VARS, "web_ui")["battery"]
    assert battery["cutoff_cell_v"] == CUTOFF_CELL_V
    assert battery["resume_cell_v"] == RESUME_CELL_V
    assert battery["stale_s"] == 5.0
    assert battery["cells"] * battery["cutoff_cell_v"] == pytest.approx(EXPECTED_CUTOFF_V)
    assert battery["resume_cell_v"] >= battery["cutoff_cell_v"]


def test_mcp_server_battery_matches_web_ui() -> None:
    """mcp_server gates motion on the same topic and thresholds as web_ui rejects commands."""
    web_ui = node_config(CLIENT_VARS, "web_ui")["battery"]
    mcp = node_config(CLIENT_VARS, "mcp_server")["battery"]
    assert mcp == web_ui
    assert mcp["topic"] == BATTERY_TOPIC
    assert mcp["cells"] * mcp["cutoff_cell_v"] == pytest.approx(EXPECTED_CUTOFF_V)


def test_leader_does_not_publish_battery() -> None:
    """The leader arm bus is not the robot pack: its bridge has battery reading disabled."""
    assert node_config(SERVER_VARS, "lerobot_leader")["battery_interval_s"] == 0


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### test_battery_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
