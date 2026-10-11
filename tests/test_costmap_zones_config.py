"""Static invariants of the costmap_zones wiring (Ansible type, entry, playbook, metrics, docs)."""

import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
DEPLOY_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "costmap_zones"
DOCS = [
    REPO_ROOT / "ansible" / "README.md",
    REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md",
]
METRICS_TARGET = {"node": "costmap_zones", "address": "127.0.0.1:19111", "path": "/metrics"}

sys.path.insert(0, str(NODE_DIR))
from costmap_zones.config import CostmapZonesConfig  # noqa: E402


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_costmap_zones_node_type_defaults() -> None:
    node_type = client_vars()["ros2_node_type_defaults"]["costmap_zones"]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == "python3 -m costmap_zones"
    assert node_type["node_src_dir"] == "nodes/costmap_zones"
    assert node_type["src_shared"] is True
    assert node_type["memory_low"] == "96M" and node_type["memory_max"] == "256M"
    assert (node_type["cpu_weight"], node_type["io_weight"], node_type["oom_score_adjust"]) == (500, 500, -500)
    assert node_type["config_path"] == "/etc/ros2/costmap_zones"
    assert node_type["apt_packages"] == ["ros-jazzy-nav2-msgs"]
    assert "COSTMAP_ZONES_CONFIG=/etc/ros2/costmap_zones/config.yaml" in node_type["env"]


def test_costmap_zones_entry_after_poi_store_with_config() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    assert names.index("costmap_zones") == names.index("poi_store") + 1
    entry = nodes[names.index("costmap_zones")]
    assert entry["node_type"] == "costmap_zones" and entry["present"] and entry["enabled"]
    config = yaml.safe_load(entry["config"])
    assert set(config) == set(CostmapZonesConfig.model_fields)
    assert config["metrics_port"] == 19111
    assert CostmapZonesConfig(**config).model_dump() == config


def test_metrics_target_for_costmap_zones() -> None:
    assert METRICS_TARGET in client_vars()["monitoring_agent_node_targets"]


def test_playbook_deploys_costmap_zones_after_poi_store() -> None:
    plays = yaml.safe_load(DEPLOY_PLAYBOOK.read_text())
    tasks = [t for play in plays for block in play.get("tasks", []) for t in block.get("block", [block])]
    names = [t.get("name") for t in tasks]
    assert names.index("Deploy costmap_zones") > names.index("Deploy poi_store")
    task = tasks[names.index("Deploy costmap_zones")]
    assert task["vars"]["_deploy_node_name"] == "costmap_zones"
    assert task["tags"] == ["costmap_zones", "apt", "python", "build", "config"]


def test_node_files_exist() -> None:
    for name in ("pyproject.toml", "uv.lock", "README.md"):
        assert (NODE_DIR / name).is_file()


def test_docs_mention_costmap_zones_and_port() -> None:
    for doc in DOCS:
        assert re.search(r"costmap_zones", doc.read_text()), doc
    assert re.search(r"`costmap_zones` \| 19111", DOCS[0].read_text())
