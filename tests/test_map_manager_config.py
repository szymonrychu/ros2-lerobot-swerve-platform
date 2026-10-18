"""Static invariants of the map_manager wiring (Ansible node type, ros2_nodes entry, playbook, docs)."""

import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO_ROOT / "nodes" / "map_manager"), str(REPO_ROOT / "shared")]

from map_manager.config import MapManagerConfig  # noqa: E402

CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
DEPLOY_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
MAPS_DIR_TASK = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "slam_maps_dir.yml"
DOCS = [
    REPO_ROOT / "nodes" / "README.md",
    REPO_ROOT / "ansible" / "README.md",
    REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md",
]
MAPS_PARENT = "/var/lib/ros2/maps"


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def entry() -> dict:
    """Return the map_manager ros2_nodes entry.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == "map_manager")


def test_map_manager_node_type_defaults() -> None:
    node_type = client_vars()["ros2_node_type_defaults"]["map_manager"]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == "python3 -m map_manager"
    assert node_type["node_src_dir"] == "nodes/map_manager"
    assert node_type["src_shared"] is True
    assert node_type["config_path"] == "/etc/ros2/map_manager"
    assert "MAP_MANAGER_CONFIG=/etc/ros2/map_manager/config.yaml" in node_type["env"]
    assert (node_type["cpu_weight"], node_type["io_weight"], node_type["oom_score_adjust"]) == (500, 500, -500)
    assert node_type["memory_low"] == "64M" and node_type["memory_max"] == "256M"
    assert "ros-jazzy-slam-toolbox" in node_type["apt_packages"]


def test_map_manager_entry_present_and_enabled_after_poi_store() -> None:
    names = [n["name"] for n in client_vars()["ros2_nodes"]]
    assert names.index("map_manager") > names.index("poi_store")
    node = entry()
    assert node["node_type"] == "map_manager" and node["present"] is True and node["enabled"] is True


def test_map_manager_config_validates_and_documents_every_key() -> None:
    config = yaml.safe_load(entry()["config"])
    parsed = MapManagerConfig(**config)
    assert parsed.metrics_port == 19113
    assert parsed.maps_root == config["maps_root"]
    assert set(config) == set(MapManagerConfig.model_fields)
    assert config["maps_root"] == "/var/lib/ros2/maps/named"
    assert config["maps_root"].startswith(MAPS_PARENT + "/")


def test_map_manager_config_matches_defaults() -> None:
    config = yaml.safe_load(entry()["config"])
    defaults = MapManagerConfig().model_dump()
    assert MapManagerConfig(**config).model_dump() == defaults


def test_map_manager_config_has_group_comments() -> None:
    text = entry()["config"]
    assert sum(1 for line in text.splitlines() if line.lstrip().startswith("#")) >= 6


def test_maps_dir_task_creates_parent_owned_by_node_user() -> None:
    task = yaml.safe_load(MAPS_DIR_TASK.read_text())[0]["ansible.builtin.file"]
    assert task["path"] == MAPS_PARENT and task["owner"] == "{{ ansible_user }}"


def test_playbook_deploys_map_manager_after_poi_store_with_maps_dir() -> None:
    full = DEPLOY_PLAYBOOK.read_text()
    assert "_deploy_node_name: map_manager" in full
    assert full.index("_deploy_node_name: poi_store") < full.index("_deploy_node_name: map_manager")
    tail = full[full.index("_deploy_node_name: poi_store") :]
    block = tail[: tail.index("_deploy_node_name: map_manager")]
    assert "tasks/slam_maps_dir.yml" in block
    plays = yaml.safe_load(full)
    tasks = [t for play in plays for t in play.get("tasks", [])]
    flat = [s for t in tasks for s in t.get("block", [t])]
    deploy = next(t for t in flat if t.get("vars", {}).get("_deploy_node_name") == "map_manager")
    assert "map_manager" in deploy["tags"]
    maps = [t for t in flat if "slam_maps_dir.yml" in str(t.get("ansible.builtin.include_tasks", ""))]
    assert any("map_manager" in t["tags"] for t in maps)


def test_docs_mention_map_manager() -> None:
    for doc in DOCS:
        assert "map_manager" in doc.read_text(), doc
