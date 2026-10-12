"""Static invariants of the leader_bridge deployment (Ansible group_vars, playbook, lint and CI coverage, docs).

leader_bridge runs on the client, relays /leader/joint_states from the server's lerobot_leader to filter_node's leader
input, and replaces the disabled master2master leader rule. The server needs no entry: it already publishes the topic.
"""

import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes" / "leader_bridge"))

from leader_bridge.config import DEFAULT_DEST_TOPIC, DEFAULT_SOURCE_TOPIC, BridgeConfig  # noqa: E402

GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE = "leader_bridge"


def load_group(name: str) -> dict:
    """Load one group_vars file.

    Args:
        name (str): Group name (client, server).

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load((GROUP_VARS / f"{name}.yml").read_text())


def client_node(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name (str): Node entry name.

    Returns:
        dict: The entry.
    """
    return next(n for n in load_group("client")["ros2_nodes"] if n["name"] == name)


def test_node_type_is_declared_on_the_client() -> None:
    defaults = load_group("client")["ros2_node_type_defaults"][NODE]
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_src_dir"] == "nodes/leader_bridge"
    assert defaults["config_path"] == "/etc/ros2/leader_bridge"
    assert "LEADER_BRIDGE_CONFIG=/etc/ros2/leader_bridge/config.yaml" in defaults["env"]
    assert "ROS_STATIC_PEERS={{ ros2_server_hostname }}" in defaults["env"]


def test_ros2_nodes_entry_follows_the_schema() -> None:
    node = client_node(NODE)
    assert node["node_type"] == NODE
    assert node["present"] is True and node["enabled"] is True
    assert isinstance(node["config"], str)


def test_deployed_config_validates_against_the_pydantic_config() -> None:
    config = BridgeConfig.model_validate(yaml.safe_load(client_node(NODE)["config"]))
    assert config.source_topic == DEFAULT_SOURCE_TOPIC == "/leader/joint_states"
    assert config.dest_topic == DEFAULT_DEST_TOPIC == "/filter/input_joint_updates"


def test_master2master_stays_disabled() -> None:
    node = client_node("master2master")
    assert node["present"] is True and node["enabled"] is False


def test_server_needs_no_bridge_entry_and_still_runs_the_leader() -> None:
    server = load_group("server")
    assert NODE not in server.get("ros2_node_type_defaults", {})
    assert all(n["node_type"] != NODE for n in server["ros2_nodes"])
    leader = next(n for n in server["ros2_nodes"] if n["name"] == "lerobot_leader")
    assert leader["present"] is True and leader["enabled"] is True


def test_client_playbook_deploys_the_bridge() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert "_deploy_node_name: leader_bridge" in text
    assert "tags: [leader_bridge, apt, python, build, config]" in text


def test_ci_matrix_lists_the_node() -> None:
    assert "- leader_bridge" in (REPO_ROOT / ".github" / "workflows" / "ci.yml").read_text()


def test_docs_list_the_node_and_this_test() -> None:
    assert "**leader_bridge/**" in (REPO_ROOT / "nodes" / "README.md").read_text()
    assert "test_leader_bridge_config.py" in (REPO_ROOT / "tests" / "README.md").read_text()
