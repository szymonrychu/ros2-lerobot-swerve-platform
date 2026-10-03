"""Invariants of the FastDDS Discovery Server setup in Ansible group_vars and templates.

Every ROS2 node gets ROS_DISCOVERY_SERVER from the shared ros2_dds_env (rendered by the unit template);
no node may fall back to the old mesh discovery variables, each host runs one discovery server with a
unique ID, and the cross-host bridge points at both servers in ID order.
"""

from pathlib import Path

import pytest
import yaml

ANSIBLE_DIR = Path(__file__).resolve().parent.parent / "ansible"
GROUP_VARS = ANSIBLE_DIR / "group_vars"
SERVICE_TEMPLATE = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-native.service.j2"
LEGACY_DISCOVERY_VARS = (
    "ROS_AUTOMATIC_DISCOVERY_RANGE",
    "ROS_STATIC_PEERS",
    "ROS_LOCALHOST_ONLY",
)
DISCOVERY_NODE_TYPE = "fastdds_discovery_server"
SUPER_CLIENT_TYPES = ("ros2_master", "topic_scraper_api", "web_ui", "master2master")


def load_vars(name: str) -> dict:
    """Load one group_vars file.

    Args:
        name: Group name (client, server, all).

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load((GROUP_VARS / f"{name}.yml").read_text())


def all_env_entries(group: dict) -> list[str]:
    """Collect env entries from node type defaults and node entries.

    Args:
        group: Parsed group_vars.

    Returns:
        list[str]: Every env string.
    """
    entries: list[str] = []
    for defaults in group.get("ros2_node_type_defaults", {}).values():
        entries += defaults.get("env", []) or []
    for node in group.get("ros2_nodes", []):
        entries += node.get("env", []) or []
    return entries


@pytest.mark.parametrize("host", ["client", "server"])
def test_no_legacy_discovery_variables(host: str) -> None:
    for entry in all_env_entries(load_vars(host)):
        assert not entry.startswith(LEGACY_DISCOVERY_VARS), f"{host}: legacy discovery env {entry}"


@pytest.mark.parametrize("host", ["client", "server"])
def test_discovery_server_node_is_first_and_present(host: str) -> None:
    group = load_vars(host)
    first = group["ros2_nodes"][0]
    assert first["node_type"] == DISCOVERY_NODE_TYPE
    assert first["name"] == "fastdds_discovery_server"
    assert first.get("present", True) and first.get("enabled", True)
    command = group["ros2_node_type_defaults"][DISCOVERY_NODE_TYPE]["node_launch_command"]
    assert "fastdds discovery" in command
    assert "-i {{ ros2_dds_server_id }}" in command
    assert "-p {{ ros2_dds_discovery_port }}" in command


def test_server_ids_unique_and_ordered() -> None:
    assert load_vars("client")["ros2_dds_server_id"] == 0
    assert load_vars("server")["ros2_dds_server_id"] == 1


def test_shared_env_sets_local_discovery_server() -> None:
    common = load_vars("all")
    assert common["ros2_dds_discovery_port"] == 11811
    # Position in ROS_DISCOVERY_SERVER must equal the server ID: pad with one ';' per ID.
    assert common["ros2_dds_local_discovery"] == "{{ ';' * ros2_dds_server_id }}127.0.0.1:{{ ros2_dds_discovery_port }}"
    assert common["ros2_dds_env"] == ["ROS_DISCOVERY_SERVER={{ ros2_dds_local_discovery }}"]


@pytest.mark.parametrize("host", ["client", "server"])
def test_introspection_nodes_are_super_clients(host: str) -> None:
    defaults = load_vars(host)["ros2_node_type_defaults"]
    for node_type in SUPER_CLIENT_TYPES:
        if node_type in defaults:
            assert "ROS_SUPER_CLIENT=TRUE" in defaults[node_type].get("env", []), f"{host}:{node_type}"


def test_master2master_uses_both_servers_in_id_order() -> None:
    env = load_vars("client")["ros2_node_type_defaults"]["master2master"]["env"]
    assert (
        "ROS_DISCOVERY_SERVER=127.0.0.1:{{ ros2_dds_discovery_port }};"
        "{{ ros2_server_hostname }}:{{ ros2_dds_discovery_port }}"
    ) in env


def test_service_template_emits_shared_env_before_node_env_and_orders_after_server() -> None:
    text = SERVICE_TEMPLATE.read_text()
    assert "ros2_dds_env" in text
    assert text.index("ros2_dds_env") < text.index("node_env")
    assert "After=network-online.target ros2-fastdds_discovery_server.service" in text
    assert "node_type != 'fastdds_discovery_server'" in text


def test_steamdeck_points_at_client_discovery_server() -> None:
    defaults = yaml.safe_load((ANSIBLE_DIR / "roles" / "steamdeck_ui" / "defaults" / "main.yml").read_text())
    assert defaults["steamdeck_ros2_discovery_server"] == "{{ ros2_client_hostname }}:11811"
    assert "steamdeck_ros2_static_peers" not in defaults
