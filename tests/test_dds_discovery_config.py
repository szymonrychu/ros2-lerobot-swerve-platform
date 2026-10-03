"""Invariants of the DDS discovery setup in Ansible group_vars and templates.

All ROS2 nodes use simple discovery restricted to the host (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST from the shared
ros2_dds_env rendered by the unit template). Only the client's master2master adds the server as a static peer, so it
is the single process bridging the two hosts; server nodes list no static peers (that caused every client node to mesh
with the server over Wi-Fi). The former FastDDS discovery server is uninstalled (present: false): launch_ros' one-shot
lifecycle/component service calls hung intermittently through it. logind must keep the node user's shared memory.
"""

from pathlib import Path

import pytest
import yaml

ANSIBLE_DIR = Path(__file__).resolve().parent.parent / "ansible"
GROUP_VARS = ANSIBLE_DIR / "group_vars"
SERVICE_TEMPLATE = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-native.service.j2"
FORBIDDEN_NODE_ENV = ("ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE")
DISCOVERY_NODE = "fastdds_discovery_server"


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
def test_nodes_do_not_override_discovery_except_master2master_peer(host: str) -> None:
    group = load_vars(host)
    for entry in all_env_entries(group):
        assert not entry.startswith(FORBIDDEN_NODE_ENV), f"{host}: {entry}"
    peers = [
        (node_type, entry)
        for node_type, defaults in group["ros2_node_type_defaults"].items()
        for entry in defaults.get("env", []) or []
        if entry.startswith("ROS_STATIC_PEERS")
    ]
    peers += [
        (n["name"], e) for n in group["ros2_nodes"] for e in n.get("env", []) or [] if e.startswith("ROS_STATIC_PEERS")
    ]
    expected = [("master2master", "ROS_STATIC_PEERS={{ ros2_server_hostname }}")] if host == "client" else []
    assert peers == expected


def test_shared_env_is_localhost_simple_discovery() -> None:
    common = load_vars("all")
    assert common["ros2_dds_env"] == ["ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"]
    for removed in ("ros2_dds_discovery_port", "ros2_dds_local_discovery"):
        assert removed not in common


@pytest.mark.parametrize("host", ["client", "server"])
def test_discovery_server_uninstalled(host: str) -> None:
    group = load_vars(host)
    node = next(n for n in group["ros2_nodes"] if n["name"] == DISCOVERY_NODE)
    assert node["present"] is False and node["enabled"] is False
    assert "ros2_dds_server_id" not in group


def test_service_template_emits_shared_env_first_without_discovery_server_ordering() -> None:
    text = SERVICE_TEMPLATE.read_text()
    assert text.index("ros2_dds_env") < text.index("node_env")
    assert "fastdds_discovery_server" not in text


def test_steamdeck_uses_client_static_peer() -> None:
    defaults = yaml.safe_load((ANSIBLE_DIR / "roles" / "steamdeck_ui" / "defaults" / "main.yml").read_text())
    assert defaults["steamdeck_ros2_static_peers"] == "{{ ros2_client_hostname }}"


def test_shell_env_uses_localhost_discovery() -> None:
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "tasks" / "dds_host_setup.yml").read_text())
    profile = next(t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == "/etc/profile.d/ros2_dds.sh")
    content = profile["ansible.builtin.copy"]["content"]
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in content
    assert "unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT" in content


def test_host_setup_keeps_ipc_of_node_user() -> None:
    """logind must not delete the node user's shared memory (FastDDS SHM transport) when SSH sessions end."""
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "tasks" / "dds_host_setup.yml").read_text())
    logind = [
        t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest", "").startswith("/etc/systemd/logind.conf.d/")
    ]
    assert logind and "RemoveIPC=no" in logind[0]["ansible.builtin.copy"]["content"]
    restarts = [t for t in tasks if "ros2-" in str(t.get("ansible.builtin.shell", ""))]
    assert restarts and "when" in restarts[0]
