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


@pytest.mark.parametrize("host", ["client", "server"])
def test_env_keys_are_lists_when_present(host: str) -> None:
    """An `env:` key with no items parses as None and breaks env concatenation in resolve_and_deploy.yml."""
    group = load_vars(host)
    for name, defaults in group["ros2_node_type_defaults"].items():
        if "env" in defaults:
            assert isinstance(defaults["env"], list), f"{host}: node type {name} env is {defaults['env']!r}"
    for node in group["ros2_nodes"]:
        if "env" in node:
            assert isinstance(node["env"], list), f"{host}: node {node['name']} env is {node['env']!r}"


@pytest.mark.parametrize("host", ["client", "server"])
def test_ros_packages_synced_before_node_deploys(host: str) -> None:
    """Mixed ROS syncs break ABI (laser_filters vs diagnostic_updater, 2026-10-03): --all deploys upgrade all
    ros-jazzy packages together first and restart running nodes when anything was upgraded."""
    play = yaml.safe_load((ANSIBLE_DIR / "playbooks" / f"deploy_nodes_{host}.yml").read_text())[0]
    includes = [t.get("ansible.builtin.include_tasks", "") for t in play["pre_tasks"]]
    assert "tasks/ros_packages_sync.yml" in includes
    assert includes.index("tasks/ros_packages_sync.yml") > includes.index("tasks/repo_sync.yml")
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "tasks" / "ros_packages_sync.yml").read_text())
    text = yaml.safe_dump(tasks)
    assert "ros-jazzy-" in text and "only_upgrade: true" in text and "cache_valid_time" in text
    restart = [t for t in tasks if "systemctl" in str(t.get("ansible.builtin.shell", ""))]
    assert restart and "when" in restart[0]


def _deploy_playbooks() -> list[Path]:
    """Every deploy playbook: the --all playbooks and each per-node playbook."""
    pbs = [ANSIBLE_DIR / "playbooks" / f"deploy_nodes_{h}.yml" for h in ("client", "server")]
    return pbs + sorted((ANSIBLE_DIR / "playbooks" / "nodes").glob("*/*.yml"))


@pytest.mark.parametrize("playbook", _deploy_playbooks(), ids=lambda p: f"{p.parent.name}/{p.name}")
def test_deploy_stops_all_nodes_first_and_starts_them_gradually(playbook: Path) -> None:
    """Builds on the client overheated it next to the running stack (2026-10-03): every deploy first stops all ROS
    nodes and finally starts the enabled ones one by one."""
    play = yaml.safe_load(playbook.read_text())[0]
    pre = [t.get("ansible.builtin.include_tasks", "") for t in play["pre_tasks"]]
    stop = [i for i, inc in enumerate(pre) if inc.endswith("tasks/stop_ros_nodes.yml")]
    assert stop and stop[0] == 0, f"stop_ros_nodes must be the first pre_task: {pre}"
    post = [t.get("ansible.builtin.include_tasks", "") for t in play.get("post_tasks", [])]
    assert post and post[-1].endswith("tasks/start_ros_nodes.yml"), post


def test_start_ros_nodes_is_gradual_and_respects_present_enabled() -> None:
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "tasks" / "start_ros_nodes.yml").read_text())
    text = yaml.safe_dump(tasks)
    assert "present" in text and "enabled" in text
    assert "sleep {{ ros2_node_start_interval_s" in text
    assert load_vars("all")["ros2_node_start_interval_s"] == 2


def test_web_ui_deployed_first_in_client_all() -> None:
    """The web_ui frontend build is the heaviest step: run it while almost nothing else has been started."""
    tasks = yaml.safe_load((ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml").read_text())[0]["tasks"]
    order = [t.get("vars", {}).get("_deploy_node_name") for t in tasks if t.get("vars", {}).get("_deploy_node_name")]
    assert order[0] == "web_ui", order[:3]


def test_netplan_secondary_ethernet_is_optional() -> None:
    """An unplugged secondary ethernet (eth0 when the primary is Wi-Fi) made systemd-networkd-wait-online hold every
    ROS service for its 2-minute timeout at boot (2026-10-03)."""
    jinja2 = pytest.importorskip("jinja2")
    template = (ANSIBLE_DIR / "roles" / "network" / "templates" / "netplan.yaml.j2").read_text()
    rendered = jinja2.Template(template).render(
        _network_interface="wlan0",
        _network_interface_is_wlan=True,
        _other_ethernets=["eth0"],
        network_address="192.168.1.34/24",
        network_gateway="192.168.1.1",
        network_nameservers=["192.168.1.1"],
        network_wifi_ssid="main",
        network_wifi_password="x",
    )
    eth0 = yaml.safe_load(rendered)["network"]["ethernets"]["eth0"]
    assert eth0["optional"] is True


@pytest.mark.parametrize("host", ["client", "server"])
def test_boot_does_not_wait_for_every_interface(host: str) -> None:
    """Deploys install a wait-online drop-in (any interface, 30 s cap) without touching netplan."""
    play = yaml.safe_load((ANSIBLE_DIR / "playbooks" / f"deploy_nodes_{host}.yml").read_text())[0]
    includes = [t.get("ansible.builtin.include_tasks", "") for t in play["pre_tasks"]]
    assert "tasks/network_wait_online.yml" in includes
    text = (ANSIBLE_DIR / "playbooks" / "tasks" / "network_wait_online.yml").read_text()
    assert "systemd-networkd-wait-online.service.d" in text
    assert "--any" in text and "--timeout=30" in text
