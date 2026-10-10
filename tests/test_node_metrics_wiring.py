"""Node metrics wiring: client group_vars ports, shared/ros2_metrics dependencies and the Alloy node scrape targets."""

from pathlib import Path

import jinja2
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
MONITORING_DEFAULTS = ANSIBLE_DIR / "roles" / "monitoring" / "defaults" / "main.yml"
RENDER_VARS = {"ros2_server_hostname": "server.ros2.lan", "ros2_repo_dest": "/opt/ros2-lerobot-swerve-platform"}

# Port contract: nodes without an HTTP server get `metrics_port` (config) or METRICS_PORT (env).
CONFIG_METRICS_PORTS = {
    "lerobot_follower": 19101,
    "filter_node": 19102,
    "bno055_imu": 19103,
    "gps_rtk_rover": 19104,
    "swerve_controller": 19107,
    "rf2o_odom_relay": 19108,
    "poi_store": 19109,
}
ENV_METRICS_PORTS = {"gripper_uvc_camera": 19105, "rplidar_a1": 19106}
# Nodes with an HTTP server serve /metrics on it: node -> key path of the port in its config.
HTTP_PORT_KEYS = {
    "web_ui": ("http_port",),
    "test_joint_api": ("port",),
    "topic_scraper_api": ("port",),
    "mcp_server": ("server", "port"),
    "claude_agent": ("http_port",),
}
HTTP_PORTS = {
    "web_ui": 8080,
    "test_joint_api": 18080,
    "topic_scraper_api": 18100,
    "mcp_server": 18200,
    "claude_agent": 18300,
}
CONTRACT_NODES = set(CONFIG_METRICS_PORTS) | set(ENV_METRICS_PORTS) | set(HTTP_PORT_KEYS)
# Node types that import ros2_metrics (venv nodes through shared/, rplidar_a1 through PYTHONPATH).
SHARED_NODE_TYPES = [
    "feetech_servos",
    "filter_node",
    "swerve_controller",
    "rf2o_odom_relay",
    "bno055_imu",
    "gps_rtk",
    "uvc_camera",
    "poi_store",
    "claude_agent",
    "test_joint_api",
    "topic_scraper_api",
    "mcp_server",
    "web_ui",
]


def client_vars() -> dict:
    """Parse ansible/group_vars/client.yml.

    Returns:
        dict: Group variables.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def client_nodes() -> dict[str, dict]:
    """Return the client ros2_nodes entries by name.

    Returns:
        dict[str, dict]: Node name to its entry.
    """
    return {n["name"]: n for n in client_vars()["ros2_nodes"]}


def node_config(entry: dict) -> dict:
    """Render and parse a node's `config: |` block the way Ansible writes it.

    Args:
        entry (dict): A ros2_nodes entry.

    Returns:
        dict: Parsed config (empty when the node has none).
    """
    text = (
        jinja2.Environment(undefined=jinja2.StrictUndefined).from_string(entry.get("config", "")).render(**RENDER_VARS)
    )
    return yaml.safe_load(text) or {}


def node_env(entry: dict) -> dict[str, str]:
    """Merge the type's env with the node's env (node wins), as resolve_and_deploy.yml does.

    Args:
        entry (dict): A ros2_nodes entry.

    Returns:
        dict[str, str]: Variable name to value.
    """
    node_type = client_vars()["ros2_node_type_defaults"][entry["node_type"]]
    out: dict[str, str] = {}
    for item in node_type.get("env", []) + entry.get("env", []):
        key, value = item.split("=", 1)
        out[key] = value
    return out


def expected_port(entry: dict) -> int:
    """Return the metrics port a node is configured to serve on.

    Args:
        entry (dict): A ros2_nodes entry of a contract node.

    Returns:
        int: Port number.
    """
    name = entry["name"]
    if name in ENV_METRICS_PORTS:
        return int(node_env(entry)["METRICS_PORT"])
    config = node_config(entry)
    if name in CONFIG_METRICS_PORTS:
        return config["metrics_port"]
    value: object = config
    for key in HTTP_PORT_KEYS[name]:
        assert isinstance(value, dict)
        value = value[key]
    assert isinstance(value, int)
    return value


def node_targets() -> dict[str, dict]:
    """Return monitoring_node_targets by node name.

    Returns:
        dict[str, dict]: Node name to its target.
    """
    targets = yaml.safe_load(MONITORING_DEFAULTS.read_text())["monitoring_node_targets"]
    by_node = {t["node"]: t for t in targets}
    assert len(by_node) == len(targets), "duplicate node in monitoring_node_targets"
    return by_node


@pytest.mark.parametrize(("node", "port"), sorted(CONFIG_METRICS_PORTS.items()))
def test_config_metrics_port(node: str, port: int) -> None:
    """Nodes without an HTTP server have the contract metrics_port at the top level of their config."""
    assert node_config(client_nodes()[node]).get("metrics_port") == port


@pytest.mark.parametrize(("node", "port"), sorted(ENV_METRICS_PORTS.items()))
def test_env_metrics_port(node: str, port: int) -> None:
    """Env-configured nodes get METRICS_PORT."""
    assert node_env(client_nodes()[node]).get("METRICS_PORT") == str(port)


@pytest.mark.parametrize(("node", "port"), sorted(HTTP_PORTS.items()))
def test_http_nodes_keep_their_port(node: str, port: int) -> None:
    """HTTP nodes serve /metrics on their existing port; no extra metrics_port key."""
    entry = client_nodes()[node]
    assert expected_port(entry) == port
    assert "metrics_port" not in node_config(entry)


def test_rplidar_imports_ros2_metrics_without_a_venv() -> None:
    """rplidar_a1 runs system python: python3-prometheus-client from apt, ros2_metrics through PYTHONPATH."""
    types = client_vars()["ros2_node_type_defaults"]
    rplidar = types["rplidar_a1"]
    assert not rplidar["node_src_dir"]
    assert "python3-prometheus-client" in rplidar["apt_packages"]
    assert "ros-jazzy-rplidar-ros" in rplidar["apt_packages"]
    assert "shared/ros2_metrics" in rplidar["src_extra_paths"]
    env = node_env(client_nodes()["rplidar_a1"])
    rendered = jinja2.Environment().from_string(env["PYTHONPATH"]).render(**RENDER_VARS)
    assert rendered.split(":")[0] == "/opt/ros2-lerobot-swerve-platform/shared/ros2_metrics"
    assert (REPO_ROOT / "shared" / "ros2_metrics" / "ros2_metrics" / "__init__.py").is_file()


@pytest.mark.parametrize("node_type", SHARED_NODE_TYPES)
def test_node_types_using_ros2_metrics_declare_src_shared(node_type: str) -> None:
    """Every venv node type importing ros2_metrics restarts and re-syncs when shared/ changes."""
    assert client_vars()["ros2_node_type_defaults"][node_type].get("src_shared") is True


def test_server_group_vars_untouched() -> None:
    """Metrics are robot (client) only: no metrics_port or METRICS_PORT on the server."""
    text = SERVER_VARS.read_text()
    assert "metrics_port" not in text and "METRICS_PORT" not in text


def test_every_enabled_contract_node_has_a_matching_scrape_target() -> None:
    """Each enabled contract node in client ros2_nodes is scraped on 127.0.0.1:<its configured port>/metrics."""
    targets = node_targets()
    nodes = client_nodes()
    checked = 0
    for name in sorted(CONTRACT_NODES):
        entry = nodes[name]
        if not (entry.get("present", True) and entry.get("enabled", True)):
            continue
        target = targets[name]
        assert target["address"] == f"127.0.0.1:{expected_port(entry)}", name
        assert target.get("path", "/metrics") == "/metrics", name
        checked += 1
    assert checked == len(CONTRACT_NODES) == 14
    assert set(targets) == CONTRACT_NODES
