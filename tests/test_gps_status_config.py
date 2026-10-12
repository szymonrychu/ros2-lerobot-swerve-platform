"""GPS status wiring in Ansible group_vars: web_ui chips read the rover topic and the base caster's GET /status."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
SERVER_HOST_VAR = "{{ ros2_server_hostname }}"
TOPIC_SCRAPER_PORT = 18100


def node_config(path: Path, name: str) -> dict:
    """Parse the `config: |` block of one ros2_nodes entry, with the server hostname variable left as text.

    Args:
        path (Path): group_vars YAML file.
        name (str): Node name.

    Returns:
        dict: Parsed config YAML of the node.
    """
    nodes = yaml.safe_load(path.read_text())["ros2_nodes"]
    block = next(n for n in nodes if n["name"] == name)["config"]
    return yaml.safe_load(block.replace(SERVER_HOST_VAR, "SERVER_HOST"))


def test_web_ui_base_url_is_base_caster_status_route() -> None:
    """The base chip polls GET /status on the base node's NTRIP caster port, not the topic scraper."""
    base = node_config(SERVER_VARS, "gps_rtk_base")
    gps = node_config(CLIENT_VARS, "web_ui")["gps_status"]
    assert gps["base_url"] == f"http://SERVER_HOST:{base['rtcm_tcp_port']}/status"
    assert str(TOPIC_SCRAPER_PORT) not in gps["base_url"]


def test_web_ui_rover_topic_is_rover_status_topic() -> None:
    rover = node_config(CLIENT_VARS, "gps_rtk_rover")
    gps = node_config(CLIENT_VARS, "web_ui")["gps_status"]
    assert gps["rover_topic"] == rover["status_topic"]


def test_rover_connects_to_base_caster_port() -> None:
    base = node_config(SERVER_VARS, "gps_rtk_base")
    rover = node_config(CLIENT_VARS, "gps_rtk_rover")
    assert rover["rtcm_server_port"] == base["rtcm_tcp_port"]


def test_rover_nmea_timeout_below_web_ui_stale_window() -> None:
    """The silent status (timeout plus one status period) arrives before the web UI calls the rover chip stale."""
    rover = node_config(CLIENT_VARS, "gps_rtk_rover")
    gps = node_config(CLIENT_VARS, "web_ui")["gps_status"]
    status_period_s = 1.0 / rover.get("status_hz", 1.0)
    assert 0 < rover["nmea_timeout_s"]
    assert rover["nmea_timeout_s"] + status_period_s < gps["stale_after_s"]
