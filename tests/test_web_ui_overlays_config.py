"""Wiring of the web_ui status bar overlays in Ansible group_vars: live client topics, name-based steer selection."""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"

DEAD_PREFIX = "/controller/"


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client ros2_nodes entry.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config YAML of the node.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def overlays() -> dict[str, dict]:
    """Return the web_ui overlays keyed by label.

    Returns:
        dict[str, dict]: label -> overlay entry.
    """
    return {o["label"]: o for o in node_config("web_ui")["overlays"]}


def test_overlays_use_no_relay_only_topics() -> None:
    """No overlay reads a /controller/* topic (they only existed through the disabled master2master relay)."""
    assert all(not o["topic"].startswith(DEAD_PREFIX) for o in node_config("web_ui")["overlays"])


def test_lat_lon_come_from_the_client_gps_fix() -> None:
    """Lat/Lon read latitude/longitude of the rover fix published by gps_rtk_rover."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    block = next(n for n in nodes if n["name"] == "gps_rtk_rover")["config"]
    # The block holds Jinja placeholders, so it is not valid YAML before rendering: match the topic line.
    rover = re.search(r"^topic: (\S+)$", block, re.MULTILINE).group(1)
    assert rover == "/client/gps/fix"
    assert (overlays()["Lat"]["topic"], overlays()["Lat"]["field"]) == (rover, "latitude")
    assert (overlays()["Lon"]["topic"], overlays()["Lon"]["field"]) == (rover, "longitude")


def test_vel_comes_from_the_swerve_odometry() -> None:
    """Vel reads twist.twist.linear.x of the swerve_controller odom_topic."""
    odom = node_config("swerve_controller")["odom_topic"]
    assert odom == "/odom"
    assert (overlays()["Vel"]["topic"], overlays()["Vel"]["field"]) == (odom, "twist.twist.linear.x")


def test_fl_steer_is_selected_by_joint_name() -> None:
    """FL Steer reads the fl_steer joint of the swerve joint states by name, not by index."""
    joint_states = node_config("swerve_controller")["joint_states_topic"]
    item = overlays()["FL Steer"]
    assert item["topic"] == joint_states
    assert item["field"] == "position[name=fl_steer]"
    assert "fl_steer" in node_config("swerve_controller")["joint_names"]
