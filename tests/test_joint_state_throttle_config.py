"""10 Hz joint-state copies for idle consumers: ros-jazzy-topic-tools `throttle messages` on the client."""

import re
import shlex
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
APT_PACKAGE = "ros-jazzy-topic-tools"
THROTTLE_RATE_HZ = 10.0
THROTTLED_SUFFIX = "/throttled"
# Node name -> source topic: every 95 Hz joint-state topic mcp_server subscribes to (topics.follower_joint_states
# and topics.swerve_joint_states in nodes/mcp_server/mcp_server/config.py).
THROTTLES = {
    "joint_state_throttle_follower": "/follower/joint_states",
    "joint_state_throttle_swerve": "/swerve_drive/joint_states",
}
BACKGROUND_WEIGHTS = {"cpu_weight": 100, "io_weight": 100, "oom_score_adjust": 0}


def client_vars() -> dict:
    """Parse ansible/group_vars/client.yml.

    Returns:
        dict: Group variables.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_mcp_server_topic_defaults_are_the_throttled_sources() -> None:
    """The throttled sources are the joint-state topics mcp_server's config defaults subscribe to."""
    text = (REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "config.py").read_text()
    subscribed = set(re.findall(r'^\s+(?:follower|swerve)_joint_states: str = "([^"]+)"', text, re.M))
    assert subscribed == set(THROTTLES.values())


def test_each_throttle_is_an_apt_topic_tools_throttle_with_exact_source_rate_dest() -> None:
    """`throttle messages <topic> 10.0 <topic>/throttled`, run from apt, one node type per topic."""
    defaults = client_vars()["ros2_node_type_defaults"]
    for name, source in THROTTLES.items():
        entry = defaults[name]
        assert entry["deploy_mode"] == "native"
        assert entry["apt_packages"] == [APT_PACKAGE]
        args = shlex.split(entry["node_launch_command"])
        assert args[:3] == ["ros2", "run", "topic_tools"], name
        assert args[3:8] == ["throttle", "messages", source, str(THROTTLE_RATE_HZ), source + THROTTLED_SUFFIX], name
        assert args[8:] == ["--ros-args", "-r", f"__node:={name}"], name
        assert entry["node_src_dir"] == ""
        assert entry["src_paths"] == []


def test_each_throttle_is_a_present_enabled_node_in_the_background_tier() -> None:
    """One ros2_nodes entry per topic, present and enabled, with the background tier weights and memory protection."""
    group = client_vars()
    nodes = {n["name"]: n for n in group["ros2_nodes"]}
    for name in THROTTLES:
        assert nodes[name]["node_type"] == name
        assert nodes[name]["present"] is True
        assert nodes[name]["enabled"] is True
        entry = group["ros2_node_type_defaults"][name]
        for key, value in BACKGROUND_WEIGHTS.items():
            assert entry[key] == value, (name, key)
        assert entry["memory_low"] and entry["memory_max"]


def test_each_throttle_has_a_deploy_step_in_the_client_playbook() -> None:
    """The client playbook deploys every throttle node by name."""
    text = CLIENT_PLAYBOOK.read_text()
    for name in THROTTLES:
        assert f"_deploy_node_name: {name}\n" in text
        assert f"tags: [{name}, apt, python, build, config]" in text
