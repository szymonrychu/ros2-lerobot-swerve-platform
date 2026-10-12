"""Static invariants of the arducam_tof deployment (Ansible group_vars, SDK install task, playbook wiring).

arducam_tof runs on the client and publishes the Arducam ToF depth, amplitude and camera_info topics. It deploys
disabled (the camera is not plugged in yet) and its SDK install never fails a deploy.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
SDK_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "arducam_tof_sdk.yml"
NODE = "arducam_tof"
METRICS_PORT = 19112
APT_REPO = "https://arducam.github.io/arducam_ppa linux main"
SDK_PACKAGE = "arducam-tof-sdk-dev"
NORMAL_TIER = (500, 500, -500)


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
    assert defaults["node_launch_command"] == "python3 -m arducam_tof"
    assert defaults["node_src_dir"] == "nodes/bridges/arducam_tof"
    assert defaults["pythonpath_parent"] is True
    assert defaults["src_shared"] is True
    assert defaults["config_path"] == "/etc/ros2/arducam_tof"
    assert "ARDUCAM_TOF_CONFIG=/etc/ros2/arducam_tof/config.yaml" in defaults["env"]


def test_node_type_is_in_the_normal_cgroup_tier_with_memory_limits() -> None:
    defaults = load_group("client")["ros2_node_type_defaults"][NODE]
    assert (defaults["cpu_weight"], defaults["io_weight"], defaults["oom_score_adjust"]) == NORMAL_TIER
    assert defaults["memory_low"] and defaults["memory_max"]


def test_node_type_has_no_dds_peer() -> None:
    defaults = load_group("client")["ros2_node_type_defaults"][NODE]
    assert not any(e.startswith("ROS_STATIC_PEERS") for e in defaults["env"])
    assert not any(e.startswith("ROS_STATIC_PEERS") for e in client_node(NODE).get("env", []))


def test_ros2_nodes_entry_deploys_disabled() -> None:
    node = client_node(NODE)
    assert node["node_type"] == NODE
    assert node["present"] is True
    assert node["enabled"] is False
    assert isinstance(node["config"], str)


def test_deployed_config_holds_the_defaults_and_head_tab_topics() -> None:
    config = yaml.safe_load(client_node(NODE)["config"])
    assert config["camera_index"] == 1
    assert config["metrics_port"] == METRICS_PORT
    for key in ("range_mode", "confidence_threshold", "frame_id", "publish_rate_hz"):
        assert key in config
    assert config["depth_topic"] == "/tof/depth/image_raw"
    assert config["amplitude_topic"] == "/tof/amplitude/image_raw"
    assert config["camera_info_topic"] == "/tof/camera_info"


def test_metrics_port_is_scraped() -> None:
    targets = {t["node"]: t for t in load_group("client")["monitoring_agent_node_targets"]}
    assert targets[NODE]["address"] == f"127.0.0.1:{METRICS_PORT}"


def test_sdk_task_adds_the_arducam_repo_and_installs_the_sdk() -> None:
    text = SDK_TASKS.read_text()
    assert APT_REPO in text
    assert SDK_PACKAGE in text
    tasks = yaml.safe_load(text)
    assert tasks, "task file is empty"


def test_sdk_task_is_idempotent_and_never_fails_the_deploy() -> None:
    text = SDK_TASKS.read_text()
    assert "ansible.builtin.apt_repository" in text
    assert "state: present" in text
    assert "block:" in text and "rescue:" in text
    assert "state: absent" in text  # a failed install removes the repo again so the batched node apt run stays clean
    assert "ansible.builtin.debug" in text
    assert "ansible.builtin.command" not in text and "ansible.builtin.shell" not in text


def test_client_playbook_includes_the_sdk_task_and_deploys_the_node() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert "tasks/arducam_tof_sdk.yml" in text
    assert "name: Deploy arducam_tof" in text
    assert "_deploy_node_name: arducam_tof" in text
    assert "tags: [arducam_tof, apt, python, build, config]" in text
