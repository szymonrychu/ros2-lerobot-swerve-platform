"""Static invariants of the arducam_tof deployment (Ansible group_vars, SDK install task, playbook wiring).

arducam_tof runs on the client and publishes the Arducam ToF depth, amplitude and camera_info topics from CSI cam1. It
deploys enabled and scraped by Alloy, and its SDK install never fails a deploy.
"""

import importlib.util
import os
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
SDK_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "arducam_tof_sdk.yml"
NODE = "arducam_tof"
DRIVER_CONFIG = Path(
    os.environ.get("ARDUCAM_TOF_DRIVER_CONFIG", REPO_ROOT / "nodes/bridges/arducam_tof/arducam_tof/config.py")
)
METRICS_PORT = 19112
CAM1_CONTROLLER = "1f00128000.csi"
APT_REPO = "https://arducam.github.io/arducam_ppa linux main"
SDK_PACKAGE = "arducam-tof-sdk-dev"
DRIVER_KEYS = {
    "csi_device",
    "range_mode",
    "confidence_threshold",
    "frame_id",
    "publish_rate_hz",
    "metrics_port",
    "stamp_latency_ms",
    "tof_calibration_file",
    "depth_is_radial",
    "flying_pixel_filter",
    "flying_pixel_jump_ratio",
    "flying_pixel_min_jump_mm",
}
HEAD_TAB_TOPICS = ("/tof/depth/image_raw", "/tof/amplitude/image_raw", "/tof/camera_info")
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
    assert not defaults.get("pythonpath_parent")  # nested layout: the package is nodes/bridges/arducam_tof/arducam_tof
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


def test_ros2_nodes_entry_deploys_enabled() -> None:
    node = client_node(NODE)
    assert node["node_type"] == NODE
    assert node["present"] is True
    assert node["enabled"] is True
    assert isinstance(node["config"], str)


def test_deployed_config_holds_only_keys_the_driver_accepts() -> None:
    """The driver config (T1 TofConfig) rejects unknown keys and takes range_mode near or far."""
    config = yaml.safe_load(client_node(NODE)["config"])
    assert set(config) == DRIVER_KEYS
    assert config["csi_device"] == CAM1_CONTROLLER  # resolved to its csi2_ch0 node at open: numbering varies by kernel
    assert config["metrics_port"] == METRICS_PORT
    assert config["range_mode"] in ("near", "far")


def test_deployed_config_loads_in_the_real_driver_load_config(tmp_path: Path) -> None:
    """The deployed YAML passes the driver's own load_config (extra keys forbidden, ranges checked)."""
    spec = importlib.util.spec_from_file_location("arducam_tof_driver_config", DRIVER_CONFIG)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    path = tmp_path / "config.yaml"
    path.write_text(client_node(NODE)["config"])
    loaded = module.load_config(path)
    assert loaded.csi_device == CAM1_CONTROLLER
    assert loaded.metrics_port == METRICS_PORT


def test_head_tab_topics_are_documented_in_the_config_comments() -> None:
    """The topics are fixed in the driver, so they are comments (a topic key would fail validation)."""
    text = client_node(NODE)["config"]
    for topic in HEAD_TAB_TOPICS:
        assert f"# {topic}" in text or f" {topic}" in text.split("#", 1)[1]


def test_metrics_port_is_scraped_exactly_when_the_node_is_enabled() -> None:
    """Alloy scrapes only enabled nodes (a disabled target would read as down); add the target when enabling."""
    group = load_group("client")
    targets = {t["node"]: t for t in group["monitoring_agent_node_targets"]}
    entry = next(n for n in group["ros2_nodes"] if n["name"] == NODE)
    if entry.get("present", True) and entry.get("enabled", True):
        assert targets[NODE]["address"] == f"127.0.0.1:{METRICS_PORT}"
    else:
        assert NODE not in targets


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
