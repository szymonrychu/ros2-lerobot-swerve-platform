"""Static invariants of the jetson_bridge_cpp deployment on the client (node type, entry, tier, metrics, playbook).

jetson_bridge_cpp is the Pi side of the Jetson depth offload: a ZeroMQ SUB from depthd over the direct cable that
republishes /stereo/* and /overview_camera/*. It replaces the camera in the robot (normal) tier. The Pi's Alloy also
scrapes depthd on the Jetson, because the Jetson has no route to the server.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
ANSIBLE_README = ANSIBLE_DIR / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
CONFIG_HPP = REPO_ROOT / "nodes" / "jetson_bridge_cpp" / "include" / "jetson_bridge_cpp" / "config.hpp"
NODE = "jetson_bridge_cpp"
WORKSPACE = "/opt/ros2-ws"
METRICS_PORT = 19114
DEPTHD_METRICS_PORT = 8090
ZMQ_PORT = 5550
NORMAL_TIER = {"cpu_weight": 500, "io_weight": 500, "oom_score_adjust": -500}
APT_PACKAGES = ("libzmq3-dev", "libzstd-dev", "prometheus-cpp-dev", "libyaml-cpp-dev", "ros-jazzy-sensor-msgs")
# Every BridgeConfig key (nodes/jetson_bridge_cpp/include/jetson_bridge_cpp/config.hpp) and its default.
CONFIG_DEFAULTS = {
    "endpoint": f"tcp://{{{{ jetson_link_jetson_ip }}}}:{ZMQ_PORT}",
    "depth_topic": "/stereo/depth/image_raw",
    "camera_info_topic": "/stereo/camera_info",
    "fused_topic": "/stereo/depth/fused",
    "rgb_topic": "/overview_camera/image_raw/compressed",
    "rgb_info_topic": "/overview_camera/camera_info",
    "max_age_ms": 500,
    "absent_after_ms": 2000,
    "metrics_port": METRICS_PORT,
}


def client() -> dict:
    """Parse ansible/group_vars/client.yml.

    Returns:
        dict: Group variables.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def entry() -> dict:
    """Return the jetson_bridge_cpp ros2_nodes entry.

    Returns:
        dict: The entry.
    """
    return next(n for n in client()["ros2_nodes"] if n["name"] == NODE)


def test_node_type_is_a_native_local_colcon_build() -> None:
    node_type = client()["ros2_node_type_defaults"][NODE]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == "ros2 run jetson_bridge_cpp jetson_bridge_node"
    assert node_type["node_src_dir"] == "" and node_type["src_paths"] == []
    sources = node_type["colcon_source"]
    assert [(s["local"], s["package"], s["workspace"]) for s in sources] == [
        ("shared/ros2_common_cpp", "ros2_common_cpp", WORKSPACE),
        ("nodes/jetson_bridge_cpp", "jetson_bridge_cpp", WORKSPACE),
    ]
    for source in sources:
        assert source["jobs"] == 2
        assert source["stamp_apt_packages"], "apt versions are part of the build stamp"
        assert set(source["stamp_apt_packages"]) <= set(node_type["apt_packages"])
    for package in APT_PACKAGES:
        assert package in node_type["apt_packages"], package
    assert node_type["config_path"] == f"/etc/ros2/{NODE}"
    assert node_type["env"] == [f"JETSON_BRIDGE_CONFIG=/etc/ros2/{NODE}/config.yaml"]


def test_node_type_is_in_the_normal_robot_tier_with_commented_memory() -> None:
    node_type = client()["ros2_node_type_defaults"][NODE]
    assert {k: node_type[k] for k in NORMAL_TIER} == NORMAL_TIER
    assert node_type["memory_low"] == "32M" and node_type["memory_max"] == "256M"
    lines = CLIENT_VARS.read_text().splitlines()
    low = next(i for i, line in enumerate(lines) if line.strip() == 'memory_low: "32M"')
    comment = [line for line in lines[max(0, low - 4) : low] if line.strip().startswith("#")]
    assert any("estimate" in line.lower() for line in comment), "memory is an estimate and must say so"


def test_entry_is_present_enabled_with_every_default_explicit() -> None:
    e = entry()
    assert e["node_type"] == NODE
    assert e["present"] is True and e["enabled"] is True
    config = yaml.safe_load(e["config"])
    assert config == CONFIG_DEFAULTS


def test_config_defaults_match_the_node_source() -> None:
    """The explicit keys are exactly the C++ config keys and its default topics."""
    text = CONFIG_HPP.read_text()
    for key, value in CONFIG_DEFAULTS.items():
        if key != "endpoint" and isinstance(value, str):
            assert f'"{value}"' in text, key
    assert f"DEFAULT_METRICS_PORT = {METRICS_PORT}" in text


def test_metrics_targets_scrape_the_bridge_and_the_jetson_depthd() -> None:
    targets = {t["node"]: t for t in client()["monitoring_agent_node_targets"]}
    assert targets[NODE] == {"node": NODE, "address": f"127.0.0.1:{METRICS_PORT}", "path": "/metrics"}
    depthd = targets["jetson_depthd"]
    assert depthd["address"] == f"{{{{ jetson_link_jetson_ip }}}}:{DEPTHD_METRICS_PORT}"
    assert depthd["path"] == "/metrics"


def test_client_playbook_has_the_tagged_deploy_block() -> None:
    plays = yaml.safe_load(CLIENT_PLAYBOOK.read_text())
    tasks = [c for t in plays[0]["tasks"] for c in (t["block"] if "block" in t else [t])]
    step = next(t for t in tasks if t.get("name") == f"Deploy {NODE}")
    assert step["vars"]["_deploy_node_name"] == NODE
    assert {NODE, "apt", "build", "config"} <= set(step["tags"])
    assert step["ansible.builtin.include_tasks"]["apply"]["tags"] == [NODE]


def test_docs_list_the_node_tier_and_metrics_ports() -> None:
    readme = ANSIBLE_README.read_text()
    assert NODE in readme and str(METRICS_PORT) in readme and "jetson_depthd" in readme
    assert "test_jetson_bridge_config.py" in TESTS_README.read_text()
