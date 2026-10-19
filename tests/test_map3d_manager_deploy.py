"""Deploy wiring of the map3d_manager_cpp node type on the server (G3): colcon local sources, apt list, caps, entry
order, playbook step and config topics equal to the manager side of the map3d topic contract."""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_server.yml"
CONFIG_HPP = REPO_ROOT / "nodes" / "map3d_manager_cpp" / "include" / "map3d_manager_cpp" / "config.hpp"
PACKAGE_XML = REPO_ROOT / "nodes" / "map3d_manager_cpp" / "package.xml"
NODE_DIR = REPO_ROOT / "nodes" / "rtabmap_mapping"
sys.path.insert(0, str(NODE_DIR))

from map3d_topics import load_map3d_topics  # noqa: E402

TYPE = "map3d_manager_cpp"
ENTRY = "map3d_manager"
WORKSPACE = "/opt/ros2-ws"
REQUIRED_APT = {
    "ros-jazzy-rtabmap-msgs",
    "nlohmann-json3-dev",
    "libyaml-cpp-dev",
    "prometheus-cpp-dev",
    "ros-jazzy-rclcpp",
    "ros-jazzy-std-msgs",
    "ros-jazzy-std-srvs",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-ament-cmake",
    "build-essential",
    "cmake",
    "rsync",
    "python3-colcon-common-extensions",
}
TIER_KEYS = {"cpu_weight", "io_weight", "oom_score_adjust", "memory_low", "memory_high"}


def server_vars() -> dict:
    return yaml.safe_load(SERVER_VARS.read_text())


def node_type() -> dict:
    return server_vars()["ros2_node_type_defaults"][TYPE]


def playbook_tasks() -> list[dict]:
    plays = yaml.safe_load(PLAYBOOK.read_text())
    tasks = [t for play in plays for t in play.get("tasks", [])]
    return [s for t in tasks for s in t.get("block", [t])]


def test_node_type_fields() -> None:
    nt = node_type()
    assert nt["deploy_mode"] == "native"
    assert nt["node_launch_command"] == "ros2 run map3d_manager_cpp map3d_manager_node"
    assert nt["node_src_dir"] == ""
    assert nt["src_paths"] == []
    assert nt["config_path"] == "/etc/ros2/map3d_manager"
    assert nt["env"] == ["MAP3D_MANAGER_CONFIG=/etc/ros2/map3d_manager/config.yaml"]


def test_colcon_sources_common_then_node_release_jobs_2() -> None:
    sources = node_type()["colcon_source"]
    assert [(s["local"], s["package"], s["workspace"]) for s in sources] == [
        ("shared/ros2_common_cpp", "ros2_common_cpp", WORKSPACE),
        ("nodes/map3d_manager_cpp", "map3d_manager_cpp", WORKSPACE),
    ]
    for s in sources:
        assert s["cmake_args"] == ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"]
        assert s["jobs"] == 2 and s["base_paths"] == ["."]


def test_apt_covers_every_package_xml_dependency() -> None:
    apt = set(node_type()["apt_packages"])
    assert REQUIRED_APT <= apt
    keys = {e.text.strip() for tag in ("depend", "buildtool_depend") for e in ET.parse(PACKAGE_XML).getroot().iter(tag)}
    for key in keys - {"ros2_common_cpp", "yaml-cpp", "nlohmann-json-dev"}:
        assert "ros-jazzy-" + key.replace("_", "-") in apt, key


def test_quota_and_cap_without_tier_keys() -> None:
    nt = node_type()
    assert nt["cpu_quota"] == "20%"
    assert nt["memory_max"] == "128M"
    assert not TIER_KEYS & set(nt)


def test_ros2_nodes_entry_after_rtabmap_mapping() -> None:
    nodes = server_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    assert names.index(ENTRY) == names.index("rtabmap_mapping") + 1
    entry = nodes[names.index(ENTRY)]
    assert entry["node_type"] == TYPE and entry["present"] is True and entry["enabled"] is True
    assert not entry.get("config")
    block = SERVER_VARS.read_text().split(f"  - name: {ENTRY}\n", 1)[1].split("\n  - name:")[0]
    assert "#" in block, "a comment says no config differs from the code defaults"


def test_config_defaults_equal_the_contract_manager_topics() -> None:
    """With no config block the code defaults apply, so they must be the manager side of the topic contract."""
    topics = load_map3d_topics()
    header = CONFIG_HPP.read_text()
    default = {k: v for k, v in re.findall(r'std::string (\w+_topic) = "([^"]+)";', header)}
    uplink = {s.source: s.dest for s in topics.uplink if "map3d_manager" in s.server_readers}
    downlink = {s.source: s.dest for s in topics.downlink}
    assert default["command_topic"] == next(d for s, d in uplink.items() if s == "/map3d/command")
    assert default["map_reset_topic"] == uplink["/map_guard/map_reset"]
    assert default["result_topic"] in downlink and downlink[default["result_topic"]] == "/map3d/result"
    assert default["status_topic"] in downlink and downlink[default["status_topic"]] == "/map3d/status"


def test_playbook_deploys_after_rtabmap_mapping() -> None:
    tasks = playbook_tasks()
    deploys = [t["vars"]["_deploy_node_name"] for t in tasks if "_deploy_node_name" in t.get("vars", {})]
    assert deploys.index(ENTRY) == deploys.index("rtabmap_mapping") + 1
    step = next(t for t in tasks if t.get("vars", {}).get("_deploy_node_name") == ENTRY)
    assert step["name"] == f"Deploy {ENTRY}"
    assert {ENTRY, "apt", "build", "config"} <= set(step["tags"])
    assert step["ansible.builtin.include_tasks"]["apply"]["tags"] == [ENTRY]
