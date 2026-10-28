"""End-to-end topic chain of the head D435 obstacle cloud (design D5).

d435_obstacle_cloud (/head/d435/points, d435_depth_optical_frame) -> laser_filter self filter
(/head/d435/points_filtered) -> ground_estimator_cpp. Each file states its own topic, so one rename breaks the chain
silently; this test reads every file and requires the names to agree. The Nav2, web_ui and mcp_server consumers are
re-pointed from the retired ToF filtered cloud one task at a time and each pins its D435 topic in its own tests; here
every consumer must read a self-filtered cloud (the laser_filter output, or the ToF one it has not left yet), never a
raw cloud. No ROS is needed: the YAML is read directly.
"""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
WEB_UI_DEFAULT = REPO_ROOT / "nodes" / "web_ui" / "config" / "default.yaml"
MCP_CONFIG_PY = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "config.py"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
SLAM_PARAMS = REPO_ROOT / "nodes" / "slam_toolbox" / "config" / "slam_params.yaml"
EKF_PARAMS = REPO_ROOT / "nodes" / "robot_localization_ekf" / "config" / "ekf.yaml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

D435_CLOUD_TOPIC = "/head/d435/points"
D435_FILTERED_TOPIC = "/head/d435/points_filtered"
# Self-filtered cloud of the retired head ToF, still read by the consumers that are not re-pointed yet.
RETIRED_TOF_FILTERED_TOPIC = "/tof/points_filtered"
STVL_PLUGIN = "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
MCP_CLOUD_KEYS = ("tof_points", "depth_points")
CHAIN_TESTS = ("test_d435_obstacle_cloud_config.py", "test_d435_obstacle_wiring.py")
CHAIN_NODES = ("d435_obstacle_cloud", "laser_filter")


def load_client() -> dict[str, Any]:
    """Load the client group_vars.

    Returns:
        dict[str, Any]: Parsed client.yml.
    """
    return yaml.safe_load(CLIENT_YML.read_text())


def node_entry(name: str) -> dict[str, Any]:
    """Return one `ros2_nodes` entry of client.yml.

    Args:
        name (str): Node name.

    Returns:
        dict[str, Any]: The entry.
    """
    return next(n for n in load_client()["ros2_nodes"] if n["name"] == name)


def node_config(name: str) -> dict[str, Any]:
    """Return the parsed `config: |` block of a `ros2_nodes` entry.

    Args:
        name (str): Node name.

    Returns:
        dict[str, Any]: Parsed config.
    """
    return yaml.safe_load(node_entry(name)["config"])


def launch_command(node_type: str) -> str:
    """Return the whitespace-normalised launch command of a client node type.

    Args:
        node_type (str): Key of `ros2_node_type_defaults`.

    Returns:
        str: The launch command on one line.
    """
    return " ".join(str(load_client()["ros2_node_type_defaults"][node_type]["node_launch_command"]).split())


def d435_cloud_output() -> str:
    """Return the `cloud` remap target of the d435_obstacle_cloud launch command.

    Returns:
        str: Topic the point_cloud_xyz node publishes.
    """
    return dict(re.findall(r"-r (\S+?):=(\S+)", launch_command("d435_obstacle_cloud")))["cloud"]


def find_key(data: Any, key: str) -> list[Any]:
    """Collect every value stored under `key` anywhere in a nested structure.

    Args:
        data (Any): Nested dicts and lists.
        key (str): Key to look for.

    Returns:
        list[Any]: Values found.
    """
    found: list[Any] = []
    if isinstance(data, dict):
        for k, v in data.items():
            if k == key:
                found.append(v)
            found.extend(find_key(v, key))
    elif isinstance(data, list):
        for item in data:
            found.extend(find_key(item, key))
    return found


def head_cloud_consumers() -> dict[str, list[str]]:
    """Return the topics every consumer of the self-filtered head cloud reads, by consumer.

    Returns:
        dict[str, list[str]]: collision_monitor pointcloud sources, local costmap STVL sources (not the /ground/ ones),
            web_ui map_nav `obstacle_cloud_topic` (client.yml and default.yaml) and the mcp_server cloud topic
            (client.yml and the config.py default).
    """
    nav2 = yaml.safe_load(NAV2_PARAMS.read_text())
    monitor = nav2["collision_monitor"]["ros__parameters"]
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    stvl = [
        local[layer][source]["topic"]
        for layer in local["plugins"]
        if local[layer].get("plugin") == STVL_PLUGIN
        for source in local[layer]["observation_sources"].split()
        if not local[layer][source]["topic"].startswith("/ground/")
    ]
    mcp_topics = node_config("mcp_server")["topics"]
    mcp_default = re.findall(r'(?:tof|depth)_points:\s*str\s*=\s*"([^"]+)"', MCP_CONFIG_PY.read_text())
    return {
        "collision_monitor": [
            monitor[s]["topic"] for s in monitor["observation_sources"] if monitor[s]["type"] == "pointcloud"
        ],
        "local_costmap": stvl,
        "web_ui": find_key(node_config("web_ui"), "obstacle_cloud_topic")
        + find_key(yaml.safe_load(WEB_UI_DEFAULT.read_text()), "obstacle_cloud_topic"),
        "mcp_server": [mcp_topics[k] for k in MCP_CLOUD_KEYS if k in mcp_topics] + mcp_default,
    }


def laser_filter_config() -> dict[str, Any]:
    """Load the laser_filter node config.

    Returns:
        dict[str, Any]: Parsed footprint_filter.yaml.
    """
    return yaml.safe_load(LASER_FILTER_CONFIG.read_text())


def test_d435_cloud_feeds_the_self_filter() -> None:
    lf = laser_filter_config()
    assert d435_cloud_output() == D435_CLOUD_TOPIC
    assert lf["cloud_topic"] == D435_CLOUD_TOPIC
    assert lf["cloud_filtered_topic"] == D435_FILTERED_TOPIC
    for name in CHAIN_NODES:
        assert node_entry(name)["present"] is True, name


def test_ground_estimator_input_is_the_laser_filter_output() -> None:
    assert node_config("ground_estimator_cpp")["cloud_topic"] == laser_filter_config()["cloud_filtered_topic"]


def test_every_consumer_reads_a_self_filtered_cloud() -> None:
    allowed = {laser_filter_config()["cloud_filtered_topic"], RETIRED_TOF_FILTERED_TOPIC}
    for consumer, topics in head_cloud_consumers().items():
        assert topics, consumer
        assert set(topics) <= allowed, (consumer, topics)


def test_slam_toolbox_publishes_map_to_odom() -> None:
    params = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    assert params["transform_publish_period"] > 0
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "odom"


def test_no_other_client_node_publishes_map_to_odom() -> None:
    client = load_client()
    publishers = [
        n["name"]
        for n in client["ros2_nodes"]
        if n.get("present", True) and "publish_tf: true" in str(n.get("config", ""))
    ]
    # robot_localization_ekf publishes odom -> base_link only (nodes/robot_localization_ekf/config/ekf.yaml).
    assert set(publishers) <= {"robot_localization_ekf"}, publishers
    static = node_config("static_tf_publisher")
    assert static["parent_frame"] != "map"
    assert "odom" not in [f["child"] for f in static["frames"]]
    ekf = yaml.safe_load(EKF_PARAMS.read_text())
    assert ekf["ekf_filter_node"]["ros__parameters"]["world_frame"] == "odom"
    assert "publish_tf" not in " ".join(str(client["ros2_node_type_defaults"]["d435_obstacle_cloud"].values()))


def test_chain_nodes_and_tests_are_in_the_tests_readme() -> None:
    readme = TESTS_README.read_text()
    for name in CHAIN_TESTS:
        assert f"### {name}" in readme, name
    for node in CHAIN_NODES:
        assert node in readme, node
