"""End-to-end topic chain of the head D435 obstacle cloud (design D5).

d435_obstacle_cloud (/head/d435/points, d435_depth_optical_frame) -> laser_filter self filter
(/head/d435/points_filtered) -> ground_estimator_cpp. Each file states its own topic, so one rename breaks the chain
silently; this test reads every file and requires the names to agree. The Nav2, web_ui and mcp_server consumers of the
filtered cloud are pinned by their own tests. No ROS is needed: the YAML is read directly.
"""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
SLAM_PARAMS = REPO_ROOT / "nodes" / "slam_toolbox" / "config" / "slam_params.yaml"
EKF_PARAMS = REPO_ROOT / "nodes" / "robot_localization_ekf" / "config" / "ekf.yaml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

D435_CLOUD_TOPIC = "/head/d435/points"
D435_FILTERED_TOPIC = "/head/d435/points_filtered"
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
