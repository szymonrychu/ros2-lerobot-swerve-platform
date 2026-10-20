"""End-to-end topic and frame chain of the head ToF obstacle pipeline (design D9, D12; U4).

arducam_tof -> tof_obstacle_cloud (/tof/points) -> laser_filter (/tof/points_filtered) -> Nav2 STVL, collision_monitor,
web_ui map_nav and mcp_server get_map_summary. Each file states its own topic, so one rename breaks the chain silently;
this test reads every file and requires the names to agree. No ROS is needed: the YAML is read directly.
"""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
WEB_UI_DEFAULT = REPO_ROOT / "nodes" / "web_ui" / "config" / "default.yaml"
MCP_CONFIG_PY = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "config.py"
SLAM_PARAMS = REPO_ROOT / "nodes" / "slam_toolbox" / "config" / "slam_params.yaml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

STVL_SOURCES = ("tof_mark", "tof_cliff_mark", "tof_clear")
COLLISION_TOF_SOURCES = ("tof_obstacles", "tof_cliff")
# collision_monitor source -> STVL marking source with the same height band.
BAND_PAIRS = {"tof_obstacles": "tof_mark", "tof_cliff": "tof_cliff_mark"}
TOF_FRAME = "tof_optical_frame"
NEW_NODE_TESTS = (
    "test_tof_obstacle_cloud_config.py",
    "test_nav_tof_voxel_layer.py",
    "test_nav_tof_collision.py",
    "test_tof_obstacle_wiring.py",
    "test_nav_ground_voxel_layer.py",
)
NEW_NODE_NAMES = ("tof_obstacle_cloud",)


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


def cloud_remap_output() -> str:
    """Return the `cloud` remap target of the tof_obstacle_cloud launch command.

    Returns:
        str: Topic the point_cloud_xyz node publishes.
    """
    cmd = " ".join(str(load_client()["ros2_node_type_defaults"]["tof_obstacle_cloud"]["node_launch_command"]).split())
    return dict(re.findall(r"-r (\S+?):=(\S+)", cmd))["cloud"]


def nav2_params() -> dict[str, Any]:
    """Load nav2_params.yaml.

    Returns:
        dict[str, Any]: Parsed file.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


def stvl() -> dict[str, Any]:
    """Return the local costmap tof_voxel_layer block.

    Returns:
        dict[str, Any]: STVL parameters.
    """
    return nav2_params()["local_costmap"]["local_costmap"]["ros__parameters"]["tof_voxel_layer"]


def collision_monitor() -> dict[str, Any]:
    """Return the collision_monitor parameters.

    Returns:
        dict[str, Any]: collision_monitor ros__parameters.
    """
    return nav2_params()["collision_monitor"]["ros__parameters"]


def laser_filter_config() -> dict[str, Any]:
    """Load the laser_filter node config.

    Returns:
        dict[str, Any]: Parsed footprint_filter.yaml.
    """
    return yaml.safe_load(LASER_FILTER_CONFIG.read_text())


def test_tof_obstacle_cloud_output_is_the_laser_filter_input() -> None:
    assert cloud_remap_output() == laser_filter_config()["cloud_topic"]


def test_laser_filter_output_feeds_every_consumer() -> None:
    out = laser_filter_config()["cloud_filtered_topic"]
    assert out != laser_filter_config()["cloud_topic"]
    layer = stvl()
    for source in STVL_SOURCES:
        assert layer[source]["topic"] == out, source
    monitor = collision_monitor()
    for source in COLLISION_TOF_SOURCES:
        assert monitor[source]["topic"] == out, source
    assert yaml.safe_load(WEB_UI_DEFAULT.read_text()) is not None
    assert set(find_key(yaml.safe_load(WEB_UI_DEFAULT.read_text()), "obstacle_cloud_topic")) == {out}
    assert set(find_key(node_config("web_ui"), "obstacle_cloud_topic")) == {out}
    assert node_config("mcp_server")["topics"]["tof_points"] == out
    default = re.search(r'tof_points:\s*str\s*=\s*"([^"]+)"', MCP_CONFIG_PY.read_text())
    assert default is not None
    assert default.group(1) == out


def test_ground_estimator_input_is_the_laser_filter_output_and_the_ground_clear_topic() -> None:
    out = laser_filter_config()["cloud_filtered_topic"]
    ground = node_config("ground_estimator_cpp")
    assert ground["cloud_topic"] == out
    ground_layer = nav2_params()["local_costmap"]["local_costmap"]["ros__parameters"]["ground_voxel_layer"]
    assert ground_layer["ground_clear"]["topic"] == out


def test_stvl_observation_sources_are_exactly_the_three_tof_sources() -> None:
    assert set(stvl()["observation_sources"].split()) == set(STVL_SOURCES)


def test_arducam_tof_frame_id_is_the_tof_optical_frame() -> None:
    assert node_config("arducam_tof")["frame_id"] == TOF_FRAME


def test_stvl_and_collision_monitor_height_bands_agree() -> None:
    layer = stvl()
    monitor = collision_monitor()
    for cm_source, stvl_source in BAND_PAIRS.items():
        assert monitor[cm_source]["min_height"] == layer[stvl_source]["min_obstacle_height"], cm_source
        assert monitor[cm_source]["max_height"] == layer[stvl_source]["max_obstacle_height"], cm_source


def test_ranges_and_collision_sources_are_listed() -> None:
    monitor = collision_monitor()
    assert {"scan", *COLLISION_TOF_SOURCES} <= set(monitor["observation_sources"])


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
    ekf = yaml.safe_load((REPO_ROOT / "nodes" / "robot_localization_ekf" / "config" / "ekf.yaml").read_text())
    assert ekf["ekf_filter_node"]["ros__parameters"]["world_frame"] == "odom"
    assert "publish_tf" not in " ".join(str(load_client()["ros2_node_type_defaults"]["tof_obstacle_cloud"].values()))


def test_new_nodes_and_tests_are_in_the_tests_readme() -> None:
    readme = TESTS_README.read_text()
    for name in NEW_NODE_TESTS:
        assert f"### {name}" in readme, name
    for node in NEW_NODE_NAMES:
        assert node in readme, node
