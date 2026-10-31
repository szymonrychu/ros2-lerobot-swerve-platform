"""Wiring of the full-rate filtered D435 depth (head_rgbd_cpp filtered_depth -> d435_obstacle_cloud).

head_rgbd_cpp masks the depth edges of every full-FOV D435 frame (/head/d435/depth/image_rect_raw) and republishes it
on /head/d435/depth_filtered/{image_raw, camera_info}; d435_obstacle_cloud (rtabmap_util point_cloud_xyz) builds
/head/d435/points from that, so laser_filter, ground_estimator_cpp, Nav2 and the collision monitor never see the
mixed pixels at object edges. Each file names its own topics, so this test reads all of them and requires them to
agree, requires the deployed head_rgbd_cpp config to be the repo defaults, and pins the tier and metrics wiring the
new safety-path role needs. No ROS is needed: the YAML is read directly.
"""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
HEAD_RGBD_DEFAULTS = REPO_ROOT / "nodes" / "head_rgbd_cpp" / "config" / "head_rgbd.yaml"
HEAD_RGBD_README = REPO_ROOT / "nodes" / "head_rgbd_cpp" / "README.md"
REALSENSE_TOPICS = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
REALSENSE_README = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "README.md"
POINT_CLOUD_PARAMS = REPO_ROOT / "nodes" / "d435_obstacle_cloud" / "config" / "point_cloud_xyz.yaml"
LASER_FILTER_CONFIG = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
COMMON_CPP_CMAKE = REPO_ROOT / "shared" / "ros2_common_cpp" / "CMakeLists.txt"
TESTS_README = REPO_ROOT / "tests" / "README.md"

RAW_DEPTH_TOPIC = "/head/d435/depth/image_rect_raw"
RAW_INFO_TOPIC = "/head/d435/depth/camera_info"
FILTERED_NAMESPACE = "/head/d435/depth_filtered"
D435_CLOUD_TOPIC = "/head/d435/points"
METRICS_PORT = 19117
# The obstacle cloud is a safety input (collision monitor), so its producers share the normal tier.
NORMAL_TIER = (500, 500, -500)
# The head D435 at 15 fps: an approximate pair may span at most half a frame period.
D435_FPS = 15


def load_client() -> dict[str, Any]:
    """Load the client group_vars.

    Returns:
        dict[str, Any]: Parsed client.yml.
    """
    return yaml.safe_load(CLIENT_YML.read_text())


def head_rgbd_configs() -> list[dict[str, Any]]:
    """Return the deployed (client.yml) and the repo default head_rgbd_cpp configs.

    Returns:
        list[dict[str, Any]]: [client.yml ros2_nodes config, config/head_rgbd.yaml].
    """
    entry = next(n for n in load_client()["ros2_nodes"] if n["name"] == "head_rgbd_cpp")
    return [yaml.safe_load(entry["config"]), yaml.safe_load(HEAD_RGBD_DEFAULTS.read_text())]


def obstacle_cloud_remaps() -> dict[str, str]:
    """Return the -r remaps of the d435_obstacle_cloud launch command.

    Returns:
        dict[str, str]: Remap source -> target.
    """
    command = " ".join(str(node_type("d435_obstacle_cloud")["node_launch_command"]).split())
    return dict(re.findall(r"-r (\S+?):=(\S+)", command))


def node_type(name: str) -> dict[str, Any]:
    """Return one client node type.

    Args:
        name (str): Node type name.

    Returns:
        dict[str, Any]: ros2_node_type_defaults[name].
    """
    return load_client()["ros2_node_type_defaults"][name]


def test_obstacle_cloud_reads_the_head_rgbd_filtered_depth() -> None:
    remaps = obstacle_cloud_remaps()
    for config in head_rgbd_configs():
        filtered = config["filtered_depth"]
        assert filtered["enabled"] is True
        assert remaps["depth/image"] == filtered["output_image_topic"]
        assert remaps["depth/camera_info"] == filtered["output_info_topic"]
    assert remaps["depth/image"] == f"{FILTERED_NAMESPACE}/image_raw"
    assert remaps["cloud"] == D435_CLOUD_TOPIC
    assert RAW_DEPTH_TOPIC not in remaps.values()


def test_filtered_stage_reads_the_full_fov_depth_the_camera_publishes() -> None:
    topics = yaml.safe_load(REALSENSE_TOPICS.read_text())["topics"]
    for config in head_rgbd_configs():
        filtered = config["filtered_depth"]
        assert filtered["depth_topic"] == RAW_DEPTH_TOPIC
        assert filtered["info_topic"] == RAW_INFO_TOPIC
    assert topics[RAW_DEPTH_TOPIC]["type"] == "sensor_msgs/msg/Image"
    assert topics[RAW_INFO_TOPIC]["type"] == "sensor_msgs/msg/CameraInfo"
    assert topics[RAW_DEPTH_TOPIC]["frame_id"] == topics[RAW_INFO_TOPIC]["frame_id"]


def test_obstacle_chain_downstream_is_unchanged() -> None:
    """Nav2 and the ground estimator keep reading the laser_filter output of /head/d435/points."""
    lf = yaml.safe_load(LASER_FILTER_CONFIG.read_text())
    assert lf["cloud_topic"] == D435_CLOUD_TOPIC
    assert lf["cloud_filtered_topic"] == "/head/d435/points_filtered"


def test_point_cloud_xyz_pairs_image_and_info_exactly_with_a_reliable_match() -> None:
    """head_rgbd_cpp publishes image and camera_info with one header on a reliable publisher: exact sync and the system
    default (reliable) subscription of point_cloud_xyz both match."""
    params = yaml.safe_load(POINT_CLOUD_PARAMS.read_text())["d435_obstacle_cloud"]["ros__parameters"]
    assert params["approx_sync"] is False
    assert params["qos"] == 0
    text = POINT_CLOUD_PARAMS.read_text()
    assert "head_rgbd_cpp" in text and "depth_filtered" in text


def test_edge_mask_and_pairing_defaults_are_deployed() -> None:
    deployed, defaults = head_rgbd_configs()
    for key in ("edge_mask", "filtered_depth", "pairing"):
        assert deployed[key] == defaults[key], key
    assert defaults["edge_mask"]["enabled"] is True
    assert defaults["pairing"]["policy"] == "exact"
    assert 0 < defaults["pairing"]["max_interval_s"] <= 0.5 / D435_FPS


def test_deployed_config_is_the_repo_defaults() -> None:
    deployed, defaults = head_rgbd_configs()
    assert deployed == {**defaults, "metrics_port": METRICS_PORT}


def test_head_rgbd_shares_the_obstacle_cloud_tier() -> None:
    """The obstacle cloud stops when head_rgbd_cpp stops, so both carry the normal (robot) tier."""
    for name in ("head_rgbd_cpp", "d435_obstacle_cloud"):
        nt = node_type(name)
        assert (nt["cpu_weight"], nt["io_weight"], nt["oom_score_adjust"]) == NORMAL_TIER, name


def test_metrics_port_is_scraped() -> None:
    targets = {t["node"]: t for t in load_client()["monitoring_agent_node_targets"]}
    assert targets["head_rgbd_cpp"]["address"] == f"127.0.0.1:{METRICS_PORT}"
    for config in head_rgbd_configs():
        assert config.get("metrics_port", METRICS_PORT) == METRICS_PORT


def test_common_cpp_builds_without_opencv_where_it_is_not_installed() -> None:
    """map_guard_cpp (client) and map3d_manager_cpp (server) build ros2_common_cpp without libopencv-dev, so the
    depth_mask target must stay optional; head_rgbd_cpp installs OpenCV."""
    cmake = COMMON_CPP_CMAKE.read_text()
    assert "find_package(OpenCV QUIET" in cmake and "if(OpenCV_FOUND)" in cmake
    assert "libopencv-dev" in node_type("head_rgbd_cpp")["apt_packages"]


def test_docs_name_the_filtered_depth() -> None:
    readme = HEAD_RGBD_README.read_text()
    for needle in (f"{FILTERED_NAMESPACE}", "edge_mask", "pairing.policy", "head_rgbd_pair_stamp_delta_seconds"):
        assert needle in readme, needle
    realsense = REALSENSE_README.read_text()
    assert "## Filters" in realsense and FILTERED_NAMESPACE in realsense
    tests_readme = TESTS_README.read_text()
    assert "### test_d435_depth_filter_wiring.py" in tests_readme
