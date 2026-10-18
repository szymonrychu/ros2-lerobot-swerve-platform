"""Static invariants of the tof_obstacle_cloud deployment (config-only rtabmap_util point_cloud_xyz node).

The node turns the arducam_tof depth image into /tof/points in tof_optical_frame. It has no Python package; the
launch command, params file, Ansible type, playbook block and the TF ownership rules are checked here.
"""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "tof_obstacle_cloud"
PARAMS_FILE = NODE_DIR / "config" / "point_cloud_xyz.yaml"
NODE = "tof_obstacle_cloud"
APT_PACKAGE = "ros-jazzy-rtabmap-util"
RANGE_M = {"near": 2.0, "far": 4.0}
EXPECTED_PARAMS = {
    "decimation": 4,
    "min_depth": 0.2,
    "max_depth": 3.0,
    "voxel_size": 0.05,
    "filter_nans": True,
    "approx_sync": False,
    "noise_filter_radius": 0.1,
    "noise_filter_min_neighbors": 2,
}
EXPECTED_REMAPS = {
    "depth/image": "/tof/depth/image_raw",
    "depth/camera_info": "/tof/camera_info",
    "cloud": "/tof/points",
    "__node": NODE,
}
NORMAL_TIER = (500, 500, -500)


def load_client() -> dict:
    """Load the client group_vars.

    Returns:
        dict: Parsed client.yml.
    """
    return yaml.safe_load((GROUP_VARS / "client.yml").read_text())


def node_type() -> dict:
    """Return the tof_obstacle_cloud node type.

    Returns:
        dict: The node type entry of client.yml.
    """
    return load_client()["ros2_node_type_defaults"][NODE]


def launch_command() -> str:
    """Return the launch command with whitespace collapsed.

    Returns:
        str: Single-line launch command.
    """
    return " ".join(str(node_type()["node_launch_command"]).split())


def test_launch_command_runs_point_cloud_xyz_with_params_file() -> None:
    cmd = launch_command()
    assert cmd.startswith("ros2 run rtabmap_util point_cloud_xyz --ros-args")
    assert "--params-file {{ ros2_repo_dest }}/nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml" in cmd


def test_launch_command_remaps_topics_and_node_name() -> None:
    remaps = dict(re.findall(r"-r (\S+?):=(\S+)", launch_command()))
    assert remaps == EXPECTED_REMAPS


def test_node_type_is_native_config_only_with_apt_package() -> None:
    nt = node_type()
    assert nt["deploy_mode"] == "native"
    assert nt["node_src_dir"] == ""
    assert nt["src_paths"] == ["nodes/tof_obstacle_cloud"]
    assert nt["apt_packages"] == [APT_PACKAGE]
    assert not (NODE_DIR / "pyproject.toml").exists()


def test_node_type_is_normal_tier_with_estimated_memory() -> None:
    nt = node_type()
    assert (nt["cpu_weight"], nt["io_weight"], nt["oom_score_adjust"]) == NORMAL_TIER
    assert nt["memory_low"] == "64M"
    assert nt["memory_max"] == "192M"


def test_params_file_values() -> None:
    params = yaml.safe_load(PARAMS_FILE.read_text())[NODE]["ros__parameters"]
    for key, value in EXPECTED_PARAMS.items():
        assert params[key] == value, key
        assert type(params[key]) is type(value), key
    assert "qos" in params


def test_params_file_comments_every_key() -> None:
    lines = PARAMS_FILE.read_text().splitlines()
    for key in [*EXPECTED_PARAMS, "qos"]:
        index = next(i for i, line in enumerate(lines) if re.match(rf"\s+{key}:", line))
        assert lines[index - 1].lstrip().startswith("#"), f"{key} has no comment above it"


def test_max_depth_within_arducam_range_and_min_depth_positive() -> None:
    client = load_client()
    arducam = next(n for n in client["ros2_nodes"] if n["name"] == "arducam_tof")
    config = yaml.safe_load(arducam["config"])
    params = yaml.safe_load(PARAMS_FILE.read_text())[NODE]["ros__parameters"]
    assert params["max_depth"] <= RANGE_M[config["range_mode"]]
    assert params["max_depth"] <= min(RANGE_M.values()) * 1.5
    assert params["min_depth"] > 0


def test_arducam_frame_id_is_tof_optical_frame() -> None:
    arducam = next(n for n in load_client()["ros2_nodes"] if n["name"] == "arducam_tof")
    assert yaml.safe_load(arducam["config"])["frame_id"] == "tof_optical_frame"


def test_no_tf_publishing_argument_and_slam_toolbox_owns_map_odom() -> None:
    cmd = launch_command()
    assert "publish_tf" not in cmd and "/tf" not in cmd
    for name, nt in load_client()["ros2_node_type_defaults"].items():
        if name == "slam_toolbox":
            continue
        text = str(nt.get("node_launch_command", ""))
        assert "amcl" not in text and "static_transform_publisher" not in text, name
    assert "slam_toolbox" in load_client()["ros2_node_type_defaults"]


def test_playbook_deploys_node_after_arducam_tof() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert "_deploy_node_name: tof_obstacle_cloud" in text
    assert text.index("Deploy arducam_tof") < text.index("Deploy tof_obstacle_cloud")


def test_ros2_nodes_entry_present_enabled_after_arducam_tof() -> None:
    names = [n["name"] for n in load_client()["ros2_nodes"]]
    entry = next(n for n in load_client()["ros2_nodes"] if n["name"] == NODE)
    assert entry["node_type"] == NODE
    assert entry["present"] is True
    assert entry["enabled"] is True
    assert names.index(NODE) == names.index("arducam_tof") + 1


def test_node_is_not_on_the_server() -> None:
    server = yaml.safe_load((GROUP_VARS / "server.yml").read_text())
    assert NODE not in server.get("ros2_node_type_defaults", {})
    assert NODE not in [n["name"] for n in server.get("ros2_nodes", [])]


def test_readme_documents_topics_and_placeholder_rule() -> None:
    text = (NODE_DIR / "README.md").read_text()
    for needle in ("/tof/points", "tof_optical_frame", "depth_image_proc", "obstacles_detection", "empty cloud"):
        assert needle in text, needle
