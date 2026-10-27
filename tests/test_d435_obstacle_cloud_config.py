"""Static invariants of the d435_obstacle_cloud deployment (config-only rtabmap_util point_cloud_xyz node).

The node turns the realsense_d435 full-FOV depth image (/head/d435/depth/image_rect_raw, not the aligned one) into
/head/d435/points in d435_depth_optical_frame (design D5). It has no Python package; the launch command, params file,
Ansible type, playbook block and the uninstall of the retired tof_obstacle_cloud instance are checked here.
"""

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "d435_obstacle_cloud"
PARAMS_FILE = NODE_DIR / "config" / "point_cloud_xyz.yaml"
OLD_PARAMS_FILE = REPO_ROOT / "nodes" / "tof_obstacle_cloud" / "config" / "point_cloud_xyz.yaml"
REALSENSE_PARAMS = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "realsense_d435.yaml"
HEAD_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
NODE = "d435_obstacle_cloud"
OLD_NODE = "tof_obstacle_cloud"
APT_PACKAGE = "ros-jazzy-rtabmap-util"
DEPTH_FRAME = "d435_depth_optical_frame"
# Upper bound of the decimated depth image: 160x90 = 14400 points per frame on the Pi 5 CPU budget.
MAX_DECIMATED = (160, 90)
# D435 min-Z at 640x360 (realsense_d435.yaml comment): nothing nearer is measured.
D435_MIN_Z_M = 0.15
EXPECTED_PARAMS = {
    "decimation": 4,
    "min_depth": 0.15,
    "max_depth": 3.0,
    "voxel_size": 0.05,
    "filter_nans": True,
    "approx_sync": False,
    "noise_filter_radius": 0.1,
    "noise_filter_min_neighbors": 2,
    # 0 = rmw RELIABILITY_SYSTEM_DEFAULT, the reliability realsense2_camera publishes depth with (IMAGE_QOS).
    "qos": 0,
}
EXPECTED_REMAPS = {
    "depth/image": "/head/d435/depth/image_rect_raw",
    "depth/camera_info": "/head/d435/depth/camera_info",
    "cloud": "/head/d435/points",
    "__node": NODE,
}
NORMAL_TIER = (500, 500, -500)


def load_client() -> dict:
    """Load the client group_vars.

    Returns:
        dict: Parsed client.yml.
    """
    return yaml.safe_load((GROUP_VARS / "client.yml").read_text())


def node_type(name: str = NODE) -> dict:
    """Return a client node type.

    Args:
        name (str): Node type name, d435_obstacle_cloud by default.

    Returns:
        dict: The node type entry of client.yml.
    """
    return load_client()["ros2_node_type_defaults"][name]


def instance(name: str) -> dict:
    """Return a client ros2_nodes entry.

    Args:
        name (str): Instance name.

    Returns:
        dict: The ros2_nodes entry of client.yml.
    """
    return next(n for n in load_client()["ros2_nodes"] if n["name"] == name)


def launch_command() -> str:
    """Return the launch command with whitespace collapsed.

    Returns:
        str: Single-line launch command.
    """
    return " ".join(str(node_type()["node_launch_command"]).split())


def params() -> dict:
    """Return the point_cloud_xyz ROS params of the node.

    Returns:
        dict: The ros__parameters mapping of the params file.
    """
    return yaml.safe_load(PARAMS_FILE.read_text())[NODE]["ros__parameters"]


def depth_profile() -> tuple[int, int, int]:
    """Return the realsense_d435 depth stream profile.

    Returns:
        tuple[int, int, int]: Width, height and fps of depth_module.depth_profile.
    """
    width, height, fps = yaml.safe_load(REALSENSE_PARAMS.read_text())["depth_module.depth_profile"].split("x")
    return int(width), int(height), int(fps)


def test_launch_command_runs_point_cloud_xyz_with_params_file() -> None:
    cmd = launch_command()
    assert cmd.startswith("ros2 run rtabmap_util point_cloud_xyz --ros-args")
    assert "--params-file {{ ros2_repo_dest }}/nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml" in cmd


def test_remaps_read_the_d435_depth() -> None:
    remaps = dict(re.findall(r"-r (\S+?):=(\S+)", launch_command()))
    assert remaps == EXPECTED_REMAPS
    assert "aligned_depth_to_color" not in launch_command()


def test_node_type_is_native_config_only_with_apt_package() -> None:
    nt = node_type()
    assert nt["deploy_mode"] == "native"
    assert nt["node_src_dir"] == ""
    assert nt["src_paths"] == ["nodes/d435_obstacle_cloud"]
    assert nt["apt_packages"] == [APT_PACKAGE]
    assert not (NODE_DIR / "pyproject.toml").exists()


def test_node_type_is_normal_tier_with_estimated_memory() -> None:
    nt = node_type()
    assert (nt["cpu_weight"], nt["io_weight"], nt["oom_score_adjust"]) == NORMAL_TIER
    assert nt["memory_low"] == "64M"
    assert nt["memory_max"] == "192M"


def test_params_fit_the_d435_profile() -> None:
    width, height, _fps = depth_profile()
    assert (width, height) == (640, 360)
    p = params()
    for key, value in EXPECTED_PARAMS.items():
        assert p[key] == value, key
        assert type(p[key]) is type(value), key
    decimation = p["decimation"]
    assert width % decimation == 0 and height % decimation == 0
    assert width // decimation <= MAX_DECIMATED[0] and height // decimation <= MAX_DECIMATED[1]
    assert p["min_depth"] >= D435_MIN_Z_M
    assert p["min_depth"] < p["max_depth"]


def test_params_file_has_only_the_expected_keys() -> None:
    """No extra option (an empty-cloud or placeholder publish switch among them): only real depth frames publish."""
    assert set(params()) == set(EXPECTED_PARAMS)


def test_params_file_comments_every_key() -> None:
    lines = PARAMS_FILE.read_text().splitlines()
    for key in EXPECTED_PARAMS:
        index = next(i for i, line in enumerate(lines) if re.match(rf"\s+{key}:", line))
        assert lines[index - 1].lstrip().startswith("#"), f"{key} has no comment above it"
    text = PARAMS_FILE.read_text()
    assert "640x360" in text and "tof" not in text.lower()


def test_depth_frame_is_a_head_urdf_link() -> None:
    assert f'<link name="{DEPTH_FRAME}"/>' in HEAD_URDF.read_text()


def test_no_tf_publishing_argument_and_slam_toolbox_owns_map_odom() -> None:
    cmd = launch_command()
    assert "publish_tf" not in cmd and "/tf" not in cmd
    for name, nt in load_client()["ros2_node_type_defaults"].items():
        if name == "slam_toolbox":
            continue
        text = str(nt.get("node_launch_command", ""))
        assert "amcl" not in text and "static_transform_publisher" not in text, name
    assert "slam_toolbox" in load_client()["ros2_node_type_defaults"]


def test_playbook_deploys_node_after_realsense_d435() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    assert "_deploy_node_name: d435_obstacle_cloud" in text
    assert text.index("- name: Deploy realsense_d435\n") < text.index("- name: Deploy d435_obstacle_cloud\n")
    step = text[text.index("- name: Deploy d435_obstacle_cloud\n") :].split("\n\n")[0]
    assert "file: tasks/resolve_and_deploy.yml" in step
    assert "tags: [d435_obstacle_cloud]" in step
    assert "tags: [d435_obstacle_cloud, apt, python, build, config]" in step


def test_playbook_keeps_the_tof_step_so_the_uninstall_runs() -> None:
    assert "_deploy_node_name: tof_obstacle_cloud" in CLIENT_PLAYBOOK.read_text()


def test_old_instance_uninstalls_new_enabled() -> None:
    old = instance(OLD_NODE)
    assert old["node_type"] == OLD_NODE
    assert old["present"] is False
    assert "config" not in old and "enabled" not in old
    new = instance(NODE)
    assert new["node_type"] == NODE
    assert new["present"] is True
    assert new["enabled"] is True
    names = [n["name"] for n in load_client()["ros2_nodes"]]
    assert names.index(NODE) == names.index("realsense_d435") + 1


def test_old_node_type_keeps_only_what_the_uninstall_path_needs() -> None:
    stub = node_type(OLD_NODE)
    assert set(stub) == {"deploy_mode", "node_launch_command"}
    assert stub["deploy_mode"] == "native"
    assert stub["node_launch_command"]


def test_old_params_file_is_moved() -> None:
    assert not OLD_PARAMS_FILE.exists()


def test_node_is_not_on_the_server() -> None:
    server = yaml.safe_load((GROUP_VARS / "server.yml").read_text())
    assert NODE not in server.get("ros2_node_type_defaults", {})
    assert NODE not in [n["name"] for n in server.get("ros2_nodes", [])]
