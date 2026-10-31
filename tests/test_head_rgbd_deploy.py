"""Static invariants of the head_rgbd_cpp deploy: node type, build dependencies, ros2_nodes entry, D435 inputs, rate.

head_rgbd_cpp adapts the D435 aligned depth and colour to the /head_rgbd contract and republishes the full-FOV depth
edge-masked for d435_obstacle_cloud. The obstacle cloud (collision monitor, Nav2) stops when it stops, so it gets the
normal cgroup tier of d435_obstacle_cloud, not the background one.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
REALSENSE_CONFIG = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "realsense_d435.yaml"
MAP3D_TOPICS = REPO_ROOT / "nodes" / "rtabmap_mapping" / "config" / "map3d_topics.yaml"
NODE_DIR = REPO_ROOT / "nodes" / "head_rgbd_cpp"
DEFAULTS = NODE_DIR / "config" / "head_rgbd.yaml"
WORKSPACE = "/opt/ros2-ws"
METRICS_PORT = 19117
GROUND_METRICS_PORT = 19119
# Floor distances the ground guidance needs in the depth range, m (design D2: the floor at 0.3-2 m).
FLOOR_RANGE_M = (0.3, 2.0)
TIER = {"cpu_weight": 500, "io_weight": 500, "oom_score_adjust": -500, "memory_low": "48M", "memory_max": "256M"}
REQUIRED_APT_PACKAGES = (
    "build-essential",
    "cmake",
    "rsync",
    "python3-colcon-common-extensions",
    "ros-jazzy-ament-cmake",
    "ros-jazzy-rclcpp",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-message-filters",
    "ros-jazzy-cv-bridge",
    "ros-jazzy-image-geometry",
    "ros-jazzy-depth-image-proc",
    "libopencv-dev",
    "libyaml-cpp-dev",
    "prometheus-cpp-dev",
)
# What only the removed ToF registration, motion compensation and guided upsampling used: the ToF calibration file
# parser, the head TF lookups and their Eigen maths, OpenCV calib3d (in libopencv-dev) and contrib ximgproc.
TOF_ONLY_APT_PACKAGES = (
    "ros-jazzy-tf2",
    "ros-jazzy-tf2-ros",
    "ros-jazzy-tf2-eigen",
    "ros-jazzy-camera-calibration-parsers",
    "libopencv-contrib-dev",
    "libeigen3-dev",
)
HEAD_RGBD_STAMP_APT_PACKAGES = [
    "ros-jazzy-rclcpp",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-message-filters",
    "ros-jazzy-cv-bridge",
    "ros-jazzy-image-geometry",
    "ros-jazzy-depth-image-proc",
    "libopencv-dev",
]


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type_of(name: str) -> dict:
    """Return the node type defaults of a node type.

    Args:
        name (str): Node type name.

    Returns:
        dict: ros2_node_type_defaults[name].
    """
    return client_vars()["ros2_node_type_defaults"][name]


def node_type() -> dict:
    """Return the head_rgbd_cpp node type defaults.

    Returns:
        dict: ros2_node_type_defaults["head_rgbd_cpp"].
    """
    return node_type_of("head_rgbd_cpp")


def entry_config(name: str) -> dict:
    """Return the parsed config block of a ros2_nodes entry.

    Args:
        name (str): Entry name.

    Returns:
        dict: The parsed `config: |` block.
    """
    entry = next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)
    return yaml.safe_load(entry["config"])


def test_node_type_keys() -> None:
    entry = node_type()
    assert entry["deploy_mode"] == "native"
    assert entry["node_launch_command"] == "ros2 run head_rgbd_cpp head_rgbd_node"
    assert entry["node_src_dir"] == ""
    assert entry["src_paths"] == []
    assert "ROS_STATIC_PEERS" not in " ".join(entry["env"])


def test_apt_packages() -> None:
    """The adapter and cloud.cpp build dependencies, none of the ToF-only ones."""
    apt = node_type()["apt_packages"]
    assert apt == list(REQUIRED_APT_PACKAGES)
    assert not set(TOF_ONLY_APT_PACKAGES) & set(apt)


def test_normal_tier_values() -> None:
    entry = node_type()
    assert {k: entry[k] for k in TIER} == TIER


def test_config_path_and_env() -> None:
    entry = node_type()
    assert entry["config_path"] == "/etc/ros2/head_rgbd_cpp"
    assert entry["env"] == ["HEAD_RGBD_CONFIG=/etc/ros2/head_rgbd_cpp/config.yaml"]


def test_colcon_sources_order_and_build() -> None:
    sources = node_type()["colcon_source"]
    assert [(s["local"], s["package"], s["workspace"]) for s in sources] == [
        ("shared/ros2_common_cpp", "ros2_common_cpp", WORKSPACE),
        ("nodes/head_rgbd_cpp", "head_rgbd_cpp", WORKSPACE),
    ]
    for source in sources:
        assert source["cmake_args"] == ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"]
        assert source["base_paths"] == ["."]
        assert source["jobs"] == 2
    assert sources[0]["stamp_apt_packages"] == ["libyaml-cpp-dev", "prometheus-cpp-dev"]
    stamp = sources[1]["stamp_apt_packages"]
    assert stamp == HEAD_RGBD_STAMP_APT_PACKAGES
    assert set(stamp) <= set(node_type()["apt_packages"])
    assert not set(TOF_ONLY_APT_PACKAGES) & set(stamp)


def test_depth_output_feeds_ground_guidance() -> None:
    """Ground guidance and RTAB-Map read <ns>/depth/* (no upsampled depth): namespace, halved depth, floor in range."""
    for config in (entry_config("head_rgbd_cpp"), yaml.safe_load(DEFAULTS.read_text())):
        assert config["output_namespace"] == "/head_rgbd"
        assert config["output"]["divisor"] == 2
        assert config["depth"]["min_m"] <= FLOOR_RANGE_M[0] and config["depth"]["max_m"] >= FLOOR_RANGE_M[1]
        assert "upsampling" not in config


def test_ros2_nodes_entry() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    entry = nodes[names.index("head_rgbd_cpp")]
    assert entry["node_type"] == "head_rgbd_cpp"
    assert entry["present"] is True and entry["enabled"] is True
    config = entry["config"]
    assert "#" in config
    parsed = yaml.safe_load(config)
    defaults = yaml.safe_load(DEFAULTS.read_text())
    assert parsed == {**defaults, "metrics_port": METRICS_PORT}
    assert names.index("head_rgbd_cpp") > names.index("head_camera_cpp")
    assert names.index("head_rgbd_cpp") > names.index("arducam_tof")


def test_metrics_targets_are_present() -> None:
    """The enabled RGB-D adapter and ground estimator are scraped by Alloy on their metrics ports."""
    targets = {t["node"]: t for t in client_vars()["monitoring_agent_node_targets"]}
    for name, port in (("head_rgbd_cpp", METRICS_PORT), ("ground_estimator_cpp", GROUND_METRICS_PORT)):
        assert targets[name]["address"] == f"127.0.0.1:{port}", name
        assert targets[name].get("path", "/metrics") == "/metrics", name


def test_inputs_are_the_d435_aligned_pair() -> None:
    """The deployed config and the defaults read the realsense_d435 aligned depth and raw colour at 2 Hz."""
    launch = node_type_of("realsense_d435")["node_launch_command"]
    assert "camera_namespace:=head" in launch and "camera_name:=d435" in launch
    assert yaml.safe_load(REALSENSE_CONFIG.read_text())["align_depth.enable"] is True
    for config in (entry_config("head_rgbd_cpp"), yaml.safe_load(DEFAULTS.read_text())):
        assert config["depth_topic"] == "/head/d435/aligned_depth_to_color/image_raw"
        assert config["rgb_image_topic"] == "/head/d435/color/image_raw"
        assert config["rgb_info_topic"] == "/head/d435/color/camera_info"
        assert config["output"]["rate_hz"] == 2.0


def test_output_rate_matches_the_mapping_budget() -> None:
    """output.rate_hz is the expected_rate_hz every /head_rgbd uplink stream of map3d_topics.yaml is budgeted for."""
    streams = [s for s in yaml.safe_load(MAP3D_TOPICS.read_text())["uplink"] if s["source"].startswith("/head_rgbd/")]
    assert {s["source"] for s in streams} == {
        "/head_rgbd/rgb/image_raw/compressed",
        "/head_rgbd/rgb/camera_info",
        "/head_rgbd/depth/image_raw",
        "/head_rgbd/depth/camera_info",
    }
    for config in (entry_config("head_rgbd_cpp"), yaml.safe_load(DEFAULTS.read_text())):
        assert {s["expected_rate_hz"] for s in streams} == {config["output"]["rate_hz"]}


def test_client_playbook_step_order() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    blocks = [t for t in play["tasks"] if "block" in t]
    steps = next(b["block"] for b in blocks if any(s["name"] == "Deploy head_camera_cpp" for s in b["block"]))
    names = [s["name"] for s in steps]
    index = names.index("Deploy head_rgbd_cpp")
    assert index > names.index("Deploy head_camera_cpp")
    assert index > names.index("Deploy arducam_tof")
    step = steps[index]
    include = step["ansible.builtin.include_tasks"]
    assert include["file"] == "tasks/resolve_and_deploy.yml"
    assert include["apply"]["tags"] == ["head_rgbd_cpp"]
    assert step["vars"] == {"_deploy_node_name": "head_rgbd_cpp"}
    assert step["tags"] == ["head_rgbd_cpp", "apt", "python", "build", "config"]


def test_rgbd_and_ground_nodes_are_enabled() -> None:
    """User 2026-10-28: ground (slope and obstacle) estimation runs on the D435 (design D1, D7). The RGB-D adapter, the
    ground estimator and their D435 inputs run and are scraped; gripper_uvc_camera and arducam_tof are retired
    (present: false), so the enabled memory_low sum stays within the cap (D14); neither is scraped."""
    nodes = {n["name"]: n for n in client_vars()["ros2_nodes"]}
    for name in ("head_rgbd_cpp", "ground_estimator_cpp", "realsense_d435", "d435_obstacle_cloud", "head_camera_cpp"):
        assert nodes[name]["present"] is True and nodes[name]["enabled"] is True, name
    assert nodes["gripper_uvc_camera"]["present"] is False
    assert nodes["arducam_tof"]["present"] is False
    targets = {t["node"] for t in client_vars()["monitoring_agent_node_targets"]}
    assert {"head_rgbd_cpp", "ground_estimator_cpp"} <= targets
    assert not targets & {"arducam_tof", "gripper_uvc_camera"}
