"""Static invariants of the head_rgbd_cpp deploy: node type, ros2_nodes entry, head camera mapping, ToF stamp setting.

head_rgbd_cpp adapts the D435 aligned depth and colour to the /head_rgbd contract. It is background work (a map input,
not a safety path), so it gets the background cgroup tier, not protection.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "head_rgbd_cpp"
DEFAULTS = NODE_DIR / "config" / "head_rgbd.yaml"
WORKSPACE = "/opt/ros2-ws"
METRICS_PORT = 19117
# Floor distances the ground guidance needs in the depth range, m (design D2: the floor at 0.3-2 m).
FLOOR_RANGE_M = (0.3, 2.0)
TIER = {"cpu_weight": 100, "io_weight": 100, "oom_score_adjust": 0, "memory_low": "48M", "memory_max": "256M"}
REQUIRED_APT_PACKAGES = (
    "build-essential",
    "cmake",
    "rsync",
    "python3-colcon-common-extensions",
    "ros-jazzy-ament-cmake",
    "ros-jazzy-rclcpp",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-message-filters",
    "ros-jazzy-tf2-ros",
    "ros-jazzy-tf2-eigen",
    "ros-jazzy-cv-bridge",
    "ros-jazzy-image-geometry",
    "ros-jazzy-camera-calibration-parsers",
    "ros-jazzy-depth-image-proc",
    "libopencv-dev",
    "libopencv-contrib-dev",
    "libeigen3-dev",
    "libyaml-cpp-dev",
    "prometheus-cpp-dev",
)
MAPPING = {"enabled": True, "lock_focus": True, "rate_hz": 2.0, "width": 576, "height": 324, "jpeg_quality": 80}


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type() -> dict:
    """Return the head_rgbd_cpp node type defaults.

    Returns:
        dict: ros2_node_type_defaults["head_rgbd_cpp"].
    """
    return client_vars()["ros2_node_type_defaults"]["head_rgbd_cpp"]


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
    apt = node_type()["apt_packages"]
    for package in REQUIRED_APT_PACKAGES:
        assert package in apt
    assert "libopencv-contrib-dev" in apt
    assert "ros-jazzy-depth-image-proc" in apt


def test_background_tier_values() -> None:
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
    assert {"ros-jazzy-rclcpp", "ros-jazzy-depth-image-proc", "libopencv-dev", "libopencv-contrib-dev"} <= set(stamp)
    assert set(stamp) <= set(node_type()["apt_packages"])


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
    assert entry["present"] is True and entry["enabled"] is False  # disabled (user 2026-10-24)
    config = entry["config"]
    assert "#" in config
    parsed = yaml.safe_load(config)
    defaults = yaml.safe_load(DEFAULTS.read_text())
    assert parsed == {**defaults, "metrics_port": METRICS_PORT}
    assert names.index("head_rgbd_cpp") > names.index("head_camera_cpp")
    assert names.index("head_rgbd_cpp") > names.index("arducam_tof")


def test_metrics_target_is_absent_while_disabled() -> None:
    """A disabled node is not scraped (it would read as down); add the target back when enabling."""
    targets = client_vars()["monitoring_agent_node_targets"]
    assert "head_rgbd_cpp" not in {t["node"] for t in targets}


def test_head_camera_mapping_and_focus_lock() -> None:
    config = entry_config("head_camera_cpp")
    assert config["lens_position"] == 1.0
    assert config["mapping"] == MAPPING
    assert config.get("af_mode", "continuous") == "continuous"


def test_arducam_tof_stamp_latency() -> None:
    config = entry_config("arducam_tof")
    assert config["stamp_latency_ms"] == 0.0


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


def test_rgbd_fusion_and_ground_nodes_are_disabled_not_removed() -> None:
    """User 2026-10-24: no RGB-D fusion and no ground estimate for driving; the raw ToF, the head obstacle cloud
    (d435_obstacle_cloud, D5) and the head camera stay on."""
    nodes = {n["name"]: n for n in client_vars()["ros2_nodes"]}
    for name in ("head_rgbd_cpp", "ground_estimator_cpp"):
        assert nodes[name]["present"] is True and nodes[name]["enabled"] is False, name
    for name in ("arducam_tof", "d435_obstacle_cloud", "head_camera_cpp"):
        assert nodes[name]["present"] is True and nodes[name]["enabled"] is True, name
    targets = {t["node"] for t in client_vars()["monitoring_agent_node_targets"]}
    assert not targets & {"head_rgbd_cpp", "ground_estimator_cpp"}
