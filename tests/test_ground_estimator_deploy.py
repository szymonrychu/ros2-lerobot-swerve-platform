"""Static invariants of the ground_estimator_cpp deploy: node type, ros2_nodes entry, playbook step, config consistency.

ground_estimator_cpp feeds the Nav2 local costmap, so it runs in the normal cgroup tier (protected above the
background work). Its config must agree with the nodes it listens to and the limits it mirrors (design D3).
"""

import math
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "ground_estimator_cpp"
DEFAULTS = NODE_DIR / "config" / "ground_estimator.yaml"
LASER_FILTER_DEFAULTS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
NAME = "ground_estimator_cpp"
WORKSPACE = "/opt/ros2-ws"
METRICS_PORT = 19119
TIER = {"cpu_weight": 500, "io_weight": 500, "oom_score_adjust": -500, "memory_low": "48M", "memory_max": "256M"}
REQUIRED_APT_PACKAGES = (
    "build-essential",
    "cmake",
    "rsync",
    "python3-colcon-common-extensions",
    "ros-jazzy-ament-cmake",
    "ros-jazzy-rclcpp",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-std-msgs",
    "ros-jazzy-tf2-ros",
    "ros-jazzy-tf2-eigen",
    "ros-jazzy-pcl-conversions",
    "libpcl-dev",
    "libopencv-dev",
    "libeigen3-dev",
    "libyaml-cpp-dev",
    "nlohmann-json3-dev",
    "prometheus-cpp-dev",
)


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type() -> dict:
    """Return the ground_estimator_cpp node type defaults.

    Returns:
        dict: ros2_node_type_defaults["ground_estimator_cpp"].
    """
    return client_vars()["ros2_node_type_defaults"][NAME]


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
    assert entry["node_launch_command"] == "ros2 run ground_estimator_cpp ground_estimator_node"
    assert entry["node_src_dir"] == ""
    assert entry["src_paths"] == []
    assert "ROS_STATIC_PEERS" not in " ".join(entry["env"])


def test_apt_packages() -> None:
    apt = node_type()["apt_packages"]
    for package in REQUIRED_APT_PACKAGES:
        assert package in apt


def test_normal_tier_values() -> None:
    entry = node_type()
    assert {k: entry[k] for k in TIER} == TIER


def test_config_path_and_env() -> None:
    entry = node_type()
    assert entry["config_path"] == "/etc/ros2/ground_estimator_cpp"
    assert entry["env"] == ["GROUND_ESTIMATOR_CONFIG=/etc/ros2/ground_estimator_cpp/config.yaml"]


def test_colcon_sources_order_and_build() -> None:
    sources = node_type()["colcon_source"]
    assert [(s["local"], s["package"], s["workspace"]) for s in sources] == [
        ("shared/ros2_common_cpp", "ros2_common_cpp", WORKSPACE),
        ("nodes/ground_estimator_cpp", "ground_estimator_cpp", WORKSPACE),
    ]
    for source in sources:
        assert source["cmake_args"] == ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"]
        assert source["base_paths"] == ["."]
        assert source["jobs"] == 2
    assert sources[0]["stamp_apt_packages"] == ["libyaml-cpp-dev", "prometheus-cpp-dev"]
    stamp = sources[1]["stamp_apt_packages"]
    assert {"ros-jazzy-rclcpp", "ros-jazzy-pcl-conversions", "libpcl-dev", "libopencv-dev"} <= set(stamp)
    assert set(stamp) <= set(node_type()["apt_packages"])


def test_ros2_nodes_entry() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    entry = nodes[names.index(NAME)]
    assert entry["node_type"] == NAME
    assert entry["present"] is True and entry["enabled"] is True
    assert "#" in entry["config"]
    parsed = yaml.safe_load(entry["config"])
    assert parsed == {**yaml.safe_load(DEFAULTS.read_text()), "metrics_port": METRICS_PORT}
    for upstream in ("arducam_tof", "laser_filter", "head_rgbd_cpp", "bno055_imu"):
        assert names.index(NAME) > names.index(upstream)


def test_config_keys_equal_the_shipped_yaml_keys() -> None:
    def key_paths(tree: dict, prefix: tuple = ()) -> set:
        paths = set()
        for key, value in tree.items():
            if isinstance(value, dict):
                paths |= key_paths(value, (*prefix, key))
            else:
                paths.add((*prefix, key))
        return paths

    assert key_paths(entry_config(NAME)) == key_paths(yaml.safe_load(DEFAULTS.read_text()))


def test_metrics_target_is_present() -> None:
    targets = client_vars()["monitoring_agent_node_targets"]
    assert {"node": NAME, "address": f"127.0.0.1:{METRICS_PORT}", "path": "/metrics"} in targets


def test_client_playbook_step_order() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    blocks = [t for t in play["tasks"] if "block" in t]
    steps = next(b["block"] for b in blocks if any(s["name"] == "Deploy ground_estimator_cpp" for s in b["block"]))
    names = [s["name"] for s in steps]
    index = names.index("Deploy ground_estimator_cpp")
    for upstream in ("Deploy arducam_tof", "Deploy laser_filter", "Deploy head_rgbd_cpp"):
        assert index > names.index(upstream)
    step = steps[index]
    include = step["ansible.builtin.include_tasks"]
    assert include["file"] == "tasks/resolve_and_deploy.yml"
    assert include["apply"]["tags"] == [NAME]
    assert step["vars"] == {"_deploy_node_name": NAME}
    assert step["tags"] == [NAME, "apt", "build", "config"]


def test_topics_match_their_producers() -> None:
    ground = entry_config(NAME)
    laser = yaml.safe_load(LASER_FILTER_DEFAULTS.read_text())
    assert ground["cloud_topic"] == laser["cloud_filtered_topic"]
    assert ground["imu_topic"] == entry_config("bno055_imu")["topic"]
    assert ground["rgbd_cloud_topic"] == entry_config("head_rgbd_cpp")["output_namespace"] + "/points"


def test_limits_match_tilt_guard_and_swerve() -> None:
    ground = entry_config(NAME)
    swerve = entry_config("swerve_controller")
    tilt = swerve["tilt_guard"]
    assert ground["imu"]["level_reference_roll_deg"] == tilt["level_reference_roll_deg"]
    assert ground["imu"]["level_reference_pitch_deg"] == tilt["level_reference_pitch_deg"]
    for key in ("roll_warn_deg", "roll_error_deg", "pitch_warn_deg", "pitch_error_deg"):
        assert ground["slope"][key] == tilt[key]
    trav = ground["traversability"]
    assert trav["wheel_radius_m"] == swerve["wheel_radius_m"] == 0.06
    max_step = trav["wheel_radius_m"] * 2 * trav["max_step_fraction"]
    assert math.isclose(max_step, 2 * trav["wheel_radius_m"] / 3, abs_tol=1e-4)
    assert trav["max_slope_deg"] <= min(tilt["roll_warn_deg"], tilt["pitch_warn_deg"])
