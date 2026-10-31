"""Static invariants of the head_aim_cpp deploy: node type, ros2_nodes entry, playbook step, config consistency.

head_aim_cpp steers the D435 that feeds the Nav2 local costmap, so it runs in the normal cgroup tier. Its config must
agree with the nodes it listens to (nav2, the D435 cloud chain), the head bridge it commands and the footprint it
mirrors.
"""

import math
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
NODE_DIR = REPO_ROOT / "nodes" / "head_aim_cpp"
DEFAULTS = NODE_DIR / "config" / "head_aim.yaml"
NAME = "head_aim_cpp"
SOURCE_TAG = "head_aim"
WORKSPACE = "/opt/ros2-ws"
HEAD_JOINTS = ("head_pan", "head_tilt")
BASE_NODE = "base_servos"
STEPS_PER_TURN = 4096
STEP_CENTER = 2048
STEP_TOLERANCE_RAD = 2 * math.pi / STEPS_PER_TURN
FOOTPRINT_TOLERANCE_M = 5e-4  # the config rounds the box to 3 decimals (0.1958 -> 0.196)
TIER = {"cpu_weight": 500, "io_weight": 500, "oom_score_adjust": -500, "memory_low": "32M", "memory_max": "96M"}
REQUIRED_APT_PACKAGES = (
    "build-essential",
    "cmake",
    "rsync",
    "python3-colcon-common-extensions",
    "ros-jazzy-ament-cmake",
    "ros-jazzy-rclcpp",
    "ros-jazzy-nav-msgs",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-action-msgs",
    "ros-jazzy-geometry-msgs",
    "ros-jazzy-tf2",
    "ros-jazzy-tf2-ros",
    "ros-jazzy-tf2-geometry-msgs",
    "libyaml-cpp-dev",
)


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type() -> dict:
    """Return the head_aim_cpp node type defaults.

    Returns:
        dict: ros2_node_type_defaults["head_aim_cpp"].
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


def head_joints() -> dict[str, dict]:
    """Return the head joints of the base_servos bridge by name.

    Returns:
        dict[str, dict]: Joint entries of the `head` extra group.
    """
    group = next(g for g in entry_config(BASE_NODE)["extra_groups"] if g["namespace"] == "head")
    return {j["name"]: j for j in group["joint_names"]}


def steps_to_rad(steps: int, inverted: bool) -> float:
    """Convert servo steps to centred radians.

    Args:
        steps (int): Servo position steps.
        inverted (bool): Whether the joint is mounted reversed.

    Returns:
        float: Angle in rad.
    """
    rad = (steps - STEP_CENTER) * 2 * math.pi / STEPS_PER_TURN
    return -rad if inverted else rad


def test_node_type_keys() -> None:
    entry = node_type()
    assert entry["deploy_mode"] == "native"
    assert entry["node_launch_command"] == "ros2 run head_aim_cpp head_aim_node"
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
    assert entry["config_path"] == "/etc/ros2/head_aim_cpp"
    assert entry["env"] == ["HEAD_AIM_CONFIG=/etc/ros2/head_aim_cpp/config.yaml"]


def test_colcon_sources_order_and_build() -> None:
    sources = node_type()["colcon_source"]
    assert [(s["local"], s["package"], s["workspace"]) for s in sources] == [
        ("shared/ros2_common_cpp", "ros2_common_cpp", WORKSPACE),
        ("nodes/head_aim_cpp", "head_aim_cpp", WORKSPACE),
    ]
    for source in sources:
        assert source["cmake_args"] == ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"]
        assert source["base_paths"] == ["."]
        assert source["jobs"] == 2
    assert sources[0]["stamp_apt_packages"] == ["libyaml-cpp-dev", "prometheus-cpp-dev"]
    stamp = sources[1]["stamp_apt_packages"]
    assert {"ros-jazzy-rclcpp", "ros-jazzy-tf2-ros", "ros-jazzy-nav-msgs"} <= set(stamp)
    assert set(stamp) <= set(node_type()["apt_packages"])


def test_ros2_nodes_entry() -> None:
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    entry = nodes[names.index(NAME)]
    assert entry["node_type"] == NAME
    assert entry["present"] is True and entry["enabled"] is True
    assert "#" in entry["config"]
    assert yaml.safe_load(entry["config"]) == yaml.safe_load(DEFAULTS.read_text())
    assert names.index(NAME) > names.index("ground_estimator_cpp")


def test_client_playbook_step_order() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    blocks = [t for t in play["tasks"] if "block" in t]
    steps = next(b["block"] for b in blocks if any(s["name"] == f"Deploy {NAME}" for s in b["block"]))
    names = [s["name"] for s in steps]
    index = names.index(f"Deploy {NAME}")
    assert index == names.index("Deploy ground_estimator_cpp") + 1
    step = steps[index]
    include = step["ansible.builtin.include_tasks"]
    assert include["file"] == "tasks/resolve_and_deploy.yml"
    assert include["apply"]["tags"] == [NAME]
    assert step["vars"] == {"_deploy_node_name": NAME}
    assert step["tags"] == [NAME, "apt", "python", "build", "config"]


def test_source_tag_is_a_direct_command_source_of_the_head_bridge() -> None:
    config = entry_config(NAME)
    sources = entry_config(BASE_NODE)["direct_command_sources"]
    assert config["source_tag"] == SOURCE_TAG
    assert SOURCE_TAG in sources
    assert len(sources) == len(set(sources))


def test_yield_sources_are_exactly_the_other_direct_sources() -> None:
    config = entry_config(NAME)
    sources = entry_config(BASE_NODE)["direct_command_sources"]
    assert set(config["yield_sources"]) == set(sources) - {config["source_tag"]}


def test_footprint_box_equals_the_nav2_footprint_bounding_box() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    footprint = yaml.safe_load(params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
    xs, ys = [p[0] for p in footprint], [p[1] for p in footprint]
    config = entry_config(NAME)
    box = (config["footprint_min_x"], config["footprint_max_x"], config["footprint_min_y"], config["footprint_max_y"])
    expected = (min(xs), max(xs), min(ys), max(ys))
    assert all(math.isclose(a, b, abs_tol=FOOTPRINT_TOLERANCE_M) for a, b in zip(box, expected, strict=True))


def test_limits_and_home_equal_the_head_servo_config() -> None:
    config = entry_config(NAME)
    joints = head_joints()
    assert tuple(joints) == HEAD_JOINTS
    pan, tilt = joints["head_pan"], joints["head_tilt"]
    for joint, low_key, high_key in ((pan, "pan_min_rad", "pan_max_rad"), (tilt, "tilt_min_rad", "tilt_max_rad")):
        inverted = bool(joint.get("inverted", False))
        ends = sorted(steps_to_rad(joint[k], inverted) for k in ("command_min_steps", "command_max_steps"))
        assert math.isclose(config[low_key], ends[0], abs_tol=STEP_TOLERANCE_RAD)
        assert math.isclose(config[high_key], ends[1], abs_tol=STEP_TOLERANCE_RAD)
    assert config["home_pan_rad"] == pan["startup_position_rad"]
    assert config["home_tilt_rad"] == tilt["startup_position_rad"]


def test_topics_match_their_producers() -> None:
    config = entry_config(NAME)
    command = client_vars()["ros2_node_type_defaults"]["d435_obstacle_cloud"]["node_launch_command"]
    remaps = dict(re.findall(r"-r (\S+):=(\S+)", command))
    assert config["camera_info_topic"] == remaps["depth/camera_info"]
    assert config["head_command_topic"] == "/head/joint_commands"
    nav2 = yaml.safe_load(NAV2_PARAMS.read_text())
    assert config["cmd_vel_topic"] == "/" + nav2["collision_monitor"]["ros__parameters"]["cmd_vel_in_topic"]
    assert config["plan_topic"] == "/transformed_global_plan"
    assert config["goal_status_topic"] == "/navigate_to_pose/_action/status"
