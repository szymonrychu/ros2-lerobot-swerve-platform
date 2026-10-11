"""Cgroup priority tiers and unit template directives for the ROS2 systemd units."""

import subprocess
from pathlib import Path

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_DIR = REPO_ROOT / "ansible" / "roles" / "ros2_node_deploy" / "templates"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
SERVER_VARS = REPO_ROOT / "ansible" / "group_vars" / "server.yml"
RESOLVE_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "resolve_and_deploy.yml"

# node_type -> [cpu_quota, memory_max] of the client. Raised 2026-10-11 after 24 h peaks hit the cap: topic_scraper_api
# 128/128 (OOM), test_joint_api 58.2/64, bno055_imu 57.6/64, gps_rtk (rover) 55.8/64.
LIMITS_SNAPSHOT = {
    "ros2_master": ["20%", "128M"],
    "master2master": ["25%", "128M"],
    "feetech_servos": ["50%", "128M"],
    "uvc_camera": ["30%", "256M"],
    "filter_node": ["25%", "128M"],
    "test_joint_api": ["15%", "96M"],
    "topic_scraper_api": ["25%", "192M"],
    "bno055_imu": ["45%", "96M"],
    "haptic_controller": ["25%", "64M"],
    "gps_rtk": ["25%", "96M"],
    "rplidar_a1": ["30%", "128M"],
    "realsense_d435i": ["200%", "512M"],
    "overview_camera": ["100%", "256M"],
    "swerve_controller": ["30%", "128M"],
    "laser_filter": ["20%", "128M"],
    "static_tf_publisher": ["10%", "64M"],
    "rf2o_laser_odometry": ["25%", "128M"],
    "rf2o_odom_relay": ["10%", "64M"],
    "map_guard": ["20%", "128M"],
    "robot_localization_ekf": ["25%", "128M"],
    "nav2_bringup": ["75%", "512M"],
    "slam_toolbox": ["75%", "512M"],
    "web_ui": ["30%", "256M"],
    "mcp_server": ["25%", "384M"],
    "poi_store": ["10%", "128M"],
    "claude_agent": ["150%", "1G"],
}

CRITICAL = {
    "weights": (400, 400, -500),
    "types": ["feetech_servos", "swerve_controller", "ros2_master", "filter_node", "static_tf_publisher", "bno055_imu"],
}
NORMAL = {
    "weights": (100, None, 0),
    "types": [
        "rplidar_a1", "rf2o_laser_odometry", "rf2o_odom_relay", "robot_localization_ekf", "nav2_bringup",
        "slam_toolbox", "laser_filter", "gps_rtk", "poi_store", "overview_camera", "uvc_camera", "test_joint_api",
        "map_guard",
    ],
}  # fmt: skip
LOW = {
    "weights": (50, 50, 300),
    "types": ["web_ui", "mcp_server", "claude_agent", "topic_scraper_api", "master2master", "haptic_controller"],
}
TIERS = [CRITICAL, NORMAL, LOW]


def render(**variables: object) -> str:
    """Render the native unit template.

    Args:
        **variables (object): Template variables (node_name, node_cpu_weight, ...).

    Returns:
        str: The rendered unit file.
    """
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(TEMPLATE_DIR), keep_trailing_newline=True)
    return env.get_template("ros2-node-native.service.j2").render(node_name="x", ansible_user="u", **variables)


def type_defaults(path: Path) -> dict:
    """Return ros2_node_type_defaults of a group_vars file.

    Args:
        path (Path): group_vars YAML file.

    Returns:
        dict: node_type -> defaults.
    """
    return yaml.safe_load(path.read_text())["ros2_node_type_defaults"]


def test_template_omits_new_directives_when_unset() -> None:
    """Without the variables the unit has none of the new directives and keeps the quota and cap."""
    text = render(node_cpu_quota="20%", node_memory_max="64M")
    for key in ("CPUWeight", "IOWeight", "MemoryHigh", "OOMScoreAdjust"):
        assert key not in text
    assert "CPUQuota=20%" in text and "MemoryMax=64M" in text


def test_template_omits_directives_for_empty_strings() -> None:
    """resolve_and_deploy.yml passes an empty string for unset keys, which must not render."""
    text = render(node_cpu_weight="", node_io_weight="", node_memory_high="", node_oom_score_adjust="")
    for key in ("CPUWeight", "IOWeight", "MemoryHigh", "OOMScoreAdjust"):
        assert key not in text


def test_template_renders_directives_when_set() -> None:
    """Every directive appears with its value, a zero OOMScoreAdjust included."""
    text = render(node_cpu_weight=400, node_io_weight=50, node_memory_high="100M", node_oom_score_adjust=-500)
    assert "CPUWeight=400\n" in text
    assert "IOWeight=50\n" in text
    assert "MemoryHigh=100M\n" in text
    assert "OOMScoreAdjust=-500\n" in text
    assert "OOMScoreAdjust=0\n" in render(node_oom_score_adjust=0)


def test_directives_sit_in_the_service_section() -> None:
    """The directives are rendered between [Service] and [Install]."""
    text = render(node_cpu_weight=100, node_oom_score_adjust=0)
    assert text.index("[Service]") < text.index("CPUWeight=100") < text.index("[Install]")
    assert text.index("[Service]") < text.index("OOMScoreAdjust=0") < text.index("[Install]")


def test_resolve_maps_new_keys() -> None:
    """resolve_and_deploy.yml maps the type defaults keys to the node_* variables (memory_high is derived, see below)."""
    text = RESOLVE_TASKS.read_text()
    for var, key in (
        ("node_cpu_weight", "cpu_weight"),
        ("node_io_weight", "io_weight"),
        ("node_oom_score_adjust", "oom_score_adjust"),
    ):
        assert f"{var}: \"{{{{ ros2_node_type_defaults[_node.node_type].{key} | default('') }}}}\"" in text


def test_tiers_have_expected_weights() -> None:
    """Each client node type of a tier carries that tier's weights (io_weight absent for normal)."""
    defaults = type_defaults(CLIENT_VARS)
    for tier in TIERS:
        cpu, io, oom = tier["weights"]
        for node_type in tier["types"]:
            entry = defaults[node_type]
            assert entry.get("cpu_weight") == cpu, node_type
            assert entry.get("io_weight") == io, node_type
            assert entry.get("oom_score_adjust") == oom, node_type


def test_tiers_do_not_overlap() -> None:
    """A node type is in exactly one tier."""
    names = [t for tier in TIERS for t in tier["types"]]
    assert len(names) == len(set(names))


def test_untiered_types_have_no_weights() -> None:
    """Types outside the tiers (realsense_d435i, fastdds_discovery_server) are left alone."""
    tiered = {t for tier in TIERS for t in tier["types"]}
    for node_type, entry in type_defaults(CLIENT_VARS).items():
        if node_type not in tiered:
            for key in ("cpu_weight", "io_weight", "memory_high", "oom_score_adjust"):
                assert key not in entry, (node_type, key)


def test_existing_limits_unchanged() -> None:
    """cpu_quota and memory_max match the snapshot of main, and the git history agrees when available."""
    defaults = type_defaults(CLIENT_VARS)
    for node_type, (quota, memory) in LIMITS_SNAPSHOT.items():
        assert defaults[node_type].get("cpu_quota") == quota, node_type
        assert defaults[node_type].get("memory_max") == memory, node_type
    assert "cpu_quota" not in defaults["fastdds_discovery_server"]


def to_mib(size: str) -> int:
    """Convert a systemd size such as "96M" or "1G" to MiB.

    Args:
        size (str): Size with an M or G suffix.

    Returns:
        int: Size in MiB.
    """
    return int(size[:-1]) * (1024 if size.endswith("G") else 1)


def test_limits_unchanged_versus_main() -> None:
    """Against git show main:..., cpu_quota is equal and memory_max never shrinks (skipped as a no-op without main)."""
    proc = subprocess.run(
        ["git", "show", "main:ansible/group_vars/client.yml"], cwd=REPO_ROOT, capture_output=True, text=True
    )
    if proc.returncode != 0:
        return
    old = yaml.safe_load(proc.stdout)["ros2_node_type_defaults"]
    new = type_defaults(CLIENT_VARS)
    for node_type, entry in old.items():
        assert new[node_type].get("cpu_quota") == entry.get("cpu_quota"), node_type
        if entry.get("memory_max") is not None:
            assert to_mib(new[node_type]["memory_max"]) >= to_mib(entry["memory_max"]), node_type


def memory_high_of(node_type: str, defaults: dict) -> str:
    """Evaluate the node_memory_high expression of resolve_and_deploy.yml for one node type.

    Args:
        node_type (str): Key of ros2_node_type_defaults.
        defaults (dict): The ros2_node_type_defaults mapping.

    Returns:
        str: The rendered value, "" when none is derived.
    """
    vars_ = yaml.safe_load(RESOLVE_TASKS.read_text())[1]["vars"]
    env = jinja2.Environment()
    ctx: dict = {"ros2_node_type_defaults": defaults, "_node": {"node_type": node_type}}
    ctx["node_memory_max"] = env.from_string(vars_["node_memory_max"]).render(**ctx)
    return env.from_string(vars_["node_memory_high"]).render(**ctx).strip()


def test_memory_high_is_seven_eighths_of_memory_max() -> None:
    """Without memory_high the unit gets 7/8 of memory_max, M and G suffixes handled, a missing memory_max is 256M."""
    defaults = {
        "m": {"memory_max": "64M"},
        "m96": {"memory_max": "96M"},
        "g": {"memory_max": "1G"},
        "g2": {"memory_max": "2G"},
        "none": {},
        "odd": {"memory_max": "1000K"},
    }
    assert memory_high_of("m", defaults) == "56M"
    assert memory_high_of("m96", defaults) == "84M"
    assert memory_high_of("g", defaults) == "896M"
    assert memory_high_of("g2", defaults) == "1792M"
    assert memory_high_of("none", defaults) == "224M"
    assert memory_high_of("odd", defaults) == ""


def test_explicit_memory_high_wins() -> None:
    """A node type's memory_high is used as is."""
    defaults = {"x": {"memory_max": "256M", "memory_high": "100M"}}
    assert memory_high_of("x", defaults) == "100M"


def test_every_node_type_gets_memory_high_below_memory_max() -> None:
    """On both hosts every node type resolves a MemoryHigh strictly below its MemoryMax and the unit renders it."""
    for path in (CLIENT_VARS, SERVER_VARS):
        defaults = type_defaults(path)
        for node_type, entry in defaults.items():
            high = memory_high_of(node_type, defaults)
            top = (entry or {}).get("memory_max", "256M")
            assert high and to_mib(high) < to_mib(top), (path.name, node_type)
            text = render(node_memory_max=top, node_memory_high=high)
            assert f"MemoryHigh={high}\n" in text and f"MemoryMax={top}\n" in text


def test_scraper_and_sensor_caps_raised_on_both_hosts() -> None:
    """The scraper (client and server) has 192M, the three sensor units on the client 96M."""
    assert type_defaults(SERVER_VARS)["topic_scraper_api"]["memory_max"] == "192M"
    assert type_defaults(CLIENT_VARS)["topic_scraper_api"]["memory_max"] == "192M"
    for node_type in ("test_joint_api", "bno055_imu", "gps_rtk"):
        assert type_defaults(CLIENT_VARS)[node_type]["memory_max"] == "96M", node_type


def test_server_gets_no_tiers() -> None:
    """Only the robot (client) is tuned: the server's node types carry no cgroup tier keys."""
    for node_type, entry in type_defaults(SERVER_VARS).items():
        for key in ("cpu_weight", "io_weight", "memory_high", "oom_score_adjust"):
            assert key not in (entry or {}), (node_type, key)
