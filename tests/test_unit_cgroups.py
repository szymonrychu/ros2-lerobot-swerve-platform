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

# node_type -> [cpu_quota, memory_max] on main before the tiers landed (None: key absent).
LIMITS_SNAPSHOT = {
    "ros2_master": ["20%", "128M"],
    "master2master": ["25%", "128M"],
    "feetech_servos": ["50%", "128M"],
    "uvc_camera": ["30%", "256M"],
    "filter_node": ["25%", "128M"],
    "test_joint_api": ["15%", "64M"],
    "topic_scraper_api": ["25%", "128M"],
    "bno055_imu": ["15%", "64M"],
    "haptic_controller": ["25%", "64M"],
    "gps_rtk": ["25%", "64M"],
    "rplidar_a1": ["30%", "128M"],
    "realsense_d435i": ["200%", "512M"],
    "overview_camera": ["100%", "256M"],
    "swerve_controller": ["30%", "128M"],
    "laser_filter": ["20%", "128M"],
    "static_tf_publisher": ["10%", "64M"],
    "rf2o_laser_odometry": ["25%", "128M"],
    "rf2o_odom_relay": ["10%", "64M"],
    "robot_localization_ekf": ["25%", "128M"],
    "nav2_bringup": ["75%", "512M"],
    "slam_toolbox": ["75%", "512M"],
    "web_ui": ["30%", "256M"],
    "mcp_server": ["25%", "256M"],
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
    """resolve_and_deploy.yml maps the four type defaults keys to the node_* variables."""
    text = RESOLVE_TASKS.read_text()
    for var, key in (
        ("node_cpu_weight", "cpu_weight"),
        ("node_io_weight", "io_weight"),
        ("node_memory_high", "memory_high"),
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


def test_limits_unchanged_versus_main() -> None:
    """Compare against git show main:... when the main ref exists (skipped as a no-op otherwise)."""
    proc = subprocess.run(
        ["git", "show", "main:ansible/group_vars/client.yml"], cwd=REPO_ROOT, capture_output=True, text=True
    )
    if proc.returncode != 0:
        return
    old = yaml.safe_load(proc.stdout)["ros2_node_type_defaults"]
    new = type_defaults(CLIENT_VARS)
    for node_type, entry in old.items():
        for key in ("cpu_quota", "memory_max"):
            assert new[node_type].get(key) == entry.get(key), (node_type, key)


def test_server_gets_no_tiers() -> None:
    """Only the robot (client) is tuned: the server's node types carry no cgroup tier keys."""
    for node_type, entry in type_defaults(SERVER_VARS).items():
        for key in ("cpu_weight", "io_weight", "memory_high", "oom_score_adjust"):
            assert key not in (entry or {}), (node_type, key)
