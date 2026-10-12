"""Cgroup model of the ROS2 systemd units: robot headroom guaranteed, background constrained.

Robot node units carry no CPUQuota and no MemoryHigh: priority comes from CPUWeight/IOWeight tiers, memory protection
from MemoryLow at about the measured working set, MemoryMax is a generous safety net, and Restart=always brings a unit
back after any exit. Background work (user.slice: ssh sessions and the deploy's uv sync; monitoring.slice) loses every
contention.
"""

import re
from pathlib import Path

import jinja2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
TEMPLATE_DIR = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
ALL_VARS = ANSIBLE_DIR / "group_vars" / "all.yml"
RESOLVE_TASKS = ANSIBLE_DIR / "playbooks" / "tasks" / "resolve_and_deploy.yml"
HEADROOM_TASKS = ANSIBLE_DIR / "playbooks" / "tasks" / "cgroup_headroom.yml"
HEADROOM_TEMPLATES = ANSIBLE_DIR / "playbooks" / "templates"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
MONITORING_DEFAULTS = ANSIBLE_DIR / "roles" / "monitoring_agent" / "defaults" / "main.yml"
NODE_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml"
NODES_DIR = REPO_ROOT / "nodes"
# A CPU limit stated in a node README: a literal CPUQuota value, the unit's cpu_quota key or an "N % CPU quota"
# (whitespace includes a line break, so a phrase wrapped across two README lines still matches).
STALE_CPU_QUOTA_RE = re.compile(r"CPUQuota=\d|`cpu_quota`|CPU\s+quota", re.IGNORECASE)
STOP_FOR_BUILD = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "stop_for_build.yml"

CRITICAL = {
    "weights": (1000, 1000, -900),
    "types": ["feetech_servos", "swerve_controller", "ros2_master", "filter_node", "static_tf_publisher", "bno055_imu"],
}
NORMAL = {
    "weights": (500, 500, -500),
    "types": [
        "rplidar_a1", "rf2o_laser_odometry", "rf2o_odom_relay", "robot_localization_ekf", "nav2_bringup",
        "slam_toolbox", "laser_filter", "gps_rtk", "poi_store", "overview_camera", "uvc_camera", "realsense_d435i",
        "test_joint_api", "map_guard", "costmap_zones", "arducam_tof",
    ],
}  # fmt: skip
AGENT = {
    "weights": (300, 300, -200),
    "types": [
        "mcp_server",
        "claude_agent",
        "web_ui",
        "topic_scraper_api",
        "master2master",
        "leader_bridge",
        "haptic_controller",
    ],
}
TIERS = [CRITICAL, NORMAL, AGENT]
ROBOT_TYPES = {t for tier in TIERS for t in tier["types"]}
# systemd's default CPUWeight/IOWeight of every unit and slice that sets none.
DEFAULT_WEIGHT = 100
# Client Pi 5 RAM; the OS, page cache and unprotected services keep everything above the summed MemoryLow.
CLIENT_RAM_MIB = 8 * 1024
MAX_MEMORY_LOW_SUM_MIB = 2560


def render(**variables: object) -> str:
    """Render the native unit template.

    Args:
        **variables (object): Template variables (node_name, node_cpu_weight, ...).

    Returns:
        str: The rendered unit file.
    """
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(TEMPLATE_DIR), keep_trailing_newline=True)
    return env.get_template("ros2-node-native.service.j2").render(node_name="x", ansible_user="u", **variables)


def render_headroom(name: str, **variables: object) -> str:
    """Render one of the host-level cgroup drop-in templates of playbooks/templates.

    Args:
        name (str): Template file name.
        **variables (object): Template variables.

    Returns:
        str: Rendered text.
    """
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(HEADROOM_TEMPLATES), keep_trailing_newline=True)
    return env.get_template(name).render(ansible_managed="managed", **variables)


def load(path: Path) -> dict:
    """Load a YAML file.

    Args:
        path (Path): YAML file.

    Returns:
        dict: Parsed document.
    """
    return yaml.safe_load(path.read_text())


def type_defaults(path: Path) -> dict:
    """Return ros2_node_type_defaults of a group_vars file.

    Args:
        path (Path): group_vars YAML file.

    Returns:
        dict: node_type -> defaults.
    """
    return load(path)["ros2_node_type_defaults"]


def to_mib(size: str) -> int:
    """Convert a systemd size such as "96M" or "1G" to MiB.

    Args:
        size (str): Size with an M or G suffix.

    Returns:
        int: Size in MiB.
    """
    return int(size[:-1]) * (1024 if size.endswith("G") else 1)


def resolved(node_type: str, defaults: dict) -> dict[str, str]:
    """Evaluate the node_* expressions of resolve_and_deploy.yml for one node type.

    Args:
        node_type (str): Key of ros2_node_type_defaults.
        defaults (dict): The ros2_node_type_defaults mapping.

    Returns:
        dict[str, str]: node_* variable -> rendered value (stripped).
    """
    vars_ = yaml.safe_load(RESOLVE_TASKS.read_text())[1]["vars"]
    env = jinja2.Environment()
    ctx: dict = {"ros2_node_type_defaults": defaults, "_node": {"node_type": node_type}}
    keys = ("node_cpu_quota", "node_memory_max", "node_memory_high", "node_memory_low", "node_cpu_weight")
    return {key: env.from_string(vars_[key]).render(**ctx).strip() for key in keys}


# --- unit template ----------------------------------------------------------------------------------------------------


def test_template_omits_optional_directives_when_unset() -> None:
    """Without the variables the unit has no CPUQuota, weights, MemoryHigh, MemoryLow or OOMScoreAdjust, and keeps the
    256M MemoryMax safety net."""
    text = render()
    for key in ("CPUQuota", "CPUWeight", "IOWeight", "MemoryHigh", "MemoryLow", "OOMScoreAdjust"):
        assert key not in text, key
    assert "MemoryMax=256M\n" in text


def test_template_omits_directives_for_empty_strings() -> None:
    """resolve_and_deploy.yml passes an empty string for unset keys, which must not render."""
    text = render(
        node_cpu_quota="",
        node_cpu_weight="",
        node_io_weight="",
        node_memory_high="",
        node_memory_low="",
        node_oom_score_adjust="",
    )
    for key in ("CPUQuota", "CPUWeight", "IOWeight", "MemoryHigh", "MemoryLow", "OOMScoreAdjust"):
        assert key not in text, key


def test_template_renders_directives_when_set() -> None:
    """Every directive appears with its value, a zero OOMScoreAdjust included."""
    text = render(
        node_cpu_quota="20%",
        node_memory_max="64M",
        node_cpu_weight=1000,
        node_io_weight=300,
        node_memory_high="100M",
        node_memory_low="48M",
        node_oom_score_adjust=-900,
    )
    for line in (
        "CPUQuota=20%",
        "MemoryMax=64M",
        "CPUWeight=1000",
        "IOWeight=300",
        "MemoryHigh=100M",
        "MemoryLow=48M",
        "OOMScoreAdjust=-900",
    ):
        assert f"{line}\n" in text, line
    assert "OOMScoreAdjust=0\n" in render(node_oom_score_adjust=0)


def test_directives_sit_in_the_service_section() -> None:
    """The directives are rendered between [Service] and [Install]."""
    text = render(node_cpu_weight=100, node_memory_low="48M", node_oom_score_adjust=0)
    for line in ("CPUWeight=100", "MemoryLow=48M", "OOMScoreAdjust=0"):
        assert text.index("[Service]") < text.index(line) < text.index("[Install]"), line


def test_unit_restarts_after_any_exit() -> None:
    """Restart=always with RestartSec=15: nav2_bringup's launch exited 0 after its container segfaulted (2026-10-11
    09:45) and stayed down under Restart=on-failure."""
    text = render()
    assert "Restart=always\n" in text and "RestartSec=15\n" in text
    assert "on-failure" not in text


def test_config_error_exit_is_not_restarted() -> None:
    """A config/token error exits 78 (os.EX_CONFIG); relaunching every 15 s cannot fix it and costs an rclpy import."""
    assert "RestartPreventExitStatus=78\n" in render()


def test_stopped_and_disabled_nodes_stay_stopped() -> None:
    """Restart=always never restarts a unit stopped by systemctl: the deploy stops nodes only that way (stop_for_build
    `systemctl stop`, uninstall and enabled: false `state: stopped`), never with kill, which would count as an exit."""
    assert "xargs systemctl stop" in STOP_FOR_BUILD.read_text()
    tasks = yaml.safe_load(NODE_TASKS.read_text())
    uninstall = next(t for t in tasks if t.get("name") == "Uninstall ROS2 node (present is false)")
    stop = next(t for t in uninstall["block"] if t.get("name") == "Stop ROS2 node service")
    assert stop["ansible.builtin.service"]["state"] == "stopped"
    enabled = next(t for t in tasks if t.get("name") == "Set service enabled state (native)")
    assert "'stopped'" in enabled["ansible.builtin.service"]["state"]
    for path in (NODE_TASKS, STOP_FOR_BUILD):
        assert "systemctl kill" not in path.read_text(), path.name


# --- resolve_and_deploy.yml -------------------------------------------------------------------------------------------


def test_resolve_maps_cgroup_keys() -> None:
    """resolve_and_deploy.yml maps each type key straight to its node_* variable, empty when unset."""
    text = RESOLVE_TASKS.read_text()
    for var, key in (
        ("node_cpu_quota", "cpu_quota"),
        ("node_cpu_weight", "cpu_weight"),
        ("node_io_weight", "io_weight"),
        ("node_memory_high", "memory_high"),
        ("node_memory_low", "memory_low"),
        ("node_oom_score_adjust", "oom_score_adjust"),
    ):
        assert f"{var}: \"{{{{ ros2_node_type_defaults[_node.node_type].{key} | default('') }}}}\"" in text, var


def test_no_memory_high_is_derived() -> None:
    """MemoryHigh is no longer derived from MemoryMax: a unit without memory_high gets none (no reclaim throttling)."""
    for path in (CLIENT_VARS, SERVER_VARS):
        defaults = type_defaults(path)
        for node_type in defaults:
            assert resolved(node_type, defaults)["node_memory_high"] == "", (path.name, node_type)
    assert resolved("x", {"x": {"memory_max": "1G"}})["node_memory_high"] == ""
    assert resolved("x", {"x": {"memory_high": "100M"}})["node_memory_high"] == "100M"


def test_unset_cpu_quota_resolves_empty() -> None:
    """A type without cpu_quota resolves an empty node_cpu_quota (no CPUQuota line), memory_max defaults to 256M."""
    values = resolved("x", {"x": {}})
    assert values["node_cpu_quota"] == "" and values["node_memory_max"] == "256M" and values["node_memory_low"] == ""
    assert "CPUQuota" not in render(node_cpu_quota=values["node_cpu_quota"])


# --- client tiers -----------------------------------------------------------------------------------------------------


def test_robot_types_carry_no_cpu_quota_or_memory_high() -> None:
    """Every robot (client) node type has no cpu_quota and no memory_high."""
    for node_type, entry in type_defaults(CLIENT_VARS).items():
        assert "cpu_quota" not in entry, node_type
        assert "memory_high" not in entry, node_type


def test_tiers_have_expected_weights() -> None:
    """Each client node type of a tier carries that tier's cpu_weight, io_weight and oom_score_adjust."""
    defaults = type_defaults(CLIENT_VARS)
    for tier in TIERS:
        cpu, io, oom = tier["weights"]
        for node_type in tier["types"]:
            entry = defaults[node_type]
            assert (entry.get("cpu_weight"), entry.get("io_weight"), entry.get("oom_score_adjust")) == (cpu, io, oom), (
                node_type
            )


def test_tiers_dominate_default_system_work() -> None:
    """Every robot tier outweighs the default weight 100, critical > normal > agent, and every OOMScoreAdjust is
    negative with critical the most negative."""
    weights = [tier["weights"] for tier in TIERS]
    assert all(cpu > DEFAULT_WEIGHT and io > DEFAULT_WEIGHT and oom < 0 for cpu, io, oom in weights)
    assert weights[0][0] > weights[1][0] > weights[2][0]
    assert weights[0][2] < weights[1][2] < weights[2][2]


def test_tiers_do_not_overlap() -> None:
    """A node type is in exactly one tier."""
    names = [t for tier in TIERS for t in tier["types"]]
    assert len(names) == len(set(names))


def test_every_client_type_is_tiered() -> None:
    """Every client node type except the uninstalled fastdds_discovery_server is in a tier."""
    assert set(type_defaults(CLIENT_VARS)) - ROBOT_TYPES == {"fastdds_discovery_server"}
    entry = type_defaults(CLIENT_VARS)["fastdds_discovery_server"]
    for key in ("cpu_weight", "io_weight", "memory_low", "memory_max", "oom_score_adjust"):
        assert key not in entry, key


# --- client memory ----------------------------------------------------------------------------------------------------


def test_every_robot_type_has_memory_low_below_memory_max() -> None:
    """Every robot node type protects some memory and its MemoryMax safety net is at least twice MemoryLow; mcp_server
    gets at least 1G."""
    defaults = type_defaults(CLIENT_VARS)
    for node_type in ROBOT_TYPES:
        low = to_mib(defaults[node_type]["memory_low"])
        assert low >= 32, node_type
        assert to_mib(defaults[node_type]["memory_max"]) >= 2 * low, node_type
    assert to_mib(defaults["mcp_server"]["memory_max"]) >= 1024


def enabled_memory_low_sum(variables: dict) -> int:
    """Sum memory_low (MiB) over the present and enabled nodes of a group_vars document.

    Args:
        variables (dict): group_vars document with ros2_nodes and ros2_node_type_defaults.

    Returns:
        int: Sum in MiB.
    """
    defaults = variables["ros2_node_type_defaults"]
    return sum(
        to_mib(defaults[node["node_type"]].get("memory_low", "0M"))
        for node in variables["ros2_nodes"]
        if node.get("present", True) and node.get("enabled", True)
    )


def test_memory_low_sum_leaves_room_for_the_os() -> None:
    """The enabled robot nodes protect at most MAX_MEMORY_LOW_SUM_MIB in total, well under a third of the 8 GB Pi."""
    total = enabled_memory_low_sum(load(CLIENT_VARS))
    assert total <= MAX_MEMORY_LOW_SUM_MIB < CLIENT_RAM_MIB / 3


# --- server -----------------------------------------------------------------------------------------------------------


def test_server_keeps_its_limits_without_tiers() -> None:
    """The server shares the template but keeps its quotas and caps: no tier keys, no memory_low, no MemoryHigh."""
    defaults = type_defaults(SERVER_VARS)
    assert defaults["topic_scraper_api"]["memory_max"] == "192M"
    for node_type, entry in defaults.items():
        for key in ("cpu_weight", "io_weight", "memory_high", "memory_low", "oom_score_adjust"):
            assert key not in (entry or {}), (node_type, key)
        if node_type != "fastdds_discovery_server":
            values = resolved(node_type, defaults)
            text = render(
                node_cpu_quota=values["node_cpu_quota"],
                node_memory_max=values["node_memory_max"],
                node_memory_high=values["node_memory_high"],
                node_memory_low=values["node_memory_low"],
            )
            assert f"CPUQuota={entry['cpu_quota']}\n" in text and f"MemoryMax={entry['memory_max']}\n" in text
            assert "MemoryHigh" not in text and "MemoryLow" not in text


# --- host-level slices (background constrained) ----------------------------------------------------------------------


def test_system_slice_allocates_the_robot_memory_low() -> None:
    """A unit's MemoryLow only protects as far as its ancestors allocate: system.slice gets the sum of the enabled
    nodes' memory_low."""
    variables = load(CLIENT_VARS)
    text = render_headroom("system-slice-headroom.conf.j2", **variables)
    assert "[Slice]\n" in text
    assert f"MemoryLow={enabled_memory_low_sum(variables)}M\n" in text
    assert "MemoryLow=0M\n" in render_headroom(
        "system-slice-headroom.conf.j2", ros2_nodes=[], ros2_node_type_defaults={}
    )
    nodes = [
        {"node_type": "a"},
        {"node_type": "b", "enabled": False},
        {"node_type": "c", "present": False},
        {"node_type": "d"},
    ]
    types = {"a": {"memory_low": "1G"}, "b": {"memory_low": "64M"}, "c": {"memory_low": "64M"}, "d": {}}
    assert "MemoryLow=1024M\n" in render_headroom(
        "system-slice-headroom.conf.j2", ros2_nodes=nodes, ros2_node_type_defaults=types
    )


def test_user_slice_loses_to_robot_units() -> None:
    """user.slice (ssh sessions, the deploy's become steps such as uv sync) gets CPUWeight/IOWeight 50, below the
    system.slice default 100 that holds the robot units."""
    variables = load(CLIENT_VARS)
    assert variables["ros2_user_slice_cpu_weight"] == 50
    assert variables["ros2_user_slice_io_weight"] == 50
    text = render_headroom("user-slice-headroom.conf.j2", **variables)
    assert "[Slice]\nCPUWeight=50\nIOWeight=50\n" in text


def test_headroom_tasks_are_reversible_and_wired_into_the_client_deploy() -> None:
    """cgroup_headroom.yml installs both drop-ins when ros2_cgroup_headroom_enabled, removes them otherwise, reloads
    systemd on a change, and the client deploy runs it on every run (tags config, always)."""
    assert load(CLIENT_VARS)["ros2_cgroup_headroom_enabled"] is True
    tasks = yaml.safe_load(HEADROOM_TASKS.read_text())
    text = HEADROOM_TASKS.read_text()
    install = next(t for t in tasks if "ansible.builtin.template" in t)
    assert (
        install["ansible.builtin.template"]["dest"] == "/etc/systemd/system/{{ item.slice }}.d/50-robot-headroom.conf"
    )
    assert [item["slice"] for item in install["loop"]] == ["system.slice", "user.slice"]
    assert "state: absent" in text and "ros2_cgroup_headroom_enabled" in text
    assert any("daemon_reload" in str(t) for t in tasks)
    playbook = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    include = next(t for t in playbook["pre_tasks"] if "cgroup_headroom.yml" in str(t))
    assert set(include["tags"]) >= {"config", "always"}


def test_monitoring_slice_stays_low_priority() -> None:
    """monitoring.slice keeps a low CPU/IO weight and its memory caps; its units are the OOM killer's first pick."""
    defaults = load(MONITORING_DEFAULTS)
    assert defaults["monitoring_agent_slice_cpu_weight"] == 20
    assert defaults["monitoring_agent_slice_io_weight"] == 20
    assert defaults["monitoring_agent_slice_memory_max"] and defaults["monitoring_agent_slice_memory_high"]
    assert defaults["monitoring_agent_oom_score_adjust"] > 0


def test_builds_run_at_background_cpu_weight() -> None:
    """Source and frontend builds run in a transient scope with ros2_build_cpu_weight (20), below every robot tier."""
    assert load(ALL_VARS)["ros2_build_cpu_weight"] == 20
    paths = (NODE_TASKS, ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml")
    text = " ".join(" ".join(path.read_text().split()) for path in paths)
    prefix = (
        "systemd-run --quiet --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p CPUWeight={{ ros2_build_cpu_weight }} "
    )
    assert text.count("systemd-run") == text.count(prefix) == 3


def test_node_readmes_state_no_cpu_quota() -> None:
    """Node READMEs describe the weight model: no robot unit CPU quota, no literal build quota (name the variable)."""
    stale = [
        f"{path.relative_to(REPO_ROOT)}:{text.count(chr(10), 0, match.start()) + 1}: {match.group(0)!r}"
        for path in sorted(NODES_DIR.rglob("README.md"))
        if "node_modules" not in path.parts
        for text in [path.read_text()]
        for match in STALE_CPU_QUOTA_RE.finditer(text)
    ]
    assert stale == []
