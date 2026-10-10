"""Grafana dashboards of the server stack (roles/monitoring/files/*.json): panels, metric contract and host scoping.

Every PromQL and LogQL query is also parsed by the real tools (promtool, Loki): tests/test_monitoring_real_binaries.py.
"""

import json
import re

import pytest

from tests.monitoring_helpers import DASHBOARDS_DIR

# Metric contract (shared with the cgroup-limits task) plus every node_* host metric.
CONTRACT_METRICS = {
    "container_cpu_usage_seconds_total",
    "container_memory_working_set_bytes",
    "container_memory_rss",
    "container_fs_reads_bytes_total",
    "container_fs_writes_bytes_total",
    "container_spec_memory_limit_bytes",
    "container_spec_cpu_quota",
    "container_spec_cpu_period",
    "rpi_throttled_flags",
    "rpi_throttled",
}

# PromQL functions, aggregations and keywords: identifiers in an expression that are not metric names.
PROMQL_WORDS = {
    "abs", "absent", "and", "avg", "avg_over_time", "bool", "by", "ceil", "clamp", "clamp_max", "clamp_min", "count",
    "count_over_time", "delta", "deriv", "floor", "group_left", "group_right", "ignoring", "increase", "irate",
    "label_replace", "last_over_time", "max", "max_over_time", "min", "min_over_time", "offset", "on", "or", "rate",
    "round", "scalar", "sort", "sort_desc", "sum", "sum_over_time", "time", "topk", "bottomk", "unless", "vector",
    "without", "e", "inf", "nan", "histogram_quantile", "changes", "resets", "timestamp", "group", "label_join",
    "present_over_time", "quantile_over_time", "sgn",
}  # fmt: skip
GROUPING_WORDS = {"by", "without", "on", "ignoring", "group_left", "group_right"}

# Node metric contract (exporters brief): every name our nodes export. Histograms add _bucket/_sum/_count.
NODE_CONTRACT_METRICS = {
    # lerobot_follower
    "servo_bus_read_failures_total", "servo_bus_up", "servo_present_load", "servo_temperature_celsius",
    "servo_voltage_volts", "servo_torque_enabled", "servo_set_register_failures_total", "servo_commands_total",
    # filter_node
    "filter_input_age_seconds", "filter_active_source", "filter_source_switches_total", "filter_loop_overruns_total",
    # swerve_controller
    "swerve_cmd_vel_age_seconds", "swerve_joint_states_stale_total", "swerve_slip_residual",
    "swerve_odom_published_total",
    # rf2o_odom_relay
    "relay_messages_total", "relay_dt_rejected_total",
    # bno055_imu
    "imu_read_errors_total", "imu_soft_restores_total", "imu_reinits_total", "imu_init_attempts_total",
    "imu_calibration_level", "imu_published_total", "imu_mode", "imu_seconds_since_publish",
    # gps_rtk_rover
    "gps_fix_quality", "gps_satellites", "gps_hdop", "gps_diff_age_seconds", "gps_ntrip_connected",
    "gps_ntrip_rx_bytes_total", "gps_ntrip_reconnects_total", "gps_serial_errors_total", "gps_fixes_published_total",
    # gripper_uvc_camera
    "camera_frames_published_total", "camera_frames_dropped_total", "camera_reopens_total",
    # rplidar_a1
    "lidar_scans_total", "lidar_scan_rate_hz", "lidar_scan_gap_seconds_max", "lidar_driver_restarts_total",
    # mcp_server
    "mcp_tool_calls_total", "mcp_motion_queue_depth", "mcp_motion_steps_total", "mcp_grasp_plans_total",
    "mcp_grasp_attempts_total", "mcp_gripper_effort", "mcp_grip_profile_uses_total", "mcp_floor_guard_slowdowns_total",
    "mcp_robot_events_total", "mcp_sample_age_seconds", "mcp_nav_goals_total",
    # poi_store
    "poi_commands_total", "poi_save_failures_total", "poi_count",
    # claude_agent
    "agent_busy", "agent_instructions_total", "agent_turns_total", "agent_tool_uses_total",
    "agent_seconds_since_activity", "agent_watchdog_fires_total", "agent_session_resets_total", "agent_tokens_total",
    "agent_cost_usd_total",
    # web_ui
    "webui_ws_clients", "webui_ws_disconnects_total", "webui_broadcaster_slow_total", "webui_topic_stale_total",
    "webui_http_requests_total", "webui_map_updates_total", "webui_map_age_seconds", "webui_robot_pose_ok",
    "webui_battery_cutoff_active",
    # test_joint_api, topic_scraper_api
    "jointapi_requests_total", "scraper_messages_total", "scraper_subscriptions",
}  # fmt: skip
NODE_CONTRACT_HISTOGRAMS = {
    "servo_cycle_duration_seconds", "swerve_loop_duration_seconds", "camera_encode_seconds",
    "mcp_tool_duration_seconds", "mcp_motion_tracking_error_rad", "mcp_nav_goal_duration_seconds",
    "agent_instruction_duration_seconds", "webui_broadcast_duration_seconds", "scraper_callback_seconds",
}  # fmt: skip
ALLOWED_METRICS = (
    CONTRACT_METRICS
    | NODE_CONTRACT_METRICS
    | {f"{h}_{suffix}" for h in NODE_CONTRACT_HISTOGRAMS for suffix in ("bucket", "sum", "count")}
    | {"up"}
)
ALLOWED_PREFIXES = ("node_", "container_", "rpi_throttled", "robot_node_")

REQUIRED_PANELS = [
    "Host CPU % by mode",
    "Load average",
    "Memory used / available",
    "CPU temperature",
    "Throttling flags",
    "Root disk usage",
    "Root disk IO",
    "Network rx/tx",
    "Per-unit CPU cores used vs CPUQuota",
    "Per-unit memory working set vs MemoryMax",
    "Top 10 units by CPU",
    "Monitoring stack overhead",
]

# Dashboard file -> (title, uid, required panel titles).
DASHBOARDS: dict[str, tuple[str, str, list[str]]] = {
    "robot-resources.json": ("Robot resources", "robot-resources", REQUIRED_PANELS),
    "robot-agent.json": (
        "Robot agent",
        "robot-agent",
        [
            "Agent busy",
            "Instructions by status",
            "Instruction duration p50/p95",
            "Turns and tool uses by tool",
            "MCP tool call rate",
            "MCP tool latency p50/p95",
            "MCP tool errors",
            "Motion queue depth",
            "Motion step statuses",
            "Robot events by severity",
            "Seconds since agent activity",
            "Tokens",
            "Cost",
            "claude_agent and mcp_server CPU",
            "claude_agent and mcp_server memory",
        ],
    ),
    "robot-slam.json": (
        "Robot SLAM",
        "robot-slam",
        [
            "SLAM stack unit CPU",
            "SLAM stack unit memory",
            "Lidar scan rate",
            "Lidar max scan gap",
            "Lidar driver restarts",
            "rf2o relay message rate",
            "rf2o relay rejections",
            "Swerve slip residual",
            "Swerve odom rate",
            "Map updates",
            "Map age",
            "Robot pose TF ok",
            "Nav goals by result",
            "Nav goal duration",
            "IMU publish health",
            "GPS fix quality",
            "GPS satellites",
        ],
    ),
    "robot-grasping.json": (
        "Robot grasping",
        "robot-grasping",
        [
            "Grasp plans by mode and outcome",
            "Grasp attempts by mode and result",
            "Grasp success ratio",
            "Gripper effort",
            "Grip profile uses",
            "Floor guard slowdowns",
            "Arm motion steps by status",
            "Tracking error p50/p95",
            "Servo load per joint",
            "Servo temperature per joint",
            "Servo torque enabled per joint",
            "Servo bus failures",
            "Grasp tool latency p50/p95",
        ],
    ),
    "robot-health.json": (
        "Robot health",
        "robot-health",
        [
            "Unit states",
            "Unit restarts in window",
            "Node up/down",
            "Time since node start",
            "Errors per node in window",
            "Throttling",
            "Temperature",
            "Disk pressure",
            "Memory pressure",
            "Error log lines per unit",
            "Recent error log lines",
        ],
    ),
}
# Dashboards with a host picker (both Pis have these metrics); the rest show robot-only metrics.
HOST_DASHBOARDS = {"robot-resources.json", "robot-health.json"}
ROBOT_ONLY_DASHBOARDS = set(DASHBOARDS) - HOST_DASHBOARDS
LOKI_PANELS = {"Error log lines per unit", "Recent error log lines"}
# journald priority keywords at error or worse (Alloy's level label).
ERROR_LEVELS = 'level=~"emerg|alert|crit|err"'
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SLAM_UNITS = [
    "slam_toolbox",
    "nav2_bringup",
    "robot_localization_ekf",
    "rf2o_laser_odometry",
    "laser_filter",
    "rplidar_a1",
]
GRASP_TOOLS = ["plan_grasp", "grasp_object", "release_object", "move_arm_cartesian"]
# Error counters the health dashboard must show per node.
HEALTH_ERROR_METRICS = {
    "servo_bus_read_failures_total", "servo_set_register_failures_total", "imu_read_errors_total", "imu_reinits_total",
    "gps_serial_errors_total", "gps_ntrip_reconnects_total", "camera_frames_dropped_total", "camera_reopens_total",
    "lidar_driver_restarts_total", "webui_ws_disconnects_total", "webui_topic_stale_total",
    "agent_watchdog_fires_total", "mcp_tool_calls_total",
}  # fmt: skip


def load_dashboard(name: str) -> dict:
    """Parse one dashboard JSON file of the role.

    Args:
        name (str): File name under files/.

    Returns:
        dict: Dashboard model.
    """
    return json.loads((DASHBOARDS_DIR / name).read_text())


def panel_list(dashboard: dict) -> list[dict]:
    """Return every panel of a dashboard, including the ones nested in rows.

    Args:
        dashboard (dict): Dashboard JSON model.

    Returns:
        list[dict]: Panels.
    """
    out: list[dict] = []
    for panel in dashboard["panels"]:
        out.append(panel)
        out.extend(panel.get("panels", []))
    return out


def is_loki(panel: dict) -> bool:
    """Whether a panel queries Loki.

    Args:
        panel (dict): Panel model.

    Returns:
        bool: True for the loki-robot datasource.
    """
    return panel.get("datasource", {}).get("uid") == "loki-robot"


def dashboard_exprs(dashboard: dict, loki: bool = False) -> list[tuple[str, str]]:
    """Return every (panel title, expression) of a dashboard for one query language.

    Args:
        dashboard (dict): Dashboard model.
        loki (bool): True for the LogQL (Loki) panels, False for the PromQL ones.

    Returns:
        list[tuple[str, str]]: Panel title and expression pairs.
    """
    return [
        (p["title"], t["expr"])
        for p in panel_list(dashboard)
        if p.get("type") != "row" and is_loki(p) == loki
        for t in p["targets"]
    ]


def panel_exprs(dashboard: dict, title: str) -> str:
    """Join the expressions of one panel.

    Args:
        dashboard (dict): Dashboard model.
        title (str): Panel title.

    Returns:
        str: All its expressions, newline separated.
    """
    return "\n".join(e for t, e in dashboard_exprs(dashboard) + dashboard_exprs(dashboard, loki=True) if t == title)


def skip_string(expr: str, i: int) -> int:
    """Index just past the string literal that starts at expr[i].

    Args:
        expr (str): Expression.
        i (int): Index of the opening quote.

    Returns:
        int: Index after the closing quote.
    """
    quote, i = expr[i], i + 1
    while i < len(expr) and expr[i] != quote:
        i += 2 if expr[i] == "\\" else 1
    return i + 1


def skip_balanced(expr: str, i: int, opening: str, closing: str) -> int:
    """Index just past the bracket group that starts at expr[i], skipping string literals.

    Args:
        expr (str): Expression.
        i (int): Index of the opening bracket.
        opening (str): Opening bracket.
        closing (str): Closing bracket.

    Returns:
        int: Index after the matching closing bracket.
    """
    depth = 0
    while i < len(expr):
        char = expr[i]
        if char in "\"'`":
            i = skip_string(expr, i)
            continue
        depth += char == opening
        depth -= char == closing
        i += 1
        if depth == 0:
            break
    return i


def vector_selectors(expr: str) -> list[tuple[str, str]]:
    """Find the vector selectors of a PromQL expression (Grafana variables allowed).

    Args:
        expr (str): PromQL expression.

    Returns:
        list[tuple[str, str]]: (metric name, label matcher text without braces) per selector.
    """
    out: list[tuple[str, str]] = []
    i = 0
    while i < len(expr):
        char = expr[i]
        if char in "\"'`":
            i = skip_string(expr, i)
        elif char == "[":
            i = skip_balanced(expr, i, "[", "]")
        elif char == "$":
            i = skip_balanced(expr, i + 1, "{", "}") if expr[i + 1 : i + 2] == "{" else i + 1
            while i < len(expr) and (expr[i].isalnum() or expr[i] == "_"):
                i += 1
        elif char.isdigit() or (char == "." and expr[i + 1 : i + 2].isdigit()):
            while i < len(expr) and (expr[i].isalnum() or expr[i] == "."):
                i += 1
        elif char.isalpha() or char in "_:":
            end = i
            while end < len(expr) and (expr[end].isalnum() or expr[end] in "_:"):
                end += 1
            word = expr[i:end]
            after = end
            while after < len(expr) and expr[after] == " ":
                after += 1
            following = expr[after : after + 1]
            if word in GROUPING_WORDS and following == "(":
                i = skip_balanced(expr, after, "(", ")")
            elif following == "(" or word.lower() in PROMQL_WORDS:
                i = end
            elif following == "{":
                close = skip_balanced(expr, after, "{", "}")
                out.append((word, expr[after + 1 : close - 1]))
                i = close
            else:
                out.append((word, ""))
                i = end
        else:
            i += 1
    return out


def metric_names(expr: str) -> set[str]:
    """Extract the metric names a PromQL expression references.

    Args:
        expr (str): PromQL expression (Grafana variables allowed).

    Returns:
        set[str]: Metric names.
    """
    return {name for name, _ in vector_selectors(expr)}


# --- helpers ------------------------------------------------------------------------------------------------------


def test_selector_helper() -> None:
    """Guard against vacuous passes: the scanner finds selectors and skips functions, labels, ranges and variables."""
    expr = 'topk(10, sum by (id) (rate(container_cpu_usage_seconds_total{id=~"/system.slice/.+"}[$__rate_interval])))'
    assert vector_selectors(expr) == [("container_cpu_usage_seconds_total", 'id=~"/system.slice/.+"')]
    assert metric_names("node_load1 / on() group_left count(node_cpu_seconds_total)") == {
        "node_load1",
        "node_cpu_seconds_total",
    }
    replaced = 'label_replace(up{host="$host"} > 0, "unit", "$1", "id", ".*/(.+)\\\\.service")'
    assert vector_selectors(replaced) == [("up", 'host="$host"')]
    assert vector_selectors("time() - max by (node) (robot_node_start_time_seconds)") == [
        ("robot_node_start_time_seconds", "")
    ]
    assert vector_selectors("histogram_quantile(0.95, sum by (le) (rate(x_bucket[5m]))) * 1e3") == [("x_bucket", "")]


# --- dashboard structure ------------------------------------------------------------------------------------------


def test_dashboard_set_is_exactly_the_five_files() -> None:
    """files/ holds exactly the five provisioned dashboards."""
    assert sorted(p.name for p in DASHBOARDS_DIR.glob("*.json")) == sorted(DASHBOARDS)


@pytest.mark.parametrize("name", sorted(DASHBOARDS))
def test_dashboard_is_valid_json_with_required_panels(name: str) -> None:
    """Each dashboard parses, has its title/uid and every required panel by title, with unique panel ids."""
    title, uid, panels = DASHBOARDS[name]
    dashboard = load_dashboard(name)
    assert dashboard["title"] == title
    assert dashboard["uid"] == uid
    titles = [p.get("title") for p in panel_list(dashboard)]
    for panel in panels:
        assert panel in titles, panel
    ids = [p["id"] for p in panel_list(dashboard)]
    assert len(ids) == len(set(ids)), "duplicate panel id"


@pytest.mark.parametrize("name", sorted(DASHBOARDS))
def test_dashboard_common_settings(name: str) -> None:
    """Refresh 5s, last 1h, tag robot, links to every robot dashboard; datasource prometheus-robot or loki-robot."""
    dashboard = load_dashboard(name)
    assert dashboard["refresh"] == "5s"
    assert dashboard["time"] == {"from": "now-1h", "to": "now"}
    assert "robot" in dashboard["tags"]
    links = [link for link in dashboard["links"] if link["type"] == "dashboards"]
    assert len(links) == 1 and links[0]["tags"] == ["robot"] and links[0]["keepTime"] is True
    for panel in panel_list(dashboard):
        if panel.get("type") == "row":
            continue
        uid = "loki-robot" if panel["title"] in LOKI_PANELS else "prometheus-robot"
        assert panel["datasource"]["uid"] == uid, panel["title"]
        assert panel["targets"], panel["title"]
        for target in panel["targets"]:
            assert target["datasource"]["uid"] == uid, panel["title"]


@pytest.mark.parametrize("name", sorted(DASHBOARDS))
def test_dashboard_queries_use_contract_metrics_only(name: str) -> None:
    """Every PromQL expression names only contract metrics, node_*, container_*, rpi_throttled*, robot_node_* or up."""
    for title, expr in dashboard_exprs(load_dashboard(name)):
        names = metric_names(expr)
        assert names, (title, expr)
        bad = {n for n in names if n not in ALLOWED_METRICS and not n.startswith(ALLOWED_PREFIXES)}
        assert not bad, (title, bad)


# --- host scoping -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(HOST_DASHBOARDS))
def test_host_dashboards_have_a_host_picker(name: str) -> None:
    """Robot resources and Robot health pick the host from label_values(up, host), default client."""
    variables = {v["name"]: v for v in load_dashboard(name)["templating"]["list"]}
    host = variables["host"]
    assert host["type"] == "query"
    assert host["datasource"]["uid"] == "prometheus-robot"
    assert host["query"]["query"] == "label_values(up, host)"
    assert host["current"]["value"] == "client"
    assert host.get("multi") is not True and host.get("includeAll") is not True


@pytest.mark.parametrize("name", sorted(DASHBOARDS))
def test_every_selector_is_scoped_to_a_host(name: str) -> None:
    """Mimir holds both hosts: host dashboards select host="$host", robot-only dashboards host="client"."""
    matcher = 'host="$host"' if name in HOST_DASHBOARDS else 'host="client"'
    checked = 0
    for title, expr in dashboard_exprs(load_dashboard(name)):
        for metric, labels in vector_selectors(expr):
            assert matcher in labels, (title, metric, expr)
            checked += 1
    assert checked >= 10


@pytest.mark.parametrize("name", sorted(ROBOT_ONLY_DASHBOARDS))
def test_robot_only_dashboards_have_no_host_picker(name: str) -> None:
    """The agent, SLAM and grasping dashboards are robot-only: no host variable."""
    assert not [v for v in load_dashboard(name)["templating"]["list"] if v["name"] == "host"]


# --- per-dashboard content ----------------------------------------------------------------------------------------


def test_resources_dashboard_still_covers_units_and_throttling() -> None:
    """Guard against vacuous passes on the resources dashboard; the stack units are the server's now."""
    dashboard = load_dashboard("robot-resources.json")
    seen = {n for _, e in dashboard_exprs(dashboard) for n in metric_names(e)}
    assert {"container_cpu_usage_seconds_total", "container_spec_cpu_quota", "rpi_throttled"} <= seen
    per_unit = panel_exprs(dashboard, "Per-unit CPU cores used vs CPUQuota")
    for unit in ("mimir", "loki", "grafana-server", "alloy"):
        assert unit in per_unit, unit
    assert "prometheus" not in per_unit


def test_agent_dashboard_queries() -> None:
    """Agent dashboard: histograms as p50/p95 quantiles, CPU/memory of claude_agent and mcp_server from cgroups."""
    dashboard = load_dashboard("robot-agent.json")
    duration = panel_exprs(dashboard, "Instruction duration p50/p95")
    assert "histogram_quantile(0.5" in duration and "histogram_quantile(0.95" in duration
    assert "agent_instruction_duration_seconds_bucket" in duration
    latency = panel_exprs(dashboard, "MCP tool latency p50/p95")
    assert "mcp_tool_duration_seconds_bucket" in latency and "by (le, tool)" in latency
    assert 'outcome="error"' in panel_exprs(dashboard, "MCP tool errors")
    assert "agent_tool_uses_total" in panel_exprs(dashboard, "Turns and tool uses by tool")
    assert "agent_tokens_total" in panel_exprs(dashboard, "Tokens")
    assert "agent_cost_usd_total" in panel_exprs(dashboard, "Cost")
    for title in ["claude_agent and mcp_server CPU", "claude_agent and mcp_server memory"]:
        expr = panel_exprs(dashboard, title)
        assert "container_" in expr and "ros2-" in expr and "claude_agent" in expr and "mcp_server" in expr, title


def test_slam_dashboard_queries() -> None:
    """SLAM dashboard: unit CPU/memory of the upstream SLAM stack from cgroups, GPS and IMU health from our nodes."""
    dashboard = load_dashboard("robot-slam.json")
    for title in ["SLAM stack unit CPU", "SLAM stack unit memory"]:
        expr = panel_exprs(dashboard, title)
        assert "container_" in expr and "ros2-" in expr, title
        for unit in SLAM_UNITS:
            assert unit in expr, (title, unit)
    assert "lidar_scan_gap_seconds_max" in panel_exprs(dashboard, "Lidar max scan gap")
    assert "relay_dt_rejected_total" in panel_exprs(dashboard, "rf2o relay rejections")
    assert "swerve_odom_published_total" in panel_exprs(dashboard, "Swerve odom rate")
    assert "webui_robot_pose_ok" in panel_exprs(dashboard, "Robot pose TF ok")
    assert "mcp_nav_goal_duration_seconds_bucket" in panel_exprs(dashboard, "Nav goal duration")
    assert "imu_published_total" in panel_exprs(dashboard, "IMU publish health")
    assert "gps_fix_quality" in panel_exprs(dashboard, "GPS fix quality")


def test_grasping_dashboard_queries() -> None:
    """Grasping dashboard: servo panels per arm joint, success ratio of attempts, grasp tool latencies."""
    dashboard = load_dashboard("robot-grasping.json")
    for title in ["Servo load per joint", "Servo temperature per joint", "Servo torque enabled per joint"]:
        expr = panel_exprs(dashboard, title)
        for joint in ARM_JOINTS:
            assert joint in expr, (title, joint)
    ratio = panel_exprs(dashboard, "Grasp success ratio")
    assert "mcp_grasp_attempts_total" in ratio and "/" in ratio
    bus = panel_exprs(dashboard, "Servo bus failures")
    assert "servo_bus_read_failures_total" in bus and "servo_set_register_failures_total" in bus
    tools = panel_exprs(dashboard, "Grasp tool latency p50/p95")
    for tool in GRASP_TOOLS:
        assert tool in tools, tool
    assert "mcp_motion_tracking_error_rad_bucket" in panel_exprs(dashboard, "Tracking error p50/p95")


def test_health_dashboard_queries() -> None:
    """Health dashboard: systemd unit states and restarts, node up/start time, every error counter, host pressure."""
    dashboard = load_dashboard("robot-health.json")
    states = panel_exprs(dashboard, "Unit states")
    for unit in ("ros2-", "alloy", "mimir", "loki", "grafana-server", "nginx"):
        assert unit in states, unit
    assert "node_systemd_unit_state" in states and "prometheus" not in states
    assert "node_systemd_service_restart_total" in panel_exprs(dashboard, "Unit restarts in window")
    up = panel_exprs(dashboard, "Node up/down")
    assert "up{" in up and 'job="ros2_nodes"' in up
    assert "robot_node_start_time_seconds" in panel_exprs(dashboard, "Time since node start")
    errors = metric_names(panel_exprs(dashboard, "Errors per node in window"))
    assert HEALTH_ERROR_METRICS <= errors, HEALTH_ERROR_METRICS - errors
    assert "rpi_throttled" in panel_exprs(dashboard, "Throttling")
    assert "node_filesystem_avail_bytes" in panel_exprs(dashboard, "Disk pressure")
    assert "node_memory_MemAvailable_bytes" in panel_exprs(dashboard, "Memory pressure")


def test_health_dashboard_loki_panels() -> None:
    """Error-or-worse journal lines of the picked host: a per-unit rate and the recent lines themselves."""
    dashboard = load_dashboard("robot-health.json")
    panels = {p["title"]: p for p in panel_list(dashboard)}
    rate = panels["Error log lines per unit"]
    assert rate["type"] == "timeseries"
    [rate_expr] = [t["expr"] for t in rate["targets"]]
    assert re.fullmatch(r"sum by \(unit\) \(rate\(\{[^}]*\}\[\$__auto\]\)\)", rate_expr), rate_expr
    logs = panels["Recent error log lines"]
    assert logs["type"] == "logs"
    [logs_expr] = [t["expr"] for t in logs["targets"]]
    for expr in (rate_expr, logs_expr):
        selector = re.search(r"\{([^}]*)\}", expr)
        assert selector, expr
        assert 'host="$host"' in selector.group(1) and ERROR_LEVELS in selector.group(1), expr
    assert logs_expr == "{" + selector.group(1) + "}"
    assert len(dashboard_exprs(dashboard, loki=True)) == 2
