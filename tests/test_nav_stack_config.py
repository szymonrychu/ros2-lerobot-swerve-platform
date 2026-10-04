"""Static invariants of the mapping/navigation stack (lidar frame, EKF, slam_toolbox, Nav2, web_ui Map tab).

No ROS is available on the dev machine, so these tests read the repo files directly: YAML via
yaml.safe_load, launch files via ast. They pin the frame chain map -> odom -> base_link -> laser_frame,
the single odom->base_link publisher (EKF), the Ansible wiring and config overrides of the slam_toolbox node,
and the disabled-by-default collision_monitor StopBox.
"""

import ast
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
RPLIDAR_LAUNCH = REPO_ROOT / "nodes" / "bridges" / "rplidar_a1" / "launch" / "rplidar_a1.launch.py"
EKF_CONFIG = REPO_ROOT / "nodes" / "robot_localization_ekf" / "config" / "ekf.yaml"
EKF_LAUNCH = REPO_ROOT / "nodes" / "robot_localization_ekf" / "launch" / "ekf.launch.py"
SLAM_DIR = REPO_ROOT / "nodes" / "slam_toolbox"
SLAM_PARAMS = SLAM_DIR / "config" / "slam_params.yaml"
SLAM_LAUNCH = SLAM_DIR / "launch" / "slam.launch.py"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
# The lidar sees the robot body (returns 0.22-0.24 m from the sensor, inside the footprint); laser_filter removes the
# footprint box from /scan and every consumer (SLAM, costmaps, collision monitor) reads the filtered topic.
FILTERED_SCAN = "/scan_filtered"
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"

LASER_FRAME = "laser_frame"
MAPS_DIR = "/var/lib/ros2/maps"
MAP_BASE = "/var/lib/ros2/maps/slam_map"
SLAM_CONFIG_DIR = "/etc/ros2/slam_toolbox"
SLAM_CONFIG_FILE = "/etc/ros2/slam_toolbox/config.yaml"
LOCAL_PLAN_TOPIC = "/optimal_trajectory"
# Outer frame 470 x 386 mm, centred on base_link.
FOOTPRINT_HALF_X = 0.235
FOOTPRINT_HALF_Y = 0.193
# Planner (costmap) footprint: the outer frame minus the 5 cm worst-wheel margin on every side (370 x 286 mm). The
# self-filter and StopBox stay sized from the outer frame.
PLANNER_HALF_X = 0.185
PLANNER_HALF_Y = 0.143
# robot_localization 15-element config order.
EKF_INDEX = {
    "x": 0,
    "y": 1,
    "z": 2,
    "roll": 3,
    "pitch": 4,
    "yaw": 5,
    "vx": 6,
    "vy": 7,
    "vz": 8,
}
EKF_INDEX.update({"vroll": 9, "vpitch": 10, "vyaw": 11, "ax": 12, "ay": 13, "az": 14})
NAV2_SECTIONS = (
    "bt_navigator",
    "controller_server",
    "planner_server",
    "global_costmap",
    "local_costmap",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "collision_monitor",
    "docking_server",
    "waypoint_follower",
    "route_server",
)


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_entry(name: str) -> dict:
    """Return one ros2_nodes entry of the client by name.

    Args:
        name: Node name.

    Returns:
        dict: The ros2_nodes entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client node.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config YAML.
    """
    return yaml.safe_load(node_entry(name)["config"])


def module_constants(path: Path) -> dict[str, Any]:
    """Collect module-level `NAME = <literal>` assignments of a Python file.

    Args:
        path: Python source file.

    Returns:
        dict[str, Any]: Constant name to literal value (non-literal assignments are skipped).
    """
    constants: dict[str, Any] = {}
    for stmt in ast.parse(path.read_text()).body:
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1 and isinstance(stmt.targets[0], ast.Name):
            try:
                constants[stmt.targets[0].id] = ast.literal_eval(stmt.value)
            except ValueError:
                continue
    return constants


def load_function(path: Path, name: str, namespace: dict[str, Any]) -> Any:
    """Compile a single top-level function out of a launch file (which imports ROS) and return it.

    Args:
        path: Python source file.
        name: Function name.
        namespace: Globals the function needs (e.g. Path).

    Returns:
        Any: The function object.
    """
    tree = ast.parse(path.read_text())
    func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[func], type_ignores=[])
    exec(compile(module, str(path), "exec"), namespace)  # noqa: S102 - trusted repo source
    return namespace[name]


def ekf_params(doc: dict) -> dict:
    """Return the ekf_filter_node ros__parameters.

    Args:
        doc: Parsed EKF YAML.

    Returns:
        dict: Parameters.
    """
    return doc["ekf_filter_node"]["ros__parameters"]


def fused(config: list[bool]) -> set[str]:
    """Names of the fused state variables of a robot_localization sensor config.

    Args:
        config: 15-element boolean list.

    Returns:
        set[str]: Fused variable names.
    """
    assert len(config) == 15
    return {name for name, idx in EKF_INDEX.items() if config[idx]}


def nav2() -> dict:
    """Load the repo Nav2 params.

    Returns:
        dict: Parsed nav2_params.yaml.
    """
    return yaml.safe_load(NAV2_PARAMS.read_text())


def ros_params(doc: dict, section: str) -> dict:
    """ros__parameters of one Nav2 server (costmaps are nested one level deeper).

    Args:
        doc: Parsed nav2 params.
        section: Top-level key.

    Returns:
        dict: Parameters.
    """
    node = doc[section]
    if section in ("global_costmap", "local_costmap"):
        node = node[section]
    return node["ros__parameters"]


def expected_footprint() -> list[list[float]]:
    """Footprint polygon of the outer frame.

    Returns:
        list[list[float]]: Corner points.
    """
    x, y = FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y
    return [[x, y], [x, -y], [-x, -y], [-x, y]]


# --- Lidar frame ---------------------------------------------------------------------------------


def test_rplidar_default_frame_matches_static_tf_child() -> None:
    constants = module_constants(RPLIDAR_LAUNCH)
    assert constants["DEFAULT_FRAME_ID"] == LASER_FRAME
    assert constants["FRAME_ID_ENV"] == "RPLIDAR_FRAME_ID"
    source = RPLIDAR_LAUNCH.read_text()
    assert "os.environ.get(FRAME_ID_ENV, DEFAULT_FRAME_ID)" in source
    children = [f["child"] for f in node_config("static_tf_publisher")["frames"]]
    assert LASER_FRAME in children
    assert node_config("static_tf_publisher")["parent_frame"] == "base_link"


# --- EKF -----------------------------------------------------------------------------------------


def test_ekf_repo_config_matches_ansible_block() -> None:
    repo = yaml.safe_load(EKF_CONFIG.read_text())
    assert repo == node_config("robot_localization_ekf")


@pytest.mark.parametrize("source", ["repo", "ansible"])
def test_ekf_frames_inputs_and_tf(source: str) -> None:
    doc = yaml.safe_load(EKF_CONFIG.read_text()) if source == "repo" else node_config("robot_localization_ekf")
    p = ekf_params(doc)
    assert p["two_d_mode"] is True
    assert p["publish_tf"] is True
    assert p["world_frame"] == "odom"
    assert p["odom_frame"] == "odom"
    assert p["base_link_frame"] == "base_link"
    assert p["odom0"] == "/odom"
    assert fused(p["odom0_config"]) == {"vx", "vy", "vyaw"}
    assert p["odom0_differential"] is False
    assert p["odom0_relative"] is False
    assert p["imu0"] == "/imu/data"
    imu = fused(p["imu0_config"])
    assert "vyaw" in imu
    if "yaw" in imu:
        assert p["imu0_relative"] is True
    else:
        assert imu == {"vyaw"}


def test_ekf_launch_uses_repo_launch_and_deployed_config() -> None:
    defaults = client_vars()["ros2_node_type_defaults"]["robot_localization_ekf"]
    assert defaults["node_launch_command"] == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/robot_localization_ekf/launch/ekf.launch.py"
    )
    assert defaults["config_path"] == "/etc/ros2/robot_localization_ekf"
    assert "ROBOT_LOCALIZATION_EKF_CONFIG=/etc/ros2/robot_localization_ekf/config.yaml" in defaults["env"]
    constants = module_constants(EKF_LAUNCH)
    assert constants["DEFAULT_EKF_CONFIG_PATH"] == "/etc/ros2/robot_localization_ekf/config.yaml"
    assert "ROBOT_LOCALIZATION_EKF_CONFIG" in EKF_LAUNCH.read_text()


def test_swerve_controller_does_not_publish_odom_tf() -> None:
    assert node_config("swerve_controller")["publish_tf"] is False


# --- slam_toolbox --------------------------------------------------------------------------------


def test_slam_params_frames_and_topics() -> None:
    p = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    assert p["base_frame"] == "base_link"
    assert p["odom_frame"] == "odom"
    assert p["map_frame"] == "map"
    assert p["scan_topic"] == FILTERED_SCAN
    assert p["mode"] == "mapping"
    assert p["resolution"] == pytest.approx(0.05)
    assert "map_file_name" not in p, "posegraph loading is decided by the launch file"


def test_slam_launch_starts_async_lifecycle_node() -> None:
    source = SLAM_LAUNCH.read_text()
    ast.parse(source)
    assert "async_slam_toolbox_node" in source
    assert "LifecycleNode" in source
    assert "TRANSITION_CONFIGURE" in source and "TRANSITION_ACTIVATE" in source
    assert module_constants(SLAM_LAUNCH)["DEFAULT_MAP_BASE"] == MAP_BASE


def test_slam_launch_resumes_only_when_posegraph_exists(tmp_path: Path) -> None:
    resume = load_function(SLAM_LAUNCH, "map_resume_parameters", {"Path": Path})
    base = tmp_path / "slam_map"
    # Missing posegraph: map_file_name is blanked explicitly, so a map_file_name from a params file cannot make
    # slam_toolbox try to load a file that does not exist.
    assert resume(base) == {"map_file_name": ""}
    base.with_suffix(".posegraph").write_bytes(b"x")
    assert resume(base) == {"map_file_name": str(base), "map_start_at_dock": True}


def test_slam_launch_appends_override_params_only_when_present(tmp_path: Path) -> None:
    params_files = load_function(SLAM_LAUNCH, "params_files", {"Path": Path})
    defaults = tmp_path / "slam_params.yaml"
    defaults.write_text("slam_toolbox:\n  ros__parameters:\n    resolution: 0.05\n")
    override = tmp_path / "config.yaml"
    assert params_files(defaults, override) == [str(defaults)]
    override.write_text("")
    assert params_files(defaults, override) == [str(defaults)], "an empty deployed config is skipped"
    override.write_text("slam_toolbox:\n  ros__parameters:\n    max_laser_range: 8.0\n")
    # ROS params files: later wins, so the deployed overrides come after the repo defaults.
    assert params_files(defaults, override) == [str(defaults), str(override)]


def test_slam_launch_override_defaults_and_env() -> None:
    constants = module_constants(SLAM_LAUNCH)
    assert constants["CONFIG_ENV"] == "SLAM_TOOLBOX_CONFIG"
    assert constants["DEFAULT_CONFIG_PATH"] == SLAM_CONFIG_FILE
    source = SLAM_LAUNCH.read_text()
    assert "os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH)" in source


def test_slam_launch_map_base_from_configuration(tmp_path: Path) -> None:
    namespace = {"Path": Path, "yaml": yaml, **module_constants(SLAM_LAUNCH)}
    map_base = load_function(SLAM_LAUNCH, "configured_map_base", namespace)
    defaults = tmp_path / "slam_params.yaml"
    defaults.write_text("slam_toolbox:\n  ros__parameters:\n    resolution: 0.05\n")
    assert map_base([str(defaults)], None) == Path(MAP_BASE)
    override = tmp_path / "config.yaml"
    override.write_text("slam_toolbox:\n  ros__parameters:\n    map_file_name: /data/maps/garage\n")
    assert map_base([str(defaults), str(override)], None) == Path("/data/maps/garage")
    later = tmp_path / "later.yaml"
    later.write_text("/**:\n  ros__parameters:\n    map_file_name: /data/maps/yard\n")
    assert map_base([str(defaults), str(override), str(later)], None) == Path("/data/maps/yard"), "later file wins"
    # An explicit env override (SLAM_TOOLBOX_MAP_BASE) beats the params files.
    assert map_base([str(defaults), str(override)], "/tmp/env_map") == Path("/tmp/env_map")


def test_slam_toolbox_ansible_wiring() -> None:
    group = client_vars()
    defaults = group["ros2_node_type_defaults"]["slam_toolbox"]
    assert defaults["deploy_mode"] == "native"
    assert "ros-jazzy-slam-toolbox" in defaults["apt_packages"]
    assert defaults["node_launch_command"] == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/slam_toolbox/launch/slam.launch.py"
    )
    assert defaults["cpu_quota"] and defaults["memory_max"]
    assert defaults["config_path"] == SLAM_CONFIG_DIR
    assert f"SLAM_TOOLBOX_CONFIG={SLAM_CONFIG_FILE}" in defaults["env"]
    entry = node_entry("slam_toolbox")
    assert entry["node_type"] == "slam_toolbox"
    assert entry["present"] is True and entry["enabled"] is True


def test_slam_toolbox_ansible_config_overrides() -> None:
    doc = node_config("slam_toolbox")
    p = doc["slam_toolbox"]["ros__parameters"]
    assert p["map_file_name"] == MAP_BASE
    assert p["min_laser_range"] == pytest.approx(0.15)
    assert p["max_laser_range"] == pytest.approx(12.0)
    defaults = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    overridable = set(defaults) | {"map_file_name"}
    assert set(p) <= overridable, f"unknown slam_toolbox params in the Ansible block: {set(p) - overridable}"
    # The web UI saves the posegraph where slam_toolbox resumes it from.
    web_map = next(t for t in node_config("web_ui")["tabs"] if t["id"] == "map")
    assert web_map["map_save_path"] == p["map_file_name"]


def maps_dir_tasks(tasks: list[dict]) -> list[dict]:
    """Find tasks that create the SLAM maps directory, directly or via an included task file.

    Args:
        tasks: Playbook task list.

    Returns:
        list[dict]: ansible.builtin.file arguments that create MAPS_DIR.
    """
    found: list[dict] = []
    for task in tasks:
        file_args = task.get("ansible.builtin.file")
        if file_args and file_args.get("path") == MAPS_DIR:
            found.append(file_args)
        include = task.get("ansible.builtin.include_tasks")
        if isinstance(include, str) and "slam_maps_dir" in include:
            found += maps_dir_tasks(yaml.safe_load((PLAYBOOKS_DIR / "tasks" / "slam_maps_dir.yml").read_text()))
    return found


def deploys(tasks: list[dict], name: str) -> bool:
    """Whether a task list deploys a node via resolve_and_deploy.

    Args:
        tasks: Playbook task list.
        name: Node name.

    Returns:
        bool: True when a task includes resolve_and_deploy.yml for the node.
    """
    return any(
        "resolve_and_deploy.yml" in str(t.get("ansible.builtin.include_tasks", ""))
        and t.get("vars", {}).get("_deploy_node_name") == name
        for t in tasks
    )


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml", "nodes/client/slam_toolbox.yml"])
def test_slam_playbooks_deploy_node_and_create_maps_dir(playbook: str) -> None:
    plays = yaml.safe_load((PLAYBOOKS_DIR / playbook).read_text())
    tasks = [t for play in plays for t in play.get("pre_tasks", []) + play.get("tasks", [])]
    assert deploys(tasks, "slam_toolbox")
    created = maps_dir_tasks(tasks)
    assert created, f"{playbook}: no task creates {MAPS_DIR}"
    args = created[0]
    assert args["state"] == "directory"
    assert args["owner"] == "{{ ansible_user }}"
    assert args["mode"] == "0755"


def test_nav2_launch_passes_repo_params_without_localization() -> None:
    command = client_vars()["ros2_node_type_defaults"]["nav2_bringup"]["node_launch_command"]
    assert "params_file:={{ ros2_repo_dest }}/nodes/nav2_bringup/config/nav2_params.yaml" in command
    assert "use_localization:=False" in command
    assert "slam:=True" not in command


def test_nav2_has_every_server_section() -> None:
    doc = nav2()
    for section in NAV2_SECTIONS:
        assert "ros__parameters" in (doc[section][section] if "costmap" in section else doc[section]), section


def test_nav2_frames() -> None:
    doc = nav2()
    bt = ros_params(doc, "bt_navigator")
    assert (bt["global_frame"], bt["robot_base_frame"], bt["odom_topic"]) == (
        "map",
        "base_link",
        "/odometry/filtered",
    )
    g = ros_params(doc, "global_costmap")
    assert (g["global_frame"], g["robot_base_frame"]) == ("map", "base_link")
    loc = ros_params(doc, "local_costmap")
    assert (loc["global_frame"], loc["robot_base_frame"], loc["rolling_window"]) == (
        "odom",
        "base_link",
        True,
    )
    cm = ros_params(doc, "collision_monitor")
    assert (cm["base_frame_id"], cm["odom_frame_id"]) == ("base_link", "odom")
    assert (cm["cmd_vel_in_topic"], cm["cmd_vel_out_topic"]) == (
        "cmd_vel_smoothed",
        "cmd_vel",
    )
    beh = ros_params(doc, "behavior_server")
    assert (beh["local_frame"], beh["global_frame"], beh["robot_base_frame"]) == (
        "odom",
        "map",
        "base_link",
    )
    dock = ros_params(doc, "docking_server")
    assert dock["base_frame"] == "base_link"
    route = ros_params(doc, "route_server")
    assert route["base_frame"] == "base_link"
    assert route.get("graph_filepath", "") == ""


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_nav2_costmap_layers_and_footprint(costmap: str) -> None:
    p = ros_params(nav2(), costmap)
    x, y = PLANNER_HALF_X, PLANNER_HALF_Y
    assert yaml.safe_load(p["footprint"]) == [[x, y], [x, -y], [-x, -y], [-x, y]]
    assert "robot_radius" not in p
    assert "obstacle_layer" in p["plugins"] and "inflation_layer" in p["plugins"]
    obstacle = p["obstacle_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    sources = obstacle["observation_sources"].split()
    assert any(obstacle[s]["topic"] == FILTERED_SCAN and obstacle[s]["data_type"] == "LaserScan" for s in sources)
    assert p["inflation_layer"]["plugin"] == "nav2_costmap_2d::InflationLayer"
    if costmap == "global_costmap":
        assert p["plugins"][0] == "static_layer"
        static = p["static_layer"]
        assert static["plugin"] == "nav2_costmap_2d::StaticLayer"
        assert static["map_topic"] == "/map"
        assert static["map_subscribe_transient_local"] is True


def test_nav2_mppi_omni_with_swerve_limits() -> None:
    ctrl = ros_params(nav2(), "controller_server")
    follow = ctrl[ctrl["controller_plugins"][0]]
    assert follow["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert follow["motion_model"] == "Omni"
    assert follow["vx_max"] == pytest.approx(0.25)
    assert follow["vx_min"] == pytest.approx(-0.25)
    assert follow["vy_max"] == pytest.approx(0.25)
    assert follow["wz_max"] == pytest.approx(0.5)
    # visualize enables the nav_msgs/Path on /optimal_trajectory (the local plan for the web UI).
    assert follow["visualize"] is True
    assert ctrl["min_y_velocity_threshold"] < 0.25


def test_nav2_planner_navfn_allows_unknown() -> None:
    planner = ros_params(nav2(), "planner_server")
    plugin = planner[planner["planner_plugins"][0]]
    assert plugin["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert plugin["allow_unknown"] is True


def test_nav2_velocity_smoother_matches_swerve() -> None:
    vs = ros_params(nav2(), "velocity_smoother")
    assert vs["max_velocity"] == pytest.approx([0.25, 0.25, 0.5])
    assert vs["min_velocity"] == pytest.approx([-0.25, -0.25, -0.5])
    assert vs["odom_topic"] == "/odometry/filtered"


def test_nav2_collision_monitor_uses_scan() -> None:
    cm = ros_params(nav2(), "collision_monitor")
    assert cm["polygons"]
    for name in cm["polygons"]:
        polygon = cm[name]
        assert polygon["type"] in ("polygon", "circle")
        assert polygon["action_type"] in ("stop", "slowdown", "approach", "limit")
        if polygon["type"] == "polygon":
            assert len(yaml.safe_load(polygon["points"])) >= 3
        else:
            assert polygon["radius"] > 0
        assert isinstance(polygon["min_points"], int) and polygon["min_points"] > 0
    sources = cm["observation_sources"]
    assert any(
        cm[s]["type"] == "scan" and cm[s]["topic"] == FILTERED_SCAN and cm[s]["enabled"] is True for s in sources
    )


def test_nav2_collision_monitor_stopbox_enabled_on_filtered_scan() -> None:
    """StopBox (5 cm outside the footprint) is enabled: on /scan_filtered the robot body is removed and a 20 s
    stationary sample on the robot (2026-10-03) had 0 returns in the 5 cm band (min_points 4)."""
    cm = ros_params(nav2(), "collision_monitor")
    assert (cm["cmd_vel_in_topic"], cm["cmd_vel_out_topic"]) == ("cmd_vel_smoothed", "cmd_vel")
    assert cm["base_frame_id"] == "base_link" and cm["odom_frame_id"] == "odom"
    for key in ("state_topic", "transform_tolerance", "source_timeout", "stop_pub_timeout"):
        assert key in cm, key
    assert "StopBox" in cm["polygons"], "the polygon stays declared so collision_monitor configures"
    stop = cm["StopBox"]
    assert stop["enabled"] is True
    assert stop["action_type"] == "stop"
    assert stop["type"] == "polygon"
    assert stop["min_points"] >= 4


def test_nav2_readme_documents_collision_monitor_validation() -> None:
    text = NAV2_README.read_text()
    assert "StopBox" in text
    assert "enabled: true" in text
    assert "/scan_filtered" in text and "collision_monitor_state" in text


def test_nav2_docking_server_configures_without_docks() -> None:
    dock = ros_params(nav2(), "docking_server")
    assert dock["dock_plugins"]
    assert "docks" not in dock and "dock_database" not in dock


def test_nav2_readme_documents_plan_topics() -> None:
    text = NAV2_README.read_text()
    assert LOCAL_PLAN_TOPIC in text
    assert "/plan" in text


# --- web_ui --------------------------------------------------------------------------------------


def test_web_ui_has_map_nav_tab() -> None:
    tabs = node_config("web_ui")["tabs"]
    tab = next(t for t in tabs if t["id"] == "map")
    assert tab == {
        "id": "map",
        "type": "map_nav",
        "label": "Map",
        "map_topic": "/map",
        "global_plan_topic": "/plan",
        "local_plan_topic": LOCAL_PLAN_TOPIC,
        "goal_topic": "/goal_pose",
        "map_frame": "map",
        "base_frame": "base_link",
        "map_save_path": MAP_BASE,
        # Merged 3D view: local costmap, swerve base and the interactive arm on the client-local topics.
        "local_costmap_topic": "/local_costmap/costmap",
        "base_urdf": "robot.urdf",
        "base_joint_states_topic": "/swerve_drive/joint_states",
        "arm_urdf": "so101_arm.urdf",
        "arm_joint_states_topic": "/follower/joint_states",
        "arm_command_topic": "/filter/web_ui_joint_commands",
        # Home / Set home buttons call the std_srvs/Trigger services served by mcp_server.
        "arm_home_service": "/arm/home",
        "arm_set_home_service": "/arm/set_home",
    }


def test_web_ui_installs_slam_toolbox_for_its_service_imports() -> None:
    """web_ui imports slam_toolbox.srv at module top, so deploying it alone must install the package."""
    defaults = client_vars()["ros2_node_type_defaults"]["web_ui"]
    assert "ros-jazzy-slam-toolbox" in defaults.get("apt_packages", [])


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file (repo convention: update it whenever tests change)."""
    text = TESTS_README.read_text()
    section = text.split("### test_nav_stack_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"


def test_lidar_static_tf_mounted_backwards() -> None:
    """RPLidar A1 is mounted rotated 180 deg (bench test 2026-10-03: driving 9.3 cm forward shortened the 180 deg range
    by 9.4 cm), so base_link -> laser_frame carries yaw = pi; position from Platform dimensions.md."""
    import math

    entry = node_entry("static_tf_publisher")
    frames = yaml.safe_load(entry["config"])["frames"]
    laser = next(f for f in frames if f["child"] == LASER_FRAME)
    assert (laser["x"], laser["y"], laser["z"]) == (0.15, 0.04, 0.20)
    assert laser["yaw"] == pytest.approx(math.pi, abs=1e-4)


def test_laser_filter_box_covers_footprint() -> None:
    doc = yaml.safe_load(LASER_FILTER_PARAMS.read_text())
    filters = doc["scan_to_scan_filter_chain"]["ros__parameters"]
    box = next(f for f in filters.values() if f["type"] == "laser_filters/LaserScanBoxFilter")
    params = box["params"]
    assert params["box_frame"] == "base_link"
    assert params["invert"] is False
    xs = [x for x, _y in expected_footprint()]
    ys = [y for _x, y in expected_footprint()]
    assert params["min_x"] <= min(xs) and params["max_x"] >= max(xs)
    assert params["min_y"] <= min(ys) and params["max_y"] >= max(ys)
    assert params["max_x"] - max(xs) <= 0.05 and min(xs) - params["min_x"] <= 0.05, "margin stays small"
    assert params["min_z"] < 0 < params["max_z"]


def test_laser_filter_ansible_wiring() -> None:
    group = client_vars()
    defaults = group["ros2_node_type_defaults"]["laser_filter"]
    assert "ros-jazzy-laser-filters" in defaults["apt_packages"]
    cmd = defaults["node_launch_command"]
    assert "laser_filters scan_to_scan_filter_chain" in cmd
    assert "{{ ros2_repo_dest }}/nodes/laser_filter/config/footprint_filter.yaml" in cmd
    assert "scan:=/scan" in cmd and f"scan_filtered:={FILTERED_SCAN}" in cmd
    entry = node_entry("laser_filter")
    assert entry["present"] is True and entry["enabled"] is True
    assert (PLAYBOOKS_DIR / "nodes" / "client" / "laser_filter.yml").is_file()
    tasks = yaml.safe_load((PLAYBOOKS_DIR / "deploy_nodes_client.yml").read_text())[0]["tasks"]
    assert "laser_filter" in [t.get("vars", {}).get("_deploy_node_name") for t in tasks]


def test_node_apt_install_refreshes_stale_cache() -> None:
    """A stale apt index 404s on new packages (laser_filters deploy, 2026-10-03): refresh it, at most hourly."""
    tasks = yaml.safe_load((ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml").read_text())
    apt = [
        t["ansible.builtin.apt"]
        for block in tasks
        for t in block.get("block", [block])
        if isinstance(t, dict) and "ansible.builtin.apt" in t
    ]
    assert apt, "apt install task not found"
    assert apt[0]["update_cache"] is True and apt[0]["cache_valid_time"] == 3600


def test_nav2_goal_tolerance_tight_enough_for_swerve() -> None:
    """15 cm let a 0.4 m goal finish 13.6 cm short. 10 cm (not tighter): the rotation shim turns to the goal heading
    only while inside this tolerance and its check is not latched, so it needs margin for the small base_link shift
    of an in-place turn (at 8 cm a turn drifted out, MPPI took over and overshot 58 deg on the robot)."""
    checker = ros_params(nav2(), "controller_server")["general_goal_checker"]
    assert checker["xy_goal_tolerance"] == pytest.approx(0.10)
    assert checker["yaw_goal_tolerance"] == pytest.approx(0.15)


def test_rplidar_runs_under_scan_supervisor() -> None:
    """The A1 driver can wedge without publishing /scan; the supervisor restarts it through systemd."""
    defaults = client_vars()["ros2_node_type_defaults"]["rplidar_a1"]
    assert defaults["node_launch_command"] == "python3 {{ ros2_repo_dest }}/nodes/bridges/rplidar_a1/scan_supervisor.py"
    supervisor = (REPO_ROOT / "nodes" / "bridges" / "rplidar_a1" / "scan_supervisor.py").read_text()
    assert "rplidar_a1.launch.py" in supervisor and "EXIT_RESTART" in supervisor


def test_nav2_rotates_toward_path_first_and_keeps_front_leading() -> None:
    """The lidar is partly covered at the back and right: the robot turns to face the path at the start of a move
    (RotationShimController around MPPI) and MPPI prefers driving front-first (PathAngleCritic forward preference)."""
    ctrl = ros_params(nav2(), "controller_server")
    follow = ctrl[ctrl["controller_plugins"][0]]
    assert follow["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert 0.3 <= follow["angular_dist_threshold"] <= 1.0
    assert follow["rotate_to_heading_angular_vel"] <= follow["wz_max"]
    assert follow["rotate_to_goal_heading"] is True
    angle = follow["PathAngleCritic"]
    assert angle["enabled"] is True and angle["mode"] == 0
    # The shim alone turns to the goal heading; MPPI's GoalAngleCritic fought it near the goal.
    assert follow["GoalAngleCritic"]["enabled"] is False
    assert follow["GoalCritic"]["cost_weight"] >= 8.0


def test_web_ui_frontend_build_runs_at_lowest_priority() -> None:
    """npm ci / npm run build on the Pi overheated and froze it next to the running ROS stack (2026-10-03):
    run them in a cgroup capped to one core with low IO weight, at the lowest CPU (nice 19) and I/O (ionice idle
    class) priority."""
    tasks = yaml.safe_load((ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml").read_text())
    cmds = [
        t["ansible.builtin.command"]["cmd"]
        for block in tasks
        for t in block.get("block", [block])
        if isinstance(t, dict) and "npm" in str(t.get("ansible.builtin.command", {}).get("cmd", ""))
    ]
    assert len(cmds) == 2, cmds
    for cmd in cmds:
        # cgroup cap (one core of four, low IO weight) plus lowest CPU and IO scheduling priority.
        assert cmd.startswith("systemd-run --quiet --scope -p CPUQuota=100% -p IOWeight=10 "), cmd
        assert "nice -n 19 ionice -c 3 npm " in cmd, cmd


def test_restart_handler_skips_disabled_nodes() -> None:
    """A node deployed with enabled: false was stopped by the role and then restarted by the change handler
    (gripper_uvc_camera, 2026-10-03): the handler must not restart disabled nodes."""
    handlers = yaml.safe_load((ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "handlers" / "main.yml").read_text())
    restart = next(h for h in handlers if h["name"] == "Restart ROS2 node")
    conditions = restart["when"] if isinstance(restart["when"], list) else [restart["when"]]
    assert any("ros2_node_restart_enabled" in c for c in conditions), conditions
    tasks = yaml.safe_load((ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml").read_text())
    facts = next(
        t["ansible.builtin.set_fact"]
        for t in tasks
        if "ros2_node_restart_now" in str(t.get("ansible.builtin.set_fact"))
    )
    assert "node_enabled" in facts["ros2_node_restart_enabled"]


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_costmaps_add_no_margin_beyond_given_dimensions(costmap: str) -> None:
    """The planner footprint drops the 5 cm worst-wheel margin of the 470 x 386 mm outer dimensions: no footprint
    padding, and inflation only up to its circumscribed radius (needed for footprint collision costs) with a steep
    falloff."""
    import math

    p = ros_params(nav2(), costmap)
    assert p["footprint_padding"] == 0.0
    circumscribed = math.hypot(PLANNER_HALF_X, PLANNER_HALF_Y)
    inflation = p["inflation_layer"]
    assert circumscribed <= inflation["inflation_radius"] <= circumscribed + 0.01
    assert inflation["cost_scaling_factor"] >= 10.0


def test_safety_boxes_hug_the_given_dimensions() -> None:
    """Outer dimensions already include the wheel margin: the self-filter box is footprint + 1 cm and the
    collision-monitor StopBox is footprint + 2 cm, strictly outside the filter box so it can still see obstacles."""
    params = yaml.safe_load(LASER_FILTER_PARAMS.read_text())["scan_to_scan_filter_chain"]["ros__parameters"]
    box = next(f for f in params.values() if f["type"] == "laser_filters/LaserScanBoxFilter")["params"]
    assert (box["max_x"], box["max_y"]) == pytest.approx((0.245, 0.203))
    assert (box["min_x"], box["min_y"]) == pytest.approx((-0.245, -0.203))
    stop = ros_params(nav2(), "collision_monitor")["StopBox"]
    xs = [abs(x) for x, _ in yaml.safe_load(stop["points"])]
    ys = [abs(y) for _, y in yaml.safe_load(stop["points"])]
    assert max(xs) == pytest.approx(0.255) and max(ys) == pytest.approx(0.213)
    assert min(xs) > box["max_x"] and min(ys) > box["max_y"]
