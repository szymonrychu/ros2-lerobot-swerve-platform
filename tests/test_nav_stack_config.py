"""Static invariants of the mapping/navigation stack (lidar frame, EKF, slam_toolbox, Nav2, web_ui Map tab).

No ROS is available on the dev machine, so these tests read the repo files directly: YAML via
yaml.safe_load, launch files via ast. They pin the frame chain map -> odom -> base_link -> laser_frame,
the single odom->base_link publisher (EKF), and the Ansible wiring of the slam_toolbox node.
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
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"

LASER_FRAME = "laser_frame"
MAPS_DIR = "/var/lib/ros2/maps"
MAP_BASE = "/var/lib/ros2/maps/slam_map"
LOCAL_PLAN_TOPIC = "/optimal_trajectory"
# Outer frame 470 x 386 mm, centred on base_link.
FOOTPRINT_HALF_X = 0.235
FOOTPRINT_HALF_Y = 0.193
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
    assert p["scan_topic"] == "/scan"
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
    assert resume(base) == {}
    base.with_suffix(".posegraph").write_bytes(b"x")
    assert resume(base) == {"map_file_name": str(base), "map_start_at_dock": True}


def test_slam_toolbox_ansible_wiring() -> None:
    group = client_vars()
    defaults = group["ros2_node_type_defaults"]["slam_toolbox"]
    assert defaults["deploy_mode"] == "native"
    assert "ros-jazzy-slam-toolbox" in defaults["apt_packages"]
    assert defaults["node_launch_command"] == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/slam_toolbox/launch/slam.launch.py"
    )
    assert defaults["cpu_quota"] and defaults["memory_max"]
    names = [n["name"] for n in group["ros2_nodes"]]
    assert names[0] == "fastdds_discovery_server"
    assert names.index("slam_toolbox") > names.index("fastdds_discovery_server")
    entry = node_entry("slam_toolbox")
    assert entry["node_type"] == "slam_toolbox"
    assert entry["present"] is True and entry["enabled"] is True


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


def test_slam_deployed_after_discovery_server_in_client_playbook() -> None:
    tasks = yaml.safe_load((PLAYBOOKS_DIR / "deploy_nodes_client.yml").read_text())[0]["tasks"]
    order = [t.get("vars", {}).get("_deploy_node_name") for t in tasks]
    assert order.index("slam_toolbox") > order.index("fastdds_discovery_server")


# --- Nav2 ----------------------------------------------------------------------------------------


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
    assert yaml.safe_load(p["footprint"]) == expected_footprint()
    assert "robot_radius" not in p
    assert "obstacle_layer" in p["plugins"] and "inflation_layer" in p["plugins"]
    obstacle = p["obstacle_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    sources = obstacle["observation_sources"].split()
    assert any(obstacle[s]["topic"] == "/scan" and obstacle[s]["data_type"] == "LaserScan" for s in sources)
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
    assert follow["plugin"] == "nav2_mppi_controller::MPPIController"
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
    sources = cm["observation_sources"]
    assert any(cm[s]["type"] == "scan" and cm[s]["topic"] == "/scan" for s in sources)


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
    }


def test_web_ui_installs_slam_toolbox_for_its_service_imports() -> None:
    """web_ui imports slam_toolbox.srv at module top, so deploying it alone must install the package."""
    defaults = client_vars()["ros2_node_type_defaults"]["web_ui"]
    assert "ros-jazzy-slam-toolbox" in defaults.get("apt_packages", [])
