"""Static invariants of the mapping/navigation stack (lidar frame, EKF, slam_toolbox, Nav2, web_ui Map tab).

No ROS is available on the dev machine, so these tests read the repo files directly: YAML via
yaml.safe_load, launch files via ast. They pin the frame chain map -> odom -> base_link -> laser_frame,
the single odom->base_link publisher (EKF), the Ansible wiring and config overrides of the slam_toolbox node,
and the disabled-by-default collision_monitor StopBox.
"""

import ast
import json
import math
import re
import tomllib
from pathlib import Path
from typing import Any

import pytest
import yaml
from shapely.geometry import Point, Polygon

from scripts.gen_nav_footprint import DEFAULT_CONFIG as FOOTPRINT_CONFIG
from scripts.gen_nav_footprint import Generated, circumscribed_radius, generate, load_settings

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
COSTMAP_ZONES_CONFIG = REPO_ROOT / "nodes" / "costmap_zones" / "costmap_zones" / "config.py"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
# The lidar sees the robot body (returns 0.22-0.24 m from the sensor, inside the footprint); laser_filter removes the
# footprint box from /scan and every consumer (SLAM, costmaps, collision monitor) reads the filtered topic.
FILTERED_SCAN = "/scan_filtered"
RF2O_FUSED_TOPIC = "/odom_rf2o_twist"
RF2O_PARAMS = REPO_ROOT / "nodes" / "rf2o_laser_odometry" / "config" / "rf2o.yaml"
NAV2_LAUNCH = REPO_ROOT / "nodes" / "nav2_bringup" / "launch" / "nav2_launch.py"
NAV2_NAVIGATION_LAUNCH = REPO_ROOT / "nodes" / "nav2_bringup" / "launch" / "navigation_launch.py"
# Servers dropped from the navigation lifecycle: nothing in the repo calls them (the grasp docking server is separate).
DROPPED_NAV2_SERVERS = {"route_server", "waypoint_follower", "docking_server"}
KEPT_NAV2_SERVERS = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
]
GRASP_DOCKING_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "grasp_docking_params.yaml"
NAV2_README = REPO_ROOT / "nodes" / "nav2_bringup" / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NODES_README = REPO_ROOT / "nodes" / "README.md"

LASER_FRAME = "laser_frame"
MAPS_DIR = "/var/lib/ros2/maps"
MAP_BASE = "/var/lib/ros2/maps/slam_map"
SLAM_CONFIG_DIR = "/etc/ros2/slam_toolbox"
SLAM_CONFIG_FILE = "/etc/ros2/slam_toolbox/config.yaml"
LOCAL_PLAN_TOPIC = "/optimal_trajectory"
# Map and costmap cell size (m): 2.5 cm SLAM map, both costmaps and the Smac lattice primitives match it.
MAP_RESOLUTION = 0.025
ALL_VARS = ANSIBLE_DIR / "group_vars" / "all.yml"
LATTICE_DIR = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "lattice"
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


def footprint_geometry() -> Generated:
    """The H footprint, inflation, StopBox and laser polygon from scripts/gen_nav_footprint.py.

    Returns:
        Generated: Generator output for the repo config.
    """
    return generate(load_settings(FOOTPRINT_CONFIG), REPO_ROOT)


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
    assert p["resolution"] == pytest.approx(MAP_RESOLUTION)
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
    assert "cpu_quota" not in defaults and defaults["memory_low"] and defaults["memory_max"]
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
        if "slam_maps_dir" in str(include):
            found += maps_dir_tasks(yaml.safe_load((PLAYBOOKS_DIR / "tasks" / "slam_maps_dir.yml").read_text()))
    return found


def flat(tasks: list[dict]) -> list[dict]:
    """Expand the deploy playbook's single block into its tasks.

    Args:
        tasks: A play's task list.

    Returns:
        list[dict]: The tasks, with block children in place of the block.
    """
    return [c for t in tasks for c in (t["block"] if "block" in t else [t])]


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


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml"])
def test_slam_playbooks_deploy_node_and_create_maps_dir(playbook: str) -> None:
    plays = yaml.safe_load((PLAYBOOKS_DIR / playbook).read_text())
    tasks = [t for play in plays for t in play.get("pre_tasks", []) + flat(play.get("tasks", []))]
    assert deploys(tasks, "slam_toolbox")
    created = maps_dir_tasks(tasks)
    assert created, f"{playbook}: no task creates {MAPS_DIR}"
    args = created[0]
    assert args["state"] == "directory"
    assert args["owner"] == "{{ ansible_user }}"
    assert args["mode"] == "0755"


def test_nav2_launch_uses_the_repo_wrapper() -> None:
    """Ansible starts nav2 through the repo wrapper (stock bringup plus the grasp docking server)."""
    defaults = client_vars()["ros2_node_type_defaults"]["nav2_bringup"]
    launch = "{{ ros2_repo_dest }}/nodes/nav2_bringup/launch/nav2_launch.py"
    assert defaults["node_launch_command"] == f"ros2 launch {launch}"
    assert "ros-jazzy-opennav-docking" in defaults["apt_packages"]


def launch_calls(name: str) -> list[ast.Call]:
    """Return every call to a function or class named `name` in the nav2 wrapper launch file.

    Args:
        name: Simple (unqualified) callee name.

    Returns:
        list[ast.Call]: The matching call nodes.
    """
    tree = ast.parse(NAV2_LAUNCH.read_text())
    return [
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.Call) and (getattr(n.func, "id", None) == name or getattr(n.func, "attr", None) == name)
    ]


def call_kwarg(call: ast.Call, key: str) -> Any:
    """Literal value of a keyword argument of a call (ast.literal_eval).

    Args:
        call: The call node.
        key: Keyword name.

    Returns:
        Any: The evaluated literal.
    """
    kw = next(k for k in call.keywords if k.arg == key)
    return ast.literal_eval(kw.value)


def test_nav2_launch_includes_repo_navigation_launch_without_localization() -> None:
    """The wrapper includes the repo navigation_launch.py, not bringup_launch.py (which pulls in the stock one)."""
    source = NAV2_LAUNCH.read_text()
    assert "bringup_launch.py" not in source
    assert "navigation_launch.py" in source
    assert "nav2_params.yaml" in source
    includes = launch_calls("IncludeLaunchDescription")
    assert len(includes) == 1
    args = next(k for k in includes[0].keywords if k.arg == "launch_arguments").value
    assert "params_file" in ast.unparse(args)
    assert "use_composition" in ast.unparse(args)


def navigation_lifecycle_nodes() -> list[str]:
    """The `lifecycle_nodes` list literal of the repo navigation_launch.py.

    Returns:
        list[str]: Server names the navigation lifecycle manager manages.
    """
    tree = ast.parse(NAV2_NAVIGATION_LAUNCH.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "lifecycle_nodes" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("lifecycle_nodes not assigned in navigation_launch.py")


def test_lifecycle_nodes_exclude_unused_servers() -> None:
    nodes = navigation_lifecycle_nodes()
    assert set(nodes).isdisjoint(DROPPED_NAV2_SERVERS)
    assert nodes == KEPT_NAV2_SERVERS
    source = NAV2_NAVIGATION_LAUNCH.read_text()
    for server in DROPPED_NAV2_SERVERS:
        assert f"name='{server}'" not in source and f'name="{server}"' not in source


def test_nothing_in_the_repo_calls_the_dropped_servers() -> None:
    """The stock docking_server, waypoint_follower and route_server have no client (grasp uses /grasp/dock_robot)."""
    pattern = re.compile(r"FollowWaypoints|NavigateThroughPoses|ComputeRoute|set_route_graph|(?<!grasp)/dock_robot")
    for path in (REPO_ROOT / "nodes").rglob("*.py"):
        if ".venv" in path.parts or "tests" in path.parts:
            continue
        for number, line in enumerate(path.read_text().splitlines(), start=1):
            assert not pattern.search(line), f"{path}:{number} uses a dropped server"


def test_lifecycle_bond_timeout_is_zero() -> None:
    """bond_timeout 0.0 makes the lifecycle manager skip bond creation (no ~95 Hz bond traffic, no bond-miss reset)."""
    source = NAV2_NAVIGATION_LAUNCH.read_text()
    assert len(re.findall(r"[\"']bond_timeout[\"']: 0\.0", source)) == 2  # composable + standalone lifecycle manager
    grasp = next(c for c in launch_calls("Node") if call_kwarg(c, "name") == "lifecycle_manager_grasp")
    assert call_kwarg(grasp, "parameters")[0]["bond_timeout"] == 0.0


def test_container_exit_shuts_down_launch() -> None:
    """A dead component container must end the launch (non-zero service exit) so systemd Restart= restarts nav2."""
    containers = [c for c in launch_calls("Node") if call_kwarg(c, "executable") == "component_container_isolated"]
    assert len(containers) == 1
    on_exit = next(k for k in containers[0].keywords if k.arg == "on_exit")
    assert ast.unparse(on_exit.value).startswith("Shutdown(")


def test_local_costmap_update_frequency_is_3hz() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    assert params["local_costmap"]["local_costmap"]["ros__parameters"]["update_frequency"] == 3.0


def test_nav2_launch_grasp_docking_server_is_namespaced_and_remapped() -> None:
    """The stock navigation_launch.py does not remap docking_server cmd_vel; the namespaced copy must."""
    pushes = launch_calls("PushRosNamespace")
    assert len(pushes) == 1
    assert ast.literal_eval(pushes[0].args[0]) == "grasp"
    assert len(launch_calls("GroupAction")) == 1
    nodes = {call_kwarg(c, "name"): c for c in launch_calls("Node") if call_kwarg(c, "name") != "nav2_container"}
    assert set(nodes) == {"docking_server", "lifecycle_manager_grasp"}
    dock = nodes["docking_server"]
    assert call_kwarg(dock, "package") == "opennav_docking"
    assert call_kwarg(dock, "executable") == "opennav_docking"
    assert call_kwarg(dock, "remappings") == [("cmd_vel", "/cmd_vel_nav"), ("navigate_to_pose", "/navigate_to_pose")]
    assert "grasp_docking_params.yaml" in NAV2_LAUNCH.read_text()
    manager = nodes["lifecycle_manager_grasp"]
    assert call_kwarg(manager, "package") == "nav2_lifecycle_manager"
    assert call_kwarg(manager, "executable") == "lifecycle_manager"
    manager_params = call_kwarg(manager, "parameters")
    assert manager_params == [{"autostart": True, "node_names": ["docking_server"], "bond_timeout": 0.0}]


def grasp_docking() -> dict:
    """Parameters of /grasp/docking_server.

    Returns:
        dict: The ros__parameters mapping.
    """
    return yaml.safe_load(GRASP_DOCKING_PARAMS.read_text())["/grasp/docking_server"]["ros__parameters"]


def test_grasp_docking_params_plugin_and_external_detection() -> None:
    p = grasp_docking()
    assert p["dock_plugins"] == ["grasp_servoed"]
    plugin = p["grasp_servoed"]
    assert plugin["plugin"] == "opennav_docking::SimpleNonChargingDock"
    assert plugin["use_external_detection_pose"] is True
    assert plugin["external_detection_timeout"] == 1.0
    assert plugin["use_stall_detection"] is False
    # The detection already is the dock pose: no offsets between the detected and the dock frame.
    for key in ("translation_x", "translation_y", "rotation_yaw", "rotation_pitch", "rotation_roll"):
        assert plugin[f"external_detection_{key}"] == 0.0


def test_grasp_docking_controller_uses_absolute_costmap_topics_and_clears_the_dock_threshold() -> None:
    p = grasp_docking()
    controller = p["controller"]
    assert controller["costmap_topic"] == "/local_costmap/costmap_raw"
    assert controller["footprint_topic"] == "/local_costmap/published_footprint"
    assert controller["use_collision_detection"] is True
    assert "simulation_time_step" in controller
    assert controller["dock_collision_threshold"] >= 0.25 + p["grasp_servoed"]["docking_threshold"]


def test_grasp_docking_angular_speed_within_swerve_limit() -> None:
    swerve_wz = nav2()["velocity_smoother"]["ros__parameters"]["max_velocity"][2]
    assert swerve_wz == 0.5
    controller = grasp_docking()["controller"]
    assert controller["v_angular_max"] <= swerve_wz
    assert controller["rotate_to_heading_angular_vel"] <= swerve_wz


def test_mcp_server_approach_block_matches_grasp_docking_params() -> None:
    approach = node_config("mcp_server")["approach"]
    p = grasp_docking()
    plugin = p["grasp_servoed"]
    assert approach["staging_offset_m"] == -plugin["staging_x_offset"]
    assert approach["docking_threshold_m"] == plugin["docking_threshold"]
    assert approach["detection_topic"] == "/grasp/detected_dock_pose"
    assert approach["dock_action"] == "/grasp/dock_robot"
    assert approach["tracker"]["rate_hz"] > 2.0 / plugin["external_detection_timeout"]


def test_mcp_server_approach_block_sets_every_weight_explicitly() -> None:
    weights = node_config("mcp_server")["approach"]["weights"]
    assert weights == {"margin": 1.0, "cost": 0.5, "travel": 0.2, "heading": 0.1}


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
    assert [tuple(pt) for pt in yaml.safe_load(p["footprint"])] == footprint_geometry().costmap_footprint
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


def test_nav2_planner_is_footprint_aware_smac_lattice() -> None:
    """Smac state lattice (checks the full H footprint per expansion) under the GridBased name the BT and web UI use,
    with bounded planning time and NavFn's goal tolerance."""
    planner = ros_params(nav2(), "planner_server")
    assert planner["planner_plugins"] == ["GridBased"]
    plugin = planner["GridBased"]
    assert plugin["plugin"] == "nav2_smac_planner::SmacPlannerLattice"
    assert plugin["allow_unknown"] is True
    assert plugin["tolerance"] == pytest.approx(0.5)
    assert 0.0 < plugin["max_planning_time"] <= 2.0
    assert plugin["allow_reverse_expansion"] is False
    assert 1.0 <= plugin["cost_penalty"] <= 3.0
    assert 0.0 < plugin["analytic_expansion_max_length"] <= 3.0


def test_nav2_lattice_primitives_are_omni_at_the_costmap_resolution() -> None:
    """The primitives file is committed in the repo at the path the robot checkout puts it, generated by Nav2's
    lattice generator (omni, 0.5 m turning radius) at the costmap resolution."""
    plugin = ros_params(nav2(), "planner_server")["GridBased"]
    repo_dest = yaml.safe_load(ALL_VARS.read_text())["ros2_repo_dest"]
    path = plugin["lattice_filepath"]
    assert path.startswith(repo_dest + "/")
    local = REPO_ROOT / path.removeprefix(repo_dest + "/")
    assert local.parent == LATTICE_DIR and local.is_file()
    lattice = json.loads(local.read_text())
    meta = lattice["lattice_metadata"]
    config = json.loads((LATTICE_DIR / "config.json").read_text())
    for key, value in config.items():
        assert meta[key] == value, key
    assert meta["motion_model"] == "omni"
    assert meta["turning_radius"] == pytest.approx(0.5)
    assert meta["number_of_trajectories"] == len(lattice["primitives"]) > 0
    for costmap in ("global_costmap", "local_costmap"):
        assert meta["grid_resolution"] == pytest.approx(ros_params(nav2(), costmap)["resolution"])


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_nav2_costmaps_match_the_slam_map_resolution(costmap: str) -> None:
    slam = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    assert ros_params(nav2(), costmap)["resolution"] == pytest.approx(slam["resolution"]) == MAP_RESOLUTION


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_nav2_obstacle_layer_keeps_obstacles_in_the_notches(costmap: str) -> None:
    """ObstacleLayer footprint clearing fills the footprint column by column (treats the H as convex) and would erase
    obstacles in the notches; the laser_filter self polygon already removes the robot's own returns."""
    assert ros_params(nav2(), costmap)["obstacle_layer"]["footprint_clearing_enabled"] is False


def costmap_zones_defaults() -> dict[str, str]:
    """Read the string field defaults of the costmap_zones settings model without importing it (needs pydantic).

    Returns:
        dict[str, str]: Field name -> default value for every `name: str = Field(default="...")` field.
    """
    tree = ast.parse(COSTMAP_ZONES_CONFIG.read_text())
    defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and isinstance(node.value, ast.Call):
            for kw in node.value.keywords:
                if kw.arg == "default" and isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, str):
                    defaults[node.target.id] = kw.value.value
    return defaults


@pytest.mark.parametrize(
    ("costmap", "plugins"),
    [
        ("global_costmap", ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]),
        (
            "local_costmap",
            [
                "obstacle_layer",
                "depth_voxel_layer",
                "ground_voxel_layer",
                "keepout_filter",
                "ground_cost_filter",
                "inflation_layer",
            ],
        ),
    ],
)
def test_nav2_keepout_filter_sits_right_before_inflation(costmap: str, plugins: list[str]) -> None:
    """The KeepoutFilter is a layer (plugins, not filters) directly below inflation: inflation then spreads the
    lethal zone cost like a real obstacle, which Smac's centre-cost shortcut needs."""
    p = ros_params(nav2(), costmap)
    assert p["plugins"] == plugins
    # The local costmap puts ground_cost_filter (a second KeepoutFilter, non-lethal) between keepout and inflation.
    below_inflation = "ground_cost_filter" if "ground_cost_filter" in p["plugins"] else "keepout_filter"
    assert p["plugins"].index(below_inflation) == p["plugins"].index("inflation_layer") - 1
    assert p["plugins"].index("keepout_filter") == p["plugins"].index(below_inflation) - (
        below_inflation == "ground_cost_filter"
    )
    assert "keepout_filter" not in p.get("filters", [])


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_nav2_keepout_filter_settings(costmap: str) -> None:
    keepout = ros_params(nav2(), costmap)["keepout_filter"]
    assert keepout["plugin"] == "nav2_costmap_2d::KeepoutFilter"
    assert keepout["enabled"] is True
    expected = costmap_zones_defaults()["keepout_info_topic"]
    assert keepout["filter_info_topic"] == expected == "/costmap_filter_info/keepout"
    assert keepout["transform_tolerance"] == 0.2
    assert keepout["override_lethal_cost"] is True
    # Jazzy declares lethal_override_cost as an integer parameter: a YAML float (252.0) makes the costmap
    # activation throw "Wrong parameter type" and aborts the whole Nav2 bringup.
    assert type(keepout["lethal_override_cost"]) is int
    assert keepout["lethal_override_cost"] == 252


def test_nav2_speed_filter_only_in_local_costmap() -> None:
    doc = nav2()
    local = ros_params(doc, "local_costmap")
    assert local["filters"] == ["speed_filter"]
    speed = local["speed_filter"]
    assert speed["plugin"] == "nav2_costmap_2d::SpeedFilter"
    assert speed["filter_info_topic"] == costmap_zones_defaults()["speed_info_topic"] == "/costmap_filter_info/speed"
    assert speed["speed_limit_topic"] == "/speed_limit"
    assert speed["transform_tolerance"] == 0.2
    assert "speed_filter" not in local["plugins"]
    glob = ros_params(doc, "global_costmap")
    assert "speed_filter" not in glob
    assert "speed_filter" not in glob["plugins"] + glob.get("filters", [])


def test_nav2_keepout_docs_describe_lethal_override_correctly() -> None:
    """252 is written only while the robot is inside a zone (so it can plan out); 1.3.10 gives live mask updates."""
    params_text = re.sub(r"\s*\n\s*#\s*", " ", NAV2_PARAMS.read_text())
    assert "the planner treats them as blocked" not in params_text
    assert params_text.count("only while the robot is inside") == 2
    readme = NAV2_README.read_text()
    section = readme.split("## Keepout and speed zones", 1)[1].split("\n## ", 1)[0]
    assert "Cells become cost 252" not in section
    assert "only while the robot is inside" in section
    assert "drop the info topic" not in section
    assert "live mask updates" in section


def test_nav2_controller_listens_on_the_speed_limit_topic() -> None:
    """The SpeedFilter publishes the absolute /speed_limit; the controller must subscribe to the same topic."""
    controller = ros_params(nav2(), "controller_server")
    local = ros_params(nav2(), "local_costmap")
    assert controller["speed_limit_topic"] == local["speed_filter"]["speed_limit_topic"] == "/speed_limit"


def test_slam_map_resolution_cost_is_bounded() -> None:
    """2.5 cm map: search windows stay whole numbers of their own cells and the raster (4x the cells of 5 cm) is
    rebuilt every 10 s instead of 5 s."""
    p = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    for dim, res in (
        ("correlation_search_space_dimension", "correlation_search_space_resolution"),
        ("loop_search_space_dimension", "loop_search_space_resolution"),
    ):
        cells = p[dim] / p[res]
        assert cells == pytest.approx(round(cells)), dim
    assert p["correlation_search_space_resolution"] <= p["resolution"]
    assert p["map_update_interval"] == pytest.approx(10.0)


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
        assert polygon["type"] in ("polygon", "circle", "velocity_polygon")
        assert polygon["action_type"] in ("stop", "slowdown", "approach", "limit")
        # nav2_collision_monitor (Jazzy) Polygon::getPolygonFromString rejects vvf.size() <= 3 even though its error
        # says "at least three points", and one bad polygon aborts the whole nav2 bringup: four points minimum.
        if polygon["type"] == "polygon" and "points" not in polygon:
            # Dynamic shape (ArmStop): received on polygon_sub_topic, no static points to validate.
            assert polygon["polygon_sub_topic"]
        elif polygon["type"] == "polygon":
            assert len(yaml.safe_load(polygon["points"])) >= 4
        elif polygon["type"] == "velocity_polygon":
            for sub in polygon["velocity_polygons"]:
                assert len(yaml.safe_load(polygon[sub]["points"])) >= 4, f"{name}.{sub} needs 4+ points"
        else:
            assert polygon["radius"] > 0
        assert isinstance(polygon["min_points"], int) and polygon["min_points"] > 0
    sources = cm["observation_sources"]
    assert any(
        cm[s]["type"] == "scan" and cm[s]["topic"] == FILTERED_SCAN and cm[s]["enabled"] is True for s in sources
    )


def test_nav2_collision_monitor_stopbox_enabled_on_filtered_scan() -> None:
    """StopBox (2 cm outside the footprint) is enabled: on /scan_filtered the robot body is removed and a 20 s
    stationary sample on the robot (2026-10-03) had 0 returns in the band (min_points 4)."""
    cm = ros_params(nav2(), "collision_monitor")
    assert (cm["cmd_vel_in_topic"], cm["cmd_vel_out_topic"]) == ("cmd_vel_smoothed", "cmd_vel")
    assert cm["base_frame_id"] == "base_link" and cm["odom_frame_id"] == "odom"
    for key in ("state_topic", "transform_tolerance", "source_timeout", "stop_pub_timeout"):
        assert key in cm, key
    assert "StopBox" in cm["polygons"], "the polygon stays declared so collision_monitor configures"
    stop = cm["StopBox"]
    assert stop["enabled"] is True
    assert stop["action_type"] == "stop"
    assert stop["type"] == "velocity_polygon" and stop["holonomic"] is True
    assert stop["min_points"] >= 4


def point_in_polygon(x: float, y: float, poly: list[list[float]]) -> bool:
    """Ray-casting point-in-polygon test (as nav2_collision_monitor Polygon::isPointInside).

    Args:
        x (float): Point x (m).
        y (float): Point y (m).
        poly (list[list[float]]): Polygon vertices.

    Returns:
        bool: True when the point is inside.
    """
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def stopbox_polygon(vx: float, vy: float, wz: float) -> list[list[float]]:
    """StopBox sub-polygon nav2_collision_monitor selects for a command (VelocityPolygon::isInRange, first match).

    Args:
        vx (float): Forward (m/s).
        vy (float): Left (m/s).
        wz (float): Yaw rate (rad/s).

    Returns:
        list[list[float]]: Selected polygon vertices.
    """
    stop = ros_params(nav2(), "collision_monitor")["StopBox"]
    magnitude = math.hypot(vx, vy)
    direction = math.atan2(vy, vx) if magnitude > 0.0 else 0.0
    for name in stop["velocity_polygons"]:
        sub = stop[name]
        start, end = sub["direction_start_angle"], sub["direction_end_angle"]
        in_dir = start <= direction <= end if start <= end else direction >= start or direction <= end
        if (
            sub["theta_min"] <= wz <= sub["theta_max"]
            and sub["linear_min"] <= magnitude <= sub["linear_max"]
            and in_dir
        ):
            return yaml.safe_load(sub["points"])
    raise AssertionError(f"velocity ({vx}, {vy}, {wz}) is not covered by any StopBox sub-polygon")


# A return 1.5 cm outside the H footprint (0.5 cm outside the laser_filter self polygon, so it survives /scan_filtered):
# ahead of the front-right wheel (envelope front x 0.1525 + 0.0625), behind the rear-left wheel and beside the side
# frames (outer face y 0.1333 + 0.0175); and ahead of the lidar dome in the front notch
# (dome front x 0.150 + 0.037, behind the frames).
BAND_POINTS = {
    "front": (0.23, -0.1333),
    "front_lidar": (0.202, 0.04),
    "back": (-0.23, 0.1333),
    "left": (0.0, 0.1658),
    "right": (0.0, -0.1658),
}
# Returns in the front notch (between the front wheels): deep inside, and 1.5 cm ahead of the crossbar; both lie under
# the parked gripper, which is part of the static footprint. In the rear notch: deep inside, and 1.5 cm behind the
# crossbar.
FRONT_NOTCH_DEEP = (0.16, -0.035)
FRONT_NOTCH_CROSSBAR = (0.115, -0.04)
REAR_NOTCH_DEEP = (-0.2, 0.0)
REAR_NOTCH_CROSSBAR = (-0.165, 0.0)
FACE_NORMALS = {
    "front": (1.0, 0.0),
    "front_lidar": (1.0, 0.0),
    "back": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}


def test_stopbox_covers_every_velocity() -> None:
    for deg in range(-180, 181, 5):
        for speed in (0.0, 0.003, 0.02, 0.1, 0.25):
            for wz in (-0.5, 0.0, 0.5):
                stopbox_polygon(speed * math.cos(math.radians(deg)), speed * math.sin(math.radians(deg)), wz)


@pytest.mark.parametrize("face", sorted(BAND_POINTS))
def test_stopbox_stops_toward_an_obstacle_and_lets_the_robot_escape(face: str) -> None:
    """Direction-aware StopBox: an obstacle in the band stops motion toward its side only, so the robot can back away
    or slide along it instead of freezing (the plain StopBox zeroed every command once an obstacle was inside)."""
    px, py = BAND_POINTS[face]
    nx, ny = FACE_NORMALS[face]
    speed = 0.1
    assert point_in_polygon(px, py, stopbox_polygon(speed * nx, speed * ny, 0.0)), "toward"
    assert point_in_polygon(px, py, stopbox_polygon(speed * (nx - ny), speed * (ny + nx), 0.0)), "diagonal toward"
    assert not point_in_polygon(px, py, stopbox_polygon(-speed * nx, -speed * ny, 0.0)), "away"
    assert not point_in_polygon(px, py, stopbox_polygon(-speed * (nx - ny), -speed * (ny + nx), 0.0)), "diag away"
    assert not point_in_polygon(px, py, stopbox_polygon(-speed * ny, speed * nx, 0.0)), "along"
    assert not point_in_polygon(px, py, stopbox_polygon(speed * ny, -speed * nx, 0.0)), "along"
    assert point_in_polygon(px, py, stopbox_polygon(0.0, 0.0, 0.5)), "rotating in place checks the footprint + 2 cm"


def test_stopbox_ignores_notch_obstacles_unless_moving_into_them() -> None:
    """An obstacle deep in the rear notch stops no motion at all (rotation in place checks the footprint + 2 cm, not
    the swept circle); one just behind the crossbar stops only the motion that drives the crossbar into it. The front notch lies
    under the parked gripper: an obstacle there is inside the footprint and stops every motion."""
    speed = 0.1
    moves = {"forward": (speed, 0.0), "back": (-speed, 0.0), "left": (0.0, speed), "right": (0.0, -speed)}
    for name, (vx, vy) in moves.items():
        assert not point_in_polygon(*REAR_NOTCH_DEEP, stopbox_polygon(vx, vy, 0.0)), name
    assert not point_in_polygon(*REAR_NOTCH_DEEP, stopbox_polygon(0.0, 0.0, 0.5))
    for name, (vx, vy) in moves.items():
        assert point_in_polygon(*REAR_NOTCH_CROSSBAR, stopbox_polygon(vx, vy, 0.0)) is (name == "back"), name
    footprint = Polygon(footprint_geometry().footprint)
    for point in (FRONT_NOTCH_DEEP, FRONT_NOTCH_CROSSBAR):
        assert footprint.contains(Point(point)), point
        for name, (vx, vy) in moves.items():
            assert point_in_polygon(*point, stopbox_polygon(vx, vy, 0.0)), (point, name)


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
        "obstacle_cloud_topic": "/head/d435/points_filtered",
        "scan_points_topic": "/scan_filtered",
        # 3D map (map3d_manager / rtabmap_mapping) layer and lidar-wall settings.
        "map3d_cloud_topic": "/map3d/cloud",
        "map3d_status_topic": "/map3d/status",
        "map3d_result_topic": "/map3d/result",
        "map3d_command_topic": "/map3d/command",
        "map3d_frame_id": "rtabmap_map",
        "map3d_max_points": 60000,
        "map3d_voxel_m": 0.05,
        # "Head RGB-D" layer: head_rgbd_cpp XYZRGB cloud (on demand).
        "rgbd_cloud_topic": "/head_rgbd/points",
        # "Ground" layer: ground_estimator_cpp labelled cloud and JSON state.
        "ground_cloud_topic": "/ground/labelled_points",
        "ground_state_topic": "/ground/state",
        "ground_state_stale_s": 2.0,
        "lidar_wall_height_m": 1.0,
        "base_urdf": "robot.urdf",
        "base_joint_states_topic": "/swerve_drive/joint_states",
        "arm_urdf": "so101_arm.urdf",
        "arm_joint_states_topic": "/follower/joint_states",
        # Arm mount in base_link (x, y, z m): the mcp_server arm.base_in_base_link measured 2026-10-10.
        "arm_offset": [0.0592, -0.05, 0.100],
        # Home / Set home buttons call the std_srvs/Trigger services served by mcp_server.
        "arm_home_service": "/arm/home",
        "arm_set_home_service": "/arm/set_home",
        # Head pan/tilt in the merged 3D view: head URDF driven by the base_servos head joint states.
        "head_urdf": "head.urdf",
        "head_joint_states_topic": "/head/joint_states",
        # Points of interest served by poi_store.
        "poi_list_topic": "/poi/list",
        "poi_command_topic": "/poi/command",
        "poi_result_topic": "/poi/result",
        "poi_photos_topic": "/poi/photos",
        # Grasp panel: JSON requests/answers of the mcp_server GraspService (POST /api/grasp).
        "grasp_command_topic": "/grasp/command",
        "grasp_result_topic": "/grasp/result",
        "lookups_dir": "/var/lib/ros2/poi/lookups",
        # Parked GPS anchor from one fix + the BNO055 NDOF compass heading, gated by its calibration.
        "gps_anchor_imu_topic": "/imu/data",
        "gps_anchor_imu_calibration_topic": "/imu/calibration",
        "magnetic_declination_deg": 6.6,
        # Satellite (Esri, {z}/{y}/{x} order) and street (keyed CARTO, key tier serves z <= 18); satellite first.
        "tile_display_zoom": 19,
        "default_tile_source": "satellite",
        "tile_sources": [
            {
                "id": "satellite",
                "label": "Satellite",
                "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                "max_zoom": 20,
                "attribution": "Esri, Maxar, Earthstar Geographics",
            },
            {
                "id": "street",
                "label": "Street",
                "url": "https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png?key={api_key}",
                "subdomains": "abcd",
                "max_zoom": 18,
                "api_key_env": "WEB_UI_TILE_API_KEY",
                "attribution": "CARTO, OpenStreetMap contributors",
            },
        ],
    }


def test_web_ui_arm_offset_matches_mcp_server_arm_mount() -> None:
    tab = next(t for t in node_config("web_ui")["tabs"] if t["id"] == "map")
    mount = node_config("mcp_server")["arm"]["base_in_base_link"]
    assert tab["arm_offset"] == [mount["x"], mount["y"], mount["z"]]


def test_web_ui_loads_the_tile_api_key_env_file_and_never_stores_the_key() -> None:
    """The unit reads WEB_UI_TILE_API_KEY from /etc/ros2/web_ui/env (EnvironmentFile=), written from CARTO_API_KEY."""
    defaults = client_vars()["ros2_node_type_defaults"]["web_ui"]
    assert defaults["environment_file"] == "/etc/ros2/web_ui/env"
    task = (ANSIBLE_DIR / "playbooks" / "tasks" / "web_ui_tile_key.yml").read_text()
    assert "lookup('env', 'CARTO_API_KEY')" in task and "WEB_UI_TILE_API_KEY=" in task
    assert 'mode: "0600"' in task and "no_log: true" in task and "line: web_ui" in task
    assert "tasks/web_ui_tile_key.yml" in (ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml").read_text()


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
    # Platform dimensions.md gave (0.15, 0.04); the in-place turn test 2026-10-05 (90 deg turns: 3.1-3.6 cm of map
    # translation, none in the EKF odometry) fits one lidar offset error of (+0.8, +2.0) cm.
    assert (laser["x"], laser["y"], laser["z"]) == (0.158, 0.06, 0.20)
    assert laser["yaw"] == pytest.approx(math.pi, abs=1e-4)


def test_tests_readme_lidar_tf_matches_client_yml() -> None:
    """The tests/README.md row for the lidar TF test quotes the same position as client.yml."""
    entry = node_entry("static_tf_publisher")
    laser = next(f for f in yaml.safe_load(entry["config"])["frames"] if f["child"] == LASER_FRAME)
    lines = TESTS_README.read_text().splitlines()
    row = next(line for line in lines if "`test_lidar_static_tf_mounted_backwards`" in line)
    assert f"({laser['x']}, {laser['y']}, {laser['z']:.2f})" in row, row


def test_nodes_readme_names_the_smac_lattice_planner() -> None:
    """nodes/README.md describes nav2_bringup with the Smac lattice planner, not the replaced NavFn."""
    line = next(line for line in NODES_README.read_text().splitlines() if "**nav2_bringup/**" in line)
    assert "NavFn" not in line
    assert "Smac" in line and "lattice" in line


def test_laser_filter_polygon_matches_the_h_outline() -> None:
    """Startup self polygon of the laser_filter node in base_link: the generated static footprint (H body + parked
    arm) + 1 cm (no padding parameter: the margin is generated into the polygon); returns in the rear notch are kept,
    the front notch lies under the parked gripper."""
    params = yaml.safe_load(LASER_FILTER_PARAMS.read_text())
    assert set(params) == {
        "executor",
        "scan_topic",
        "scan_filtered_topic",
        "polygon_frame",
        "transform_timeout_s",
        "polygon",
        "cloud_topic",
        "cloud_filtered_topic",
        "cloud_self_max_height_m",
        "cloud_transform_timeout_s",
        "metrics_port",
    }
    assert params["polygon_frame"] == "base_link"
    assert (params["scan_topic"], params["scan_filtered_topic"]) == ("/scan", FILTERED_SCAN)
    # The D435 cloud self filter (same runtime polygon): /head/d435/points -> /head/d435/points_filtered.
    assert (params["cloud_topic"], params["cloud_filtered_topic"]) == (
        "/head/d435/points",
        "/head/d435/points_filtered",
    )
    assert (params["cloud_self_max_height_m"], params["cloud_transform_timeout_s"]) == (1.0, 0.2)
    gen = footprint_geometry()
    polygon = [tuple(pt) for pt in yaml.safe_load(params["polygon"])]
    assert polygon == gen.laser_polygon
    shape = Polygon(polygon)
    assert shape.contains(Polygon(gen.footprint))
    for point in (REAR_NOTCH_DEEP, REAR_NOTCH_CROSSBAR, *BAND_POINTS.values()):
        assert not shape.contains(Point(point)), point
    for point in (FRONT_NOTCH_DEEP, FRONT_NOTCH_CROSSBAR):
        assert shape.contains(Point(point)), point


def test_laser_filter_ansible_wiring() -> None:
    group = client_vars()
    defaults = group["ros2_node_type_defaults"]["laser_filter"]
    # The repo's Python node (runtime-settable polygon), not laser_filters' scan_to_scan_filter_chain.
    assert defaults["node_launch_command"] == "python3 -m laser_filter"
    assert defaults["node_src_dir"] == "nodes/laser_filter"
    # sensor_msgs_py.point_cloud2 reads and writes the D435 cloud (/head/d435/points -> /head/d435/points_filtered).
    assert defaults["apt_packages"] == ["ros-jazzy-sensor-msgs-py"]
    assert defaults["env"] == [
        "LASER_FILTER_CONFIG={{ ros2_repo_dest }}/nodes/laser_filter/config/footprint_filter.yaml"
    ]
    entry = node_entry("laser_filter")
    assert entry["present"] is True and entry["enabled"] is True
    tasks = flat(yaml.safe_load((PLAYBOOKS_DIR / "deploy_nodes_client.yml").read_text())[0]["tasks"])
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
    """Goals finish within 1 cm / 2 deg (user requirement). The default BT replanned at 1 Hz and every replan reset
    the goal checker latch and the rotation shim's position check, so the shim lost the in-tolerance state and MPPI
    took over mid-turn (58 deg overshoot on the robot). With replanning only on an invalid path the stateful
    checker latches reliably."""
    checker = ros_params(nav2(), "controller_server")["general_goal_checker"]
    assert checker["xy_goal_tolerance"] == pytest.approx(0.01)
    assert checker["yaw_goal_tolerance"] == pytest.approx(0.035)
    assert checker["stateful"] is True


def test_nav2_replans_only_when_path_invalid() -> None:
    """Periodic replanning resets the goal checker latch; replan only if the path becomes invalid."""
    bt = ros_params(nav2(), "bt_navigator")["default_nav_to_pose_bt_xml"]
    assert bt.endswith("/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml")


def test_nav2_progress_checker_counts_rotation() -> None:
    """In-place rotation is progress: PoseProgressChecker counts angle too, and a small radius suits cm-scale goals."""
    checker = ros_params(nav2(), "controller_server")["progress_checker"]
    assert checker["plugin"] == "nav2_controller::PoseProgressChecker"
    assert checker["required_movement_radius"] <= 0.05
    assert checker["required_movement_angle"] > 0


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
    assert follow["max_angular_accel"] <= 0.5
    assert follow["rotate_to_heading_once"] is True
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
        if isinstance(t, dict)
        and isinstance(t.get("ansible.builtin.command"), dict)
        and "npm" in str(t["ansible.builtin.command"].get("cmd", ""))
    ]
    assert len(cmds) == 2, cmds
    for cmd in cmds:
        # Shared build cgroup cap (all.yml ros2_build_cpu_quota, background CPU weight, low IO weight) plus lowest CPU
        # and IO priority.
        assert cmd.startswith(
            "systemd-run --quiet --scope -p CPUQuota={{ ros2_build_cpu_quota }}"
            " -p CPUWeight={{ ros2_build_cpu_weight }} -p IOWeight=10 "
        ), cmd
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
def test_costmaps_add_no_margin_beyond_the_footprint(costmap: str) -> None:
    """Minimal penalty zone: no footprint padding, inflation only to the H footprint's circumscribed radius rounded up
    to one cell (needed for footprint collision costs) with a steep 20-30/m falloff, the same on both costmaps."""
    p = ros_params(nav2(), costmap)
    assert p["footprint_padding"] == 0.0
    circumscribed = circumscribed_radius(footprint_geometry().costmap_footprint)
    inflation = p["inflation_layer"]
    assert circumscribed <= inflation["inflation_radius"] < circumscribed + p["resolution"]
    assert 20.0 <= inflation["cost_scaling_factor"] <= 30.0
    other = ros_params(nav2(), "local_costmap" if costmap == "global_costmap" else "global_costmap")
    assert other["inflation_layer"] == inflation


def test_safety_polygons_hug_the_footprint() -> None:
    """The self-filter polygon reaches at most 1.5 cm beyond the footprint (1 cm + mitred corners) and every StopBox
    sub-polygon at most its 2 cm sweep (2.45 cm on a 70 deg diagonal sector), so far obstacles never trigger it;
    rotation in place checks the footprint + 2 cm."""
    gen = footprint_geometry()
    footprint = Polygon(gen.footprint)
    reach = {"laser": Polygon(gen.laser_polygon)}
    stop = ros_params(nav2(), "collision_monitor")["StopBox"]
    for sub in stop["velocity_polygons"]:
        reach[sub] = Polygon(yaml.safe_load(stop[sub]["points"]))
    for name, shape in reach.items():
        far = max(footprint.distance(Point(pt)) for pt in shape.exterior.segmentize(0.001).coords)
        # rotate: the footprint offset 2 cm with mitred corners (bevelled past 1.5 x the margin = 3 cm).
        limit = 0.0151 if name == "laser" else 0.0301 if name == "rotate" else 0.0246
        assert far <= limit, (name, far)


@pytest.mark.parametrize("costmap", ["global_costmap", "local_costmap"])
def test_costmaps_wait_for_odom_through_a_gradual_restart(costmap: str) -> None:
    """A deploy restarts nav2_bringup before the gradual ramp brings up the servos and EKF (odom TF); the default 60 s
    initial_transform_timeout aborted the bringup, so the costmaps wait long enough for the whole ramp."""
    assert ros_params(nav2(), costmap)["initial_transform_timeout"] >= 300.0


# --- slip resilience (traction loss, bumps, low obstacles, transient objects) -----------------------


def test_slam_scan_matcher_is_resilient_to_slip_and_transients() -> None:
    """Wheel slip on small objects shifted the map: a wider scan-matcher window absorbs odometry error, a pure
    rotation inserts a scan, a Huber loss limits the pull of outlier constraints, and a stricter occupancy
    threshold / pass-through count lets briefly visible objects be cleared."""
    p = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    assert p["correlation_search_space_dimension"] == 0.8
    # slam_toolbox requires the search space to be a whole number of grid cells.
    cells = p["correlation_search_space_dimension"] / p["correlation_search_space_resolution"]
    assert cells == pytest.approx(round(cells))
    assert p["check_min_dist_and_heading_precisely"] is True
    assert p["ceres_loss_function"] == "HuberLoss"
    assert p["occupancy_threshold"] == 0.25
    assert p["min_pass_through"] == 3
    # Values that stay as they were.
    assert p["minimum_travel_distance"] == 0.3 and p["minimum_travel_heading"] == 0.3
    assert p["do_loop_closing"] is True


def test_slam_ansible_block_does_not_override_slip_resilience_keys() -> None:
    p = node_config("slam_toolbox")["slam_toolbox"]["ros__parameters"]
    for key in (
        "correlation_search_space_dimension",
        "check_min_dist_and_heading_precisely",
        "ceres_loss_function",
        "occupancy_threshold",
        "min_pass_through",
    ):
        assert key not in p, f"{key} is overridden in the Ansible block, hiding the repo default"


def test_imu_angular_velocity_covariance_is_fixed_and_small() -> None:
    """The rolling-variance mode inflated the yaw-rate covariance exactly while turning, so the EKF trusted the
    slip-prone wheel yaw rate over the gyro: use a fixed, small gyro covariance."""
    cfg = node_config("bno055_imu")
    assert cfg["compute_covariance"] is False
    assert cfg["angular_velocity_covariance"] == 0.0004


def test_swerve_controller_slip_residual_threshold_configured() -> None:
    assert node_config("swerve_controller")["slip_residual_threshold_mps"] == 0.05


@pytest.mark.parametrize("source", ["repo", "ansible"])
def test_ekf_fuses_rf2o_and_rejects_outlying_wheel_odometry(source: str) -> None:
    """rf2o (relayed with a real covariance) is the second translation source, so the wheel odometry can be
    rejected by Mahalanobis distance when the wheels report motion the lidar does not see (full stall). No
    rejection on rf2o or the IMU; the gyro stays the only yaw-rate source besides the wheels."""
    doc = yaml.safe_load(EKF_CONFIG.read_text()) if source == "repo" else node_config("robot_localization_ekf")
    p = ekf_params(doc)
    assert p["odom1"] == RF2O_FUSED_TOPIC
    assert fused(p["odom1_config"]) == {"vx", "vy"}
    assert p["odom1_differential"] is False
    assert p["odom1_relative"] is False
    assert p["odom1_queue_size"] >= 2
    assert p["odom0_twist_rejection_threshold"] == 3.0
    assert not any(
        k.startswith(("odom1_twist_rejection", "odom1_pose_rejection", "imu0_")) and "rejection" in k for k in p
    )
    assert fused(p["imu0_config"]) == {"vyaw"}


def test_rf2o_node_type_builds_a_pinned_source_workspace() -> None:
    defaults = client_vars()["ros2_node_type_defaults"]["rf2o_laser_odometry"]
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_src_dir"] == ""
    cmd = defaults["node_launch_command"]
    assert "ros2 run rf2o_laser_odometry rf2o_laser_odometry_node" in cmd
    assert "{{ ros2_repo_dest }}/nodes/rf2o_laser_odometry/config/rf2o.yaml" in cmd
    assert "-r __node:=rf2o_laser_odometry" in cmd
    # rf2o logs "Waiting for laser_scans...." at WARN on every idle loop: only errors are kept.
    assert "--log-level rf2o_laser_odometry:=error" in cmd
    source = defaults["colcon_source"]
    assert source["repo"] == "https://github.com/MAPIRlab/rf2o_laser_odometry.git"
    assert re.fullmatch(r"[0-9a-f]{40}", source["commit"]), "pin a full commit SHA"
    assert source["workspace"] == "/opt/ros2-ws"
    assert source["package"] == "rf2o_laser_odometry"
    for pkg in (
        "python3-colcon-common-extensions",
        "git",
        "libboost-dev",
        "libeigen3-dev",
        "ros-jazzy-eigen3-cmake-module",
    ):
        assert pkg in defaults["apt_packages"], pkg


def test_rf2o_params_match_the_stack() -> None:
    params = yaml.safe_load(RF2O_PARAMS.read_text())["rf2o_laser_odometry"]["ros__parameters"]
    assert params["laser_scan_topic"] == FILTERED_SCAN
    assert params["odom_topic"] == "/odom_rf2o"
    assert params["publish_tf"] is False, "the EKF owns odom -> base_link"
    assert params["base_frame_id"] == "base_link" and params["odom_frame_id"] == "odom"
    assert params["init_pose_from_topic"] == ""
    assert 7.0 <= params["freq"] <= 10.0


def test_rf2o_nodes_are_deployed_before_the_ekf() -> None:
    names = [n["name"] for n in client_vars()["ros2_nodes"]]
    ekf = names.index("robot_localization_ekf")
    for name in ("rf2o_laser_odometry", "rf2o_odom_relay"):
        entry = node_entry(name)
        assert entry["present"] is True and entry["enabled"] is True
        assert names.index(name) < ekf
    assert names.index("rf2o_laser_odometry") < names.index("rf2o_odom_relay")
    tasks = flat(yaml.safe_load((PLAYBOOKS_DIR / "deploy_nodes_client.yml").read_text())[0]["tasks"])
    deployed = [t.get("vars", {}).get("_deploy_node_name") for t in tasks]
    assert (
        deployed.index("rf2o_laser_odometry")
        < deployed.index("rf2o_odom_relay")
        < deployed.index("robot_localization_ekf")
    )


def test_rf2o_relay_wiring_matches_ekf_input() -> None:
    """rf2o publishes no covariance, a laser-frame x speed and vy = 0: the relay derives the body twist from the
    pose and adds the covariance. Its output topic is what the EKF fuses."""
    defaults = client_vars()["ros2_node_type_defaults"]["rf2o_odom_relay"]
    assert defaults["node_launch_command"] == "python3 -m rf2o_odom_relay"
    assert defaults["node_src_dir"] == "nodes/rf2o_odom_relay"
    cfg = node_config("rf2o_odom_relay")
    assert cfg["input_topic"] == "/odom_rf2o"
    assert cfg["output_topic"] == RF2O_FUSED_TOPIC
    assert cfg["var_vx_vy"] > 0 and cfg["var_vyaw"] > 0


def test_colcon_source_build_runs_at_lowest_priority_and_only_on_a_new_commit() -> None:
    """colcon on the Pi next to the running stack overheats it: one worker, make -j2, one core, low IO weight,
    nice 19, idle IO class; a stamp file named after the pinned commit makes it idempotent."""
    path = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml"
    tasks = yaml.safe_load(path.read_text())
    git = next(t for t in tasks if "ansible.builtin.git" in t)["ansible.builtin.git"]
    assert git["version"] == "{{ colcon_src.commit }}"
    assert git["dest"].endswith("/src/{{ colcon_src.package }}")
    build = next(t for t in tasks if "colcon build" in str(t.get("ansible.builtin.command", "")))
    cmd = build["ansible.builtin.command"]["cmd"]
    assert cmd.startswith(
        "systemd-run --quiet --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p CPUWeight={{ ros2_build_cpu_weight }}"
        " -p IOWeight=10 nice -n 19 ionice -c 3 "
    )
    assert "--merge-install" in cmd and "--parallel-workers 1" in cmd
    assert "-DCMAKE_BUILD_TYPE=Release" in cmd
    assert build["environment"]["MAKEFLAGS"] == "-j{{ colcon_src.jobs | default(ros2_build_jobs) }}"
    assert "_colcon_patch_key" in build["ansible.builtin.command"]["creates"]
    assert "_colcon_patch_key" in cmd
    assert build["notify"] == "Restart ROS2 node"
    main = (ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml").read_text()
    assert "colcon_source_build.yml" in main
    assert "colcon_source_package.yml" in (path.parent / "colcon_source_build.yml").read_text()


def test_launcher_sources_the_colcon_workspace_after_ros() -> None:
    template = (ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-launcher.j2").read_text()
    assert template.index("source /opt/ros/jazzy/setup.bash") < template.index(
        "source {{ (node_colcon_source if node_colcon_source is mapping else node_colcon_source[0]).workspace }}"
        "/install/setup.bash"
    )
    resolve = (PLAYBOOKS_DIR / "tasks" / "resolve_and_deploy.yml").read_text()
    assert "node_colcon_source:" in resolve and "colcon_source" in resolve


RF2O_PATCH = REPO_ROOT / "nodes" / "rf2o_laser_odometry" / "patches" / "0001-retry-laser-tf.patch"


def test_colcon_source_build_applies_patches_and_keys_the_stamp_on_their_content() -> None:
    """Upstream rf2o ignores a missing base_link -> laser TF on its first scan (identity laser pose for the whole
    session, sign-inverted vx/vy with the 180 deg lidar): the repo patch is applied after the checkout, and the
    stamp includes the patch content hash so a patch change rebuilds. The stamp is dropped when install/setup.bash
    is missing."""
    path = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml"
    tasks = yaml.safe_load(path.read_text())
    names = [
        next(
            k
            for k in t
            if k not in ("name", "loop", "when", "become", "notify", "environment", "register", "loop_control", "vars")
        )
        for t in tasks
    ]
    clone = names.index("ansible.builtin.git")
    patch = next(i for i, t in enumerate(tasks) if "ansible.builtin.patch" in t)
    build = next(i for i, t in enumerate(tasks) if "colcon build" in str(t.get("ansible.builtin.command", "")))
    assert clone < patch < build
    assert tasks[clone]["ansible.builtin.git"]["force"] is True, "re-clone must discard the previous patch"
    patch_args = tasks[patch]["ansible.builtin.patch"]
    assert patch_args["remote_src"] is True and patch_args["strip"] == 1
    assert patch_args["basedir"].endswith("/src/{{ colcon_src.package }}")
    assert "item" in patch_args["src"]
    assert tasks[patch]["loop"] == "{{ colcon_src.patches | default([]) }}"
    text = path.read_text()
    assert "ansible.builtin.stat" in text and "checksum" in text and "_colcon_patch_key" in text
    assert "install/setup.bash" in text


def test_rf2o_retry_laser_tf_patch_exists_and_is_wired() -> None:
    source = client_vars()["ros2_node_type_defaults"]["rf2o_laser_odometry"]["colcon_source"]
    assert any(p.endswith("/nodes/rf2o_laser_odometry/patches/0001-retry-laser-tf.patch") for p in source["patches"])
    text = RF2O_PATCH.read_text()
    assert "--- a/src/CLaserOdometry2DNode.cpp" in text and "+++ b/src/CLaserOdometry2DNode.cpp" in text
    assert "+      if (!setLaserPoseFromTf())" in text
    assert "RCLCPP_INFO_THROTTLE" in text and "+        return;" in text


def test_rf2o_odom_relay_declares_numpy() -> None:
    pyproject = tomllib.loads((REPO_ROOT / "nodes" / "rf2o_odom_relay" / "pyproject.toml").read_text())
    assert any(re.match(r"numpy\b", dep) for dep in pyproject["project"]["dependencies"])


def test_ekf_rate_fits_the_cpu_budget() -> None:
    """2026-10-08: at 50 Hz the EKF kept logging 'Failed to meet update rate' on the loaded Pi; its odom TF lagged the
    scans, slam_toolbox dropped them and map->base_link went stale. 30 Hz is enough for a 0.25 m/s base."""
    assert ekf_params(yaml.safe_load(EKF_CONFIG.read_text()))["frequency"] == 30.0


def test_slam_waits_for_a_lagging_odom_transform() -> None:
    """slam_toolbox's message filter dropped every scan whose odom TF arrived after transform_timeout 0.2 s."""
    p = yaml.safe_load(SLAM_PARAMS.read_text())["slam_toolbox"]["ros__parameters"]
    assert p["transform_timeout"] >= 0.5


def test_swerve_controller_tilt_guard_block() -> None:
    """Tilt guard: larger limits on the longer (pitch) axis, warning below error, brake on error, state topic."""
    tilt = node_config("swerve_controller")["tilt_guard"]
    assert tilt["enabled"] is True
    assert tilt["imu_topic"] == "/imu/data"
    assert tilt["state_topic"] == "/tilt_guard/state"
    assert tilt["roll_warn_deg"] < tilt["roll_error_deg"]
    assert tilt["pitch_warn_deg"] < tilt["pitch_error_deg"]
    assert tilt["pitch_warn_deg"] > tilt["roll_warn_deg"]
    assert tilt["pitch_error_deg"] > tilt["roll_error_deg"]
    assert tilt["error_action"] == "brake"
    for key in ("persistence_s", "hysteresis_deg", "recovery_hold_s", "level_reference_roll_deg"):
        assert key in tilt
    # Same BNO055 mounting bias as mcp_server floor_guard.imu_level_reference; contract heartbeat 1 Hz.
    assert tilt["level_reference_roll_deg"] == 0.7
    assert tilt["level_reference_pitch_deg"] == 1.3
    assert tilt["state_publish_hz"] == 1.0
