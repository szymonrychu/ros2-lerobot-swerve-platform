"""Static invariants of the robot MCP server wiring (Ansible, unit template, filter_node lease, web_ui tabs, setup).

The mcp_server node (nodes/mcp_server) serves MCP over Streamable HTTP on the client; its bearer token lives only on the
robot in a 0600 EnvironmentFile created by Ansible, never in git. Claude Code reads it via scripts/robot_mcp_token.sh
and the repo-root .mcp.json expands ${ROBOT_MCP_TOKEN}.
"""

import ast
import json
import math
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SHARED_DIR = REPO_ROOT / "shared"
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
ROLE_DIR = ANSIBLE_DIR / "roles" / "ros2_node_deploy"
SERVICE_TEMPLATE = ROLE_DIR / "templates" / "ros2-node-native.service.j2"
RESOLVE_TASKS = PLAYBOOKS_DIR / "tasks" / "resolve_and_deploy.yml"
SETUP_TASKS = PLAYBOOKS_DIR / "tasks" / "mcp_server_setup.yml"
MCP_JSON = REPO_ROOT / ".mcp.json"
TOKEN_SCRIPT = REPO_ROOT / "scripts" / "robot_mcp_token.sh"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
NODE_DIR = REPO_ROOT / "nodes" / "mcp_server"
NODES_README = REPO_ROOT / "nodes" / "README.md"

CONFIG_DIR = "/etc/ros2/mcp_server"
TOKEN_FILE = "/etc/ros2/mcp_server/token"
ARM_DIR = "/var/lib/ros2/arm"
OBJECTS_DIR = "/var/lib/ros2/objects"
OBJECTS_FILE = "/var/lib/ros2/objects/objects.json"
POI_DIR = "/var/lib/ros2/poi"
LOOKUPS_DIR = "/var/lib/ros2/poi/lookups"
PERCEPTION_TOOLS = (
    "get_topdown_view",
    "remember_object",
    "list_objects",
    "forget_object",
    "look_around",
    "list_pois",
    "add_poi",
    "update_poi",
    "delete_poi",
)
PORT = 18200
MCP_URL = "http://client.ros2.lan:18200/mcp"
AUTONOMY = {
    "autonomy_input_topic": "/filter/autonomy_joint_commands",
    "autonomy_release_topic": "/filter/autonomy_release",
    "active_source_topic": "/filter/active_source",
}
TILE_CACHE_DIR = "/var/cache/web_ui/tiles"
TILE_CACHE_TASKS = PLAYBOOKS_DIR / "tasks" / "web_ui_tile_cache_dir.yml"
LEGACY_MAP_KEYS = {"urdf_file", "topic", "arm_urdf_file", "arm_joint_topic", "scan_topic", "costmap_topic"}
WEB_UI_TABS = ["map", "agent", "retros", "arm", "head", "cameras", "poi"]
REMOVED_WEB_UI_TABS = {"arm_servos", "local_nav", "gps_nav", "scene3d", "robot_status"}


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_entry(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name: Node name.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client node.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config.
    """
    return yaml.safe_load(node_entry(name)["config"])


def flat(tasks: list[dict]) -> list[dict]:
    """Expand the deploy playbook's single block into its tasks.

    Args:
        tasks: A play's task list.

    Returns:
        list[dict]: The tasks, with block children in place of the block.
    """
    return [c for t in tasks for c in (t["block"] if "block" in t else [t])]


def play_tasks(path: Path) -> tuple[list[dict], list[dict], list[dict]]:
    """Pre-tasks, tasks and post-tasks of the first play in a playbook.

    Args:
        path: Playbook path.

    Returns:
        tuple[list[dict], list[dict], list[dict]]: (pre_tasks, tasks, post_tasks).
    """
    play = yaml.safe_load(path.read_text())[0]
    return play.get("pre_tasks", []), flat(play.get("tasks", [])), play.get("post_tasks", [])


def deploy_index(tasks: list[dict], name: str) -> int:
    """Index of the task deploying a node through resolve_and_deploy.yml.

    Args:
        tasks: Task list.
        name: Node name.

    Returns:
        int: Task index (-1 when absent).
    """
    for i, t in enumerate(tasks):
        if "resolve_and_deploy.yml" in str(t.get("ansible.builtin.include_tasks", "")) and (
            t.get("vars", {}).get("_deploy_node_name") == name
        ):
            return i
    return -1


def include_index(tasks: list[dict], filename: str) -> int:
    """Index of the task including a task file.

    Args:
        tasks: Task list.
        filename: Included file name.

    Returns:
        int: Task index (-1 when absent).
    """
    return next((i for i, t in enumerate(tasks) if filename in str(t.get("ansible.builtin.include_tasks", ""))), -1)


def test_mcp_server_node_type_defaults() -> None:
    d = client_vars()["ros2_node_type_defaults"]["mcp_server"]
    assert d["deploy_mode"] == "native"
    assert d["node_src_dir"] == "nodes/mcp_server"
    assert d["node_launch_command"] == "python3 -m mcp_server"
    # No CPU quota (robot headroom model); MemoryMax 1G safety net while the 383 MB spike is investigated.
    assert "cpu_quota" not in d and d["memory_max"] == "1G"
    assert (d["memory_low"], d["cpu_weight"], d["oom_score_adjust"]) == ("184M", 300, -200)
    assert d["config_path"] == CONFIG_DIR
    assert f"MCP_SERVER_CONFIG={CONFIG_DIR}/config.yaml" in d["env"]
    assert d["environment_file"] == TOKEN_FILE
    # The token is never inlined into the unit's Environment= lines.
    assert not any("MCP_SERVER_TOKEN" in e for e in d["env"])


def test_mcp_server_node_entry_and_config() -> None:
    entry = node_entry("mcp_server")
    assert entry["node_type"] == "mcp_server"
    assert entry["present"] is True and entry["enabled"] is True
    cfg = node_config("mcp_server")
    assert set(cfg) <= {
        "server",
        "frontiers",
        "map3d",
        "exploration",
        "maps",
        "grasp_check",
        "topics",
        "limits",
        "timeouts",
        "nav",
        "arm",
        "head",
        "camera_arm",
        "battery",
        "monitor",
        "footprint",
        "nav_footprint",
        "topdown",
        "objects",
        "look_around",
        "motion_queue",
        "poi",
        "auto_poi",
        "lookups",
        "cameras",
        "floor_guard",
        "body",
        "body_check",
        "grasp",
        "grip_profiles",
        "control",
        "guard_rules",
        "reach_map",
        "approach",
        "calibration_board",
        "on_demand",
        "joint_state_rate",
    }
    assert cfg["server"] == {
        "host": "0.0.0.0",
        "port": PORT,
        "path": "/mcp",
        "board_park_token_file": "/etc/ros2/mcp_server/board_park_token",
    }
    assert cfg["arm"]["home_file"] == f"{ARM_DIR}/home.yaml"
    assert cfg["arm"]["urdf_path"] == "nodes/web_ui/urdf/so101_arm.urdf"
    assert (REPO_ROOT / cfg["arm"]["urdf_path"]).is_file()


def test_mcp_server_auto_poi_block_is_deployed() -> None:
    """Auto stuck-spot POIs and the poi_hint nudge (design D8b/c) are configured explicitly on the robot."""
    auto = node_config("mcp_server")["auto_poi"]
    assert auto == {
        "enabled": True,
        "merge_radius_m": 0.5,
        "event_types": ["collision_stop", "nav_abort"],
        "max_pose_age_s": 2.0,
        "hint_after_images": 3,
    }


def test_mcp_server_topics_match_filter_node_lease() -> None:
    topics = node_config("mcp_server").get("topics", {})
    flt = node_config("filter_node")
    assert topics.get("autonomy_command", AUTONOMY["autonomy_input_topic"]) == flt["autonomy_input_topic"]
    assert topics.get("autonomy_release", AUTONOMY["autonomy_release_topic"]) == flt["autonomy_release_topic"]
    assert topics.get("active_source", AUTONOMY["active_source_topic"]) == flt["active_source_topic"]
    assert topics.get("follower_joint_states", "/follower/joint_states") == flt["follower_feedback_topic"]


def test_filter_node_autonomy_params() -> None:
    cfg = node_config("filter_node")
    for key, topic in AUTONOMY.items():
        assert cfg[key] == topic, key


def test_mcp_server_arm_mount_measured_and_height_agree_with_claude_agent() -> None:
    arm = node_config("mcp_server")["arm"]
    assert arm["arm_base_height_m"] == 0.100
    assert arm["base_in_base_link"] == {"x": 0.0592, "y": -0.05, "z": 0.100, "yaw": 0.0}
    assert node_config("claude_agent")["arm_base_height_m"] == arm["arm_base_height_m"]


def test_mcp_server_floor_guard_and_grasp_blocks_are_deployed() -> None:
    cfg = node_config("mcp_server")
    guard = cfg["floor_guard"]
    assert guard["enabled"] is True and guard["margin_m"] == 0.02 and guard["slow_speed_scale"] == 0.2
    assert guard["surface_z_m"] == 0.0 and guard["imu_max_age_s"] == 1.0
    # BNO055 mounting bias on the calibration floor (2026-10-11 investigation), subtracted from the IMU tilt
    assert guard["imu_level_reference"] == {"roll_deg": 0.7, "pitch_deg": 1.3}
    grasp = cfg["grasp"]
    assert grasp["interpolation_step_m"] == 0.005 and grasp["max_object_width_m"] == 0.08
    assert [e["strategy"] for e in grasp["auto_order"]] == ["top_down", "angled", "scoop"]
    assert grasp["scoop_max_pitch_deg"] == 40 and grasp["scoop_gap_margin_m"] == 0.004
    assert grasp["tall_ratio"] == 1.5 and grasp["tall_grasp_height_fraction"] == 0.3
    assert grasp["lift_speed_scale"] == 0.05 and grasp["min_object_width_m"] == 0.01
    # centred grasps keep the fixed jaw 7.5 mm off the object's side face (half of jaw_open_margin_m)
    assert grasp["fixed_jaw_clearance_m"] == 0.0075 and grasp["jaw_open_margin_m"] == 0.015
    assert cfg["topics"]["grasp_command"] == "/grasp/command" and cfg["topics"]["grasp_result"] == "/grasp/result"
    agent = node_config("claude_agent")
    assert {"grasp_object", "release_object"} <= set(agent["effector_tools"])
    assert "plan_grasp" in agent["sensor_tools"]


def test_mcp_server_grip_profiles_block_is_deployed() -> None:
    """grip_profiles: gentle < normal < firm presets plus soft (plush) within the hard caps, default normal, the hold
    re-tighten threshold, follower register topic."""
    cfg = node_config("mcp_server")
    grip = cfg["grip_profiles"]
    presets = grip["presets"]
    assert set(presets) == {"gentle", "normal", "firm", "soft"}
    assert grip["default_grip_profile"] == "normal"
    assert grip["torque_limit_max"] == 800 and grip["squeeze_max_rad"] == 0.08
    torque = [presets[n]["torque_limit"] for n in ("gentle", "normal", "firm")]
    squeeze = [presets[n]["squeeze_rad"] for n in ("gentle", "normal", "firm")]
    assert torque == sorted(torque) and squeeze == sorted(squeeze)
    assert all(t <= grip["torque_limit_max"] for t in torque)
    assert all(q <= grip["squeeze_max_rad"] for q in squeeze)
    assert presets["gentle"]["close_speed_rps"] < presets["normal"]["close_speed_rps"]
    assert presets["gentle"]["target_load"] < presets["gentle"]["contact_effort_threshold"]
    # soft (2026-10-11 plush croc head slipped out of a relaxed normal hold): full squeeze cap, torque within the cap
    assert presets["soft"]["squeeze_rad"] == grip["squeeze_max_rad"]
    assert presets["normal"]["torque_limit"] < presets["soft"]["torque_limit"] <= grip["torque_limit_max"]
    assert presets["soft"]["crush_load"] < presets["soft"]["torque_limit"]
    assert grip["retighten_load_below"] == 30
    # tuned on the real gripper 2026-10-10 (grip tuning session); the deployed presets equal the config defaults
    assert presets == {
        "gentle": {
            "squeeze_rad": 0.01,
            "torque_limit": 250,
            "close_speed_rps": 0.2,
            "target_load": 80,
            "contact_effort_threshold": 120,
            "contact_load_delta": None,
            "crush_load": 200,
            "retighten": False,
        },
        "normal": {
            "squeeze_rad": 0.03,
            "torque_limit": 500,
            "close_speed_rps": 0.5,
            "target_load": 200,
            "contact_effort_threshold": 300,
            "contact_load_delta": None,
            "crush_load": 450,
            "retighten": True,
        },
        "firm": {
            "squeeze_rad": 0.05,
            "torque_limit": 650,
            "close_speed_rps": 0.4,
            "target_load": 300,
            "contact_effort_threshold": 350,
            "contact_load_delta": None,
            "crush_load": 600,
            "retighten": True,
        },
        "soft": {
            "squeeze_rad": 0.08,
            "torque_limit": 600,
            "close_speed_rps": 0.3,
            "target_load": 150,
            "contact_effort_threshold": 250,
            "contact_load_delta": None,
            "crush_load": 550,
            "retighten": True,
        },
    }
    for preset in presets.values():
        assert preset["close_speed_rps"] <= 0.5  # limits.gripper_velocity_rps default caps it
        assert preset["crush_load"] > preset.get("target_load", 0)
    # the free pre-grasp moves run slower than the arm maximum (2026-10-10: they ran at 0.5 with no override)
    assert cfg["grasp"]["approach_speed_scale"] == 0.3 <= cfg.get("limits", {}).get("arm_max_speed_scale", 0.5)
    # the grasp threshold comes from the profile now (a fixed close_effort_threshold would override every profile)
    assert "close_effort_threshold" not in cfg["grasp"]
    follower = yaml.safe_load(node_entry("lerobot_follower")["config"])
    assert cfg["topics"]["follower_set_register"] == f"/{follower['namespace']}/set_register"
    assert "gripper" in follower["publish_effort_joints"]  # holding_load / target_load read the gripper effort


def test_mcp_server_monitor_block_has_ordered_thresholds() -> None:
    mon = node_config("mcp_server")["monitor"]
    assert (mon["servo_temp_warn_c"], mon["servo_temp_critical_c"]) == (60, 70)
    assert (mon["cpu_temp_warn_c"], mon["cpu_temp_critical_c"]) == (75, 82)
    assert mon["bump_warn_mps2"] < mon["bump_critical_mps2"]
    assert mon["stall_s"] == 1.0 and mon["tilt_warn_deg"] == 10
    assert mon["battery_warn_margin_cell_v"] == 0.2


def test_mcp_server_monitor_topics_match_their_producers() -> None:
    topics = node_config("mcp_server")["topics"]
    assert topics["servo_registers"] == "/follower/servo_registers"
    assert topics["base_servo_registers"] == "/swerve_drive/servo_registers"
    assert topics["imu"] == "/imu/data"
    assert topics["robot_events"] == "/robot_events"
    assert topics["swerve_odom"] == node_config("swerve_controller")["odom_topic"]
    assert topics["rf2o_twist"] == node_config("rf2o_odom_relay")["output_topic"]


def test_mcp_server_perception_topics_match_their_producers() -> None:
    cfg = node_config("mcp_server")
    poi = node_config("poi_store")
    assert cfg["topics"]["local_costmap"] == "/local_costmap/costmap"
    assert cfg["topics"]["plan"] == "/plan"
    assert cfg["topics"]["poi_list"] == poi["list_topic"]
    assert cfg["topics"]["poi_command"] == poi["command_topic"]
    assert cfg["topics"]["poi_result"] == poi["result_topic"]


def test_mcp_server_depth_points_is_the_d435_cloud() -> None:
    """get_map_summary obstacles_3d reads the self-filtered head D435 cloud under topics.depth_points (no ToF key)."""
    topics = node_config("mcp_server")["topics"]
    assert topics["depth_points"] == "/head/d435/points_filtered"
    assert "tof_points" not in topics


def test_mcp_server_perception_config_sections() -> None:
    cfg = node_config("mcp_server")
    assert cfg["objects"]["store_path"] == OBJECTS_FILE
    assert cfg["objects"]["merge_radius_m"] == 0.25
    assert cfg["footprint"] == {"length_m": 0.47, "width_m": 0.386}
    assert cfg["look_around"]["default_captures"] == 4
    assert cfg["look_around"]["clearance_margin_m"] > 0
    assert cfg["topdown"]["default_radius_m"] == 2.5 and cfg["topdown"]["default_px"] == 480
    assert cfg["poi"]["request_timeout_s"] == 3.0


def test_mcp_server_config_renders_guard_rules_in_dry_run() -> None:
    """client.yml renders the five guard rules, every one in dry_run, and the watch settings."""
    cfg = node_config("mcp_server")
    assert cfg["guard_rules"]["enabled"] is True
    assert [r["name"] for r in cfg["guard_rules"]["rules"]] == [
        "base_motion_arm_outside_footprint",
        "base_motion_carrying_without_object_footprint",
        "no_arm_retake_from_human",
        "wrist_roll_needs_lifted_arm",
        "no_stretched_shoulder_overreach",
    ]
    assert {r["mode"] for r in cfg["guard_rules"]["rules"]} == {"dry_run"}
    assert cfg["motion_queue"]["watch_rate_hz"] == 10
    assert cfg["motion_queue"]["watch_max_per_call"] == 8


def mcp_base_tools() -> set[str]:
    """Tools that need the base lease, read from mcp_server tools.py TOOL_SUBSYSTEM without importing the node.

    Returns:
        set[str]: Names of the base-moving tools.
    """
    tree = ast.parse((REPO_ROOT / "nodes/mcp_server/mcp_server/tools.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "TOOL_SUBSYSTEM":
            # Skip `**{...}` spreads (head tools, lease "any"): only literal keys can be base tools.
            return {
                key.value
                for key, value in zip(node.value.keys, node.value.values, strict=True)
                if isinstance(key, ast.Constant) and ast.literal_eval(value) == "base"
            }
    raise AssertionError("TOOL_SUBSYSTEM not found in tools.py")


def test_base_guard_rules_cover_every_base_moving_tool() -> None:
    """A guard rule about base motion must apply to every tool that moves the base (incl. the frontier tools)."""
    base = mcp_base_tools()
    assert {"navigate_to_pose", "go_to_frontier", "look_toward_frontier"} <= base
    for rule in node_config("mcp_server")["guard_rules"]["rules"]:
        tools = set(rule["applies_to"].get("tools", []))
        if tools & base:
            assert base <= tools, f"{rule['name']} misses base tools {sorted(base - tools)}"


def test_mcp_server_config_lazy_steer_states() -> None:
    """The swerve joint states are subscribed only while read (review M5): first-message wait and idle destroy."""
    control = node_config("mcp_server")["control"]
    assert control["steer_states_wait_s"] == 0.5
    assert control["steer_states_idle_s"] == 30.0


def test_mcp_server_readme_documents_perception_tools() -> None:
    text = (NODE_DIR / "README.md").read_text()
    for tool in PERCEPTION_TOOLS:
        assert tool in text, tool
    for needle in (OBJECTS_FILE, "robot-up", "layers_missing", "look_around", "MOTION_TOOLS"):
        assert needle in text, needle


def test_mcp_server_map3d_topics_match_the_topic_contract() -> None:
    """The mcp_server map3d topics are the contract's downlink dests; the map3d block follows the contract frame."""
    sys.path.insert(0, str(REPO_ROOT / "nodes" / "rtabmap_mapping"))
    from map3d_topics import load_map3d_topics

    contract = load_map3d_topics()
    dests = {s.dest for s in contract.downlink}
    cfg = node_config("mcp_server")
    assert cfg["topics"]["map3d_cloud"] == "/map3d/cloud" in dests
    assert cfg["topics"]["map3d_status"] == "/map3d/status" in dests
    assert cfg["map3d"]["frame_id"] == contract.frames.map3d_frame
    assert cfg["map3d"] == {
        "frame_id": "rtabmap_map",
        "voxel_m": 0.05,
        "lidar_plane_height_m": 0.20,
        "robot_height_m": 0.55,
        "max_age_s": 120.0,
        "cloud_wait_s": 12.0,
    }


def test_mcp_server_readme_documents_map3d_tools() -> None:
    text = (NODE_DIR / "README.md").read_text()
    for tool in ("get_3d_map_status", "query_3d_map_at", "get_3d_free_space"):
        assert re.search(rf"^\| `{tool}[`(]", text, re.MULTILINE), tool
    section = text.split("\n## 3D map\n", 1)[1].split("\n## ", 1)[0]
    for needle in ("/map3d/cloud", "10 s", "lidar_2d", "unknown", "map3d:"):
        assert needle in section, needle


def test_mcp_server_readme_documents_monitor_and_events_contract() -> None:
    text = (NODE_DIR / "README.md").read_text()
    for needle in (
        "get_body_state",
        "robot_events_since_last_call",
        "/robot_events",
        "interrupted_by",
        "collision_stop",
        "human_takeover",
        "servo_temp_critical_c",
        "expected",
        "achieved",
    ):
        assert needle in text, needle


def test_web_ui_tab_set() -> None:
    tabs = node_config("web_ui")["tabs"]
    assert [t["id"] for t in tabs] == WEB_UI_TABS
    assert not REMOVED_WEB_UI_TABS & {t["id"] for t in tabs}
    assert tabs[0]["type"] == "map_nav"
    assert tabs[1]["type"] == "agent_chat"
    arm = tabs[3]
    assert arm["type"] == "arm"
    assert arm["label"] == "Arm"
    assert not [k for k in arm if k.startswith("gripper_camera")]
    assert arm["head_camera_topic"] == "/head/d435/color/image_raw/compressed"
    assert "front_camera_topic" not in arm
    assert "topic" not in arm
    head = tabs[4]
    assert head["type"] == "head"
    assert head["label"] == "Head"
    assert head["rgb_camera_topic"] == "/head/d435/color/image_raw/compressed"
    assert head["depth_topic"] == "/head/d435/depth/image_rect_raw"
    assert "fused_depth_topic" not in head
    assert not {"tof_depth_topic", "tof_amplitude_topic", "tof_range_m"} & set(head)
    assert "camera_info_topic" not in head
    assert head["depth_range_m"] == 4.0
    cameras = tabs[5]
    assert cameras["type"] == "camera"
    assert [t["topic"] for t in cameras["topics"]] == [
        "/front_camera/image_raw/compressed",
        "/back_camera/image_raw/compressed",
    ]
    poi = tabs[6]
    assert poi["type"] == "poi"
    assert poi["id"] == "poi"
    assert poi["label"] == "POI"


def test_mcp_server_has_no_gripper_camera() -> None:
    """The gripper camera is retired (user decision 4): no topic, no camera settings, no intrinsics file."""
    cfg = node_config("mcp_server")
    assert "gripper_camera" not in cfg["topics"]
    assert set(cfg["cameras"]) == {"calibration_dir", "head"}
    assert "gripper_camera" not in node_entry("mcp_server")["config"]
    assert not (NODE_DIR / "calibration" / "gripper_camera.yaml").exists()


def test_web_ui_map_tab_uses_contract_fields() -> None:
    """The map tab names the frontend contract fields and the arm Trigger services mcp_server actually serves."""
    tab = node_config("web_ui")["tabs"][0]
    assert not LEGACY_MAP_KEYS & tab.keys(), f"legacy map tab keys: {sorted(LEGACY_MAP_KEYS & tab.keys())}"
    assert tab["base_urdf"] == "robot.urdf"
    assert tab["arm_urdf"] == "so101_arm.urdf"
    assert tab["base_joint_states_topic"] == "/swerve_drive/joint_states"
    assert tab["arm_joint_states_topic"] == "/follower/joint_states"
    assert tab["local_costmap_topic"] == "/local_costmap/costmap"
    topics = node_config("mcp_server").get("topics", {})
    assert tab["arm_home_service"] == topics.get("home_service", "/arm/home") == "/arm/home"
    assert tab["arm_set_home_service"] == topics.get("set_home_service", "/arm/set_home") == "/arm/set_home"
    assert tab.get("tile_cache_dir", TILE_CACHE_DIR) == TILE_CACHE_DIR


def test_web_ui_tile_cache_dir_task_owned_by_node_user() -> None:
    tasks = yaml.safe_load(TILE_CACHE_TASKS.read_text())
    file_task = next(t for t in tasks if "ansible.builtin.file" in t)
    args = file_task["ansible.builtin.file"]
    paths = file_task.get("loop", [args["path"]])
    assert TILE_CACHE_DIR in paths and "/var/cache/web_ui" in paths
    assert args["state"] == "directory"
    assert args["owner"] == "{{ ansible_user }}" and args["group"] == "{{ ansible_user }}"


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml"])
def test_playbooks_create_tile_cache_before_deploying_web_ui(playbook: str) -> None:
    _, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = deploy_index(tasks, "web_ui")
    setup = include_index(tasks, "web_ui_tile_cache_dir.yml")
    assert deploy >= 0, f"{playbook} does not deploy web_ui"
    assert 0 <= setup < deploy, f"{playbook}: the tile cache dir must exist before web_ui starts"


def test_mcp_server_setup_tasks_create_objects_dir_owned_by_node_user() -> None:
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    objects = dirs[OBJECTS_DIR]
    assert objects["state"] == "directory"
    assert objects["owner"] == "{{ ansible_user }}" and objects["group"] == "{{ ansible_user }}"


def test_mcp_server_setup_tasks_create_lookups_dir_owned_by_node_user() -> None:
    """The camera lookup store and its parent (poi_store's directory) exist before mcp_server starts, owned by the
    node user (mcp_server writes, web_ui reads as the same user)."""
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs: dict[str, dict] = {}
    for task in tasks:
        args = task.get("ansible.builtin.file")
        if args is not None:
            for path in task.get("loop", [args["path"]]):
                dirs[path] = args
    for path in (POI_DIR, LOOKUPS_DIR):
        args = dirs[path]
        assert args["state"] == "directory"
        assert args["owner"] == "{{ ansible_user }}" and args["group"] == "{{ ansible_user }}"
        assert args["mode"] == "0755"
    assert list(dirs).index(POI_DIR) < list(dirs).index(LOOKUPS_DIR)


def test_mcp_server_config_lookups_block_is_only_the_store_dir() -> None:
    """client.yml renders only the JPEG directory; poi_store owns the index and retention; it validates."""
    pytest.importorskip("pydantic")
    lookups = node_config("mcp_server")["lookups"]
    assert lookups == {"store_dir": LOOKUPS_DIR}
    sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
    try:
        from mcp_server.config import McpServerConfig

        cfg = McpServerConfig.model_validate(node_config("mcp_server"))
    finally:
        del sys.path[:2]
    assert str(cfg.lookups.store_dir) == LOOKUPS_DIR


def test_client_yml_pins_the_agent_poi_settings() -> None:
    """client.yml renders the agent POI keys (auto_poi, poi_review_followup, poi_hint_radius_m)."""
    pytest.importorskip("pydantic")
    sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
    try:
        from mcp_server.config import AutoPoiSettings, McpServerConfig
    finally:
        del sys.path[:2]
    raw = node_config("mcp_server")
    assert raw["auto_poi"] == {
        "enabled": True,
        "merge_radius_m": 0.5,
        "max_pose_age_s": 2.0,
        "event_types": ["collision_stop", "nav_abort"],
        "hint_after_images": 3,
    }
    assert McpServerConfig.model_validate(raw).auto_poi == AutoPoiSettings(**raw["auto_poi"])
    agent = node_config("claude_agent")
    assert "agent_poi_reset" not in agent
    assert agent["poi_review_followup"] is True
    assert agent["poi_hint_radius_m"] == 0.5


def test_mcp_server_config_validates_with_frontiers_block_at_defaults() -> None:
    """The client.yml mcp_server config validates with McpServerConfig; `frontiers:` lists every default."""
    pytest.importorskip("pydantic")
    sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
    try:
        from mcp_server.config import FrontierSettings, McpServerConfig
    finally:
        del sys.path[:2]
    raw = node_config("mcp_server")
    cfg = McpServerConfig.model_validate(raw)
    defaults = FrontierSettings().model_dump()
    assert raw["frontiers"] == defaults
    assert cfg.frontiers == FrontierSettings()
    block = node_entry("mcp_server")["config"].split("\nfrontiers:\n", 1)[1]
    block = re.split(r"^[a-z_]+:", block, maxsplit=1, flags=re.MULTILINE)[0]
    commented = [line for line in block.splitlines() if re.match(r"^  [a-z_]+:", line)]
    assert len(commented) == len(defaults)
    assert all("#" in line for line in commented), "every frontiers key carries a short comment"


def test_mcp_server_config_pins_auto_poi_block() -> None:
    """The client.yml mcp_server `auto_poi:` block pins the stuck-point marker and validates as AutoPoiSettings."""
    pytest.importorskip("pydantic")
    sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
    try:
        from mcp_server.config import AutoPoiSettings, McpServerConfig
    finally:
        del sys.path[:2]
    raw = node_config("mcp_server")
    assert raw["auto_poi"] == {
        "enabled": True,
        "merge_radius_m": 0.5,
        "max_pose_age_s": 2.0,
        "event_types": ["collision_stop", "nav_abort"],
        "hint_after_images": 3,
    }
    cfg = McpServerConfig.model_validate(raw)
    assert cfg.auto_poi == AutoPoiSettings.model_validate(raw["auto_poi"])
    assert set(raw["auto_poi"]) == set(AutoPoiSettings.model_fields), "every AutoPoiSettings key is pinned"


def test_mcp_server_setup_tasks_create_token_and_arm_dir() -> None:
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    assert dirs[CONFIG_DIR]["state"] == "directory"
    arm = dirs[ARM_DIR]
    assert arm["state"] == "directory"
    assert arm["owner"] == "{{ ansible_user }}" and arm["group"] == "{{ ansible_user }}"
    token_task = next(t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == TOKEN_FILE)
    copy = token_task["ansible.builtin.copy"]
    assert copy["force"] is False, "an existing token must never be overwritten"
    assert copy["mode"] == "0640", "group-readable so claude_agent (group mcp-token) can use it; never world-readable"
    assert copy["owner"] == "{{ ansible_user }}" and copy["group"] == "mcp-token"
    assert token_task["no_log"] is True
    content = copy["content"]
    assert content.startswith("MCP_SERVER_TOKEN=")
    assert "lookup('ansible.builtin.password', '/dev/null'" in content
    assert "length=48" in content


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml"])
def test_playbooks_run_setup_before_deploying_mcp_server(playbook: str) -> None:
    pre, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = deploy_index(tasks, "mcp_server")
    setup = include_index(tasks, "mcp_server_setup.yml")
    assert deploy >= 0, f"{playbook} does not deploy mcp_server"
    assert 0 <= setup < deploy, f"{playbook}: token/arm dir setup must run before the deploy"


def test_unit_template_renders_environment_file_only_when_set() -> None:
    jinja2 = pytest.importorskip("jinja2")
    template = jinja2.Template(SERVICE_TEMPLATE.read_text())
    base = {"node_name": "mcp_server", "ansible_user": "ubuntu", "node_env": ["MCP_SERVER_CONFIG=x"]}
    with_file = template.render(**base, node_environment_file=TOKEN_FILE)
    assert f"EnvironmentFile={TOKEN_FILE}\n" in with_file
    assert with_file.index("EnvironmentFile=") < with_file.index("ExecStart=")
    assert "EnvironmentFile" not in template.render(**base)
    assert "EnvironmentFile" not in template.render(**base, node_environment_file="")


def test_resolve_and_deploy_passes_environment_file() -> None:
    tasks = yaml.safe_load(RESOLVE_TASKS.read_text())
    role_vars = next(t for t in tasks if "ansible.builtin.include_role" in t)["vars"]
    assert "environment_file" in role_vars["node_environment_file"]
    assert "default('')" in role_vars["node_environment_file"]


def test_token_file_never_in_repo() -> None:
    tracked = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.splitlines()
    assert not [p for p in tracked if Path(p).name == "token" or p.endswith(".token")], "token files tracked in git"
    literal = re.compile(r"(MCP_SERVER_TOKEN|ROBOT_MCP_TOKEN)=['\"]?[A-Za-z0-9]{16,}")
    for rel in tracked:
        path = REPO_ROOT / rel
        if rel.startswith(("ansible/", "scripts/", "nodes/mcp_server/", "tests/")) or rel == ".mcp.json":
            if path.is_file() and path.suffix in {".yml", ".yaml", ".sh", ".json", ".py", ".md", ".j2", ""}:
                assert not literal.search(path.read_text(errors="ignore")), f"literal token in {rel}"
    assert "${ROBOT_MCP_TOKEN}" in MCP_JSON.read_text()


def test_mcp_json_registers_robot_server() -> None:
    doc = json.loads(MCP_JSON.read_text())
    robot = doc["mcpServers"]["robot"]
    assert robot["type"] == "http"
    assert robot["url"] == MCP_URL
    assert robot["headers"] == {"Authorization": "Bearer ${ROBOT_MCP_TOKEN}"}
    assert str(PORT) in robot["url"] and node_config("mcp_server")["server"]["path"] in robot["url"]


def test_robot_mcp_token_script() -> None:
    assert TOKEN_SCRIPT.is_file() and os.access(TOKEN_SCRIPT, os.X_OK)
    text = TOKEN_SCRIPT.read_text()
    assert text.startswith("#!/usr/bin/env bash")
    assert "set -euo pipefail" in text
    assert "ssh" in text and TOKEN_FILE in text
    assert "export ROBOT_MCP_TOKEN=" in text
    result = subprocess.run(["bash", "-n", str(TOKEN_SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_robot_mcp_token_script_prints_export_line(tmp_path: Path) -> None:
    fake_ssh = tmp_path / "ssh"
    fake_ssh.write_text("#!/usr/bin/env bash\necho 'MCP_SERVER_TOKEN=abc123XYZ'\n")
    fake_ssh.chmod(0o755)
    env = os.environ | {"PATH": f"{tmp_path}:{os.environ['PATH']}"}
    result = subprocess.run(["bash", str(TOKEN_SCRIPT)], capture_output=True, text=True, env=env)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "export ROBOT_MCP_TOKEN='abc123XYZ'"


def test_mcp_server_node_package_layout() -> None:
    pyproject = tomllib.loads((NODE_DIR / "pyproject.toml").read_text())
    assert any(dep.startswith("mcp==") for dep in pyproject["project"]["dependencies"])
    assert pyproject["tool"]["hatch"]["build"]["targets"]["wheel"]["packages"] == ["mcp_server"]
    assert (NODE_DIR / "uv.lock").is_file()
    assert (NODE_DIR / "mcp_server" / "__main__.py").is_file()
    assert (NODE_DIR / "README.md").read_text().count("claude mcp add") >= 1


def test_mcp_server_listed_in_nodes_readme_index() -> None:
    """nodes/README.md Layout index must carry an entry for the mcp_server node."""
    layout = NODES_README.read_text().split("## Layout", 1)[1]
    assert re.search(r"^- \*\*mcp_server/\*\* .*MCP", layout, re.MULTILINE)


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### test_mcp_server_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"


def test_mcp_server_nav_tolerances_match_nav2_goal_checker() -> None:
    """The precision the agent is told must equal the Nav2 goal checker (0.01 m, 0.035 rad ~ 2 deg)."""
    nav = node_config("mcp_server")["nav"]
    checker = yaml.safe_load(NAV2_PARAMS.read_text())["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert nav["goal_xy_tolerance_m"] == pytest.approx(checker["xy_goal_tolerance"])
    assert nav["goal_yaw_tolerance_deg"] == pytest.approx(math.degrees(checker["yaw_goal_tolerance"]), abs=0.05)


def test_claude_agent_nav_tolerances_match_mcp_server() -> None:
    agent = node_config("claude_agent")
    nav = node_config("mcp_server")["nav"]
    assert agent["nav_goal_xy_tolerance_cm"] == pytest.approx(nav["goal_xy_tolerance_m"] * 100)
    assert agent["nav_goal_yaw_tolerance_deg"] == pytest.approx(nav["goal_yaw_tolerance_deg"])
    assert agent["nav_intermediate_xy_tolerance_cm"] == pytest.approx(nav["intermediate_xy_tolerance_m"] * 100)
    assert agent["nav_intermediate_yaw_tolerance_deg"] == pytest.approx(nav["intermediate_yaw_tolerance_deg"])


def test_mcp_server_head_camera_is_the_head_camera_stream() -> None:
    """mcp_server's `head` camera reads the head D435 colour JPEG stream (CompressedImage)."""
    topics = node_config("mcp_server").get("topics", {})
    assert topics["head_camera"] == "/head/d435/color/image_raw/compressed"
    assert "realsense_camera" not in topics


def test_mcp_server_topics_name_the_csi_cameras() -> None:
    """get_camera_image 'front' and 'back' read the body-fixed CSI Camera Module 3 compressed streams."""
    topics = node_config("mcp_server")["topics"]
    assert topics["front_camera"] == "/front_camera/image_raw/compressed"
    assert topics["back_camera"] == "/back_camera/image_raw/compressed"


def test_mcp_server_states_no_hardcoded_front_camera_size() -> None:
    """Tool descriptions name no size; the mcp_server README gives 640x360 / 384x216 for the head D435 colour."""
    tools_source = (REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "tools.py").read_text()
    assert "overhead camera (640x480)" not in tools_source
    assert "front of the robot, 640x480" not in tools_source
    mcp_readme = (REPO_ROOT / "nodes" / "mcp_server" / "README.md").read_text()
    assert "640x480 for both" not in mcp_readme
    assert "640x360" in mcp_readme and "384x216" in mcp_readme
    assert "pi_cameras front Camera Module 3 streaming 640x480" not in mcp_readme


def test_mcp_server_get_camera_image_docs_name_the_front_d435_colour() -> None:
    """The get_camera_image row names the head D435 colour at 640x360; no IMX219, 480x360 or 640x480 front size."""
    mcp_dir = REPO_ROOT / "nodes" / "mcp_server"
    texts = {str(p): p.read_text() for p in [mcp_dir / "README.md", *(mcp_dir / "mcp_server").glob("*.py")]}
    for name, text in texts.items():
        assert "IMX219" not in text, name
        assert "480x360" not in text, name
    readme_lines = texts[str(mcp_dir / "README.md")].splitlines()
    row = next(line for line in readme_lines if line.startswith("| `get_camera_image("))
    assert "D435" in row and "640x360" in row and "384x216" in row
    assert "head_camera" in row and "/head/d435/color/image_raw/compressed" in row
    assert "topics.front_camera" in row and "topics.back_camera" in row and "1152x648" in row
    assert "front`: 640x480" not in row and "640x480 at 10 fps" not in row and "overhead" not in row
    assert "640x480" not in row
    assert "defaults to 384x288" not in texts[str(mcp_dir / "README.md")]


def test_mcp_gripper_closed_target_is_inside_the_follower_gripper_command_range() -> None:
    """Direct (autonomy) gripper commands are clamped to the follower servo's command range: -0.165 rad (steps
    2048 + rad * 4096 / 2pi, not inverted) must lie above command_min_steps, and the README documents the steps."""
    closed_rad = -0.165
    follower = node_config("lerobot_follower")
    gripper = next(j for j in follower["joint_names"] if j["name"] == "gripper")
    assert not gripper.get("inverted", False)
    steps = round(closed_rad * 4096 / (2 * math.pi) + 2048)
    assert steps == 1940
    assert gripper["command_min_steps"] <= steps
    assert "direct_command_sources" in follower and "autonomy" in follower["direct_command_sources"]
    readme = (NODE_DIR / "README.md").read_text()
    assert "-0.165" in readme and "1940" in readme and "command_min_steps" in readme


def test_shoulder_lift_upper_limit_allows_reaching_below_the_floor() -> None:
    """2026-10-08 test: the shoulder reached 1.87 rad without collision but cannot lift the stretched arm beyond about
    1.85 rad; 1.9 rad (1.85 usable after the margin) lets a folded arm reach about 5 cm below floor level."""
    overrides = node_config("mcp_server")["arm"]["joint_limit_overrides_rad"]
    assert overrides == {"shoulder_lift": [-1.745, 1.9]}


def test_mcp_server_sag_compensation_uses_the_held_out_fit() -> None:
    """Gravity sag compensation is enabled with the gains of docs/calibration/2026-10-10/sag_fit_results.json (held-out
    residual reduction above 50% for shoulder_lift and elbow_flex); wrist_flex stays uncompensated (its data cover
    only a sliver of its torque range) and the README documents the feature and how to re-fit it."""
    sag = node_config("mcp_server")["arm"]["sag_compensation"]
    fit = json.loads((REPO_ROOT / "docs" / "calibration" / "2026-10-10" / "sag_fit_results.json").read_text())
    assert sag["enabled"] is True
    assert set(sag["k"]) == {"shoulder_lift", "elbow_flex"}
    for joint, gain in sag["k"].items():
        assert gain == pytest.approx(fit["k"][joint], abs=5e-4)
        assert fit["held_out_pooled"]["reduction"][joint] > 0.5
    assert sag["k_lowering"] == {"shoulder_lift": pytest.approx(fit["k_lowering"]["shoulder_lift"], abs=5e-4)}
    assert 0.0 < sag["max_rad"] <= 0.12
    assert (REPO_ROOT / "docs" / "calibration" / "2026-10-10" / "sag_fit.md").is_file()
    readme = (NODE_DIR / "README.md").read_text()
    assert "Gravity sag compensation" in readme and "arm_settle" in readme and "sag_fit.py" in readme


def test_mcp_server_unit_pins_the_allocator() -> None:
    """mcp_server returns freed numpy temporaries to the OS (glibc kept them; RSS grew 185 -> 383 MB on 2026-10-11)."""
    env = yaml.safe_load(CLIENT_VARS.read_text())["ros2_node_type_defaults"]["mcp_server"]["env"]
    assert "MALLOC_MMAP_THRESHOLD_=1048576" in env
    assert "MALLOC_ARENA_MAX=2" in env
    assert "NUMPY_MADVISE_HUGEPAGE=0" in env


# Load-rise contact is off on every preset: on the robot (2026-10-11) empty closes rose 35-124 load units above the
# free-closing baseline at our closing speeds, so no delta separates closing from touching (the paper closes at 5 mm/s).
GRIP_PRESET_CONTACT_LOAD_DELTA = None


def validated_mcp_config():
    """Load the client.yml mcp_server config through the node's McpServerConfig (unknown keys are rejected).

    Returns:
        McpServerConfig: The validated config.
    """
    for path in (REPO_ROOT / "nodes" / "mcp_server", SHARED_DIR):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from mcp_server.config import McpServerConfig

    return McpServerConfig.model_validate(node_config("mcp_server"))


def test_mcp_server_client_config_validates_and_reads_follower_currents() -> None:
    """client.yml mcp_server passes McpServerConfig and subscribes the follower's gripper current topic."""
    cfg = validated_mcp_config()
    assert cfg.topics.follower_joint_currents == "/follower/joint_currents"
    assert node_config("mcp_server")["topics"]["follower_joint_currents"] == "/follower/joint_currents"


def test_mcp_server_grip_profiles_set_delta_contact_keys() -> None:
    """client.yml sets the delta-contact sampling keys, the current vote key and a contact_load_delta per preset."""
    raw = node_config("mcp_server")["grip_profiles"]
    grips = validated_mcp_config().grip_profiles
    assert raw["contact_baseline_samples"] == grips.contact_baseline_samples == 10
    assert raw["contact_persist_samples"] == grips.contact_persist_samples == 3
    assert "current_contact_delta_a" in raw  # null until tuned on the robot; validated gt 0 when set
    assert grips.current_contact_delta_a is None or grips.current_contact_delta_a > 0
    assert set(raw["presets"]) == set(grips.presets)
    for name, preset in grips.presets.items():
        assert preset.contact_load_delta == GRIP_PRESET_CONTACT_LOAD_DELTA, name
        assert raw["presets"][name]["contact_load_delta"] == preset.contact_load_delta, name


def test_mcp_server_grip_presets_keep_absolute_backstops() -> None:
    """Every preset keeps its absolute contact_effort_threshold and target_load (delta contact is off until retuned)."""
    for name, preset in validated_mcp_config().grip_profiles.presets.items():
        assert preset.contact_effort_threshold > (preset.contact_load_delta or 0), name
        assert preset.target_load > 0, name


def test_mcp_server_grasp_check_block() -> None:
    """client.yml renders the grasp_check block with the jaw-crop settings."""
    raw = node_config("mcp_server")["grasp_check"]
    assert raw == {"crop_depth_m": 0.045, "crop_margin_px": 24, "crop_min_px": 64, "image_px": 640}
    grasp = validated_mcp_config().grasp_check
    assert (grasp.crop_depth_m, grasp.crop_margin_px, grasp.crop_min_px, grasp.image_px) == (0.045, 24, 64, 640)


def test_follower_current_read_is_rolled_back() -> None:
    """The longer state read (Present_Current) failed the 95 Hz gate on the robot (93.4 Hz, p99 18 ms, 2026-10-11), so
    publish_current_joints is empty (the rollback the design defines); the grasp band runs without the current vote."""
    assert node_config("lerobot_follower")["publish_current_joints"] == []


def test_claude_agent_confirm_grasp_is_a_sensor_tool() -> None:
    """confirm_grasp records a verdict without moving, so it is a sensor tool and not an effector."""
    cfg = node_config("claude_agent")
    assert "confirm_grasp" in cfg["sensor_tools"]
    assert "confirm_grasp" not in cfg["effector_tools"]


def test_mcp_server_readme_lookups_example_has_only_store_dir() -> None:
    """The README config reference must not list the removed lookups retention keys."""
    text = (NODE_DIR / "README.md").read_text()
    block = text.split("\nlookups:", 1)[1].split("\ncontrol:", 1)[0]
    assert "store_dir" in block
    for key in ("max_records", "max_bytes", "max_age_days"):
        assert key not in block, key


def test_client_yml_lookups_comment_points_to_an_existing_readme_section() -> None:
    """Every quoted README section named in the client.yml lookups comment exists as a heading."""
    lines = CLIENT_VARS.read_text().splitlines()
    idx = next(i for i, line in enumerate(lines) if line.strip() == "lookups:" and "store_dir" in lines[i + 1])
    comment = "\n".join(line for line in lines[idx - 4 : idx] if line.strip().startswith("#"))
    sections = re.findall(r'README\.md, "([^"]+)"', comment)
    assert sections
    readme = (NODE_DIR / "README.md").read_text()
    for section in sections:
        assert f"\n## {section}\n" in readme, section


REACH_MAP_FILE = NODE_DIR / "mcp_server" / "data" / "reach_map.npz"
REACH_MAP_SCRIPT = REPO_ROOT / "scripts" / "gen_reach_map.py"
REACH_MAP_KEY_PROGRAM = (
    "from pathlib import Path; from mcp_server.reach_map import checker_from_config, config_from_client_yml, "
    "reach_map_key; c = config_from_client_yml(Path('ansible/group_vars/client.yml')); "
    "print(reach_map_key(checker_from_config(c), c.reach_map))"
)


def mcp_server_yml_config() -> dict:
    """The parsed mcp_server config block of client.yml.

    Returns:
        dict: The node's config.
    """
    doc = yaml.safe_load(CLIENT_VARS.read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    return yaml.safe_load(node["config"])


def load_reach_map_meta() -> dict:
    """The meta record of the committed reach map (fails naming the generator when the file is missing).

    Returns:
        dict: meta JSON of reach_map.npz.
    """
    assert REACH_MAP_FILE.is_file(), f"{REACH_MAP_FILE} is missing: run scripts/gen_reach_map.py"
    with np.load(REACH_MAP_FILE, allow_pickle=False) as data:
        return json.loads(str(data["meta"]))


def test_reach_map_artifact_key_matches_deployed_config() -> None:
    """The committed reach map was built for the inputs of client.yml's mcp_server config ."""
    meta = load_reach_map_meta()
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}
    result = subprocess.run(
        ["uv", "run", "--quiet", "--frozen", "--project", str(NODE_DIR), "python", "-c", REACH_MAP_KEY_PROGRAM],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr[-2000:]
    assert meta["key"] == result.stdout.strip(), (
        "nodes/mcp_server/mcp_server/data/reach_map.npz is stale for ansible/group_vars/client.yml (body, arm mount, "
        "joint limits/offsets, margins or reach_map grid changed): regenerate it with scripts/gen_reach_map.py"
    )


def test_reach_map_artifact_grid_equals_configured_grid() -> None:
    """The artifact's meta grid equals the explicit reach_map: block of client.yml."""
    config = mcp_server_yml_config()["reach_map"]
    meta = load_reach_map_meta()
    grid = meta["grid"]
    assert grid["heading_step_deg"] == config["heading_step_deg"]
    assert grid["r_step_m"] == config["r_step_m"]
    assert grid["z_step_m"] == config["z_step_m"]
    assert len(grid["r_m"]) == round(config["r_max_m"] / config["r_step_m"])
    assert len(grid["z_m"]) == round(config["z_max_m"] / config["z_step_m"])
    assert meta["planar_samples"] == config["planar_samples"]
    assert meta["samples_per_cell"] == config["samples_per_cell"]


def test_reach_map_block_is_explicit_and_default_file() -> None:
    """client.yml spells out every reach_map grid field, names the generator and leaves the file at its default."""
    config = mcp_server_yml_config()["reach_map"]
    assert set(config) == {
        "heading_step_deg",
        "r_step_m",
        "r_max_m",
        "z_step_m",
        "z_max_m",
        "planar_samples",
        "samples_per_cell",
    }
    assert "scripts/gen_reach_map.py" in CLIENT_VARS.read_text()


def test_gen_reach_map_check_passes_on_committed_artifact() -> None:
    """scripts/gen_reach_map.py --check exits 0 for the committed artifact."""
    result = subprocess.run(
        ["uv", "run", "python", str(REACH_MAP_SCRIPT), "--check"], cwd=REPO_ROOT, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_mcp_server_maps_section_validates_and_matches_map_manager_topics() -> None:
    """client.yml mcp_server `maps:` validates and uses the topics map_manager is configured with."""
    pytest.importorskip("pydantic")
    sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
    try:
        from mcp_server.config import McpServerConfig

        raw = node_config("mcp_server")
        cfg = McpServerConfig.model_validate(raw)
    finally:
        del sys.path[:2]
    manager = node_config("map_manager")
    assert raw["maps"]["command"] == manager["command_topic"]
    assert raw["maps"]["result"] == manager["result_topic"]
    assert raw["maps"]["status"] == manager["status_topic"]
    assert (cfg.maps.command, cfg.maps.result, cfg.maps.status) == (
        manager["command_topic"],
        manager["result_topic"],
        manager["status_topic"],
    )
    for key in ("save_timeout_s", "load_timeout_s", "delete_timeout_s"):
        assert raw["maps"][key] > 0


def test_mcp_server_readme_documents_the_named_map_tools() -> None:
    """The README tool table lists the four map tools (and delete_map) and has a `## Named maps` section."""
    text = (NODE_DIR / "README.md").read_text()
    rows = [line for line in text.splitlines() if line.startswith("| `")]
    for tool in ("save_map", "list_maps", "load_map", "clear_pois", "delete_map"):
        assert any(row.startswith(f"| `{tool}") for row in rows), tool
    assert "\n## Named maps\n" in text
