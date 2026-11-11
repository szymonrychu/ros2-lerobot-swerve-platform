"""Servo calibration keys in Ansible group_vars: bridge blocks, web_ui groups and the mcp_server record path agree.

The bridge `servo_calibration` blocks (lerobot_follower, base_servos primary, head extra group), the web_ui
servo_groups flags and mcp_server's `servo_calibration_record` must name the same groups, joints and record files.
"""

import math
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
BASE_STATE_DIR_TASKS = ANSIBLE_DIR / "playbooks" / "tasks" / "base_servos_state_dir.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

FEETECH_TYPE = "feetech_servos"
HALF_PI = 1.5707963
STEER_JOINTS = ["fl_steer", "fr_steer", "rl_steer", "rr_steer"]
DRIVE_JOINTS = ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]
ARM_TARGETS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -HALF_PI,
    "elbow_flex": HALF_PI,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
HEAD_TARGETS = {"head_pan": 0.0, "head_tilt": 0.0}
STEER_TRAVEL = [-1.5708, 1.5708]
HEAD_TRAVEL = {"head_pan": [-1.5708, 1.5708], "head_tilt": [-0.5236, 0.7854]}
ARM_RECORD_DIR = "/var/lib/ros2/arm"
BASE_RECORD_DIR = "/var/lib/ros2/base_servos"
MCP_RECORD = "/var/lib/ros2/arm/servo_calibration-follower-latest.json"
SET_HOME_SERVICE = "/arm/set_home_from_calibration"
WEB_GROUP_TO_NAMESPACE = {"arm": "follower", "base": "swerve_drive", "head": "head"}


def node_config(name: str) -> dict:
    """Parse the config block of one client ros2_nodes entry.

    Args:
        name (str): Entry name.

    Returns:
        dict: The entry's config YAML.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def calibration_groups() -> dict[str, dict]:
    """Every group of the client feetech bridges, by namespace.

    Returns:
        dict[str, dict]: namespace -> group config (primary config or extra group entry).
    """
    groups: dict[str, dict] = {}
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    for entry in nodes:
        if entry["node_type"] != FEETECH_TYPE:
            continue
        cfg = yaml.safe_load(entry["config"])
        groups[cfg["namespace"]] = cfg
        for extra in cfg.get("extra_groups") or []:
            groups[extra["namespace"]] = extra
    return groups


def test_every_calibration_group_has_a_block() -> None:
    groups = calibration_groups()
    assert set(groups) == set(WEB_GROUP_TO_NAMESPACE.values())
    for namespace, group in groups.items():
        block = group["servo_calibration"]
        position = {j["name"] for j in group["joint_names"] if j.get("mode", "position") == "position"}
        expected = position - set(block.get("offset_exclude_joints", []))
        assert set(block["home_reference_rad"]) == expected, namespace
        assert block["record_dir"].startswith("/var/lib/ros2/"), namespace


def test_arm_targets_match_spec() -> None:
    arm = calibration_groups()["follower"]["servo_calibration"]
    assert set(arm["home_reference_rad"]) == set(ARM_TARGETS)
    for joint, target in ARM_TARGETS.items():
        assert math.isclose(arm["home_reference_rad"][joint], target, abs_tol=1e-7), joint
    assert arm["full_turn_joints"] == ["wrist_roll"]
    assert arm["record_dir"] == ARM_RECORD_DIR
    groups = calibration_groups()
    assert groups["head"]["servo_calibration"]["home_reference_rad"] == HEAD_TARGETS
    assert groups["head"]["servo_calibration"]["required_travel_rad"] == HEAD_TRAVEL
    base = groups["swerve_drive"]["servo_calibration"]
    assert base["home_reference_rad"] == dict.fromkeys(STEER_JOINTS, 0.0)
    assert base["required_travel_rad"] == dict.fromkeys(STEER_JOINTS, STEER_TRAVEL)
    assert base["record_dir"] == BASE_RECORD_DIR
    assert groups["head"]["servo_calibration"]["record_dir"] == BASE_RECORD_DIR
    for namespace in ("swerve_drive", "head"):
        assert "full_turn_joints" not in groups[namespace]["servo_calibration"]


def test_gripper_excluded_from_offset() -> None:
    arm = calibration_groups()["follower"]["servo_calibration"]
    assert arm["offset_exclude_joints"] == ["gripper"]
    assert "gripper" not in arm["home_reference_rad"]


def test_drive_servos_not_in_calibration() -> None:
    block = calibration_groups()["swerve_drive"]["servo_calibration"]
    for key in ("home_reference_rad", "required_travel_rad"):
        assert not set(block[key]) & set(DRIVE_JOINTS)
    assert set(block["home_reference_rad"]) == set(STEER_JOINTS)


def test_web_ui_groups_offer_servo_calibration_for_each_block() -> None:
    groups = calibration_groups()
    web = {g["id"]: g for g in node_config("web_ui")["calibration"]["servo_groups"]}
    assert set(web) == set(WEB_GROUP_TO_NAMESPACE)
    for group_id, namespace in WEB_GROUP_TO_NAMESPACE.items():
        assert web[group_id]["namespace"] == namespace
        assert "servo_calibration" in groups[namespace]
        assert web[group_id]["servo_calibration"] is True
    assert web["arm"]["set_home_from_calibration_service"] == SET_HOME_SERVICE
    assert "set_home_from_calibration_service" not in web["base"]
    assert "set_home_from_calibration_service" not in web["head"]


def test_mcp_record_path_matches_arm_record_dir() -> None:
    arm = calibration_groups()["follower"]["servo_calibration"]
    record = node_config("mcp_server")["arm"]["servo_calibration_record"]
    assert record == MCP_RECORD
    assert record == f"{arm['record_dir']}/servo_calibration-follower-latest.json"


def test_base_servos_record_dir_is_created_before_base_servos_deploys() -> None:
    tasks = yaml.safe_load(BASE_STATE_DIR_TASKS.read_text())
    assert BASE_RECORD_DIR in [t.get("ansible.builtin.file", {}).get("path") for t in tasks]
    assert "tasks/base_servos_state_dir.yml" in CLIENT_PLAYBOOK.read_text()


def test_tests_readme_lists_this_file() -> None:
    assert "### `test_servo_calibration_config.py`" in TESTS_README.read_text()
