"""Servo torque-off and neutral keys in Ansible group_vars: the client bridges, command sources and web UI agree.

One client deploy turns the feature on. The web_ui servo groups, the torque state topics of filter_node,
swerve_controller and mcp_server, the arm stale marker and the command topics must all name the same bridge groups.
The leader bridge on the server keeps its zero (no neutral keys).
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
MCP_SETUP = ANSIBLE_DIR / "playbooks" / "tasks" / "mcp_server_setup.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

FEETECH_TYPE = "feetech_servos"
FOLLOWER = "lerobot_follower"
BASE = "base_servos"
LEADER = "lerobot_leader"
STALE_MARKER = "/var/lib/ros2/arm/calibration_stale.json"
ARM_DIR = "/var/lib/ros2/arm"
BASE_NEUTRAL_JOINTS = ["fl_steer", "fr_steer", "rl_steer", "rr_steer"]
GROUP_IDS = {"arm", "base", "head"}


def nodes(path: Path) -> list[dict]:
    """Return the ros2_nodes entries of a group_vars file.

    Args:
        path (Path): group_vars YAML file.

    Returns:
        list[dict]: ros2_nodes entries.
    """
    return yaml.safe_load(path.read_text())["ros2_nodes"]


def node_config(name: str, path: Path = CLIENT_VARS) -> dict:
    """Parse the config block of one ros2_nodes entry.

    Args:
        name (str): Entry name.
        path (Path): group_vars file.

    Returns:
        dict: The entry's config YAML.
    """
    return yaml.safe_load(next(n for n in nodes(path) if n["name"] == name)["config"])


def bridge_groups() -> dict[str, dict]:
    """Every group of every client feetech bridge, by namespace.

    Returns:
        dict[str, dict]: namespace -> {node, config (group keys), joints, primary}.
    """
    groups: dict[str, dict] = {}
    for entry in nodes(CLIENT_VARS):
        if entry["node_type"] != FEETECH_TYPE:
            continue
        cfg = yaml.safe_load(entry["config"])
        groups[cfg["namespace"]] = {"node": entry["name"], "config": cfg, "joints": cfg["joint_names"], "primary": True}
        for extra in cfg.get("extra_groups") or []:
            groups[extra["namespace"]] = {
                "node": entry["name"],
                "config": extra,
                "joints": extra["joint_names"],
                "primary": False,
            }
    return groups


def web_ui_groups() -> dict[str, dict]:
    """The web_ui calibration.servo_groups entries by id."""
    return {g["id"]: g for g in node_config("web_ui")["calibration"]["servo_groups"]}


def tab_value(key: str) -> str:
    """The value of a web_ui tab key (e.g. arm_command_topic).

    Args:
        key (str): Tab config key.

    Returns:
        str: The single configured value.
    """
    values = [t[key] for t in node_config("web_ui")["tabs"] if key in t]
    assert len(values) == 1, f"web_ui tabs need exactly one {key}"
    return values[0]


def test_web_ui_groups_are_bridge_groups_and_neutral_matches() -> None:
    groups = bridge_groups()
    web = web_ui_groups()
    assert set(web) == GROUP_IDS
    for gid, group in web.items():
        bridge = groups[group["namespace"]]
        assert bridge["config"].get("neutral_enabled", False) is group["neutral"] is True, gid
    assert web["arm"]["namespace"] == "follower" and groups["follower"]["node"] == FOLLOWER
    assert web["base"]["namespace"] == "swerve_drive" and groups["swerve_drive"]["node"] == BASE
    assert web["head"]["namespace"] == "head" and groups["head"]["node"] == BASE


def test_base_neutral_joints_are_the_four_steering_servos() -> None:
    base = bridge_groups()["swerve_drive"]
    excluded = set(base["config"].get("neutral_exclude_joints") or [])
    neutral = [
        j["name"] for j in base["joints"] if j.get("mode", "position") == "position" and j["name"] not in excluded
    ]
    assert neutral == BASE_NEUTRAL_JOINTS
    assert not any(j.get("mode") == "velocity" and j["name"] in neutral for j in base["joints"])


def test_follower_neutral_excludes_only_the_gripper() -> None:
    cfg = node_config(FOLLOWER)
    assert cfg["neutral_enabled"] is True
    assert cfg["neutral_exclude_joints"] == ["gripper"]
    assert "gripper" in [j["name"] for j in cfg["joint_names"]]


def test_stale_marker_path_is_identical_everywhere() -> None:
    follower = node_config(FOLLOWER)["neutral_stale_marker"]
    mcp = node_config("mcp_server")["arm"]["calibration_stale_marker"]
    assert follower == mcp == web_ui_groups()["arm"]["stale_marker"] == STALE_MARKER
    assert STALE_MARKER.startswith(ARM_DIR + "/")


def test_torque_state_topics_match_bridge_groups() -> None:
    namespaces = bridge_groups()
    assert node_config("filter_node")["follower_torque_state_topic"] == "/follower/torque_state"
    assert node_config("swerve_controller")["torque_state_topic"] == "/swerve_drive/torque_state"
    mcp = node_config("mcp_server")
    assert mcp["topics"]["follower_torque_state"] == "/follower/torque_state"
    assert mcp["topics"]["base_torque_state"] == "/swerve_drive/torque_state"
    assert mcp["topics"]["head_torque_state"] == "/head/torque_state"
    assert mcp["timeouts"]["servo_torque_stale_s"] > 0
    topics = [
        node_config("filter_node")["follower_torque_state_topic"],
        node_config("swerve_controller")["torque_state_topic"],
        *(mcp["topics"][k] for k in ("follower_torque_state", "base_torque_state", "head_torque_state")),
    ]
    for topic in topics:
        ns = topic.split("/")[1]
        assert ns in namespaces and topic == f"/{ns}/torque_state"
    for entry in (FOLLOWER, BASE):
        assert node_config(entry)["torque_state_interval_s"] == 1.0


def test_leader_bridge_has_no_neutral_keys() -> None:
    leader = node_config(LEADER, SERVER_VARS)
    assert not [k for k in leader if k.startswith("neutral_")]
    for extra in leader.get("extra_groups") or []:
        assert not [k for k in extra if k.startswith("neutral_")]


def test_arm_command_topic_is_on_the_arm_tab() -> None:
    holders = [t for t in node_config("web_ui")["tabs"] if "arm_command_topic" in t]
    assert [t["type"] for t in holders] == ["arm"]


def test_web_ui_command_topics_match_configured_topics() -> None:
    web = web_ui_groups()
    assert web["arm"]["command_topics"] == [tab_value("arm_command_topic")]
    assert web["head"]["command_topics"] == [tab_value("head_command_topic")]
    assert web["base"]["command_topics"] == [tab_value("goal_topic")]


def test_arm_state_directory_created_on_client() -> None:
    tasks = yaml.safe_load(MCP_SETUP.read_text())
    assert ARM_DIR in [t.get("ansible.builtin.file", {}).get("path") for t in tasks]


def test_tests_readme_lists_this_file() -> None:
    text = TESTS_README.read_text()
    assert "### `test_servo_neutral_config.py`" in text
