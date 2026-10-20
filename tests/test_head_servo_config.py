"""Head (pan/tilt) servo group in Ansible group_vars: an extra `head` group on the base_servos bridge.

The head servos share the base bus (U1) and never sit on the arm bridge. Their step ranges must equal the joint limits
of nodes/web_ui/urdf/head.urdf (4096 steps per turn around 2048), their speed/acceleration caps and startup (home)
position must be set, and autonomy and web_ui must be allowed to command them directly.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
HEAD_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"

BASE_NODE = "base_servos"
FOLLOWER_NODE = "lerobot_follower"
HEAD_NAMESPACE = "head"
HEAD_JOINTS = {"head_pan": 20, "head_tilt": 21}
STEPS_PER_TURN = 4096
STEP_CENTER = 2048
STEP_TOLERANCE = 1
REQUIRED_SOURCES = {"autonomy", "web_ui"}


def node_config(name: str) -> dict:
    """Parse the config block of a ros2_nodes entry of client.yml.

    Args:
        name (str): ros2_nodes entry name.

    Returns:
        dict: The node's config YAML as a dict.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def head_group() -> dict:
    """The `head` entry of base_servos extra_groups."""
    groups = [g for g in node_config(BASE_NODE).get("extra_groups") or [] if g["namespace"] == HEAD_NAMESPACE]
    assert len(groups) == 1, "base_servos needs exactly one extra_groups entry with namespace head"
    return groups[0]


def head_joints() -> dict[str, dict]:
    """Head joint entries by name."""
    return {j["name"]: j for j in head_group()["joint_names"]}


def urdf_limits() -> dict[str, tuple[float, float]]:
    """Revolute joint limits (lower, upper) in rad of head.urdf by joint name."""
    root = ET.parse(HEAD_URDF).getroot()
    return {
        j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
        for j in root.findall("joint")
        if j.get("type") == "revolute"
    }


def rad_to_steps(rad: float, inverted: bool) -> int:
    """Centred radians to servo steps (0 rad = 2048, 4096 steps per turn)."""
    return round((-rad if inverted else rad) * STEPS_PER_TURN / (2 * math.pi) + STEP_CENTER)


def test_head_group_exists_only_on_base_servos() -> None:
    assert set(head_joints()) == set(HEAD_JOINTS)
    follower = node_config(FOLLOWER_NODE)
    assert not [g for g in follower.get("extra_groups") or [] if g["namespace"] == HEAD_NAMESPACE]
    assert not {j["name"] for j in follower["joint_names"]} & set(HEAD_JOINTS)


def test_head_joint_ids_are_the_defaults() -> None:
    assert {n: j["id"] for n, j in head_joints().items()} == HEAD_JOINTS


def test_joint_names_and_ids_are_unique_on_the_base_bus() -> None:
    cfg = node_config(BASE_NODE)
    joints = [*cfg["joint_names"], *(j for g in cfg["extra_groups"] for j in g["joint_names"])]
    names = [j["name"] for j in joints]
    ids = [j["id"] for j in joints]
    assert len(names) == len(set(names))
    assert len(ids) == len(set(ids))


def test_head_steps_equal_the_urdf_limits() -> None:
    limits = urdf_limits()
    for name, joint in head_joints().items():
        lower, upper = limits[name]
        inverted = bool(joint.get("inverted", False))
        lo, hi = sorted((rad_to_steps(lower, inverted), rad_to_steps(upper, inverted)))
        assert abs(joint["command_min_steps"] - lo) <= STEP_TOLERANCE, name
        assert abs(joint["command_max_steps"] - hi) <= STEP_TOLERANCE, name


def test_head_caps_are_present_and_positive() -> None:
    for name, joint in head_joints().items():
        assert joint["max_speed_rad_s"] > 0, name
        assert joint["max_acceleration_rad_s2"] > 0, name


def test_head_startup_positions_are_inside_the_limits() -> None:
    limits = urdf_limits()
    for name, joint in head_joints().items():
        lower, upper = limits[name]
        assert lower <= joint["startup_position_rad"] <= upper, name
        steps = rad_to_steps(joint["startup_position_rad"], bool(joint.get("inverted", False)))
        assert joint["command_min_steps"] <= steps <= joint["command_max_steps"], name


def test_autonomy_and_web_ui_may_command_the_head_directly() -> None:
    assert REQUIRED_SOURCES <= set(node_config(BASE_NODE)["direct_command_sources"])


def test_head_tilt_servo_is_mounted_reversed_and_pan_is_not() -> None:
    """Found 2026-10-20 after re-centring: a positive tilt command looked up, so the tilt servo is `inverted`."""
    joints = head_joints()
    assert joints["head_tilt"].get("inverted") is True
    assert not joints["head_pan"].get("inverted", False)
    assert (joints["head_tilt"]["command_min_steps"], joints["head_tilt"]["command_max_steps"]) == (1707, 2389)
    assert rad_to_steps(joints["head_tilt"]["startup_position_rad"], True) == 1707
