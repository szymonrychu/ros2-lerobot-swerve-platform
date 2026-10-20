"""Cross-component consistency of the pan/tilt head: home pose, ranges, joint names and topics.

The head servos (`head` group of the base bus node `base_servos`), the head URDF, the mcp_server `head:` block and the
web_ui head tab each carry their own copy of these values. This test pins every copy against the others, so they can
never silently disagree. The mcp_server and web_ui defaults are read from their config modules.
"""

import ast
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest
import yaml

pytest.importorskip("pydantic")

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
WEB_UI_DEFAULT_YAML = REPO_ROOT / "nodes" / "web_ui" / "config" / "default.yaml"
LAUNCH_FILE = REPO_ROOT / "nodes" / "head_state_publisher" / "launch" / "head_state_publisher.launch.py"

BASE_NODE = "base_servos"
HEAD_NAMESPACE = "head"
JOINT_NAMES = ["head_pan", "head_tilt"]
STATES_TOPIC = "/head/joint_states"
COMMANDS_TOPIC = "/head/joint_commands"
STEPS_PER_TURN = 4096
STEP_CENTER = 2048
HOME_TOLERANCE_RAD = 1e-3
STEP_TOLERANCE = 1
EXPECTED_HOME_RAD = (0.0, math.radians(30.0))
RGBD_RGB_TOPIC = "/head_rgbd/rgb/image_raw/compressed"
RGBD_DEPTH_TOPIC = "/head_rgbd/depth/image_raw"
RGBD_CLOUD_TOPIC = "/head_rgbd/points"

sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "nodes" / "web_ui"), str(REPO_ROOT / "shared")]
try:
    from mcp_server.config import HeadSettings, McpServerConfig
    from mcp_server.head_kinematics import HeadKinematics
    from web_ui.config import AppConfig
finally:
    del sys.path[:3]


def client_node_config(name: str) -> dict[str, Any]:
    """Parse the config block of a ros2_nodes entry of client.yml.

    Args:
        name (str): ros2_nodes entry name.

    Returns:
        dict[str, Any]: The node's config YAML.
    """
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def bridge_head_group() -> dict[str, Any]:
    """The `head` extra group of base_servos."""
    groups = [g for g in client_node_config(BASE_NODE)["extra_groups"] if g["namespace"] == HEAD_NAMESPACE]
    assert len(groups) == 1
    return groups[0]


def bridge_joints() -> dict[str, dict[str, Any]]:
    """Head bridge joint entries by name."""
    return {j["name"]: j for j in bridge_head_group()["joint_names"]}


def mcp_settings() -> list[HeadSettings]:
    """The mcp_server head settings as rendered by client.yml, then the code default."""
    return [McpServerConfig.model_validate(client_node_config("mcp_server")).head, HeadSettings()]


def web_ui_head_tabs() -> list[Any]:
    """The web_ui head tab from client.yml and from default.yaml, with code defaults filled in."""
    tabs = []
    for raw in (client_node_config("web_ui"), yaml.safe_load(WEB_UI_DEFAULT_YAML.read_text())):
        tabs.append(next(t for t in AppConfig.model_validate(raw).tabs if t.type == "head"))
    return tabs


def web_ui_default_tab() -> Any:
    """A head tab with every field left to the code defaults."""
    return next(t for t in AppConfig.model_validate({"tabs": [{"id": "h", "type": "head", "label": "H"}]}).tabs)


def urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    """Revolute joint limits (lower, upper) in rad of a URDF by joint name."""
    root = ET.parse(path).getroot()
    return {
        j.attrib["name"]: (float(j.find("limit").attrib["lower"]), float(j.find("limit").attrib["upper"]))
        for j in root.findall("joint")
        if j.attrib["type"] == "revolute"
    }


def urdf_for_mcp() -> Path:
    """The head URDF the mcp_server is configured with (repo-relative path resolved against the repo root)."""
    path = mcp_settings()[0].urdf_path
    return path if path.is_absolute() else REPO_ROOT / path


def steps_to_rad(steps: int) -> float:
    """Servo steps (4096 per turn, 2048 = 0) to radians."""
    return (steps - STEP_CENTER) * 2 * math.pi / STEPS_PER_TURN


def bridge_limits_rad(joint: dict[str, Any]) -> tuple[float, float]:
    """Command limits of a bridge joint in joint radians, honouring `inverted`."""
    lo, hi = steps_to_rad(joint["command_min_steps"]), steps_to_rad(joint["command_max_steps"])
    return (-hi, -lo) if joint.get("inverted") else (lo, hi)


def test_home_pose_is_equal_everywhere_and_inside_the_limits() -> None:
    """Bridge startup, mcp_server home (client.yml and default) and web_ui head_home_deg (client.yml, yaml, default)."""
    bridge = tuple(bridge_joints()[n]["startup_position_rad"] for n in JOINT_NAMES)
    homes = {"bridge startup_position_rad": bridge}
    for label, s in zip(("mcp_server client.yml", "mcp_server default"), mcp_settings(), strict=True):
        homes[label] = (s.home_pan_rad, s.home_tilt_rad)
    for label, tab in zip(("web_ui client.yml", "web_ui default.yaml"), web_ui_head_tabs(), strict=True):
        homes[label] = tuple(math.radians(d) for d in tab.head_home_deg)
    homes["web_ui code default"] = tuple(math.radians(d) for d in web_ui_default_tab().head_home_deg)
    for label, home in homes.items():
        for got, want, joint in zip(home, EXPECTED_HOME_RAD, JOINT_NAMES, strict=True):
            assert abs(got - want) <= HOME_TOLERANCE_RAD, f"{label}: {joint} home {got} rad != {want} rad"
    limits = urdf_limits(urdf_for_mcp())
    for label, home in homes.items():
        for got, joint in zip(home, JOINT_NAMES, strict=True):
            lo, hi = limits[joint]
            assert lo <= got <= hi, f"{label}: {joint} home {got} outside URDF limits [{lo}, {hi}]"


def test_ranges_agree_across_urdf_bridge_web_ui_and_mcp_server() -> None:
    """URDF limits == bridge command steps == web_ui slider ranges == mcp_server limits (its configured URDF)."""
    urdf = urdf_limits(urdf_for_mcp())
    from_ansible_path = urdf_limits(REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf")
    for joint in JOINT_NAMES:
        assert urdf[joint] == pytest.approx(from_ansible_path[joint]), f"mcp_server URDF != head.urdf: {joint}"
        lo, hi = bridge_limits_rad(bridge_joints()[joint])
        step_rad = 2 * math.pi / STEPS_PER_TURN
        assert lo == pytest.approx(urdf[joint][0], abs=STEP_TOLERANCE * step_rad), f"bridge min {joint}"
        assert hi == pytest.approx(urdf[joint][1], abs=STEP_TOLERANCE * step_rad), f"bridge max {joint}"
    kin = HeadKinematics(urdf_for_mcp()).limits
    for joint in JOINT_NAMES:
        assert kin[joint] == pytest.approx(urdf[joint])
    for label, tab in zip(("client.yml", "default.yaml"), web_ui_head_tabs(), strict=True):
        for joint, rng in zip(JOINT_NAMES, (tab.head_pan_range_deg, tab.head_tilt_range_deg), strict=True):
            want = tuple(math.degrees(v) for v in urdf[joint])
            assert tuple(rng) == pytest.approx(want, abs=0.01), f"web_ui {label} {joint} range"
    tab = web_ui_default_tab()
    for joint, rng in zip(JOINT_NAMES, (tab.head_pan_range_deg, tab.head_tilt_range_deg), strict=True):
        assert tuple(rng) == pytest.approx(tuple(math.degrees(v) for v in urdf[joint]), abs=0.01)


def test_joint_names_agree_across_urdf_bridge_and_mcp_server() -> None:
    """The URDF revolute joints, the bridge group and mcp_server head.joint_names are the same two names."""
    assert sorted(urdf_limits(urdf_for_mcp())) == sorted(JOINT_NAMES)
    assert list(bridge_joints()) == JOINT_NAMES
    for s in mcp_settings():
        assert list(s.joint_names) == JOINT_NAMES


def launch_remaps() -> dict[str, str]:
    """The remappings literal of the head_state_publisher launch file."""
    tree = ast.parse(LAUNCH_FILE.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.Assign) and n.targets[0].id == "REMAPPINGS")  # type: ignore[attr-defined]
    return dict(ast.literal_eval(node.value))


def test_topics_agree_across_bridge_mcp_server_web_ui_and_launch() -> None:
    """/head/joint_states and /head/joint_commands: bridge namespace, mcp_server, web_ui, launch remap."""
    assert bridge_head_group()["namespace"] == HEAD_NAMESPACE
    bridge_states, bridge_commands = f"/{HEAD_NAMESPACE}/joint_states", f"/{HEAD_NAMESPACE}/joint_commands"
    assert (bridge_states, bridge_commands) == (STATES_TOPIC, COMMANDS_TOPIC)
    for s in mcp_settings():
        assert (s.state_topic, s.command_topic) == (STATES_TOPIC, COMMANDS_TOPIC)
    tabs = [*web_ui_head_tabs(), web_ui_default_tab()]
    for tab in tabs:
        assert (tab.head_joint_states_topic, tab.head_command_topic) == (STATES_TOPIC, COMMANDS_TOPIC)
    for map_tab in AppConfig.model_validate(client_node_config("web_ui")).tabs:
        if map_tab.type == "map_nav":
            assert map_tab.head_joint_states_topic == STATES_TOPIC
    assert launch_remaps()["joint_states"] == STATES_TOPIC


def test_rgbd_topics_agree_between_client_yml_and_default_yaml() -> None:
    """The web_ui RGB-D keys (head tab view, map tab layer) are the head_rgbd_cpp output topics in both configs."""
    for raw in (client_node_config("web_ui"), yaml.safe_load(WEB_UI_DEFAULT_YAML.read_text())):
        tabs = AppConfig.model_validate(raw).tabs
        head = next(t for t in tabs if t.type == "head")
        nav = next(t for t in tabs if t.type == "map_nav")
        assert head.rgbd_rgb_topic == RGBD_RGB_TOPIC
        assert head.rgbd_depth_topic == RGBD_DEPTH_TOPIC
        assert nav.rgbd_cloud_topic == RGBD_CLOUD_TOPIC
