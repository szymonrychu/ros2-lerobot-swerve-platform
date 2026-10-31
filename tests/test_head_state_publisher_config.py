"""head_state_publisher: robot_state_publisher on the head URDF, wired through Ansible (client host)."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
LAUNCH_FILE = REPO_ROOT / "nodes" / "head_state_publisher" / "launch" / "head_state_publisher.launch.py"
README = REPO_ROOT / "nodes" / "head_state_publisher" / "README.md"
HEAD_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"

NODE = "head_state_publisher"
BASE_NODE = "base_servos"
EXPECTED_TIER = {
    "memory_low": "32M",
    "memory_max": "128M",
    "cpu_weight": 500,
    "io_weight": 500,
    "oom_score_adjust": -500,
}


def client_vars() -> dict:
    """Parsed client.yml group_vars."""
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_node_type_fields_and_tier() -> None:
    """Native launch-file node type with apt package, src paths and the mid cgroup tier."""
    node_type = client_vars()["ros2_node_type_defaults"][NODE]
    assert node_type["deploy_mode"] == "native"
    assert node_type["node_launch_command"] == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/head_state_publisher/launch/head_state_publisher.launch.py"
    )
    assert "'" not in node_type["node_launch_command"]
    assert node_type["node_src_dir"] == ""
    assert node_type["src_paths"] == [
        "nodes/head_state_publisher",
        "nodes/web_ui/urdf/head.urdf",
        "shared/ros2_common/head_rgbd_calibration/urdf_text.py",
    ]
    assert node_type["apt_packages"] == ["ros-jazzy-robot-state-publisher"]
    for key, value in EXPECTED_TIER.items():
        assert node_type[key] == value, key


def test_entry_present_enabled_after_base_servos() -> None:
    """The ros2_nodes entry is present, enabled, and listed after base_servos."""
    nodes = client_vars()["ros2_nodes"]
    names = [n["name"] for n in nodes]
    entry = nodes[names.index(NODE)]
    assert entry["node_type"] == NODE
    assert entry["present"] is True
    assert entry["enabled"] is True
    assert names.index(NODE) > names.index(BASE_NODE)


def launch_node_call() -> ast.Call:
    """The Node(...) call of the launch file, parsed without importing ROS."""
    tree = ast.parse(LAUNCH_FILE.read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "Node"]
    assert len(calls) == 1
    return calls[0]


def test_launch_file_runs_robot_state_publisher_with_remaps() -> None:
    """The launch file starts robot_state_publisher named head_state_publisher with both remaps."""
    call = launch_node_call()
    kwargs = {k.arg: k.value for k in call.keywords}
    assert ast.literal_eval(kwargs["package"]) == "robot_state_publisher"
    assert ast.literal_eval(kwargs["executable"]) == "robot_state_publisher"
    assert ast.literal_eval(kwargs["name"]) == NODE
    assert kwargs["remappings"].id == "REMAPPINGS"
    remappings = next(
        ast.literal_eval(t.value)
        for t in ast.parse(LAUNCH_FILE.read_text()).body
        if isinstance(t, ast.Assign) and t.targets[0].id == "REMAPPINGS"
    )
    assert remappings == [
        ("joint_states", "/head/joint_states"),
        ("robot_description", "/head/robot_description"),
    ]
    assert "robot_description" in ast.unparse(kwargs["parameters"])


def test_launch_file_reads_head_urdf_relative_to_itself() -> None:
    """The URDF path is resolved from __file__ and names web_ui/urdf/head.urdf."""
    source = LAUNCH_FILE.read_text()
    assert "__file__" in source
    assert "head.urdf" in source
    assert '"web_ui"' in source
    assert HEAD_URDF.is_file()


def test_playbook_step_exists_with_tags() -> None:
    """The client playbook has a Deploy head_state_publisher step tagged for node and apt."""
    plays = yaml.safe_load(PLAYBOOK.read_text())
    steps = []
    for play in plays:
        for task in play.get("tasks") or []:
            steps.append(task)
            steps.extend(task.get("block", []))
    step = next(s for s in steps if s.get("name") == f"Deploy {NODE}")
    assert step["vars"]["_deploy_node_name"] == NODE
    assert {NODE, "apt"} <= set(step["tags"])


def test_readme_documents_frames_and_check() -> None:
    """The node README explains what is published and how to check it."""
    text = README.read_text()
    assert "/tf_static" in text
    assert "tf2_echo base_link d435_color_optical_frame" in text
    assert "head_camera_optical_frame" not in text and "tof_optical_frame" not in text


D435_FRAMES = [
    "d435_link",
    "d435_depth_frame",
    "d435_depth_optical_frame",
    "d435_color_frame",
    "d435_color_optical_frame",
]


def test_d435_frames_resolve_from_head_urdf() -> None:
    """Each d435 frame is a link with one parent joint that reaches base_link, so it is published once."""
    root = ET.parse(HEAD_URDF).getroot()
    links = {link.get("name") for link in root.findall("link")}
    parents: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link", "")  # type: ignore[union-attr]
        child = joint.find("child").get("link", "")  # type: ignore[union-attr]
        parents.setdefault(child, []).append(parent)
    for frame in D435_FRAMES:
        assert frame in links, frame
        assert len(parents.get(frame, [])) == 1, f"{frame} must have exactly one parent"
        node, hops = frame, 0
        while node != "base_link":
            node = parents[node][0]
            hops += 1
            assert hops < 20, f"{frame} does not reach base_link"
