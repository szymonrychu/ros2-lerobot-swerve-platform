"""Static invariants of the mcp_server camera tools wiring: calibration directory task, config keys, README contract."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
SETUP_TASKS = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "mcp_server_setup.yml"
MCP_README = REPO_ROOT / "nodes" / "mcp_server" / "README.md"
CALIBRATION_DIR = "/var/lib/ros2/camera_calibration"
HEAD_URDF = "nodes/web_ui/urdf/head.urdf"
D435_TOPICS = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
D435_COLOUR_TOPIC = "/head/d435/color/image_raw/compressed"
D435_COLOUR_INFO_TOPIC = "/head/d435/color/camera_info"
D435_COLOUR_FRAME = "d435_color_frame"
D435_COLOUR_OPTICAL_FRAME = "d435_color_optical_frame"
# The head camera takes the D435 factory colour intrinsics from its CameraInfo (no calibration file, no rescale).
D435_HEAD_INTRINSICS = {"camera_info_topic": D435_COLOUR_INFO_TOPIC}
# The head (D435 colour) camera mount is the identity on d435_color_frame: head.urdf (with the applied head chain,
# through HeadKinematics) is the real mount.
D435_COLOUR_MOUNT = {
    "parent_frame": D435_COLOUR_FRAME,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
CAMERA_TOOLS = (
    "pixel_to_ground",
    "pixel_to_point",
    "camera_point_to_arm",
    "get_annotated_camera_image",
    "mark_candidate_points",
    "resolve_candidate",
    "capture_calibration_sample",
    "solve_camera_calibration",
    "clear_calibration_samples",
)


def mcp_config() -> dict:
    """Parse the mcp_server `config: |` block of group_vars/client.yml.

    Returns:
        dict: The node config.
    """
    entry = next(n for n in yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"] if n["name"] == "mcp_server")
    return yaml.safe_load(entry["config"])


def test_setup_tasks_create_the_calibration_directory_owned_by_the_node_user() -> None:
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    calib = dirs[CALIBRATION_DIR]
    assert calib["state"] == "directory"
    assert calib["owner"] == "{{ ansible_user }}" and calib["group"] == "{{ ansible_user }}"


def test_cameras_config_head_on_the_d435() -> None:
    cameras = mcp_config()["cameras"]
    assert cameras["calibration_dir"] == CALIBRATION_DIR
    assert set(cameras) == {"calibration_dir", "head"}  # the gripper camera is retired
    # The head camera is the D435 colour stream: on d435_color_frame with a zero mount and its CameraInfo intrinsics.
    assert cameras["head"] == {
        "parent_frame": D435_COLOUR_FRAME,
        "intrinsics": D435_HEAD_INTRINSICS,
        "mount": D435_COLOUR_MOUNT,
    }


def test_head_camera_is_the_d435_colour() -> None:
    """The head camera reads the live D435 colour stream, hangs on the head.urdf colour frame whose optical child is
    the frame the stream is stamped in, and takes its intrinsics from the colour CameraInfo of the same frame."""
    cfg = mcp_config()
    live = yaml.safe_load(D435_TOPICS.read_text())["topics"]
    topic = cfg["topics"]["head_camera"]
    assert topic == D435_COLOUR_TOPIC
    assert live[topic]["type"] == "sensor_msgs/msg/CompressedImage"
    assert live[topic]["frame_id"] == D435_COLOUR_OPTICAL_FRAME
    head = cfg["cameras"]["head"]
    assert head["parent_frame"] == head["mount"]["parent_frame"] == D435_COLOUR_FRAME
    robot = ET.parse(REPO_ROOT / cfg["head"]["urdf_path"]).getroot()
    assert D435_COLOUR_FRAME in {link.attrib["name"] for link in robot.iter("link")}
    children = [
        joint.find("child").attrib["link"]
        for joint in robot.iter("joint")
        if joint.find("parent").attrib["link"] == D435_COLOUR_FRAME
    ]
    assert children == [D435_COLOUR_OPTICAL_FRAME]
    assert head["intrinsics"] == D435_HEAD_INTRINSICS
    info = live[head["intrinsics"]["camera_info_topic"]]
    assert info["type"] == "sensor_msgs/msg/CameraInfo"
    assert info["frame_id"] == live[topic]["frame_id"]


def test_head_parent_frame_is_a_head_urdf_link_and_the_mount_is_the_zero_offset_on_it() -> None:
    cfg = mcp_config()
    assert cfg["head"]["urdf_path"] == HEAD_URDF
    robot = ET.parse(REPO_ROOT / cfg["head"]["urdf_path"]).getroot()
    head = cfg["cameras"]["head"]
    assert head["parent_frame"] in {link.attrib["name"] for link in robot.iter("link")}
    assert head["parent_frame"] == head["mount"]["parent_frame"] == D435_COLOUR_FRAME
    mount = head["mount"]
    assert [mount[k] for k in ("x", "y", "z", "roll", "pitch", "yaw")] == [0.0] * 6


def test_arm_reach_keys_are_ordered() -> None:
    arm = mcp_config()["arm"]
    assert 0 < arm["reach_inner_m"] < arm["reach_outer_m"]


def test_readme_documents_the_camera_tools_frames_and_calibration() -> None:
    text = MCP_README.read_text()
    for tool in CAMERA_TOOLS:
        assert f"`{tool}" in text, tool
    for needle in (
        "## Camera tools and calibration",
        "not calibrated",
        "base_in_base_link",
        CALIBRATION_DIR,
        "gripper_link",
        "solve_camera_calibration",
    ):
        assert needle in text, needle


def test_camera_image_wait_covers_discovery_on_a_loaded_pi() -> None:
    """Each get_camera_image call subscribes afresh; with the full stack running, DDS discovery took over the 2 s
    default (2026-10-08: 'no frame ... within 2.0 s' while the topic ran at 18 Hz), so the wait is 5 s."""
    assert mcp_config()["timeouts"]["image_timeout_s"] >= 5.0
