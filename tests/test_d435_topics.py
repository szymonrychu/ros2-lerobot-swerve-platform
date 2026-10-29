"""Pins of the live D435 bring-up: the /head/d435/* topic list and the factory colour extrinsic in head.urdf.

nodes/bridges/realsense_d435/config/topics.yaml is the topic list read from the running client (topic_scraper_api,
scripts/d435_diag.sh --topics). Every /head/d435/ topic that ansible/group_vars/client.yml names must be in it with
the type its consumers expect, and every frame id it records must be a link of head.urdf (design D4: head.urdf owns
every d435 frame). The colour joint of head.urdf is the factory Depth-to-Color extrinsic of the head D435, read with
`scripts/d435_diag.sh --device` (rs-enumerate-devices -c) at bring-up.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
TOPICS_FILE = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
HEAD_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
D435_TOPIC = re.compile(r"/head/d435/[A-Za-z0-9_/]+")
# Streams the D435 consumers read (head_rgbd_cpp, d435_obstacle_cloud, ground_estimator_cpp, mcp_server).
CONSUMED_STREAMS = {
    "/head/d435/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "/head/d435/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/head/d435/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
    "/head/d435/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/head/d435/depth/image_rect_raw": "sensor_msgs/msg/Image",
    "/head/d435/depth/camera_info": "sensor_msgs/msg/CameraInfo",
}
# Factory extrinsics of the head D435 (device serial 935422070045, ASIC serial / firmware update id 935723024187),
# rs-enumerate-devices -c on 2026-10-28. Rows of the rotation as printed; p_to = R p_from + t, in optical axes (m).
COLOR_TO_DEPTH_ROTATION = (
    (0.99995, -0.00827042, -0.00569708),
    (0.00826205, 0.999965, -0.00149191),
    (0.00570922, 0.00144477, 0.999983),
)
COLOR_TO_DEPTH_TRANSLATION = (-0.0150171248242259, 6.98006697348319e-05, 0.000176108413143083)
DEPTH_TO_COLOR_ROTATION = (
    (0.99995, 0.00826205, 0.00570922),
    (-0.00827042, 0.999965, 0.00144477),
    (-0.00569708, -0.00149191, 0.999983),
)
DEPTH_TO_COLOR_TRANSLATION = (0.0150147853419185, -0.000194250649656169, -0.000261554960161448)
ASIC_SERIAL = "935723024187"
DEVICE_SERIAL = "935422070045"
CAPTURE_DATE = "2026-10-28"
# body (x forward, y left, z up) = OPTICAL_TO_BODY @ optical (x right, y down, z forward).
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# The URDF writes 7 significant decimals: 1e-7 m and 1e-6 rad cover the rounding.
XYZ_TOL_M = 1e-7
RPY_TOL_RAD = 1e-6


def expected_type(topic: str) -> str:
    """Return the message type a /head/d435/ topic must have, from its name.

    Args:
        topic (str): Topic name.

    Returns:
        str: ROS message type (e.g. sensor_msgs/msg/CameraInfo).

    Raises:
        AssertionError: The name has no known suffix; extend this rule for a new kind of topic.
    """
    if topic.endswith("/camera_info"):
        return "sensor_msgs/msg/CameraInfo"
    if topic.endswith(("/compressed", "/compressedDepth", "/zstd")):
        return "sensor_msgs/msg/CompressedImage"
    if topic.endswith("/image_raw") or topic.endswith("/image_rect_raw"):
        return "sensor_msgs/msg/Image"
    if topic.endswith("/points") or topic.endswith("/points_filtered"):
        return "sensor_msgs/msg/PointCloud2"
    raise AssertionError(f"no expected type for {topic}: add its suffix to expected_type()")


def configured_d435_topics() -> set[str]:
    """Return every /head/d435/ topic named anywhere in client.yml (keys, values and comments).

    Returns:
        set[str]: Topic names without a trailing slash.
    """
    found = {match.rstrip("/") for match in D435_TOPIC.findall(CLIENT_VARS.read_text())}
    return {topic for topic in found if topic != "/head/d435"}


def pinned_topics() -> dict[str, dict]:
    """Return the pinned topic list.

    Returns:
        dict[str, dict]: Topic name -> {type, frame_id (header topics), node}.
    """
    return yaml.safe_load(TOPICS_FILE.read_text())["topics"]


def urdf_links() -> set[str]:
    """Return the link names of head.urdf.

    Returns:
        set[str]: Link names.
    """
    return {link.get("name") for link in ET.parse(HEAD_URDF).getroot().findall("link")}


def joint_origin(name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return the origin of a head.urdf joint.

    Args:
        name (str): Joint name.

    Returns:
        tuple[np.ndarray, np.ndarray]: xyz (m) and rpy (rad).
    """
    joint = next(j for j in ET.parse(HEAD_URDF).getroot().findall("joint") if j.get("name") == name)
    origin = joint.find("origin")
    return (
        np.array([float(v) for v in origin.get("xyz").split()]),
        np.array([float(v) for v in origin.get("rpy").split()]),
    )


def test_every_configured_d435_topic_is_pinned() -> None:
    """Every /head/d435/ topic client.yml names is in topics.yaml with the type its name implies."""
    configured = configured_d435_topics()
    assert configured, "client.yml names no /head/d435/ topic"
    pinned = pinned_topics()
    missing = sorted(configured - set(pinned))
    assert not missing, f"not in {TOPICS_FILE.name}: {missing}"
    for topic in sorted(configured):
        assert pinned[topic]["type"] == expected_type(topic), topic


def test_topics_yaml_lists_the_consumed_streams() -> None:
    """topics.yaml lists the colour compressed topic, aligned depth, depth image_rect_raw and their camera_info."""
    pinned = pinned_topics()
    for topic, type_name in CONSUMED_STREAMS.items():
        assert topic in pinned, topic
        assert pinned[topic]["type"] == type_name, topic


def test_pinned_types_follow_the_naming_rule() -> None:
    """Every pinned image, camera_info and cloud topic has the type its name implies; every entry has a node."""
    for topic, entry in pinned_topics().items():
        assert topic.startswith("/head/d435/"), topic
        assert entry["node"], topic
        if entry["type"].startswith("sensor_msgs/"):
            assert entry["type"] == expected_type(topic), topic


def test_pinned_frame_ids_are_head_urdf_links() -> None:
    """Every recorded header frame_id is a link of head.urdf (D4); every published sensor_msgs topic records one.

    An advertised topic that never carries a message (published: false) has frame_id null.
    """
    links = urdf_links()
    for topic, entry in pinned_topics().items():
        if entry.get("published", True) is False:
            assert entry["frame_id"] is None, topic
            continue
        if entry["type"].startswith("sensor_msgs/") or entry.get("frame_id") is not None:
            assert entry["frame_id"] in links, f"{topic}: {entry.get('frame_id')}"


def test_depth_topics_list_no_compressed_or_theora_transport() -> None:
    """The 16UC1 depth topics advertise raw, compressedDepth and zstd only; the colour keeps compressed."""
    pinned = pinned_topics()
    for base in ("/head/d435/depth/image_rect_raw", "/head/d435/aligned_depth_to_color/image_raw"):
        transports = {topic[len(base) + 1 :] for topic in pinned if topic.startswith(base + "/")}
        assert transports == {"compressedDepth", "zstd"}, base
    assert "/head/d435/color/image_raw/compressed" in pinned


def test_factory_extrinsics_are_mutual_inverses() -> None:
    """The two captured extrinsics invert each other, so the capture is self-consistent."""
    r_cd, t_cd = np.array(COLOR_TO_DEPTH_ROTATION), np.array(COLOR_TO_DEPTH_TRANSLATION)
    r_dc, t_dc = np.array(DEPTH_TO_COLOR_ROTATION), np.array(DEPTH_TO_COLOR_TRANSLATION)
    np.testing.assert_allclose(r_cd @ r_dc, np.eye(3), atol=2e-6)
    np.testing.assert_allclose(r_cd @ t_dc + t_cd, np.zeros(3), atol=1e-8)


def test_head_urdf_colour_joint_is_the_factory_extrinsic() -> None:
    """d435_color_joint (d435_link = depth frame -> colour frame) is the factory Color-to-Depth extrinsic in body axes.

    The pose of the colour frame in the depth frame maps colour coordinates to depth coordinates, which is the
    Color-to-Depth extrinsic; the URDF writes it in body axes (x forward, y left, z up), so both the translation and
    the rotation are conjugated by the optical-to-body rotation. A comment above the joint names the serial and date.
    """
    rotation = OPTICAL_TO_BODY @ np.array(COLOR_TO_DEPTH_ROTATION) @ OPTICAL_TO_BODY.T
    u, _, vt = np.linalg.svd(rotation)
    expected_rpy = Rotation.from_matrix(u @ vt).as_euler("xyz")
    expected_xyz = OPTICAL_TO_BODY @ np.array(COLOR_TO_DEPTH_TRANSLATION)
    xyz, rpy = joint_origin("d435_color_joint")
    np.testing.assert_allclose(xyz, expected_xyz, atol=XYZ_TOL_M)
    np.testing.assert_allclose(rpy, expected_rpy, atol=RPY_TOL_RAD)
    # The colour imager sits on the camera's left of the depth origin (+y), 15 mm away.
    assert 0.0145 < xyz[1] < 0.0155
    text = HEAD_URDF.read_text()
    before_joint = text[: text.index('<joint name="d435_color_joint"')]
    comment = before_joint[before_joint.rindex("<!--") :]
    assert "-->" in comment
    for fact in (ASIC_SERIAL, DEVICE_SERIAL, CAPTURE_DATE):
        assert fact in comment, fact
