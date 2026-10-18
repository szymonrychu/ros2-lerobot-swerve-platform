"""The pan/tilt head URDF (urdf/head.urdf): chain, limits, frames, post clearance and scanner status."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from web_ui.urdf_scanner import scan_urdf_directory

URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"
HEAD_URDF = URDF_DIR / "head.urdf"

LIDAR_XYZ = (0.158, 0.06, 0.20)
LIDAR_RADIUS = 0.035
FOOTPRINT_LENGTH = 0.47
FOOTPRINT_WIDTH = 0.386
OPTICAL_RPY = (-1.5708, 0.0, -1.5708)
TOL = 1e-6
PRIMITIVES = {"box", "cylinder", "sphere"}


@pytest.fixture(scope="module")
def urdf() -> ET.Element:
    """Parsed head.urdf root."""
    return ET.parse(HEAD_URDF).getroot()


@pytest.fixture(scope="module")
def joints(urdf: ET.Element) -> dict[str, ET.Element]:
    """Map joint name to element."""
    return {j.get("name", ""): j for j in urdf.findall("joint")}


def vec(text: str | None) -> list[float]:
    """Parse a whitespace separated float list."""
    return [float(v) for v in (text or "").split()]


def origin(joint: ET.Element, attr: str = "xyz") -> list[float]:
    """Joint origin attribute, zeros when absent."""
    el = joint.find("origin")
    if el is None or el.get(attr) is None:
        return [0.0, 0.0, 0.0]
    return vec(el.get(attr))


def parent_child(joint: ET.Element) -> tuple[str, str]:
    """Return (parent, child) link names."""
    return joint.find("parent").get("link", ""), joint.find("child").get("link", "")  # type: ignore[union-attr]


def test_chain_parent_child_order(joints: dict[str, ET.Element]) -> None:
    """The joints form base_link -> pan -> tilt -> head -> camera/tof frames."""
    expected = {
        "head_mount_joint": ("base_link", "head_pan_base"),
        "head_pan": ("head_pan_base", "head_pan_link"),
        "head_tilt": ("head_pan_link", "head_link"),
        "head_camera_joint": ("head_link", "head_camera_link"),
        "head_camera_optical_joint": ("head_camera_link", "head_camera_optical_frame"),
        "tof_joint": ("head_link", "tof_link"),
        "tof_optical_joint": ("tof_link", "tof_optical_frame"),
    }
    for name, pc in expected.items():
        assert parent_child(joints[name]) == pc, name


def test_root_link_is_base_link(urdf: ET.Element, joints: dict[str, ET.Element]) -> None:
    """base_link is the only link that is never a child."""
    links = {link.get("name") for link in urdf.findall("link")}
    children = {parent_child(j)[1] for j in joints.values()}
    assert links - children == {"base_link"}


def test_joint_types_axes_limits(joints: dict[str, ET.Element]) -> None:
    """Pan and tilt are revolute with the configured axes, limits and D3 velocity caps."""
    pan, tilt = joints["head_pan"], joints["head_tilt"]
    assert pan.get("type") == tilt.get("type") == "revolute"
    assert vec(pan.find("axis").get("xyz")) == [0, 0, 1]  # type: ignore[union-attr]
    assert vec(tilt.find("axis").get("xyz")) == [0, 1, 0]  # type: ignore[union-attr]
    pl, tl = pan.find("limit"), tilt.find("limit")
    assert pl is not None and tl is not None
    assert (float(pl.get("lower", "")), float(pl.get("upper", ""))) == (-1.5708, 1.5708)
    assert (float(tl.get("lower", "")), float(tl.get("upper", ""))) == (-0.5236, 0.7854)
    for lim in (pl, tl):
        assert float(lim.get("velocity", "")) == 1.0
        assert float(lim.get("effort", "0")) > 0
    for name in (
        "head_mount_joint",
        "head_camera_joint",
        "head_camera_optical_joint",
        "tof_joint",
        "tof_optical_joint",
    ):
        assert joints[name].get("type") == "fixed", name


def test_mount_and_camera_origins(joints: dict[str, ET.Element]) -> None:
    """Mount, tilt offset and camera / ToF origins."""
    assert origin(joints["head_mount_joint"]) == pytest.approx([0.15, 0.10, 0.31])
    assert origin(joints["head_pan"]) == pytest.approx([0, 0, 0])
    assert origin(joints["head_tilt"]) == pytest.approx([0, 0, 0.04])
    assert origin(joints["head_camera_joint"]) == pytest.approx([0.02, -0.025, 0])
    assert origin(joints["tof_joint"]) == pytest.approx([0.02, 0.025, 0])


def test_optical_rpy(joints: dict[str, ET.Element]) -> None:
    """Both optical frames use the REP 103 optical rotation."""
    for name in ("head_camera_optical_joint", "tof_optical_joint"):
        assert origin(joints[name], "rpy") == pytest.approx(list(OPTICAL_RPY))
        assert origin(joints[name]) == pytest.approx([0, 0, 0])


def test_tof_left_of_rgb_with_spacing(joints: dict[str, ET.Element]) -> None:
    """ToF (left, +y) sits 0.05 m from the RGB camera (right, -y), same x and z."""
    rgb, tof = origin(joints["head_camera_joint"]), origin(joints["tof_joint"])
    assert tof[1] > rgb[1]
    assert math.isclose(tof[1] - rgb[1], 0.05, abs_tol=TOL)
    assert rgb[0] == tof[0] and rgb[2] == tof[2]


def visual_boxes_and_cylinders(link: ET.Element) -> list[tuple[ET.Element, list[float]]]:
    """Return (geometry shape, visual origin xyz) of each visual."""
    out = []
    for vis in link.findall("visual"):
        shape = list(vis.find("geometry"))[0]  # type: ignore[arg-type]
        out.append((shape, origin(vis)))
    return out


def test_post_behind_lidar_and_inside_footprint(urdf: ET.Element, joints: dict[str, ET.Element]) -> None:
    """head_post is fixed to base_link; at the lidar plane it is behind the lidar and inside the footprint."""
    post_joints = [j for j in joints.values() if parent_child(j)[1] == "head_post"]
    assert len(post_joints) == 1 and post_joints[0].get("type") == "fixed"
    assert parent_child(post_joints[0])[0] == "base_link"
    joint_xyz = origin(post_joints[0])
    post = next(link for link in urdf.findall("link") if link.get("name") == "head_post")
    lidar_back = LIDAR_XYZ[0] - LIDAR_RADIUS
    reaches_pan_axis = False
    covers_lidar_plane = False
    for shape, xyz in visual_boxes_and_cylinders(post):
        centre = [joint_xyz[i] + xyz[i] for i in range(3)]
        if shape.tag == "cylinder":
            half_z = float(shape.get("length", "0")) / 2
            half_x = half_y = float(shape.get("radius", "0"))
        else:
            sx, sy, sz = vec(shape.get("size"))
            half_x, half_y, half_z = sx / 2, sy / 2, sz / 2
        if centre[2] + half_z >= 0.31 - TOL and centre[0] - half_x <= 0.15 <= centre[0] + half_x + 0.06:
            reaches_pan_axis = True
        if centre[2] - half_z <= LIDAR_XYZ[2] <= centre[2] + half_z:
            covers_lidar_plane = True
            assert centre[0] + half_x < lidar_back
            assert abs(centre[0]) + half_x <= FOOTPRINT_LENGTH / 2
            assert abs(centre[1]) + half_y <= FOOTPRINT_WIDTH / 2
    assert covers_lidar_plane, "post must pass the lidar plane (comes up from behind)"
    assert reaches_pan_axis, "post must reach the pan axis height"


def test_visuals_use_primitives(urdf: ET.Element) -> None:
    """No mesh anywhere; every visual is a box, cylinder or sphere."""
    assert not urdf.findall(".//mesh")
    shapes = [list(v.find("geometry"))[0].tag for v in urdf.iter("visual")]  # type: ignore[arg-type]
    assert shapes and set(shapes) <= PRIMITIVES


def test_scanner_reports_head_urdf_ok() -> None:
    """The /api/urdf/status scan lists head.urdf as ok."""
    results = scan_urdf_directory(URDF_DIR)
    head = [r for r in results if r.path.endswith("head.urdf")]
    assert head and all(r.status == "ok" for r in head), results
