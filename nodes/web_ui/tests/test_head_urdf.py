"""The pan/tilt head URDF (urdf/head.urdf): chain, limits, frames, post clearance and scanner status."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import structlog
from fastapi.testclient import TestClient

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
    assert (float(tl.get("lower", "")), float(tl.get("upper", ""))) == (-0.5236, 0.5236)
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
    # tof_joint is overwritten by the RGB-D calibration handoff: only bounds are asserted here.
    tof = origin(joints["tof_joint"])
    assert abs(tof[0]) <= 0.05
    assert all(abs(a) <= 0.1 for a in origin(joints["tof_joint"], "rpy"))


def test_optical_rpy(joints: dict[str, ET.Element]) -> None:
    """Both optical frames use the REP 103 optical rotation."""
    for name in ("head_camera_optical_joint", "tof_optical_joint"):
        assert origin(joints[name], "rpy") == pytest.approx(list(OPTICAL_RPY))
        assert origin(joints[name]) == pytest.approx([0, 0, 0])


def test_tof_left_of_rgb_with_spacing(joints: dict[str, ET.Element]) -> None:
    """ToF (left, +y) sits 0.05 +- 0.015 m from the RGB camera (right, -y); the calibration may nudge it."""
    rgb, tof = origin(joints["head_camera_joint"]), origin(joints["tof_joint"])
    assert tof[1] > rgb[1]
    spacing = math.dist(rgb, tof)
    assert math.isclose(spacing, 0.05, abs_tol=0.015)


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


# --- applied head chain served over /api/urdf ----------------------------------------------------------------------


def make_client(applied: Path | None, head_urdf: str | None = None) -> TestClient:
    """TestClient on the shipped URDF directory with the given applied dir and optional tab head_urdf."""
    from web_ui.config import AppConfig, TabConfig
    from web_ui.server import build_app

    tabs = [TabConfig(id="h", type="head", label="H", head_urdf=head_urdf)] if head_urdf else []
    cfg = AppConfig(tabs=tabs, head_applied_calibration_dir=str(applied) if applied else None)
    return TestClient(build_app(config=cfg, urdf_dir=URDF_DIR, static_dir=URDF_DIR))


def write_chain(applied: Path, mount: list[float]) -> None:
    """Write a head_chain.json moving only head_mount_joint."""
    base = {"head_mount_joint": {"xyz": mount, "rpy": [0, 0, 0.1]}}
    (applied / "head_chain.json").write_text(json.dumps({"note": "test", "joints": base}))


def mount_xyz(text: str) -> list[float]:
    """head_mount_joint origin xyz from URDF text."""
    root = ET.fromstring(text)
    joint = next(j for j in root.findall("joint") if j.get("name") == "head_mount_joint")
    return vec(joint.find("origin").get("xyz"))  # type: ignore[union-attr]


def test_override_applied_to_served_head_urdf(tmp_path: Path) -> None:
    """A valid head_chain.json changes the served head URDF, no-store."""
    write_chain(tmp_path, [0.2, 0.11, 0.33])
    resp = make_client(tmp_path).get("/api/urdf/head.urdf")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("application/xml")
    assert resp.headers["cache-control"] == "no-store"
    assert mount_xyz(resp.text) == pytest.approx([0.2, 0.11, 0.33])


def test_no_dir_serves_file_bytes() -> None:
    """Without head_applied_calibration_dir the file is served unchanged."""
    resp = make_client(None).get("/api/urdf/head.urdf")
    assert resp.content == HEAD_URDF.read_bytes()


def test_custom_tab_head_urdf_name_is_overridden(tmp_path: Path) -> None:
    """A tab's head_urdf name is treated as a head URDF."""
    (tmp_path / "x").mkdir()
    write_chain(tmp_path, [0.2, 0.11, 0.33])
    custom = tmp_path / "urdfs"
    custom.mkdir()
    (custom / "myhead.urdf").write_bytes(HEAD_URDF.read_bytes())
    from web_ui.config import AppConfig, TabConfig
    from web_ui.server import build_app

    cfg = AppConfig(
        tabs=[TabConfig(id="h", type="head", label="H", head_urdf="myhead.urdf")],
        head_applied_calibration_dir=str(tmp_path),
    )
    client = TestClient(build_app(config=cfg, urdf_dir=custom, static_dir=custom))
    assert mount_xyz(client.get("/api/urdf/myhead.urdf").text) == pytest.approx([0.2, 0.11, 0.33])


def test_invalid_override_serves_chain_and_logs(tmp_path: Path) -> None:
    """An invalid head_chain.json leaves the repo chain and logs a warning."""
    (tmp_path / "head_chain.json").write_text("{not json")
    with structlog.testing.capture_logs() as logs:
        resp = make_client(tmp_path).get("/api/urdf/head.urdf")
    assert resp.status_code == 200
    assert mount_xyz(resp.text) == pytest.approx([0.15, 0.10, 0.31])
    assert any(entry["log_level"] == "warning" and "head_chain.json" in str(entry) for entry in logs)


def test_other_urdf_files_untouched(tmp_path: Path) -> None:
    """Non-head URDF files and traversal checks are unchanged."""
    write_chain(tmp_path, [0.2, 0.11, 0.33])
    client = make_client(tmp_path)
    other = next(p for p in URDF_DIR.glob("*.urdf") if p.name != "head.urdf")
    resp = client.get(f"/api/urdf/{other.name}")
    assert resp.content == other.read_bytes()
    assert resp.headers["cache-control"] != "no-store"
    assert client.get("/api/urdf/status").status_code == 200
    assert client.get("/api/urdf/nothere.urdf").status_code == 404
