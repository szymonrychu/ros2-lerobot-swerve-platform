"""The swerve base URDF (urdf/robot.urdf) matches the swerve controller config in ansible/group_vars/client.yml."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
ROBOT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot.urdf"

# Module name -> (sign of x, sign of y) in base_link.
MODULES: dict[str, tuple[int, int]] = {"fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1)}
TOL = 1e-6


def ros2_node_config(name: str) -> Any:
    """Return the parsed YAML config block of a ros2_nodes entry of client.yml.

    Args:
        name (str): ros2_nodes entry name.

    Returns:
        Any: the parsed config mapping.
    """
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    entry = next(n for n in nodes if n["name"] == name)
    return yaml.safe_load(entry["config"]) if isinstance(entry["config"], str) else entry["config"]


@pytest.fixture(scope="module")
def swerve() -> dict[str, Any]:
    """Swerve controller config of client.yml."""
    return ros2_node_config("swerve_controller")


@pytest.fixture(scope="module")
def urdf() -> ET.Element:
    """Parsed robot.urdf root."""
    return ET.parse(ROBOT_URDF).getroot()


def joints_by_name(urdf: ET.Element) -> dict[str, ET.Element]:
    """Map joint name to its element."""
    return {j.get("name", ""): j for j in urdf.findall("joint")}


def vec(text: str | None) -> list[float]:
    """Parse a whitespace separated float triple."""
    return [float(v) for v in (text or "").split()]


def test_joint_names_match_swerve_config(swerve: dict[str, Any], urdf: ET.Element) -> None:
    """Every swerve joint name is a URDF joint and no other joint is in the base URDF."""
    assert set(joints_by_name(urdf)) == set(swerve["joint_names"])


def test_steer_joints_sit_on_module_positions(swerve: dict[str, Any], urdf: ET.Element) -> None:
    """Steering axes are at (+-half_length, +-half_width), axis z, limited to the steering range."""
    joints = joints_by_name(urdf)
    for module, (sx, sy) in MODULES.items():
        joint = joints[f"{module}_steer"]
        assert joint.get("type") == "revolute"
        assert joint.find("parent").get("link") == "base_link"  # type: ignore[union-attr]
        xyz = vec(joint.find("origin").get("xyz"))  # type: ignore[union-attr]
        assert xyz[0] == pytest.approx(sx * swerve["half_length_m"], abs=TOL)
        assert xyz[1] == pytest.approx(sy * swerve["half_width_m"], abs=TOL)
        assert vec(joint.find("axis").get("xyz")) == [0.0, 0.0, 1.0]  # type: ignore[union-attr]
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.get("upper", "nan")) == pytest.approx(swerve["max_steer_angle_rad"], abs=1e-4)
        assert float(limit.get("lower", "nan")) == pytest.approx(-swerve["max_steer_angle_rad"], abs=1e-4)


def test_wheel_axle_height_equals_radius(swerve: dict[str, Any], urdf: ET.Element) -> None:
    """base_link is on the floor, so the steering joint origin height is the wheel radius (wheels touch the floor)."""
    joints = joints_by_name(urdf)
    for module in MODULES:
        z = vec(joints[f"{module}_steer"].find("origin").get("xyz"))[2]  # type: ignore[union-attr]
        assert z == pytest.approx(swerve["wheel_radius_m"], abs=TOL)


def test_drive_joints_spin_about_the_axle(urdf: ET.Element) -> None:
    """Drive joints are continuous, children of the steer link, axis +y (forward rolling is positive)."""
    joints = joints_by_name(urdf)
    for module in MODULES:
        joint = joints[f"{module}_drive"]
        assert joint.get("type") == "continuous"
        assert joint.find("parent").get("link") == f"{module}_steer_link"  # type: ignore[union-attr]
        assert vec(joint.find("axis").get("xyz")) == [0.0, 1.0, 0.0]  # type: ignore[union-attr]


def test_roller_is_a_cylinder_of_wheel_radius(swerve: dict[str, Any], urdf: ET.Element) -> None:
    """Each wheel link draws a cylinder of the configured radius with its axis turned onto the axle (y)."""
    links = {link.get("name"): link for link in urdf.findall("link")}
    for module in MODULES:
        visuals = links[f"{module}_wheel_link"].findall("visual")
        assert visuals, "wheel link has no visual"
        cylinders = [v.find("geometry/cylinder") for v in visuals if v.find("geometry/cylinder") is not None]
        assert cylinders, "wheel link has no cylinder"
        assert float(cylinders[0].get("radius", "nan")) == pytest.approx(swerve["wheel_radius_m"], abs=TOL)  # type: ignore[union-attr]
        assert float(cylinders[0].get("length", "0")) > 0  # type: ignore[union-attr]
        rpy = vec(visuals[0].find("origin").get("rpy"))  # type: ignore[union-attr]
        assert abs(rpy[0]) == pytest.approx(math.pi / 2, abs=1e-3)  # cylinder z axis -> y axis
        assert not list(urdf.iter("mesh")), "base URDF must not need meshes"


def test_body_matches_footprint_and_arm_mount(urdf: ET.Element) -> None:
    """The body box is the 0.47 x 0.386 m outer footprint and its top is the arm mount height."""
    mcp = ros2_node_config("mcp_server")
    foot = mcp["footprint"]
    web = ros2_node_config("web_ui")
    tab = next(t for t in web["tabs"] if t.get("type") == "map_nav")
    base = next(link for link in urdf.findall("link") if link.get("name") == "base_link")
    box = base.find("visual/geometry/box")
    assert box is not None
    size = vec(box.get("size"))
    assert size[0] == pytest.approx(foot["length_m"], abs=TOL)
    assert size[1] == pytest.approx(foot["width_m"], abs=TOL)
    z = vec(base.find("visual/origin").get("xyz"))[2]  # type: ignore[union-attr]
    assert z + size[2] / 2 == pytest.approx(tab["arm_offset"][2], abs=TOL)
    assert z - size[2] / 2 > 0, "body must clear the floor"
