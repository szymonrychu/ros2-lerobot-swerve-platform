"""Marker POIs in the agent tools: PoiView marker fields, facing and approach pose, marker_kind filter, approach z."""

import math
from pathlib import Path
from typing import Any

import pytest

from mcp_server.approach import ApproachTarget, resolve_target
from mcp_server.approach_tools import POI_KINDS, poi_z_lookup
from mcp_server.config import McpServerConfig
from mcp_server.perception_models import marker_facing
from mcp_server.tools import build_mcp_server

from .test_tools import TOKEN, FakeRobot, call

SQRT_HALF = math.sqrt(0.5)
STANDOFF_M = 0.6


def marker_poi(
    poi_id: str, kind: str, x: float, y: float, z: float, q: tuple[float, float, float, float]
) -> dict[str, Any]:
    return {
        "id": poi_id,
        "kind": "marker",
        "name": f"{kind} marker",
        "x": x,
        "y": y,
        "z": z,
        "orientation": list(q),
        "status": "open",
        "created_by": "agent",
        "times_seen": 3,
        "last_seen": 1000.0,
        "confidence": 0.9,
        "marker": {
            "kind": kind,
            "id": "1",
            "key": f"{kind}:1",
            "size_mm": 150.0,
            "params": {"slot": "a"},
            "payload": "x",
            "version": 1,
            "pose_samples": 5,
        },
    }


# +z axis of the marker -> +x (wall marker facing +x) and -> straight up (marker lying on the floor).
WALL_Q = (0.0, SQRT_HALF, 0.0, SQRT_HALF)
FLOOR_Q = (0.0, 0.0, 0.0, 1.0)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    robot = FakeRobot(tmp_path)
    robot.pois = [
        marker_poi("m1", "charger", 2.0, 1.0, 0.8, WALL_Q),
        marker_poi("m2", "base", 0.0, 0.0, 0.0, FLOOR_Q),
        {"id": "p1", "kind": "point", "name": "table", "x": 1.0, "y": 1.0, "status": "open"},
    ]
    return robot


@pytest.fixture
def server(robot: FakeRobot, tmp_path: Path) -> Any:
    config = McpServerConfig(objects={"store_path": tmp_path / "objects" / "objects.json"})
    return build_mcp_server(robot, config, TOKEN)


def views(result: Any) -> dict[str, dict[str, Any]]:
    return {v["id"]: v for v in result.structured_content["pois"]}


def test_wall_marker_facing_and_approach_pose() -> None:
    yaw, pose = marker_facing(marker_poi("m1", "charger", 2.0, 1.0, 0.8, WALL_Q), STANDOFF_M)
    assert yaw == pytest.approx(0.0)
    assert pose is not None
    assert (pose["x"], pose["y"]) == pytest.approx((2.6, 1.0))
    assert pose["yaw_deg"] == pytest.approx(180.0)


def test_floor_marker_has_no_facing() -> None:
    assert marker_facing(marker_poi("m2", "base", 0.0, 0.0, 0.0, FLOOR_Q), STANDOFF_M) == (None, None)


def test_list_pois_marker_fields(server: Any) -> None:
    by_id = views(call(server, "list_pois", {}))
    wall = by_id["m1"]
    assert wall["z"] == pytest.approx(0.8)
    assert wall["facing_yaw_deg"] == pytest.approx(0.0)
    assert wall["approach_pose"]["x"] == pytest.approx(2.6)
    assert wall["marker"]["kind"] == "charger"
    assert wall["marker"]["times_seen"] == 3 and wall["marker"]["size_mm"] == 150.0
    assert by_id["m2"]["facing_yaw_deg"] is None and by_id["m2"]["approach_pose"] is None
    assert by_id["m2"]["marker"]["kind"] == "base"


def test_point_view_has_null_marker_fields(server: Any) -> None:
    point = views(call(server, "list_pois", {}))["p1"]
    assert point["z"] is None and point["marker"] is None
    assert point["facing_yaw_deg"] is None and point["approach_pose"] is None


def test_list_pois_marker_kind_filter(server: Any) -> None:
    assert set(views(call(server, "list_pois", {"marker_kind": "charger"}))) == {"m1"}
    assert set(views(call(server, "list_pois", {}))) == {"m1", "m2", "p1"}


def test_add_poi_stays_point_or_area(server: Any) -> None:
    with pytest.raises(Exception, match="marker"):
        call(server, "add_poi", {"kind": "marker", "name": "x"})


def test_approach_marker_poi_takes_its_z(robot: FakeRobot) -> None:
    assert "marker" in POI_KINDS
    z_of = poi_z_lookup(robot, POI_KINDS)
    xy = {"m1": (2.0, 1.0), "p1": (1.0, 1.0)}

    def resolve(target: ApproachTarget) -> Any:
        return resolve_target(target, {}.get, xy.get, lambda *a: None, z_of)

    assert resolve(ApproachTarget(poi_id="m1")).z == pytest.approx(0.8)
    assert resolve(ApproachTarget(poi_id="m1", z_m=0.5)).z == pytest.approx(0.5)
    assert resolve(ApproachTarget(poi_id="p1", z_m=0.7)).z == pytest.approx(0.7)
    with pytest.raises(ValueError, match="z_m is required"):
        resolve(ApproachTarget(poi_id="p1"))
