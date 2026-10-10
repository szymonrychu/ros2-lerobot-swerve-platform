"""Tool-level tests of the perception/memory/POI module: get_topdown_view, object memory, look_around, POI tools."""

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server import perception_tools
from mcp_server.base_motion import SpinOutcome
from mcp_server.config import McpServerConfig
from mcp_server.models import BasePose, NavigationResult
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, build_mcp_server

from .test_tools import TOKEN, FakeRobot, call, tool_descriptions


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot, tmp_path: Path) -> Any:
    config = McpServerConfig(objects={"store_path": tmp_path / "objects" / "objects.json"})
    return build_mcp_server(robot, config, TOKEN)


def decode(content: Any) -> np.ndarray:
    import base64

    return cv2.imdecode(np.frombuffer(base64.b64decode(content.data), np.uint8), cv2.IMREAD_COLOR)


def test_classification() -> None:
    assert "look_around" in MOTION_TOOLS
    for name in (
        "get_topdown_view",
        "remember_object",
        "list_objects",
        "forget_object",
        "list_pois",
        "add_poi",
        "update_poi",
        "delete_poi",
    ):
        assert name in ALWAYS_ALLOWED_TOOLS and name not in MOTION_TOOLS


def test_descriptions_document_the_view_conventions(server: Any) -> None:
    docs = tool_descriptions(server)
    assert "robot-up" in docs["get_topdown_view"] and "0.5 m" in docs["get_topdown_view"]
    assert "ONE motion" in docs["look_around"] or "one motion" in docs["look_around"]
    assert "poi_store" in docs["add_poi"]


# --- get_topdown_view ------------------------------------------------------------------------------------------


def test_topdown_returns_png_and_metadata_with_missing_layers(server: Any) -> None:
    res = call(server, "get_topdown_view", {"radius_m": 2.0, "px": 400})
    image = next(c for c in res.content if c.type == "image")
    assert image.mime_type == "image/png" and decode(image).shape == (400, 400, 3)
    meta = res.structured_content
    assert meta["pose"]["x"] == 1.0
    assert meta["scale_m_per_px"] == pytest.approx(0.01)
    assert "lidar" in meta["layers_present"] and "path" in meta["layers_present"]
    assert "costmap" in meta["layers_missing"] and "1.5 s" in meta["layers_missing"]["costmap"]
    assert "map" in meta["layers_missing"]
    assert meta["data_ages"]["scan"] == pytest.approx(0.1)
    assert meta["orientation"].startswith("robot-up")


def test_topdown_reach_circle_is_centred_on_the_configured_shoulder_pan_axis() -> None:
    """The reach circle sits on the shoulder_pan axis: the arm mount plus the URDF pan joint origin (38.8 mm forward)."""
    style = perception_tools.topdown_style(McpServerConfig())
    assert (style.reach_x_m, style.reach_y_m) == pytest.approx((0.0592 + 0.0388353, -0.05))
    turned = McpServerConfig.model_validate(
        {"arm": {"base_in_base_link": {"x": 0.1, "y": 0.0, "z": 0.100, "yaw": 1.5708}}}
    )
    style = perception_tools.topdown_style(turned)
    assert (style.reach_x_m, style.reach_y_m) == pytest.approx((0.1, 0.0388353), abs=1e-5)
    unmounted = McpServerConfig.model_validate({"arm": {"base_in_base_link": None}})
    style = perception_tools.topdown_style(unmounted)
    assert (style.reach_x_m, style.reach_y_m) == (0.0, 0.0)


def test_topdown_layer_selection_and_validation(server: Any) -> None:
    res = call(server, "get_topdown_view", {"layers": ["footprint"]})
    assert res.structured_content["layers_present"] == ["footprint"]
    assert res.structured_content["layers_missing"] == {}
    with pytest.raises(ToolError):
        call(server, "get_topdown_view", {"layers": ["weather"]})
    with pytest.raises(ToolError):
        call(server, "get_topdown_view", {"px": 4000})


def test_topdown_includes_pois_and_objects_when_available(server: Any, robot: FakeRobot) -> None:
    robot.pois.append(
        {"id": "a", "kind": "point", "name": "dock", "x": 1.5, "y": 2.0, "radius_m": 0.2, "status": "open"}
    )
    call(server, "remember_object", {"label": "cup", "x": 1.5, "y": 2.2})
    res = call(server, "get_topdown_view", {"layers": ["pois", "objects"]})
    assert res.structured_content["layers_present"] == ["pois", "objects"]


def test_topdown_draws_each_poi_once_objects_only_in_the_objects_layer(
    server: Any, robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    seen: dict[str, Any] = {}
    real = perception_tools.render_topdown

    def spy(*args: Any, **kwargs: Any) -> Any:
        seen.update(kwargs)
        return real(*args, **kwargs)

    monkeypatch.setattr(perception_tools, "render_topdown", spy)
    robot.pois.append({"id": "a", "kind": "point", "name": "dock", "x": 1.5, "y": 2.0, "radius_m": 0.2})
    call(server, "remember_object", {"label": "cup", "x": 1.5, "y": 2.2})
    call(server, "get_topdown_view", {"layers": ["pois", "objects"]})
    assert [p["name"] for p in seen["pois"]] == ["dock"]
    assert [o["label"] for o in seen["objects"]] == ["cup"]
    call(server, "get_topdown_view", {"layers": ["objects"]})
    assert seen["pois"] is None and [o["label"] for o in seen["objects"]] == ["cup"]


def test_topdown_object_layer_missing_when_store_down(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    res = call(server, "get_topdown_view", {"layers": ["pois", "objects"]})
    assert "poi_store" in res.structured_content["layers_missing"]["objects"]
    assert "poi_store" in res.structured_content["layers_missing"]["pois"]


def test_topdown_poi_layer_missing_when_store_down(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    res = call(server, "get_topdown_view", {"layers": ["pois"]})
    assert "poi_store" in res.structured_content["layers_missing"]["pois"]


# --- object memory ---------------------------------------------------------------------------------------------


def test_object_round_trip_and_merge(server: Any) -> None:
    first = call(server, "remember_object", {"label": "cup", "x": 3.0, "y": 2.0, "note": "blue"}).structured_content
    assert first["times_seen"] == 1 and first["confidence"] == 0.7 and first["note"] == "blue"
    again = call(server, "remember_object", {"label": "cup", "x": 3.1, "y": 2.0}).structured_content
    assert again["id"] == first["id"] and again["times_seen"] == 2
    listed = call(server, "list_objects", {}).structured_content["objects"]
    assert len(listed) == 1
    assert listed[0]["distance_m"] == pytest.approx(((3.05 - 1.0) ** 2 + 0.0) ** 0.5, abs=0.05)
    assert "bearing_deg" in listed[0]
    forgotten = call(server, "forget_object", {"id": first["id"]}).structured_content
    assert forgotten["id"] == first["id"]
    assert call(server, "list_objects", {}).structured_content["objects"] == []


def test_objects_are_agent_made_object_pois_in_poi_store(server: Any, robot: FakeRobot) -> None:
    obj = call(server, "remember_object", {"label": "cup", "x": 3.0, "y": 2.0}).structured_content
    (poi,) = robot.pois
    assert poi["id"] == obj["id"] and poi["kind"] == "object" and poi["name"] == "cup"
    assert poi["created_by"] == "agent" and poi["times_seen"] == 1
    robot.pois.append({"id": "z", "kind": "point", "name": "dock", "x": 0.0, "y": 0.0, "status": "open"})
    assert [o["label"] for o in call(server, "list_objects", {}).structured_content["objects"]] == ["cup"]
    kinds = {p["name"]: p["kind"] for p in call(server, "list_pois", {}).structured_content["pois"]}
    assert kinds == {"cup": "object", "dock": "point"}
    assert "object" in tool_descriptions(server)["list_pois"]


def test_legacy_objects_json_is_imported_into_poi_store(robot: FakeRobot, tmp_path: Path) -> None:
    legacy = tmp_path / "objects" / "objects.json"
    legacy.parent.mkdir()
    legacy.write_text(
        json.dumps(
            {
                "objects": [
                    {
                        "id": "a" * 32,
                        "label": "key",
                        "x": 4.0,
                        "y": 5.0,
                        "confidence": 0.5,
                        "first_seen": 1.0,
                        "last_seen": 2.0,
                        "times_seen": 2,
                    }
                ]
            }
        )
    )
    server = build_mcp_server(robot, McpServerConfig(objects={"store_path": legacy}), TOKEN)
    (obj,) = call(server, "list_objects", {}).structured_content["objects"]
    assert obj["label"] == "key" and obj["times_seen"] == 2
    assert not legacy.exists() and legacy.with_name("objects.json.migrated").exists()


def test_object_validation_errors_are_tool_errors(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "remember_object", {"label": "cup", "x": 1.0, "y": 1.0, "frame": "odom"})
    with pytest.raises(ToolError):
        call(server, "remember_object", {"label": "cup", "x": 1.0, "y": 1.0, "confidence": 2.0})
    with pytest.raises(ToolError, match="unknown object id"):
        call(server, "forget_object", {"id": "nope"})
    with pytest.raises(ToolError, match="near_x and near_y"):
        call(server, "list_objects", {"near_x": 1.0})


def test_list_objects_filters(server: Any) -> None:
    call(server, "remember_object", {"label": "red cup", "x": 1.0, "y": 2.0})
    call(server, "remember_object", {"label": "key", "x": 9.0, "y": 9.0})
    names = [o["label"] for o in call(server, "list_objects", {"label_contains": "cup"}).structured_content["objects"]]
    assert names == ["red cup"]
    near = call(server, "list_objects", {"near_x": 9.0, "near_y": 9.0, "radius_m": 0.5}).structured_content["objects"]
    assert [o["label"] for o in near] == ["key"]


# --- POI tools -------------------------------------------------------------------------------------------------


def test_add_poi_point_defaults_to_robot_position_and_agent_creator(server: Any, robot: FakeRobot) -> None:
    res = call(server, "add_poi", {"kind": "point", "name": "dock", "note": "charge here"}).structured_content
    op, poi = robot.poi_commands[-1]
    assert op == "add"
    assert poi["x"] == 1.0 and poi["y"] == 2.0 and poi["created_by"] == "agent" and poi["kind"] == "point"
    assert poi["name"] == "dock" and poi["note"] == "charge here" and poi["frame"] == "map"
    assert res["ok"] is True and res["poi"]["name"] == "dock"


def test_add_poi_explicit_position_radius_and_area(server: Any, robot: FakeRobot) -> None:
    call(server, "add_poi", {"kind": "point", "name": "a", "x": 5.0, "y": 6.0, "radius_m": 0.5})
    assert robot.poi_commands[-1][1]["x"] == 5.0 and robot.poi_commands[-1][1]["radius_m"] == 0.5
    call(server, "add_poi", {"kind": "area", "name": "rug", "polygon": [[0, 0], [1, 0], [1, 1]]})
    poi = robot.poi_commands[-1][1]
    assert poi["kind"] == "area" and poi["polygon"] == [[0, 0], [1, 0], [1, 1]]
    assert "x" not in poi and "radius_m" not in poi  # the store computes the centroid


def test_add_poi_argument_errors(server: Any) -> None:
    with pytest.raises(ToolError, match="polygon"):
        call(server, "add_poi", {"kind": "area", "name": "rug"})
    with pytest.raises(ToolError, match="polygon"):
        call(server, "add_poi", {"kind": "point", "name": "x", "polygon": [[0, 0], [1, 0], [1, 1]]})
    with pytest.raises(ToolError, match="both"):
        call(server, "add_poi", {"kind": "point", "name": "x", "x": 1.0})


def test_update_and_delete_poi(server: Any, robot: FakeRobot) -> None:
    created = call(server, "add_poi", {"kind": "point", "name": "dock"}).structured_content["poi"]
    call(server, "update_poi", {"id": created["id"], "status": "done", "note": "ok"})
    assert robot.poi_commands[-1] == ("update", {"id": created["id"], "status": "done", "note": "ok"})
    with pytest.raises(ToolError, match="nothing to update"):
        call(server, "update_poi", {"id": created["id"]})
    with pytest.raises(ToolError):
        call(server, "update_poi", {"id": created["id"], "status": "finished"})
    call(server, "delete_poi", {"id": created["id"]})
    assert robot.poi_commands[-1] == ("delete", {"id": created["id"]})
    with pytest.raises(ToolError, match="unknown poi id"):
        call(server, "delete_poi", {"id": created["id"]})


def test_poi_tools_error_when_store_not_running(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    for name, args in (
        ("list_pois", {}),
        ("add_poi", {"kind": "point", "name": "x"}),
        ("update_poi", {"id": "a", "note": "n"}),
        ("delete_poi", {"id": "a"}),
    ):
        with pytest.raises(ToolError, match="poi_store is not running"):
            call(server, name, args)


def test_list_pois_with_distance_bearing_filter_and_near(server: Any, robot: FakeRobot) -> None:
    robot.pois += [
        {"id": "p1", "kind": "point", "name": "far", "x": 21.0, "y": 2.0, "status": "open"},
        {"id": "p2", "kind": "point", "name": "ahead", "x": 2.0, "y": 2.0, "status": "done"},
        {
            "id": "p3",
            "kind": "area",
            "name": "rug",
            "x": 1.0,
            "y": 3.0,
            "polygon": [[0.5, 2.5], [1.5, 2.5], [1.5, 3.5], [0.5, 3.5]],
            "status": "open",
        },
    ]
    allp = call(server, "list_pois", {}).structured_content["pois"]
    assert {p["id"] for p in allp} == {"p1", "p2", "p3"}
    ahead = next(p for p in allp if p["id"] == "p2")
    assert ahead["distance_m"] == pytest.approx(1.0) and ahead["bearing_deg"] == pytest.approx(-28.6479, abs=0.06)
    open_only = call(server, "list_pois", {"status": "open"}).structured_content["pois"]
    assert {p["id"] for p in open_only} == {"p1", "p3"}
    near = call(server, "list_pois", {"near": True}).structured_content["pois"]
    assert {p["id"] for p in near} == {"p2", "p3"}  # p1 is 20 m away, beyond poi.near_radius_m
    distances = [p["distance_m"] for p in near]
    assert distances == sorted(distances)


# --- look_around -----------------------------------------------------------------------------------------------


class RotatingRobot(FakeRobot):
    """FakeRobot whose move_relative really turns the pose."""

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("move_relative", (dx, dy, dyaw, timeout_s)))
        self.pose = BasePose(frame="map", x=self.pose.x, y=self.pose.y, yaw=self.pose.yaw + dyaw)
        return NavigationResult(status="succeeded", final_pose=self.pose)

    def spin(self, wz: float, angle_rad: float, marks_rad: list[float], on_mark: Any, timeout_s: float) -> SpinOutcome:
        self.calls.append(("spin", (wz, angle_rad, tuple(marks_rad), timeout_s)))
        start = self.pose.yaw
        for index, mark in enumerate(marks_rad):
            self.pose = BasePose(frame="map", x=self.pose.x, y=self.pose.y, yaw=start + mark)
            on_mark(index)
        self.pose = BasePose(frame="map", x=self.pose.x, y=self.pose.y, yaw=start + angle_rad)
        return SpinOutcome(status="completed", commanded_wz=wz, rotated_rad=angle_rad, marks_reached=len(marks_rad))


def test_look_around_tool_returns_montage_summary_and_topdown(tmp_path: Path) -> None:
    robot = RotatingRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    res = call(server, "look_around", {"captures": 4, "camera": "gripper", "mode": "steps", "return_to_start": True})
    images = [c for c in res.content if c.type == "image"]
    assert len(images) == 2
    assert images[0].mime_type == "image/jpeg" and images[1].mime_type == "image/png"
    sc = res.structured_content
    assert sc["status"] == "completed" and sc["returned_to_start"] is True
    assert len(sc["headings"]) == 4 and sc["headings"][1]["heading_deg"] == 90
    assert sc["expected"]["rotation_deg"] == 360 and "achieved" in sc
    assert sc["topdown"]["layers_present"]
    moves = [c for c in robot.calls if c[0] == "move_relative"]
    assert len(moves) == 4
    cams = [c[1][0] for c in robot.calls if c[0] == "camera_image"]
    assert cams == ["gripper"] * 4


def test_look_around_tool_defaults_to_one_spin_without_returning(tmp_path: Path) -> None:
    robot = RotatingRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    sc = call(server, "look_around", {"captures": 4}).structured_content
    spins = [c for c in robot.calls if c[0] == "spin"]
    assert len(spins) == 1 and not [c for c in robot.calls if c[0] == "move_relative"]
    assert sc["status"] == "completed" and sc["returned_to_start"] is False
    assert sc["expected"]["mode"] == "spin" and sc["expected"]["rotation_deg"] == 270
    assert [h["heading_deg"] for h in sc["headings"]] == [0, 90, 180, 270]
    back = call(server, "look_around", {"captures": 4, "return_to_start": True}).structured_content
    assert back["returned_to_start"] is True


def test_look_around_defaults_come_from_config(tmp_path: Path) -> None:
    robot = RotatingRobot(tmp_path)
    config = McpServerConfig(look_around={"default_captures": 3})
    server = build_mcp_server(robot, config, TOKEN)
    res = call(server, "look_around", {})
    assert len(res.structured_content["headings"]) == 3


def test_look_around_validates_arguments(tmp_path: Path) -> None:
    server = build_mcp_server(RotatingRobot(tmp_path), McpServerConfig(), TOKEN)
    for args in ({"captures": 2}, {"captures": 40}, {"camera": "thermal"}):
        with pytest.raises(ToolError):
            call(server, "look_around", args)


def test_look_around_refusal_is_a_tool_error_and_does_not_move(tmp_path: Path) -> None:
    from mcp_server.models import MapSummary, SectorObstacle

    robot = RotatingRobot(tmp_path)
    robot.map_summary = lambda *a, **k: (  # type: ignore[method-assign]
        MapSummary(obstacles=[SectorObstacle(sector="left", nearest_m=0.3, bearing_rad=1.5)]),
        None,
    )
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    with pytest.raises(ToolError, match="rotation"):
        call(server, "look_around", {})
    assert not [c for c in robot.calls if c[0] == "move_relative"]


def test_digest_is_attached_to_the_new_tools(server: Any) -> None:
    res = call(server, "list_objects", {})
    assert "robot_events_since_last_call" in res.structured_content
    assert json.dumps(res.structured_content)
