"""Camera lookups end to end: the camera tools record every successful call, and the list/get/annotate tools."""

import base64
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import anyio
import cv2
import numpy as np
import pytest
from mcp.server.context import ServerRequestContext
from mcp.server.mcpserver import Context
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.lookup_store import INDEX_FILE
from mcp_server.metrics import LOOKUP_WRITE_FAILURES
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, TOOL_NAMES, build_mcp_server

from .camera_helpers import camera_config
from .test_camera_tools import CameraRobot
from .test_tools import TOKEN, FakeRobot, call

LOOKUP_TOOLS = {"list_lookups", "get_lookup_image", "annotate_lookup"}


def payload(res: Any) -> dict[str, Any]:
    """The tool's own JSON text block (not the digest block)."""
    for block in res.content:
        if block.type == "text":
            data = json.loads(block.text)
            if "robot_events_since_last_call" not in data:
                return data
    raise AssertionError("no payload text block")


def image_bytes(res: Any) -> bytes:
    return base64.b64decode(next(c for c in res.content if c.type == "image").data)


def index_rows(store_dir: Path) -> list[dict[str, Any]]:
    path = store_dir / INDEX_FILE
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def session_call(server: Any, session_id: str, name: str, args: dict[str, Any]) -> Any:
    """Call a tool as one MCP session (Mcp-Session-Id header)."""
    request_context = ServerRequestContext(
        session=None,  # type: ignore[arg-type]
        lifespan_context={},
        protocol_version="2025-11-25",
        method="tools/call",
        request=SimpleNamespace(headers={"mcp-session-id": session_id}),
    )
    context = Context(request_context=request_context, mcp_server=server)

    async def run() -> Any:
        return await server.call_tool(name, args, context)

    return anyio.run(run)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def store_dir(tmp_path: Path) -> Path:
    return tmp_path / "store"


@pytest.fixture
def server(robot: FakeRobot, store_dir: Path) -> Any:
    cfg = McpServerConfig()
    cfg.lookups.store_dir = store_dir
    return build_mcp_server(robot, cfg, TOKEN)


def test_lookup_tools_are_registered_read_only_tools() -> None:
    assert LOOKUP_TOOLS <= set(TOOL_NAMES)
    assert LOOKUP_TOOLS <= ALWAYS_ALLOWED_TOOLS
    assert not LOOKUP_TOOLS & MOTION_TOOLS


def test_default_store_dir_is_next_to_poi_json() -> None:
    from mcp_server import config

    assert config.LookupSettings.model_fields["max_records"].default == 3000
    assert config.LookupSettings.model_fields["max_bytes"].default == 300_000_000
    assert config.LookupSettings.model_fields["max_age_days"].default == 14.0
    assert str(config.DEFAULT_LOOKUPS_DIR).endswith("lookups")


def test_get_camera_image_writes_exactly_one_record_and_returns_lookup_id(
    server: Any, robot: FakeRobot, store_dir: Path
) -> None:
    res = call(server, "get_camera_image", {"camera": "gripper"})
    meta = payload(res)
    rows = index_rows(store_dir)
    assert len(rows) == 1
    row = rows[0]
    assert meta["lookup_id"] == row["id"]
    assert row["tool"] == "get_camera_image" and row["camera"] == "gripper"
    assert (row["width"], row["height"]) == (20, 10)
    assert (store_dir / row["file"]).read_bytes() == image_bytes(res)  # exactly the JPEG the model received
    assert row["bytes"] == len(image_bytes(res))
    assert row["base_pose_map"] == {"x": 1.0, "y": 2.0, "yaw": 0.5}
    assert set(row["arm_joints"]) >= {"shoulder_pan", "elbow_flex"}
    assert row["camera_pose_base"] is None  # no mount configured
    assert row["mcp_session"] == "" and row["note"] == "" and row["poi_ids"] == []


def test_failed_camera_call_writes_nothing(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    robot.camera_error = "no frame"
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "front"})
    assert index_rows(store_dir) == []


def test_session_id_reaches_the_record(server: Any, store_dir: Path) -> None:
    meta = payload(session_call(server, "sess-42", "get_camera_image", {"camera": "front"}))
    assert index_rows(store_dir)[0]["mcp_session"] == "sess-42"
    listed = payload(session_call(server, "other", "list_lookups", {"mcp_session": "sess-42"}))
    assert [r["id"] for r in listed["lookups"]] == [meta["lookup_id"]]


def test_write_failure_does_not_fail_the_camera_tool(robot: FakeRobot, tmp_path: Path) -> None:
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("a file where the store directory should be")
    cfg = McpServerConfig()
    cfg.lookups.store_dir = blocker / "lookups"
    server = build_mcp_server(robot, cfg, TOKEN)
    before = LOOKUP_WRITE_FAILURES._value.get()
    res = call(server, "get_camera_image", {"camera": "front"})
    assert any(c.type == "image" for c in res.content)
    assert payload(res)["lookup_id"] is None
    assert LOOKUP_WRITE_FAILURES._value.get() == before + 1


def test_pose_read_failure_still_records(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    def broken() -> None:
        raise RuntimeError("tf exploded")

    robot.robot_pose = broken  # type: ignore[method-assign]
    payload(call(server, "get_camera_image", {"camera": "front"}))
    assert index_rows(store_dir)[0]["base_pose_map"] is None


def test_annotated_camera_image_records_the_annotated_jpeg(tmp_path: Path) -> None:
    cfg = camera_config()
    cfg.cameras.calibration_dir = tmp_path / "calib"
    cfg.lookups.store_dir = tmp_path / "ann"
    robot = CameraRobot(tmp_path)
    server = build_mcp_server(robot, cfg, TOKEN)
    res = call(server, "get_annotated_camera_image", {"camera": "front"})
    rows = index_rows(tmp_path / "ann")
    assert len(rows) == 1
    row = rows[0]
    assert payload(res)["lookup_id"] == row["id"]
    assert row["tool"] == "get_annotated_camera_image" and row["camera"] == "front"
    assert (tmp_path / "ann" / row["file"]).read_bytes() == image_bytes(res)
    assert row["camera_pose_base"] is not None
    assert len(row["camera_pose_base"]["xyz"]) == 3 and len(row["camera_pose_base"]["rpy"]) == 3
    assert row["camera_pose_base"]["xyz"][2] > 0.1  # the front camera is mounted above the floor


def test_list_lookups_newest_first_metadata_only(server: Any) -> None:
    ids = [payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"] for _ in range(3)]
    res = call(server, "list_lookups", {})
    assert not [c for c in res.content if c.type == "image"]
    listed = res.structured_content["lookups"]
    assert [r["id"] for r in listed] == list(reversed(ids))
    assert {"id", "ts", "camera", "base_pose_map", "note", "poi_ids"} <= set(listed[0])
    assert [r["id"] for r in call(server, "list_lookups", {"limit": 2}).structured_content["lookups"]] == [
        ids[2],
        ids[1],
    ]
    with pytest.raises(ToolError):
        call(server, "list_lookups", {"limit": 21})


def test_list_lookups_filters(server: Any, robot: FakeRobot) -> None:
    robot.pois = [{"id": "dock"}]
    near_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]  # pose (1, 2)
    robot.pose = robot.pose.model_copy(update={"x": 10.0, "y": 10.0})
    far_id = payload(call(server, "get_camera_image", {"camera": "gripper"}))["lookup_id"]
    call(server, "annotate_lookup", {"lookup_id": near_id, "note": "Charger by the door", "poi_ids": ["dock"]})

    def listed(args: dict[str, Any]) -> list[str]:
        return [r["id"] for r in call(server, "list_lookups", args).structured_content["lookups"]]

    assert listed({"near": {"x": 1.0, "y": 2.0, "radius_m": 0.5}}) == [near_id]
    assert listed({"camera": "gripper"}) == [far_id]
    assert listed({"poi_id": "dock"}) == [near_id]
    assert listed({"text": "charger"}) == [near_id]
    assert listed({"since": 0.0, "until": 1.0}) == []
    assert set(listed({"since": 0.0})) == {near_id, far_id}


def test_get_lookup_image_returns_the_image_and_metadata(server: Any) -> None:
    taken = call(server, "get_camera_image", {"camera": "front"})
    lookup_id = payload(taken)["lookup_id"]
    res = call(server, "get_lookup_image", {"lookup_id": lookup_id})
    img = next(c for c in res.content if c.type == "image")
    assert img.mime_type == "image/jpeg"
    assert base64.b64decode(img.data) == image_bytes(taken)
    meta = payload(res)
    assert meta["id"] == lookup_id and meta["camera"] == "front"


def test_get_lookup_image_max_px_downscales(server: Any) -> None:
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]  # 20x10 frame
    res = call(server, "get_lookup_image", {"lookup_id": lookup_id, "max_px": 10})
    arr = cv2.imdecode(np.frombuffer(image_bytes(res), np.uint8), cv2.IMREAD_COLOR)
    assert arr.shape[:2] == (5, 10)
    assert payload(res)["returned_size"] == {"width": 10, "height": 5}


def test_get_lookup_image_unknown_id_is_a_tool_error(server: Any) -> None:
    with pytest.raises(ToolError, match="unknown lookup"):
        call(server, "get_lookup_image", {"lookup_id": "0" * 32})


def test_annotate_lookup_rewrites_the_line(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    robot.pois = [{"id": "p1"}]
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    res = call(server, "annotate_lookup", {"lookup_id": lookup_id, "note": "mug", "poi_ids": ["p1"]})
    assert res.structured_content["note"] == "mug" and res.structured_content["poi_ids"] == ["p1"]
    rows = index_rows(store_dir)
    assert len(rows) == 1 and rows[0]["note"] == "mug" and rows[0]["poi_ids"] == ["p1"]
    with pytest.raises(ToolError, match="unknown lookup"):
        call(server, "annotate_lookup", {"lookup_id": "1" * 32, "note": "x"})


def test_annotate_lookup_rejects_dangling_poi_ids(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    robot.pois = [{"id": "known"}]
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    with pytest.raises(ToolError, match="unknown POI id.*ghost"):
        call(server, "annotate_lookup", {"lookup_id": lookup_id, "poi_ids": ["known", "ghost"]})
    assert index_rows(store_dir)[0]["poi_ids"] == []
    res = call(server, "annotate_lookup", {"lookup_id": lookup_id, "poi_ids": ["known"]})
    assert res.structured_content["poi_ids"] == ["known"]


def test_annotate_lookup_without_poi_store_keeps_the_ids_unverified(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    res = call(server, "annotate_lookup", {"lookup_id": lookup_id, "poi_ids": ["anything"]})
    assert res.structured_content["poi_ids"] == ["anything"]


def test_annotate_lookup_clearing_links_needs_no_poi_store(server: Any, robot: FakeRobot) -> None:
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    robot.poi_store_up = False
    assert call(server, "annotate_lookup", {"lookup_id": lookup_id, "poi_ids": []}).structured_content["poi_ids"] == []


def test_since_accepts_iso_string(server: Any) -> None:
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    res = call(server, "list_lookups", {"since": "2000-01-01T00:00:00Z", "until": "2999-01-01T00:00:00+00:00"})
    assert [r["id"] for r in res.structured_content["lookups"]] == [lookup_id]
    assert call(server, "list_lookups", {"since": "2999-01-01T00:00:00Z"}).structured_content["count"] == 0
    with pytest.raises(ToolError, match="ISO-8601"):
        call(server, "list_lookups", {"since": "yesterday"})


def test_list_lookups_description_names_every_returned_field(server: Any) -> None:
    tool = next(t for t in anyio.run(server.list_tools) if t.name == "list_lookups")
    payload(call(server, "get_camera_image", {"camera": "front"}))
    row = call(server, "list_lookups", {}).structured_content["lookups"][0]
    missing = [key for key in row if key not in tool.description]
    assert not missing, missing


def test_text_filter_without_notes_explains_itself(server: Any) -> None:
    payload(call(server, "get_camera_image", {"camera": "front"}))
    res = call(server, "list_lookups", {"text": "mug"})
    assert res.structured_content["count"] == 0
    assert "note" in res.structured_content["hint"] and "annotate_lookup" in res.structured_content["hint"]
    assert "hint" not in call(server, "list_lookups", {}).structured_content
