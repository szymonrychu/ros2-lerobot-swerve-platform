"""get_camera_image with the optional zoom parameter, on a fake robot (head camera zoom crops)."""

import base64
import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.tools import build_mcp_server
from mcp_server.zoom_client import ZoomFailed, ZoomNodeDown, ZoomTimeout

from .test_tools import TOKEN, FakeRobot, call

ZOOM = {"u": 0.25, "v": 0.75, "factor": 4}


@pytest.fixture
def lookup_robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(lookup_robot: FakeRobot, tmp_path: Path) -> Any:
    cfg = McpServerConfig()
    cfg.lookups.store_dir = tmp_path / "lookups"
    return build_mcp_server(lookup_robot, cfg, TOKEN)


def texts(res: Any) -> dict[str, Any]:
    for block in res.content:
        if block.type == "text":
            data = json.loads(block.text)
            if "robot_events_since_last_call" not in data:
                return data
    raise AssertionError("no payload text block")


def test_zoom_on_front_calls_camera_zoom_with_default_max_px(server: Any, lookup_robot: FakeRobot) -> None:
    res = call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM})
    assert lookup_robot.zoom_calls == [("front", 0.25, 0.75, 4, 1024)]
    assert ("camera_image", ("front", 384)) not in lookup_robot.calls
    image = next(c for c in res.content if c.type == "image")
    assert base64.b64decode(image.data) == lookup_robot.zoom_result.jpeg
    meta = texts(res)
    assert meta["status"] == "ok" and meta["crop"] == [0, 0, 1152, 648] and meta["full"] == [4608, 2592]
    assert meta["out"] == [1024, 576] and (meta["width"], meta["height"]) == (1024, 576)
    assert meta["af_state"] == "focused" and meta["lens_position"] == 4.0 and "af" in meta["phase_ms"]
    assert meta["stamp_s"] == 1.0
    assert meta["request"] == {"u": 0.25, "v": 0.75, "zoom": 4}
    assert "lookup_id" not in meta


def test_zoom_passes_max_px(server: Any, lookup_robot: FakeRobot) -> None:
    call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM, "max_px": 512})
    assert lookup_robot.zoom_calls[0][4] == 512


def test_zoom_on_gripper_is_a_tool_error_and_calls_nothing(server: Any, lookup_robot: FakeRobot) -> None:
    with pytest.raises(ToolError, match="'front'"):  # the schema offers the front camera only
        call(server, "get_camera_image", {"camera": "gripper", "zoom": ZOOM})
    assert lookup_robot.zoom_calls == [] and lookup_robot.calls == []


@pytest.mark.parametrize(
    "zoom", [{"u": 0.5, "v": 0.5, "factor": 3}, {"u": 1.5, "v": 0.5, "factor": 2}, {"u": 0.5, "v": -0.1, "factor": 8}]
)
def test_invalid_zoom_is_rejected(server: Any, lookup_robot: FakeRobot, zoom: dict[str, float]) -> None:
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "front", "zoom": zoom})
    assert lookup_robot.zoom_calls == []


def test_af_failed_returns_the_image_and_a_note(server: Any, lookup_robot: FakeRobot) -> None:
    lookup_robot.zoom_result = lookup_robot.zoom_result.model_copy(update={"status": "af_failed", "af_state": "failed"})
    res = call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM})
    assert any(c.type == "image" for c in res.content)
    meta = texts(res)
    assert meta["status"] == "af_failed" and "did not converge" in meta["note"] and "again" in meta["note"]


def test_ok_has_no_af_note(server: Any) -> None:
    assert "note" not in texts(call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM}))


@pytest.mark.parametrize(
    "error, text",
    [
        (ZoomFailed("busy", "another zoom is running"), "busy"),
        (ZoomFailed("camera_error", "boom"), "camera_error"),
        (ZoomTimeout("head camera did not answer within 8 s"), "did not answer"),
        (ZoomNodeDown("head camera is not running"), "not running"),
    ],
)
def test_zoom_failures_become_tool_errors(server: Any, lookup_robot: FakeRobot, error: Exception, text: str) -> None:
    lookup_robot.zoom_result = error
    with pytest.raises(ToolError, match=text):
        call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM})


def test_zoom_is_never_recorded_as_a_lookup(server: Any, lookup_robot: FakeRobot, tmp_path: Path) -> None:
    call(server, "get_camera_image", {"camera": "front", "zoom": ZOOM})
    assert lookup_robot.photo_commands == []
    assert not (tmp_path / "lookups" / "index.jsonl").exists()


def test_plain_call_is_unchanged_and_records_a_lookup(server: Any, lookup_robot: FakeRobot) -> None:
    meta = texts(call(server, "get_camera_image", {"camera": "front"}))
    assert ("camera_image", ("front", McpServerConfig().limits.default_camera_px)) in lookup_robot.calls
    assert lookup_robot.zoom_calls == []
    assert meta["lookup_id"] and meta["width"] == 20


def test_description_names_head_camera_and_zoom(server: Any) -> None:
    import anyio

    tools = anyio.run(server.list_tools)
    tool = next(t for t in tools if t.name == "get_camera_image")
    text = tool.description + json.dumps(tool.input_schema)
    assert "overhead" not in text and "640x480" not in text
    assert "head" in text and "zoom" in text and "1024" in text and "384x216" in text
