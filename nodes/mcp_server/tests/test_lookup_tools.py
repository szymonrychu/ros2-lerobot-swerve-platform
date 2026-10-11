"""Photos (photo POIs) end to end: the camera tools write the JPEG and add a photo POI through poi_store, and the
list / get / annotate / delete_photos tools.

The FakeRobot's photo store (tests/fakes.py FakePhotoStore) plays poi_store: it owns index.jsonl and records every
photo command it receives (robot.photo_commands).
"""

import base64
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import anyio
import cv2
import numpy as np
import pytest
from mcp.server.context import ServerRequestContext
from mcp.server.mcpserver import Context
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.photos import PhotoAdd, PhotoSelector

from mcp_server import lookup_tools
from mcp_server.config import McpServerConfig
from mcp_server.lookup_store import INDEX_FILE, LookupRecord
from mcp_server.lookup_tools import LookupRecorder, make_recorder, record_lookup
from mcp_server.metrics import LOOKUP_WRITE_FAILURES
from mcp_server.models import RobotError
from mcp_server.poi_client import PoiStoreDown, PoiTimeout
from mcp_server.tool_context import ToolContext
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, TOOL_NAMES, build_mcp_server

from .camera_helpers import HFOV_DEG, camera_config
from .test_camera_tools import CameraRobot
from .test_tools import TOKEN, FakeRobot, call

LOOKUP_TOOLS = {"list_lookups", "get_lookup_image", "annotate_lookup", "delete_photos"}
DAY_S = 86400.0
JPEG = b"\xff\xd8\xff\xe0" + b"x" * 96


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


def jpegs(store_dir: Path) -> list[Path]:
    return sorted(store_dir.glob("*.jpg")) if store_dir.exists() else []


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
def store_dir(robot: FakeRobot) -> Path:
    return robot.photo_store.root  # the default lookups dir (conftest), where poi_store's fake indexes photos


@pytest.fixture
def server(robot: FakeRobot, store_dir: Path) -> Any:
    cfg = McpServerConfig()
    cfg.lookups.store_dir = store_dir
    return build_mcp_server(robot, cfg, TOKEN)


def photo_adds(robot: FakeRobot) -> list[dict[str, Any]]:
    return [poi for op, poi in robot.photo_commands if op == "add"]


def test_lookup_tools_are_registered_read_only_tools() -> None:
    assert LOOKUP_TOOLS <= set(TOOL_NAMES)
    assert LOOKUP_TOOLS <= ALWAYS_ALLOWED_TOOLS
    assert not LOOKUP_TOOLS & MOTION_TOOLS


def test_default_store_dir_is_next_to_poi_json() -> None:
    from mcp_server import config

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
    assert (row["kind"], row["status"], row["created_by"]) == ("photo", "open", "agent")
    # no gripper mount: the view is the base position and heading, nothing invented
    assert row["view"] == {
        "x": 1.0,
        "y": 2.0,
        "z": None,
        "yaw": 0.5,
        "pitch": None,
        "hfov": None,
        "source": "base_heading",
    }


def test_failed_camera_call_writes_nothing(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    robot.camera_error = "no frame"
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "front"})
    assert index_rows(store_dir) == [] and robot.photo_commands == []


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
    assert robot.photo_commands == []  # no JPEG, no photo POI


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
    robot.photo_store.root = tmp_path / "ann"
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
    view = row["view"]
    assert view["source"] == "camera_mount"
    assert view["hfov"] == pytest.approx(math.radians(HFOV_DEG))  # 2*atan(w/(2*fx)) of the configured intrinsics
    assert view["pitch"] == pytest.approx(1.0, abs=1e-3)  # the mount looks 1 rad down
    assert view["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)  # robot heading +y, camera straight ahead


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
    res = call(server, "annotate_lookup", {"lookup_id": lookup_id, "note": "mug", "poi_ids": ["p1", "p1"]})
    assert res.structured_content["note"] == "mug" and res.structured_content["poi_ids"] == ["p1"]
    assert robot.photo_commands[-1] == ("update", {"id": lookup_id, "note": "mug", "poi_ids": ["p1"]})
    assert set(payload(res)) == set(call(server, "list_lookups", {}).structured_content["lookups"][0])
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


def test_annotate_lookup_errors_when_poi_store_is_down(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    robot.poi_store_up = False
    for args in ({"note": "mug"}, {"poi_ids": []}, {"poi_ids": ["anything"]}):
        with pytest.raises(ToolError, match="poi_store is not running"):
            call(server, "annotate_lookup", {"lookup_id": lookup_id, **args})
    assert index_rows(store_dir)[0]["note"] == ""


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


# --- recorder: JPEG first, then the photo POI through poi_store --------------------------------------------------


def recorder(robot: FakeRobot, cfg: McpServerConfig | None = None) -> LookupRecorder:
    cfg = cfg or McpServerConfig()
    cfg.lookups.store_dir = robot.photo_store.root
    return make_recorder(robot, cfg)


def test_record_sends_a_valid_photo_add_after_writing_the_jpeg(
    robot: FakeRobot, store_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    seen: list[bool] = []
    real = robot.photo_request

    def spy(op: str, poi: dict) -> dict:
        seen.append((store_dir / f"{poi.get('id')}.jpg").exists())  # the JPEG is on disk before the add
        return real(op, poi)

    monkeypatch.setattr(robot, "photo_request", spy)
    lookup_id = recorder(robot).record("get_camera_image", "front", JPEG, 20, 10)
    [add] = photo_adds(robot)
    assert seen == [True]
    assert add["kind"] == "photo" and add["id"] == lookup_id and add["file"] == f"{lookup_id}.jpg"
    photo = PhotoAdd.model_validate({k: v for k, v in add.items() if k != "kind"})
    assert (photo.tool, photo.camera, photo.width, photo.height) == ("get_camera_image", "front", 20, 10)
    assert photo.created_by == "agent" and "bytes" not in add  # poi_store sets bytes from the file
    assert (store_dir / photo.file).read_bytes() == JPEG
    assert [r["id"] for r in index_rows(store_dir)] == [lookup_id]


def test_full_frame_false_leaves_hfov_unknown(tmp_path: Path) -> None:
    robot = CameraRobot(tmp_path)
    cfg = camera_config()
    rec = recorder(robot, cfg)
    rec.record("get_camera_image", "front", JPEG, 640, 480)
    rec.record("grasp_object", "front", JPEG, 64, 48, full_frame=False)
    full, crop = (add["view"] for add in photo_adds(robot))
    assert full["hfov"] == pytest.approx(math.radians(HFOV_DEG))
    assert crop["hfov"] is None and crop["source"] == "camera_mount"  # pose still known, FOV of a crop is not


def test_hfov_unknown_without_intrinsics(robot: FakeRobot) -> None:
    recorder(robot).record("get_camera_image", "front", JPEG, 20, 10)  # default config: no intrinsics
    assert photo_adds(robot)[0]["view"]["hfov"] is None


def test_no_map_pose_gives_no_view(robot: FakeRobot, store_dir: Path) -> None:
    robot.pose = None
    recorder(robot).record("get_camera_image", "front", JPEG, 20, 10)
    add = photo_adds(robot)[0]
    assert add["view"] is None and add["base_pose_map"] is None
    assert index_rows(store_dir)[0]["view"] is None


@pytest.mark.parametrize(
    "error",
    [PoiStoreDown("poi_store is not running"), RobotError("poi_store rejected the add: id exists"), RuntimeError("x")],
)
def test_store_down_or_rejection_removes_the_jpeg_and_counts(
    robot: FakeRobot, store_dir: Path, error: Exception
) -> None:
    robot.photo_error = error
    before = LOOKUP_WRITE_FAILURES._value.get()
    assert recorder(robot).record("get_camera_image", "front", JPEG, 20, 10) is None
    assert jpegs(store_dir) == [] and index_rows(store_dir) == []
    assert LOOKUP_WRITE_FAILURES._value.get() == before + 1


def test_poi_store_not_running_removes_the_jpeg(robot: FakeRobot, store_dir: Path) -> None:
    robot.poi_store_up = False
    assert recorder(robot).record("get_camera_image", "front", JPEG, 20, 10) is None
    assert jpegs(store_dir) == []


def test_timeout_keeps_the_jpeg_and_counts(robot: FakeRobot, store_dir: Path) -> None:
    robot.photo_error = PoiTimeout("poi_store did not answer within 3 s")
    before = LOOKUP_WRITE_FAILURES._value.get()
    assert recorder(robot).record("get_camera_image", "front", JPEG, 20, 10) is None
    [kept] = jpegs(store_dir)  # poi_store may still index it; otherwise its orphan sweep removes it
    assert kept.read_bytes() == JPEG
    assert LOOKUP_WRITE_FAILURES._value.get() == before + 1


def test_camera_tool_succeeds_when_poi_store_is_down(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    robot.poi_store_up = False
    res = call(server, "get_camera_image", {"camera": "front"})
    assert any(c.type == "image" for c in res.content)
    assert payload(res)["lookup_id"] is None and jpegs(store_dir) == []


def test_record_lookup_keeps_its_positional_contract(robot: FakeRobot) -> None:
    calls: list[tuple[Any, ...]] = []

    class Spy:
        def record(self, *args: Any, **kwargs: Any) -> str:
            calls.append((args, kwargs))
            return "id"

    ctx = cast(ToolContext, SimpleNamespace(lookups=Spy()))
    assert record_lookup(ctx, "t", "front", JPEG, 20, 10) == "id"
    assert record_lookup(ctx, "t", "gripper", JPEG, 4, 2, full_frame=False) == "id"
    assert calls == [
        (("t", "front", JPEG, 20, 10), {"full_frame": True}),
        (("t", "gripper", JPEG, 4, 2), {"full_frame": False}),
    ]
    assert record_lookup(cast(ToolContext, SimpleNamespace(lookups=None)), "t", "front", JPEG, 20, 10) is None


def test_store_records_still_returns_lookup_records(robot: FakeRobot) -> None:
    """E5 contract: frontier_tools reads ctx.lookups.store.records() as LookupRecord with base_pose_map."""
    rec = recorder(robot)
    rec.record("look_toward_frontier", "front", JPEG, 20, 10)
    [got] = rec.store.records()
    assert isinstance(got, LookupRecord) and got.tool == "look_toward_frontier"
    assert got.base_pose_map is not None and got.base_pose_map.x == 1.0


# --- list / annotate / delete_photos ------------------------------------------------------------------------------


def test_list_lookups_returns_photo_poi_fields(server: Any) -> None:
    payload(call(server, "get_camera_image", {"camera": "front"}))
    row = call(server, "list_lookups", {}).structured_content["lookups"][0]
    assert {"kind", "name", "status", "created_by", "updated_at", "view"} <= set(row)
    assert row["kind"] == "photo" and row["view"]["source"] == "base_heading"


def test_list_and_get_lookup_image_derive_the_view_of_legacy_lines(server: Any, store_dir: Path) -> None:
    """A pre-E11 index line has no stored view; both read tools return the view derived on read."""
    lid = "0123456789abcdef0123456789abcdef"
    store_dir.mkdir(parents=True, exist_ok=True)
    (store_dir / f"{lid}.jpg").write_bytes(JPEG)
    legacy = {
        "id": lid,
        "ts": 1760000000.5,
        "tool": "get_camera_image",
        "camera": "front",
        "width": 640,
        "height": 480,
        "file": f"{lid}.jpg",
        "bytes": len(JPEG),
        "base_pose_map": {"x": 1.0, "y": 2.0, "yaw": 0.5},
        "arm_joints": None,
        "camera_pose_base": None,
        "mcp_session": "sess-1",
        "note": "",
        "poi_ids": [],
    }
    with (store_dir / INDEX_FILE).open("a") as fh:
        fh.write(json.dumps(legacy) + "\n")
    [row] = [r for r in call(server, "list_lookups", {}).structured_content["lookups"] if r["id"] == lid]
    assert row["view"] is not None and row["view"]["source"] == "base_heading"
    assert (row["view"]["x"], row["view"]["y"], row["view"]["yaw"]) == (1.0, 2.0, 0.5)
    meta = payload(call(server, "get_lookup_image", {"lookup_id": lid}))
    assert meta["view"] == row["view"]


def test_annotate_lookup_refuses_a_non_photo_id(server: Any, robot: FakeRobot) -> None:
    robot.pois = [{"id": "dock", "kind": "point"}]
    with pytest.raises(ToolError, match="unknown lookup"):
        call(server, "annotate_lookup", {"lookup_id": "dock", "note": "x"})
    assert robot.photo_commands == [] and robot.poi_commands == []


def test_annotate_lookup_passes_a_poi_store_rejection_on(server: Any, robot: FakeRobot) -> None:
    lookup_id = payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"]
    robot.photo_error = RobotError("poi_store rejected the update: photo capture data is immutable")
    with pytest.raises(ToolError, match="immutable"):
        call(server, "annotate_lookup", {"lookup_id": lookup_id, "note": "x"})


def take(server: Any, n: int) -> list[str]:
    return [payload(call(server, "get_camera_image", {"camera": "front"}))["lookup_id"] for _ in range(n)]


def test_delete_photos_by_ids(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    a, b = take(server, 2)
    size_a = (store_dir / f"{a}.jpg").stat().st_size
    res = call(server, "delete_photos", {"ids": [a, "f" * 32]})
    assert robot.photo_commands[-1] == ("delete_photos", {"ids": [a, "f" * 32], "dry_run": False})
    assert payload(res) == {
        "removed": 1,
        "bytes_freed": size_a,
        "missing": ["f" * 32],
    }
    assert [r["id"] for r in index_rows(store_dir)] == [b]
    assert [p.name for p in jpegs(store_dir)] == [f"{b}.jpg"]


def test_delete_photos_maps_days_to_seconds_and_keep_newest(server: Any, robot: FakeRobot) -> None:
    take(server, 3)
    dry = call(server, "delete_photos", {"older_than_days": 2.5, "dry_run": True})
    assert robot.photo_commands[-1] == ("delete_photos", {"older_than_s": 2.5 * DAY_S, "dry_run": True})
    assert payload(dry) == {"would_remove": 0, "bytes": 0}
    res = call(server, "delete_photos", {"keep_newest": 1})
    assert robot.photo_commands[-1] == ("delete_photos", {"keep_newest": 1, "dry_run": False})
    assert res.structured_content["removed"] == 2
    PhotoSelector.model_validate(robot.photo_commands[-1][1])


@pytest.mark.parametrize(
    "args",
    [{}, {"ids": ["a" * 32], "keep_newest": 1}, {"older_than_days": 1.0, "keep_newest": 0}, {"dry_run": True}],
)
def test_delete_photos_needs_exactly_one_selector(server: Any, robot: FakeRobot, args: dict[str, Any]) -> None:
    with pytest.raises(ToolError, match="exactly one of ids, older_than_days, keep_newest"):
        call(server, "delete_photos", args)
    assert robot.photo_commands == []


def test_delete_photos_rejects_bad_values(server: Any, robot: FakeRobot) -> None:
    for args in ({"ids": []}, {"older_than_days": 0}, {"keep_newest": -1}):
        with pytest.raises(ToolError):
            call(server, "delete_photos", args)
    assert robot.photo_commands == []


def test_delete_photos_errors_when_poi_store_is_down(server: Any, robot: FakeRobot) -> None:
    robot.poi_store_up = False
    with pytest.raises(ToolError, match="poi_store is not running"):
        call(server, "delete_photos", {"keep_newest": 0})


def test_delete_poi_and_update_poi_accept_photo_ids(server: Any, robot: FakeRobot, store_dir: Path) -> None:
    a, b = take(server, 2)
    call(server, "update_poi", {"id": a, "name": "kitchen view", "status": "done"})
    row = next(r for r in index_rows(store_dir) if r["id"] == a)
    assert (row["name"], row["status"]) == ("kitchen view", "done")
    call(server, "delete_poi", {"id": b})
    assert [r["id"] for r in index_rows(store_dir)] == [a]


def test_descriptions_explain_photo_pois(server: Any) -> None:
    tools = {t.name: t.description or "" for t in anyio.run(server.list_tools)}
    for name in LOOKUP_TOOLS:
        assert "POI of kind 'photo'" in tools[name], name
    for name in ("delete_photos", "list_lookups"):
        assert "delete_poi" in tools[name] and "update_poi" in tools[name], name
    assert "photo" in tools["update_poi"] and "photo" in tools["delete_poi"]
    assert "list_lookups" in tools["list_pois"] and "photo" in tools["list_pois"]


def test_delete_photos_is_a_sensor_class_tool() -> None:
    assert "delete_photos" in lookup_tools.TOOL_NAMES
    assert "delete_photos" in ALWAYS_ALLOWED_TOOLS and "delete_photos" not in MOTION_TOOLS
