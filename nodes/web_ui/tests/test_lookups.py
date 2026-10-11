"""Tests for the read-only camera lookup routes: GET /api/lookups and GET /api/lookups/{id}.jpg."""

from __future__ import annotations

import inspect
import json
import logging
from pathlib import Path

import cv2
import numpy as np
import pytest
import ros2_common.lookups as shared_lookups
from fastapi.testclient import TestClient

from web_ui import lookups
from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

JPEG = b"\xff\xd8\xff\xe0fake-jpeg"


def record(lookup_id: str, ts: float, **extra: object) -> dict[str, object]:
    """One index.jsonl record in the shape mcp_server lookup_store.py writes."""
    base: dict[str, object] = {
        "id": lookup_id,
        "ts": ts,
        "tool": "get_camera_image",
        "camera": "head",
        "width": 640,
        "height": 480,
        "file": f"{lookup_id}.jpg",
        "bytes": len(JPEG),
        "base_pose_map": {"x": 1.0, "y": 2.0, "yaw": 0.0},
        "arm_joints": None,
        "camera_pose_base": None,
        "mcp_session": "s1",
        "note": "",
        "poi_ids": [],
    }
    return base | extra


def write_store(root: Path, records: list[dict[str, object]], extra_lines: list[str] | None = None) -> None:
    root.mkdir(parents=True, exist_ok=True)
    lines = [json.dumps(r) for r in records] + (extra_lines or [])
    (root / "index.jsonl").write_text("\n".join(lines) + "\n")
    for r in records:
        (root / str(r["file"])).write_bytes(JPEG)


def make_client(tmp_path: Path, urdf_dir: Path, lookups_dir: Path | None = None) -> TestClient:
    kwargs: dict[str, object] = {} if lookups_dir is None else {"lookups_dir": str(lookups_dir)}
    tab = TabConfig(id="map", type="map_nav", label="Map", **kwargs)  # type: ignore[arg-type]
    return TestClient(build_app(AppConfig(tabs=[tab]), urdf_dir, tmp_path / "none"))


@pytest.fixture
def store(tmp_path: Path) -> Path:
    root = tmp_path / "lookups"
    write_store(
        root,
        [
            record("a", 100.0, note="red mug on table", poi_ids=["p1"]),
            record("b", 200.0, camera="gripper", mcp_session="s2", base_pose_map={"x": 10.0, "y": 10.0, "yaw": 0}),
            record("c", 300.0, poi_ids=["p1", "p2"], note="Shelf"),
        ],
    )
    return root


@pytest.fixture
def client(tmp_path: Path, urdf_dir: Path, store: Path) -> TestClient:
    return make_client(tmp_path, urdf_dir, store)


def ids(client: TestClient, query: str = "") -> list[str]:
    resp = client.get(f"/api/lookups{query}")
    assert resp.status_code == 200
    return [r["id"] for r in resp.json()["lookups"]]


def test_default_lookups_dir_is_the_mcp_server_store() -> None:
    assert TabConfig(id="m", type="map_nav", label="M").lookups_dir == "/var/lib/ros2/poi/lookups"


def test_lists_newest_first_with_metadata(client: TestClient) -> None:
    body = client.get("/api/lookups").json()
    assert [r["id"] for r in body["lookups"]] == ["c", "b", "a"]
    assert body["lookups"][0]["poi_ids"] == ["p1", "p2"]
    assert body["lookups"][0]["camera"] == "head"
    assert body["lookups"][0]["time"] == "1970-01-01T00:05:00Z"


def test_limit_caps_the_list_and_is_validated(client: TestClient) -> None:
    assert ids(client, "?limit=2") == ["c", "b"]
    assert client.get("/api/lookups?limit=0").status_code == 422
    assert client.get("/api/lookups?limit=21").status_code == 422


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        ("?since=150", ["c", "b"]),
        ("?until=250", ["b", "a"]),
        ("?since=150&until=250", ["b"]),
        ("?poi_id=p1", ["c", "a"]),
        ("?poi_id=p2", ["c"]),
        ("?text=MUG", ["a"]),
        ("?camera=gripper", ["b"]),
        ("?mcp_session=s2", ["b"]),
        ("?near_x=1&near_y=2&radius_m=1", ["c", "a"]),
        ("?near_x=10&near_y=10&radius_m=0.5", ["b"]),
        ("?camera=head&poi_id=p1&since=150", ["c"]),
        ("?poi_id=nope", []),
    ],
)
def test_filters(client: TestClient, query: str, expected: list[str]) -> None:
    assert ids(client, query) == expected


def test_near_needs_all_three_parts(client: TestClient) -> None:
    assert client.get("/api/lookups?near_x=1&near_y=2").status_code == 422
    assert client.get("/api/lookups?near_x=1&near_y=2&radius_m=0").status_code == 422


def test_corrupt_and_blank_lines_are_skipped(tmp_path: Path, urdf_dir: Path) -> None:
    root = tmp_path / "lk"
    write_store(root, [record("a", 1.0), record("b", 2.0)], extra_lines=["{not json", "", '{"id": "x"}'])
    assert ids(make_client(tmp_path, urdf_dir, root)) == ["b", "a"]


def test_missing_store_lists_nothing(tmp_path: Path, urdf_dir: Path) -> None:
    assert ids(make_client(tmp_path, urdf_dir, tmp_path / "absent")) == []


def test_jpg_is_served_read_only(client: TestClient) -> None:
    resp = client.get("/api/lookups/a.jpg")
    assert resp.status_code == 200
    assert resp.headers["content-type"] == "image/jpeg"
    assert resp.content == JPEG
    assert client.post("/api/lookups/a.jpg").status_code in (404, 405)
    assert client.delete("/api/lookups/a.jpg").status_code in (404, 405)


def test_unknown_id_is_404(client: TestClient) -> None:
    assert client.get("/api/lookups/zzz.jpg").status_code == 404


def test_indexed_record_whose_file_is_missing_is_404(client: TestClient, store: Path) -> None:
    (store / "a.jpg").unlink()
    assert client.get("/api/lookups/a.jpg").status_code == 404


def test_traversal_by_id_is_rejected(client: TestClient, tmp_path: Path) -> None:
    (tmp_path / "secret.jpg").write_bytes(b"secret")
    for bad in ("..%2Fsecret", "..%2F..%2Fsecret", "%2e%2e%2fsecret", "..", "a%2F..%2F..%2Fsecret"):
        resp = client.get(f"/api/lookups/{bad}.jpg")
        assert resp.status_code in (400, 404)
        assert b"secret" not in resp.content


def test_traversal_by_indexed_file_name_is_rejected(tmp_path: Path, urdf_dir: Path) -> None:
    root = tmp_path / "lk"
    (tmp_path / "secret.jpg").write_bytes(b"secret")
    write_store(root, [record("evil", 1.0, file="../secret.jpg")])
    (root / "../secret.jpg").resolve().write_bytes(b"secret")
    client = make_client(tmp_path, urdf_dir, root)
    resp = client.get("/api/lookups/evil.jpg")
    assert resp.status_code == 404
    assert b"secret" not in resp.content


def real_jpeg(width: int, height: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((height, width, 3), 128, np.uint8))
    assert ok
    return buf.tobytes()


def test_record_contract_is_the_shared_one() -> None:
    assert lookups.LookupRecord is shared_lookups.LookupRecord
    assert lookups.LookupQuery is shared_lookups.LookupQuery
    rec = lookups.LookupRecord.model_validate(record("a", 1.0, camera_pose_base={"xyz": [0, 0, 1], "rpy": [0, 0, 0]}))
    assert rec.base_pose_map is not None and rec.base_pose_map.x == 1.0
    assert lookups.record_summary(rec)["time"] == "1970-01-01T00:00:01Z"


def test_read_records_cached_until_index_changes(store: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    parsed: list[str] = []
    real = shared_lookups.parse_line
    monkeypatch.setattr(shared_lookups, "parse_line", lambda line: parsed.append(line) or real(line))
    assert [r.id for r in lookups.read_records(store)] == ["a", "b", "c"]
    lookups.read_records(store)
    assert len(parsed) == 3
    write_store(store, [record("a", 100.0), record("d", 400.0)])
    assert [r.id for r in lookups.read_records(store)] == ["a", "d"]
    assert len(parsed) == 5


def test_corrupt_line_warns_once_across_requests(
    tmp_path: Path, urdf_dir: Path, caplog: pytest.LogCaptureFixture
) -> None:
    root = tmp_path / "lk"
    write_store(root, [record("a", 1.0)], extra_lines=["{not json"])
    client = make_client(tmp_path, urdf_dir, root)
    with caplog.at_level(logging.WARNING):
        for _ in range(3):
            assert ids(client) == ["a"]
    assert len([m for m in caplog.messages if "corrupt" in m]) == 1


def test_routes_do_not_block_the_event_loop(client: TestClient) -> None:
    routes = {r.path: r.endpoint for r in client.app.routes if r.path.startswith("/api/lookups")}  # type: ignore[attr-defined]
    assert len(routes) == 2
    assert not any(inspect.iscoroutinefunction(endpoint) for endpoint in routes.values())


def test_since_and_until_accept_iso_strings(client: TestClient) -> None:
    assert ids(client, "?since=1970-01-01T00:02:30Z") == ["c", "b"]
    assert ids(client, "?until=1970-01-01T00:04:10%2B00:00") == ["b", "a"]
    assert client.get("/api/lookups?since=yesterday").status_code == 422


def test_thumbnail_is_downscaled_server_side(tmp_path: Path, urdf_dir: Path) -> None:
    root = tmp_path / "lk"
    write_store(root, [record("a", 1.0)])
    big = real_jpeg(640, 480)
    (root / "a.jpg").write_bytes(big)
    client = make_client(tmp_path, urdf_dir, root)
    resp = client.get("/api/lookups/a.jpg?max_px=96")
    assert resp.status_code == 200 and resp.headers["content-type"] == "image/jpeg"
    img = cv2.imdecode(np.frombuffer(resp.content, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (72, 96) and len(resp.content) < len(big)
    assert "max-age" in resp.headers["cache-control"]
    assert client.get("/api/lookups/a.jpg").content == big  # no max_px: the stored file
    assert client.get("/api/lookups/a.jpg?max_px=5000").content == big  # never upscaled
    assert client.get("/api/lookups/a.jpg?max_px=2").status_code == 422
    assert client.get("/api/lookups/zzz.jpg?max_px=96").status_code == 404


def test_thumbnail_of_an_undecodable_file_is_404(client: TestClient) -> None:
    assert client.get("/api/lookups/a.jpg?max_px=64").status_code == 404
