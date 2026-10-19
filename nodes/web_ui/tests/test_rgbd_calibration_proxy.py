"""RGB-D calibration proxy: web_ui forwards /api/calibration/rgbd/* to the calibration node."""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from web_ui.config import AppConfig, CalibrationConfig
from web_ui.rgbd_calibration import register_rgbd_calibration_routes, valid_path

RGBD_URL = "http://calib.test:18400"
PREFIX = "/api/calibration/rgbd/"


def make_client(handler: httpx.MockTransport | None, rgbd_url: str | None = RGBD_URL) -> TestClient:
    app = FastAPI()
    config = AppConfig(calibration=CalibrationConfig(rgbd_url=rgbd_url))
    register_rgbd_calibration_routes(app, config, transport=handler)
    return TestClient(app)


def recording(response: httpx.Response, seen: list[httpx.Request]) -> httpx.MockTransport:
    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request)
        return response

    return httpx.MockTransport(handler)


def test_config_flag_on_and_off(tmp_path: Path) -> None:
    from web_ui.server import build_app

    def flag(rgbd_url: str | None) -> dict[str, object]:
        config = AppConfig(calibration=CalibrationConfig(rgbd_url=rgbd_url))
        app = build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=None)
        return TestClient(app).get("/api/calibration/config").json()

    assert flag(RGBD_URL) == {"imu": True, "rgbd": True, "arm": "deferred"}
    assert flag(None) == {"imu": True, "rgbd": False, "arm": "deferred"}


def test_rgbd_url_must_be_http() -> None:
    with pytest.raises(ValueError):
        CalibrationConfig(rgbd_url="ftp://x")
    with pytest.raises(ValueError):
        CalibrationConfig(rgbd_timeout_s=0)


def test_get_forwards_path_and_query() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={"poses": []}), seen))
    response = client.get(PREFIX + "sessions/abc/poses?limit=5&x=y")
    assert response.status_code == 200
    assert response.json() == {"poses": []}
    assert seen[0].method == "GET"
    assert str(seen[0].url) == f"{RGBD_URL}/api/sessions/abc/poses?limit=5&x=y"


def test_post_forwards_body_and_content_type() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(202, json={"ok": True}), seen))
    response = client.post(PREFIX + "capture/start", content=b'{"a": 1}', headers={"content-type": "application/json"})
    assert response.status_code == 202
    assert seen[0].method == "POST"
    assert seen[0].content == b'{"a": 1}'
    assert seen[0].headers["content-type"] == "application/json"
    assert seen[0].url.path == "/api/capture/start"


def test_delete_forwarded() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={"ok": True}), seen))
    assert client.delete(PREFIX + "sessions/abc/poses/3").status_code == 200
    assert seen[0].method == "DELETE"
    assert seen[0].url.path == "/api/sessions/abc/poses/3"


def test_binary_body_and_headers_pass_through() -> None:
    payload = bytes(range(256)) * 40
    upstream = httpx.Response(
        200,
        content=payload,
        headers={"content-type": "application/zip", "content-disposition": 'attachment; filename="r.zip"'},
    )
    client = make_client(recording(upstream, []))
    response = client.get(PREFIX + "sessions/abc/download")
    assert response.content == payload
    assert response.headers["content-type"] == "application/zip"
    assert response.headers["content-disposition"] == 'attachment; filename="r.zip"'


def test_upstream_error_status_passes_through() -> None:
    client = make_client(recording(httpx.Response(409, json={"ok": False, "message": "busy"}), []))
    response = client.post(PREFIX + "capture/start")
    assert response.status_code == 409
    assert response.json() == {"ok": False, "message": "busy"}


@pytest.mark.parametrize("path", ["a/%2e%2e/b", "a%20b", "a%00b", "a;b"])
def test_bad_path_rejected(path: str) -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={}), seen))
    assert client.get(PREFIX + path).status_code == 400
    assert seen == []


def test_unreachable_is_503() -> None:
    def boom(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused")

    response = make_client(httpx.MockTransport(boom)).get(PREFIX + "status")
    assert response.status_code == 503
    assert response.json() == {"ok": False, "message": "calibration node unreachable"}


def test_timeout_is_503() -> None:
    def slow(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("slow")

    assert make_client(httpx.MockTransport(slow)).get(PREFIX + "status").status_code == 503


def test_unconfigured_is_404() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={}), seen), rgbd_url=None)
    response = client.get(PREFIX + "status")
    assert response.status_code == 404
    assert response.json() == {"ok": False, "message": "RGB-D calibration not configured"}
    assert seen == []


@pytest.mark.parametrize("path", ["../etc/passwd", "a/../b", "..", "a/.."])
def test_dotdot_segment_invalid(path: str) -> None:
    assert not valid_path(path)
    assert valid_path("sessions/s.1/poses/3/rgb-thumb.jpg")


def test_forwards_method_path_query_body() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={"ok": True}), seen))
    client.get(PREFIX + "sessions/abc/poses?limit=5")
    client.post(PREFIX + "capture/start?x=1", content=b'{"a": 1}', headers={"content-type": "application/json"})
    client.delete(PREFIX + "sessions/abc/poses/3")
    assert [r.method for r in seen] == ["GET", "POST", "DELETE"]
    assert str(seen[0].url) == f"{RGBD_URL}/api/sessions/abc/poses?limit=5"
    assert str(seen[1].url) == f"{RGBD_URL}/api/capture/start?x=1"
    assert seen[1].content == b'{"a": 1}'
    assert seen[2].url.path == "/api/sessions/abc/poses/3"


def test_binary_bodies_and_headers_pass_through() -> None:
    test_binary_body_and_headers_pass_through()
    jpeg = b"\xff\xd8\xff\xe0" + bytes(range(256))
    client = make_client(recording(httpx.Response(200, content=jpeg, headers={"content-type": "image/jpeg"}), []))
    response = client.get(PREFIX + "sessions/abc/poses/1/rgb.jpg")
    assert response.content == jpeg
    assert response.headers["content-type"] == "image/jpeg"


def test_rejects_traversal_path() -> None:
    seen: list[httpx.Request] = []
    client = make_client(recording(httpx.Response(200, json={}), seen))
    assert client.get(PREFIX + "a/%2e%2e/b").status_code == 400
    assert seen == []


def test_unreachable_node_is_503() -> None:
    test_unreachable_is_503()


def test_not_configured_is_404() -> None:
    test_unconfigured_is_404()
