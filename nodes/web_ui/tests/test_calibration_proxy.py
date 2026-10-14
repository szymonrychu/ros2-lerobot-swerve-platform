"""Tests for the stereo calibration proxy to Jetson depthd."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

from web_ui.config import AppConfig
from web_ui.server import build_app

URL = "http://10.77.0.2:8090"
JPEG = b"\xff\xd8\xff\xe0fakejpeg"

ROUTES = [
    ("GET", "/stereo/preview", "/calibration/preview"),
    ("GET", "/stereo/preview/left.jpg", "/calibration/preview/left.jpg"),
    ("GET", "/stereo/preview/right.jpg", "/calibration/preview/right.jpg"),
    ("GET", "/stereo/captures", "/calibration/captures"),
    ("POST", "/stereo/captures", "/calibration/captures"),
    ("DELETE", "/stereo/captures?confirm=true", "/calibration/captures"),
    ("GET", "/stereo/captures/cap-001/left.jpg", "/calibration/captures/cap-001/left.jpg"),
    ("GET", "/stereo/captures/cap-001/right.jpg", "/calibration/captures/cap-001/right.jpg"),
    ("GET", "/stereo/solve", "/calibration/solve"),
    ("POST", "/stereo/solve", "/calibration/solve"),
]


class Upstream:
    def __init__(self) -> None:
        self.requests: list[httpx.Request] = []
        self.response: Callable[[httpx.Request], httpx.Response] = lambda r: httpx.Response(200, json={"seen": True})

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.requests.append(request)
        return self.response(request)


@pytest.fixture
def upstream() -> Upstream:
    return Upstream()


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path, upstream: Upstream) -> Callable[..., TestClient]:
    def factory(configured: bool = True) -> TestClient:
        config = AppConfig.model_validate({"calibration": {"depthd_url": URL + "/"}} if configured else {})
        app = build_app(
            config=config,
            urdf_dir=urdf_dir,
            static_dir=tmp_path / "none",
            calibration_transport=httpx.MockTransport(upstream),
        )
        return TestClient(app)

    return factory


@pytest.mark.parametrize(("method", "path", "upstream_path"), ROUTES)
def test_route_forwards_method_path_and_passes_response(
    make_client: Callable[..., TestClient], upstream: Upstream, method: str, path: str, upstream_path: str
) -> None:
    upstream.response = lambda r: httpx.Response(202, json={"state": "running"}, headers={"x-extra": "1"})
    resp = make_client().request(method, "/api/calibration" + path)
    assert resp.status_code == 202
    assert resp.json() == {"state": "running"}
    (req,) = upstream.requests
    assert req.method == method
    assert req.url.path == upstream_path
    assert str(req.url).startswith(URL + "/calibration")


def test_query_and_body_forwarded(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    make_client().post(
        "/api/calibration/stereo/solve?a=1", content=b'{"x": 2}', headers={"content-type": "application/json"}
    )
    (req,) = upstream.requests
    assert req.url.query == b"a=1"
    assert req.content == b'{"x": 2}'
    assert req.headers["content-type"] == "application/json"


def test_delete_forwards_confirm(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    make_client().delete("/api/calibration/stereo/captures?confirm=true")
    assert upstream.requests[0].url.query == b"confirm=true"


@pytest.mark.parametrize("query", ["", "?confirm=false", "?confirm=1", "?confirm=TRUE"])
def test_delete_without_confirm_is_400_without_upstream(
    make_client: Callable[..., TestClient], upstream: Upstream, query: str
) -> None:
    resp = make_client().delete("/api/calibration/stereo/captures" + query)
    assert resp.status_code == 400
    assert resp.json()["ok"] is False
    assert "confirm" in resp.json()["message"]
    assert upstream.requests == []


def test_image_bytes_and_content_type_pass_through(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    upstream.response = lambda r: httpx.Response(200, content=JPEG, headers={"content-type": "image/jpeg"})
    resp = make_client().get("/api/calibration/stereo/preview/left.jpg")
    assert resp.content == JPEG
    assert resp.headers["content-type"] == "image/jpeg"
    assert resp.headers["cache-control"] == "no-store"


def test_upstream_error_status_passes_through(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    upstream.response = lambda r: httpx.Response(503, json={"reason": "no fresh pair"})
    resp = make_client().get("/api/calibration/stereo/preview")
    assert resp.status_code == 503
    assert resp.json() == {"reason": "no fresh pair"}


def test_connect_error_is_502(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    def boom(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused")

    upstream.response = boom
    resp = make_client().get("/api/calibration/stereo/preview")
    assert resp.status_code == 502
    body = resp.json()
    assert body["ok"] is False
    assert body["message"].startswith(f"Jetson depthd unreachable at {URL}: ")
    assert "refused" in body["message"]


def test_timeout_is_504(make_client: Callable[..., TestClient], upstream: Upstream) -> None:
    def slow(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("too slow")

    upstream.response = slow
    resp = make_client().post("/api/calibration/stereo/solve")
    assert resp.status_code == 504
    assert resp.json()["message"].startswith(f"Jetson depthd unreachable at {URL}: ")


@pytest.mark.parametrize(
    "path",
    [
        "/stereo/preview/up.jpg",
        "/stereo/captures/cap-001/middle.jpg",
        "/stereo/captures/bad%20id/left.jpg",
        "/stereo/captures/..%2Fetc/left.jpg",
        "/stereo/captures/a.b/left.jpg",
    ],
)
def test_invalid_side_or_id_is_422_without_upstream(
    make_client: Callable[..., TestClient], upstream: Upstream, path: str
) -> None:
    resp = make_client().get("/api/calibration" + path)
    assert resp.status_code == 422
    assert resp.json()["ok"] is False
    assert upstream.requests == []


def test_config_route(make_client: Callable[..., TestClient]) -> None:
    resp = make_client().get("/api/calibration/config")
    assert resp.status_code == 200
    assert resp.json() == {"stereo": True, "imu": True, "arm": "deferred"}


@pytest.mark.parametrize(
    ("method", "path"), [(m, "/api/calibration" + p) for m, p, _ in ROUTES] + [("GET", "/api/calibration/config")]
)
def test_unconfigured_is_404(
    make_client: Callable[..., TestClient], upstream: Upstream, method: str, path: str
) -> None:
    resp = make_client(configured=False).request(method, path)
    assert resp.status_code == 404
    assert resp.json() == {"ok": False, "message": "calibration page not configured"}
    assert upstream.requests == []
