"""Tests for FastAPI server routes."""

from __future__ import annotations

from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from web_ui.config import AppConfig


@pytest.fixture
def app(tmp_path: Path, urdf_dir: Path):
    """Build a FastAPI app with a minimal config and test URDF dir."""
    from web_ui.server import build_app

    config = AppConfig(http_port=8080, tabs=[], overlays=[])
    static_dir = tmp_path / "static"
    static_dir.mkdir()
    (static_dir / "index.html").write_text("<html><body>ok</body></html>")
    return build_app(config=config, urdf_dir=urdf_dir, static_dir=static_dir)


def test_config_endpoint(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/config")
    assert resp.status_code == 200
    data = resp.json()
    assert data["http_port"] == 8080


def test_urdf_status_endpoint(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/status")
    assert resp.status_code == 200
    data = resp.json()
    assert "files" in data
    assert any(f["name"] == "robot.urdf" for f in data["files"])


def test_urdf_file_serves(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/robot.urdf")
    assert resp.status_code == 200


def test_urdf_path_traversal_blocked(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/../../../etc/passwd")
    assert resp.status_code in (400, 404)


def test_security_headers_present(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/config")
    assert resp.headers.get("x-content-type-options") == "nosniff"
    assert resp.headers.get("x-frame-options") == "SAMEORIGIN"
    assert "content-security-policy" in resp.headers


def test_static_fallback(app) -> None:
    client = TestClient(app)
    resp = client.get("/")
    assert resp.status_code == 200
    assert b"ok" in resp.content
