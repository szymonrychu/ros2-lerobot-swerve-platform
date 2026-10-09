"""Tests for the tile proxy API key: {api_key} template placeholder, env var lookup, cache fingerprint, no leaks."""

from __future__ import annotations

import logging
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app, make_tile_proxy
from web_ui.tiles import build_tile_url, tile_cache_fingerprint

PNG_BYTES = b"\x89PNG\r\n\x1a\nfake-tile"
KEY_ENV = "TEST_TILE_API_KEY"
SECRET = "s3cr3t-key-value"
TEMPLATE = "https://{s}.tiles.test/{z}/{x}/{y}.png?key={api_key}"


def make_client(
    tmp_path: Path, urdf_dir: Path, requests: list[httpx.Request], template: str = TEMPLATE, env: str | None = KEY_ENV
) -> TestClient:
    def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        return httpx.Response(200, content=PNG_BYTES, headers={"content-type": "image/png"}, request=request)

    tab = TabConfig(
        id="map",
        type="map_nav",
        label="Map",
        tile_url=template,
        tile_subdomains="ab",
        tile_cache_dir=str(tmp_path / "cache"),
        tile_api_key_env=env,
    )
    app = build_app(
        config=AppConfig(tabs=[tab]),
        urdf_dir=urdf_dir,
        static_dir=tmp_path / "none",
        tile_transport=httpx.MockTransport(handler),
    )
    return TestClient(app)


def test_tile_api_key_env_defaults_to_none() -> None:
    assert TabConfig(id="m", type="map_nav", label="Map").tile_api_key_env is None


def test_build_tile_url_substitutes_api_key() -> None:
    url = build_tile_url(TEMPLATE, "ab", 5, 1, 2, api_key="abc")
    assert url == "https://b.tiles.test/5/1/2.png?key=abc"


def test_build_tile_url_without_api_key_placeholder_unchanged() -> None:
    assert build_tile_url("https://t.test/{z}/{x}/{y}.png", "", 1, 0, 1, api_key="abc") == "https://t.test/1/0/1.png"


def test_fingerprint_is_short_hex_and_hides_key() -> None:
    fp = tile_cache_fingerprint(TEMPLATE, SECRET)
    assert len(fp) == 8
    int(fp, 16)
    assert SECRET not in fp


def test_fingerprint_changes_with_key_and_template_and_presence() -> None:
    base = tile_cache_fingerprint(TEMPLATE, "k1")
    assert tile_cache_fingerprint(TEMPLATE, "k2") != base
    assert tile_cache_fingerprint(TEMPLATE + "&x=1", "k1") != base
    assert tile_cache_fingerprint(TEMPLATE, None) != base
    assert tile_cache_fingerprint(TEMPLATE, "k1") == base


def test_fetch_uses_key_from_env_and_caches_under_fingerprint(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)
    requests: list[httpx.Request] = []
    resp = make_client(tmp_path, urdf_dir, requests).get("/api/tiles/3/5/2.png")
    assert resp.status_code == 200
    assert str(requests[0].url) == f"https://b.tiles.test/3/5/2.png?key={SECRET}"
    cached = tmp_path / "cache" / tile_cache_fingerprint(TEMPLATE, SECRET) / "3" / "5" / "2.png"
    assert cached.read_bytes() == PNG_BYTES
    assert not (tmp_path / "cache" / "3").exists()
    assert SECRET not in str(cached)


def test_changed_key_does_not_serve_old_cache(tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(KEY_ENV, "old-key")
    requests: list[httpx.Request] = []
    make_client(tmp_path, urdf_dir, requests).get("/api/tiles/3/5/2.png")
    monkeypatch.setenv(KEY_ENV, "new-key")
    make_client(tmp_path, urdf_dir, requests).get("/api/tiles/3/5/2.png")
    assert [r.url.params["key"] for r in requests] == ["old-key", "new-key"]


@pytest.mark.parametrize("value", [None, ""])
def test_missing_key_returns_503_and_never_fetches(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch, value: str | None
) -> None:
    monkeypatch.delenv(KEY_ENV, raising=False)
    if value is not None:
        monkeypatch.setenv(KEY_ENV, value)
    requests: list[httpx.Request] = []
    resp = make_client(tmp_path, urdf_dir, requests).get("/api/tiles/3/5/2.png")
    assert resp.status_code == 503
    assert requests == []


def test_missing_env_name_with_key_placeholder_returns_503(tmp_path: Path, urdf_dir: Path) -> None:
    requests: list[httpx.Request] = []
    assert make_client(tmp_path, urdf_dir, requests, env=None).get("/api/tiles/3/5/2.png").status_code == 503
    assert requests == []


def test_missing_key_logs_warning_once_at_startup_without_key(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.delenv(KEY_ENV, raising=False)
    tab = TabConfig(id="map", type="map_nav", label="Map", tile_url=TEMPLATE, tile_api_key_env=KEY_ENV)
    assert make_tile_proxy(AppConfig(tabs=[tab])) is None
    out = capsys.readouterr()
    assert (out.out + out.err).count("tile_api_key_missing") == 1
    assert KEY_ENV in out.out + out.err


def test_keyless_template_still_works_without_env(tmp_path: Path, urdf_dir: Path) -> None:
    requests: list[httpx.Request] = []
    resp = make_client(tmp_path, urdf_dir, requests, template="https://{s}.tiles.test/{z}/{x}/{y}.png", env=None).get(
        "/api/tiles/3/5/2.png"
    )
    assert resp.status_code == 200
    assert str(requests[0].url) == "https://b.tiles.test/3/5/2.png"


def test_api_config_does_not_expose_key(tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)
    body = make_client(tmp_path, urdf_dir, []).get("/api/config").text
    assert SECRET not in body


def test_key_never_logged_on_upstream_failure(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(403, request=request)

    tab = TabConfig(
        id="map", type="map_nav", label="Map", tile_url=TEMPLATE, tile_cache_dir=str(tmp_path / "c"),
        tile_subdomains="a", tile_api_key_env=KEY_ENV,
    )  # fmt: skip
    logging.getLogger("httpx").setLevel(logging.NOTSET)
    app = build_app(AppConfig(tabs=[tab]), urdf_dir, tmp_path / "none", tile_transport=httpx.MockTransport(handler))
    assert TestClient(app).get("/api/tiles/3/5/2.png").status_code == 404
    out = capsys.readouterr()
    assert SECRET not in out.out + out.err
    assert logging.getLogger("httpx").level >= logging.WARNING


def test_api_config_exposes_tile_version_as_cache_fingerprint(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The browser appends tile_version to tile URLs, so a new key or source never reuses cached tiles."""
    monkeypatch.setenv(KEY_ENV, SECRET)
    version = make_client(tmp_path, urdf_dir, []).get("/api/config").json()["tabs"][0]["tile_version"]
    assert version == tile_cache_fingerprint(TEMPLATE, SECRET)
    monkeypatch.setenv(KEY_ENV, SECRET + "2")
    assert make_client(tmp_path, urdf_dir, []).get("/api/config").json()["tabs"][0]["tile_version"] != version


def test_api_config_tile_version_is_none_without_proxy(tmp_path: Path, urdf_dir: Path) -> None:
    body = make_client(tmp_path, urdf_dir, [], env=None).get("/api/config").json()
    assert body["tabs"][0]["tile_version"] is None
