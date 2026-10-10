"""Tests for multiple tile sources: config resolution, per-source route/cache/key/max zoom, /api/config exposure."""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from web_ui.config import AppConfig, TabConfig, TileSourceConfig
from web_ui.server import build_app
from web_ui.tiles import tile_cache_fingerprint

PNG_BYTES = b"\x89PNG\r\n\x1a\nfake-tile"
KEY_ENV = "TEST_SOURCES_TILE_KEY"
SECRET = "s3cr3t"
SAT_URL = "https://sat.test/tile/{z}/{y}/{x}"
STREET_URL = "https://{s}.street.test/{z}/{x}/{y}.png?key={api_key}"


def sources() -> list[TileSourceConfig]:
    return [
        TileSourceConfig(id="satellite", label="Satellite", url=SAT_URL, max_zoom=20, attribution="Esri"),
        TileSourceConfig(
            id="street", label="Street", url=STREET_URL, subdomains="ab", max_zoom=18, api_key_env=KEY_ENV,
            attribution="CARTO",
        ),
    ]  # fmt: skip


def make_client(tmp_path: Path, urdf_dir: Path, requests: list[httpx.Request], **tab_kwargs: object) -> TestClient:
    def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        return httpx.Response(200, content=PNG_BYTES, headers={"content-type": "image/png"}, request=request)

    kwargs: dict[str, object] = {"tile_sources": sources(), "default_tile_source": "satellite"} | tab_kwargs
    tab = TabConfig(id="map", type="map_nav", label="Map", tile_cache_dir=str(tmp_path / "cache"), **kwargs)  # type: ignore[arg-type]
    app = build_app(AppConfig(tabs=[tab]), urdf_dir, tmp_path / "none", tile_transport=httpx.MockTransport(handler))
    return TestClient(app)


def test_legacy_single_tile_url_is_one_default_source() -> None:
    tab = TabConfig(id="m", type="map_nav", label="Map", tile_url="https://t/{z}/{x}/{y}.png", tile_max_zoom=17)
    [src] = tab.resolved_tile_sources()
    assert (src.id, src.url, src.max_zoom) == ("default", "https://t/{z}/{x}/{y}.png", 17)
    assert tab.default_source_id() == "default"


def test_no_tiles_configured_resolves_no_sources() -> None:
    assert TabConfig(id="m", type="camera", label="Cam").resolved_tile_sources() == []


def test_unknown_default_and_duplicate_ids_are_rejected() -> None:
    with pytest.raises(ValidationError):
        TabConfig(id="m", type="map_nav", label="M", tile_sources=sources(), default_tile_source="nope")
    with pytest.raises(ValidationError):
        TabConfig(id="m", type="map_nav", label="M", tile_sources=sources() + sources())


def test_source_route_uses_template_order_and_separate_cache(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)
    requests: list[httpx.Request] = []
    client = make_client(tmp_path, urdf_dir, requests)
    assert client.get("/api/tiles/satellite/19/5/2.png").status_code == 200
    assert str(requests[0].url) == "https://sat.test/tile/19/2/5"
    assert client.get("/api/tiles/street/10/5/2.png").status_code == 200
    assert str(requests[1].url) == f"https://b.street.test/10/5/2.png?key={SECRET}"
    sat_dir = tmp_path / "cache" / tile_cache_fingerprint(SAT_URL, None)
    street_dir = tmp_path / "cache" / tile_cache_fingerprint(STREET_URL, SECRET)
    assert (sat_dir / "19" / "5" / "2.png").exists() and (street_dir / "10" / "5" / "2.png").exists()
    assert sat_dir != street_dir


def test_zoom_above_source_max_is_404_without_upstream(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)
    requests: list[httpx.Request] = []
    client = make_client(tmp_path, urdf_dir, requests)
    assert client.get("/api/tiles/street/19/5/2.png").status_code == 404
    assert client.get("/api/tiles/satellite/21/5/2.png").status_code == 404
    assert requests == []
    assert client.get("/api/tiles/satellite/20/5/2.png").status_code == 200


def test_unknown_source_is_404_and_old_route_serves_default_source(tmp_path: Path, urdf_dir: Path) -> None:
    requests: list[httpx.Request] = []
    client = make_client(tmp_path, urdf_dir, requests)
    assert client.get("/api/tiles/nope/3/1/1.png").status_code == 404
    assert client.get("/api/tiles/3/5/2.png").status_code == 200
    assert str(requests[0].url) == "https://sat.test/tile/3/2/5"


def test_source_missing_key_is_503_only_for_that_source(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv(KEY_ENV, raising=False)
    requests: list[httpx.Request] = []
    client = make_client(tmp_path, urdf_dir, requests)
    assert client.get("/api/tiles/street/3/1/1.png").status_code == 503
    assert client.get("/api/tiles/satellite/3/1/1.png").status_code == 200


def test_api_config_exposes_public_source_list_without_urls_or_keys(
    tmp_path: Path, urdf_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(KEY_ENV, SECRET)
    tab = make_client(tmp_path, urdf_dir, []).get("/api/config").json()["tabs"][0]
    assert tab["default_tile_source"] == "satellite"
    assert tab["tile_sources"] == [
        {
            "id": "satellite", "label": "Satellite", "max_zoom": 20, "attribution": "Esri",
            "version": tile_cache_fingerprint(SAT_URL, None),
        },
        {
            "id": "street", "label": "Street", "max_zoom": 18, "attribution": "CARTO",
            "version": tile_cache_fingerprint(STREET_URL, SECRET),
        },
    ]  # fmt: skip
    text = str(tab)
    assert SECRET not in text and "sat.test" not in text and KEY_ENV not in text


def test_tile_display_zoom_defaults_to_19_is_range_checked_and_exposed(tmp_path: Path, urdf_dir: Path) -> None:
    assert TabConfig(id="m", type="map_nav", label="M").tile_display_zoom == 19
    with pytest.raises(ValidationError):
        TabConfig(id="m", type="map_nav", label="M", tile_display_zoom=23)
    tab = make_client(tmp_path, urdf_dir, [], tile_display_zoom=20).get("/api/config").json()["tabs"][0]
    assert tab["tile_display_zoom"] == 20
