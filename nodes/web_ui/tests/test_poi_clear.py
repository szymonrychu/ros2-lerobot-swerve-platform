"""Tests for the clear op of POST /api/poi (bulk POI deletion with poi_store's filters)."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from tests.test_poi import FakeBridge, make_client

RESULT = {"request_id": "x", "ok": True, "message": "cleared", "poi": {"removed": 3}}


@pytest.mark.parametrize(
    "extra",
    [
        {},
        {"created_by": "agent"},
        {"created_by": "user"},
        {"created_by": None},
        {"kinds": ["point", "object"]},
        {"statuses": ["done", "cancelled"]},
        {"keep_zones": True},
        {"ids": ["a", "b"]},
        {"created_by": "agent", "kinds": ["area"], "statuses": ["open"], "keep_zones": False, "ids": None},
    ],
)
def test_clear_forwards_only_given_fields(tmp_path: Path, extra: dict[str, Any]) -> None:
    bridge = FakeBridge(RESULT)
    resp = make_client(tmp_path, bridge).post("/api/poi?tab=map", json={"op": "clear", **extra})
    assert resp.status_code == 200 and resp.json() == RESULT
    assert bridge.payloads == [{"op": "clear", **extra}]


@pytest.mark.parametrize(
    "extra",
    [
        {"created_by": "robot"},
        {"created_by": 1},
        {"kinds": "point"},
        {"kinds": ["photo"]},
        {"statuses": ["closed"]},
        {"statuses": "open"},
        {"keep_zones": "yes"},
        {"ids": "abc"},
        {"ids": [""]},
        {"ids": [1]},
        {"ids": ["a"] * 501},
        {"keep_zone": True},
        {"kind": ["point"]},
    ],
)
def test_clear_invalid_filters_422(tmp_path: Path, extra: dict[str, Any]) -> None:
    bridge = FakeBridge(RESULT)
    resp = make_client(tmp_path, bridge).post("/api/poi?tab=map", json={"op": "clear", **extra})
    assert resp.status_code == 422 and resp.json()["ok"] is False and resp.json()["message"]
    assert bridge.payloads == []


def test_clear_accepts_500_ids(tmp_path: Path) -> None:
    bridge = FakeBridge(RESULT)
    ids = [f"id{i}" for i in range(500)]
    assert make_client(tmp_path, bridge).post("/api/poi?tab=map", json={"op": "clear", "ids": ids}).status_code == 200


def test_clear_503_when_store_not_running(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(none=True)).post("/api/poi?tab=map", json={"op": "clear"})
    assert resp.status_code == 503


def test_clear_504_on_timeout(tmp_path: Path) -> None:
    with patch("web_ui.server.POI_TIMEOUT_S", 0.05):
        resp = make_client(tmp_path, FakeBridge(never=True)).post("/api/poi?tab=map", json={"op": "clear"})
    assert resp.status_code == 504


def test_clear_not_battery_guarded(tmp_path: Path) -> None:
    guard = MagicMock()
    guard.is_cutoff.return_value = True
    assert (
        make_client(tmp_path, FakeBridge(RESULT), guard).post("/api/poi?tab=map", json={"op": "clear"}).status_code
        == 200
    )


def test_unknown_op_message_lists_clear(tmp_path: Path) -> None:
    resp = make_client(tmp_path, FakeBridge(RESULT)).post("/api/poi?tab=map", json={"op": "explode"})
    assert resp.status_code == 422 and "clear" in resp.json()["message"]
