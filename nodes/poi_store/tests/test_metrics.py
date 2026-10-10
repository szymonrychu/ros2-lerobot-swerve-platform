"""Tests for the poi_store Prometheus metrics (default registry, deltas)."""

import json
from pathlib import Path

import pytest
from prometheus_client import REGISTRY

from poi_store.config import PoiStoreConfig
from poi_store.store import PoiStore


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def cmd(op: str, **poi) -> str:
    return json.dumps({"op": op, "request_id": "r", "poi": poi})


@pytest.fixture
def store(tmp_path: Path) -> PoiStore:
    return PoiStore(tmp_path / "poi.json", clock=lambda: 1.0)


def test_commands_counted_by_op_and_ok(store):
    ok_before = sample("poi_commands_total", {"op": "add", "ok": "true"})
    store.handle_message(cmd("add", kind="point", x=1, y=2, name="a", created_by="agent"))
    assert sample("poi_commands_total", {"op": "add", "ok": "true"}) == ok_before + 1
    fail_before = sample("poi_commands_total", {"op": "delete", "ok": "false"})
    store.handle_message(cmd("delete", id="nope"))
    assert sample("poi_commands_total", {"op": "delete", "ok": "false"}) == fail_before + 1


def test_bad_command_counted_as_invalid(store):
    before = sample("poi_commands_total", {"op": "invalid", "ok": "false"})
    store.handle_message("not json")
    assert sample("poi_commands_total", {"op": "invalid", "ok": "false"}) == before + 1


def test_poi_count_tracks_store(store):
    store.handle_message(cmd("add", kind="point", x=1, y=2, name="a", created_by="agent"))
    store.handle_message(cmd("add", kind="point", x=3, y=2, name="b", created_by="agent"))
    assert sample("poi_count") == 2
    store.handle_message(json.dumps({"op": "clear", "request_id": "r", "created_by": "agent"}))
    assert sample("poi_count") == 0


def test_poi_count_set_on_load(tmp_path):
    first = PoiStore(tmp_path / "p.json")
    first.handle_message(cmd("add", kind="point", x=1, y=2, name="a", created_by="agent"))
    PoiStore(tmp_path / "p.json")
    assert sample("poi_count") == 1


def test_save_failure_counted(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    store = PoiStore(blocker / "poi.json")
    before = sample("poi_save_failures_total")
    result = json.loads(store.handle_message(cmd("add", kind="point", x=1, y=2, name="a", created_by="agent")))
    assert not result["ok"]
    assert sample("poi_save_failures_total") == before + 1
    assert sample("poi_count") == 0


def test_metrics_port_config_default_none():
    assert PoiStoreConfig().metrics_port is None
    assert PoiStoreConfig(metrics_port=19109).metrics_port == 19109
