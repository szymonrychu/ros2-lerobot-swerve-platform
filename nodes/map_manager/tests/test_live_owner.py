"""Tests for the persisted live-set owner (active_map) and the honest delete (bundle and live counts)."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
from ros2_common.maps import MapCommand

from map_manager.manager import ACTIVE_MAP_FILE
from tests.test_manager import BUNDLE_POIS, Rig, load, make_rig

LIVE = [{"id": "p1", "name": "a"}, {"id": "p2", "name": "b"}, {"id": "ph1", "name": "pic", "kind": "photo"}]


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    return make_rig(tmp_path)


def save(rig: Rig, name: str = "home"):
    rig.poi.pois = [dict(p) for p in LIVE]
    return rig.manager.handle(MapCommand(op="save", request_id="s", name=name))


def delete(rig: Rig, name: str = "home"):
    return rig.manager.handle(MapCommand(op="delete", request_id="d", name=name))


def test_save_then_delete_empties_live_set_and_reports_counts(rig: Rig) -> None:
    assert save(rig).ok
    assert rig.manager.status().active_map == "home"
    result = delete(rig)
    assert result.ok, result.message
    assert rig.poi.pois == []
    assert result.data["was_active"] is True
    assert (result.data["bundle_pois"], result.data["bundle_photos"]) == (2, 1)
    assert (result.data["live_pois_removed"], result.data["live_photos_removed"]) == (2, 1)
    assert "removed 2 live POIs and 1 live photos" in result.message


def test_delete_after_restart_empties_live_set(tmp_path: Path) -> None:
    first = make_rig(tmp_path)
    assert save(first).ok
    second = make_rig(tmp_path)
    assert second.manager.status().active_map == "home"
    second.poi.pois = [dict(p) for p in LIVE]
    result = delete(second)
    assert result.ok and second.poi.pois == []
    assert make_rig(tmp_path).manager.status().active_map is None
    assert json.loads((Path(second.config.maps_root) / ACTIVE_MAP_FILE).read_text()) == {"active_map": None}


def test_load_then_delete_still_works(rig: Rig) -> None:
    rig.save_map("home")
    rig.manager.set_active_map(None)
    assert load(rig).ok
    result = delete(rig)
    assert result.ok and rig.poi.pois == []
    assert result.data["live_pois_removed"] == 2


def test_delete_non_owner_leaves_live_set_and_reports_bundle_counts(rig: Rig) -> None:
    assert save(rig, "other").ok
    assert save(rig, "home").ok
    rig.log.calls.clear()
    result = delete(rig, "other")
    assert result.ok
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == LIVE
    assert (result.data["bundle_pois"], result.data["bundle_photos"]) == (2, 1)
    assert result.data["live_pois_removed"] is None
    assert result.message.endswith("live set (owned by home) unchanged")


def test_owner_whose_bundle_was_removed_by_hand_reads_as_none(tmp_path: Path) -> None:
    first = make_rig(tmp_path)
    assert save(first).ok
    shutil.rmtree(first.store.path_of("home"))
    assert make_rig(tmp_path).manager.status().active_map is None


def test_unreadable_owner_file_reads_as_none(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    (Path(rig.config.maps_root)).mkdir(parents=True, exist_ok=True)
    (Path(rig.config.maps_root) / ACTIVE_MAP_FILE).write_text("{not json")
    assert make_rig(tmp_path).manager.status().active_map is None


def test_failed_replace_on_owner_delete_is_not_ok(rig: Rig) -> None:
    assert save(rig).ok
    rig.poi.replace_ok = False
    result = delete(rig)
    assert not result.ok
    assert "replace rejected" in result.message and "stayed" in result.message
    assert rig.poi.pois == LIVE


def test_autosave_before_refused_load_does_not_move_owner(rig: Rig) -> None:
    rig.save_map("home")
    assert save(rig, "office").ok
    rig.slam.deserialize_ok = False
    assert not load(rig).ok
    assert rig.manager.status().active_map == "office"
    assert json.loads((Path(rig.config.maps_root) / ACTIVE_MAP_FILE).read_text()) == {"active_map": "office"}
    assert BUNDLE_POIS
