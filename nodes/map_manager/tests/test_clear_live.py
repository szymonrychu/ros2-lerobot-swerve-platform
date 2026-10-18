"""Tests for the clear_live op and the kidnap discard emptying live POIs and photos and the live owner."""

from __future__ import annotations

from pathlib import Path

import pytest
from ros2_common.maps import MapCommand

from tests.test_live_owner import LIVE, delete, save
from tests.test_manager import Rig, make_rig
from tests.test_relocalise import LIVE_POIS, named_tree, relocalise, relocalise_rig


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    return make_rig(tmp_path)


def clear(rig: Rig):
    return rig.manager.handle(MapCommand(op="clear_live", request_id="c1"))


def test_clear_live_empties_pois_and_photos_and_reports_counts(rig: Rig) -> None:
    assert save(rig).ok
    result = clear(rig)
    assert result.ok, result.message
    assert rig.poi.pois == []
    assert result.data == {"pois_removed": 2, "photos_removed": 1, "active_map": None}
    assert result.message == "live set cleared: removed 2 POIs and 1 photos; saved maps kept"


def test_clear_live_clears_persisted_owner_so_delete_leaves_new_live_set(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    assert save(rig).ok
    assert clear(rig).ok
    assert make_rig(tmp_path).manager.status().active_map is None
    rig.poi.pois = [{"id": "n1", "name": "new"}]
    result = delete(rig)
    assert result.ok and result.data["was_active"] is False
    assert rig.poi.pois == [{"id": "n1", "name": "new"}]


def test_clear_live_failed_replace_keeps_owner_and_is_not_ok(rig: Rig) -> None:
    assert save(rig).ok
    rig.poi.replace_ok = False
    result = clear(rig)
    assert not result.ok
    assert "replace rejected" in result.message
    assert rig.manager.status().active_map == "home"
    assert rig.poi.pois == LIVE


def test_clear_live_keeps_bundles_on_disk(rig: Rig) -> None:
    assert save(rig).ok
    tree = named_tree(rig)
    assert clear(rig).ok
    assert named_tree(rig) == tree
    assert [m.name for m in rig.manager.status().maps] == ["home"]


def test_clear_live_refused_while_busy(rig: Rig) -> None:
    rig.manager.operation_lock.acquire()
    try:
        result = clear(rig)
    finally:
        rig.manager.operation_lock.release()
    assert not result.ok and result.message.startswith("busy")


def kidnapped(tmp_path: Path, discard: bool = True):
    rig, clock = relocalise_rig(tmp_path)
    rig.poi.pois = [dict(p) for p in LIVE_POIS] + [{"id": "ph1", "name": "pic", "kind": "photo"}]
    rig.save_map("home")
    rig.manager.set_active_map("home")
    rig.manager.record_pose()
    rig.slam.confirmed_pose = None
    clock.now += 5
    return rig, relocalise(rig, discard_on_fail=discard)


def test_failed_relocalise_with_discard_removes_photos_and_clears_owner(tmp_path: Path) -> None:
    rig, result = kidnapped(tmp_path)
    assert not result.ok
    assert rig.poi.pois == []
    assert result.data["discarded_pois"] == len(LIVE_POIS)
    assert result.data["discarded_photos"] == 1
    assert result.data["active_map"] is None
    assert "1 photos" in result.message
    assert make_rig(tmp_path).manager.status().active_map is None


def test_failed_relocalise_discard_keeps_bundles(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.save_map("home")
    rig.manager.record_pose()
    tree = named_tree(rig)
    rig.slam.confirmed_pose = None
    relocalise(rig, discard_on_fail=True)
    assert named_tree(rig) == tree


def test_failed_discard_keeps_owner(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.save_map("home")
    rig.manager.set_active_map("home")
    rig.manager.record_pose()
    rig.slam.confirmed_pose = None
    rig.poi.replace_ok = False
    result = relocalise(rig, discard_on_fail=True)
    assert result.data["discarded_pois"] == 0 and result.data["discarded_photos"] == 0
    assert result.data["active_map"] == "home"


def test_recovered_relocalise_touches_nothing(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.manager.set_active_map("home")
    rig.manager.record_pose()
    clock.now += 5
    result = relocalise(rig, discard_on_fail=True)
    assert result.ok
    assert result.data["discarded_photos"] == 0
    assert result.data["active_map"] == "home"
    assert "poi.replace" not in rig.log.names()
