"""Tests for the filtered poi_store clear op (creator, kinds, keep_zones, ids, statuses)."""

from pathlib import Path

import pytest

from poi_store.models import Command
from poi_store.store import PoiStore


@pytest.fixture
def store(tmp_path: Path) -> PoiStore:
    return PoiStore(tmp_path / "poi.json", clock=lambda: 100.0)


def add(store: PoiStore, **kw) -> str:
    base = {"kind": "point", "x": 1.0, "y": 2.0, "name": "p", "created_by": "agent"}
    base.update(kw)
    ok, msg, poi = store.apply(Command(op="add", poi=base))
    assert ok, msg
    return poi["id"]


def area(store: PoiStore, zone: str = "none", **kw) -> str:
    base = {"kind": "area", "polygon": [[0, 0], [2, 0], [2, 2], [0, 2]], "zone": zone, "created_by": "user"}
    if zone == "slow":
        base["speed_limit_pct"] = 50
    base.update(kw)
    return add(store, **base)


def clear(store: PoiStore, **kw):
    return store.apply(Command(op="clear", **kw))


def test_clear_without_created_by_removes_every_creator(store):
    add(store, created_by="agent")
    add(store, created_by="user")
    ok, _, result = clear(store)
    assert ok and result == {"removed": 2, "pois": 2, "photos": 0} and store.pois == {}


def test_clear_kinds_filter(store):
    p = add(store)
    a = area(store)
    o = add(store, kind="object")
    _, _, result = clear(store, kinds=["area", "object"])
    assert result == {"removed": 2, "pois": 2, "photos": 0} and list(store.pois) == [p]
    assert a not in store.pois and o not in store.pois


def test_clear_keep_zones_keeps_keepout_and_slow_areas(store):
    keep = area(store, "keepout")
    slow = area(store, "slow", status="done")
    plain = area(store)
    pt = add(store)
    _, _, result = clear(store, keep_zones=True)
    assert result == {"removed": 2, "pois": 2, "photos": 0}
    assert set(store.pois) == {keep, slow}
    assert plain not in store.pois and pt not in store.pois


def test_clear_filters_combine(store):
    agent_pt = add(store, created_by="agent")
    add(store, created_by="user")
    agent_area = area(store, created_by="agent")
    keep = area(store, "keepout", created_by="agent")
    done = add(store, created_by="agent", status="done")
    _, _, result = clear(store, created_by="agent", kinds=["point", "area"], keep_zones=True, statuses=["open"])
    assert result == {"removed": 2, "pois": 2, "photos": 0}
    assert agent_pt not in store.pois and agent_area not in store.pois
    assert keep in store.pois and done in store.pois


def test_clear_ids_removes_only_listed(store):
    a = add(store)
    b = add(store)
    c = add(store)
    _, _, result = clear(store, ids=[a, c])
    assert result == {"removed": 2, "pois": 2, "photos": 0} and list(store.pois) == [b]


def test_clear_ids_combine_with_keep_zones(store):
    keep = area(store, "keepout")
    pt = add(store)
    other = add(store)
    _, _, result = clear(store, ids=[keep, pt], keep_zones=True)
    assert result == {"removed": 1, "pois": 1, "photos": 0}
    assert set(store.pois) == {keep, other}


def test_clear_ids_empty_or_unknown_removes_nothing(store):
    add(store)
    revision = store.revision
    assert clear(store, ids=[])[2] == {"removed": 0, "pois": 0, "photos": 0}
    assert clear(store, ids=["nope"])[2] == {"removed": 0, "pois": 0, "photos": 0}
    assert clear(store, kinds=[])[2] == {"removed": 0, "pois": 0, "photos": 0}
    assert store.revision == revision and len(store.pois) == 1


def test_clear_agent_only_unchanged(store):
    agent = add(store, created_by="agent")
    agent_obj = add(store, kind="object", created_by="agent")
    agent_zone = area(store, "keepout", created_by="agent")
    user = add(store, created_by="user")
    _, _, result = clear(store, created_by="agent")
    assert result == {"removed": 3, "pois": 3, "photos": 0}
    assert list(store.pois) == [user]
    assert agent not in store.pois and agent_obj not in store.pois and agent_zone not in store.pois


def test_clear_nothing_matching_keeps_revision(store):
    add(store, created_by="user")
    revision = store.revision
    _, _, result = clear(store, created_by="agent", kinds=["area"])
    assert result == {"removed": 0, "pois": 0, "photos": 0} and store.revision == revision


def test_clear_removal_bumps_revision_once(store):
    add(store)
    add(store)
    revision = store.revision
    clear(store)
    assert store.revision == revision + 1


def test_clear_command_accepts_photo_kind():
    assert Command(op="clear", kinds=["photo", "point"]).kinds == ["photo", "point"]
    with pytest.raises(ValueError):
        Command(op="clear", kinds=["bogus"])


def test_clear_message_reports_both_counts(store):
    add(store)
    _, msg, result = clear(store)
    assert msg == "cleared 1 POIs and 0 photos" and result == {"removed": 1, "pois": 1, "photos": 0}
