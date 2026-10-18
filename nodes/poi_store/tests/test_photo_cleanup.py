"""The clear op removes photo POIs (record and JPEG) by the same filters as other POIs and reports both counts."""

from poi_store.models import Command

from . import test_store
from .test_store import add_photo, add_poi, photo_id

photos = test_store.photos
pstore = test_store.pstore


def jpeg(photos, n: int):
    return photos.root / f"{photo_id(n)}.jpg"


def clear(store, **kw):
    return store.apply(Command(op="clear", **kw))


def seed(pstore, photos):
    """Agent point + user point + agent photo 1 + user photo 2 (status done)."""
    agent_poi = add_poi(pstore, created_by="agent")
    user_poi = add_poi(pstore, name="u", created_by="user")
    add_photo(pstore, photos, 1, created_by="agent")
    add_photo(pstore, photos, 2, created_by="user")
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(2), "status": "done"}))
    assert ok, msg
    return agent_poi, user_poi


def test_clear_without_filters_removes_pois_photos_and_jpegs(pstore, photos):
    seed(pstore, photos)
    revision = pstore.revision
    ok, msg, result = clear(pstore)
    assert ok and result == {"removed": 4, "pois": 2, "photos": 2}
    assert msg == "cleared 2 POIs and 2 photos"
    assert pstore.pois == {} and photos.records() == []
    assert not jpeg(photos, 1).exists() and not jpeg(photos, 2).exists()
    assert pstore.revision == revision + 1


def test_clear_kinds_photo_removes_only_photos(pstore, photos):
    agent_poi, user_poi = seed(pstore, photos)
    ok, _, result = clear(pstore, kinds=["photo"])
    assert ok and result == {"removed": 2, "pois": 0, "photos": 2}
    assert set(pstore.pois) == {agent_poi, user_poi} and photos.records() == []


def test_clear_kinds_point_keeps_photos(pstore, photos):
    seed(pstore, photos)
    ok, _, result = clear(pstore, kinds=["point"])
    assert ok and result == {"removed": 2, "pois": 2, "photos": 0}
    assert len(photos.records()) == 2 and jpeg(photos, 1).exists()


def test_clear_created_by_user_removes_only_user_photos_and_pois(pstore, photos):
    agent_poi, _ = seed(pstore, photos)
    ok, _, result = clear(pstore, created_by="user")
    assert ok and result == {"removed": 2, "pois": 1, "photos": 1}
    assert list(pstore.pois) == [agent_poi]
    assert [r.id for r in photos.records()] == [photo_id(1)]
    assert jpeg(photos, 1).exists() and not jpeg(photos, 2).exists()


def test_clear_statuses_matches_photo_status(pstore, photos):
    seed(pstore, photos)
    ok, _, result = clear(pstore, statuses=["done"])
    assert ok and result == {"removed": 1, "pois": 0, "photos": 1}
    assert [r.id for r in photos.records()] == [photo_id(1)]


def test_clear_ids_mixed_removes_exactly_those(pstore, photos):
    agent_poi, user_poi = seed(pstore, photos)
    ok, _, result = clear(pstore, ids=[agent_poi, photo_id(2), "f" * 32])
    assert ok and result == {"removed": 2, "pois": 1, "photos": 1}
    assert list(pstore.pois) == [user_poi]
    assert [r.id for r in photos.records()] == [photo_id(1)]


def test_clear_keep_zones_still_removes_photos(pstore, photos):
    seed(pstore, photos)
    ok, _, result = clear(pstore, keep_zones=True, kinds=["photo"])
    assert ok and result["photos"] == 2 and photos.records() == []


def test_deleting_non_photo_poi_keeps_linked_photo_and_jpeg(pstore, photos):
    target = add_poi(pstore)
    add_photo(pstore, photos, 1, poi_ids=[target])
    ok, _, _ = pstore.apply(Command(op="delete", poi={"id": target}))
    assert ok
    assert photos.get(photo_id(1)).poi_ids == [] and jpeg(photos, 1).exists()


def test_single_delete_of_photo_removes_jpeg(pstore, photos):
    add_photo(pstore, photos, 1)
    ok, _, _ = pstore.apply(Command(op="delete", poi={"id": photo_id(1)}))
    assert ok and photos.records() == [] and not jpeg(photos, 1).exists()


def test_clear_matching_nothing_does_not_bump_revision(pstore, photos):
    seed(pstore, photos)
    revision = pstore.revision
    ok, msg, result = clear(pstore, ids=["f" * 32])
    assert ok and result == {"removed": 0, "pois": 0, "photos": 0} and msg == "cleared 0 POIs and 0 photos"
    assert pstore.revision == revision


def test_photo_store_oserror_leaves_pois_and_revision_untouched(pstore, photos, monkeypatch):
    seed(pstore, photos)
    revision, pois = pstore.revision, dict(pstore.pois)

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(photos, "rewrite", boom)
    ok, msg, _ = clear(pstore)
    assert not ok and "disk full" in msg
    assert pstore.pois == pois and pstore.revision == revision
    assert len(photos.records()) == 2
