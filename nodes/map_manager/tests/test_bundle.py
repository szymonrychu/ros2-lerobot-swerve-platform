"""Tests for the named map bundle store."""

import json
from pathlib import Path

import numpy as np
import pytest
from ros2_common.maps import Pose2D
from ros2_common.scan_agreement import GridMap

from map_manager.bundle import (
    BundleExists,
    BundleInvalid,
    BundleMeta,
    BundleNotFound,
    BundleStore,
)


def make_grid() -> GridMap:
    cells = np.full((4, 5), -1, dtype=np.int8)
    cells[1, 2] = 100
    cells[2, 3] = 0
    return GridMap(resolution=0.05, origin_x=-1.0, origin_y=-2.0, origin_yaw=0.25, cells=cells)


def make_meta(name: str = "home") -> BundleMeta:
    return BundleMeta(
        version=1,
        name=name,
        saved_at=1700000000.0,
        note="n",
        last_pose=Pose2D(x=1.0, y=2.0, yaw=0.5),
        pois=[{"id": "a", "name": "kitchen"}, {"id": "b", "name": "door"}],
        poi_revision=7,
        map_info={"resolution": 0.05, "width": 5, "height": 4, "origin_x": -1.0, "origin_y": -2.0, "origin_yaw": 0.25},
        laser_frame="laser",
    )


def stage(store: BundleStore, name: str = "home", files: tuple[str, ...] = ("map.posegraph", "map.data")) -> Path:
    tmp = store.begin(name)
    for f in files:
        (tmp / f).write_bytes(b"x" + f.encode())
    return tmp


@pytest.fixture
def store(tmp_path: Path) -> BundleStore:
    return BundleStore(tmp_path / "maps")


def test_commit_writes_bundle(store: BundleStore) -> None:
    tmp = stage(store)
    summary = store.commit(tmp, "home", make_meta(), make_grid(), overwrite=False)
    assert summary.name == "home" and summary.valid and summary.poi_count == 2
    assert summary.last_pose == Pose2D(x=1.0, y=2.0, yaw=0.5)
    final = store.root / "home"
    assert {p.name for p in final.iterdir()} == {"map.posegraph", "map.data", "occupancy.npz", "bundle.json"}
    assert not tmp.exists()


def test_begin_creates_temp_dir(store: BundleStore) -> None:
    tmp = store.begin("home")
    assert tmp.is_dir() and tmp.parent == store.root and tmp.name.startswith(".tmp-home-")


def test_occupancy_round_trip_keeps_unknown(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    loaded = store.load("home")
    grid = make_grid()
    assert loaded.grid.cells.dtype == np.int8
    assert np.array_equal(loaded.grid.cells, grid.cells)
    assert (loaded.grid.resolution, loaded.grid.origin_x, loaded.grid.origin_y, loaded.grid.origin_yaw) == (
        0.05,
        -1.0,
        -2.0,
        0.25,
    )
    assert loaded.meta == make_meta()
    assert loaded.posegraph_base == store.root / "home" / "map"


def test_overwrite_refused_without_flag(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    tmp = stage(store)
    with pytest.raises(BundleExists):
        store.commit(tmp, "home", make_meta(), make_grid(), overwrite=False)
    assert not tmp.exists()
    assert store.load("home").meta.note == "n"


def test_overwrite_swaps(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    meta = make_meta().model_copy(update={"note": "new"})
    store.commit(stage(store), "home", meta, make_grid(), overwrite=True)
    assert store.load("home").meta.note == "new"
    assert sorted(p.name for p in store.root.iterdir()) == ["home"]


def test_failure_leaves_nothing(store: BundleStore) -> None:
    tmp = stage(store, files=("map.posegraph",))
    with pytest.raises(BundleInvalid, match="map.data"):
        store.commit(tmp, "home", make_meta(), make_grid(), overwrite=False)
    assert list(store.root.iterdir()) == []


def test_failure_keeps_existing_bundle(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    tmp = stage(store, files=("map.data",))
    with pytest.raises(BundleInvalid):
        store.commit(tmp, "home", make_meta(), make_grid(), overwrite=True)
    assert store.load("home").meta.note == "n"
    assert sorted(p.name for p in store.root.iterdir()) == ["home"]


def test_list_sorted_with_invalid_bundle(store: BundleStore) -> None:
    store.commit(stage(store, "b"), "b", make_meta("b"), make_grid(), overwrite=False)
    store.commit(stage(store, "a"), "a", make_meta("a"), make_grid(), overwrite=False)
    (store.root / "broken").mkdir()
    (store.root / "broken" / "bundle.json").write_text("{not json")
    (store.root / "empty").mkdir()
    stage(store, "c")  # temp dir is ignored
    result = store.list()
    assert [s.name for s in result] == ["a", "b", "broken", "empty"]
    assert [s.valid for s in result] == [True, True, False, False]
    assert result[2].error and result[3].error


def test_load_missing_file_names_it(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    (store.root / "home" / "occupancy.npz").unlink()
    with pytest.raises(BundleInvalid, match="occupancy.npz"):
        store.load("home")
    (store.root / "home" / "bundle.json").write_text(json.dumps({"version": 2}))
    with pytest.raises(BundleInvalid, match="bundle.json"):
        store.load("home")


def test_load_unknown_name(store: BundleStore) -> None:
    with pytest.raises(BundleNotFound):
        store.load("nope")


def test_delete(store: BundleStore) -> None:
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    store.delete("home")
    assert store.list() == []
    with pytest.raises(BundleNotFound):
        store.delete("home")


def test_cleanup_temp(store: BundleStore) -> None:
    stage(store, "x")
    store.commit(stage(store), "home", make_meta(), make_grid(), overwrite=False)
    stage(store, "y")
    store.cleanup_temp()
    assert sorted(p.name for p in store.root.iterdir()) == ["home"]


@pytest.mark.parametrize("name", ["../evil", "a/b", "", ".hidden", "x" * 41])
def test_path_traversal_refused(store: BundleStore, name: str) -> None:
    with pytest.raises(ValueError):
        store.begin(name)
    with pytest.raises(ValueError):
        store.load(name)
    with pytest.raises(ValueError):
        store.delete(name)
