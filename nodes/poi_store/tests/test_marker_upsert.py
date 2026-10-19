"""upsert_marker: first sighting adds, later sightings merge or relocate, user edits survive."""

import json
import math
from pathlib import Path

import numpy as np
import pytest
from ros2_common.marker_payload import MarkerInfo

from poi_store.config import MarkerMergeConfig
from poi_store.markers import angle_between_deg
from poi_store.store import PoiStore

NOW = [1000.0]


def info(key: str = "base/HOME", **over) -> dict:
    """MarkerInfo dict for a key."""
    kind, ident = key.split("/")
    d = {
        "kind": kind,
        "id": ident,
        "key": key,
        "size_mm": 150.0,
        "params": {},
        "payload": "P1/K:BASE/N:HOME/S:150",
        "version": 1,
        "pose_samples": 1,
    }
    d.update(over)
    return MarkerInfo(**d).model_dump()


def quat_z(deg: float, flip: bool = False) -> list[float]:
    """Quaternion for a yaw of deg degrees."""
    h = math.radians(deg) / 2
    q = [0.0, 0.0, math.sin(h), math.cos(h)]
    return [-c for c in q] if flip else q


def sighting(
    x: float = 1.0, y: float = 2.0, yaw: float = 0.0, conf: float = 0.8, key: str = "base/HOME", **over
) -> dict:
    """An upsert_marker sighting poi dict."""
    d = {
        "kind": "marker",
        "x": x,
        "y": y,
        "z": 0.5,
        "orientation": quat_z(yaw),
        "marker": info(key),
        "confidence": conf,
    }
    d.update(over)
    return d


@pytest.fixture
def store(tmp_path: Path) -> PoiStore:
    NOW[0] = 1000.0
    return PoiStore(tmp_path / "poi.json", clock=lambda: NOW[0], markers=MarkerMergeConfig())


def upsert(store: PoiStore, s: dict):
    return store.op_upsert_marker(s)


def test_first_sighting_adds_with_default_name(store: PoiStore) -> None:
    ok, msg, poi = upsert(store, sighting())
    assert ok and msg == "added"
    assert len(store.pois) == 1 and store.revision == 1
    assert poi["name"] == "base HOME" and poi["created_by"] == "agent"
    assert poi["times_seen"] == 1 and poi["first_seen"] == poi["last_seen"] == 1000.0
    assert poi["marker"]["pose_samples"] == 1


def test_ten_noisy_sightings_weighted_mean(store: PoiStore) -> None:
    rng = np.random.default_rng(1)
    xs, ws = [], []
    for i in range(10):
        x = 1.0 + float(rng.normal(0, 0.01))
        c = 0.5 + 0.05 * i
        xs.append(x)
        ws.append(c)
        NOW[0] += 1
        res = upsert(store, sighting(x=x, conf=c))
        assert res[0]
    assert len(store.pois) == 1
    poi = next(iter(store.pois.values()))
    assert poi.times_seen == 10 and poi.marker.pose_samples == 10
    # sequential weighted mean: replay the same recurrence
    mean = xs[0]
    conf = ws[0]
    for i in range(1, 10):
        old_w = min(i, 20) * conf
        mean = (mean * old_w + xs[i] * ws[i]) / (old_w + ws[i])
        conf = (conf * old_w + ws[i] * ws[i]) / (old_w + ws[i])
    assert abs(poi.x - mean) < 1e-3
    assert poi.last_seen == 1010.0 and poi.first_seen == 1001.0


def test_far_sighting_relocates(store: PoiStore) -> None:
    for _ in range(3):
        upsert(store, sighting())
    ok, msg, poi = upsert(store, sighting(x=1.5, conf=0.6))
    assert ok and msg == "updated"
    assert poi["x"] == 1.5 and poi["confidence"] == 0.6
    assert poi["marker"]["pose_samples"] == 1 and poi["times_seen"] == 4


def test_rotation_gate_and_sign_flip(store: PoiStore) -> None:
    upsert(store, sighting(yaw=0.0))
    _, _, poi = upsert(store, sighting(yaw=5.0, orientation=quat_z(5.0, flip=True)))
    assert poi["marker"]["pose_samples"] == 2
    got = poi["orientation"]
    assert angle_between_deg(got, quat_z(2.5)) < 0.2
    _, _, poi = upsert(store, sighting(yaw=30.0))
    assert poi["marker"]["pose_samples"] == 1
    assert angle_between_deg(poi["orientation"], quat_z(30.0)) < 1e-6


def test_user_edits_survive(store: PoiStore) -> None:
    _, _, poi = upsert(store, sighting())
    ok, _, _ = store.op_update({"id": poi["id"], "name": "Dock", "note": "mine", "status": "done"})
    assert ok
    NOW[0] += 5
    _, _, poi = upsert(store, sighting(x=1.01))
    assert (poi["name"], poi["note"], poi["status"]) == ("Dock", "mine", "done")
    assert poi["created_by"] == "agent" and poi["created_at"] == 1000.0 and poi["first_seen"] == 1000.0


def test_marker_fields_come_from_sighting(store: PoiStore) -> None:
    upsert(store, sighting())
    s = sighting()
    s["marker"] = info(size_mm=200.0, params={"A": "B"}, payload="P1/K:BASE/N:HOME/S:200/A:B")
    _, _, poi = upsert(store, s)
    assert poi["marker"]["size_mm"] == 200.0 and poi["marker"]["params"] == {"A": "B"}


def test_two_keys_two_pois(store: PoiStore) -> None:
    upsert(store, sighting(key="base/HOME"))
    upsert(store, sighting(key="charger/C1"))
    assert len(store.pois) == 2


def test_invalid_sighting_changes_nothing(store: PoiStore) -> None:
    upsert(store, sighting())
    bad = sighting()
    del bad["marker"]
    ok, msg, poi = upsert(store, bad)
    assert not ok and poi is None and msg
    assert store.revision == 1 and len(store.pois) == 1
    assert not upsert(store, {"kind": "point", "x": 1, "y": 1})[0]


def test_reload_keeps_matching(tmp_path: Path) -> None:
    path = tmp_path / "poi.json"
    s1 = PoiStore(path, clock=lambda: 1.0, markers=MarkerMergeConfig())
    s1.op_upsert_marker(sighting())
    s2 = PoiStore(path, clock=lambda: 2.0, markers=MarkerMergeConfig())
    ok, msg, poi = s2.op_upsert_marker(sighting())
    assert ok and msg == "updated" and len(s2.pois) == 1 and poi["times_seen"] == 2


def test_handle_message_end_to_end(store: PoiStore) -> None:
    raw = json.dumps({"op": "upsert_marker", "request_id": "r1", "poi": sighting()})
    res = json.loads(store.handle_message(raw))
    assert res["ok"] and res["message"] == "added" and res["request_id"] == "r1"
    res = json.loads(store.handle_message(raw))
    assert res["ok"] and res["message"] == "updated" and res["poi"]["times_seen"] == 2
    assert PoiStore(store.path).revision == 2


def test_config_defaults() -> None:
    cfg = MarkerMergeConfig()
    assert (cfg.relocate_m, cfg.relocate_deg, cfg.max_pose_samples) == (0.25, 20.0, 20)
