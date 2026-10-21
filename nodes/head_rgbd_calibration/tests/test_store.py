"""Tests for the session store: layout, ids, atomic writes, coverage and path safety."""

import json
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.frames import CameraInfoData

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import InvalidId, NotFound, SessionStore
from tests.pose_factory import few_rgb_corner_pose, make_pose

NOW = datetime(2026, 10, 19, 12, 30, 45)
LENS = 1.5
RGB_SIZE = (576, 324)
TOF_SIZE = (240, 180)


@pytest.fixture
def store(tmp_path: Path) -> SessionStore:
    return SessionStore(tmp_path / "state", NodeConfig())


def tof_info() -> CameraInfoData:
    return CameraInfoData(
        width=240,
        height=180,
        distortion_model="plumb_bob",
        d=np.array([0.1, -0.2, 0.0, 0.0, 0.05]),
        k=np.array([191.0, 0, 120, 0, 191, 90, 0, 0, 1]),
        p=np.array([191.0, 0, 120, 0, 0, 191, 90, 0, 0, 0, 1, 0]),
    )


def test_create_session_id_and_session_json(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    assert sid == "calib-20261019-123045"
    data = json.loads((store.state_dir / "sessions" / sid / "session.json").read_text())
    assert data["id"] == sid
    assert data["lens_position"] == LENS
    assert data["next_pose"] == 1
    assert data["tof_info"] is None
    assert data["rgb_size"] is None and data["tof_size"] is None


def test_create_session_same_second_gets_suffix(store: SessionStore) -> None:
    ids = [store.create_session(NOW, LENS) for _ in range(3)]
    assert ids == ["calib-20261019-123045", "calib-20261019-123045-2", "calib-20261019-123045-3"]


def test_sessions_listed_newest_first(store: SessionStore) -> None:
    old = store.create_session(datetime(2026, 10, 18, 8, 0, 0), LENS)
    new = store.create_session(NOW, LENS)
    store.add_pose(new, make_pose())
    store.add_pose(new, make_pose(2, seed=2))
    rid = store.new_result_id(NOW)
    store.result_dir(new, rid).mkdir(parents=True)
    (store.result_dir(new, rid) / "result.json").write_text(json.dumps({"id": rid}))
    listed = store.list_sessions()
    assert [s["id"] for s in listed] == [new, old]
    assert (listed[0]["pose_count"], listed[0]["result_count"]) == (2, 1)
    assert (listed[1]["pose_count"], listed[1]["result_count"]) == (0, 0)


def test_list_sessions_empty_state_dir(store: SessionStore) -> None:
    assert store.list_sessions() == []


def test_set_tof_info_roundtrip(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    info = tof_info()
    store.set_tof_info(sid, info)
    stored = json.loads((store.state_dir / "sessions" / sid / "session.json").read_text())["tof_info"]
    assert set(stored) == {"k", "p", "d", "width", "height", "distortion_model"}
    loaded = store.load_session(sid)["tof_info"]
    assert loaded["distortion_model"] == "plumb_bob" and loaded["k"] == list(info.k)


def test_set_image_sizes_and_mismatch_refused(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    store.set_image_sizes(sid, RGB_SIZE, TOF_SIZE)
    store.set_image_sizes(sid, RGB_SIZE, TOF_SIZE)
    data = store.load_session(sid)
    assert data["rgb_size"] == list(RGB_SIZE) and data["tof_size"] == list(TOF_SIZE)
    with pytest.raises(ValueError, match="size"):
        store.set_image_sizes(sid, (640, 480), TOF_SIZE)


def test_add_pose_writes_all_files_atomically(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pose = make_pose()
    pid = store.add_pose(sid, pose)
    assert pid == "001"
    pose_dir = store.state_dir / "sessions" / sid / "poses" / pid
    assert sorted(p.name for p in pose_dir.iterdir()) == [
        "amplitude.png",
        "depth.png",
        "pose.json",
        "rgb.jpg",
        "rgb_thumb.jpg",
        "tof_thumb.jpg",
    ]
    assert (pose_dir / "rgb.jpg").read_bytes() == pose.rgb_raw
    amp = cv2.imread(str(pose_dir / "amplitude.png"), cv2.IMREAD_UNCHANGED)
    assert amp.dtype == np.uint16 and np.array_equal(amp, pose.amplitude.image)
    depth = cv2.imread(str(pose_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    assert depth.dtype == np.uint16 and np.array_equal(depth, pose.depth.image)
    meta = json.loads((pose_dir / "pose.json").read_text())
    assert meta["rgb"]["stamp_ns"] == pose.rgb.stamp_ns and meta["tof"]["stamp_ns"] == pose.amplitude.stamp_ns
    assert meta["rgb"]["ids"] == pose.rgb_detection.ids.tolist()
    assert np.allclose(meta["tof"]["points"], pose.tof_detection.points)
    session = store.load_session(sid)
    assert session["next_pose"] == 2 and session["rgb_size"] == list(RGB_SIZE) and session["tof_size"] == list(TOF_SIZE)
    assert not [p for p in (pose_dir.parent).iterdir() if p.name != pid]


def test_add_pose_without_depth_and_raw_encodes_frame(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, make_pose(with_depth=False, raw=False))
    pose_dir = store.state_dir / "sessions" / sid / "poses" / pid
    assert not (pose_dir / "depth.png").exists()
    assert cv2.imread(str(pose_dir / "rgb.jpg")).shape[:2] == (324, 576)


def test_pose_ids_never_reused_after_delete(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    first = store.add_pose(sid, make_pose())
    second = store.add_pose(sid, make_pose(2, seed=2))
    store.delete_pose(sid, second)
    third = store.add_pose(sid, make_pose(3, seed=3))
    assert (first, second, third) == ("001", "002", "003")
    store.delete_pose(sid, first)
    assert store.load_session(sid)["pose_ids"] == ["003"]


def test_delete_pose_missing_is_not_found(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    with pytest.raises(NotFound):
        store.delete_pose(sid, "001")


def test_add_pose_of_other_size_refused_and_not_stored(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    store.add_pose(sid, make_pose())
    pose = make_pose(2, seed=2)
    small = cv2.resize(pose.rgb.image, (288, 162))
    other = type(pose)(**{**pose.__dict__, "rgb": type(pose.rgb)(pose.rgb.stamp_ns, small), "rgb_raw": None})
    with pytest.raises(ValueError, match="size"):
        store.add_pose(sid, other)
    assert store.load_session(sid)["pose_ids"] == ["001"]
    assert store.load_session(sid)["next_pose"] == 2


def test_add_pose_with_empty_amplitude_not_stored(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pose = make_pose()
    flat = type(pose.amplitude)(pose.amplitude.stamp_ns, np.full(pose.amplitude.image.shape, 7, dtype=np.uint16))
    empty = type(pose)(**{**pose.__dict__, "amplitude": flat})
    with pytest.raises(ValueError, match="empty"):
        store.add_pose(sid, empty)
    assert store.load_session(sid)["pose_ids"] == []
    poses = store.state_dir / "sessions" / sid / "poses"
    assert not poses.exists() or not list(poses.iterdir())


def test_load_views_round_trip(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    store.set_tof_info(sid, tof_info())
    a, b = make_pose(), make_pose(2, seed=2, with_depth=False)
    store.add_pose(sid, a)
    store.add_pose(sid, b)
    loaded = store.load_views(sid)
    assert loaded.pose_ids == ["001", "002"]
    assert loaded.rgb_size == RGB_SIZE and loaded.tof_size == TOF_SIZE
    assert loaded.lens_position == LENS
    assert np.array_equal(loaded.tof_info.k, tof_info().k) and loaded.tof_info.width == 240
    first, second = loaded.views
    assert np.array_equal(first.rgb.ids, a.rgb_detection.ids)
    assert np.allclose(first.tof.points, a.tof_detection.points)
    assert np.array_equal(first.depth, a.depth.image)
    assert second.depth is None


def test_load_views_without_tof_info(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    assert store.load_views(sid).tof_info is None


def test_coverage_counts_corners_per_cell(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    empty = store.coverage(sid)
    assert empty["rgb"]["cols"] == 4 and empty["rgb"]["rows"] == 3
    assert sum(map(sum, empty["rgb"]["counts"])) == 0 and sum(map(sum, empty["tof"]["counts"])) == 0
    pose = make_pose()
    store.add_pose(sid, pose)
    cov = store.coverage(sid)
    assert sum(map(sum, cov["rgb"]["counts"])) == len(pose.rgb_detection.ids)
    assert sum(map(sum, cov["tof"]["counts"])) == len(pose.tof_detection.ids)
    assert len(cov["rgb"]["counts"]) == 3 and len(cov["rgb"]["counts"][0]) == 4
    first = pose.rgb_detection.points[0]
    col, row = int(first[0] / RGB_SIZE[0] * 4), int(first[1] / RGB_SIZE[1] * 3)
    assert cov["rgb"]["counts"][row][col] >= 1


def test_coverage_drops_with_deleted_pose(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, make_pose())
    store.delete_pose(sid, pid)
    assert sum(map(sum, store.coverage(sid)["rgb"]["counts"])) == 0


def test_results_ids_and_listing_newest_first(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    assert store.new_result_id(NOW) == "r-20261019-123045"
    for rid in ("r-20261019-123045", "r-20261019-130000"):
        store.result_dir(sid, rid).mkdir(parents=True)
        (store.result_dir(sid, rid) / "result.json").write_text(json.dumps({"id": rid}))
    store.result_dir(sid, "r-20261019-140000").mkdir(parents=True)
    assert [r["id"] for r in store.list_results(sid)] == ["r-20261019-130000", "r-20261019-123045"]


def test_thumbnail_path(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, make_pose())
    assert store.thumbnail_path(sid, pid, "rgb").name == "rgb_thumb.jpg"
    assert store.thumbnail_path(sid, pid, "tof").name == "tof_thumb.jpg"
    with pytest.raises(InvalidId):
        store.thumbnail_path(sid, pid, "depth")
    with pytest.raises(NotFound):
        store.thumbnail_path(sid, "009", "rgb")


@pytest.mark.parametrize(
    "sid", ["../etc", "calib-x", "calib-20261019-123045/../..", "", "/abs", "calib-20261019-123045\n"]
)
def test_invalid_session_ids_rejected(store: SessionStore, sid: str) -> None:
    with pytest.raises(InvalidId):
        store.load_session(sid)


@pytest.mark.parametrize("pid", ["1", "01", "../001", "00a", "001/..", ".."])
def test_rejects_traversal_ids(store: SessionStore, pid: str) -> None:
    sid = store.create_session(NOW, LENS)
    with pytest.raises(InvalidId):
        store.delete_pose(sid, pid)
    with pytest.raises(InvalidId):
        store.thumbnail_path(sid, pid, "rgb")


def test_invalid_result_id_rejected(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    with pytest.raises(InvalidId):
        store.result_dir(sid, "../../x")


def test_missing_session_not_found(store: SessionStore) -> None:
    with pytest.raises(NotFound):
        store.load_session("calib-20261019-000000")
    with pytest.raises(NotFound):
        store.add_pose("calib-20261019-000000", make_pose())


def test_symlinked_session_escaping_state_dir_rejected(store: SessionStore, tmp_path: Path) -> None:
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "session.json").write_text("{}")
    sessions = store.state_dir / "sessions"
    sessions.mkdir(parents=True)
    (sessions / "calib-20261019-111111").symlink_to(outside)
    with pytest.raises(InvalidId):
        store.load_session("calib-20261019-111111")


def test_concurrent_add_pose_gives_unique_ids(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    poses = [make_pose(i + 1, seed=i + 1) for i in range(4)]
    ids: list[str] = []
    threads = [threading.Thread(target=lambda p=p: ids.append(store.add_pose(sid, p))) for p in poses]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert sorted(ids) == ["001", "002", "003", "004"]


def read_pose_json(store: SessionStore, sid: str, pid: str) -> dict:
    return json.loads((store.state_dir / "sessions" / sid / "poses" / pid / "pose.json").read_text())


def test_add_pose_writes_hints_of_a_few_corner_pose(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, few_rgb_corner_pose())
    hints = read_pose_json(store, sid, pid)["hints"]
    assert hints[0] == "few corners: 6 RGB - move the board closer / fill the frame"
    assert store.pose_hints(sid, pid) == hints


def test_add_pose_of_a_good_pose_has_empty_hints(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, make_pose())
    assert read_pose_json(store, sid, pid)["hints"] == []
    assert store.pose_hints(sid, pid) == []


def test_pose_hints_legacy_pose_json_without_key_is_computed(store: SessionStore) -> None:
    sid = store.create_session(NOW, LENS)
    pid = store.add_pose(sid, few_rgb_corner_pose())
    path = store.state_dir / "sessions" / sid / "poses" / pid / "pose.json"
    meta = json.loads(path.read_text())
    del meta["hints"]
    path.write_text(json.dumps(meta))
    assert store.pose_hints(sid, pid)[0].startswith("few corners: 6 RGB")
    assert "hints" not in json.loads(path.read_text())
