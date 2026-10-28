"""Tests for the session store: layout, ids, atomic writes, coverage and path safety."""

import dataclasses
import json
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.frames import StampedImage
from ros2_common.head_rgbd_calibration.views import ARM_JOINTS, HEAD_JOINTS

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import InvalidId, NotFound, SessionStore
from tests.pose_factory import RGB_SIZE, factory_info, few_corner_still, make_still, still_arm_pose

NOW = datetime(2026, 10, 19, 12, 30, 45)


@pytest.fixture
def store(tmp_path: Path) -> SessionStore:
    return SessionStore(tmp_path / "state", NodeConfig())


def test_create_session_id_and_session_json(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    assert sid == "calib-20261019-123045"
    data = json.loads((store.state_dir / "sessions" / sid / "session.json").read_text())
    assert data == {"id": sid, "kind": "intrinsics", "created": NOW.isoformat(), "rgb_size": None, "next_pose": 1}


def test_create_session_same_second_gets_suffix(store: SessionStore) -> None:
    ids = [store.create_session(NOW) for _ in range(3)]
    assert ids == ["calib-20261019-123045", "calib-20261019-123045-2", "calib-20261019-123045-3"]


def test_sessions_listed_newest_first(store: SessionStore) -> None:
    old = store.create_session(datetime(2026, 10, 18, 8, 0, 0))
    new = store.create_session(NOW)
    store.add_pose(new, make_still())
    store.add_pose(new, make_still(2, seed=2))
    rid = store.new_result_id(NOW)
    store.result_dir(new, rid).mkdir(parents=True)
    (store.result_dir(new, rid) / "result.json").write_text(json.dumps({"id": rid}))
    listed = store.list_sessions()
    assert [s["id"] for s in listed] == [new, old]
    assert (listed[0]["pose_count"], listed[0]["result_count"]) == (2, 1)
    assert (listed[1]["pose_count"], listed[1]["result_count"]) == (0, 0)


def test_list_sessions_empty_state_dir(store: SessionStore) -> None:
    assert store.list_sessions() == []


def test_add_pose_writes_all_files_atomically(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    still = make_still()
    pid = store.add_pose(sid, still)
    assert pid == "001"
    pose_dir = store.state_dir / "sessions" / sid / "poses" / pid
    assert sorted(p.name for p in pose_dir.iterdir()) == [
        "depth.png",
        "depth_thumb.jpg",
        "pose.json",
        "rgb.jpg",
        "rgb_thumb.jpg",
    ]
    assert (pose_dir / "rgb.jpg").read_bytes() == still.rgb_raw
    depth = cv2.imread(str(pose_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    assert depth.dtype == np.uint16 and np.array_equal(depth, still.depth.image)
    meta = json.loads((pose_dir / "pose.json").read_text())
    assert meta["rgb"]["stamp_ns"] == still.rgb.stamp_ns and meta["depth_stamp_ns"] == still.depth.stamp_ns
    assert meta["rgb"]["ids"] == still.detection.ids.tolist()
    assert np.allclose(meta["rgb"]["points"], still.detection.points)
    assert meta["camera_info"]["k"] == list(still.camera_info.k) and meta["camera_info"]["width"] == RGB_SIZE[0]
    assert 1000 < meta["plane_bias_samples"] < int(np.count_nonzero(still.depth.image))
    session = store.load_session(sid)
    assert session["next_pose"] == 2 and session["rgb_size"] == list(RGB_SIZE)
    assert not [p for p in (pose_dir.parent).iterdir() if p.name != pid]


def test_add_pose_without_raw_encodes_frame(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, make_still(raw=False))
    pose_dir = store.state_dir / "sessions" / sid / "poses" / pid
    assert cv2.imread(str(pose_dir / "rgb.jpg")).shape[:2] == (RGB_SIZE[1], RGB_SIZE[0])


def test_pose_ids_never_reused_after_delete(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    first = store.add_pose(sid, make_still())
    second = store.add_pose(sid, make_still(2, seed=2))
    store.delete_pose(sid, second)
    third = store.add_pose(sid, make_still(3, seed=3))
    assert (first, second, third) == ("001", "002", "003")
    store.delete_pose(sid, first)
    assert store.load_session(sid)["pose_ids"] == ["003"]


def test_delete_pose_missing_is_not_found(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    with pytest.raises(NotFound):
        store.delete_pose(sid, "001")


def assert_nothing_stored(store: SessionStore, sid: str, count: int = 0) -> None:
    assert len(store.load_session(sid)["pose_ids"]) == count
    assert store.load_session(sid)["next_pose"] == count + 1


def test_add_pose_of_other_size_refused_and_not_stored(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    store.add_pose(sid, make_still())
    still = make_still(2, seed=2)
    small_rgb = cv2.resize(still.rgb.image, (288, 162))
    small_depth = cv2.resize(still.depth.image, (288, 162), interpolation=cv2.INTER_NEAREST)
    small_info = dataclasses.replace(factory_info(), width=288, height=162)
    other = dataclasses.replace(
        still,
        rgb=StampedImage(still.rgb.stamp_ns, small_rgb),
        rgb_raw=None,
        depth=StampedImage(still.depth.stamp_ns, small_depth),
        camera_info=small_info,
    )
    with pytest.raises(ValueError, match="rgb_size"):
        store.add_pose(sid, other)
    assert_nothing_stored(store, sid, 1)


def test_add_pose_with_unaligned_depth_refused(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    still = make_still()
    small = cv2.resize(still.depth.image, (288, 162), interpolation=cv2.INTER_NEAREST)
    with pytest.raises(ValueError, match="aligned depth"):
        store.add_pose(sid, dataclasses.replace(still, depth=StampedImage(still.depth.stamp_ns, small)))
    assert_nothing_stored(store, sid)


def test_add_pose_with_camera_info_of_another_size_refused(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    still = make_still()
    with pytest.raises(ValueError, match="camera_info"):
        store.add_pose(sid, dataclasses.replace(still, camera_info=dataclasses.replace(factory_info(), width=640)))
    assert_nothing_stored(store, sid)


def test_add_pose_with_empty_depth_not_stored(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    still = make_still()
    empty = StampedImage(still.depth.stamp_ns, np.zeros_like(still.depth.image))
    with pytest.raises(ValueError, match="no valid pixel"):
        store.add_pose(sid, dataclasses.replace(still, depth=empty))
    assert_nothing_stored(store, sid)
    poses = store.state_dir / "sessions" / sid / "poses"
    assert not poses.exists() or not list(poses.iterdir())


def test_load_stills_round_trip(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    a, b = make_still(), make_still(2, seed=2, distance_m=0.4)
    store.add_pose(sid, a)
    store.add_pose(sid, b)
    first, second = store.load_stills(sid)
    assert (first.pose_id, second.pose_id) == ("001", "002")
    assert first.rgb.shape == a.rgb.image.shape and first.rgb.dtype == np.uint8
    assert np.array_equal(first.depth, a.depth.image) and np.array_equal(second.depth, b.depth.image)
    info = first.camera_info
    assert info is not None and info.width == RGB_SIZE[0] and info.distortion_model == "plumb_bob"
    assert np.array_equal(info.k, a.camera_info.k) and np.array_equal(info.p, a.camera_info.p)


def test_load_stills_without_camera_info(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, make_still())
    path = store.pose_dir(sid, pid) / "pose.json"
    meta = json.loads(path.read_text())
    del meta["camera_info"]
    path.write_text(json.dumps(meta))
    assert store.load_stills(sid)[0].camera_info is None


def test_load_stills_needs_an_intrinsics_session(store: SessionStore) -> None:
    sid = store.create_session(NOW, kind="head_arm")
    with pytest.raises(ValueError, match="intrinsics"):
        store.load_stills(sid)


def test_coverage_counts_corners_per_cell(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    empty = store.coverage(sid)
    assert set(empty) == {"rgb"}
    assert empty["rgb"]["cols"] == 4 and empty["rgb"]["rows"] == 3
    assert sum(map(sum, empty["rgb"]["counts"])) == 0
    still = make_still()
    store.add_pose(sid, still)
    cov = store.coverage(sid)
    assert set(cov) == {"rgb"}
    assert sum(map(sum, cov["rgb"]["counts"])) == len(still.detection.ids)
    assert len(cov["rgb"]["counts"]) == 3 and len(cov["rgb"]["counts"][0]) == 4
    first = still.detection.points[0]
    col, row = int(first[0] / RGB_SIZE[0] * 4), int(first[1] / RGB_SIZE[1] * 3)
    assert cov["rgb"]["counts"][row][col] >= 1


def test_coverage_drops_with_deleted_pose(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, make_still())
    store.delete_pose(sid, pid)
    assert sum(map(sum, store.coverage(sid)["rgb"]["counts"])) == 0


def test_results_ids_and_listing_newest_first(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    assert store.new_result_id(NOW) == "r-20261019-123045"
    for rid in ("r-20261019-123045", "r-20261019-130000"):
        store.result_dir(sid, rid).mkdir(parents=True)
        (store.result_dir(sid, rid) / "result.json").write_text(json.dumps({"id": rid}))
    store.result_dir(sid, "r-20261019-140000").mkdir(parents=True)
    assert [r["id"] for r in store.list_results(sid)] == ["r-20261019-130000", "r-20261019-123045"]


def test_thumbnail_path(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, make_still())
    assert store.thumbnail_path(sid, pid, "rgb").name == "rgb_thumb.jpg"
    assert store.thumbnail_path(sid, pid, "depth").name == "depth_thumb.jpg"
    with pytest.raises(InvalidId):
        store.thumbnail_path(sid, pid, "tof")
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
    sid = store.create_session(NOW)
    with pytest.raises(InvalidId):
        store.delete_pose(sid, pid)
    with pytest.raises(InvalidId):
        store.thumbnail_path(sid, pid, "rgb")


def test_invalid_result_id_rejected(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    with pytest.raises(InvalidId):
        store.result_dir(sid, "../../x")


def test_missing_session_not_found(store: SessionStore) -> None:
    with pytest.raises(NotFound):
        store.load_session("calib-20261019-000000")
    with pytest.raises(NotFound):
        store.add_pose("calib-20261019-000000", make_still())


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
    sid = store.create_session(NOW)
    stills = [make_still(i + 1, seed=i + 1) for i in range(4)]
    ids: list[str] = []
    threads = [threading.Thread(target=lambda s=s: ids.append(store.add_pose(sid, s))) for s in stills]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert sorted(ids) == ["001", "002", "003", "004"]


def read_pose_json(store: SessionStore, sid: str, pid: str) -> dict:
    return json.loads((store.state_dir / "sessions" / sid / "poses" / pid / "pose.json").read_text())


def test_add_pose_stores_hints(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, few_corner_still())
    hints = read_pose_json(store, sid, pid)["hints"]
    assert hints == ["few corners: 6 RGB - move the board closer / fill the frame"]
    assert store.pose_hints(sid, pid) == hints


def test_add_pose_of_a_good_pose_has_empty_hints(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    pid = store.add_pose(sid, make_still())
    assert read_pose_json(store, sid, pid)["hints"] == []
    assert store.pose_hints(sid, pid) == []


# head_arm sessions (design D36)

JOINTS = {name: 0.1 * i for i, name in enumerate((*HEAD_JOINTS, *ARM_JOINTS))}


def add_still(store: SessionStore, sid: str, **kwargs):
    pose = still_arm_pose(kwargs.pop("joints", JOINTS), **kwargs)
    return store.add_still_pose(
        sid, pose.rgb_jpeg, pose.rgb_image, pose.detection, pose.joints, pose.stamps, pose.arm_pose, pose.tier,
        pose.head_offset,
    )  # fmt: skip


@pytest.fixture
def arm_sid(store: SessionStore) -> str:
    return store.create_session(NOW, kind="head_arm")


def test_session_without_kind_is_a_tof_era_rgbd_session(store: SessionStore) -> None:
    sid = store.create_session(NOW)
    assert store.load_session(sid)["kind"] == "intrinsics"
    path = store.state_dir / "sessions" / sid / "session.json"
    data = json.loads(path.read_text())
    del data["kind"]
    path.write_text(json.dumps(data))
    assert store.load_session(sid)["kind"] == "rgbd"
    assert store.list_sessions()[0]["kind"] == "rgbd"
    with pytest.raises(ValueError, match="intrinsics"):
        store.add_pose(sid, make_still())


def test_head_arm_session_records_kind(store: SessionStore, arm_sid: str) -> None:
    data = json.loads((store.state_dir / "sessions" / arm_sid / "session.json").read_text())
    assert data["kind"] == "head_arm"
    assert store.list_sessions()[0]["kind"] == "head_arm"


def test_still_pose_layout(store: SessionStore, arm_sid: str) -> None:
    pose = still_arm_pose(JOINTS, arm_pose=3, tier=1, head_offset=2)
    pid = store.add_still_pose(
        arm_sid, pose.rgb_jpeg, pose.rgb_image, pose.detection, pose.joints, pose.stamps, 3, 1, 2
    )  # fmt: skip
    assert pid == "001"
    path = store.state_dir / "sessions" / arm_sid / "poses" / pid
    assert sorted(p.name for p in path.iterdir()) == ["pose.json", "rgb.jpg", "rgb_thumb.jpg"]
    assert (path / "rgb.jpg").read_bytes() == pose.rgb_jpeg
    meta = json.loads((path / "pose.json").read_text())
    assert set(meta) == {"stamps", "rgb", "joints", "arm_pose", "tier", "head_offset"}
    assert meta["joints"] == JOINTS and meta["stamps"] == pose.stamps
    assert (meta["arm_pose"], meta["tier"], meta["head_offset"]) == (3, 1, 2)
    assert meta["rgb"]["ids"] == [int(i) for i in pose.detection.ids]
    assert len(meta["rgb"]["points"]) == len(pose.detection.ids)
    assert store.load_session(arm_sid)["rgb_size"] == [576, 324]
    assert store.pose_hints(arm_sid, pid) == []


@pytest.mark.parametrize("kwargs", [{"tier": 3}, {"tier": -1}, {"arm_pose": -1}, {"head_offset": -1}])
def test_still_pose_refusals(store: SessionStore, arm_sid: str, kwargs: dict) -> None:
    for name in JOINTS:
        with pytest.raises(ValueError, match=name):
            add_still(store, arm_sid, joints={k: v for k, v in JOINTS.items() if k != name})
    with pytest.raises(ValueError):
        add_still(store, arm_sid, **kwargs)
    assert store.pose_ids(arm_sid) == []


def test_still_pose_refuses_another_image_size(store: SessionStore, arm_sid: str) -> None:
    add_still(store, arm_sid)
    pose = still_arm_pose(JOINTS)
    small = cv2.resize(pose.rgb_image, (288, 162))
    with pytest.raises(ValueError, match="size"):
        store.add_still_pose(arm_sid, pose.rgb_jpeg, small, pose.detection, pose.joints, pose.stamps, 0, 0, 0)
    assert store.pose_ids(arm_sid) == ["001"]


def test_wrong_kind_raises(store: SessionStore, arm_sid: str) -> None:
    with pytest.raises(ValueError):
        store.add_pose(arm_sid, make_still())
    check = store.create_session(NOW)
    with pytest.raises(ValueError):
        add_still(store, check)
    with pytest.raises(ValueError):
        store.load_arm_board_views(check)
    with pytest.raises(ValueError):
        store.captured_arm_poses(check)
    assert store.pose_ids(check) == []


def test_load_arm_board_views(store: SessionStore, arm_sid: str) -> None:
    other = {k: v + 1.0 for k, v in JOINTS.items()}
    add_still(store, arm_sid, arm_pose=2, tier=0, head_offset=0)
    add_still(store, arm_sid, joints=other, arm_pose=5, tier=2, head_offset=1)
    views = store.load_arm_board_views(arm_sid)
    assert [v.view_id for v in views] == ["001", "002"]
    assert [(v.arm_pose, v.tier, v.head_offset) for v in views] == [(2, 0, 0), (5, 2, 1)]
    assert views[0].joints == JOINTS and views[1].joints == other
    expected = still_arm_pose(JOINTS).detection
    assert np.array_equal(views[0].detection.ids, expected.ids)
    assert np.allclose(views[0].detection.points, expected.points)


def test_captured_arm_poses(store: SessionStore, arm_sid: str) -> None:
    assert store.captured_arm_poses(arm_sid) == set()
    add_still(store, arm_sid, arm_pose=4)
    add_still(store, arm_sid, arm_pose=4, head_offset=1)
    add_still(store, arm_sid, arm_pose=7)
    assert store.captured_arm_poses(arm_sid) == {4, 7}


def test_head_arm_coverage_has_only_rgb(store: SessionStore, arm_sid: str) -> None:
    add_still(store, arm_sid)
    coverage = store.coverage(arm_sid)
    assert set(coverage) == {"rgb"}
    assert sum(sum(row) for row in coverage["rgb"]["counts"]) > 0


def test_still_pose_depth_thumbnail_is_not_found(store: SessionStore, arm_sid: str) -> None:
    pid = add_still(store, arm_sid)
    with pytest.raises(NotFound):
        store.thumbnail_path(arm_sid, pid, "depth")
    assert store.thumbnail_path(arm_sid, pid, "rgb").is_file()
