"""Tests for web_ui.ground_recorder: the ground recording layout (superset of the 2026-10-22 files)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from web_ui import ground_recorder as gr
from web_ui.ground_recorder import GroundRecording

NS = 1_000_000_000
POSE = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
OLD_DTYPES = {
    "stamps_ns": np.int64,
    "offsets": np.int64,
    "points_mm": np.int16,
    "base_T_cloud": np.float64,
    "odom_T_base": np.float64,
    "imu": np.float64,
    "base_T_imu": np.float64,
    "head_js": np.float64,
}


def full_recording() -> GroundRecording:
    rec = GroundRecording("box", 10.0)
    pts = np.array([[1.0, 2.0, 3.0], [1.5004, -2.0, 0.0004]], dtype=np.float32)
    rec.add_cloud(NS, "tof_link", pts, POSE, POSE)
    rec.add_cloud(2 * NS, "tof_link", pts[:1], POSE, POSE)
    rec.set_imu_frame("imu_link", POSE)
    rec.add_imu(NS, (0, 0, 0, 1), (0, 0, 9.8), (0.1, 0.2, 0.3))
    rec.add_head_js(NS, 0.1, -0.2)
    rec.set_tof_info(np.eye(3).ravel(), np.zeros(5), 4, 3, "tof_optical")
    rec.add_tof_frame(NS, np.full((3, 4), 7, np.uint16), np.full((3, 4), 9, np.uint16), POSE)
    rec.set_rgb_info(np.eye(3).ravel(), np.zeros(5), 8, 6, "rgb_optical")
    rec.add_pair(NS, b"\xff\xd8abc", np.full((2, 3), 5, np.uint16), POSE, POSE)
    rec.add_pair(2 * NS, b"\xff\xd8de", np.full((2, 3), 6, np.uint16), POSE, POSE)
    rec.add_state_line(NS, '{"a": 1}')
    return rec


def load(rec: GroundRecording, tmp_path: Path) -> dict[str, np.ndarray]:
    path = tmp_path / "rec.npz"
    size = rec.write(path)
    assert size == path.stat().st_size
    with np.load(path, allow_pickle=False) as data:
        return {k: data[k] for k in data.files}


def test_round_trip_all_streams(tmp_path: Path) -> None:
    rec = full_recording()
    assert rec.counts()["cloud"] == 2
    a = load(rec, tmp_path)
    assert int(a[gr.KEY_LAYOUT_VERSION]) == gr.LAYOUT_VERSION == 2
    assert str(a[gr.KEY_NAME]) == "box"
    assert a[gr.KEY_STAMPS_NS].tolist() == [NS, 2 * NS]
    assert a[gr.KEY_OFFSETS].tolist() == [0, 2, 3]
    assert a[gr.KEY_POINTS_MM].tolist() == [[1000, 2000, 3000], [1500, -2000, 0], [1000, 2000, 3000]]
    assert str(a[gr.KEY_CLOUD_FRAME]) == "tof_link"
    assert a[gr.KEY_BASE_T_CLOUD].shape == (2, 7)
    assert a[gr.KEY_IMU].shape == (1, 11)
    assert a[gr.KEY_IMU][0].tolist() == [NS, 0, 0, 0, 1, 0, 0, 9.8, 0.1, 0.2, 0.3]
    assert str(a[gr.KEY_IMU_FRAME]) == "imu_link"
    assert a[gr.KEY_HEAD_JS].shape == (1, 3)
    assert a[gr.KEY_TOF_DEPTH_MM].shape == (1, 3, 4)
    assert a[gr.KEY_TOF_DEPTH_MM].dtype == np.uint16
    assert a[gr.KEY_TOF_AMPLITUDE][0, 0, 0] == 9
    assert a[gr.KEY_TOF_SIZE].tolist() == [4, 3]
    assert a[gr.KEY_TOF_K].shape == (9,)
    assert str(a[gr.KEY_TOF_FRAME]) == "tof_optical"
    assert a[gr.KEY_PAIR_JPEG].dtype == np.uint8
    assert a[gr.KEY_PAIR_JPEG_OFFSETS].tolist() == [0, 5, 9]
    assert bytes(a[gr.KEY_PAIR_JPEG][5:9]) == b"\xff\xd8de"
    assert a[gr.KEY_PAIR_DEPTH_MM].shape == (2, 2, 3)
    assert a[gr.KEY_PAIR_BASE_T_RGB].shape == (2, 7)
    assert a[gr.KEY_PAIR_ODOM_T_BASE].shape == (2, 7)
    assert a[gr.KEY_RGB_SIZE].tolist() == [8, 6]
    assert str(a[gr.KEY_RGB_FRAME]) == "rgb_optical"
    assert a[gr.KEY_STATE_JSON].tolist() == ['{"a": 1}']
    assert a[gr.KEY_STATE_STAMPS_NS].tolist() == [NS]


def test_points_out_of_int16_range_dropped_not_clipped(tmp_path: Path) -> None:
    rec = GroundRecording("r", 5.0)
    pts = np.array([[1, 1, 1], [40.0, 0, 0], [0, np.nan, 0], [0, 0, -40.0], [np.inf, 0, 0], [32.767, 0, 0]], np.float32)
    rec.add_cloud(NS, "f", pts, POSE, POSE)
    a = load(rec, tmp_path)
    assert a[gr.KEY_POINTS_MM].shape == (2, 3)
    assert a[gr.KEY_OFFSETS].tolist() == [0, 2]
    assert rec.dropped()["cloud"]["range"] == 4


def test_empty_streams_written_as_zero_length(tmp_path: Path) -> None:
    a = load(GroundRecording("empty", 5.0), tmp_path)
    assert a[gr.KEY_STAMPS_NS].shape == (0,)
    assert a[gr.KEY_OFFSETS].tolist() == [0]
    assert a[gr.KEY_POINTS_MM].shape == (0, 3)
    assert a[gr.KEY_BASE_T_CLOUD].shape == (0, 7)
    assert a[gr.KEY_IMU].shape == (0, 11)
    assert a[gr.KEY_HEAD_JS].shape == (0, 3)
    assert a[gr.KEY_TOF_DEPTH_MM].shape == (0, 0, 0)
    assert a[gr.KEY_TOF_DEPTH_MM].dtype == np.uint16
    assert a[gr.KEY_PAIR_JPEG].shape == (0,)
    assert a[gr.KEY_PAIR_JPEG_OFFSETS].tolist() == [0]
    assert a[gr.KEY_PAIR_DEPTH_MM].shape == (0, 0, 0)
    assert a[gr.KEY_STATE_JSON].shape == (0,)
    assert a[gr.KEY_TOF_SIZE].shape == (2,)
    assert a[gr.KEY_TOF_K].shape == (9,)
    assert all(v == 0 for v in GroundRecording("e", 1.0).counts().values())


def test_layout_superset_of_2026_10_22_keys(tmp_path: Path) -> None:
    a = load(full_recording(), tmp_path)
    for key, dtype in OLD_DTYPES.items():
        assert key in a, key
        assert a[key].dtype == dtype, key
    assert a["cloud_frame"].dtype.kind == "U"
    assert a["base_T_imu"].shape == (7,)


def test_mismatched_frame_size_dropped(tmp_path: Path) -> None:
    rec = full_recording()
    rec.add_tof_frame(2 * NS, np.zeros((5, 5), np.uint16), np.zeros((5, 5), np.uint16), POSE)
    rec.add_tof_frame(2 * NS, np.zeros((3, 4), np.uint16), np.zeros((3, 5), np.uint16), POSE)
    rec.add_pair(3 * NS, b"x", np.zeros((9, 9), np.uint16), POSE, POSE)
    assert rec.counts()["tof"] == 1
    assert rec.counts()["pair"] == 2
    assert rec.dropped()["tof"]["size"] == 2
    assert rec.dropped()["pair"]["size"] == 1
    a = load(rec, tmp_path)
    assert a[gr.KEY_TOF_DEPTH_MM].shape == (1, 3, 4)


def test_done_after_window() -> None:
    rec = GroundRecording("w", 2.0)
    assert not rec.done(100 * NS)
    assert rec.elapsed_s(100 * NS) == 0.0
    rec.add_imu(10 * NS, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
    assert rec.elapsed_s(11 * NS) == 1.0
    assert not rec.done(11 * NS)
    assert rec.done(12 * NS)
