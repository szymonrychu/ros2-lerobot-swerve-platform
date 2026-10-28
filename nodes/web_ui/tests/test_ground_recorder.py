"""Tests for web_ui.ground_recorder: the ground recording layout (superset of the 2026-10-22 files)."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import yaml

from web_ui import ground_recorder as gr
from web_ui.config import GroundRecorderConfig
from web_ui.ground_recorder import GroundRecording
from web_ui.ground_session import GroundSession

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
    rec.add_cloud(NS, "d435_depth_optical_frame", pts, POSE, POSE)
    rec.add_cloud(2 * NS, "d435_depth_optical_frame", pts[:1], POSE, POSE)
    rec.set_imu_frame("imu_link", POSE)
    rec.add_imu(NS, (0, 0, 0, 1), (0, 0, 9.8), (0.1, 0.2, 0.3))
    rec.add_head_js(NS, 0.1, -0.2)
    rec.set_depth_info(np.eye(3).ravel(), np.zeros(5), 4, 3, "d435_depth_optical_frame")
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
    assert int(a[gr.KEY_LAYOUT_VERSION]) == gr.LAYOUT_VERSION == 3
    assert str(a[gr.KEY_NAME]) == "box"
    assert a[gr.KEY_STAMPS_NS].tolist() == [NS, 2 * NS]
    assert a[gr.KEY_OFFSETS].tolist() == [0, 2, 3]
    assert a[gr.KEY_POINTS_MM].tolist() == [[1000, 2000, 3000], [1500, -2000, 0], [1000, 2000, 3000]]
    assert str(a[gr.KEY_CLOUD_FRAME]) == "d435_depth_optical_frame"
    assert a[gr.KEY_BASE_T_CLOUD].shape == (2, 7)
    assert a[gr.KEY_IMU].shape == (1, 11)
    assert a[gr.KEY_IMU][0].tolist() == [NS, 0, 0, 0, 1, 0, 0, 9.8, 0.1, 0.2, 0.3]
    assert str(a[gr.KEY_IMU_FRAME]) == "imu_link"
    assert a[gr.KEY_HEAD_JS].shape == (1, 3)
    assert a[gr.KEY_DEPTH_SIZE].tolist() == [4, 3]
    assert a[gr.KEY_DEPTH_K].shape == (9,)
    assert str(a[gr.KEY_DEPTH_FRAME]) == "d435_depth_optical_frame"
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
    assert a[gr.KEY_PAIR_JPEG].shape == (0,)
    assert a[gr.KEY_PAIR_JPEG_OFFSETS].tolist() == [0]
    assert a[gr.KEY_PAIR_DEPTH_MM].shape == (0, 0, 0)
    assert a[gr.KEY_STATE_JSON].shape == (0,)
    assert a[gr.KEY_DEPTH_SIZE].tolist() == [0, 0]
    assert a[gr.KEY_DEPTH_K].shape == (9,)
    assert a[gr.KEY_DEPTH_D].shape == (0,)
    assert all(v == 0 for v in GroundRecording("e", 1.0).counts().values())


def test_layout_superset_of_2026_10_22_keys(tmp_path: Path) -> None:
    a = load(full_recording(), tmp_path)
    for key, dtype in OLD_DTYPES.items():
        assert key in a, key
        assert a[key].dtype == dtype, key
    assert a["cloud_frame"].dtype.kind == "U"
    assert a["base_T_imu"].shape == (7,)


def test_mismatched_pair_size_dropped(tmp_path: Path) -> None:
    rec = full_recording()
    rec.add_pair(3 * NS, b"x", np.zeros((9, 9), np.uint16), POSE, POSE)
    assert rec.counts()["pair"] == 2
    assert rec.dropped()["pair"]["size"] == 1
    a = load(rec, tmp_path)
    assert a[gr.KEY_PAIR_DEPTH_MM].shape == (2, 2, 3)


def test_done_after_window() -> None:
    rec = GroundRecording("w", 2.0)
    assert not rec.done(100 * NS)
    assert rec.elapsed_s(100 * NS) == 0.0
    rec.add_imu(10 * NS, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
    assert rec.elapsed_s(11 * NS) == 1.0
    assert not rec.done(11 * NS)
    assert rec.done(12 * NS)


D435_TOPICS = {
    "cloud_topic": "/head/d435/points_filtered",
    "depth_info_topic": "/head/d435/depth/camera_info",
    "pair_depth_topic": "/head_rgbd/depth/image_raw",
    "pair_info_topic": "/head_rgbd/depth/camera_info",
    "pair_rgb_topic": "/head_rgbd/rgb/image_raw/compressed",
}
DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "default.yaml"
D435_K = [385.2, 0.0, 321.4, 0.0, 385.2, 181.7, 0.0, 0.0, 1.0]
D435_D = [0.0, 0.0, 0.0, 0.0, 0.0]


def test_config_defaults_are_the_d435_topics() -> None:
    cfg = GroundRecorderConfig()
    assert {field: getattr(cfg, field) for field in D435_TOPICS} == D435_TOPICS
    assert not [field for field in GroundRecorderConfig.model_fields if "tof" in field]
    assert "D435" in (GroundRecorderConfig.__doc__ or "")
    shipped = yaml.safe_load(DEFAULT_YAML.read_text())["ground_recorder"]
    assert GroundRecorderConfig.model_validate(shipped) == cfg.model_copy(update={"enabled": shipped["enabled"]})


def header(stamp_ns: int, frame: str) -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // NS, nanosec=stamp_ns % NS), frame_id=frame)


def cloud_msg(stamp_ns: int) -> SimpleNamespace:
    fields = [SimpleNamespace(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("xyz")]
    pts = np.array([[1.0, 0.0, 0.2], [2.0, 0.5, 0.0]], dtype="<f4")
    return SimpleNamespace(
        header=header(stamp_ns, "d435_depth_optical_frame"),
        height=1,
        width=len(pts),
        fields=fields,
        is_bigendian=False,
        point_step=12,
        row_step=12 * len(pts),
        data=pts.tobytes(),
        is_dense=True,
    )


def imu_msg(stamp_ns: int) -> SimpleNamespace:
    vec = SimpleNamespace(x=0.0, y=0.0, z=9.8)
    quat = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    return SimpleNamespace(
        header=header(stamp_ns, "imu_link"), orientation=quat, linear_acceleration=vec, angular_velocity=vec
    )


def record(tmp_path: Path, messages: list[tuple[str, Any]]) -> dict[str, np.ndarray]:
    """Run one GroundSession over the default topics, feed messages by topic, return the written npz arrays."""
    callbacks: dict[str, Any] = {}

    def subscribe(_cls: type, topic: str, callback: Any, _qos: Any) -> str:
        callbacks[topic] = callback
        return topic

    session = GroundSession(
        GroundRecorderConfig(enabled=True, dir=str(tmp_path)),
        "scene",
        5.0,
        subscribe,
        lambda _s: None,
        lambda *_a: POSE,
        lambda: None,
    )
    session.open()
    for topic, msg in messages:
        callbacks[topic](msg)
    session.close()
    assert session.writer is not None
    session.writer.join(5)
    with np.load(tmp_path / "scene.npz", allow_pickle=False) as data:
        return {k: data[k] for k in data.files}


def test_layout3_records_the_depth_model(tmp_path: Path) -> None:
    info = SimpleNamespace(header=header(NS, "d435_depth_optical_frame"), k=D435_K, d=D435_D, width=640, height=360)
    a = record(tmp_path, [(D435_TOPICS["depth_info_topic"], info), (D435_TOPICS["cloud_topic"], cloud_msg(NS))])
    assert int(a[gr.KEY_LAYOUT_VERSION]) == 3
    assert a[gr.KEY_DEPTH_K].tolist() == D435_K
    assert a[gr.KEY_DEPTH_D].tolist() == D435_D
    assert a[gr.KEY_DEPTH_SIZE].tolist() == [640, 360]
    assert str(a[gr.KEY_DEPTH_FRAME]) == "d435_depth_optical_frame"
    assert (gr.KEY_DEPTH_K, gr.KEY_DEPTH_D, gr.KEY_DEPTH_SIZE, gr.KEY_DEPTH_FRAME) == (
        "depth_K",
        "depth_D",
        "depth_size",
        "depth_frame",
    )
    assert [k for k in a if k.startswith("tof")] == []
    assert a[gr.KEY_OFFSETS].tolist() == [0, 2]


def test_recording_without_depth_info_keeps_the_streams_and_a_zero_depth_size(tmp_path: Path) -> None:
    a = record(tmp_path, [(D435_TOPICS["cloud_topic"], cloud_msg(NS)), ("/imu/data", imu_msg(NS))])
    assert a[gr.KEY_STAMPS_NS].tolist() == [NS]
    assert a[gr.KEY_IMU].shape == (1, 11)
    assert a[gr.KEY_DEPTH_SIZE].tolist() == [0, 0]
    assert str(a[gr.KEY_DEPTH_FRAME]) == ""


def test_streams_have_no_tof_frames() -> None:
    assert set(GroundRecording("s", 1.0).counts()) == {"cloud", "imu", "head", "pair", "state"}
    assert set(GroundRecording("s", 1.0).dropped()) == {"cloud", "imu", "head", "pair", "state"}
    assert not hasattr(GroundRecording, "add_tof_frame")
