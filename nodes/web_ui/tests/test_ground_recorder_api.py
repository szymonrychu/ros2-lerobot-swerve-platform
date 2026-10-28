"""Tests for the ground recorder API: routes (fake recorder bridge) and the bridge recorder (ROS doubles)."""

from __future__ import annotations

import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np
import pytest
from fastapi.testclient import TestClient
from tf2_ros import ExtrapolationException

from web_ui import bridge as bridge_mod
from web_ui.bridge import BridgeNode
from web_ui.config import AppConfig, GroundRecorderConfig
from web_ui.ground_recorder import (
    KEY_BASE_T_CLOUD,
    KEY_BASE_T_IMU,
    KEY_HEAD_JS,
    KEY_ODOM_T_BASE,
    KEY_PAIR_BASE_T_RGB,
    KEY_PAIR_DEPTH_MM,
    KEY_PAIR_JPEG,
    KEY_PAIR_ODOM_T_BASE,
    KEY_POINTS_MM,
    KEY_STATE_JSON,
    KEY_TOF_AMPLITUDE,
)
from web_ui.ground_session import MATCH_CAPACITY

from .test_bridge_on_demand import FakeRos, ros  # noqa: F401  (ros is a fixture)

# --- routes --------------------------------------------------------------------------------------------------------


class FakeRecorderBridge:
    """Stand-in bridge holding recordings as files in a directory."""

    def __init__(self, directory: Path) -> None:
        self.dir = directory
        self.running: str | None = None
        self.started: list[tuple[str, float]] = []

    def ground_recording_start(self, name: str, duration_s: float) -> bool:
        if self.running is not None:
            return False
        self.running = name
        self.started.append((name, duration_s))
        return True

    def ground_recording_info(self, name: str) -> dict[str, Any] | None:
        if name == self.running:
            return {"name": name, "bytes": 0, "done": False, "counts": {}, "dropped": {}}
        path = self.dir / f"{name}.npz"
        if not path.exists():
            return None
        return {"name": name, "bytes": path.stat().st_size, "done": True, "counts": {"cloud": 3}, "dropped": {}}

    def ground_recordings(self) -> list[dict[str, Any]]:
        names = ([self.running] if self.running else []) + sorted(p.stem for p in self.dir.glob("*.npz"))
        return [info for n in names if (info := self.ground_recording_info(n)) is not None]

    def ground_recording_path(self, name: str) -> Path:
        return self.dir / f"{name}.npz"

    def ground_recording_delete(self, name: str) -> None:
        (self.dir / f"{name}.npz").unlink()


def make_client(tmp_path: Path, enabled: bool = True) -> tuple[TestClient, FakeRecorderBridge]:
    from web_ui.server import build_app

    fake = FakeRecorderBridge(tmp_path)
    config = AppConfig(ground_recorder=GroundRecorderConfig(enabled=enabled, dir=str(tmp_path), max_duration_s=30))
    app = build_app(config=config, urdf_dir=tmp_path, static_dir=tmp_path / "none", bridge_node=fake)
    return TestClient(app), fake


def test_start_list_download_delete(tmp_path: Path) -> None:
    client, fake = make_client(tmp_path)
    reply = client.post("/api/ground/recordings", json={"name": "box_1", "duration_s": 12})
    assert reply.status_code == 202
    assert reply.json() == {"name": "box_1", "duration_s": 12}
    assert fake.started == [("box_1", 12)]
    assert client.get("/api/ground/recordings/box_1").status_code == 409
    listed = client.get("/api/ground/recordings").json()
    assert listed == [{"name": "box_1", "bytes": 0, "done": False, "counts": {}, "dropped": {}}]
    fake.running = None
    (tmp_path / "box_1.npz").write_bytes(b"npzdata")
    listed = client.get("/api/ground/recordings").json()
    assert listed == [{"name": "box_1", "bytes": 7, "done": True, "counts": {"cloud": 3}, "dropped": {}}]
    download = client.get("/api/ground/recordings/box_1")
    assert download.status_code == 200
    assert download.headers["content-type"] == "application/octet-stream"
    assert download.content == b"npzdata"
    assert client.get("/api/ground/recordings/nope").status_code == 404
    assert client.delete("/api/ground/recordings/nope").status_code == 404
    assert client.delete("/api/ground/recordings/box_1").status_code == 200
    assert not (tmp_path / "box_1.npz").exists()
    assert client.get("/api/ground/recordings").json() == []


def test_second_start_conflicts(tmp_path: Path) -> None:
    client, fake = make_client(tmp_path)
    assert client.post("/api/ground/recordings", json={"name": "a", "duration_s": 5}).status_code == 202
    assert client.post("/api/ground/recordings", json={"name": "b", "duration_s": 5}).status_code == 409
    assert fake.started == [("a", 5)]


def test_disabled_recorder_is_404(tmp_path: Path) -> None:
    client, fake = make_client(tmp_path, enabled=False)
    assert client.post("/api/ground/recordings", json={"name": "a", "duration_s": 5}).status_code == 404
    assert client.get("/api/ground/recordings").status_code == 404
    assert client.get("/api/ground/recordings/a").status_code == 404
    assert client.delete("/api/ground/recordings/a").status_code == 404
    assert fake.started == []


@pytest.mark.parametrize(
    "body",
    [
        {"name": "Bad Name", "duration_s": 5},
        {"name": "", "duration_s": 5},
        {"name": "x" * 33, "duration_s": 5},
        {"name": "../etc", "duration_s": 5},
        {"name": "a", "duration_s": 0.5},
        {"name": "a", "duration_s": 31},
        {"name": "a", "duration_s": "long"},
        {"name": "a"},
        {},
    ],
)
def test_bad_name_or_duration_is_400(tmp_path: Path, body: dict[str, Any]) -> None:
    client, fake = make_client(tmp_path)
    assert client.post("/api/ground/recordings", json=body).status_code == 400
    assert fake.started == []


# --- bridge recorder -----------------------------------------------------------------------------------------------


class Clock:
    """Controllable replacement for time.monotonic."""

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def pose_tf(x: float) -> SimpleNamespace:
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=0.0, z=0.5),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )


def header(stamp_ns: int, frame: str) -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id=frame)


def cloud_msg(stamp_ns: int, points: np.ndarray, frame: str = "tof_optical") -> SimpleNamespace:
    fields = [SimpleNamespace(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("xyz")]
    pts = np.asarray(points, dtype="<f4")
    return SimpleNamespace(
        header=header(stamp_ns, frame),
        height=1,
        width=len(pts),
        fields=fields,
        is_bigendian=False,
        point_step=12,
        row_step=12 * len(pts),
        data=pts.tobytes(),
        is_dense=True,
    )


def image_msg(stamp_ns: int, array: np.ndarray, encoding: str, frame: str) -> SimpleNamespace:
    arr = np.asarray(array, dtype="<u2")
    return SimpleNamespace(
        header=header(stamp_ns, frame),
        height=arr.shape[0],
        width=arr.shape[1],
        encoding=encoding,
        is_bigendian=False,
        step=arr.shape[1] * 2,
        data=arr.tobytes(),
    )


STAMP = 5_000_000_000
POINTS = np.array([[1.0, 0.0, 0.2], [2.0, 0.5, 0.0], [1.5, -0.5, 0.1]])


@pytest.fixture
def clock() -> Any:
    clk = Clock()
    with patch.object(bridge_mod.time, "monotonic", clk):
        yield clk


def recorder_node(tmp_path: Path, **overrides: Any) -> BridgeNode:
    cfg = GroundRecorderConfig(enabled=True, dir=str(tmp_path), tf_timeout_s=0.5, **overrides)
    node = BridgeNode(
        topics=[], allowed_publish_topics=set(), robot_pose_frames=("map", "base_link"), ground_recorder=cfg
    )
    return node


def set_tf(node: BridgeNode, available: set[tuple[str, str]]) -> None:
    def lookup(target: str, source: str, _time: Any) -> SimpleNamespace:
        if (target, source) not in available:
            raise ExtrapolationException("not yet")
        return pose_tf(1.0 if target == "base_link" else 7.0)

    node._tf_buffer.lookup_transform.side_effect = lookup


def start(node: BridgeNode, name: str = "scene", duration_s: float = 5.0) -> None:
    assert node.ground_recording_start(name, duration_s) is True
    node.apply_demand()


def test_message_without_tf_dropped_and_counted(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, set())
    start(node)
    ros.callback("/tof/points_filtered")(cloud_msg(STAMP, POINTS))
    clock.now += 0.4
    node.apply_demand()
    assert node.ground_recording_info("scene")["counts"]["cloud"] == 0  # still waiting for TF
    clock.now += 0.2
    node.apply_demand()
    info = node.ground_recording_info("scene")
    assert info["counts"]["cloud"] == 0
    assert info["dropped"]["cloud"] == {"tf": 1}


def test_message_with_tf_recorded_with_its_pose(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, set())
    start(node)
    ros.callback("/tof/points_filtered")(cloud_msg(STAMP, POINTS))
    set_tf(node, {("base_link", "tof_optical"), ("odom", "base_link")})  # TF arrives within the timeout
    clock.now += 0.2
    node.apply_demand()
    ros.callback("/tof/depth/image_raw")(image_msg(STAMP, np.full((2, 3), 800), "16UC1", "tof_optical"))
    ros.callback("/tof/amplitude/image_raw")(image_msg(STAMP, np.full((2, 3), 90), "mono16", "tof_optical"))
    clock.now += 10.0
    node.apply_demand()  # window over
    node.join_ground_recording(5)
    path = tmp_path / "scene.npz"
    assert path.exists()
    data = np.load(path)
    assert data[KEY_BASE_T_CLOUD].tolist() == [[1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]]
    assert data[KEY_ODOM_T_BASE].tolist() == [[7.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]]
    assert data[KEY_POINTS_MM].tolist() == (POINTS * 1000).astype(int).tolist()
    assert data[KEY_TOF_AMPLITUDE].shape == (1, 2, 3)
    assert node.ground_recording_info("scene")["done"] is True


def test_subscriptions_destroyed_after_window(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, set())
    assert "/tof/points_filtered" not in ros.live_topics()
    start(node, duration_s=5.0)
    live = set(ros.live_topics())
    cfg = node._ground_cfg
    expected = {
        cfg.cloud_topic,
        cfg.tof_depth_topic,
        cfg.tof_amplitude_topic,
        cfg.tof_info_topic,
        cfg.pair_depth_topic,
        cfg.pair_info_topic,
        cfg.pair_rgb_topic,
        cfg.imu_topic,
        cfg.head_js_topic,
        cfg.state_topic,
    }
    assert expected <= live
    assert node.ground_recording_start("other", 5.0) is False  # one at a time
    clock.now += 4.9
    node.apply_demand()
    assert expected <= set(ros.live_topics())
    clock.now += 0.2
    node.apply_demand()
    assert expected.isdisjoint(ros.live_topics())
    node.join_ground_recording(5)
    assert (tmp_path / "scene.npz").exists()
    assert node.ground_recording_start("other", 5.0) is True  # free again
    assert node.publishers_ == {}  # the recorder publishes nothing


def test_oldest_files_beyond_max_files_are_deleted(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path, max_files=2)
    set_tf(node, set())
    for i, name in enumerate(("a", "b", "c")):
        start(node, name, 1.0)
        clock.now += 2.0
        node.apply_demand()
        node.join_ground_recording(5)
        assert (tmp_path / f"{name}.npz").exists()
        os.utime(tmp_path / f"{name}.npz", (1000 + i, 1000 + i))
    assert sorted(p.name for p in tmp_path.glob("*.npz")) == ["b.npz", "c.npz"]
    assert [r["name"] for r in node.ground_recordings()] == ["b", "c"]
    assert node.ground_recording_delete("b") is True
    assert node.ground_recording_info("b") is None


def lookup_stamps(node: BridgeNode) -> set[int]:
    """Stamps (ns) passed as the time argument of every TF lookup."""
    return {call.args[2].nanoseconds for call in node._tf_buffer.lookup_transform.call_args_list}


def test_tf_is_looked_up_at_the_message_stamp(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, {("base_link", "tof_optical"), ("odom", "base_link")})
    start(node)
    ros.callback("/tof/points_filtered")(cloud_msg(STAMP, POINTS))
    ros.callback("/tof/points_filtered")(cloud_msg(STAMP + 7, POINTS))
    assert node._tf_buffer.lookup_transform.call_count == 4
    assert lookup_stamps(node) == {STAMP, STAMP + 7}  # never Time(0) / latest


def finish(node: BridgeNode, clock: Clock, tmp_path: Path) -> Any:
    clock.now += 10.0
    node.apply_demand()
    node.join_ground_recording(5)
    return np.load(tmp_path / "scene.npz")


def test_rgb_and_upsampled_depth_pair_recorded_by_stamp(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, {("base_link", "rgb_optical"), ("odom", "base_link")})
    start(node)
    ros.callback("/head_rgbd/depth_upsampled/image_raw")(image_msg(STAMP, np.full((2, 3), 900), "16UC1", "rgb_optical"))
    assert node.ground_recording_info("scene")["counts"]["pair"] == 0  # half a frame is not a sample
    ros.callback("/head_rgbd/rgb/image_raw/compressed")(
        SimpleNamespace(header=header(STAMP, "rgb_optical"), format="jpeg", data=b"\xff\xd8jpeg")
    )
    assert node.ground_recording_info("scene")["counts"]["pair"] == 1
    assert lookup_stamps(node) == {STAMP}
    data = finish(node, clock, tmp_path)
    assert bytes(data[KEY_PAIR_JPEG]) == b"\xff\xd8jpeg"
    assert data[KEY_PAIR_DEPTH_MM].shape == (1, 2, 3)
    assert data[KEY_PAIR_BASE_T_RGB].tolist() == [[1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]]
    assert data[KEY_PAIR_ODOM_T_BASE].tolist() == [[7.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]]


def test_pair_without_tf_dropped_and_unmatched_half_counted(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, set())
    start(node)
    ros.callback("/head_rgbd/depth_upsampled/image_raw")(image_msg(STAMP, np.full((2, 3), 900), "16UC1", "rgb_optical"))
    ros.callback("/head_rgbd/rgb/image_raw/compressed")(
        SimpleNamespace(header=header(STAMP, "rgb_optical"), format="jpeg", data=b"j")
    )
    ros.callback("/head_rgbd/rgb/image_raw/compressed")(
        SimpleNamespace(header=header(STAMP + 1, "rgb_optical"), format="jpeg", data=b"lonely")
    )
    clock.now += 1.0
    node.apply_demand()
    clock.now += 10.0
    node.apply_demand()
    node.join_ground_recording(5)
    info = node.ground_recording_info("scene")
    assert info["counts"]["pair"] == 0
    assert info["dropped"]["pair"] == {"tf": 1, "unmatched": 1}


def test_unmatched_halves_beyond_capacity_are_dropped(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, set())
    start(node)
    for i in range(MATCH_CAPACITY + 3):
        ros.callback("/tof/amplitude/image_raw")(image_msg(STAMP + i, np.full((2, 3), 1), "mono16", "tof_optical"))
    assert node.ground_recording_info("scene")["dropped"]["depth"] == {"unmatched": 3}


def test_undecodable_messages_are_dropped_and_counted(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, {("base_link", "tof_optical"), ("odom", "base_link")})
    start(node)
    ros.callback("/tof/depth/image_raw")(image_msg(STAMP, np.full((2, 3), 1), "rgb8", "tof_optical"))
    bad_cloud = cloud_msg(STAMP, POINTS)
    bad_cloud.fields = []
    ros.callback("/tof/points_filtered")(bad_cloud)
    info = node.ground_recording_info("scene")
    assert info["dropped"]["depth"] == {"decode": 1}
    assert info["dropped"]["cloud"] == {"decode": 1}
    assert info["counts"]["cloud"] == 0


def test_imu_head_and_state_are_recorded(ros: FakeRos, clock: Clock, tmp_path: Path) -> None:  # noqa: F811
    node = recorder_node(tmp_path)
    set_tf(node, {("base_link", "imu_link")})
    start(node)
    vec = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)  # noqa: E731
    imu = SimpleNamespace(
        header=header(STAMP, "imu_link"),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        linear_acceleration=vec(0.0, 0.0, 9.8),
        angular_velocity=vec(0.0, 0.0, 0.1),
    )
    ros.callback("/imu/data")(imu)
    ros.callback("/imu/data")(imu)
    assert lookup_stamps(node) == {STAMP}
    assert node._tf_buffer.lookup_transform.call_count == 1  # the mount is looked up once
    ros.callback("/head/joint_states")(
        SimpleNamespace(header=header(STAMP, ""), name=["head_tilt", "head_pan"], position=[0.2, -0.1])
    )
    ros.callback("/head/joint_states")(SimpleNamespace(header=header(STAMP, ""), name=["other"], position=[1.0]))
    ros.callback("/ground/state")(SimpleNamespace(data='{"phase": "ok"}'))
    info = node.ground_recording_info("scene")
    assert info["counts"]["imu"] == 2
    assert info["counts"]["head"] == 1
    assert info["counts"]["state"] == 1
    assert info["dropped"]["head"] == {"joints": 1}
    data = finish(node, clock, tmp_path)
    assert data[KEY_BASE_T_IMU].tolist() == [1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
    assert np.allclose(data[KEY_HEAD_JS][0][-2:], [-0.1, 0.2])
    assert '"phase"' in str(data[KEY_STATE_JSON][0])
