"""Tests for the live capture controller: subscriptions and QoS, their lifetime, decoding, pairing and stored stills."""

import json
import threading
from collections.abc import Callable
from datetime import UTC, datetime, timedelta
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pytest
from rclpy.qos import HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from head_rgbd_calibration.capture import (
    MOVING,
    NO_BOARD,
    NO_CAMERA_INFO,
    WAITING,
    Busy,
    CaptureController,
    CaptureError,
    StillOutcome,
    captured_status,
    decode_mono16,
    depth_change_fraction,
)
from head_rgbd_calibration.config import NodeConfig, StillConfig
from head_rgbd_calibration.store import NotFound, SessionStore
from tests.pose_factory import RGB_SIZE, make_still, synthetic_view

NOW = datetime(2026, 10, 19, 12, 30, 45, tzinfo=UTC)
NS_PER_S = 1_000_000_000
T0_NS = 1_700_000_000 * NS_PER_S
FRAME_PERIOD_NS = 66_666_667  # D435 colour and aligned depth at 15 Hz, one stamp per pair
NEAR_M = 0.33
STEPS = 8
RGB_TOPIC = "/head/d435/color/image_raw/compressed"
DEPTH_TOPIC = "/head/d435/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/head/d435/color/camera_info"
FACTORY_K = [432.0, 0.0, 288.0, 0.0, 432.0, 162.0, 0.0, 0.0, 1.0]


class FakeSubscription:
    """A subscription created by FakeNode."""

    def __init__(self, msg_type: type, topic: str, callback: Callable[[Any], None], qos: Any) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class FakeTimer:
    """A timer created by FakeNode."""

    def __init__(self, period_s: float, callback: Callable[[], None]) -> None:
        self.period_s = period_s
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class FakeLogger:
    """Records log calls."""

    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, msg: str, **_: Any) -> None:
        self.records.append(("info", msg))

    def warning(self, msg: str, **_: Any) -> None:
        self.records.append(("warning", msg))

    def error(self, msg: str, **_: Any) -> None:
        self.records.append(("error", msg))


class FakeNode:
    """Records create/destroy calls of subscriptions, timers and publishers; can fail the n-th subscription."""

    def __init__(self, fail_subscription_at: int | None = None, fail_timer: bool = False) -> None:
        self.subscriptions: list[FakeSubscription] = []
        self.destroyed: list[FakeSubscription] = []
        self.timers: list[FakeTimer] = []
        self.destroyed_timers: list[FakeTimer] = []
        self.publishers: list[Any] = []
        self.created = 0
        self.fail_subscription_at = fail_subscription_at
        self.fail_timer = fail_timer
        self.logger = FakeLogger()

    def create_subscription(self, msg_type: type, topic: str, callback: Callable[[Any], None], qos: Any) -> Any:
        self.created += 1
        if self.fail_subscription_at == self.created:
            raise RuntimeError("subscription failed")
        sub = FakeSubscription(msg_type, topic, callback, qos)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, sub: FakeSubscription) -> bool:
        self.subscriptions.remove(sub)
        self.destroyed.append(sub)
        return True

    def create_timer(self, period_s: float, callback: Callable[[], None]) -> FakeTimer:
        if self.fail_timer:
            raise RuntimeError("timer failed")
        timer = FakeTimer(period_s, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer: FakeTimer) -> bool:
        self.timers.remove(timer)
        self.destroyed_timers.append(timer)
        return True

    def create_publisher(self, *args: Any, **kwargs: Any) -> Any:
        self.publishers.append((args, kwargs))
        return SimpleNamespace(publish=lambda _msg: None)

    def get_logger(self) -> FakeLogger:
        return self.logger

    def subscription(self, topic: str) -> FakeSubscription:
        return next(s for s in self.subscriptions if s.topic == topic)


def header(stamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // NS_PER_S, nanosec=stamp_ns % NS_PER_S))


def image_msg(
    stamp_ns: int, image: np.ndarray, encoding: str = "mono16", pad: int = 0, bigendian: bool = False
) -> SimpleNamespace:
    """A sensor_msgs/Image-like message of a uint16 image, rows padded by pad bytes."""
    height, width = image.shape
    rows = image.astype(">u2" if bigendian else "<u2").view(np.uint8).reshape(height, width * 2)
    rows = np.hstack([rows, np.full((height, pad), 0xAB, dtype=np.uint8)])
    return SimpleNamespace(
        header=header(stamp_ns),
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=int(bigendian),
        step=width * 2 + pad,
        data=rows.tobytes(),
    )


def compressed_msg(stamp_ns: int, data: bytes) -> SimpleNamespace:
    return SimpleNamespace(header=header(stamp_ns), format="jpeg", data=data)


def camera_info_msg(width: int = RGB_SIZE[0]) -> SimpleNamespace:
    """The factory colour camera_info of the synthetic camera, as the D435 publishes it."""
    return SimpleNamespace(
        header=header(T0_NS),
        width=width,
        height=RGB_SIZE[1],
        distortion_model="plumb_bob",
        d=[0.0, 0.0, 0.0, 0.0, 0.0],
        k=FACTORY_K,
        p=[432.0, 0.0, 288.0, 0.0, 0.0, 432.0, 162.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


def make_cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig.model_validate(
        {"state_dir": str(tmp_path / "state"), "max_capture_s": 120.0, "selection": {"skip_start_s": 0.0}}
    )


def make_controller(tmp_path: Path, node: FakeNode | None = None) -> CaptureController:
    cfg = make_cfg(tmp_path)
    return CaptureController(node or FakeNode(), SessionStore(Path(cfg.state_dir), cfg), cfg, lambda: NOW)


def jpeg(image: np.ndarray, quality: int = 95) -> bytes:
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def feed_still_board(
    node: FakeNode,
    steps: int = STEPS,
    rgb_data: bytes | None = None,
    depth: np.ndarray | None = None,
    info: SimpleNamespace | None = None,
    depth_offset_ns: int = 0,
    send_info: bool = True,
    start_ns: int = T0_NS,
) -> None:
    """Feed a still board as the D435 delivers it: camera_info, colour and aligned depth with one stamp per frame."""
    rgb, aligned = synthetic_view(NEAR_M, 1)
    data = rgb_data if rgb_data is not None else jpeg(rgb)
    frame = aligned if depth is None else depth
    for step in range(steps):
        ns = start_ns + step * FRAME_PERIOD_NS
        if send_info:
            node.subscription(INFO_TOPIC).callback(info or camera_info_msg())
        node.subscription(RGB_TOPIC).callback(compressed_msg(ns, data))
        node.subscription(DEPTH_TOPIC).callback(image_msg(ns + depth_offset_ns, frame, encoding="16UC1"))


# subscriptions and QoS


def test_d435_subscriptions_and_qos(tmp_path: Path) -> None:
    node = FakeNode()
    make_controller(tmp_path, node).start(None)
    assert sorted(s.topic for s in node.subscriptions) == sorted([RGB_TOPIC, DEPTH_TOPIC, INFO_TOPIC])
    depth, info, rgb = (node.subscription(t) for t in (DEPTH_TOPIC, INFO_TOPIC, RGB_TOPIC))
    assert depth.msg_type is Image
    assert depth.qos.reliability == ReliabilityPolicy.RELIABLE
    assert depth.qos.history == HistoryPolicy.KEEP_LAST
    assert depth.qos.depth == 10
    assert info.msg_type is CameraInfo
    assert info.qos.reliability == ReliabilityPolicy.RELIABLE
    assert info.qos.depth == 1
    assert rgb.msg_type is CompressedImage
    assert rgb.qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert rgb.qos.history == HistoryPolicy.KEEP_LAST
    assert rgb.qos.depth == 5


def test_nothing_is_subscribed_before_start(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    assert node.subscriptions == [] and node.timers == []
    assert controller.state() == {
        "running": False,
        "session_id": None,
        "status": "idle",
        "pose_count": 0,
        "started_at": None,
        "stops_at": None,
    }


def test_stop_destroys_all_subscriptions(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    timer = node.timers[0]
    state = controller.stop()
    assert node.subscriptions == []
    assert len(node.destroyed) == 3
    assert node.timers == [] and node.destroyed_timers == [timer] and timer.cancelled
    assert state["running"] is False
    assert controller.stop()["running"] is False  # a second stop is harmless
    assert len(node.destroyed) == 3


def test_timeout_stops_capture(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    state = controller.start(None)
    timer = node.timers[0]
    assert timer.period_s == 120.0
    assert state["started_at"] == NOW.isoformat()
    assert state["stops_at"] == (NOW + timedelta(seconds=120.0)).isoformat()
    timer.callback()
    assert node.subscriptions == [] and node.timers == []
    state = controller.state()
    assert state["running"] is False
    assert "max_capture_s" in state["status"]


def test_failed_start_leaves_nothing_subscribed(tmp_path: Path) -> None:
    for node in (FakeNode(fail_subscription_at=3), FakeNode(fail_timer=True)):
        controller = make_controller(tmp_path, node)
        with pytest.raises(RuntimeError):
            controller.start(None)
        assert node.subscriptions == [] and node.timers == []
        assert controller.state()["running"] is False
        assert not (tmp_path / "state" / "sessions").exists() or not any((tmp_path / "state" / "sessions").iterdir())
    # the controller is not left busy: a later start works
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    assert controller.start(None)["running"] is True


def test_start_while_running_is_busy(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    with pytest.raises(Busy):
        controller.start(None)
    assert len(node.subscriptions) == 3


def test_unknown_session_is_not_found(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    with pytest.raises(NotFound):
        controller.start("calib-20260101-000000")
    assert node.created == 0


def test_reopening_a_head_arm_session_for_the_check_is_refused(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.store.create_session(NOW, "head_arm")
    with pytest.raises(CaptureError, match="head_arm"):
        controller.start(sid)
    assert node.created == 0


def test_capture_publishes_nothing(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    feed_still_board(node)
    controller.stop()
    assert node.publishers == []


# pairing, still selection and stored stills


def test_still_stores_colour_depth_and_camera_info(tmp_path: Path) -> None:
    """An exact-stamp colour + aligned depth still is stored with the factory CameraInfo and no amplitude."""
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    assert controller.state()["status"] == WAITING
    feed_still_board(node)
    store = controller.store
    assert store.pose_ids(sid) == ["001"]
    pose_dir = Path(controller.cfg.state_dir) / "sessions" / sid / "poses" / "001"
    assert {p.name for p in pose_dir.iterdir()} == {
        "rgb.jpg",
        "depth.png",
        "pose.json",
        "rgb_thumb.jpg",
        "depth_thumb.jpg",
    }
    meta = json.loads((pose_dir / "pose.json").read_text())
    assert set(meta) == {"rgb", "depth_stamp_ns", "camera_info", "plane_bias_samples", "hints"}
    stamp = T0_NS + FRAME_PERIOD_NS  # the second pair closes the first still window
    assert meta["rgb"]["stamp_ns"] == stamp and meta["depth_stamp_ns"] == stamp
    _, aligned = synthetic_view(NEAR_M, 1)
    depth = cv2.imread(str(pose_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    assert depth.dtype == np.uint16 and np.array_equal(depth, aligned)
    info = camera_info_msg()
    assert meta["camera_info"] == {
        "width": info.width,
        "height": info.height,
        "distortion_model": info.distortion_model,
        "d": info.d,
        "k": info.k,
        "p": info.p,
    }
    assert meta["plane_bias_samples"] > 1000
    session = store.read_session(sid)
    assert session["kind"] == "intrinsics"
    assert not {"lens_position", "tof_info", "tof_size"} & set(session)
    state = controller.state()
    assert state["running"] is True
    assert state["pose_count"] == 1
    assert state["status"] == captured_status(1)


def test_pairs_need_the_exact_stamp(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    feed_still_board(node, depth_offset_ns=1_000_000)
    assert controller.store.pose_ids(sid) == []
    assert controller.state()["status"] == WAITING


def test_moving_depth_is_not_still(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    rgb, aligned = synthetic_view(NEAR_M, 1)
    for step in range(STEPS):
        ns = T0_NS + step * FRAME_PERIOD_NS
        node.subscription(INFO_TOPIC).callback(camera_info_msg())
        node.subscription(RGB_TOPIC).callback(compressed_msg(ns, jpeg(rgb)))
        depth = np.where(aligned > 0, aligned + (30 if step % 2 else 0), 0).astype(np.uint16)  # about 9 %
        node.subscription(DEPTH_TOPIC).callback(image_msg(ns, depth, encoding="16UC1"))
    assert controller.store.pose_ids(sid) == []
    assert controller.state()["status"] == MOVING


def test_moving_colour_is_not_still(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    near, aligned = synthetic_view(NEAR_M, 1)
    far, _ = synthetic_view(0.45, 1)
    for step in range(STEPS):
        ns = T0_NS + step * FRAME_PERIOD_NS
        node.subscription(INFO_TOPIC).callback(camera_info_msg())
        node.subscription(RGB_TOPIC).callback(compressed_msg(ns, jpeg(near if step % 2 else far)))
        node.subscription(DEPTH_TOPIC).callback(image_msg(ns, aligned, encoding="16UC1"))
    assert controller.store.pose_ids(sid) == []
    assert controller.state()["status"] == MOVING


def test_no_camera_info_stores_nothing_until_it_arrives(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    feed_still_board(node, send_info=False)
    assert controller.store.pose_ids(sid) == []
    assert controller.state()["status"] == NO_CAMERA_INFO
    feed_still_board(node, start_ns=T0_NS + STEPS * FRAME_PERIOD_NS)
    assert controller.store.pose_ids(sid) == ["001"]


def test_no_board_stores_nothing_in_the_check(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    blank = np.full((RGB_SIZE[1], RGB_SIZE[0], 3), 127, dtype=np.uint8)
    feed_still_board(node, rgb_data=jpeg(blank))
    assert controller.store.pose_ids(sid) == []
    assert controller.state()["status"] == NO_BOARD


def test_depth_change_fraction() -> None:
    a = np.full((4, 4), 500, dtype=np.uint16)
    b = a.copy()
    b[0, 0] = 0  # invalid in one frame: not compared
    assert depth_change_fraction(a, b) == 0.0
    c = np.full((4, 4), 510, dtype=np.uint16)
    assert depth_change_fraction(a, c) == pytest.approx(10 / 510)
    assert depth_change_fraction(a, np.zeros_like(a)) == float("inf")


def test_raw_rgb_bytes_are_stored(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    rgb, _ = synthetic_view(NEAR_M, 1)
    data = jpeg(rgb, 90)
    feed_still_board(node, rgb_data=data)
    assert (Path(controller.cfg.state_dir) / "sessions" / sid / "poses" / "001" / "rgb.jpg").read_bytes() == data


def test_resume_continues_pose_numbers(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.store.create_session(NOW)
    controller.store.add_pose(sid, make_still(1))
    controller.store.add_pose(sid, make_still(2))
    state = controller.start(sid)
    assert state["session_id"] == sid and state["pose_count"] == 2
    feed_still_board(node)
    assert controller.store.pose_ids(sid) == ["001", "002", "003"]
    state = controller.state()
    assert state["pose_count"] == 3
    assert state["status"] == captured_status(3)


def test_undecodable_frame_dropped(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    _, aligned = synthetic_view(NEAR_M, 1)
    # depth frames whose data does not match height x step or that are not 16-bit, colour frames that are not images
    short = image_msg(T0_NS, aligned)
    short.data = short.data[:-10]
    node.subscription(DEPTH_TOPIC).callback(short)
    node.subscription(DEPTH_TOPIC).callback(image_msg(T0_NS, aligned, encoding="rgb8"))
    for step in range(STEPS):
        ns = T0_NS + step * FRAME_PERIOD_NS
        node.subscription(INFO_TOPIC).callback(camera_info_msg())
        node.subscription(RGB_TOPIC).callback(compressed_msg(ns, b"not a jpeg"))
        node.subscription(RGB_TOPIC).callback(compressed_msg(ns, b""))
        node.subscription(DEPTH_TOPIC).callback(image_msg(ns, aligned, encoding="16UC1"))
    # the colour stream never delivered a decodable frame, so no pair was ever judged
    assert controller.state()["status"] == WAITING
    assert controller.store.pose_ids(sid) == []
    warnings = [msg for level, msg in node.logger.records if level == "warning"]
    assert any(RGB_TOPIC in msg for msg in warnings)
    assert any(DEPTH_TOPIC in msg for msg in warnings)
    # valid frames afterwards are still used
    feed_still_board(node, start_ns=T0_NS + STEPS * FRAME_PERIOD_NS)
    assert controller.store.pose_ids(sid) == ["001"]


def test_frames_after_stop_are_ignored(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    callbacks = {s.topic: s.callback for s in node.subscriptions}
    controller.stop()
    node.subscriptions = [FakeSubscription(Image, t, cb, None) for t, cb in callbacks.items()]
    feed_still_board(node)
    assert controller.store.pose_ids(sid) == []


@pytest.mark.parametrize("bigendian", [False, True])
def test_decode_mono16_honours_step_and_endianness(bigendian: bool) -> None:
    image = np.arange(12, dtype=np.uint16).reshape(3, 4) * 1000 + 7
    decoded = decode_mono16(image_msg(T0_NS, image, pad=6, bigendian=bigendian))
    assert decoded.dtype == np.uint16
    np.testing.assert_array_equal(decoded, image)


def test_state_is_safe_while_frames_arrive(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    errors: list[BaseException] = []
    done = threading.Event()

    def poll() -> None:
        try:
            while not done.is_set():
                assert set(controller.state()) == {
                    "running",
                    "session_id",
                    "status",
                    "pose_count",
                    "started_at",
                    "stops_at",
                }
        except BaseException as exc:  # surfaced by the assertion below
            errors.append(exc)

    poller = threading.Thread(target=poll)
    poller.start()
    feed_still_board(node)
    done.set()
    poller.join()
    assert errors == []
    assert controller.state()["pose_count"] == 1


def test_refused_still_shows_the_reason(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    feed_still_board(node, info=camera_info_msg(width=640))
    assert controller.store.pose_ids(sid) == []
    state = controller.state()
    assert state["running"] is True and state["pose_count"] == 0
    assert state["status"].startswith("pose 1 not stored: camera_info")
    assert any(level == "error" and "not stored" in msg for level, msg in node.logger.records)


# head_arm still capture

HEAD_TOPIC = "/head/joint_states"
ARM_TOPIC = "/follower/joint_states"
HEAD_NAMES = ("head_pan", "head_tilt")
ARM_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
STILL_TIMEOUT_S = 0.3
RGB_PERIOD_NS = 500_000_000


def joint_msg(stamp_ns: int, names: tuple[str, ...], base: float, drift: float = 0.0) -> JointState:
    """A JointState of names; joint i reads base + i / 10 + drift."""
    msg = JointState()
    msg.header = header(stamp_ns)
    msg.name = list(names)
    msg.position = [base + i / 10 + drift for i in range(len(names))]
    return msg


def make_still_controller(tmp_path: Path, node: FakeNode | None = None) -> CaptureController:
    cfg = make_cfg(tmp_path).model_copy(update={"still": StillConfig(timeout_s=STILL_TIMEOUT_S)})
    return CaptureController(node or FakeNode(), SessionStore(Path(cfg.state_dir), cfg), cfg, lambda: NOW)


def feed_still(
    node: FakeNode,
    rgb_jpeg: bytes | None = None,
    arm_drift: float = 0.0,
    arm_offset_ns: int = 0,
    arm_names: tuple[str, ...] = ARM_NAMES,
    head: bool = True,
    arm: bool = True,
    send_info: bool = True,
) -> int:
    """Feed the colour camera_info, 4 RGB frames at 2 Hz and joint samples at 50 Hz around them; returns the newest
    frame stamp."""
    if rgb_jpeg is None:
        frame, _ = synthetic_view(NEAR_M, 1)
        rgb_jpeg = jpeg(frame)
    if send_info:
        node.subscription(INFO_TOPIC).callback(camera_info_msg())
    for k in range(4):
        node.subscription(RGB_TOPIC).callback(compressed_msg(T0_NS + k * RGB_PERIOD_NS, rgb_jpeg))
    for step in range(0, 4 * 25 + 1):
        ns = T0_NS + step * 20_000_000
        if head:
            node.subscription(HEAD_TOPIC).callback(joint_msg(ns, HEAD_NAMES, 0.3))
        if arm:
            node.subscription(ARM_TOPIC).callback(joint_msg(ns + arm_offset_ns, arm_names, -0.2, arm_drift * step))
    return T0_NS + 3 * RGB_PERIOD_NS


def test_still_session_subscribes_four_topics(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    assert controller.store.kind(sid) == "head_arm"
    assert sorted(s.topic for s in node.subscriptions) == sorted([RGB_TOPIC, INFO_TOPIC, HEAD_TOPIC, ARM_TOPIC])
    info = node.subscription(INFO_TOPIC)
    assert info.msg_type is CameraInfo
    assert info.qos.reliability == ReliabilityPolicy.RELIABLE
    assert info.qos.depth == 1
    assert node.subscription(RGB_TOPIC).msg_type is CompressedImage
    assert node.subscription(RGB_TOPIC).qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert node.subscription(RGB_TOPIC).qos.depth == 5
    for topic in (HEAD_TOPIC, ARM_TOPIC):
        sub = node.subscription(topic)
        assert sub.msg_type is JointState
        assert sub.qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert sub.qos.history == HistoryPolicy.KEEP_LAST
        assert sub.qos.depth == 10
    assert controller.state()["kind"] == "head_arm"
    assert controller.state()["running"] is True


def test_capture_still_stores_pose_with_nearest_joints(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    stamp = feed_still(node, arm_offset_ns=5_000_000)
    outcome = controller.capture_still(arm_pose=3, tier=1, head_offset=2)
    assert outcome == StillOutcome(stored=True, pose_id=outcome.pose_id, reason="stored")
    assert outcome.pose_id is not None
    meta = json.loads((controller.store.pose_dir(sid, outcome.pose_id) / "pose.json").read_text())
    assert meta["arm_pose"] == 3 and meta["tier"] == 1 and meta["head_offset"] == 2
    assert meta["stamps"]["rgb"] == stamp
    assert meta["joints"] == pytest.approx(
        {"head_pan": 0.3, "head_tilt": 0.4, **{n: -0.2 + i / 10 for i, n in enumerate(ARM_NAMES)}}
    )
    info = camera_info_msg()
    assert meta["camera_info"] == {
        "width": info.width,
        "height": info.height,
        "distortion_model": info.distortion_model,
        "d": info.d,
        "k": info.k,
        "p": info.p,
    }
    assert controller.state()["pose_count"] == 1


def test_capture_still_without_camera_info_stores_nothing(tmp_path: Path) -> None:
    """A still is stored only with the factory colour camera_info (head-from-arm fits with it)."""
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    assert controller.still_camera_info() is None
    feed_still(node, send_info=False)
    outcome = controller.capture_still(0, 0, 0)
    assert (outcome.stored, outcome.pose_id, outcome.reason) == (False, None, "no colour camera_info")
    assert controller.store.pose_ids(sid) == []
    node.subscription(INFO_TOPIC).callback(camera_info_msg())
    info = controller.still_camera_info()
    assert info is not None and (info.width, info.height) == RGB_SIZE
    assert list(info.k) == FACTORY_K
    assert controller.capture_still(0, 0, 0).stored is True
    controller.stop()
    assert controller.still_camera_info() is None


def test_capture_still_waits_for_still_joints(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    feed_still(node, arm_drift=0.001)
    outcome = controller.capture_still(0, 0, 0)
    assert outcome.stored is False and outcome.pose_id is None
    assert outcome.reason == f"not still within {STILL_TIMEOUT_S:g} s"
    assert controller.store.pose_ids(sid) == []


@pytest.mark.parametrize("missing", ["no_arm", "short_names"])
def test_capture_still_refuses_stale_joints(tmp_path: Path, missing: str) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    if missing == "no_arm":
        feed_still(node, arm=False)
    else:
        feed_still(node, arm_names=ARM_NAMES[:4])
    outcome = controller.capture_still(0, 0, 0)
    assert (outcome.stored, outcome.pose_id, outcome.reason) == (False, None, "joints stale")
    assert controller.store.pose_ids(sid) == []


def test_joints_far_from_frame_stamp_are_stale(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    feed_still(node, arm_offset_ns=3 * NS_PER_S)
    assert controller.capture_still(0, 0, 0).reason == "joints stale"
    assert controller.store.pose_ids(sid) == []


def test_still_without_board_stores_nothing(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    blank = cv2.imencode(".jpg", np.full((480, 640, 3), 127, dtype=np.uint8))[1].tobytes()
    feed_still(node, rgb_jpeg=blank)
    outcome = controller.capture_still(0, 0, 0)
    assert (outcome.stored, outcome.pose_id, outcome.reason) == (False, None, "no board")
    assert controller.store.pose_ids(sid) == []


def test_store_refusal_is_reported(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    feed_still(node)
    outcome = controller.capture_still(0, 7, 0)  # tier 7 is refused by the store
    assert outcome.stored is False and outcome.pose_id is None
    assert outcome.reason.startswith("refused: ") and "tier" in outcome.reason


def test_still_session_stop_unsubscribes_everything(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    controller.stop()
    assert node.subscriptions == [] and len(node.destroyed) == 4
    assert controller.state()["running"] is False
    assert controller.stop()["running"] is False


def test_failed_still_start_leaves_nothing_subscribed(tmp_path: Path) -> None:
    node = FakeNode(fail_subscription_at=3)
    controller = make_still_controller(tmp_path, node)
    with pytest.raises(RuntimeError):
        controller.start_still_session(None)
    assert node.subscriptions == []
    controller.stop()
    assert node.subscriptions == []


def test_still_and_rgbd_exclude_each_other(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start(None)
    with pytest.raises(Busy):
        controller.start_still_session(None)
    assert len(node.subscriptions) == 3  # nothing else was subscribed
    controller.stop()
    controller.start_still_session(None)
    with pytest.raises(Busy):
        controller.start(None)


def test_reopening_an_intrinsics_session_is_refused(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    controller.stop()
    with pytest.raises(CaptureError, match="intrinsics"):
        controller.start_still_session(sid)
    assert node.subscriptions == []


def test_reopening_a_head_arm_session_keeps_it(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    controller.stop()
    assert controller.start_still_session(sid) == sid
    assert controller.state()["session_id"] == sid


def test_capture_still_without_a_session_raises(tmp_path: Path) -> None:
    controller = make_still_controller(tmp_path)
    with pytest.raises(CaptureError):
        controller.capture_still(0, 0, 0)
