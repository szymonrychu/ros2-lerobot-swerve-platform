"""Tests for the live capture controller: subscriptions and QoS, their lifetime, decoding and stored poses."""

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
from ros2_common.head_rgbd_calibration.config import SelectionConfig
from ros2_common.head_rgbd_calibration.poses import WAITING, captured_status
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from head_rgbd_calibration.capture import Busy, CaptureController, CaptureError, StillOutcome, decode_mono16
from head_rgbd_calibration.config import NodeConfig, StillConfig
from head_rgbd_calibration.store import NotFound, SessionStore
from tests.pose_factory import make_pose, synthetic_frames

NOW = datetime(2026, 10, 19, 12, 30, 45, tzinfo=UTC)
LENS = 1.0
NS_PER_S = 1_000_000_000
T0_NS = 1_700_000_000 * NS_PER_S
TOF_PERIOD_NS = 100_000_000  # ToF at 10 Hz
RGB_EVERY = 5  # RGB at 2 Hz
RGB_OFFSET_NS = 5_000_000
NEAR_M = 0.33
DEPTH_MM = 450
STEPS = 12
TOPICS = ("/tof/amplitude/image_raw", "/tof/depth/image_raw", "/tof/camera_info")
RGB_TOPIC = "/head_camera/mapping/image_raw/compressed"


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


def camera_info_msg() -> SimpleNamespace:
    return SimpleNamespace(
        header=header(T0_NS),
        width=240,
        height=180,
        distortion_model="plumb_bob",
        d=[0.1, -0.2, 0.0, 0.0, 0.05],
        k=[191.0, 0, 120, 0, 191, 90, 0, 0, 1],
        p=[191.0, 0, 120, 0, 0, 191, 90, 0, 0, 0, 1, 0],
    )


def write_head_config(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "head_camera.yaml"
    path.write_text(text)
    return path


def make_cfg(tmp_path: Path, head_text: str = f"jpeg_quality: 70\nlens_position: {LENS}\n") -> NodeConfig:
    return NodeConfig(
        state_dir=str(tmp_path / "state"),
        head_camera_config_file=str(write_head_config(tmp_path, head_text)),
        default_lens_position=2.5,
        max_capture_s=120.0,
        selection=SelectionConfig(skip_start_s=0.0),
    )


def make_controller(tmp_path: Path, node: FakeNode | None = None, **cfg_kwargs: Any) -> CaptureController:
    cfg = make_cfg(tmp_path, **cfg_kwargs)
    return CaptureController(node or FakeNode(), SessionStore(Path(cfg.state_dir), cfg), cfg, lambda: NOW)


def feed_still_board(node: FakeNode, steps: int = STEPS, rgb_data: bytes | None = None) -> None:
    """Feed a still board as the topics deliver it: ToF amplitude and depth at 10 Hz, RGB at 2 Hz."""
    rgb, amplitude = synthetic_frames(NEAR_M, 1)
    jpeg = rgb_data if rgb_data is not None else cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    depth = np.full(amplitude.shape, DEPTH_MM, dtype=np.uint16)
    node.subscription(TOPICS[2]).callback(camera_info_msg())
    for step in range(steps):
        ns = T0_NS + step * TOF_PERIOD_NS
        node.subscription(TOPICS[0]).callback(image_msg(ns, amplitude))
        node.subscription(TOPICS[1]).callback(image_msg(ns, depth, encoding="16UC1"))
        if step % RGB_EVERY == 0:
            node.subscription(RGB_TOPIC).callback(compressed_msg(ns + RGB_OFFSET_NS, jpeg))


# subscriptions and QoS


def test_tof_subscriptions_are_reliable(tmp_path: Path) -> None:
    node = FakeNode()
    make_controller(tmp_path, node).start(None)
    amplitude, depth, info = (node.subscription(topic) for topic in TOPICS)
    for sub in (amplitude, depth):
        assert sub.msg_type is Image
        assert sub.qos.reliability == ReliabilityPolicy.RELIABLE
        assert sub.qos.history == HistoryPolicy.KEEP_LAST
        assert sub.qos.depth == 10
    assert info.msg_type is CameraInfo
    assert info.qos.reliability == ReliabilityPolicy.RELIABLE
    assert info.qos.depth == 1


def test_mapping_subscription_best_effort(tmp_path: Path) -> None:
    node = FakeNode()
    make_controller(tmp_path, node).start(None)
    rgb = node.subscription(RGB_TOPIC)
    assert rgb.msg_type is CompressedImage
    assert rgb.qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert rgb.qos.history == HistoryPolicy.KEEP_LAST
    assert rgb.qos.depth == 5
    assert len(node.subscriptions) == 4


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
    assert len(node.destroyed) == 4
    assert node.timers == [] and node.destroyed_timers == [timer] and timer.cancelled
    assert state["running"] is False
    assert controller.stop()["running"] is False  # a second stop is harmless
    assert len(node.destroyed) == 4


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


@pytest.mark.parametrize("head_text", ["lens_position: [1, 2\n", "- a list\n", "lens_position: far\n"])
def test_invalid_head_camera_config_fails_start(tmp_path: Path, head_text: str) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node, head_text=head_text)
    with pytest.raises(CaptureError, match="head_camera.yaml"):
        controller.start(None)
    assert node.created == 0 and node.timers == []


def test_unreadable_head_camera_config_fails_start(tmp_path: Path) -> None:
    node = FakeNode()
    cfg = make_cfg(tmp_path).model_copy(update={"head_camera_config_file": str(tmp_path / "missing.yaml")})
    controller = CaptureController(node, SessionStore(Path(cfg.state_dir), cfg), cfg, lambda: NOW)
    with pytest.raises(CaptureError, match="missing.yaml"):
        controller.start(None)
    assert node.created == 0


def test_absent_lens_position_uses_default(tmp_path: Path) -> None:
    controller = make_controller(tmp_path, head_text="jpeg_quality: 70\n")
    sid = controller.start(None)["session_id"]
    assert controller.store.read_session(sid)["lens_position"] == 2.5


def test_start_while_running_is_busy(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    with pytest.raises(Busy):
        controller.start(None)
    assert len(node.subscriptions) == 4


def test_unknown_session_is_not_found(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    with pytest.raises(NotFound):
        controller.start("calib-20260101-000000")
    assert node.created == 0


def test_reopen_with_another_lens_position_fails(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.store.create_session(NOW, 0.5)
    with pytest.raises(CaptureError, match="lens_position"):
        controller.start(sid)
    assert node.created == 0


def test_capture_publishes_nothing(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    controller.start(None)
    feed_still_board(node)
    controller.stop()
    assert node.publishers == []


# frames and poses


def test_still_board_pose_is_stored(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    assert controller.state()["status"] == WAITING
    feed_still_board(node)
    store = controller.store
    assert store.pose_ids(sid) == ["001"]
    pose_dir = Path(controller.cfg.state_dir) / "sessions" / sid / "poses" / "001"
    assert {p.name for p in pose_dir.iterdir()} >= {"rgb.jpg", "amplitude.png", "depth.png", "pose.json"}
    meta = json.loads((pose_dir / "pose.json").read_text())
    assert meta["rgb"]["stamp_ns"] == T0_NS + RGB_EVERY * TOF_PERIOD_NS + RGB_OFFSET_NS
    assert meta["tof"]["stamp_ns"] == T0_NS + RGB_EVERY * TOF_PERIOD_NS
    depth = cv2.imread(str(pose_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    assert depth.dtype == np.uint16 and int(depth[0, 0]) == DEPTH_MM
    session = store.read_session(sid)
    assert session["tof_info"]["width"] == 240 and session["tof_info"]["k"][0] == 191.0
    assert session["lens_position"] == LENS
    state = controller.state()
    assert state["running"] is True
    assert state["pose_count"] == 1
    assert state["status"] == captured_status(1)


def test_raw_rgb_bytes_are_stored(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    rgb, _ = synthetic_frames(NEAR_M, 1)
    jpeg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
    feed_still_board(node, rgb_data=jpeg)
    assert (Path(controller.cfg.state_dir) / "sessions" / sid / "poses" / "001" / "rgb.jpg").read_bytes() == jpeg


def test_resume_continues_pose_numbers(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.store.create_session(NOW, LENS)
    controller.store.add_pose(sid, make_pose(1))
    controller.store.add_pose(sid, make_pose(2))
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
    rgb, amplitude = synthetic_frames(NEAR_M, 1)
    # amplitude frames whose data does not match height x step, an RGB frame that is not an image
    short = image_msg(T0_NS, amplitude)
    short.data = short.data[:-10]
    node.subscription(TOPICS[0]).callback(short)
    node.subscription(TOPICS[0]).callback(image_msg(T0_NS, amplitude, encoding="rgb8"))
    for step in range(STEPS):
        ns = T0_NS + step * TOF_PERIOD_NS
        if step % RGB_EVERY == 0:
            node.subscription(RGB_TOPIC).callback(compressed_msg(ns, b"not a jpeg"))
            node.subscription(RGB_TOPIC).callback(compressed_msg(ns, b""))
        node.subscription(TOPICS[0]).callback(image_msg(ns, amplitude))
    # the RGB stream never delivered a decodable frame, so the counter was never fed one
    assert controller.state()["status"] == WAITING
    assert controller.store.pose_ids(sid) == []
    warnings = [msg for level, msg in node.logger.records if level == "warning"]
    assert any(RGB_TOPIC in msg for msg in warnings)
    assert any(TOPICS[0] in msg for msg in warnings)
    # valid frames afterwards are still used
    feed_still_board(node)
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
    assert controller.store.read_session(sid)["tof_info"] is None


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


def test_refused_pose_shows_the_reason(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    sid = controller.store.create_session(NOW, LENS)
    controller.store.set_image_sizes(sid, (640, 480), (240, 180))
    controller.start(sid)
    feed_still_board(node)
    assert controller.store.pose_ids(sid) == []
    state = controller.state()
    assert state["running"] is True and state["pose_count"] == 0
    assert state["status"].startswith("pose 1 not stored: rgb_size")
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
) -> int:
    """Feed 4 RGB frames at 2 Hz and joint samples at 50 Hz around them; returns the newest frame stamp."""
    if rgb_jpeg is None:
        frame, _ = synthetic_frames(NEAR_M, 1)
        rgb_jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    for k in range(4):
        node.subscription(RGB_TOPIC).callback(compressed_msg(T0_NS + k * RGB_PERIOD_NS, rgb_jpeg))
    for step in range(0, 4 * 25 + 1):
        ns = T0_NS + step * 20_000_000
        if head:
            node.subscription(HEAD_TOPIC).callback(joint_msg(ns, HEAD_NAMES, 0.3))
        if arm:
            node.subscription(ARM_TOPIC).callback(joint_msg(ns + arm_offset_ns, arm_names, -0.2, arm_drift * step))
    return T0_NS + 3 * RGB_PERIOD_NS


def test_still_session_subscribes_three_topics(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    assert controller.store.kind(sid) == "head_arm"
    assert sorted(s.topic for s in node.subscriptions) == sorted([RGB_TOPIC, HEAD_TOPIC, ARM_TOPIC])
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


def test_still_pose_stored_with_nearest_joints(tmp_path: Path) -> None:
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
    assert controller.state()["pose_count"] == 1


def test_moving_arm_is_not_still(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start_still_session(None)
    feed_still(node, arm_drift=0.001)
    outcome = controller.capture_still(0, 0, 0)
    assert outcome.stored is False and outcome.pose_id is None
    assert outcome.reason == f"not still within {STILL_TIMEOUT_S:g} s"
    assert controller.store.pose_ids(sid) == []


@pytest.mark.parametrize("missing", ["no_arm", "short_names"])
def test_missing_arm_joints_mean_stale_joints(tmp_path: Path, missing: str) -> None:
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


def test_no_board_stores_nothing(tmp_path: Path) -> None:
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


def test_still_stop_leaves_nothing_subscribed(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    controller.stop()
    assert node.subscriptions == [] and len(node.destroyed) == 3
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
    assert len(node.subscriptions) == 4  # nothing else was subscribed
    controller.stop()
    controller.start_still_session(None)
    with pytest.raises(Busy):
        controller.start(None)


def test_reopening_an_rgbd_session_is_refused(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    sid = controller.start(None)["session_id"]
    controller.stop()
    with pytest.raises(CaptureError, match="rgbd"):
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
