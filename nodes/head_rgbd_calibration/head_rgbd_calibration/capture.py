"""Live capture (design D4): subscribe the head topics only while a session runs and store each pose as it is judged.

Threads: the subscription and timer callbacks run on the ROS executor thread; start, stop and state run on HTTP
threads. `lock` guards the shared state (the current capture and its status fields) and is only held for short field
updates. Decoding, detection (PoseCounter) and pose file writes run on the executor thread, serialised by
`feed_lock`, which no HTTP call takes. Subscribing the mapping topic engages the head camera focus lock, so every
subscription and the timer are destroyed when the capture stops, times out or fails to start. Nothing is published.
"""

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from functools import partial
from pathlib import Path
from typing import Any

import yaml
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_common.head_rgbd_calibration.charuco import make_board
from ros2_common.head_rgbd_calibration.frames import (
    StampedImage,
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    stamp_ns,
)
from ros2_common.head_rgbd_calibration.poses import AMPLITUDE, DEPTH, RGB, WAITING, PoseCounter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import SessionStore

MONO16_ENCODINGS = ("mono16", "16UC1")
BYTES_PER_MONO16_PIXEL = 2
TOF_INFO_QOS_DEPTH = 1
IDLE = "idle"
STOPPED = "stopped"
DROP_LOG_THROTTLE_S = 5.0
LENS_POSITION_KEY = "lens_position"


class CaptureError(RuntimeError):
    """A capture cannot start (the reason is the message); nothing stays subscribed."""


class Busy(CaptureError):
    """A capture is already running or starting (maps to 409)."""


@dataclass
class Capture:
    """One capture run: its session, counter, ROS handles and the status shown to the operator.

    Attributes:
        session_id: Session the poses are stored in.
        counter: The core PoseCounter, fed on the executor thread only.
        started_at: Start time.
        stops_at: Time the timer stops the capture.
        pose_count: Poses stored in the session.
        status: Status line: the counter status, a store failure, or why the capture stopped.
        running: False once stopped; callbacks of a stopped capture drop their frames.
        tof_info_stored: The first /tof/camera_info of the capture was handed to the store.
        store_error: (counter status at the failure, message) of the last pose the store refused.
        subscriptions: Subscriptions created by start.
        timer: The max_capture_s timer.
    """

    session_id: str
    counter: PoseCounter
    started_at: datetime
    stops_at: datetime
    pose_count: int
    status: str = WAITING
    running: bool = True
    tof_info_stored: bool = False
    store_error: tuple[str, str] | None = None
    subscriptions: list[Any] = field(default_factory=list)
    timer: Any = None


def read_lens_position(path: str | Path, default: float) -> float:
    """lens_position of the head_camera_cpp node config.

    Args:
        path: The node's config.yaml (a flat mapping).
        default: Value when the file sets no lens_position.

    Returns:
        The lens position in dioptres.

    Raises:
        CaptureError: the file is unreadable, not YAML, not a mapping, or lens_position is not a number.
    """
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, UnicodeDecodeError) as exc:
        raise CaptureError(f"cannot read the head camera config {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise CaptureError(f"the head camera config {path} is not valid YAML: {exc}") from exc
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise CaptureError(f"the head camera config {path} is not a mapping")
    value = data.get(LENS_POSITION_KEY, default)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise CaptureError(f"{LENS_POSITION_KEY} {value!r} in the head camera config {path} is not a number")
    return float(value)


def tof_qos(depth: int) -> QoSProfile:
    """RELIABLE KEEP_LAST profile: the ToF publisher is RELIABLE and BEST_EFFORT loses frames under load."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth)


def rgb_qos(depth: int) -> QoSProfile:
    """BEST_EFFORT KEEP_LAST profile matching the mapping stream publisher."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth)


class CaptureController:
    """Starts and stops live captures; one at a time."""

    def __init__(self, node: Any, store: SessionStore, cfg: NodeConfig, clock: Callable[[], datetime]) -> None:
        """Create an idle controller (nothing is subscribed until start).

        Args:
            node: The rclpy node the subscriptions and the timer are created on.
            store: Session store the poses go to.
            cfg: Node config (topics, qos, max_capture_s, lens position source, board, selection).
            clock: Returns the current time; it names new sessions and sets started_at / stops_at.
        """
        self.node = node
        self.store = store
        self.cfg = cfg
        self.clock = clock
        self.logger = node.get_logger()
        self.board = make_board(cfg.board)
        self.lock = threading.Lock()
        self.feed_lock = threading.Lock()
        self.capture: Capture | None = None
        self.starting = False

    def state(self) -> dict[str, Any]:
        """The capture state for the HTTP API.

        Returns:
            {running, session_id, status, pose_count, started_at, stops_at}: status is the counter status
            ("waiting for frames" until both streams delivered a frame), "idle" before the first start, or why the
            last capture stopped; started_at and stops_at are ISO times, stops_at only while running.
        """
        with self.lock:
            capture = self.capture
            if capture is None:
                return {
                    "running": False,
                    "session_id": None,
                    "status": IDLE,
                    "pose_count": 0,
                    "started_at": None,
                    "stops_at": None,
                }
            return {
                "running": capture.running,
                "session_id": capture.session_id,
                "status": capture.status,
                "pose_count": capture.pose_count,
                "started_at": capture.started_at.isoformat(),
                "stops_at": capture.stops_at.isoformat() if capture.running else None,
            }

    def start(self, session_id: str | None) -> dict[str, Any]:
        """Start a capture on a new session, or on an existing one to add poses to it.

        Args:
            session_id: Session to reopen, None for a new session.

        Returns:
            The state() after the start.

        Raises:
            Busy: a capture is running or starting.
            CaptureError: the head camera config is unreadable or invalid, or the reopened session was captured at
                another lens position.
            NotFound, InvalidId: unknown or malformed session id.
            Exception: whatever creating a subscription, the timer or the session raised; everything created so
                far is destroyed first.
        """
        with self.lock:
            if self.starting or (self.capture is not None and self.capture.running):
                raise Busy("a capture is already running")
            self.starting = True
        try:
            capture = self.open(session_id)
            with self.lock:
                self.capture = capture
        finally:
            with self.lock:
                self.starting = False
        self.logger.info(f"capture started on session {capture.session_id}")
        return self.state()

    def open(self, session_id: str | None) -> Capture:
        """Build a capture and create its subscriptions and timer; the session is created last.

        Callbacks drop every frame until the returned capture is installed as the current one.
        """
        lens = read_lens_position(self.cfg.head_camera_config_file, self.cfg.default_lens_position)
        stored = 0
        if session_id is not None:
            recorded = float(self.store.read_session(session_id)["lens_position"])
            if not math.isclose(recorded, lens):
                raise CaptureError(
                    f"session {session_id} was captured at lens_position {recorded:g}, the head camera is set to "
                    f"{lens:g}: start a new session"
                )
            stored = len(self.store.pose_ids(session_id))
        started = self.clock()
        capture = Capture(
            session_id=session_id or "",
            counter=PoseCounter(self.board, self.cfg.selection, start_count=stored),
            started_at=started,
            stops_at=started + timedelta(seconds=self.cfg.max_capture_s),
            pose_count=stored,
        )
        topics, qos = self.cfg.topics, self.cfg.qos
        subscriptions = (
            (
                Image,
                topics.amplitude,
                partial(self.on_mono16, capture, AMPLITUDE, topics.amplitude),
                tof_qos(qos.tof_depth),
            ),
            (Image, topics.depth, partial(self.on_mono16, capture, DEPTH, topics.depth), tof_qos(qos.tof_depth)),
            (CameraInfo, topics.tof_info, partial(self.on_tof_info, capture), tof_qos(TOF_INFO_QOS_DEPTH)),
            (CompressedImage, topics.rgb, partial(self.on_rgb, capture, topics.rgb), rgb_qos(qos.rgb_depth)),
        )
        try:
            for msg_type, topic, callback, profile in subscriptions:
                capture.subscriptions.append(self.node.create_subscription(msg_type, topic, callback, profile))
            capture.timer = self.node.create_timer(self.cfg.max_capture_s, partial(self.expire, capture))
            if session_id is None:
                capture.session_id = self.store.create_session(started, lens)
        except BaseException:
            capture.running = False
            self.release(capture)
            raise
        return capture

    def stop(self) -> dict[str, Any]:
        """Stop the running capture (no-op when none runs).

        Returns:
            The state() after the stop.
        """
        with self.lock:
            capture = self.capture
        if capture is not None:
            self.finish(capture, STOPPED)
        return self.state()

    def expire(self, capture: Capture) -> None:
        """Timer callback: stop this capture after max_capture_s."""
        self.finish(capture, f"{STOPPED}: max_capture_s ({self.cfg.max_capture_s:g} s) reached")

    def finish(self, capture: Capture, reason: str) -> None:
        """Mark the capture stopped with reason and destroy its subscriptions and timer (once)."""
        with self.lock:
            if not capture.running:
                return
            capture.running = False
            capture.status = reason
        self.release(capture)
        self.logger.info(f"capture on session {capture.session_id} {reason}")

    def release(self, capture: Capture) -> None:
        """Destroy every subscription and the timer of a capture; a failing destroy does not stop the others."""
        handles, capture.subscriptions = capture.subscriptions, []
        timer, capture.timer = capture.timer, None
        for sub in handles:
            try:
                self.node.destroy_subscription(sub)
            except Exception as exc:  # keep destroying the rest
                self.logger.error(f"destroying a capture subscription failed: {exc}")
        if timer is not None:
            try:
                timer.cancel()
                self.node.destroy_timer(timer)
            except Exception as exc:  # logged, the subscriptions are gone already
                self.logger.error(f"destroying the capture timer failed: {exc}")

    def active(self, capture: Capture) -> bool:
        """The capture is the current one and still running."""
        with self.lock:
            return self.capture is capture and capture.running

    def drop(self, topic: str, reason: Exception) -> None:
        """Log a frame that does not decode (throttled); it is never fed."""
        self.logger.warning(f"dropped a frame on {topic}: {reason}", throttle_duration_sec=DROP_LOG_THROTTLE_S)

    # executor-thread callbacks

    def on_mono16(self, capture: Capture, kind: str, topic: str, msg: Any) -> None:
        """Amplitude or depth frame."""
        if not self.active(capture):
            return
        try:
            frame = StampedImage(stamp_ns(msg), decode_mono16(msg))
        except ValueError as exc:
            self.drop(topic, exc)
            return
        self.feed(capture, kind, frame)

    def on_rgb(self, capture: Capture, topic: str, msg: Any) -> None:
        """Mapping RGB frame; its compressed bytes travel with it so the stored rgb.jpg is the received JPEG."""
        if not self.active(capture):
            return
        raw = bytes(msg.data)
        try:
            frame = StampedImage(stamp_ns(msg), decode_compressed(msg, topic))
        except ValueError as exc:
            self.drop(topic, exc)
            return
        self.feed(capture, RGB, frame, raw)

    def on_tof_info(self, capture: Capture, msg: Any) -> None:
        """Store the first /tof/camera_info of the capture in session.json."""
        with self.lock:
            if self.capture is not capture or not capture.running or capture.tof_info_stored:
                return
            capture.tof_info_stored = True
        try:
            self.store.set_tof_info(capture.session_id, decode_camera_info(msg))
        except Exception as exc:  # an executor callback must not raise; retried on the next message
            self.logger.error(f"storing the ToF camera_info failed: {exc}")
            with self.lock:
                capture.tof_info_stored = False

    def feed(self, capture: Capture, kind: str, frame: StampedImage, raw: bytes | None = None) -> None:
        """Feed a decoded frame to the counter and store every pose it captures."""
        with self.feed_lock:
            if not self.active(capture):
                return
            try:
                events = capture.counter.add(kind, frame, raw)
            except Exception as exc:  # an executor callback must not raise
                self.logger.error(f"judging a {kind} frame failed: {exc}")
                return
            stored = 0
            for event in events:
                if event.pose is None:
                    continue
                try:
                    pid = self.store.add_pose(capture.session_id, event.pose)
                except Exception as exc:  # the capture goes on; the operator sees the reason
                    message = f"pose {event.pose.number} not stored: {exc}"
                    self.logger.error(message)
                    capture.store_error = (capture.counter.status, message)
                    continue
                stored += 1
                capture.store_error = None
                self.logger.info(f"session {capture.session_id}: pose {pid} stored")
            status = capture.counter.status
            if capture.store_error is not None and capture.store_error[0] == status:
                status = capture.store_error[1]
            with self.lock:
                capture.pose_count += stored
                if capture.running:
                    capture.status = status
