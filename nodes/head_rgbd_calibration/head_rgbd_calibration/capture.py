"""Live capture (design D4): subscribe the D435 topics only while a session runs and store each still as it is judged.

An intrinsics check capture pairs the D435 colour frame and the depth aligned to it by their exact stamp (the driver
stamps both with the colour frame time), judges each pair for stillness (colour and depth MADs over a window),
detects the ChArUco board on the colour only and stores the still with the latest factory colour camera_info. A
head_arm still session stores each still with the measured joints and that camera_info as well.

Threads: the subscription and timer callbacks run on the ROS executor thread; start, stop and state run on HTTP
threads. `lock` guards the shared state (the current capture and its status fields) and is only held for short field
updates. Decoding, pairing and detection (StillPairCounter) and pose file writes run on the executor thread,
serialised by `feed_lock`, which no HTTP call takes. Every subscription and the timer are destroyed when the capture
stops, times out or fails to start. Nothing is published.
"""

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from functools import partial
from typing import Any

import cv2
import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.config import SelectionConfig
from ros2_common.head_rgbd_calibration.frames import (
    NS_PER_S,
    CameraInfoData,
    StampedImage,
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    rgb_difference,
    stamp_ns,
)
from ros2_common.head_rgbd_calibration.views import ARM_JOINTS, HEAD_JOINTS
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import KIND_HEAD_ARM, KIND_INTRINSICS, ColourDepthStill, SessionStore

INFO_QOS_DEPTH = 1
IDLE = "idle"
STOPPED = "stopped"
DROP_LOG_THROTTLE_S = 5.0
WAITING = "waiting for frames"
MOVING = "moving"
NO_BOARD = "still - no board in the colour image"
NO_CAMERA_INFO = "still - no colour camera_info yet"
PAIR_HISTORY_S = 1.0  # an unpaired colour or depth frame this much older than the newest frame is dropped
JOINT_SUBSCRIPTION_DEPTH = 10  # as FloorSweep's /head/joint_states
RGB_HISTORY = 16
JOINT_HISTORY = 400
STILL_POLL_S = 0.02
MIN_STILL_SAMPLES = 2
REASON_STORED = "stored"
REASON_NO_BOARD = "no board"
REASON_JOINTS_STALE = "joints stale"
REASON_NO_CAMERA_INFO = "no colour camera_info"
REASON_STOPPED = "stopped"


class CaptureError(RuntimeError):
    """A capture cannot start (the reason is the message); nothing stays subscribed."""


class Busy(CaptureError):
    """A capture is already running or starting (maps to 409)."""


def captured_status(count: int) -> str:
    """Status line of a captured still.

    Args:
        count: Number of the still.

    Returns:
        str: "pose N captured - move to the next pose".
    """
    return f"pose {count} captured - move to the next pose"


def depth_change_fraction(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute change between two aligned depth frames as a fraction of the median depth of the second.

    Args:
        a: Earlier uint16 depth frame, mm (0 is no depth).
        b: Later frame of the same size.

    Returns:
        float: mean |b - a| / median(b) over the pixels valid in both; inf when no pixel is valid in both.
    """
    valid = (a > 0) & (b > 0)
    if not valid.any():
        return float("inf")
    later = b[valid].astype(np.float64)
    return float(np.mean(np.abs(later - a[valid])) / np.median(later))


@dataclass(frozen=True)
class PairFrame:
    """A colour frame and the aligned depth of the same stamp, with the colour's grey image and compressed bytes."""

    stamp_ns: int
    rgb: np.ndarray
    raw: bytes | None
    grey: np.ndarray
    depth: np.ndarray


@dataclass(frozen=True)
class StillEvent:
    """The judgement of one colour + depth pair: its status, and the still when this pair captured one."""

    status: str
    still: ColourDepthStill | None = None


class StillPairCounter:
    """Captures still windows of exact-stamp colour + aligned depth pairs in which the colour shows the board.

    A pair is still when every consecutive pair in the last `still_window_s` (at least two pairs) differs by less than
    `still_rgb_mad` (colour) and `still_depth_mad` (depth); one still is captured per still window, once the board is
    detected on the colour and a factory colour camera_info has arrived. Pairs before `skip_start_s` are not captured.
    """

    def __init__(self, board: cv2.aruco.CharucoBoard, sel: SelectionConfig, start_count: int = 0) -> None:
        """Start with no frames.

        Args:
            board: The ChArUco board.
            sel: Selection settings (still_window_s, still_rgb_mad, still_depth_mad, skip_start_s).
            start_count: Stills already stored; the next captured still is numbered start_count + 1.
        """
        self.board = board
        self.sel = sel
        self.count = start_count
        self.camera_info: CameraInfoData | None = None
        self.rgb: dict[int, tuple[np.ndarray, bytes | None]] = {}
        self.depth: dict[int, np.ndarray] = {}
        self.newest_ns = 0
        self.window: list[PairFrame] = []
        self.still_steps: list[bool] = []  # still_steps[i]: window[i] and window[i + 1] differ less than both MADs
        self.first_ns: int | None = None
        self.window_counted = False
        self.last_status: str | None = None

    @property
    def status(self) -> str:
        """WAITING until a pair was judged, then the status of the last judged pair."""
        return self.last_status or WAITING

    def add_rgb(self, frame: StampedImage, raw: bytes | None) -> list[StillEvent]:
        """Add a decoded colour frame and its compressed bytes; judges the pair when its depth has arrived."""
        self.rgb[frame.stamp_ns] = (frame.image, raw)
        return self.pair(frame.stamp_ns)

    def add_depth(self, frame: StampedImage) -> list[StillEvent]:
        """Add a decoded aligned depth frame; judges the pair when its colour frame has arrived."""
        self.depth[frame.stamp_ns] = frame.image
        return self.pair(frame.stamp_ns)

    def pair(self, stamp: int) -> list[StillEvent]:
        """Judge the pair of this stamp when both frames are in, and drop unpaired frames older than PAIR_HISTORY_S."""
        self.newest_ns = max(self.newest_ns, stamp)
        events = []
        if stamp in self.rgb and stamp in self.depth:
            image, raw = self.rgb.pop(stamp)
            depth = self.depth.pop(stamp)
            grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            event = self.judge(PairFrame(stamp, image, raw, grey, depth))
            if event is not None:
                self.last_status = event.status
                events.append(event)
        horizon = self.newest_ns - int(PAIR_HISTORY_S * NS_PER_S)
        for pending in (self.rgb, self.depth):
            for old in [ns for ns in pending if ns < horizon]:
                del pending[old]
        return events

    def judge(self, pair: PairFrame) -> StillEvent | None:
        """Judge one pair; None for a pair older than the last judged one (it arrived out of order)."""
        if self.window and pair.stamp_ns <= self.window[-1].stamp_ns:
            return None
        if self.window:
            previous = self.window[-1]
            self.still_steps.append(
                rgb_difference(previous.grey, pair.grey) < self.sel.still_rgb_mad
                and depth_change_fraction(previous.depth, pair.depth) < self.sel.still_depth_mad
            )
        self.window.append(pair)
        since = pair.stamp_ns - int(self.sel.still_window_s * NS_PER_S)
        while self.window[0].stamp_ns < since:
            del self.window[0]
            del self.still_steps[0]
        if self.first_ns is None:
            self.first_ns = pair.stamp_ns + int(self.sel.skip_start_s * NS_PER_S)
        still = pair.stamp_ns >= self.first_ns and len(self.window) >= MIN_STILL_SAMPLES and all(self.still_steps)
        if not still:
            self.window_counted = False
            return StillEvent(MOVING)
        if self.window_counted:
            return StillEvent(captured_status(self.count))
        detection = detect(pair.rgb, self.board)
        if detection is None:
            return StillEvent(NO_BOARD)
        if self.camera_info is None:
            return StillEvent(NO_CAMERA_INFO)
        self.count += 1
        self.window_counted = True
        stamped = StampedImage(pair.stamp_ns, pair.rgb)
        still_frame = ColourDepthStill(
            self.count, stamped, pair.raw, StampedImage(pair.stamp_ns, pair.depth), detection, self.camera_info
        )
        return StillEvent(captured_status(self.count), still_frame)


@dataclass
class Capture:
    """One capture run: its session, counter, ROS handles and the status shown to the operator.

    Attributes:
        session_id: Session the stills are stored in.
        counter: The StillPairCounter, fed on the executor thread only.
        started_at: Start time.
        stops_at: Time the timer stops the capture.
        pose_count: Poses stored in the session.
        status: Status line: the counter status, a store failure, or why the capture stopped.
        running: False once stopped; callbacks of a stopped capture drop their frames.
        store_error: (counter status at the failure, message) of the last pose the store refused.
        subscriptions: Subscriptions created by start.
        timer: The max_capture_s timer.
    """

    session_id: str
    counter: StillPairCounter
    started_at: datetime
    stops_at: datetime
    pose_count: int
    status: str = WAITING
    running: bool = True
    store_error: tuple[str, str] | None = None
    subscriptions: list[Any] = field(default_factory=list)
    timer: Any = None


@dataclass(frozen=True)
class JointsSample:
    """Measured positions (rad, by joint name) of one joint group with the header stamp (ns) of the message."""

    stamp_ns: int
    positions: dict[str, float]


@dataclass(frozen=True)
class StillFrame:
    """A decoded mapping RGB frame with the JPEG bytes it arrived as and its grey image."""

    stamp_ns: int
    image: Any
    raw: bytes
    grey: Any


@dataclass(frozen=True)
class StillOutcome:
    """Result of capture_still: whether a pose was stored, its id, and the reason (stored, no board, ...)."""

    stored: bool
    pose_id: str | None
    reason: str


@dataclass
class StillRun:
    """One head_arm still session: its subscriptions and the recent frames and joint samples.

    Attributes:
        session_id: The head_arm session the poses are stored in.
        started_at: Start time.
        running: False once stopped.
        pose_count: Poses stored in the session.
        subscriptions: Subscriptions created by start_still_session.
        camera_info: The latest factory colour camera_info, None until one has arrived.
        frames: Recent RGB frames, newest last.
        head: Recent head joint samples, newest last.
        arm: Recent arm joint samples, newest last.
    """

    session_id: str
    started_at: datetime
    pose_count: int
    running: bool = True
    subscriptions: list[Any] = field(default_factory=list)
    camera_info: CameraInfoData | None = None
    frames: deque[StillFrame] = field(default_factory=lambda: deque(maxlen=RGB_HISTORY))
    head: deque[JointsSample] = field(default_factory=lambda: deque(maxlen=JOINT_HISTORY))
    arm: deque[JointsSample] = field(default_factory=lambda: deque(maxlen=JOINT_HISTORY))


def reliable_qos(depth: int) -> QoSProfile:
    """RELIABLE KEEP_LAST profile, as the D435 driver publishes depth and camera_info (its default QoS)."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth)


def rgb_qos(depth: int) -> QoSProfile:
    """BEST_EFFORT KEEP_LAST profile for the colour and joint streams (a late frame is better dropped than queued)."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth)


class CaptureController:
    """Starts and stops live captures; one at a time."""

    def __init__(self, node: Any, store: SessionStore, cfg: NodeConfig, clock: Callable[[], datetime]) -> None:
        """Create an idle controller (nothing is subscribed until start).

        Args:
            node: The rclpy node the subscriptions and the timer are created on.
            store: Session store the poses go to.
            cfg: Node config (topics, qos, max_capture_s, board, selection, still).
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
        self.still: StillRun | None = None
        self.latest_is_still = False
        self.starting = False

    def state(self) -> dict[str, Any]:
        """The capture state for the HTTP API.

        Returns:
            {running, session_id, kind, status, pose_count, started_at, stops_at}: kind ("head_arm") is only
            present for a still session, an intrinsics check capture has no kind key; status is the counter status
            ("waiting for frames" until a colour + depth pair was judged), "idle" before the first start, or why the
            last capture stopped; started_at and stops_at are ISO times, stops_at only while running.
        """
        with self.lock:
            still = self.still if self.latest_is_still else None
            if still is not None:
                return {
                    "running": still.running,
                    "session_id": still.session_id,
                    "kind": KIND_HEAD_ARM,
                    "status": "still capture" if still.running else STOPPED,
                    "pose_count": still.pose_count,
                    "started_at": still.started_at.isoformat(),
                    "stops_at": None,
                }
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
            CaptureError: the reopened session is not an intrinsics session.
            NotFound, InvalidId: unknown or malformed session id.
            Exception: whatever creating a subscription, the timer or the session raised; everything created so
                far is destroyed first.
        """
        with self.lock:
            self.require_idle()
            self.starting = True
        try:
            capture = self.open(session_id)
            with self.lock:
                self.capture = capture
                self.latest_is_still = False
        finally:
            with self.lock:
                self.starting = False
        self.logger.info(f"capture started on session {capture.session_id}")
        return self.state()

    def require_idle(self) -> None:
        """Raise Busy unless no capture or still session runs or starts; the caller holds the lock."""
        if self.starting or (self.capture is not None and self.capture.running):
            raise Busy("a capture is already running")
        if self.still is not None and self.still.running:
            raise Busy("a still capture session is already running")

    def open(self, session_id: str | None) -> Capture:
        """Build a capture and create its subscriptions and timer; the session is created last.

        Callbacks drop every frame until the returned capture is installed as the current one.
        """
        stored = 0
        if session_id is not None:
            kind = self.store.kind(session_id)
            if kind != KIND_INTRINSICS:
                raise CaptureError(f"session {session_id} is a {kind} session, not {KIND_INTRINSICS}")
            stored = len(self.store.pose_ids(session_id))
        started = self.clock()
        capture = Capture(
            session_id=session_id or "",
            counter=StillPairCounter(self.board, self.cfg.selection, start_count=stored),
            started_at=started,
            stops_at=started + timedelta(seconds=self.cfg.max_capture_s),
            pose_count=stored,
        )
        topics, qos = self.cfg.topics, self.cfg.qos
        subscriptions = (
            (Image, topics.depth, partial(self.on_depth, capture, topics.depth), reliable_qos(qos.depth)),
            (CameraInfo, topics.rgb_info, partial(self.on_rgb_info, capture), reliable_qos(INFO_QOS_DEPTH)),
            (CompressedImage, topics.rgb, partial(self.on_rgb, capture, topics.rgb), rgb_qos(qos.rgb)),
        )
        try:
            for msg_type, topic, callback, profile in subscriptions:
                capture.subscriptions.append(self.node.create_subscription(msg_type, topic, callback, profile))
            capture.timer = self.node.create_timer(self.cfg.max_capture_s, partial(self.expire, capture))
            if session_id is None:
                capture.session_id = self.store.create_session(started)
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
            still = self.still
        if capture is not None:
            self.finish(capture, STOPPED)
        if still is not None:
            self.finish_still(still)
        return self.state()

    def start_still_session(self, session_id: str | None) -> str:
        """Start a head_arm still session: subscribe the colour stream, its camera_info and the head and arm joint
        states.

        Args:
            session_id: head_arm session to add poses to, None for a new one.

        Returns:
            The session id.

        Raises:
            Busy: an intrinsics check capture or another still session runs or starts.
            CaptureError: the session is not a head_arm session.
            NotFound, InvalidId: unknown or malformed session id.
            Exception: whatever creating a subscription or the session raised; everything created so far is
                destroyed first.
        """
        with self.lock:
            self.require_idle()
            self.starting = True
        try:
            run = self.open_still(session_id)
            with self.lock:
                self.still = run
                self.latest_is_still = True
        finally:
            with self.lock:
                self.starting = False
        self.logger.info(f"still capture started on session {run.session_id}")
        return run.session_id

    def open_still(self, session_id: str | None) -> StillRun:
        """Build a still run and create its subscriptions; the session is created last."""
        stored = 0
        if session_id is not None:
            kind = self.store.kind(session_id)
            if kind != KIND_HEAD_ARM:
                raise CaptureError(f"session {session_id} is a {kind} session, not {KIND_HEAD_ARM}")
            stored = len(self.store.pose_ids(session_id))
        run = StillRun(session_id=session_id or "", started_at=self.clock(), pose_count=stored)
        topics = self.cfg.topics
        joint_qos = rgb_qos(JOINT_SUBSCRIPTION_DEPTH)
        subscriptions = (
            (CompressedImage, topics.rgb, partial(self.on_still_rgb, run, topics.rgb), rgb_qos(self.cfg.qos.rgb)),
            (CameraInfo, topics.rgb_info, partial(self.on_still_info, run), reliable_qos(INFO_QOS_DEPTH)),
            (JointState, topics.head_joint_states, partial(self.on_joints, run.head, HEAD_JOINTS), joint_qos),
            (JointState, topics.arm_joint_states, partial(self.on_joints, run.arm, ARM_JOINTS), joint_qos),
        )
        try:
            for msg_type, topic, callback, profile in subscriptions:
                run.subscriptions.append(self.node.create_subscription(msg_type, topic, callback, profile))
            if session_id is None:
                run.session_id = self.store.create_session(run.started_at, KIND_HEAD_ARM)
        except BaseException:
            run.running = False
            self.release_still(run)
            raise
        return run

    def finish_still(self, run: StillRun) -> None:
        """Mark the still run stopped and destroy its subscriptions (once)."""
        with self.lock:
            if not run.running:
                return
            run.running = False
        self.release_still(run)
        self.logger.info(f"still capture on session {run.session_id} {STOPPED}")

    def release_still(self, run: StillRun) -> None:
        """Destroy every subscription of a still run; a failing destroy does not stop the others."""
        handles, run.subscriptions = run.subscriptions, []
        for sub in handles:
            try:
                self.node.destroy_subscription(sub)
            except Exception as exc:  # keep destroying the rest
                self.logger.error(f"destroying a still capture subscription failed: {exc}")

    def still_camera_info(self) -> CameraInfoData | None:
        """The latest factory colour camera_info of the running still session (None without a session or before the
        first camera_info)."""
        with self.lock:
            run = self.still
            return run.camera_info if run is not None and run.running else None

    def capture_still(self, arm_pose: int, tier: int, head_offset: int) -> StillOutcome:
        """Wait for a still scene, then store the newest frame with the measured head and arm joints.

        Blocking: call it from the sequencer thread, never from the executor (which delivers the frames).

        Args:
            arm_pose: Index of the scripted arm pose.
            tier: Capture tier 0, 1 or 2.
            head_offset: Index of the head offset.

        Returns:
            StillOutcome: stored with the pose id, or why nothing was stored: "no board", "no colour camera_info",
            "joints stale", "not still within <t> s", "refused: <store message>", or "stopped" when the session was
            stopped while waiting. Missing joints are never filled in.

        Raises:
            CaptureError: no still session was started.
        """
        with self.lock:
            run = self.still
        if run is None:
            raise CaptureError("no still session: start one first")
        still = self.cfg.still
        deadline = time.monotonic() + still.timeout_s
        while True:
            if not run.running:
                return StillOutcome(False, None, REASON_STOPPED)
            found = self.still_snapshot(run)
            if found is not None:
                return self.store_still(run, found, arm_pose, tier, head_offset)
            if time.monotonic() >= deadline:
                break
            time.sleep(STILL_POLL_S)
        if self.joints_near_newest_frame(run) is None:
            return StillOutcome(False, None, REASON_JOINTS_STALE)
        return StillOutcome(False, None, f"not still within {still.timeout_s:g} s")

    def joints_near_newest_frame(self, run: StillRun) -> tuple[StillFrame, JointsSample, JointsSample] | None:
        """The newest frame with the head and arm samples nearest its stamp within max_joint_dt_s, else None."""
        with self.lock:
            frames, head, arm = list(run.frames), list(run.head), list(run.arm)
        if not frames:
            return None
        frame = frames[-1]
        limit = self.cfg.still.max_joint_dt_s * NS_PER_S
        picked = []
        for samples in (head, arm):
            if not samples:
                return None
            best = min(samples, key=lambda s: abs(s.stamp_ns - frame.stamp_ns))
            if abs(best.stamp_ns - frame.stamp_ns) > limit:
                return None
            picked.append(best)
        return frame, picked[0], picked[1]

    def still_snapshot(self, run: StillRun) -> tuple[StillFrame, JointsSample, JointsSample] | None:
        """(frame, head sample, arm sample) when the last window_s of frames and joints is still, else None."""
        near = self.joints_near_newest_frame(run)
        if near is None:
            return None
        frame, head, arm = near
        still = self.cfg.still
        since = frame.stamp_ns - int(still.window_s * NS_PER_S)
        with self.lock:
            frames = [f for f in run.frames if f.stamp_ns >= since]
            groups = (
                (HEAD_JOINTS, [s for s in run.head if s.stamp_ns >= since]),
                (ARM_JOINTS, [s for s in run.arm if s.stamp_ns >= since]),
            )
        if len(frames) < MIN_STILL_SAMPLES:
            return None
        if any(rgb_difference(a.grey, b.grey) > still.rgb_mad for a, b in zip(frames, frames[1:], strict=False)):
            return None
        for names, samples in groups:
            if len(samples) < MIN_STILL_SAMPLES:
                return None
            for name in names:
                values = [s.positions[name] for s in samples]
                if max(values) - min(values) >= still.joint_still_rad:
                    return None
        return frame, head, arm

    def store_still(
        self,
        run: StillRun,
        found: tuple[StillFrame, JointsSample, JointsSample],
        arm_pose: int,
        tier: int,
        head_offset: int,
    ) -> StillOutcome:
        """Detect the board in the frame and store it with its joints and the latest colour camera_info."""
        frame, head, arm = found
        detection = detect(frame.image, make_board(self.cfg.fit.board))
        if detection is None:
            return StillOutcome(False, None, REASON_NO_BOARD)
        with self.lock:
            info = run.camera_info
        if info is None:
            return StillOutcome(False, None, REASON_NO_CAMERA_INFO)
        try:
            pid = self.store.add_still_pose(
                run.session_id,
                frame.raw,
                frame.image,
                detection,
                {**head.positions, **arm.positions},
                {"rgb": frame.stamp_ns, "head_joints": head.stamp_ns, "arm_joints": arm.stamp_ns},
                arm_pose,
                tier,
                head_offset,
                info,
            )
        except (ValueError, OSError) as exc:
            return StillOutcome(False, None, f"refused: {exc}")
        with self.lock:
            run.pose_count += 1
        self.logger.info(f"session {run.session_id}: still pose {pid} stored")
        return StillOutcome(True, pid, REASON_STORED)

    # still executor-thread callbacks

    def on_still_rgb(self, run: StillRun, topic: str, msg: Any) -> None:
        """Colour frame of a still session; the compressed bytes travel with it."""
        if not run.running:
            return
        try:
            image = decode_compressed(msg, topic)
        except ValueError as exc:
            self.drop(topic, exc)
            return
        frame = StillFrame(stamp_ns(msg), image, bytes(msg.data), cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        with self.lock:
            run.frames.append(frame)

    def on_still_info(self, run: StillRun, msg: Any) -> None:
        """Keep the latest factory colour camera_info of a still session; each still stores the current one."""
        if not run.running:
            return
        info = decode_camera_info(msg)
        with self.lock:
            run.camera_info = info

    def on_joints(self, samples: deque[JointsSample], names: tuple[str, ...], msg: Any) -> None:
        """Keep a stamped sample of a joint group; a message that lacks a joint of the group is ignored."""
        index = {name: i for i, name in enumerate(msg.name)}
        try:
            positions = {name: float(msg.position[index[name]]) for name in names}
        except (KeyError, IndexError):
            return
        with self.lock:
            samples.append(JointsSample(stamp_ns(msg), positions))

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

    def on_depth(self, capture: Capture, topic: str, msg: Any) -> None:
        """Aligned depth frame (16UC1 mm)."""
        if not self.active(capture):
            return
        try:
            frame = StampedImage(stamp_ns(msg), decode_mono16(msg))
        except ValueError as exc:
            self.drop(topic, exc)
            return
        self.feed(capture, "depth", lambda: capture.counter.add_depth(frame))

    def on_rgb(self, capture: Capture, topic: str, msg: Any) -> None:
        """Colour frame; its compressed bytes travel with it so the stored rgb.jpg is the received JPEG."""
        if not self.active(capture):
            return
        raw = bytes(msg.data)
        try:
            frame = StampedImage(stamp_ns(msg), decode_compressed(msg, topic))
        except ValueError as exc:
            self.drop(topic, exc)
            return
        self.feed(capture, "colour", lambda: capture.counter.add_rgb(frame, raw))

    def on_rgb_info(self, capture: Capture, msg: Any) -> None:
        """Keep the latest factory colour camera_info; each still stores the one current when it is captured."""
        if not self.active(capture):
            return
        info = decode_camera_info(msg)
        with self.feed_lock:
            capture.counter.camera_info = info

    def feed(self, capture: Capture, kind: str, add: Callable[[], list[StillEvent]]) -> None:
        """Feed a decoded frame to the counter (add) and store every still it captures."""
        with self.feed_lock:
            if not self.active(capture):
                return
            try:
                events = add()
            except Exception as exc:  # an executor callback must not raise
                self.logger.error(f"judging a {kind} frame failed: {exc}")
                return
            stored = 0
            for event in events:
                if event.still is None:
                    continue
                try:
                    pid = self.store.add_pose(capture.session_id, event.still)
                except Exception as exc:  # the capture goes on; the operator sees the reason
                    message = f"pose {event.still.number} not stored: {exc}"
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
