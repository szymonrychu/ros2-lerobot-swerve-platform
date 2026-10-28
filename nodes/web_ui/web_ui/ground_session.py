"""One ground recording run: ROS subscriptions in, a GroundRecording npz out.

The session is created by BridgeNode.ground_recording_start. Every sensor message is stored with the TF at its own
stamp; a message whose TF does not arrive within tf_timeout_s is dropped and counted, never recorded with a guessed
or latest pose. Camera infos are subscribed best effort, so they match both reliable and best-effort publishers
(head_rgbd_cpp publishes best effort). The session publishes nothing and commands nothing.
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import structlog
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformException

from .config import GroundRecorderConfig
from .ground_recorder import (
    STREAM_CLOUD,
    STREAM_HEAD,
    STREAM_IMU,
    STREAM_PAIR,
    GroundRecording,
    Pose7,
)
from .msg_serializer import cloud_xyz

log = structlog.get_logger(__name__)

NAME_PATTERN = re.compile(r"^[a-z0-9_-]{1,32}$")
NPZ_SUFFIX = ".npz"
META_SUFFIX = ".json"
PART_SUFFIX = ".part"
HEAD_PAN_JOINT = "head_pan"
HEAD_TILT_JOINT = "head_tilt"
DEPTH_ENCODINGS = frozenset({"16uc1"})
# Unmatched half-pairs (depth without its RGB yet, or the reverse) kept before the oldest is dropped.
MATCH_CAPACITY = 8
REASON_TF = "tf"
REASON_DECODE = "decode"
REASON_JOINTS = "joints"
REASON_UNMATCHED = "unmatched"
KIND_DEPTH = "depth"
KIND_OTHER = "other"
NS_PER_S = 1_000_000_000
RELIABLE_DEPTH = 10

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=RELIABLE_DEPTH)

PoseLookup = Callable[[str, str, int], Pose7]


@dataclass
class Pending:
    """A converted message waiting for its TF."""

    received: float
    stream: str
    apply: Callable[[], None]


def stamp_ns(header: Any) -> int:
    """Header stamp in nanoseconds.

    :param header: std_msgs/Header (Any).
    :return: stamp, ns (int).
    """
    return int(header.stamp.sec) * NS_PER_S + int(header.stamp.nanosec)


def image_array(msg: Any, encodings: frozenset[str]) -> np.ndarray | None:
    """Convert a 16-bit sensor_msgs/Image to a native uint16 array.

    :param msg: sensor_msgs/Image (Any).
    :param encodings: accepted lower-case encodings (frozenset[str]).
    :return: array (height, width) uint16, or None for another encoding or short data.
    """
    if str(msg.encoding).lower() not in encodings:
        return None
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    row_items = int(msg.step) // dtype.itemsize
    raw = bytes(msg.data)
    if row_items < msg.width or len(raw) < msg.height * row_items * dtype.itemsize:
        return None
    rows = np.frombuffer(raw, dtype=dtype, count=msg.height * row_items).reshape(msg.height, row_items)
    return np.ascontiguousarray(rows[:, : msg.width]).astype(np.uint16)


def list_recordings(directory: Path) -> list[Path]:
    """Finished recording files, oldest first.

    :param directory: recording directory (Path).
    :return: npz paths sorted by modification time then name (list[Path]).
    """
    return sorted(directory.glob(f"*{NPZ_SUFFIX}"), key=lambda p: (p.stat().st_mtime, p.name))


def remove_recording(directory: Path, name: str) -> bool:
    """Delete a recording and its sidecar.

    :param directory: recording directory (Path).
    :param name: recording name (str).
    :return: True when the npz existed (bool).
    """
    (directory / f"{name}{META_SUFFIX}").unlink(missing_ok=True)
    npz = directory / f"{name}{NPZ_SUFFIX}"
    existed = npz.exists()
    npz.unlink(missing_ok=True)
    return existed


def prune_recordings(directory: Path, max_files: int) -> None:
    """Delete the oldest recordings beyond max_files.

    :param directory: recording directory (Path).
    :param max_files: how many to keep (int).
    """
    files = list_recordings(directory)
    for old in files[: max(0, len(files) - max_files)]:
        remove_recording(directory, old.stem)


def read_recording_info(directory: Path, name: str) -> dict[str, Any] | None:
    """Describe a finished recording from its file and sidecar.

    :param directory: recording directory (Path).
    :param name: recording name (str).
    :return: {name, bytes, done, counts, dropped, topics}, or None when the file does not exist.
    """
    npz = directory / f"{name}{NPZ_SUFFIX}"
    if not npz.exists():
        return None
    meta: dict[str, Any] = {}
    try:
        meta = json.loads((directory / f"{name}{META_SUFFIX}").read_text())
    except (OSError, ValueError):
        pass
    return {
        "name": name,
        "bytes": npz.stat().st_size,
        "done": True,
        "counts": meta.get("counts", {}),
        "dropped": meta.get("dropped", {}),
        "topics": meta.get("topics", {}),
    }


class GroundSession:
    """Subscribes the recorder topics for one window and writes the npz at its end."""

    def __init__(
        self,
        cfg: GroundRecorderConfig,
        name: str,
        duration_s: float,
        subscribe: Callable[[type, str, Callable[[Any], None], Any], Any],
        unsubscribe: Callable[[Any], None],
        lookup: PoseLookup,
        on_done: Callable[[], None],
    ) -> None:
        """Create a session; nothing is subscribed until open().

        :param cfg: recorder settings (GroundRecorderConfig).
        :param name: recording name (str).
        :param duration_s: window length, seconds (float).
        :param subscribe: create_subscription(msg_cls, topic, callback, qos) of the node.
        :param unsubscribe: destroy_subscription of the node.
        :param lookup: lookup(target, source, stamp_ns) -> Pose7, raising TransformException when unavailable.
        :param on_done: called from the writer thread once the file is written (or failed).
        """
        self.name = name
        self.duration_s = float(duration_s)
        self.cfg = cfg
        self.recording = GroundRecording(name, duration_s)
        self.subscribe = subscribe
        self.unsubscribe = unsubscribe
        self.lookup = lookup
        self.on_done = on_done
        self.lock = threading.RLock()
        self.subscriptions: list[Any] = []
        self.pending: list[Pending] = []
        self.waiting: dict[int, dict[str, Any]] = {}
        self.imu_mount: Pose7 | None = None
        self.started: float | None = None
        self.closed = False
        self.writer: threading.Thread | None = None

    def plan(self) -> list[tuple[type, str, Callable[[Any], None], Any]]:
        """The subscriptions of a recording.

        :return: (message class, GroundRecorderConfig topic field, callback, QoS) per subscription (list[tuple]).
        """
        return [
            (PointCloud2, "cloud_topic", self.on_cloud, RELIABLE_QOS),
            (CameraInfo, "depth_info_topic", self.on_depth_info, SENSOR_QOS),
            (Image, "pair_depth_topic", self.on_depth, SENSOR_QOS),
            (CameraInfo, "pair_info_topic", self.on_pair_info, SENSOR_QOS),
            (CompressedImage, "pair_rgb_topic", self.on_rgb, SENSOR_QOS),
            (Imu, "imu_topic", self.on_imu, SENSOR_QOS),
            (JointState, "head_js_topic", self.on_head, RELIABLE_QOS),
            (String, "state_topic", self.on_state, RELIABLE_QOS),
        ]

    def topics(self) -> dict[str, str]:
        """The topics this recording subscribes.

        :return: GroundRecorderConfig topic field to topic name (dict[str, str]).
        """
        return {field: getattr(self.cfg, field) for _, field, _, _ in self.plan()}

    def open(self) -> None:
        """Create the subscriptions and start the window (executor thread)."""
        with self.lock:
            self.subscriptions = [
                self.subscribe(cls, getattr(self.cfg, field), callback, qos)
                for cls, field, callback, qos in self.plan()
            ]
            self.started = time.monotonic()

    def submit(self, stream: str, apply: Callable[[], None]) -> None:
        """Run apply now; when its TF is not available yet hold it until tf_timeout_s.

        :param stream: stream name for drop counting (str).
        :param apply: stores the sample, raising TransformException before storing anything (Callable).
        """
        with self.lock:
            if self.closed:
                return
            try:
                apply()
            except TransformException:
                if self.cfg.tf_timeout_s > 0:
                    self.pending.append(Pending(time.monotonic(), stream, apply))
                else:
                    self.recording.drop(stream, REASON_TF)

    def service(self, now: float) -> None:
        """Retry held samples, then close the window when it is over (executor thread).

        :param now: monotonic time, seconds (float).
        """
        with self.lock:
            if self.closed:
                return
            still: list[Pending] = []
            for item in self.pending:
                try:
                    item.apply()
                except TransformException:
                    if now - item.received >= self.cfg.tf_timeout_s:
                        self.recording.drop(item.stream, REASON_TF)
                    else:
                        still.append(item)
            self.pending = still
            if self.started is not None and now - self.started >= self.duration_s:
                self.close()

    def close(self) -> None:
        """Destroy the subscriptions, count what never completed and write the file on a thread."""
        with self.lock:
            self.closed = True
            for sub in self.subscriptions:
                self.unsubscribe(sub)
            self.subscriptions = []
            for item in self.pending:
                self.recording.drop(item.stream, REASON_TF)
            self.pending = []
            if self.waiting:
                self.recording.drop(STREAM_PAIR, REASON_UNMATCHED, len(self.waiting))
            self.waiting.clear()
            self.writer = threading.Thread(target=self.write, name=f"ground-write-{self.name}", daemon=True)
            self.writer.start()

    def write(self) -> None:
        """Write npz and sidecar atomically, prune old files, then report done (writer thread)."""
        directory = Path(self.cfg.dir)
        try:
            part = directory / f"{self.name}{NPZ_SUFFIX}{PART_SUFFIX}"
            size = self.recording.write(part)
            meta = {"counts": self.recording.counts(), "dropped": self.recording.dropped(), "topics": self.topics()}
            (directory / f"{self.name}{META_SUFFIX}").write_text(json.dumps(meta))
            os.replace(part, directory / f"{self.name}{NPZ_SUFFIX}")
            prune_recordings(directory, self.cfg.max_files)
            log.info("ground_recording_written", name=self.name, bytes=size, counts=meta["counts"])
        except OSError:
            log.error("ground_recording_write_failed", name=self.name, exc_info=True)
        finally:
            self.on_done()

    def info(self) -> dict[str, Any]:
        """Describe the running recording.

        :return: {name, bytes, done, counts, dropped, topics} with done False (dict).
        """
        with self.lock:
            return {
                "name": self.name,
                "bytes": 0,
                "done": False,
                "counts": self.recording.counts(),
                "dropped": self.recording.dropped(),
                "topics": self.topics(),
            }

    def match(self, stamp: int, kind: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        """Pair the depth and the RGB half of a frame by stamp.

        :param stamp: header stamp, ns (int).
        :param kind: KIND_DEPTH or KIND_OTHER (str).
        :param payload: this half's data (dict).
        :return: both halves merged once complete, else None (dict | None).
        """
        entries = self.waiting
        entry = entries.setdefault(stamp, {})
        entry[kind] = payload
        if KIND_DEPTH in entry and KIND_OTHER in entry:
            del entries[stamp]
            return {**entry[KIND_DEPTH], **entry[KIND_OTHER]}
        while len(entries) > MATCH_CAPACITY:
            del entries[min(entries)]
            self.recording.drop(STREAM_PAIR, REASON_UNMATCHED)
        return None

    def on_cloud(self, msg: Any) -> None:
        """Record one point cloud with base <- frame and odom <- base at its stamp.

        :param msg: sensor_msgs/PointCloud2 (Any).
        """
        stamp, frame = stamp_ns(msg.header), msg.header.frame_id
        try:
            points = cloud_xyz(msg)
        except ValueError:
            with self.lock:
                self.recording.drop(STREAM_CLOUD, REASON_DECODE)
            return
        base, odom = self.cfg.base_frame, self.cfg.odom_frame

        def apply() -> None:
            base_t, odom_t = self.lookup(base, frame, stamp), self.lookup(odom, base, stamp)
            self.recording.add_cloud(stamp, frame, points, base_t, odom_t)

        self.submit(STREAM_CLOUD, apply)

    def on_depth(self, msg: Any) -> None:
        """Take one 16UC1 registered depth image and complete a pair when its RGB is in.

        :param msg: sensor_msgs/Image (Any).
        """
        array = image_array(msg, DEPTH_ENCODINGS)
        stamp = stamp_ns(msg.header)
        with self.lock:
            if self.closed:
                return
            if array is None:
                self.recording.drop(STREAM_PAIR, REASON_DECODE)
                return
            frame = self.match(stamp, KIND_DEPTH, {"depth": array})
        if frame is not None:
            self.add_pair(stamp, frame)

    def on_rgb(self, msg: Any) -> None:
        """Take one compressed RGB image; its bytes are stored as-is.

        :param msg: sensor_msgs/CompressedImage (Any).
        """
        stamp = stamp_ns(msg.header)
        with self.lock:
            if self.closed:
                return
            frame = self.match(stamp, KIND_OTHER, {"jpeg": bytes(msg.data), "rgb_frame": msg.header.frame_id})
        if frame is not None:
            self.add_pair(stamp, frame)

    def add_pair(self, stamp: int, frame: dict[str, Any]) -> None:
        """Record a complete RGB + registered depth pair with base <- RGB frame and odom <- base.

        :param stamp: header stamp, ns (int).
        :param frame: merged halves (dict).
        """
        base, odom, rgb_frame = self.cfg.base_frame, self.cfg.odom_frame, frame["rgb_frame"]

        def apply() -> None:
            base_t, odom_t = self.lookup(base, rgb_frame, stamp), self.lookup(odom, base, stamp)
            self.recording.add_pair(stamp, frame["jpeg"], frame["depth"], base_t, odom_t)

        self.submit(STREAM_PAIR, apply)

    def on_depth_info(self, msg: Any) -> None:
        """Keep the camera model of the D435 depth stream behind the cloud.

        :param msg: sensor_msgs/CameraInfo (Any).
        """
        with self.lock:
            self.recording.set_depth_info(msg.k, msg.d, msg.width, msg.height, msg.header.frame_id)

    def on_pair_info(self, msg: Any) -> None:
        """Keep the RGB camera info (the registered depth shares it).

        :param msg: sensor_msgs/CameraInfo (Any).
        """
        with self.lock:
            self.recording.set_rgb_info(msg.k, msg.d, msg.width, msg.height, msg.header.frame_id)

    def on_imu(self, msg: Any) -> None:
        """Record one IMU sample; the base <- IMU mount is looked up once, at the first stamp.

        :param msg: sensor_msgs/Imu (Any).
        """
        stamp, frame = stamp_ns(msg.header), msg.header.frame_id
        q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        row = ((q.x, q.y, q.z, q.w), (a.x, a.y, a.z), (w.x, w.y, w.z))

        def apply() -> None:
            mount = self.imu_mount or self.lookup(self.cfg.base_frame, frame, stamp)
            self.imu_mount = mount
            self.recording.set_imu_frame(frame, mount)
            self.recording.add_imu(stamp, *row)

        self.submit(STREAM_IMU, apply)

    def on_head(self, msg: Any) -> None:
        """Record the head pan and tilt angles.

        :param msg: sensor_msgs/JointState (Any).
        """
        positions = dict(zip(msg.name, msg.position, strict=False))
        with self.lock:
            if self.closed:
                return
            if HEAD_PAN_JOINT not in positions or HEAD_TILT_JOINT not in positions:
                self.recording.drop(STREAM_HEAD, REASON_JOINTS)
                return
            self.recording.add_head_js(stamp_ns(msg.header), positions[HEAD_PAN_JOINT], positions[HEAD_TILT_JOINT])

    def on_state(self, msg: Any) -> None:
        """Record one /ground/state line stamped with the wall clock.

        :param msg: std_msgs/String (Any).
        """
        with self.lock:
            if not self.closed:
                self.recording.add_state_line(time.time_ns(), msg.data)
