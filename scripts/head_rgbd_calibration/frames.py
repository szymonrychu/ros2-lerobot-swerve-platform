"""Capture a calibration session from the robot and turn the bag into still, paired RGB and ToF frames."""

import shlex
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from scripts.head_rgbd_calibration.config import CaptureConfig, SelectionConfig

NS_PER_S = 1_000_000_000
BAG_STORAGE = "sqlite3"
STEP_NAMES = ("record", "copy", "remove")
MAX_UINT8 = 255
RGB_DIFF_BLUR_SIGMA = 1.0  # px; suppresses sensor and JPEG noise before comparing frames

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


@dataclass(frozen=True)
class TopicNames:
    """Topics read from a bag (defaults are the calibration capture topics)."""

    rgb: str = "/head_camera/mapping/image_raw/compressed"
    rgb_info: str = "/head_camera/mapping/camera_info"
    amplitude: str = "/tof/amplitude/image_raw"
    depth: str = "/tof/depth/image_raw"
    tof_info: str = "/tof/camera_info"


@dataclass(frozen=True)
class StampedImage:
    """One decoded image: stamp_ns is the message header stamp, image the pixel array."""

    stamp_ns: int
    image: np.ndarray


@dataclass(frozen=True)
class CameraInfoData:
    """The sensor_msgs/CameraInfo fields calibration needs (k row-major 3x3, p row-major 3x4)."""

    width: int
    height: int
    distortion_model: str
    d: np.ndarray
    k: np.ndarray
    p: np.ndarray


@dataclass
class FrameSet:
    """Decoded bag content, each list sorted by header stamp; infos are the last ones in the bag."""

    rgb: list[StampedImage] = field(default_factory=list)
    amplitude: list[StampedImage] = field(default_factory=list)
    depth: list[StampedImage] = field(default_factory=list)
    rgb_info: CameraInfoData | None = None
    tof_info: CameraInfoData | None = None


@dataclass(frozen=True)
class FramePair:
    """An RGB frame with its nearest ToF frame: amplitude (uint16), depth (uint16 mm, None if absent)."""

    rgb_stamp_ns: int
    tof_stamp_ns: int
    rgb: np.ndarray
    amplitude: np.ndarray
    depth: np.ndarray | None

    @property
    def dt_s(self) -> float:
        """Absolute RGB to ToF stamp difference in seconds."""
        return abs(self.rgb_stamp_ns - self.tof_stamp_ns) / NS_PER_S


def capture_commands(cfg: CaptureConfig, name: str, topics: list[str] | None = None) -> list[list[str]]:
    """Build the three capture steps: record on the robot, copy the bag back, remove the remote bag.

    The recording is bounded by timeout (SIGINT lets ros2 bag close the bag cleanly; exit 124 is the normal end),
    --max-bag-duration equals the duration so the bag is not split earlier.

    Args:
        cfg: Capture config.
        name: Bag directory name.
        topics: Topics to record (default cfg.topics).

    Returns:
        [ssh record, scp -r copy, ssh rm -rf] argument lists.
    """
    target = f"{cfg.ssh_user}@{cfg.host}"
    remote_bag = f"{cfg.remote_dir}/{name}"
    duration = f"{cfg.duration_s:g}"
    record = (
        f"source {shlex.quote(cfg.ros_setup)} && mkdir -p {shlex.quote(cfg.remote_dir)} && "
        f"{{ timeout -s INT {duration} ros2 bag record -s {BAG_STORAGE} --max-bag-duration {duration} "
        f"-o {shlex.quote(remote_bag)} {' '.join(shlex.quote(t) for t in (topics or cfg.topics))}; "
        "rc=$?; [ $rc -eq 0 ] || [ $rc -eq 124 ]; }"
    )
    return [
        ["ssh", target, record],
        ["scp", "-r", f"{target}:{remote_bag}", f"{cfg.local_dir}/{name}"],
        ["ssh", target, f"rm -rf {shlex.quote(remote_bag)}"],
    ]


def capture_bag(
    cfg: CaptureConfig,
    name: str,
    topics: list[str] | None = None,
    run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> Path:
    """Record on the robot, copy the bag to cfg.local_dir/name and remove the remote copy.

    Args:
        cfg: Capture config.
        name: Bag directory name.
        topics: Topics to record (default cfg.topics).
        run: Command runner (subprocess.run signature; injected in tests).

    Returns:
        The local bag directory.

    Raises:
        RuntimeError: naming the step (record, copy or remove) whose command failed.
    """
    Path(cfg.local_dir).mkdir(parents=True, exist_ok=True)
    for step, cmd in zip(STEP_NAMES, capture_commands(cfg, name, topics), strict=True):
        result = run(cmd, check=False)
        if result.returncode != 0:
            raise RuntimeError(f"capture step '{step}' failed with exit code {result.returncode}: {' '.join(cmd[:2])}")
    return Path(cfg.local_dir) / name


def stamp_ns(msg: object) -> int:
    """Header stamp of a ROS message in ns."""
    stamp = msg.header.stamp
    return int(stamp.sec) * NS_PER_S + int(stamp.nanosec)


def decode_mono16(msg: object) -> np.ndarray:
    """Decode a sensor_msgs/Image of 16-bit single channel (mono16 or 16UC1) to a uint16 array."""
    dtype = ">u2" if msg.is_bigendian else "<u2"
    rows = np.asarray(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return rows[:, : msg.width * 2].copy().view(dtype).astype(np.uint16).reshape(msg.height, msg.width)


def decode_compressed(msg: object, topic: str) -> np.ndarray:
    """Decode a sensor_msgs/CompressedImage to a BGR uint8 array.

    Raises:
        ValueError: naming the stamp and topic when the data does not decode.
    """
    image = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode compressed image at {stamp_ns(msg)} ns on {topic}")
    return image


def decode_camera_info(msg: object) -> CameraInfoData:
    """Reduce a sensor_msgs/CameraInfo to CameraInfoData."""
    return CameraInfoData(
        width=int(msg.width),
        height=int(msg.height),
        distortion_model=str(msg.distortion_model),
        d=np.asarray(msg.d, dtype=np.float64),
        k=np.asarray(msg.k, dtype=np.float64),
        p=np.asarray(msg.p, dtype=np.float64),
    )


def read_frames(bag: Path, topics: TopicNames) -> FrameSet:
    """Read a rosbag2 directory into decoded frames.

    Args:
        bag: Bag directory (metadata.yaml inside).
        topics: Topic names to read.

    Returns:
        FrameSet with BGR uint8 RGB frames, uint16 amplitude and depth (mm), and the last camera infos.

    Raises:
        ValueError: when a compressed RGB frame cannot be decoded.
    """
    frames = FrameSet()
    wanted = {topics.rgb, topics.rgb_info, topics.amplitude, topics.depth, topics.tof_info}
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic in wanted]
        for conn, _, raw in reader.messages(connections=connections):
            msg = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == topics.rgb:
                frames.rgb.append(StampedImage(stamp_ns(msg), decode_compressed(msg, conn.topic)))
            elif conn.topic == topics.amplitude:
                frames.amplitude.append(StampedImage(stamp_ns(msg), decode_mono16(msg)))
            elif conn.topic == topics.depth:
                frames.depth.append(StampedImage(stamp_ns(msg), decode_mono16(msg)))
            elif conn.topic == topics.rgb_info:
                frames.rgb_info = decode_camera_info(msg)
            else:
                frames.tof_info = decode_camera_info(msg)
    for series in (frames.rgb, frames.amplitude, frames.depth):
        series.sort(key=lambda f: f.stamp_ns)
    return frames


def normalise_amplitude(amp: np.ndarray, percentiles: tuple[float, float]) -> np.ndarray | None:
    """Stretch a ToF amplitude frame to uint8 so the low/high percentiles map to 0/255.

    Args:
        amp: Amplitude image (uint16).
        percentiles: (low, high) percentiles, e.g. (1, 99).

    Returns:
        uint8 image, or None for an empty frame (no dynamic range), never a fake image.
    """
    low, high = np.percentile(amp, percentiles)
    if high <= low:
        return None
    stretched = (amp.astype(np.float64) - low) / (high - low)
    return np.rint(np.clip(stretched, 0.0, 1.0) * MAX_UINT8).astype(np.uint8)


def mean_abs_diff(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute difference of two equally shaped images (inf if the shapes differ)."""
    if a.shape != b.shape:
        return float("inf")
    return float(np.mean(np.abs(a.astype(np.float64) - b.astype(np.float64))))


def rgb_difference(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute difference of two grey frames with mains flicker removed (inf if the shapes differ).

    Under mains lighting the rolling shutter paints moving horizontal bands, so a still scene differs by several
    grey levels between frames. Both frames are blurred by RGB_DIFF_BLUR_SIGMA, b is compared with a scaled row by
    row by the ratio of their row means, and the mean absolute residual is returned.

    Args:
        a: Earlier uint8 grey frame.
        b: Later uint8 grey frame.

    Returns:
        The flicker-compensated mean absolute difference in grey levels.
    """
    if a.shape != b.shape:
        return float("inf")
    a_blur = cv2.GaussianBlur(a.astype(np.float64), (0, 0), RGB_DIFF_BLUR_SIGMA)
    b_blur = cv2.GaussianBlur(b.astype(np.float64), (0, 0), RGB_DIFF_BLUR_SIGMA)
    a_rows = a_blur.mean(axis=1, keepdims=True)
    gain = np.divide(b_blur.mean(axis=1, keepdims=True), a_rows, out=np.ones_like(a_rows), where=a_rows > 0)
    return float(np.mean(np.abs(b_blur - a_blur * gain)))


def tof_diff_fraction(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute amplitude difference as a fraction of the amplitude range; inf for an empty frame."""
    amp_range = float(max(a.max(), b.max())) - float(min(a.min(), b.min()))
    if a.shape != b.shape or amp_range <= 0:
        return float("inf")
    return mean_abs_diff(a, b) / amp_range


def nearest_index(stamps: np.ndarray, target: int) -> int:
    """Index of the stamp closest to target (stamps sorted, non-empty)."""
    pos = int(np.searchsorted(stamps, target))
    candidates = [i for i in (pos - 1, pos) if 0 <= i < len(stamps)]
    return min(candidates, key=lambda i: abs(int(stamps[i]) - target))


def still_pairs(frames: FrameSet, sel: SelectionConfig) -> list[FramePair]:
    """Select one RGB and ToF pair per still window.

    An RGB frame is still when its rgb_difference to the previous RGB frame is below sel.still_rgb_mad grey levels and
    every consecutive ToF amplitude pair within sel.still_window_s before it differs by less than sel.still_tof_mad
    (fraction of the amplitude range). Consecutive still frames form a window; its middle frame is paired with the
    nearest ToF amplitude within sel.max_pair_dt_s. Frames in the first sel.skip_start_s are ignored.

    Args:
        frames: Decoded bag content.
        sel: Selection thresholds.

    Returns:
        Pairs in time order, at most one per still window.
    """
    if not frames.rgb or not frames.amplitude:
        return []
    window_ns = int(sel.still_window_s * NS_PER_S)
    max_dt_ns = int(sel.max_pair_dt_s * NS_PER_S)
    first_ns = frames.rgb[0].stamp_ns + int(sel.skip_start_s * NS_PER_S)
    grey = [cv2.cvtColor(f.image, cv2.COLOR_BGR2GRAY) for f in frames.rgb]
    amp_stamps = np.array([f.stamp_ns for f in frames.amplitude])
    depth_stamps = np.array([f.stamp_ns for f in frames.depth])
    tof_still = [
        tof_diff_fraction(frames.amplitude[i - 1].image, frames.amplitude[i].image) < sel.still_tof_mad
        for i in range(1, len(frames.amplitude))
    ]

    def is_still(i: int) -> bool:
        rgb_ns = frames.rgb[i].stamp_ns
        if rgb_ns < first_ns or i == 0 or rgb_difference(grey[i - 1], grey[i]) >= sel.still_rgb_mad:
            return False
        lo = int(np.searchsorted(amp_stamps, rgb_ns - window_ns))
        hi = int(np.searchsorted(amp_stamps, rgb_ns, side="right"))
        return hi - lo >= 2 and all(tof_still[lo : hi - 1])

    runs: list[list[int]] = []
    previous = False
    for i in range(len(frames.rgb)):
        current = is_still(i)
        if current:
            if previous:
                runs[-1].append(i)
            else:
                runs.append([i])
        previous = current

    pairs = []
    for run in runs:
        rgb = frames.rgb[run[len(run) // 2]]
        k = nearest_index(amp_stamps, rgb.stamp_ns)
        if abs(int(amp_stamps[k]) - rgb.stamp_ns) > max_dt_ns:
            continue
        amp = frames.amplitude[k]
        depth = None
        if len(depth_stamps):
            d = nearest_index(depth_stamps, amp.stamp_ns)
            if abs(int(depth_stamps[d]) - amp.stamp_ns) <= max_dt_ns:
                depth = frames.depth[d].image
        pairs.append(FramePair(rgb.stamp_ns, amp.stamp_ns, rgb.image, amp.image, depth))
    return pairs
