"""Capture a calibration session from the robot and turn the bag into still, paired RGB and ToF frames."""

import shlex
import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import yaml
from ros2_common.head_rgbd_calibration.frames import (
    MAX_UINT8,
    NS_PER_S,
    RGB_DIFF_BLUR_SIGMA,
    CameraInfoData,
    FramePair,
    FrameSet,
    StampedImage,
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    mean_abs_diff,
    nearest_index,
    normalise_amplitude,
    rgb_difference,
    stamp_ns,
    still_pairs,
    tof_diff_fraction,
)
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from scripts.head_rgbd_calibration.config import CaptureConfig

# the frame types and still-pose selection live in the core; re-exported for importers of this module
__all__ = [
    "MAX_UINT8",
    "NS_PER_S",
    "RGB_DIFF_BLUR_SIGMA",
    "CameraInfoData",
    "FramePair",
    "FrameSet",
    "StampedImage",
    "mean_abs_diff",
    "nearest_index",
    "normalise_amplitude",
    "rgb_difference",
    "still_pairs",
    "tof_diff_fraction",
    "BAG_METADATA",
    "BAG_STORAGE",
    "ROS_CLI_ENV",
    "STEP_NAMES",
    "TYPESTORE",
    "TopicNames",
    "bag_message_count",
    "capture_bag",
    "capture_commands",
    "decode_camera_info",
    "decode_compressed",
    "decode_mono16",
    "read_frames",
    "stamp_ns",
]

BAG_STORAGE = "sqlite3"
STEP_NAMES = ("record", "copy", "remove")
# the robot's ROS CLI environment: without it a CLI process discovers the topics but receives nothing
ROS_CLI_ENV = "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
BAG_METADATA = "metadata.yaml"

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


@dataclass(frozen=True)
class TopicNames:
    """Topics read from a bag (defaults are the calibration capture topics)."""

    rgb: str = "/head_camera/mapping/image_raw/compressed"
    rgb_info: str = "/head_camera/mapping/camera_info"
    amplitude: str = "/tof/amplitude/image_raw"
    depth: str = "/tof/depth/image_raw"
    tof_info: str = "/tof/camera_info"


def capture_commands(cfg: CaptureConfig, name: str, topics: list[str] | None = None) -> list[list[str]]:
    """Build the three capture steps: record on the robot, copy the bag back, remove the remote bag.

    The recording runs in the robot's ROS CLI environment (ROS_CLI_ENV) and is bounded by timeout (SIGINT lets
    ros2 bag close the bag cleanly; exit 124 is the normal end),
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
        f"source {shlex.quote(cfg.ros_setup)} && export {ROS_CLI_ENV} && mkdir -p {shlex.quote(cfg.remote_dir)} && "
        f"{{ timeout -s INT {duration} ros2 bag record -s {BAG_STORAGE} --max-bag-duration {duration} "
        f"-o {shlex.quote(remote_bag)} {' '.join(shlex.quote(t) for t in (topics or cfg.topics))}; "
        "rc=$?; [ $rc -eq 0 ] || [ $rc -eq 124 ]; }"
    )
    return [
        ["ssh", target, record],
        ["scp", "-r", f"{target}:{remote_bag}", f"{cfg.local_dir}/{name}"],
        ["ssh", target, f"rm -rf {shlex.quote(remote_bag)}"],
    ]


def bag_message_count(bag: Path) -> int:
    """Message count from a rosbag2 directory's metadata.yaml (0 when the file is missing)."""
    metadata = bag / BAG_METADATA
    if not metadata.is_file():
        return 0
    info = yaml.safe_load(metadata.read_text()) or {}
    return int(info.get("rosbag2_bagfile_information", {}).get("message_count", 0))


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
        RuntimeError: naming the step (record, copy or remove) whose command failed, or when the bag recorded no
            messages.
    """
    Path(cfg.local_dir).mkdir(parents=True, exist_ok=True)
    for step, cmd in zip(STEP_NAMES, capture_commands(cfg, name, topics), strict=True):
        result = run(cmd, check=False)
        if result.returncode != 0:
            raise RuntimeError(f"capture step '{step}' failed with exit code {result.returncode}: {' '.join(cmd[:2])}")
    bag = Path(cfg.local_dir) / name
    if bag_message_count(bag) == 0:
        raise RuntimeError(f"the bag {bag} recorded no messages: are the topics published on the robot?")
    return bag


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


