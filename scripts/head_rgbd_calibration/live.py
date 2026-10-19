"""Live pose counter for the calibration capture: how many usable board poses are recorded so far.

While `capture` records the bag, live_relay.py runs on the robot (piped over ssh) and streams the RGB mapping frames
and the ToF amplitude frames. PoseCounter applies the selection of frames.still_pairs as the frames arrive (still
RGB frame, still ToF window, ToF frame within max_pair_dt_s) and counts a still window once the board is detected in
both cameras, so the person holding the board knows when to move to the next pose. calibrate still selects its pairs
from the recorded bag; the live count is the guide.
"""

import shlex
import struct
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import BinaryIO, TextIO

import cv2
import numpy as np

from scripts.head_rgbd_calibration.charuco import detect, make_board
from scripts.head_rgbd_calibration.config import CalibrationConfig, CaptureConfig, SelectionConfig
from scripts.head_rgbd_calibration.frames import (
    NS_PER_S,
    TYPESTORE,
    StampedImage,
    TopicNames,
    decode_compressed,
    decode_mono16,
    nearest_index,
    rgb_difference,
    stamp_ns,
    tof_diff_fraction,
)
from scripts.head_rgbd_calibration.pipeline import detect_tof

FRAME_HEADER = "<BI"  # (topic index, payload length), shared with the relay through its command line
RGB_INDEX = 0
AMPLITUDE_INDEX = 1
MESSAGE_TYPES = {RGB_INDEX: "sensor_msgs/msg/CompressedImage", AMPLITUDE_INDEX: "sensor_msgs/msg/Image"}
RELAY_SCRIPT = Path(__file__).with_name("live_relay.py")
# the robot's ROS CLI environment (without it the relay discovers the topics but receives nothing)
RELAY_ENV = "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
RELAY_GRACE_S = 10  # the relay outlives the recording by this much at most
STOP_TIMEOUT_S = 5.0
TOF_HISTORY_S = 3.0
BELL = "\a"
MOVING = "moving"
NO_TOF_FRAME = "still - no ToF frame close in time"
NO_RGB_BOARD = "still - no board in the RGB image"
NO_TOF_BOARD = "still - board not found by the ToF: move closer"


def captured_status(count: int) -> str:
    """Status line of a counted pose."""
    return f"pose {count} captured - move to the next pose"


def read_stream(stream: BinaryIO) -> Iterator[tuple[int, StampedImage]]:
    """Decode the relay stream into (topic index, frame); unknown indices are skipped, a truncated tail ends it.

    Args:
        stream: Binary stream of FRAME_HEADER-prefixed serialized messages.

    Yields:
        (RGB_INDEX, BGR frame) or (AMPLITUDE_INDEX, uint16 amplitude frame), stamped with the header stamp.
    """
    size = struct.calcsize(FRAME_HEADER)
    while True:
        head = stream.read(size)
        if len(head) < size:
            return
        index, length = struct.unpack(FRAME_HEADER, head)
        data = stream.read(length)
        if len(data) < length:
            return
        if index not in MESSAGE_TYPES:
            continue
        msg = TYPESTORE.deserialize_cdr(data, MESSAGE_TYPES[index])
        image = decode_compressed(msg, "live RGB") if index == RGB_INDEX else decode_mono16(msg)
        yield index, StampedImage(stamp_ns(msg), image)


class PoseCounter:
    """Counts still windows in which both cameras see the board, as frames arrive (ToF and RGB in stamp order)."""

    def __init__(self, board: cv2.aruco.CharucoBoard, sel: SelectionConfig) -> None:
        """Start with no frames; count is the number of captured poses."""
        self.board = board
        self.sel = sel
        self.count = 0
        self.tof: list[StampedImage] = []
        self.tof_still: list[bool] = []  # tof_still[j]: tof[j] and tof[j + 1] differ less than still_tof_mad
        self.pending: list[StampedImage] = []
        self.previous_grey: np.ndarray | None = None
        self.first_ns: int | None = None
        self.window_counted = False

    def add(self, index: int, frame: StampedImage) -> list[tuple[str, bool]]:
        """Add a frame; RGB frames are judged once a ToF frame max_pair_dt_s past them has arrived.

        Returns:
            (status, newly captured) for each RGB frame judged by this call.
        """
        if index == AMPLITUDE_INDEX:
            if self.tof:
                self.tof_still.append(tof_diff_fraction(self.tof[-1].image, frame.image) < self.sel.still_tof_mad)
            self.tof.append(frame)
        elif index == RGB_INDEX:
            self.pending.append(frame)
        max_dt_ns = int(self.sel.max_pair_dt_s * NS_PER_S)
        events = []
        while self.pending and self.tof and self.tof[-1].stamp_ns >= self.pending[0].stamp_ns + max_dt_ns:
            events.append(self.judge(self.pending.pop(0)))
        self.prune()
        return events

    def prune(self) -> None:
        """Drop ToF frames older than TOF_HISTORY_S before the oldest frame still needed."""
        horizon = (self.pending[0].stamp_ns if self.pending else self.tof[-1].stamp_ns if self.tof else 0) - int(
            TOF_HISTORY_S * NS_PER_S
        )
        drop = 0
        while drop < len(self.tof) - 1 and self.tof[drop].stamp_ns < horizon:
            drop += 1
        del self.tof[:drop]
        del self.tof_still[:drop]

    def tof_window_still(self, rgb_ns: int) -> bool:
        """At least two ToF frames in the still window before rgb_ns, every consecutive pair still."""
        stamps = np.array([f.stamp_ns for f in self.tof])
        lo = int(np.searchsorted(stamps, rgb_ns - int(self.sel.still_window_s * NS_PER_S)))
        hi = int(np.searchsorted(stamps, rgb_ns, side="right"))
        return hi - lo >= 2 and all(self.tof_still[lo : hi - 1])

    def judge(self, rgb: StampedImage) -> tuple[str, bool]:
        """Status of one RGB frame, counting the pose when its still window is new and both cameras see the board."""
        grey = cv2.cvtColor(rgb.image, cv2.COLOR_BGR2GRAY)
        previous, self.previous_grey = self.previous_grey, grey
        if self.first_ns is None:
            self.first_ns = rgb.stamp_ns + int(self.sel.skip_start_s * NS_PER_S)
        still = (
            rgb.stamp_ns >= self.first_ns
            and previous is not None
            and rgb_difference(previous, grey) < self.sel.still_rgb_mad
            and self.tof_window_still(rgb.stamp_ns)
        )
        if not still:
            self.window_counted = False
            return MOVING, False
        if self.window_counted:
            return captured_status(self.count), False
        stamps = np.array([f.stamp_ns for f in self.tof])
        k = nearest_index(stamps, rgb.stamp_ns)
        if abs(int(stamps[k]) - rgb.stamp_ns) > int(self.sel.max_pair_dt_s * NS_PER_S):
            return NO_TOF_FRAME, False
        if detect(rgb.image, self.board) is None:
            return NO_RGB_BOARD, False
        if detect_tof(self.tof[k].image, self.board, self.sel) is None:
            return NO_TOF_BOARD, False
        self.count += 1
        self.window_counted = True
        return captured_status(self.count), True


def monitor(stream: BinaryIO, counter: PoseCounter, out: TextIO, clock: Callable[[], float] = time.monotonic) -> None:
    """Feed the relay stream to counter and print each status change; a newly captured pose rings the bell."""
    start = clock()
    last = None
    for index, frame in read_stream(stream):
        for status, captured in counter.add(index, frame):
            if status == last and not captured:
                continue
            last = status
            bell = BELL if captured else ""
            print(f"[{clock() - start:4.0f}s] poses {counter.count} | {status}{bell}", file=out, flush=True)


def relay_command(cfg: CaptureConfig, topics: TopicNames | None = None) -> list[str]:
    """ssh command running the relay (source piped on stdin) for at most the capture duration plus RELAY_GRACE_S."""
    topics = topics or TopicNames()
    args = " ".join(shlex.quote(a) for a in (FRAME_HEADER, topics.rgb, topics.amplitude))
    remote = (
        f"source {shlex.quote(cfg.ros_setup)} && export {RELAY_ENV} && "
        f"timeout -s INT {cfg.duration_s + RELAY_GRACE_S:g} python3 - {args}"
    )
    return ["ssh", f"{cfg.ssh_user}@{cfg.host}", remote]


def start_live(
    cfg: CalibrationConfig, popen: Callable[..., subprocess.Popen] = subprocess.Popen, out: TextIO = sys.stderr
) -> Callable[[], None]:
    """Start the relay on the robot and the counter thread; the returned stop() ends both and prints the total."""
    process = popen(relay_command(cfg.capture), stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    process.stdin.write(RELAY_SCRIPT.read_bytes())
    process.stdin.close()
    counter = PoseCounter(make_board(cfg.board), cfg.selection)
    thread = threading.Thread(target=monitor, args=(process.stdout, counter, out), daemon=True)
    thread.start()

    def stop() -> None:
        process.terminate()
        try:
            process.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.kill()
        thread.join(timeout=STOP_TIMEOUT_S)
        print(f"live: {counter.count} poses counted (calibrate selects from the recorded bag)", file=out, flush=True)

    return stop
