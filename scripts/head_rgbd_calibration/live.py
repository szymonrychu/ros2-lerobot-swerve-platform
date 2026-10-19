"""Live pose counter CLI for the calibration capture: how many usable board poses are recorded so far.

While `capture` records the bag, live_relay.py runs on the robot (piped over ssh) and streams the RGB mapping frames
and the ToF amplitude frames. PoseCounter (ros2_common.head_rgbd_calibration.poses) applies the selection of
frames.still_pairs as the frames arrive and counts a still window once the board is detected in both cameras, so the
person holding the board knows when to move to the next pose. calibrate still selects its pairs from the recorded
bag; the live count is the guide.
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

from ros2_common.head_rgbd_calibration.poses import AMPLITUDE, RGB, PoseCounter

from scripts.head_rgbd_calibration.charuco import make_board
from scripts.head_rgbd_calibration.config import CalibrationConfig, CaptureConfig
from scripts.head_rgbd_calibration.frames import (
    ROS_CLI_ENV,
    TYPESTORE,
    StampedImage,
    TopicNames,
    decode_compressed,
    decode_mono16,
    stamp_ns,
)

FRAME_HEADER = "<BI"  # (topic index, payload length), shared with the relay through its command line
RGB_INDEX = 0
AMPLITUDE_INDEX = 1
INDEX_KINDS = {RGB_INDEX: RGB, AMPLITUDE_INDEX: AMPLITUDE}
MESSAGE_TYPES = {RGB_INDEX: "sensor_msgs/msg/CompressedImage", AMPLITUDE_INDEX: "sensor_msgs/msg/Image"}
RELAY_SCRIPT = Path(__file__).with_name("live_relay.py")
RELAY_GRACE_S = 10  # the relay outlives the recording by this much at most
STOP_TIMEOUT_S = 5.0
BELL = "\a"


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


def monitor(stream: BinaryIO, counter: PoseCounter, out: TextIO, clock: Callable[[], float] = time.monotonic) -> None:
    """Feed the relay stream to counter and print each status change; a newly captured pose rings the bell."""
    start = clock()
    last = None
    for index, frame in read_stream(stream):
        for event in counter.add(INDEX_KINDS[index], frame):
            captured = event.pose is not None
            if event.status == last and not captured:
                continue
            last = event.status
            bell = BELL if captured else ""
            print(f"[{clock() - start:4.0f}s] poses {counter.count} | {event.status}{bell}", file=out, flush=True)


def relay_command(cfg: CaptureConfig, topics: TopicNames | None = None) -> list[str]:
    """ssh command running the relay (source piped on stdin) for at most the capture duration plus RELAY_GRACE_S."""
    topics = topics or TopicNames()
    args = " ".join(shlex.quote(a) for a in (FRAME_HEADER, topics.rgb, topics.amplitude))
    remote = (
        f"source {shlex.quote(cfg.ros_setup)} && export {ROS_CLI_ENV} && "
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
