"""Live pose counter: which still board poses are usable, judged as the RGB, amplitude and depth frames arrive.

PoseCounter applies the selection of frames.still_pairs as the frames arrive (still RGB frame, still ToF window, ToF
frame within max_pair_dt_s) and captures a still window once the board is detected in both cameras. Each captured
pose carries the frames, the depth frame nearest the amplitude and the detections computed by the judgement, so the
caller stores it without detecting again. calibrate still selects its pairs from the stored poses; the live count is
the guide for the person holding the board.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.calibrate import detect_tof
from ros2_common.head_rgbd_calibration.charuco import Detection, detect
from ros2_common.head_rgbd_calibration.config import SelectionConfig
from ros2_common.head_rgbd_calibration.frames import (
    NS_PER_S,
    StampedImage,
    nearest_index,
    rgb_difference,
    tof_diff_fraction,
)

TOF_HISTORY_S = 3.0
WAITING = "waiting for frames"
MOVING = "moving"
NO_TOF_FRAME = "still - no ToF frame close in time"
NO_RGB_BOARD = "still - no board in the RGB image"
NO_TOF_BOARD = "still - board not found by the ToF: move closer"
RGB = "rgb"
AMPLITUDE = "amplitude"
DEPTH = "depth"
KINDS = (RGB, AMPLITUDE, DEPTH)


def captured_status(count: int) -> str:
    """Status line of a captured pose.

    Args:
        count: Number of the pose.

    Returns:
        "pose N captured - move to the next pose".
    """
    return f"pose {count} captured - move to the next pose"


@dataclass(frozen=True)
class CapturedPose:
    """A captured still pose: the paired frames and the detections that qualified it.

    Attributes:
        number: Pose number, continuing from start_count.
        rgb: Decoded BGR frame.
        rgb_raw: Compressed bytes the RGB frame was decoded from, None when not passed.
        amplitude: ToF amplitude frame (uint16) paired with the RGB frame.
        depth: Depth frame nearest the amplitude within max_pair_dt_s, None when there is none.
        rgb_detection: Board detection in the RGB image.
        tof_detection: Board detection in the amplitude image.
    """

    number: int
    rgb: StampedImage
    rgb_raw: bytes | None
    amplitude: StampedImage
    depth: StampedImage | None
    rgb_detection: Detection
    tof_detection: Detection


@dataclass(frozen=True)
class PoseEvent:
    """The judgement of one RGB frame: its status, and the pose when this frame captured one."""

    status: str
    pose: CapturedPose | None = None


class PoseCounter:
    """Captures still windows in which both cameras see the board, as frames arrive (ToF and RGB in stamp order)."""

    def __init__(self, board: cv2.aruco.CharucoBoard, sel: SelectionConfig, start_count: int = 0) -> None:
        """Start with no frames.

        Args:
            board: The ChArUco board.
            sel: Selection settings.
            start_count: Poses already stored; the next captured pose is numbered start_count + 1.
        """
        self.board = board
        self.sel = sel
        self.count = start_count
        self.tof: list[StampedImage] = []
        self.tof_still: list[bool] = []  # tof_still[j]: tof[j] and tof[j + 1] differ less than still_tof_mad
        self.depth: list[StampedImage] = []
        self.pending: list[tuple[StampedImage, bytes | None]] = []
        self.previous_grey: np.ndarray | None = None
        self.first_ns: int | None = None
        self.window_counted = False
        self.seen_rgb = False
        self.last_status = WAITING

    @property
    def status(self) -> str:
        """WAITING until an RGB and an amplitude frame arrived, then the status of the last judged RGB frame."""
        return self.last_status if self.seen_rgb and self.tof else WAITING

    def add(self, kind: str, frame: StampedImage, raw: bytes | None = None) -> list[PoseEvent]:
        """Add a frame; RGB frames are judged once a ToF frame max_pair_dt_s past them has arrived.

        Args:
            kind: RGB, AMPLITUDE or DEPTH.
            frame: The decoded frame.
            raw: Compressed bytes of an RGB frame, kept on the captured pose.

        Returns:
            One PoseEvent for each RGB frame judged by this call.

        Raises:
            ValueError: Unknown kind.
        """
        if kind == AMPLITUDE:
            if self.tof:
                self.tof_still.append(tof_diff_fraction(self.tof[-1].image, frame.image) < self.sel.still_tof_mad)
            self.tof.append(frame)
        elif kind == RGB:
            self.seen_rgb = True
            self.pending.append((frame, raw))
        elif kind == DEPTH:
            self.depth.append(frame)
        else:
            raise ValueError(f"unknown frame kind {kind!r}")
        max_dt_ns = int(self.sel.max_pair_dt_s * NS_PER_S)
        events = []
        while self.pending and self.tof and self.tof[-1].stamp_ns >= self.pending[0][0].stamp_ns + max_dt_ns:
            rgb, rgb_raw = self.pending.pop(0)
            event = self.judge(rgb, rgb_raw)
            self.last_status = event.status
            events.append(event)
        self.prune()
        return events

    def prune(self) -> None:
        """Drop ToF and depth frames older than TOF_HISTORY_S before the oldest frame still needed."""
        horizon = (self.pending[0][0].stamp_ns if self.pending else self.tof[-1].stamp_ns if self.tof else 0) - int(
            TOF_HISTORY_S * NS_PER_S
        )
        drop = 0
        while drop < len(self.tof) - 1 and self.tof[drop].stamp_ns < horizon:
            drop += 1
        del self.tof[:drop]
        del self.tof_still[:drop]
        keep = sum(f.stamp_ns < horizon for f in self.depth)
        del self.depth[:keep]

    def tof_window_still(self, rgb_ns: int) -> bool:
        """At least two ToF frames in the still window before rgb_ns, every consecutive pair still."""
        stamps = np.array([f.stamp_ns for f in self.tof])
        lo = int(np.searchsorted(stamps, rgb_ns - int(self.sel.still_window_s * NS_PER_S)))
        hi = int(np.searchsorted(stamps, rgb_ns, side="right"))
        return hi - lo >= 2 and all(self.tof_still[lo : hi - 1])

    def nearest_depth(self, amplitude_ns: int) -> StampedImage | None:
        """Depth frame nearest the amplitude stamp within max_pair_dt_s, None when there is none."""
        if not self.depth:
            return None
        stamps = np.array([f.stamp_ns for f in self.depth])
        k = nearest_index(stamps, amplitude_ns)
        return self.depth[k] if abs(int(stamps[k]) - amplitude_ns) <= int(self.sel.max_pair_dt_s * NS_PER_S) else None

    def judge(self, rgb: StampedImage, rgb_raw: bytes | None = None) -> PoseEvent:
        """Judge one RGB frame, capturing the pose when its still window is new and both cameras see the board."""
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
            return PoseEvent(MOVING)
        if self.window_counted:
            return PoseEvent(captured_status(self.count))
        stamps = np.array([f.stamp_ns for f in self.tof])
        k = nearest_index(stamps, rgb.stamp_ns)
        if abs(int(stamps[k]) - rgb.stamp_ns) > int(self.sel.max_pair_dt_s * NS_PER_S):
            return PoseEvent(NO_TOF_FRAME)
        rgb_detection = detect(rgb.image, self.board)
        if rgb_detection is None:
            return PoseEvent(NO_RGB_BOARD)
        amplitude = self.tof[k]
        tof_detection = detect_tof(amplitude.image, self.board, self.sel)
        if tof_detection is None:
            return PoseEvent(NO_TOF_BOARD)
        self.count += 1
        self.window_counted = True
        pose = CapturedPose(
            self.count, rgb, rgb_raw, amplitude, self.nearest_depth(amplitude.stamp_ns), rgb_detection, tof_detection
        )
        return PoseEvent(captured_status(self.count), pose)
