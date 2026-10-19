"""Frame types and still-pose selection for the head RGB-D calibration (no bag or rosbags dependency)."""

from dataclasses import dataclass, field

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.config import SelectionConfig

NS_PER_S = 1_000_000_000
MAX_UINT8 = 255
RGB_DIFF_BLUR_SIGMA = 1.0  # px; suppresses sensor and JPEG noise before comparing frames


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
