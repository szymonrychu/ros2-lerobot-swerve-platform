"""Frame types, the still-frame difference and the message decoders of the head RGB-D calibration."""

from dataclasses import dataclass

import cv2
import numpy as np

NS_PER_S = 1_000_000_000
MONO16_ENCODINGS = ("mono16", "16UC1")
BYTES_PER_MONO16_PIXEL = 2
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


def stamp_ns(msg: object) -> int:
    """Header stamp of a ROS message in ns.

    Args:
        msg: Message with a std_msgs/Header `header` (rosbags and rclpy messages alike).

    Returns:
        sec * 1e9 + nanosec.
    """
    stamp = msg.header.stamp
    return int(stamp.sec) * NS_PER_S + int(stamp.nanosec)


def decode_mono16(msg: object) -> np.ndarray:
    """Decode a 16-bit single-channel sensor_msgs/Image (mono16 or 16UC1), honouring step and is_bigendian.

    Args:
        msg: The Image message; data is any buffer of bytes (bytes, array.array, uint8 ndarray).

    Returns:
        (height, width) uint16 array.

    Raises:
        ValueError: another encoding, or data that does not hold height rows of step bytes.
    """
    if msg.encoding not in MONO16_ENCODINGS:
        raise ValueError(f"encoding {msg.encoding!r} is not one of {', '.join(MONO16_ENCODINGS)}")
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if height <= 0 or width <= 0 or step < width * BYTES_PER_MONO16_PIXEL or data.size != height * step:
        raise ValueError(f"{data.size} bytes do not hold a {width}x{height} mono16 image with step {step}")
    rows = data.reshape(height, step)[:, : width * BYTES_PER_MONO16_PIXEL]
    return rows.copy().view(">u2" if msg.is_bigendian else "<u2").astype(np.uint16)


def decode_compressed(msg: object, topic: str = "") -> np.ndarray:
    """Decode a sensor_msgs/CompressedImage to a BGR uint8 array.

    Args:
        msg: The CompressedImage message (header stamp and data bytes).
        topic: Topic name, for the error message.

    Returns:
        The BGR image.

    Raises:
        ValueError: naming the stamp and topic when the data does not decode.
    """
    try:
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    except cv2.error as exc:
        raise ValueError(f"cannot decode compressed image at {stamp_ns(msg)} ns on {topic}: {exc}") from exc
    if image is None:
        raise ValueError(f"cannot decode compressed image at {stamp_ns(msg)} ns on {topic}")
    return image


def decode_camera_info(msg: object) -> CameraInfoData:
    """Reduce a sensor_msgs/CameraInfo to CameraInfoData.

    Args:
        msg: The CameraInfo message.

    Returns:
        width, height, distortion_model, d, k and p.
    """
    return CameraInfoData(
        width=int(msg.width),
        height=int(msg.height),
        distortion_model=str(msg.distortion_model),
        d=np.asarray(msg.d, dtype=np.float64),
        k=np.asarray(msg.k, dtype=np.float64),
        p=np.asarray(msg.p, dtype=np.float64),
    )
