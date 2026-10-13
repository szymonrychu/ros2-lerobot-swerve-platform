"""Pure frame helpers: rotation (pixels and lossless JPEG) and publish-rate throttling (no ROS)."""

from ctypes import byref, c_int

import numpy as np
from turbojpeg import (
    TJXOP_ROT90,
    TJXOP_ROT180,
    TJXOP_ROT270,
    TJXOPT_TRIM,
    CroppingRegion,
    TransformStruct,
    TurboJPEG,
)

# Clockwise degrees -> number of counter-clockwise quarter turns for np.rot90.
QUARTER_TURNS_CCW = {0: 0, 90: 3, 180: 2, 270: 1}
JPEG_TRANSFORM_OPS = {90: TJXOP_ROT90, 180: TJXOP_ROT180, 270: TJXOP_ROT270}
REOPEN_AFTER_FAILURES = 50  # about 5 s of failed reads at the 0.1 s retry spin


def rotate_frame(frame: np.ndarray, rotate_deg: int) -> np.ndarray:
    """Rotate an image clockwise by a multiple of 90 degrees.

    Args:
        frame (np.ndarray): Image of shape (height, width, channels).
        rotate_deg (int): 0, 90, 180 or 270 degrees clockwise.

    Returns:
        np.ndarray: C-contiguous rotated image (the input itself for 0); width and height swap for 90/270.

    Raises:
        ValueError: When rotate_deg is not 0, 90, 180 or 270.
    """
    if rotate_deg not in QUARTER_TURNS_CCW:
        raise ValueError(f"rotate_deg must be one of {tuple(QUARTER_TURNS_CCW)}, got {rotate_deg}")
    if rotate_deg == 0:
        return frame
    return np.ascontiguousarray(np.rot90(frame, QUARTER_TURNS_CCW[rotate_deg]))


def frame_due(last_publish_s: float | None, now_s: float, max_fps: float | None) -> bool:
    """Decide whether a captured frame should be published under a frame rate cap.

    Args:
        last_publish_s (float | None): Monotonic time of the last published frame, None before the first.
        now_s (float): Monotonic time of the current frame.
        max_fps (float | None): Publish rate cap, None for no cap.

    Returns:
        bool: True when the frame should be published.
    """
    if max_fps is None or last_publish_s is None:
        return True
    return now_s - last_publish_s >= 1.0 / max_fps - 1e-9


def reopen_due(consecutive_read_failures: int) -> bool:
    """Decide whether the capture device should be closed and reopened.

    Args:
        consecutive_read_failures (int): Frame reads that failed in a row.

    Returns:
        bool: True once the failures reach REOPEN_AFTER_FAILURES (a hung UVC device needs a reopen to recover).
    """
    return consecutive_read_failures >= REOPEN_AFTER_FAILURES


class RotatingTurboJPEG(TurboJPEG):
    """PyTurboJPEG with a lossless rotation, which the library exposes only as crop (it has no rotate call)."""

    def rotate(self, jpeg_buf: bytes, op: int) -> bytes:
        """Rotate a JPEG in the DCT domain (no decode, no re-encode) with libjpeg-turbo's tj3Transform.

        Args:
            jpeg_buf (bytes): Input JPEG.
            op (int): One of TJXOP_ROT90, TJXOP_ROT180, TJXOP_ROT270.

        Returns:
            bytes: Rotated JPEG; edge blocks that cannot be rotated losslessly are trimmed (at most one MCU).
        """
        handle = self._TurboJPEG__init_transform()
        try:
            source = np.frombuffer(jpeg_buf, dtype=np.uint8)
            address = self._TurboJPEG__getaddr(source)
            width, height, subsample, colorspace = c_int(), c_int(), c_int(), c_int()
            status = self._TurboJPEG__decompress_header(
                handle, address, source.size, byref(width), byref(height), byref(subsample), byref(colorspace)
            )
            if status != 0:
                self._TurboJPEG__report_error(handle)
            transform = TransformStruct(CroppingRegion(), op, TJXOPT_TRIM)  # no CROP flag: the region is ignored
            return self._TurboJPEG__do_transform(handle, address, source.size, 1, byref(transform))[0]
        finally:
            self._TurboJPEG__destroy(handle)


_turbo: RotatingTurboJPEG | None = None


def rotate_jpeg(jpeg: bytes, rotate_deg: int) -> bytes:
    """Rotate a JPEG clockwise by a multiple of 90 degrees without decoding it.

    Args:
        jpeg (bytes): Input JPEG.
        rotate_deg (int): 0, 90, 180 or 270 degrees clockwise.

    Returns:
        bytes: The input itself for 0, otherwise the losslessly rotated JPEG (width and height swap for 90/270).

    Raises:
        ValueError: When rotate_deg is not 0, 90, 180 or 270.
    """
    global _turbo
    if rotate_deg not in QUARTER_TURNS_CCW:
        raise ValueError(f"rotate_deg must be one of {tuple(QUARTER_TURNS_CCW)}, got {rotate_deg}")
    if rotate_deg == 0:
        return jpeg
    if _turbo is None:
        _turbo = RotatingTurboJPEG()
    return _turbo.rotate(jpeg, JPEG_TRANSFORM_OPS[rotate_deg])
