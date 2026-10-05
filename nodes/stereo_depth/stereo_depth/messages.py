"""ROS message conversion for stereo_depth (no rclpy; testable with stubbed message packages)."""

import copy

import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage

from .config import DepthConfig
from .depth import disparity_to_depth_mm

DISPARITY_ENCODING = "32FC1"
DEPTH_ENCODING = "16UC1"
FLOAT32_BYTES = 4
UINT16_BYTES = 2


def disparity_to_array(msg: DisparityImage) -> np.ndarray | None:
    """Decode the 32FC1 disparity Image of a DisparityImage message.

    Args:
        msg: stereo_image_proc disparity message.

    Returns:
        np.ndarray | None: float32 (height, width) disparity in pixels, or None when the image is empty, not 32FC1
            or its data is shorter than height * step.
    """
    img = msg.image
    if img.width == 0 or img.height == 0 or img.encoding != DISPARITY_ENCODING:
        return None
    row_floats = img.step // FLOAT32_BYTES
    if row_floats < img.width or len(img.data) < img.height * img.step:
        return None
    dtype = np.dtype(">f4" if img.is_bigendian else "<f4")
    raw = np.frombuffer(img.data, dtype=dtype, count=img.height * row_floats)
    return raw.reshape(img.height, row_floats)[:, : img.width].astype(np.float32)


class DepthConverter:
    """Turns DisparityImage messages into 16UC1 depth images, holding the "first valid disparity" state.

    Attributes:
        config: Depth limits.
        seen_valid: True once a disparity with at least one valid depth pixel was converted.
    """

    def __init__(self, config: DepthConfig) -> None:
        """Create a converter.

        Args:
            config: Depth limits.
        """
        self.config = config
        self.seen_valid = False

    def convert(
        self, disparity: DisparityImage, camera_info: CameraInfo | None
    ) -> tuple[Image, CameraInfo | None] | None:
        """Convert one disparity message.

        Nothing is returned for an empty or malformed disparity, for non-positive f or T, or while no disparity with
        a valid pixel has been seen yet. After the first valid frame an all-invalid frame is published as all zeros
        (REP 118 "no reading").

        Args:
            disparity: stereo_image_proc disparity message.
            camera_info: Latest rectified left CameraInfo, or None when none arrived yet.

        Returns:
            tuple[Image, CameraInfo | None] | None: 16UC1 depth in mm with the disparity header, and the camera info
                copy with the same header (None when camera_info is None); None when nothing may be published.
        """
        array = disparity_to_array(disparity)
        if array is None or disparity.f <= 0.0 or disparity.t <= 0.0:
            return None
        depth_mm = disparity_to_depth_mm(
            array,
            disparity.f,
            disparity.t,
            disparity.min_disparity,
            self.config.min_depth_m,
            self.config.max_depth_m,
        )
        if not self.seen_valid:
            if not depth_mm.any():
                return None
            self.seen_valid = True
        height, width = depth_mm.shape
        depth = Image()
        depth.header = copy.deepcopy(disparity.header)
        depth.height, depth.width = height, width
        depth.encoding = DEPTH_ENCODING
        depth.is_bigendian = 0
        depth.step = width * UINT16_BYTES
        depth.data = depth_mm.astype("<u2").tobytes()
        info = None
        if camera_info is not None:
            info = copy.deepcopy(camera_info)
            info.header = copy.deepcopy(disparity.header)
        return depth, info
