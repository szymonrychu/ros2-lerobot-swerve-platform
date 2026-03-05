"""Convert ROS image messages to JPEG bytes for streaming and preview."""

from typing import Any

IMAGE_TYPE_RAW = "sensor_msgs/msg/Image"
IMAGE_TYPE_COMPRESSED = "sensor_msgs/msg/CompressedImage"
IMAGE_TYPE_NAMES = (IMAGE_TYPE_RAW, IMAGE_TYPE_COMPRESSED)


def is_image_type(type_name: str) -> bool:
    """Return True if the ROS type is an image type we can encode to JPEG."""
    return (type_name or "").strip() in IMAGE_TYPE_NAMES


def image_message_to_jpeg_bytes(msg: Any, type_name: str) -> bytes | None:
    """Convert a ROS Image or CompressedImage message to JPEG bytes.

    Args:
        msg: ROS message (sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage).
        type_name: ROS type name string.

    Returns:
        JPEG bytes, or None if conversion fails or encoding is unsupported.
    """
    if not msg or not type_name:
        return None
    type_name = type_name.strip()
    if type_name == IMAGE_TYPE_COMPRESSED:
        return _compressed_image_to_jpeg(msg)
    if type_name == IMAGE_TYPE_RAW:
        return _raw_image_to_jpeg(msg)
    return None


def _compressed_image_to_jpeg(msg: Any) -> bytes | None:
    """Use CompressedImage data as JPEG when format is jpeg."""
    fmt = (getattr(msg, "format", None) or "").strip().lower()
    if "jpeg" in fmt or "jpg" in fmt:
        data = getattr(msg, "data", None)
        if data is not None:
            return bytes(data)
    return None


def _raw_image_to_jpeg(msg: Any) -> bytes | None:
    """Decode raw Image (height, width, encoding, step, data) to JPEG via numpy + Pillow."""
    try:
        import numpy as np
        from PIL import Image as PILImage
    except ImportError:
        return None
    height = getattr(msg, "height", None)
    width = getattr(msg, "width", None)
    encoding = (getattr(msg, "encoding", None) or "").strip().lower()
    step = getattr(msg, "step", None)
    data = getattr(msg, "data", None)
    if height is None or width is None or not encoding or data is None:
        return None
    try:
        h, w = int(height), int(width)
    except (TypeError, ValueError):
        return None
    if h <= 0 or w <= 0:
        return None
    try:
        arr = np.frombuffer(bytes(data), dtype=np.uint8)
    except (TypeError, ValueError):
        return None
    step_val = int(step) if step is not None else (w * 3)
    if step_val < w:
        step_val = w * 3
    if arr.size < h * step_val:
        return None
    try:
        arr = arr[: h * step_val].reshape((h, step_val))
    except ValueError:
        return None
    if encoding in ("bgr8", "bgr888"):
        arr = arr[:, : w * 3].reshape((h, w, 3))
        arr = arr[:, :, ::-1]
        pil_mode = "RGB"
    elif encoding in ("rgb8", "rgb888"):
        arr = arr[:, : w * 3].reshape((h, w, 3))
        pil_mode = "RGB"
    elif encoding == "mono8":
        arr = arr[:, :w]
        pil_mode = "L"
    else:
        return None
    try:
        pil_img = PILImage.fromarray(arr, mode=pil_mode)
    except Exception:
        return None
    import io

    buf = io.BytesIO()
    pil_img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()
