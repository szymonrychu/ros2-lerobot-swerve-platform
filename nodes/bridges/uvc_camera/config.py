"""Environment-based configuration for UVC camera bridge (no ROS/OpenCV deps)."""

import os

DEFAULT_DEVICE = "/dev/video0"
DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_FRAME_ID = "camera_optical_frame"
ENV_DEVICE_KEY = "UVC_DEVICE"
ENV_TOPIC_KEY = "UVC_TOPIC"
ENV_FRAME_ID_KEY = "UVC_FRAME_ID"
ENV_ROTATE_DEG_KEY = "UVC_ROTATE_DEG"
DEFAULT_ROTATE_DEG = 0
ALLOWED_ROTATE_DEG = (0, 90, 180, 270)


def get_config() -> tuple[str | int, str, str]:
    """Read (device, topic, frame_id) from environment.

    Returns:
        tuple[str | int, str, str]: (device path or index, topic name, frame_id).
        UVC_DEVICE can be integer (e.g. 0) or path; UVC_TOPIC and UVC_FRAME_ID are strings.
    """
    raw = (os.environ.get(ENV_DEVICE_KEY) or DEFAULT_DEVICE).strip()
    topic = (os.environ.get(ENV_TOPIC_KEY) or DEFAULT_TOPIC).strip() or DEFAULT_TOPIC
    frame_id = (os.environ.get(ENV_FRAME_ID_KEY) or DEFAULT_FRAME_ID).strip() or DEFAULT_FRAME_ID
    try:
        device: str | int = int(raw)
    except ValueError:
        device = raw
    return device, topic, frame_id


def get_rotate_deg() -> int:
    """Read the clockwise rotation applied to every published frame from UVC_ROTATE_DEG.

    Returns:
        int: One of ALLOWED_ROTATE_DEG (default 0).

    Raises:
        ValueError: When UVC_ROTATE_DEG is not an integer in ALLOWED_ROTATE_DEG.
    """
    raw = (os.environ.get(ENV_ROTATE_DEG_KEY) or "").strip()
    if not raw:
        return DEFAULT_ROTATE_DEG
    try:
        value = int(raw)
    except ValueError:
        value = -1
    if value not in ALLOWED_ROTATE_DEG:
        raise ValueError(f"{ENV_ROTATE_DEG_KEY} must be one of {ALLOWED_ROTATE_DEG}, got {raw!r}")
    return value
