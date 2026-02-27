"""Environment-based configuration for UVC camera bridge (no ROS/OpenCV deps)."""

import os

DEFAULT_DEVICE = "/dev/video0"
DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_FRAME_ID = "camera_optical_frame"
ENV_DEVICE_KEY = "UVC_DEVICE"
ENV_TOPIC_KEY = "UVC_TOPIC"
ENV_FRAME_ID_KEY = "UVC_FRAME_ID"


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
