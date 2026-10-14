"""Pure-Python helpers of the pi_cameras launch file (no ROS imports, unit-tested from the repo root).

Settings merge, front/back selection from the CSI port and the parameter dictionaries of the two camera_ros
CameraNodes (Camera Module 3, IMX708).
"""

from pathlib import Path
from typing import Any

import yaml

ROLES = ("front", "back")
PORTS = ("cam0", "cam1")
LAUNCH_KEYS = ("front_port",)
JPEG_QUALITY_PARAMETER = "image_raw.compressed.jpeg_quality"
NAMESPACE = "/{role}_camera"
FRAME_ID = "{role}_camera_optical_frame"


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Merge two nested dicts without modifying either.

    Args:
        base: Defaults.
        override: Values that win; nested dicts are merged key by key.

    Returns:
        dict[str, Any]: A new merged dict.
    """
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_settings(defaults_path: Path, override_path: Path) -> dict[str, Any]:
    """Repo defaults with the deployed override applied on top.

    Args:
        defaults_path: config/params.yaml.
        override_path: Deployed /etc/ros2/pi_cameras/config.yaml; skipped when missing or empty. The flat key
            front_port is moved into the `launch` section.

    Returns:
        dict[str, Any]: Merged settings.
    """
    settings = yaml.safe_load(defaults_path.read_text())
    if not override_path.is_file():
        return settings
    override = yaml.safe_load(override_path.read_text()) or {}
    flat = {key: override.pop(key) for key in LAUNCH_KEYS if key in override}
    return deep_merge(settings, deep_merge(override, {"launch": flat} if flat else {}))


def role_camera_ids(settings: dict[str, Any]) -> dict[str, str]:
    """libcamera camera ID of the front and the back camera.

    Args:
        settings: Merged settings (`launch.front_port`, `launch.camera_ids`).

    Returns:
        dict[str, str]: {"front": id, "back": id}; the camera on front_port is the front one.

    Raises:
        ValueError: When front_port is not cam0 or cam1.
    """
    launch = settings["launch"]
    front = launch["front_port"]
    if front not in PORTS:
        raise ValueError(f"front_port: unknown value {front!r}, expected one of {list(PORTS)}")
    back = next(port for port in PORTS if port != front)
    return {"front": launch["camera_ids"][front], "back": launch["camera_ids"][back]}


def role_namespace(role: str) -> str:
    """ROS namespace of one camera.

    Args:
        role: "front" or "back".

    Returns:
        str: /front_camera or /back_camera.

    Raises:
        ValueError: On an unknown role.
    """
    if role not in ROLES:
        raise ValueError(f"role: unknown value {role!r}, expected one of {list(ROLES)}")
    return NAMESPACE.format(role=role)


def camera_parameters(settings: dict[str, Any], role: str, camera_id: str) -> dict[str, Any]:
    """Parameters of one camera_ros CameraNode.

    Args:
        settings: Merged settings.
        role: "front" or "back".
        camera_id: libcamera camera ID from role_camera_ids.

    Returns:
        dict[str, Any]: Parameter dict (node params plus libcamera control params).

    Raises:
        ValueError: On an unknown role.
    """
    role_namespace(role)
    params = dict(settings["camera"])
    params[JPEG_QUALITY_PARAMETER] = settings["compressed"]["jpeg_quality"]
    params.update(camera=camera_id, frame_id=FRAME_ID.format(role=role))
    return params
