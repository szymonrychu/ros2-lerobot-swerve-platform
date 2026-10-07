"""Pure-Python helpers of the overview_camera launch file (no ROS imports, unit-tested from the repo root).

Settings merge, camera selection by libcamera ID or index and the parameter dictionary of camera_ros.
"""

from pathlib import Path
from typing import Any

import yaml

LAUNCH_KEYS = ("camera_id",)
AF_MODES = {"manual": 0, "auto": 1, "continuous": 2}
JPEG_QUALITY_PARAMETER = "image_raw.compressed.jpeg_quality"
DEFAULT_CAMERA_INDEX = 0


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
        override_path: Deployed /etc/ros2/overview_camera/config.yaml; skipped when missing or empty. The flat key
            camera_id is moved into the `launch` section.

    Returns:
        dict[str, Any]: Merged settings.
    """
    settings = yaml.safe_load(defaults_path.read_text())
    if not override_path.is_file():
        return settings
    override = yaml.safe_load(override_path.read_text()) or {}
    flat = {key: override.pop(key) for key in LAUNCH_KEYS if key in override}
    return deep_merge(settings, deep_merge(override, {"launch": flat} if flat else {}))


def camera_selector(camera_id: str) -> tuple[str | int, str | None]:
    """Value of camera_ros' `camera` parameter.

    Args:
        camera_id: libcamera camera ID string (`cam -l`); empty when not known yet.

    Returns:
        tuple[str | int, str | None]: The ID, or index 0 with a warning text.
    """
    camera_id = str(camera_id or "").strip()
    if camera_id:
        return camera_id, None
    warning = (
        f"No camera ID configured: selecting camera by index {DEFAULT_CAMERA_INDEX}. Read the libcamera ID with "
        "`cam -l` and set camera_id."
    )
    return DEFAULT_CAMERA_INDEX, warning


def af_mode_value(name: str) -> int:
    """Look up the libcamera AfMode enum value by its lower-case name.

    Args:
        name: manual, auto or continuous (case-insensitive).

    Returns:
        int: The enum value.

    Raises:
        ValueError: When the name is unknown.
    """
    try:
        return AF_MODES[str(name).lower()]
    except KeyError:
        raise ValueError(f"AfMode: unknown value {name!r}, expected one of {sorted(AF_MODES)}") from None


def camera_parameters(settings: dict[str, Any], camera: str | int) -> dict[str, Any]:
    """Parameters of the camera_ros CameraNode.

    Args:
        settings: Merged settings.
        camera: libcamera ID or index from camera_selector.

    Returns:
        dict[str, Any]: Parameter dict (node params plus libcamera control params). No camera_info_url: camera_ros
        then publishes an uncalibrated camera_info.

    Raises:
        ValueError: On an unknown AfMode name.
    """
    params = dict(settings["camera"])
    params["AfMode"] = af_mode_value(params["AfMode"])
    params[JPEG_QUALITY_PARAMETER] = params.pop("jpeg_quality")
    params["camera"] = camera
    return params
