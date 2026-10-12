"""Pure-Python helpers of the overview_camera launch file (no ROS imports, unit-tested from the repo root).

Settings merge, camera selection by libcamera ID or index, the calibration gate of camera_info_url and the
parameter dictionaries of the two camera_ros CameraNodes (left: overview camera and sync server; right: sync client).
"""

import math
from pathlib import Path
from typing import Any

import yaml

LAUNCH_KEYS = ("left_camera_id", "right_camera_id")
AE_EXPOSURE_MODES = {"normal": 0, "short": 1, "long": 2}
SYNC_MODES = {"off": 0, "server": 1, "client": 2}
SIDE_INDEX = {"left": 0, "right": 1}
CALIBRATION_FILE = "stereo_{side}.yaml"
JPEG_QUALITY_PARAMETER = "image_raw.compressed.jpeg_quality"
SIDE_LAUNCH_ONLY_KEYS = ("namespace", "sync_mode", "jpeg_quality")
PROJECTION_SIZE = 12
FOCAL_INDEXES = (0, 5)


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
        override_path: Deployed /etc/ros2/overview_camera/config.yaml; skipped when missing or empty. The flat keys
            left_camera_id and right_camera_id are moved into the `launch` section.

    Returns:
        dict[str, Any]: Merged settings.
    """
    settings = yaml.safe_load(defaults_path.read_text())
    if not override_path.is_file():
        return settings
    override = yaml.safe_load(override_path.read_text()) or {}
    flat = {key: override.pop(key) for key in LAUNCH_KEYS if key in override}
    return deep_merge(settings, deep_merge(override, {"launch": flat} if flat else {}))


def camera_selector(camera_id: str, index: int) -> tuple[str | int, str | None]:
    """Value of camera_ros' `camera` parameter for one camera.

    Args:
        camera_id: libcamera camera ID string (`cam -l`); empty when not known yet.
        index: Camera index used while the ID is unknown (0 left, 1 right).

    Returns:
        tuple[str | int, str | None]: The ID, or the index with a warning text.
    """
    camera_id = str(camera_id or "").strip()
    if camera_id:
        return camera_id, None
    warning = (
        f"No camera ID configured: selecting camera by index {index}. The index order is not stable across boots, "
        "read the libcamera IDs with `cam -l` and set left_camera_id / right_camera_id."
    )
    return index, warning


def projection_is_valid(projection: Any) -> bool:
    """True for a 3x4 projection matrix P that is finite and has non-zero focal lengths.

    Args:
        projection: The `projection_matrix.data` list of a camera_info YAML, or None.

    Returns:
        bool: Whether the matrix is a real calibration result.
    """
    if not isinstance(projection, list) or len(projection) != PROJECTION_SIZE:
        return False
    try:
        values = [float(v) for v in projection]
    except (TypeError, ValueError):
        return False
    return all(math.isfinite(v) for v in values) and all(values[i] != 0.0 for i in FOCAL_INDEXES)


def calibration_problem(path: Path) -> str | None:
    """Why one camera_info file cannot be handed to camera_ros, or None when it can.

    Args:
        path: Calibration YAML (as written by stereo_depth.calibrate).

    Returns:
        str | None: Problem description, or None.
    """
    if not path.is_file():
        return f"{path.name} is missing"
    try:
        doc = yaml.safe_load(path.read_text())
    except yaml.YAMLError as err:
        return f"{path.name} is not valid YAML ({err.__class__.__name__})"
    projection = (doc.get("projection_matrix") or {}).get("data") if isinstance(doc, dict) else None
    if not projection_is_valid(projection):
        return f"{path.name} has no non-zero projection matrix P"
    return None


def calibration_path(calibration_dir: Path, side: str) -> Path:
    """Calibration file of one side.

    Args:
        calibration_dir: Directory holding stereo_left.yaml / stereo_right.yaml.
        side: "left" or "right".

    Returns:
        Path: The file path.
    """
    return calibration_dir / CALIBRATION_FILE.format(side=side)


def camera_info_url(calibration_dir: Path, side: str) -> tuple[str, str | None]:
    """camera_info_url of one side, gated on a usable calibration file.

    Args:
        calibration_dir: Directory holding stereo_left.yaml / stereo_right.yaml.
        side: "left" or "right".

    Returns:
        tuple[str, str | None]: The file:// URL and None, or "" and the reason the file is unusable.
    """
    path = calibration_path(calibration_dir, side)
    problem = calibration_problem(path)
    if problem:
        return "", problem
    return f"file://{path}", None


def enum_value(table: dict[str, int], name: str, what: str) -> int:
    """Look up a libcamera enum value by its lower-case name.

    Args:
        table: Name to value mapping.
        name: Configured name.
        what: Setting name for the error message.

    Returns:
        int: The enum value.

    Raises:
        ValueError: When the name is not in the table.
    """
    try:
        return table[str(name).lower()]
    except KeyError:
        raise ValueError(f"{what}: unknown value {name!r}, expected one of {sorted(table)}") from None


def side_namespace(settings: dict[str, Any], side: str) -> str:
    """ROS namespace of one camera.

    Args:
        settings: Merged settings.
        side: "left" or "right".

    Returns:
        str: The namespace (/overview_camera for the left camera).
    """
    return str(settings[side]["namespace"])


def camera_parameters(settings: dict[str, Any], side: str, camera: str | int, info_url: str) -> dict[str, Any]:
    """Parameters of one camera_ros CameraNode.

    Args:
        settings: Merged settings.
        side: "left" or "right".
        camera: libcamera ID or index from camera_selector.
        info_url: camera_info_url of this side ("" publishes an uncalibrated camera_info).

    Returns:
        dict[str, Any]: Parameter dict (node params plus libcamera control params).

    Raises:
        ValueError: On an unknown AeExposureMode or sync_mode name.
    """
    params = dict(settings["camera"])
    params["AeExposureMode"] = enum_value(AE_EXPOSURE_MODES, params["AeExposureMode"], "AeExposureMode")
    side_settings = settings[side]
    params["SyncMode"] = enum_value(SYNC_MODES, side_settings["sync_mode"], "sync_mode")
    params.update({k: v for k, v in side_settings.items() if k not in SIDE_LAUNCH_ONLY_KEYS})
    if "jpeg_quality" in side_settings:
        params[JPEG_QUALITY_PARAMETER] = side_settings["jpeg_quality"]
    params.update(camera=camera, camera_info_url=info_url)
    return params
